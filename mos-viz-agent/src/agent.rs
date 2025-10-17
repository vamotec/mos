use crate::config::Config;
use crate::grpc::mos_robot_v1::{
    robot_controller_client::RobotControllerClient, MoveToJointTargetRequest,
};
use crate::protocol::{CloudCommand, CloudMsg, RosbridgeMsg, SetJointsArgs};
use futures_util::{
    stream::{SplitSink, SplitStream},
    SinkExt, StreamExt,
};
use hyper_util::rt::tokio::TokioIo;
use log::{error, info, warn};
use std::path::PathBuf;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::{
    net::{TcpStream, UnixStream},
    sync::mpsc,
};
use tokio_tungstenite::{
    connect_async, tungstenite::protocol::Message, MaybeTlsStream, WebSocketStream,
};
use tonic::transport::{Channel, Endpoint, Uri};
use tower::service_fn;
use url::Url;

const RECONNECT_DELAY: Duration = Duration::from_secs(5);
type WsStream = WebSocketStream<MaybeTlsStream<TcpStream>>;

pub struct VizAgent {
    config: Config,
}

impl VizAgent {
    pub fn new(config: Config) -> Self {
        info!("Initializing MOS Visualization Agent...");
        Self { config }
    }

    // --- MODIFIED: run now returns a Result to allow use of `?` ---
    pub async fn run(&self) -> anyhow::Result<()> {
        info!("Starting MOS Visualization Agent...");

        loop {
            // --- 1. Establish all connections ---
            let local_url = Url::parse(&self.config.viz_agent.rosbridge_url)?;
            let cloud_url = Url::parse(&self.config.viz_agent.cloud_url)?;

            info!("Connecting to local rosbridge: {}", local_url);
            let local_ws = match connect_async(local_url.as_str()).await {
                Ok((ws, _)) => {
                    info!("Connected to local rosbridge.");
                    ws
                }
                Err(e) => {
                    error!("Failed to connect to local rosbridge: {}. Retrying...", e);
                    tokio::time::sleep(RECONNECT_DELAY).await;
                    continue;
                }
            };

            info!("Connecting to cloud: {}", cloud_url);
            let cloud_ws = match connect_async(cloud_url.as_str()).await {
                Ok((ws, _)) => {
                    info!("Connected to cloud.");
                    ws
                }
                Err(e) => {
                    error!("Failed to connect to cloud: {}. Retrying...", e);
                    tokio::time::sleep(RECONNECT_DELAY).await;
                    continue;
                }
            };

            // --- MODIFIED: Flexible gRPC connection logic ---
            let channel = if let Some(addr) = &self.config.robot_controller.ros2_grpc_address {
                info!("Connecting to gRPC server via TCP: {}", addr);
                match Endpoint::from_shared(addr.clone())?.connect().await {
                    Ok(channel) => {
                        info!("Connected to gRPC server via TCP.");
                        channel
                    }
                    Err(e) => {
                        error!("Failed to connect to gRPC server via TCP: {}. Retrying...", e);
                        tokio::time::sleep(RECONNECT_DELAY).await;
                        continue;
                    }
                }
            } else if let Some(socket_path_str) = &self.config.robot_controller.ros2_grpc_socket {
                info!("Connecting to gRPC server via UDS: {}", socket_path_str);
                let socket_path = std::env::current_dir()?.join(socket_path_str);
                let socket_path = socket_path.clone();
                match Endpoint::try_from("http://[::]:50051")? // Dummy URI
                    .connect_with_connector(service_fn(move |_: Uri| {
                        let path = socket_path.clone();
                        Box::pin(async move {
                            let stream = UnixStream::connect(path).await?;
                            Ok::<_, std::io::Error>(TokioIo::new(stream))
                        })
                    }))
                    .await
                {
                    Ok(channel) => {
                        info!("Connected to gRPC server via UDS.");
                        channel
                    }
                    Err(e) => {
                        error!("Failed to connect to gRPC server via UDS: {}. Retrying...", e);
                        tokio::time::sleep(RECONNECT_DELAY).await;
                        continue;
                    }
                }
            } else {
                error!("No gRPC address or socket path provided for mos-ros2. Retrying...");
                tokio::time::sleep(RECONNECT_DELAY).await;
                continue;
            };

            let grpc_client = RobotControllerClient::new(channel);

            // --- 2. Setup tasks and channel ---
            let (tx, rx) = mpsc::channel::<CloudMsg>(100);

            let (mut local_write, local_read) = local_ws.split();
            let (cloud_write, cloud_read) = cloud_ws.split();

            // --- NEW: Subscribe to rosbridge topics ---
            info!("Subscribing to /joint_states topic...");
            let subscribe_msg = serde_json::json!({
                "op": "subscribe",
                "topic": "/joint_states",
                "type": "sensor_msgs/msg/JointState"
            });
            if let Ok(json_msg) = serde_json::to_string(&subscribe_msg) {
                if local_write.send(Message::Text(json_msg)).await.is_err() {
                    error!("Failed to send subscribe message to local rosbridge. Retrying...");
                    tokio::time::sleep(RECONNECT_DELAY).await;
                    continue;
                }
                info!("Successfully subscribed to /joint_states.");
            }
            // --- END NEW ---

            let local_reader_handle = tokio::spawn(local_reader_task(local_read, tx));
            let cloud_writer_handle = tokio::spawn(cloud_writer_task(cloud_write, rx));
            let cloud_reader_handle = tokio::spawn(cloud_reader_task(cloud_read, grpc_client));

            // --- 3. Supervise tasks ---
            tokio::select! {
                res = local_reader_handle => warn!("Local reader task exited with result: {:?}", res),
                res = cloud_writer_handle => warn!("Cloud writer task exited with result: {:?}", res),
                res = cloud_reader_handle => warn!("Cloud reader task exited with result: {:?}", res),
            }

            warn!("A task has terminated. Restarting all connections...");
            tokio::time::sleep(RECONNECT_DELAY).await;
        }
    }
}

// --- ADDED BACK: Task function definitions ---

/// Task: Reads from local rosbridge, wraps messages, and sends them to the cloud_writer task.
async fn local_reader_task(mut reader: SplitStream<WsStream>, tx: mpsc::Sender<CloudMsg>) {
    while let Some(msg) = reader.next().await {
        let msg = match msg {
            Ok(msg) => msg,
            Err(e) => {
                error!("Error reading from local rosbridge: {}", e);
                break;
            }
        };

        if let Message::Text(text) = msg {
            if let Ok(ros_msg) = serde_json::from_str::<RosbridgeMsg>(&text) {
                let timestamp_ms = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis();
                let cloud_msg = CloudMsg {
                    r#type: "ros_message".to_string(),
                    timestamp_ms,
                    payload: ros_msg,
                };
                if tx.send(cloud_msg).await.is_err() {
                    error!("Receiver dropped. Cannot send message to cloud writer task.");
                    break;
                }
            }
        }
    }
}

/// Task: Receives messages from other tasks via a channel and sends them to the cloud.
async fn cloud_writer_task(mut writer: SplitSink<WsStream, Message>, mut rx: mpsc::Receiver<CloudMsg>) {
    while let Some(cloud_msg) = rx.recv().await {
        match serde_json::to_string(&cloud_msg) {
            Ok(json) => {
                if writer.send(Message::Text(json)).await.is_err() {
                    error!("Failed to send message to cloud; connection closed.");
                    break;
                }
            }
            Err(e) => error!("Failed to serialize CloudMsg: {}", e),
        }
    }
}

/// Task: Reads commands from the cloud and executes them via gRPC.
async fn cloud_reader_task(
    mut reader: SplitStream<WsStream>,
    mut grpc_client: RobotControllerClient<Channel>,
) {
    while let Some(msg) = reader.next().await {
        let msg = match msg {
            Ok(msg) => msg,
            Err(e) => {
                error!("Error reading from cloud: {}", e);
                break;
            }
        };

        if let Message::Text(text) = msg {
            if let Ok(cmd) = serde_json::from_str::<CloudCommand>(&text) {
                info!("Received command from cloud: op='{}'", cmd.op);
                if cmd.op == "set_joint_angles" {
                    if let Ok(args) = serde_json::from_value::<SetJointsArgs>(cmd.args) {
                        let request = tonic::Request::new(MoveToJointTargetRequest {
                            joint_positions: args.joint_positions,
                            speed_ratio: None,
                            acceleration_ratio: None,
                        });
                        if let Err(e) = grpc_client.move_to_joint_target(request).await {
                            error!("gRPC call failed: {}", e);
                        } else {
                            info!("gRPC call 'move_to_joint_target' successful.");
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Config;

    #[test]
    fn it_creates_agent_successfully() {
        let config = Config::default();
        // The test simply needs to ensure that `new` can be called without panicking.
        let _agent = VizAgent::new(config);
        assert!(true);
    }
}