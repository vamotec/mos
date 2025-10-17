// mos-core/src/hal/robot_controller.rs

use crate::error::MosError;
use crate::grpc::mos_robot_v1::{
    robot_controller_client::RobotControllerClient, GetJointStateRequest, MoveToJointTargetRequest,
};
use async_trait::async_trait;
use serde::Deserialize;
use std::sync::{Arc, Mutex};

// --- Re-add UDS imports ---
use hyper_util::rt::tokio::TokioIo;
use tokio::net::UnixStream;
use tonic::transport::{Channel, Endpoint, Uri};
use tower::service_fn;

/// Defines the types of Robot Controllers available.
#[derive(Debug, Deserialize, Clone, Default)]
#[serde(rename_all = "lowercase")]
pub enum RobotControllerType {
    Grpc,
    #[default]
    Mock,
}

/// Configuration for the Robot Controller.
#[derive(Debug, Deserialize, Clone, Default)]
pub struct RobotControllerConfig {
    #[serde(default)]
    pub controller_type: RobotControllerType,
    // For development on Mac/Windows
    pub ros2_grpc_address: Option<String>,
    // For production/Linux
    pub ros2_grpc_socket: Option<String>,
}

/// A trait defining the universal interface for controlling a robot.
#[async_trait]
pub trait RobotController: Send + Sync {
    async fn get_joint_angles(&self) -> Result<Vec<f64>, MosError>;
    async fn set_joint_angles(&self, angles: &[f64]) -> Result<(), MosError>;
}

// --- GrpcRobotController Implementation ---
pub struct GrpcRobotController {
    client: RobotControllerClient<Channel>,
}

impl GrpcRobotController {
    // --- MODIFIED: `new` now takes a pre-built channel ---
    pub fn new(channel: Channel) -> Self {
        let client = RobotControllerClient::new(channel);
        Self { client }
    }
}

#[async_trait]
impl RobotController for GrpcRobotController {
    async fn get_joint_angles(&self) -> Result<Vec<f64>, MosError> {
        let mut client = self.client.clone();
        let request = tonic::Request::new(GetJointStateRequest {});
        let response = client
            .get_joint_state(request)
            .await
            .map_err(|e| MosError::RobotControllerError(e.to_string()))?;
        Ok(response.into_inner().position)
    }

    async fn set_joint_angles(&self, angles: &[f64]) -> Result<(), MosError> {
        let mut client = self.client.clone();
        let request = tonic::Request::new(MoveToJointTargetRequest {
            joint_positions: angles.to_vec(),
            speed_ratio: None,
            acceleration_ratio: None,
        });
        client
            .move_to_joint_target(request)
            .await
            .map_err(|e| MosError::RobotControllerError(e.to_string()))?;
        Ok(())
    }
}

// --- MockRobotController Implementation ---
pub struct MockRobotController {
    joint_angles: Arc<Mutex<Vec<f64>>>,
}

impl MockRobotController {
    pub fn new(initial_angles: Vec<f64>) -> Self {
        Self {
            joint_angles: Arc::new(Mutex::new(initial_angles)),
        }
    }
}

#[async_trait]
impl RobotController for MockRobotController {
    async fn get_joint_angles(&self) -> Result<Vec<f64>, MosError> {
        let angles = self.joint_angles.lock().unwrap().clone();
        log::info!("[Mock HAL] Getting joint angles: {:?}", angles);
        Ok(angles)
    }

    async fn set_joint_angles(&self, angles: &[f64]) -> Result<(), MosError> {
        let mut joint_angles = self.joint_angles.lock().unwrap();
        *joint_angles = angles.to_vec();
        log::info!("[Mock HAL] Setting joint angles to: {:?}", angles);
        Ok(())
    }
}

// --- Factory Function ---

// --- MODIFIED: Factory now contains all connection logic ---
pub async fn create_robot_controller(
    config: &RobotControllerConfig,
) -> Result<Box<dyn RobotController>, MosError> {
    match config.controller_type {
        RobotControllerType::Grpc => {
            let channel = if let Some(addr) = &config.ros2_grpc_address {
                // Prioritize TCP address for development
                log::info!("Connecting to robot_controller via TCP: {}", addr);
                Endpoint::from_shared(addr.clone())
                    .map_err(|e| MosError::RobotControllerError(e.to_string()))?
                    .connect()
                    .await
                    .map_err(|e| MosError::RobotControllerError(e.to_string()))?
            } else if let Some(socket_path) = &config.ros2_grpc_socket {
                // Fallback to UDS for production
                log::info!("Connecting to robot_controller via UDS: {}", socket_path);
                let socket_path = socket_path.clone(); // Clone for the closure
                Endpoint::try_from("http://[::]:50051") // Dummy URI
                    .map_err(|e| MosError::RobotControllerError(e.to_string()))?
                    .connect_with_connector(service_fn(move |_: Uri| {
                        let path = socket_path.clone();
                        Box::pin(async move {
                            let stream = UnixStream::connect(path).await?;
                            Ok::<_, std::io::Error>(TokioIo::new(stream))
                        })
                    }))
                    .await
                    .map_err(|e| MosError::RobotControllerError(e.to_string()))?
            } else {
                return Err(MosError::RobotControllerError(
                    "No gRPC address or socket path provided in config".into(),
                ));
            };

            let controller = GrpcRobotController::new(channel);
            Ok(Box::new(controller))
        }
        RobotControllerType::Mock => {
            let controller = MockRobotController::new(vec![0.0; 6]);
            Ok(Box::new(controller))
        }
    }
}