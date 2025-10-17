use anyhow::Result;
use clap::{Parser, Subcommand};
use hyper_util::rt::tokio::TokioIo;
use tokio::net::UnixStream;
use tokio_stream::StreamExt;
use tonic::transport::{Endpoint, Uri};
use tonic::Request;
use tower::service_fn;

mod config;
use config::load_config;

use mos_viz_agent::VizAgent;

use crate::grpc::mos::{
    mos_client::MosClient, GetJointAnglesRequest, SetJointAnglesRequest, TaskRequest,
};

mod grpc {
    pub mod mos;
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
#[command(propagate_version = true)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug, Clone)]
enum Commands {
    #[command(name = "task")]
    Task(Task),
    #[command(name = "telemetry")]
    Telemetry(Telemetry),
    #[command(name = "robot")]
    Robot(Robot),
    StartVizAgent,
}

#[derive(Parser, Debug, Clone)]
pub struct Task {
    #[command(subcommand)]
    pub action: TaskAction,
}

#[derive(Parser, Debug, Clone)]
pub struct Telemetry {
    #[command(subcommand)]
    pub action: TelemetryAction,
}

#[derive(Parser, Debug, Clone)]
pub struct Robot {
    #[command(subcommand)]
    pub action: RobotAction,
}

#[derive(Subcommand, Debug, Clone)]
pub enum TaskAction {
    Schedule {
        #[arg(short, long, default_value_t = 10)]
        priority: u32,
    },
}

#[derive(Subcommand, Debug, Clone)]
pub enum TelemetryAction {
    Stream,
}

#[derive(Subcommand, Debug, Clone)]
pub enum RobotAction {
    Set {
        #[arg(required = true, num_args = 1..)]
        angles: Vec<f64>,
    },
    Get,
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init();
    let cli = Cli::parse();

    if let Commands::StartVizAgent = &cli.command {
        return handle_start_viz_agent().await;
    }

    log::info!("Loading configuration...");
    let config = load_config("config.toml")
        .map_err(|e| anyhow::anyhow!("Failed to load config.toml: {}", e))?;

    let socket_path = std::env::current_dir()?.join(config.core_grpc_socket);
    log::info!("Connecting to mos-core server via UDS: {:?}", socket_path);

    let channel = Endpoint::try_from("http://[::]:50052") // Dummy URI
        .unwrap()
        .connect_with_connector(service_fn(move |_: Uri| {
            let path = socket_path.clone();
            Box::pin(async move {
                let stream = UnixStream::connect(path).await?;
                Ok::<_, std::io::Error>(TokioIo::new(stream))
            })
        }))
        .await
        .map_err(|e| anyhow::anyhow!("Failed to connect to mos-core via UDS: {}", e))?;

    let mut client = MosClient::new(channel);

    match &cli.command {
        Commands::Task(task) => match &task.action {
            TaskAction::Schedule { priority } => {
                handle_schedule_task(&mut client, *priority).await?;
            }
        },
        Commands::Telemetry(telemetry) => match &telemetry.action {
            TelemetryAction::Stream => {
                handle_stream_telemetry(&mut client).await?;
            }
        },
        Commands::Robot(robot) => match &robot.action {
            RobotAction::Set { angles } => {
                handle_set_joint_angles(&mut client, angles).await?;
            }
            RobotAction::Get => {
                handle_get_joint_angles(&mut client).await?;
            }
        },
        Commands::StartVizAgent => unreachable!(),
    }

    Ok(())
}
#[warn(unused_must_use)]
async fn handle_start_viz_agent() -> Result<()> {
    log::info!("Loading configuration for Viz Agent...");
    let config = mos_viz_agent::load_config("config.toml")
        .map_err(|e| anyhow::anyhow!("Failed to load config.toml: {}", e))?;

    log::info!("Creating Viz Agent...");
    let agent = VizAgent::new(config);

    log::info!("Starting Viz Agent run loop...");
    agent.run().await.expect("TODO: panic message");

    Ok(())
}

async fn handle_schedule_task(client: &mut MosClient<tonic::transport::Channel>, priority: u32) -> Result<()> {
    let request = Request::new(TaskRequest {
        priority,
        ..Default::default()
    });
    let response = client.schedule_task(request).await?;
    println!("Successfully scheduled task with ID: {}", response.into_inner().id);
    Ok(())
}

async fn handle_stream_telemetry(client: &mut MosClient<tonic::transport::Channel>) -> Result<()> {
    let request = Request::new(tokio_stream::empty());
    let mut stream = client.stream_telemetry(request).await?.into_inner();
    println!("Receiving telemetry data (Press Ctrl+C to stop):");
    while let Some(item) = stream.next().await {
        match item {
            Ok(telemetry) => {
                println!("Data: {:?}", telemetry);
            }
            Err(e) => {
                eprintln!("Error receiving telemetry data: {}", e);
                break;
            }
        }
    }
    Ok(())
}

async fn handle_set_joint_angles(
    client: &mut MosClient<tonic::transport::Channel>,
    angles: &[f64],
) -> Result<()> {
    let request = Request::new(SetJointAnglesRequest { angles: angles.to_vec() });
    let response = client.set_joint_angles(request).await?;
    let cmd_res = response.into_inner();
    if cmd_res.success {
        println!("Success: {}", cmd_res.message);
    } else {
        eprintln!("Failure: {}", cmd_res.message);
    }
    Ok(())
}

async fn handle_get_joint_angles(client: &mut MosClient<tonic::transport::Channel>) -> Result<()> {
    let request = Request::new(GetJointAnglesRequest {});
    let response = client.get_joint_angles(request).await?;
    println!("Current angles: {:?}", response.into_inner().angles);
    Ok(())
}
