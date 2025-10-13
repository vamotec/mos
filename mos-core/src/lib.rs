pub mod config;
pub mod error;
pub mod grpc;
pub mod hal;
pub mod logging;
pub mod ota;
pub mod scheduler;
pub mod skill;
pub mod telemetry;
pub mod types;

use crate::grpc::server::MosService;
use crate::hal::robot_controller::{create_robot_controller, RobotController};
use crate::scheduler::scheduler::{Scheduler, TaskCommand};
use crate::types::TelemetryData;
pub use config::Config;
pub use error::MosError;
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tonic::transport::Server;

// MOS 核心结构体，整合所有功能
pub struct MosCore {
    config: Config,
    scheduler_tx: mpsc::Sender<TaskCommand>,
    telemetry_tx: broadcast::Sender<TelemetryData>,
    scheduler: Option<Scheduler>,
    robot_controller: Arc<Box<dyn RobotController>>,
}

impl MosCore {
    /// 创建 MOS 核心实例
    pub fn new(config: Config, robot_controller: Box<dyn RobotController>) -> Result<Self, MosError> {
        let (scheduler, scheduler_tx) = Scheduler::new();
        let (telemetry_tx, _) = broadcast::channel(100); // Capacity of 100
        Ok(Self {
            config,
            scheduler: Some(scheduler),
            scheduler_tx,
            telemetry_tx,
            robot_controller: Arc::new(robot_controller),
        })
    }

    /// 启动 MOS 核心（例如运行调度器、gRPC 服务）
    pub async fn run(mut self) -> Result<(), MosError> {
        if let Some(scheduler) = self.scheduler.take() {
            tokio::spawn(scheduler.run());
        }

        let addr = format!("[::1]:{}", self.config.grpc_port).parse().unwrap();
        let mos_service = MosService::new(
            self.scheduler_tx.clone(),
            self.telemetry_tx.clone(),
            self.robot_controller.clone(),
        );

        log::info!("gRPC server listening on {}", addr);

        Server::builder()
            .add_service(grpc::mos::mos_server::MosServer::new(mos_service))
            .serve(addr)
            .await
            .map_err(|e| MosError::SchedulerError(e.to_string()))?; // TODO: Better error type

        Ok(())
    }
}

// 提供便捷的初始化函数
pub async fn init(config_path: &str) -> Result<MosCore, MosError> {
    let config = Config::from_file(config_path)?;
    logging::init_logging(&config.log_level)
        .map_err(|e| MosError::SchedulerError(e.to_string()))?; // TODO: Better error type

    // Create the robot controller using the factory
    let robot_controller = create_robot_controller(&config.robot_controller).await?;

    MosCore::new(config, robot_controller)
}