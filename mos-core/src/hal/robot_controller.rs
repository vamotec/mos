
// mos-core/src/hal/robot_controller.rs

use crate::error::MosError;
use crate::grpc::mos_robot_v1::{
    robot_controller_client::RobotControllerClient, MoveToJointTargetRequest, GetJointStateRequest,
};
use async_trait::async_trait;
use serde::Deserialize;
use std::sync::{Arc, Mutex};
use tonic::transport::Channel;

/// Defines the types of Robot Controllers available.
/// This enum is used for configuration purposes.
#[derive(Debug, Deserialize, Clone)]
#[serde(rename_all = "lowercase")]
pub enum RobotControllerType {
    Grpc,
    Mock,
}

/// Configuration for the Robot Controller.
#[derive(Debug, Deserialize, Clone)]
pub struct RobotControllerConfig {
    pub controller_type: RobotControllerType,
    pub address: Option<String>, // Only used for Grpc type
}

/// A trait defining the universal interface for controlling a robot.
/// `async_trait` is used to allow async methods in traits.
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
    pub async fn new(addr: String) -> Result<Self, MosError> {
        let client = RobotControllerClient::connect(addr)
            .await
            .map_err(|e| MosError::RobotControllerError(e.to_string()))?;
        Ok(Self { client })
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

/// A mock implementation of the RobotController for simulation and testing.
/// It simulates the robot's state in memory.
pub struct MockRobotController {
    /// Use Arc<Mutex<T>> for interior mutability in an async context.
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

/// Creates and returns a specific RobotController instance based on the configuration.
/// This returns a `Box<dyn RobotController>` which is a trait object, allowing
/// for different implementations to be used interchangeably.
pub async fn create_robot_controller(
    config: &RobotControllerConfig,
) -> Result<Box<dyn RobotController>, MosError> {
    match config.controller_type {
        RobotControllerType::Grpc => {
            let addr = config
                .address
                .clone()
                .ok_or_else(|| MosError::RobotControllerError("gRPC address not provided".into()))?;
            let controller = GrpcRobotController::new(addr).await?;
            Ok(Box::new(controller))
        }
        RobotControllerType::Mock => {
            // For mock, we can start with a default state, e.g., 6 joints at 0.0
            let controller = MockRobotController::new(vec![0.0; 6]);
            Ok(Box::new(controller))
        }
    }
}
