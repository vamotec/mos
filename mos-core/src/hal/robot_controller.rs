// mos-core/src/hal/robot_controller.rs

use crate::error::MosError;

// The specific definition of the trait goes here
pub trait RobotController: Send + Sync {
    fn get_joint_angles(&self) -> Result<[f64; 6], MosError>;
    fn set_joint_angles(&self, angles: [f64; 6]) -> Result<(), MosError>;
}
