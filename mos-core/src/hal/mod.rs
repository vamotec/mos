// mos-core/src/hal/mod.rs

// 1. Declare that a submodule named robot_controller exists
pub mod robot_controller;

// 2. Re-export the RobotController trait from the submodule
// This allows external code to access it via mos_core::hal::RobotController
// instead of the longer mos_core::hal::robot_controller::RobotController
pub use robot_controller::RobotController;