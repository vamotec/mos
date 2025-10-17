use crate::config::ConfigError;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum MosError {
    #[error("Config error: {0}")]
    Config(#[from] ConfigError),

    #[error("Scheduler error: {0}")]
    SchedulerError(String),

    #[error("WASM error: {0}")]
    WasmError(String),

    #[error("WASI error: {0}")]
    WasiError(#[from] anyhow::Error),

    #[error("HAL error: {0}")]
    HalError(String),

    #[error("Robot controller error: {0}")]
    RobotControllerError(String),

    #[error("Other error: {0}")]
    Other(String),
}