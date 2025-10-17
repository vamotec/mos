use crate::hal::robot_controller::RobotControllerConfig;
use serde::Deserialize;
use std::fs;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("Failed to read config file: {0}")]
    ReadFile(#[from] std::io::Error),
    #[error("Failed to parse config file: {0}")]
    Parse(#[from] toml::de::Error),
}

#[derive(Deserialize, Clone, Debug)]
pub struct Config {
    #[serde(default = "default_log_level")]
    pub log_level: String,
    #[serde(default = "default_scheduler_max_tasks")]
    pub scheduler_max_tasks: usize,
    #[serde(default = "default_grpc_port")]
    pub grpc_port: u16,
    #[serde(default = "default_core_grpc_socket")]
    pub core_grpc_socket: String,
    #[serde(default = "default_wasm_runtime")]
    pub wasm_runtime: String,
    #[serde(default)]
    pub robot_controller: RobotControllerConfig,
}

fn default_log_level() -> String { "info".to_string() }
fn default_scheduler_max_tasks() -> usize { 100 }
fn default_core_grpc_socket() -> String { "/tmp/mos-core.sock".to_string() }
fn default_wasm_runtime() -> String { "wasmtime".to_string() }

fn default_grpc_port() -> u16 { 50052 }

impl Config {
    pub fn from_file(path: &str) -> Result<Self, ConfigError> {
        let contents = fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            log_level: default_log_level(),
            scheduler_max_tasks: default_scheduler_max_tasks(),
            grpc_port: default_grpc_port(),
            core_grpc_socket: default_core_grpc_socket(),
            wasm_runtime: default_wasm_runtime(),
            robot_controller: RobotControllerConfig::default(),
        }
    }
}