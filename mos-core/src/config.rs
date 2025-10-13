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

#[derive(Deserialize)]
pub struct Config {
    pub log_level: String,
    pub scheduler_max_tasks: usize,
    pub grpc_port: u16,
    pub wasm_runtime: String, // e.g., "wasmtime"
}

impl Config {
    pub fn from_file(path: &str) -> Result<Self, ConfigError> {
        let contents = fs::read_to_string(path)?;
        let config: Config = toml::from_str(&contents)?;
        Ok(config)
    }
}
