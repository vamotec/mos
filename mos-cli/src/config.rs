use serde::Deserialize;
use std::path::Path;

#[derive(Deserialize, Debug)]
pub struct Config {
    #[serde(default = "default_core_grpc_socket")]
    pub core_grpc_socket: String,
}

fn default_core_grpc_socket() -> String {
    "run/mos-core.sock".to_string()
}

pub fn load_config<P: AsRef<Path>>(path: P) -> Result<Config, anyhow::Error> {
    let content = std::fs::read_to_string(path)?;
    let config: Config = toml::from_str(&content)?;
    Ok(config)
}
