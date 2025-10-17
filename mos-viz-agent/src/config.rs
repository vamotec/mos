use serde::Deserialize;
use std::path::Path;

// This struct maps to the whole config.toml file
#[derive(Deserialize, Debug, Clone, Default)]
pub struct Config {
    #[serde(default)]
    pub robot_controller: RobotControllerConfig,
    #[serde(default)]
    pub viz_agent: VizAgentConfig,
}

// This struct maps to the [robot_controller] section
#[derive(Deserialize, Debug, Clone, Default)]
pub struct RobotControllerConfig {
    // For development on Mac/Windows
    pub ros2_grpc_address: Option<String>,
    // For production/Linux
    pub ros2_grpc_socket: Option<String>,
}

// This struct maps to the [viz_agent] section
#[derive(Deserialize, Debug, Default, Clone)]
pub struct VizAgentConfig {
    #[serde(default = "default_rosbridge_url")]
    pub rosbridge_url: String,
    #[serde(default = "default_cloud_url")]
    pub cloud_url: String,
}

fn default_rosbridge_url() -> String {
    "ws://127.0.0.1:9090".to_string()
}

fn default_cloud_url() -> String {
    "wss://viz.mos-cloud.com/ingress".to_string()
}

pub fn load_config<P: AsRef<Path>>(path: P) -> Result<Config, anyhow::Error> {
    let content = std::fs::read_to_string(path)?;
    let config: Config = toml::from_str(&content)?;
    Ok(config)
}
