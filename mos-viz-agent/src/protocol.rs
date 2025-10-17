use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Represents an incoming message from rosbridge_server.
/// We are primarily interested in the 'publish' operation.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct RosbridgeMsg {
    pub op: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub msg: Option<Value>,
}

/// Represents the message structure we send to the MOS Cloud.
/// It wraps the original rosbridge message with additional metadata.
#[derive(Serialize, Deserialize, Debug)]
pub struct CloudMsg {
    pub r#type: String, // Using r# to allow 'type' as a field name
    pub timestamp_ms: u128,
    pub payload: RosbridgeMsg,
}

/// Represents a command coming from the MOS Cloud, to be executed by the agent.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CloudCommand {
    pub op: String, // e.g., "set_joint_angles"
    // Using Value allows for flexible arguments for different commands
    pub args: Value,
}

/// Specific arguments for the "set_joint_angles" command.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SetJointsArgs {
    pub joint_positions: Vec<f64>,
}

