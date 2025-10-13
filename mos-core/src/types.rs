use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Task {
    pub id: u64,
    pub priority: u32,
    pub state: TaskState,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TaskState {
    Pending,
    Running,
    Completed,
}

#[derive(Debug, Clone)]
pub struct TelemetryData {
    pub source: String,
    pub value: f64,
    pub timestamp: u64,
}