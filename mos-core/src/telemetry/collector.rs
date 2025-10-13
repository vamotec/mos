use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::broadcast;
use tokio::time;
use crate::types::TelemetryData;

pub struct TelemetryCollector;

impl TelemetryCollector {
    pub async fn run(tx: broadcast::Sender<TelemetryData>) {
        log::info!("Telemetry collector is running.");
        let mut interval = time::interval(Duration::from_secs(1));
        let mut phase: f64 = 0.0;

        loop {
            interval.tick().await;

            // Generate a sine wave as mock sensor data
            phase += 0.1;
            let value = phase.sin() * 100.0;

            let data = TelemetryData {
                source: "mock_sensor".to_string(),
                value,
                timestamp: SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs(),
            };

            if let Err(e) = tx.send(data) {
                log::error!("Failed to send telemetry data: {}", e);
            }
        }
    }
}
