use super::mos;
use crate::scheduler::scheduler::TaskCommand;
use crate::types::{Task, TaskState, TelemetryData};
use std::pin::Pin;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::{broadcast, mpsc};
use tokio_stream::{Stream, wrappers::ReceiverStream};
use tonic::{Request, Response, Status};

pub struct MosService {
    scheduler_tx: mpsc::Sender<TaskCommand>,
    telemetry_tx: broadcast::Sender<TelemetryData>,
    // Use an atomic counter for mock task IDs
    task_id_counter: Arc<AtomicU64>,
}

impl MosService {
    pub fn new(
        scheduler_tx: mpsc::Sender<TaskCommand>,
        telemetry_tx: broadcast::Sender<TelemetryData>,
    ) -> Self {
        Self {
            scheduler_tx,
            telemetry_tx,
            task_id_counter: Arc::new(AtomicU64::new(1)),
        }
    }
}

#[tonic::async_trait]
impl mos::mos_server::Mos for MosService {
    async fn schedule_task(
        &self,
        request: Request<mos::TaskRequest>,
    ) -> Result<Response<mos::TaskResponse>, Status> {
        let req = request.into_inner();

        // `optional int64` is represented as `Option<i64>`.
        // If the client omits the ID, it will be `None`.
        let task_id = match req.id {
            Some(id) if id > 0 => id as u64, // Use client-provided ID if it's positive.
            _ => self.task_id_counter.fetch_add(1, Ordering::SeqCst), // Otherwise, generate a new ID.
        };

        let task = Task {
            id: task_id,
            priority: req.priority,
            state: TaskState::Pending,
        };

        log::info!("Received schedule_task request: {:?}", task);

        self.scheduler_tx
            .send(TaskCommand::AddTask(task))
            .await
            .map_err(|e| {
                log::error!("Failed to send task to scheduler: {}", e);
                Status::internal("Failed to schedule task")
            })?;

        Ok(Response::new(mos::TaskResponse { id: task_id }))
    }

    type StreamTelemetryStream =
        Pin<Box<dyn Stream<Item = Result<mos::TelemetryData, Status>> + Send + 'static>>;

    async fn stream_telemetry(
        &self,
        _request: Request<tonic::Streaming<mos::TelemetryRequest>>,
    ) -> Result<Response<Self::StreamTelemetryStream>, Status> {
        let (tx, rx) = mpsc::channel(128);
        let mut telemetry_rx = self.telemetry_tx.subscribe();

        tokio::spawn(async move {
            loop {
                match telemetry_rx.recv().await {
                    Ok(data) => {
                        let response = mos::TelemetryData {
                            source: data.source,
                            value: data.value,
                            timestamp: data.timestamp,
                        };
                        if tx.send(Ok(response)).await.is_err() {
                            // Receiver dropped, so we can stop.
                            log::info!("Telemetry stream client disconnected.");
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        log::warn!("Telemetry stream lagged by {} messages.", n);
                    }
                    Err(broadcast::error::RecvError::Closed) => {
                        break;
                    }
                }
            }
        });

        let stream = ReceiverStream::new(rx);
        Ok(Response::new(Box::pin(stream) as Self::StreamTelemetryStream))
    }
}