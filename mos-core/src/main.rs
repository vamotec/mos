use mos_core::{init, MosError};

#[tokio::main]
async fn main() -> Result<(), MosError> {
    // Initialize the MosCore instance using the new async init function.
    // This will read the config, set up logging, and create the appropriate
    // RobotController (e.g., Mock or Grpc) based on the config file.
    let core = init("config.toml").await?;

    // Run the core services (gRPC server, scheduler, etc.)
    core.run().await
}