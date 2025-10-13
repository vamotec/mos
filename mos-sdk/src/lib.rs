
use anyhow::Result;

// Re-export the generated gRPC client and request/response types
pub mod grpc {
    pub mod mos;
}

// Re-export core data types for convenience
pub use mos_core::types;

use grpc::mos::mos_client::MosClient;

/// Establishes a connection to the mos-core gRPC server.
///
/// # Arguments
///
/// * `addr` - The address of the server (e.g., "http://mosia.app:50051").
///
/// # Returns
///
/// A `Result` containing the connected `MosClient` or an error.
pub async fn connect(addr: String) -> Result<MosClient<tonic::transport::Channel>> {
    let client = MosClient::connect(addr).await?;
    Ok(client)
}
