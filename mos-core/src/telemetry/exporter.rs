use tonic::async_trait;
use crate::MosError;
#[async_trait]
pub trait Exporter {
    async fn export(&self, data: &[u8]) -> Result<(), MosError>;
}