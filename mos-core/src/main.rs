use mos_core::{init, MosError};

#[tokio::main]
async fn main() -> Result<(), MosError> {
    let core = init("config.toml")?;
    core.run().await
}