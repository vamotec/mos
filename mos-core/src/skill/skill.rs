//! Defines the `Skill` trait and the `WasmSkill` implementation.

#![cfg(feature = "wasm")]

use async_trait::async_trait;
use crate::MosError;
use crate::skill::wasm_runtime::WasmRuntime;

/// Represents a skill that can be executed by the mos-core.
#[async_trait]
pub trait Skill: Send {
    /// Returns the name of the skill.
    fn name(&self) -> &str;

    /// Calls the skill with a given command (as raw bytes) and returns the result.
    /// This provides a generic interface for interacting with skills.
    async fn call(&mut self, command: &[u8]) -> Result<Vec<u8>, MosError>;
}

/// A skill implementation that is backed by a WebAssembly module.
pub struct WasmSkill {
    name: String,
    runtime: WasmRuntime,
}

impl WasmSkill {
    /// Creates a new WasmSkill from a given WASM module's bytes.
    pub fn new(name: &str, wasm_bytes: &[u8]) -> Result<Self, MosError> {
        let runtime = WasmRuntime::new(wasm_bytes)?;
        Ok(Self {
            name: name.to_string(),
            runtime,
        })
    }
}

#[async_trait]
impl Skill for WasmSkill {
    fn name(&self) -> &str {
        &self.name
    }

    async fn call(&mut self, command: &[u8]) -> Result<Vec<u8>, MosError> {
        // Here, we would call a specific exported function from the WASM module,
        // for example, a function named "invoke".
        // The actual implementation depends on the ABI defined between the host and the skill.
        // For now, we'll use a placeholder.
        self.runtime.call("invoke", command).await
    }
}
