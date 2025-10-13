#![cfg(feature = "wasm")]

use crate::MosError;
use wasmtime::*;
// Use the correct, modern builder and context struct.
use wasmtime_wasi::WasiCtxBuilder;
// Use the precise path to the preview1 linker function as you pointed out.
use wasmtime_wasi::preview1::{WasiP1Ctx, wasi_snapshot_preview1};

pub struct WasmRuntime {
    // The store must own the context struct.
    store: Store<WasiP1Ctx>,
    instance: Instance,
}

impl WasmRuntime {
    /// Creates a new runtime and instance from the given WASM bytes using WASI Snapshot Preview 1.
    pub fn new(wasm_bytes: &[u8]) -> Result<Self, MosError> {
        let engine = Engine::default();
        let mut linker = Linker::new(&engine);

        // Link WASI imports using the specific, correct path.
        wasi_snapshot_preview1::add_to_linker(&mut linker, |ctx: &mut WasiP1Ctx| ctx)?;

        // Configure and build the WASI context using the modern builder.
        let wasi_ctx = WasiCtxBuilder::new()
            .inherit_stdio()
            .build_p1();

        let mut store = Store::new(&engine, wasi_ctx);

        // Load the module
        let module =
            Module::from_binary(&engine, wasm_bytes).map_err(|e| MosError::WasmError(e.to_string()))?;

        // Instantiate the module
        let instance = linker
            .instantiate(&mut store, &module)
            .map_err(|e| MosError::WasmError(e.to_string()))?;

        Ok(Self { store, instance })
    }

    /// Calls an exported function on the WASM module.
    pub async fn call(&mut self, func_name: &str, _command: &[u8]) -> Result<Vec<u8>, MosError> {
        let func = self
            .instance
            .get_typed_func::<(), ()>(&mut self.store, func_name)
            .map_err(|e| {
                MosError::WasmError(format!(
                    "Failed to get exported function '{}': {}",
                    func_name, e
                ))
            })?;

        func.call_async(&mut self.store, ())
            .await
            .map_err(|e| MosError::WasmError(e.to_string()))?;

        // Placeholder for return value.
        Ok(Vec::new())
    }
}
