//! Defines the Foreign Function Interface (FFI) for WASM skills.
//!
//! This module defines the contract between the `mos-core` host and the
//! WASM skill guests. It specifies the function signatures that the host

//! expects to find in the WASM module and can call.
//!
//! For a `wasmtime`-based runtime, this does not require `wasm_bindgen`.
//! Instead, it's about agreeing on function names, parameters, and return
//! types that can be passed across the WASM boundary.

#![cfg(feature = "wasm")]

// Example of a function signature that a skill might be expected to export.
// The host would then look for a function named `invoke` in the WASM module.
//
// pub const SKILL_INVOKE_FUNCTION_NAME: &str = "invoke";
//
// The function signature in the guest (the skill written in Rust) would look like:
//
// #[no_mangle]
// pub extern "C" fn invoke(ptr: *mut u8, len: usize) -> u64 {
//     // ... implementation ...
//     // The return value could be a pointer/length pair packed into a u64.
// }

// For now, this file is a placeholder for these definitions.
