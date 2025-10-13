extern crate proc_macro;

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// A procedural macro to mark a function as a "mos skill".
///
/// In the future, this macro could automatically handle:
/// - Registering the skill with the mos-core runtime.
/// - Setting up communication channels.
/// - Handling lifecycle events (start, stop, pause).
///
/// For now, it serves as a proof of concept.
#[proc_macro_attribute]
pub fn mos_skill(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the input tokens into a syntax tree
    let func = parse_macro_input!(item as ItemFn);

    // Check if the function is async
    if func.sig.asyncness.is_none() {
        return syn::Error::new_spanned(func.sig, "Skill function must be async")
            .to_compile_error()
            .into();
    }

    let func_name = &func.sig.ident;
    let func_block = &func.block;
    let func_inputs = &func.sig.inputs;
    let func_generics = &func.sig.generics;
    let func_output = &func.sig.output;

    // Build the output token stream
    let output = quote! {
        // This is where we could add registration logic in the future.
        // For now, we just reconstruct the function.
        #[allow(dead_code)] // The skill might not be called directly
        async fn #func_name #func_generics(#func_inputs) #func_output {
            println!("Initializing mos skill: {}", stringify!(#func_name));
            #func_block
        }
    };

    TokenStream::from(output)
}