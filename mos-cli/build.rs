use std::fs;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("cargo:rerun-if-changed=../mos-core/proto/mos.proto");

    // Ensure the output directory exists before code generation
    fs::create_dir_all("src/grpc")?;

    tonic_prost_build::configure()
        .build_client(true)
        .build_server(false) // We only need the client for the CLI
        .out_dir("src/grpc") // Output the generated code in a specific directory
        .compile_protos(
            &["../mos-core/proto/mos.proto"],
            &["../mos-core/proto"],
        )?;

    Ok(())
}