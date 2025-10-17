fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_prost_build::configure()
        .build_client(true) // We only need to generate the client code
        .build_server(false) // We don't need the server code
        .compile_protos(
            &["../mos-core/proto/robot_controller.v1.proto"], // The proto file to compile
            &["../mos-core/proto", "proto/"], // The directory to search for imports
        )?;
    Ok(())
}
