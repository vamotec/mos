// mos-core/build.rs
fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_prost_build::configure()
        .out_dir("src/grpc") // 输出到 mos-core/src/grpc
        .compile_protos(&["proto/mos.proto"], &["proto"])?;
    Ok(())
}