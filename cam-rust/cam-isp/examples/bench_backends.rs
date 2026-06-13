//! Quick benchmark: call cam_isp::init() which runs MNN backend benchmark.
//! Run:
//!   cd cam-rust
//!   RUST_LOG=debug LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     cargo run --example bench_backends -p cam-isp --features mnn

fn main() {
    // Init logger so cam_isp's info/debug messages show
    env_logger::init();

    // This triggers register_factories() which benchmarks all MNN backends
    cam_isp::init();

    println!("Done.");
}
