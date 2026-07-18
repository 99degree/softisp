//! MNN GPU benchmark with proper synchronization.
//!
//! Uses `mnn_benchmark_sync` (tensor map/unmap) to get accurate GPU->CPU
//! end-to-end latency — not the async fire-and-forget illusion.
//!
//! Run:
//!   LD_LIBRARY_PATH=cam-rust/lib/aarch64-v8a \
//!     cargo run --example bench_mnn_gpu -p cam-isp --features mnn -- \
//!     /path/to/model.mnn [warmup] [loops] [precision]
//!
//! precision: 0=high, 1=normal, 2=low(FP16)

use std::env;
use std::time::Instant;

/// Map MNNForwardType int to a name.
fn forward_type_name(code: i32) -> &'static str {
    match code {
        0 => "CPU",
        1 => "OpenCL",
        3 => "Metal",
        7 => "Vulkan",
        _ => "Other",
    }
}

fn main() {
    let _ = env_logger::builder()
        .is_test(false)
        .filter_level(log::LevelFilter::Warn)
        .try_init();

    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!(
            "Usage: bench_mnn_gpu <model.mnn> [warmup] [loops] [precision]\n\
             \n\
             precision: 0=high, 1=normal, 2=low(FP16)\n\
             \n\
             This benchmark uses mnn_benchmark_sync() which forces GPU->CPU\n\
             synchronization via tensor map/unmap, giving accurate latency."
        );
        std::process::exit(1);
    }

    let model_path = &args[0];
    let warmup: i32 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(10);
    let loops: i32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(50);
    let precision: i32 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(2);

    let backend_name = "Vulkan/MNN";
    eprintln!(
        "MNN GPU Benchmark (sync)\n\
         model:   {model_path}\n\
         backend: {backend_name}\n\
         warmup:  {warmup}\n\
         loops:   {loops}\n\
         precision: {precision} ({})\n",
        match precision {
            0 => "HIGH",
            1 => "NORMAL",
            2 => "LOW/FP16",
            _ => "???",
        }
    );

    // Load model
    let t0 = Instant::now();
    let interp = cam_isp::mnn_sys::MnnInterpreterSafe::from_file(model_path)
        .expect("Failed to load MNN model");
    eprintln!("  model loaded in {:.1?}", t0.elapsed());

    // Create session with Vulkan backend
    let backend = cam_isp::mnn_sys::MnnBackendType::Vulkan;
    let session = interp
        .create_session(backend, 4)
        .expect("Failed to create Vulkan session");
    eprintln!("  Vulkan session created");

    // Resize
    session.resize().expect("Session resize failed");
    eprintln!("  session resized");

    // Query actual backend (may differ from requested due to fallback)
    let actual_backend = session.get_actual_backend();
    let actual_name = forward_type_name(actual_backend);
    eprintln!("  actual backend: {actual_name} ({actual_backend})");
    if actual_backend != 7 {
        eprintln!("  NOTE: Vulkan not available, running on {actual_name}");
        eprintln!("  (GPU sync via map/unmap is only meaningful on Vulkan/OpenCL/Metal)");
    }

    // Run benchmark
    eprintln!("\nRunning {loops} iterations ({warmup} warmup)...\n");
    let t_bench = Instant::now();
    let (avg, min, max, _per_frame) = session
        .benchmark_sync(warmup, loops, precision)
        .expect("Benchmark failed");
    let total = t_bench.elapsed();

    // Print results
    let fps_avg = if avg > 0.0 { 1000.0 / avg } else { 0.0 };
    let fps_min = if max > 0.0 { 1000.0 / max } else { 0.0 }; // max latency = min FPS
    let fps_max = if min > 0.0 { 1000.0 / min } else { 0.0 }; // min latency = max FPS

    eprintln!("═══════════════════════════════════════════════════════");
    eprintln!("  MNN Vulkan Benchmark (GPU-synced via map/unmap)");
    eprintln!("═══════════════════════════════════════════════════════");
    eprintln!("  avg: {avg:8.3} ms  ({fps_avg:6.1} FPS)");
    eprintln!("  min: {min:8.3} ms  ({fps_max:6.1} FPS)");
    eprintln!("  max: {max:8.3} ms  ({fps_min:6.1} FPS)");
    eprintln!("  total: {:.1?} for {loops} iterations", total);
    eprintln!("═══════════════════════════════════════════════════════");

    // Model info
    if let Ok(mem) = session.get_model_info(cam_isp::mnn_sys::MnnModelInfo::MEMORY) {
        eprintln!("  model memory: {:.1} MB", mem);
    }
    if let Ok(flops) = session.get_model_info(cam_isp::mnn_sys::MnnModelInfo::FLOPS) {
        eprintln!("  model FLOPs:  {:.2}", flops);
    }
}
