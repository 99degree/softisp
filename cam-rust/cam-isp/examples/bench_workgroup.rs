//! Benchmark workgroup sizes using MNN Vulkan backend.
//! Tests: query optimal, set preset, set explicit.
//!
//! Usage: cargo run --example bench_workgroup -p cam-isp --features mnn

use cam_isp::mnnengine::{MnnEngine, MnnBackend};
use std::time::Instant;

fn main() {
    println!("=== Workgroup Size Benchmark ===\n");

    let engine = MnnEngine::new(MnnBackend::Vulkan);
    let (opt_x, opt_y) = MnnEngine::query_optimal_workgroup();
    println!("GPU optimal workgroup: {}x{}\n", opt_x, opt_y);

    let sizes = vec![
        ("16x16", 16u32, 16u32),
        ("32x8",  32, 8),
        ("8x32",  8, 32),
        ("32x16", 32, 16),
        ("64x4",  64, 4),
        ("optimal", opt_x, opt_y),
    ];

    let model_path = std::env::args().nth(1)
        .unwrap_or_else(|| "/data/local/tmp/test_model.mnn".into());

    unsafe {
        use cam_isp::mnn_sys;
        use std::ffi::CString;

        let c_model = CString::new(model_path.as_str()).unwrap();
        let interp = mnn_sys::mnn_interpreter_create_from_file(c_model.as_ptr());
        if interp.is_null() {
            eprintln!("Failed to load: {}", model_path);
            return;
        }

        println!("{:>10} {:>10} {:>10}", "Size", "Avg(ms)", "FPS");
        println!("{:>10} {:>10} {:>10}", "----", "-------", "---");

        for (name, wx, wy) in &sizes {
            let session = mnn_sys::mnn_session_create(interp, mnn_sys::MnnBackendType::Vulkan, 1);
            if session.is_null() { continue; }

            // Set workgroup
            mnn_sys::MNNVulkanSetSessionWorkgroup(session as *mut _, *wx as i32, *wy as i32);

            // Init
            mnn_sys::mnn_session_run(interp, session);

            // Warmup
            for _ in 0..3 {
                mnn_sys::mnn_session_run(interp, session);
            }

            // Benchmark
            let iters = 20;
            let start = Instant::now();
            for _ in 0..iters {
                mnn_sys::mnn_session_run(interp, session);
            }
            let avg_ms = start.elapsed().as_secs_f64() * 1000.0 / iters as f64;
            let fps = 1000.0 / avg_ms;

            println!("{:>10} {:>10.2} {:>10.1}", name, avg_ms, fps);

            mnn_sys::mnn_session_release(interp, session);
        }

        mnn_sys::mnn_interpreter_destroy(interp);
    }
}
