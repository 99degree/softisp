//! Benchmark workgroup sizes using MNN Vulkan backend.
//! Tests the new profile API: query optimal, set preset, set explicit.
use cam_isp::mnnengine::MnnEngine;
use std::time::Instant;

fn main() {
    // Create Vulkan engine
    let engine = MnnEngine::create_vulkan(0);
    
    // Query optimal workgroup for this GPU
    let (opt_x, opt_y) = MnnEngine::query_optimal_workgroup();
    println!("GPU optimal workgroup: {}x{}", opt_x, opt_y);

    // Test various workgroup sizes
    let sizes = vec![
        ("wg16x16", 16u32, 16u32),
        ("wg32x8", 32, 8),
        ("wg8x32", 8, 32),
        ("wg32x16", 32, 16),
        ("wg64x4", 64, 4),
        ("wg4x64", 4, 64),
        ("fast_4k", 32, 8),
        ("optimal", opt_x, opt_y),
    ];

    // Load model
    let onnx_path = std::env::args().nth(1).expect("Usage: bench_workgroup <model.onnx>");
    let mnn_path = std::env::args().nth(2).unwrap_or_else(|| "/data/local/tmp/test_model.mnn".into());
    
    // Convert and load
    engine.set_model_path(&onnx_path);
    engine.convert_to_mnn(&mnn_path).expect("Convert failed");
    engine.load_model(&mnn_path).expect("Load failed");

    // Create session
    let session = engine.create_session().expect("Session failed");
    let input_w = 3840i64;
    let input_h = 2160i64;

    for (name, wx, wy) in &sizes {
        // Apply workgroup preset
        engine.set_workgroup_preset(name);
        // Also try explicit setting
        engine.set_workgroup_size(session as *mut _, *wx, *wy);
        
        // Create test input
        let input = engine.create_input(session as *mut _, "input", &[1, 1, input_h, input_w]).expect("Input failed");
        engine.set_input(session as *mut _, &input).expect("Set input failed");

        // Warmup
        for _ in 0..3 {
            let _ = engine.run(session as *mut _);
        }

        // Benchmark
        let iters = 20;
        let start = Instant::now();
        for _ in 0..iters {
            let _ = engine.run(session as *mut _);
        }
        let elapsed = start.elapsed();
        let avg_ms = elapsed.as_secs_f64() * 1000.0 / iters as f64;
        let fps = 1000.0 / avg_ms;

        println!("{:>12}: {:>6.2} ms  ({:>5.1} FPS)", name, avg_ms, fps);
    }
}
