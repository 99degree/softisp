//! Unified pipeline benchmark: 4K Bayer → FHD + ARGB8888 output.
//!
//! Tests the full pipeline with GPU format conversion.
//!
//! Usage:
//!   cargo run --example bench_unified_argb -p cam-isp --features mnn

use cam_isp::unified_pipeline::UnifiedConfig;
use cam_isp::engine::OutputFormat;
use std::time::Instant;

fn main() {
    println!("=== Unified Pipeline Benchmark: 4K Bayer → FHD ARGB8888 ===\n");

    // Register engines
    cam_isp::cpu::register_cpu_engine();
    #[cfg(feature = "mnn")]
    {
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);
    }

    // 4K input, FHD output
    let input_w = 3840;
    let input_h = 2160;
    let target_w = 1920;
    let target_h = 1080;

    // Create config with ARGB8888 output
    let config = UnifiedConfig {
        target_width: target_w,
        bayer_pattern: 0, // RGGB
        output_format: OutputFormat::Argb,
        gpu_warp_enabled: false,
        engine_preference: "vulkan".into(),
        ..Default::default()
    };

    println!("Config: {}×{} input → {}×{} output, format={:?}",
        input_w, input_h, target_w, target_h, config.output_format);

    // Build pipeline
    let t0 = Instant::now();
    let mut pipeline = match cam_isp::unified_pipeline::UnifiedPipeline::new(config) {
        Ok(p) => p,
        Err(e) => {
            println!("Pipeline build FAILED: {:?}", e);
            return;
        }
    };
    let build_ms = t0.elapsed().as_secs_f64() * 1000.0;
    println!("Pipeline build: {:.2} ms", build_ms);
    // pipeline.info() not available in this version

    // Generate test input: 4K INT16 Bayer
    let raw_size = (input_w * input_h * 2) as usize;
    let raw: Vec<u8> = vec![128u8; raw_size];

    // Warmup
    println!("\nWarmup (3 frames)...");
    for _ in 0..3 {
        let _ = pipeline.process(&raw, input_w, input_h);
    }

    // Benchmark
    let iterations = 20;
    println!("Benchmarking {} frames...", iterations);

    let mut times = Vec::new();
    for _ in 0..iterations {
        let t = Instant::now();
        let _ = pipeline.process(&raw, input_w, input_h);
        times.push(t.elapsed().as_secs_f64() * 1000.0);
    }

    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let avg: f64 = times.iter().sum::<f64>() / times.len() as f64;
    let p50 = times[times.len() / 2];
    let p99 = {
        let idx = ((times.len() as f64 * 0.99) as usize).min(times.len() - 1);
        times[idx]
    };

    println!("\nResults ({} iterations):", iterations);
    println!("  Avg: {:.2} ms  ({:.0} FPS)", avg, 1000.0 / avg);
    println!("  P50: {:.2} ms", p50);
    println!("  P99: {:.2} ms", p99);
    println!("\nDone.");
}
