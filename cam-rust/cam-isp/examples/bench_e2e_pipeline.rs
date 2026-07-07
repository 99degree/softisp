//! E2E benchmark: ONNX generation → MNN conversion → Vulkan inference.
//!
//! Measures the full pipeline throughput at HD, FHD, and 4K.
//! Uses the engine abstraction (auto-selects Vulkan if available).
//!
//! Usage:
//!   ENGINE=vulkan cargo run --example bench_e2e_pipeline -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::pipeline::IspBlock;
use cam_isp::engine::{ProcessParams, OutputFormat};
use std::time::Instant;

fn bench_resolution(name: &str, w: u32, h: u32, iterations: u32) {
    println!("{} ({}x{}):", name, w, h);

    // Build pipeline
    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(UnpackBlock::new()
            .with_concrete_dims(h as i64, w as i64)),
        Box::new(DemosaicCcmBlock::new(0)),
        Box::new(WarpGridBlock::new(w, h)
            .with_gdc(-0.1, 0.0, 0.0)
            .with_lens_shading(1.2, 1.0)),
        Box::new(DisplayBlock::new(w)),
    ];

    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();

    // Generate ONNX
    let t0 = Instant::now();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(
        &block_refs, &[], 8).unwrap();
    let emit_ms = t0.elapsed().as_secs_f64() * 1000.0;
    println!("  ONNX emit:  {:.2} ms  ({} bytes)", emit_ms, onnx.len());

    // Register engines
    cam_isp::cpu::register_cpu_engine();
    #[cfg(feature = "mnn")]
    {
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
        cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);
    }

    // Select engine
    let engine_name = std::env::var("ENGINE").unwrap_or_else(|_| "vulkan".to_string());
    let mut engine = match cam_isp::engine::select_engine_by_name(&engine_name) {
        Some(e) => e,
        None => match cam_isp::engine::select_engine() {
            Some(e) => e,
            None => Box::new(cam_isp::cpu::CpuEngine::new()),
        },
    };
    println!("  Engine: {}", engine.backend_name());

    // Build pipeline
    let head = blocks.remove(0);
    let all = blocks;
    if let Err(e) = engine.build(head, all, None, 21) {
        println!("  Build FAILED: {}", e);
        return;
    }

    // Test input: INT16 bayer [1, 1, H, W]
    let raw: Vec<u16> = vec![512; (w * h) as usize];
    let mut raw_bytes = Vec::with_capacity(raw.len() * 2);
    for v in &raw {
        raw_bytes.extend_from_slice(&v.to_le_bytes());
    }
    let mut params = ProcessParams::new(w, h, &raw_bytes);
    params.target_width = w;
    params.target_height = h;
    params.sensor_max = 1023.0;
    params.output_format = OutputFormat::FloatBgra;

    // Warmup
    for _ in 0..3 {
        let _ = engine.process(&params);
    }

    // Timed iterations
    let mut times = Vec::new();
    for _ in 0..iterations {
        let t2 = Instant::now();
        let _ = engine.process(&params);
        times.push(t2.elapsed().as_secs_f64() * 1000.0);
    }

    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let avg: f64 = times.iter().sum::<f64>() / times.len() as f64;
    let p50 = times[times.len() / 2];
    let p99 = { let idx = ((times.len() as f64 * 0.99) as usize).min(times.len() - 1); times[idx] };

    println!("  Inference ({} iter):", iterations);
    println!("    Avg:   {:.2} ms  ({:.0} FPS)", avg, 1000.0 / avg);
    println!("    P50:   {:.2} ms", p50);
    println!("    P99:   {:.2} ms", p99);
}

fn main() {
    println!("=== E2E Pipeline Benchmark ===\n");
    bench_resolution("HD",  1280, 720, 20);
    println!();
    bench_resolution("FHD", 1920, 1080, 20);
    println!();
    bench_resolution("4K",  3840, 2160, 10);
    println!("\nDone.");
}
