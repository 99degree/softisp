//! Full pipeline benchmark: 4K/HD/FHD → various output formats on Vulkan.
//!
//! Usage:
//!   cargo run --example bench_unified_full -p cam-isp --features mnn --release

use cam_isp::engine::OutputFormat;
use cam_isp::unified_pipeline::{UnifiedConfig, GpuWarpParams};
use cam_isp::pipeline::IspBlock;
use cam_isp::blocks::*;
use std::time::Instant;

fn bench_raw_pipeline(name: &str, w: u32, h: u32, fmt: OutputFormat, iters: u32) {
    // Build ISP blocks manually for raw benchmark
    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(UnpackBlock::new().with_concrete_dims(h as i64, w as i64)),
        Box::new(DemosaicCcmBlock::new(0)),
        Box::new(DisplayBlock::new(w).with_output_format(fmt)),
    ];
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    // Register engines
    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Cpu);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    let head = blocks.remove(0);
    let aux = blocks;
    engine.build(head, aux, None, 21).unwrap();

    let raw: Vec<u16> = vec![512; (w * h) as usize];
    let mut raw_bytes = Vec::with_capacity(raw.len() * 2);
    for v in &raw {
        raw_bytes.extend_from_slice(&v.to_le_bytes());
    }
    let mut params = cam_isp::engine::ProcessParams::new(w, h, &raw_bytes);
    params.target_width = w;
    params.target_height = h;
    params.sensor_max = 1023.0;
    params.output_format = fmt;

    // Warmup
    for _ in 0..3 { let _ = engine.process(&params); }

    // Timed
    let mut times = Vec::new();
    for _ in 0..iters {
        let t = Instant::now();
        let _ = engine.process(&params);
        times.push(t.elapsed().as_secs_f64() * 1000.0);
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let avg: f64 = times.iter().sum::<f64>() / times.len() as f64;
    let p50 = times[times.len() / 2];
    println!("  {:12} {:?}:  avg={:.2}ms ({:.0} FPS)  p50={:.2}ms", name, fmt, avg, 1000.0 / avg, p50);
}

fn main() {
    println!("=== ISP + DisplayBlock GPU Format Conversion Benchmark ===\n");

    println!("── 4K (3840×2160) ──");
    bench_raw_pipeline("4K", 3840, 2160, OutputFormat::Argb, 20);
    bench_raw_pipeline("4K", 3840, 2160, OutputFormat::Rgba, 20);
    bench_raw_pipeline("4K", 3840, 2160, OutputFormat::FloatRgb, 20);
    bench_raw_pipeline("4K", 3840, 2160, OutputFormat::Float16Rgb, 20);
    bench_raw_pipeline("4K", 3840, 2160, OutputFormat::Abgr, 20);
    println!();

    println!("── FHD (1920×1080) ──");
    bench_raw_pipeline("FHD", 1920, 1080, OutputFormat::Argb, 30);
    bench_raw_pipeline("FHD", 1920, 1080, OutputFormat::Rgba, 30);
    bench_raw_pipeline("FHD", 1920, 1080, OutputFormat::FloatRgb, 30);
    bench_raw_pipeline("FHD", 1920, 1080, OutputFormat::Float16Rgb, 30);
    bench_raw_pipeline("FHD", 1920, 1080, OutputFormat::Abgr, 30);
    println!();

    println!("── HD (1280×720) ──");
    bench_raw_pipeline("HD", 1280, 720, OutputFormat::Argb, 50);
    bench_raw_pipeline("HD", 1280, 720, OutputFormat::Rgba, 50);
    bench_raw_pipeline("HD", 1280, 720, OutputFormat::FloatRgb, 50);
    bench_raw_pipeline("HD", 1280, 720, OutputFormat::Float16Rgb, 50);
    bench_raw_pipeline("HD", 1280, 720, OutputFormat::Abgr, 50);

    println!("\nDone.");
}
