//! HEAVY profile benchmark: full ISP + all stages + GPU format conversion.
//!
//! Tests a realistic camera pipeline with all processing stages.
//!
//! Usage:
//!   cargo run --example bench_heavy_formats -p cam-isp --features mnn --release

use cam_isp::blocks::*;
use cam_isp::engine::OutputFormat;
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;
use std::time::Instant;

fn bench_heavy(name: &str, w: u32, h: u32, fmt: OutputFormat, iters: u32) {
    let profile = PipelineProfile::HEAVY;
    let blocks = profile.build_blocks(w, 0); // RGGB
    let block_count = blocks.len();
    let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = cam_isp::pipeline::GraphComposer::compose_from_vec(&block_refs, &[], 21).unwrap();

    cam_isp::cpu::register_cpu_engine();
    cam_isp::register_mnn_engine!(cam_isp::mnnengine::MnnBackend::Vulkan);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").unwrap();
    println!("  Engine: {}", engine.backend_name());
    let mut block_vec = blocks;
    let head = block_vec.remove(0);
    let aux = block_vec;
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

    for _ in 0..3 {
        let _ = engine.process(&params);
    }

    let mut times = Vec::new();
    let mut errors = 0u32;
    for _ in 0..iters {
        let t = Instant::now();
        match engine.process(&params) {
            Ok(frame) => {
                times.push(t.elapsed().as_secs_f64() * 1000.0);
                if times.len() == 1 {
                    println!(
                        "  First frame: {}×{} data={}B",
                        frame.width,
                        frame.height,
                        frame.data.len()
                    );
                }
            }
            Err(e) => {
                errors += 1;
                if errors == 1 {
                    println!("  ERROR: {:?}", e);
                }
            }
        }
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if times.is_empty() {
        println!(
            "  {:6} {:12}  {} blocks  ALL {} FRAMES FAILED",
            name,
            format!("{:?}", fmt),
            block_count,
            errors
        );
        return;
    }
    let avg: f64 = times.iter().sum::<f64>() / times.len() as f64;
    let p50 = times[times.len() / 2];
    println!(
        "  {:6} {:12}  {} blocks  {:>6.2}ms ({:>4.0} FPS)  p50={:.2}ms  errors={}",
        name,
        format!("{:?}", fmt),
        block_count,
        avg,
        1000.0 / avg,
        p50,
        errors
    );
}

fn main() {
    println!("=== HEAVY Profile Benchmark: Full ISP Pipeline + GPU Format Conversion ===\n");
    println!("Pipeline stages: BLC → WB → Demosaic → CCM → Gamma → EE → LDC → Denoise → Display\n");

    println!("── 4K (3840×2160) ──");
    bench_heavy("4K", 3840, 2160, OutputFormat::Argb, 10);
    bench_heavy("4K", 3840, 2160, OutputFormat::Rgba, 10);
    bench_heavy("4K", 3840, 2160, OutputFormat::FloatRgb, 10);
    bench_heavy("4K", 3840, 2160, OutputFormat::Float16Rgb, 10);
    println!();

    println!("── FHD (1920×1080) ──");
    bench_heavy("FHD", 1920, 1080, OutputFormat::Argb, 20);
    bench_heavy("FHD", 1920, 1080, OutputFormat::Rgba, 20);
    bench_heavy("FHD", 1920, 1080, OutputFormat::FloatRgb, 20);
    bench_heavy("FHD", 1920, 1080, OutputFormat::Float16Rgb, 20);
    println!();

    println!("── HD (1280×720) ──");
    bench_heavy("HD", 1280, 720, OutputFormat::Argb, 30);
    bench_heavy("HD", 1280, 720, OutputFormat::Rgba, 30);
    bench_heavy("HD", 1280, 720, OutputFormat::FloatRgb, 30);
    bench_heavy("HD", 1280, 720, OutputFormat::Float16Rgb, 30);

    println!("\nDone.");
}
