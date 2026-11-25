//! Dual-GPU 4K→FHD benchmark — measure whether dual MNN/Vulkan engines
//! overlap inference across frames and whether MNN serialization remains
//! the bottleneck at 4K.
//!
//! Runs two MnnEngine instances (engine A / engine B) in round-robin order
//! (DualGpuEngine) so frame N goes to engine A, frame N+1 to engine B.
//! Measures end-to-end throughput over 50 frames and reports avg ms/frame
//! and FPS. Compares against single-engine throughput to show whether the
//! second engine helps or whether MNN serialization is still the bottleneck.
//!
//! Run:
//!   cargo run --release --example bench_4k_dual_engine -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::dual_engine::DualGpuEngine;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::MnnBackend;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use std::time::Instant;

fn build_chain(w: u32, h: u32) -> (Box<dyn IspBlock>, Vec<Box<dyn IspBlock>>) {
    let full_w = w as i64;
    let full_h = h as i64;
    let _pipe_w = 1920u32;
    let pipe_h = 1080u32;
    let post_w = 960u32;
    let post_h = 540u32;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(RawInput16Block::new().with_concrete_dims(full_h, full_w)),
        Box::new(
            UnpackCfaBlock::new()
                .with_mode(cam_isp::blocks::UnpackMode::NativeInt16)
                .with_fast_unpack(true)
                .with_concrete_dims(full_h, full_w)
                .with_downscale(4)
                .with_sensor_max(1023.0)
                .with_blc(true),
        ),
        Box::new(
            DemosaicCcmBlock::new(0).with_concrete_dims(pipe_h as i64, post_w as i64),
        ),
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),
        Box::new(
            DisplayBlock::new(post_w)
                .with_output_format(OutputFormat::FloatRgb)
                .with_concrete_dims(post_h as i64, post_w as i64),
        ),
    ];

    GraphComposer::wire_blocks(&mut blocks);
    let head = blocks.remove(0);
    (head, blocks)
}

fn build_engine(name: &str, head: Box<dyn IspBlock>, aux: Vec<Box<dyn IspBlock>>) -> Box<dyn IspEngine> {
    let mut engine = cam_isp::mnnengine::MnnEngine::new(MnnBackend::Vulkan);
    engine.build(head, aux, None, 21).expect("engine.build failed");
    Box::new(engine)
}

fn main() {
    cam_isp::init();

    let sensor_w = 3840u32;
    let sensor_h = 2160u32;
    let n_frames = 50;

    // Generate 4K Bayer test data as INT16
    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF;
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }
    let raw = raw_buf;

    // Build two identical engines for dual-GPU ping-pong
    let (head_a, aux_a) = build_chain(sensor_w, sensor_h);
    let (head_b, aux_b) = build_chain(sensor_w, sensor_h);

    let engine_a = build_engine("A", head_a, aux_a);
    let engine_b = build_engine("B", head_b, aux_b);

    let mut dual = DualGpuEngine::new(engine_a, engine_b);

    // Shared params
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = 1920;
    params.target_height = 1080;
    params.sensor_max = 1023.0;
    params.output_format = OutputFormat::FloatRgb;

    // Warmup (3 frames, one per engine where possible)
    for _ in 0..3 {
        let _ = dual.next().process(&params);
    }

    // Timed run
    let mut sum_ms = 0.0f64;
    let mut min_ms = f64::INFINITY;
    let mut max_ms = 0.0f64;
    let mut errors = 0u32;

    println!("Running {} frames on dual MNN/Vulkan engines...\n", n_frames);

    let t0 = Instant::now();
    for i in 0..n_frames {
        let t_frame = Instant::now();
        let result = dual.next().process(&params);
        let ms = t_frame.elapsed().as_secs_f64() * 1000.0;
        match result {
            Ok(frame) => {
                sum_ms += ms;
                min_ms = min_ms.min(ms);
                max_ms = max_ms.max(ms);
                if i < 5 || i == n_frames - 1 {
                    println!(
                        "  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)",
                        i,
                        frame.width,
                        frame.height,
                        ms,
                        frame.data.len()
                    );
                }
            }
            Err(e) => {
                errors += 1;
                println!("  [{:2}] ERROR: {}  ({:.1}ms)", i, e, ms);
            }
        }
    }
    let total_elapsed = t0.elapsed().as_secs_f64();

    let avg_ms = sum_ms / n_frames as f64;

    println!();
    println!("=== Dual-GPU 4K→FHD Results ===");
    println!("  Frames:       {}", n_frames);
    println!("  Errors:       {}", errors);
    println!("  Avg:          {:.2} ms/frame", avg_ms);
    println!("  Min:          {:.2} ms", min_ms);
    println!("  Max:          {:.2} ms", max_ms);
    println!("  Throughput:   {:.1} FPS", n_frames as f64 / total_elapsed);
    println!("  (wall-clock total: {:.2} s)", total_elapsed);

    if avg_ms < 33.3 {
        println!("  ✅ Exceeds 30fps target!");
    } else {
        println!("  ❌ Misses 30fps target (33.3ms/frame)");
    }

    // Compare against single engine
    println!();
    println!("  To compare, run: cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn");
    println!("  and note the single-engine FPS vs dual-engine FPS above.");
}
