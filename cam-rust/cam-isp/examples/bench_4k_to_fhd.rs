//! Benchmark 4K Bayer → FHD pipeline with fused downscale in UnpackCfaBlock.
//!
//! Uses UnpackCfaBlock.with_downscale(2) to fuse the 2× width stride
//! directly into the unpack Conv, eliminating the need for a separate
//! AdaptiveDownscaleBlock.  The UnpackCfaBlock already does stride=2 in
//! height (kernel=(2,1)), so with stride=2 in width (kernel=(2,2)) the
//! output is [1,4,H/2,W/4] packed = [1,4,1080,960] = FHD resolution.
//!
//! Bayer test data generated inline (deterministic pseudo-random).
//!
//! Run: LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!        cargo run --release --example bench_4k_to_fhd -p cam-isp --features mnn

use std::time::Instant;
use cam_isp::pipeline::{IspBlock, GraphComposer};
use cam_isp::engine::{IspEngine, ProcessParams};
use cam_isp::blocks::*;

fn main() {
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Debug).try_init();
    cam_isp::init();

    let sensor_w = 3840u32;   // UHD 4K sensor width
    let sensor_h = 2160u32;   // UHD 4K sensor height
    let pipe_w   = 1920u32;   // pipeline downscale target
    let pipe_h   = 1080u32;   // pipeline downscale target height
    let n_frames = 20;

    // Generate UHD Bayer test data inline (deterministic pseudo-random)
    use std::io::Write;
    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF; // 10-bit
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }
    let raw = raw_buf;

    // ── Build pipeline blocks ──────────────────────────────────────
    // Sensor: 3840×2160 packed → unpack_cfa(stride_w=2) → aux_hook_src
    //   → lsc → bayer_wb → demosaic_ccm → tone(id) → aux_hook_out
    //   → fcs → ldci → ee → DisplayBlock(1920×1080)
    //
    // UnpackCfaBlock with stride_w=2 does fused unpack+norm+CFA+width-downscale:
    //   kernel=(2,2), stride=(2,2) → [1,4,H/2,W/4] packed = 1080×1920 actual.
    // No separate AdaptiveDownscaleBlock needed.
    let packed_w = (sensor_w / 2) as i64;
    let full_w   = sensor_w as i64;
    let full_h   = sensor_h as i64;
    let ds_w     = pipe_w as i64;
    let ds_h     = pipe_h as i64;

    // Number of blocks after removing AdaptiveDownscaleBlock
    let n_blocks = 13u32;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. Packed INT32 input (sensor resolution)
        Box::new(RawInputBlock::new()
            .with_elem_type(6)
            .with_concrete_dims(full_h, packed_w)),

        // 2. Unpack CFA with stride_w=1 (packing already halves width 3840→1920)
        //    stride=2 in height halves 2160→1080, stride=1 in width keeps 1920
        //    Output: [1,4,1080,1920] packed = FHD
        Box::new(UnpackCfaBlock::new()
            .with_concrete_width(full_w)
            .with_concrete_dims(full_h, full_w)
            .with_downscale(1)
            .with_sensor_max(1023.0)
            .with_blc(true)),

        // 3. Aux hook (reference-corrected Bayer for stats)
        Box::new(IdentityBlock::new("aux_hook_src")),

        // 4. LSC (lens shading correction)
        Box::new(CcmBlock::new()),

        // 5. Bayer WB (white balance gains)
        Box::new(BayerWbBlock::new()),

        // 6. Fused demosaic + CCM
        Box::new(DemosaicCcmBlock::new(0)
            .with_concrete_dims(ds_h, ds_w)),

        // 7. Tone (identity when fused)
        Box::new(IdentityBlock::new("tone")),

        // 8. Aux hook out
        Box::new(IdentityBlock::new("aux_hook_out")),

        // 9–11. Cosmetic blocks
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),

        // 12. Display output at FHD (bg4a: Conv 1×1 BGRA float [0,255])
        //     Does channel swap + mul(255) + alpha in ONNX.
        Box::new(DisplayBlock::new(pipe_w)
            .with_pack_rgba(false)
            .with_bg4a(true)
            .with_concrete_dims(ds_h, ds_w)),
    ];

    GraphComposer::wire_blocks(&mut blocks);

    // ── Select engine and build ────────────────────────────────────
    let mut all = blocks;
    let head = all.remove(0);
    let mut engine: Box<dyn IspEngine> = match cam_isp::engine::select_engine() {
        Some(e) => e,
        None => Box::new(cam_isp::cpu::CpuEngine::new()),
    };
    println!("Engine: {} (priority {})", engine.backend_name(), engine.priority());

    // Build with actual sensor dimensions as input
    let (input_h, input_w) = (sensor_h, sensor_w);
    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Engine build FAILED: {}", e);
    }
    result.unwrap();

    println!("Pipeline: {} blocks", engine.controller().lock().unwrap().frame_count + 14);
    println!("Input: {}×{} → Pipeline: {}×{}", sensor_w, sensor_h, pipe_w, pipe_h);
    println!("Running {} frames...\n", n_frames);

    // Shared params — bayer buffer reused across all frames (no per-frame alloc)
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = pipe_w;
    params.target_height = pipe_h;
    params.sensor_max = 1023.0;
    params.output_channels = 4;

    // Warmup
    for _ in 0..3 {
        let _ = engine.process(&params);
    }

    // Timed run
    let mut sum_ms = 0.0f64;
    for i in 0..n_frames {
        let t0 = Instant::now();
        let result = engine.process(&params);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        sum_ms += ms;
        match result {
            Ok(frame) => {
                println!("  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)",
                    i, frame.width, frame.height, ms, frame.data.len());
            }
            Err(e) => println!("  [{:2}] ERROR: {}  ({:.1}ms)", i, e, ms),
        }
    }
    let avg_ms = sum_ms / n_frames as f64;
    let fps = n_frames as f64 / sum_ms * 1000.0;
    println!();
    println!("=== 4K→FHD Results ===");
    println!("  Average: {:.1}ms/frame", avg_ms);
    println!("  FPS:     {:.1}", fps);
    if avg_ms < 33.3 {
        println!("  ✅ {:.1} fps — exceeds 30fps target!", fps);
    } else {
        println!("  ❌ {:.1}ms — misses 30fps target (33.3ms)", avg_ms);
    }
}
