//! Benchmark 8K Bayer → FHD pipeline — clean architecture.
//!
//! Main flow (12 blocks):
//!   RawInput → UnpackCfa(4K Bayer) → aux_hook_src
//!     → bayer_wb(id) → DemosaicCcm(4K, fused WB+CCM+tone)
//!     → AdaptiveDownscale(FHD) → tone(id) → aux_hook_out
//!     → FCS → LDCI → EE → Display(BGRA)
//!
//! Design principle: chain fusible ops together (WB+CCM+tone into
//! DemosaicCcmBlock at 4K), then downscale before non-fusible blocks
//! (FCS, LDCI, EE) so they run at FHD with 4× fewer pixels.
//!
//! All stats blocks (ZoneStats, ChannelMeans, ToneStats, CoarseHistogram)
//! tap exclusively at aux_hook_src — they read Bayer for statistics without
//! modifying the main pipeline.
//!
//! WB and Tone kept as IdentityBlock placeholders — architecture reference.
//! Actual fusion is in DemosaicCcmBlock. Controller passes gains, matrix,
//! and params as extra inputs at inference time.
//!
//! Bayer test data generated inline (deterministic pseudo-random).
//!
//! Run:
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     RUST_LOG=warn cargo run --release --example bench_8k_to_fhd -p cam-isp --features mnn

use cam_isp::blocks::*;
use cam_isp::engine::{IspEngine, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use std::time::Instant;

fn main() {
    let _ = env_logger::builder()
        .is_test(false)
        .filter_level(log::LevelFilter::Info)
        .try_init();
    cam_isp::init();

    // ── 8K sensor → FHD pipeline ──
    let sensor_w = 7680u32;
    let sensor_h = 4320u32;
    let pipe_w = 1920u32;
    let pipe_h = 1080u32;
    let n_frames = 20;

    // Generate 8K Bayer test data inline
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

    // ── Build main pipeline blocks (10 blocks) ─────────────────
    //
    // All stats/blocks connect to aux_hook_src as aux branches —
    // they read Bayer for statistics without modifying main flow.
    //
    // Controller params (WB gains, CCM matrix, tone curve) are passed
    // as extra inputs to DemosaicCcmBlock at inference time.
    let packed_w = (sensor_w / 2) as i64;
    let full_w = sensor_w as i64;
    let full_h = sensor_h as i64;
    let ds_w = pipe_w as i64;
    let ds_h = pipe_h as i64;
    let mid_h = full_h / 2; // 2160 (after UnpackCfa stride=2)
    let mid_w = packed_w; // 3840

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. Packed INT32 input
        Box::new(
            RawInputBlock::new()
                .with_elem_type(6)
                .with_concrete_dims(full_h, packed_w),
        ),
        // 2. Unpack CFA: stride=2 height (4320→2160), stride=1 width
        //    Output: [1,4,2160,3840] = 4K Bayer (unpacked + normalized + BLC)
        Box::new(
            UnpackCfaBlock::new()
                .with_concrete_width(full_w)
                .with_concrete_dims(full_h, full_w)
                .with_downscale(1)
                .with_sensor_max(1023.0)
                .with_blc(true),
        ),
        // 3. Stats hook — all stats blocks tap here
        Box::new(IdentityBlock::new("aux_hook_src")),
        // 4. White balance (identity — fused into DemosaicCcmBlock)
        Box::new(IdentityBlock::new("bayer_wb")),
        // 5. Fused demosaic + WB + CCM + tone (single Conv block @ 4K)
        //    Runs at 2160×3840 for full demosaic quality before downscale.
        Box::new(
            DemosaicCcmBlock::new(0) // 0 = RGGB
                .with_concrete_dims(mid_h, mid_w),
        ),
        // 6. Adaptive downscale: 2160×3840 → 1080×1920 (FHD)
        //    After fusible chain. Everything after runs at FHD: 4× fewer pixels.
        Box::new(
            AdaptiveDownscaleBlock::new(ds_w, ds_h, 1, "reflect", "fit")
                .with_concrete_dims(mid_h, mid_w),
        ),
        // 7. Tone (identity — already fused, placeholder for topology)
        Box::new(IdentityBlock::new("tone")),
        // 8. Aux hook out
        Box::new(IdentityBlock::new("aux_hook_out")),
        // 7–9. Cosmetic post-processing at FHD
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),
        // 10. Display output (bg4a: Conv 1×1 BGRA float [0,255])
        Box::new(
            DisplayBlock::new(pipe_w)
                .with_pack_rgba(false)
                .with_bg4a(true)
                .with_concrete_dims(ds_h, ds_w),
        ),
    ];

    GraphComposer::wire_blocks(&mut blocks);

    // ── Select engine and build ────────────────────────────────
    let mut all = blocks;
    let head = all.remove(0);
    let mut engine: Box<dyn IspEngine> = match cam_isp::engine::select_engine() {
        Some(e) => e,
        None => Box::new(cam_isp::cpu::CpuEngine::new()),
    };
    println!(
        "Engine: {} (priority {})",
        engine.backend_name(),
        engine.priority()
    );

    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Engine build FAILED: {}", e);
    }
    result.unwrap();

    println!("Pipeline: 10 blocks (main) + 4 aux (stats tap at aux_hook_src)");
    println!(
        "Input: {}×{} → 4K Bayer → FHD Output: {}×{}",
        sensor_w, sensor_h, pipe_w, pipe_h
    );
    println!("Running {} frames...\n", n_frames);

    // Shared params — bayer buffer reused across all frames
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = pipe_w;
    params.target_height = pipe_h;
    params.sensor_max = 1023.0;
    params.output_format = cam_isp::engine::OutputFormat::FloatBgra;

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
                println!(
                    "  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)",
                    i,
                    frame.width,
                    frame.height,
                    ms,
                    frame.data.len()
                );
            }
            Err(e) => println!("  [{:2}] ERROR: {}  ({:.1}ms)", i, e, ms),
        }
    }
    let avg_ms = sum_ms / n_frames as f64;
    let fps = n_frames as f64 / sum_ms * 1000.0;
    let mb_in = (sensor_w * sensor_h * 2) as f64 / 1_048_576.0;
    let mb_out = (pipe_w * pipe_h * 4) as f64 / 1_048_576.0;
    println!();
    println!("=== 8K→FHD Results ===");
    println!(
        "  Input:  {}×{} ({:.0} MB Bayer)",
        sensor_w, sensor_h, mb_in
    );
    println!("  Output: {}×{} ({:.1} MB BGRA)", pipe_w, pipe_h, mb_out);
    println!("  Average: {:.1}ms/frame", avg_ms);
    println!("  FPS:     {:.1}", fps);
    if fps >= 30.0 {
        println!("  ✅ {:.1} fps — exceeds 30fps target!", fps);
    } else if fps >= 15.0 {
        println!("  ✅ {:.1} fps — acceptable for 8K input", fps);
    } else {
        println!("  ❌ {:.1} fps — below 15fps target", fps);
    }
}
