//! Benchmark 4K Bayer → FHD pipeline — clean architecture.
//!
//! Main flow (10 blocks):
//!   RawInput → UnpackCfa(FHD Bayer) → aux_hook_src
//!     → bayer_wb(id) → DemosaicCcm(4K fused WB+CCM+tone)
//!     → tone(id) → aux_hook_out → FCS → LDCI → EE → Display(BGRA)
//!
//! For 4K→FHD, UnpackCfaBlock stride=2 height directly produces
//! [1,4,1080,1920] — no separate downscale block needed.
//!
//! All stats blocks tap at aux_hook_src. WB/CCM/tone fused into
//! DemosaicCcmBlock. Identity placeholders maintain topology.
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
    let _ = env_logger::builder().is_test(false).filter_level(log::LevelFilter::Info).try_init();
    cam_isp::init();

    let sensor_w = 3840u32;
    let sensor_h = 2160u32;
    let pipe_w   = 1920u32;
    let pipe_h   = 1080u32;
    let post_w   = 960u32;   // post-downscale width for expensive blocks
    let post_h   = 540u32;   // post-downscale height
    let n_frames = 20;

    // Generate 4K Bayer test data inline
    use std::io::Write;
    let mut raw_buf = Vec::with_capacity((sensor_w * sensor_h * 2) as usize);
    let mut rng_state = 42u64;
    for _ in 0..sensor_w * sensor_h {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let val = (rng_state >> 22) as u16 & 0x3FF;
        raw_buf.extend_from_slice(&val.to_le_bytes());
    }
    let raw = raw_buf;

    // ── Build main pipeline (10 blocks) ────────────────────────
    let packed_w = (sensor_w / 2) as i64;
    let full_w   = sensor_w as i64;
    let full_h   = sensor_h as i64;
    let ds_w     = pipe_w as i64;
    let ds_h     = pipe_h as i64;
    let post_w_i = post_w as i64;
    let post_h_i = post_h as i64;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. Packed INT32 input
        Box::new(RawInputBlock::new()
            .with_elem_type(6)
            .with_concrete_dims(full_h, packed_w)),

        // 2. Unpack CFA at 4K (no downscale): [1,4,2160,3840]
        Box::new(UnpackCfaBlock::new()
            .with_concrete_width(full_w)
            .with_concrete_dims(full_h, full_w)
            .with_downscale(1)       // no width downscale
            .with_sensor_max(1023.0)
            .with_blc(true)),

        // 3. Stats hook
        Box::new(IdentityBlock::new("aux_hook_src")),

        // 4. White balance identity (fused into DemosaicCcmBlock)
        Box::new(IdentityBlock::new("bayer_wb")),

        // 5. Resize Bayer 4K→2K: [1,4,2160,3840] → [1,4,1080,1920]
        //    Cheaper than RGB resize (4ch nearest vs 3ch bilinear)
        Box::new(ResizeBlock::new(0.5)
            .with_concrete_dims(full_h, full_w)),

        // 6. DemosaicCcm at 2K: [1,4,1080,1920] → [1,3,1080,1920]
        Box::new(DemosaicCcmBlock::new(0)
            .with_concrete_dims(ds_h, ds_w)),

        // 7. Downscale RGB 2K→FHD: [1,3,1080,1920] → [1,3,540,960]
        Box::new(ResizeBlock::new(0.5)
            .with_concrete_dims(ds_h, ds_w)),

        // 8. Tone identity (fused)
        Box::new(IdentityBlock::new("tone")),

        // 9. Aux hook out
        Box::new(IdentityBlock::new("aux_hook_out")),

        // 10–12. Cosmetic post-processing at FHD
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),

        // 13. Display output (bg4a: Conv 1×1 BGRA float [0,255])
        Box::new(DisplayBlock::new(post_w)
            .with_pack_rgba(false)
            .with_bg4a(true)
            .with_concrete_dims(post_h_i, post_w_i)),
    ];

    GraphComposer::wire_blocks(&mut blocks);

    // ── Select engine and build ────────────────────────────────
    let mut all = blocks;
    let head = all.remove(0);
    let mut engine: Box<dyn IspEngine> = match cam_isp::engine::select_engine_by_name("vulkan") {
        Some(e) => e,
        None => match cam_isp::engine::select_engine() {
            Some(e) => e,
            None => Box::new(cam_isp::cpu::CpuEngine::new()),
        },
    };
    println!("Engine: {} (priority {})", engine.backend_name(), engine.priority());

    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Engine build FAILED: {}", e);
    }
    result.unwrap();

    println!("Pipeline: 13 blocks (4K→2K debayer→FHD)");
    println!("Input: {}×{} → Resize 2K → Debayer → Resize FHD → {}×{}", sensor_w, sensor_h, post_w, post_h);
    println!("Running {} frames...\n", n_frames);

    // Shared params
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = post_w;
    params.target_height = post_h;
    params.sensor_max = 1023.0;
    params.output_format = cam_isp::engine::OutputFormat::FloatBgra;

    // Warmup
    for _ in 0..3 { let _ = engine.process(&params); }

    // Timed run
    let mut sum_ms = 0.0f64;
    for i in 0..n_frames {
        let t0 = Instant::now();
        let result = engine.process(&params);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        sum_ms += ms;
        match result {
            Ok(frame) => println!("  [{:2}] {:4}×{:<4}  {:7.1}ms  ({} bytes)",
                i, frame.width, frame.height, ms, frame.data.len()),
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
