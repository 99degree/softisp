//! Benchmark: HEAVY ISP pipeline (4K Bayer→FHD) + Post-processing with deshake.
//!
//! Measures end-to-end latency:
//!   1. HEAVY profile ISP pipeline on MNN Vulkan → FloatRgb [1,3,H,W] f32
//!   2. Post-processing pipeline with deshake (reads planar float directly)
//!      via process_float() — no f32→u8 repack needed
//!
//! Run with:
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo run --release --example bench_heavy_postproc -p cam-isp --features mnn

use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::postprocess::{PostProcessConfig, PostProcessPipeline};

fn main() {
    let _ = env_logger::builder()
        .is_test(false)
        .filter_level(log::LevelFilter::Warn)
        .try_init();
    cam_isp::init();

    // ── Resolution ─────────────────────────────────────────────
    let sensor_w = 3840u32;
    let sensor_h = 2160u32;
    let pipe_w = 1920u32; // FHD pipeline width
    let pipe_h = 1080u32; // FHD pipeline height
    let n_frames = 30;
    let warmup = 5;

    let deshake_crop = 0.08;
    let display_w = (pipe_w as f32 * (1.0 - deshake_crop)) as u32;
    let display_h = (pipe_h as f32 * (1.0 - deshake_crop)) as u32;

    println!("═══ HEAVY Pipeline + Deshake Benchmark ═══");
    println!("Sensor:      {}×{} Bayer INT32", sensor_w, sensor_h);
    println!("ISP output:  {}×{} FloatRgb [1,3,H,W] f32 (no repack)", pipe_w, pipe_h);
    println!("Postproc:    EIS(off) Deshake(on) GDC(off) Denoise(off)");
    println!("Display:     {}×{} (after crop)", display_w, display_h);
    println!("Frames:      {} ({} warmup)", n_frames, warmup);
    println!();

    // ── Generate 4K Bayer test data (Packed INT32) ─────────────
    let packed_pixels = sensor_w * sensor_h / 2;
    let mut raw_buf = Vec::with_capacity((packed_pixels * 4) as usize);
    let mut rng_state = 42u64;
    for _ in 0..packed_pixels {
        rng_state = rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let lo = (rng_state >> 22) as u16 & 0x3FF;
        let hi = (rng_state >> 12) as u16 & 0x3FF;
        let packed = (hi as u32) << 20 | (lo as u32);
        raw_buf.extend_from_slice(&packed.to_le_bytes());
    }
    let raw = raw_buf;

    // ── Build HEAVY ISP pipeline ───────────────────────────────
    let full_w = sensor_w as i64;
    let full_h = sensor_h as i64;
    let ds_w = pipe_w as i64;
    let ds_h = pipe_h as i64;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. Packed INT32 input
        Box::new(RawInputBlock::new()
            .with_elem_type(6)
            .with_concrete_dims(full_h, full_w / 2)),
        // 2. Fused unpack+normalize+CFA+BLC
        Box::new(UnpackCfaBlock::new()
            .with_concrete_width(full_w)
            .with_blc(true)
            .with_mode(UnpackMode::PackedInt32)),
        // 3–5. Pipeline blocks
        Box::new(IdentityBlock::new("aux_hook_src")),
        Box::new(BayerWbBlock::new()),
        Box::new(DemosaicCcmBlock::new(0).with_concrete_dims(ds_h, ds_w / 2)),
        Box::new(IdentityBlock::new("tone")),
        Box::new(IdentityBlock::new("aux_hook_out")),
        Box::new(FcsBlock::new()),
        Box::new(LdciBlock::new()),
        Box::new(EeBlock::new()),
        // 11. Display → FloatRgb [1,3,H,W] f32 in [0,1]
        //     Uses identity path: Pow+Clip → isp.display (R6 detection)
        //     No format conversion — postproc reads planar float directly
        Box::new(DisplayBlock::new(pipe_w)
            .with_output_format(OutputFormat::FloatRgb)
            .with_concrete_dims(ds_h, ds_w)),
    ];

    GraphComposer::wire_blocks(&mut blocks);

    // ── Select engine and build ────────────────────────────────
    let mut all = blocks;
    let head = all.remove(0);
    let mut engine: Box<dyn IspEngine> =
        match cam_isp::engine::select_engine_by_name("vulkan") {
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
        return;
    }
    result.unwrap();

    println!("Pipeline: 11 blocks (HEAVY {}→{})", sensor_w, pipe_w);

    // ── Build post-processing pipeline with deshake ─────────────
    let mut postproc = PostProcessPipeline::new(PostProcessConfig {
        deshake_enabled: true,
        deshake_block_size: 32,
        deshake_search_radius: 16,
        deshake_crop_fraction: deshake_crop,
        deshake_smoothing_alpha: 0.08,
        ..Default::default()
    })
    .with_deshake(32, 16, deshake_crop, 0.08);

    // ── ProcessParams (FloatRgb output → planar [1,3,H,W] f32) ─
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = pipe_w;
    params.target_height = pipe_h;
    params.sensor_max = 1023.0;
    params.output_format = OutputFormat::FloatRgb;

    // ── Warmup ─────────────────────────────────────────────────
    println!("Warming up ({} frames)...", warmup);
    for _ in 0..warmup {
        let _ = engine.process(&params);
    }

    // ── Timed run ──────────────────────────────────────────────
    println!("Running {} frames...\n", n_frames);

    let mut isp_latencies = Vec::with_capacity(n_frames);
    let mut postproc_latencies = Vec::with_capacity(n_frames);
    let mut total_latencies = Vec::with_capacity(n_frames);
    let mut total_pixels = Vec::with_capacity(n_frames);

    for i in 0..n_frames {
        let t_total = Instant::now();

        // ── Stage 1: ISP inference (MNN Vulkan) ────────────────
        let t_isp = Instant::now();
        let frame = match engine.process(&params) {
            Ok(f) => f,
            Err(e) => {
                println!("  [{:2}] ISP ERROR: {}", i, e);
                continue;
            }
        };
        let isp_ms = t_isp.elapsed().as_secs_f64() * 1000.0;

        let out_w = frame.width.max(1);
        let out_h = frame.height.max(1);
        let out_pixels = (out_w * out_h) as f64;

        // ISP output with FloatRgb format: frame.data contains raw float bytes
        // in [1,3,H,W] planar layout, values in [0,1].
        let float_data: Vec<f32> = if frame.data.len() == (out_w * out_h * 3 * 4) as usize {
            // Planar float32 RGB: reinterpret bytes as f32 slice, then copy
            unsafe {
                std::slice::from_raw_parts(
                    frame.data.as_ptr() as *const f32,
                    frame.data.len() / 4,
                )
            }
            .to_vec()
        } else if let Some(ref fd) = frame.float_data {
            fd.clone()
        } else {
            eprintln!("  [{:2}] Unexpected output format: {} bytes", i, frame.data.len());
            continue;
        };

        // ── Stage 2: Post-process with deshake ──────────────────
        // process_float() converts planar [0,1] f32 → packed u8 RGBA internally
        let t_postproc = Instant::now();
        let postproc_result = postproc.process_float(
            &float_data,
            out_w,
            out_h,
            None,
            params.timestamp_ns,
            None,
        );
        let postproc_ms = t_postproc.elapsed().as_secs_f64() * 1000.0;

        let _output_pixels = match &postproc_result {
            Ok(f) => (f.width * f.height) as f64,
            Err(e) => {
                eprintln!("  [{:2}] Postproc ERROR: {}", i, e);
                0.0
            }
        };

        let total_ms = t_total.elapsed().as_secs_f64() * 1000.0;

        isp_latencies.push(isp_ms);
        postproc_latencies.push(postproc_ms);
        total_latencies.push(total_ms);
        total_pixels.push(out_pixels);

        println!(
            "  [{:2}] ISP={:>6.1}ms  Post={:>6.1}ms  Total={:>7.1}ms  {}×{} ({:>5.1}MP/s)",
            i, isp_ms, postproc_ms, total_ms, out_w, out_h,
            out_pixels / (total_ms / 1000.0) / 1_000_000.0
        );
    }

    // ── Results ─────────────────────────────────────────────────
    let n = total_latencies.len();
    if n < 3 {
        println!("\n❌ Not enough valid frames for statistics");
        return;
    }

    let steady_start = 3.min(n - 1);
    let steady_latencies: Vec<f64> = total_latencies[steady_start..].to_vec();
    let steady_isp: Vec<f64> = isp_latencies[steady_start..].to_vec();
    let steady_postproc: Vec<f64> = postproc_latencies[steady_start..].to_vec();

    let avg = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let mn = |v: &[f64]| v.iter().cloned().fold(f64::INFINITY, f64::min);
    let mx = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let avg_total = avg(&steady_latencies);
    let avg_isp = avg(&steady_isp);
    let avg_postproc = avg(&steady_postproc);
    let avg_pixels = avg(&total_pixels[steady_start..]);

    let fps = 1000.0 / avg_total;
    let mp_s = avg_pixels / (avg_total / 1000.0) / 1_000_000.0;

    println!();
    println!("═══ Results (steady-state, frames {}+) ═══", steady_start);
    println!("             avg     min     max");
    println!("ISP:     {:>7.1} {:>7.1} {:>7.1}  ms", avg_isp, mn(&steady_isp), mx(&steady_isp));
    println!("Deshake: {:>7.1} {:>7.1} {:>7.1}  ms",
        avg_postproc, mn(&steady_postproc), mx(&steady_postproc));
    println!("Total:   {:>7.1} {:>7.1} {:>7.1}  ms",
        avg_total, mn(&steady_latencies), mx(&steady_latencies));
    println!();
    println!("Throughput: {:.1} FPS  ({:.1} MP/s  @ {}×{})",
        fps, mp_s, pipe_w, pipe_h);
    println!("Pipeline: HEAVY (11 blocks) + FloatRgb → process_float → deshake");
    println!("Engine: {}", engine.backend_name());
    println!();

    if avg_total < 33.3 {
        println!("✅ {:.1}ms ({:.1} FPS) — real-time (≤33.3ms)!", avg_total, fps);
    } else if avg_total < 66.7 {
        println!("⚠️  {:.1}ms ({:.1} FPS) — near real-time (≤66.7ms)", avg_total, fps);
    } else {
        println!("❌ {:.1}ms ({:.1} FPS) — above 66.7ms (15 FPS target)", avg_total, fps);
    }
}
