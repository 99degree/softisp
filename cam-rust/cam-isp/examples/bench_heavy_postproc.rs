//! Benchmark: HEAVY ISP pipeline (4K Bayer→FHD) + Post-processing with deshake.
//!
//! Measures end-to-end latency:
//!   1. HEAVY profile pipeline (UnpackCfa→DemosaicCcm→FCS→LDCI→EE→Display)
//!      on MNN Vulkan backend
//!   2. Float32→u8 conversion for postproc
//!   3. Post-processing pipeline with deshake (block-matching stabilization)
//!
//! Run with:
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!     cargo run --release --example bench_heavy_postproc -p cam-isp --features mnn
//!
//! For detailed deshake timing, add DESHAKE_DEBUG=1.

use std::time::Instant;

use cam_isp::blocks::*;
use cam_isp::cpu;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock, IspFrame};
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
    let n_frames = 30; // enough for deshake convergence + stats
    let warmup = 5;

    // Deshake uses a crop to hide black borders from warp
    let deshake_crop = 0.08;

    // Post-downscale for deshake matching resolution (after postproc crop)
    let display_w = (pipe_w as f32 * (1.0 - deshake_crop)) as u32;
    let display_h = (pipe_h as f32 * (1.0 - deshake_crop)) as u32;

    println!("═══ HEAVY Pipeline + Deshake Benchmark ═══");
    println!("Sensor:      {}×{} Bayer INT32", sensor_w, sensor_h);
    println!("ISP output:  {}×{} BGRA float32", pipe_w, pipe_h);
    println!("Postproc:    EIS(off) Deshake(on) GDC(off) Denoise(off)");
    println!("Display:     {}×{} (after crop)", display_w, display_h);
    println!("Frames:      {} ({} warmup)", n_frames, warmup);
    println!();

    // ── Generate 4K Bayer test data (Packed INT32) ─────────────
    let packed_pixels = sensor_w * sensor_h / 2; // two 10-bit pixels per INT32
    let mut raw_buf = Vec::with_capacity((packed_pixels * 4) as usize);
    let mut rng_state = 42u64;
    for _ in 0..packed_pixels {
        rng_state = rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let lo = (rng_state >> 22) as u16 & 0x3FF; // pixel A (10-bit)
        let hi = (rng_state >> 12) as u16 & 0x3FF; // pixel B (10-bit)
        let packed = (hi as u32) << 20 | (lo as u32); // INT32: hi:20bits + lo:20bits
        raw_buf.extend_from_slice(&packed.to_le_bytes());
    }
    let raw = raw_buf;

    // ── Build HEAVY ISP pipeline ───────────────────────────────
    let full_w = sensor_w as i64;
    let full_h = sensor_h as i64;
    let ds_w = pipe_w as i64;
    let ds_h = pipe_h as i64;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        // 1. Packed INT32 input: [1,1,2160,1920]
        Box::new(
            RawInputBlock::new()
                .with_elem_type(6) // INT32
                .with_concrete_dims(full_h, full_w / 2),
        ),
        // 2. Fused unpack+normalize+CFA+BLC: [1,4,2160,960]
        Box::new(
            UnpackCfaBlock::new()
                .with_concrete_width(full_w)
                .with_blc(true)
                .with_mode(UnpackMode::PackedInt32),
        ),
        // 3. Identity aux_hook_src (keeps graph topology compatible)
        Box::new(IdentityBlock::new("aux_hook_src")),
        // 4. Bayer white balance
        Box::new(BayerWbBlock::new()),
        // 5. Fused Demosaic+CCM: [1,4,1080,960] → [1,3,1080,960]
        Box::new(DemosaicCcmBlock::new(0).with_concrete_dims(ds_h, ds_w / 2)),
        // 6. Identity tone (fused into DemosaicCcmBlock)
        Box::new(IdentityBlock::new("tone")),
        // 7. Identity aux_hook_out
        Box::new(IdentityBlock::new("aux_hook_out")),
        // 8. FCS (luma-based adaptive correction)
        Box::new(FcsBlock::new()),
        // 9. LDCI (local contrast enhancement)
        Box::new(LdciBlock::new()),
        // 10. EE (edge enhancement)
        Box::new(EeBlock::new()),
        // 11. Display → BGRA float32 [0,255] output
        //     (postproc expects BGRA/RGBA u8, so we convert f32→u8)
        Box::new(
            DisplayBlock::new(pipe_w)
                .with_output_format(OutputFormat::FloatBgra)
                .with_concrete_dims(ds_h, ds_w),
        ),
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
    println!(
        "Engine: {} (priority {})",
        engine.backend_name(),
        engine.priority()
    );

    // Build with 16 blocks max (11 blocks + some internal)
    let result = engine.build(head, all, None, 21);
    if let Err(ref e) = result {
        eprintln!("Engine build FAILED: {}", e);
        return;
    }
    result.unwrap();

    let block_count = 11; // as defined above
    println!("Pipeline: {} blocks (HEAVY {}→{})", block_count, sensor_w, pipe_w);

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

    // ── ProcessParams ──────────────────────────────────────────
    let mut params = ProcessParams::new(sensor_w, sensor_h, &raw);
    params.target_width = pipe_w;
    params.target_height = pipe_h;
    params.sensor_max = 1023.0;
    params.output_format = OutputFormat::FloatBgra;

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
    let mut output_sizes = Vec::with_capacity(n_frames);

    for i in 0..n_frames {
        let t_total = Instant::now();

        // ── Stage 1: ISP inference (MNN Vulkan) ────────────────
        let t_isp = Instant::now();
        let frame_result = engine.process(&params);
        let isp_ms = t_isp.elapsed().as_secs_f64() * 1000.0;

        let frame = match frame_result {
            Ok(f) => f,
            Err(e) => {
                println!("  [{:2}] ISP ERROR: {} ({:.1}ms)", i, e, isp_ms);
                continue;
            }
        };

        // Determine output dimensions based on actual frame data
        let out_w = frame.width.max(1);
        let out_h = frame.height.max(1);
        let out_pixels = (out_w * out_h) as f64;

        // ── Stage 2: Float32→u8 BGRA conversion ────────────────
        // The ISP outputs [1,4,H,W] float32 BGRA [0,255].
        // Deshake uses u8 BGRA data, so we convert planar float32 to packed u8.
        let t_convert = Instant::now();
        let out_bytes = frame.data.len();
        let u8_data = if out_bytes == (out_w * out_h * 4 * 4) as usize {
            // Planar float32 BGRA: structure is [B,G,R,A,B,G,R,A,...] as f32 × H×W
            // Convert to packed u8: [B,G,R,A] × H×W (one byte each, clamped)
            let float_data: &[f32] = unsafe {
                std::slice::from_raw_parts(
                    frame.data.as_ptr() as *const f32,
                    frame.data.len() / 4,
                )
            };
            let mut u8_out = vec![0u8; (out_w * out_h * 4) as usize];
            let n = (out_w * out_h) as usize;
            for j in 0..n.min(float_data.len() / 4) {
                let b = (float_data[j * 4].clamp(0.0, 255.0)) as u8;
                let g = (float_data[j * 4 + 1].clamp(0.0, 255.0)) as u8;
                let r = (float_data[j * 4 + 2].clamp(0.0, 255.0)) as u8;
                let a = 255u8;
                u8_out[j * 4] = r; // deshake expects RGBA: R at [0]
                u8_out[j * 4 + 1] = g;
                u8_out[j * 4 + 2] = b;
                u8_out[j * 4 + 3] = a;
            }
            u8_out
        } else {
            // Already u8 format, use as-is
            frame.data.clone()
        };
        let convert_ms = t_convert.elapsed().as_secs_f64() * 1000.0;

        // ── Stage 3: Post-process with deshake ──────────────────
        let t_postproc = Instant::now();
        let postproc_frame = cam_isp::pipeline::IspFrame {
            data: u8_data,
            width: out_w,
            height: out_h,
            format: cam_types::FrameFormat::Rgba8888,
            float_data: None,
            aux: None,
            timestamp_ns: params.timestamp_ns,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };
        let postproc_result = postproc.process(&postproc_frame);
        let postproc_ms = t_postproc.elapsed().as_secs_f64() * 1000.0;

        let output_pixels = match &postproc_result {
            Ok(f) => (f.width * f.height) as f64,
            Err(_) => 0.0,
        };

        let total_ms = t_total.elapsed().as_secs_f64() * 1000.0;

        isp_latencies.push(isp_ms);
        postproc_latencies.push(postproc_ms);
        total_latencies.push(total_ms);
        total_pixels.push(out_pixels);
        output_sizes.push(output_pixels);

        println!(
            "  [{:2}] ISP={:>6.1}ms Conv={:>4.1}ms Post={:>6.1}ms Total={:>7.1}ms  {}×{} ({:>5.1}MP/s)",
            i,
            isp_ms,
            convert_ms,
            postproc_ms,
            total_ms,
            out_w,
            out_h,
            out_pixels / (total_ms / 1000.0) / 1_000_000.0
        );
    }

    // ── Results ─────────────────────────────────────────────────
    let n = total_latencies.len();
    if n < 3 {
        println!("\n❌ Not enough valid frames for statistics");
        return;
    }

    // Skip first 3 frames for steady-state (deshake needs a few frames to converge)
    let steady_start = 3.min(n - 1);
    let steady_latencies: Vec<f64> = total_latencies[steady_start..].to_vec();
    let steady_isp: Vec<f64> = isp_latencies[steady_start..].to_vec();
    let steady_postproc: Vec<f64> = postproc_latencies[steady_start..].to_vec();

    let avg = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let min = |v: &[f64]| v.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = |v: &[f64]| v.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    let avg_total = avg(&steady_latencies);
    let avg_isp = avg(&steady_isp);
    let avg_postproc = avg(&steady_postproc);
    let avg_pixels = avg(&total_pixels[steady_start..]);

    let fps = 1000.0 / avg_total;
    let mp_s = avg_pixels / (avg_total / 1000.0) / 1_000_000.0;

    println!();
    println!("═══ Results (steady-state, frames {}+) ═══", steady_start);
    println!("             avg     min     max");
    println!("ISP:     {:>7.1} {:>7.1} {:>7.1}  ms", avg_isp, min(&steady_isp), max(&steady_isp));
    println!("Deshake: {:>7.1} {:>7.1} {:>7.1}  ms", avg_postproc, min(&steady_postproc), max(&steady_postproc));
    println!("Total:   {:>7.1} {:>7.1} {:>7.1}  ms", avg_total, min(&steady_latencies), max(&steady_latencies));
    println!();
    println!("Throughput: {:.1} FPS  ({:.1} MP/s  @ {}×{})",
        fps, mp_s, pipe_w, pipe_h);
    println!("Pipeline: HEAVY profile (11 blocks) + postproc deshake");
    println!("Engine: {} (Vulkan backend)", engine.backend_name());
    println!();

    // Target: 30 FPS real-time = 33.3ms
    if avg_total < 33.3 {
        println!("✅ {:.1}ms  ({:.1} FPS) — real-time (≤33.3ms)!", avg_total, fps);
    } else if avg_total < 66.7 {
        println!("⚠️  {:.1}ms  ({:.1} FPS) — near real-time (≤66.7ms for 15 FPS)", avg_total, fps);
    } else if avg_total < 100.0 {
        println!("⚠️  {:.1}ms  ({:.1} FPS) — >100ms may miss 10 FPS", avg_total, fps);
    } else {
        println!("❌ {:.1}ms  ({:.1} FPS) — above 100ms threshold", avg_total, fps);
    }
}
