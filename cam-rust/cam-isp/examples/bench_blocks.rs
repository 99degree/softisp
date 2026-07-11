//! Block-by-block performance profiler for the ISP pipeline.
//!
//! Builds the pipeline incrementally — one block at a time —
//! and measures inference time for each configuration.
//! Uses the real packed INT32 pipeline (RawInput INT32 + UnpackBlock)
//! matching `PipelineProfile::build_blocks()` — the default production path.
//!
//! Usage:
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     cargo run --example bench_blocks -p cam-isp --features mnn -- [backend]
//!
//!   backend: cpu (default), vulkan, opencl, opengl

use std::time::{Duration, Instant};
use cam_isp::engine::IspEngine;
use cam_isp::pipeline::IspBlock;

/// Build prefix of the packed INT32 pipeline blocks up to `count`.
/// Mirrors `PipelineProfile::build_blocks()` — the default production path.
fn build_blocks_up_to(target_width: u32, target_height: u32, count: usize, fused: bool, legacy: bool) -> Vec<Box<dyn IspBlock>> {
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
    let full_w = target_width as i64;
    let packed_w = (target_width / 2) as i64;
    let h = target_height as i64;

    if fused {
        // Fused: raw → UnpackCfaBlock (unpack+norm+CFA+BLC)
        blocks.push(Box::new(cam_isp::blocks::RawInputBlock::new()
            .with_elem_type(6)
            .with_concrete_dims(h, packed_w)));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::UnpackCfaBlock::new()
            .with_concrete_dims(h, full_w)
            .with_blc(true)));
        if blocks.len() >= count { return wire(blocks); }
        // blc_id: identity (BLC fused into unpack_cfa)
        blocks.push(Box::new(cam_isp::blocks::IdentityBlock::new("blc_id")));
    } else if legacy {
        // Legacy FLOAT: raw → norm → cfa
        blocks.push(Box::new(cam_isp::blocks::RawInputBlock::new()
            .with_elem_type(1)
            .with_concrete_dims(h, full_w)));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::NormalizeBlock::new()));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::CfaBlock::new()
            .with_concrete_dims(h, full_w)));
    } else {
        // Standard packed: raw → unpack → norm → cfa → blc
        blocks.push(Box::new(cam_isp::blocks::RawInputBlock::new()
            .with_elem_type(6)
            .with_concrete_dims(h, packed_w)));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::UnpackBlock::new()
            .with_concrete_dims(h, full_w)));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::NormalizeBlock::new()));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::CfaBlock::new()
            .with_concrete_dims(h, full_w)));
        if blocks.len() >= count { return wire(blocks); }
        blocks.push(Box::new(cam_isp::blocks::BlcBlock::new()));
    }
    if blocks.len() >= count { return wire(blocks); }

    // Source hook (after BLC)
    blocks.push(Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_src")));
    if blocks.len() >= count { return wire(blocks); }

    // Main processing
    blocks.push(Box::new(cam_isp::blocks::BayerWbBlock::new()));
    if blocks.len() >= count { return wire(blocks); }
    blocks.push(Box::new(cam_isp::blocks::DemosaicBlock::new(0)
        .with_concrete_dims(h / 2, full_w / 2)));
    if blocks.len() >= count { return wire(blocks); }
    blocks.push(Box::new(cam_isp::blocks::CcmBlock::new()));
    if blocks.len() >= count { return wire(blocks); }
    blocks.push(Box::new(cam_isp::blocks::ToneBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // Output hook
    blocks.push(Box::new(cam_isp::blocks::IdentityBlock::new("aux_hook_out")));
    if blocks.len() >= count { return wire(blocks); }

    // Aux blocks: FCS, LDCI, EE
    blocks.push(Box::new(cam_isp::blocks::FcsBlock::new()));
    if blocks.len() >= count { return wire(blocks); }
    blocks.push(Box::new(cam_isp::blocks::LdciBlock::new()));
    if blocks.len() >= count { return wire(blocks); }
    blocks.push(Box::new(cam_isp::blocks::EeBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(target_width)));
    wire(blocks)
}

/// Wire blocks by setting input_source on each block from previous block's frame_tensor.
fn wire(mut blocks: Vec<Box<dyn IspBlock>>) -> Vec<Box<dyn IspBlock>> {
    let n = blocks.len();
    for i in 1..n {
        let prev = blocks[i - 1].frame_tensor().unwrap_or("").to_string();
        blocks[i].set_input_source(&prev);
    }
    blocks
}

/// Build an MNN model from the given blocks and bench it for `budget` duration.
/// Returns (frame_count, total_prep_ns, total_infer_ns, total_total_ns).
fn build_and_bench(
    blocks: Vec<Box<dyn IspBlock>>,
    backend: cam_isp::mnnengine::MnnBackend,
    w: u32,
    h: u32,
    budget: Duration,
) -> Option<(u32, u64, u64, u64)> {
    if blocks.is_empty() {
        return None;
    }
    let mut all = blocks;
    let head = all.remove(0);
    let aux = all;

    let deadline = Instant::now() + budget;
    let (tx, rx) = std::sync::mpsc::channel();

    std::thread::spawn(move || {
        let mut engine = cam_isp::mnnengine::MnnEngine::new(backend);
        if let Err(e) = engine.build(head, aux, None, 21) {
            eprintln!("    engine.build error: {}", e);
            let _ = tx.send(None);
            return;
        }

        // Fill u16 test buffer with checkerboard pattern
        let mut buf = vec![0u8; w as usize * h as usize * 2];
        for y in 0..h {
            for x in 0..w {
                let off = (y * w + x) as usize * 2;
                let val = (x ^ y) as u16;
                buf[off] = val as u8;
                buf[off + 1] = (val >> 8) as u8;
            }
        }

        let mut count = 0u32;
        let mut total_prep_ns = 0u64;
        let mut total_infer_ns = 0u64;
        let mut total_total_ns = 0u64;

        while Instant::now() < deadline {
            let result = engine.process(&cam_isp::engine::ProcessParams::new(w, h, &buf));
            match result {
                Ok(frame) => {
                    total_prep_ns += frame.prep_duration_ns;
                    total_infer_ns += frame.inference_duration_ns;
                    total_total_ns += frame.total_duration_ns;
                    count += 1;
                }
                Err(e) => {
                    eprintln!("    process error: {}", e);
                    break;
                }
            }
        }
        let _ = tx.send(Some((count, total_prep_ns, total_infer_ns, total_total_ns)));
    });

    rx.recv_timeout(budget + Duration::from_secs(2)).ok()?
}

fn main() {
    std::env::set_var("RUST_LOG", "warn");
    cam_isp::init();

    let args: Vec<String> = std::env::args().collect();
    let mut backend_name = "cpu";
    let mut use_legacy = false;
    let mut w = 640u32;
    let mut h = 480u32;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--legacy" | "-l" => use_legacy = true,
            "--width" | "-w" => { i += 1; if i < args.len() { w = args[i].parse().unwrap_or(640); } }
            "--height" | "-H" => { i += 1; if i < args.len() { h = args[i].parse().unwrap_or(480); } }
            _ if !args[i].starts_with('-') => backend_name = &args[i],
            _ => {}
        }
        i += 1;
    }

    let backend = match backend_name {
        "vulkan" => cam_isp::mnnengine::MnnBackend::Vulkan,
        "opencl" => cam_isp::mnnengine::MnnBackend::Opencl,
        "opengl" => cam_isp::mnnengine::MnnBackend::OpenGl,
        "cpu" => cam_isp::mnnengine::MnnBackend::Cpu,
        _ => { eprintln!("Unknown backend: '{}' (use cpu, vulkan, opencl, opengl)", backend_name); return; }
    };

    // (aux blocks are always included in the incremental bench)

    let pipeline_desc = if use_legacy { "FLOAT input (legacy)" } else { "packed INT32 → Unpack" };

    eprintln!("=== Block-by-block pipeline profiling ===");
    eprintln!("Backend: {}", backend_name);
    eprintln!("Pipeline: {}", pipeline_desc);
    eprintln!("Resolution: {}x{}", w, h);
    // Build blocks once to determine names and count
    let profile = cam_isp::profile::PipelineProfile::custom(
        "BENCH", cam_isp::profile::PipelineLevel::Lite,
        !use_legacy, // use_unpack
        true,  // use_fcs
        true,  // use_ldci
        true,  // use_ee
        false, // use_bad_pixel
        cam_isp::profile::DemosaicQuality::Standard,
        false, // use_local_contrast
        false, // use_unsharp
        false, // use_lsc
        false, // use_warp
        false, // use_hdr
        true,  // use_fused_unpack (faster fused unpack+norm+CFA)
        true,  // use_demosaic_ccm (fused demosaic+CCM, saves 1 session)
        true,  // use_fused_tone
        0,     // rotate_mode: none
        false, // use_zone_stats
        false, // use_channel_means
        false, // use_tone_stats
        false, // use_histogram
        0,     // stats_downscale_max
        0,     // pipeline_downscale_target
        0.0,   // eis_margin
        false, // use_bilateral
        true,  // use_saturation
        false, // use_vignetting
        false, // use_colorspace
        false, // use_gamma
        false, // use_sharpen
        false, // use_wavelet_denoise
        false, // use_auto_contrast
        true,  // use_normalize
        false, // use_tiled_rendering
        1,     // tile_count_x
        1,     // tile_count_y
        0,     // tile_overlap
    );
    let all_blocks = profile.build_blocks(w, 0);
    let max_blocks = all_blocks.len(); // use actual block count from profile
    let block_names: Vec<String> = all_blocks.iter().map(|b| b.id().to_string()).collect();
    drop(all_blocks);

    eprintln!("Blocks: {} total", max_blocks);
    eprintln!();

    let mut prev_total_ms = 0.0;
    let mut results: Vec<(usize, String, f64, f64, f64, f64)> = Vec::new();

    for n in 1..=max_blocks {
        let blocks = build_blocks_up_to(w, h, n, !use_legacy, use_legacy);
        assert_eq!(blocks.len(), n, "build_blocks_up_to({}) returned {} blocks", n, blocks.len());

        let label = &block_names[n - 1];
        eprint!("  [{:2}/{}] {:26} ", n, max_blocks, label);

        let result = build_and_bench(blocks, backend, w, h, Duration::from_millis(2000));

        match result {
            Some((count, total_prep_ns, total_infer_ns, total_total_ns)) => {
                let avg_prep_ms = if count > 0 { total_prep_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let _avg_infer_ms = if count > 0 { total_infer_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let avg_total_ms = if count > 0 { total_total_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let fps = if avg_total_ms > 0.0 { 1000.0 / avg_total_ms } else { 0.0 };

                let inc_cost = if n > 1 { avg_total_ms - prev_total_ms } else { avg_total_ms };

                eprintln!(
                    "{:>5.1} fps | {:>5.1}ms total | {:>+5.1}ms | prep={:.1}ms",
                    fps, avg_total_ms, inc_cost, avg_prep_ms
                );

                results.push((n, label.to_string(), avg_total_ms, inc_cost, _avg_infer_ms, fps));
                prev_total_ms = avg_total_ms;
            }
            None => {
                eprintln!("FAILED (build/run error)");
                results.push((n, label.to_string(), -1.0, -1.0, -1.0, -1.0));
            }
        }
    }

    // Summary table
    eprintln!();
    eprintln!("=== Per-Block Cost Summary ===");
    eprintln!("{:<30} {:>12} {:>12} {:>10}", "Block", "Total (ms)", "Incr (ms)", "FPS");
    eprintln!("{}", "-".repeat(66));
    for (_, name, total, inc, _, fps) in &results {
        if *total < 0.0 {
            eprintln!("{:<30} {:>12}", name, "FAILED");
        } else {
            eprintln!("{:<30} {:>9.2}ms {:>+9.2}ms {:>7.1}fps", name, total, inc, fps);
        }
    }
    eprintln!("{}", "-".repeat(66));

    // Top 5 most expensive blocks
    eprintln!();
    eprintln!("=== Most Expensive Blocks (by incremental cost) ===");
    let mut sorted: Vec<_> = results.iter()
        .filter(|(_, _, total, inc, _, _)| *inc >= 0.0 && *total >= 0.0)
        .collect();
    sorted.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap_or(std::cmp::Ordering::Equal));
    let final_total = results.last()
        .filter(|(_, _, t, _, _, _)| *t > 0.0)
        .map(|(_, _, t, _, _, _)| *t)
        .unwrap_or(1.0);
    for (i, (_, name, _, inc, _, _)) in sorted.iter().enumerate().take(5) {
        let bar = "#".repeat((*inc as usize).clamp(1, 40));
        let pct = (*inc / final_total) * 100.0;
        eprintln!("  #{:2}  +{:>6.2}ms ({:>4.1}%)  {}  {}", i + 1, inc, pct, bar, name);
    }
}
