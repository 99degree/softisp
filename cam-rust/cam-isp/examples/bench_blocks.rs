//! Block-by-block performance profiler for ISP pipeline.
//!
//! Builds the pipeline incrementally — one block at a time —
//! and measures inference time for each configuration.
//! The incremental cost reveals which blocks are computationally expensive.
//!
//! Usage:
//!   LD_LIBRARY_PATH=$PWD/lib/aarch64 \
//!     cargo run --example bench_blocks -p cam-isp --features mnn -- [backend]
//!
//!   backend: cpu (default), vulkan, opencl, opengl

use std::time::{Duration, Instant};
use cam_isp::engine::IspEngine;
use cam_isp::pipeline::IspBlock;

/// Build prefix of standard (non-packed) pipeline blocks up to `count`.
/// Uses standard UINT16→FLOAT pipeline (no packed INT32 zero-copy)
/// so we can profile each block's computational cost reliably.
/// Uses fully concrete dims (both H and W) to avoid symbolic dims that
/// confuse MNN's resizeSession/copyFromHostTensor.
fn build_blocks_up_to(target_width: u32, target_height: u32, count: usize) -> Vec<Box<dyn IspBlock>> {
    let mut blocks: Vec<Box<dyn IspBlock>> = Vec::new();
    let w = target_width as i64;
    let h = target_height as i64;

    // 0: RawInputBlock (FLOAT input — elem_type=1)
    blocks.push(Box::new(cam_isp::blocks::RawInputBlock::new()
        .with_elem_type(1)  // FLOAT
        .with_concrete_dims(h, w)));
    if blocks.len() >= count { return wire(blocks); }

    // 1: NormalizeBlock — cast UINT16→FLOAT, divide by sensor_max
    blocks.push(Box::new(cam_isp::blocks::NormalizeBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 2: CfaBlock — unpack Bayer CFA
    blocks.push(Box::new(cam_isp::blocks::CfaBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 3: BlcBlock (black level correction)
    blocks.push(Box::new(cam_isp::blocks::BlcBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 4: BayerWbBlock (white balance)
    blocks.push(Box::new(cam_isp::blocks::BayerWbBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 5: DemosaicBlock
    blocks.push(Box::new(cam_isp::blocks::DemosaicBlock::new(0)));
    if blocks.len() >= count { return wire(blocks); }

    // 6: CcmBlock (color correction matrix)
    blocks.push(Box::new(cam_isp::blocks::CcmBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 7: ToneBlock (gamma / contrast / saturation)
    blocks.push(Box::new(cam_isp::blocks::ToneBlock::new()));
    if blocks.len() >= count { return wire(blocks); }

    // 8: DisplayBlock (float → BGRA U8)
    blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(target_width)));
    if blocks.len() >= count { return wire(blocks); }

    // 9: FcsBlock (false color suppression) — after Tone, before Display
    blocks.push(Box::new(cam_isp::blocks::FcsBlock::new()));

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

        let params = cam_isp::engine::default_tone_params();

        let mut count = 0u32;
        let mut total_prep_ns = 0u64;
        let mut total_infer_ns = 0u64;
        let mut total_total_ns = 0u64;

        while Instant::now() < deadline {
            let result = engine.process(
                w, h, w, &buf, 1023.0, w, None, &params,
                None, None, 1.0, 0.0, None, None, None,
            );
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
    let _ = std::env::set_var("RUST_LOG", "warn");
    cam_isp::init();

    let backend_name = std::env::args().nth(1).unwrap_or_else(|| "cpu".to_string());
    let backend = match backend_name.as_str() {
        "vulkan" => cam_isp::mnnengine::MnnBackend::Vulkan,
        "opencl" => cam_isp::mnnengine::MnnBackend::Opencl,
        "opengl" => cam_isp::mnnengine::MnnBackend::OpenGl,
        "cpu" => cam_isp::mnnengine::MnnBackend::Cpu,
        _ => { eprintln!("Unknown backend: '{}' (use cpu, vulkan, opencl, opengl)", backend_name); return; }
    };

    let (w, h) = (640u32, 480u32);
    let max_blocks = 10; // raw → norm → cfa → blc → wb → demo → ccm → tone → display → fcs

    eprintln!("=== Block-by-block pipeline profiling ===");
    eprintln!("Backend: {}", backend_name);
    eprintln!("Resolution: {}x{}", w, h);
    eprintln!("Blocks available: {}", max_blocks);
    eprintln!();

    let block_names = [
        "RawInput",
        "Normalize",
        "Cfa (CFA unpack)",
        "Blc (black level)",
        "BayerWb",
        "Demosaic",
        "Ccm (color matrix)",
        "Tone (gamma/contrast)",
        "Display",
        "Fcs (false color)",
    ];

    let mut prev_total_ms = 0.0;
    let mut results: Vec<(usize, String, f64, f64, f64, f64)> = Vec::new();

    for n in 1..=max_blocks {
        let blocks = build_blocks_up_to(w, h, n);
        assert_eq!(blocks.len(), n, "build_blocks_up_to({}) returned {} blocks", n, blocks.len());

        let label = block_names[n - 1];
        eprint!("  [{:2}/{}] {:25} ", n, max_blocks, label);

        let result = build_and_bench(blocks, backend, w, h, Duration::from_millis(2000));

        match result {
            Some((count, total_prep_ns, total_infer_ns, total_total_ns)) => {
                let _avg_prep_ms = if count > 0 { total_prep_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let avg_infer_ms = if count > 0 { total_infer_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let avg_total_ms = if count > 0 { total_total_ns as f64 / count as f64 / 1_000_000.0 } else { 0.0 };
                let fps = if avg_total_ms > 0.0 { 1000.0 / avg_total_ms } else { 0.0 };

                let inc_cost = if n > 1 { avg_total_ms - prev_total_ms } else { avg_total_ms };

                eprintln!(
                    "{:>5.1} fps | {:>5.1}ms total | +{:>5.1}ms",
                    fps, avg_total_ms, inc_cost
                );

                results.push((n, label.to_string(), avg_total_ms, inc_cost, avg_infer_ms, fps));
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
    for (i, (_, name, _, inc, _, _)) in sorted.iter().enumerate().take(5) {
        let bar = "#".repeat((*inc as usize).max(1).min(40));
        let pct = if prev_total_ms > 0.0 { (*inc / prev_total_ms) * 100.0 } else { 0.0 };
        eprintln!("  #{:2}  +{:>6.2}ms ({:>4.1}%)  {}  {}", i + 1, inc, pct, bar, name);
    }
}
