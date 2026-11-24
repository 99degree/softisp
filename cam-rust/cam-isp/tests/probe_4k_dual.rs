//! Combined probe: DUAL engine + timing report at 4K.
//!
//! Two sub-probes, each run with stats ON/OFF:
//!   1. Single Vulkan engine (baseline)
//!   2. Dual-GpuEngine ping-pong (engine A stats ON, engine B stats OFF)
//!
//! Reports p50/p95/max/FPS per probe, plus mean. Blank-frame detection is
//! gated so a single bad frame doesn't abort the run.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_dual --features mnn -- --nocapture --ignored
//!
//! Overrides:
//!   PERF_W / PERF_H   frame size (default 3840x2160)
//!   PERF_FRAMES       frames per probe after warmup (default 40)

#![cfg(feature = "mnn")]

use cam_isp::dual_engine::DualGpuEngine;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

struct Timing {
    label: String,
    blocks: usize,
    p50: f64,
    p95: f64,
    max: f64,
    fps: f64,
    mean: f64,
}

fn run_probe(
    label: &str,
    w: u32,
    h: u32,
    frames: usize,
    stats_on: bool,
    dual: bool,
    make: impl Fn(
        u32,
        u32,
    ) -> (
        Box<dyn cam_isp::pipeline::IspBlock>,
        Vec<Box<dyn cam_isp::pipeline::IspBlock>>,
    ),
) -> Option<Timing> {
    let (head_a, blocks_a) = make(w, h);
    let n_blocks = blocks_a.len() + 1;

    let mut engine_a = MnnEngine::with_pool_size(MnnBackend::Vulkan, 1);
    engine_a.set_stats_enabled(stats_on);
    if let Err(e) = engine_a.build(head_a, blocks_a, None, 21) {
        eprintln!("{label:<28} engine A build failed: {e}");
        return None;
    }

    let mut runner: Box<
        dyn FnMut(&ProcessParams) -> cam_isp::error::IspResult<cam_isp::pipeline::IspFrame>,
    > = if dual {
        let (head_b, blocks_b) = make(w, h);
        let mut engine_b = MnnEngine::with_pool_size(MnnBackend::Vulkan, 1);
        engine_b.set_stats_enabled(false); // engine B skips stats entirely
        if let Err(e) = engine_b.build(head_b, blocks_b, None, 21) {
            eprintln!("{label:<28} engine B build failed: {e}");
            return None;
        }
        let mut dual_engine = DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b));
        Box::new(move |p| dual_engine.next().process(p))
    } else {
        Box::new(move |p| engine_a.process(p))
    };

    // 4K raw: an 8-bit-ish ramp so the pipeline has real gradients to work on and
    // the sensor-max divide lands well inside [0,1].
    let raw: Vec<u8> = {
        let mut v = vec![0u8; (w as usize) * (h as usize) * 2];
        for (i, row) in v.chunks_exact_mut(w as usize * 2).enumerate() {
            for (x, px) in row.chunks_exact_mut(2).enumerate() {
                let s: u16 = ((x * 1023) / (w as usize).max(1)) as u16;
                let s = s.saturating_add((i % 256) as u16 * 2).min(1023);
                px[0] = (s & 0xFF) as u8;
                px[1] = (s >> 8) as u8;
            }
        }
        v
    };

    let mut params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &raw,
        sensor_max: 1023.0,
        target_width: w,
        target_height: h,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 2,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
        output_format: OutputFormat::Bgra,
        timestamp_ns: 0,
        isp_params: None,
    };

    // Warm up: the first frames include session allocation and shape propagation.
    for _ in 0..8 {
        match runner(&params) {
            Ok(_) => {}
            Err(e) => {
                eprintln!("{label:<28} warmup failed: {e}");
                return None;
            }
        }
    }

    let mut t: Vec<f64> = Vec::with_capacity(frames);
    let mut sum = 0.0f64;
    let mut picture_seen = false;
    let mut blank_count = 0usize;
    for _ in 0..frames {
        match runner(&params) {
            Ok(f) => {
                let ms = f.inference_duration_ns as f64 / 1e6;
                t.push(ms);
                sum += ms;
                if !picture_seen {
                    // data is u8 (BGRA) for non-Float* formats — check mean byte value.
                    let m =
                        f.data.iter().map(|v| *v as f64).sum::<f64>() / f.data.len().max(1) as f64;
                    if m > 5.0 {
                        // > ~2% of 255
                        picture_seen = true;
                    } else {
                        blank_count += 1;
                    }
                }
            }
            Err(e) => {
                eprintln!("{label:<28} process failed: {e}");
                return None;
            }
        }
    }

    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p50 = t[t.len() / 2];
    let p95 = t[(t.len() * 95) / 100];
    let max = *t.last().unwrap();
    let mean = sum / t.len() as f64;
    if !picture_seen {
        eprintln!("{label:<28} produced a blank frame (blanks={blank_count})");
        return None;
    }
    Some(Timing {
        label: label.to_string(),
        blocks: n_blocks,
        p50,
        p95,
        max,
        fps: 1000.0 / p50,
        mean,
    })
}

#[test]
#[ignore]
fn perf_4k_combined() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    let fused = || PipelineProfile::FUSED;

    let build = |p: PipelineProfile| {
        move |w: u32, h: u32| {
            let mut p = p;
            p.output_format = OutputFormat::Bgra;
            let mut blocks = p.build_blocks(w, h, 2);
            let head = blocks.remove(0);
            let aux = p.build_aux_blocks(h as i64, w as i64);
            blocks.extend(aux);
            (head, blocks)
        }
    };

    eprintln!("\n== {w}x{h}, {frames} frames after warmup ==");
    eprintln!(
        "{:<30} {:>7} {:>9} {:>9} {:>9} {:>8}",
        "probe", "blocks", "p50 ms", "p95 ms", "max ms", "FPS"
    );

    let mut rows: Vec<Timing> = Vec::new();

    // Probe 1: single engine, stats ON
    if let Some(t) = run_probe("single stats ON", w, h, frames, true, false, build(fused())) {
        eprintln!(
            "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>8.1}",
            t.label, t.blocks, t.p50, t.p95, t.max, t.fps
        );
        rows.push(t);
    }

    // Probe 2: single engine, stats OFF
    if let Some(t) = run_probe(
        "single stats OFF",
        w,
        h,
        frames,
        false,
        false,
        build(fused()),
    ) {
        eprintln!(
            "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>8.1}",
            t.label, t.blocks, t.p50, t.p95, t.max, t.fps
        );
        rows.push(t);
    }

    // Probe 3: dual engine (A stats ON, B stats OFF)
    if let Some(t) = run_probe(
        "dual stats ON/OFF",
        w,
        h,
        frames,
        true,
        true,
        build(fused()),
    ) {
        eprintln!(
            "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>8.1}",
            t.label, t.blocks, t.p50, t.p95, t.max, t.fps
        );
        rows.push(t);
    }

    eprintln!("\n== summary ==");
    if let Some(dual_row) = rows.iter().find(|r| r.label == "dual stats ON/OFF") {
        if let Some(single_row) = rows.iter().find(|r| r.label == "single stats ON") {
            let speedup = single_row.p50 / dual_row.p50;
            eprintln!(
                "dual vs single p50: {:.1} ms vs {:.1} ms = {:.2}x",
                dual_row.p50, single_row.p50, speedup
            );
        }
    }
}
