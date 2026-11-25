//! 4K throughput: what the fused profiles do at a real sensor size.
//!
//! Everything measured so far ran at 1296x970 — half the sensor in each axis — because
//! that is the RGB tail's geometry and the probes were about correctness. 4K is where
//! the pipeline actually runs, and where the per-dispatch cost stops dominating.
//!
//! Reports mean/p50/p95/max over a fixed number of frames per profile, the 3A
//! statistics switch on and off, and the same stage set built from primitives so the
//! fusion's effect is visible at this size rather than extrapolated.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_perf --features mnn -- --nocapture --ignored
//!
//! Overrides:
//!   PERF_W / PERF_H   frame size (default 3840x2160)
//!   PERF_FRAMES       frames per profile after warmup (default 40)

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

#[derive(Clone, Copy, Default)]
struct PhaseStats {
    mean: f64,
    p50: f64,
    p95: f64,
    max: f64,
}

struct Timing {
    label: String,
    blocks: usize,
    total: PhaseStats,
    prep: PhaseStats,
    infer: PhaseStats,
    post: PhaseStats,
    fps: f64,
}

fn phase_stats(mut samples: Vec<f64>, sum: f64) -> PhaseStats {
    if samples.is_empty() {
        return PhaseStats::default();
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let len = samples.len();
    PhaseStats {
        mean: sum / len as f64,
        p50: samples[len / 2],
        p95: samples[(len * 95) / 100],
        max: samples[len - 1],
    }
}

fn ms(ns: u64) -> f64 {
    ns as f64 / 1e6
}

fn run_single(
    label: &str,
    w: u32,
    h: u32,
    frames: usize,
    stats_on: bool,
    fmt: OutputFormat,
    make: impl Fn(
        u32,
        u32,
    ) -> (
        Box<dyn cam_isp::pipeline::IspBlock>,
        Vec<Box<dyn cam_isp::pipeline::IspBlock>>,
    ),
) -> Option<Timing> {
    let (head, mut blocks) = make(w, h);
    let n_blocks = blocks.len() + 1;

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.set_stats_enabled(stats_on);
    if let Err(e) = engine.build(head, blocks.drain(..).collect(), None, 21) {
        eprintln!("{label:<28} build failed: {e}");
        return None;
    }

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

    let params = ProcessParams {
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
        output_format: fmt,
        timestamp_ns: 0,
        isp_params: None,
    };

    // Warm up: the first frames include session allocation and shape propagation.
    for _ in 0..8 {
        if let Err(e) = engine.process(&params) {
            eprintln!("{label:<28} warmup failed: {e}");
            return None;
        }
    }

    let (mut total, mut prep, mut infer, mut post) = (
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
    );
    let (mut total_sum, mut prep_sum, mut infer_sum, mut post_sum) = (0.0, 0.0, 0.0, 0.0);
    let mut picture = false;
    for _ in 0..frames {
        match engine.process(&params) {
            Ok(f) => {
                let total_ms = ms(f.total_duration_ns);
                let prep_ms = ms(f.prep_duration_ns);
                let infer_ms = ms(f.inference_duration_ns);
                let post_ms = ms(f
                    .total_duration_ns
                    .saturating_sub(f.prep_duration_ns.saturating_add(f.inference_duration_ns)));
                total.push(total_ms);
                prep.push(prep_ms);
                infer.push(infer_ms);
                post.push(post_ms);
                total_sum += total_ms;
                prep_sum += prep_ms;
                infer_sum += infer_ms;
                post_sum += post_ms;
                if !picture {
                    // Packed output is BGRA bytes in a u32 word, not four f32
                    // bytes. Interpret it as floats only made every frame look
                    // blank (NaNs compare false), so use a byte-level check.
                    picture = f.data.iter().any(|b| *b != 0);
                }
            }
            Err(e) => {
                eprintln!("{label:<28} process failed: {e}");
                return None;
            }
        }
    }
    let total = phase_stats(total, total_sum);
    let prep = phase_stats(prep, prep_sum);
    let infer = phase_stats(infer, infer_sum);
    let post = phase_stats(post, post_sum);
    if !picture {
        eprintln!("{label:<28} produced a blank frame");
        return None;
    }
    Some(Timing {
        label: label.to_string(),
        blocks: n_blocks,
        fps: 1000.0 / total.p50.max(0.001),
        total,
        prep,
        infer,
        post,
    })
}

// Dual-engine runner: engine A (stats on) alternates with engine B (stats off).
// Both engines share the same MNN graph but engine B skips stats readback,
// saving the per-frame GPU->CPU readback stall (~10-20 ms at 4K).
fn run_dual(
    label: &str,
    w: u32,
    h: u32,
    frames: usize,
    fmt: OutputFormat,
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
    engine_a.set_stats_enabled(true); // engine A reads stats every frame
    if let Err(e) = engine_a.build(head_a, blocks_a, None, 21) {
        eprintln!("{label:<28} engine A build failed: {e}");
        return None;
    }

    let mut engine_b = MnnEngine::with_pool_size(MnnBackend::Vulkan, 1);
    engine_b.set_stats_enabled(false); // engine B skips stats entirely
    let (head_b, blocks_b) = make(w, h);
    if let Err(e) = engine_b.build(head_b, blocks_b, None, 21) {
        eprintln!("{label:<28} engine B build failed: {e}");
        return None;
    }

    let dual = DualGpuEngine::new(Box::new(engine_a), Box::new(engine_b));

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

    let params = ProcessParams {
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
        output_format: fmt,
        timestamp_ns: 0,
        isp_params: None,
    };

    // Warm up: the first frames include session allocation and shape propagation.
    for _ in 0..8 {
        match dual.next().process(&params) {
            Ok(_) => {}
            Err(e) => {
                eprintln!("{label:<28} warmup failed: {e}");
                return None;
            }
        }
    }

    let (mut total, mut prep, mut infer, mut post) = (
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
        Vec::with_capacity(frames),
    );
    let (mut total_sum, mut prep_sum, mut infer_sum, mut post_sum) = (0.0, 0.0, 0.0, 0.0);
    let mut picture = false;
    for _ in 0..frames {
        match dual.next().process(&params) {
            Ok(f) => {
                let total_ms = ms(f.total_duration_ns);
                let prep_ms = ms(f.prep_duration_ns);
                let infer_ms = ms(f.inference_duration_ns);
                let post_ms = ms(f
                    .total_duration_ns
                    .saturating_sub(f.prep_duration_ns.saturating_add(f.inference_duration_ns)));
                total.push(total_ms);
                prep.push(prep_ms);
                infer.push(infer_ms);
                post.push(post_ms);
                total_sum += total_ms;
                prep_sum += prep_ms;
                infer_sum += infer_ms;
                post_sum += post_ms;
                if !picture {
                    // Packed output is BGRA bytes in a u32 word, not four f32
                    // bytes. Interpret it as floats only made every frame look
                    // blank (NaNs compare false), so use a byte-level check.
                    picture = f.data.iter().any(|b| *b != 0);
                }
            }
            Err(e) => {
                eprintln!("{label:<28} process failed: {e}");
                return None;
            }
        }
    }
    let total = phase_stats(total, total_sum);
    let prep = phase_stats(prep, prep_sum);
    let infer = phase_stats(infer, infer_sum);
    let post = phase_stats(post, post_sum);
    if !picture {
        eprintln!("{label:<28} produced a blank frame");
        return None;
    }
    Some(Timing {
        label: label.to_string(),
        blocks: n_blocks,
        fps: 1000.0 / total.p50.max(0.001),
        total,
        prep,
        infer,
        post,
    })
}

#[test]
#[ignore]
fn perf_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    let fused = || PipelineProfile::FUSED;
    let med = || PipelineProfile::MED;
    let whole = || PipelineProfile::WHOLE;
    let unified = || PipelineProfile::UNIFIED;

    let build = |p: PipelineProfile, fmt: OutputFormat| {
        move |w: u32, h: u32| {
            let mut p = p;
            p.output_format = fmt;
            let mut blocks = p.build_blocks(w, h, 2);
            let head = blocks.remove(0);
            let aux = p.build_aux_blocks(h as i64, w as i64);
            blocks.extend(aux);
            (head, blocks)
        }
    };

    let formats = [
        ("PackedRgb", OutputFormat::PackedRgb),
        ("Bgra", OutputFormat::Bgra),
        ("Argb", OutputFormat::Argb),
    ];

    eprintln!("\n== {w}x{h}, {frames} frames after warmup (single GPU engine) ==");
    eprintln!(
        "{:<30} {:>7} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>8} {:<10}",
        "profile",
        "blocks",
        "prep ms",
        "infer ms",
        "post ms",
        "p50 ms",
        "p95 ms",
        "max ms",
        "FPS",
        "format"
    );

    let mut rows: Vec<Timing> = Vec::new();
    for (fmt_name, fmt) in formats {
        for (label, p) in [
            ("MED (primitives)", med()),
            ("FUSED", fused()),
            ("FUSED, stats off", fused()),
            ("WHOLE", whole()),
            ("UNIFIED", unified()),
        ] {
            let stats_on = !label.ends_with("stats off");
            if let Some(t) = run_single(label, w, h, frames, stats_on, fmt, build(p, fmt)) {
                eprintln!(
                    "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>8.1} {:<10}",
                    format!("{} ({})", t.label, fmt_name),
                    t.blocks,
                    t.prep.mean,
                    t.infer.mean,
                    t.post.mean,
                    t.total.p50,
                    t.total.p95,
                    t.total.max,
                    t.fps,
                    fmt_name
                );
                rows.push(t);
            }
        }
    }

    if let (Some(med_r), Some(fused_r)) = (
        rows.iter().find(|r| r.label.starts_with("MED")),
        rows.iter().find(|r| r.label == "FUSED"),
    ) {
        eprintln!(
            "\nfusion: {:.2}x at p50, mean {:.1} -> {:.1} ms",
            med_r.total.p50 / fused_r.total.p50.max(0.001),
            med_r.total.mean,
            fused_r.total.mean
        );
    }
    if let (Some(on), Some(off)) = (
        rows.iter().find(|r| r.label == "FUSED"),
        rows.iter().find(|r| r.label.ends_with("stats off")),
    ) {
        eprintln!(
            "statistics readback: {:.1} ms per frame ({:.0}% of the p50)",
            on.post.mean - off.post.mean,
            100.0 * (on.post.mean - off.post.mean) / on.total.p50.max(0.01)
        );
    }
}

#[test]
#[ignore]
fn perf_4k_dual() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    let fused = || PipelineProfile::FUSED;

    let build = |p: PipelineProfile, fmt: OutputFormat| {
        move |w: u32, h: u32| {
            let mut p = p;
            p.output_format = fmt;
            let mut blocks = p.build_blocks(w, h, 2);
            let head = blocks.remove(0);
            let aux = p.build_aux_blocks(h as i64, w as i64);
            blocks.extend(aux);
            (head, blocks)
        }
    };

    let formats = [
        ("PackedRgb", OutputFormat::PackedRgb),
        ("Bgra", OutputFormat::Bgra),
        ("Argb", OutputFormat::Argb),
    ];

    eprintln!("\n== {w}x{h}, {frames} frames after warmup (DUAL GPU engine) ==");
    eprintln!(
        "{:<30} {:>7} {:>9} {:>9} {:>9} {:>9} {:>9} {:>9} {:>8} {:<10}",
        "profile",
        "blocks",
        "prep ms",
        "infer ms",
        "post ms",
        "p50 ms",
        "p95 ms",
        "max ms",
        "FPS",
        "format"
    );

    let mut rows: Vec<Timing> = Vec::new();
    for (fmt_name, fmt) in formats {
        for (label, p) in [("DUAL fused", fused())] {
            if let Some(t) = run_dual(label, w, h, frames, fmt, build(p, fmt)) {
                eprintln!(
                    "{:<30} {:>7} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>9.2} {:>8.1} {:<10}",
                    format!("{} ({})", t.label, fmt_name),
                    t.blocks,
                    t.prep.mean,
                    t.infer.mean,
                    t.post.mean,
                    t.total.p50,
                    t.total.p95,
                    t.total.max,
                    t.fps,
                    fmt_name
                );
                rows.push(t);
            }
        }
    }

    if let Some(fused_r) = rows.first() {
        eprintln!(
            "\nDUAL engine: engine A stats ON + engine B stats OFF, mean {:.1} ms \
             (prep {:.1}, infer {:.1}, post {:.1})",
            fused_r.total.mean, fused_r.prep.mean, fused_r.infer.mean, fused_r.post.mean
        );
    }
}
