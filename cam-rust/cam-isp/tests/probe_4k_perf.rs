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
//! Run one profile at a time:
//!   cargo test -p cam-isp --test probe_4k_perf perf_4k_fused --features mnn -- --nocapture --ignored
//!   cargo test -p cam-isp --test probe_4k_perf perf_4k_heavy --features mnn -- --nocapture --ignored
//!
//! `perf_4k` is kept as an alias for the FUSED breakage check. Use `perf_4k_all`
//! only when the complete matrix is wanted. `perf_critical_path` is the focused
//! P2h probe: it splits the visible path into prefill + inference + output and
//! reports the deferred 3A readback separately.
//!
//! Overrides:
//!   PERF_W / PERF_H   frame size (default 3840x2160)
//!   PERF_FRAMES       frames per profile after warmup (default 40)

#![cfg(feature = "mnn")]

use cam_isp::dual_engine::DualGpuEngine;
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::pipeline::IspFrame;
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
    phases: Phases,
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

/// Engine-owned phase split, reported by `MnnEngine` through `IspFrame::aux`.
///
/// Camera fill and display handoff remain outside the engine; these are the
/// phases a staged caller can actually move off the visible path.
#[derive(Clone, Copy, Default)]
struct Phases {
    setup: PhaseStats,
    upload: PhaseStats,
    inference: PhaseStats,
    output_meta: PhaseStats,
    output_copy: PhaseStats,
    readback: PhaseStats,
    visible: PhaseStats,
}

#[derive(Default)]
struct PhaseSamples {
    total: Vec<f64>,
    total_sum: f64,
    prep: Vec<f64>,
    prep_sum: f64,
    infer: Vec<f64>,
    infer_sum: f64,
    post: Vec<f64>,
    post_sum: f64,
    setup: Vec<f64>,
    setup_sum: f64,
    upload: Vec<f64>,
    upload_sum: f64,
    output_meta: Vec<f64>,
    output_meta_sum: f64,
    output_copy: Vec<f64>,
    output_copy_sum: f64,
    readback: Vec<f64>,
    readback_sum: f64,
    visible: Vec<f64>,
    visible_sum: f64,
}

fn sample(into: &mut Vec<f64>, sum: &mut f64, value: f64) {
    into.push(value);
    *sum += value;
}

impl PhaseSamples {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            total: Vec::with_capacity(capacity),
            prep: Vec::with_capacity(capacity),
            infer: Vec::with_capacity(capacity),
            post: Vec::with_capacity(capacity),
            setup: Vec::with_capacity(capacity),
            upload: Vec::with_capacity(capacity),
            output_meta: Vec::with_capacity(capacity),
            output_copy: Vec::with_capacity(capacity),
            readback: Vec::with_capacity(capacity),
            visible: Vec::with_capacity(capacity),
            ..Default::default()
        }
    }

    fn push(&mut self, f: &IspFrame) {
        let total_ms = ms(f.total_duration_ns);
        let prep_ms = ms(f.prep_duration_ns);
        let infer_ms = ms(f.inference_duration_ns);
        let post_ms = ms(f
            .total_duration_ns
            .saturating_sub(f.prep_duration_ns.saturating_add(f.inference_duration_ns)));
        sample(&mut self.total, &mut self.total_sum, total_ms);
        sample(&mut self.prep, &mut self.prep_sum, prep_ms);
        sample(&mut self.infer, &mut self.infer_sum, infer_ms);
        sample(&mut self.post, &mut self.post_sum, post_ms);

        if let Some(t) = f.aux.as_ref().and_then(|a| a.timing) {
            sample(&mut self.setup, &mut self.setup_sum, ms(t.setup_ns));
            sample(
                &mut self.upload,
                &mut self.upload_sum,
                ms(t.tensor_upload_ns),
            );
            sample(
                &mut self.inference,
                &mut self.inference_sum,
                ms(t.inference_ns),
            );
            sample(
                &mut self.output_meta,
                &mut self.output_meta_sum,
                ms(t.output_meta_ns),
            );
            sample(
                &mut self.output_copy,
                &mut self.output_copy_sum,
                ms(t.output_copy_ns),
            );
            sample(
                &mut self.readback,
                &mut self.readback_sum,
                ms(t.readback_ns),
            );
            sample(&mut self.visible, &mut self.visible_sum, ms(t.visible_ns()));
        }
    }

    fn finish(self) -> (PhaseStats, PhaseStats, PhaseStats, PhaseStats, Phases) {
        let phases = Phases {
            setup: phase_stats(self.setup, self.setup_sum),
            upload: phase_stats(self.upload, self.upload_sum),
            inference: phase_stats(self.infer, self.infer_sum),
            output_meta: phase_stats(self.output_meta, self.output_meta_sum),
            output_copy: phase_stats(self.output_copy, self.output_copy_sum),
            readback: phase_stats(self.readback, self.readback_sum),
            visible: phase_stats(self.visible, self.visible_sum),
        };
        (
            phase_stats(self.total, self.total_sum),
            phase_stats(self.prep, self.prep_sum),
            phase_stats(self.infer, self.infer_sum),
            phase_stats(self.post, self.post_sum),
            phases,
        )
    }
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

    let mut samples = PhaseSamples::with_capacity(frames);
    let mut picture = false;
    for _ in 0..frames {
        match engine.process(&params) {
            Ok(f) => {
                samples.push(&f);
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
    let (total, prep, infer, post, phases) = samples.finish();
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
        phases,
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

    let mut samples = PhaseSamples::with_capacity(frames);
    let mut picture = false;
    for _ in 0..frames {
        match dual.next().process(&params) {
            Ok(f) => {
                samples.push(&f);
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
    let (total, prep, infer, post, phases) = samples.finish();
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
        phases,
    })
}

const PERF_FORMATS: &[(&str, OutputFormat)] = &[
    ("PackedRgb", OutputFormat::PackedRgb),
    ("Bgra", OutputFormat::Bgra),
    ("Argb", OutputFormat::Argb),
];

fn print_perf_header(title: &str) {
    eprintln!("\n== {title} ==");
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
}

fn print_timing(t: &Timing, fmt_name: &str) {
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
    if t.phases.visible.mean > 0.0 {
        let p = &t.phases;
        eprintln!(
            "{:<30} critical {:.2} ms = prefill {:.2} (setup {:.2} + upload {:.2}) \
             + infer {:.2} + output {:.2} (meta {:.2} + copy {:.2}); \
             deferred readback {:.2}",
            "",
            p.visible.mean,
            p.setup.mean + p.upload.mean,
            p.setup.mean,
            p.upload.mean,
            p.inference.mean,
            p.output_meta.mean + p.output_copy.mean,
            p.output_meta.mean,
            p.output_copy.mean,
            p.readback.mean
        );
    }
}

fn build_profile(
    profile: PipelineProfile,
    fmt: OutputFormat,
) -> impl Fn(
    u32,
    u32,
) -> (
    Box<dyn cam_isp::pipeline::IspBlock>,
    Vec<Box<dyn cam_isp::pipeline::IspBlock>>,
) {
    move |w: u32, h: u32| {
        let mut p = profile;
        p.output_format = fmt;
        let mut blocks = p.build_blocks(w, h, 2);
        let head = blocks.remove(0);
        let aux = p.build_aux_blocks(h as i64, w as i64);
        blocks.extend(aux);
        (head, blocks)
    }
}

fn init_probe() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();
}

/// One profile, all output formats, single engine. `include_stats_off` adds the
/// FUSED stats-disabled arm so the readback cost stays measurable, but it is
/// not repeated for every profile.
fn run_single_profile_case(label: &str, profile: PipelineProfile, include_stats_off: bool) {
    init_probe();
    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    print_perf_header(&format!(
        "{w}x{h}, {frames} frames after warmup (single GPU engine)"
    ));

    let mut rows: Vec<Timing> = Vec::new();
    for (fmt_name, fmt) in PERF_FORMATS {
        let cases: [(&str, PipelineProfile, bool); 2] =
            [(label, profile, true), ("FUSED, stats off", profile, false)];
        for (case_label, p, stats_on) in cases {
            if !stats_on && !(include_stats_off && label == "FUSED") {
                continue;
            }
            if let Some(t) = run_single(
                case_label,
                w,
                h,
                frames,
                stats_on,
                *fmt,
                build_profile(p, *fmt),
            ) {
                print_timing(&t, fmt_name);
                rows.push(t);
            }
        }
    }

    if include_stats_off && label == "FUSED" {
        if let (Some(on), Some(off)) = (
            rows.iter().find(|r| r.label == "FUSED"),
            rows.iter().find(|r| r.label.ends_with("stats off")),
        ) {
            let saved = (on.phases.readback.mean - off.phases.readback.mean).max(0.0);
            eprintln!(
                "statistics readback: {:.1} ms per frame ({:.0}% of the p50)",
                saved,
                100.0 * saved / on.total.p50.max(0.01)
            );
        }
    }
}

fn run_dual_fused_case() {
    init_probe();
    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    print_perf_header(&format!(
        "{w}x{h}, {frames} frames after warmup (DUAL GPU engine)"
    ));

    let mut rows: Vec<Timing> = Vec::new();
    for (fmt_name, fmt) in PERF_FORMATS {
        if let Some(t) = run_dual(
            "DUAL fused",
            w,
            h,
            frames,
            *fmt,
            build_profile(PipelineProfile::FUSED, *fmt),
        ) {
            print_timing(&t, fmt_name);
            rows.push(t);
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

/// Focused critical-path run: FUSED/PackedRgb with stats on and off, printed as
/// prefill + inference + output versus deferred readback. This is the probe for
/// the P2h goal of shrinking the visible path toward `runSession` alone.
fn run_critical_path_case() {
    init_probe();
    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 40);

    eprintln!("\n== critical path: FUSED / PackedRgb, {w}x{h}, {frames} frames ==");
    let on = run_single(
        "FUSED stats on",
        w,
        h,
        frames,
        true,
        OutputFormat::PackedRgb,
        build_profile(PipelineProfile::FUSED, OutputFormat::PackedRgb),
    );
    let off = run_single(
        "FUSED stats off",
        w,
        h,
        frames,
        false,
        OutputFormat::PackedRgb,
        build_profile(PipelineProfile::FUSED, OutputFormat::PackedRgb),
    );
    if let (Some(on), Some(off)) = (on, off) {
        print_timing(&on, "PackedRgb");
        print_timing(&off, "PackedRgb");
        eprintln!(
            "\nvisible critical path: {:.2} ms with stats, {:.2} ms without; \
             deferred readback {:.2} ms",
            on.phases.visible.mean, off.phases.visible.mean, on.phases.readback.mean
        );
    }
}

/// Backward-compatible name: the usual breakage check only runs FUSED.
#[test]
#[ignore]
fn perf_4k() {
    run_single_profile_case("FUSED", PipelineProfile::FUSED, true);
}

#[test]
#[ignore]
fn perf_4k_fused() {
    run_single_profile_case("FUSED", PipelineProfile::FUSED, true);
}

#[test]
#[ignore]
fn perf_4k_med() {
    run_single_profile_case("MED (primitives)", PipelineProfile::MED, false);
}

#[test]
#[ignore]
fn perf_4k_heavy() {
    run_single_profile_case("HEAVY", PipelineProfile::HEAVY, false);
}

#[test]
#[ignore]
fn perf_4k_whole() {
    run_single_profile_case("WHOLE", PipelineProfile::WHOLE, false);
}

#[test]
#[ignore]
fn perf_4k_unified() {
    run_single_profile_case("UNIFIED", PipelineProfile::UNIFIED, false);
}

#[test]
#[ignore]
fn perf_4k_dual() {
    run_dual_fused_case();
}

#[test]
#[ignore]
fn perf_critical_path() {
    run_critical_path_case();
}

/// Explicit opt-in for the full matrix. Individual profile tests above are the
/// default way to check a change without paying for every profile.
#[test]
#[ignore]
fn perf_4k_all() {
    run_single_profile_case("MED (primitives)", PipelineProfile::MED, false);
    run_single_profile_case("FUSED", PipelineProfile::FUSED, true);
    run_single_profile_case("HEAVY", PipelineProfile::HEAVY, false);
    run_single_profile_case("WHOLE", PipelineProfile::WHOLE, false);
    run_single_profile_case("UNIFIED", PipelineProfile::UNIFIED, false);
    run_dual_fused_case();
}
