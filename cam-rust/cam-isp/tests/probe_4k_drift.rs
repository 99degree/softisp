//! Is the "statistics are 67% of 4K" number real, or clock drift?
//!
//! `probe_4k_stages` measured configurations sequentially, and a 4K run is long
//! enough for the GPU to throttle partway through. That probe already reported one
//! impossible result (`no ee` *saving* negative time), so the sequential sweep has to
//! be checked before its numbers are quoted.
//!
//! This measures baseline / no-stats / baseline again in one process. If the two
//! baselines agree, the middle number is trustworthy; if they do not, the spread is
//! the drift and the claim is not supported.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_drift --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

fn env_usize(key: &str, default: usize) -> usize {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn measure(label: &str, stats: bool, w: u32, h: u32, frames: usize) -> Option<f64> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    if !stats {
        p.use_zone_stats = false;
        p.use_channel_means = false;
        p.use_tone_stats = false;
    }
    let mut blocks = p.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(h as i64, w as i64);
    let n = blocks.len() + aux.len();
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

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
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
    };
    for _ in 0..6 {
        engine.process(&params).ok()?;
    }
    let mut t = Vec::new();
    for _ in 0..frames {
        t.push(engine.process(&params).ok()?.inference_duration_ns as f64 / 1e6);
    }
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p50 = t[t.len() / 2];
    let p10 = t[t.len() / 10];
    let p90 = t[(t.len() * 9) / 10];
    eprintln!("  {label:<26} {n:>3} blocks  p50 {p50:>8.2}  p10 {p10:>8.2}  p90 {p90:>8.2} ms");
    Some(p50)
}

#[test]
#[ignore]
fn drift_check_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 20);

    eprintln!("\n== {w}x{h}: baseline, no-stats, baseline again ==");
    let a = measure("FUSED (first)", true, w, h, frames);
    let b = measure("FUSED, no statistics", false, w, h, frames);
    let c = measure("FUSED (again)", true, w, h, frames);

    if let (Some(a), Some(b), Some(c)) = (a, b, c) {
        let drift = (c - a).abs();
        eprintln!("\nbaseline twice: {a:.2} then {c:.2} ms -> drift {drift:.2} ms");
        if drift > 20.0 {
            eprintln!(
                "  the sweep drifts by more than the effect being measured;\n  \
                 the {:.0} ms 'saving' below is not separable from it",
                a - b
            );
        }
        eprintln!(
            "statistics cost (with the drift caveat): {:.2} ms of {:.2} ({:.0}%)",
            a - b,
            a,
            100.0 * (a - b) / a.max(0.01)
        );
    }
}
