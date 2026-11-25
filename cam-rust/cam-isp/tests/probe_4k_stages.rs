//! Where does 4K time actually go? Per-stage cost by removal.
//!
//! `perf_4k` says FUSED is ~323 ms at 3840x2160. That is 27 ms per block for twelve
//! blocks, which no amount of bandwidth explains — so the question is which stages
//! are doing pixel work and which are paying per-dispatch overhead.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_stages --features mnn -- --nocapture --ignored
//!
//! Overrides:
//!   PERF_W / PERF_H     frame size (default 3840x2160)
//!   PERF_FRAMES         frames per configuration (default 20)

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

fn measure(label: &str, mut p: PipelineProfile, w: u32, h: u32, frames: usize) -> Option<f64> {
    p.output_format = OutputFormat::FloatRgb;
    let mut blocks = p.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(h as i64, w as i64);
    let n = blocks.len() + 1;
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  {label:<24} build failed: {e}");
        return None;
    }

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
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
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
    eprintln!("  {label:<24} {n:>3} blocks  {p50:>8.2} ms");
    Some(p50)
}

#[test]
#[ignore]
fn stages_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 20);

    let base = PipelineProfile::FUSED;
    eprintln!("\n== {w}x{h}, cost of each stage (measured by turning it off) ==");

    let Some(full) = measure("full FUSED", base, w, h, frames) else {
        panic!("baseline failed");
    };
    eprintln!();

    let cases: [(&str, fn(&mut PipelineProfile)); 6] = [
        ("no ee", |p| p.use_ee = false),
        ("no tone-shaped tail", |p| p.use_saturation = false),
        ("no stats", |p| {
            p.use_zone_stats = false;
            p.use_channel_means = false;
            p.use_tone_stats = false;
        }),
        ("no zone stats", |p| p.use_zone_stats = false),
        ("no channel means", |p| p.use_channel_means = false),
        ("no tone stats", |p| p.use_tone_stats = false),
    ];

    eprintln!("\nwithout each stage:");
    let mut saved: Vec<(&str, f64)> = Vec::new();
    for (label, f) in cases {
        let mut p = base;
        f(&mut p);
        if let Some(t) = measure(label, p, w, h, frames) {
            saved.push((label, full - t));
        }
    }

    eprintln!("\nsavings, largest first:");
    saved.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    for (label, s) in &saved {
        eprintln!(
            "  {label:<24} {s:>8.2} ms  ({:.0}% of {full:.0} ms)",
            100.0 * s / full
        );
    }
}
