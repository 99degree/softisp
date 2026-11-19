//! How much of the 4K statistics cost is GPU work, and how much is readback?
//!
//! `probe_4k_stages` showed the statistics are 67% of the 4K chain. Two very
//! different fixes follow from which half that is:
//!
//!   * GPU work — the reductions traverse a 99 MB tensor several times each, and the
//!     fix is to reduce the frame *first* (cheap) and reduce that.
//!   * readback — `copyToHostTensor` stalls the queue, and the fix is to fetch less
//!     often (`set_stats_interval`) or not at all (`set_stats_enabled`).
//!
//! `set_stats_interval` gates only the fetch, so the difference between interval 1
//! and interval 64 is the readback. Whatever remains is GPU.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_stats_split --features mnn -- --nocapture --ignored

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

fn measure(
    label: &str,
    w: u32,
    h: u32,
    frames: usize,
    interval: u32,
    enabled: bool,
) -> Option<f64> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    let mut blocks = p.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.set_stats_interval(interval);
    engine.set_stats_enabled(enabled);
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  {label:<34} build failed: {e}");
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
    eprintln!("  {label:<34} {p50:>8.2} ms");
    Some(p50)
}

#[test]
#[ignore]
fn stats_split_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 15);

    eprintln!("\n== {w}x{h}: the statistics, split by where the cost is ==");
    let a = measure("interval 1 (GPU + readback)", w, h, frames, 1, true);
    let b = measure("interval 64 (GPU only)", w, h, frames, 64, true);
    let c = measure("disabled (nothing to fetch)", w, h, frames, 1, false);

    if let (Some(a), Some(b), Some(c)) = (a, b, c) {
        eprintln!(
            "\nreadback cost        {:>8.2} ms/frame  (interval 1 -> 64)",
            a - b
        );
        eprintln!(
            "GPU work cost        {:>8.2} ms/frame  (interval 64 -> off)",
            b - c
        );
        eprintln!("statistics total     {:>8.2} ms/frame", a - c);
        eprintln!(
            "\nThe GPU half is the reductions traversing a {:.0} MB frame;\n\
             the readback half is `copyToHostTensor` waiting for the queue.",
            (w as f64) * (h as f64) * 3.0 * 4.0 / 1e6
        );
    }
}
