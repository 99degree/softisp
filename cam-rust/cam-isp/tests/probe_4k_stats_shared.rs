//! 4K: what does the shared statistics downscale actually save?
//!
//! `probe_4k_stages` measures `FUSED` with `stats_downscale_max = 0` — the profile's
//! default — so every statistics block still reduces the full 100 MB frame. This
//! sweeps the lever: one pool, then four reductions of a small tensor.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_4k_stats_shared --features mnn -- --nocapture --ignored

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

fn measure(label: &str, stats_max: u32, w: u32, h: u32, frames: usize) -> Option<f64> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(h as i64, w as i64);
    let n = aux.len();
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  {label:<28} build failed: {e}");
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
    let mut mean = 0.0;
    for _ in 0..frames {
        let f = engine.process(&params).ok()?;
        t.push(f.inference_duration_ns as f64 / 1e6);
        if mean == 0.0 {
            let vals: Vec<f32> = f
                .data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect();
            mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len().max(1) as f64;
        }
    }
    t.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p50 = t[t.len() / 2];
    eprintln!("  {label:<28} {n:>2} aux  {p50:>8.2} ms  frame mean {mean:.4}");
    Some(p50)
}

#[test]
#[ignore]
fn stats_shared_4k() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let w = env_usize("PERF_W", 3840) as u32;
    let h = env_usize("PERF_H", 2160) as u32;
    let frames = env_usize("PERF_FRAMES", 20);

    eprintln!("\n== {w}x{h}: one pool, four reductions ==");
    let a = measure("statistics at full res", 0, w, h, frames);
    let b = measure("pooled to 1080", 1080, w, h, frames);
    let c = measure("pooled to 540", 540, w, h, frames);
    let d = measure("pooled to 384", 384, w, h, frames);

    if let (Some(a), Some(c)) = (a, c) {
        eprintln!(
            "\nshared downscale at 540: {:.2} ms saved of {:.2} ms ({:.0}%)",
            a - c,
            a,
            100.0 * (a - c) / a.max(0.01)
        );
    }
    let _ = (b, d);
}
