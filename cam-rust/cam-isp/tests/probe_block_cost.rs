//! Which MED stages cost the most?
//!
//! The chain is 48 blocks and ~370 ms at 1296x970. This turns one stage off at
//! a time using the profile's own toggles, so the builder still produces a valid
//! chain — removing a block after the fact leaves the aux hooks pointing at
//! tensors that no longer exist, which crashes the backend rather than
//! measuring anything.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_block_cost --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

fn measure(label: &str, mut profile: PipelineProfile) -> Option<f64> {
    let (w, h) = (2592u32, 1940u32);
    let sample: u16 = 512;

    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let n_blocks = blocks.len() + 1;

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  {label:<22} build failed: {e}");
        return None;
    }

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
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

    if let Err(e) = engine.process(&params) {
        eprintln!("  {label:<22} process failed: {e}");
        return None;
    }

    let n = 3;
    let mut infer = 0.0;
    for _ in 0..n {
        if let Ok(f) = engine.process(&params) {
            infer = f.inference_duration_ns as f64 / 1e6;
        }
    }
    eprintln!("  {label:<22} {n_blocks:>3} blocks   infer {infer:7.1} ms");
    Some(infer)
}

#[test]
#[ignore]
fn per_stage_cost() {
    cam_isp::init();

    let Some(base) = measure("full MED", PipelineProfile::MED) else {
        panic!("baseline failed");
    };
    eprintln!();

    let mut results: Vec<(&str, f64)> = Vec::new();
    let cases: [(&str, fn(&mut PipelineProfile)); 8] = [
        ("use_ldci = false", |p| p.use_ldci = false),
        ("use_ee = false", |p| p.use_ee = false),
        ("use_bilateral = false", |p| p.use_bilateral = false),
        ("use_sharpen = false", |p| p.use_sharpen = false),
        ("use_saturation = false", |p| p.use_saturation = false),
        ("use_gamma = false", |p| p.use_gamma = false),
        ("use_colorspace = false", |p| p.use_colorspace = false),
        ("use_lsc = false", |p| p.use_lsc = false),
    ];

    for (label, f) in cases {
        let mut p = PipelineProfile::MED;
        f(&mut p);
        if let Some(t) = measure(label, p) {
            results.push((label, base - t));
        }
    }

    eprintln!("\ncost of each stage (measured by turning it off):");
    results.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    for (label, saved) in &results {
        eprintln!("  {label:<22} {saved:7.1} ms");
    }
}
