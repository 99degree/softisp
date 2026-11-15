//! Rebuild the *exact* graph the app builds and report what comes out.
//!
//! `probe_float_rgb` passes with a minimal chain (raw -> unpack -> normalize ->
//! CFA -> demosaic -> display). The app instead builds `PipelineProfile::MED`
//! plus its aux blocks and reports `rgb mean [0.0,0.0,0.0], max 0`. This walks
//! the same construction and prints each block, so the stage that zeroes the
//! frame is identified rather than guessed.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_med_chain --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

#[test]
#[ignore]
fn med_profile_output() {
    cam_isp::init();

    // Override the frame size to reproduce a device-only failure: the app runs
    // this same chain at 2592x1940 and reports an all-zero frame, while a small
    // frame renders fine.
    let w: u32 = std::env::var("PROBE_W")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(64);
    let h: u32 = std::env::var("PROBE_H")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(48);
    eprintln!("frame size {w}x{h}");
    let sample: u16 = 512;

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    eprintln!("MED chain ({} blocks):", blocks.len() + 1);
    eprintln!("  head: {}", head.id());
    for b in &blocks {
        eprintln!("  -> {}", b.id());
    }

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan");
    engine.build(head, blocks, None, 21).expect("build");

    let mut buf = vec![0u8; (w * h * 2) as usize];
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

    let f = engine.process(&params).expect("process");
    let n = f.data.len() / 4 / 3;
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    eprintln!(
        "frame {}x{} -> {} bytes = {} f32",
        f.width,
        f.height,
        f.data.len(),
        vals.len()
    );

    if n > 0 && vals.len() >= n * 3 {
        for (i, name) in ["R", "G", "B"].iter().enumerate() {
            let plane = &vals[i * n..(i + 1) * n];
            let mean: f64 = plane.iter().map(|v| *v as f64).sum::<f64>() / n as f64;
            let max = plane.iter().cloned().fold(f32::MIN, f32::max);
            let nz = plane.iter().filter(|v| **v != 0.0).count();
            eprintln!("  {name}: mean {mean:.4} max {max:.4} nonzero {nz}/{n}");
        }
    }
}
