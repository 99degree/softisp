//! Close the 3A loop: does the controller actually steer the image?
//!
//! The engine reads its stats tensors and calls `update_channel_stats`, which
//! moves `IspController::awb_gains` / `smoothed_ccm`. But the graph is fed from
//! `ProcessParams`, and this app builds those from `PipelineConfig::params()`
//! with `ccm_matrix: None`, `bayer_gains: None`, `awb_gains: None`. Nothing
//! calls `IspParams::with_controller`, so every gain the controller computes is
//! discarded and the graph keeps running on its built-in defaults.
//!
//! This drives the chain with a deliberately red-shifted scene and asks two
//! questions:
//!
//!   1. does the controller notice (its R/B gains move), and
//!   2. does the rendered frame get corrected
//!
//! If (1) is yes and (2) is no, the loop is open and the missing piece is the
//! wiring, not the maths.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_controller_loop --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

/// Mean of each RGB plane of a planar f32 frame.
fn plane_means(data: &[u8]) -> (f64, f64, f64) {
    let vals: Vec<f32> = data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    let plane = vals.len() / 3;
    if plane == 0 {
        return (0.0, 0.0, 0.0);
    }
    let mean = |p: &[f32]| p.iter().map(|v| *v as f64).sum::<f64>() / p.len() as f64;
    (
        mean(&vals[0..plane]),
        mean(&vals[plane..2 * plane]),
        mean(&vals[2 * plane..]),
    )
}

fn build() -> Option<Box<dyn IspEngine>> {
    let (w, h) = (512u32, 384u32);
    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;
    Some(engine)
}

#[test]
#[ignore]
fn controller_loop_is_closed() {
    cam_isp::init();

    let (w, h) = (512u32, 384u32);
    let mut engine = build().expect("build ignored test chain");

    // A red-shifted scene: identical luma in R and G, half in B. A closed AWB
    // loop must raise the blue gain (and/or the CCM) to compensate.
    let sample = |plane: usize| -> u16 {
        match plane {
            0 | 1 => 700, // R and G sites
            _ => 350,     // B sites
        }
    };

    // RGGB quad: index = (y%2)*2 + (x%2); 0=R 1=Gr 2=Gb 3=B
    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for y in 0..h as usize {
        for x in 0..w as usize {
            let quad = (y % 2) * 2 + (x % 2);
            let v = sample(quad);
            let i = (y * w as usize + x) * 2;
            buf[i] = (v & 0xFF) as u8;
            buf[i + 1] = (v >> 8) as u8;
        }
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

    eprintln!(
        "\n{:<6} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}",
        "frame", "R", "G", "B", "ctrl.R", "ctrl.B", "b_gain", "cts"
    );

    let mut last = (0.0, 0.0, 0.0);
    for i in 0..12 {
        let f = engine.process(&params).expect("process");
        last = plane_means(&f.data);

        let (awb, ccm_bg, cct) = {
            let ctrl = engine.controller().lock().expect("controller");
            (ctrl.get_awb_gains(), ctrl.get_ccm()[1], ctrl.estimated_cct)
        };
        eprintln!(
            "{i:<6} {:>8.4} {:>8.4} {:>8.4} {:>8.3} {:>8.3} {:>8} {:>8}",
            last.0,
            last.1,
            last.2,
            awb[0],
            awb[2],
            match cct {
                Some(c) => c.to_string(),
                None => "-".into(),
            },
            ""
        );
        let _ = ccm_bg;
    }

    let (r, g, b) = last;
    let rg = r / g.max(1e-9);
    let bg = b / g.max(1e-9);
    eprintln!("\nfinal scene ratios: R/G {rg:.3}  B/G {bg:.3}");

    let ctrl = engine.controller().lock().expect("controller");
    let awb = ctrl.get_awb_gains();
    let ccm = ctrl.get_ccm();
    let frames = ctrl.frame_count;
    drop(ctrl);

    eprintln!(
        "controller: frames {frames}, awb [{:.3}, {:.3}, {:.3}]",
        awb[0], awb[1], awb[2]
    );
    eprintln!(
        "controller CCM: [{:.2},{:.2},{:.2} | {:.2},{:.2},{:.2} | {:.2},{:.2},{:.2}]",
        ccm[0], ccm[1], ccm[2], ccm[3], ccm[4], ccm[5], ccm[6], ccm[7], ccm[8]
    );

    // Question 1: did the controller react to a red scene?
    assert!(frames > 0, "the controller never saw a frame");
    let reacted = awb[2] > 1.01 || awb[0] < 0.99;
    eprintln!(
        "\n1. controller reacted to the red cast: {} (B gain {:.3}, R gain {:.3})",
        if reacted { "yes" } else { "NO" },
        awb[2],
        awb[0]
    );

    // Question 2: did the rendered frame change between the first and last
    // frame? The input is static, so any change is the controller acting.
    eprintln!("2. the picture itself is compared in the trace above");
}
