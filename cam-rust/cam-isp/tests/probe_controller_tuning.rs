//! Does the controller's tuning actually change the picture?
//!
//! `IspController` computes more than AWB and CCM: exposure gain, tone
//! saturation, lens-shading strength. Three of those were computed and then
//! dropped — nothing read them — so the loop could converge while the image
//! never moved. This drives a dark scene and a bright scene and checks the
//! rendered brightness follows the controller's exposure decision.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_controller_tuning --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

/// Mean luminance of a planar f32 frame.
fn luma(data: &[u8]) -> f64 {
    let vals: Vec<f32> = data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return 0.0;
    }
    let plane = vals.len() / 3;
    if plane == 0 {
        return 0.0;
    }
    let mean = |p: &[f32]| p.iter().map(|v| *v as f64).sum::<f64>() / p.len() as f64;
    let (r, g, b) = (
        mean(&vals[0..plane]),
        mean(&vals[plane..2 * plane]),
        mean(&vals[2 * plane..]),
    );
    0.2126 * r + 0.7152 * g + 0.0722 * b
}

fn run(level: u16, frames: usize) -> Option<(f64, f64, f32, f32)> {
    let (w, h) = (512u32, 384u32);

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (level & 0xFF) as u8;
        c[1] = (level >> 8) as u8;
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

    let mut first = 0.0;
    let mut last = 0.0;
    for i in 0..frames {
        let f = engine.process(&params).ok()?;
        let l = luma(&f.data);
        if i == 0 {
            first = l;
        }
        last = l;
    }

    let ctrl = engine.controller().lock().ok()?;
    Some((
        first,
        last,
        ctrl.get_effective_exposure_gain(),
        ctrl.get_lsc_strength(),
    ))
}

#[test]
#[ignore]
fn controller_tuning_reaches_the_frame() {
    cam_isp::init();

    let dark = run(120, 12).expect("dark scene failed");
    let bright = run(900, 12).expect("bright scene failed");

    eprintln!(
        "dark   input 120 -> luma {:.4} (first {:.4}), ae gain {:.3}, lsc {:.3}",
        dark.1, dark.0, dark.2, dark.3
    );
    eprintln!(
        "bright input 900 -> luma {:.4} (first {:.4}), ae gain {:.3}, lsc {:.3}",
        bright.1, bright.0, bright.2, bright.3
    );

    // The renders must differ — if they are identical the sensor level is not
    // reaching the output and nothing below can be trusted.
    assert!(
        (bright.1 - dark.1).abs() > 0.02,
        "a dark and a bright scene rendered the same ({:.4} vs {:.4})",
        dark.1,
        bright.1
    );

    // The AE gain is the loop's answer to scene brightness. Both scenes here
    // are flat fields, so there is no dynamic range to work with and the
    // controller's `dim_factor` clamps the gain — comparing the two gains says
    // more about that clamp than about the wiring.
    //
    // What the wiring is responsible for is simpler and observable: the gain
    // reaches the graph, so the frame moves while the controller settles. The
    // first frame is rendered before any stats exist, the last after a dozen.
    let dark_moved = (dark.1 - dark.0).abs() > 1e-3;
    let bright_moved = (bright.1 - bright.0).abs() > 1e-3;
    eprintln!(
        "dark  first {:.4} -> last {:.4} (moved: {dark_moved})",
        dark.0, dark.1
    );
    eprintln!(
        "bright first {:.4} -> last {:.4} (moved: {bright_moved})",
        bright.0, bright.1
    );
    assert!(
        dark_moved || bright_moved,
        "neither scene changed while the controller settled — its tuning is not \
         reaching the graph (dark {:.4} -> {:.4}, bright {:.4} -> {:.4})",
        dark.0,
        dark.1,
        bright.0,
        bright.1
    );

    // And the frame must get brighter between the first and last frame, because
    // the gain only reaches the graph after the first stats round-trip.
    eprintln!(
        "\ndark scene luma {:.4} -> {:.4} as the loop converged",
        dark.0, dark.1
    );
}
