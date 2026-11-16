//! Turn on direct `isp.*` emission and see what the chain does with it.
//!
//! `PipelineProfile::opset_mode` selects how each fusion-capable block emits
//! itself: 0 = Primitive (ONNX ops), 1 = Custom, 2 = MnnIspMin (fused `isp.*`
//! Extra ops). MED ships with 0, so nothing in the live preview has ever
//! exercised the fused path end to end.
//!
//! This runs the *same* profile twice, once per mode, and reports what each
//! produces: whether the graph builds, whether the frame still looks like a
//! picture, and how long it takes. The point is not to prove fusion is faster —
//! it is to find out which stages survive it.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_direct_opset --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

struct Outcome {
    infer_ms: f64,
    mean: f64,
    peak: f64,
    nonfinite: usize,
    distinct: usize,
}

fn run(opset_mode: i32) -> Result<Outcome, String> {
    let (w, h) = (2592u32, 1940u32);
    let sample: u16 = 600;

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.opset_mode = opset_mode;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| "mnn_vulkan engine not registered".to_string())?;
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

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

    // One warm-up so the timing below is steady state.
    engine.process(&params).map_err(|e| format!("first: {e}"))?;

    let t0 = Instant::now();
    let frame = engine
        .process(&params)
        .map_err(|e| format!("second: {e}"))?;
    let _ = t0;
    let infer_ms = frame.inference_duration_ns as f64 / 1e6;

    let vals: Vec<f32> = frame
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return Err("empty frame".into());
    }
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
    let peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let nonfinite = vals.iter().filter(|v| !v.is_finite()).count();
    let plane0 = &vals[..vals.len() / 3];
    let step = (plane0.len() / 4096).max(1);
    let distinct = {
        let mut seen = std::collections::HashSet::new();
        for v in plane0.iter().step_by(step) {
            seen.insert(v.to_bits());
        }
        seen.len()
    };

    Ok(Outcome {
        infer_ms,
        mean,
        peak,
        nonfinite,
        distinct,
    })
}

#[test]
#[ignore]
fn direct_opset_emission() {
    cam_isp::init();

    eprintln!(
        "\n{:<22} {:>9} {:>8} {:>8} {:>9} {:>9}  {}",
        "opset_mode", "infer ms", "mean", "peak", "non-finite", "distinct", "state"
    );

    for (mode, label) in [(0, "0 Primitive"), (2, "2 MnnIspMin")] {
        match run(mode) {
            Ok(o) => {
                let state = if o.nonfinite > 0 {
                    "NON-FINITE"
                } else if o.mean <= 0.05 || o.peak <= 0.2 {
                    "BLANK"
                } else {
                    "picture"
                };
                eprintln!(
                    "{label:<22} {:>9.1} {:>8.4} {:>8.3} {:>9} {:>9}  {}",
                    o.infer_ms, o.mean, o.peak, o.nonfinite, o.distinct, state
                );
            }
            Err(e) => eprintln!("{label:<22} FAILED: {e}"),
        }
    }

    // Deliberately not asserting: the point is to see which modes survive on
    // this device, and a fused chain that fails is a finding, not a test bug.
    eprintln!(
        "\nNote: isp.demosaic_ccm / isp.ee / isp.ldci are placeholder shaders in the\n\
         MNN build currently shipped, so MnnIspMin drops those stages even when the\n\
         graph builds. Only isp.display is implemented."
    );
}
