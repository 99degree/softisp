//! Which levers actually move inference time, measured one at a time.
//!
//! Every knob below is already implemented; none of them are on. Rather than
//! guess, this runs the MED profile with one change at a time and reports the
//! inference latency, so the choice is made from numbers.
//!
//! The knobs:
//!
//!   opset        0 = Primitive, 2 = MnnIspMin (fused isp.* Extra ops)
//!   fp16         fp16 tensor storage (engine default: on)
//!   tail         post_ccm_downscale — shrink everything after CCM
//!   stats        stats_downscale_max — shrink only the statistics tap
//!
//! Run:
//!   cargo test -p cam-isp --test probe_speed_levers --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::profile::PipelineProfile;

/// One configuration to measure.
#[derive(Clone, Copy)]
struct Lever {
    label: &'static str,
    opset: i32,
    fp16: bool,
    tail_div: u32,
    stats_max: u32,
}

#[derive(Default)]
struct Result_ {
    infer_ms: f64,
    wall_ms: f64,
    mean: f64,
    peak: f64,
    /// Distinct values in a sparse sample of plane 0 — 1 means a flat frame.
    distinct: usize,
}

fn measure(l: Lever) -> Option<Result_> {
    let (w, h) = (2592u32, 1940u32);
    let sample: u16 = 600;

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.opset_mode = l.opset;
    if l.stats_max > 0 {
        profile.stats_downscale_max = l.stats_max;
    }
    if l.tail_div > 1 {
        // The tail runs on the demosaiced grid, so its geometry is half the
        // sensor in each axis before the extra division is applied.
        profile.post_ccm_downscale = ((w / 2 / l.tail_div).max(2), (h / 2 / l.tail_div).max(2));
    }

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    if let Some(m) = engine
        .as_any_mut()
        .downcast_mut::<cam_isp::mnnengine::MnnEngine>()
    {
        m.set_fp16_storage(l.fp16);
    }
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("{:<26} BUILD FAIL: {e}", l.label);
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
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };

    if engine.process(&params).is_err() {
        eprintln!("{:<26} RUN FAIL", l.label);
        return None;
    }

    let mut out = Result_::default();
    let n = 3;
    let t0 = Instant::now();
    let mut last = None;
    for _ in 0..n {
        match engine.process(&params) {
            Ok(f) => {
                out.infer_ms = f.inference_duration_ns as f64 / 1e6;
                last = Some(f);
            }
            Err(e) => {
                eprintln!("{:<26} FAIL: {e}", l.label);
                return None;
            }
        }
    }
    out.wall_ms = t0.elapsed().as_secs_f64() * 1000.0 / n as f64;

    let f = last?;
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return None;
    }
    out.mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
    out.peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let plane0 = &vals[..vals.len() / 3];
    let step = (plane0.len() / 2048).max(1);
    let mut seen = std::collections::HashSet::new();
    for v in plane0.iter().step_by(step) {
        seen.insert(v.to_bits());
    }
    out.distinct = seen.len();
    Some(out)
}

#[test]
#[ignore]
fn speed_levers() {
    cam_isp::init();

    // `baseline` is the configuration the app ships today.
    let levers = [
        Lever {
            label: "baseline (as shipped)",
            opset: 0,
            fp16: true,
            tail_div: 1,
            stats_max: 0,
        },
        Lever {
            label: "fp32 storage",
            opset: 0,
            fp16: false,
            tail_div: 1,
            stats_max: 0,
        },
        Lever {
            label: "fused isp.* (opset 2)",
            opset: 2,
            fp16: true,
            tail_div: 1,
            stats_max: 0,
        },
        Lever {
            label: "stats tap @540",
            opset: 0,
            fp16: true,
            tail_div: 1,
            stats_max: 540,
        },
        Lever {
            label: "tail /2",
            opset: 0,
            fp16: true,
            tail_div: 2,
            stats_max: 0,
        },
        Lever {
            label: "tail /2 + fused",
            opset: 2,
            fp16: true,
            tail_div: 2,
            stats_max: 0,
        },
        Lever {
            label: "tail /2 + stats @540",
            opset: 0,
            fp16: true,
            tail_div: 2,
            stats_max: 540,
        },
        Lever {
            label: "all three",
            opset: 2,
            fp16: true,
            tail_div: 2,
            stats_max: 540,
        },
    ];

    eprintln!(
        "\n{:<26} {:>9} {:>9} {:>8} {:>8} {:>9}  {}",
        "configuration", "infer ms", "wall ms", "mean", "peak", "distinct", "verdict"
    );

    let mut base = 0.0;
    for (i, l) in levers.iter().enumerate() {
        match measure(*l) {
            Some(r) => {
                if i == 0 {
                    base = r.infer_ms;
                }
                let delta = if base > 0.0 {
                    format!("{:+.0}%", (r.infer_ms - base) / base * 100.0)
                } else {
                    "-".into()
                };
                // A frame that is flat or black is not a speed-up, it is a
                // deleted stage.
                let verdict = if r.peak <= 0.2 || r.mean <= 0.05 {
                    "BLANK"
                } else if r.peak > 1.5 {
                    "OVER-RANGE"
                } else {
                    delta.as_str()
                };
                eprintln!(
                    "{:<26} {:>9.1} {:>9.1} {:>8.4} {:>8.3} {:>9}  {}",
                    l.label, r.infer_ms, r.wall_ms, r.mean, r.peak, r.distinct, verdict
                );
            }
            None => eprintln!("{:<26} (failed)", l.label),
        }
    }

    eprintln!(
        "\n`OVER-RANGE` means the fused path writes values above 1.0 — it is faster\n\
         but not yet value-identical to the primitive chain."
    );
}
