//! Turn fused `isp.*` emission on one stage at a time, in the real profile.
//!
//! `probe_fusion_stepwise` proves the maths for a hand-wired chain. This asks the
//! other question: does the *profile's* chain still render when a stage is
//! switched on, and does the picture survive the stages around it.
//!
//! Each step enables exactly one stage in `PipelineProfile::fuse_stages` and
//! compares against the previous step — so a regression is attributable to the
//! stage that was just turned on, not to "fusion" in general.
//!
//! A stage is only kept if the frame is finite, bright, and close to the
//! reference. Once one fails, the sweep stops: later steps would be measured
//! against a broken baseline.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_enable_fused_stages --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 1296;
const H: u32 = 970;

struct Frame {
    data: Vec<u8>,
    infer_ms: f64,
}

fn render(stages: FuseStages) -> Result<Frame, String> {
    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    // MnnIspMin is what makes `fuse_stages` meaningful; the allowlist decides
    // which stages actually take it up.
    profile.opset_mode = 2;
    profile.fuse_stages = stages;

    let mut blocks = profile.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| "mnn_vulkan engine not registered".to_string())?;
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

    // Mid-grey Bayer field: bright enough that a stage which deletes the frame
    // shows as black rather than as "an unusually dark scene".
    let sample: u16 = 600;
    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: W,
        target_height: H,
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

    // Warm-up: the first call allocates and resizes.
    engine.process(&params).map_err(|e| format!("first: {e}"))?;
    let f = engine
        .process(&params)
        .map_err(|e| format!("second: {e}"))?;
    Ok(Frame {
        infer_ms: f.inference_duration_ns as f64 / 1e6,
        data: f.data,
    })
}

fn floats(data: &[u8]) -> Vec<f32> {
    data.as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect()
}

fn stats(data: &[u8]) -> (f64, f64, usize) {
    let v = floats(data);
    if v.is_empty() {
        return (0.0, 0.0, 0);
    }
    let mean = v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    let peak = v.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let nonfinite = v.iter().filter(|x| !x.is_finite()).count();
    (mean, peak, nonfinite)
}

/// Mean and peak absolute difference, or `None` if the shapes disagree.
fn diff(a: &[u8], b: &[u8]) -> Option<(f64, f64)> {
    let (va, vb) = (floats(a), floats(b));
    if va.len() != vb.len() || va.is_empty() {
        return None;
    }
    let mut sum = 0.0f64;
    let mut peak = 0.0f64;
    for (x, y) in va.iter().zip(vb.iter()) {
        let d = (*x as f64 - *y as f64).abs();
        sum += d;
        if d > peak {
            peak = d;
        }
    }
    Some((sum / va.len() as f64, peak))
}

#[test]
#[ignore]
fn enable_fused_stages_one_by_one() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    // The steps accumulate: each row is the previous allowlist plus one stage.
    let steps: [(&str, FuseStages); 5] = [
        (
            "ccm",
            FuseStages {
                ccm: true,
                ..FuseStages::NONE
            },
        ),
        (
            "+ ee",
            FuseStages {
                ccm: true,
                ee: true,
                ..FuseStages::NONE
            },
        ),
        (
            "+ ldci",
            FuseStages {
                ccm: true,
                ee: true,
                ldci: true,
                ..FuseStages::NONE
            },
        ),
        (
            "+ display",
            FuseStages {
                ccm: true,
                ee: true,
                ldci: true,
                display: true,
                ..FuseStages::NONE
            },
        ),
        (
            "demosaic_ccm",
            FuseStages {
                demosaic_ccm: true,
                ..FuseStages::NONE
            },
        ),
    ];

    eprintln!(
        "\n{:<16} {:>9} {:>8} {:>8} {:>9} {:>9}  {}",
        "stage added", "infer ms", "mean", "peak", "mean|Δ|", "peak|Δ|", "verdict"
    );

    // Reference: same profile with nothing fused. `opset_mode` is still 2 so the
    // only difference between rows is the allowlist.
    let base = match render(FuseStages::NONE) {
        Ok(f) => f,
        Err(e) => panic!("baseline failed: {e}"),
    };
    let (bm, bp, _) = stats(&base.data);
    eprintln!(
        "{:<16} {:>9.1} {:>8.4} {:>8.3} {:>9} {:>9}  {}",
        "none (ref)", base.infer_ms, bm, bp, "-", "-", "reference"
    );

    let mut reference = base;

    for (label, stages) in steps {
        match render(stages) {
            Ok(f) => {
                let (m, p, nonf) = stats(&f.data);
                match diff(&reference.data, &f.data) {
                    Some((dm, dp)) => {
                        let verdict = if nonf > 0 {
                            "NON-FINITE"
                        } else if p <= 0.2 || m <= 0.05 {
                            "BLANK"
                        } else if dm > 0.02 || dp > 0.25 {
                            "DIFFERS"
                        } else {
                            "ok"
                        };
                        eprintln!(
                            "{:<16} {:>9.1} {:>8.4} {:>8.3} {:>9.4} {:>9.4}  {}",
                            label, f.infer_ms, m, p, dm, dp, verdict
                        );
                        if verdict != "ok" {
                            eprintln!(
                                "\n{label} did not match — stopping here, later rows would be \
                                 compared against a broken baseline."
                            );
                            break;
                        }
                    }
                    None => {
                        eprintln!("{:<16} frame geometry changed", label);
                        break;
                    }
                }
                reference = f;
            }
            Err(e) => {
                eprintln!("{:<16} FAILED: {e}", label);
                break;
            }
        }
    }

    eprintln!(
        "\nDIFFERS is not automatically a bug: a fused op and its primitive form\n\
         are not required to be bit-identical, only close. The threshold is\n\
         mean|Δ| <= 0.02 and peak|Δ| <= 0.25."
    );
}
