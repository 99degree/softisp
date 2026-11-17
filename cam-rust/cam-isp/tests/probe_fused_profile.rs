//! Does the `FUSED` profile — the one the app runs — build and render?
//!
//! `FUSED` is MED's stage set with every fusion-capable stage emitting a single
//! `isp.*` dispatch. Each substitution was verified individually; this checks the
//! combination, which is a different question: the ops now share one graph, one
//! session pool and one precision decision.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fused_profile --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 1296;
const H: u32 = 970;

struct Outcome {
    infer_ms: f64,
    mean: f64,
    peak: f64,
    nonfinite: usize,
    blocks: usize,
}

fn render(profile: PipelineProfile) -> Result<Outcome, String> {
    let mut profile = profile;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let n = blocks.len() + 1;
    let aux = profile.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| "mnn_vulkan engine not registered".to_string())?;
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

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

    engine.process(&params).map_err(|e| format!("first: {e}"))?;
    let f = engine
        .process(&params)
        .map_err(|e| format!("second: {e}"))?;

    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return Err("empty frame".into());
    }
    Ok(Outcome {
        infer_ms: f.inference_duration_ns as f64 / 1e6,
        mean: vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64,
        peak: vals.iter().cloned().fold(f32::MIN, f32::max) as f64,
        nonfinite: vals.iter().filter(|v| !v.is_finite()).count(),
        blocks: n,
    })
}

#[test]
#[ignore]
fn fused_profile_renders() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .is_test(false)
        .try_init();
    cam_isp::init();

    // A MED profile with the same allowlist, for a like-for-like comparison.
    let mut med_all = PipelineProfile::MED;
    med_all.opset_mode = 2;
    med_all.fuse_stages = FuseStages::ALL;

    eprintln!(
        "\n{:<12} {:>8} {:>7} {:>8} {:>8} {:>10}  {}",
        "profile", "blocks", "ms", "mean", "peak", "non-finite", "state"
    );

    for (label, profile) in [
        ("MED (prim)", PipelineProfile::MED),
        ("FUSED", PipelineProfile::FUSED),
        ("MED+ALL", med_all),
    ] {
        match render(profile) {
            Ok(o) => {
                let state = if o.nonfinite > 0 {
                    "NON-FINITE"
                } else if o.mean <= 0.05 || o.peak <= 0.2 {
                    "BLANK"
                } else {
                    "picture"
                };
                eprintln!(
                    "{label:<12} {:>8} {:>7.1} {:>8.4} {:>8.3} {:>10}  {state}",
                    o.blocks, o.infer_ms, o.mean, o.peak, o.nonfinite
                );
                assert_eq!(state, "picture", "{label} did not render a picture");
            }
            Err(e) => {
                eprintln!("{label:<12} FAILED: {e}");
                panic!("{label} failed to run");
            }
        }
    }

    // FUSED and MED+ALL differ only in how the profile is spelled; if the
    // allowlist plumbing works they must build the same chain.
    eprintln!("\nFUSED is MED's stage set with every verified fused stage enabled.");
}
