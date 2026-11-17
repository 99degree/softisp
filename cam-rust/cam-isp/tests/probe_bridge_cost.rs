//! What does removing the identity bridges cost — or save?
//!
//! `build_blocks` ends with `wire_blocks_with_identities`, which inserts a
//! `Transpose(perm=[0,1,2,3])` between every adjacent pair of blocks. Across the
//! FUSED chain that is 19 nodes — 58% of its 33 ONNX nodes — and each one is a
//! full-frame read + write that computes nothing.
//!
//! Plain `wire_blocks` does the same wiring without them. This measures both, so
//! the decision is a number rather than an argument about debugging ergonomics.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_bridge_cost --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 1296;
const H: u32 = 970;

fn render(bridges: bool) -> Result<(f64, f64, f64, usize), String> {
    let profile = PipelineProfile::FUSED;

    let mut blocks = profile.build_blocks(W, H, 2);
    if !bridges {
        // `build_blocks` already expanded the list; rebuild it without the
        // expansion rather than trying to filter the bridges back out, because
        // the singleton identity blocks a profile inserts for disabled stages
        // are indistinguishable by id.
        blocks = profile.build_blocks_no_bridges(W, H, 2);
    }
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
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
    let peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    Ok((f.inference_duration_ns as f64 / 1e6, mean, peak, n))
}

#[test]
#[ignore]
fn bridge_cost() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!(
        "\n{:<18} {:>7} {:>8} {:>8}  {}",
        "chain", "blocks", "ms", "mean", "state"
    );
    for (label, bridges) in [("with bridges", true), ("no bridges", false)] {
        match render(bridges) {
            Ok((ms, mean, peak, n)) => eprintln!(
                "{label:<18} {n:>7} {ms:>8.1} {mean:>8.4}  {}",
                if mean > 0.05 && peak > 0.2 {
                    "picture"
                } else {
                    "BLANK"
                }
            ),
            Err(e) => eprintln!("{label:<18} FAILED: {e}"),
        }
    }
}

/// Kept out of the profile so the comparison in this file is the only consumer;
/// if it proves out, `build_blocks` can take the bridgeless path directly.
#[allow(dead_code)]
fn unused() {
    let _ = GraphComposer::wire_blocks;
}
