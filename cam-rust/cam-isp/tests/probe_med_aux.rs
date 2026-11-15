//! Is the cost in the image chain or in the statistics/aux blocks?
//!
//! The MED profile is 132 ONNX ops and takes ~390 ms to infer at 1296x970,
//! while a hand-built 11-block chain takes ~142 ms. The difference is too large
//! to be eleven more pixel ops, so this separates the two halves: the RGB chain
//! that walks every pixel, and the aux blocks (`build_aux_blocks`) that feed the
//! 3A controller.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_med_aux --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use std::time::Instant;

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::IspBlock;
use cam_isp::profile::PipelineProfile;

fn measure(tag: &str, w: u32, h: u32, with_aux: bool) {
    let sample: u16 = 512;

    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    // The RGB tail is where the memory traffic is. Halving it after CCM should
    // cut the bandwidth-bound work by ~4x for every stage past demosaic.
    if std::env::var("PROBE_DOWNSCALE").is_ok() {
        profile.post_ccm_downscale = (w / 4, h / 4);
    }

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let chain_ops = blocks.len();
    let aux_ops = if with_aux {
        let aux = profile.build_aux_blocks(h as i64, w as i64);
        let n = aux.len();
        blocks.extend(aux);
        n
    } else {
        0
    };

    let mut engine = match cam_isp::engine::select_engine_by_name("mnn_vulkan") {
        Some(e) => e,
        None => return,
    };
    if engine.build(head, blocks, None, 21).is_err() {
        eprintln!("{tag:<34} BUILD FAIL");
        return;
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

    if engine.process(&params).is_err() {
        eprintln!("{tag:<34} RUN FAIL");
        return;
    }

    let n = 3;
    let t0 = Instant::now();
    let mut infer = 0.0;
    for _ in 0..n {
        if let Ok(f) = engine.process(&params) {
            infer = f.inference_duration_ns as f64 / 1e6;
        }
    }
    let per = t0.elapsed().as_secs_f64() * 1000.0 / n as f64;
    eprintln!(
        "{tag:<34} chain {chain_ops:>3} + aux {aux_ops:>2} = {:>3} blocks   infer {infer:7.1} ms",
        chain_ops + aux_ops
    );
}

#[test]
#[ignore]
fn med_chain_or_aux() {
    cam_isp::init();

    let (w, h) = (2592u32, 1940u32);

    measure("MED, no aux   @2592x1940", w, h, false);
    measure("MED, +aux     @2592x1940", w, h, true);
}
