//! Reproduce the camera's COMPUTE_SIZE_ERROR: model built at one geometry,
//! frames arriving at another, with the engine's session pool in play.
//!
//! MNN invalidates a session (`Session::setNeedResize`) whenever an input
//! tensor's extent changes, and `Session::run()` then returns
//! COMPUTE_SIZE_ERROR until that session is resized. The pool hands sessions
//! out round-robin, so an engine-wide "last shape" cache means only the first
//! session is ever resized and the rest fail forever.
//!
//! Run:
//!   cd cam-rust
//!   cargo test -p cam-isp --test repro_pool_resize --features mnn \
//!     -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::error::IspError;

/// Build the MED chain at `build_w`x`build_h`, then feed `frames` frames at
/// `run_w`x`run_h`. Returns the index of the first failing frame, if any.
fn run(
    build_w: u32,
    build_h: u32,
    run_w: u32,
    run_h: u32,
    frames: usize,
) -> Result<(), (usize, IspError)> {
    let mut profile = cam_isp::profile::PipelineProfile::MED;
    profile.output_format = OutputFormat::Bgra;

    let mut blocks = profile.build_blocks(build_w, build_h, 0);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(build_h as i64, build_w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| (0, IspError::Config("no mnn_vulkan".into())))?;
    engine.build(head, blocks, None, 21).map_err(|e| (0, e))?;

    let mut buf = vec![0u8; (run_w * run_h * 2) as usize];
    for (i, chunk) in buf.chunks_exact_mut(2).enumerate() {
        let v = ((i % 251) as u16) * 13;
        chunk[0] = (v & 0xFF) as u8;
        chunk[1] = (v >> 8) as u8;
    }

    let params = ProcessParams {
        width: run_w,
        height: run_h,
        stride_width: run_w,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: run_w,
        target_height: run_h,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 0,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::Bgra,
        timestamp_ns: 0,
        isp_params: None,
    };

    for frame in 0..frames {
        if let Err(e) = engine.process(&params) {
            return Err((frame, e));
        }
    }
    Ok(())
}

#[test]
#[ignore]
fn every_pooled_session_survives_a_geometry_change() {
    cam_isp::init();

    // The model is compiled for FHD but frames arrive at the capture
    // geometry — the shape change is what invalidates the sessions.
    let result = run(1920, 1080, 1920, 1440, 8);

    match result {
        Ok(()) => eprintln!("  all 8 frames OK across pooled sessions"),
        Err((frame, e)) => panic!("frame {frame} failed: {e}"),
    }
}
