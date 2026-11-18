//! Does the run-time statistics switch work, and does a frame say so?
//!
//! Run:
//!   cargo test -p cam-isp --test probe_stats_switch --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

fn build() -> Option<MnnEngine> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    p.opset_mode = 2;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;
    Some(engine)
}

fn params<'a>(raw: &'a [u8]) -> ProcessParams<'a> {
    ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: raw,
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
    }
}

#[test]
#[ignore]
fn stats_switch() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let raw = cam_isp::synth_bayer::bayer_color_diffuse(W, H);
    let mut engine = build().expect("build");

    // On: the first frame measures.
    assert!(engine.stats_enabled(), "statistics default to measured");
    let f = engine.process(&params(&raw)).expect("on");
    eprintln!(
        "on  -> measured={:?}",
        f.aux.as_ref().map(|a| a.stats_measured)
    );
    assert_eq!(
        f.aux.as_ref().map(|a| a.stats_measured),
        Some(true),
        "a frame with the switch on must report that it measured"
    );

    // Off: the same session, no rebuild, and the frame says nothing was fetched.
    engine.set_stats_enabled(false);
    let mut off_measured = 0;
    let mut off_frames = 0;
    for _ in 0..4 {
        let f = engine.process(&params(&raw)).expect("off");
        off_frames += 1;
        if f.aux.as_ref().map(|a| a.stats_measured) == Some(true) {
            off_measured += 1;
        }
    }
    eprintln!("off -> {off_measured}/{off_frames} frames reported measuring");
    assert_eq!(off_measured, 0, "the switch off must skip every readback");

    // Back on, same session.
    engine.set_stats_enabled(true);
    let f = engine.process(&params(&raw)).expect("on again");
    eprintln!(
        "on again -> measured={:?}",
        f.aux.as_ref().map(|a| a.stats_measured)
    );
    assert_eq!(
        f.aux.as_ref().map(|a| a.stats_measured),
        Some(true),
        "re-enabling must take effect without a rebuild"
    );
}
