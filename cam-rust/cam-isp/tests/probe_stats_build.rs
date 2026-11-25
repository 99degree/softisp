//! Does any profile with `stats_downscale_max > 0` actually build?
//!
//! `HEAVY` sets it to 540. The stats path inserts an `AdaptiveDownscaleBlock` into
//! the *aux* branches, and MNN rejects the resulting graph ("AdaptiveDownscaleBlock/
//! frame has empty input, the index is 1") — an empty string in the Resize node's
//! `roi`/`scales` slots, which the ONNX importer turns into a tensors-less input.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_stats_build --features mnn -- --nocapture

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

fn try_build(label: &str, mut p: PipelineProfile, stats_max: u32) -> bool {
    p.output_format = OutputFormat::FloatRgb;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("{label:<22} build failed: {e}");
        return false;
    }
    let raw = cam_isp::synth_bayer::bayer_color_diffuse(W, H);
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &raw,
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
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };
    match engine.process(&params) {
        Ok(f) => {
            eprintln!(
                "{label:<22} ok  {:.0} ms  {}x{}",
                f.inference_duration_ns as f64 / 1e6,
                f.width,
                f.height
            );
            true
        }
        Err(e) => {
            eprintln!("{label:<22} process failed: {e}");
            false
        }
    }
}

#[test]
fn stats_downscale_builds() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!();
    let full = try_build("FUSED (stats full)", PipelineProfile::FUSED, 0);
    let ds = try_build("FUSED (stats 540)", PipelineProfile::FUSED, 540);
    let heavy = try_build("HEAVY (ships 540)", PipelineProfile::HEAVY, 540);

    assert!(full, "the full-resolution stats chain must build");
    assert!(
        ds,
        "stats_downscale_max > 0 is offered by the profile API and used by HEAVY, \
         so it has to build"
    );
    assert!(heavy, "HEAVY sets stats_downscale_max and must render");
}
