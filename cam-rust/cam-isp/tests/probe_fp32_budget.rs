//! How much of an fp32 pipeline actually fits on this device?
//!
//! Fusion is correct but needs 32-bit tensor storage (`isp.*` shaders declare
//! `float[]` buffers, and MNN allocates them at two bytes when fp16 storage is
//! on). fp16 is what makes a four-session pool fit at 2592x1940, so the question
//! is whether an fp32 pipeline can run at all — and with how many sessions.
//!
//! `SessionPool` already halves its session count on allocation failure and
//! retries down to one, so this measures whether that degradation reaches a
//! working configuration instead of failing outright.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fp32_budget --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::MnnEngine;
use cam_isp::profile::PipelineProfile;

fn try_pool(pool: usize, w: u32, h: u32) -> Result<String, String> {
    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(cam_isp::mnnengine::MnnBackend::Vulkan, pool);
    engine.set_fp16_storage(false);
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = 0x58;
        c[1] = 0x02;
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
    let f = engine
        .process(&params)
        .map_err(|e| format!("process: {e}"))?;
    Ok(format!("ok, {} f32 out", f.data.len() / 4))
}

#[test]
#[ignore]
fn fp32_pipeline_memory_budget() {
    cam_isp::init();

    for (w, h) in [(2592u32, 1940u32), (1296, 970)] {
        eprintln!("\n== {w}x{h} ==");
        for pool in [4usize, 2, 1] {
            match try_pool(pool, w, h) {
                Ok(msg) => eprintln!("  pool={pool}: {msg}"),
                Err(e) => eprintln!("  pool={pool}: FAILED - {e}"),
            }
        }
    }
}
