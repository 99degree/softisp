//! Does a 10-bit sensor render a visible picture?
//!
//! `NormalizeBlock/max_val` used to be hardcoded to 65535. The camera on this
//! device reports a white level of 1023, so a fully exposed pixel normalised to
//! 1023/65535 = 0.0156 and the display — whose Conv carries the [0,1] → [0,255]
//! factor — returned 4/255. Every frame was essentially black while every stage
//! reported success.
//!
//! This renders a mid-grey field through the MED profile at that sensor's white
//! level and asserts the result is actually visible.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_black_frame --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{OutputFormat, ProcessParams};

fn render(sensor_max: f32, sample: u16) -> Result<Vec<u8>, String> {
    let (w, h) = (640u32, 480u32);

    let mut profile = cam_isp::profile::PipelineProfile::MED;
    profile.output_format = OutputFormat::Bgra;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan");
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| e.to_string())?;

    let mut buf = vec![0u8; (w * h * 2) as usize];
    for chunk in buf.chunks_exact_mut(2) {
        chunk[0] = (sample & 0xFF) as u8;
        chunk[1] = (sample >> 8) as u8;
    }

    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max,
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
        output_format: OutputFormat::Bgra,
        timestamp_ns: 0,
        isp_params: None,
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };

    engine
        .process(&params)
        .map(|f| f.data)
        .map_err(|e| e.to_string())
}

fn mean_bgr(bgra: &[u8]) -> (f64, f64, f64) {
    let n = (bgra.len() / 4).max(1) as f64;
    let mut r = 0u64;
    let mut g = 0u64;
    let mut b = 0u64;
    for px in bgra.as_chunks::<4>().0 {
        b += px[0] as u64;
        g += px[1] as u64;
        r += px[2] as u64;
    }
    (r as f64 / n, g as f64 / n, b as f64 / n)
}

#[test]
#[ignore]
fn ten_bit_sensor_is_visible() {
    cam_isp::init();

    // Half-scale on a 10-bit sensor, the level a normal exposure sits near.
    let frame = match render(1023.0, 512) {
        Ok(f) => f,
        Err(e) => panic!("MED render failed: {e}"),
    };
    let (r, g, b) = mean_bgr(&frame);
    let peak = frame
        .as_chunks::<4>()
        .0
        .iter()
        .map(|p| p[0].max(p[1]).max(p[2]))
        .max()
        .unwrap_or(0);
    eprintln!("sensor_max=1023 sample=512 -> mean rgb [{r:.1},{g:.1},{b:.1}] peak {peak}");

    assert!(
        peak > 40,
        "half-scale input produced a peak of {peak}/255 — the frame is black"
    );
    assert!(r > 10.0 || g > 10.0 || b > 10.0, "means are all near zero");

    // A 12-bit sensor must behave the same way: the divisor follows the
    // reported white level, not a constant.
    let frame12 = match render(4095.0, 2048) {
        Ok(f) => f,
        Err(e) => panic!("MED render failed at 12-bit: {e}"),
    };
    let (r12, g12, b12) = mean_bgr(&frame12);
    eprintln!("sensor_max=4095 sample=2048 -> mean rgb [{r12:.1},{g12:.1},{b12:.1}]");
    assert!(
        r12 > 10.0 || g12 > 10.0 || b12 > 10.0,
        "12-bit sensor rendered black: [{r12:.1},{g12:.1},{b12:.1}]"
    );
}
