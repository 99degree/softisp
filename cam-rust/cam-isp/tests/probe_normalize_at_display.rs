//! Does deferring the normalise divide change the picture?
//!
//! `PipelineProfile::normalize_at_display` removes `NormalizeBlock` (a Cast +
//! Div by `sensor_max`) from the head of the chain, leaving the graph in raw
//! sensor counts, and instead scales the display Conv weights and the clamp
//! bounds by the same factor. The two are algebraically equivalent:
//!
//!   normalise:  out = 255 * (x / smax)
//!   deferred:   out = (255 / smax) * x
//!
//! This checks that equivalence against real inference, because the deferred
//! form also has to keep every intermediate clamp honest — a `Clip(x, 0, 1)`
//! left at 1.0 would flatten the whole image to a single count.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_normalize_at_display --features mnn \
//!     -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};

fn render(defer: bool) -> Result<Vec<u8>, String> {
    let (w, h) = (640u32, 480u32);
    let smax = 1023.0f32;

    let mut profile = cam_isp::profile::PipelineProfile::MED;
    profile.output_format = OutputFormat::Bgra;
    profile.normalize_at_display = defer;

    let mut blocks = profile.build_blocks(w, h, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(h as i64, w as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::new(MnnBackend::Vulkan);
    engine.set_normalizes_at_input(!defer);
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| e.to_string())?;

    // A gradient rather than a flat field: a wrong clamp would be invisible on
    // a constant.
    let mut buf = vec![0u8; (w * h * 2) as usize];
    for (i, chunk) in buf.as_chunks_mut::<2>().0.iter_mut().enumerate() {
        let v = ((i % w as usize) * 1023 / w as usize) as u16;
        chunk[0] = (v & 0xFF) as u8;
        chunk[1] = (v >> 8) as u8;
    }

    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max: smax,
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

#[test]
#[ignore]
fn deferred_normalize_matches_normalized() {
    cam_isp::init();

    let a = match render(false) {
        Ok(v) => v,
        Err(e) => panic!("normalized build failed: {e}"),
    };
    let b = match render(true) {
        Ok(v) => v,
        Err(e) => panic!("deferred build failed: {e}"),
    };

    eprintln!("normalized: {} bytes, deferred: {} bytes", a.len(), b.len());
    assert_eq!(a.len(), b.len(), "frame sizes differ");

    let max_diff = a
        .iter()
        .zip(b.iter())
        .map(|(x, y)| (*x as i32 - *y as i32).abs())
        .max()
        .unwrap_or(0);
    let mean_a: f64 = a.iter().map(|v| *v as f64).sum::<f64>() / a.len() as f64;
    let mean_b: f64 = b.iter().map(|v| *v as f64).sum::<f64>() / b.len() as f64;

    eprintln!("mean: normalized {mean_a:.2}, deferred {mean_b:.2}, max |diff| {max_diff}");

    assert!(mean_a > 1.0, "normalized frame is black ({mean_a:.2})");
    assert!(mean_b > 1.0, "deferred frame is black ({mean_b:.2})");
    assert!(
        max_diff <= 3,
        "deferred output differs by {max_diff} levels (mean {mean_a:.2} vs {mean_b:.2})"
    );
}
