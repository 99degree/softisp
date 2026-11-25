//! Does `isp.unpack_blc` compute what its primitive form promises?
//!
//! The fused op replaces `Cast → Div → Sub → Clip → Conv(2x2, stride 2)`:
//! packed INT32 samples in, normalised and BLC-corrected planar quad out.
//!
//! Two things had to be fixed before this could run at all, and both were in
//! the probe rather than in MNN:
//!
//!   * the head block's `ProcessParams` must describe the *packed* frame it is
//!     actually fed (`W` samples per row, `W*2` bytes), not the sensor geometry
//!     the chain would have after unpacking;
//!   * the block needs an input tensor name — it used to default to the empty
//!     string, which no session can resolve.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_unpack_blc_fused --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::{DemosaicBlock, DisplayBlock, NormalizeBlcUnpackCfa};
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

/// Sensor geometry. The packed buffer holds `W * H` samples, two per INT32 word.
const W: u32 = 256;
const H: u32 = 192;
const SENSOR_MAX: f32 = 1023.0;
const BLC: f32 = 0.02;

/// `W * H` samples, two u16 per word, gradient so a mis-indexed quad is visible.
fn packed_input() -> Vec<u8> {
    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2];
    for i in 0..(W as usize) * (H as usize) {
        let v = ((i * 7) % 900 + 50) as u16;
        buf[i * 2..i * 2 + 2].copy_from_slice(&v.to_le_bytes());
    }
    buf
}

/// One raw sample at (x, y), matching `packed_input`.
fn sample(x: usize, y: usize) -> f32 {
    let i = y * W as usize + x;
    (((i * 7) % 900 + 50) as f32 / SENSOR_MAX - BLC).clamp(0.0, 1.0)
}

/// The quad the shader should produce: plane p = quad site p, planar.
fn expected_quad() -> Vec<f32> {
    let qw = (W / 2) as usize;
    let qh = (H / 2) as usize;
    let plane = qw * qh;
    let mut out = vec![0.0f32; plane * 4];
    for qy in 0..qh {
        for qx in 0..qw {
            let idx = qy * qw + qx;
            out[0 * plane + idx] = sample(qx * 2, qy * 2); // TL
            out[1 * plane + idx] = sample(qx * 2 + 1, qy * 2); // TR
            out[2 * plane + idx] = sample(qx * 2, qy * 2 + 1); // BL
            out[3 * plane + idx] = sample(qx * 2 + 1, qy * 2 + 1); // BR
        }
    }
    out
}

#[test]
#[ignore]
fn unpack_blc_computes_the_quad() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let mode = match std::env::var("PROBE_MODE").as_deref() {
        Ok("prim") => BlockOpsetMode::Primitive,
        _ => BlockOpsetMode::MnnIspMin,
    };
    eprintln!("mode = {mode:?}");

    let block = NormalizeBlcUnpackCfa::new()
        .with_concrete_dims(H as i64, W as i64)
        .with_sensor_max(SENSOR_MAX)
        .with_blc_offsets([BLC; 4])
        .with_opset_mode(mode);

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(block),
        // The op emits a 4-plane quad, so the tail has to consume 4 channels:
        // demosaic, then display. Feeding the quad straight to `DisplayBlock`
        // (which expects 3 planes) is a channel mismatch, and MNN reports it by
        // faulting rather than by returning an error.
        Box::new(DemosaicBlock::new(2)),
        Box::new(
            DisplayBlock::new(W)
                .with_output_format(OutputFormat::FloatRgb)
                .with_opset_mode(0),
        ),
    ];
    GraphComposer::wire_blocks(&mut blocks);
    let head = blocks.remove(0);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .expect("mnn_vulkan engine not registered");
    engine.build(head, blocks, None, 21).expect("build");

    let buf = packed_input();
    // The head consumes the packed frame directly: one sample per pixel of a
    // W x H INT32 plane, so the geometry is W x H, not the post-unpack quad.
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &buf,
        sensor_max: SENSOR_MAX,
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

    let f = engine.process(&params).expect("process");

    let got: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let want = expected_quad();
    eprintln!(
        "frame {}x{} -> {} f32 (expected {})",
        f.width,
        f.height,
        got.len(),
        want.len()
    );

    assert_eq!(
        got.len(),
        want.len(),
        "output has {} values, the quad is {}",
        got.len(),
        want.len()
    );

    let plane = got.len() / 4;
    for (i, name) in ["TL", "TR", "BL", "BR"].iter().enumerate() {
        let g: f64 = got[i * plane..(i + 1) * plane]
            .iter()
            .map(|v| *v as f64)
            .sum::<f64>()
            / plane as f64;
        let w: f64 = want[i * plane..(i + 1) * plane]
            .iter()
            .map(|v| *v as f64)
            .sum::<f64>()
            / plane as f64;
        eprintln!("  {name}: got {g:.4}  want {w:.4}");
        assert!(
            (g - w).abs() < 0.01,
            "{name} plane mean is {g:.4}, expected {w:.4}"
        );
    }

    let mut worst = 0.0f64;
    for (a, b) in got.iter().zip(want.iter()) {
        let d = (*a as f64 - *b as f64).abs();
        if d > worst {
            worst = d;
        }
    }
    eprintln!("\nworst per-pixel difference {worst:.5}");
    assert!(
        worst < 0.01,
        "fused unpack_blc differs from the computed quad by {worst:.5}"
    );
}
