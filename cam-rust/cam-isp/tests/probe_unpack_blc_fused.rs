//! Does `isp.unpack_blc` compute what its primitive form promises?
//!
//! The fused op replaces `Cast → Div → Sub → Clip → Conv(2x2, stride 2)`:
//! packed INT32 samples in, normalised and BLC-corrected planar quad out.
//!
//! The comparison is against values computed here on the CPU, not against
//! `NormalizeBlcUnpackCfa`'s primitive form. That form is not usable standalone
//! — it runs `Conv(stride 2)` over an already-packed `[1,1,H,W/2]` tensor, so its
//! output width is half of what it declares — and the first version of this
//! probe crashed inside MNN's session creation trying to build it.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_unpack_blc_fused --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::NormalizeBlcUnpackCfa;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

const W: u32 = 256;
const H: u32 = 192;
const SENSOR_MAX: f32 = 1023.0;
const BLC: f32 = 0.02;

/// Two u16 samples per INT32 word, gradient so a mis-indexed quad is visible.
fn packed_input() -> Vec<u8> {
    let words_per_row = (W / 2) as usize;
    let mut buf = vec![0u8; words_per_row * H as usize * 4];
    for y in 0..H as usize {
        for k in 0..words_per_row {
            let lo = ((y * 7 + k * 2) % 900 + 50) as u16;
            let hi = ((y * 7 + k * 2 + 1) % 900 + 50) as u16;
            let packed = (hi as u32) << 16 | lo as u32;
            let i = (y * words_per_row + k) * 4;
            buf[i..i + 4].copy_from_slice(&packed.to_le_bytes());
        }
    }
    buf
}

/// What the shader should produce, plane by plane, computed on the CPU.
fn expected_quad() -> Vec<f32> {
    let words_per_row = (W / 2) as usize;
    let qw = words_per_row;
    let qh = H as usize / 2;
    let plane = qw * qh;
    let mut out = vec![0.0f32; plane * 4];

    let norm = |v: u16| ((v as f32) / SENSOR_MAX - BLC).clamp(0.0, 1.0);

    for qy in 0..qh {
        for qx in 0..qw {
            let lo = ((qy * 2 * 7 + qx * 2) % 900 + 50) as u16;
            let hi = ((qy * 2 * 7 + qx * 2 + 1) % 900 + 50) as u16;
            let lo2 = (((qy * 2 + 1) * 7 + qx * 2) % 900 + 50) as u16;
            let hi2 = (((qy * 2 + 1) * 7 + qx * 2 + 1) % 900 + 50) as u16;

            let idx = qy * qw + qx;
            out[0 * plane + idx] = norm(lo); // TL
            out[1 * plane + idx] = norm(hi); // TR
            out[2 * plane + idx] = norm(lo2); // BL
            out[3 * plane + idx] = norm(hi2); // BR
        }
    }
    out
}

#[test]
#[ignore]
fn unpack_blc_computes_the_quad() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let block = NormalizeBlcUnpackCfa::new()
        .with_concrete_dims(H as i64, W as i64)
        .with_sensor_max(SENSOR_MAX)
        .with_blc_offsets([BLC; 4])
        .with_opset_mode(BlockOpsetMode::MnnIspMin);

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![Box::new(block)];
    GraphComposer::wire_blocks(&mut blocks);
    let head = blocks.remove(0);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .expect("mnn_vulkan engine not registered");
    engine.build(head, blocks, None, 21).expect("build");

    let buf = packed_input();
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
        "output has {} values, the shader declares {}",
        got.len(),
        want.len()
    );

    // Per-plane means first: a swapped plane order shows up here immediately.
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
