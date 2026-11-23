//! Does `isp.unpack16_blc_normal_cfa` turn INT16 RAW into the expected CFA quad?
//!
//! The fused head is run alone and compared against values computed straight from
//! the input buffer, so a failure points at the block rather than at a reference
//! chain. GBRG (`bayer_pattern = 2`) is the pattern under test, and plane `p`
//! holds channel `p`, so the four planes are fed by different sites:
//!
//!   plane 0 = R  = BL = (2qx,   2qy+1)
//!   plane 1 = G1 = BR = (2qx+1, 2qy+1)
//!   plane 2 = G2 = TL = (2qx,   2qy  )
//!   plane 3 = B  = TR = (2qx+1, 2qy  )
//!
//! Each site carries a different mean, so a plane mix-up or a `bayer_pattern`
//! tensor that never reached the shader moves the mean by ~0.04 — well past the
//! 0.01 tolerance. Every value is `sensor_count / sensor_max` with BLC left at
//! zero, so the check is only "did the shader read the sensor plane, normalise
//! it, and put each site in its own plane". It also catches the failure this
//! test was written for: the block declared `extra_input_defaults` but not
//! `extra_inputs`, so `sensor_max`/`blc_vals`/`bayer_pattern` never reached the
//! graph boundary and the shader read unbound memory — the quad came back
//! 100× too bright.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fused_head --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::NormalizeBlcUnpack16Cfa;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

const W: u32 = 512;
const H: u32 = 384;
const SENSOR_MAX: f32 = 1023.0;
const BAYER: i32 = 2; // GBRG

/// 10-bit sensor counts spanning most of the range, offset per 2x2 site so
/// the four planes have four distinguishable means.
fn sample(x: usize, y: usize) -> u16 {
    let site = (y & 1) * 2 + (x & 1); // 0=TL 1=TR 2=BL 3=BR
    (((y * W as usize + x) * 7) % 800 + 50 + site * 40) as u16
}

fn raw_frame(w: u32, h: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity(w as usize * h as usize * 2);
    for y in 0..h as usize {
        for x in 0..w as usize {
            buf.extend_from_slice(&sample(x, y).to_le_bytes());
        }
    }
    buf
}

fn floats(data: &[u8]) -> Vec<f32> {
    data.as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect()
}

fn params(buf: &[u8]) -> ProcessParams<'_> {
    ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf,
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
    }
}

/// Expected plane means, in plane order, for the GBRG pattern under test.
fn expected_means() -> [f64; 4] {
    let qw = W as usize / 2;
    let qh = H as usize / 2;
    let mut sum = [0.0f64; 4];
    for qy in 0..qh {
        for qx in 0..qw {
            sum[0] += sample(2 * qx, 2 * qy + 1) as f64; // BL → R
            sum[1] += sample(2 * qx + 1, 2 * qy + 1) as f64; // BR → G1
            sum[2] += sample(2 * qx, 2 * qy) as f64; // TL → G2
            sum[3] += sample(2 * qx + 1, 2 * qy) as f64; // TR → B
        }
    }
    let n = (qw * qh) as f64;
    [
        sum[0] / n / SENSOR_MAX as f64,
        sum[1] / n / SENSOR_MAX as f64,
        sum[2] / n / SENSOR_MAX as f64,
        sum[3] / n / SENSOR_MAX as f64,
    ]
}

fn plane_means(v: &[f32]) -> [f64; 4] {
    let plane = v.len() / 4;
    let mut out = [0.0; 4];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = v[i * plane..(i + 1) * plane]
            .iter()
            .map(|x| *x as f64)
            .sum::<f64>()
            / plane as f64;
    }
    out
}

#[test]
#[ignore]
fn fused_head_int16_produces_expected_quad() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let buf = raw_frame(W, H);
    let want = expected_means();

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![Box::new(
        NormalizeBlcUnpack16Cfa::new()
            .with_concrete_dims(H as i64, W as i64)
            .with_sensor_max(SENSOR_MAX)
            .with_bayer_pattern(BAYER)
            .with_opset_mode(BlockOpsetMode::MnnIspMin),
    )];
    GraphComposer::wire_blocks(&mut blocks);
    let head = blocks.remove(0);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan").expect("mnn_vulkan");
    if let Some(mnn) = engine
        .as_any_mut()
        .downcast_mut::<cam_isp::mnnengine::MnnEngine>()
    {
        mnn.set_preserve_input_type(true);
    }
    engine.build(head, blocks, None, 21).expect("build");

    // First frame allocates the session and resolves shapes; measure the third.
    let mut got = Vec::new();
    for _ in 0..3 {
        got = floats(&engine.process(&params(&buf)).expect("process").data);
    }

    let have = plane_means(&got);
    let plane = got.len() / 4;
    for (i, name) in ["TL", "TR", "BL", "BR"].iter().enumerate() {
        let p = &got[i * plane..(i + 1) * plane];
        let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
        let mut nan = 0usize;
        for &v in p {
            if v.is_nan() {
                nan += 1;
                continue;
            }
            lo = lo.min(v);
            hi = hi.max(v);
        }
        eprintln!(
            "{name}: min {lo:.5} max {hi:.5} nan {nan} head {:?}",
            &p[..8.min(p.len())]
        );
        eprintln!(
            "{name} bits: {:?}",
            p[..8.min(p.len())]
                .iter()
                .map(|v| format!("{:08x}", v.to_bits()))
                .collect::<Vec<_>>()
        );
    }
    eprintln!(
        "{:<9} n={:<7} TL {:.5}  TR {:.5}  BL {:.5}  BR {:.5}",
        "expected",
        4 * (W / 2) as usize * (H / 2) as usize,
        want[0],
        want[1],
        want[2],
        want[3]
    );
    eprintln!(
        "{:<9} n={:<7} TL {:.5}  TR {:.5}  BL {:.5}  BR {:.5}",
        "fused",
        got.len(),
        have[0],
        have[1],
        have[2],
        have[3]
    );

    assert_eq!(
        got.len(),
        (W as usize / 2) * (H as usize / 2) * 4,
        "quad geometry: expected [1,4,{},{}/2], got {} floats",
        H / 2,
        W,
        got.len()
    );
    for (i, name) in ["TL", "TR", "BL", "BR"].iter().enumerate() {
        assert!(
            (have[i] - want[i]).abs() < 0.01,
            "{name} plane mean is {:.5}, expected {:.5}",
            have[i],
            want[i]
        );
    }
}
