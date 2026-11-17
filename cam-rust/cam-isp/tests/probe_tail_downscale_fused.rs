//! Does the fused tail downscale match the primitive one?
//!
//! `post_ccm_downscale` shrinks the RGB tail: everything after CCM runs at half
//! the demosaiced size, which is where most of the pipeline's memory traffic is.
//! Primitive form is `AdaptiveDownscaleBlock` → an ONNX `Resize`; fused form is
//! one `isp.downscale` dispatch.
//!
//! The shader only implements bilinear + aspect-fit + letterbox, so only that
//! configuration is comparable. `crop` and the EIS margin stay primitive, and
//! `with_aspect_mode` is what keeps them apart.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_tail_downscale_fused --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::{AdaptiveDownscaleBlock, RgbDownscaleBlock};
use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

/// Source size of the RGB tail: half the 2592x1940 sensor.
const W: u32 = 1296;
const H: u32 = 970;
/// Target of a 2x reduction.
const TW: i64 = 648;
const TH: i64 = 485;

fn render(fused: bool) -> Option<(Vec<f32>, f64, usize, usize)> {
    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(
            cam_isp::blocks::RawInputPackedBlock::new()
                .with_concrete_width(W as i64 * 2)
                .with_concrete_height(H as i64 * 2),
        ),
        Box::new(cam_isp::blocks::UnpackBlock::new().with_concrete_width(W as i64 * 2)),
        Box::new(cam_isp::blocks::NormalizeBlock::new()),
        Box::new(cam_isp::blocks::CfaBlockPacked::new()),
        Box::new(cam_isp::blocks::DemosaicBlock::new(2)),
        Box::new(cam_isp::blocks::CcmBlock::with_instance("ccm")),
    ];
    blocks.push(if fused {
        Box::new(
            RgbDownscaleBlock::new(TW, TH)
                .with_concrete_dims(H as i64, W as i64)
                .with_opset_mode(BlockOpsetMode::MnnIspMin),
        )
    } else {
        Box::new(
            AdaptiveDownscaleBlock::new(TW, TH, 0, "edge", "fit")
                .with_concrete_dims(H as i64, W as i64),
        )
    });
    blocks.push(Box::new(
        cam_isp::blocks::DisplayBlock::new(TW as u32).with_output_format(OutputFormat::FloatRgb),
    ));
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let n = blocks.len() + 1;
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    engine.build(head, blocks, None, 21).ok()?;

    let sample: u16 = 600;
    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2 * 4];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: W * 2,
        height: H * 2,
        stride_width: W * 2,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: W * 2,
        target_height: H * 2,
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

    let f = engine.process(&params).ok()?;
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    Some((
        vals,
        f.inference_duration_ns as f64 / 1e6,
        f.width as usize,
        f.height as usize,
    ))
}

#[test]
#[ignore]
fn fused_tail_downscale_matches() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let (prim, prim_ms, pw, ph) = render(false).expect("primitive downscale failed");
    let (fused, fused_ms, fw, fh) = render(true).expect("fused downscale failed");

    eprintln!(
        "primitive {:>7.1} ms  {}x{}  {} values\nfused     {:>7.1} ms  {}x{}  {} values",
        prim_ms,
        pw,
        ph,
        prim.len(),
        fused_ms,
        fw,
        fh,
        fused.len()
    );

    assert_eq!(
        (fw, fh),
        (pw, ph),
        "the two forms disagree about the output geometry"
    );
    assert_eq!(prim.len(), fused.len(), "value counts differ");

    let mean = |v: &[f32]| v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    eprintln!(
        "means: primitive {:.4}  fused {:.4}",
        mean(&prim),
        mean(&fused)
    );

    let mut sum = 0.0f64;
    let mut peak = 0.0f64;
    for (a, b) in prim.iter().zip(fused.iter()) {
        let d = (*a as f64 - *b as f64).abs();
        sum += d;
        if d > peak {
            peak = d;
        }
    }
    let dm = sum / prim.len() as f64;
    eprintln!("difference mean|Δ| {dm:.5}  peak|Δ| {peak:.5}");
    assert!(
        mean(&fused) > 0.05,
        "fused downscale produced a blank frame"
    );
    assert!(
        dm < 0.02 && peak < 0.25,
        "fused downscale differs from Resize (mean|Δ| {dm:.4}, peak|Δ| {peak:.4})"
    );
}
