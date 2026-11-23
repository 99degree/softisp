//! Isolate `isp.ccm`: does it run, and does it see its input?
//!
//! The stepwise sweep shows `+ccm` blanking the frame. Three things could cause
//! that and they need separating, because the fix is different for each:
//!
//!   the op is never dispatched        (global size 0, shader early-returns)
//!   the op runs on an empty buffer    (frame bound to the wrong slot)
//!   the op runs and writes elsewhere  (output bound to the wrong slot)
//!
//! Each variant below changes exactly one of those and prints the frame mean, so
//! a zero can be attributed rather than guessed at.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_ccm_isolated --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::*;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

const W: u32 = 256;
const H: u32 = 192;

fn run(tag: &str, ccm_mode: BlockOpsetMode) {
    run_matrix(tag, ccm_mode, None);
}

fn run_matrix(tag: &str, ccm_mode: BlockOpsetMode, ccm_matrix: Option<[f32; 9]>) {
    let sample: u16 = 600;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(
            RawInputPackedBlock::new()
                .with_concrete_width(W as i64)
                .with_concrete_height(H as i64),
        ),
        Box::new(UnpackBlock::new().with_concrete_width(W as i64)),
        Box::new(NormalizeBlock::new()),
        Box::new(CfaBlockPacked::new()),
        Box::new(DemosaicBlock::new(2)),
        Box::new(CcmBlock::with_instance("ccm").with_opset_mode(ccm_mode)),
        Box::new(DisplayBlock::new(W).with_output_format(OutputFormat::FloatRgb)),
    ];
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = match cam_isp::engine::select_engine_by_name("mnn_vulkan") {
        Some(e) => e,
        None => {
            eprintln!("{tag:<22} no engine");
            return;
        }
    };
    // Tensor storage precision.
    //
    // `isp.*` shaders declare plain `float` SSBOs. With MNN's default
    // `Precision_Normal` on Vulkan every float tensor is stored as fp16, so a
    // shader reading `float params[9]` sees half-width data interpreted as
    // floats. At 256x192 an fp32 session fits comfortably, so this probe can
    // isolate that from everything else.
    if std::env::var("PROBE_FP32").is_ok() {
        if let Some(m) = engine
            .as_any_mut()
            .downcast_mut::<cam_isp::mnnengine::MnnEngine>()
        {
            m.set_fp16_storage(false);
        }
    }
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("{tag:<22} build failed: {e}");
        return;
    }

    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2];
    for c in buf.as_chunks_mut::<2>().0 {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: W,
        target_height: H,
        ccm_matrix,
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

    match engine.process(&params) {
        Err(e) => eprintln!("{tag:<22} process failed: {e}"),
        Ok(_) => match engine.process(&params) {
            Err(e) => eprintln!("{tag:<22} process failed: {e}"),
            Ok(f) => {
                let v: Vec<f32> = f
                    .data
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|b| f32::from_le_bytes(*b))
                    .collect();
                let mean = if v.is_empty() {
                    0.0
                } else {
                    v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64
                };
                let peak = v.iter().cloned().fold(f32::MIN, f32::max);
                let nz = v.iter().filter(|x| **x != 0.0).count();
                eprintln!(
                    "{tag:<22} mean {mean:.4}  peak {peak:.4}  non-zero {nz}/{}",
                    v.len()
                );
            }
        },
    }
}

#[test]
#[ignore]
fn ccm_fused_vs_primitive() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    eprintln!();
    run("ccm primitive", BlockOpsetMode::Primitive);
    run("ccm fused", BlockOpsetMode::MnnIspMin);
    run("ccm custom(pass)", BlockOpsetMode::Custom);
    // Identity matrix: with clamp() an all-zero output can only mean the
    // shader never read its coefficients (or never ran).
    let ident = [1.0f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    run_matrix("ccm fused identity", BlockOpsetMode::MnnIspMin, Some(ident));
    run_matrix("ccm prim identity", BlockOpsetMode::Primitive, Some(ident));
    // All-ones: output = 3*input when the coefficients arrive, 0 when they do
    // not. Distinguishes "params are zero" from "input is zero".
    let ones = [1.0f32; 9];
    run_matrix("ccm fused ones", BlockOpsetMode::MnnIspMin, Some(ones));
}
