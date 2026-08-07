//! Pass1: MNN→MNN re-conversion round-trip verification.
//!
//! Takes each block's ONNX graph, converts ONNX→MNN (pass0), then
//! MNN→MNN (pass1) via `convert_mnn_buffer`. Verifies:
//!   (a) pass1 MNN is non-empty and runs on Vulkan
//!   (b) pass1 output matches pass0 output numerically (lossless round-trip)
//!
//! IspChainFusion (which would replace primitive ops with `isp.*` custom
//! ops carrying pre-compiled SPIR-V) is NOT yet implemented — the current
//! pass1 is a graph-optimize-only round-trip.
//!
//! Run:
//! ```sh
//! cd cam-rust
//! LD_LIBRARY_PATH=$PWD/cam-isp/lib/aarch64-v8a \
//!   cargo test --test test_mnn_pass1 -p cam-isp --features mnn \
//!   -- --nocapture --ignored
//! ```

#![cfg(feature = "mnn")]

use cam_isp::mnn_converter::{convert_mnn_buffer, convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
use cam_isp::pipeline::{GraphComposer, IspBlock};

use std::sync::Mutex;

/// MNN's converter keeps a process-global `Global<modelConfig>` singleton;
/// concurrent convert/dump calls from parallel test threads race on it.
static CONVERT_LOCK: Mutex<()> = Mutex::new(());

fn with_convert_lock<T>(f: impl FnOnce() -> T) -> T {
    let _guard = CONVERT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    f()
}

/// Extract op type strings from an mnn2json dump.
fn extract_op_types(json: &str) -> Vec<String> {
    let val: serde_json::Value = serde_json::from_str(json).expect("parse mnn2json");
    val.get("oplists")
        .and_then(|o| o.as_array())
        .map(|ops| {
            ops.iter()
                .map(|op| {
                    let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
                    if ty == "Extra" {
                        op.get("main")
                            .and_then(|m| m.get("type"))
                            .and_then(|t| t.as_str())
                            .unwrap_or("Extra")
                            .to_string()
                    } else {
                        ty.to_string()
                    }
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Run inference with caller-provided input data.
fn infer_mnn_with_input(
    name: &str,
    mnn: &[u8],
    backend: MnnBackendType,
    shape: &[i32],
    input: &[f32],
) -> Vec<f32> {
    let interp = MnnInterpreterSafe::from_buffer(mnn).expect("load mnn");
    let sess = interp.create_session(backend, 4).expect("create session");

    if let Some(t) = interp.get_first_input(&sess) {
        let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), shape);
    }
    let _ = sess.resize();

    let max_out = (input.len() * 4).max(1024);
    let mut out = vec![0.0f32; max_out];

    unsafe {
        let ret = cam_isp::mnn_sys::mnn_run_host_tensors(
            interp.as_ptr(),
            sess.as_ptr(),
            input.as_ptr(),
            shape.as_ptr(),
            shape.len() as i32,
            out.as_mut_ptr(),
            max_out as i32,
        );
        assert!(ret > 0, "[pass1:{}] inference failed (ret={})", name, ret);
    }
    out
}

/// Generate input data for a block. `raw_range` blocks (unpack_blc16,
/// unpack_cfa) need values in [100, 65500] so that after BLC subtraction
/// and ReLU6 the output is non-zero.
fn make_input(shape: &[i32], raw_range: bool) -> Vec<f32> {
    let input_count: usize = shape.iter().map(|&d| d.max(1) as usize).product();
    let mut input = vec![0.0f32; input_count];
    for (i, v) in input.iter_mut().enumerate() {
        if raw_range {
            *v = 100.0 + ((i % 4093) as f32) % 65400.0;
        } else {
            *v = ((i % 251) as f32) / 255.0;
        }
    }
    input
}

/// Compare two float vectors. Returns (MAE, max_abs, PSNR).
fn compare(a: &[f32], b: &[f32]) -> (f32, f32, f32) {
    assert_eq!(a.len(), b.len());
    let n = a.len() as f32;
    let mut sum_abs = 0.0f32;
    let mut max_abs = 0.0f32;
    let mut sum_sq_diff = 0.0f32;
    let mut sum_sq_ref = 0.0f32;
    for i in 0..a.len() {
        let d = (a[i] - b[i]).abs();
        sum_abs += d;
        if d > max_abs {
            max_abs = d;
        }
        sum_sq_diff += d * d;
        sum_sq_ref += a[i] * a[i];
    }
    let mae = sum_abs / n;
    let psnr = if sum_sq_diff == 0.0 {
        f32::INFINITY
    } else {
        10.0 * ((sum_sq_ref / (n * sum_sq_diff)).max(1e-20)).log10()
    };
    (mae, max_abs, psnr)
}

/// Run pass1 round-trip for a single block and verify inference on Vulkan.
/// `raw_range` — if true, generate input in [100,65500] (for BLC blocks).
fn run_pass1_block(
    name: &str,
    chain: Vec<Box<dyn IspBlock>>,
    shape: &[i32],
    raw_range: bool,
) -> Vec<f32> {
    let mut chain = chain;
    GraphComposer::wire_blocks(&mut chain);
    let refs: Vec<&dyn IspBlock> = chain.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    // Pass0: ONNX → MNN.
    let mnn0 = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 onnx→mnn failed");
    let json0 = with_convert_lock(|| dump_mnn_to_json(&mnn0)).expect("pass0 json dump failed");
    let ops0 = extract_op_types(&json0);

    // Pass1: MNN → MNN.
    let mnn1 = with_convert_lock(|| convert_mnn_buffer(&mnn0)).expect("pass1 mnn→mnn failed");
    assert!(!mnn1.is_empty(), "[pass1:{}] pass1 mnn empty", name);
    let json1 = with_convert_lock(|| dump_mnn_to_json(&mnn1)).expect("pass1 json dump failed");
    let ops1 = extract_op_types(&json1);

    println!("[pass1:{}] pass0 ops ({})", name, ops0.len());
    println!("[pass1:{}] pass1 ops ({})", name, ops1.len());

    // Generate input and run inference on Vulkan for both pass0 and pass1.
    let input = make_input(shape, raw_range);
    let out0 = infer_mnn_with_input(name, &mnn0, MnnBackendType::Vulkan, shape, &input);
    let out1 = infer_mnn_with_input(name, &mnn1, MnnBackendType::Vulkan, shape, &input);

    // Numerical comparison.
    let (mae, max_abs, psnr) = compare(&out0, &out1);
    println!(
        "[pass1:{}] n={} MAE={:.6} PSNR={:.2}dB max_abs={:.6}",
        name,
        out1.len(),
        mae,
        psnr,
        max_abs
    );

    // Pass1 output must be finite and non-zero.
    assert!(
        out1.iter().any(|&v| v != 0.0),
        "[pass1:{}] all-zero output",
        name
    );
    for &v in out1.iter() {
        assert!(v.is_finite(), "[pass1:{}] non-finite output: {}", name, v);
    }

    // Round-trip should be numerically lossless.
    assert!(
        mae < 1e-4,
        "[pass1:{}] round-trip MAE too high: {} (max_abs={}, PSNR={}dB)",
        name,
        mae,
        max_abs,
        psnr
    );

    out1
}

/// All 17 blocks: (name, chain, shape, raw_range).
fn core_blocks() -> Vec<(&'static str, Vec<Box<dyn IspBlock>>, Vec<i32>, bool)> {
    use cam_isp::blocks::*;
    vec![
        (
            "unpack_blc16",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_elem_type(5)
                        .with_concrete_dims(16, 16),
                ),
                Box::new(UnpackBlc16Block::new()),
            ],
            vec![1, 1, 16, 16],
            true,
        ),
        (
            "demosaic",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(DemosaicCcmBlock::new(0)),
            ],
            vec![1, 4, 16, 16],
            false,
        ),
        (
            "fcs",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(FcsBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "ee",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(EeBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "ldci",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(LdciBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "display",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(DisplayBlock::new(16)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "unpack_cfa",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_elem_type(6)
                        .with_concrete_width(8)
                        .with_concrete_height(16),
                ),
                Box::new(UnpackCfaBlock::new().with_concrete_width(16).with_blc(true)),
            ],
            vec![1, 1, 16, 8],
            true,
        ),
        (
            "bayer_wb",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(BayerWbBlock::new().with_gains(1.2, 1.0, 1.0, 0.8)),
            ],
            vec![1, 4, 16, 16],
            false,
        ),
        (
            "ccm",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(CcmBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "bilateral",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(BilateralBlock::new_default()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "vignetting",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(VignettingBlock::new(16, 16, 0.5)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "gamma",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(GammaBlock::new(2.2)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "sharpen",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(SharpenBlock::new(0.5)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "saturation",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(SaturationBlock::new(1.2)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "auto_contrast",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(AutoContrastBlock::new(1.2)),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "wavelet_denoise",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(WaveletDenoiseBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
        (
            "tone",
            vec![
                Box::new(
                    RawInputBlock::new()
                        .with_concrete_dims(16, 16)
                        .with_elem_type(1),
                ),
                Box::new(ToneBlock::new()),
            ],
            vec![1, 3, 16, 16],
            false,
        ),
    ]
}

/// Run all 17 blocks through MNN→MNN round-trip and verify
/// (a) inference on Vulkan produces finite non-zero output,
/// (b) pass1 output matches pass0 output numerically (lossless).
#[test]
#[ignore]
fn test_pass1_all_blocks_round_trip() {
    for (name, chain, shape, raw_range) in core_blocks() {
        let out = run_pass1_block(name, chain, &shape, raw_range);
        println!(
            "[pass1:{}] ✓ round-trip ok (out[0..4]={:?})",
            name,
            &out[..4.min(out.len())]
        );
    }
}

/// Pass1 round-trip for the full LITE pipeline.
/// Graph-only verification (ops count/type preserved); multi-output
/// pipeline graphs are not supported by mnn_run_host_tensors.
#[test]
#[ignore]
fn test_pass1_lite_pipeline_round_trip() {
    use cam_isp::profile::PipelineProfile;

    let mut blocks = PipelineProfile::LITE.build_blocks(16, 0);
    blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    GraphComposer::wire_blocks(&mut blocks);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    let mnn0 = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 failed");
    let json0 = with_convert_lock(|| dump_mnn_to_json(&mnn0)).expect("json0 failed");
    let ops0 = extract_op_types(&json0);

    let mnn1 = with_convert_lock(|| convert_mnn_buffer(&mnn0)).expect("pass1 failed");
    assert!(!mnn1.is_empty());
    let json1 = with_convert_lock(|| dump_mnn_to_json(&mnn1)).expect("json1 failed");
    let ops1 = extract_op_types(&json1);

    println!("[pass1:lite] pass0 ops ({}): {:?}", ops0.len(), ops0);
    println!("[pass1:lite] pass1 ops ({}): {:?}", ops1.len(), ops1);

    // Fusion: pass1 should have fewer ops (primitives → ISP custom ops).
    assert!(
        ops1.len() <= ops0.len(),
        "[pass1:lite] pass1 should have ≤ pass0 ops, got pass0={} pass1={}",
        ops0.len(),
        ops1.len()
    );
    // Verify expected ISP ops were fused.
    let has_isp = ops1.iter().any(|o| o.starts_with("isp."));
    assert!(has_isp, "[pass1:lite] no ISP ops fused");
    println!(
        "[pass1:lite] ✓ fusion ok ({} → {} ops)",
        ops0.len(),
        ops1.len()
    );
}

/// Pass1 round-trip for the full HEAVY pipeline.
#[test]
#[ignore]
fn test_pass1_heavy_pipeline_round_trip() {
    use cam_isp::profile::PipelineProfile;

    let mut blocks = PipelineProfile::HEAVY.build_blocks(16, 0);
    blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    GraphComposer::wire_blocks(&mut blocks);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    let mnn0 = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 failed");
    let json0 = with_convert_lock(|| dump_mnn_to_json(&mnn0)).expect("json0 failed");
    let ops0 = extract_op_types(&json0);

    let mnn1 = with_convert_lock(|| convert_mnn_buffer(&mnn0)).expect("pass1 failed");
    assert!(!mnn1.is_empty());
    let json1 = with_convert_lock(|| dump_mnn_to_json(&mnn1)).expect("json1 failed");
    let ops1 = extract_op_types(&json1);

    println!("[pass1:heavy] pass0 ops ({}): {:?}", ops0.len(), ops0);
    println!("[pass1:heavy] pass1 ops ({}): {:?}", ops1.len(), ops1);

    // Fusion: pass1 should have fewer ops (primitives → ISP custom ops).
    assert!(
        ops1.len() <= ops0.len(),
        "[pass1:heavy] pass1 should have ≤ pass0 ops, got pass0={} pass1={}",
        ops0.len(),
        ops1.len()
    );
    // Verify expected ISP ops were fused.
    let has_isp = ops1.iter().any(|o| o.starts_with("isp."));
    assert!(has_isp, "[pass1:heavy] no ISP ops fused");
    println!(
        "[pass1:heavy] ✓ fusion ok ({} → {} ops)",
        ops0.len(),
        ops1.len()
    );
}
