//! A/B comparison test: Pass0 (reference, primitive ops) vs Pass1
//! (`isp.*` custom opset on Vulkan), per ISP block.
//!
//! For each block:
//!   1. Pass0: ONNX → MNN → inference on the CPU/primitive path. This is
//!      the reference base.
//!   2. Pass1: MNN → MNN (IspChainFusion) → Vulkan inference with the
//!      `isp.*` SPIR-V custom ops.
//!   3. Compare outputs with MAE (< 1e-2), PSNR (> 40 dB), and NaN/Inf
//!      rejection. The two runs must be numerically close — the fused
//!      `isp.*` shaders must reproduce the reference math.
//!
//! Run:
//! ```sh
//! cd cam-rust
//! LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!   cargo test --test test_mnn_ab_comparison -p cam-isp --features mnn \
//!   -- --nocapture --ignored
//! ```

#![cfg(feature = "mnn")]

use cam_isp::mnn_converter::{convert_mnn_buffer, convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
use cam_isp::pipeline::{GraphComposer, IspBlock};

use std::ffi::CString;
use std::os::raw::c_void;
use std::sync::Mutex;

/// MNN's converter keeps a process-global `Global<modelConfig>` singleton;
/// concurrent convert/dump calls from parallel test threads race on it.
static CONVERT_LOCK: Mutex<()> = Mutex::new(());

fn with_convert_lock<T>(f: impl FnOnce() -> T) -> T {
    let _guard = CONVERT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    f()
}

/// Run inference on an MNN buffer with the given backend, return output.
fn infer_mnn(
    mnn: &[u8],
    backend: MnnBackendType,
    shape: &[i32],
    dtype_code: i32,
    type_bits: i32,
    out_name: &str,
) -> Vec<f32> {
    let interp = MnnInterpreterSafe::from_buffer(mnn).expect("load mnn");
    let sess = interp.create_session(backend, 4).expect("create session");

    if let Some(t) = interp.get_first_input(&sess) {
        let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), shape);
    }
    let _ = sess.resize();

    let input_count: usize = shape.iter().map(|&d| d.max(1) as usize).product();
    let mut input = vec![0.0f32; input_count];
    for (i, v) in input.iter_mut().enumerate() {
        *v = ((i % 251) as f32) / 255.0;
    }
    let input_ptr: *const c_void;
    let mut input_ints;
    let mut input_i16;
    if dtype_code == 0 {
        input_ints = vec![0i32; input_count];
        for (i, v) in input_ints.iter_mut().enumerate() {
            *v = ((i % 4093) as i32) % 2048;
        }
        input_ptr = input_ints.as_ptr() as *const c_void;
    } else if dtype_code == 5 {
        // INT16 input (unpack_blc16 block)
        input_i16 = vec![0i16; input_count];
        for (i, v) in input_i16.iter_mut().enumerate() {
            *v = ((i % 4093) as i16) % 2048;
        }
        input_ptr = input_i16.as_ptr() as *const c_void;
    } else {
        input_ptr = input.as_ptr() as *const c_void;
    }

    let out_name_c = CString::new(out_name).expect("cstring");
    let max_out = (input_count * 4).max(1024);
    let mut out = vec![0.0f32; max_out];

    unsafe {
        let ret = cam_isp::mnn_sys::mnn_run_with_output(
            interp.as_ptr(),
            sess.as_ptr(),
            input_ptr,
            dtype_code,
            type_bits,
            shape.as_ptr(),
            shape.len() as i32,
            out_name_c.as_ptr(),
            out.as_mut_ptr(),
            max_out as i32,
        );
        assert_eq!(ret, 0, "inference failed (ret={})", ret);
    }
    out
}

/// Parse mnn2json into op types (used to confirm Pass1 is isp.*-only).
fn extract_op_types(json: &str) -> Vec<String> {
    let val: serde_json::Value = serde_json::from_str(json).expect("parse json");
    let oplists = val
        .get("oplists")
        .and_then(|o| o.as_array())
        .cloned()
        .unwrap_or_default();
    let mut types = Vec::new();
    for op in oplists.iter() {
        let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
        if ty == "Extra" {
            let extra = op
                .get("main")
                .and_then(|m| m.get("type"))
                .and_then(|t| t.as_str())
                .unwrap_or("Extra");
            types.push(extra.to_string());
        } else {
            types.push(ty.to_string());
        }
    }
    types
}

/// Numeric comparison: MAE + PSNR + NaN/Inf rejection.
fn compare_outputs(name: &str, a: &[f32], b: &[f32]) {
    assert_eq!(a.len(), b.len(), "[ab:{}] length mismatch", name);

    let n = a.len();
    let mut mae = 0.0f64;
    let mut mse = 0.0f64;
    let mut max_abs = 0.0f64;
    for i in 0..n {
        let da = a[i] as f64;
        let db = b[i] as f64;
        assert!(
            da.is_finite() && db.is_finite(),
            "[ab:{}] NaN/Inf at {}",
            name,
            i
        );
        let diff = (da - db).abs();
        mae += diff;
        mse += diff * diff;
        max_abs = max_abs.max(diff);
    }
    mae /= n as f64;
    mse /= n as f64;
    let psnr = if mse > 0.0 {
        10.0 * (1.0f64 / mse).log10()
    } else {
        f64::INFINITY
    };

    println!(
        "[ab:{}] n={} MAE={:.6} PSNR={:.2}dB max_abs={:.6}",
        name, n, mae, psnr, max_abs
    );

    assert!(mae < 1e-2, "[ab:{}] MAE {:.6} >= 1e-2", name, mae);
    assert!(psnr > 40.0, "[ab:{}] PSNR {:.2}dB <= 40dB", name, psnr);
}

/// Full A/B flow for one block chain.
fn run_ab(
    name: &str,
    chain: Vec<Box<dyn IspBlock>>,
    shape: &[i32],
    dtype_code: i32,
    type_bits: i32,
) {
    let mut chain = chain;
    GraphComposer::wire_blocks(&mut chain);
    let refs: Vec<&dyn IspBlock> = chain.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    // Pass0: ONNX → MNN (reference, primitive ops).
    let mnn0 = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 convert failed");
    let json0 = with_convert_lock(|| dump_mnn_to_json(&mnn0)).expect("pass0 json");
    println!(
        "[ab:{}] pass0 ops: {}",
        name,
        extract_op_types(&json0).join(",")
    );

    // Pass1: MNN → MNN (IspChainFusion) — must be isp.*-only.
    let mnn1 = with_convert_lock(|| convert_mnn_buffer(&mnn0)).expect("pass1 convert failed");
    let json1 = with_convert_lock(|| dump_mnn_to_json(&mnn1)).expect("pass1 json");
    let types1 = extract_op_types(&json1);
    let non_isp: Vec<&String> = types1
        .iter()
        .filter(|t| !t.starts_with("isp.") && t.as_str() != "Input" && t.as_str() != "Const")
        .collect();
    assert!(
        non_isp.is_empty(),
        "[ab:{}] pass1 non-isp ops: {:?}",
        name,
        non_isp
    );
    println!("[ab:{}] pass1 ops: {}", name, types1.join(","));

    let out_name = chain
        .last()
        .expect("chain tail")
        .graph_output_name()
        .expect("out name");

    // Reference: Pass0 on CPU path (exact float32 reference).
    let ref_out = infer_mnn(
        &mnn0,
        MnnBackendType::Cpu,
        shape,
        dtype_code,
        type_bits,
        out_name,
    );
    // Candidate: Pass1 on Vulkan with isp.* SPIR-V.
    let vk_out = infer_mnn(
        &mnn1,
        MnnBackendType::Vulkan,
        shape,
        dtype_code,
        type_bits,
        out_name,
    );

    compare_outputs(name, &ref_out, &vk_out);
}

fn core_blocks() -> Vec<(&'static str, Vec<Box<dyn IspBlock>>, Vec<i32>, i32, i32)> {
    use cam_isp::blocks::*;
    vec![
        // Chains start with RawInputBlock (production topology): a head block
        // emitting nodes needs input != output tensor names, else the graph is
        // `x = f(x)` and MNN DCEs it to a bare Input op.
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
            5,
            16,
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
            2,
            32,
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
            2,
            32,
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
            2,
            32,
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
            2,
            32,
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
            2,
            32,
        ),
    ]
}

#[test]
#[ignore]
fn test_ab_core_blocks_pass0_vs_pass1() {
    for (name, chain, shape, dtype, bits) in core_blocks() {
        run_ab(name, chain, &shape, dtype, bits);
    }
}
