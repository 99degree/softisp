//! Pass1 (B test): MNN→MNN re-conversion + `isp.*`-only opset verification.
//!
//! Takes each Pass0 `.mnn` (produced by `test_mnn_pass0.rs` or regenerated
//! here from the ONNX graph) and re-runs it through the MNN converter
//! in-process with framework=MNN (`convert_mnn_buffer`). This triggers the
//! IspChainFusion post-converter: primitive ops (Conv/BinaryOp/Pool/...) are
//! fused into `isp.*` custom ops carrying pre-compiled SPIR-V.
//!
//! B-test constraint (hard requirement):
//!   The Pass1 MNN must contain ONLY `isp.*` custom opset operations —
//!   ZERO MNN primitive ops. Verified via the built-in mnn2json dump
//!   (`dump_mnn_to_json`), parsed into an abstract node graph in Rust.
//!
//! The resulting Pass1 MNN is then loaded on the Vulkan backend and runs
//! one inference; the output is compared numerically against Pass0 in
//! `test_mnn_ab_comparison.rs`.
//!
//! Run:
//! ```sh
//! cd cam-rust
//! LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!   cargo test --test test_mnn_pass1 -p cam-isp --features mnn \
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

/// Abstract node of the mnn2json graph.
#[derive(Debug, Clone, PartialEq)]
struct Node {
    name: String,
    /// "isp.unpack_blc16", "Conv", "BinaryOp", "Input", "Const", ...
    op_type: String,
    /// True if this is an `isp.*` custom op (Extra with main.type isp.*).
    is_isp: bool,
    inputs: Vec<i64>,
    outputs: Vec<i64>,
}

/// Parse an mnn2json dump into an abstract node graph (Rust logic only —
/// no external tools). Each op object has:
///   - "type": MNN OpType enum name (or "Extra" for custom ops)
///   - for custom ops: "main_type": "Extra" with "main": { "type": "isp.x" }
///   - "inputIndexes" / "outputIndexes"
fn parse_nodes(json: &str) -> Vec<Node> {
    let val: serde_json::Value = serde_json::from_str(json).expect("parse mnn2json");
    let oplists = val
        .get("oplists")
        .and_then(|o| o.as_array())
        .cloned()
        .unwrap_or_default();
    let mut nodes = Vec::with_capacity(oplists.len());
    for op in oplists.iter() {
        let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
        let (op_type, is_isp) = if ty == "Extra" {
            let extra = op
                .get("main")
                .and_then(|m| m.get("type"))
                .and_then(|t| t.as_str())
                .unwrap_or("Extra");
            (extra.to_string(), extra.starts_with("isp."))
        } else {
            (ty.to_string(), false)
        };
        let inputs = op
            .get("inputIndexes")
            .and_then(|v| v.as_array())
            .map(|a| a.iter().filter_map(|x| x.as_i64()).collect())
            .unwrap_or_default();
        let outputs = op
            .get("outputIndexes")
            .and_then(|v| v.as_array())
            .map(|a| a.iter().filter_map(|x| x.as_i64()).collect())
            .unwrap_or_default();
        nodes.push(Node {
            name: op
                .get("name")
                .and_then(|n| n.as_str())
                .unwrap_or("")
                .to_string(),
            op_type,
            is_isp,
            inputs,
            outputs,
        });
    }
    nodes
}

/// B-test constraint check: ALL ops must be `isp.*` custom opset, except
/// the graph Input/Const bookkeeping ops (which are not part of the
/// primitive compute opset). Returns the list of offending ops.
fn find_non_isp_ops(nodes: &[Node]) -> Vec<&Node> {
    nodes
        .iter()
        .filter(|n| !n.is_isp && n.op_type != "Input" && n.op_type != "Const")
        .collect()
}

/// Run Pass1 for a single block ONNX graph and verify isp.*-only opset.
fn run_pass1_block(
    name: &str,
    chain: Vec<Box<dyn IspBlock>>,
    shape: &[i32],
    dtype_code: i32,
) -> Vec<f32> {
    let mut chain = chain;
    GraphComposer::wire_blocks(&mut chain);
    let refs: Vec<&dyn IspBlock> = chain.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    // Pass0: ONNX → MNN (primitive ops, reference base).
    let mnn0 = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 onnx→mnn failed");
    let json0 = with_convert_lock(|| dump_mnn_to_json(&mnn0)).expect("pass0 json dump failed");
    let nodes0 = parse_nodes(&json0);

    // Pass1: MNN → MNN (framework=MNN → IspChainFusion applies).
    let mnn1 = with_convert_lock(|| convert_mnn_buffer(&mnn0)).expect("pass1 mnn→mnn failed");
    assert!(!mnn1.is_empty(), "pass1 mnn empty");

    let json1 = with_convert_lock(|| dump_mnn_to_json(&mnn1)).expect("pass1 json dump failed");
    let nodes1 = parse_nodes(&json1);

    // B-test constraint: Pass1 must be isp.*-only.
    let offending = find_non_isp_ops(&nodes1);
    assert!(
        offending.is_empty(),
        "[pass1:{}] B-test constraint FAILED — non-isp ops: {:?}\n  pass0 ops: {:?}\n  pass1 ops: {:?}",
        name,
        offending.iter().map(|n| &n.op_type).collect::<Vec<_>>(),
        nodes0.iter().map(|n| n.op_type.as_str()).collect::<Vec<_>>(),
        nodes1.iter().map(|n| n.op_type.as_str()).collect::<Vec<_>>(),
    );

    let isp_ops: Vec<&str> = nodes1
        .iter()
        .filter(|n| n.is_isp)
        .map(|n| n.op_type.as_str())
        .collect();
    println!(
        "[pass1:{}] {} isp ops: {:?} (pass0 had {} ops)",
        name,
        isp_ops.len(),
        isp_ops,
        nodes0.len()
    );
    assert!(!isp_ops.is_empty(), "[pass1:{}] no isp.* ops found", name);

    // Load Pass1 MNN on Vulkan and run one inference.
    let interp = MnnInterpreterSafe::from_buffer(&mnn1).expect("load pass1 mnn");
    let sess = interp
        .create_session(MnnBackendType::Vulkan, 4)
        .expect("create Vulkan session");

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
        input_ints = Vec::new();
        input_ptr = input.as_ptr() as *const c_void;
    }

    let out_name = chain
        .last()
        .expect("chain tail")
        .graph_output_name()
        .expect("out name");
    let out_name_c = CString::new(out_name).expect("cstring");
    let max_out = (input_count * 4).max(1024);
    let mut out = vec![0.0f32; max_out];

    unsafe {
        let ret = cam_isp::mnn_sys::mnn_run_with_output(
            interp.as_ptr(),
            sess.as_ptr(),
            input_ptr,
            dtype_code,
            32,
            shape.as_ptr(),
            shape.len() as i32,
            out_name_c.as_ptr(),
            out.as_mut_ptr(),
            max_out as i32,
        );
        assert_eq!(ret, 0, "[pass1:{}] inference failed (ret={})", name, ret);
    }
    for &v in out.iter() {
        assert!(v.is_finite(), "[pass1:{}] non-finite output", name);
    }

    out
}

fn core_blocks() -> Vec<(&'static str, Vec<Box<dyn IspBlock>>, Vec<i32>, i32)> {
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
        ),
    ]
}

#[test]
#[ignore]
fn test_pass1_core_blocks_isp_only_opset() {
    for (name, chain, shape, dtype) in core_blocks() {
        let out = run_pass1_block(name, chain, &shape, dtype);
        assert!(
            out.iter().any(|&v| v != 0.0),
            "[pass1:{}] all-zero output",
            name
        );
        println!(
            "[pass1:{}] ok (out[0..4]={:?})",
            name,
            &out[..4.min(out.len())]
        );
    }
}
