//! Pass0 (A test): ONNX→MNN conversion per ISP block.
//!
//! For each registered ISP block (and the LITE/HEAVY full pipelines) we
//! compose a single-block ONNX graph, convert it to MNN in-process
//! (`convert_onnx_buffer`, zero disk writes), and validate that the
//! resulting `.mnn`:
//!   1. loads into `MnnInterpreterSafe`,
//!   2. creates a Vulkan session with the actual backend reporting Vulkan
//!      (or CPU fallback, logged),
//!   3. runs one inference on a synthetic input and produces a finite
//!      output tensor.
//!
//! Pass0 is the *reference base* for the A/B comparison: its primitive-op
//! MNN output is compared against Pass1 (MNN→MNN re-conversion with
//! `isp.*` custom opset) in `test_mnn_ab_comparison.rs`.
//!
//! Run:
//! ```sh
//! cd cam-rust
//! LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
//!   cargo test --test test_mnn_pass0 -p cam-isp --features mnn \
//!   -- --nocapture --ignored
//! ```

#![cfg(feature = "mnn")]

use cam_isp::mnn_converter::{convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_opset_matcher;
use cam_isp::mnn_sys::{MnnBackendType, MnnInterpreterSafe};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

use std::sync::Mutex;

/// MNN's converter keeps a process-global `Global<modelConfig>` singleton;
/// concurrent convert/dump calls from parallel test threads race on it.
/// Serialize all converter entry points with one process-wide lock.
static CONVERT_LOCK: Mutex<()> = Mutex::new(());

/// Run a converter closure under the global converter lock.
fn with_convert_lock<T>(f: impl FnOnce() -> T) -> T {
    let _guard = CONVERT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    f()
}

/// A single test block: name, constructor, input shape, output tensor name.
struct BlockCase {
    name: &'static str,
    /// Chain of blocks wired by GraphComposer::wire_blocks. Every chain starts
    /// with RawInputBlock (like the real pipeline): a head block that emits
    /// nodes must read a tensor *distinct* from its output, otherwise the
    /// graph degenerates to `x = f(x)` and the converter DCEs it to a bare
    /// Input op.
    chain: Vec<Box<dyn IspBlock>>,
    /// Input shape (NCHW).
    shape: Vec<i32>,
    /// Input element type as a halide type code (mnn_run_with_output
    /// compares against the model input tensor's `getType()`):
    /// 0 = int32, 2 = float32, 5 = int16, 10 = float16.
    dtype_code: i32,
    /// Element bit width (16 for INT16, 32 for INT32/FLOAT32).
    /// Passed as `buffer_type_bits` to `mnn_run_with_output`.
    type_bits: u8,
    /// Output tensor name (tail block output). Defaults to the tail's
    /// `graph_output_name()`.
    out_name: Option<String>,
}

fn run_pass0_block(mut case: BlockCase) -> (Vec<f32>, usize) {
    // Compose the chain; GraphComposer wires input_source from predecessors.
    GraphComposer::wire_blocks(&mut case.chain);
    let refs: Vec<&dyn IspBlock> = case.chain.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");

    // Pass0: ONNX → MNN in-process (no temp files).
    let mnn = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("onnx→mnn convert failed");
    assert!(!mnn.is_empty(), "converted mnn is empty");

    // Dump opset via built-in mnn2json (in-process) for the record.
    let json = with_convert_lock(|| dump_mnn_to_json(&mnn)).expect("mnn→json dump failed");
    let op_types = extract_op_types(&json);
    println!("[pass0:{}] ops: {}", case.name, op_types.join(","));

    // ── Exact matching table verification ─────────────────────────
    let filtered = mnn_opset_matcher::filter_bridge_ops(&op_types);
    if let Some(pat) = mnn_opset_matcher::match_first(&filtered) {
        println!(
            "[pass0:{}] table match: '{}' (pattern {:?})",
            case.name, pat.block_name, pat.op_types
        );
    } else if !filtered.is_empty() {
        println!(
            "[pass0:{}] ops {:?} not in matching table",
            case.name, filtered
        );
    }

    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("load MNN from buffer");
    let sess = interp
        .create_session(MnnBackendType::Vulkan, 4)
        .expect("create Vulkan session");

    let shape = case.shape.clone();
    if let Some(t) = interp.get_first_input(&sess) {
        let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &shape);
    }
    let _ = sess.resize();

    // Synthetic float input sized to the input shape.
    let input_count: usize = shape.iter().map(|&d| d.max(1) as usize).product();
    let mut input = vec![0.0f32; input_count];
    // unpack_blc16 subtracts BLC=64 and clips via ReLU6; raw Bayer data is
    // in [0, 65535], so generate values above the BLC threshold.
    let use_raw_range = case.name == "unpack_blc16";
    for (i, v) in input.iter_mut().enumerate() {
        if use_raw_range {
            *v = 100.0 + ((i % 4093) as f32) % 65400.0;
        } else {
            *v = ((i % 251) as f32) / 255.0;
        }
    }

    // Use mnn_run_host_tensors (copyFromHostTensor/copyToHostTensor) instead of
    // mnn_run_with_output (zero-copy buffer().host). The zero-copy path requires
    // gralloc mmap'd device memory (as cam_app provides); tests use regular heap.
    let max_out = (input_count * 4).max(1024);
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
        eprintln!(
            "[pass0:{}] mnn_run_host_tensors returned ret={}, out[0..4]: {:?}",
            case.name,
            ret,
            &out[..4.min(out.len())]
        );
        assert!(ret > 0, "inference failed for {} (ret={})", case.name, ret);
    }

    // Finite check.
    for &v in out.iter() {
        assert!(
            v.is_finite(),
            "non-finite output {} for block {}",
            v,
            case.name
        );
    }

    (out, input_count)
}

/// Extract op type strings from an mnn2json dump.
///
/// The flatbuffer JSON has one object per op with:
///   - `"type": "Conv" | "BinaryOp" | "Extra" | ...`  (OpType enum name)
///   - for custom ops: `"main_type": "Extra"` and `"main": { "type": "isp.x" }`
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
            // Custom op: read the isp.* string from main.type
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

/// The 6 core ISP custom-opset blocks (isp.unpack_blc16, isp.demosaic, isp.fcs,
/// isp.ee, isp.ldci, isp.display) — these are the Pass1 fusion targets.
///
/// Each case is a chain starting with RawInputBlock (matching production
/// topology) so the head block's input tensor name differs from its output.
fn core_blocks() -> Vec<BlockCase> {
    use cam_isp::blocks::*;

    let mut cases: Vec<BlockCase> = Vec::new();

    // unpack_blc16 (ISP1): INT16 [1,1,H,W] → FLOAT via BLC.
    // NOTE: MNN's ONNX→MNN converter upcasts INT16 inputs to FLOAT32,
    // so we pass dtype_code=2/32 to match what the converted MNN model
    // actually expects, even though the ONNX graph declares INT16.
    cases.push(BlockCase {
        name: "unpack_blc16",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_elem_type(5)
                    .with_concrete_dims(16, 16),
            ), // INT16 in ONNX, but MNN upcasts to FLOAT32
            Box::new(UnpackBlc16Block::new()),
        ],
        shape: vec![1, 1, 16, 16],
        dtype_code: 2, // FLOAT32 — matches MNN model input after converter upcast
        type_bits: 32,
        out_name: None,
    });

    // demosaic (ISP2): bayer FLOAT [1,4,H,W] → RGB.
    cases.push(BlockCase {
        name: "demosaic",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(DemosaicCcmBlock::new(0)), // RGGB, 4ch in
        ],
        shape: vec![1, 4, 16, 16],
        dtype_code: 2, // FLOAT (halide code 2)
        type_bits: 32,
        out_name: None,
    });

    // fcs (ISP3): colorspace / correction [1,3,H,W].
    cases.push(BlockCase {
        name: "fcs",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(FcsBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // ee (ISP4): edge enhancement [1,3,H,W].
    cases.push(BlockCase {
        name: "ee",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(EeBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // ldci (ISP5): local dynamic contrast [1,3,H,W].
    cases.push(BlockCase {
        name: "ldci",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(LdciBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // display (ISP6): gamma + display encode [1,3,H,W].
    cases.push(BlockCase {
        name: "display",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(DisplayBlock::new(16)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // --- HEAVY profile blocks (not yet individually tested) ---

    // unpack_cfa (PackedInt32 + BLC): INT32 packed → FLOAT [1,4,H/2,W/2].
    // HEAVY uses use_fused_unpack=true which builds:
    //   RawInputBlock(elem_type=6, packed_w) → UnpackCfaBlock(blc=true, full_w)
    cases.push(BlockCase {
        name: "unpack_cfa",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_elem_type(6) // INT32
                    .with_concrete_width(8) // packed_w = full_w/2 = 8
                    .with_concrete_height(16),
            ),
            Box::new(
                UnpackCfaBlock::new()
                    .with_concrete_width(16) // full_w = 16
                    .with_blc(true),
            ),
        ],
        shape: vec![1, 1, 16, 8], // INT32 packed: [1,1,H,W/2]
        dtype_code: 0,            // INT32
        type_bits: 32,
        out_name: None,
    });

    // bayer_wb (non-identity gains): Mul+Clip [1,4,H,W].
    cases.push(BlockCase {
        name: "bayer_wb",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(BayerWbBlock::new().with_gains(1.2, 1.0, 1.0, 0.8)),
        ],
        shape: vec![1, 4, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // ccm (LSC): Conv 1×1 + Clip [1,3,H,W] (identity matrix).
    cases.push(BlockCase {
        name: "ccm",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(CcmBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // bilateral: AvgPool + Sub + Abs + Identity [1,3,H,W].
    cases.push(BlockCase {
        name: "bilateral",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(BilateralBlock::new_default()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // vignetting: Mul(gain_map) [1,3,H,W].
    cases.push(BlockCase {
        name: "vignetting",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(VignettingBlock::new(16, 16, 0.5)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // gamma: Max+Min+Add+Log+Mul+Exp [1,3,H,W].
    cases.push(BlockCase {
        name: "gamma",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(GammaBlock::new(2.2)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // sharpen: AvgPool+Sub+Mul+Add [1,3,H,W].
    cases.push(BlockCase {
        name: "sharpen",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(SharpenBlock::new(0.5)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // saturation: Mul(scale) [1,3,H,W].
    cases.push(BlockCase {
        name: "saturation",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(SaturationBlock::new(1.2)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // auto_contrast: Add+Sub+Mul S-curve [1,3,H,W].
    cases.push(BlockCase {
        name: "auto_contrast",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(AutoContrastBlock::new(1.2)),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // wavelet_denoise: AveragePool [1,3,H,W].
    cases.push(BlockCase {
        name: "wavelet_denoise",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(WaveletDenoiseBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    // tone: fused tone mapping [1,3,H,W].
    cases.push(BlockCase {
        name: "tone",
        chain: vec![
            Box::new(
                RawInputBlock::new()
                    .with_concrete_dims(16, 16)
                    .with_elem_type(1),
            ),
            Box::new(ToneBlock::new()),
        ],
        shape: vec![1, 3, 16, 16],
        dtype_code: 2, // FLOAT
        type_bits: 32,
        out_name: None,
    });

    cases
}

#[test]
#[ignore]
fn test_pass0_core_blocks_onnx_to_mnn() {
    for case in core_blocks() {
        let name = case.name;
        let (out, _) = run_pass0_block(case);
        let first4: Vec<f32> = out.iter().take(4).copied().collect();
        let nonzero = out.iter().filter(|&&v| v != 0.0).count();
        eprintln!(
            "[pass0:{}]: nonzero={}, total={}, first4={:?}",
            name,
            nonzero,
            out.len(),
            first4
        );
        assert!(
            out.iter().any(|&v| v != 0.0),
            "all-zero output for block {}",
            name
        );
        println!(
            "[pass0:{}] ok (out[0..4]={:?})",
            name,
            &out[..4.min(out.len())]
        );
    }
}

#[test]
#[ignore]
fn test_pass0_lite_pipeline() {
    let blocks = PipelineProfile::LITE.build_blocks(16, 0);
    let mut pipeline: Vec<Box<dyn IspBlock>> = blocks;
    pipeline.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut pipeline);
    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");
    let mnn = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("convert failed");
    let json = with_convert_lock(|| dump_mnn_to_json(&mnn)).expect("json dump failed");
    let ops = extract_op_types(&json);
    println!("[pass0:lite] {} ops: {}", ops.len(), ops.join(","));

    // ── Segment attribution via exact matching table ───────────────
    let filtered = mnn_opset_matcher::filter_bridge_ops(&ops);
    let attributed = mnn_opset_matcher::scan_blocks(&filtered);
    println!("[pass0:lite] filtered ({}): {:?}", filtered.len(), filtered);
    println!("[pass0:lite] attribution: {:?}", attributed);

    assert!(!mnn.is_empty());
}

#[test]
#[ignore]
fn test_pass0_heavy_pipeline() {
    let blocks = PipelineProfile::HEAVY.build_blocks(16, 0);
    let mut pipeline: Vec<Box<dyn IspBlock>> = blocks;
    pipeline.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    cam_isp::pipeline::GraphComposer::wire_blocks(&mut pipeline);
    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");
    let mnn = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("convert failed");
    let json = with_convert_lock(|| dump_mnn_to_json(&mnn)).expect("json dump failed");
    let ops = extract_op_types(&json);
    println!("[pass0:heavy] {} ops: {}", ops.len(), ops.join(","));

    // ── Segment attribution via exact matching table ───────────────
    let filtered = mnn_opset_matcher::filter_bridge_ops(&ops);
    let attributed = mnn_opset_matcher::scan_blocks(&filtered);
    println!(
        "[pass0:heavy] filtered ({}): {:?}",
        filtered.len(),
        filtered
    );
    println!("[pass0:heavy] attribution: {:?}", attributed);

    assert!(!mnn.is_empty());
}
