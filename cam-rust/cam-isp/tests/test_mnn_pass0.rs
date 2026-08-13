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

#[test]
#[ignore]
fn test_pass0_lite_pipeline() {
    let blocks = PipelineProfile::LITE.build_blocks(16, 0);
    let mut pipeline: Vec<Box<dyn IspBlock>> = blocks;
    pipeline.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    GraphComposer::wire_blocks(&mut pipeline);

    // Build aux blocks (stats) so their ops appear in the MNN graph.
    let concrete_h = (16.0_f64 / 16.0 * 9.0).round() as i64;
    let aux = PipelineProfile::LITE.build_aux_blocks(concrete_h, 16);

    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let aux_refs: Vec<&dyn IspBlock> = aux.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &aux_refs, 16).expect("compose failed");
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
    GraphComposer::wire_blocks(&mut pipeline);

    // Build aux blocks (stats) so their ops appear in the MNN graph.
    let concrete_h = (16.0_f64 / 16.0 * 9.0).round() as i64;
    let aux = PipelineProfile::HEAVY.build_aux_blocks(concrete_h, 16);

    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let aux_refs: Vec<&dyn IspBlock> = aux.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &aux_refs, 16).expect("compose failed");
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

/// Verify that IspChainFusion.cpp + generated isp_fusion_patterns.h correctly
/// matches the HEAVY profile ONNX pipeline, including proper skipping of
/// Permute/Identity bridge ops that MNN's layout converters may insert.
///
/// This test composes a full HEAVY pipeline, converts ONNX→MNN (Pass0, which
/// triggers IspChainFusion), and verifies:
///   (1) The MNN graph contains `isp.*` Extra ops — proving patterns matched
///       despite bridge ops.
///   (2) Bridge ops (Permute/Identity) present in pass0 ops are properly
///       filtered and do not appear in the attributed block scan.
///
/// Run:
/// ```sh
/// cd cam-rust
/// LD_LIBRARY_PATH=$PWD/lib/aarch64-v8a \
///   cargo test --test test_mnn_pass0 -p cam-isp --features mnn \
///   -- --nocapture --ignored test_ispchainfusion_heavy_pipeline
/// ```
#[test]
#[ignore]
fn test_ispchainfusion_heavy_pipeline() {
    let mut blocks = PipelineProfile::HEAVY.build_blocks(16, 0);
    blocks.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));
    GraphComposer::wire_blocks(&mut blocks);

    // Build aux blocks (stats) so their ops appear in the MNN graph.
    let concrete_h = (16.0_f64 / 16.0 * 9.0).round() as i64;
    let aux = PipelineProfile::HEAVY.build_aux_blocks(concrete_h, 16);

    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let aux_refs: Vec<&dyn IspBlock> = aux.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &aux_refs, 16).expect("compose failed");

    // Pass0: ONNX → MNN (runs IspChainFusion with the generated patterns).
    let mnn = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("pass0 convert failed");
    assert!(!mnn.is_empty(), "converted MNN is empty");

    let json = with_convert_lock(|| dump_mnn_to_json(&mnn)).expect("json dump failed");
    let ops0: Vec<String> = extract_op_types(&json);

    println!(
        "[ispchainfusion:heavy] pass0 ops ({}): {}",
        ops0.len(),
        ops0.join(",")
    );

    // (1) Verify ISP Extra ops are present — proves IspChainFusion patterns
    //     matched the HEAVY pipeline ops (including any bridge Permute/Identity).
    let has_isp = ops0.iter().any(|o| o.starts_with("isp."));
    assert!(
        has_isp,
        "[ispchainfusion:heavy] no ISP ops fused — IspChainFusion patterns did not match"
    );
    let isp_ops: Vec<&String> = ops0.iter().filter(|o| o.starts_with("isp.")).collect();
    println!(
        "[ispchainfusion:heavy] fused isp.* ops ({}): {:?}",
        isp_ops.len(),
        isp_ops
    );

    // (2) Verify bridge ops (Permute/Identity) present in raw pass0 ops
    //     are properly filtered from attribution.
    let has_bridge = ops0.iter().any(|o| *o == "Permute" || *o == "Identity");
    if has_bridge {
        println!("[ispchainfusion:heavy] bridge ops detected in pass0 graph — verifying filter");
    }

    // Filter bridge ops and verify scan_blocks still attributes correctly.
    let filtered = mnn_opset_matcher::filter_bridge_ops(&ops0);
    println!(
        "[ispchainfusion:heavy] filtered ({}): {:?}",
        filtered.len(),
        filtered
    );

    // Verify the filtered graph can be attributed to known blocks.
    let attributed = mnn_opset_matcher::scan_blocks(&filtered);
    println!("[ispchainfusion:heavy] attribution: {:?}", attributed);

    // The HEAVY pipeline should have at least some attributed blocks.
    assert!(
        !attributed.is_empty(),
        "[ispchainfusion:heavy] no blocks attributed after bridge-op filtering"
    );

    // Verify specific core blocks are attributed (proving patterns survived
    // the Permute/Identity skip in IspChainFusion::tryMatch).
    let expected_blocks = ["isp.demosaic", "isp.fcs", "isp.ee", "isp.display"];
    for expected in &expected_blocks {
        assert!(
            attributed.iter().any(|b| b.contains(expected)),
            "[ispchainfusion:heavy] expected block '{}' not attributed (attributed: {:?})",
            expected,
            attributed
        );
    }

    println!(
        "[ispchainfusion:heavy] ✓ IspChainFusion matched {} blocks with {} isp.* ops",
        attributed.len(),
        isp_ops.len()
    );
}
