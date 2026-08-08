//! Analyze the MNN opset structure with bridge-permute separators.
//!
//! Builds the HEAVY pipeline with BridgeIdentityBlock inserted between every
//! pair, converts to MNN, dumps the full op list, and segments on Permute ops
//! (the Transpose[0,1,2,3] bridges) to attribute each segment to a block.

#![cfg(feature = "mnn")]

use cam_isp::mnn_converter::{convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_opset_matcher;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;
use std::sync::Mutex;

/// Build HEAVY pipeline: build_blocks() already inserts BridgeIdentityBlocks
/// via wire_blocks_with_identities — no extra DisplayBlock or wire_blocks call needed.
static CONVERT_LOCK: Mutex<()> = Mutex::new(());

fn with_convert_lock<T>(f: impl FnOnce() -> T) -> T {
    let _guard = CONVERT_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    f()
}

/// Extract ops with their input/output tensor names for boundary analysis.
struct OpInfo {
    idx: usize,
    op_type: String,
    inputs: Vec<String>,
    outputs: Vec<String>,
}

fn extract_ops_detailed(json: &str) -> Vec<OpInfo> {
    let val: serde_json::Value = serde_json::from_str(json).expect("parse json");
    let oplists = val
        .get("oplists")
        .and_then(|o| o.as_array())
        .cloned()
        .unwrap_or_default();
    let mut ops = Vec::new();
    for (idx, op) in oplists.iter().enumerate() {
        let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
        let op_type = if ty == "Extra" {
            op.get("main")
                .and_then(|m| m.get("type"))
                .and_then(|t| t.as_str())
                .unwrap_or("Extra")
                .to_string()
        } else {
            ty.to_string()
        };
        let inputs: Vec<String> = op
            .get("inputIndexes")
            .and_then(|i| i.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default();
        let outputs: Vec<String> = op
            .get("outputIndexes")
            .and_then(|i| i.as_array())
            .map(|a| {
                a.iter()
                    .filter_map(|v| v.as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default();
        ops.push(OpInfo {
            idx,
            op_type,
            inputs,
            outputs,
        });
    }
    ops
}

#[test]
#[ignore]
fn test_pass0_heavy_opset_analysis() {
    let blocks = PipelineProfile::HEAVY.build_blocks(16, 0);
    let pipeline: Vec<Box<dyn IspBlock>> = blocks;
    let pipeline_ids: Vec<String> = pipeline.iter().map(|b| b.id().to_string()).collect();
    // build_blocks() already called wire_blocks_with_identities — bridges are present.

    // Build aux blocks (stats) so their ops appear in the MNN graph.
    let concrete_h = (16.0_f64 / 16.0 * 9.0).round() as i64;
    let aux = PipelineProfile::HEAVY.build_aux_blocks(concrete_h, 16);

    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let aux_refs: Vec<&dyn IspBlock> = aux.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &aux_refs, 16).expect("compose failed");
    let mnn = with_convert_lock(|| convert_onnx_buffer(&onnx)).expect("convert failed");
    let json = with_convert_lock(|| dump_mnn_to_json(&mnn)).expect("json dump failed");

    // 1) Print the block chain showing identity bridges
    println!("\n=== BLOCK CHAIN ({} blocks) ===", pipeline_ids.len());
    for (i, id) in pipeline_ids.iter().enumerate() {
        let marker = if id.starts_with("id_") {
            "  [IDENTITY BRIDGE]"
        } else {
            "  [REAL BLOCK]"
        };
        println!("  {:3}: {}{}", i, id, marker);
    }

    // 2) Print full opset
    let ops = extract_ops_detailed(&json);
    println!("\n=== MNN OPSET ({} ops) ===", ops.len());
    for op in &ops {
        println!(
            "  {:3}: {:30}  in={:?}  out={:?}",
            op.idx, op.op_type, op.inputs, op.outputs
        );
    }

    // 3) Bridge-permute sandwich: BridgeIdentityBlock emits Transpose[0,1,2,3]
    //    → MNN Permute. These survive as reliable separators.
    println!("\n=== PERMUTE BRIDGE SANDWICH ===");
    println!("  BridgeIdentityBlock emits Transpose[0,1,2,3] → MNN Permute.");
    println!("  These Permute ops are the real pipeline separators.\n");

    let permute_ops: Vec<&OpInfo> = ops.iter().filter(|o| o.op_type == "Permute").collect();
    println!("  Total Permute ops in MNN: {}", permute_ops.len());
    println!(
        "  Bridge blocks in chain: {}",
        pipeline_ids
            .iter()
            .filter(|id| id.starts_with("id_"))
            .count()
    );

    for p_op in &permute_ops {
        let idx = p_op.idx;
        let before = if idx > 0 {
            ops[idx - 1].op_type.as_str()
        } else {
            "START"
        };
        let after = if idx + 1 < ops.len() {
            ops[idx + 1].op_type.as_str()
        } else {
            "END"
        };
        println!("  op {:3}: Permute   <- {} -> {}", idx, before, after);
    }

    // 3b) Internal Identity ops (not bridges)
    let identity_ops: Vec<&OpInfo> = ops.iter().filter(|o| o.op_type == "Identity").collect();
    println!("\n  Internal Identity ops: {}", identity_ops.len());
    for id_op in &identity_ops {
        let idx = id_op.idx;
        let before = if idx > 0 {
            ops[idx - 1].op_type.as_str()
        } else {
            "START"
        };
        let after = if idx + 1 < ops.len() {
            ops[idx + 1].op_type.as_str()
        } else {
            "END"
        };
        println!("  op {:3}: Identity  <- {} -> {}", idx, before, after);
    }

    // 4) Count total ops
    let op_type_counts = {
        let mut counts = std::collections::HashMap::new();
        for op in &ops {
            *counts.entry(op.op_type.clone()).or_insert(0) += 1;
        }
        counts
    };
    println!("\n=== OP TYPE COUNTS ===");
    let mut sorted: Vec<_> = op_type_counts.into_iter().collect();
    sorted.sort_by_key(|b| std::cmp::Reverse(b.1));
    for (ty, count) in &sorted {
        println!("  {:30}: {}", ty, count);
    }

    // 5) Segment on Permute ops (bridges) — one segment per real block
    println!("\n=== PERMUTE-SEGMENTED BLOCKS (non-Permute runs between Permute bridges) ===");
    let mut segments: Vec<(usize, usize, Vec<String>)> = Vec::new();
    let mut current_run: Vec<String> = Vec::new();
    let mut run_start: usize = 0;
    for op in &ops {
        if op.op_type == "Permute" {
            // Close previous segment (skip empty segments from consecutive Permutes)
            if !current_run.is_empty() {
                segments.push((run_start, op.idx, current_run.clone()));
                current_run.clear();
            }
            run_start = op.idx + 1;
        } else {
            current_run.push(op.op_type.clone());
        }
    }
    if !current_run.is_empty() {
        segments.push((run_start, 999, current_run));
    }
    println!("  {} non-Permute segments found:\n", segments.len());
    for (i, (start, end, ops_in_seg)) in segments.iter().enumerate() {
        let op_list = ops_in_seg.join(", ");
        let range = if *end == 999 {
            format!("{}..end", start)
        } else {
            format!("{}..{}", start, end)
        };
        println!(
            "  seg {:2}: ops {:>3}  ({:>2} ops): {}",
            i,
            range,
            ops_in_seg.len(),
            op_list
        );
    }

    // ───────────────────────────────────────────────────────────────────
    // 6) SEGMENT → BLOCK ATTRIBUTION (positional — ground truth)
    // ───────────────────────────────────────────────────────────────────
    // BridgeIdentityBlock emits Transpose[0,1,2,3] → MNN Permute.
    // There are N-1 bridges for N real blocks. Permute ops are reliable
    // separators, so segment[i] == real_block_ids[i] by construction.
    let real_block_ids: Vec<&str> = pipeline_ids
        .iter()
        .filter(|id| !id.starts_with("id_"))
        .map(|s| s.as_str())
        .collect();

    // CORE ASSERTION: segment count must equal block count.
    // If this fails, bridges aren't separating blocks correctly.
    assert_eq!(
        segments.len(),
        real_block_ids.len(),
        "segment count ({}) must equal real block count ({})",
        segments.len(),
        real_block_ids.len()
    );

    println!("\n=== SEGMENT → BLOCK ATTRIBUTION ===");
    println!("  Positional: segment[i] = block[i] (ground truth from bridge order).\n");

    // ───────────────────────────────────────────────────────────────────
    // 7) PATTERN MATCHING (diagnostic — not pass/fail)
    // The EXACT_MATCH_TABLE was built from per-block ONNX→MNN conversion.
    // In the full pipeline, MNN cross-block optimization shifts ops across
    // Permute boundaries (e.g., trailing ConvertTensor moves to next seg).
    // Pattern matching is therefore a diagnostic showing which blocks
    // produce recognizable op sequences in the full pipeline context.
    let mut pattern_matched = 0u32;
    let mut pattern_total = 0u32;
    let mut ops_per_block: Vec<(&str, usize)> = Vec::new();

    for (i, (_start, _end, ops_in_seg)) in segments.iter().enumerate() {
        let block_name = real_block_ids[i];
        ops_per_block.push((block_name, ops_in_seg.len()));
        let op_refs: Vec<&str> = ops_in_seg.iter().map(|s| s.as_str()).collect();
        let scan = mnn_opset_matcher::scan_blocks(&op_refs);

        // Diagnostic: does the matcher recognize this segment's ops?
        let recognized = !scan.is_empty();
        if recognized {
            pattern_matched += 1;
        }
        pattern_total += 1;

        let ops_display = if ops_in_seg.len() <= 8 {
            ops_in_seg.join(", ")
        } else {
            format!(
                "{}…{}",
                ops_in_seg[..4].join(", "),
                ops_in_seg.last().unwrap()
            )
        };
        println!(
            "  {:2}: {:<22} {:>3} ops  {:5}  scan={:?}  {}",
            i,
            block_name,
            ops_in_seg.len(),
            if recognized { "MATCH" } else { "     " },
            scan,
            ops_display,
        );
    }

    // ───────────────────────────────────────────────────────────────────
    // 8) SUMMARY
    // ───────────────────────────────────────────────────────────────────
    println!("\n=== ATTRIBUTION SUMMARY ===");
    println!(
        "  Positional attribution: {}/{} segments (by construction)",
        real_block_ids.len(),
        segments.len()
    );
    println!(
        "  Pattern coverage:       {}/{} segments recognized by matcher",
        pattern_matched, pattern_total
    );
    println!("  Note: pattern coverage < 100% is expected — MNN cross-block");
    println!("  optimization shifts ops across Permute boundaries, breaking");
    println!("  per-block EXACT_MATCH_TABLE patterns in the full pipeline.");

    // Report per-block op counts for profiling
    println!("\n=== PER-BLOCK OP COUNTS ===");
    let total_ops: usize = ops_per_block.iter().map(|(_, n)| n).sum();
    println!(
        "  Total ops across {} blocks: {}",
        ops_per_block.len(),
        total_ops
    );
    for (name, count) in &ops_per_block {
        if *count > 0 {
            println!("  {:>24}: {:>3} ops", name, count);
        }
    }
}
