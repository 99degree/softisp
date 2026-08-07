//! Analyze the MNN opset structure with identity bridges.
//!
//! Builds the HEAVY pipeline with identity blocks inserted between every
//! pair, converts to MNN, dumps the full op list with input/output tensor
//! names, and prints the identity sandwich pattern.

#![cfg(feature = "mnn")]

use cam_isp::mnn_converter::{convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_opset_matcher;
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;
use std::sync::Mutex;

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
    let block_ids: Vec<String> = blocks.iter().map(|b| b.id().to_string()).collect();
    let mut pipeline: Vec<Box<dyn IspBlock>> = blocks;
    pipeline.push(Box::new(cam_isp::blocks::DisplayBlock::new(16)));

    let pipeline_ids: Vec<String> = pipeline.iter().map(|b| b.id().to_string()).collect();
    let refs: Vec<&dyn IspBlock> = pipeline.iter().map(|b| b.as_ref()).collect();
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 16).expect("compose failed");
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

    // 3) Analyze identity sandwich: find every Identity op that bridges blocks
    println!("\n=== IDENTITY SANDWICH PATTERN ===");
    println!("  Between every pair of consecutive real blocks there should be");
    println!("  an Identity op (from IdentityBlock) serving as a named bridge.\n");

    let identity_ops: Vec<&OpInfo> = ops.iter().filter(|o| o.op_type == "Identity").collect();
    println!("  Total Identity ops in MNN: {}", identity_ops.len());
    println!(
        "  Identity bridges in block chain: {}",
        pipeline_ids
            .iter()
            .filter(|id| id.starts_with("id_"))
            .count()
    );

    // Print sandwich: for each identity op, show what comes before and after
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
    sorted.sort_by(|a, b| b.1.cmp(&a.1));
    for (ty, count) in &sorted {
        println!("  {:30}: {}", ty, count);
    }

    // 5) Identify sandwich segments: run of non-Identity ops between Identity boundaries
    println!("\n=== BLOCK SANDWICH SEGMENTS (non-Identity runs between Identity ops) ===");
    let mut segments: Vec<(usize, usize, Vec<String>)> = Vec::new();
    let mut current_run: Vec<String> = Vec::new();
    let mut run_start: usize = 0;
    for op in &ops {
        if op.op_type == "Identity" {
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
    println!("  {} non-Identity segments found:\n", segments.len());
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

    // 6) Consecutive Identity chains
    println!("\n=== CONSECUTIVE IDENTITY CHAINS ===");
    let mut chains: Vec<(usize, usize)> = Vec::new();
    let mut chain_start: Option<usize> = None;
    for op in &ops {
        if op.op_type == "Identity" {
            if chain_start.is_none() {
                chain_start = Some(op.idx);
            }
        } else {
            if let Some(start) = chain_start.take() {
                chains.push((start, op.idx - 1));
            }
        }
    }
    if let Some(start) = chain_start {
        chains.push((start, ops.len() - 1));
    }
    for (start, end) in &chains {
        let len = end - start + 1;
        let label = if len > 1 {
            format!(
                "← {} consecutive Identity ops (possible zero-compute blocks or MNN fusion)",
                len
            )
        } else {
            format!("← single Identity bridge")
        };
        println!("  Identity[{}..{}]: {}", start, end, label);
    }

    // 7) Segment-to-block attribution using exact matching table
    println!("\n=== SEGMENT → BLOCK ATTRIBUTION (exact matching table) ===");
    for (i, (start, end, ops_in_seg)) in segments.iter().enumerate() {
        let op_refs: Vec<&str> = ops_in_seg.iter().map(|s| s.as_str()).collect();
        let matched = mnn_opset_matcher::scan_blocks(&op_refs);
        let range = if *end == 999 {
            format!("{}..end", start)
        } else {
            format!("{}..{}", start, end)
        };
        println!(
            "  seg {:2}: ops {:>3}  ({:>2} ops): matched → {:?}",
            i,
            range,
            ops_in_seg.len(),
            matched
        );
        // Also show exact match candidates for disambiguation
        for prefix_len in (1..=ops_in_seg.len()).rev() {
            let candidates = mnn_opset_matcher::match_exact(&op_refs[..prefix_len]);
            if !candidates.is_empty() {
                let names: Vec<&str> = candidates.iter().map(|p| p.block_name).collect();
                if names.len() > 1 {
                    println!(
                        "         prefix[0..{}]: ambiguous candidates → {:?}",
                        prefix_len, names
                    );
                }
                break;
            }
        }
    }
}
