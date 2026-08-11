//! Dump the ONNX→MNN op sequence for each of the three input styles, to
//! derive and verify the per-style IspChainFusion head patterns.
//!
//! For each `InputStyle` it builds the HEAVY pipeline with that input
//! convention, converts to MNN, and prints the op-type sequence (the
//! ground truth that `tryMatch` in IspChainFusion scans).
//!
//! Run on device:
//!   cargo run -p cam-isp --example dump_style_patterns \
//!     --features "rectifier mnn" --target aarch64-linux-android
use cam_isp::mnn_converter::{convert_onnx_buffer, dump_mnn_to_json};
use cam_isp::mnn_opset_matcher::{merge_fusion_patterns, InputStyle};
use cam_isp::pipeline::{GraphComposer, IspBlock};
use cam_isp::profile::PipelineProfile;

/// Extract op type strings from an mnn2json dump (same as test_mnn_pass0).
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

fn main() {
    for style in InputStyle::ALL {
        let (use_unpack, force16) = style.profile_flags();
        let mut profile = PipelineProfile::HEAVY.with_use_unpack(use_unpack);
        if force16 {
            profile = profile.with_force_input16();
        }
        let mut blocks = profile.build_blocks(1280, 0);
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = match GraphComposer::compose_from_vec(&refs, &[], 21) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("[{}] compose failed: {}", style.label(), e);
                continue;
            }
        };
        let mnn = match convert_onnx_buffer(&onnx) {
            Ok(b) => b,
            Err(e) => {
                eprintln!("[{}] convert failed: {}", style.label(), e);
                continue;
            }
        };
        let json = match dump_mnn_to_json(&mnn) {
            Ok(j) => j,
            Err(e) => {
                eprintln!("[{}] dump failed: {}", style.label(), e);
                continue;
            }
        };
        let ops = extract_op_types(&json);
        println!("\n=== {} ({}) ===", style.label(), ops.len());
        for (i, op) in ops.iter().enumerate() {
            println!("  [{:3}] {}", i, op);
        }
        // Head detail: op type, input/output tensor names, and for
        // BinaryOp/Const/Convolution the sub-parameters needed to write a
        // FusionSpec (bin_op_type, const_elems, conv_weight_elems).
        let val: serde_json::Value = serde_json::from_str(&json).expect("parse json");
        let oplists = val
            .get("oplists")
            .and_then(|o| o.as_array())
            .cloned()
            .unwrap_or_default();
        println!("  -- head detail (first 12 ops) --");
        for op in oplists.iter().take(12) {
            let ty = op.get("type").and_then(|t| t.as_str()).unwrap_or("?");
            let name = op.get("name").and_then(|t| t.as_str()).unwrap_or("?");
            let nin = op
                .get("inputIndexes")
                .and_then(|t| t.as_array())
                .map(|a| a.len())
                .unwrap_or(0);
            let nout = op
                .get("outputIndexes")
                .and_then(|t| t.as_array())
                .map(|a| a.len())
                .unwrap_or(0);
            let mut detail = String::new();
            if ty == "BinaryOp" {
                if let Some(b) = op.get("main").and_then(|m| m.get("opType")) {
                    detail = format!(" opType={}", b);
                }
            }
            if ty == "Const" {
                if let Some(blob) = op.get("main").and_then(|m| m.get("blob")) {
                    if let Some(dims) = blob.get("dims") {
                        let dims: Vec<serde_json::Value> =
                            dims.as_array().cloned().unwrap_or_default();
                        let elems: i64 = dims.iter().map(|d| d.as_i64().unwrap_or(0)).product();
                        detail = format!(" elems={}", elems);
                    }
                }
            }
            if ty == "Convolution" || ty == "ConvolutionDepthwise" {
                if let Some(main) = op.get("main") {
                    if let Some(w) = main
                        .get("weight")
                        .and_then(|w| w.as_array())
                        .map(|a| a.len())
                    {
                        detail = format!(" weights={}", w);
                    }
                    if let Some(k) = main.get("kernelX").and_then(|k| k.as_i64()) {
                        detail = format!("{} kernelX={}", detail, k);
                    }
                    if let Some(s) = main.get("strideX").and_then(|s| s.as_i64()) {
                        detail = format!("{} strideX={}", detail, s);
                    }
                }
            }
            if ty == "Permute" {
                if let Some(p) = op.get("main").and_then(|m| m.get("perm")) {
                    detail = format!(" perm={}", p);
                }
            }
            println!("    {} ({} in/{} out) [{}]{}", ty, nin, nout, name, detail);
        }
    }

    // ── Merged match table (single table covering all three styles) ──
    let merged = merge_fusion_patterns();
    println!("\n=== MERGED TABLE ({} entries) ===", merged.len());
    for style in InputStyle::ALL {
        let count = merged.iter().filter(|(s, _)| s.contains(&style)).count();
        println!("  {}: {} patterns", style.label(), count);
    }
    for (styles, p) in merged.iter() {
        let tags: Vec<&str> = styles.iter().map(|s| s.label()).collect();
        let fused = p
            .fusion
            .as_ref()
            .map(|f| f.isp_type)
            .unwrap_or("(no fusion)");
        println!(
            "  [{:>8}] {} -> {}  {:?}",
            tags.join("|"),
            p.block_name,
            fused,
            p.op_types
        );
    }
}
