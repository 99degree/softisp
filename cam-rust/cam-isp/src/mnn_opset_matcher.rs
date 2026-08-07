//! Exact matching table: MNN op type sequences → ISP block names.
//!
//! Built from pass0 per-block ONNX→MNN conversion dumps. Each entry maps
//! an op sequence (excluding leading `Input`) to a block name. Ambiguous
//! pairs (identical op signatures for different blocks) are documented and
//! disambiguated by bridge position in pipeline context.
//!
//! # Ambiguous Pairs
//!
//! | Block A       | Block B          | Shared Signature                               |
//! |---------------|------------------|------------------------------------------------|
//! | vignetting    | saturation       | `Const, BinaryOp`                              |
//! | bilateral     | wavelet_denoise  | `ConvertTensor, Pooling, ConvertTensor`        |
//! | ldci          | sharpen          | `ConvertTensor, Pooling, ConvertTensor, BinaryOp, Const, BinaryOp, BinaryOp` |
//! | demosaic      | ccm              | `ConvertTensor, Convolution, ReLU6, ConvertTensor` |
//! | unpack_blc16  | bayer_wb         | `Const, BinaryOp, ReLU6`                       |

/// A single pattern entry mapping an MNN op type sequence to an ISP block name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpPattern {
    /// ISP block name (e.g., `"demosaic"`, `"ccm"`).
    pub block_name: &'static str,
    /// Expected op types in order, excluding the leading `Input` op.
    pub op_types: &'static [&'static str],
}

/// The exact matching table — all ISP block op signatures from pass0.
///
/// Sorted by pattern length (ascending) to allow longest-match scanning.
pub const EXACT_MATCH_TABLE: &[OpPattern] = &[
    // ── Length 1 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "tone",
        op_types: &["ReLU6"],
    },
    // ── Length 2 ───────────────────────────────────────────────────
    // AMBIGUOUS: vignetting and saturation share the same signature.
    OpPattern {
        block_name: "vignetting",
        op_types: &["Const", "BinaryOp"],
    },
    OpPattern {
        block_name: "saturation",
        op_types: &["Const", "BinaryOp"],
    },
    // ── Length 3 ───────────────────────────────────────────────────
    // AMBIGUOUS: unpack_blc16 and bayer_wb share the same signature.
    OpPattern {
        block_name: "unpack_blc16",
        op_types: &["Const", "BinaryOp", "ReLU6"],
    },
    OpPattern {
        block_name: "bayer_wb",
        op_types: &["Const", "BinaryOp", "ReLU6"],
    },
    // AMBIGUOUS: bilateral and wavelet_denoise share the same signature.
    OpPattern {
        block_name: "bilateral",
        op_types: &["ConvertTensor", "Pooling", "ConvertTensor"],
    },
    OpPattern {
        block_name: "wavelet_denoise",
        op_types: &["ConvertTensor", "Pooling", "ConvertTensor"],
    },
    OpPattern {
        block_name: "ee",
        op_types: &["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor"],
    },
    OpPattern {
        block_name: "display",
        op_types: &["ConvertTensor", "Convolution", "ConvertTensor"],
    },
    // ── Length 4 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "fcs",
        op_types: &["Const", "BinaryOp", "Const", "BinaryOp"],
    },
    // AMBIGUOUS: demosaic and ccm share the same signature.
    OpPattern {
        block_name: "demosaic",
        op_types: &["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"],
    },
    OpPattern {
        block_name: "ccm",
        op_types: &["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"],
    },
    // ── Length 5 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "auto_contrast",
        op_types: &["Const", "BinaryOp", "Const", "BinaryOp", "BinaryOp"],
    },
    // ── Length 7 ───────────────────────────────────────────────────
    // AMBIGUOUS: ldci and sharpen share the same signature.
    OpPattern {
        block_name: "ldci",
        op_types: &[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ],
    },
    OpPattern {
        block_name: "sharpen",
        op_types: &[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ],
    },
    // ── Length 8 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "gamma",
        op_types: &[
            "Const", "BinaryOp", "Const", "BinaryOp", "UnaryOp", "Const", "BinaryOp", "UnaryOp",
        ],
    },
    // ── Length 17 ──────────────────────────────────────────────────
    OpPattern {
        block_name: "unpack_cfa",
        op_types: &[
            "Const",
            "BinaryOp",
            "Cast",
            "Const",
            "BinaryOp",
            "BinaryOp",
            "Cast",
            "BinaryOp",
            "Concat",
            "ConvertTensor",
            "Convolution",
            "Const",
            "BinaryOp",
            "ReLU6",
            "Const",
            "BinaryOp",
            "ConvertTensor",
        ],
    },
];

/// Return all patterns whose `op_types` exactly match the given op slice.
///
/// For ambiguous pairs this returns multiple entries; callers can disambiguate
/// by pipeline position or by inspecting operator attributes.
///
/// ```
/// use cam_isp::mnn_opset_matcher::{match_exact, EXACT_MATCH_TABLE};
/// let matches = match_exact(&["ReLU6"]);
/// assert!(matches.iter().any(|p| p.block_name == "tone"));
/// ```
pub fn match_exact<'a>(ops: &[&str]) -> Vec<&'a OpPattern> {
    EXACT_MATCH_TABLE
        .iter()
        .filter(|p| p.op_types == ops)
        .collect()
}

/// Return the **first** matching pattern for the given op slice (longest-match
/// is not needed here because the table is exact — every entry has a distinct
/// length or content). Returns `None` if no pattern matches.
///
/// When multiple patterns match (ambiguous pair), the first entry in the
/// table wins. Callers needing all candidates should use [`match_exact`].
///
/// ```
/// use cam_isp::mnn_opset_matcher::match_first;
/// let m = match_first(&["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor"]);
/// assert_eq!(m.map(|p| p.block_name), Some("ee"));
/// ```
pub fn match_first(ops: &[&str]) -> Option<&'static OpPattern> {
    EXACT_MATCH_TABLE.iter().find(|p| p.op_types == ops)
}

/// Scan an op list and greedily match the longest pattern at the current
/// position, advancing past matched ops. Returns matched block names in order.
///
/// This is the primary entry point for attributing pipeline opset segments
/// to real ISP blocks. It walks the op list left-to-right, trying the
/// longest table entry first at each position.
///
/// ```
/// use cam_isp::mnn_opset_matcher::scan_blocks;
/// let ops = vec!["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor",
///                "Const", "BinaryOp", "Const", "BinaryOp"];
/// let names = scan_blocks(&ops);
/// // Should find "ee" (length 3) then "fcs" (length 4)
/// assert_eq!(names, vec!["ee", "fcs"]);
/// ```
pub fn scan_blocks(ops: &[&str]) -> Vec<&'static str> {
    let mut result = Vec::new();
    let mut pos = 0;
    while pos < ops.len() {
        let remaining = &ops[pos..];
        // Try longest patterns first (table sorted by length ascending,
        // so iterate in reverse for longest-match)
        let mut found = false;
        for p in EXACT_MATCH_TABLE.iter().rev() {
            let plen = p.op_types.len();
            if plen <= remaining.len() && &remaining[..plen] == p.op_types {
                result.push(p.block_name);
                pos += plen;
                found = true;
                break;
            }
        }
        if !found {
            // Skip unrecognized op
            pos += 1;
        }
    }
    result
}

/// List every block name registered in the exact matching table.
pub fn all_block_names() -> Vec<&'static str> {
    EXACT_MATCH_TABLE.iter().map(|p| p.block_name).collect()
}

/// Count how many distinct block names are in the table.
pub fn block_count() -> usize {
    all_block_names().len()
}

/// Filter out bridge/infrastructure ops that are not part of any ISP block's
/// actual computation. Returns the remaining op types as `&str` slices.
///
/// Stripped ops:
/// - `Input` — ONNX graph input, not a compute op
/// - `Permute` — IdentityBridgeBlock transpose (bridge between blocks)
/// - `Identity` — bare identity passes (MNN optimization artifacts)
pub fn filter_bridge_ops(ops: &[String]) -> Vec<&str> {
    ops.iter()
        .filter(|o| o.as_str() != "Input" && o.as_str() != "Permute" && o.as_str() != "Identity")
        .map(|s| s.as_str())
        .collect()
}

/// Look up a block name in the table and assert the given ops match.
/// Returns `Ok(matched_block_name)` on match, `Err(description)` on mismatch.
/// For ambiguous pairs, all candidates are checked — the assertion passes if
/// the expected name is among them.
pub fn assert_block_ops(expected_name: &str, ops: &[&str]) -> Result<&'static OpPattern, String> {
    let candidates = match_exact(ops);
    if candidates.is_empty() {
        return Err(format!(
            "block '{}' no match for ops {:?}",
            expected_name, ops
        ));
    }
    let matched: Vec<&str> = candidates.iter().map(|p| p.block_name).collect();
    if matched.contains(&expected_name) {
        // Return the first candidate that matches
        Ok(candidates
            .iter()
            .find(|p| p.block_name == expected_name)
            .unwrap())
    } else {
        Err(format!(
            "block '{}' not in candidates {:?} for ops {:?}",
            expected_name, matched, ops
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── match_exact tests ──────────────────────────────────────────

    #[test]
    fn test_exact_tone() {
        let m = match_exact(&["ReLU6"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "tone");
    }

    #[test]
    fn test_exact_vignetting_saturation_ambiguous() {
        let m = match_exact(&["Const", "BinaryOp"]);
        assert_eq!(m.len(), 2);
        let names: Vec<&str> = m.iter().map(|p| p.block_name).collect();
        assert!(names.contains(&"vignetting"));
        assert!(names.contains(&"saturation"));
    }

    #[test]
    fn test_exact_bayer_wb_unpack_blc16_ambiguous() {
        let m = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(m.len(), 2);
        let names: Vec<&str> = m.iter().map(|p| p.block_name).collect();
        assert!(names.contains(&"bayer_wb"));
        assert!(names.contains(&"unpack_blc16"));
    }

    #[test]
    fn test_exact_bilateral_wavelet_ambiguous() {
        let m = match_exact(&["ConvertTensor", "Pooling", "ConvertTensor"]);
        assert_eq!(m.len(), 2);
        let names: Vec<&str> = m.iter().map(|p| p.block_name).collect();
        assert!(names.contains(&"bilateral"));
        assert!(names.contains(&"wavelet_denoise"));
    }

    #[test]
    fn test_exact_demosaic_ccm_ambiguous() {
        let m = match_exact(&["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"]);
        assert_eq!(m.len(), 2);
        let names: Vec<&str> = m.iter().map(|p| p.block_name).collect();
        assert!(names.contains(&"demosaic"));
        assert!(names.contains(&"ccm"));
    }

    #[test]
    fn test_exact_ldci_sharpen_ambiguous() {
        let m = match_exact(&[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ]);
        assert_eq!(m.len(), 2);
        let names: Vec<&str> = m.iter().map(|p| p.block_name).collect();
        assert!(names.contains(&"ldci"));
        assert!(names.contains(&"sharpen"));
    }

    #[test]
    fn test_exact_ee_unique() {
        let m = match_exact(&["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "ee");
    }

    #[test]
    fn test_exact_fcs_unique() {
        let m = match_exact(&["Const", "BinaryOp", "Const", "BinaryOp"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "fcs");
    }

    #[test]
    fn test_exact_gamma_unique() {
        let m = match_exact(&[
            "Const", "BinaryOp", "Const", "BinaryOp", "UnaryOp", "Const", "BinaryOp", "UnaryOp",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "gamma");
    }

    #[test]
    fn test_exact_auto_contrast_unique() {
        let m = match_exact(&["Const", "BinaryOp", "Const", "BinaryOp", "BinaryOp"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "auto_contrast");
    }

    #[test]
    fn test_exact_display_unique() {
        let m = match_exact(&["ConvertTensor", "Convolution", "ConvertTensor"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "display");
    }

    #[test]
    fn test_exact_unpack_cfa_unique() {
        let m = match_exact(&[
            "Const",
            "BinaryOp",
            "Cast",
            "Const",
            "BinaryOp",
            "BinaryOp",
            "Cast",
            "BinaryOp",
            "Concat",
            "ConvertTensor",
            "Convolution",
            "Const",
            "BinaryOp",
            "ReLU6",
            "Const",
            "BinaryOp",
            "ConvertTensor",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "unpack_cfa");
    }

    #[test]
    fn test_exact_no_match() {
        let m = match_exact(&["NonexistentOp"]);
        assert!(m.is_empty());
    }

    #[test]
    fn test_exact_empty() {
        let m = match_exact(&[]);
        assert!(m.is_empty());
    }

    // ── match_first tests ──────────────────────────────────────────

    #[test]
    fn test_first_returns_first_ambiguous() {
        let m = match_first(&["Const", "BinaryOp"]);
        assert!(m.is_some());
        // First entry for this signature is vignetting
        assert_eq!(m.unwrap().block_name, "vignetting");
    }

    #[test]
    fn test_first_no_match() {
        assert!(match_first(&["Bogus"]).is_none());
    }

    // ── scan_blocks tests ──────────────────────────────────────────

    #[test]
    fn test_scan_ee_fcs() {
        let ops = vec![
            "ConvertTensor",
            "ConvolutionDepthwise",
            "ConvertTensor",
            "Const",
            "BinaryOp",
            "Const",
            "BinaryOp",
        ];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["ee", "fcs"]);
    }

    #[test]
    fn test_scan_longest_preferred() {
        // "Const, BinaryOp, Const, BinaryOp, BinaryOp" is auto_contrast (len 5),
        // not fcs + leftover (len 4 + 1 unrecognized)
        let ops = vec!["Const", "BinaryOp", "Const", "BinaryOp", "BinaryOp"];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["auto_contrast"]);
    }

    #[test]
    fn test_scan_subsumption() {
        // "ConvertTensor, Convolution, ReLU6, ConvertTensor" is demosaic/ccm (len 4),
        // which does NOT subsume bilateral's "ConvertTensor, Pooling, ConvertTensor" (len 3)
        // because the ops differ at position 1 (Convolution vs Pooling).
        let ops = vec!["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"];
        let names = scan_blocks(&ops);
        // demosaic or ccm (ambiguous pair, second wins via rev iteration)
        assert_eq!(names, vec!["ccm"]);
    }

    #[test]
    fn test_scan_empty() {
        assert!(scan_blocks(&[]).is_empty());
    }

    #[test]
    fn test_scan_unknown_ops_skipped() {
        let ops = vec!["BogusOp", "Const", "BinaryOp", "BogusOp2"];
        let names = scan_blocks(&ops);
        // BogusOp skipped, then vignetting/saturation matched (second wins via rev)
        assert_eq!(names, vec!["saturation"]);
    }

    #[test]
    fn test_scan_ldci_preferred_over_bilateral() {
        // Longer pattern (ldci/sharpen, len 7) should be matched before bilateral (len 3)
        let ops = vec![
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ];
        let names = scan_blocks(&ops);
        // Longest match wins (ldci or sharpen — both have len 7, second in table wins via rev)
        assert_eq!(names, vec!["sharpen"]); // longest match wins over bilateral
    }

    // ── Table integrity tests ──────────────────────────────────────

    #[test]
    fn test_table_has_all_expected_blocks() {
        let names = all_block_names();
        let expected = [
            "tone",
            "vignetting",
            "saturation",
            "unpack_blc16",
            "bayer_wb",
            "bilateral",
            "wavelet_denoise",
            "ee",
            "display",
            "fcs",
            "demosaic",
            "ccm",
            "auto_contrast",
            "ldci",
            "sharpen",
            "gamma",
            "unpack_cfa",
        ];
        for e in &expected {
            assert!(
                names.contains(e),
                "Missing block '{}' in EXACT_MATCH_TABLE",
                e
            );
        }
    }

    #[test]
    fn test_table_entry_count() {
        // 17 unique block names, but some ambiguous pairs share signatures
        // so more entries than unique names
        assert!(EXACT_MATCH_TABLE.len() >= 17);
    }

    // ── filter_bridge_ops tests ────────────────────────────────────

    #[test]
    fn test_filter_removes_input_permute_identity() {
        let ops = vec![
            "Input".to_string(),
            "Permute".to_string(),
            "Const".to_string(),
            "BinaryOp".to_string(),
            "Identity".to_string(),
        ];
        let filtered = filter_bridge_ops(&ops);
        assert_eq!(filtered, vec!["Const", "BinaryOp"]);
    }

    #[test]
    fn test_filter_preserves_compute_ops() {
        let ops = vec![
            "ConvertTensor".to_string(),
            "ConvolutionDepthwise".to_string(),
            "ConvertTensor".to_string(),
        ];
        let filtered = filter_bridge_ops(&ops);
        assert_eq!(
            filtered,
            vec!["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor"]
        );
    }

    #[test]
    fn test_filter_empty() {
        let ops: Vec<String> = vec![];
        let filtered = filter_bridge_ops(&ops);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_filter_all_bridges() {
        let ops = vec![
            "Input".to_string(),
            "Permute".to_string(),
            "Identity".to_string(),
        ];
        let filtered = filter_bridge_ops(&ops);
        assert!(filtered.is_empty());
    }

    // ── assert_block_ops tests ─────────────────────────────────────

    #[test]
    fn test_assert_block_ops_exact_match() {
        let result = assert_block_ops("tone", &["ReLU6"]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().block_name, "tone");
    }

    #[test]
    fn test_assert_block_ops_ambiguous_pair() {
        // vignetting and saturation share Const,BinaryOp
        let result = assert_block_ops("saturation", &["Const", "BinaryOp"]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().block_name, "saturation");
    }

    #[test]
    fn test_assert_block_ops_wrong_name() {
        let result = assert_block_ops("tone", &["Const", "BinaryOp"]);
        assert!(result.is_err());
    }

    #[test]
    fn test_assert_block_ops_no_match() {
        let result = assert_block_ops("bogus", &["BogusOp"]);
        assert!(result.is_err());
    }
}
