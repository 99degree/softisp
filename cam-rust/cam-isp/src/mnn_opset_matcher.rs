//! Exact matching table: MNN op type sequences → ISP block names.
//!
//! Built from pass0 per-block ONNX→MNN conversion dumps. Each entry maps
//! an op sequence (excluding leading `Input`) to a block name. Ambiguous
//! pairs (identical op signatures for different blocks) are documented and
//! disambiguated by bridge position in pipeline context.
//!
//! Only HEAVY-profile blocks are covered (saturation, wavelet_denoise,
//! auto_contrast, unpack_cfa are Identity in HEAVY and excluded).
//!
//! # Ambiguous Pairs
//!
//! | Block A       | Block B          | Shared Signature                               |
//! |---------------|------------------|------------------------------------------------|
//! | ldci          | sharpen          | `ConvertTensor, Pooling, ConvertTensor, BinaryOp, Const, BinaryOp, BinaryOp` |
//! | demosaic      | ccm              | `ConvertTensor, Convolution, ReLU6, ConvertTensor` |
//! | unpack_blc16  | bayer_wb         | `Const, BinaryOp, ReLU6`                       |
//!
//! Disambiguation is handled by [`DISAMBIG_RULES`] (operator name prefix)
//! and [`ATTR_DISAMBIG_RULES`] (per-op attributes like BinaryOp.opType).

/// A single pattern entry mapping an MNN op type sequence to an ISP block name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpPattern {
    /// ISP block name (e.g., `"demosaic"`, `"ccm"`).
    pub block_name: &'static str,
    /// Expected op types in order, excluding the leading `Input` op.
    pub op_types: &'static [&'static str],
}

/// The exact matching table — HEAVY-profile ISP block op signatures from pass0.
///
/// Sorted by pattern length (ascending) to allow longest-match scanning.
///
/// Blocks with >3 ONNX ops use the `isp.*` naming convention to denote
/// custom ISP opset patterns that exceed simple elementwise signatures.
pub const EXACT_MATCH_TABLE: &[OpPattern] = &[
    // ── Length 1 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "tone",
        op_types: &["ReLU6"],
    },
    // ── Length 2 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "vignetting",
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
    OpPattern {
        block_name: "bilateral",
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
    // ── isp.* opset entries (blocks with >3 ops) ───────────────────
    // ToneStatsBlock: luma conv → mean/min/max → clip/shadow masks → concat.
    // 16 ops: ConvertTensor, Convolution, ConvertTensor, Reduction×3,
    //         Const, BinaryOp, ConvertTensor, Reduction, Const, BinaryOp,
    //         ConvertTensor, Reduction, Size, Concat
    OpPattern {
        block_name: "isp.tone_stats",
        op_types: &[
            "ConvertTensor",
            "Convolution",
            "ConvertTensor",
            "Reduction",
            "Reduction",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Size",
            "Concat",
        ],
    },
    // CalibrationBlock: variance → min/max range → lum/noise stats → concat+reshape.
    // 18 ops: Reduction, UnaryOp, Reduction, UnaryOp, BinaryOp,
    //         Reduction, Reduction, BinaryOp, Const, BinaryOp, BinaryOp,
    //         Reduction×4, Concat, Const, Reshape
    OpPattern {
        block_name: "isp.calibration",
        op_types: &[
            "Reduction",
            "UnaryOp",
            "Reduction",
            "UnaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "Reduction",
            "Reduction",
            "Concat",
            "Const",
            "Reshape",
        ],
    },
    // isp.histogram is variable-length (N bins → 2+4+6(N-2)+3+2 ops).
    // Matched via [`match_isp_histogram_prefix`] in scan_blocks.
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

/// Histogram opset prefix — the unique leading ops of CoarseHistogramBlock.
///
/// Histograms are variable-length (N bins → 2+4+6(N-2)+3+2 ops), so exact
/// matching is impractical.  Instead, `scan_blocks` checks this prefix to
/// identify histogram segments and consumes ops up to the trailing Concat.
const HISTOGRAM_OPS_PREFIX: &[&str] = &[
    "ConvertTensor",
    "Convolution",
    "Const",
    "BinaryOp",
    "ConvertTensor",
    "Reduction",
];

/// Check whether `ops` begin with the histogram prefix and return the total
/// op count for the segment (including the trailing `Const, Concat`).
///
/// Returns `None` if the prefix doesn't match.  The bin count is detected
/// by counting how many `BinaryOp, Const, BinaryOp, BinaryOp, ConvertTensor,
/// Reduction`6-tuples appear between the first bin and the final
/// `BinaryOp, ConvertTensor, Reduction, Const, Concat` tail.
pub fn match_isp_histogram_prefix(ops: &[&str]) -> Option<usize> {
    let prefix_len = HISTOGRAM_OPS_PREFIX.len();
    if ops.len() < prefix_len || ops[..prefix_len] != *HISTOGRAM_OPS_PREFIX {
        return None;
    }
    // Walk the repeating bin structure after the prefix.
    // Bin0 tail: BinaryOp, Const, BinaryOp, BinaryOp, ConvertTensor, Reduction (6 ops)
    // ... repeated for bins 1..N-1
    // Final tail: BinaryOp, ConvertTensor, Reduction, Const, Concat (5 ops)
    let mut pos = prefix_len;
    // First bin inner (after prefix already consumed Const,BinaryOp,ConvertTensor,Reduction): none
    // Actually the prefix is ConvertTensor,Convolution,Const,BinaryOp,ConvertTensor,Reduction
    // which is bin0's Const + b0 + ConvertTensor + Reduction.
    // After prefix: BinaryOp(lo), Const, BinaryOp(hi), BinaryOp(and), ConvertTensor, Reduction × (N-2)
    //              then BinaryOp(last), ConvertTensor, Reduction, Const, Concat
    while pos < ops.len() {
        // Check for final tail: BinaryOp, ConvertTensor, Reduction, Const, Concat
        if pos + 5 <= ops.len()
            && ops[pos] == "BinaryOp"
            && ops[pos + 1] == "ConvertTensor"
            && ops[pos + 2] == "Reduction"
            && ops[pos + 3] == "Const"
            && ops[pos + 4] == "Concat"
        {
            return Some(pos + 5);
        }
        // Check for inner bin: BinaryOp, Const, BinaryOp, BinaryOp, ConvertTensor, Reduction
        if pos + 6 <= ops.len()
            && ops[pos] == "BinaryOp"
            && ops[pos + 1] == "Const"
            && ops[pos + 2] == "BinaryOp"
            && ops[pos + 3] == "BinaryOp"
            && ops[pos + 4] == "ConvertTensor"
            && ops[pos + 5] == "Reduction"
        {
            pos += 6;
            continue;
        }
        break;
    }
    None
}

/// Scan an op list and greedily match the longest pattern at the current
/// position, advancing past matched ops. Returns matched block names in order.
///
/// This is the primary entry point for attributing pipeline opset segments
/// to real ISP blocks. It walks the op list left-to-right, trying the
/// longest table entry first at each position.  After exact-table scanning,
/// variable-length ISP patterns (e.g., `isp.histogram`) are tried.
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
        if found {
            continue;
        }
        // Try variable-length ISP prefix patterns
        if let Some(hist_len) = match_isp_histogram_prefix(remaining) {
            result.push("isp.histogram");
            pos += hist_len;
            continue;
        }
        // Skip unrecognized op
        pos += 1;
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

// ── Disambiguation tables ────────────────────────────────────────────

/// Operator name prefix for each block, extracted from MNN JSON dumps.
///
/// Each MNN operator has a `name` field like `"DemosaicCcmBlock/conv_out"`.
/// The prefix before the first `/` identifies the ISP block.  This table
/// maps block names to their known prefixes — used to resolve ambiguous
/// op-type pairs.
///
/// Source: `dump_blocks/*.json`
pub const OP_NAME_PREFIXES: &[(&str, &str)] = &[
    ("demosaic", "DemosaicCcmBlock"),
    ("display", "DisplayBlock"),
    ("ee", "EeBlock"),
    ("fcs", "FcsBlock"),
    ("ldci", "LdciBlock"),
    ("unpack_blc16", "UnpackBlc16Block"),
];

/// Per-op attribute rules for disambiguation of ambiguous pairs.
///
/// When op-type matching returns multiple candidates, these rules check
/// specific operator attributes from the MNN JSON to narrow down to one.
/// Each rule ties a block name to an expected attribute on a specific op
/// index within its pattern.
///
/// Source: `dump_blocks/*.json` — `main.opType` inside each operator.
pub const ATTR_DISAMBIG_RULES: &[(&str, usize, &str, &str)] = &[
    // (block_name, op_index_in_pattern, attr_key, expected_value)
    //
    // unpack_blc16 vs bayer_wb — BinaryOp.opType differs:
    //   unpack_blc16: SUB (subtract black level)
    //   bayer_wb:     MUL (multiply by white balance gains)
    ("unpack_blc16", 1, "opType", "SUB"),
    ("bayer_wb", 1, "opType", "MUL"),
    //
    // fcs — scaled=MUL, frame=ADD
    ("fcs", 1, "opType", "MUL"),
    ("fcs", 3, "opType", "ADD"),
    //
    // ldci — diff=SUB, boost=MUL, frame=ADD
    ("ldci", 3, "opType", "SUB"),
    ("ldci", 5, "opType", "MUL"),
    ("ldci", 6, "opType", "ADD"),
];

/// Disambiguate among multiple candidates using operator name prefixes.
///
/// Given a list of candidate patterns (from [`match_exact`]) and an operator
/// name string from the MNN JSON, returns the single candidate whose name
/// prefix matches.  Returns `None` if no prefix matches or the list is
/// ambiguous after filtering.
pub fn disambiguate_by_name<'a>(
    candidates: &[&'a OpPattern],
    op_name: &str,
) -> Option<&'a OpPattern> {
    let prefix = op_name.split('/').next().unwrap_or(op_name);
    let matches: Vec<&OpPattern> = candidates
        .iter()
        .copied()
        .filter(|p| {
            OP_NAME_PREFIXES
                .iter()
                .any(|(name, pre)| *name == p.block_name && prefix.starts_with(pre))
        })
        .collect();
    if matches.len() == 1 {
        Some(matches[0])
    } else {
        None
    }
}

/// Disambiguate among multiple candidates using per-op attributes.
///
/// `per_op_attrs` maps op_index → (attr_key → attr_value) as extracted from
/// the MNN JSON.  Each candidate is checked against [`ATTR_DISAMBIG_RULES`];
/// candidates whose rules don't match are eliminated.  Returns the single
/// surviving candidate, or `None` if disambiguation fails.
pub fn disambiguate_by_attrs<'a>(
    candidates: &[&'a OpPattern],
    per_op_attrs: &[(usize, &str, &str)], // [(op_index, attr_key, value)]
) -> Option<&'a OpPattern> {
    let surviving: Vec<&OpPattern> = candidates
        .iter()
        .copied()
        .filter(|p| {
            ATTR_DISAMBIG_RULES.iter().any(|(name, idx, key, val)| {
                *name == p.block_name
                    && per_op_attrs
                        .iter()
                        .any(|(oi, ak, av)| *oi == *idx && *ak == *key && *av == *val)
            })
        })
        .collect();
    if surviving.len() == 1 {
        Some(surviving[0])
    } else {
        None
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
    fn test_exact_vignetting_unique() {
        let m = match_exact(&["Const", "BinaryOp"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "vignetting");
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
    fn test_exact_bilateral_unique() {
        let m = match_exact(&["ConvertTensor", "Pooling", "ConvertTensor"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "bilateral");
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
    fn test_exact_display_unique() {
        let m = match_exact(&["ConvertTensor", "Convolution", "ConvertTensor"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "display");
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
    fn test_scan_fcs_then_unrecognized() {
        // After removing auto_contrast, "Const, BinaryOp, Const, BinaryOp, BinaryOp"
        // matches fcs (len 4) then skips the trailing BinaryOp.
        let ops = vec!["Const", "BinaryOp", "Const", "BinaryOp", "BinaryOp"];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["fcs"]);
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
        // BogusOp skipped, then vignetting matched (only entry for this pattern)
        assert_eq!(names, vec!["vignetting"]);
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
            "unpack_blc16",
            "bayer_wb",
            "bilateral",
            "ee",
            "display",
            "fcs",
            "demosaic",
            "ccm",
            "ldci",
            "sharpen",
            "gamma",
            "isp.tone_stats",
            "isp.calibration",
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
        // 15 HEAVY-profile blocks (+ ambiguous pairs share signatures,
        // so more entries than unique names)
        assert!(EXACT_MATCH_TABLE.len() >= 15);
    }

    // ── isp.* opset tests ─────────────────────────────────────────

    #[test]
    fn test_exact_isp_tone_stats() {
        let m = match_exact(&[
            "ConvertTensor",
            "Convolution",
            "ConvertTensor",
            "Reduction",
            "Reduction",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Size",
            "Concat",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "isp.tone_stats");
    }

    #[test]
    fn test_exact_isp_calibration() {
        let m = match_exact(&[
            "Reduction",
            "UnaryOp",
            "Reduction",
            "UnaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "Reduction",
            "Reduction",
            "Concat",
            "Const",
            "Reshape",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "isp.calibration");
    }

    #[test]
    fn test_scan_isp_tone_stats() {
        let ops = vec![
            "ConvertTensor",
            "Convolution",
            "ConvertTensor",
            "Reduction",
            "Reduction",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
            "Size",
            "Concat",
        ];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["isp.tone_stats"]);
    }

    #[test]
    fn test_scan_isp_calibration() {
        let ops = vec![
            "Reduction",
            "UnaryOp",
            "Reduction",
            "UnaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
            "Reduction",
            "Reduction",
            "Reduction",
            "Reduction",
            "Concat",
            "Const",
            "Reshape",
        ];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["isp.calibration"]);
    }

    #[test]
    fn test_histogram_prefix_16_bins() {
        // CoarseHistogramBlock with 16 bins: 6 + 14*6 + 5 = 95 ops
        let mut ops: Vec<&str> = Vec::new();
        // Prefix (bin0)
        ops.extend_from_slice(&[
            "ConvertTensor",
            "Convolution",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
        ]);
        // 14 inner bins
        for _ in 0..14 {
            ops.extend_from_slice(&[
                "BinaryOp",
                "Const",
                "BinaryOp",
                "BinaryOp",
                "ConvertTensor",
                "Reduction",
            ]);
        }
        // Final tail
        ops.extend_from_slice(&["BinaryOp", "ConvertTensor", "Reduction", "Const", "Concat"]);
        assert_eq!(ops.len(), 95);
        assert_eq!(match_isp_histogram_prefix(&ops), Some(95));
    }

    #[test]
    fn test_histogram_prefix_4_bins() {
        // CoarseHistogramBlock with 4 bins: 6 + 2*6 + 5 = 23 ops
        let mut ops: Vec<&str> = Vec::new();
        ops.extend_from_slice(&[
            "ConvertTensor",
            "Convolution",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
        ]);
        for _ in 0..2 {
            ops.extend_from_slice(&[
                "BinaryOp",
                "Const",
                "BinaryOp",
                "BinaryOp",
                "ConvertTensor",
                "Reduction",
            ]);
        }
        ops.extend_from_slice(&["BinaryOp", "ConvertTensor", "Reduction", "Const", "Concat"]);
        assert_eq!(ops.len(), 23);
        assert_eq!(match_isp_histogram_prefix(&ops), Some(23));
    }

    #[test]
    fn test_histogram_prefix_no_match() {
        let ops = vec!["ConvertTensor", "Convolution", "Const", "BinaryOp"];
        assert_eq!(match_isp_histogram_prefix(&ops), None);
    }

    #[test]
    fn test_histogram_prefix_wrong_start() {
        let ops = vec![
            "Const",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
        ];
        assert_eq!(match_isp_histogram_prefix(&ops), None);
    }

    #[test]
    fn test_scan_histogram_prefix_in_pipeline() {
        // Simulate: ee (3 ops) then histogram (23 ops) then fcs (4 ops)
        let mut ops: Vec<&str> = Vec::new();
        // ee
        ops.extend_from_slice(&["ConvertTensor", "ConvolutionDepthwise", "ConvertTensor"]);
        // histogram (4 bins)
        ops.extend_from_slice(&[
            "ConvertTensor",
            "Convolution",
            "Const",
            "BinaryOp",
            "ConvertTensor",
            "Reduction",
        ]);
        for _ in 0..2 {
            ops.extend_from_slice(&[
                "BinaryOp",
                "Const",
                "BinaryOp",
                "BinaryOp",
                "ConvertTensor",
                "Reduction",
            ]);
        }
        ops.extend_from_slice(&["BinaryOp", "ConvertTensor", "Reduction", "Const", "Concat"]);
        // fcs
        ops.extend_from_slice(&["Const", "BinaryOp", "Const", "BinaryOp"]);
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["ee", "isp.histogram", "fcs"]);
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
        // vignetting is now the only entry for Const,BinaryOp
        let result = assert_block_ops("vignetting", &["Const", "BinaryOp"]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().block_name, "vignetting");
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

    // ── Disambiguation tests ───────────────────────────────────────

    #[test]
    fn test_disambiguate_by_name_demosaic() {
        let candidates = match_exact(&["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"]);
        assert_eq!(candidates.len(), 2); // demosaic + ccm
        let result = disambiguate_by_name(&candidates, "DemosaicCcmBlock/conv_out");
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "demosaic");
    }

    #[test]
    fn test_disambiguate_by_name_ldci() {
        let candidates = match_exact(&[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ]);
        assert_eq!(candidates.len(), 2); // ldci + sharpen
        let result = disambiguate_by_name(&candidates, "LdciBlock/local_mean");
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "ldci");
    }

    #[test]
    fn test_disambiguate_by_name_no_prefix_match() {
        let candidates = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(candidates.len(), 2);
        // Unknown prefix — disambiguation fails
        let result = disambiguate_by_name(&candidates, "UnknownBlock/x");
        assert!(result.is_none());
    }

    #[test]
    fn test_disambiguate_by_attrs_unpack_blc16() {
        let candidates = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(candidates.len(), 2); // unpack_blc16 + bayer_wb
                                         // unpack_blc16 has BinaryOp(opType=SUB)
        let attrs = vec![(1, "opType", "SUB")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "unpack_blc16");
    }

    #[test]
    fn test_disambiguate_by_attrs_bayer_wb() {
        let candidates = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(candidates.len(), 2);
        // bayer_wb has BinaryOp(opType=MUL)
        let attrs = vec![(1, "opType", "MUL")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "bayer_wb");
    }

    #[test]
    fn test_disambiguate_by_attrs_no_rule() {
        // demosaic/ccm have no ATTR_DISAMBIG_RULES — disambiguation fails
        let candidates = match_exact(&["ConvertTensor", "Convolution", "ReLU6", "ConvertTensor"]);
        assert_eq!(candidates.len(), 2);
        let attrs = vec![(1, "kernelX", "1")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_none());
    }

    #[test]
    fn test_attrs_fcs_scaled_mul() {
        let candidates = match_exact(&["Const", "BinaryOp", "Const", "BinaryOp"]);
        assert_eq!(candidates.len(), 1);
        assert_eq!(candidates[0].block_name, "fcs");
        // fcs op[1] (scaled) is MUL
        let attrs = vec![(1, "opType", "MUL")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "fcs");
    }

    #[test]
    fn test_attrs_ldci_diff_sub() {
        let candidates = match_exact(&[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "Const",
            "BinaryOp",
            "BinaryOp",
        ]);
        assert_eq!(candidates.len(), 2); // ldci + sharpen
                                         // ldci op[3] (diff) is SUB
        let attrs = vec![(3, "opType", "SUB")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "ldci");
    }
}
