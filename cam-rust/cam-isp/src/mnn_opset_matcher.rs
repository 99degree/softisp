//! Exact matching table: MNN op type sequences → ISP block names.
//!
//! Built from full-pipeline HEAVY profile ONNX→MNN conversion. Each entry
//! maps an op sequence (excluding leading `Input` and bridge `Permute` ops)
//! to a block name. Patterns are extracted from the actual opset produced
//! when all blocks are fused into a single MNN graph.
//!
//! # Ambiguous Groups
//!
//! MNN cross-block optimization fuses/eliminates ops, making some blocks
//! produce identical short op sequences:
//!
//! | Group | Shared Signature | Blocks |
//! |-------|-----------------|--------|
//! | ct_identity | `ConvertTensor, Identity` | demosaic, ee, sharpen, aux_hook_src |
//! | ct_conv     | `ConvertTensor, Convolution` | cfa, wavelet_denoise |
//!
//! These require positional disambiguation (pipeline block order).

/// A single pattern entry mapping an MNN op type sequence to an ISP block name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpPattern {
    /// ISP block name (e.g., `"demosaic"`, `"ccm"`).
    pub block_name: &'static str,
    /// Expected op types in order, excluding the leading `Input` op.
    pub op_types: &'static [&'static str],
}

/// The exact matching table — full-pipeline HEAVY-profile ISP block op signatures.
///
/// Patterns extracted from the actual MNN opset when all blocks are fused
/// into a single graph. MNN cross-block optimization significantly changes
/// op sequences vs per-block conversion (e.g., `bilateral` becomes `Identity`,
/// `tone` becomes `ConvertTensor, Pooling, ...`).
///
/// Sorted by pattern length (ascending) so `scan_blocks` tries longest
/// patterns first (reverse iteration).
pub const EXACT_MATCH_TABLE: &[OpPattern] = &[
    // ── Length 1 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "raw_input",
        op_types: &["Input"],
    },
    OpPattern {
        block_name: "bayer_wb",
        op_types: &["ReLU6"],
    },
    OpPattern {
        block_name: "bilateral",
        op_types: &["Identity"],
    },
    OpPattern {
        block_name: "fcs",
        op_types: &["Pooling"],
    },
    OpPattern {
        block_name: "gamma",
        op_types: &["ConvertTensor"],
    },
    // ── Length 2 ───────────────────────────────────���───────���───────
    OpPattern {
        block_name: "aux_hook_out",
        op_types: &["ConvertTensor", "ConvolutionDepthwise"],
    },
    OpPattern {
        block_name: "colorspace",
        op_types: &["Identity", "ConvertTensor"],
    },
    OpPattern {
        block_name: "ldci",
        op_types: &["Const", "BinaryOp"],
    },
    // AMBIGUOUS GROUP ct_identity: demosaic, ee, sharpen, aux_hook_src
    // all produce ConvertTensor,Identity in the full pipeline.
    OpPattern {
        block_name: "isp.ct_identity",
        op_types: &["ConvertTensor", "Identity"],
    },
    // AMBIGUOUS GROUP ct_conv: cfa, wavelet_denoise
    // both produce ConvertTensor,Convolution in the full pipeline.
    OpPattern {
        block_name: "isp.ct_conv",
        op_types: &["ConvertTensor", "Convolution"],
    },
    // ── Length 3 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "normalize",
        op_types: &["Cast", "Const", "BinaryOp"],
    },
    OpPattern {
        block_name: "blc",
        op_types: &["Const", "BinaryOp", "ReLU6"],
    },
    OpPattern {
        block_name: "ccm",
        op_types: &["ConvertTensor", "Convolution", "ReLU6"],
    },
    // ── Length 4 ───────────────────────────────────────────────────
    // Second CCM application (post-demosaic RGB domain).
    OpPattern {
        block_name: "ccm_post",
        op_types: &["Const", "BinaryOp", "Const", "BinaryOp"],
    },
    // ── Length 5 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "auto_contrast",
        op_types: &[
            "ConvertTensor",
            "ConvertTensor",
            "ConvertTensor",
            "ConvertTensor",
            "Identity",
        ],
    },
    // ── Length 6 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "saturation",
        op_types: &[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
        ],
    },
    // ── Length 7 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "tone",
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
        block_name: "vignetting",
        op_types: &[
            "Const", "BinaryOp", "Const", "BinaryOp", "UnaryOp", "Const", "BinaryOp", "UnaryOp",
        ],
    },
    // ── Length 10 ──────────────────────────────────────────────────
    OpPattern {
        block_name: "unpack",
        op_types: &[
            "Cast", "Cast", "Const", "Reshape", "Const", "BinaryOp", "Reshape", "Concat", "Const",
            "Reshape",
        ],
    },
    // ── isp.* sub-patterns (within display mega-segment) ──────────
    // ToneStatsBlock: luma conv → mean/min/max → clip/shadow masks → concat.
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
/// assert!(matches.iter().any(|p| p.block_name == "bayer_wb"));
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
/// When multiple patterns match (ambiguous group), the first entry in the
/// table wins. Callers needing all candidates should use [`match_exact`].
///
/// ```
/// use cam_isp::mnn_opset_matcher::match_first;
/// let m = match_first(&["ConvertTensor", "ConvolutionDepthwise"]);
/// assert_eq!(m.map(|p| p.block_name), Some("aux_hook_out"));
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
/// to real ISP blocks. It walks the op list left-to-right, trying variable-
/// length ISP prefix patterns (e.g., `isp.histogram`) first at each position
/// (since their 6-op prefix overlaps with short exact patterns), then falling
/// back to longest-match-first exact table scanning.
///
/// ```
/// use cam_isp::mnn_opset_matcher::scan_blocks;
/// let ops = vec!["ConvertTensor", "Identity",
///                "Const", "BinaryOp"];
/// let names = scan_blocks(&ops);
/// // "ConvertTensor,Identity" → isp.ct_identity, "Const,BinaryOp" → ldci
/// assert_eq!(names, vec!["isp.ct_identity", "ldci"]);
/// ```
pub fn scan_blocks(ops: &[&str]) -> Vec<&'static str> {
    let mut result = Vec::new();
    let mut pos = 0;
    while pos < ops.len() {
        let remaining = &ops[pos..];
        // Try variable-length ISP prefix patterns first — histogram is
        // variable-length and its 6-op prefix overlaps with short exact
        // patterns (e.g., ConvertTensor,Convolution → isp.ct_conv).
        if let Some(hist_len) = match_isp_histogram_prefix(remaining) {
            result.push("isp.histogram");
            pos += hist_len;
            continue;
        }
        // Try longest exact patterns first (table sorted by length ascending,
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
/// op-type groups (e.g., `isp.ct_identity` → demosaic/ee/sharpen/aux_hook_src).
///
/// Source: MNN JSON dumps from full HEAVY pipeline conversion.
pub const OP_NAME_PREFIXES: &[(&str, &str)] = &[
    // ct_identity group: ConvertTensor,Identity
    // Group entry maps the aggregate pattern to ALL constituent block prefixes.
    ("isp.ct_identity", "DemosaicCcmBlock"),
    ("isp.ct_identity", "EeBlock"),
    ("isp.ct_identity", "SharpenBlock"),
    ("isp.ct_identity", "AuxHook"),
    // ct_conv group: ConvertTensor,Convolution
    ("isp.ct_conv", "CfaBlock"),
    ("isp.ct_conv", "WaveletBlock"),
    // Individual block prefixes for finer-grained disambiguation
    ("demosaic", "DemosaicCcmBlock"),
    ("ee", "EeBlock"),
    ("sharpen", "SharpenBlock"),
    ("aux_hook_src", "AuxHook"),
    ("cfa", "CfaBlock"),
    ("wavelet_denoise", "WaveletBlock"),
    // Other blocks with unique patterns
    ("unpack", "UnpackBlock"),
    ("blc", "BlcBlock"),
    ("ccm", "CcmBlock"),
    ("tone", "ToneBlock"),
    ("vignetting", "VignettingBlock"),
];

/// Per-op attribute rules for disambiguation of ambiguous groups.
///
/// When op-type matching returns multiple candidates, these rules check
/// specific operator attributes from the MNN JSON to narrow down to one.
/// Each rule ties a block name to an expected attribute on a specific op
/// index within its pattern.
///
/// With the full-pipeline table, most per-block ambiguity is resolved by
/// op-type sequences. The remaining ambiguous groups (`isp.ct_identity`,
/// `isp.ct_conv`) are disambiguated by pipeline position or name prefixes.
pub const ATTR_DISAMBIG_RULES: &[(&str, usize, &str, &str)] = &[
    // (block_name, op_index_in_pattern, attr_key, expected_value)
    //
    // No attribute-based disambiguation needed for current ambiguous groups.
    // ct_identity (4-way) and ct_conv (2-way) are resolved by positional
    // pipeline order, not by operator attributes.
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
    fn test_exact_bayer_wb() {
        let m = match_exact(&["ReLU6"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "bayer_wb");
    }

    #[test]
    fn test_exact_ldci_unique() {
        let m = match_exact(&["Const", "BinaryOp"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "ldci");
    }

    #[test]
    fn test_exact_ct_identity_ambiguous() {
        // ConvertTensor,Identity matches 4 blocks: ct_identity group
        let m = match_exact(&["ConvertTensor", "Identity"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "isp.ct_identity");
    }

    #[test]
    fn test_exact_bilateral_unique() {
        let m = match_exact(&["Identity"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "bilateral");
    }

    #[test]
    fn test_exact_ct_conv_ambiguous() {
        // ConvertTensor,Convolution matches 2 blocks: ct_conv group
        let m = match_exact(&["ConvertTensor", "Convolution"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "isp.ct_conv");
    }

    #[test]
    fn test_exact_blc_unique() {
        let m = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "blc");
    }

    #[test]
    fn test_exact_aux_hook_out_unique() {
        let m = match_exact(&["ConvertTensor", "ConvolutionDepthwise"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "aux_hook_out");
    }

    #[test]
    fn test_exact_ccm_post_unique() {
        let m = match_exact(&["Const", "BinaryOp", "Const", "BinaryOp"]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "ccm_post");
    }

    #[test]
    fn test_exact_vignetting_unique() {
        let m = match_exact(&[
            "Const", "BinaryOp", "Const", "BinaryOp", "UnaryOp", "Const", "BinaryOp", "UnaryOp",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "vignetting");
    }

    #[test]
    fn test_exact_saturation_unique() {
        let m = match_exact(&[
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
        ]);
        assert_eq!(m.len(), 1);
        assert_eq!(m[0].block_name, "saturation");
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
    fn test_first_returns_unique_match() {
        let m = match_first(&["Const", "BinaryOp"]);
        assert!(m.is_some());
        assert_eq!(m.unwrap().block_name, "ldci");
    }

    #[test]
    fn test_first_no_match() {
        assert!(match_first(&["Bogus"]).is_none());
    }

    // ── scan_blocks tests ───────────────────────────────────���──────

    #[test]
    fn test_scan_aux_hook_out_then_ldci() {
        let ops = vec!["ConvertTensor", "ConvolutionDepthwise", "Const", "BinaryOp"];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["aux_hook_out", "ldci"]);
    }

    #[test]
    fn test_scan_ccm_post_then_unrecognized() {
        // ccm_post (len 4) consumes Const,BO,Const,BO; trailing BO is skipped.
        let ops = vec!["Const", "BinaryOp", "Const", "BinaryOp", "BinaryOp"];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["ccm_post"]);
    }

    #[test]
    fn test_scan_ct_conv_is_not_ct_identity() {
        // ConvertTensor,Convolution,ReLU6 matches ccm (len 3), not ct_identity (len 2)
        let ops = vec!["ConvertTensor", "Convolution", "ReLU6"];
        let names = scan_blocks(&ops);
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
        // BogusOp skipped, then ldci matched (Const,BinaryOp)
        assert_eq!(names, vec!["ldci"]);
    }

    #[test]
    fn test_scan_saturation_longer_than_ct() {
        // saturation (len 6) should be matched before ct_identity (len 2)
        let ops = vec![
            "ConvertTensor",
            "Pooling",
            "ConvertTensor",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
        ];
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["saturation"]);
    }

    #[test]
    fn test_scan_tone_longer_than_saturation() {
        // tone (len 7) matches before saturation (len 6) due to Const at position 4
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
        assert_eq!(names, vec!["tone"]);
    }

    // ── Table integrity tests ──────────────────────────────────────

    #[test]
    fn test_table_has_all_expected_blocks() {
        let names = all_block_names();
        let expected = [
            "raw_input",
            "bayer_wb",
            "bilateral",
            "fcs",
            "gamma",
            "aux_hook_out",
            "colorspace",
            "ldci",
            "isp.ct_identity",
            "isp.ct_conv",
            "normalize",
            "blc",
            "ccm",
            "ccm_post",
            "auto_contrast",
            "saturation",
            "tone",
            "vignetting",
            "unpack",
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
        // 19 standalone + 2 isp.* sub-patterns = 21 entries
        assert_eq!(EXACT_MATCH_TABLE.len(), 21);
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
        // Simulate: aux_hook_out (2 ops) then histogram (23 ops) then ldci (2 ops)
        let mut ops: Vec<&str> = Vec::new();
        // aux_hook_out
        ops.extend_from_slice(&["ConvertTensor", "ConvolutionDepthwise"]);
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
        // ldci
        ops.extend_from_slice(&["Const", "BinaryOp"]);
        let names = scan_blocks(&ops);
        assert_eq!(names, vec!["aux_hook_out", "isp.histogram", "ldci"]);
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
        ];
        let filtered = filter_bridge_ops(&ops);
        assert_eq!(filtered, vec!["ConvertTensor", "ConvolutionDepthwise"]);
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
        let result = assert_block_ops("bayer_wb", &["ReLU6"]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().block_name, "bayer_wb");
    }

    #[test]
    fn test_assert_block_ops_ct_identity_group() {
        // isp.ct_identity is the only entry for ConvertTensor,Identity
        let result = assert_block_ops("isp.ct_identity", &["ConvertTensor", "Identity"]);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().block_name, "isp.ct_identity");
    }

    #[test]
    fn test_assert_block_ops_wrong_name() {
        // tone expects ReLU6, not Const,BinaryOp
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
    fn test_disambiguate_by_name_ct_identity_demosaic() {
        let candidates = match_exact(&["ConvertTensor", "Identity"]);
        assert_eq!(candidates.len(), 1); // isp.ct_identity (group entry)
        assert_eq!(candidates[0].block_name, "isp.ct_identity");
        // Name prefix disambiguation on a single candidate returns that candidate
        let result = disambiguate_by_name(&candidates, "DemosaicCcmBlock/conv_out");
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "isp.ct_identity");
    }

    #[test]
    fn test_disambiguate_by_name_ct_conv() {
        let candidates = match_exact(&["ConvertTensor", "Convolution"]);
        assert_eq!(candidates.len(), 1); // isp.ct_conv (group entry)
        let result = disambiguate_by_name(&candidates, "CfaBlock/cfa");
        assert!(result.is_some());
        assert_eq!(result.unwrap().block_name, "isp.ct_conv");
    }

    #[test]
    fn test_disambiguate_by_name_no_prefix_match() {
        // bayer_wb has no prefix in OP_NAME_PREFIXES
        let candidates = match_exact(&["ReLU6"]);
        assert_eq!(candidates.len(), 1);
        let result = disambiguate_by_name(&candidates, "UnknownBlock/x");
        assert!(result.is_none());
    }

    #[test]
    fn test_disambiguate_by_attrs_all_eliminated() {
        // ATTR_DISAMBIG_RULES is empty, so no candidate can match any rule.
        let candidates = match_exact(&["ConvertTensor", "Identity"]);
        assert_eq!(candidates.len(), 1);
        let attrs = vec![(1, "opType", "SUB")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        // No rules → no surviving candidates
        assert!(result.is_none());
    }

    #[test]
    fn test_disambiguate_by_attrs_empty_rules() {
        // With empty ATTR_DISAMBIG_RULES, no disambiguation is possible
        let candidates = match_exact(&["Const", "BinaryOp", "ReLU6"]);
        assert_eq!(candidates.len(), 1); // blc (unique)
        let attrs = vec![(1, "opType", "MUL")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        // No rules → no match
        assert!(result.is_none());
    }

    #[test]
    fn test_disambiguate_by_attrs_no_surviving() {
        // No rules exist, so disambiguation always fails
        let candidates = match_exact(&["ConvertTensor", "Convolution"]);
        assert_eq!(candidates.len(), 1);
        let attrs = vec![(1, "kernelX", "1")];
        let result = disambiguate_by_attrs(&candidates, &attrs);
        assert!(result.is_none());
    }
}
