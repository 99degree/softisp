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

/// C++ fusion metadata for auto-generating `ExactPattern` entries in IspChainFusion.cpp.
///
/// When present, the codegen binary (`cargo run -p cam-app -- gen-isp-patterns`)
/// emits a C++ `ExactPattern` entry for this pattern. When `None`, the block has
/// no auto-generated fusion rule (e.g., `raw_input`, stats blocks).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FusionSpec {
    /// The Extra op type name emitted by the C++ converter (e.g., `"isp.fcs"`).
    pub isp_type: &'static str,
    /// SPIR-V kernel name and named float vector key (e.g., `"fcs"`), or `None`.
    pub named_key: Option<&'static str>,
    /// Expected const element count (`-1` = any).
    pub const_elems: i32,
    /// Which `inputIndexes` position holds the const blob (`-1` = any).
    pub const_index: i32,
    /// BinaryOp sub-type (`-1` = any, 0=ADD, 1=SUB, 2=MUL, 3=POW, 8=REALDIV).
    pub bin_op_type: i32,
    /// Convolution weight element count (`-1` = any).
    pub conv_weight_elems: i32,
    /// When `true`, the chain is a guard — consumed but kept primitive (no Extra op emitted).
    pub no_fuse: bool,
    /// Required MNN OpType of the op following this pattern (`-1` = any).
    pub next_op_type: i32,
}

/// A single pattern entry mapping an MNN op type sequence to an ISP block name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OpPattern {
    /// ISP block name (e.g., `"demosaic"`, `"ccm"`).
    pub block_name: &'static str,
    /// Expected op types in order, excluding the leading `Input` op.
    pub op_types: &'static [&'static str],
    /// Optional C++ fusion metadata for auto-generating `ExactPattern` entries.
    /// `None` means no auto-generated fusion rule.
    pub fusion: Option<FusionSpec>,
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
        fusion: None,
    },
    OpPattern {
        block_name: "bayer_wb",
        op_types: &["ReLU6"],
        fusion: None,
    },
    OpPattern {
        block_name: "bilateral",
        op_types: &["Identity"],
        fusion: None,
    },
    OpPattern {
        block_name: "fcs",
        op_types: &["Pooling"],
        fusion: None,
    },
    OpPattern {
        block_name: "gamma",
        op_types: &["ConvertTensor"],
        fusion: None,
    },
    // ── Length 2 ───────────────────────────────────���───────���───────
    OpPattern {
        block_name: "aux_hook_out",
        op_types: &["ConvertTensor", "ConvolutionDepthwise"],
        fusion: None,
    },
    OpPattern {
        block_name: "colorspace",
        op_types: &["Identity", "ConvertTensor"],
        fusion: None,
    },
    OpPattern {
        block_name: "ldci",
        op_types: &["Const", "BinaryOp"],
        fusion: None,
    },
    // AMBIGUOUS GROUP ct_identity: demosaic, ee, sharpen, aux_hook_src
    // all produce ConvertTensor,Identity in the full pipeline.
    OpPattern {
        block_name: "isp.ct_identity",
        op_types: &["ConvertTensor", "Identity"],
        fusion: None,
    },
    // AMBIGUOUS GROUP ct_conv: cfa, wavelet_denoise
    // both produce ConvertTensor,Convolution in the full pipeline.
    OpPattern {
        block_name: "isp.ct_conv",
        op_types: &["ConvertTensor", "Convolution"],
        fusion: None,
    },
    // ── Length 3 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "normalize",
        op_types: &["Cast", "Const", "BinaryOp"],
        fusion: None,
    },
    OpPattern {
        block_name: "blc",
        op_types: &["Const", "BinaryOp", "ReLU6"],
        fusion: None,
    },
    OpPattern {
        block_name: "ccm",
        op_types: &["ConvertTensor", "Convolution", "ReLU6"],
        fusion: None,
    },
    // ── Length 4 ───────────────────────────────────────────────────
    // Second CCM application (post-demosaic RGB domain).
    OpPattern {
        block_name: "ccm_post",
        op_types: &["Const", "BinaryOp", "Const", "BinaryOp"],
        fusion: None,
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
        fusion: None,
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
        fusion: None,
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
        fusion: None,
    },
    // ── Length 8 ───────────────────────────────────────────────────
    OpPattern {
        block_name: "vignetting",
        op_types: &[
            "Const", "BinaryOp", "Const", "BinaryOp", "UnaryOp", "Const", "BinaryOp", "UnaryOp",
        ],
        fusion: None,
    },
    // ── Length 10 ──────────────────────────────────────────────────
    OpPattern {
        block_name: "unpack",
        op_types: &[
            "Cast", "Cast", "Const", "Reshape", "Const", "BinaryOp", "Reshape", "Concat", "Const",
            "Reshape",
        ],
        fusion: None,
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
        fusion: None,
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
        fusion: None,
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

// ═══════════════════════════════════════════════════════════════════════════
// CPP_FUSION_TABLE — comprehensive C++ fusion patterns from IspChainFusion.cpp
// ═══════════════════════════════════════════════════════════════════════════
//
// Complete pattern table for generating `ExactPattern` entries in
// IspChainFusion.cpp.  Each entry maps an MNN op-type sequence to
// an ISP block with fusion metadata.
//
// C++ check order (first match wins):
//   Guard chains → long multi-op → medium → 1-op standalone → profile variants

/// Comprehensive pattern table ported from MNN's `IspChainFusion.cpp`.
///
/// Covers all ISP block op-type patterns with `FusionSpec` metadata for
/// auto-generating C++ `ExactPattern` entries.  `EXACT_MATCH_TABLE` (above)
/// is the legacy Rust-side matcher; this table documents the full C++ surface.
pub const CPP_FUSION_TABLE: &[OpPattern] = &[
    // ── Guard / noFuse patterns (consumed but kept primitive) ────────────

    // AlgoGammaChain (8-op): guard — realign gamma to ISP block boundaries.
    OpPattern {
        block_name: "algo_gamma",
        op_types: &[
            "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp",
            "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.noop_gamma",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: true,
            next_op_type: -1,
        }),
    },
    // AlgoCctChain (21-op): guard — realign CCT to ISP block boundaries.
    OpPattern {
        block_name: "algo_cct",
        op_types: &[
            "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp",
            "UnaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp",
            "UnaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.noop_cct",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: true,
            next_op_type: -1,
        }),
    },
    // ── Long multi-op patterns (8-21 ops) ───────────────────────────────

    // CalibrationBlock (21-op): variance → min/max range → lum/noise stats.
    OpPattern {
        block_name: "calibration_block",
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
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.calib_stats",
            named_key: Some("calib_stats"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // UnifiedStatsBlock (19-op): calib stats without trailing ternary.
    OpPattern {
        block_name: "unified_stats_block",
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
            "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.ispc_stats",
            named_key: Some("ispc_stats"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DisplayBlock (16-op): packed output with shape/rank/gather.
    OpPattern {
        block_name: "display_block",
        op_types: &[
            "Permute",
            "Padding",
            "Shape",
            "Rank",
            "BinaryOp",
            "BinaryOp",
            "Unsqueeze",
            "BinaryOp",
            "Unsqueeze",
            "StridedSlice",
            "Squeeze",
            "BinaryOp",
            "BinaryOp",
            "GatherV2",
            "BinaryOp",
            "Cast",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.display",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // FcsBlock (13-op): complex FCS chain (blur+sharp+denoise).
    OpPattern {
        block_name: "fcs_block",
        op_types: &[
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
            "ConvolutionDepthwise",
            "UnaryOp",
            "BinaryOp",
            "ReLU6",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
            "ReLU6",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ToneBlock (12-op): tone curve with saturation/contrast.
    OpPattern {
        block_name: "tone_block",
        op_types: &[
            "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp", "UnaryOp", "BinaryOp", "BinaryOp",
            "BinaryOp", "BinaryOp", "ReLU", "BinaryOp", "ReLU6",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.tone",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // AlgoAwbBlock (11-op): AWB stats with reshape.
    OpPattern {
        block_name: "algo_awb_block",
        op_types: &[
            "Reduction",
            "StridedSlice",
            "StridedSlice",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
            "StridedSlice",
            "BinaryOp",
            "BinaryOp",
            "Concat",
            "Reshape",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.awb",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // AlgoAeBlock (9-op): AE with clamp/saturation.
    OpPattern {
        block_name: "algo_ae_block",
        op_types: &[
            "StridedSlice",
            "StridedSlice",
            "BinaryOp",
            "BinaryOp",
            "UnaryOp",
            "BinaryOp",
            "ReLU6",
            "BinaryOp",
            "UnaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.ae",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // FocusBlock (8-op): AF focus with laplacian conv chain.
    OpPattern {
        block_name: "focus_block",
        op_types: &[
            "Convolution",
            "Convolution",
            "UnaryOp",
            "Convolution",
            "UnaryOp",
            "BinaryOp",
            "Reduction",
            "Squeeze",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.af_focus",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ── Mid-length patterns (4-7 ops) ───────────────────────────────────

    // LdciBlock (6-op): LDCI Pool+Clip chain.
    OpPattern {
        block_name: "ldci_block",
        op_types: &[
            "Pooling", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "ReLU6",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.ldci",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // LdciABlock (4-op): LDCI_A Pool no clip.
    OpPattern {
        block_name: "ldci_a_block",
        op_types: &["Pooling", "BinaryOp", "BinaryOp", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.ldci_a",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // EeBlock (6-op): edge enhancement with depthwise conv.
    OpPattern {
        block_name: "ee_block",
        op_types: &[
            "ConvolutionDepthwise",
            "BinaryOp",
            "BinaryOp",
            "ReLU6",
            "BinaryOp",
            "ReLU6",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.ee",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // BilateralBlock (5-op): bilateral denoise.
    OpPattern {
        block_name: "bilateral_block",
        op_types: &[
            "ConvolutionDepthwise",
            "ConvolutionDepthwise",
            "BinaryOp",
            "BinaryOp",
            "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.bilateral",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // RefYuvSatBlock (6-op): YUV saturation reference path.
    OpPattern {
        block_name: "ref_yuv_sat_block",
        op_types: &[
            "StridedSlice",
            "StridedSlice",
            "BinaryOp",
            "Convolution",
            "ReLU6",
            "Concat",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // LocalContrastBlock (5-op): local contrast via pooling.
    OpPattern {
        block_name: "local_contrast_block",
        op_types: &["Pooling", "BinaryOp", "BinaryOp", "BinaryOp", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // SaturationBlock (5-op): saturation via reduction.
    OpPattern {
        block_name: "saturation_block",
        op_types: &["Reduction", "BinaryOp", "BinaryOp", "BinaryOp", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // UnsharpBlock (5-op): unsharp mask via pooling.
    OpPattern {
        block_name: "unsharp_block",
        op_types: &["Pooling", "BinaryOp", "BinaryOp", "BinaryOp", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // RefToneBlock (4-op): tone reference (profile variant).
    OpPattern {
        block_name: "ref_tone_block",
        op_types: &["Pooling", "BinaryOp", "BinaryOp", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.tone",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // YuvSatBlock (7-op): YUV saturation with 6 BinaryOps + ReLU6.
    OpPattern {
        block_name: "yuv_sat_block",
        op_types: &[
            "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "ReLU6",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: 1,
            const_index: 1,
            bin_op_type: 1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // AutoContrastBlock (6-op): auto contrast with unary + 5 BinaryOps.
    OpPattern {
        block_name: "auto_contrast_block",
        op_types: &[
            "UnaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp", "BinaryOp",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.auto_contrast",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // WarpBlock (6-op): chromatic aberration / grid sample.
    OpPattern {
        block_name: "warp_block",
        op_types: &[
            "StridedSlice",
            "GridSample",
            "StridedSlice",
            "StridedSlice",
            "GridSample",
            "Concat",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.warp",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ── Display variants (4-6 ops) ──────────────────────────────────────

    // Display with conv input (6-op).
    OpPattern {
        block_name: "display_conv",
        op_types: &[
            "Convolution",
            "Permute",
            "Padding",
            "GatherV2",
            "BinaryOp",
            "Cast",
        ],
        fusion: Some(FusionSpec {
            isp_type: "isp.display",
            named_key: Some("display"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // Display short (5-op): Permute+Padding+GatherV2+BO+Cast.
    OpPattern {
        block_name: "display_short",
        op_types: &["Permute", "Padding", "GatherV2", "BinaryOp", "Cast"],
        fusion: Some(FusionSpec {
            isp_type: "isp.display",
            named_key: Some("display"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ── Unpack / Blc variants (2-3 ops) ─────────────────────────────────

    // UnpackBlc (2-op): BinaryOp(SUB) + ReLU6.
    OpPattern {
        block_name: "unpack_blc_2op",
        op_types: &["BinaryOp", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.unpack_blc",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: 1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // UnpackBlc (3-op): Reshape + ReLU6 + BinaryOp.
    OpPattern {
        block_name: "unpack_blc_3op",
        op_types: &["Reshape", "ReLU6", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.unpack_blc",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // RawBlcBlock (2-op): Cast → SUB (new lib, noFuse).
    OpPattern {
        block_name: "raw_blc_2op",
        op_types: &["Cast", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.unpack_blc",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: 1,
            conv_weight_elems: -1,
            no_fuse: true,
            next_op_type: -1,
        }),
    },
    // ── Normalize 2-op guard ────────────────────────────────────────────

    // Normalize guard (2-op): Cast → REALDIV (noFuse).
    OpPattern {
        block_name: "normalize_guard",
        op_types: &["Cast", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: 8,
            conv_weight_elems: -1,
            no_fuse: true,
            next_op_type: -1,
        }),
    },
    // ── Demosaic 2-op variants ──────────────────────────────────────────

    // DemosaicCcmBlock (2-op): Conv(12) + ReLU6.
    OpPattern {
        block_name: "demosaic_ccm_2op",
        op_types: &["Convolution", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_ccm",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 12,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // RefCcmBlock (2-op): Conv(9) + BinaryOp.
    OpPattern {
        block_name: "ref_ccm_block",
        op_types: &["Convolution", "BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // VignettingBlock (2-op): MUL(1024) + ReLU6.
    OpPattern {
        block_name: "vignetting_block",
        op_types: &["BinaryOp", "ReLU6"],
        fusion: Some(FusionSpec {
            isp_type: "isp.vignetting",
            named_key: None,
            const_elems: 1024,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ── 1-op patterns (standalone blocks, profile pass0) ─────────────────

    // FcsConv (1-op): Conv(9) → isp.fcs with named_key "fcs".
    OpPattern {
        block_name: "fcs_conv",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: Some("fcs"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // FcsWbGains (1-op): MUL(3,1) → isp.fcs.
    OpPattern {
        block_name: "fcs_wbgains",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: Some("fcs"),
            const_elems: 3,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // FcsBayerWb (1-op): MUL(4,1) → isp.fcs.
    OpPattern {
        block_name: "fcs_bayer_wb",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: Some("fcs"),
            const_elems: 4,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DemosaicConv300 (1-op): Conv(300) → isp.demosaic_ccm.
    OpPattern {
        block_name: "demosaic_conv300",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_ccm",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 300,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DemosaicConv36 (1-op): Conv(36) → isp.demosaic_ccm.
    OpPattern {
        block_name: "demosaic_conv36",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_ccm",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 36,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ColorspaceConv (1-op): Conv(9) → isp.colorspace with named_key.
    OpPattern {
        block_name: "colorspace_conv",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.colorspace",
            named_key: Some("colorspace"),
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // AeMul (1-op): MUL(3,1) → isp.ae.
    OpPattern {
        block_name: "ae_mul",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.ae",
            named_key: None,
            const_elems: 3,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // AfFocusConv (1-op): Conv(9) → isp.af_focus.
    OpPattern {
        block_name: "af_focus_conv",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.af_focus",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // PyramidConv16 (1-op): Conv(16) → isp.pyramid.
    OpPattern {
        block_name: "pyramid_conv16",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.pyramid",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 16,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ── Profile variant 1-op tables (no named_key, dynamic weights) ─────

    // CcmBlock (1-op): Conv(9) → isp.fcs (dynamic).
    OpPattern {
        block_name: "ccm_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // CfaBlock (1-op): Conv(16) → isp.pyramid.
    OpPattern {
        block_name: "cfa_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.pyramid",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 16,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // ColorSpaceMatMul (1-op): MatMul → isp.colorspace.
    OpPattern {
        block_name: "colorspace_matmul",
        op_types: &["MatMul"],
        fusion: Some(FusionSpec {
            isp_type: "isp.colorspace",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // CscBlock (1-op): Conv(9) → isp.colorspace.
    OpPattern {
        block_name: "csc_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.colorspace",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 9,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DebayerBlock (1-op): Conv(300) → isp.demosaic_debayer.
    OpPattern {
        block_name: "debayer_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_debayer",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 300,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DemosaicABlock (1-op): Conv(12) → isp.demosaic_a.
    OpPattern {
        block_name: "demosaic_a_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_a",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 12,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // EdgeDemosaicBlock (1-op): Conv(108) → isp.demosaic_edge.
    OpPattern {
        block_name: "edge_demosaic_block",
        op_types: &["Convolution"],
        fusion: Some(FusionSpec {
            isp_type: "isp.demosaic_edge",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 108,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // GammaBlock (1-op): POW(1,1) → isp.gamma.
    OpPattern {
        block_name: "gamma_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.gamma",
            named_key: None,
            const_elems: 1,
            const_index: 1,
            bin_op_type: 3,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // LscBlock (1-op): MUL(1024,1) → isp.lsc.
    OpPattern {
        block_name: "lsc_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.lsc",
            named_key: None,
            const_elems: 1024,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // NormalizeBlock (1-op): MUL(1,1) → isp.fcs.
    OpPattern {
        block_name: "normalize_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: 1,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // NormalizeBlockNoFuse (1-op): REALDIV(1,1) → isp.fcs (guard).
    OpPattern {
        block_name: "normalize_block_nofuse",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: 1,
            const_index: 1,
            bin_op_type: 8,
            conv_weight_elems: -1,
            no_fuse: true,
            next_op_type: -1,
        }),
    },
    // RawBlcBlock (1-op): SUB(1,1) → isp.unpack_blc.
    OpPattern {
        block_name: "raw_blc_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.unpack_blc",
            named_key: None,
            const_elems: 1,
            const_index: 1,
            bin_op_type: 1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // BayerWbBlock (1-op): MUL(4,1) → isp.fcs.
    OpPattern {
        block_name: "bayer_wb_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: 4,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // BlcBlock (1-op): SUB(4,1) → isp.fcs.
    OpPattern {
        block_name: "blc_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: 4,
            const_index: 1,
            bin_op_type: 1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // WbGainsBlock (1-op): MUL(3,1) → isp.awb.
    OpPattern {
        block_name: "wb_gains_block",
        op_types: &["BinaryOp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.awb",
            named_key: None,
            const_elems: 3,
            const_index: 1,
            bin_op_type: 2,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // EeAtomicBlock (1-op): ConvDW(27) → isp.ee.
    OpPattern {
        block_name: "ee_atomic_block",
        op_types: &["ConvolutionDepthwise"],
        fusion: Some(FusionSpec {
            isp_type: "isp.ee",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: 27,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // DownscaleInterp (1-op): Interp → isp.interp.
    OpPattern {
        block_name: "downscale_interp",
        op_types: &["Interp"],
        fusion: Some(FusionSpec {
            isp_type: "isp.interp",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
    // CropStridedSlice (1-op): StridedSlice → isp.fcs.
    OpPattern {
        block_name: "crop_strided_slice",
        op_types: &["StridedSlice"],
        fusion: Some(FusionSpec {
            isp_type: "isp.fcs",
            named_key: None,
            const_elems: -1,
            const_index: -1,
            bin_op_type: -1,
            conv_weight_elems: -1,
            no_fuse: false,
            next_op_type: -1,
        }),
    },
];

// ═══════════════════════════════════════════════════════════════════════════
// JSON I/O — serialize/deserialize pattern tables
// ═══════════════════════════════════════════════════════════════════════════

/// JSON-serializable version of [`OpPattern`].
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct IspPatternJson {
    pub block_name: String,
    pub op_types: Vec<String>,
    pub fusion: Option<FusionSpecJson>,
}

/// JSON-serializable version of [`FusionSpec`].
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FusionSpecJson {
    pub isp_type: String,
    pub named_key: Option<String>,
    pub const_elems: i32,
    pub const_index: i32,
    pub bin_op_type: i32,
    pub conv_weight_elems: i32,
    #[serde(default)]
    pub no_fuse: bool,
    #[serde(default)]
    pub next_op_type: i32,
}

impl From<&FusionSpec> for FusionSpecJson {
    fn from(f: &FusionSpec) -> Self {
        Self {
            isp_type: f.isp_type.to_string(),
            named_key: f.named_key.map(|s| s.to_string()),
            const_elems: f.const_elems,
            const_index: f.const_index,
            bin_op_type: f.bin_op_type,
            conv_weight_elems: f.conv_weight_elems,
            no_fuse: f.no_fuse,
            next_op_type: f.next_op_type,
        }
    }
}

impl From<&OpPattern> for IspPatternJson {
    fn from(p: &OpPattern) -> Self {
        Self {
            block_name: p.block_name.to_string(),
            op_types: p.op_types.iter().map(|s| s.to_string()).collect(),
            fusion: p.fusion.as_ref().map(FusionSpecJson::from),
        }
    }
}

/// Save a pattern table to a JSON file.
pub fn save_patterns_json(
    path: impl AsRef<std::path::Path>,
    patterns: &[OpPattern],
) -> Result<(), Box<dyn std::error::Error>> {
    let json: Vec<IspPatternJson> = patterns.iter().map(IspPatternJson::from).collect();
    let s = serde_json::to_string_pretty(&json)?;
    std::fs::write(path, s)?;
    Ok(())
}

/// Load patterns from a JSON file.
pub fn load_patterns_json(
    path: impl AsRef<std::path::Path>,
) -> Result<Vec<IspPatternJson>, Box<dyn std::error::Error>> {
    let s = std::fs::read_to_string(path)?;
    let json: Vec<IspPatternJson> = serde_json::from_str(&s)?;
    Ok(json)
}

// ═══════════════════════════════════════════════════════════════════════════
// C++ codegen — generate ExactPattern entries for IspChainFusion.cpp
// ═══════════════════════════════════════════════════════════════════════════

/// Map MNN op type string to C++ `MNN::OpType_*` enum name.
fn cpp_mnn_op_type(op: &str) -> &'static str {
    match op {
        "BinaryOp" => "MNN::OpType_BinaryOp",
        "UnaryOp" => "MNN::OpType_UnaryOp",
        "Convolution" => "MNN::OpType_Convolution",
        "ConvolutionDepthwise" => "MNN::OpType_ConvolutionDepthwise",
        "ReLU" => "MNN::OpType_ReLU",
        "ReLU6" => "MNN::OpType_ReLU6",
        "Pooling" => "MNN::OpType_Pooling",
        "Const" => "MNN::OpType_Const",
        "Cast" => "MNN::OpType_Cast",
        "Concat" => "MNN::OpType_Concat",
        "Reshape" => "MNN::OpType_Reshape",
        "Identity" => "MNN::OpType_Identity",
        "ConvertTensor" => "MNN::OpType_ConvertTensor",
        "Input" => "MNN::OpType_Input",
        "Reduction" => "MNN::OpType_Reduction",
        "Size" => "MNN::OpType_Size",
        "StridedSlice" => "MNN::OpType_StridedSlice",
        "Squeeze" => "MNN::OpType_Squeeze",
        "Unsqueeze" => "MNN::OpType_Unsqueeze",
        "Gather" => "MNN::OpType_Gather",
        "GatherV2" => "MNN::OpType_GatherV2",
        "Permute" => "MNN::OpType_Permute",
        "Padding" => "MNN::OpType_Padding",
        "Shape" => "MNN::OpType_Shape",
        "Rank" => "MNN::OpType_Rank",
        "MatMul" => "MNN::OpType_MatMul",
        "GridSample" => "MNN::OpType_GridSample",
        "Interp" => "MNN::OpType_Interp",
        _ => "MNN::OpType_Unknown",
    }
}

/// Map binary op code to C++ `MNN::BinaryOpOperation_*` enum name.
fn cpp_bin_op(code: i32) -> &'static str {
    match code {
        0 => "MNN::BinaryOpOperation_ADD",
        1 => "MNN::BinaryOpOperation_SUB",
        2 => "MNN::BinaryOpOperation_MUL",
        3 => "MNN::BinaryOpOperation_POW",
        8 => "MNN::BinaryOpOperation_REALDIV",
        _ => "MNN::BinaryOpOperation_INVALID",
    }
}

/// Generate a C++ `ExactPattern(...)` entry string for the given pattern.
///
/// The output matches the exact format used in `IspChainFusion.cpp` so it can
/// be pasted directly into the `static const ExactPattern kExact*[]` tables.
pub fn generate_cpp_entry(p: &OpPattern) -> String {
    let f = match &p.fusion {
        Some(f) => f,
        None => return format!("    // {} — no fusion spec", p.block_name),
    };

    let ops: Vec<&str> = p.op_types.iter().map(|o| cpp_mnn_op_type(o)).collect();
    let chain = p.op_types.len();

    // Positional args: opTypes, constElems, constIndex, ispType, typeKey
    let mut line = format!(
        "    // {} (chain={})\n    ExactPattern({{{}}}, {}, {}, \"{}\", \"{}\"",
        p.block_name,
        chain,
        ops.join(", "),
        f.const_elems,
        f.const_index,
        f.isp_type,
        f.isp_type,
    );

    // 6th positional: namedKey or nullptr
    match f.named_key {
        Some(k) => line.push_str(&format!(", \"{}\"", k)),
        None => line.push_str(", nullptr"),
    }

    // 7th positional: binOp (omit if INVALID = -1)
    if f.bin_op_type >= 0 {
        line.push_str(&format!(", {}", cpp_bin_op(f.bin_op_type)));
    }

    // 8th positional: convWeightElems (omit if -1)
    if f.conv_weight_elems >= 0 {
        if f.bin_op_type < 0 {
            line.push_str(", nullptr");
        }
        line.push_str(&format!(", {}", f.conv_weight_elems));
    }

    // noFuse flag
    if f.no_fuse {
        line.push_str(", true");
    }

    line.push(')');
    line
}

/// Generate a complete C++ `static const ExactPattern[]` table declaration.
///
/// Returns the full table as a `String` ready to paste into `IspChainFusion.cpp`.
pub fn generate_cpp_table(name: &str, entries: &[OpPattern]) -> String {
    let mut out = format!("static const ExactPattern {}[] = {{\n", name);
    for p in entries {
        let entry = generate_cpp_entry(p);
        out.push_str(&entry);
        out.push_str(";\n");
    }
    out.push_str("};\n");
    out
}

/// Classify an `OpPattern` into a pass category for the C++ header tables.
///
/// - **Pass0** (first-match fusion): concrete const/weight checks (`no_fuse == false`,
///   and at least one of `const_elems >= 0` or `conv_weight_elems >= 0`).
/// - **Pass1** (guard chains): `no_fuse == true` (consumed but not fused).
/// - **Profile** (structural-only): `const_elems == -1` and `conv_weight_elems == -1`
///   (relaxed matching).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CppPass {
    Pass0,
    Pass1,
    Profile,
}

/// Classify a pattern's fusion spec into a pass category.
pub fn classify_pass(p: &OpPattern) -> CppPass {
    match &p.fusion {
        None => CppPass::Profile,
        Some(f) => {
            if f.no_fuse {
                CppPass::Pass1
            } else if f.const_elems >= 0 || f.conv_weight_elems >= 0 {
                CppPass::Pass0
            } else {
                CppPass::Profile
            }
        }
    }
}

/// Generate a complete C++ header file with all ISP fusion patterns.
///
/// The header contains:
/// - `#pragma once` include guard
/// - Auto-generation comment with source provenance
/// - `#include` directives for MNN types
/// - Three `static const ExactPattern[]` tables: Pass0, Pass1, Profile
pub fn generate_cpp_header() -> String {
    let mut pass0: Vec<OpPattern> = CPP_FUSION_TABLE
        .iter()
        .filter(|p| classify_pass(p) == CppPass::Pass0)
        .cloned()
        .collect();
    let mut pass1: Vec<OpPattern> = CPP_FUSION_TABLE
        .iter()
        .filter(|p| classify_pass(p) == CppPass::Pass1)
        .cloned()
        .collect();
    let mut profile: Vec<OpPattern> = CPP_FUSION_TABLE
        .iter()
        .filter(|p| classify_pass(p) == CppPass::Profile)
        .cloned()
        .collect();

    // Sort each table by chain length descending (longest first) so the
    // first-match-wins scan in runPass() tries the most specific pattern
    // before shorter ones that could shadow it.
    let chain_len = |p: &OpPattern| std::cmp::Reverse(p.op_types.len());
    pass0.sort_by_key(chain_len);
    pass1.sort_by_key(chain_len);
    profile.sort_by_key(chain_len);

    let mut h = String::with_capacity(16384);

    // Header guard + auto-gen comment
    h.push_str("// Auto-generated by cam-isp mnn_opset_matcher.rs — DO NOT EDIT.\n");
    h.push_str("// Source: CPP_FUSION_TABLE in cam-rust/cam-isp/src/mnn_opset_matcher.rs\n");
    h.push_str("//\n");
    h.push_str("// Classification:\n");
    h.push_str("//   Pass0   — first-match fusion (concrete const/weight checks)\n");
    h.push_str("//   Pass1   — guard chains (no_fuse, consumed but kept primitive)\n");
    h.push_str("//   Profile — structural-only matching (relaxed const checks)\n");
    h.push_str("#pragma once\n");
    h.push('\n');

    // Includes
    h.push_str("#include <MNN/Define.hpp>\n");
    h.push_str("#include <MNN/expr/Expr.hpp>\n");
    h.push_str("#include <MNN/optimizer/Optimizer.hpp>\n");
    h.push('\n');
    h.push_str("// ExactPattern struct — defines the five constructor overloads\n");
    h.push_str("// used by the static tables below (6-arg base, 7a/7b, 8a/8b).\n");
    h.push_str("#include \"ExactPattern.h\"\n");
    h.push('\n');
    h.push_str("namespace MNN {\n");
    h.push('\n');

    // Pass0 table
    h.push_str(&format!(
        "// ── Pass0: first-match fusion ({} entries) ─────────────────\n",
        pass0.len()
    ));
    h.push_str(&generate_cpp_table("kExactFusionTablesPass0", &pass0));
    h.push('\n');

    // Pass1 table
    h.push_str(&format!(
        "// ── Pass1: guard chains ({} entries) ──────────────────────\n",
        pass1.len()
    ));
    h.push_str(&generate_cpp_table("kExactFusionTablesPass1", &pass1));
    h.push('\n');

    // Profile table
    h.push_str(&format!(
        "// ── Profile: structural-only matching ({} entries) ─────────\n",
        profile.len()
    ));
    h.push_str(&generate_cpp_table("kExactProfileVariants", &profile));
    h.push('\n');

    h.push_str("} // namespace MNN\n");
    h
}

/// Write the generated C++ header to a file.
pub fn save_cpp_header(
    path: impl AsRef<std::path::Path>,
) -> Result<(), Box<dyn std::error::Error>> {
    std::fs::write(path, generate_cpp_header())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fuse(
        isp: &'static str,
        key: Option<&'static str>,
        ce: i32,
        ci: i32,
        bo: i32,
        cw: i32,
    ) -> Option<FusionSpec> {
        Some(FusionSpec {
            isp_type: isp,
            named_key: key,
            const_elems: ce,
            const_index: ci,
            bin_op_type: bo,
            conv_weight_elems: cw,
            no_fuse: false,
            next_op_type: -1,
        })
    }

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

    // ── Codegen tests ─────────────────────────────────────────────

    #[test]
    fn test_classify_pass_no_fusion() {
        let p = OpPattern {
            op_types: &["ReLU6"],
            block_name: "test",
            fusion: None,
        };
        assert_eq!(classify_pass(&p), CppPass::Profile);
    }

    #[test]
    fn test_classify_pass_no_fuse_flag() {
        let p = OpPattern {
            op_types: &["Const"],
            block_name: "test",
            fusion: Some(FusionSpec {
                isp_type: "test",
                named_key: None,
                const_elems: 0,
                const_index: 0,
                bin_op_type: 1,
                conv_weight_elems: -1,
                no_fuse: true,
                next_op_type: -1,
            }),
        };
        assert_eq!(classify_pass(&p), CppPass::Pass1);
    }

    #[test]
    fn test_classify_pass_concrete_const() {
        let p = OpPattern {
            op_types: &["Const", "BinaryOp"],
            block_name: "test",
            fusion: Some(FusionSpec {
                isp_type: "test",
                named_key: None,
                const_elems: 1,
                const_index: 0,
                bin_op_type: 0,
                conv_weight_elems: -1,
                no_fuse: false,
                next_op_type: -1,
            }),
        };
        assert_eq!(classify_pass(&p), CppPass::Pass0);
    }

    #[test]
    fn test_classify_pass_structural_only() {
        let p = OpPattern {
            op_types: &["Convolution"],
            block_name: "test",
            fusion: Some(FusionSpec {
                isp_type: "test",
                named_key: None,
                const_elems: -1,
                const_index: -1,
                bin_op_type: -1,
                conv_weight_elems: -1,
                no_fuse: false,
                next_op_type: -1,
            }),
        };
        assert_eq!(classify_pass(&p), CppPass::Profile);
    }

    #[test]
    fn test_generate_cpp_entry_simple() {
        let p = OpPattern {
            op_types: &["BinaryOp"],
            block_name: "tone_curve",
            fusion: fuse("tone", None, 0, 0, 0, -1),
        };
        let line = generate_cpp_entry(&p);
        assert!(line.contains("ExactPattern("), "got: {line}");
        assert!(line.contains("OpType_BinaryOp"));
        assert!(line.contains("\"tone\""));
        assert!(
            line.contains("tone_curve"),
            "must include block_name comment: {line}"
        );
    }

    #[test]
    fn test_generate_cpp_entry_no_fuse() {
        let p = OpPattern {
            op_types: &["Const"],
            block_name: "guard_chain",
            fusion: Some(FusionSpec {
                isp_type: "guard",
                named_key: None,
                const_elems: 0,
                const_index: 0,
                bin_op_type: 1,
                conv_weight_elems: -1,
                no_fuse: true,
                next_op_type: 3,
            }),
        };
        let line = generate_cpp_entry(&p);
        assert!(
            line.contains("true)"),
            "no_fuse entry must include true flag: {line}"
        );
    }

    #[test]
    fn test_generate_cpp_entry_conv() {
        let p = OpPattern {
            op_types: &["Convolution"],
            block_name: "isp.conv",
            fusion: Some(FusionSpec {
                isp_type: "conv",
                named_key: None,
                const_elems: -1,
                const_index: -1,
                bin_op_type: -1,
                conv_weight_elems: 9,
                no_fuse: false,
                next_op_type: -1,
            }),
        };
        let line = generate_cpp_entry(&p);
        assert!(line.contains("Convolution"));
        assert!(
            line.contains("9)"),
            "conv_weight_elems=9 must appear: {line}"
        );
    }

    #[test]
    fn test_generate_cpp_table_count() {
        let table = generate_cpp_table("TestTable", &CPP_FUSION_TABLE);
        // Count entry lines: each entry ends with "ExactPattern(...);\n"
        let entry_count = table
            .lines()
            .filter(|l| l.trim().starts_with("ExactPattern("))
            .count();
        assert_eq!(
            entry_count,
            CPP_FUSION_TABLE.len(),
            "Table must have one entry per CPP_FUSION_TABLE element"
        );
    }

    #[test]
    fn test_generate_cpp_table_has_static_array() {
        let table = generate_cpp_table("kMyTable", &[]);
        assert!(table.contains("static const ExactPattern kMyTable[]"));
        assert!(table.contains("};"));
    }

    #[test]
    fn test_generate_cpp_header_structure() {
        let header = generate_cpp_header();
        assert!(header.contains("#pragma once"));
        assert!(header.contains("Auto-generated"));
        assert!(header.contains("#include <MNN/Define.hpp>"));
        assert!(header.contains("namespace MNN {"));
        assert!(header.contains("kExactFusionTablesPass0"));
        assert!(header.contains("kExactFusionTablesPass1"));
        assert!(header.contains("kExactProfileVariants"));
        assert!(header.contains("} // namespace MNN"));
    }

    #[test]
    fn test_generate_cpp_header_all_entries_covered() {
        let header = generate_cpp_header();
        let total = CPP_FUSION_TABLE.len();
        // Count entries across all 3 tables by counting exact-pattern entry lines
        // Each entry has exactly one "ExactPattern(" occurrence
        let pass0_count = header.matches("ExactPattern(").count();
        // All entries must appear somewhere
        assert_eq!(
            pass0_count, total,
            "Header must contain all {} CPP_FUSION_TABLE entries, found {}",
            total, pass0_count
        );
    }

    #[test]
    fn test_json_round_trip() {
        let dir = std::env::temp_dir().join("mnn_opset_test");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("fusion_table.json");

        save_patterns_json(&path, &CPP_FUSION_TABLE).unwrap();
        let loaded = load_patterns_json(&path).unwrap();
        assert_eq!(loaded.len(), CPP_FUSION_TABLE.len());

        for (orig, loaded) in CPP_FUSION_TABLE.iter().zip(loaded.iter()) {
            assert_eq!(orig.block_name, loaded.block_name);
            if let (Some(of), Some(lf)) = (&orig.fusion, &loaded.fusion) {
                assert_eq!(of.isp_type, lf.isp_type);
                assert_eq!(of.const_elems, lf.const_elems);
                assert_eq!(of.bin_op_type, lf.bin_op_type);
                assert_eq!(of.no_fuse, lf.no_fuse);
            }
        }

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_save_cpp_header_writes_file() {
        let dir = std::env::temp_dir().join("mnn_opset_header_test");
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("isp_fusion_patterns.h");

        save_cpp_header(&path).unwrap();
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.contains("#pragma once"));
        assert!(content.contains("kExactFusionTablesPass0"));

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ISP type → expected SPIR-V filename substrings (searched across
    /// `vulkan_isp/`).
    ///
    /// Every ISP type from `CPP_FUSION_TABLE` must have at least one matching
    /// `.spv` file.  The test walks `vulkan_isp/**/*.spv` and verifies each
    /// type has at least one file whose name contains the expected substring.
    fn spv_substrings_for(isp_type: &str) -> &'static [&'static str] {
        match isp_type {
            "isp.unpack_blc" => &["unpack_blc", "unpack_packed"],
            "isp.demosaic_ccm" => &["demosaic_ccm", "demosaic"],
            "isp.demosaic_noscale" => &["demosaic_noscale"],
            "isp.fcs" => &["fcs"],
            "isp.ee" => &["shader4_ee", "_ee_"],
            "isp.ldci" => &["ldci"],
            "isp.ldci_a" => &["ldci_a"],
            "isp.display" => &["display"],
            "isp.bilateral" => &["bilateral"],
            "isp.auto_contrast" => &["auto_contrast"],
            "isp.colorspace" => &["colorspace"],
            "isp.vignetting" => &["vignetting"],
            "isp.warp" => &["warp"],
            "isp.gamma" => &["shader_gamma"],
            "isp.pyramid" => &["pyramid"],
            "isp.interp" => &["demosaic_interp", "_interp"],
            // Formerly CPU-only — now all Vulkan-optimized
            "isp.ae" => &["shader_ae"],
            "isp.awb" => &["shader_awb"],
            "isp.af_focus" => &["af_focus"],
            "isp.tone" => &["shader_tone"],
            "isp.calib_stats" => &["calib_stats"],
            "isp.ispc_stats" => &["ispc_stats"],
            "isp.noop_gamma" => &["noop"],
            "isp.noop_cct" => &["noop"],
            "isp.lsc" => &["shader_lsc"],
            "isp.demosaic_debayer" => &["demosaic_debayer"],
            "isp.demosaic_a" => &["demosaic_a."],
            "isp.demosaic_edge" => &["demosaic_edge"],
            // Unknown types — fail so new ISP types get shader coverage
            other => panic!(
                "unknown ISP type '{}' — add SPIR-V mapping in spv_substrings_for()",
                other
            ),
        }
    }

    #[test]
    fn test_isp_types_have_corresponding_spirv() {
        // Collect ALL unique ISP types from the fusion table
        let mut all_isp_types: Vec<&str> = CPP_FUSION_TABLE
            .iter()
            .filter_map(|p| p.fusion.as_ref())
            .map(|f| f.isp_type)
            .collect();
        all_isp_types.sort();
        all_isp_types.dedup();
        assert!(
            !all_isp_types.is_empty(),
            "fusion table must have ISP types"
        );

        // Walk vulkan_isp/ and collect all .spv filenames
        let manifest = env!("CARGO_MANIFEST_DIR");
        let vulkan_isp = std::path::Path::new(manifest).join("../../vulkan_isp");
        assert!(
            vulkan_isp.exists(),
            "vulkan_isp/ directory not found at {}",
            vulkan_isp.display()
        );

        let mut spv_names: Vec<String> = Vec::new();
        for entry in walk_spv_dir(&vulkan_isp) {
            if let Some(name) = entry.file_name().to_str() {
                spv_names.push(name.to_string());
            }
        }
        assert!(
            !spv_names.is_empty(),
            "no .spv files found under vulkan_isp/"
        );

        // Check each ISP type has at least one matching .spv
        let mut missing: Vec<String> = Vec::new();
        for isp_type in &all_isp_types {
            let subs = spv_substrings_for(isp_type);
            let found = spv_names
                .iter()
                .any(|name| subs.iter().any(|sub| name.contains(sub)));
            if !found {
                missing.push(format!(
                    "{} — expected .spv containing {:?}",
                    isp_type, subs
                ));
            }
        }

        if !missing.is_empty() {
            panic!(
                "\nISP types without corresponding SPIR-V shaders:\n  {}\n\n\
                   Available .spv files: {:?}\n",
                missing.join("\n  "),
                spv_names
            );
        }
    }

    /// Recursively walk a directory yielding all entries whose extension is `spv`.
    fn walk_spv_dir(dir: &std::path::Path) -> Vec<std::fs::DirEntry> {
        let mut results = Vec::new();
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    results.extend(walk_spv_dir(&path));
                } else if path.extension().and_then(|e| e.to_str()) == Some("spv") {
                    results.push(entry);
                }
            }
        }
        results
    }
}
