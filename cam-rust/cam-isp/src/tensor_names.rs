//! Canonical runtime tensor names mirrored from `docs/reference/TensorNames.java`.
//!
//! The legacy names are kept as aliases: an already-converted MNN model still
//! declares the old tensor inputs, and the engine writes both spellings.

/// Bayer-plane white-balance gains `[R, Gr, Gb, B]`.
pub const BAYER_WB_GAINS: &str = "BAYERWBBLOCK/BAYER_GAINS";
/// Legacy spelling of [`BAYER_WB_GAINS`].
pub const BAYER_WB_GAINS_LEGACY: &str = "BayerWbBlock/gains";
/// Canonical-first aliases for Bayer-plane WB gains.
pub const BAYER_WB_GAINS_ALIASES: [&str; 2] = [BAYER_WB_GAINS, BAYER_WB_GAINS_LEGACY];

/// Fused demosaic + CCM weight matrix `[3,4,1,1]`.
pub const DEMOSAIC_CCM_W: &str = "DEMOSAICCCMBLOCK/W";
/// Legacy spelling of [`DEMOSAIC_CCM_W`].
pub const DEMOSAIC_CCM_W_LEGACY: &str = "DemosaicCcmBlock/w";
/// Canonical-first aliases for fused weights.
pub const DEMOSAIC_CCM_W_ALIASES: [&str; 2] = [DEMOSAIC_CCM_W, DEMOSAIC_CCM_W_LEGACY];

/// Fused demosaic + CCM bias `[3]`.
pub const DEMOSAIC_CCM_B: &str = "DEMOSAICCCMBLOCK/B";
/// Legacy spelling of [`DEMOSAIC_CCM_B`].
pub const DEMOSAIC_CCM_B_LEGACY: &str = "DemosaicCcmBlock/b";
/// Canonical-first aliases for fused bias.
pub const DEMOSAIC_CCM_B_ALIASES: [&str; 2] = [DEMOSAIC_CCM_B, DEMOSAIC_CCM_B_LEGACY];

/// 3×3 colour-correction matrix.
pub const CCM_MATRIX: &str = "CCMBLOCK/CCM_MATRIX";
/// Production-instance spelling of [`CCM_MATRIX`] in older graphs.
pub const CCM_MATRIX_LEGACY: &str = "CcmBlock_ccm/matrix";
/// Generic-instance spelling of [`CCM_MATRIX`] in older graphs.
pub const CCM_MATRIX_LEGACY_DEFAULT: &str = "CcmBlock/matrix";

/// Per-channel colour-correction bias `[3]`.
pub const CCM_BIAS: &str = "CCMBLOCK/CCM_BIAS";
/// Production-instance spelling of [`CCM_BIAS`] in older graphs.
pub const CCM_BIAS_LEGACY: &str = "CcmBlock_ccm/bias";
/// Generic-instance spelling of [`CCM_BIAS`] in older graphs.
pub const CCM_BIAS_LEGACY_DEFAULT: &str = "CcmBlock/bias";

/// Black level per Bayer channel `[R, Gr, Gb, B]` — canonical name.
///
/// Emitted by the fused RAW heads (`isp_unpack_blc`, `isp_unpack16_blc`,
/// `isp_unpack32_blc`) and by standalone `BlcBlock` / `BlcBlock_dpc` when the
/// head is not fused. The engine writes `ACAMERA_SENSOR_BLACK_LEVEL_PATTERN`
/// here so black level reaches whichever head the profile actually uses.
pub const BLC_VALS: &str = "ISPBLKBLOCK/BLC_VALS";
/// Legacy spelling of [`BLC_VALS`] for the default `BlcBlock` namespace.
pub const BLC_VALS_LEGACY_DEFAULT: &str = "BlcBlock/blc_vals";
/// Legacy spelling for the bad-pixel `BlcBlock_dpc` namespace.
pub const BLC_VALS_LEGACY_DPC: &str = "BlcBlock_dpc/blc_vals";
/// Legacy spelling for the fused packed-INT32 head namespace.
pub const BLC_VALS_LEGACY_PACKED: &str = "isp_unpack_blc/blc_vals";
/// Legacy spelling for the fused INT16 head namespace.
pub const BLC_VALS_LEGACY_INT16: &str = "isp_unpack16_blc/blc_vals";
/// Legacy spelling for the fused INT32 head namespace.
pub const BLC_VALS_LEGACY_INT32: &str = "isp_unpack32_blc/blc_vals";
/// Canonical-first aliases the engine writes for black level.
pub const BLC_VALS_ALIASES: [&str; 6] = [
    BLC_VALS,
    BLC_VALS_LEGACY_DEFAULT,
    BLC_VALS_LEGACY_DPC,
    BLC_VALS_LEGACY_PACKED,
    BLC_VALS_LEGACY_INT16,
    BLC_VALS_LEGACY_INT32,
];

/// Tone curve inverse gamma exponent `[1]` — canonical name.
///
/// Emitted by `ToneBlock` (per-pixel pow) and `GammaBlock` (gamma block).
/// Writing both spellings ensures whichever block the profile uses gets the
/// value; names the session lacks are silently dropped by the flush.
pub const TONE_INV_GAMMA: &str = "ISPTONEBLOCK/INV_GAMMA";
/// Legacy spelling for `ToneBlock`.
pub const TONE_INV_GAMMA_LEGACY_TONE: &str = "ToneBlock/inv_gamma";
/// Legacy spelling for `GammaBlock`.
pub const TONE_INV_GAMMA_LEGACY_GAMMA: &str = "GammaBlock/inv_gamma";
/// Canonical-first aliases for tone inverse gamma.
pub const TONE_INV_GAMMA_ALIASES: [&str; 3] = [
    TONE_INV_GAMMA,
    TONE_INV_GAMMA_LEGACY_TONE,
    TONE_INV_GAMMA_LEGACY_GAMMA,
];

/// Tone contrast factor `[1]` — canonical name.
///
/// Emitted by `ToneBlock` and read/folded by `AutoContrastBlock`.
pub const TONE_CONTRAST: &str = "ISPTONEBLOCK/CONTRAST";
/// Legacy spelling for `ToneBlock`.
pub const TONE_CONTRAST_LEGACY_TONE: &str = "ToneBlock/contrast";
/// Legacy spelling folded into `AutoContrastBlock`.
pub const TONE_CONTRAST_LEGACY_AUTO: &str = "AutoContrastBlock/contrast_w";
/// Canonical-first aliases for tone contrast.
pub const TONE_CONTRAST_ALIASES: [&str; 3] = [
    TONE_CONTRAST,
    TONE_CONTRAST_LEGACY_TONE,
    TONE_CONTRAST_LEGACY_AUTO,
];

/// Tone brightness offset `[1]` — canonical name.
///
/// Emitted by `ToneBlock`.
pub const TONE_BRIGHTNESS: &str = "ISPTONEBLOCK/BRIGHTNESS";
/// Legacy spelling for `ToneBlock`.
pub const TONE_BRIGHTNESS_LEGACY: &str = "ToneBlock/brightness";
/// Canonical-first aliases for tone brightness.
pub const TONE_BRIGHTNESS_ALIASES: [&str; 2] = [TONE_BRIGHTNESS, TONE_BRIGHTNESS_LEGACY];

/// Tone exposure gain `[1]` — canonical name.
///
/// Emitted by `ToneBlock` (exposure Mul before contrast).
pub const TONE_EXPOSURE: &str = "ISPTONEBLOCK/EXPOSURE";
/// Legacy spelling for `ToneBlock`.
pub const TONE_EXPOSURE_LEGACY: &str = "ToneBlock/exposure";
/// Canonical-first aliases for tone exposure.
pub const TONE_EXPOSURE_ALIASES: [&str; 2] = [TONE_EXPOSURE, TONE_EXPOSURE_LEGACY];

/// Sensor white level `[1]` — canonical name.
///
/// Emitted by fused RAW heads (`isp_unpack_blc`, `isp_unpack16_blc`,
/// `isp_unpack32_blc`) and by `NormalizeBlock`.
pub const ISP_WHITE_LEVEL: &str = "ISP_PIPELINE/WHITE";
/// Legacy spelling for `isp_pipeline`.
pub const ISP_WHITE_LEVEL_LEGACY_PIPELINE: &str = "isp_pipeline/white";
/// Legacy spelling for `NormalizeBlock`.
pub const ISP_WHITE_LEVEL_LEGACY_NORMALIZE: &str = "NormalizeBlock/max_val";
/// Canonical-first aliases for white level.
pub const ISP_WHITE_LEVEL_ALIASES: [&str; 3] = [
    ISP_WHITE_LEVEL,
    ISP_WHITE_LEVEL_LEGACY_PIPELINE,
    ISP_WHITE_LEVEL_LEGACY_NORMALIZE,
];

/// Clip zero bound `[1]` — shared by `BlcBlock`, `ToneBlock`, `AutoContrastBlock`,
/// `DisplayBlock`, and others.
pub const CLIP_ZERO: &str = "ISP_CLIP/ZERO";
pub const CLIP_ZERO_LEGACY_BLC: &str = "BlcBlock/zero";
pub const CLIP_ZERO_LEGACY_TONE: &str = "ToneBlock/zero";
pub const CLIP_ZERO_LEGACY_AUTO: &str = "AutoContrastBlock/zero";
pub const CLIP_ZERO_LEGACY_DISPLAY: &str = "DisplayBlock/zero";
pub const CLIP_ZERO_ALIASES: [&str; 5] = [
    CLIP_ZERO,
    CLIP_ZERO_LEGACY_BLC,
    CLIP_ZERO_LEGACY_TONE,
    CLIP_ZERO_LEGACY_AUTO,
    CLIP_ZERO_LEGACY_DISPLAY,
];

/// Clip one bound `[1]` — shared by `BlcBlock`, `ToneBlock`, `AutoContrastBlock`,
/// `DisplayBlock`, and others.
pub const CLIP_ONE: &str = "ISP_CLIP/ONE";
pub const CLIP_ONE_LEGACY_BLC: &str = "BlcBlock/one";
pub const CLIP_ONE_LEGACY_TONE: &str = "ToneBlock/one";
pub const CLIP_ONE_LEGACY_AUTO: &str = "AutoContrastBlock/one";
pub const CLIP_ONE_LEGACY_DISPLAY: &str = "DisplayBlock/one";
pub const CLIP_ONE_ALIASES: [&str; 5] = [
    CLIP_ONE,
    CLIP_ONE_LEGACY_BLC,
    CLIP_ONE_LEGACY_TONE,
    CLIP_ONE_LEGACY_AUTO,
    CLIP_ONE_LEGACY_DISPLAY,
];

/// Bayer pattern phase `[1]` — shared by fused RAW heads and `BlcBlock`.
pub const BAYER_PATTERN: &str = "ISP_BAYER/PATTERN";
pub const BAYER_PATTERN_LEGACY_PACKED: &str = "isp_unpack_blc/bayer_pattern";
pub const BAYER_PATTERN_LEGACY_INT16: &str = "isp_unpack16_blc/bayer_pattern";
pub const BAYER_PATTERN_LEGACY_INT32: &str = "isp_unpack32_blc/bayer_pattern";
pub const BAYER_PATTERN_ALIASES: [&str; 4] = [
    BAYER_PATTERN,
    BAYER_PATTERN_LEGACY_PACKED,
    BAYER_PATTERN_LEGACY_INT16,
    BAYER_PATTERN_LEGACY_INT32,
];

/// The tensor name emitted by `CcmBlock` for the given instance.
///
/// The registry has one CCM namespace, so the production `ccm` instance and
/// the default instance converge on it. Other instances keep their unique
/// legacy namespace to avoid collisions in multi-CCM profiles.
pub fn ccm_matrix_name(instance: &str) -> String {
    if uses_canonical_ccm_namespace(instance) {
        CCM_MATRIX.to_owned()
    } else {
        legacy_ccm_tensor(instance, "matrix")
    }
}

/// Canonical-first names the engine writes for a CCM matrix.
pub fn ccm_matrix_aliases(instance: &str) -> Vec<String> {
    if uses_canonical_ccm_namespace(instance) {
        vec![
            CCM_MATRIX.to_owned(),
            CCM_MATRIX_LEGACY.to_owned(),
            CCM_MATRIX_LEGACY_DEFAULT.to_owned(),
        ]
    } else {
        vec![ccm_matrix_name(instance)]
    }
}

/// The tensor name emitted by `CcmBlock` for the given instance.
pub fn ccm_bias_name(instance: &str) -> String {
    if uses_canonical_ccm_namespace(instance) {
        CCM_BIAS.to_owned()
    } else {
        legacy_ccm_tensor(instance, "bias")
    }
}

/// Canonical-first names the engine writes for a CCM bias.
pub fn ccm_bias_aliases(instance: &str) -> Vec<String> {
    if uses_canonical_ccm_namespace(instance) {
        vec![
            CCM_BIAS.to_owned(),
            CCM_BIAS_LEGACY.to_owned(),
            CCM_BIAS_LEGACY_DEFAULT.to_owned(),
        ]
    } else {
        vec![ccm_bias_name(instance)]
    }
}

/// Find the first session tensor whose name appears in `names`.
///
/// Callers pass canonical-first alias lists so a graph carrying both spellings
/// binds the canonical tensor once.
pub fn find_first<'a, T, S: AsRef<str>>(pool: &'a [(String, T)], names: &[S]) -> Option<&'a T> {
    names.iter().find_map(|name| {
        pool.iter()
            .find(|(candidate, _)| candidate == name.as_ref())
            .map(|(_, tensor)| tensor)
    })
}

fn uses_canonical_ccm_namespace(instance: &str) -> bool {
    instance.is_empty() || instance == "ccm"
}

fn legacy_ccm_tensor(instance: &str, suffix: &str) -> String {
    if instance.is_empty() {
        format!("CcmBlock/{suffix}")
    } else {
        format!("CcmBlock_{instance}/{suffix}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_match_the_canonical_registry() {
        assert_eq!(BAYER_WB_GAINS, "BAYERWBBLOCK/BAYER_GAINS");
        assert_eq!(DEMOSAIC_CCM_W, "DEMOSAICCCMBLOCK/W");
        assert_eq!(DEMOSAIC_CCM_B, "DEMOSAICCCMBLOCK/B");
        assert_eq!(CCM_MATRIX, "CCMBLOCK/CCM_MATRIX");
        assert_eq!(CCM_BIAS, "CCMBLOCK/CCM_BIAS");
    }

    #[test]
    fn find_first_prefers_the_canonical_alias() {
        let pool = vec![
            (BAYER_WB_GAINS_LEGACY.to_owned(), 1u8),
            (BAYER_WB_GAINS.to_owned(), 2u8),
        ];
        assert_eq!(find_first(&pool, &BAYER_WB_GAINS_ALIASES), Some(&2));
        assert_eq!(find_first(&pool, &[BAYER_WB_GAINS_LEGACY]), Some(&1));
        assert_eq!(find_first(&pool, &["absent"]), None);
    }

    #[test]
    fn ccm_names_converge_only_for_registry_instances() {
        assert_eq!(ccm_matrix_name(""), CCM_MATRIX);
        assert_eq!(ccm_matrix_name("ccm"), CCM_MATRIX);
        assert_eq!(ccm_matrix_name("lsc"), "CcmBlock_lsc/matrix");
        assert_eq!(ccm_bias_name("ccm"), CCM_BIAS);
        assert_eq!(ccm_bias_name("lsc"), "CcmBlock_lsc/bias");

        let matrix = ccm_matrix_aliases("ccm");
        assert_eq!(matrix[0], CCM_MATRIX);
        assert!(matrix.contains(&CCM_MATRIX_LEGACY.to_owned()));
        assert!(matrix.contains(&CCM_MATRIX_LEGACY_DEFAULT.to_owned()));
        assert_eq!(ccm_matrix_aliases("lsc"), vec!["CcmBlock_lsc/matrix"]);
    }
}
