//! CCM (Color Correction Matrix) computation and sanitization engine.
//!
//! Ported from `com.camcore.isp.pipeline.controller.CcmEngine` (Java).
//!
//! Provides:
//! - Factory helpers: `identity_ccm`, `default_sensor_ccm`, `zero_ccm`
//! - Quadratic CCT-based CCM interpolation: `quadratic_ccm`, `select_ccm`
//! - CCM goalkeeper: `sanitize_ccm` enforcing ISP invariants

/// Typical mobile sensor raw gray response (R:G:B ratios from HW ISP AWB gains).
/// Derived from Camera2 hardware ISP: hw_awb_rg ≈ 2.2, hw_awb_bg ≈ 1.7
/// → sensor raw gray = `[1/2.2, 1.0, 1/1.7]` ≈ `[0.46, 1.0, 0.60]`.
const SENSOR_GRAY_R: f32 = 0.46;
const SENSOR_GRAY_G: f32 = 1.0;
const SENSOR_GRAY_B: f32 = 0.60;

/// Quadratic coefficients for CCT-based CCM interpolation.
///
/// ccmelement`[i]` = a2`[i]` * cct² + a1`[i]` * cct + a0`[i]`
/// 9 elements × 3 coefficients = 27 floats.
///
/// Fitted from 3-point targets at 2000K, 6500K, 10000K.
/// All off-diagonals are negative across full CCT range — no purple contamination.
const CCM_QUADRATIC: [f32; 27] = [
    // a2                    a1                      a0
    4.960_317_5e-9,
    -1.532_738_1e-4,
    2.586_706_4, // R-R
    -5.357_142_7e-9,
    1.455_357_1e-4,
    -1.269_642_8, // R-G
    3.968_254e-10,
    7.738_095_5e-6,
    -3.170_634_8e-1, // R-B
    2.182_539_7e-9,
    -7.440_476e-6,
    -3.438_492e-1, // G-R
    -6.150_793_7e-9,
    3.005_952_4e-5,
    1.714_484_1, // G-G
    3.968_254e-9,
    -2.261_904_8e-5,
    -3.706_349e-1, // G-B
    -3.968_254e-10,
    -7.738_095_5e-6,
    -3.293_651e-2, // B-R
    2.380_952_3e-9,
    -5.357_142_8e-5,
    -2.023_809_6e-1, // B-G
    -1.984_127e-9,
    6.130_952_5e-5,
    1.235_317_5, // B-B
];

/// Clamp feedback flags.
pub const CCM_R_OUT_NEGATIVE: i32 = 0x01;
pub const CCM_AWB_UNFEASIBLE: i32 = 0x02;
pub const CCM_DIAGONAL_EXTREME: i32 = 0x04;

// ── Factory helpers ──

/// Identity 3×3 CCM (row-major).
pub fn identity_ccm() -> [f32; 9] {
    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
}

/// Zero offset 3×3 CCM (row-major).
pub fn zero_ccm() -> [f32; 9] {
    [0.0; 9]
}

/// Default sensor→sRGB CCM for typical mobile Bayer sensors.
/// Each row sums ≈ 1.0 to preserve white point for neutral input.
pub fn default_sensor_ccm() -> [f32; 9] {
    [1.40, -0.30, -0.10, -0.25, 1.45, -0.20, -0.05, -0.30, 1.35]
}

// ── CCT-based CCM interpolation ──

/// Compute CCM from CCT using quadratic fit.
///
/// @param cct Correlated color temperature in Kelvin (2000-10000)
/// @return 3×3 CCM matrix (row-major, row-normalized)
pub fn quadratic_ccm(cct: i32) -> [f32; 9] {
    let cct_f = cct as f32;
    let mut raw = [0.0f32; 9];
    for i in 0..9 {
        let a2 = CCM_QUADRATIC[i * 3];
        let a1 = CCM_QUADRATIC[i * 3 + 1];
        let a0 = CCM_QUADRATIC[i * 3 + 2];
        raw[i] = a2 * cct_f * cct_f + a1 * cct_f + a0;
    }

    // Row-normalize: each row sums to 1.0
    let mut result = [0.0f32; 9];
    for row in 0..3 {
        let base = row * 3;
        let row_sum: f32 = raw[base..base + 3].iter().sum();
        if row_sum > 0.001 {
            for col in 0..3 {
                result[base + col] = raw[base + col] / row_sum;
            }
        } else {
            // Fallback to identity for degenerate case
            result[base + row] = 1.0;
        }
    }
    result
}

/// Select (and optionally interpolate) CCM for the given CCT.
///
/// Base estimate: quadratic polynomial fit from 3-point targets.
///
/// @param cct Correlated color temperature in Kelvin
/// @return 3×3 CCM matrix (row-major, sanitized)
pub fn select_ccm(cct: i32) -> [f32; 9] {
    let mut quad = quadratic_ccm(cct);
    sanitize_ccm(&mut quad, cct, "quad", None, None);
    quad
}

// ── CCM goalkeeper ──

/// Validate and clamp a 3×3 CCM matrix (row-major) against ISP sanity criteria.
///
/// Enforced invariants:
/// 1. Each row sums to 1.0 (neutral preservation).
/// 2. R-B < 0 and B-R < 0 (no purple cross-contamination).
/// 3. All off-diagonals ∈ `[-1.0, 0]` (no positive cross-talk).
/// 4. Main diagonal ∈ `[0.5, 3.0]` (reasonable gain range).
/// 5. AWB feasibility: for sensor gray `[0.46, 1.0, 0.60]`,
///    required R_gain, B_gain ∈ `[0.5, 3.0]` (no AWB clamp saturation).
///
/// @param matrix 3×3 row-major CCM (modified in place)
/// @param cct    CCT hint for diagnostics (default 5500)
/// @param tag    Short tag for log messages
/// @param flags  Optional mutable reference to clamp flags (set when clamping occurs)
/// @param cct_ref  Optional mutable reference to clamp CCT ref
pub fn sanitize_ccm(
    matrix: &mut [f32; 9],
    cct: i32,
    tag: &str,
    flags: Option<&mut i32>,
    cct_ref: Option<&mut i32>,
) {
    let orig = *matrix;
    let mut clamped = false;

    // Clone the option references into local booleans so we don't need
    // to destructure flags/cct_ref multiple times (they can't be moved).
    let mut flag_value = 0i32;
    let mut cct_ref_value = 0i32;
    let has_flags = flags.is_some();
    let has_cct_ref = cct_ref.is_some();
    if let Some(f) = flags.as_ref() {
        flag_value = **f;
    }
    if let Some(c) = cct_ref.as_ref() {
        cct_ref_value = **c;
    }

    macro_rules! or_flag {
        ($bit:expr) => {
            flag_value |= $bit;
        };
    }
    macro_rules! set_cct {
        () => {
            if has_cct_ref {
                cct_ref_value = cct;
            }
        };
    }

    // ── 1. Row-normalize (skip if row is purely diagonal) ──
    for r in 0..3 {
        let base = r * 3;
        let row_sum: f32 = matrix[base..base + 3].iter().sum();
        if (row_sum - 1.0).abs() > 0.0001 {
            if row_sum > 0.001 {
                let diag_idx = base + r;
                let off_diag_sum =
                    matrix[base] + matrix[base + 1] + matrix[base + 2] - matrix[diag_idx];
                if off_diag_sum.abs() < 0.001 {
                    // Diagonal-only row — skip normalization to preserve per-channel gain
                    continue;
                }
                let inv = 1.0 / row_sum;
                for c in 0..3 {
                    matrix[base + c] *= inv;
                }
                clamped = true;
            } else {
                // Degenerate row (zero or negative sum): fall back to identity row
                matrix[base] = 0.0;
                matrix[base + 1] = 0.0;
                matrix[base + 2] = 0.0;
                matrix[base + r] = 1.0;
                clamped = true;
            }
        }
    }

    // ── 2. Enforce negative R-B and B-R (anti-purple) ──
    if matrix[2] > 0.001 {
        matrix[2] = -0.05;
        clamped = true;
    }
    if matrix[6] > 0.001 {
        matrix[6] = -0.05;
        clamped = true;
    }

    // ── 3. Clamp off-diagonals to [-1.0, 0] ──
    for i in 0..9 {
        if i != 0 && i != 4 && i != 8 {
            if matrix[i] > 0.0 {
                matrix[i] = 0.0;
                clamped = true;
            }
            if matrix[i] < -1.0 {
                matrix[i] = -1.0;
                clamped = true;
            }
        }
    }

    // ── 4. Clamp main diagonal to [0.5, 3.0] ──
    let mut diagonal_extreme = false;
    for &idx in &[0, 4, 8] {
        if matrix[idx] < 0.5 {
            matrix[idx] = 0.5;
            clamped = true;
            diagonal_extreme = true;
        }
        if matrix[idx] > 3.0 {
            matrix[idx] = 3.0;
            clamped = true;
            diagonal_extreme = true;
        }
    }
    if diagonal_extreme {
        or_flag!(CCM_DIAGONAL_EXTREME);
        set_cct!();
    }

    // ── 5. Re-normalize after clamping (skip diagonal-only rows) ──
    for r in 0..3 {
        let base = r * 3;
        let row_sum: f32 = matrix[base..base + 3].iter().sum();
        if (row_sum - 1.0).abs() > 0.0001 && row_sum > 0.001 {
            let diag_idx = base + r;
            let off_diag_sum =
                matrix[base] + matrix[base + 1] + matrix[base + 2] - matrix[diag_idx];
            if off_diag_sum.abs() < 0.001 {
                continue;
            }
            let inv = 1.0 / row_sum;
            for c in 0..3 {
                matrix[base + c] *= inv;
            }
        }
    }

    // ── 6. AWB feasibility: sensor gray → check required gains ──
    let r_out = matrix[0] * SENSOR_GRAY_R + matrix[1] * SENSOR_GRAY_G + matrix[2] * SENSOR_GRAY_B;
    let g_out = matrix[3] * SENSOR_GRAY_R + matrix[4] * SENSOR_GRAY_G + matrix[5] * SENSOR_GRAY_B;
    let b_out = matrix[6] * SENSOR_GRAY_R + matrix[7] * SENSOR_GRAY_G + matrix[8] * SENSOR_GRAY_B;

    // Flag: R output for sensor gray is negative → CCT estimate is fundamentally wrong
    if r_out < 0.001 {
        or_flag!(CCM_R_OUT_NEGATIVE);
        set_cct!();
    }

    // Scale R row so R_gain = g_out / r_out ≈ 2.0 (midpoint of feasible range)
    let mut awb_unfeasible = false;
    if r_out > 0.001 {
        let r_gain = g_out / r_out;
        if !(0.5..=3.0).contains(&r_gain) {
            let target_r_out = g_out * 0.5; // yields R_gain = 2.0
            let scale = target_r_out / r_out;
            matrix[0] *= scale;
            matrix[1] *= scale;
            matrix[2] *= scale;
            // Re-normalize R row
            let off_diag_r = matrix[1] + matrix[2];
            if off_diag_r.abs() > 0.001 {
                let r_sum = matrix[0] + matrix[1] + matrix[2];
                if (r_sum - 1.0).abs() > 0.0001 && r_sum > 0.001 {
                    let inv = 1.0 / r_sum;
                    matrix[0] *= inv;
                    matrix[1] *= inv;
                    matrix[2] *= inv;
                }
            }
            clamped = true;
            awb_unfeasible = true;
        }
    }

    // Scale B row so B_gain = g_out / b_out ≈ 2.0
    if b_out > 0.001 {
        let b_gain = g_out / b_out;
        if !(0.5..=3.0).contains(&b_gain) {
            let target_b_out = g_out * 0.5;
            let scale = target_b_out / b_out;
            matrix[6] *= scale;
            matrix[7] *= scale;
            matrix[8] *= scale;
            // Re-normalize B row
            let off_diag_b = matrix[7] + matrix[8];
            if off_diag_b.abs() > 0.001 {
                let b_sum = matrix[6] + matrix[7] + matrix[8];
                if (b_sum - 1.0).abs() > 0.0001 && b_sum > 0.001 {
                    let inv = 1.0 / b_sum;
                    matrix[6] *= inv;
                    matrix[7] *= inv;
                    matrix[8] *= inv;
                }
            }
            clamped = true;
            awb_unfeasible = true;
        }
    }
    if awb_unfeasible {
        or_flag!(CCM_AWB_UNFEASIBLE);
        set_cct!();
    }

    if clamped {
        let fixed_r_out =
            matrix[0] * SENSOR_GRAY_R + matrix[1] * SENSOR_GRAY_G + matrix[2] * SENSOR_GRAY_B;
        let fixed_g_out =
            matrix[3] * SENSOR_GRAY_R + matrix[4] * SENSOR_GRAY_G + matrix[5] * SENSOR_GRAY_B;
        let fixed_b_out =
            matrix[6] * SENSOR_GRAY_R + matrix[7] * SENSOR_GRAY_G + matrix[8] * SENSOR_GRAY_B;
        let fixed_rg = if fixed_r_out > 0.0 {
            fixed_g_out / fixed_r_out
        } else {
            0.0
        };
        let fixed_bg = if fixed_b_out > 0.0 {
            fixed_g_out / fixed_b_out
        } else {
            0.0
        };
        let src_rg = if r_out > 0.0 { g_out / r_out } else { 0.0 };
        let src_bg = if b_out > 0.0 { g_out / b_out } else { 0.0 };
        log::warn!(
            "CcmGuard {} CCT={}\n  IN:  [{:+.4},{:+.4},{:+.4}]  OUT: [{:+.4},{:+.4},{:+.4}]\n  IN:  [{:+.4},{:+.4},{:+.4}]  OUT: [{:+.4},{:+.4},{:+.4}]\n  IN:  [{:+.4},{:+.4},{:+.4}]  OUT: [{:+.4},{:+.4},{:+.4}]\n  R_gain: {:.2}->{:.2}  B_gain: {:.2}->{:.2}",
            tag, cct,
            orig[0], orig[1], orig[2], matrix[0], matrix[1], matrix[2],
            orig[3], orig[4], orig[5], matrix[3], matrix[4], matrix[5],
            orig[6], orig[7], orig[8], matrix[6], matrix[7], matrix[8],
            src_rg, fixed_rg, src_bg, fixed_bg,
        );
    }

    // Write back flag values
    if has_flags {
        if let Some(f) = flags {
            *f = flag_value;
        }
    }
    if has_cct_ref {
        if let Some(c) = cct_ref {
            *c = cct_ref_value;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_ccm() {
        let ccm = identity_ccm();
        assert_eq!(ccm[0], 1.0);
        assert_eq!(ccm[4], 1.0);
        assert_eq!(ccm[8], 1.0);
        for i in 0..9 {
            if i != 0 && i != 4 && i != 8 {
                assert_eq!(ccm[i], 0.0);
            }
        }
    }

    #[test]
    fn test_default_sensor_ccm_row_sums() {
        let ccm = default_sensor_ccm();
        for row in 0..3 {
            let base = row * 3;
            let sum: f32 = ccm[base..base + 3].iter().sum();
            assert!((sum - 1.0).abs() < 0.01, "Row {} sum = {}", row, sum);
        }
    }

    #[test]
    fn test_quadratic_ccm_daylight() {
        let ccm = quadratic_ccm(5500);
        // All rows should sum to ≈1.0
        for row in 0..3 {
            let base = row * 3;
            let sum: f32 = ccm[base..base + 3].iter().sum();
            assert!(
                (sum - 1.0).abs() < 0.01,
                "Row {} sum = {} at 5500K",
                row,
                sum
            );
        }
        // Off-diagonals should be negative
        for i in 0..9 {
            if i != 0 && i != 4 && i != 8 {
                assert!(
                    ccm[i] < 0.0,
                    "Element {} = {} should be negative at 5500K",
                    i,
                    ccm[i]
                );
            }
        }
    }

    #[test]
    fn test_quadratic_ccm_warm_cool() {
        let warm = quadratic_ccm(3000);
        let cool = quadratic_ccm(8000);
        // CCM varies with CCT — warm and cool should differ
        let diff = warm
            .iter()
            .zip(cool.iter())
            .map(|(a, b)| (a - b).abs())
            .sum::<f32>();
        assert!(
            diff > 0.01,
            "Warm and cool CCM should differ: diff={}",
            diff
        );
    }

    #[test]
    fn test_sanitize_identity() {
        let mut ccm = identity_ccm();
        // Identity should pass through unmodified
        sanitize_ccm(&mut ccm, 5500, "test", None, None);
        assert_eq!(ccm, identity_ccm());
    }

    #[test]
    fn test_sanitize_fixes_purple() {
        let mut ccm = [1.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0, 1.0];
        sanitize_ccm(&mut ccm, 5500, "test", None, None);
        // R-B (index 2) and B-R (index 6) should be negative
        assert!(ccm[2] < 0.0, "R-B should be negative: {}", ccm[2]);
        assert!(ccm[6] < 0.0, "B-R should be negative: {}", ccm[6]);
    }

    #[test]
    fn test_sanitize_sets_clamp_flags() {
        let mut ccm = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]; // very weak diagonal
        let mut flags: i32 = 0;
        let mut cct_ref: i32 = 5500;
        sanitize_ccm(&mut ccm, 6000, "test", Some(&mut flags), Some(&mut cct_ref));
        assert!(
            flags & CCM_DIAGONAL_EXTREME != 0,
            "DIAGONAL_EXTREME flag should be set"
        );
        assert_eq!(cct_ref, 6000);
    }

    #[test]
    fn test_select_ccm() {
        for cct in [3000, 5500, 8000].iter() {
            let ccm = select_ccm(*cct);
            // All rows should sum to ≈1.0
            for row in 0..3 {
                let base = row * 3;
                let sum: f32 = ccm[base..base + 3].iter().sum();
                assert!(
                    (sum - 1.0).abs() < 0.01,
                    "Row {} sum = {} at {}K",
                    row,
                    sum,
                    cct
                );
            }
            // R-B and B-R should be negative
            assert!(ccm[2] < 0.0, "R-B = {} at {}K", ccm[2], cct);
            assert!(ccm[6] < 0.0, "B-R = {} at {}K", ccm[6], cct);
        }
    }

    #[test]
    fn test_sanitize_awb_feasibility() {
        // A CCM with R-R extreme and strong positive R-G that requires AWB fixup
        // After R-R clamped to 3.0, R-G=1.0 is clamped to 0, then R row = [3.0, 0, 0]
        // But after row normalization (diagonal-only detected, skip), r_out = 3.0*0.46 = 1.38
        // r_gain = 1.0/1.38 = 0.72 — still in range.
        // Use a different matrix where G row also affects: make G-R > 0 so G row doesn't
        // produce neutral output, making AWB gains unfeasible.
        // Actually, let's use a matrix where r_out is huge and g_out is small.
        let mut ccm = [
            3.0, -2.0, 0.0, // R row: off-diag = -2.0, one-diag = 3.0
            0.0, 1.0, 0.0, // G row: identity
            0.0, 0.0, 1.0, // B row: identity
        ];
        let mut flags = 0i32;
        // R row sum = 3.0 - 2.0 + 0 = 1.0. Normalize: (3/1, -2/1, 0/1) = (3.0, -2.0, 0.0)
        // r_out = 3.0*0.46 + (-2.0)*1.0 + 0*0.60 = 1.38 - 2.0 = -0.62 < 0.001
        // This should trigger R_OUT_NEGATIVE
        sanitize_ccm(&mut ccm, 4000, "awb_test", Some(&mut flags), None);
        // Should have set AWB_UNFEASIBLE flag (r_gain would be > 3.0)
        assert!(
            flags & CCM_AWB_UNFEASIBLE != 0,
            "AWB_UNFEASIBLE should be set (flags=0x{:x})",
            flags
        );
        // R row should now give non-negative output for sensor gray
        let r_out = ccm[0] * SENSOR_GRAY_R + ccm[1] * SENSOR_GRAY_G + ccm[2] * SENSOR_GRAY_B;
        assert!(r_out >= 0.0, "R output should be non-negative: {}", r_out);
    }
}
