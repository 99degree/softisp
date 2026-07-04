//! Pure-algo match computations: color ratios, gamma estimation, CCT, CCM.
//!
//! Ported from `com.camcore.isp.pipeline.controller.MatchComputer` (Kotlin).
//! No state, no Android dependencies.

/// Sample average RGB from a center crop of an ARGB pixel array.
pub fn sample_center_rgb(pixels: &[u32], count: usize) -> [f32; 3] {
    let n = if count > 0 { count.min(pixels.len()) } else { pixels.len() };
    if n == 0 {
        return [0.0; 3];
    }
    let mut sr: u64 = 0;
    let mut sg: u64 = 0;
    let mut sb: u64 = 0;
    for &p in pixels.iter().take(n) {
        sr += ((p >> 16) & 0xff) as u64;
        sg += ((p >> 8) & 0xff) as u64;
        sb += (p & 0xff) as u64;
    }
    let n_f = n as f32 * 255.0;
    [sr as f32 / n_f, sg as f32 / n_f, sb as f32 / n_f]
}

/// Color ratios normalized to G=1: (R/G, B/G).
pub fn color_ratios(rgb: [f32; 3]) -> (f32, f32) {
    let eps = 0.001;
    (rgb[0] / (rgb[1] + eps), rgb[2] / (rgb[1] + eps))
}

/// Correction factors: what to multiply `our` ratio by to match `ref`.
/// Returns (corr_r, corr_b) clamped to `[0.5, 2.0]`.
pub fn correction_ratios(ref_rg: f32, ref_bg: f32, our_rg: f32, our_bg: f32) -> (f32, f32) {
    (
        (ref_rg / our_rg).clamp(0.5, 2.0),
        (ref_bg / our_bg).clamp(0.5, 2.0),
    )
}

/// Build a diagonal 3×3 CCM from R-correction and B-correction (G remains 1).
pub fn diagonal_ccm(corr_r: f32, corr_b: f32) -> [f32; 9] {
    [
        corr_r, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, corr_b,
    ]
}

/// Estimate gamma from luminance ratio.
pub fn estimate_gamma(lum_ref: f32, lum_our: f32) -> f32 {
    let eps = 0.001;
    if lum_ref > eps && lum_our > eps {
        (lum_our.ln() / lum_ref.ln()).clamp(0.5, 4.0)
    } else {
        1.0
    }
}

/// Luminance from average RGB (simple mean).
pub fn luminance(rgb: [f32; 3]) -> f32 {
    (rgb[0] + rgb[1] + rgb[2]) / 3.0
}

/// Rough CCT estimate from R/G and B/G ratios.
pub fn estimate_cct(rg: f32, bg: f32) -> i32 {
    let cct = 4400.0 - 1300.0 * bg + 2100.0 * rg;
    (cct as i32).clamp(2000, 10000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_center_rgb_empty() {
        assert_eq!(sample_center_rgb(&[], 0), [0.0; 3]);
    }

    #[test]
    fn test_sample_center_rgb_white() {
        let pixels = vec![0xFFFFFFFFu32; 100];
        let rgb = sample_center_rgb(&pixels, 100);
        assert!((rgb[0] - 1.0).abs() < 0.01);
        assert!((rgb[1] - 1.0).abs() < 0.01);
        assert!((rgb[2] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_color_ratios() {
        let (rg, bg) = color_ratios([0.6, 0.3, 0.2]);
        assert!((rg - 2.0).abs() < 0.01);
        assert!((bg - 0.666).abs() < 0.01);
    }

    #[test]
    fn test_correction_ratios_in_range() {
        let (cr, cb) = correction_ratios(1.5, 1.2, 1.0, 1.0);
        assert!((cr - 1.5).abs() < 0.01);
        assert!((cb - 1.2).abs() < 0.01);
    }

    #[test]
    fn test_correction_ratios_clamped() {
        let (cr, cb) = correction_ratios(10.0, 10.0, 1.0, 1.0);
        assert!((cr - 2.0).abs() < 0.01);
        assert!((cb - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_diagonal_ccm() {
        let ccm = diagonal_ccm(1.5, 1.2);
        assert_eq!(ccm[0], 1.5);
        assert_eq!(ccm[4], 1.0);
        assert_eq!(ccm[8], 1.2);
        assert_eq!(ccm[1], 0.0);
    }

    #[test]
    fn test_estimate_gamma() {
        let g = estimate_gamma(0.5, 0.25);
        assert!(g > 0.5 && g < 4.0);
    }

    #[test]
    fn test_estimate_cct() {
        let cct = estimate_cct(0.5, 0.4);
        assert!(cct >= 2000 && cct <= 10000);
    }
}
