//! Calibration statistics — comprehensive per-frame sensor metadata.
//!
//! Ported from `CalibrationBlock.kt` (Java, 6004 bytes).
//!
//! Extracts quad-level statistics from raw Bayer CFA data for use by the
//! learner/calibration system:
//!
//! Output: `[CalibrationStats; 24]`
//!   [0:4]   quad_means     — mean of each quad channel (TL, TR, BL, BR)
//!   [4:8]   quad_vars      — variance (read noise per quad)
//!   [8:12]  quad_mins      — minimum value per quad (DSNU)
//!   [12:16] quad_maxs      — maximum value per quad (saturation check)
//!   [16:20] quad_ranges    — (max-min)/max per quad (dynamic range)
//!   [20]    frame_lum      — overall mean luminance (avg of 4 means)
//!   [21]    frame_noise    — mean of 4 variances (overall noise floor)
//!   [22]    frame_min      — minimum across all quads (darkest pixel)
//!   [23]    frame_max      — maximum across all quads (brightest pixel)

use cam_types::BayerPattern;

/// Calibration statistics array — 24 floats.
pub struct CalibrationStats(pub [f32; 24]);

impl CalibrationStats {
    pub fn quad_means(&self) -> &[f32] { &self.0[0..4] }
    pub fn quad_vars(&self) -> &[f32] { &self.0[4..8] }
    pub fn quad_mins(&self) -> &[f32] { &self.0[8..12] }
    pub fn quad_maxs(&self) -> &[f32] { &self.0[12..16] }
    pub fn quad_ranges(&self) -> &[f32] { &self.0[16..20] }
    pub fn frame_luminance(&self) -> f32 { self.0[20] }
    pub fn frame_noise(&self) -> f32 { self.0[21] }
    pub fn frame_min(&self) -> f32 { self.0[22] }
    pub fn frame_max(&self) -> f32 { self.0[23] }
}

/// Extract calibration statistics from raw Bayer data.
///
/// The input is a `[1, 4, H/2, W/2]` NCHW float tensor representing the
/// 4 Bayer quadrants (TL=R, TR=Gr, BL=Gb, BR=B) BEFORE black level correction,
/// matching the CFA block output format.
///
/// Returns 24-element stats array as described above.
pub fn compute_calibration_stats(
    cfa_data: &[f32],
    cfa_channels: usize,
    cfa_height: usize,
    cfa_width: usize,
    _bayer: BayerPattern,
) -> CalibrationStats {
    assert!(
        cfa_channels >= 4,
        "CFA data must have at least 4 channels (quadrants)"
    );

    let quad_stride = cfa_height * cfa_width; // elements per quad channel
    let n = (cfa_height * cfa_width) as f32;

    if n <= 0.0 {
        return CalibrationStats([0.0f32; 24]);
    }

    let mut means = [0.0f32; 4];
    let mut vars = [0.0f32; 4];
    let mut mins = [f32::MAX; 4];
    let mut maxs = [f32::MIN; 4];

    // First pass: compute means and mins/maxs
    for ch in 0..4 {
        let offset = ch * quad_stride;
        let mut sum = 0.0f64;
        for i in 0..quad_stride {
            let val = cfa_data[offset + i];
            sum += val as f64;
            if val < mins[ch] {
                mins[ch] = val;
            }
            if val > maxs[ch] {
                maxs[ch] = val;
            }
        }
        means[ch] = (sum / n as f64) as f32;
    }

    // Second pass: compute variances
    for ch in 0..4 {
        let offset = ch * quad_stride;
        let mean = means[ch] as f64;
        let mut sum_sq = 0.0f64;
        for i in 0..quad_stride {
            let diff = cfa_data[offset + i] as f64 - mean;
            sum_sq += diff * diff;
        }
        vars[ch] = (sum_sq / n as f64) as f32;
    }

    // Dynamic range per quad: (max - min) / max
    let mut ranges = [0.0f32; 4];
    for ch in 0..4 {
        ranges[ch] = if maxs[ch] > 0.0 {
            (maxs[ch] - mins[ch]) / maxs[ch]
        } else {
            0.0
        };
    }

    // Frame-level aggregates
    let frame_lum: f32 = means.iter().sum::<f32>() / 4.0;
    let frame_noise: f32 = vars.iter().sum::<f32>() / 4.0;
    let frame_min: f32 = mins.iter().fold(f32::MAX, |a, &b| a.min(b));
    let frame_max: f32 = maxs.iter().fold(f32::MIN, |a, &b| a.max(b));

    let mut data = [0.0f32; 24];
    data[0..4].copy_from_slice(&means);
    data[4..8].copy_from_slice(&vars);
    data[8..12].copy_from_slice(&mins);
    data[12..16].copy_from_slice(&maxs);
    data[16..20].copy_from_slice(&ranges);
    data[20] = frame_lum;
    data[21] = frame_noise;
    data[22] = frame_min;
    data[23] = frame_max;

    CalibrationStats(data)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Create a synthetic 2×2 CFA (4 channels, 1x1 spatial each).
    fn make_cfa(quad_vals: &[f32; 4]) -> Vec<f32> {
        // [1, 4, 1, 1] → flat [4 * 1 * 1]
        quad_vals.to_vec()
    }

    #[test]
    fn test_uniform_cfa() {
        let cfa = make_cfa(&[0.5, 0.5, 0.5, 0.5]);
        let stats = compute_calibration_stats(&cfa, 4, 1, 1, BayerPattern::Rggb);
        assert!((stats.quad_means()[0] - 0.5).abs() < 1e-6);
        // Uniform → zero variance
        assert!((stats.quad_vars()[0]).abs() < 1e-6);
        // min == max
        assert!((stats.quad_mins()[0] - 0.5).abs() < 1e-6);
        assert!((stats.quad_maxs()[0] - 0.5).abs() < 1e-6);
        // range = 0
        assert!((stats.quad_ranges()[0]).abs() < 1e-6);
        // frame lum = 0.5
        assert!((stats.frame_luminance() - 0.5).abs() < 1e-6);
        assert!((stats.frame_noise()).abs() < 1e-6);
    }

    #[test]
    fn test_varied_quad() {
        // Simulate a single 2×2 Bayer quad with varying values
        let mut cfa = Vec::with_capacity(4 * 4);
        // 4 channels, 2×2 spatial = 4 elements each
        for ch in 0..4 {
            for i in 0..4 {
                cfa.push(0.1 + ch as f32 * 0.2 + i as f32 * 0.01);
            }
        }
        let stats = compute_calibration_stats(&cfa, 4, 2, 2, BayerPattern::Rggb);
        // Channel 0 (R) vals: 0.1, 0.11, 0.12, 0.13 → mean=0.115
        assert!((stats.quad_means()[0] - 0.115).abs() < 0.01);
        // Channel 3 (B) vals: 0.7, 0.71, 0.72, 0.73 → mean=0.715
        assert!((stats.quad_means()[3] - 0.715).abs() < 0.01);
        // Mins/maxs
        assert!((stats.quad_mins()[0] - 0.1).abs() < 0.01);
        assert!((stats.quad_maxs()[3] - 0.73).abs() < 0.01);
        // Frame-level
        let expected_lum = (0.115 + 0.315 + 0.515 + 0.715) / 4.0; // 0.415
        assert!((stats.frame_luminance() - expected_lum).abs() < 0.01);
    }

    #[test]
    fn test_empty_data() {
        let stats = compute_calibration_stats(&[], 4, 0, 0, BayerPattern::Rggb);
        assert_eq!(stats.0, [0.0f32; 24]);
    }

    #[test]
    fn test_saturated_quad() {
        // All maxed out at 1.0
        let cfa = make_cfa(&[1.0, 1.0, 1.0, 1.0]);
        let stats = compute_calibration_stats(&cfa, 4, 1, 1, BayerPattern::Rggb);
        assert!((stats.quad_maxs()[0] - 1.0).abs() < 1e-6);
        assert!((stats.quad_ranges()[0]).abs() < 1e-6); // no range when all same
    }

    #[test]
    fn test_full_range() {
        // Mix of dark and bright across quads
        let mut cfa = Vec::with_capacity(4 * 9);
        for ch in 0..4 {
            for i in 0..9 {
                let v = match ch {
                    0 => 0.01,      // dark R
                    1 => 0.5,       // mid Gr
                    2 => 0.5,       // mid Gb
                    3 => 0.99,      // bright B
                    _ => 0.0,
                };
                cfa.push(v + i as f32 * 0.001);
            }
        }
        let stats = compute_calibration_stats(&cfa, 4, 3, 3, BayerPattern::Rggb);
        // R mean should be ~0.014, B mean ~0.994
        assert!(stats.quad_means()[0] > 0.01 && stats.quad_means()[0] < 0.02);
        assert!(stats.quad_means()[3] > 0.98 && stats.quad_means()[3] < 1.0);
        // Frame min should be near 0.01, max near 0.99
        assert!(stats.frame_min() < 0.02);
        assert!(stats.frame_max() > 0.98);
    }

    #[test]
    fn test_noise_detection() {
        // High variance in one quad
        let mut cfa = Vec::with_capacity(4 * 4);
        for ch in 0..4 {
            let base = match ch {
                0 => 0.3,
                _ => 0.5,
            };
            let noise = if ch == 0 { 0.2 } else { 0.01 };
            for i in 0..4 {
                cfa.push(base + noise * (i as f32 - 1.5));
            }
        }
        let stats = compute_calibration_stats(&cfa, 4, 2, 2, BayerPattern::Rggb);
        // R quad should have higher variance
        assert!(
            stats.quad_vars()[0] > stats.quad_vars()[1],
            "R quad variance ({}) should be > G quad variance ({})",
            stats.quad_vars()[0],
            stats.quad_vars()[1]
        );
        // Frame noise should be > 0
        assert!(stats.frame_noise() > 0.001);
    }
}
