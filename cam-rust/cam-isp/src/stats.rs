//! Statistics computation — channel means, tone stats, histogram, zone stats.
//!
//! Extracted from `cpu.rs` for modularity.

/// Compute per-channel means from RGB.
pub(crate) fn compute_channel_means(rgb: &[f32]) -> [f32; 3] {
    let count = rgb.len() / 3;
    if count == 0 {
        return [0.0; 3];
    }
    let mut r_sum = 0.0f64;
    let mut g_sum = 0.0f64;
    let mut b_sum = 0.0f64;
    for i in 0..count {
        let idx = i * 3;
        r_sum += rgb[idx] as f64;
        g_sum += rgb[idx + 1] as f64;
        b_sum += rgb[idx + 2] as f64;
    }
    let n = count as f64;
    [
        (r_sum / n) as f32,
        (g_sum / n) as f32,
        (b_sum / n) as f32,
    ]
}

/// Compute tone statistics: mean luminance, min, max.
pub(crate) fn compute_tone_stats(rgb: &[f32]) -> [f32; 3] {
    let count = rgb.len() / 3;
    if count == 0 {
        return [0.0, 0.0, 0.0];
    }
    let mut sum = 0.0f64;
    let mut min_lum = 1.0f64;
    let mut max_lum = 0.0f64;
    for i in 0..count {
        let idx = i * 3;
        let luma = (0.299 * rgb[idx] + 0.587 * rgb[idx + 1] + 0.114 * rgb[idx + 2]) as f64;
        sum += luma;
        if luma < min_lum { min_lum = luma; }
        if luma > max_lum { max_lum = luma; }
    }
    [
        (sum / count as f64) as f32,
        min_lum as f32,
        max_lum as f32,
    ]
}

/// Compute 256-bin luminance histogram from RGB.
pub(crate) fn compute_histogram(rgb: &[f32]) -> [f32; 256] {
    let mut hist = [0.0f64; 256];
    let count = rgb.len() / 3;
    if count == 0 {
        return [0.0; 256];
    }
    for i in 0..count {
        let idx = i * 3;
        let luma = 0.299 * rgb[idx] + 0.587 * rgb[idx + 1] + 0.114 * rgb[idx + 2];
        let bin = (luma * 255.0).round() as usize;
        if bin < 256 {
            hist[bin] += 1.0;
        }
    }
    // Normalize
    let total = count as f64;
    let mut result = [0.0f32; 256];
    for i in 0..256 {
        result[i] = (hist[i] / total) as f32;
    }
    result
}

/// Compute 6x8 zone stats from RGB for multi-illuminant AWB.
pub(crate) fn compute_zone_stats(
    rgb: &[f32],
    width: usize,
    height: usize,
    rows: usize,
    cols: usize,
) -> Vec<f32> {
    let mut zones = Vec::with_capacity(rows * cols * 3);
    for r in 0..rows {
        for c in 0..cols {
            let y_start = r * height / rows;
            let y_end = ((r + 1) * height / rows).min(height);
            let x_start = c * width / cols;
            let x_end = ((c + 1) * width / cols).min(width);
            let mut r_sum = 0.0f64;
            let mut g_sum = 0.0f64;
            let mut b_sum = 0.0f64;
            let mut cnt = 0u64;
            for y in y_start..y_end {
                for x in x_start..x_end {
                    let idx = (y * width + x) * 3;
                    if idx + 2 < rgb.len() {
                        r_sum += rgb[idx] as f64;
                        g_sum += rgb[idx + 1] as f64;
                        b_sum += rgb[idx + 2] as f64;
                        cnt += 1;
                    }
                }
            }
            if cnt > 0 {
                zones.push((r_sum / cnt as f64) as f32);
                zones.push((g_sum / cnt as f64) as f32);
                zones.push((b_sum / cnt as f64) as f32);
            } else {
                zones.push(0.0);
                zones.push(0.0);
                zones.push(0.0);
            }
        }
    }
    zones
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_channel_means() {
        let rgb = vec![0.8, 0.6, 0.4, 0.2, 0.4, 0.6];
        let means = compute_channel_means(&rgb);
        assert!((means[0] - 0.5).abs() < 0.001);
        assert!((means[1] - 0.5).abs() < 0.001);
        assert!((means[2] - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_compute_channel_means_empty() {
        assert_eq!(compute_channel_means(&[]), [0.0; 3]);
    }

    #[test]
    fn test_compute_tone_stats() {
        let rgb = vec![0.1, 0.2, 0.3, 0.9, 0.8, 0.7];
        let stats = compute_tone_stats(&rgb);
        assert!(stats[1] <= stats[0]);
        assert!(stats[2] >= stats[0]);
    }

    #[test]
    fn test_compute_zone_stats() {
        let mut rgb = vec![0.0f32; 4 * 4 * 3];
        for i in 0..4 {
            rgb[i * 3] = 0.8;
        }
        let zones = compute_zone_stats(&rgb, 4, 4, 2, 2);
        assert_eq!(zones.len(), 12);
        assert!(zones[0] > 0.0); // R channel of top-left zone
    }
}
