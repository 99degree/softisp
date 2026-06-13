//! Pure scalar backend — always available, runs on any architecture.
//! Used as fallback when no SIMD is detected.

use crate::simd::selector::SimdEngine;

pub struct Scalar;

impl Scalar {
    pub const fn new() -> Self {
        Scalar
    }
}

impl SimdEngine for Scalar {
    fn name(&self) -> &'static str {
        "scalar"
    }

    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32) {
        assert_eq!(input.len(), output.len());
        let recip = 1.0 / max_val;
        for i in 0..input.len() {
            output[i] = input[i] as f32 * recip;
        }
    }

    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
        let count = rgb.len() / 3;
        let mut out = vec![0.0f32; rgb.len()];
        for i in 0..count {
            let idx = i * 3;
            let r = rgb[idx];
            let g = rgb[idx + 1];
            let b = rgb[idx + 2];
            out[idx]     = (matrix[0] * r + matrix[1] * g + matrix[2] * b).clamp(0.0, 1.0);
            out[idx + 1] = (matrix[3] * r + matrix[4] * g + matrix[5] * b).clamp(0.0, 1.0);
            out[idx + 2] = (matrix[6] * r + matrix[7] * g + matrix[8] * b).clamp(0.0, 1.0);
        }
        out
    }

    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32> {
        if (gain - 1.0).abs() < 0.001 {
            return rgb.to_vec();
        }
        rgb.iter().map(|&v| (v * gain).min(1.0)).collect()
    }

    fn display_output(&self, rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
        if rgb.len() < 3 {
            return Vec::new();
        }
        let target_h = if target_w < src_w {
            (src_h as f32 * target_w as f32 / src_w as f32) as usize
        } else {
            src_h
        };
        let mut out = vec![0u8; target_w * target_h * 4];
        for y in 0..target_h {
            for x in 0..target_w {
                let sx = x * src_w / target_w;
                let sy = y * src_h / target_h;
                let src_idx = (sy * src_w + sx) * 3;
                let dst_idx = (y * target_w + x) * 4;
                if src_idx + 2 < rgb.len() {
                    out[dst_idx]     = (rgb[src_idx + 2].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 1] = (rgb[src_idx + 1].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 2] = (rgb[src_idx].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 3] = 255;
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::simd::selector::SimdEngine;

    #[test]
    fn test_scalar_normalize() {
        let s = Scalar::new();
        let input = vec![0u16, 32768, 65535];
        let mut output = vec![0.0f32; 3];
        s.normalize_u16_to_f32(&input, &mut output, 65535.0);
        assert!((output[0] - 0.0).abs() < 0.001);
        assert!((output[1] - 0.5).abs() < 0.01);
        assert!(output[2] <= 1.0);
    }

    #[test]
    fn test_scalar_ccm_identity() {
        let s = Scalar::new();
        let rgb = vec![0.5f32, 0.3f32, 0.7f32];
        let m = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0];
        let out = s.apply_ccm(&rgb, &m);
        assert_eq!(out.len(), 3);
        assert!((out[0] - 0.5).abs() < 0.001);
        assert!((out[1] - 0.3).abs() < 0.001);
        assert!((out[2] - 0.7).abs() < 0.001);
    }

    #[test]
    fn test_scalar_ae_gain() {
        let s = Scalar::new();
        let rgb = vec![0.5f32, 0.3f32, 0.7f32];
        let out = s.apply_ae_gain(&rgb, 2.0);
        assert_eq!(out.len(), 3);
        assert!((out[0] - 1.0).abs() < 0.001);
        assert!((out[1] - 0.6).abs() < 0.001);
        assert!((out[2] - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_scalar_display_output() {
        let s = Scalar::new();
        let rgb = vec![1.0f32, 0.0f32, 0.0f32, 0.0f32, 1.0f32, 0.0f32]; // 2×1 RGB
        let out = s.display_output(&rgb, 2, 1, 2);
        assert_eq!(out.len(), 8); // 2×1×4 BGRA
        assert_eq!(out[0], 0); // B
        assert_eq!(out[1], 0); // G
        assert_eq!(out[2], 255); // R
        assert_eq!(out[3], 255); // A
    }
}