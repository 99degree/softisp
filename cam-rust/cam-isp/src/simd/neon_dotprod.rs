//! ARMv8.4-A DOTPROD backend.
//!
//! The `udot` / `sdot` instructions dot-multiply 8-bit values and
//! accumulate into 32-bit. This accelerates:
//!   - Demosaic (5×5 convolution on Bayer data)
//!   - Gaussian denoise (3×3 on u8)
//!   - Any convolution kernel that can be quantized to i8/u8
//!
//! For the current ISP pipeline's f32 path, DOTPROD doesn't help
//! directly, but it's reserved for:
//!   - A future quantized (INT8) pipeline profile
//!   - Pixel-sum operations (histogram, zone stats)
//!   - Downscale (bilinear → integer)

use crate::simd::selector::SimdEngine;

pub struct NeonDotprod;

impl Default for NeonDotprod {
    fn default() -> Self {
        Self::new()
    }
}

impl NeonDotprod {
    pub const fn new() -> Self {
        Self
    }
}

impl SimdEngine for NeonDotprod {
    fn name(&self) -> &'static str {
        "neon-dotprod"
    }

    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32) {
        // Fall back to NEON (DOTPROD doesn't help u16→f32)
        #[cfg(target_arch = "aarch64")]
        {
            let recip = 1.0 / max_val;
            let n = input.len();
            let simd_end = n - (n % 8);
            unsafe {
                use std::arch::aarch64::*;
                let vrecip = vdupq_n_f32(recip);
                for i in (0..simd_end).step_by(8) {
                    let v = vld1q_u16(input.as_ptr().add(i));
                    let lo = vmovl_u16(vget_low_u16(v));
                    let hi = vmovl_u16(vget_high_u16(v));
                    let flo = vmulq_f32(vcvtq_f32_u32(lo), vrecip);
                    let fhi = vmulq_f32(vcvtq_f32_u32(hi), vrecip);
                    vst1q_f32(output.as_mut_ptr().add(i), flo);
                    vst1q_f32(output.as_mut_ptr().add(i + 4), fhi);
                }
            }
            for i in simd_end..n {
                output[i] = input[i] as f32 * recip;
            }
        }

        #[cfg(not(target_arch = "aarch64"))]
        {
            let recip = 1.0 / max_val;
            for i in 0..input.len() {
                output[i] = input[i] as f32 * recip;
            }
        }
    }

    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
        // Same NEON as Neon backend (dotprod doesn't help f32)
        let count = rgb.len() / 3;
        if count == 0 {
            return Vec::new();
        }
        let mut out = vec![0.0f32; rgb.len()];

        #[cfg(target_arch = "aarch64")]
        {
            use std::arch::aarch64::*;
            let simd_count = count - (count % 4);
            let mut idx = 0usize;
            unsafe {
                for _ in (0..simd_count).step_by(4) {
                    let r = vld1q_f32(rgb.as_ptr().add(idx));
                    let g = vld1q_f32(rgb.as_ptr().add(idx + 4));
                    let b = vld1q_f32(rgb.as_ptr().add(idx + 8));

                    let nr = vmlaq_f32(
                        vmulq_f32(r, vdupq_n_f32(matrix[0])),
                        g,
                        vdupq_n_f32(matrix[1]),
                    );
                    let nr = vmlaq_f32(nr, b, vdupq_n_f32(matrix[2]));
                    let ng = vmlaq_f32(
                        vmulq_f32(r, vdupq_n_f32(matrix[3])),
                        g,
                        vdupq_n_f32(matrix[4]),
                    );
                    let ng = vmlaq_f32(ng, b, vdupq_n_f32(matrix[5]));
                    let nb = vmlaq_f32(
                        vmulq_f32(r, vdupq_n_f32(matrix[6])),
                        g,
                        vdupq_n_f32(matrix[7]),
                    );
                    let nb = vmlaq_f32(nb, b, vdupq_n_f32(matrix[8]));

                    let zero = vdupq_n_f32(0.0);
                    let one = vdupq_n_f32(1.0);
                    vst1q_f32(
                        out.as_mut_ptr().add(idx),
                        vminq_f32(vmaxq_f32(nr, zero), one),
                    );
                    vst1q_f32(
                        out.as_mut_ptr().add(idx + 4),
                        vminq_f32(vmaxq_f32(ng, zero), one),
                    );
                    vst1q_f32(
                        out.as_mut_ptr().add(idx + 8),
                        vminq_f32(vmaxq_f32(nb, zero), one),
                    );
                    idx += 12;
                }
            }
            for i in simd_count..count {
                let rgb_idx = i * 3;
                let r = rgb[rgb_idx];
                let g = rgb[rgb_idx + 1];
                let b = rgb[rgb_idx + 2];
                out[rgb_idx] = (matrix[0] * r + matrix[1] * g + matrix[2] * b).clamp(0.0, 1.0);
                out[rgb_idx + 1] = (matrix[3] * r + matrix[4] * g + matrix[5] * b).clamp(0.0, 1.0);
                out[rgb_idx + 2] = (matrix[6] * r + matrix[7] * g + matrix[8] * b).clamp(0.0, 1.0);
            }
        }

        #[cfg(not(target_arch = "aarch64"))]
        {
            for i in 0..count {
                let idx = i * 3;
                let r = rgb[idx];
                let g = rgb[idx + 1];
                let b = rgb[idx + 2];
                out[idx] = (matrix[0] * r + matrix[1] * g + matrix[2] * b).clamp(0.0, 1.0);
                out[idx + 1] = (matrix[3] * r + matrix[4] * g + matrix[5] * b).clamp(0.0, 1.0);
                out[idx + 2] = (matrix[6] * r + matrix[7] * g + matrix[8] * b).clamp(0.0, 1.0);
            }
        }
        out
    }

    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32> {
        if (gain - 1.0).abs() < 0.001 {
            return rgb.to_vec();
        }
        let mut out = vec![0.0f32; rgb.len()];

        #[cfg(target_arch = "aarch64")]
        {
            use std::arch::aarch64::*;
            let n = rgb.len();
            let simd_end = n - (n % 4);
            unsafe {
                let vgain = vdupq_n_f32(gain);
                for i in (0..simd_end).step_by(4) {
                    let v = vld1q_f32(rgb.as_ptr().add(i));
                    vst1q_f32(out.as_mut_ptr().add(i), vmulq_f32(v, vgain));
                }
            }
            for i in simd_end..n {
                out[i] = (rgb[i] * gain).min(1.0);
            }
        }

        #[cfg(not(target_arch = "aarch64"))]
        {
            for i in 0..rgb.len() {
                out[i] = (rgb[i] * gain).min(1.0);
            }
        }
        out
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
                    out[dst_idx] = (rgb[src_idx + 2].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 1] = (rgb[src_idx + 1].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 2] = (rgb[src_idx].clamp(0.0, 1.0) * 255.0) as u8;
                    out[dst_idx + 3] = 255;
                }
            }
        }
        out
    }

    fn bilinear_sample_4ch(&self, src: &[u8], width: u32, height: u32, x: f32, y: f32) -> [u8; 4] {
        // Delegate to NEON backend
        super::neon::Neon.bilinear_sample_4ch(src, width, height, x, y)
    }
}
