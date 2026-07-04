//! NEON-accelerated backend (aarch64).
//! Provides ~4x throughput on normalize, CCM, AE gain, and display ops.

use crate::simd::selector::SimdEngine;

pub struct Neon;

impl Default for Neon {
    fn default() -> Self {
        Self::new()
    }
}

impl Neon {
    pub const fn new() -> Self {
        Neon
    }
}

impl SimdEngine for Neon {
    fn name(&self) -> &'static str {
        "neon"
    }

    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32) {
        #[cfg(not(target_arch = "aarch64"))]
        {
            // Fallback on non-aarch64
            let recip = 1.0 / max_val;
            for i in 0..input.len() {
                output[i] = input[i] as f32 * recip;
            }
        }

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
    }

    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
        let count = rgb.len() / 3;
        if count == 0 {
            return Vec::new();
        }
        let mut out = vec![0.0f32; rgb.len()];

        #[cfg(not(target_arch = "aarch64"))]
        {
            // Scalar fallback
            for i in 0..count {
                let idx = i * 3;
                let r = rgb[idx];
                let g = rgb[idx + 1];
                let b = rgb[idx + 2];
                out[idx] = (matrix[0]*r + matrix[1]*g + matrix[2]*b).clamp(0.0, 1.0);
                out[idx+1] = (matrix[3]*r + matrix[4]*g + matrix[5]*b).clamp(0.0, 1.0);
                out[idx+2] = (matrix[6]*r + matrix[7]*g + matrix[8]*b).clamp(0.0, 1.0);
            }
        }

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
                        g, vdupq_n_f32(matrix[1]));
                    let nr = vmlaq_f32(nr, b, vdupq_n_f32(matrix[2]));

                    let ng = vmlaq_f32(
                        vmulq_f32(r, vdupq_n_f32(matrix[3])),
                        g, vdupq_n_f32(matrix[4]));
                    let ng = vmlaq_f32(ng, b, vdupq_n_f32(matrix[5]));

                    let nb = vmlaq_f32(
                        vmulq_f32(r, vdupq_n_f32(matrix[6])),
                        g, vdupq_n_f32(matrix[7]));
                    let nb = vmlaq_f32(nb, b, vdupq_n_f32(matrix[8]));

                    let zero = vdupq_n_f32(0.0);
                    let one = vdupq_n_f32(1.0);
                    let nr = vminq_f32(vmaxq_f32(nr, zero), one);
                    let ng = vminq_f32(vmaxq_f32(ng, zero), one);
                    let nb = vminq_f32(vmaxq_f32(nb, zero), one);

                    vst1q_f32(out.as_mut_ptr().add(idx), nr);
                    vst1q_f32(out.as_mut_ptr().add(idx + 4), ng);
                    vst1q_f32(out.as_mut_ptr().add(idx + 8), nb);
                    idx += 12;
                }
            }
            for i in simd_count..count {
                let rgb_idx = i * 3;
                let r = rgb[rgb_idx];
                let g = rgb[rgb_idx + 1];
                let b = rgb[rgb_idx + 2];
                out[rgb_idx] = (matrix[0]*r + matrix[1]*g + matrix[2]*b).clamp(0.0, 1.0);
                out[rgb_idx+1] = (matrix[3]*r + matrix[4]*g + matrix[5]*b).clamp(0.0, 1.0);
                out[rgb_idx+2] = (matrix[6]*r + matrix[7]*g + matrix[8]*b).clamp(0.0, 1.0);
            }
        }
        out
    }

    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32> {
        if (gain - 1.0).abs() < 0.001 {
            return rgb.to_vec();
        }

        let mut out = vec![0.0f32; rgb.len()];

        #[cfg(not(target_arch = "aarch64"))]
        {
            for i in 0..rgb.len() {
                out[i] = (rgb[i] * gain).min(1.0);
            }
        }

        #[cfg(target_arch = "aarch64")]
        {
            let n = rgb.len();
            let simd_end = n - (n % 4);
            unsafe {
                use std::arch::aarch64::*;
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

        // Fast path: no resize, NEON
        #[cfg(target_arch = "aarch64")]
        if target_w == src_w && target_h == src_h {
            for y in 0..src_h {
                    for x in (0..src_w).step_by(4) {
                        let src_base = (y * src_w + x) * 3;
                        let dst_base = (y * src_w + x) * 4;
                        for px in 0..4 {
                            let s = src_base + px * 3;
                            let d = dst_base + px * 4;
                            if s + 2 < rgb.len() {
                                out[d]     = (rgb[s + 2].clamp(0.0, 1.0) * 255.0) as u8;
                                out[d + 1] = (rgb[s + 1].clamp(0.0, 1.0) * 255.0) as u8;
                                out[d + 2] = (rgb[s].clamp(0.0, 1.0) * 255.0) as u8;
                                out[d + 3] = 255;
                            }
                        }
                    }
                }
            return out;
        }

        // Resize path (nearest neighbor, scalar)
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

    fn bilinear_sample_4ch(
        &self,
        src: &[u8],
        width: u32,
        height: u32,
        x: f32,
        y: f32,
    ) -> [u8; 4] {
        #[cfg(not(target_arch = "aarch64"))]
        {
            // Fallback to scalar on non-aarch64
            let x0 = x.floor() as i32;
            let y0 = y.floor() as i32;
            let x1 = x0 + 1;
            let y1 = y0 + 1;
            let fx = x - x0 as f32;
            let fy = y - y0 as f32;
            let w = width as i32 - 1;
            let h = height as i32 - 1;
            let x0 = x0.clamp(0, w) as u32;
            let y0 = y0.clamp(0, h) as u32;
            let x1 = x1.clamp(0, w) as u32;
            let y1 = y1.clamp(0, h) as u32;
            let idx = |sx: u32, sy: u32| -> usize { ((sy * width + sx) * 4) as usize };
            let p00 = [src[idx(x0, y0)], src[idx(x0, y0)+1], src[idx(x0, y0)+2], src[idx(x0, y0)+3]];
            let p10 = [src[idx(x1, y0)], src[idx(x1, y0)+1], src[idx(x1, y0)+2], src[idx(x1, y0)+3]];
            let p01 = [src[idx(x0, y1)], src[idx(x0, y1)+1], src[idx(x0, y1)+2], src[idx(x0, y1)+3]];
            let p11 = [src[idx(x1, y1)], src[idx(x1, y1)+1], src[idx(x1, y1)+2], src[idx(x1, y1)+3]];
            let mut result = [0u8; 4];
            for c in 0..4 {
                let v = p00[c] as f32 * (1.0 - fx) * (1.0 - fy)
                    + p10[c] as f32 * fx * (1.0 - fy)
                    + p01[c] as f32 * (1.0 - fx) * fy
                    + p11[c] as f32 * fx * fy;
                result[c] = v.round().clamp(0.0, 255.0) as u8;
            }
            return result;
        }

        #[cfg(target_arch = "aarch64")]
        {
            unsafe { neon_bilinear_sample_4ch(src, width, height, x, y) }
        }
    }
}

#[cfg(target_arch = "aarch64")]
unsafe fn neon_bilinear_sample_4ch(
    src: &[u8],
    width: u32,
    height: u32,
    x: f32,
    y: f32,
) -> [u8; 4] {
    use std::arch::aarch64::*;

    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let w = width as i32 - 1;
    let h = height as i32 - 1;
    let x0c = x0.clamp(0, w) as u32;
    let y0c = y0.clamp(0, h) as u32;
    let x1c = x1.clamp(0, w) as u32;
    let y1c = y1.clamp(0, h) as u32;

    let fx = x - x0 as f32;
    let fy = y - y0 as f32;

    let idx = |sx: u32, sy: u32| -> usize { ((sy * width + sx) * 4) as usize };

    // Load 4 bytes from each corner using NEON
    // vld1_u8 loads 8 bytes, we use the lower 4
    let p00_u8 = vld1_u8(src.as_ptr().add(idx(x0c, y0c)));
    let p10_u8 = vld1_u8(src.as_ptr().add(idx(x1c, y0c)));
    let p01_u8 = vld1_u8(src.as_ptr().add(idx(x0c, y1c)));
    let p11_u8 = vld1_u8(src.as_ptr().add(idx(x1c, y1c)));

    // Expand u8 → u16 → u32 → f32
    let expand_to_f32 = |v: uint8x8_t| -> float32x4_t {
        let u16 = vmovl_u8(v);           // u8 → u16 (8 lanes)
        let u32 = vmovl_u16(vget_low_u16(u16)); // lower 4 × u16 → u32
        vcvtq_f32_u32(u32)               // u32 → f32
    };

    let v_p00 = expand_to_f32(p00_u8);
    let v_p10 = expand_to_f32(p10_u8);
    let v_p01 = expand_to_f32(p01_u8);
    let v_p11 = expand_to_f32(p11_u8);

    // Interpolate: result = w00*p00 + w10*p10 + w01*p01 + w11*p11
    let w00 = (1.0 - fx) * (1.0 - fy);
    let w10 = fx * (1.0 - fy);
    let w01 = (1.0 - fx) * fy;
    let w11 = fx * fy;

    let v_w00 = vdupq_n_f32(w00);
    let v_w10 = vdupq_n_f32(w10);
    let v_w01 = vdupq_n_f32(w01);
    let v_w11 = vdupq_n_f32(w11);

    let mut v_result = vmulq_f32(v_p00, v_w00);
    v_result = vmlaq_f32(v_result, v_p10, v_w10);
    v_result = vmlaq_f32(v_result, v_p01, v_w01);
    v_result = vmlaq_f32(v_result, v_p11, v_w11);

    // Clamp to [0, 255] and convert back to u8
    let v_255 = vdupq_n_f32(255.0);
    let v_zero = vdupq_n_f32(0.0);
    v_result = vmaxq_f32(vminq_f32(v_result, v_255), v_zero);

    // f32 → u32 → u16 → u8
    let u32_result = vcvtq_u32_f32(v_result);      // f32 → u32
    let u16_result = vqmovn_u32(u32_result);        // u32 → u16 (saturating)
    let u8_result = vqmovn_u16(vcombine_u16(u16_result, u16_result)); // u16 → u8 (saturating)

    let mut result = [0u8; 4];
    // Store the lower 4 bytes (1 u32 lane) from the 8-byte register
    // Reinterpret uint8x8_t as uint32x2_t, store lane 0
    let u32_2 = vreinterpret_u32_u8(u8_result);
    vst1_lane_u32(result.as_mut_ptr() as *mut u32, u32_2, 0);
    result
}