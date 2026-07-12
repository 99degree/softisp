//! SSE2 SIMD backend — x86_64 baseline.
//!
//! Provides 4-wide f32 operations via __m128.
//! Falls back to scalar for operations not easily vectorized.

#![cfg(target_arch = "x86_64")]

use std::arch::x86_64::*;

use super::SimdEngine;

pub struct Sse2;

impl Sse2 {
    pub fn new() -> Self {
        Self
    }
}

unsafe fn normalize_u16_to_f32_sse2(input: &[u16], output: &mut [f32], max_val: f32) {
    let inv_max = 1.0 / max_val;
    let v_inv = _mm_set1_ps(inv_max);
    let v_max = _mm_set1_ps(1.0);
    let v_zero = _mm_setzero_ps();

    let chunks = input.len() / 4;
    for i in 0..chunks {
        let base = i * 4;
        let raw = _mm_loadl_epi64(input[base..].as_ptr() as *const __m128i);
        let i32_4 = _mm_cvtepu16_epi32(raw);
        let f32_4 = _mm_cvtepi32_ps(i32_4);
        let normalized = _mm_mul_ps(f32_4, v_inv);
        let clamped = _mm_min_ps(_mm_max_ps(normalized, v_zero), v_max);
        _mm_storeu_ps(output[base..].as_mut_ptr(), clamped);
    }

    for i in (chunks * 4)..input.len() {
        let v = input[i] as f32 * inv_max;
        output[i] = v.clamp(0.0, 1.0);
    }
}

unsafe fn apply_ccm_sse2(rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
    let mut out = Vec::with_capacity(rgb.len());
    let n = rgb.len() / 3;

    // Process 4 pixels at a time
    let chunks = n / 4;
    for c in 0..chunks {
        let base = c * 12;
        let mut r = [0.0f32; 4];
        let mut g = [0.0f32; 4];
        let mut b = [0.0f32; 4];
        for p in 0..4 {
            r[p] = rgb[base + p * 3];
            g[p] = rgb[base + p * 3 + 1];
            b[p] = rgb[base + p * 3 + 2];
        }
        let v_r = _mm_loadu_ps(r.as_ptr());
        let v_g = _mm_loadu_ps(g.as_ptr());
        let v_b = _mm_loadu_ps(b.as_ptr());

        let m_r = _mm_set1_ps(matrix[0]);
        let m_g = _mm_set1_ps(matrix[1]);
        let m_b = _mm_set1_ps(matrix[2]);
        let m2r = _mm_set1_ps(matrix[3]);
        let m2g = _mm_set1_ps(matrix[4]);
        let m2b = _mm_set1_ps(matrix[5]);
        let m3r = _mm_set1_ps(matrix[6]);
        let m3g = _mm_set1_ps(matrix[7]);
        let m3b = _mm_set1_ps(matrix[8]);
        let v_max = _mm_set1_ps(1.0);
        let v_zero = _mm_setzero_ps();

        let out_r = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(m_r, v_r), _mm_mul_ps(m_g, v_g)),
            _mm_mul_ps(m_b, v_b),
        );
        let out_g = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(m2r, v_r), _mm_mul_ps(m2g, v_g)),
            _mm_mul_ps(m2b, v_b),
        );
        let out_b = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(m3r, v_r), _mm_mul_ps(m3g, v_g)),
            _mm_mul_ps(m3b, v_b),
        );

        let out_r = _mm_min_ps(_mm_max_ps(out_r, v_zero), v_max);
        let out_g = _mm_min_ps(_mm_max_ps(out_g, v_zero), v_max);
        let out_b = _mm_min_ps(_mm_max_ps(out_b, v_zero), v_max);

        let mut or = [0.0f32; 4];
        let mut og = [0.0f32; 4];
        let mut ob = [0.0f32; 4];
        _mm_storeu_ps(or.as_mut_ptr(), out_r);
        _mm_storeu_ps(og.as_mut_ptr(), out_g);
        _mm_storeu_ps(ob.as_mut_ptr(), out_b);
        for p in 0..4 {
            out.push(or[p]);
            out.push(og[p]);
            out.push(ob[p]);
        }
    }

    let start = chunks * 4;
    for i in start..n {
        let base = i * 3;
        let r = rgb[base];
        let g = rgb[base + 1];
        let b = rgb[base + 2];
        out.push((matrix[0] * r + matrix[1] * g + matrix[2] * b).clamp(0.0, 1.0));
        out.push((matrix[3] * r + matrix[4] * g + matrix[5] * b).clamp(0.0, 1.0));
        out.push((matrix[6] * r + matrix[7] * g + matrix[8] * b).clamp(0.0, 1.0));
    }
    out
}

unsafe fn apply_ae_gain_sse2(rgb: &[f32], gain: f32) -> Vec<f32> {
    let v_gain = _mm_set1_ps(gain);
    let v_max = _mm_set1_ps(1.0);
    let v_zero = _mm_setzero_ps();
    let mut out = Vec::with_capacity(rgb.len());

    let chunks = rgb.len() / 4;
    for c in 0..chunks {
        let base = c * 4;
        let v = _mm_loadu_ps(rgb[base..].as_ptr());
        let gained = _mm_mul_ps(v, v_gain);
        let clamped = _mm_min_ps(_mm_max_ps(gained, v_zero), v_max);
        unsafe {
            let slice = std::slice::from_raw_parts_mut(out.as_mut_ptr().add(base), 4);
            _mm_storeu_ps(slice.as_mut_ptr(), clamped);
        }
        out.set_len(base + 4);
    }

    let start = chunks * 4;
    for i in start..rgb.len() {
        out.push((rgb[i] * gain).clamp(0.0, 1.0));
    }
    out
}

unsafe fn display_output_sse2(rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
    let scale_x = src_w as f32 / target_w as f32;
    let mut out = Vec::with_capacity(target_w * src_h * 4);
    let v_255 = _mm_set1_ps(255.0);
    let v_zero = _mm_setzero_ps();

    for y in 0..src_h {
        for x_chunk in (0..target_w).step_by(4) {
            let count = (target_w - x_chunk).min(4);
            let mut r = [0.0f32; 4];
            let mut g = [0.0f32; 4];
            let mut b = [0.0f32; 4];
            for p in 0..count {
                let sx = ((x_chunk + p) as f32 * scale_x) as usize;
                let sx = sx.min(src_w - 1);
                let si = (y * src_w + sx) * 3;
                r[p] = rgb[si];
                g[p] = rgb[si + 1];
                b[p] = rgb[si + 2];
            }
            let v_r = _mm_loadu_ps(r.as_ptr());
            let v_g = _mm_loadu_ps(g.as_ptr());
            let v_b = _mm_loadu_ps(b.as_ptr());
            let rb = _mm_cvtps_epi32(_mm_min_ps(
                _mm_max_ps(_mm_mul_ps(v_r, v_255), v_zero),
                v_255,
            ));
            let gb = _mm_cvtps_epi32(_mm_min_ps(
                _mm_max_ps(_mm_mul_ps(v_g, v_255), v_zero),
                v_255,
            ));
            let bb = _mm_cvtps_epi32(_mm_min_ps(
                _mm_max_ps(_mm_mul_ps(v_b, v_255), v_zero),
                v_255,
            ));

            let mut rb_i = [0i32; 4];
            let mut gb_i = [0i32; 4];
            let mut bb_i = [0i32; 4];
            _mm_storeu_si128(rb_i.as_mut_ptr() as *mut __m128i, rb);
            _mm_storeu_si128(gb_i.as_mut_ptr() as *mut __m128i, gb);
            _mm_storeu_si128(bb_i.as_mut_ptr() as *mut __m128i, bb);

            for p in 0..count {
                out.push(bb_i[p] as u8);
                out.push(gb_i[p] as u8);
                out.push(rb_i[p] as u8);
                out.push(255);
            }
        }
    }
    out
}

/// SSE2 bilinear sample for 4 channels (BGRA) at position (x, y).
/// Processes all 4 channel interpolations in parallel using SSE floats.
#[cfg(target_arch = "x86_64")]
pub(crate) unsafe fn bilinear_sample_4ch_sse2(
    src: &[u8],
    width: u32,
    height: u32,
    x: f32,
    y: f32,
) -> [u8; 4] {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let w = width as i32 - 1;
    let h = height as i32 - 1;
    let x0c = x0.clamp(0, w);
    let y0c = y0.clamp(0, h);
    let x1c = x1.clamp(0, w);
    let y1c = y1.clamp(0, h);

    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let w00 = (1.0 - fx) * (1.0 - fy);
    let w10 = fx * (1.0 - fy);
    let w01 = (1.0 - fx) * fy;
    let w11 = fx * fy;

    let idx = |sx: i32, sy: i32| -> usize { ((sy as u32 * width + sx as u32) * 4) as usize };

    let load_4bytes = |i: usize| -> __m128 {
        let bytes = _mm_cvtsi32_si128(*(src.as_ptr().add(i) as *const i32));
        let zero = _mm_setzero_si128();
        let u16 = _mm_unpacklo_epi8(bytes, zero);
        let u32 = _mm_unpacklo_epi16(u16, zero);
        _mm_cvtepi32_ps(u32)
    };

    let v_p00 = load_4bytes(idx(x0c, y0c));
    let v_p10 = load_4bytes(idx(x1c, y0c));
    let v_p01 = load_4bytes(idx(x0c, y1c));
    let v_p11 = load_4bytes(idx(x1c, y1c));

    let v_w00 = _mm_set1_ps(w00);
    let v_w10 = _mm_set1_ps(w10);
    let v_w01 = _mm_set1_ps(w01);
    let v_w11 = _mm_set1_ps(w11);

    let mut v_result = _mm_mul_ps(v_p00, v_w00);
    v_result = _mm_add_ps(v_result, _mm_mul_ps(v_p10, v_w10));
    v_result = _mm_add_ps(v_result, _mm_mul_ps(v_p01, v_w01));
    v_result = _mm_add_ps(v_result, _mm_mul_ps(v_p11, v_w11));

    let v_255 = _mm_set1_ps(255.0);
    let v_zero = _mm_setzero_ps();
    v_result = _mm_min_ps(_mm_max_ps(v_result, v_zero), v_255);
    let i32_result = _mm_cvtps_epi32(v_result);

    let zero_i16 = _mm_setzero_si128();
    let u16_lo = _mm_packus_epi32(i32_result, zero_i16);
    let u8_all = _mm_packus_epi16(u16_lo, zero_i16);

    let mut result = [0u8; 4];
    _mm_storel_epi64(&mut result as *mut _ as *mut __m128i, u8_all);
    result
}

impl SimdEngine for Sse2 {
    fn name(&self) -> &'static str {
        "sse2"
    }

    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32) {
        unsafe { normalize_u16_to_f32_sse2(input, output, max_val) }
    }

    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
        unsafe { apply_ccm_sse2(rgb, matrix) }
    }

    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32> {
        unsafe { apply_ae_gain_sse2(rgb, gain) }
    }

    fn display_output(&self, rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
        unsafe { display_output_sse2(rgb, src_w, src_h, target_w) }
    }

    fn bilinear_sample_4ch(&self, src: &[u8], width: u32, height: u32, x: f32, y: f32) -> [u8; 4] {
        unsafe { bilinear_sample_4ch_sse2(src, width, height, x, y) }
    }
}
