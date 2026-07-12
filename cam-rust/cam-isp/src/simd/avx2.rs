//! AVX2 SIMD backend — x86_64 with AVX2 + FMA.
//!
//! Provides:
//! - 8-wide f32 operations via __m256
//! - FMA (fused multiply-add) for CCM
//! - Pack/unload via `_mm256_cvtps_epi32` → u8

#![cfg(target_arch = "x86_64")]

use std::arch::x86_64::*;

use super::SimdEngine;

pub struct Avx2;

impl Avx2 {
    pub fn new() -> Self {
        Self
    }
}

// SAFETY: AVX2 detected at runtime by caller.
unsafe fn normalize_u16_to_f32_avx2(input: &[u16], output: &mut [f32], max_val: f32) {
    let inv_max = 1.0 / max_val;
    let v_inv = _mm256_set1_ps(inv_max);
    let v_max = _mm256_set1_ps(1.0);
    let v_zero = _mm256_setzero_ps();

    let chunks = input.len() / 8;
    for i in 0..chunks {
        let base = i * 8;
        // Load 8 u16 → zero-extend to 32-bit integers
        let raw = _mm_loadl_epi64(input[base..].as_ptr() as *const __m128i);
        let i32_8 = _mm256_cvtepu16_epi32(raw);
        let f32_8 = _mm256_cvtepi32_ps(i32_8);
        let normalized = _mm256_mul_ps(f32_8, v_inv);
        let clamped = _mm256_min_ps(_mm256_max_ps(normalized, v_zero), v_max);
        _mm256_storeu_ps(output[base..].as_mut_ptr(), clamped);
    }

    // Scalar tail
    for i in (chunks * 8)..input.len() {
        let v = input[i] as f32 * inv_max;
        output[i] = v.clamp(0.0, 1.0);
    }
}

unsafe fn apply_ccm_avx2(rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
    let mut out = Vec::with_capacity(rgb.len());
    let m_r = _mm256_set1_ps(matrix[0]);
    let m_g = _mm256_set1_ps(matrix[1]);
    let m_b = _mm256_set1_ps(matrix[2]);
    let m2r = _mm256_set1_ps(matrix[3]);
    let m2g = _mm256_set1_ps(matrix[4]);
    let m2b = _mm256_set1_ps(matrix[5]);
    let m3r = _mm256_set1_ps(matrix[6]);
    let m3g = _mm256_set1_ps(matrix[7]);
    let m3b = _mm256_set1_ps(matrix[8]);
    let v_max = _mm256_set1_ps(1.0);
    let v_zero = _mm256_setzero_ps();

    // Process 8 pixels at a time (24 floats = 3 × 8)
    let n = rgb.len() / 3;
    let chunks = n / 8;
    for c in 0..chunks {
        let base = c * 24;
        // Deinterleave R, G, B from interleaved RGB
        // rgb[base..base+24] = [R0,G0,B0, R1,G1,B1, ..., R7,G7,B7]
        // Load 24 floats
        let mut r = [0.0f32; 8];
        let mut g = [0.0f32; 8];
        let mut b = [0.0f32; 8];
        for p in 0..8 {
            r[p] = rgb[base + p * 3];
            g[p] = rgb[base + p * 3 + 1];
            b[p] = rgb[base + p * 3 + 2];
        }
        let v_r = _mm256_loadu_ps(r.as_ptr());
        let v_g = _mm256_loadu_ps(g.as_ptr());
        let v_b = _mm256_loadu_ps(b.as_ptr());

        // Row 0: out_r = m[0]*R + m[1]*G + m[2]*B
        let out_r = _mm256_fmadd_ps(m_r, v_r, _mm256_fmadd_ps(m_g, v_g, _mm256_mul_ps(m_b, v_b)));
        // Row 1: out_g = m[3]*R + m[4]*G + m[5]*B
        let out_g = _mm256_fmadd_ps(m2r, v_r, _mm256_fmadd_ps(m2g, v_g, _mm256_mul_ps(m2b, v_b)));
        // Row 2: out_b = m[6]*R + m[7]*G + m[8]*B
        let out_b = _mm256_fmadd_ps(m3r, v_r, _mm256_fmadd_ps(m3g, v_g, _mm256_mul_ps(m3b, v_b)));

        // Clamp
        let out_r = _mm256_min_ps(_mm256_max_ps(out_r, v_zero), v_max);
        let out_g = _mm256_min_ps(_mm256_max_ps(out_g, v_zero), v_max);
        let out_b = _mm256_min_ps(_mm256_max_ps(out_b, v_zero), v_max);

        // Store interleaved
        let mut or = [0.0f32; 8];
        let mut og = [0.0f32; 8];
        let mut ob = [0.0f32; 8];
        _mm256_storeu_ps(or.as_mut_ptr(), out_r);
        _mm256_storeu_ps(og.as_mut_ptr(), out_g);
        _mm256_storeu_ps(ob.as_mut_ptr(), out_b);
        for p in 0..8 {
            out.push(or[p]);
            out.push(og[p]);
            out.push(ob[p]);
        }
    }

    // Scalar tail
    let start = chunks * 8;
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

unsafe fn apply_ae_gain_avx2(rgb: &[f32], gain: f32) -> Vec<f32> {
    let v_gain = _mm256_set1_ps(gain);
    let v_max = _mm256_set1_ps(1.0);
    let v_zero = _mm256_setzero_ps();
    let mut out = Vec::with_capacity(rgb.len());

    let chunks = rgb.len() / 8;
    for c in 0..chunks {
        let base = c * 8;
        let v = _mm256_loadu_rgb(rgb[base..].as_ptr());
        let gained = _mm256_mul_ps(v, v_gain);
        let clamped = _mm256_min_ps(_mm256_max_ps(gained, v_zero), v_max);
        _mm256_storeu_ps(out[base..].as_mut_ptr(), clamped);
        out.resize(base + 8, 0.0);
    }

    // Scalar tail
    let start = chunks * 8;
    for i in start..rgb.len() {
        out.push((rgb[i] * gain).clamp(0.0, 1.0));
    }
    out
}

unsafe fn display_output_avx2(rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
    let scale_x = src_w as f32 / target_w as f32;
    let mut out = Vec::with_capacity(target_w * src_h * 4);
    let v_255 = _mm256_set1_ps(255.0);
    let v_zero = _mm256_setzero_ps();

    for y in 0..src_h {
        for x_chunk in (0..target_w).step_by(8) {
            let count = (target_w - x_chunk).min(8);
            let mut r = [0.0f32; 8];
            let mut g = [0.0f32; 8];
            let mut b = [0.0f32; 8];
            for p in 0..count {
                let sx = ((x_chunk + p) as f32 * scale_x) as usize;
                let sx = sx.min(src_w - 1);
                let si = (y * src_w + sx) * 3;
                r[p] = rgb[si];
                g[p] = rgb[si + 1];
                b[p] = rgb[si + 2];
            }
            let v_r = _mm256_loadu_ps(r.as_ptr());
            let v_g = _mm256_loadu_ps(g.as_ptr());
            let v_b = _mm256_loadu_ps(b.as_ptr());
            let rb = _mm256_cvtps_epi32(_mm256_min_ps(
                _mm256_max_ps(_mm256_mul_ps(v_r, v_255), v_zero),
                v_255,
            ));
            let gb = _mm256_cvtps_epi32(_mm256_min_ps(
                _mm256_max_ps(_mm256_mul_ps(v_g, v_255), v_zero),
                v_255,
            ));
            let bb = _mm256_cvtps_epi32(_mm256_min_ps(
                _mm256_max_ps(_mm256_mul_ps(v_b, v_255), v_zero),
                v_255,
            ));

            let mut rb_i = [0i32; 8];
            let mut gb_i = [0i32; 8];
            let mut bb_i = [0i32; 8];
            _mm256_storeu_si256(rb_i.as_mut_ptr() as *mut __m256i, rb);
            _mm256_storeu_si256(gb_i.as_mut_ptr() as *mut __m256i, gb);
            _mm256_storeu_si256(bb_i.as_mut_ptr() as *mut __m256i, bb);

            for p in 0..count {
                out.push(bb_i[p] as u8); // B
                out.push(gb_i[p] as u8); // G
                out.push(rb_i[p] as u8); // R
                out.push(255); // A
            }
        }
    }
    out
}

impl SimdEngine for Avx2 {
    fn name(&self) -> &'static str {
        "avx2"
    }

    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32) {
        // SAFETY: AVX2 detected at runtime
        unsafe { normalize_u16_to_f32_avx2(input, output, max_val) }
    }

    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32> {
        unsafe { apply_ccm_avx2(rgb, matrix) }
    }

    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32> {
        unsafe { apply_ae_gain_avx2(rgb, gain) }
    }

    fn display_output(&self, rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8> {
        unsafe { display_output_avx2(rgb, src_w, src_h, target_w) }
    }

    fn bilinear_sample_4ch(&self, src: &[u8], width: u32, height: u32, x: f32, y: f32) -> [u8; 4] {
        unsafe { super::sse2::bilinear_sample_4ch_sse2(src, width, height, x, y) }
    }
}
