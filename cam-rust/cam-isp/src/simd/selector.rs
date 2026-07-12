//! Backend trait and runtime selector — picks the fastest available SIMD.
//!
//! # Selection Order (best → worst)
//! | Priority | Backend     | When                          |
//! |----------|-------------|-------------------------------|
//! | 4        | `dotprod`   | aarch64 + dotprod + fp16 + neon |
//! | 3        | `fp16`      | aarch64 + fp16 + neon          |
//! | 2        | `neon`      | aarch64 + neon                 |
//! | 1        | `avx2`/`sse2`| x86_64                        |
//! | 0        | `scalar`    | always                         |

use std::sync::OnceLock;

/// Trait for a SIMD backend.
/// All operations must produce bit-identical output to the scalar backend.
pub trait SimdEngine: Send + Sync {
    fn name(&self) -> &'static str;

    /// u16 → f32 normalized to [0, 1].
    fn normalize_u16_to_f32(&self, input: &[u16], output: &mut [f32], max_val: f32);

    /// 3×3 CCM matrix on RGB float array.
    fn apply_ccm(&self, rgb: &[f32], matrix: &[f32; 9]) -> Vec<f32>;

    /// `out[i] = (in[i] * gain).min(1.0)`.
    fn apply_ae_gain(&self, rgb: &[f32], gain: f32) -> Vec<f32>;

    /// f32 RGB → u8 BGRA, optionally resized.
    fn display_output(&self, rgb: &[f32], src_w: usize, src_h: usize, target_w: usize) -> Vec<u8>;

    /// Bilinear sample from a BGRA/RGBA image.
    /// Returns 4 interpolated bytes for position (x, y) in source space.
    /// Clamps to image bounds.
    fn bilinear_sample_4ch(&self, src: &[u8], width: u32, height: u32, x: f32, y: f32) -> [u8; 4];
}

/// Detected backend kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendKind {
    Scalar,
    Sse2,
    Avx2,
    Neon,
    NeonFp16,
    NeonDotprod,
    Avx512,
}

// ─────────────────────────────────────────────────────────────────────────────
//  Backend detection
// ─────────────────────────────────────────────────────────────────────────────

fn detect_best() -> Box<dyn SimdEngine> {
    // ARM64 detection chain (most → least powerful)
    #[cfg(target_arch = "aarch64")]
    {
        // Check for dotprod (ARMv8.4)
        if std::arch::is_aarch64_feature_detected!("dotprod")
            && std::arch::is_aarch64_feature_detected!("fp16")
            && std::arch::is_aarch64_feature_detected!("neon")
        {
            return Box::new(super::neon_dotprod::NeonDotprod::new());
        }

        // Check for fp16 (ARMv8.2)
        if std::arch::is_aarch64_feature_detected!("fp16")
            && std::arch::is_aarch64_feature_detected!("neon")
        {
            return Box::new(super::neon_fp16::NeonFp16::new());
        }

        // NEON (ARMv8.0) — baseline
        if std::arch::is_aarch64_feature_detected!("neon") {
            return Box::new(super::neon::Neon::new());
        }
    }

    // x86_64
    #[cfg(target_arch = "x86_64")]
    {
        if std::arch::is_x86_feature_detected!("avx2") {
            return Box::new(super::avx2::Avx2::new());
        }
        if std::arch::is_x86_feature_detected!("sse2") {
            return Box::new(super::sse2::Sse2::new());
        }
    }

    // Scalar — always available
    Box::new(super::scalar::Scalar::new())
}

/// Get the best available SIMD backend (cached, static lifetime).
pub fn best_backend() -> &'static dyn SimdEngine {
    static BACKEND: OnceLock<Box<dyn SimdEngine>> = OnceLock::new();
    &**BACKEND.get_or_init(detect_best)
}

/// Return active backend name.
pub fn active_backend_name() -> &'static str {
    best_backend().name()
}

/// Return detected backend kind.
pub fn backend_kind() -> BackendKind {
    match best_backend().name() {
        "neon-dotprod" => BackendKind::NeonDotprod,
        "neon-fp16" => BackendKind::NeonFp16,
        "neon" => BackendKind::Neon,
        "sse2" => BackendKind::Sse2,
        "avx2" => BackendKind::Avx2,
        "avx512" => BackendKind::Avx512,
        _ => BackendKind::Scalar,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_backend_selection() {
        let be = best_backend();
        assert!(!be.name().is_empty());
        match be.name() {
            "scalar" | "neon" | "neon-fp16" | "neon-dotprod" | "sse2" | "avx2" => (),
            other => panic!("Unknown backend: {}", other),
        }
    }

    #[test]
    fn test_backend_consistent() {
        let ptr1 = best_backend() as *const dyn SimdEngine;
        let ptr2 = best_backend() as *const dyn SimdEngine;
        assert_eq!(ptr1, ptr2);
    }

    #[test]
    fn test_backend_kind_matches_name() {
        let kind = backend_kind();
        let name = active_backend_name();
        match kind {
            BackendKind::Scalar => assert_eq!(name, "scalar"),
            BackendKind::Neon => assert_eq!(name, "neon"),
            BackendKind::NeonFp16 => assert_eq!(name, "neon-fp16"),
            BackendKind::NeonDotprod => assert_eq!(name, "neon-dotprod"),
            _ => {} // x86 backends
        }
    }
}
