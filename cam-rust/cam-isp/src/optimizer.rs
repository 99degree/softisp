//! PipelineOptimizer — automatic pipeline configuration based on constraints.
//!
//! Given target resolution and thermal/power constraints, selects optimal
//! block variants (demosaic algorithm, workgroup size, filter strengths).

/// Thermal/power constraint level.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PerfTier {
    /// Max performance: all features, MHC demosaic, large workgroups.
    High,
    /// Balanced: bilinear demosaic, moderate filters.
    Medium,
    /// Low power: binning demosaic, minimal filtering, small workgroups.
    Low,
}

/// Pipeline optimization profile.
#[derive(Debug, Clone)]
pub struct OptProfile {
    pub tier: PerfTier,
    pub width: u32,
    pub height: u32,
    pub demosaic_algo: i32,
    pub sharpen_strength: f32,
    pub denoise_threshold: f32,
    pub contrast_strength: f32,
    pub warp_grid_size: u32,
    pub use_gdc: bool,
    pub use_ca: bool,
    pub output_format: String,
}

impl OptProfile {
    /// Auto-select profile based on resolution and perf tier.
    pub fn auto_select(w: u32, h: u32, tier: PerfTier) -> Self {
        let pixels = w as u64 * h as u64;
        let is_4k = pixels >= 3840 * 2160;
        let is_fhd = pixels >= 1920 * 1080;

        match tier {
            PerfTier::High => Self {
                tier,
                width: w,
                height: h,
                demosaic_algo: if is_4k { 2 } else { 1 },  // MHC for 4K, bilinear for smaller
                sharpen_strength: 0.5,
                denoise_threshold: 0.05,
                contrast_strength: 1.2,
                warp_grid_size: if is_4k { 64 } else { 32 },
                use_gdc: true,
                use_ca: is_fhd,
                output_format: "RGB".into(),
            },
            PerfTier::Medium => Self {
                tier,
                width: w,
                height: h,
                demosaic_algo: if is_4k { 1 } else { 0 },  // bilinear for 4K, binning for smaller
                sharpen_strength: 0.3,
                denoise_threshold: 0.08,
                contrast_strength: 1.1,
                warp_grid_size: if is_4k { 32 } else { 16 },
                use_gdc: false,
                use_ca: false,
                output_format: "RGB".into(),
            },
            PerfTier::Low => Self {
                tier,
                width: w,
                height: h,
                demosaic_algo: 0,  // binning only
                sharpen_strength: 0.0,
                denoise_threshold: 0.15,
                contrast_strength: 1.0,
                warp_grid_size: 0,  // no warp
                use_gdc: false,
                use_ca: false,
                output_format: "RGB".into(),
            },
        }
    }

    /// Check if this profile needs GPU warp.
    pub fn needs_warp(&self) -> bool {
        self.warp_grid_size > 0
    }

    /// Check if this profile needs chromatic aberration.
    pub fn needs_ca(&self) -> bool {
        self.use_ca
    }

    /// Estimated dispatch count for this profile.
    pub fn estimated_dispatches(&self) -> u32 {
        let mut n = 2u32;  // unpack + demosaic
        if self.sharpen_strength > 0.0 { n += 1; }
        if self.denoise_threshold < 0.5 { n += 1; }
        if self.contrast_strength > 1.0 { n += 1; }
        if self.needs_warp() { n += 1; }
        if self.needs_ca() { n += 1; }
        n += 1;  // display
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auto_select_high_4k() {
        let p = OptProfile::auto_select(3840, 2160, PerfTier::High);
        assert_eq!(p.tier, PerfTier::High);
        assert_eq!(p.demosaic_algo, 2); // MHC for 4K high
        assert!(p.use_gdc);
        assert!(p.use_ca);
    }

    #[test]
    fn test_auto_select_medium_fhd() {
        let p = OptProfile::auto_select(1920, 1080, PerfTier::Medium);
        assert_eq!(p.demosaic_algo, 0); // binning for smaller medium
        assert!(!p.use_gdc);
        assert!(!p.use_ca);
    }

    #[test]
    fn test_auto_select_low() {
        let p = OptProfile::auto_select(640, 480, PerfTier::Low);
        assert_eq!(p.demosaic_algo, 0);
        assert_eq!(p.sharpen_strength, 0.0);
        assert!(!p.needs_warp());
    }

    #[test]
    fn test_estimated_dispatches_high() {
        let p = OptProfile::auto_select(1920, 1080, PerfTier::High);
        assert!(p.estimated_dispatches() >= 4); // unpack + demosaic + display + at least 1
    }

    #[test]
    fn test_estimated_dispatches_low() {
        let p = OptProfile::auto_select(640, 480, PerfTier::Low);
        assert!(p.estimated_dispatches() <= 4); // unpack + denoise + display
    }
}
