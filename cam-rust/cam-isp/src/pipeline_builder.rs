//! PipelineBuilder — fluent API for constructing ISP pipelines.
//!
//! Instead of manually boxing blocks and calling wire_blocks:
//!
//! ```ignore
//! let onnx = PipelineBuilder::new(1920, 1080)
//!     .unpack()
//!     .demosaic(0)       // 0=binning, 1=bilinear, 2=mhc
//!     .gamma(2.2)
//!     .sharpen(0.5)
//!     .display()
//!     .compose()
//!     .unwrap();
//! ```

use crate::blocks::*;
use crate::pipeline::{GraphComposer, IspBlock, PipelineStats};

pub struct PipelineBuilder {
    blocks: Vec<Box<dyn IspBlock>>,
    #[allow(dead_code)]
    width: u32,
    #[allow(dead_code)]
    height: u32,
}

impl PipelineBuilder {
    pub fn new(w: u32, h: u32) -> Self {
        Self { blocks: Vec::new(), width: w, height: h }
    }

    /// Add unpack block for Bayer input.
    pub fn unpack(mut self) -> Self {
        self.blocks.push(Box::new(UnpackBlock::new()));
        self
    }

    /// Add unpack block with concrete dimensions.
    pub fn unpack_dims(mut self, h: i64, w: i64) -> Self {
        self.blocks.push(Box::new(UnpackBlock::new().with_concrete_dims(h, w)));
        self
    }

    /// Add demosaic + CCM block.
    /// `bayer_pattern`: 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR
    pub fn demosaic(mut self, bayer_pattern: i32) -> Self {
        self.blocks.push(Box::new(DemosaicCcmBlock::new(bayer_pattern)));
        self
    }

    /// Add gamma block.
    pub fn gamma(mut self, g: f32) -> Self {
        self.blocks.push(Box::new(GammaBlock::new(g)));
        self
    }

    /// Add sharpen block.
    pub fn sharpen(mut self, strength: f32) -> Self {
        self.blocks.push(Box::new(SharpenBlock::new(strength)));
        self
    }

    /// Add auto-contrast block.
    pub fn contrast(mut self, strength: f32) -> Self {
        self.blocks.push(Box::new(AutoContrastBlock::new(strength)));
        self
    }

    /// Add warp block (EIS/GDC).
    pub fn warp(mut self, grid_w: u32) -> Self {
        self.blocks.push(Box::new(WarpGridBlock::new(grid_w, grid_w * 3 / 4)));
        self
    }

    /// Add warp block with GDC distortion correction.
    pub fn warp_gdc(mut self, grid_w: u32, k1: f32) -> Self {
        self.blocks.push(Box::new(
            WarpGridBlock::new(grid_w, grid_w * 3 / 4).with_gdc(k1, 0.0, 0.0)
        ));
        self
    }

    /// Add chromatic aberration correction.
    pub fn chromatic_aberration(mut self) -> Self {
        self.blocks.push(Box::new(
            ChromaticAberrationBlock::new().with_radial_correction(self.height, self.width, 1.0)
        ));
        self
    }

    /// Add temporal denoise.
    pub fn denoise(mut self, threshold: f32) -> Self {
        self.blocks.push(Box::new(TemporalDenoiseBlock::new().with_threshold(threshold)));
        self
    }

    /// Add noise estimation.
    pub fn noise_estimate(mut self) -> Self {
        self.blocks.push(Box::new(NoiseEstimateBlock::new()));
        self
    }

    /// Add display output (RGB).
    pub fn display(mut self) -> Self {
        self.blocks.push(Box::new(DisplayBlock::new(self.width)));
        self
    }

    /// Add display with explicit output width.
    pub fn display_w(mut self, w: u32) -> Self {
        self.blocks.push(Box::new(DisplayBlock::new(w)));
        self
    }

    /// Add dynamic resize.
    pub fn resize(mut self, target_w: u32, target_h: u32) -> Self {
        self.blocks.push(Box::new(DynResizeBlock::new(target_w, target_h)));
        self
    }

    /// Add aspect-ratio crop.
    pub fn crop_16_9(mut self) -> Self {
        self.blocks.push(Box::new(AspectCropBlock::ratio_16_9()));
        self
    }

    /// Add tone block.
    pub fn tone(mut self) -> Self {
        self.blocks.push(Box::new(ToneBlock::new()));
        self
    }

    /// Add normalize block.
    pub fn normalize(mut self) -> Self {
        self.blocks.push(Box::new(NormalizeBlock::new()));
        self
    }

    /// Add grayscale block.
    pub fn grayscale(mut self) -> Self {
        self.blocks.push(Box::new(GrayscaleBlock::new()));
        self
    }

    /// Validate the pipeline configuration before composing.
    /// Returns Ok(()) if valid, or Err with a list of issues.
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut issues = Vec::new();
        if self.blocks.is_empty() {
            issues.push("Pipeline has no blocks".into());
        }
        if self.width == 0 || self.height == 0 {
            issues.push(format!("Invalid resolution: {}x{}", self.width, self.height));
        }
        if issues.is_empty() { Ok(()) } else { Err(issues) }
    }

    /// Get list of block IDs in order (for inspection).
    pub fn block_ids(&self) -> Vec<String> {
        self.blocks.iter().map(|b| b.id().to_string()).collect()
    }

    /// Remove a block by ID. Returns true if found and removed.
    pub fn remove_block(&mut self, id: &str) -> bool {
        if let Some(pos) = self.blocks.iter().position(|b| b.id() == id) {
            self.blocks.remove(pos);
            true
        } else {
            false
        }
    }

    /// Replace a block by ID. Returns true if found and replaced.
    pub fn replace_block(&mut self, id: &str, new_block: Box<dyn crate::pipeline::IspBlock>) -> bool {
        if let Some(pos) = self.blocks.iter().position(|b| b.id() == id) {
            self.blocks[pos] = new_block;
            true
        } else {
            false
        }
    }

    /// Wire and compose the pipeline, returning raw ONNX bytes.
    pub fn compose(self) -> Result<Vec<u8>, String> {
        let mut blocks = self.blocks;
        GraphComposer::wire_blocks(&mut blocks);
        GraphComposer::compose_from_vec(
            &blocks.iter().map(|b| b.as_ref()).collect::<Vec<_>>(),
            &[],
            16,
        )
    }

    /// Wire and compose with full stats + validation.
    pub fn compose_full(self) -> Result<(Vec<u8>, PipelineStats, Vec<String>), String> {
        let mut blocks = self.blocks;
        GraphComposer::compose_full(&mut blocks, &[], 16)
    }

    /// Wire and compose with specific resolution for accurate FLOPs/memory estimates.
    pub fn compose_full_at(self, w: u32, h: u32) -> Result<(Vec<u8>, PipelineStats, Vec<String>), String> {
        let mut blocks = self.blocks;
        GraphComposer::compose_full_at(&mut blocks, &[], 16, w, h)
    }

    /// Get block count.
    pub fn block_count(&self) -> usize {
        self.blocks.len()
    }

    /// Compose and convert to MNN in one call.
    pub fn to_mnn(self, output_path: &str) -> Result<(Vec<u8>, String), String> {
        let mut blocks = self.blocks;
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = GraphComposer::compose_from_vec(&refs, &[], 16)?;
        let mnn_path = if output_path.ends_with(".mnn") {
            output_path.to_string()
        } else {
            format!("{}.mnn", output_path)
        };
        let onnx_path = format!("{}.onnx", mnn_path.trim_end_matches(".mnn"));
        std::fs::write(&onnx_path, &onnx).map_err(|e| format!("write onnx: {}", e))?;
        crate::mnn_converter::convert_onnx_to_mnn(
            &onnx_path, &mnn_path, None).map_err(|e| format!("convert: {}", e))?;
        Ok((onnx, mnn_path))
    }

    /// Create a pipeline from an OptProfile — one-liner pipeline generation.
    ///
    /// ```ignore
    /// let onnx = PipelineBuilder::from_profile(1920, 1080, PerfTier::High)
    ///     .compose().unwrap();
    /// ```
    pub fn from_profile(w: u32, h: u32, tier: crate::optimizer::PerfTier) -> Self {
        use crate::optimizer::OptProfile;
        let p = OptProfile::auto_select(w, h, tier);
        let mut builder = Self::new(w, h)
            .unpack()
            .demosaic(p.demosaic_algo);

        if p.sharpen_strength > 0.0 {
            builder = builder.sharpen(p.sharpen_strength);
        }
        if p.contrast_strength > 1.0 {
            builder = builder.contrast(p.contrast_strength);
        }
        if p.denoise_threshold < 0.5 {
            builder = builder.denoise(p.denoise_threshold);
        }
        if p.needs_warp() {
            builder = builder.warp(p.warp_grid_size);
        }
        if p.needs_ca() {
            builder = builder.chromatic_aberration();
        }
        builder.display()
    }

    /// Add a histogram block for AE feedback.
    pub fn histogram(mut self) -> Self {
        self.blocks.push(Box::new(CoarseHistogramBlock::new(16)));
        self
    }

    /// Compose and return comprehensive per-block statistics.
    pub fn all_stats(self, w: u32, h: u32) -> Result<Vec<(String, u64, u64)>, String> {
        let mut blocks = self.blocks;
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let (_, flops_detail) = GraphComposer::pipeline_flops_estimate(&refs, w, h);
        let (_, mem_detail) = GraphComposer::pipeline_memory_estimate(&refs, w, h);
        // Merge by block name
        let mut result = Vec::new();
        for (i, (name, flops)) in flops_detail.iter().enumerate() {
            let mem = mem_detail.get(i).map(|m| m.1).unwrap_or(0);
            result.push((name.clone(), *flops, mem));
        }
        Ok(result)
    }

    /// Photo preset: high quality still capture pipeline.
    /// Unpack → Demosaic(MHC) → Gamma → Sharpen → Contrast → Denoise → Display
    pub fn photo_preset(w: u32, h: u32) -> Self {
        Self::new(w, h)
            .unpack()
            .demosaic(2)  // MHC
            .gamma(2.2)
            .sharpen(0.6)
            .contrast(1.3)
            .denoise(0.03)
            .display()
    }

    /// Video preset: balanced quality for 30/60fps capture.
    /// Unpack → Demosaic(Bilinear) → Gamma → Sharpen → Display
    pub fn video_preset(w: u32, h: u32) -> Self {
        Self::new(w, h)
            .unpack()
            .demosaic(1)  // Bilinear
            .gamma(2.2)
            .sharpen(0.4)
            .display()
    }

    /// Night preset: aggressive denoising for low-light.
    /// Unpack → Demosaic(Binning) → Gamma → Denoise → Sharpen → Display
    pub fn night_preset(w: u32, h: u32) -> Self {
        Self::new(w, h)
            .unpack()
            .demosaic(0)  // Binning
            .gamma(2.2)
            .denoise(0.02)
            .sharpen(0.3)
            .display()
    }

    /// Minimal preset: minimal processing for preview/focus.
    /// Unpack → Demosaic(Binning) → Display
    pub fn minimal_preset(w: u32, h: u32) -> Self {
        Self::new(w, h)
            .unpack()
            .demosaic(0)  // Binning
            .display()
    }

    /// Wire, compose, and generate a full analysis report.
    pub fn compose_and_report(self) -> Result<(Vec<u8>, String), String> {
        let mut blocks = self.blocks;
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = GraphComposer::compose_from_vec(&refs, &[], 16)?;
        let report = GraphComposer::pipeline_report(&refs);
        Ok((onnx, report))
    }

    /// Convert builder state to a PipelineConfig for serialization.
    pub fn to_config(&self) -> crate::serializer::PipelineConfig {
        let mut cfg = crate::serializer::PipelineConfig::new(self.width, self.height);
        cfg.block_ids = self.blocks.iter().map(|b| b.id().to_string()).collect();
        cfg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pipeline_builder_basic() {
        let onnx = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .display()
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_pipeline_builder_full() {
        let (onnx, stats, issues) = PipelineBuilder::new(1920, 1080)
            .unpack()
            .demosaic(1)
            .gamma(2.2)
            .sharpen(0.5)
            .contrast(1.2)
            .denoise(0.05)
            .display()
            .compose_full()
            .unwrap();
        assert!(!onnx.is_empty());
        assert!(issues.is_empty());
        assert!(stats.block_count > 0);
        println!("Builder pipeline: {} blocks, {} bytes", stats.block_count, onnx.len());
    }

    #[test]
    fn test_pipeline_builder_warp_gdc() {
        let onnx = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .warp_gdc(32, -0.15)
            .chromatic_aberration()
            .display()
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_pipeline_builder_block_count() {
        let b = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .display();
        assert_eq!(b.block_count(), 3);
    }

    #[test]
    fn test_pipeline_builder_resize_crop() {
        let onnx = PipelineBuilder::new(1920, 1080)
            .unpack()
            .demosaic(0)
            .resize(960, 540)
            .crop_16_9()
            .display()
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_pipeline_builder_tone_normalize() {
        let onnx = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .tone()
            .normalize()
            .grayscale()
            .display()
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_pipeline_builder_full_10_stages() {
        let (onnx, stats, _) = PipelineBuilder::new(1920, 1080)
            .unpack()
            .demosaic(2)
            .gamma(2.2)
            .sharpen(0.5)
            .contrast(1.3)
            .warp_gdc(32, -0.1)
            .chromatic_aberration()
            .denoise(0.05)
            .resize(960, 540)
            .display()
            .compose_full()
            .unwrap();
        assert!(stats.block_count >= 10);
        println!("Full 10-stage: {} bytes", onnx.len());
    }

    #[test]
    fn test_from_profile_high_4k() {
        use crate::optimizer::PerfTier;
        let (onnx, stats, _) = PipelineBuilder::from_profile(3840, 2160, PerfTier::High)
            .compose_full()
            .unwrap();
        assert!(!onnx.is_empty());
        assert!(stats.block_count >= 3);
        println!("High 4K: {} blocks, {} bytes", stats.block_count, onnx.len());
    }

    #[test]
    fn test_from_profile_medium_fhd() {
        use crate::optimizer::PerfTier;
        let onnx = PipelineBuilder::from_profile(1920, 1080, PerfTier::Medium)
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_from_profile_low_vga() {
        use crate::optimizer::PerfTier;
        let onnx = PipelineBuilder::from_profile(640, 480, PerfTier::Low)
            .compose()
            .unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_compose_and_report() {
        let (onnx, report) = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .display()
            .compose_and_report()
            .unwrap();
        assert!(!onnx.is_empty());
        assert!(report.contains("Pipeline Report"));
        assert!(report.contains("unpack"));
        assert!(report.contains("demosaic"));
        println!("{}", report);
    }

    #[test]
    fn test_to_config_roundtrip() {
        let builder = PipelineBuilder::new(1920, 1080)
            .unpack()
            .demosaic(0)
            .gamma(2.2)
            .sharpen(0.5)
            .display();
        let cfg = builder.to_config();
        assert_eq!(cfg.width, 1920);
        assert_eq!(cfg.height, 1080);
        assert_eq!(cfg.block_count(), 5);
        let text = cfg.to_text();
        let loaded = crate::serializer::PipelineConfig::from_text(&text).unwrap();
        assert_eq!(loaded.block_ids, vec!["unpack", "demosaic_ccm", "gamma", "sharpen", "display"]);
    }

    #[test]
    fn test_compose_full_at_resolution() {
        let (onnx, stats, _) = PipelineBuilder::new(1920, 3840)
            .unpack()
            .demosaic(2)
            .display()
            .compose_full_at(3840, 2160)
            .unwrap();
        assert!(!onnx.is_empty());
        assert!(stats.estimated_flops > 0);
        assert!(stats.estimated_memory_bytes > 0);
        println!("4K: {:.1} MFLOPs, {:.1} KB",
            stats.estimated_flops as f64 / 1e6,
            stats.estimated_memory_bytes as f64 / 1024.0);
    }

    #[test]
    fn test_histogram_block() {
        let (onnx, stats, _) = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .histogram()
            .display()
            .compose_full()
            .unwrap();
        assert!(!onnx.is_empty());
        assert!(stats.block_count >= 4);
        assert!(stats.block_names.iter().any(|n| n.contains("histogram")));
    }

    #[test]
    fn test_all_stats() {
        let stats = PipelineBuilder::new(1920, 1080)
            .unpack()
            .demosaic(1)
            .gamma(2.2)
            .sharpen(0.5)
            .display()
            .all_stats(1920, 1080)
            .unwrap();
        assert!(stats.len() >= 5);
        for (name, flops, mem) in &stats {
            println!("  {:<20} {:.1} MFLOPs  {:.1} KB", name, *flops as f64 / 1e6, *mem as f64 / 1024.0);
        }
        // All blocks should have non-zero FLOPs
        assert!(stats.iter().all(|(_, f, _)| *f > 0));
    }

    #[test]
    fn test_photo_preset() {
        let (onnx, stats, _) = PipelineBuilder::photo_preset(1920, 1080)
            .compose_full().unwrap();
        assert!(!onnx.is_empty());
        assert!(stats.block_count >= 6);
        println!("Photo: {} blocks, {} bytes", stats.block_count, onnx.len());
    }

    #[test]
    fn test_video_preset() {
        let onnx = PipelineBuilder::video_preset(1920, 1080).compose().unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_night_preset() {
        let onnx = PipelineBuilder::night_preset(1920, 1080).compose().unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_minimal_preset() {
        let onnx = PipelineBuilder::minimal_preset(640, 480).compose().unwrap();
        assert!(!onnx.is_empty());
    }

    #[test]
    fn test_block_ids() {
        let ids = PipelineBuilder::new(640, 480)
            .unpack()
            .demosaic(0)
            .gamma(2.2)
            .display()
            .block_ids();
        assert_eq!(ids, vec!["unpack", "demosaic_ccm", "gamma", "display"]);
    }

    #[test]
    fn test_block_count() {
        let b = PipelineBuilder::new(640, 480).unpack().demosaic(0).display();
        assert_eq!(b.block_count(), 3);
    }

    #[test]
    fn test_remove_block() {
        let mut b = PipelineBuilder::new(640, 480)
            .unpack().demosaic(0).gamma(2.2).display();
        assert!(b.remove_block("gamma"));
        assert!(!b.remove_block("gamma"));
        assert_eq!(b.block_ids(), vec!["unpack", "demosaic_ccm", "display"]);
    }

    #[test]
    fn test_validate_empty() {
        let b = PipelineBuilder::new(0, 0);
        assert!(b.validate().is_err());
    }

    #[test]
    fn test_validate_valid() {
        let b = PipelineBuilder::new(640, 480).unpack().display();
        assert!(b.validate().is_ok());
    }
}
