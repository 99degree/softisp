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

    /// Get block count.
    pub fn block_count(&self) -> usize {
        self.blocks.len()
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
}
