use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Resize block — spatial scaling via ONNX Resize (nearest).
///
/// Downscale (scale < 1): `[1, C, H, W] → [1, C, H×s, W×s]`
/// Upscale   (scale > 1): `[1, C, H, W] → [1, C, H×s, W×s]`
///
/// Used to run aux blocks (FCS, LDCI, EE) at half resolution
/// for 4× fewer pixels, dramatically reducing Conv cost.
///
/// Uses `nearest` mode (fastest, good enough for post-processing).
use std::sync::atomic::{AtomicUsize, Ordering};
static RESIZE_COUNTER: AtomicUsize = AtomicUsize::new(0);

pub struct ResizeBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Scale factor (e.g. 0.5 for half, 2.0 for double).
    pub scale: f32,
    pub concrete_h: Option<i64>,
    pub concrete_w: Option<i64>,
    /// Number of channels (1=grayscale, 3=RGB, 4=Bayer).
    pub channels: i64,
    /// Skip resize when total pixels < threshold (0=always resize).
    /// Useful for 4K→2K resize that should only run at high resolution.
    pub skip_below_pixels: i64,
}

impl ResizeBlock {
    /// Create a resize block with the given scale factor.
    /// - `0.5` for half-resolution downscale
    /// - `2.0` for full-resolution upscale
    pub fn new(scale: f32) -> Self {
        let idx = RESIZE_COUNTER.fetch_add(1, Ordering::Relaxed);
        let name = if scale < 1.0 {
            format!("resize_down_{}", idx)
        } else {
            format!("resize_up_{}", idx)
        };
        Self {
            id: name.clone(),
            prev: None,
            next: None,
            frame_tensor: format!("{}/frame", name),
            input_source: String::new(),
            scale,
            concrete_h: None,
            concrete_w: None,
            channels: 3,
            skip_below_pixels: 0,
        }
    }

    /// Set number of channels (default: 3 for RGB).
    /// Use 4 for Bayer domain resize.
    pub fn with_channels(mut self, ch: i64) -> Self {
        self.channels = ch;
        self
    }

    /// Skip resize when total pixels (H×W) < threshold.
    /// Useful for 4K-only resize: skip_below_pixels(1920*1080)
    pub fn with_skip_below(mut self, threshold_pixels: i64) -> Self {
        self.skip_below_pixels = threshold_pixels;
        self
    }

    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.concrete_h = Some(h);
        self.concrete_w = Some(w);
        self
    }

    fn out_h(&self) -> Option<i64> {
        self.concrete_h
            .map(|h| (h as f64 * self.scale as f64).round() as i64)
    }

    fn out_w(&self) -> Option<i64> {
        self.concrete_w
            .map(|w| (w as f64 * self.scale as f64).round() as i64)
    }
}

impl IspBlock for ResizeBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        self.id.clone()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next = Some(block);
    }
    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let ns = self.tensor_ns();
        let ch = self.channels;
        let dims = if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(ch),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(ch),
                Proto::tensor_dim_param(&format!("{}_H", ns)),
                Proto::tensor_dim_param(&format!("{}_W", ns)),
            ]
        };
        Some(Proto::value_info(&self.input_source, &dims, 1))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let ns = self.tensor_ns();
        let ch = self.channels;
        let dims = if let (Some(oh), Some(ow)) = (self.out_h(), self.out_w()) {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(ch),
                Proto::tensor_dim_value(oh),
                Proto::tensor_dim_value(ow),
            ]
        } else {
            vec![
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(ch),
                Proto::tensor_dim_param(&format!("{}_OH", ns)),
                Proto::tensor_dim_param(&format!("{}_OW", ns)),
            ]
        };
        Some(Proto::value_info(&self.frame_tensor, &dims, 1))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();

        // Adaptive: skip resize when input is already small enough
        // Check if input dimensions are close to output dimensions
        let skip = if self.skip_below_pixels > 0 {
            if let (Some(h), Some(w)) = (self.concrete_h, self.concrete_w) {
                let input_pixels = h * w;
                input_pixels <= self.skip_below_pixels
            } else {
                false // unknown dims → always resize
            }
        } else {
            false
        };

        if skip {
            // Identity: pass input through unchanged
            vec![Proto::node(
                "Identity",
                &[&self.input_source],
                &[&self.frame_tensor],
                &[],
            )]
        } else {
            vec![Proto::node(
                "Resize",
                &[&self.input_source, "", "", &format!("{}/scales", ns)],
                &[&self.frame_tensor],
                &[
                    Proto::attribute_string("mode", "nearest"),
                    Proto::attribute_int("coordinate_transformation_mode", 0), // half_pixel
                ],
            )]
        }
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![(format!("{}/scales", ns).to_string(), 1, vec![4])]
    }

    fn extra_input_defaults(&self) -> Vec<(String, Vec<u8>)> {
        let ns = self.tensor_ns();
        vec![(
            format!("{}/scales", ns).to_string(),
            [1.0f32, 1.0f32, self.scale, self.scale]
                .iter()
                .flat_map(|v| v.to_ne_bytes())
                .collect::<Vec<u8>>(),
        )]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::GraphComposer;

    #[test]
    fn test_resize_block_nodes() {
        let down = ResizeBlock::new(0.5);
        assert_eq!(down.nodes().len(), 1, "Resize should produce 1 node");
        assert_eq!(
            down.extra_input_defaults().len(),
            1,
            "Should have 1 initializer (scales)"
        );
    }

    #[test]
    fn test_resize_block_scale_values() {
        let down = ResizeBlock::new(0.5);
        assert!(down.id.starts_with("resize_down_"));
        assert!((down.scale - 0.5).abs() < 1e-6);

        let up = ResizeBlock::new(2.0);
        assert!(up.id.starts_with("resize_up_"));
        assert!((up.scale - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_resize_block_concrete_dims() {
        let down = ResizeBlock::new(0.5).with_concrete_dims(1080, 1920);
        assert_eq!(down.out_h(), Some(540));
        assert_eq!(down.out_w(), Some(960));

        let up = ResizeBlock::new(2.0).with_concrete_dims(540, 960);
        assert_eq!(up.out_h(), Some(1080));
        assert_eq!(up.out_w(), Some(1920));
    }

    #[test]
    fn test_resize_pipeline_integration() {
        let b1: Box<dyn IspBlock> =
            Box::new(crate::blocks::RawInputBlock::new().with_concrete_dims(48, 64));
        let b2: Box<dyn IspBlock> = Box::new(crate::blocks::NormalizeBlock::new());
        let b3: Box<dyn IspBlock> =
            Box::new(crate::blocks::CfaBlock::new().with_concrete_dims(48, 64));
        let b4: Box<dyn IspBlock> =
            Box::new(crate::blocks::DemosaicCcmBlock::new(2).with_concrete_dims(24, 32));
        let b5: Box<dyn IspBlock> = Box::new(ResizeBlock::new(0.5).with_concrete_dims(24, 32));
        let b6: Box<dyn IspBlock> = Box::new(ResizeBlock::new(2.0).with_concrete_dims(12, 16));

        let mut blocks: Vec<Box<dyn IspBlock>> = vec![b1, b2, b3, b4, b5, b6];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let result = GraphComposer::compose_from_vec(&refs, &[], 16);
        assert!(
            result.is_ok(),
            "Pipeline through ResizeBlock: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_resize_identity() {
        let b = ResizeBlock::new(1.0).with_concrete_dims(64, 64);
        let nodes = b.nodes();
        assert!(!nodes.is_empty());
    }

    #[test]
    fn test_resize_tensor_ns() {
        let b = ResizeBlock::new(0.5).with_concrete_dims(64, 64);
        assert!(!b.tensor_ns().is_empty());
    }
}
