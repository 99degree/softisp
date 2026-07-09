//! Inference Module - Core inference logic

use crate::types::{FrameMetadata, ISPOptimizedParams};
use tract_onnx::prelude::*;

/// Run inference with tract ONNX
pub struct TractInference {
    model: SimplePlan<TypedFact, Box<dyn TypedOp>, Graph<TypedFact, Box<dyn TypedOp>>>,
}

impl TractInference {
    pub fn new<P: AsRef<std::path::Path>>(model_path: P) -> TractResult<Self> {
        let model = tract_onnx::onnx()
            .model_for_path(model_path)?
            .into_optimized()?
            .into_runnable()?;
        Ok(Self { model })
    }
    
    pub fn run(&mut self, histogram: &[f32], metadata: &[f32]) -> TractResult<ISPOptimizedParams> {
        // Create tensors
        let hist_tensor = Tensor::from_shape(&[1, 256], histogram)?.into();
        let meta_tensor = Tensor::from_shape(&[1, 11], metadata)?.into();
        
        // Run
        let outputs = self.model.run(tvec!(hist_tensor, meta_tensor))?;
        
        // Parse
        let wb = outputs[0].to_array_view::<f32>()?;
        let ccm = outputs[1].to_array_view::<f32>()?;
        let tone = outputs[2].to_array_view::<f32>()?;
        let zoom = outputs[3].to_array_view::<f32>()?;
        
        Ok(ISPOptimizedParams {
            wb_r_gain: wb[[0, 0]],
            wb_g_gain: wb[[0, 1]],
            wb_b_gain: wb[[0, 2]],
            ccm: [
                [ccm[[0, 0]], ccm[[0, 1]], ccm[[0, 2]]],
                [ccm[[0, 3]], ccm[[0, 4]], ccm[[0, 5]]],
                [ccm[[0, 6]], ccm[[0, 7]], ccm[[0, 8]]],
            ],
            tone_curve_lut: tone.iter().map(|&x| x).collect(),
            zoom_factor: zoom[[0, 0]],
        })
    }
    
    pub fn run_batch(
        &mut self,
        histograms: &[f32],
        metadatas: &[f32],
    ) -> TractResult<Vec<ISPOptimizedParams>> {
        let batch_size = histograms.len() / 256;
        
        let hist_tensor = Tensor::from_shape(&[batch_size, 256], histograms)?.into();
        let meta_tensor = Tensor::from_shape(&[batch_size, 11], metadatas)?.into();
        
        let outputs = self.model.run(tvec!(hist_tensor, meta_tensor))?;
        
        let wb = outputs[0].to_array_view::<f32>()?;
        let ccm = outputs[1].to_array_view::<f32>()?;
        let tone = outputs[2].to_array_view::<f32>()?;
        let zoom = outputs[3].to_array_view::<f32>()?;
        
        (0..batch_size).map(|i| {
            Ok(ISPOptimizedParams {
                wb_r_gain: wb[[i, 0]],
                wb_g_gain: wb[[i, 1]],
                wb_b_gain: wb[[i, 2]],
                ccm: [
                    [ccm[[i, 0]], ccm[[i, 1]], ccm[[i, 2]]],
                    [ccm[[i, 3]], ccm[[i, 4]], ccm[[i, 5]]],
                    [ccm[[i, 6]], ccm[[i, 7]], ccm[[i, 8]]],
                ],
                tone_curve_lut: (0..7).map(|j| tone[[i, j]]).collect(),
                zoom_factor: zoom[[i, 0]],
            })
        }).collect()
    }
}

/// High-level inference with automatic clamping
pub struct OptimizedInference {
    inner: TractInference,
    clamp: bool,
}

impl OptimizedInference {
    pub fn new<P: AsRef<std::path::Path>>(model_path: P, clamp: bool) -> TractResult<Self> {
        Ok(Self {
            inner: TractInference::new(model_path)?,
            clamp,
        })
    }
    
    pub fn optimize(&mut self, frame: &FrameMetadata) -> TractResult<ISPOptimizedParams> {
        let (hist, meta) = frame.to_feature_vector();
        let mut params = self.inner.run(&hist, &meta)?;
        
        if self.clamp {
            params.clamp(&crate::types::RegisterLimits::default());
        }
        
        Ok(params)
    }
}