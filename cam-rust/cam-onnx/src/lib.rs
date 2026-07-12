//! ONNX Runtime inference wrapper for the ISP pipeline.
//!
//! Provides a simple API for loading and running ONNX models.
//! Uses the `ort` crate (v2.0.0-rc.12) for ONNX Runtime bindings.
//!
//! # Usage
//!
//! ```rust,ignore
//! use cam_onnx::OnnxSession;
//!
//! // Load from file
//! let session = OnnxSession::from_file("model.onnx")?;
//!
//! // Run inference
//! let output = session.run(
//!     "input",
//!     &input_data,
//!     &[1, 3, 1080, 1920],
//! )?;
//! ```

#[allow(unused_imports)]
use log::info;

/// ONNX model session for inference.
pub struct OnnxSession {
    #[cfg(feature = "ort")]
    session: ort::session::Session,
    model_name: String,
}

impl OnnxSession {
    /// Load an ONNX model from bytes.
    pub fn from_bytes(model_data: &[u8], name: &str) -> Result<Self, String> {
        #[cfg(feature = "ort")]
        {
            let session = ort::session::Session::builder()
                .map_err(|e| format!("ORT session builder failed: {}", e))?
                .commit_from_memory(model_data)
                .map_err(|e| format!("ORT model load failed: {}", e))?;

            info!(
                "OnnxSession: loaded '{}' ({} bytes)",
                name,
                model_data.len()
            );

            Ok(Self {
                session,
                model_name: name.to_string(),
            })
        }

        #[cfg(not(feature = "ort"))]
        {
            let _ = (model_data, name);
            Err("ONNX Runtime not available (enable 'ort' feature)".into())
        }
    }

    /// Load an ONNX model from a file path.
    pub fn from_file(path: &str) -> Result<Self, String> {
        let data = std::fs::read(path).map_err(|e| format!("Failed to read {}: {}", path, e))?;
        let name = std::path::Path::new(path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("model");
        Self::from_bytes(&data, name)
    }

    /// Run inference with a named input.
    ///
    /// - `input_name`: Name of the input tensor (e.g., "input")
    /// - `data`: Input data as f32 slice
    /// - `shape`: Input shape (e.g., &[1, 3, 1080, 1920])
    ///
    /// Returns output data as `Vec<f32>`.
    pub fn run(
        &mut self,
        input_name: &str,
        data: &[f32],
        shape: &[i64],
    ) -> Result<Vec<f32>, String> {
        #[cfg(feature = "ort")]
        {
            use ort::inputs;

            let tensor =
                ort::value::Tensor::from_array((shape.to_vec(), data.to_vec().into_boxed_slice()))
                    .map_err(|e| format!("Failed to create input tensor: {}", e))?
                    .upcast();

            let outputs = self
                .session
                .run(inputs![input_name => tensor])
                .map_err(|e| format!("ORT inference failed: {}", e))?;

            // Get first output
            let (_, output_val) = outputs.iter().next().ok_or("No outputs returned")?;

            let (shape, data) = output_val
                .try_extract_tensor::<f32>()
                .map_err(|e| format!("Failed to extract output: {}", e))?;

            info!(
                "OnnxSession '{}': output shape {:?}, {} values",
                self.model_name,
                shape,
                data.len()
            );

            Ok(data.to_vec())
        }

        #[cfg(not(feature = "ort"))]
        {
            let _ = (input_name, data, shape);
            Err("ONNX Runtime not available (enable 'ort' feature)".into())
        }
    }

    /// Run inference with i16 input (common for raw sensor data).
    pub fn run_i16(
        &mut self,
        input_name: &str,
        data: &[i16],
        shape: &[i64],
    ) -> Result<Vec<f32>, String> {
        #[cfg(feature = "ort")]
        {
            use ort::inputs;

            let tensor =
                ort::value::Tensor::from_array((shape.to_vec(), data.to_vec().into_boxed_slice()))
                    .map_err(|e| format!("Failed to create input tensor: {}", e))?
                    .upcast();

            let outputs = self
                .session
                .run(inputs![input_name => tensor])
                .map_err(|e| format!("ORT inference failed: {}", e))?;

            let (_, output_val) = outputs.iter().next().ok_or("No outputs returned")?;

            let (shape, data) = output_val
                .try_extract_tensor::<f32>()
                .map_err(|e| format!("Failed to extract output: {}", e))?;

            info!(
                "OnnxSession '{}': output shape {:?}, {} values",
                self.model_name,
                shape,
                data.len()
            );

            Ok(data.to_vec())
        }

        #[cfg(not(feature = "ort"))]
        {
            let _ = (input_name, data, shape);
            Err("ONNX Runtime not available (enable 'ort' feature)".into())
        }
    }

    /// Run inference with u8 input (common for image data).
    pub fn run_u8(
        &mut self,
        input_name: &str,
        data: &[u8],
        shape: &[i64],
    ) -> Result<Vec<f32>, String> {
        #[cfg(feature = "ort")]
        {
            use ort::inputs;

            let tensor =
                ort::value::Tensor::from_array((shape.to_vec(), data.to_vec().into_boxed_slice()))
                    .map_err(|e| format!("Failed to create input tensor: {}", e))?
                    .upcast();

            let outputs = self
                .session
                .run(inputs![input_name => tensor])
                .map_err(|e| format!("ORT inference failed: {}", e))?;

            let (_, output_val) = outputs.iter().next().ok_or("No outputs returned")?;

            let (shape, data) = output_val
                .try_extract_tensor::<f32>()
                .map_err(|e| format!("Failed to extract output: {}", e))?;

            info!(
                "OnnxSession '{}': output shape {:?}, {} values",
                self.model_name,
                shape,
                data.len()
            );

            Ok(data.to_vec())
        }

        #[cfg(not(feature = "ort"))]
        {
            let _ = (input_name, data, shape);
            Err("ONNX Runtime not available (enable 'ort' feature)".into())
        }
    }

    /// Get model name.
    pub fn model_name(&self) -> &str {
        &self.model_name
    }

    /// Get input names (if ort feature enabled).
    pub fn input_names(&self) -> Vec<String> {
        #[cfg(feature = "ort")]
        {
            self.session
                .inputs()
                .iter()
                .map(|i| i.name().clone())
                .collect()
        }
        #[cfg(not(feature = "ort"))]
        vec![]
    }

    /// Get output names (if ort feature enabled).
    pub fn output_names(&self) -> Vec<String> {
        #[cfg(feature = "ort")]
        {
            self.session
                .outputs()
                .iter()
                .map(|o| o.name().clone())
                .collect()
        }
        #[cfg(not(feature = "ort"))]
        vec![]
    }
}

/// ONNX model composer for the ISP pipeline.
/// Generates ONNX protobuf format from pipeline blocks.
pub struct OnnxModelComposer;

impl OnnxModelComposer {
    /// Create an ONNX graph from nodes and initializers.
    pub fn compose_model(
        nodes: Vec<Vec<u8>>,
        initializers: Vec<Vec<u8>>,
        input_value_infos: Vec<Vec<u8>>,
        output_value_infos: Vec<Vec<u8>>,
    ) -> Vec<u8> {
        // Delegate to cam_isp's proto module if available
        // For now, return empty
        let _ = (nodes, initializers, input_value_infos, output_value_infos);
        vec![]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_session_creation_no_ort() {
        // Without ort feature, should fail gracefully
        let result = OnnxSession::from_bytes(b"not a model", "test");
        assert!(result.is_err());
    }

    #[test]
    fn test_composer_empty() {
        let model = OnnxModelComposer::compose_model(vec![], vec![], vec![], vec![]);
        assert!(model.is_empty());
    }
}
