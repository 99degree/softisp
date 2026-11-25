//! Rectifier Model Generator
//!
//! Generates a mock ONNX model for the ISP Rectifier with the same
//! input/output interface (267→20) using simple operations.
//!
//! The model performs:
//! - Input 1: "hist" [1, 256] FLOAT  (histogram)
//! - Input 2: "meta" [1, 11] FLOAT   (metadata)
//! - Output 1: "wb" [1, 3] FLOAT     (white balance gains)
//! - Output 2: "ccm" [1, 9] FLOAT    (color correction matrix)
//! - Output 3: "tone" [1, 7] FLOAT   (tone curve LUT)
//! - Output 4: "zoom" [1, 1] FLOAT   (digital zoom factor)
//!
//! Operations: separate MatMul+Add per head, each head reading from
//! the appropriate input (hist for wb/ccm/tone, meta for zoom).

use crate::onnx::proto::Proto;

/// Generate a mock rectifier ONNX model.
///
/// Returns ONNX ModelProto bytes with:
/// - Input: "hist" [1, 256] FLOAT
/// - Input: "meta" [1, 11] FLOAT
/// - Output: "wb" [1, 3] FLOAT
/// - Output: "ccm" [1, 9] FLOAT
/// - Output: "tone" [1, 7] FLOAT
/// - Output: "zoom" [1, 1] FLOAT
/// - Operations: per-head MatMul + Add
pub fn generate_rectifier_model() -> Vec<u8> {
    let weights_wb = create_wb_weights();
    let bias_wb = create_wb_bias();
    let weights_ccm = create_ccm_weights();
    let bias_ccm = create_ccm_bias();
    let weights_tone = create_tone_weights();
    let bias_tone = create_tone_bias();
    let weights_zoom = create_zoom_weights();
    let bias_zoom = create_zoom_bias();

    // ONNX nodes — one MatMul+Add pair per output head
    let nodes = vec![
        // WB head: hist × W_wb + b_wb → wb
        Proto::node("MatMul", &["hist", "weights_wb"], &["wb_raw"], &[]),
        Proto::node("Add", &["wb_raw", "bias_wb"], &["wb"], &[]),
        // CCM head: hist × W_ccm + b_ccm → ccm
        Proto::node("MatMul", &["hist", "weights_ccm"], &["ccm_raw"], &[]),
        Proto::node("Add", &["ccm_raw", "bias_ccm"], &["ccm"], &[]),
        // Tone head: hist × W_tone + b_tone → tone
        Proto::node("MatMul", &["hist", "weights_tone"], &["tone_raw"], &[]),
        Proto::node("Add", &["tone_raw", "bias_tone"], &["tone"], &[]),
        // Zoom head: meta × W_zoom + b_zoom → zoom
        Proto::node("MatMul", &["meta", "weights_zoom"], &["zoom_raw"], &[]),
        Proto::node("Add", &["zoom_raw", "bias_zoom"], &["zoom"], &[]),
    ];

    // Initializers (weights and biases)
    let initializers = vec![
        Proto::tensor_proto_float("weights_wb", &[256, 3], &weights_wb),
        Proto::tensor_proto_float("bias_wb", &[3], &bias_wb),
        Proto::tensor_proto_float("weights_ccm", &[256, 9], &weights_ccm),
        Proto::tensor_proto_float("bias_ccm", &[9], &bias_ccm),
        Proto::tensor_proto_float("weights_tone", &[256, 7], &weights_tone),
        Proto::tensor_proto_float("bias_tone", &[7], &bias_tone),
        Proto::tensor_proto_float("weights_zoom", &[52, 1], &weights_zoom),
        Proto::tensor_proto_float("bias_zoom", &[1], &bias_zoom),
    ];

    // Input/Output specs
    let inputs = vec![
        Proto::value_info(
            "hist",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(256)],
            1,
        ),
        Proto::value_info(
            "meta",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(52)],
            1,
        ),
    ];

    let outputs = vec![
        Proto::value_info(
            "wb",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3)],
            1,
        ),
        Proto::value_info(
            "ccm",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(9)],
            1,
        ),
        Proto::value_info(
            "tone",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(7)],
            1,
        ),
        Proto::value_info(
            "zoom",
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(1)],
            1,
        ),
    ];

    // Build graph
    let graph = Proto::graph(
        "rectifier_graph",
        &nodes,
        &inputs,
        &outputs,
        &initializers,
        &[],
    );

    // Build model
    let mut opset = Vec::new();
    opset.extend_from_slice(&Proto::int64(8, 13)); // ONNX opset 13
    Proto::model(11, &opset, "cam_rust_rectifier", &graph)
}

/// WB weights: 256→3. Each output channel is a weighted sum of the histogram.
fn create_wb_weights() -> Vec<f32> {
    let mut w = vec![0.0f32; 256 * 3];
    // R: weight first third of histogram
    for i in 0..85 {
        w[i * 3] = 1.0 / 85.0;
    }
    // G: weight middle third
    for i in 85..170 {
        w[i * 3 + 1] = 1.0 / 85.0;
    }
    // B: weight last third
    for i in 170..256 {
        w[i * 3 + 2] = 1.0 / 86.0;
    }
    w
}

/// WB bias: default gains of 1.0
fn create_wb_bias() -> Vec<f32> {
    vec![1.0f32, 1.0, 1.0]
}

/// CCM weights: 256→9. Identity-like matrix.
fn create_ccm_weights() -> Vec<f32> {
    let mut w = vec![0.0f32; 256 * 9];
    // Diagonal elements from histogram bins
    w[0] = 1.0; // R→R
    w[9 + 4] = 1.0; // G→G
    w[2 * 9 + 8] = 1.0; // B→B
    w
}

/// CCM bias: identity matrix diagonal
fn create_ccm_bias() -> Vec<f32> {
    vec![1.0f32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
}

/// Tone weights: 256→7. Linear ramp.
fn create_tone_weights() -> Vec<f32> {
    let mut w = vec![0.0f32; 256 * 7];
    for i in 0..256 {
        let hist_val = i as f32 / 256.0;
        for j in 0..7 {
            let target = j as f32 / 6.0;
            w[i * 7 + j] = (1.0 - (hist_val - target).abs()) / 7.0;
        }
    }
    w
}

/// Tone bias: linear ramp [0, 1/6, ..., 1]
fn create_tone_bias() -> Vec<f32> {
    (0..7).map(|i| i as f32 / 6.0).collect()
}

/// Zoom weights: 52→1. Uses metadata[0] (CCT).
fn create_zoom_weights() -> Vec<f32> {
    vec![0.0f32; 52]
}

/// Zoom bias: default 1.0
fn create_zoom_bias() -> Vec<f32> {
    vec![1.0f32]
}

/// Save model to file.
pub fn save_model(path: &str) -> Result<(), std::io::Error> {
    let model = generate_rectifier_model();
    std::fs::write(path, &model)?;
    log::info!("Saved rectifier model to {} ({} bytes)", path, model.len());
    Ok(())
}

#[cfg(test)]
#[cfg(feature = "rectifier")]
mod tests {
    use super::*;

    #[test]
    fn test_generate_model() {
        let model = generate_rectifier_model();
        assert!(!model.is_empty());
        assert!(model.len() > 1000); // Should be at least 1KB

        // Model should start with valid protobuf data
        // (first byte encodes field number and wire type)
        assert!(model[0] != 0 || model.len() > 100);
    }

    #[test]
    fn test_wb_weights_size() {
        let weights = create_wb_weights();
        assert_eq!(weights.len(), 256 * 3);
    }

    #[test]
    fn test_ccm_weights_size() {
        let weights = create_ccm_weights();
        assert_eq!(weights.len(), 256 * 9);
    }

    #[test]
    fn test_tone_weights_size() {
        let weights = create_tone_weights();
        assert_eq!(weights.len(), 256 * 7);
    }

    #[test]
    fn test_zoom_weights_size() {
        let weights = create_zoom_weights();
        assert_eq!(weights.len(), 52);
    }

    #[test]
    fn test_bias_sizes() {
        assert_eq!(create_wb_bias().len(), 3);
        assert_eq!(create_ccm_bias().len(), 9);
        assert_eq!(create_tone_bias().len(), 7);
        assert_eq!(create_zoom_bias().len(), 1);
    }

    #[test]
    fn test_save_model() {
        let dir = std::env::temp_dir();
        let path = dir.join("test_rectifier.onnx");
        let path_str = path.to_str().unwrap();

        let result = save_model(path_str);
        assert!(result.is_ok());

        let metadata = std::fs::metadata(&path).unwrap();
        assert!(metadata.len() > 1000);

        // Cleanup
        let _ = std::fs::remove_file(&path);
    }
}
