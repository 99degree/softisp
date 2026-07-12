//! Rectifier Model Generator
//!
//! Generates a mock ONNX model for the ISP Rectifier with the same
//! input/output interface (267→20) using simple operations.
//!
//! The model performs:
//! - Input: histogram\[256\] + metadata\[11\] = 267 dims
//! - Simple linear transformation
//! - Output: wb\[3\] + ccm\[9\] + tone\[7\] + zoom\[1\] = 20 dims

use crate::onnx::proto::Proto;

/// Generate a mock rectifier ONNX model.
///
/// Returns ONNX ModelProto bytes with:
/// - Input: "input" [1, 267] FLOAT
/// - Output: "output" [1, 20] FLOAT
/// - Operations: Reshape → MatMul → Add → Reshape
pub fn generate_rectifier_model() -> Vec<u8> {
    // Weight matrix: 267 → 20 (identity-like transformation)
    // For simplicity, we use a fixed weight that produces reasonable defaults
    let weights = create_weight_matrix();
    let bias = create_bias_vector();

    // ONNX nodes
    let nodes = vec![
        // Reshape input to [267]
        Proto::node("Reshape", &["input", "reshape_shape"], &["input_flat"], &[]),
        // MatMul: input_flat × weights
        Proto::node("MatMul", &["input_flat", "weights"], &["hidden"], &[]),
        // Add bias
        Proto::node("Add", &["hidden", "bias"], &["output_raw"], &[]),
        // Reshape output to [1, 20]
        Proto::node("Reshape", &["output_raw", "output_shape"], &["output"], &[]),
    ];

    // Initializers (weights and constants)
    let initializers = vec![
        // reshape_shape: [267]
        Proto::tensor_proto_int64("reshape_shape", &[267i64]),
        // weights: [267, 20]
        Proto::tensor_proto_float("weights", &[267, 20], &weights),
        // bias: [20]
        Proto::tensor_proto_float("bias", &[20], &bias),
        // output_shape: [1, 20]
        Proto::tensor_proto_int64("output_shape", &[1i64, 20i64]),
    ];

    // Input/Output specs
    let inputs = vec![Proto::value_info(
        "input",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(267)],
        1,
    )];

    let outputs = vec![Proto::value_info(
        "output",
        &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(20)],
        1,
    )];

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

/// Create weight matrix for 267→20 transformation.
///
/// This is a simple linear model that maps:
/// - histogram[0:256] → wb[0:3] (weighted sum)
/// - histogram[0:256] → ccm[3:12] (identity matrix)
/// - histogram[0:256] → tone[12:19] (linear ramp)
/// - metadata[256:267] → zoom[19] (weighted sum)
fn create_weight_matrix() -> Vec<f32> {
    let mut weights = vec![0.0f32; 267 * 20];

    // WB gains: weighted average of histogram
    // R channel: weight first 1/3 of histogram
    for i in 0..85 {
        weights[i * 20 + 0] = 1.0 / 85.0; // WB R
    }
    // G channel: weight middle 1/3
    for i in 85..170 {
        weights[i * 20 + 1] = 1.0 / 85.0; // WB G
    }
    // B channel: weight last 1/3
    for i in 170..256 {
        weights[i * 20 + 2] = 1.0 / 86.0; // WB B
    }

    // CCM: identity matrix (diagonal = 1.0)
    // Row 0: R→R
    weights[0 * 20 + 3] = 1.0;
    // Row 1: G→G
    weights[1 * 20 + 4] = 1.0;
    // Row 2: B→B
    weights[2 * 20 + 5] = 1.0;

    // Tone curve: linear ramp [0, 1/6, 2/6, ..., 1]
    for i in 0..256 {
        let hist_val = i as f32 / 256.0;
        for j in 0..7 {
            let target = j as f32 / 6.0;
            weights[i * 20 + 12 + j] = (1.0 - (hist_val - target).abs()) / 7.0;
        }
    }

    // Zoom: use metadata[0] (CCT) as input
    weights[256 * 20 + 19] = 0.0; // Will be overridden by bias

    weights
}

/// Create bias vector for 20-dim output.
fn create_bias_vector() -> Vec<f32> {
    let mut bias = vec![0.0f32; 20];

    // WB gains: default to 1.0
    bias[0] = 1.0; // R
    bias[1] = 1.0; // G
    bias[2] = 1.0; // B

    // CCM: identity matrix diagonal
    bias[3] = 1.0; // R→R
    bias[4] = 1.0; // G→G
    bias[5] = 1.0; // B→B

    // Tone curve: linear [0, 1/6, 2/6, ..., 1]
    for i in 0..7 {
        bias[12 + i] = i as f32 / 6.0;
    }

    // Zoom: default 1.0
    bias[19] = 1.0;

    bias
}

/// Save model to file.
pub fn save_model(path: &str) -> Result<(), std::io::Error> {
    let model = generate_rectifier_model();
    std::fs::write(path, &model)?;
    log::info!("Saved rectifier model to {} ({} bytes)", path, model.len());
    Ok(())
}

#[cfg(test)]
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
    fn test_weight_matrix_size() {
        let weights = create_weight_matrix();
        assert_eq!(weights.len(), 267 * 20);
    }

    #[test]
    fn test_bias_vector_size() {
        let bias = create_bias_vector();
        assert_eq!(bias.len(), 20);
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
