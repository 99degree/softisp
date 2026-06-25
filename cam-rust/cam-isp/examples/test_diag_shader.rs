//! test_diag_shader.rs — Minimal diagnostic: write constant 1.0-4.0 via VulkanFuse
//!
//! Generates an ONNX model with a single Extra op containing a trivial SPIR-V
//! shader that writes constant values to output. Verifies VulkanFuse execution.
//!
//! Usage:
//!   cargo run --example test_diag_shader --features mnn -- <spirv_path>

use std::env;
use std::fs;
use cam_isp::onnx::proto::Proto;

fn nchw_dims(dims: &[i64]) -> Vec<Vec<u8>> {
    dims.iter().map(|&d| Proto::tensor_dim_value(d)).collect()
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: test_diag_shader <spirv_path>");
        std::process::exit(1);
    }

    let spv = fs::read(&args[1]).expect("Failed to read SPIR-V");
    println!("SPIR-V: {} bytes", spv.len());

    // Output: FLOAT[1, 4, 1, 1] — just 4 floats (1.0, 2.0, 3.0, 4.0)
    let output_dims = nchw_dims(&[1, 4, 1, 1]);

    let attrs = vec![
        Proto::attribute_tensor("spirv",
            &Proto::tensor_proto_raw_bytes("spirv", &spv, 3, &[spv.len() as i64])),
        Proto::attribute_ints("output_shape", &[1, 4, 1, 1]),
        // No inputs, just output
        Proto::attribute_input_ints("input", 0, &[1, 0]),  // tensor 0 = output, binding 0
    ];

    let output_vi = Proto::value_info("output", &output_dims, 1);

    let node = Proto::node("DiagConst", &[], &["output"], &attrs);

    let graph = Proto::graph("diag", &[node], &[], &[output_vi.clone()], &[], &[output_vi.clone()]);

    let opset = Proto::opset("", 21);
    let model = Proto::model(9, &opset, "softisp", &graph);

    let output_path = "/data/data/com.termux/files/home/softisp/vulkan_isp/test_diag.onnx";
    fs::write(output_path, &model).expect("Failed to write model");
    println!("ONNX: {} ({} bytes)", output_path, model.len());

    // Convert to MNN
    println!("\nTo convert:");
    println!("  cd ~/MNN/build_vk && ./MNNConvert -f ONNX --modelFile {} --MNNModel test_diag.mnn", output_path);
}
