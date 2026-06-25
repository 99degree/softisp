//! gen_fused_isp_onnx.rs — Generate ONNX model with fused ISP shader Extra ops
//!
//! Creates an ONNX model with a single Extra op containing SPIR-V binary
//! for the UnpackBLC stage. MNNConverter handles unknown ops → OpType_Extra.
//!
//! Usage:
//!   cargo run --example gen_fused_isp_onnx --features mnn -- <spirv_dir> <output.onnx>

use std::env;
use std::fs;

use cam_isp::onnx::proto::Proto;

fn nchw_dims(dims: &[i64]) -> Vec<Vec<u8>> {
    dims.iter().map(|&d| Proto::tensor_dim_value(d)).collect()
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("Usage: gen_fused_isp_onnx <spirv_dir> <output.onnx>");
        std::process::exit(1);
    }
    let spirv_dir = &args[1];
    let output_path = &args[2];

    // Read SPIR-V
    let spv = fs::read(format!("{}/shader1_unpack_blc.spv", spirv_dir))
        .expect("Failed to read SPIR-V");
    println!("Loaded SPIR-V: {} bytes", spv.len());

    // Uniform data: [input_width, input_height, output_width, output_height,
    //                 sensor_max, blc_r, blc_gr, blc_gb, blc_b,
    //                 wb_r, wb_gr, wb_gb, wb_b]
    let uniforms: Vec<f32> = vec![
        3840.0, 2160.0, 1920.0, 1080.0,
        4095.0,
        0.0, 0.0, 0.0, 0.0,  // BLC
        1.0, 1.0, 1.0, 1.0,  // WB
    ];

    // Build the Extra op with attributes
    let attrs = vec![
        Proto::attribute_tensor("spirv",
            &Proto::tensor_proto_raw_bytes("spirv", &spv, 3, &[spv.len() as i64])), // INT8
        Proto::attribute_tensor("uniforms",
            &Proto::tensor_proto_float("uniforms", &[uniforms.len() as i64], &uniforms)),
        Proto::attribute_ints("global_size", &[1920, 1080, 1]),
        Proto::attribute_ints("group_size", &[8, 8, 1]),
        // Output shape: [N, C, H, W]
        Proto::attribute_ints("output_shape", &[1, 4, 1080, 1920]),
        // Input binding: [is_output=0 (input), binding=1, tensor_idx=0]
        // i=0 = tensor index for input, ints=[is_output=0, binding=1]
        Proto::attribute_input_ints("input", 0, &[0, 1]),
        // Output binding: [is_output=1 (output), binding=2, tensor_idx=1]
        // i=1 = tensor index for output, ints=[is_output=1, binding=2]
        Proto::attribute_input_ints("input", 1, &[1, 2]),
        // Const at binding=0 with uniform data as tensor (for VulkanFuse const handler)
        Proto::attribute_const_tensor("const", 0,
            &Proto::tensor_proto_float("const_data", &[uniforms.len() as i64], &uniforms)),
    ];

    // Input: INT16[1, 1, 2160, 3840]
    let input_dims = nchw_dims(&[1, 1, 2160, 3840]);
    let input_vi = Proto::value_info("input", &input_dims, 5);

    // Output: FLOAT[1, 4, 1080, 1920]
    let output_dims = nchw_dims(&[1, 4, 1080, 1920]);
    let output_vi = Proto::value_info("output", &output_dims, 1);

    // FusedISP node
    let node = Proto::node(
        "FusedISP_UnpackBLC",
        &["input"],
        &["output"],
        &attrs,
    );

    // Build graph
    let graph = Proto::graph(
        "fused_isp",
        &[node],
        &[input_vi.clone()],
        &[output_vi.clone()],
        &[],  // no initializers
        &[input_vi.clone()],
    );

    // Build model with opset
    let opset = Proto::opset("", 21);
    let model = Proto::model(9, &opset, "softisp", &graph);

    fs::write(output_path, &model).expect("Failed to write model");
    println!("Model saved: {} ({} bytes)", output_path, model.len());
    println!("\nTo convert to MNN:");
    println!("  cd ~/MNN/build_vk && ./MNNConvert -f ONNX --modelFile {} --MNNModel fused_isp.mnn", output_path);
    println!("To run:");
    println!("  LD_LIBRARY_PATH=./OFF ./MNNV2Basic.out fused_isp.mnn 0 0 10");
}
