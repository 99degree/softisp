//! Runtime ISP parameterization via input tensors.
//!
//! Creates an MNN pipeline whose ISP params (ccm, wb, bias, etc.)
//! are fed as input tensors instead of baked constants.
//! MNN runtime: Update via `interpreter.updateInputTensor(...)`.
use cam_isp::{blocks::*, pipeline::{PipelineBuilder, GraphComposer}, BayerPattern};

// Runtime params: matches Vulkan const buffer layout
struct IspRuntimeParams {
    wb_gains: [f32; 4],
    bayer_bias: [f32; 4],
    ccm: [f32; 9],
    fcs_str: f32,
    fcs_off: f32,
    gamma: f32,
}

fn create_runtime_pipeline() -> Vec<u8> {
    let mut pipeline = PipelineBuilder::new();
    // Indirection: ISP params from "isp_params" tensor
    let params_node = vec![];
    
    // Bayer → Demosaic
    pipeline.add_unpack().with_bayer_pattern(BayerPattern::RGGB)
          .add_demosaic()
          .add_display(1920);
    
    // Extract blocks
    let blocks: Vec<&dyn cam_isp::IspBlock> = pipeline.blocks.iter().map(|b| &**b).collect();
    let model = GraphComposer::compose(blocks.as_slice(), &[], 8, "RuntimeISP");
    
    // Inject ISP params tensor as first input
    model.insert_input_tensor("isp_params", params_node);
    model.onnx_bytes()
}

// Update: MNN runtime
// fn update_params(interpreter: &MNNInterpreter, session: &MNNSession,
//                  wb_gains: [f32;4], bayer_black: [f32;4]) {
//     let data = flatten_params(wb_gains, bayer_black);
//     interpreter.updateInputTensor(session, "isp_params", data);
// }

fn main() {
    let model = create_runtime_pipeline();
    std::fs::write("target/runtime_isp.onnx", model);
    println!("✅ Runtime ISP pipeline: ONNX emitted");
}