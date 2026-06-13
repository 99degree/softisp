//! Generate LITE profile model for testing
//!
//! This generates the LITE pipeline model and saves it as an ONNX file,
//! then (optionally) converts it to MNN format.

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Generating LITE Profile Model ===\n");
    
    // Use LITE profile
    let profile = PipelineProfile::LITE;
    
    // Build pipeline for 64x48 frame
    let mut composer = GraphComposer::new(profile);
    composer.set_frame_size(64, 48);
    
    // Build the ONNX graph
    let model = composer.build_onnx()?;
    
    // Save ONNX model
    let onnx_path = "test_lite_new.onnx";
    model.save(onnx_path)?;
    println!("✅ ONNX model saved: {}", onnx_path);
    
    // Save model buffer
    let buffer = model.to_buffer()?;
    let buffer_path = "test_lite_new.mnn";
    std::fs::write(buffer_path, &buffer)?;
    println!("✅ MNN buffer saved: {}", buffer_path);
    
    // Print model info
    println!("\nModel Info:");
    println!("  Inputs: {:?}", model.inputs());
    println!("  Outputs: {:?}", model.outputs());
    println!("  IR version: {}", model.ir_version());
    
    Ok(())
}
