//! Generate and test LITE profile model
//!
//! This generates the LITE pipeline model and tests it with MNN.

use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Generating and Testing LITE Profile ===\n");
    
    // Use LITE profile
    let mut profile = PipelineProfile::LITE;
    
    // Build blocks for 64x48 frame
    let width = 64;
    let mut blocks = profile.build_blocks(width, 2); // bayer_pattern=2 (RGGB)
    
    println!("Built {} blocks:", blocks.len());
    for b in &blocks {
        println!("  {}", b.id());
    }
    
    // Wire blocks together
    GraphComposer::wire_blocks(&mut blocks);
    
    // Compose ONNX
    let block_refs: Vec<&dyn cam_isp::pipeline::IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    let onnx_buffer = GraphComposer::compose_from_vec(&block_refs, &[], 16)?;
    
    println!("\n✅ ONNX model composed: {} bytes", onnx_buffer.len());
    
    // Save ONNX
    std::fs::write("test_lite_new.onnx", &onnx_buffer)?;
    println!("✅ Saved: test_lite_new.onnx");
    
    // Convert to MNN using MNNConvert
    println!("\nConverting to MNN format...");
    #[cfg(feature = "mnn")]
    {
        use cam_isp::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};
        
        let options = MnnConvertOptions::default();
        let mnn_buffer = convert_onnx_to_mnn(&onnx_buffer, &options)?;
        
        println!("✅ MNN model: {} bytes", mnn_buffer.len());
        
        // Save MNN
        std::fs::write("test_lite_new.mnn", &mnn_buffer)?;
        println!("✅ Saved: test_lite_new.mnn");
    }
    
    #[cfg(not(feature = "mnn"))]
    {
        println!("⚠️  MNN conversion skipped (mnn feature not enabled)");
        println!("    To convert: enable mnn feature and rebuild");
    }
    
    Ok(())
}
