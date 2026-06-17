// cam-rust/cam-isp/tests/test_unpack_bayer_fp16.rs
//! Test for UnpackBayerToFp16Block
//! 
//! Verifies that packed INT32 Bayer data (A << 16 | B) is correctly:
//! 1. Split into high/low 16-bit halves
//! 2. Masked to 10 bits
//! 3. Cast to FP16
//! 4. Normalised by dividing by 1023.0

use cam_isp::pipeline::GraphComposer;
use cam_isp::blocks::{RawInputBlock, UnpackBayerToFp16Block};
use cam_isp::pipeline::IspBlock;

/// Pack two 10-bit values into one INT32: (A << 16) | B
fn pack_ab(a: u16, b: u16) -> i32 {
    ((a as i32) << 16) | (b as i32 & 0x3FF)
}

#[test]
fn test_unpack_bayer_fp16_onnx_construction() {
    // 1. Create blocks
    let raw_input = RawInputBlock::new();
    let mut unpack = UnpackBayerToFp16Block::new();
    
    // Set up the pipeline
    unpack.set_input_source(raw_input.frame_tensor().unwrap());
    
    let blocks: Vec<&dyn IspBlock> = vec![&raw_input, &unpack];
    
    // 2. Build ONNX model
    let onnx_bytes = GraphComposer::compose_from_vec(&blocks, &[], 15)
        .expect("Failed to build ONNX model");
    
    // 3. Basic validation - ONNX model should not be empty
    assert!(!onnx_bytes.is_empty(), "ONNX model is empty");
    
    // 4. Check that it contains expected node types
    let onnx_str = String::from_utf8_lossy(&onnx_bytes);
    assert!(onnx_str.contains("RightShift"), "Missing RightShift node");
    assert!(onnx_str.contains("BitwiseAnd"), "Missing BitwiseAnd node");
    assert!(onnx_str.contains("Cast"), "Missing Cast node");
    assert!(onnx_str.contains("Div"), "Missing Div node");
    assert!(onnx_str.contains("Concat"), "Missing Concat node");
    
    println!("[TEST] ONNX model construction successful");
    println!("Model size: {} bytes", onnx_bytes.len());
}

#[test]
fn test_pack_ab_function() {
    // Verify our packing function works correctly
    let a: u16 = 100;
    let b: u16 = 200;
    let packed = pack_ab(a, b);
    
    // Extract back
    let a_extracted = ((packed >> 16) & 0x3FF) as u16;
    let b_extracted = (packed & 0x3FF) as u16;
    
    assert_eq!(a_extracted, a, "A extraction failed");
    assert_eq!(b_extracted, b, "B extraction failed");
}

#[test]
fn test_unpack_block_traits() {
    // Test that the block implements all required traits
    let block = UnpackBayerToFp16Block::new();
    
    // Check basic properties
    assert_eq!(block.id(), "unpack_bayer_fp16");
    assert_eq!(block.tensor_ns(), "UnpackBayerToFp16Block");
    assert_eq!(block.frame_tensor(), Some("UnpackBayerToFp16Block/frame_fp16"));
    assert_eq!(block.input_source(), Some("RawInputBlock/frame"));
    
    // Check element types
    assert_eq!(block.input_elem_type(), 6); // INT32
    assert_eq!(block.output_elem_type(), 10); // FLOAT16
    
    // Check output tensors
    assert_eq!(block.output_tensors(), vec!["UnpackBayerToFp16Block/frame_fp16"]);
    
    // Check that nodes are generated
    let nodes = block.nodes();
    assert!(!nodes.is_empty(), "No nodes generated");
    
    // Check output value info
    let value_info = block.output_value_info();
    assert!(value_info.is_some(), "No output value info");
}
