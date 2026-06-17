// cam-rust/cam-isp/tests/test_unpack_bayer_logic.rs
//! Pure Rust test for the unpack logic without ONNX/MNN dependencies
//!
//! Tests the core unpacking algorithm:
//! - Input: packed INT32 `[1,1,H,W]` where each int32 = (A << 16) | B
//! - Output: 2-channel `[1,2,H,W]` with FP16 values normalized to [0,1]

use half::f16;

/// Pack two 10-bit values into one INT32: (A << 16) | B
fn pack_ab(a: u16, b: u16) -> i32 {
    ((a as i32) << 16) | (b as i32 & 0x3FF)
}

/// Unpack and normalize a single packed INT32 to two FP16 values
fn unpack_and_normalize(packed: i32) -> (f16, f16) {
    // Extract high 16 bits (A)
    let a_16 = (packed >> 16) as u16;
    // Extract low 16 bits (B)
    let b_16 = (packed & 0xFFFF) as u16;
    
    // Mask to 10 bits
    let a_10 = a_16 & 0x3FF;
    let b_10 = b_16 & 0x3FF;
    
    // Normalize to [0,1]
    let a_norm = a_10 as f32 / 1023.0;
    let b_norm = b_10 as f32 / 1023.0;
    
    // Convert to FP16
    (f16::from_f32(a_norm), f16::from_f32(b_norm))
}

#[test]
fn test_unpack_single_value() {
    // Test with known values
    let a: u16 = 100;
    let b: u16 = 200;
    let packed = pack_ab(a, b);
    
    let (a_out, b_out) = unpack_and_normalize(packed);
    
    let a_val = a_out.to_f32();
    let b_val = b_out.to_f32();
    
    let expected_a = 100.0 / 1023.0;
    let expected_b = 200.0 / 1023.0;
    
    assert!((a_val - expected_a).abs() < 1e-3, 
        "A: got {} expected {}", a_val, expected_a);
    assert!((b_val - expected_b).abs() < 1e-3,
        "B: got {} expected {}", b_val, expected_b);
}

#[test]
fn test_unpack_array() {
    // Test unpacking an array of packed values
    let h = 4;
    let w = 4;
    let mut packed = vec![0i32; h * w];
    
    // Fill with test data: A = i*10, B = i*20
    for i in 0..h * w {
        let a = ((i * 10) % 1024) as u16;
        let b = ((i * 20) % 1024) as u16;
        packed[i] = pack_ab(a, b);
    }
    
    // Unpack
    let mut unpacked_a = vec![f16::ZERO; h * w];
    let mut unpacked_b = vec![f16::ZERO; h * w];
    
    for i in 0..h * w {
        let (a, b) = unpack_and_normalize(packed[i]);
        unpacked_a[i] = a;
        unpacked_b[i] = b;
    }
    
    // Verify
    for i in 0..h * w {
        let expected_a = ((i * 10) % 1024) as f32 / 1023.0;
        let expected_b = ((i * 20) % 1024) as f32 / 1023.0;
        
        let a_val = unpacked_a[i].to_f32();
        let b_val = unpacked_b[i].to_f32();
        
        assert!((a_val - expected_a).abs() < 1e-3,
            "Pixel {} A: got {} expected {}", i, a_val, expected_a);
        assert!((b_val - expected_b).abs() < 1e-3,
            "Pixel {} B: got {} expected {}", i, b_val, expected_b);
    }
}

#[test]
fn test_unpack_edge_cases() {
    // Test with minimum and maximum 10-bit values
    
    // Min: 0
    let packed = pack_ab(0, 0);
    let (a, b) = unpack_and_normalize(packed);
    assert_eq!(a.to_f32(), 0.0);
    assert_eq!(b.to_f32(), 0.0);
    
    // Max: 1023
    let packed = pack_ab(1023, 1023);
    let (a, b) = unpack_and_normalize(packed);
    assert!((a.to_f32() - 1.0).abs() < 1e-3);
    assert!((b.to_f32() - 1.0).abs() < 1e-3);
    
    // Mixed
    let packed = pack_ab(0, 1023);
    let (a, b) = unpack_and_normalize(packed);
    assert_eq!(a.to_f32(), 0.0);
    assert!((b.to_f32() - 1.0).abs() < 1e-3);
}

#[test]
fn test_unpack_format_conversion() {
    // Test that the format conversion from [1,1,H,W] INT32 to [1,2,H,W] FP16 is correct
    let h = 2;
    let w = 2;
    let packed = vec![
        pack_ab(100, 200),
        pack_ab(300, 400),
        pack_ab(500, 600),
        pack_ab(700, 800),
    ];
    
    // Expected output shape: [1,2,H,W] = [1,2,2,2]
    // Channel 0 (A): [[100, 300], [500, 700]] normalized
    // Channel 1 (B): [[200, 400], [600, 800]] normalized
    
    let expected_a = vec![
        vec![100.0, 300.0],
        vec![500.0, 700.0],
    ];
    let expected_b = vec![
        vec![200.0, 400.0],
        vec![600.0, 800.0],
    ];
    
    // Unpack and verify
    for i in 0..h * w {
        let (a, b) = unpack_and_normalize(packed[i]);
        let row = i / w;
        let col = i % w;
        
        let expected_a_val = expected_a[row][col] / 1023.0;
        let expected_b_val = expected_b[row][col] / 1023.0;
        
        assert!((a.to_f32() - expected_a_val).abs() < 1e-3,
            "A[{},{}] got {} expected {}", row, col, a.to_f32(), expected_a_val);
        assert!((b.to_f32() - expected_b_val).abs() < 1e-3,
            "B[{},{}] got {} expected {}", row, col, b.to_f32(), expected_b_val);
    }
}
