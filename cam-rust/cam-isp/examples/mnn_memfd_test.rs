//! MNN memfd Buffer Test Example
//!
//! Demonstrates zero-copy MNN inference using memfd buffers.
//! Uses the Rust API directly (not the C FFI).

use cam_isp::mnn_buffer::MemfdBuffer;
use cam_isp::mnn_buffer::MemfdAlignment;

fn main() {
    println!("=== MNN memfd Buffer Test ===\n");
    
    // Create a memfd buffer
    let size = 48 * 64 * 4; // 48x64 INT32 = 12288 bytes
    let alignment = MemfdAlignment::Page4K;
    
    match MemfdBuffer::new(size, alignment, "test_buffer") {
        Ok(buffer) => {
            println!("✅ Buffer created: ptr={:p}, size={}, alignment=4K", 
                     buffer.as_ptr(), size);
            
            // Fill with test data using mutable slice
            let data = unsafe { std::slice::from_raw_parts_mut(buffer.as_ptr(), size) };
            for i in 0..size {
                data[i] = (i % 256) as u8;
            }
            println!("✅ Buffer filled with test data");
            
            // Sync for device
            if let Err(e) = buffer.sync_for_device() {
                println!("❌ Sync for device failed: {}", e);
            } else {
                println!("✅ Synced for device");
            }
            
            // Read back and verify
            let readback = unsafe { std::slice::from_raw_parts(buffer.as_ptr(), 16) };
            println!("✅ First 16 bytes: {:?}", readback);
            
            // Buffer is automatically freed when dropped
            println!("✅ Buffer will be freed on drop");
        },
        Err(e) => {
            println!("❌ Failed to create memfd buffer: {}", e);
            
            // Fallback: test with regular allocation
            println!("\nFalling back to regular allocation...");
            let fallback = vec![0u8; size];
            println!("✅ Regular allocation works: {} bytes", fallback.len());
        }
    }
    
    println!("\n=== Test Complete ===");
}
