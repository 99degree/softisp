//! MNN Buffer Management Module
//!
//! Provides buffer management utilities for MNN (Mobile Neural Network) inference.
//!
//! # Submodules
//!
//! - `memfd` - memfd-based buffer management for zero-copy inference
//! - `cma_buffer` - CMA buffer management for hardware-aligned memory
//! - `buffer` - General buffer management utilities

pub mod memfd;
pub mod cma_buffer;
pub mod buffer;

/// Re-export commonly used types
pub use memfd::{MemfdBuffer, MemfdBufferManager, Alignment as MemfdAlignment};
pub use cma_buffer::{CMABuffer, CMABufferManager, Alignment as CMAAlignment, BufferUsage, CMAAllocator};

/// MNN data type codes (from MNN DataType enum)
pub const DT_FLOAT: u8 = 2;
pub const DT_INT32: u8 = 0;
pub const DT_UINT8: u8 = 1;
pub const DT_INT8: u8 = 3;
pub const DT_INT16: u8 = 5;
pub const DT_UINT16: u8 = 4;

/// Helper to calculate tensor size from dims and type
pub fn tensor_size(dims: &[i32], elem_bits: u8) -> usize {
    let elem_size = (elem_bits as usize + 7) / 8;
    dims.iter().product::<i32>() as usize * elem_size
}

/// Helper to set MNN tensor host pointer from Rust
///
/// # Safety
/// - `tensor` must be a valid MNN Tensor pointer
/// - `data_ptr` must remain valid for the tensor's lifetime
pub unsafe fn set_tensor_host(tensor: *mut std::ffi::c_void, data_ptr: *mut u8) {
    // halide_buffer_t layout:
    // struct halide_buffer_t {
    //     uint64_t device;     // offset 0
    //     uint8_t *host;       // offset 8
    //     int32_t extent[4];   // offset 16
    //     int32_t stride[4];   // offset 32
    //     ...
    // };
    
    let buffer_ptr = tensor as *mut u8;
    let host_offset = 8; // After device (uint64_t = 8 bytes)
    let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
    *host_ptr_loc = data_ptr;
}

/// Helper to get MNN tensor host pointer from Rust
pub unsafe fn get_tensor_host(tensor: *mut std::ffi::c_void) -> *mut u8 {
    let buffer_ptr = tensor as *mut u8;
    let host_offset = 8;
    let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
    *host_ptr_loc
}

/// Helper to get MNN tensor shape
pub unsafe fn get_tensor_shape(tensor: *mut std::ffi::c_void) -> Vec<i32> {
    let buffer_ptr = tensor as *mut u8;
    let extent_offset = 16; // After device + host
    let extent_ptr = buffer_ptr.add(extent_offset) as *const i32;
    
    // Read dimensions (max 8 for MNN)
    let mut dims = Vec::new();
    for i in 0..8 {
        let dim = *extent_ptr.add(i);
        if dim == 0 || dim == 1 {
            // Stop at 0 or continue with 1
            if dim == 0 {
                break;
            }
            dims.push(dim);
        } else {
            dims.push(dim);
        }
    }
    dims
}

/// MNN inference context
///
/// Manages the lifecycle of MNN model, session, and buffers
pub struct MNNContext {
    /// Interpreter pointer
    interpreter: *mut std::ffi::c_void,
    /// Session pointer
    session: *mut std::ffi::c_void,
    /// Input tensor pointer
    input_tensor: *mut std::ffi::c_void,
    /// Output tensor pointer
    output_tensor: *mut std::ffi::c_void,
}

impl MNNContext {
    /// Create a new MNN context from a model file
    pub fn from_file(model_path: &str) -> Result<Self, String> {
        // This would use the C++ FFI to create the interpreter and session
        // For now, we'll use the C wrapper functions
        
        // In production, this would call:
        // let interpreter = mnn_interpreter_create_from_file(model_path);
        // let session = mnn_interpreter_create_session(interpreter);
        // let input = mnn_interpreter_get_session_input(session, null);
        // let output = mnn_interpreter_get_session_output(session, null);
        
        Err("MNNContext::from_file not implemented - use C++ wrapper".to_string())
    }
    
    /// Get input tensor pointer
    pub fn input_tensor(&self) -> *mut std::ffi::c_void {
        self.input_tensor
    }
    
    /// Get output tensor pointer
    pub fn output_tensor(&self) -> *mut std::ffi::c_void {
        self.output_tensor
    }
}

/// Trait for MNN buffer providers
///
/// Allows different buffer implementations to be used interchangeably
pub trait MNNBufferProvider {
    /// Allocate buffer for the given tensor spec
    fn allocate(&mut self, name: &str, size: usize) -> Result<*mut u8, Box<dyn std::error::Error>>;
    
    /// Get existing buffer
    fn get(&self, name: &str) -> Option<*mut u8>;
    
    /// Sync buffer for device access (before inference)
    fn sync_for_device(&self, name: &str) -> Result<(), Box<dyn std::error::Error>>;
    
    /// Sync buffer for CPU access (after inference)
    fn sync_for_cpu(&self, name: &str) -> Result<(), Box<dyn std::error::Error>>;
}

impl MNNBufferProvider for MemfdBufferManager {
    fn allocate(&mut self, name: &str, size: usize) -> Result<*mut u8, Box<dyn std::error::Error>> {
        let buffer = self.get_buffer(name, size)?;
        Ok(buffer.as_ptr())
    }
    
    fn get(&self, name: &str) -> Option<*mut u8> {
        self.get_existing(name).map(|b| b.as_ptr())
    }
    
    fn sync_for_device(&self, name: &str) -> Result<(), Box<dyn std::error::Error>> {
        let buffer = self.get_existing(name)
            .ok_or_else(|| format!("Buffer {} not found", name).into())?;
        buffer.sync_for_device()?;
        Ok(())
    }
    
    fn sync_for_cpu(&self, name: &str) -> Result<(), Box<dyn std::error::Error>> {
        let buffer = self.get_existing(name)
            .ok_or_else(|| format!("Buffer {} not found", name).into())?;
        buffer.sync_for_cpu()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tensor_size() {
        // 1x1x48x64 INT32 = 48*64*4 bytes
        let dims = vec![1, 1, 48, 64];
        let size = tensor_size(&dims, 32);
        assert_eq!(size, 48 * 64 * 4);
        
        // 1x3x224x224 FLOAT = 3*224*224*4 bytes
        let dims = vec![1, 3, 224, 224];
        let size = tensor_size(&dims, 32);
        assert_eq!(size, 3 * 224 * 224 * 4);
    }
}
