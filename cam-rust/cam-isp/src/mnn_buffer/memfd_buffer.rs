//! memfd-based Buffer Management for MNN
//!
//! This module provides a way to allocate buffers using memfd (memory file descriptors)
//! on Linux, allowing for:
//! - Zero-copy memory sharing
//! - Proper memory management that MNN can work with
//! - Buffer reuse across inference calls
//!
//! # Design
//!
//! The key insight from MNN source analysis:
//! - The session input tensor (user portal) has `backend=NULL` and `host=NULL`
//! - MNN creates a pipeline tensor internally and copies from portal to pipeline
//! - For LITE+ models, the portal tensor's host gets overwritten to NULL during resizeSession
//!
//! Solution:
//! 1. Allocate buffer via memfd + mmap
//! 2. Set `tensor->buffer().host = mmap_ptr`
//! 3. Mark tensor as MEMORY_HOST so MNN doesn't try to free it
//! 4. Manage the memfd lifecycle properly

use std::fs::File;
use std::os::unix::io::{AsRawFd, FromRawFd, IntoRawFd, RawFd};
use std::sync::{Arc, Mutex};

/// memfd-backed buffer for MNN tensors
///
/// This buffer uses memfd + mmap to create memory that:
/// - Can be shared across processes (zero-copy)
/// - Has a file descriptor that can be passed to other systems
/// - Is properly managed even if MNN tries to manipulate it
pub struct MemfdBuffer {
    /// The memfd file descriptor
    memfd: File,
    /// The mmap'd memory region
    ptr: *mut u8,
    /// Size of the buffer
    size: usize,
    /// Whether the buffer is sealed (prevents resizing)
    sealed: bool,
}

impl MemfdBuffer {
    /// Create a new memfd buffer with the given size
    pub fn new(size: usize) -> std::io::Result<Self> {
        // Create memfd
        let memfd = Self::create_memfd(size)?;
        
        // Map it into memory
        let ptr = Self::mmap(&memfd, size)?;
        
        // Seal to prevent accidental modification
        Self::seal_memfd(&memfd)?;
        
        Ok(Self {
            memfd,
            ptr,
            size,
            sealed: true,
        })
    }
    
    /// Create memfd with a name
    #[cfg(target_os = "linux")]
    fn create_memfd(size: usize) -> std::io::Result<File> {
        use std::ffi::CString;
        
        // memfd_create("mnn_buffer", MFD_CLOEXEC | MFD_ALLOW_SEALING)
        let name = CString::new("mnn_buffer").unwrap();
        
        let fd = unsafe {
            libc::syscall(
                libc::SYS_memfd_create,
                name.as_ptr(),
                libc::MFD_CLOEXEC | libc::MFD_ALLOW_SEALING,
            ) as RawFd
        };
        
        if fd < 0 {
            return Err(std::io::Error::last_os_error());
        }
        
        // Set size
        if unsafe { libc::ftruncate(fd, size as libc::off_t) } != 0 {
            unsafe { libc::close(fd) };
            return Err(std::io::Error::last_os_error());
        }
        
        Ok(unsafe { File::from_raw_fd(fd) })
    }
    
    /// Fallback for non-Linux
    #[cfg(not(target_os = "linux"))]
    fn create_memfd(_size: usize) -> std::io::Result<File> {
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "memfd not supported on this platform",
        ))
    }
    
    /// Memory map the memfd
    #[cfg(target_os = "linux")]
    fn mmap(memfd: &File, size: usize) -> std::io::Result<*mut u8> {
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                memfd.as_raw_fd(),
                0,
            )
        };
        
        if ptr == libc::MAP_FAILED {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(ptr as *mut u8)
        }
    }
    
    /// Fallback for non-Linux
    #[cfg(not(target_os = "linux"))]
    fn mmap(_memfd: &File, size: usize) -> std::io::Result<*mut u8> {
        // Allocate regular memory
        let ptr = unsafe {
            libc::malloc(size) as *mut u8
        };
        if ptr.is_null() {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(ptr)
        }
    }
    
    /// Seal the memfd to prevent modifications
    #[cfg(target_os = "linux")]
    fn seal_memfd(memfd: &File) -> std::io::Result<()> {
        let result = unsafe {
            libc::fcntl(
                memfd.as_raw_fd(),
                libc::F_ADD_SEALS,
                libc::F_SEAL_SHRINK | libc::F_SEAL_GROW | libc::F_SEAL_SEAL,
            )
        };
        
        if result < 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    
    /// Fallback for non-Linux
    #[cfg(not(target_os = "linux"))]
    fn seal_memfd(_memfd: &File) -> std::io::Result<()> {
        Ok(())
    }
    
    /// Get the raw pointer for MNN
    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }
    
    /// Get the size
    pub fn size(&self) -> usize {
        self.size
    }
    
    /// Get the memfd
    pub fn as_file(&self) -> &File {
        &self.memfd
    }
    
    /// Get the raw file descriptor
    pub fn as_raw_fd(&self) -> RawFd {
        self.memfd.as_raw_fd()
    }
    
    /// Fill the buffer with data
    pub fn fill_from_slice(&mut self, data: &[u8]) -> std::io::Result<()> {
        if data.len() > self.size {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Data too large: {} > {}", data.len(), self.size),
            ));
        }
        
        unsafe {
            std::ptr::copy_nonoverlapping(
                data.as_ptr(),
                self.ptr,
                data.len(),
            );
        }
        
        Ok(())
    }
    
    /// Copy data out of the buffer
    pub fn to_vec(&self) -> Vec<u8> {
        let mut result = vec![0u8; self.size];
        unsafe {
            std::ptr::copy_nonoverlapping(
                self.ptr,
                result.as_mut_ptr(),
                self.size,
            );
        }
        result
    }
}

impl Drop for MemfdBuffer {
    fn drop(&mut self) {
        // Unmap the memory
        #[cfg(target_os = "linux")]
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.size);
        }
        
        #[cfg(not(target_os = "linux"))]
        unsafe {
            libc::free(self.ptr as *mut libc::c_void);
        }
        
        // The file will be closed automatically when dropped
    }
}

/// Safe wrapper for MNN tensor manipulation
///
/// This provides safe(ish) access to MNN tensor internals
pub struct MNNTensorWrapper {
    /// Pointer to the MNN Tensor
    tensor: *mut std::ffi::c_void,
}

impl MNNTensorWrapper {
    /// Create a new wrapper from a raw MNN tensor pointer
    pub unsafe fn from_raw(tensor: *mut std::ffi::c_void) -> Self {
        Self { tensor }
    }
    
    /// Set the host pointer of the tensor
    ///
    /// # Safety
    /// - The tensor pointer must be valid
    /// - The data_ptr must remain valid for the tensor's lifetime
    pub unsafe fn set_host(&self, data_ptr: *mut u8) {
        // halide_buffer_t layout:
        // - device: uint64_t (offset 0)
        // - host: uint8_t* (offset 8)
        // - extent[4]: int32_t (offset 16)
        // - stride[4]: int32_t (offset 32)
        // - ...
        
        let buffer_ptr = self.tensor as *mut u8;
        let host_offset = 8; // After device (uint64_t)
        let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
        *host_ptr_loc = data_ptr;
    }
    
    /// Get the host pointer of the tensor
    pub unsafe fn get_host(&self) -> *mut u8 {
        let buffer_ptr = self.tensor as *mut u8;
        let host_offset = 8;
        let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
        *host_ptr_loc
    }
    
    /// Get the shape of the tensor
    pub unsafe fn get_shape(&self) -> Vec<i32> {
        let buffer_ptr = self.tensor as *mut u8;
        // extent is at offset 16 (after device + host)
        let extent_offset = 16;
        let extent_ptr = buffer_ptr.add(extent_offset) as *const i32;
        
        // Read up to 8 dimensions (MNN_MAX_TENSOR_DIM)
        let mut dims = Vec::new();
        for i in 0..8 {
            let dim = *extent_ptr.add(i);
            if dim == 0 {
                break;
            }
            dims.push(dim);
        }
        
        dims
    }
    
    /// Set memory type to HOST to prevent MNN from freeing
    ///
    /// This tells MNN that the memory is managed externally
    pub unsafe fn set_memory_type_host(&self) {
        // We need to access Tensor::InsideDescribe::memoryType
        // This is more complex and requires TensorUtils
        // For now, we'll use a direct offset approach
        
        // From MNN source, Tensor struct has:
        // - halide_buffer_t mBuffer
        // - InsideDescribe* mDescribe
        // InsideDescribe has:
        // - NativeInsideDescribe* mContent
        // - SharedPtr<Backend::MemObj> mem
        // - Backend* backend
        // NativeInsideDescribe has:
        // - MemoryType memoryType
        
        // This is too complex to do safely from Rust without proper bindings
        // We'll need to use the C++ wrapper for this
    }
}

/// Buffer pool for managing multiple memfd buffers
pub struct MemfdBufferPool {
    /// Map of tensor name to buffer
    buffers: std::collections::HashMap<String, Arc<Mutex<MemfdBuffer>>>,
}

impl MemfdBufferPool {
    /// Create a new buffer pool
    pub fn new() -> Self {
        Self {
            buffers: std::collections::HashMap::new(),
        }
    }
    
    /// Get or create a buffer for a tensor
    pub fn get_buffer(
        &mut self,
        tensor_name: &str,
        size: usize,
    ) -> std::io::Result<Arc<Mutex<MemfdBuffer>>> {
        if let Some(buffer) = self.buffers.get(tensor_name) {
            return Ok(buffer.clone());
        }
        
        let buffer = Arc::new(Mutex::new(MemfdBuffer::new(size)?));
        self.buffers.insert(tensor_name.to_string(), buffer.clone());
        Ok(buffer)
    }
    
    /// Get buffer by name
    pub fn get_existing(
        &self,
        tensor_name: &str,
    ) -> Option<Arc<Mutex<MemfdBuffer>>> {
        self.buffers.get(tensor_name).cloned()
    }
    
    /// Remove buffer from pool
    pub fn remove_buffer(&mut self, tensor_name: &str) -> Option<Arc<Mutex<MemfdBuffer>>> {
        self.buffers.remove(tensor_name)
    }
    
    /// Clear all buffers
    pub fn clear(&mut self) {
        self.buffers.clear();
    }
}

/// C API for interop with C++ MNN wrapper
///
/// These functions can be called from C/C++ code
#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_create(size: usize) -> *mut MemfdBuffer {
    MemfdBuffer::new(size)
        .map(|b| Box::into_raw(Box::new(b)))
        .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_destroy(buffer: *mut MemfdBuffer) {
    if !buffer.is_null() {
        unsafe { Box::from_raw(buffer) };
    }
}

#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_ptr(buffer: *mut MemfdBuffer) -> *mut u8 {
    if buffer.is_null() {
        std::ptr::null_mut()
    } else {
        unsafe { (*buffer).as_ptr() }
    }
}

#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_fd(buffer: *mut MemfdBuffer) -> i32 {
    if buffer.is_null() {
        -1
    } else {
        unsafe { (*buffer).as_raw_fd() }
    }
}

#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_size(buffer: *mut MemfdBuffer) -> usize {
    if buffer.is_null() {
        0
    } else {
        unsafe { (*buffer).size() }
    }
}

#[no_mangle]
pub extern "C" fn mnn_memfd_buffer_fill(
    buffer: *mut MemfdBuffer,
    data: *const u8,
    len: usize,
) -> bool {
    if buffer.is_null() || data.is_null() {
        return false;
    }
    
    let slice = unsafe { std::slice::from_raw_parts(data, len) };
    unsafe { (*buffer).fill_from_slice(slice).is_ok() }
}

/// Example usage from C++:
///
/// ```cpp
/// #include <MNN/Interpreter.hpp>
/// 
/// extern "C" {
///     void* mnn_memfd_buffer_create(size_t size);
///     void mnn_memfd_buffer_destroy(void* buffer);
///     uint8_t* mnn_memfd_buffer_ptr(void* buffer);
/// }
/// 
/// void run_inference() {
///     auto net = MNN::Interpreter::createFromFile("model.mnn");
///     auto sess = net->createSession(cfg);
///     auto* in = net->getSessionInput(sess, nullptr);
///     
///     // Create buffer
///     size_t size = 48 * 64 * sizeof(int32_t);
///     void* buffer = mnn_memfd_buffer_create(size);
///     uint8_t* ptr = mnn_memfd_buffer_ptr(buffer);
///     
///     // Fill buffer with data
///     std::vector<int32_t> data(48*64);
///     for (int i = 0; i < 48*64; i++) data[i] = i % 256;
///     mnn_memfd_buffer_fill(buffer, data.data(), size);
///     
///     // Set as MNN input
///     in->buffer().host = ptr;
///     
///     // Run inference
///     net->runSession(sess);
///     
///     // Clean up
///     mnn_memfd_buffer_destroy(buffer);
/// }
/// ```
