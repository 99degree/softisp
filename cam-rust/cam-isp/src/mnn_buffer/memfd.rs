//! memfd-based Buffer Management for MNN Inference
//!
//! Provides a clean, focused implementation using Linux memfd for:
//! - Zero-copy buffer sharing between client and MNN
//! - Hardware-aligned memory allocation
//! - Proper lifetime management (client allocates, MNN uses, client frees)
//!
//! # Why memfd?
//!
//! 1. **Zero-copy**: Memory is mapped directly, no copying needed
//! 2. **File descriptor**: Can be shared across processes (e.g., camera → MNN)
//! 3. **Hardware alignment**: memfd + mmap provides page-aligned memory
//! 4. **MIPI/ISP compatible**: Works with V4L2, Camera HAL, ISP hardware
//! 5. **DMA-capable**: Can be used with dma-buf for GPU/ISP access
//!
//! # Usage Pattern
//!
//! ```text
//! 1. Client: Create memfd buffer with required size
//! 2. Client: Map buffer into memory (mmap)
//! 3. Client: Fill buffer with camera/ISP data
//! 4. Client: Set tensor->buffer().host = mmap_ptr
//! 5. MNN:    Read from tensor (no copy)
//! 6. Client: Free buffer when done (munmap + close)
//! ```

use std::fs::File;
use std::os::unix::io::{AsRawFd, FromRawFd, IntoRawFd, RawFd};
use std::sync::Arc;

/// Alignment for hardware access
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alignment {
    /// 4KB - Page size, minimum for mmap
    Page4K = 4096,
    /// 64KB - Common for camera/ISP
    Page64K = 65536,
    /// 2MB - For large frames
    Page2M = 2 * 1024 * 1024,
}

impl Default for Alignment {
    fn default() -> Self {
        Self::Page4K
    }
}

/// memfd buffer for zero-copy MNN inference
///
/// This buffer uses Linux memfd_create + mmap to provide:
/// - Zero-copy memory sharing
/// - Hardware-aligned allocation
/// - File descriptor for cross-process sharing
/// - Proper cleanup (munmap + close)
pub struct MemfdBuffer {
    /// The memfd file descriptor
    fd: File,
    /// Mapped memory pointer
    ptr: *mut u8,
    /// Buffer size in bytes
    size: usize,
    /// Alignment used
    alignment: Alignment,
    /// Original size (before alignment)
    original_size: usize,
}

impl MemfdBuffer {
    /// Create a new memfd buffer with the given size
    ///
    /// # Arguments
    /// * `size` - Buffer size in bytes (will be rounded up to alignment)
    /// * `alignment` - Memory alignment requirement
    /// * `name` - Name for the memfd (for debugging)
    pub fn new(size: usize, alignment: Alignment, name: &str) -> std::io::Result<Self> {
        // Round up to alignment
        let aligned_size = Self::align_up(size, alignment as usize);
        
        // Create memfd
        let fd = Self::create_memfd(name, aligned_size)?;
        
        // Map into memory
        let ptr = Self::mmap(&fd, aligned_size)?;
        
        Ok(Self {
            fd,
            ptr,
            size: aligned_size,
            alignment,
            original_size: size,
        })
    }
    
    /// Create memfd with default settings
    pub fn with_size(size: usize) -> std::io::Result<Self> {
        Self::new(size, Alignment::default(), "mnn_buffer")
    }
    
    /// Align size up to alignment boundary
    fn align_up(size: usize, alignment: usize) -> usize {
        (size + alignment - 1) & !(alignment - 1)
    }
    
    /// Create memfd using syscall
    ///
    /// # Safety
    /// Uses raw syscall - only works on Linux
    #[cfg(target_os = "linux")]
    fn create_memfd(name: &str, size: usize) -> std::io::Result<File> {
        use std::ffi::CString;
        
        let name_c = CString::new(name).map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::InvalidInput, e)
        })?;
        
        // memfd_create(name, MFD_CLOEXEC | MFD_ALLOW_SEALING)
        let fd = unsafe {
            libc::syscall(
                libc::SYS_memfd_create,
                name_c.as_ptr(),
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
    
    /// Fallback for non-Linux (uses anonymous mmap)
    #[cfg(not(target_os = "linux"))]
    fn create_memfd(_name: &str, size: usize) -> std::io::Result<File> {
        // On non-Linux, we can't use memfd, so we'll use a temporary file
        // or just regular allocation
        use tempfile::tempfile;
        let mut file = tempfile()?;
        file.set_len(size as u64)?;
        Ok(file)
    }
    
    /// Memory map the file
    #[cfg(target_os = "linux")]
    fn mmap(file: &File, size: usize) -> std::io::Result<*mut u8> {
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                0,
            )
        };
        
        if ptr == libc::MAP_FAILED {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(ptr as *mut u8)
        }
    }
    
    /// Fallback mmap for non-Linux
    #[cfg(not(target_os = "linux"))]
    fn mmap(file: &File, size: usize) -> std::io::Result<*mut u8> {
        // Use regular malloc
        let ptr = unsafe {
            libc::malloc(size) as *mut u8
        };
        if ptr.is_null() {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(ptr)
        }
    }
    
    /// Get the raw pointer for MNN
    pub fn as_ptr(&self) -> *mut u8 {
        self.ptr
    }
    
    /// Get the size (aligned)
    pub fn size(&self) -> usize {
        self.size
    }
    
    /// Get the original requested size
    pub fn original_size(&self) -> usize {
        self.original_size
    }
    
    /// Get the file descriptor
    pub fn as_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }
    
    /// Get the alignment
    pub fn alignment(&self) -> Alignment {
        self.alignment
    }
    
    /// Fill buffer from slice
    pub fn fill_from_slice(&self, data: &[u8]) -> std::io::Result<()> {
        if data.len() > self.original_size {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Data too large: {} > {}", data.len(), self.original_size),
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
    
    /// Copy buffer to slice
    pub fn copy_to_slice(&self, data: &mut [u8]) -> std::io::Result<()> {
        if data.len() > self.original_size {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Buffer too small: {} < {}", self.original_size, data.len()),
            ));
        }
        
        unsafe {
            std::ptr::copy_nonoverlapping(
                self.ptr,
                data.as_mut_ptr(),
                data.len(),
            );
        }
        
        Ok(())
    }
    
    /// Seal the memfd to prevent modifications
    ///
    /// This prevents the buffer from being resized or shrunk
    #[cfg(target_os = "linux")]
    pub fn seal(&self) -> std::io::Result<()> {
        let result = unsafe {
            libc::fcntl(
                self.fd.as_raw_fd(),
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
    pub fn seal(&self) -> std::io::Result<()> {
        Ok(())
    }
    
    /// Sync buffer for device access (flush CPU cache)
    ///
    /// Call this after filling the buffer and before MNN inference
    #[cfg(target_os = "linux")]
    pub fn sync_for_device(&self) -> std::io::Result<()> {
        // Use msync to flush CPU cache
        let result = unsafe {
            libc::msync(
                self.ptr as *mut libc::c_void,
                self.size,
                libc::MS_SYNC,
            )
        };
        
        if result != 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    
    /// Sync buffer for CPU access (invalidate CPU cache)
    ///
    /// Call this after MNN inference if you need to read the output
    #[cfg(target_os = "linux")]
    pub fn sync_for_cpu(&self) -> std::io::Result<()> {
        // Use msync to invalidate CPU cache
        let result = unsafe {
            libc::msync(
                self.ptr as *mut libc::c_void,
                self.size,
                libc::MS_INVALIDATE,
            )
        };
        
        if result != 0 {
            Err(std::io::Error::last_os_error())
        } else {
            Ok(())
        }
    }
    
    /// Fallback for non-Linux
    #[cfg(not(target_os = "linux"))]
    pub fn sync_for_device(&self) -> std::io::Result<()> {
        Ok(())
    }
    
    /// Fallback for non-Linux
    #[cfg(not(target_os = "linux"))]
    pub fn sync_for_cpu(&self) -> std::io::Result<()> {
        Ok(())
    }
}

impl Drop for MemfdBuffer {
    fn drop(&mut self) {
        // Unmap memory
        #[cfg(target_os = "linux")]
        {
            if !self.ptr.is_null() {
                unsafe {
                    libc::munmap(self.ptr as *mut libc::c_void, self.size);
                }
            }
        }
        
        #[cfg(not(target_os = "linux"))]
        {
            if !self.ptr.is_null() {
                unsafe {
                    libc::free(self.ptr as *mut libc::c_void);
                }
            }
        }
        
        // The File will be closed automatically when dropped
    }
}

impl Clone for MemfdBuffer {
    fn clone(&self) -> Self {
        // Create a new buffer with the same size
        Self::new(self.original_size, self.alignment, "mnn_buffer_clone")
            .map(|mut new_buf| {
                // Copy data
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        self.ptr,
                        new_buf.ptr,
                        self.original_size,
                    );
                }
                new_buf
            })
            .unwrap_or_else(|_| {
                // Fallback: create empty buffer
                Self {
                    fd: File::open("/dev/null").unwrap(),
                    ptr: std::ptr::null_mut(),
                    size: 0,
                    alignment: self.alignment,
                    original_size: 0,
                }
            })
    }
}

/// Buffer manager for MNN inference
///
/// Manages memfd buffers for multiple tensors
pub struct MemfdBufferManager {
    /// Map of tensor name to buffer
    buffers: std::collections::HashMap<String, Arc<MemfdBuffer>>,
    /// Default alignment
    default_alignment: Alignment,
}

impl MemfdBufferManager {
    /// Create a new buffer manager
    pub fn new() -> Self {
        Self {
            buffers: std::collections::HashMap::new(),
            default_alignment: Alignment::default(),
        }
    }
    
    /// Create with custom alignment
    pub fn with_alignment(alignment: Alignment) -> Self {
        Self {
            buffers: std::collections::HashMap::new(),
            default_alignment: alignment,
        }
    }
    
    /// Get or create a buffer for a tensor
    pub fn get_buffer(
        &mut self,
        tensor_name: &str,
        size: usize,
    ) -> std::io::Result<Arc<MemfdBuffer>> {
        if let Some(buffer) = self.buffers.get(tensor_name) {
            return Ok(buffer.clone());
        }
        
        let buffer = Arc::new(MemfdBuffer::new(
            size,
            self.default_alignment,
            tensor_name,
        )?);
        
        self.buffers.insert(tensor_name.to_string(), buffer.clone());
        Ok(buffer)
    }
    
    /// Get existing buffer by name
    pub fn get_existing(&self, tensor_name: &str) -> Option<Arc<MemfdBuffer>> {
        self.buffers.get(tensor_name).cloned()
    }
    
    /// Remove buffer from manager
    pub fn remove(&mut self, tensor_name: &str) -> Option<Arc<MemfdBuffer>> {
        self.buffers.remove(tensor_name)
    }
    
    /// Clear all buffers
    pub fn clear(&mut self) {
        self.buffers.clear();
    }
    
    /// Create buffer for MNN tensor
    pub fn create_for_mnn(
        &mut self,
        tensor: *mut std::ffi::c_void,
        tensor_name: &str,
        elem_type_code: u8,
        elem_bits: u8,
        dims: &[i32],
    ) -> std::io::Result<Arc<MemfdBuffer>> {
        // Calculate size
        let elem_size = (elem_bits as usize + 7) / 8; // Bytes per element
        let total_elements: usize = dims.iter().product::<i32>() as usize;
        let size = total_elements * elem_size;
        
        // Create buffer
        let buffer = self.get_buffer(tensor_name, size)?;
        
        // Set the host pointer on the MNN tensor
        unsafe {
            // halide_buffer_t layout:
            // - device: uint64_t (offset 0)
            // - host: uint8_t* (offset 8)
            let buffer_ptr = tensor as *mut u8;
            let host_offset = 8;
            let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
            *host_ptr_loc = buffer.as_ptr();
        }
        
        Ok(buffer)
    }
    
    /// Setup MNN tensor with existing buffer
    pub fn setup_mnn_tensor(
        &self,
        tensor: *mut std::ffi::c_void,
        buffer: Arc<MemfdBuffer>,
    ) {
        unsafe {
            let buffer_ptr = tensor as *mut u8;
            let host_offset = 8;
            let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
            *host_ptr_loc = buffer.as_ptr();
        }
    }
    
    /// Sync all buffers for device access
    pub fn sync_all_for_device(&self) -> Vec<std::io::Result<()>> {
        self.buffers.values()
            .map(|b| b.sync_for_device())
            .collect()
    }
    
    /// Sync all buffers for CPU access
    pub fn sync_all_for_cpu(&self) -> Vec<std::io::Result<()>> {
        self.buffers.values()
            .map(|b| b.sync_for_cpu())
            .collect()
    }
}

impl Default for MemfdBufferManager {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// C API for C++ Interop
// ============================================================================

/// Opaque pointer to MemfdBuffer for C API
pub type CMemfdBuffer = MemfdBuffer;

/// Opaque pointer to MemfdBufferManager for C API
pub type CMemfdBufferManager = MemfdBufferManager;

#[no_mangle]
pub extern "C" fn memfd_buffer_create(
    size: usize,
    alignment: u32,
    name: *const libc::c_char,
) -> *mut CMemfdBuffer {
    if name.is_null() {
        return std::ptr::null_mut();
    }
    
    let name_str = unsafe { std::ffi::CStr::from_ptr(name).to_string_lossy().into_owned() };
    let alignment = match alignment {
        4096 => Alignment::Page4K,
        65536 => Alignment::Page64K,
        2097152 => Alignment::Page2M,
        _ => Alignment::default(),
    };
    
    MemfdBuffer::new(size, alignment, &name_str)
        .map(|b| Box::into_raw(Box::new(b)))
        .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub extern "C" fn memfd_buffer_free(buffer: *mut CMemfdBuffer) {
    if !buffer.is_null() {
        unsafe { Box::from_raw(buffer) };
    }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_ptr(buffer: *mut CMemfdBuffer) -> *mut u8 {
    if buffer.is_null() {
        std::ptr::null_mut()
    } else {
        unsafe { (*buffer).as_ptr() }
    }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_size(buffer: *mut CMemfdBuffer) -> usize {
    if buffer.is_null() {
        0
    } else {
        unsafe { (*buffer).size() }
    }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_fd(buffer: *mut CMemfdBuffer) -> i32 {
    if buffer.is_null() {
        -1
    } else {
        unsafe { (*buffer).as_raw_fd() }
    }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_fill(
    buffer: *mut CMemfdBuffer,
    data: *const u8,
    len: usize,
) -> bool {
    if buffer.is_null() || data.is_null() {
        return false;
    }
    
    let slice = unsafe { std::slice::from_raw_parts(data, len) };
    unsafe { (*buffer).fill_from_slice(slice).is_ok() }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_sync_for_device(buffer: *mut CMemfdBuffer) -> bool {
    if buffer.is_null() {
        return false;
    }
    unsafe { (*buffer).sync_for_device().is_ok() }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_sync_for_cpu(buffer: *mut CMemfdBuffer) -> bool {
    if buffer.is_null() {
        return false;
    }
    unsafe { (*buffer).sync_for_cpu().is_ok() }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_manager_new() -> *mut CMemfdBufferManager {
    Box::into_raw(Box::new(MemfdBufferManager::new()))
}

#[no_mangle]
pub extern "C" fn memfd_buffer_manager_free(manager: *mut CMemfdBufferManager) {
    if !manager.is_null() {
        unsafe { Box::from_raw(manager) };
    }
}

#[no_mangle]
pub extern "C" fn memfd_buffer_manager_get(
    manager: *mut CMemfdBufferManager,
    name: *const libc::c_char,
    size: usize,
) -> *mut CMemfdBuffer {
    if manager.is_null() || name.is_null() {
        return std::ptr::null_mut();
    }
    
    let name_str = unsafe { std::ffi::CStr::from_ptr(name).to_string_lossy().into_owned() };
    
    unsafe { (*manager).get_buffer(&name_str, size) }
        .map(|b| Box::into_raw(Box::new(b)))
        .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub extern "C" fn memfd_buffer_manager_setup_mnn(
    manager: *mut CMemfdBufferManager,
    tensor: *mut std::ffi::c_void,
    name: *const libc::c_char,
    elem_bits: u8,
    ndims: i32,
    dims: *const i32,
) -> *mut CMemfdBuffer {
    if manager.is_null() || name.is_null() || tensor.is_null() || dims.is_null() {
        return std::ptr::null_mut();
    }
    
    let name_str = unsafe { std::ffi::CStr::from_ptr(name).to_string_lossy().into_owned() };
    
    // Convert dims to slice
    let dims_slice = unsafe { std::slice::from_raw_parts(dims, ndims as usize) };
    
    // Assume INT32 (4 bytes) for now - can be parameterized later
    let elem_type_code = 0; // INT32
    let elem_bits = 32;
    
    unsafe { (*manager).create_for_mnn(tensor, &name_str, elem_type_code, elem_bits, dims_slice) }
        .map(|b| Box::into_raw(Box::new(b)))
        .unwrap_or(std::ptr::null_mut())
}

// ============================================================================
// C++ Usage Example
// ============================================================================

/// ```cpp
/// #include <MNN/Interpreter.hpp>
/// #include <vector>
/// 
/// // Forward declarations from Rust
/// extern "C" {
///     // Buffer functions
///     void* memfd_buffer_create(size_t size, uint32_t alignment, const char* name);
///     void memfd_buffer_free(void* buffer);
///     uint8_t* memfd_buffer_ptr(void* buffer);
///     size_t memfd_buffer_size(void* buffer);
///     int memfd_buffer_fd(void* buffer);
///     bool memfd_buffer_fill(void* buffer, const uint8_t* data, size_t len);
///     bool memfd_buffer_sync_for_device(void* buffer);
///     bool memfd_buffer_sync_for_cpu(void* buffer);
///     
///     // Manager functions
///     void* memfd_buffer_manager_new();
///     void memfd_buffer_manager_free(void* manager);
///     void* memfd_buffer_manager_get(void* manager, const char* name, size_t size);
///     void* memfd_buffer_manager_setup_mnn(void* manager, void* tensor, const char* name, 
///                                          uint8_t elem_bits, int ndims, const int* dims);
/// }
/// 
/// // Example 1: Simple buffer usage
/// void example_simple() {
///     // Create buffer
///     size_t size = 48 * 64 * sizeof(int32_t);
///     void* buffer = memfd_buffer_create(size, 4096, "input_buffer");
///     uint8_t* ptr = memfd_buffer_ptr(buffer);
///     
///     // Fill with data
///     std::vector<int32_t> data(48*64);
///     for (int i = 0; i < 48*64; i++) data[i] = i % 256;
///     memfd_buffer_fill(buffer, data.data(), size);
///     
///     // Set as MNN input
///     auto net = MNN::Interpreter::createFromFile("model.mnn");
///     auto sess = net->createSession(cfg);
///     auto* in = net->getSessionInput(sess, nullptr);
///     in->buffer().host = ptr;
///     
///     // Sync and run
///     memfd_buffer_sync_for_device(buffer);
///     net->runSession(sess);
///     
///     // Clean up
///     memfd_buffer_free(buffer);
/// }
/// 
/// // Example 2: Using buffer manager
/// void example_with_manager() {
///     // Create manager
///     void* manager = memfd_buffer_manager_new();
///     
///     // Setup MNN
///     auto net = MNN::Interpreter::createFromFile("model.mnn");
///     auto sess = net->createSession(cfg);
///     auto* in = net->getSessionInput(sess, nullptr);
///     
///     // Get or create buffer for this tensor
///     int dims[] = {1, 1, 48, 64};
///     void* buffer = memfd_buffer_manager_setup_mnn(
///         manager, in, "input", 32, 4, dims);
///     
///     if (!buffer) {
///         std::cerr << "Failed to setup buffer" << std::endl;
///         return;
///     }
///     
///     // Fill buffer
///     uint8_t* ptr = memfd_buffer_ptr(buffer);
///     std::vector<int32_t> data(48*64);
///     for (int i = 0; i < 48*64; i++) data[i] = i % 256;
///     memcpy(ptr, data.data(), 48*64*4);
///     
///     // Sync and run
///     memfd_buffer_sync_for_device(buffer);
///     net->runSession(sess);
///     
///     // Clean up
///     memfd_buffer_manager_free(manager);
/// }
/// 
/// // Example 3: With MIPI/ISP Hardware
/// void example_with_mipi() {
///     // Assume we have a MIPI camera that outputs to a memfd
///     int camera_fd = open_mipi_camera("/dev/video0");
///     
///     // Create buffer manager
///     void* manager = memfd_buffer_manager_new();
///     
///     // Create buffer for camera output
///     size_t size = 1920 * 1080 * 2; // RG10 format
///     void* buffer = memfd_buffer_manager_get(manager, "camera_buffer", size);
///     int buffer_fd = memfd_buffer_fd(buffer);
///     
///     // Tell camera to use this buffer
///     configure_camera_output(camera_fd, buffer_fd);
///     
///     // Start camera
///     start_camera(camera_fd);
///     
///     // Setup MNN
///     auto net = MNN::Interpreter::createFromFile("model.mnn");
///     auto sess = net->createSession(cfg);
///     auto* in = net->getSessionInput(sess, nullptr);
///     
///     // Get buffer and set as MNN input
///     uint8_t* ptr = memfd_buffer_ptr(buffer);
///     in->buffer().host = ptr;
///     
///     // In frame processing loop:
///     while (true) {
///         // Wait for frame
///         wait_for_frame(camera_fd);
///         
///         // Sync buffer for device (camera already wrote)
///         memfd_buffer_sync_for_device(buffer);
///         
///         // Run MNN inference
///         net->runSession(sess);
///         
///         // Process output...
///     }
///     
///     // Clean up
///     stop_camera(camera_fd);
///     memfd_buffer_manager_free(manager);
/// }
/// ```
//!
//! # Integration with Existing Rust Code
//!
//! ```rust,ignore
//! use cam_isp::mnn::memfd::{MemfdBuffer, MemfdBufferManager, Alignment};
//!
//! fn run_inference() -> Result<(), Box<dyn std::error::Error>> {
//!     // Create buffer manager
//!     let mut manager = MemfdBufferManager::with_alignment(Alignment::Page64K);
//!
//!     // Load MNN model
//!     let net = unsafe { MNN::Interpreter::createFromFile("model.mnn") };
//!     let sess = net.createSession(cfg);
//!     let in = net.getSessionInput(sess, None);
//!
//!     // Setup buffer for input tensor
//!     let dims = vec![1, 1, 48, 64];
//!     let buffer = manager.create_for_mnn(in, "input", 0, 32, &dims)?;
//!
//!     // Fill buffer with data
//!     let mut data = vec![0u32; 48*64];
//!     for i in 0..48*64 {
//!         data[i] = (i % 256) as u32;
//!     }
//!     buffer.fill_from_slice(&unsafe { 
//!         std::slice::from_raw_parts(
//!             data.as_ptr() as *const u8,
//!             data.len() * 4
//!         )
//!     })?;
//!
//!     // Sync for device
//!     buffer.sync_for_device()?;
//!
//!     // Run inference
//!     net.runSession(sess);
//!
//!     Ok(())
//! }
//! ```
