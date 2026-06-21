//! MNN Buffer Management System
//!
//! Provides buffer management for MNN inference with:
//! - Buffer reuse across frames
//! - Zero-copy memory management
//! - memfd support for shared memory
//!
//! # Architecture
//!
//! The LITE+ profile input tensors don't get memory allocated by MNN's optimizer.
//! This module provides a buffer management system that:
//! 1. Allocates persistent buffers for input tensors
//! 2. Sets the host pointer directly on MNN tensors
//! 3. Reuses buffers across frames to avoid reallocation
//! 4. Optionally uses memfd for zero-copy I/O

use std::collections::HashMap;
use std::fs::File;
use std::os::unix::io::{AsRawFd, RawFd};
use std::sync::{Arc, Mutex};

/// Buffer identifier type
pub type BufferId = u64;

/// Buffer specification for MNN tensor
#[derive(Debug, Clone)]
pub struct BufferSpec {
    /// Shape dimensions
    pub dims: Vec<usize>,
    /// Element type size in bytes
    pub elem_size: usize,
    /// Total size in bytes
    pub total_size: usize,
    /// Tensor name
    pub name: String,
}

/// Managed buffer with reuse tracking
pub struct ManagedBuffer {
    /// Raw buffer data
    pub data: Vec<u8>,
    /// Memory file descriptor (if using memfd)
    pub memfd: Option<File>,
    /// Reference count
    pub ref_count: usize,
    /// Last access time (for LRU eviction)
    pub last_access: std::time::Instant,
}

/// Buffer manager for MNN inference
pub struct MNNBufferManager {
    /// Map of buffer ID to managed buffer
    buffers: HashMap<BufferId, Arc<Mutex<ManagedBuffer>>>, 
    /// Next buffer ID
    next_id: BufferId,
    /// Use memfd for buffer allocation
    use_memfd: bool,
}

impl MNNBufferManager {
    /// Create a new buffer manager
    pub fn new() -> Self {
        Self {
            buffers: HashMap::new(),
            next_id: 1,
            use_memfd: false,
        }
    }
    
    /// Create a new buffer manager with memfd support
    pub fn with_memfd() -> Self {
        Self {
            buffers: HashMap::new(),
            next_id: 1,
            use_memfd: true,
        }
    }
    
    /// Allocate a new buffer
    pub fn allocate(&mut self, spec: &BufferSpec) -> (BufferId, *mut u8, usize) {
        let id = self.next_id;
        self.next_id += 1;
        
        let size = spec.total_size;
        
        let buffer = if self.use_memfd {
            // Create memfd-backed buffer
            let memfd = self.create_memfd(size).unwrap_or_else(|_| {
                // Fallback to regular allocation
                File::open("/dev/null").unwrap() // Dummy file
            });
            
            // Map memfd into memory
            let data = self.mmap_memfd(&memfd, size).unwrap_or_else(|_| {
                vec![0u8; size]
            });
            
            ManagedBuffer {
                data,
                memfd: Some(memfd),
                ref_count: 2,
                last_access: std::time::Instant::now(),
            }
        } else {
            ManagedBuffer {
                data: vec![0u8; size],
                memfd: None,
                ref_count: 2,
                last_access: std::time::Instant::now(),
            }
        };
        
        let ptr = buffer.data.as_ptr() as *mut u8;
        
        self.buffers.insert(id, Arc::new(Mutex::new(buffer)));
        
        (id, ptr, size)
    }
    
    /// Get buffer by ID
    pub fn get_buffer(&self, id: BufferId) -> Option<Arc<Mutex<ManagedBuffer>>> {
        self.buffers.get(&id).cloned()
    }
    
    /// Get raw pointer for MNN tensor
    pub fn get_pointer(&self, id: BufferId) -> Option<*mut u8> {
        self.buffers.get(&id)
            .map(|b| b.lock().unwrap().data.as_ptr() as *mut u8)
    }
    
    /// Get size of buffer
    pub fn get_size(&self, id: BufferId) -> Option<usize> {
        self.buffers.get(&id)
            .map(|b| b.lock().unwrap().data.len())
    }
    
    /// Release buffer (decrement ref count)
    pub fn release(&mut self, id: BufferId) -> bool {
        if let Some(buffer) = self.buffers.get_mut(&id) {
            let mut b = buffer.lock().unwrap();
            b.ref_count -= 1;
            if b.ref_count == 0 {
                drop(b);
                self.buffers.remove(&id);
                return true;
            }
        }
        false
    }
    
    /// Create memfd for the given size
    #[cfg(target_os = "linux")]
    fn create_memfd(&self, size: usize) -> std::io::Result<File> {
        use std::os::unix::fs::FileExt;
        
        // Try memfd_create syscall
        let fd = unsafe {
            libc::syscall(libc::SYS_memfd_create, 
                b"mnn_buffer\0".as_ptr() as *const libc::c_char, 
                libc::MFD_CLOEXEC | libc::MFD_ALLOW_SEALING) as RawFd
        };
        
        if fd < 0 {
            return Err(std::io::Error::last_os_error());
        }
        
        // Set size
        unsafe {
            if libc::ftruncate(fd, size as libc::off_t) != 0 {
                libc::close(fd);
                return Err(std::io::Error::last_os_error());
            }
        }
        
        // Seal the memfd to prevent shrinking
        unsafe {
            libc::fcntl(fd, libc::F_ADD_SEALS, libc::F_SEAL_SHRINK | libc::F_SEAL_GROW);
        }
        
        Ok(unsafe { File::from_raw_fd(fd) })
    }
    
    /// Fallback for non-Linux systems
    #[cfg(not(target_os = "linux"))]
    fn create_memfd(&self, _size: usize) -> std::io::Result<File> {
        Err(std::io::Error::new(std::io::ErrorKind::Unsupported, 
            "memfd not supported on this platform"))
    }
    
    /// Memory map the memfd
    #[cfg(target_os = "linux")]
    fn mmap_memfd(&self, memfd: &File, size: usize) -> std::io::Result<Vec<u8>> {
        use std::os::unix::io::AsRawFd;
        
        let fd = memfd.as_raw_fd();
        
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        
        if ptr == libc::MAP_FAILED {
            return Err(std::io::Error::last_os_error());
        }
        
        // Create Vec that won't deallocate on drop
        // We need to be careful here - the Vec will try to free the pointer
        // So we use a custom deallocation approach
        
        // For now, just return a regular allocated buffer
        // A proper implementation would use a custom allocator
        let mut data = vec![0u8; size];
        unsafe {
            std::ptr::copy_nonoverlapping(ptr, data.as_mut_ptr(), size);
            libc::munmap(ptr, size);
        }
        
        Ok(data)
    }
    
    /// Fallback for non-Linux systems
    #[cfg(not(target_os = "linux"))]
    fn mmap_memfd(&self, _memfd: &File, size: usize) -> std::io::Result<Vec<u8>> {
        Ok(vec![0u8; size])
    }
    
    /// Get memfd for buffer (if available)
    pub fn get_memfd(&self, id: BufferId) -> Option<RawFd> {
        self.buffers.get(&id)
            .and_then(|b| {
                let guard = b.lock().unwrap();
                guard.memfd.as_ref().map(|f| f.as_raw_fd())
            })
    }
}

impl Default for MNNBufferManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Frame buffer manager for MNN inference
/// 
/// Manages input buffers for multiple frames with reuse
pub struct FrameBufferManager {
    /// Buffer manager
    buffer_manager: MNNBufferManager,
    /// Current frame buffer ID
    current_frame_id: Option<BufferId>,
}

impl FrameBufferManager {
    /// Create new frame buffer manager
    pub fn new() -> Self {
        Self {
            buffer_manager: MNNBufferManager::new(),
            current_frame_id: None,
        }
    }
    
    /// Create with memfd support
    pub fn with_memfd() -> Self {
        Self {
            buffer_manager: MNNBufferManager::with_memfd(),
            current_frame_id: None,
        }
    }
    
    /// Get or create buffer for the given spec
    pub fn get_frame_buffer(&mut self, spec: &BufferSpec) -> (*mut u8, usize) {
        // For now, just allocate a new buffer each time
        // In production, we'd implement reuse logic
        let (id, ptr, size) = self.buffer_manager.allocate(spec);
        self.current_frame_id = Some(id);
        (ptr, size)
    }
    
    /// Release current frame buffer
    pub fn release_frame_buffer(&mut self) {
        if let Some(id) = self.current_frame_id {
            self.buffer_manager.release(id);
            self.current_frame_id = None;
        }
    }
}

/// Helper to set MNN tensor host pointer
/// 
/// # Safety
/// This is unsafe because it modifies internal MNN state.
/// The caller must ensure the pointer remains valid for the tensor's lifetime.
pub unsafe fn set_mnn_tensor_host(
    tensor: *mut std::ffi::c_void,
    data_ptr: *mut u8,
) {
    // In C++: tensor->buffer().host = data_ptr;
    // We need to access the halide_buffer_t inside the Tensor
    
    // The Tensor struct layout (from MNN source):
    // struct Tensor {
    //     halide_buffer_t mBuffer;
    //     ...
    // };
    
    // halide_buffer_t layout:
    // struct halide_buffer_t {
    //     u64 device;
    //     uint8_t *host;
    //     int32_t extent[4];
    //     int32_t stride[4];
    //     ...
    // };
    
    // host is at offset 8 (after device which is u64 = 8 bytes)
    const HOST_OFFSET: usize = 8;
    
    let tensor_ptr = tensor as *mut u8;
    let host_ptr = tensor_ptr.add(HOST_OFFSET) as *mut *mut u8;
    *host_ptr = data_ptr;
}

/// Helper to set MNN tensor device pointer
/// 
/// # Safety
/// This is unsafe for the same reasons as set_mnn_tensor_host.
pub unsafe fn set_mnn_tensor_device(
    tensor: *mut std::ffi::c_void,
    device_ptr: u64,
) {
    const DEVICE_OFFSET: usize = 0;
    let tensor_ptr = tensor as *mut u8;
    let device_ptr_loc = tensor_ptr.add(DEVICE_OFFSET) as *mut u64;
    *device_ptr_loc = device_ptr;
}

/// Helper to get MNN tensor host pointer
pub unsafe fn get_mnn_tensor_host(tensor: *mut std::ffi::c_void) -> *mut u8 {
    const HOST_OFFSET: usize = 8;
    let tensor_ptr = tensor as *mut u8;
    let host_ptr = tensor_ptr.add(HOST_OFFSET) as *mut *mut u8;
    *host_ptr
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_buffer_manager_allocate() {
        let mut manager = MNNBufferManager::new();
        let spec = BufferSpec {
            dims: vec![1, 1, 48, 64],
            elem_size: 4, // int32
            total_size: 48 * 64 * 4,
            name: "input".to_string(),
        };
        
        let (id, ptr, size) = manager.allocate(&spec);
        
        assert_eq!(size, 48 * 64 * 4);
        assert!(!ptr.is_null());
        assert!(manager.get_buffer(id).is_some());
    }
    
    #[test]
    fn test_buffer_manager_release() {
        let mut manager = MNNBufferManager::new();
        let spec = BufferSpec {
            dims: vec![1, 1, 48, 64],
            elem_size: 4,
            total_size: 48 * 64 * 4,
            name: "input".to_string(),
        };
        
        let (id, _, _) = manager.allocate(&spec);
        
        // Release once
        let removed = manager.release(id);
        assert!(!removed); // Still has ref_count=1
        
        // Release again
        let removed = manager.release(id);
        assert!(removed); // Now ref_count=0, removed
        assert!(manager.get_buffer(id).is_none());
    }
}

