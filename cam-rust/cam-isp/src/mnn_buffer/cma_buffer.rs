//! CMA (Contiguous Memory Allocator) Buffer Management for MNN
//!
//! Provides hardware-aligned, DMA-capable buffers for:
//! - MIPI camera devices
//! - ISP hardware access
//! - MNN inference
//! - Zero-copy between components
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
//! │  MIPI/ISP   │────▶│  CMA Buffer │────▶│    MNN     │
//! │   Hardware  │     │ (DMA-capable)│     │ Inference  │
//! └─────────────┘     └─────────────┘     └─────────────┘
//!       ▲                    │                    │
//!       │                    │ (mmap)            │ (buffer().host)
//!       └────────────────────┴────────────────────┘
//!                    │
//!         (CMA allocator: /dev/cma, ion, dma-buf)
//! ```
//!
//! # Key Requirements
//!
//! 1. **Hardware Alignment**: Buffers must be aligned to page boundaries (4096 bytes)
//!    or ISP-specific alignment requirements
//! 2. **Contiguous Memory**: Physical memory must be contiguous for DMA
//! 3. **Zero-Copy**: Same buffer used by MIPI, ISP, and MNN without copying
//! 4. **Synchronization**: Proper cache coherency between CPU, GPU, ISP
//! 5. **Lifetime Management**: Client allocates, MNN uses, client frees

use std::collections::HashMap;
use std::fs::OpenOptions;
use std::io;
use std::os::unix::io::{AsRawFd, IntoRawFd, RawFd};
use std::sync::{Arc, RwLock};

/// Alignment requirement for hardware access
/// - 4096 = Page size (minimum for mmap)
/// - 8192 = Common for camera/ISP
/// - 16384 = Some ISP require 16KB alignment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BufferAlignment {
    /// Page alignment (4096 bytes)
    #[default]
    Page = 4096,
    /// 8KB alignment
    Align8K = 8192,
    /// 16KB alignment
    Align16K = 16384,
    /// 64KB alignment
    Align64K = 65536,
}

/// CMA allocation method
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CMAAllocator {
    /// Android ION (Ion Memory Allocator)
    #[default]
    Ion,
    /// Linux CMA (/dev/cma)
    CMA,
    /// Linux DMA-BUF
    DMABuf,
    /// POSIX shm (fallback)
    Shm,
    /// Regular malloc (fallback for non-Linux)
    Malloc,
}

/// Buffer usage flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BufferUsage {
    /// CPU read/write
    CPU = 1,
    /// ISP hardware access
    ISP = 2,
    /// MNN inference
    MNN = 4,
    /// GPU access
    GPU = 8,
    /// Camera input
    Camera = 16,
}

/// CMA buffer information
#[derive(Debug, Clone)]
pub struct CMABufferInfo {
    /// Buffer size in bytes
    pub size: usize,
    /// Alignment requirement
    pub alignment: BufferAlignment,
    /// Allocator used
    pub allocator: CMAAllocator,
    /// Usage flags
    pub usage: Vec<BufferUsage>,
    /// File descriptor
    pub fd: RawFd,
    /// Mapped pointer
    pub ptr: *mut u8,
    /// Physical address (if available)
    pub phys_addr: Option<u64>,
    /// DMA-BUF file descriptor (if separate)
    pub dma_fd: Option<RawFd>,
}

/// CMA Buffer Manager
///
/// Manages hardware-aligned, DMA-capable buffers for the entire application.
/// Buffers are allocated once and reused across frames.
pub struct CMABufferManager {
    /// Allocated buffers by name
    buffers: RwLock<HashMap<String, Arc<CMABuffer>>>,
    /// Default allocator
    default_allocator: CMAAllocator,
    /// Default alignment
    default_alignment: BufferAlignment,
    /// Next buffer ID
    next_id: std::sync::atomic::AtomicU64,
}

/// Individual CMA buffer
#[derive(Debug)]
pub struct CMABuffer {
    /// Buffer ID
    id: u64,
    /// Buffer name
    name: String,
    /// Buffer info
    info: CMABufferInfo,
    /// Reference count
    ref_count: std::sync::atomic::AtomicUsize,
    /// Last access time
    last_access: std::sync::Mutex<std::time::Instant>,
}

impl CMABuffer {
    /// Create a new CMA buffer
    pub fn new(
        id: u64,
        name: String,
        size: usize,
        alignment: BufferAlignment,
        allocator: CMAAllocator,
        usage: Vec<BufferUsage>,
    ) -> io::Result<Self> {
        let info = CMABufferInfo::allocate(size, alignment, allocator, &usage)?;

        Ok(Self {
            id,
            name,
            info,
            ref_count: std::sync::atomic::AtomicUsize::new(1),
            last_access: std::sync::Mutex::new(std::time::Instant::now()),
        })
    }

    /// Get buffer ID
    pub fn id(&self) -> u64 {
        self.id
    }

    /// Get buffer name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get raw pointer
    pub fn as_ptr(&self) -> *mut u8 {
        self.info.ptr
    }

    /// Get size
    pub fn size(&self) -> usize {
        self.info.size
    }

    /// Get file descriptor
    pub fn as_fd(&self) -> RawFd {
        self.info.fd
    }

    /// Get DMA-BUF file descriptor
    pub fn as_dma_fd(&self) -> Option<RawFd> {
        self.info.dma_fd
    }

    /// Increment reference count
    pub fn acquire(&self) {
        self.ref_count
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        *self.last_access.lock().unwrap() = std::time::Instant::now();
    }

    /// Decrement reference count
    pub fn release(&self) -> bool {
        let count = self
            .ref_count
            .fetch_sub(1, std::sync::atomic::Ordering::SeqCst);
        count == 1 // Was 1, now 0
    }

    /// Get reference count
    pub fn ref_count(&self) -> usize {
        self.ref_count.load(std::sync::atomic::Ordering::SeqCst)
    }

    /// Sync buffer for CPU access
    pub fn sync_for_cpu(&self) -> io::Result<()> {
        CMABufferInfo::sync_for_cpu(self.info.fd, self.info.ptr, self.info.size)
    }

    /// Sync buffer for device access
    pub fn sync_for_device(&self) -> io::Result<()> {
        CMABufferInfo::sync_for_device(self.info.fd, self.info.ptr, self.info.size)
    }
}

impl Drop for CMABuffer {
    fn drop(&mut self) {
        // Only deallocate if ref_count is 0
        // (should be handled by release())
        if self.ref_count() == 0 {
            let _ = CMABufferInfo::deallocate(&self.info);
        }
    }
}

impl CMABufferInfo {
    /// Allocate a CMA buffer info
    pub fn allocate(
        size: usize,
        alignment: BufferAlignment,
        allocator: CMAAllocator,
        usage: &[BufferUsage],
    ) -> io::Result<Self> {
        // Round up size to alignment
        let aligned_size = Self::align_up(size, alignment as usize);

        match allocator {
            CMAAllocator::Ion => Self::allocate_ion(aligned_size, usage),
            CMAAllocator::CMA => Self::allocate_cma(aligned_size),
            CMAAllocator::DMABuf => Self::allocate_dmabuf(aligned_size),
            CMAAllocator::Shm => Self::allocate_shm(aligned_size),
            CMAAllocator::Malloc => Self::allocate_malloc(aligned_size, alignment),
        }
    }

    /// Deallocate a CMA buffer info
    pub fn deallocate(info: &Self) -> io::Result<()> {
        match info.allocator {
            CMAAllocator::Ion => {
                Self::deallocate_ion(info);
                Ok(())
            }
            CMAAllocator::CMA => {
                Self::deallocate_cma(info);
                Ok(())
            }
            CMAAllocator::DMABuf => {
                Self::deallocate_dmabuf(info);
                Ok(())
            }
            CMAAllocator::Shm => {
                Self::deallocate_shm(info);
                Ok(())
            }
            CMAAllocator::Malloc => {
                Self::deallocate_malloc(info);
                Ok(())
            }
        }
    }

    /// Align size up to alignment
    fn align_up(size: usize, alignment: usize) -> usize {
        (size + alignment - 1) & !(alignment - 1)
    }

    /// Allocate using Android ION
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn allocate_ion(size: usize, usage: &[BufferUsage]) -> io::Result<Self> {
        // Open /dev/ion
        let ion_fd = OpenOptions::new().read(true).write(true).open("/dev/ion")?;

        // Calculate ION flags based on usage
        let mut ion_flags = 0;
        for u in usage {
            match u {
                BufferUsage::CPU => ion_flags |= 0x00000001, // ION_FLAG_CACHED
                BufferUsage::ISP | BufferUsage::Camera => ion_flags |= 0x00000002, // ION_FLAG_CACHED_NEON
                BufferUsage::GPU => ion_flags |= 0x00000004,                       // ION_FLAG_GPU
                BufferUsage::MNN => ion_flags |= 0x00000001,                       // CPU for MNN
            }
        }

        // ION_IOC_ALLOC
        const ION_IOC_MAGIC: u8 = b'I';
        const ION_IOC_ALLOC: u8 = 0;
        const ION_IOC_ALLOC_CMD: libc::c_uint =
            ((ION_IOC_MAGIC as u32) << 8) | (ION_IOC_ALLOC as u32);

        // ion_allocation_data structure:
        // len, align, heap_mask, flags
        let heap_mask = -1i32; // All heaps (0xFFFFFFFF as i32)

        let alloc_data = [
            size as u32,
            4096, // alignment (4KB minimum for ION)
            heap_mask as u32,
            ion_flags as u32,
        ];

        let fd = unsafe {
            libc::ioctl(
                ion_fd.as_raw_fd(),
                ION_IOC_ALLOC_CMD as i32,
                &alloc_data as *const _ as *mut libc::c_void,
            ) as RawFd
        };

        if fd < 0 {
            return Err(io::Error::last_os_error());
        }

        // Map the ION buffer
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
            unsafe { libc::close(fd) };
            return Err(io::Error::last_os_error());
        }

        // Create DMA-BUF from ION
        let dma_fd = Self::ion_to_dmabuf(fd)?;

        Ok(Self {
            size,
            alignment: BufferAlignment::Page,
            allocator: CMAAllocator::Ion,
            usage: usage.to_vec(),
            fd,
            ptr: ptr as *mut u8,
            phys_addr: None,
            dma_fd: Some(dma_fd),
        })
    }

    /// Convert ION buffer to DMA-BUF
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn ion_to_dmabuf(ion_fd: RawFd) -> io::Result<RawFd> {
        const ION_IOC_MAGIC: u8 = b'I';
        const ION_IOC_SHARE: u8 = 4;
        const ION_IOC_SHARE_CMD: libc::c_uint =
            ((ION_IOC_MAGIC as u32) << 8) | (ION_IOC_SHARE as u32);

        let share_fd = unsafe {
            libc::ioctl(
                ion_fd,
                ION_IOC_SHARE_CMD as i32,
                &ion_fd as *const _ as *mut libc::c_void,
            ) as RawFd
        };

        if share_fd < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(share_fd)
        }
    }

    /// Allocate using /dev/cma
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn allocate_cma(size: usize) -> io::Result<Self> {
        // Open /dev/cma
        let cma_fd = OpenOptions::new().read(true).write(true).open("/dev/cma")?;

        // Allocate from CMA
        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                cma_fd.as_raw_fd(),
                0,
            )
        };

        if ptr == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }

        Ok(Self {
            size,
            alignment: BufferAlignment::Page,
            allocator: CMAAllocator::CMA,
            usage: vec![],
            fd: cma_fd.into_raw_fd(),
            ptr: ptr as *mut u8,
            phys_addr: None,
            dma_fd: None,
        })
    }

    /// Allocate using DMABuf (direct dma-buf import)
    fn allocate_dmabuf(_size: usize) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "DMABuf allocator not available on this platform",
        ))
    }

    /// Allocate using POSIX shared memory
    fn allocate_shm(_size: usize) -> io::Result<Self> {
        #[cfg(target_os = "linux")]
        {
            // Create unique name
            let name = format!("/mnn_cma_{}", std::process::id());
            let name_c = std::ffi::CString::new(name.clone()).unwrap();

            // shm_open
            let fd =
                unsafe { libc::shm_open(name_c.as_ptr(), libc::O_CREAT | libc::O_RDWR, 0o600) };

            if fd < 0 {
                return Err(io::Error::last_os_error());
            }

            // ftruncate
            if unsafe { libc::ftruncate(fd, size as libc::off_t) } != 0 {
                unsafe { libc::close(fd) };
                unsafe { libc::shm_unlink(name_c.as_ptr()) };
                return Err(io::Error::last_os_error());
            }

            // mmap
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
                unsafe { libc::close(fd) };
                unsafe { libc::shm_unlink(name_c.as_ptr()) };
                return Err(io::Error::last_os_error());
            }

            Ok(Self {
                size,
                alignment: BufferAlignment::Page,
                allocator: CMAAllocator::Shm,
                usage: vec![],
                fd,
                ptr: ptr as *mut u8,
                phys_addr: None,
                dma_fd: None,
            })
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "shm not supported on this platform",
            ))
        }
    }

    /// Allocate using malloc with alignment
    fn allocate_malloc(size: usize, alignment: BufferAlignment) -> io::Result<Self> {
        let aligned_size = Self::align_up(size, alignment as usize);

        let ptr = unsafe { libc::aligned_alloc(alignment as usize, aligned_size) as *mut u8 };

        if ptr.is_null() {
            return Err(io::Error::last_os_error());
        }

        Ok(Self {
            size: aligned_size,
            alignment,
            allocator: CMAAllocator::Malloc,
            usage: vec![],
            fd: -1, // No file descriptor for malloc
            ptr,
            phys_addr: None,
            dma_fd: None,
        })
    }

    /// Deallocate ION buffer
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn deallocate_ion(info: &Self) {
        if !info.ptr.is_null() {
            unsafe { libc::munmap(info.ptr as *mut libc::c_void, info.size) };
        }
        if info.fd >= 0 {
            unsafe { libc::close(info.fd) };
        }
        if let Some(dma_fd) = info.dma_fd {
            if dma_fd >= 0 {
                unsafe { libc::close(dma_fd) };
            }
        }
    }

    /// Deallocate CMA buffer
    #[cfg(any(target_os = "linux", target_os = "android"))]
    fn deallocate_cma(info: &Self) {
        if !info.ptr.is_null() {
            unsafe { libc::munmap(info.ptr as *mut libc::c_void, info.size) };
        }
        if info.fd >= 0 {
            unsafe { libc::close(info.fd) };
        }
    }

    /// Deallocate DMABuf buffer
    fn deallocate_dmabuf(_info: &Self) {
        // DMABuf handles deallocation via fd close on drop
    }

    /// Deallocate shm buffer
    #[cfg(any(target_os = "linux", target_os = "android"))]
    /// Deallocate shm buffer
    #[cfg(target_os = "linux")]
    fn deallocate_shm(info: &Self) {
        if !info.ptr.is_null() {
            unsafe { libc::munmap(info.ptr as *mut libc::c_void, info.size) };
        }
        if info.fd >= 0 {
            // Clean up shm
            let name = format!("/mnn_cma_{}", std::process::id());
            let name_c = std::ffi::CString::new(name).unwrap();
            unsafe { libc::shm_unlink(name_c.as_ptr()) };
            unsafe { libc::close(info.fd) };
        }
    }

    #[cfg(not(target_os = "linux"))]
    fn deallocate_shm(info: &Self) {
        if !info.ptr.is_null() {
            unsafe { libc::munmap(info.ptr as *mut libc::c_void, info.size) };
        }
        if info.fd >= 0 {
            unsafe { libc::close(info.fd) };
        }
    }

    /// Deallocate malloc buffer
    fn deallocate_malloc(info: &Self) {
        if !info.ptr.is_null() {
            unsafe { libc::free(info.ptr as *mut libc::c_void) };
        }
    }

    /// Sync buffer for CPU access
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn sync_for_cpu(fd: RawFd, ptr: *mut u8, size: usize) -> io::Result<()> {
        if fd >= 0 {
            // Use msync for cache invalidation (works on both Linux and Android)
            let result =
                unsafe { libc::msync(ptr as *mut libc::c_void, size, libc::MS_INVALIDATE) };

            if result != 0 {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }

    /// Sync buffer for device access
    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn sync_for_device(fd: RawFd, ptr: *mut u8, size: usize) -> io::Result<()> {
        if fd >= 0 {
            // Use msync for cache flush (works on both Linux and Android)
            let result = unsafe { libc::msync(ptr as *mut libc::c_void, size, libc::MS_SYNC) };

            if result != 0 {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }

    /// Fallback for non-Linux
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    pub fn sync_for_cpu(_fd: RawFd, _ptr: *mut u8, _size: usize) -> io::Result<()> {
        Ok(())
    }

    /// Fallback for non-Linux
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    pub fn sync_for_device(_fd: RawFd, _ptr: *mut u8, _size: usize) -> io::Result<()> {
        Ok(())
    }
}

impl CMABufferManager {
    /// Create a new CMA buffer manager
    pub fn new() -> Self {
        Self {
            buffers: RwLock::new(HashMap::new()),
            default_allocator: CMAAllocator::default(),
            default_alignment: BufferAlignment::default(),
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }

    /// Create with custom settings
    pub fn with_settings(allocator: CMAAllocator, alignment: BufferAlignment) -> Self {
        Self {
            buffers: RwLock::new(HashMap::new()),
            default_allocator: allocator,
            default_alignment: alignment,
            next_id: std::sync::atomic::AtomicU64::new(1),
        }
    }

    /// Allocate a new buffer
    pub fn allocate(
        &self,
        name: &str,
        size: usize,
        usage: &[BufferUsage],
    ) -> io::Result<Arc<CMABuffer>> {
        let id = self
            .next_id
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);

        let buffer = CMABuffer::new(
            id,
            name.to_string(),
            size,
            self.default_alignment,
            self.default_allocator,
            usage.to_vec(),
        )?;

        let buffer = Arc::new(buffer);
        self.buffers
            .write()
            .unwrap()
            .insert(name.to_string(), buffer.clone());

        Ok(buffer)
    }

    /// Get buffer by name
    pub fn get_buffer(&self, name: &str) -> Option<Arc<CMABuffer>> {
        self.buffers.read().unwrap().get(name).cloned()
    }

    /// Acquire buffer (increment ref count)
    pub fn acquire(&self, name: &str) -> Option<Arc<CMABuffer>> {
        let buffer = self.get_buffer(name)?;
        buffer.acquire();
        Some(buffer)
    }

    /// Release buffer (decrement ref count, remove if 0)
    pub fn release(&self, name: &str) -> bool {
        let mut buffers = self.buffers.write().unwrap();
        if let Some(buffer) = buffers.get_mut(name) {
            if Arc::strong_count(buffer) == 1 {
                // Only we hold the reference
                if buffer.release() {
                    buffers.remove(name);
                    return true;
                }
            } else {
                // Multiple references, just decrement
                buffer.release();
            }
        }
        false
    }

    /// Get buffer for MNN tensor
    ///
    /// Returns a buffer suitable for the given tensor spec
    pub fn get_mnn_buffer(
        &self,
        tensor_name: &str,
        _elem_type: u8, // MNN data type code
        elem_bits: u8,  // Bits per element
        dims: &[i32],
    ) -> io::Result<Arc<CMABuffer>> {
        // Calculate size
        let elem_size = (elem_bits as usize).div_ceil(8); // Round up to bytes
        let total_size = dims.iter().product::<i32>() as usize * elem_size;

        // Determine alignment based on usage
        let _alignment = if total_size >= 1024 * 1024 {
            // Large buffers: 64KB alignment for better DMA
            BufferAlignment::Align64K
        } else if total_size >= 64 * 1024 {
            // Medium buffers: 16KB alignment
            BufferAlignment::Align16K
        } else {
            // Small buffers: page alignment
            BufferAlignment::Page
        };

        // Determine usage
        let usage = vec![BufferUsage::CPU, BufferUsage::MNN];

        self.allocate(tensor_name, total_size, &usage)
    }

    /// Setup buffer for MNN tensor
    ///
    /// This sets up the buffer and returns the pointer to assign to the tensor
    pub fn setup_mnn_tensor(
        &self,
        tensor: *mut std::ffi::c_void,
        tensor_name: &str,
        elem_type: u8,
        elem_bits: u8,
        dims: &[i32],
    ) -> io::Result<Arc<CMABuffer>> {
        // Get or create buffer
        let buffer = self.get_mnn_buffer(tensor_name, elem_type, elem_bits, dims)?;

        // Set the host pointer on the MNN tensor
        unsafe {
            let buffer_ptr = tensor as *mut u8;
            let host_offset = 8; // After device (uint64_t)
            let host_ptr_loc = buffer_ptr.add(host_offset) as *mut *mut u8;
            *host_ptr_loc = buffer.as_ptr();
        }

        Ok(buffer)
    }

    /// Sync buffer for MNN inference
    ///
    /// Call this after filling the buffer and before runSession
    pub fn sync_for_mnn(&self, name: &str) -> io::Result<()> {
        let buffer = self
            .get_buffer(name)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Buffer not found"))?;
        buffer.sync_for_device()?;
        Ok(())
    }

    /// Sync buffer after MNN inference
    ///
    /// Call this after runSession if you need to read the output
    pub fn sync_from_mnn(&self, name: &str) -> io::Result<()> {
        let buffer = self
            .get_buffer(name)
            .ok_or_else(|| io::Error::new(io::ErrorKind::NotFound, "Buffer not found"))?;
        buffer.sync_for_cpu()?;
        Ok(())
    }

    /// List all buffers
    pub fn list_buffers(&self) -> Vec<(String, Arc<CMABuffer>)> {
        self.buffers
            .read()
            .unwrap()
            .iter()
            .map(|(name, buffer)| (name.clone(), buffer.clone()))
            .collect()
    }

    /// Clear all buffers
    pub fn clear(&mut self) {
        self.buffers.write().unwrap().clear();
    }
}

impl Default for CMABufferManager {
    fn default() -> Self {
        Self::new()
    }
}

/// C API for C++ interop
///
/// These functions allow C++ code to use the CMA buffer manager
#[no_mangle]
pub extern "C" fn cma_buffer_manager_new() -> *mut CMABufferManager {
    Box::into_raw(Box::new(CMABufferManager::new()))
}

#[no_mangle]
pub extern "C" fn cma_buffer_manager_free(manager: *mut CMABufferManager) {
    if !manager.is_null() {
        unsafe {
            let _ = Box::from_raw(manager);
        }
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_manager_allocate(
    manager: *mut CMABufferManager,
    name: *const libc::c_char,
    size: usize,
) -> *mut CMABuffer {
    if manager.is_null() || name.is_null() {
        return std::ptr::null_mut();
    }

    let name_str = unsafe {
        std::ffi::CStr::from_ptr(name)
            .to_string_lossy()
            .into_owned()
    };

    unsafe { (*manager).allocate(&name_str, size, &[BufferUsage::CPU, BufferUsage::MNN]) }
        .map(|b| Box::into_raw(Box::new(Arc::try_unwrap(b).unwrap())))
        .unwrap_or(std::ptr::null_mut())
}

#[no_mangle]
pub extern "C" fn cma_buffer_ptr(buffer: *mut CMABuffer) -> *mut u8 {
    if buffer.is_null() {
        std::ptr::null_mut()
    } else {
        unsafe { (*buffer).as_ptr() }
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_size(buffer: *mut CMABuffer) -> usize {
    if buffer.is_null() {
        0
    } else {
        unsafe { (*buffer).size() }
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_fd(buffer: *mut CMABuffer) -> i32 {
    if buffer.is_null() {
        -1
    } else {
        unsafe { (*buffer).info.fd }
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_acquire(buffer: *mut CMABuffer) {
    if !buffer.is_null() {
        unsafe { (*buffer).acquire() };
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_release(buffer: *mut CMABuffer) -> bool {
    if buffer.is_null() {
        false
    } else {
        unsafe { (*buffer).release() }
    }
}

#[no_mangle]
pub extern "C" fn cma_buffer_free(buffer: *mut CMABuffer) {
    if !buffer.is_null() {
        unsafe {
            let _ = Box::from_raw(buffer);
        }
    }
}

// Example usage from C++:
//
// ```cpp
// #include <MNN/Interpreter.hpp>
//
// // Forward declarations from Rust
// extern "C" {
//     void* cma_buffer_manager_new();
//     void cma_buffer_manager_free(void* manager);
//     void* cma_buffer_manager_allocate(void* manager, const char* name, size_t size);
//     uint8_t* cma_buffer_ptr(void* buffer);
//     size_t cma_buffer_size(void* buffer);
//     int cma_buffer_fd(void* buffer);
//     void cma_buffer_acquire(void* buffer);
//     bool cma_buffer_release(void* buffer);
//     void cma_buffer_free(void* buffer);
// }
//
// void run_inference_with_cma() {
//     // Create buffer manager
//     void* manager = cma_buffer_manager_new();
//
//     // Create MNN interpreter
//     auto net = MNN::Interpreter::createFromFile("model.mnn");
//     auto sess = net->createSession(cfg);
//     auto* in = net->getSessionInput(sess, nullptr);
//
//     // Allocate CMA buffer
//     size_t size = 48 * 64 * sizeof(int32_t);
//     void* buffer = cma_buffer_manager_allocate(manager, "input_buffer", size);
//     uint8_t* ptr = cma_buffer_ptr(buffer);
//
//     // Fill buffer with camera data
//     std::vector<int32_t> frame_data(48*64);
//     // ... fill from MIPI/ISP ...
//     memcpy(ptr, frame_data.data(), size);
//
//     // Set as MNN input
//     in->buffer().host = ptr;
//
//     // Run inference
//     net->runSession(sess);
//
//     // Clean up
//     cma_buffer_release(buffer);
//     cma_buffer_free(buffer);
//     cma_buffer_manager_free(manager);
// }
//
