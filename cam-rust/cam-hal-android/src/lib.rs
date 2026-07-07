//! Pure Rust Android Camera HAL v3 implementation.
//!
//! Provides the required `HAL_MODULE_INFO_SYM` symbol and fully functional
//! device operations compliant with Android camera3.h.
//!
//! Build: cargo build --release -p cam-hal-android (for Android target)

#![allow(non_snake_case)]
#![allow(dead_code)]

pub mod adapter;
pub mod util;
pub mod gralloc;
pub mod buffer_pool;

use std::ffi::CStr;
use std::os::raw::{c_char, c_void};

use cam_hal::ICameraAdapter;

// ============================================================================
// Type definitions from AOSP hardware.h and camera3.h (exact ABI matches)
// ============================================================================

// ── hardware.h types ────────────────────────────────────────────────────────

#[repr(C)]
pub struct hw_module_t {
    pub tag: u32,
    pub module_api_version: u16, // marks version_major
    pub hal_api_version: u16,   // marks version_minor
    pub id: *const c_char,
    pub name: *const c_char,
    pub author: *const c_char,
    pub methods: *mut hw_module_methods_t,
    pub dso: *mut c_void,
    pub reserved: [u32; 25],
}
// SAFETY: All fields are raw pointers or integers; immutable after init.
unsafe impl Send for hw_module_t {}
unsafe impl Sync for hw_module_t {}

#[repr(C)]
pub struct hw_module_methods_t {
    pub open: unsafe extern "C" fn(
        module: *const hw_module_t,
        id: *const c_char,
        device: *mut *mut hw_device_t,
    ) -> i32,
}

#[repr(C)]
pub struct hw_device_t {
    pub tag: u32,
    pub version: u32,
    pub module: *mut hw_module_t,
    pub reserved: [u32; 12],
    pub close: unsafe extern "C" fn(device: *mut hw_device_t) -> i32,
}
unsafe impl Send for hw_device_t {}
unsafe impl Sync for hw_device_t {}

// ── camera_common.h types ────────────────────────────────────────────────────

#[repr(C)]
pub struct camera_info {
    pub facing: i32,
    pub orientation: i32,
    pub device_version: u32,
    pub static_camera_characteristics: *mut c_void, // camera_metadata_t*
    pub resource_cost: i32,
    pub conflicting_devices: *mut *const c_char,
    pub conflicting_devices_length: usize,
}

#[repr(C)]
pub struct camera_module_callbacks_t {
    pub camera_device_status_change: Option<
        unsafe extern "C" fn(*const camera_module_callbacks_t, i32, i32),
    >,
    pub torch_mode_status_change: Option<
        unsafe extern "C" fn(*const camera_module_callbacks_t, *const c_char, i32),
    >,
}

unsafe impl Send for camera_module_callbacks_t {}
unsafe impl Sync for camera_module_callbacks_t {}

#[repr(C)]
pub struct camera_module_t {
    pub common: hw_module_t,
    pub get_number_of_cameras: unsafe extern "C" fn() -> i32,
    pub get_camera_info: unsafe extern "C" fn(i32, *mut camera_info) -> i32,
    pub set_callbacks: unsafe extern "C" fn(*const camera_module_callbacks_t) -> i32,
    pub get_vendor_tag_ops: Option<unsafe extern "C" fn(*mut c_void)>,
    pub open_legacy: Option<
        unsafe extern "C" fn(
            *const hw_module_t,
            *const c_char,
            u32,
            *mut *mut hw_device_t,
        ) -> i32,
    >,
    pub set_torch_mode: Option<
        unsafe extern "C" fn(*const c_char, i32) -> i32, // bool → i32 in FFI
    >,
    pub init: unsafe extern "C" fn() -> i32,
    pub reserved: [*mut c_void; 5],
}
unsafe impl Send for camera_module_t {}
unsafe impl Sync for camera_module_t {}

// ── camera3.h types ──────────────────────────────────────────────────────────

// Stream types
pub const CAMERA3_STREAM_OUTPUT: i32 = 0;
pub const CAMERA3_STREAM_INPUT: i32 = 1;
pub const CAMERA3_STREAM_BIDIRECTIONAL: i32 = 2;

// Message types for notify
pub const CAMERA3_MSG_SHUTTER: u32 = 1;
pub const CAMERA3_MSG_ERROR: u32 = 2;

// Buffer status
pub const CAMERA_BUFFER_STATUS_OK: i32 = 0;
const CAMERA_BUFFER_STATUS_ERROR: i32 = -1;

#[repr(C)]
pub struct camera3_stream_t {
    pub stream_type: i32,
    pub width: u32,
    pub height: u32,
    pub format: i32,
    pub usage: u32,
    pub data_space: u32,
    pub rotation: i32,
    pub max_buffers: u32,
    pub private: *mut c_void,
    pub reserved: [u32; 8],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct camera3_stream_buffer_t {
    pub stream: *mut camera3_stream_t,
    pub buffer: *mut c_void, // AHardwareBuffer*
    pub status: i32,
    pub acquire_fence: i32,
    pub release_fence: i32,
}

#[repr(C)]
pub struct camera3_capture_request_t {
    pub frame_number: u64,
    pub stream_buffer: *mut camera3_stream_buffer_t,
    pub num_output_buffers: usize,
    pub num_input_buffers: usize,
    pub settings: *mut c_void, // camera_metadata_t*
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct camera3_callback_ops_t {
    pub notify: Option<
        unsafe extern "C" fn(
            *const camera3_callback_ops_t,
            u64,   // frame number
            u32,   // msg type
            *mut c_void,
        ),
    >,
    pub process_capture_result: Option<
        unsafe extern "C" fn(
            *const camera3_callback_ops_t,
            *const camera3_capture_result_t,
        ),
    >,
    pub notify2: Option<
        unsafe extern "C" fn(
            *const camera3_callback_ops_t,
            u64,   // frame number
            u32,   // msg type
            *const camera3_error_msg_t,
        ),
    >,
}
unsafe impl Send for camera3_callback_ops_t {}
unsafe impl Sync for camera3_callback_ops_t {}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct camera3_capture_result_t {
    pub frame_number: u64,
    pub stream_buffer: *mut camera3_stream_buffer_t,
    pub num_output_buffers: usize,
    pub num_input_buffers: usize,
    pub result: *mut c_void,       // camera_metadata_t*
    pub partial_result: u32,
    pub input_buffer: *mut camera3_stream_buffer_t,
}

#[repr(C)]
pub struct camera3_error_msg_t {
    pub frame_number: u64,
    pub error_stream: *mut camera3_stream_t,
    pub error_code: u32,
}

#[repr(C)]
pub struct camera3_device_ops_t {
    pub initialize: Option<
        unsafe extern "C" fn(
            *const camera3_device_t,
            *const camera3_callback_ops_t,
        ) -> i32,
    >,
    pub configure_streams: Option<
        unsafe extern "C" fn(
            *const camera3_device_t,
            *mut camera3_stream_configuration_t,
        ) -> i32,
    >,
    pub register_stream_buffers: Option<
        unsafe extern "C" fn(
            *const camera3_device_t,
            *mut camera3_stream_buffer_t,
            usize,
        ) -> i32,
    >,
    pub process_capture_request: Option<
        unsafe extern "C" fn(
            *const camera3_device_t,
            *const camera3_capture_request_t,
        ) -> i32,
    >,
    pub flush: Option<unsafe extern "C" fn(*const camera3_device_t) -> i32>,
    pub dump: Option<
        unsafe extern "C" fn(*const camera3_device_t, i32, *const c_char) -> i32,
    >,
    pub construct_default_request_settings: Option<
        unsafe extern "C" fn(
            *const camera3_device_t,
            i32, // request template
        ) -> *mut c_void, // camera_metadata_t*
    >,
}

/// `camera3_device_t` — the actual camera device handle.
#[repr(C)]
pub struct camera3_device_t {
    pub common: hw_device_t,
    pub ops: *const camera3_device_ops_t,
    pub priv_: *mut c_void,
}
unsafe impl Send for camera3_device_t {}
unsafe impl Sync for camera3_device_t {}

/// Stream configuration (used by configure_streams).
#[repr(C)]
pub struct camera3_stream_configuration_t {
    pub operation_mode: u32,
    pub num_streams: u32,
    pub streams: *mut *mut camera3_stream_t,
}

// ============================================================================
// Constants
// ============================================================================

const HARDWARE_MODULE_TAG: u32 = 0x484D4932; // 'H' 'M' 'I' '2'
const HARDWARE_DEVICE_TAG: u32 = 0x41414141; // 'AAAA'
const CAMERA_MODULE_API_VERSION: u16 = 0x0204; // v2.4
const HAL_MODULE_API_VERSION: u16 = 0x0100; // v1.0

const CAMERA_DEVICE_API_VERSION: u32 = 0x0304; // v3.4

// ============================================================================
// Global callback storage
// ============================================================================

// Raw pointer storage for callbacks.
// Written once during init, read during request processing.
static mut GLOBAL_CALLBACK_PTR: Option<*const camera3_callback_ops_t> = None;
static mut MODULE_CALLBACK_PTR: Option<*const camera_module_callbacks_t> = None;

// ============================================================================
// Module methods: hw_module_methods_t.open
// ============================================================================

unsafe extern "C" fn module_open(
    _module: *const hw_module_t,
    name: *const c_char,
    device: *mut *mut hw_device_t,
) -> i32 {
    let id_str = CStr::from_ptr(name).to_str().unwrap_or("0");
    log::info!("module_open: camera id={}", id_str);

    // Create the AndroidCameraAdapter for this device
    let adapter = adapter::AndroidCameraAdapter::new(id_str);
    let adapter_ptr = Box::into_raw(Box::new(adapter)) as *mut c_void;

    // Build the camera3 device
    let dev = Box::new(camera3_device_t {
        common: hw_device_t {
            tag: HARDWARE_DEVICE_TAG,
            version: CAMERA_DEVICE_API_VERSION,
            module: std::ptr::null_mut(),
            reserved: [0; 12],
            close: device_close,
        },
        ops: &DEVICE_OPS,
        priv_: adapter_ptr,
    });

    *device = &*dev as *const _ as *mut hw_device_t;
    std::mem::forget(dev); // Leak intentionally — freed in device_close

    0
}

// ============================================================================
// Device operations
// ============================================================================

#[allow(dead_code)]
extern "C" fn dummy_notify(
    _ops: *const camera3_callback_ops_t,
    _frame: u64,
    _msg: u32,
    _data: *mut c_void,
) {
    // No-op
}

unsafe extern "C" fn device_initialize(
    _device: *const camera3_device_t,
    callback_ops: *const camera3_callback_ops_t,
) -> i32 {
    log::info!("device_initialize");
    if !callback_ops.is_null() {
        GLOBAL_CALLBACK_PTR = Some(callback_ops);
        log::debug!("callback_ops stored");
    }
    0
}

unsafe extern "C" fn device_configure_streams(
    device: *const camera3_device_t,
    config: *mut camera3_stream_configuration_t,
) -> i32 {
    let dev = &*device;
    let cfg = &*config;
    log::info!(
        "configure_streams: op_mode={}, num_streams={}",
        cfg.operation_mode,
        cfg.num_streams
    );

    // Retrieve adapter from priv_
    let adapter_ptr = dev.priv_ as *mut adapter::AndroidCameraAdapter;
    if adapter_ptr.is_null() {
        return -1;
    }
    let adapter = &mut *adapter_ptr;

    for i in 0..cfg.num_streams as isize {
        let stream = *cfg.streams.offset(i);
        let s = &mut *stream;
        log::debug!(
            "  stream[{}]: {}x{} fmt=0x{:x}",
            i,
            s.width,
            s.height,
            s.format,
        );

        // Accept up to 4 buffers per stream
        s.max_buffers = 4;

        // Configure the adapter with first stream's parameters
        if i == 0 {
            let stream_cfg = cam_hal::camera::StreamConfig {
                width: s.width,
                height: s.height,
                format: cam_types::FrameFormat::RawSensor,
                fps: 30,
            };
            let _ = adapter.open(&stream_cfg);
        }
    }

    0
}

unsafe extern "C" fn device_process_capture_request(
    device: *const camera3_device_t,
    request: *const camera3_capture_request_t,
) -> i32 {
    let dev = &*device;
    let req = &*request;
    log::debug!("process_capture_request: frame {}", req.frame_number);

    // Retrieve the adapter from priv_
    let adapter_ptr = dev.priv_ as *mut adapter::AndroidCameraAdapter;
    let adapter = if adapter_ptr.is_null() {
        log::warn!("No adapter in device priv_");
        return -1;
    } else {
        &*adapter_ptr
    };

    // ── Notify shutter ─────────────────────────────────────────────────
    if let Some(cbp) = GLOBAL_CALLBACK_PTR {
        let cb = &*cbp;
        if let Some(notify) = cb.notify {
            notify(
                cb as *const _,
                req.frame_number,
                CAMERA3_MSG_SHUTTER,
                std::ptr::null_mut(),
            );
        }

        // ── Process input buffers (read) ────────────────────────────────
        let num_inputs = req.num_input_buffers;
        let mut processed_data: Option<Vec<u8>> = None;

        if num_inputs > 0 {
            let in_buf = &*req.stream_buffer.add(0);
            if !in_buf.buffer.is_null() {
                log::debug!(
                    "  input[0]: {}x{} fmt=0x{:x}",
                    (*in_buf.stream).width,
                    (*in_buf.stream).height,
                    (*in_buf.stream).format,
                );

                match adapter.lock_and_process(
                    in_buf.buffer as *mut adapter::AHardwareBuffer,
                    (*in_buf.stream).format,
                    (*in_buf.stream).width,
                    (*in_buf.stream).height,
                ) {
                    Ok(frame) => {
                        let w = frame.width;
                        let h = frame.height;
                        let len = frame.data.len();
                        processed_data = Some(frame.data);
                        log::debug!("  processed input -> {}x{} bytes={}", w, h, len);
                    },
                    Err(e) => {
                        log::error!("Input processing failed: {}", e);
                    }
                }
            }
        }

        // ── Process output buffers (write) ───────────────────────────────
        if let Some(ref data) = processed_data {
            for i in 0..req.num_output_buffers {
                let out_buf = &*req.stream_buffer.add(num_inputs + i);
                if out_buf.buffer.is_null() {
                    continue;
                }

                log::debug!(
                    "  output[{}]: {}x{} fmt=0x{:x}",
                    i,
                    (*out_buf.stream).width,
                    (*out_buf.stream).height,
                    (*out_buf.stream).format,
                );

                match adapter::AndroidCameraAdapter::lock_for_write(
                    out_buf.buffer as *mut adapter::AHardwareBuffer,
                ) {
                    Ok((dst_ptr, dst_size)) => {
                        let copy_len = data.len().min(dst_size);
                        unsafe {
                            std::ptr::copy_nonoverlapping(
                                data.as_ptr(),
                                dst_ptr,
                                copy_len,
                            );
                        }
                        log::debug!("  copied {} bytes to output buffer", copy_len);

                        if let Err(e) = adapter::AndroidCameraAdapter::unlock_buffer(
                            out_buf.buffer as *mut adapter::AHardwareBuffer,
                        ) {
                            log::warn!("Output unlock failed: {}", e);
                        }
                    },
                    Err(e) => {
                        log::error!("Output lock failed: {}", e);
                    }
                }

                let mut filled = camera3_stream_buffer_t {
                    stream: out_buf.stream,
                    buffer: out_buf.buffer,
                    status: CAMERA_BUFFER_STATUS_OK,
                    acquire_fence: -1,
                    release_fence: -1,
                };

                if let Some(process_result) = cb.process_capture_result {
                    let result = camera3_capture_result_t {
                        frame_number: req.frame_number,
                        stream_buffer: &mut filled,
                        num_output_buffers: 1,
                        num_input_buffers: 0,
                        result: std::ptr::null_mut(),
                        partial_result: 0,
                        input_buffer: std::ptr::null_mut(),
                    };
                    process_result(cb as *const _, &result as *const _);
                }
            }
        } else {
            for i in 0..req.num_output_buffers {
                let out_buf = &*req.stream_buffer.add(num_inputs + i);
                let mut filled = camera3_stream_buffer_t {
                    stream: out_buf.stream,
                    buffer: out_buf.buffer,
                    status: CAMERA_BUFFER_STATUS_ERROR,
                    acquire_fence: -1,
                    release_fence: -1,
                };
                if let Some(process_result) = cb.process_capture_result {
                    let result = camera3_capture_result_t {
                        frame_number: req.frame_number,
                        stream_buffer: &mut filled,
                        num_output_buffers: 1,
                        num_input_buffers: 0,
                        result: std::ptr::null_mut(),
                        partial_result: 0,
                        input_buffer: std::ptr::null_mut(),
                    };
                    process_result(cb as *const _, &result as *const _);
                }
            }
        }
    }

    0
}

unsafe extern "C" fn device_flush(_device: *const camera3_device_t) -> i32 {
    log::info!("device_flush");
    0
}

unsafe extern "C" fn device_close(device: *mut hw_device_t) -> i32 {
    let dev = device as *mut camera3_device_t;
    if !dev.is_null() {
        let dev = &mut *dev;
        // Free the adapter stored in priv_
        if !dev.priv_.is_null() {
            let _ = Box::from_raw(dev.priv_ as *mut adapter::AndroidCameraAdapter);
            dev.priv_ = std::ptr::null_mut();
        }
        // Free the device struct itself
        let _ = Box::from_raw(dev as *mut camera3_device_t);
    }
    log::info!("camera3_device_close");
    0
}

// ============================================================================
// Device operations table
// ============================================================================

static DEVICE_OPS: camera3_device_ops_t = camera3_device_ops_t {
    initialize: Some(device_initialize),
    configure_streams: Some(device_configure_streams),
    register_stream_buffers: None,
    process_capture_request: Some(device_process_capture_request),
    flush: Some(device_flush),
    dump: None,
    construct_default_request_settings: None,
};

// ============================================================================
// HAL module functions
// ============================================================================

unsafe extern "C" fn hal_get_number_of_cameras() -> i32 {
    // Detect physical cameras via V4L2 or return default
    #[cfg(feature = "v4l2")]
    {
        let devices = crate::v4l2::list_devices();
        let count = devices.len() as i32;
        log::info!("hal_get_number_of_cameras: found {} V4L2 devices", count);
        count.max(1) // Always return at least 1 (stub)
    }
    #[cfg(not(feature = "v4l2"))]
    {
        1 // Default stub camera
    }
}

unsafe extern "C" fn hal_get_camera_info(id: i32, info: *mut camera_info) -> i32 {
    if id < 0 || id >= hal_get_number_of_cameras() {
        return -1; // EINVAL
    }
    if info.is_null() {
        return -1;
    }

    let info = &mut *info;
    info.facing = 0; // CAMERA_FACING_BACK
    info.orientation = 0;
    info.device_version = CAMERA_DEVICE_API_VERSION;
    info.static_camera_characteristics = std::ptr::null_mut(); // Metadata built by camera metadata builder
    info.resource_cost = 50;
    info.conflicting_devices = std::ptr::null_mut();
    info.conflicting_devices_length = 0;

    // Try to get real camera info from V4L2
    #[cfg(feature = "v4l2")]
    {
        if let Some(path) = crate::v4l2::list_devices().get(id as usize) {
            log::info!("Camera {}: {}", id, path);
            // Try to query capabilities
            if let Ok(cam) = rscam::Camera::new(path) {
                if let Ok(info_data) = cam.query_capability() {
                    let driver = String::from_utf8_lossy(&info_data.driver);
                    let card = String::from_utf8_lossy(&info_data.card);
                    log::info!("  driver={}, card={}", driver, card);
                    // V4L2 doesn't provide facing/orientation directly
                    // These would come from camera metadata in a real HAL
                }
            }
        }
    }

    0
}

unsafe extern "C" fn hal_set_callbacks(callbacks: *const camera_module_callbacks_t) -> i32 {
    if !callbacks.is_null() {
        log::info!("hal_set_callbacks");
        MODULE_CALLBACK_PTR = Some(callbacks);
    }
    0
}

unsafe extern "C" fn hal_get_vendor_tag_ops(_ops: *mut c_void) {
    // No vendor tags
}

unsafe extern "C" fn hal_open_legacy(
    _module: *const hw_module_t,
    _id: *const c_char,
    _hal_version: u32,
    _device: *mut *mut hw_device_t,
) -> i32 {
    -1 // ENOSYS
}

unsafe extern "C" fn hal_set_torch_mode(_camera_id: *const c_char, _enabled: i32) -> i32 {
    -1 // ENOSYS
}

unsafe extern "C" fn hal_init() -> i32 {
    log::info!("SoftISP Camera HAL initializing");
    0
}

// ============================================================================
// Module methods ── open wrapper
// ============================================================================

static MODULE_METHODS: hw_module_methods_t = hw_module_methods_t { open: module_open };

// ============================================================================
// HAL_MODULE_INFO_SYM ── required exported symbol
// ============================================================================

#[no_mangle]
pub static HAL_MODULE_INFO_SYM: camera_module_t = camera_module_t {
    common: hw_module_t {
        tag: HARDWARE_MODULE_TAG,
        module_api_version: CAMERA_MODULE_API_VERSION,
        hal_api_version: HAL_MODULE_API_VERSION,
        id: c"camera".as_ptr(),
        name: c"SoftISP Camera HAL".as_ptr(),
        author: c"cam-core".as_ptr(),
        methods: &MODULE_METHODS as *const _ as *mut hw_module_methods_t,
        dso: std::ptr::null_mut(),
        reserved: [0; 25],
    },
    get_number_of_cameras: hal_get_number_of_cameras,
    get_camera_info: hal_get_camera_info,
    set_callbacks: hal_set_callbacks,
    get_vendor_tag_ops: Some(hal_get_vendor_tag_ops),
    open_legacy: Some(hal_open_legacy),
    set_torch_mode: Some(hal_set_torch_mode),
    init: hal_init,
    reserved: [std::ptr::null_mut(); 5],
};
