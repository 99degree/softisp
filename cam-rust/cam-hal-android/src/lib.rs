//! Minimal Android Camera HAL v3 implementation (stub).
//!
//! Provides the required `HAL_MODULE_INFO_SYM` symbol and basic device
//! operations. This is a starting point for full Android camera support.
//!
//! Build: cargo build --release -p cam-hal-android (for Android target)

#![allow(non_snake_case)]
#![allow(dead_code)]

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};
use std::sync::OnceLock;

// ============================================================================
// Type definitions from Android camera3.h (simplified)
// ============================================================================

#[repr(C)]
pub struct hardware_module_t {
    pub tag: u16,
    pub version_major: u16,
    pub version_minor: u16,
    pub id: [u16; 32],
    pub name: [u8; 64],
    pub author: [u8; 64],
    pub dso: *mut c_void,
    pub reserved: [*mut c_void; 3],
}
// SAFETY: All fields are raw pointers or integers; immutable after init.
// SAFETY: All fields are raw pointers or POD integers. Safe to send and share across threads.
unsafe impl Send for hardware_module_t {}
unsafe impl Sync for hardware_module_t {}

#[repr(C)]
pub struct hardware_device_t {
    pub tag: u16,
    pub version_major: u16,
    pub version_minor: u16,
    pub id: [u16; 32],
    pub name: [u8; 64],
    pub author: [u8; 64],
    pub dso: *mut c_void,
    pub close: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    pub reserved: [*mut c_void; 10],
}
unsafe impl Sync for hardware_device_t {}

#[repr(C)]
pub struct camera_module_t {
    pub common: hardware_module_t,
    pub get_number_of_cameras: unsafe extern "C" fn() -> i32,
    pub get_camera_info: unsafe extern "C" fn(i32, *mut camera_info) -> i32,
    pub set_callbacks: unsafe extern "C" fn(*const camera_module_callbacks_t) -> i32,
    pub get_vendor_tag_ops: Option<unsafe extern "C" fn(*mut *const c_void) -> i32>,
    pub open_legacy: Option<unsafe extern "C" fn(*const hardware_module_t, *const c_char, u32, *mut *mut c_void) -> i32>,
    pub set_torch_mode: Option<unsafe extern "C" fn(*const c_char, bool) -> i32>,
    pub init: unsafe extern "C" fn() -> i32,
    pub reserved: [*mut c_void; 7],
}
unsafe impl Send for camera_module_t {}
unsafe impl Sync for camera_module_t {}

#[repr(C)]
pub struct camera_info {
    pub facing: i32,
    pub orientation: i32,
    pub device_version: u64,
    pub static_camera_characteristics: *mut c_void,
}

#[repr(C)]
pub struct camera_module_callbacks_t {
    pub camera_device_status: Option<unsafe extern "C" fn(i32, i32)>,
    pub camera_device_available: Option<unsafe extern "C" fn(i32)>,
    pub camera_device_unavailable: Option<unsafe extern "C" fn(i32)>,
    pub reserved: [*mut c_void; 4],
}

// Camera3 device structures
#[repr(C)]
pub struct camera3_stream_t {
    pub width: u32,
    pub height: u32,
    pub format: i32,
    pub stream_type: i32,
    pub usage: u32,
    pub max_buffers: u32,
    pub private: *mut c_void,
}

#[repr(C)]
pub struct camera3_buffer_t {
    pub buffer: *mut c_void,
    pub stream: *mut camera3_stream_t,
    pub status: i32,
    pub acquire_fence: i32,
    pub release_fence: i32,
}

#[repr(C)]
pub struct camera3_capture_request_t {
    pub frame_number: u64,
    pub stream_buffer: *mut camera3_buffer_t,
    pub num_output_buffers: usize,
    pub num_input_buffers: usize,
    pub settings: *mut c_void,
}

#[repr(C)]
pub struct camera3_callback_ops_t {
    pub version: i32,
    pub notify: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *const c_void)>,
    pub post_process: Option<unsafe extern "C" fn(*mut c_void)>,
    pub reserved: [*mut c_void; 3],
    pub priv_: *mut c_void,
}
// SAFETY: This struct is just a bag of raw pointers and integers.
unsafe impl Send for camera3_callback_ops_t {}
unsafe impl Sync for camera3_callback_ops_t {}

#[repr(C)]
pub struct camera3_device_ops {
    pub initialize: Option<unsafe extern "C" fn(*mut c_void)>,
    pub configure_streams: Option<unsafe extern "C" fn(*mut c_void, *mut camera3_stream_t, usize) -> i32>,
    pub register_stream_buffers: Option<unsafe extern "C" fn(*mut c_void, *mut camera3_buffer_t, usize) -> i32>,
    pub process_capture_request: Option<unsafe extern "C" fn(*mut c_void, *const camera3_capture_request_t) -> i32>,
    pub get_metadata: Option<unsafe extern "C" fn(*mut c_void, i32, *mut *mut c_void) -> i32>,
    pub dump: Option<unsafe extern "C" fn(*mut c_void, i32, *const c_char) -> i32>,
    pub close: unsafe extern "C" fn(*mut c_void) -> i32,
}

#[repr(C)]
pub struct Camera3DeviceStruct {
    pub common: hardware_device_t,
    pub ops: *const camera3_device_ops,
    pub priv_: *mut c_void,
    pub priv_mutex: *mut c_void,
    pub flags: i32,
    pub reserved: [i32; 6],
}
unsafe impl Send for Camera3DeviceStruct {}
unsafe impl Sync for Camera3DeviceStruct {}

// ============================================================================
// Constants
// ============================================================================



const CAMERA3_MSG_SHUTTER: u32 = 1;
const CAMERA3_MSG_ERROR: u32 = 2;

const CAMERA_BUFFER_STATUS_OK: i32 = 0;

const HAL_PIXEL_FORMAT_RAW: i32 = 0x21;
const HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED: i32 = 0x22;
const HAL_PIXEL_FORMAT_BLOB: i32 = 0x2F;

// ============================================================================
// Global callback storage (OnceLock for thread-safe one-time init)
// ============================================================================

static CAMERA_CALLBACKS: OnceLock<camera3_callback_ops_t> = OnceLock::new();

// ============================================================================
// Device operations
// ============================================================================

unsafe extern "C" fn device_initialize(_device: *mut c_void) {
    log::debug!("camera3_device_initialize");
}

unsafe extern "C" fn device_configure_streams(
    _device: *mut c_void,
    _streams: *mut camera3_stream_t,
    _num_streams: usize,
) -> i32 {
    log::info!("configure_streams called");
    0
}

unsafe extern "C" fn device_process_capture_request(
    device: *mut c_void,
    request: *const camera3_capture_request_t,
) -> i32 {
    let req = &*request;
    log::debug!("process_capture_request: frame {}", req.frame_number);

    // Notify shutter using stored callbacks
    if let Some(cb) = CAMERA_CALLBACKS.get() {
        if let Some(notify) = cb.notify {
            unsafe {
                notify(
                    cb.priv_,
                    req.frame_number,
                    CAMERA3_MSG_SHUTTER,
                    std::ptr::null(),
                );
            }
        }
    }

    0
}

unsafe extern "C" fn device_get_metadata(
    _device: *mut c_void,
    _type: i32,
    metadata: *mut *mut c_void,
) -> i32 {
    *metadata = std::ptr::null_mut();
    0
}

unsafe extern "C" fn device_close(device: *mut c_void) -> i32 {
    let dev = device as *mut Camera3DeviceStruct;
    // Free the device structure (which also frees its priv_ pointer)
    let _ = Box::from_raw(dev);
    log::info!("camera3_device_close");
    0
}

static DEVICE_OPS: camera3_device_ops = camera3_device_ops {
    initialize: Some(device_initialize),
    configure_streams: Some(device_configure_streams),
    register_stream_buffers: None,
    process_capture_request: Some(device_process_capture_request),
    get_metadata: Some(device_get_metadata),
    dump: None,
    close: device_close,
};

// ============================================================================
// HAL module functions
// ============================================================================

unsafe extern "C" fn hal_get_number_of_cameras() -> i32 {
    // TODO: query actual camera count from libcamera or system
    1
}

unsafe extern "C" fn hal_get_camera_info(id: i32, info: *mut camera_info) -> i32 {
    if id < 0 || id >= hal_get_number_of_cameras() {
        return -1;
    }

    if !info.is_null() {
        let info = &mut *info;
        info.facing = 0; // CAMERA_FACING_BACK
        info.orientation = 0;
        info.device_version = 0;
        info.static_camera_characteristics = std::ptr::null_mut();
    }

    0
}

extern "C" fn dummy_notify(_priv: *mut c_void, _frame: u64, _msg: u32, _data: *const c_void) {
    // No-op
}

unsafe extern "C" fn hal_set_callbacks(callbacks: *const camera_module_callbacks_t) -> i32 {
    if !callbacks.is_null() {
        log::info!("hal_set_callbacks received (stub)");
        // Install a dummy callback_ops_t so process_capture_request can call notify safely
        let ops = camera3_callback_ops_t {
            version: 1,
            notify: Some(dummy_notify),
            post_process: None,
            reserved: [std::ptr::null_mut(); 3],
            priv_: std::ptr::null_mut(),
        };
        if CAMERA_CALLBACKS.set(ops).is_err() {
            log::warn!("Callbacks already set");
            return -1; // EBUSY
        }
    }
    0
}

unsafe extern "C" fn hal_open_legacy(
    _module: *const hardware_module_t,
    _id: *const c_char,
    _hal_version: u32,
    _device: *mut *mut c_void,
) -> i32 {
    -1 // ENOSYS
}

unsafe extern "C" fn hal_set_torch_mode(_camera_id: *const c_char, _enabled: bool) -> i32 {
    -1 // ENOSYS
}

unsafe extern "C" fn hal_init() -> i32 {
    log::info!("SoftISP Camera HAL v3 initializing");
    // Initialize any global state if needed
    0
}

unsafe extern "C" fn hal_dev_open(
    _module: *const hardware_module_t,
    name: *const c_char,
    device: *mut *mut c_void,
) -> i32 {
    let id_str = CStr::from_ptr(name).to_str().unwrap_or("0");
    let id: i32 = id_str.parse().unwrap_or(0);

    log::info!("Opening camera device: id={}", id);

    // Allocate a dummy private structure (will be retrieved in close)
    struct DummyPriv;
    let priv_ptr = Box::into_raw(Box::new(DummyPriv)) as *mut c_void;

    // Build Camera3DeviceStruct
    let mut dev = Box::new(Camera3DeviceStruct {
        common: hardware_device_t {
            tag: 0x4141, // HARDWARE_DEVICE_TAG
            version_major: 3,
            version_minor: 4,
            id: [0; 32],
            name: [0; 64],
            author: [0; 64],
            dso: std::ptr::null_mut(),
            close: Some(device_close),
            reserved: [std::ptr::null_mut(); 10],
        },
        ops: &DEVICE_OPS,
        priv_: priv_ptr,
        priv_mutex: std::ptr::null_mut(),
        flags: 0,
        reserved: [0; 6],
    });

    *device = &mut *dev as *mut _ as *mut c_void;
    std::mem::forget(dev);

    0
}

// ============================================================================
// Module symbol
// ============================================================================

#[no_mangle]
pub static HAL_MODULE_INFO_SYM: camera_module_t = camera_module_t {
    common: hardware_module_t {
        tag: 0x4242, // HARDWARE_MODULE_TAG (small)
        version_major: 1,
        version_minor: 0,
        id: [0; 32],
        name: [0; 64],
        author: [0; 64],
        dso: std::ptr::null_mut(),
        reserved: [std::ptr::null_mut(); 3],
    },
    get_number_of_cameras: hal_get_number_of_cameras,
    get_camera_info: hal_get_camera_info,
    set_callbacks: hal_set_callbacks,
    get_vendor_tag_ops: None,
    open_legacy: Some(hal_open_legacy),
    set_torch_mode: Some(hal_set_torch_mode),
    init: hal_init,
    reserved: [std::ptr::null_mut(); 7],
};
