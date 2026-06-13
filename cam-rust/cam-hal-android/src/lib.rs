//! Android Camera HAL v3 Implementation (Pure Rust, no NDK crate)
//!
//! This module implements the standard Android Camera HAL3 interface
//! using raw FFI definitions. It does NOT depend on the `ndk` crate's
//! camera3 bindings (which may be incomplete). Instead, we define the
//! required structures and constants ourselves.
//!
//! The HAL bridges Android's camera service to the SoftISP pipeline
//! via the `ICameraAdapter` trait.

#![allow(non_snake_case)]
#![allow(dead_code)]

use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};
use std::sync::{Arc, Mutex, OnceLock};

use cam_hal::camera::{ICameraAdapter, CameraState, FrameCallback, StreamConfig, ByteFrame};
use cam_types::{FrameFormat, CameraSourceType};

// ============================================================================
// Android HAL Type Definitions (from hardware/*.h and camera3.h)
// ============================================================================

/// hardware_module_t (from hardware/hardware.h)
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

/// hardware_device_t (base for camera3_device_t)
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

/// camera_module_t (the main HAL entry point)
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

/// camera_module_callbacks_t (from camera_common.h)
#[repr(C)]
pub struct camera_module_callbacks_t {
    pub camera_device_status: Option<unsafe extern "C" fn(i32, i32)>,
    pub camera_device_available: Option<unsafe extern "C" fn(i32)>,
    pub camera_device_unavailable: Option<unsafe extern "C" fn(i32)>,
    pub reserved: [*mut c_void; 4],
}

/// camera_info (from camera3.h)
#[repr(C)]
pub struct camera_info {
    pub facing: i32,
    pub orientation: i32,
    pub device_version: u64,
    pub static_camera_characteristics: *mut c_void,
}

/// camera3_stream_t
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

/// camera3_buffer_t
#[repr(C)]
pub struct camera3_buffer_t {
    pub buffer: *mut c_void,
    pub stream: *mut camera3_stream_t,
    pub status: i32,
    pub acquire_fence: i32,
    pub release_fence: i32,
}

/// camera3_capture_request_t
#[repr(C)]
pub struct camera3_capture_request_t {
    pub frame_number: u64,
    pub stream_buffer: *mut camera3_buffer_t,
    pub num_output_buffers: usize,
    pub num_input_buffers: usize,
    pub settings: *mut c_void,
}

/// camera3_callback_ops_t
#[repr(C)]
pub struct camera3_callback_ops_t {
    pub version: i32,
    pub set_preview_wallpaper: Option<unsafe extern "C" fn(*mut c_void, *mut c_void)>,
    pub notify: Option<unsafe extern "C" fn(*mut c_void, u64, u32, *const c_void)>,
    pub post_process: Option<unsafe extern "C" fn(*mut c_void)>,
    pub reserved: [*mut c_void; 3],
    pub priv_: *mut c_void,
}

/// camera3_device_ops
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

/// camera3_device_t (our internal device representation)
#[repr(C)]
pub struct Camera3DeviceStruct {
    pub common: hardware_device_t,
    pub ops: *const camera3_device_ops,
    pub priv_: *mut c_void,
    pub priv_mutex: *mut c_void,
    pub flags: i32,
    pub reserved: [i32; 6],
}

// ============================================================================
// Constants (from Android NDK)
// ============================================================================

const HARDWARE_MODULE_TAG: u16 = 0x42424242; // 'HMLA' reversed? Actually hardware_module_t.tag = 0x42424242 for camera?
const HARDWARE_DEVICE_TAG: u16 = 0x41414141; // 'ADLA'

const CAMERA_MODULE_API_VERSION_2_4: u32 = 0x20024;
const CAMERA_DEVICE_API_VERSION_3_4: u32 = 0x30004;

const CAMERA3_MSG_SHUTTER: u32 = 1;
const CAMERA3_MSG_ERROR: u32 = 2;

const CAMERA_BUFFER_STATUS_OK: i32 = 0;
const CAMERA_BUFFER_STATUS_ERROR: i32 = 1;

const HAL_PIXEL_FORMAT_RAW: i32 = 0x21;
const HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED: i32 = 0x22;
const HAL_PIXEL_FORMAT_BLOB: i32 = 0x2F;

const CAMERA3_STREAM_OUTPUT: i32 = 0;
const CAMERA3_STREAM_INPUT: i32 = 1;

const AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE: u32 = 0x0100;
const AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN: u32 = 0x0200;
const AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN: u32 = 0x0400;
const AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY: u32 = 0x0800;

// ============================================================================
// Global State
// ============================================================================

struct CameraHalState {
    callbacks: Mutex<Option<camera3_callback_ops_t>>,
    devices: Mutex<HashMap<i32, Arc<Mutex<AndroidCameraDevice>>>>,
    num_cameras: i32,
}

impl CameraHalState {
    fn new() -> Self {
        Self {
            callbacks: Mutex::new(None),
            devices: Mutex::new(HashMap::new()),
            num_cameras: 1,
        }
    }
}

static HAL_STATE: OnceLock<CameraHalState> = OnceLock::new();

// ============================================================================
// AndroidCameraDevice Implementation
// ============================================================================

struct AndroidCameraDevice {
    id: i32,
    adapter: Box<dyn ICameraAdapter>,
    state: Mutex<CameraState>,
    streams: Mutex<Vec<camera3_stream_t>>,
    callback_ops: Mutex<Option<camera3_callback_ops_t>>,
}

impl AndroidCameraDevice {
    fn new(id: i32, adapter: Box<dyn ICameraAdapter>) -> Self {
        Self {
            id,
            adapter,
            state: Mutex::new(CameraState::Closed),
            streams: Mutex::new(Vec::new()),
            callback_ops: Mutex::new(None),
        }
    }

    unsafe fn process_request(&self, request: *const camera3_capture_request_t) -> i32 {
        let req = &*request;

        if self.adapter.state() != CameraState::Streaming {
            log::warn!("Process request while not streaming");
            return -1;
        }

        let num_outputs = req.num_output_buffers;
        if num_outputs == 0 {
            return 0;
        }

        let buffers = std::slice::from_raw_parts(req.stream_buffer, num_outputs);

        for buffer in buffers {
            if buffer.status != CAMERA_BUFFER_STATUS_OK {
                continue;
            }
            if buffer.buffer.is_null() {
                continue;
            }

            // TODO: Properly lock AHardwareBuffer and extract data
            // For now, we'll simulate by sending an empty frame
            let frame = ByteFrame {
                data: Vec::new(),
                width: buffer.stream.width,
                height: buffer.stream.height,
                format: FrameFormat::Raw16, // Assume RAW16 for now
                timestamp: 0,
            };
            let _ = self.adapter.send_frame(frame);

            // Notify shutter
            self.notify_shutter(req.frame_number);
        }

        0
    }

    fn notify_shutter(&self, frame_number: u64) {
        if let Some(cb) = self.callback_ops.lock().unwrap().as_ref() {
            if let Some(notify) = cb.notify {
                unsafe {
                    notify(
                        cb.priv_,
                        frame_number,
                        CAMERA3_MSG_SHUTTER,
                        std::ptr::null(),
                    );
                }
            }
        }
    }

    fn notify_error(&self, frame_number: u64, error: i32) {
        if let Some(cb) = self.callback_ops.lock().unwrap().as_ref() {
            if let Some(notify) = cb.notify {
                unsafe {
                    let reason = CString::new("HAL error").unwrap();
                    let mut error_msg = camera3_error_msg_t {
                        frame_number: frame_number as u32,
                        error_stream: std::ptr::null_mut(),
                        error_code: error as u32,
                        reason: reason.into_raw(),
                    };
                    notify(
                        cb.priv_,
                        frame_number,
                        CAMERA3_MSG_ERROR,
                        &mut error_msg as *mut _ as *mut c_void,
                    );
                }
            }
        }
    }
}

// Error message structure (camera3_error_msg_t)
#[repr(C)]
struct camera3_error_msg_t {
    frame_number: u32,
    error_stream: *mut c_void,
    error_code: u32,
    reason: *mut c_char,
}

// ============================================================================
// Module Functions (C ABI)
// ============================================================================

unsafe extern "C" fn hal_get_number_of_cameras() -> i32 {
    HAL_STATE.get_or_init(CameraHalState::new).num_cameras
}

unsafe extern "C" fn hal_get_camera_info(id: i32, info: *mut camera_info) -> i32 {
    if id < 0 || id >= HAL_STATE.get().unwrap().num_cameras {
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

unsafe extern "C" fn hal_set_callbacks(callbacks: *const camera_module_callbacks_t) -> i32 {
    if !callbacks.is_null() {
        let callbacks = &*callbacks;
        let state = HAL_STATE.get().unwrap();
        let mut cb_lock = state.callbacks.lock().unwrap();
        *cb_lock = Some(callbacks.callbacks);
        log::info!("Camera service callbacks registered");
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
    HAL_STATE.get_or_init(CameraHalState::new);
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

    let state = HAL_STATE.get().unwrap();
    let mut devices = state.devices.lock().unwrap();

    if devices.contains_key(&id) {
        log::warn!("Camera {} already opened", id);
        return -1;
    }

    // Create adapter (TODO: use real camera adapter instead of stub)
    let adapter = Box::new(StubCameraAdapter::new());
    let camera_device = AndroidCameraDevice::new(id, adapter);
    let device_arc = Arc::new(Mutex::new(camera_device));

    devices.insert(id, device_arc.clone());

    // Allocate device structure on heap
    let device_ptr = Box::into_raw(Box::new(device_arc)) as *mut c_void;

    // Build Camera3DeviceStruct
    let mut struct_box = Box::new(Camera3DeviceStruct {
        common: hardware_device_t {
            tag: HARDWARE_DEVICE_TAG,
            version_major: 3,
            version_minor: 4,
            id: [0; 32],
            name: [0; 64],
            author: [0; 64],
            dso: std::ptr::null_mut(),
            close: Some(camera3_device_close),
            reserved: [std::ptr::null_mut(); 10],
        },
        ops: &DEVICE_OPS,
        priv_: device_ptr,
        priv_mutex: std::ptr::null_mut(),
        flags: 0,
        reserved: [0; 6],
    });

    *device = &mut *struct_box as *mut _ as *mut c_void;
    std::mem::forget(struct_box);

    log::info!("Camera {} opened successfully", id);
    0
}

unsafe extern "C" fn camera3_device_close(device: *mut c_void) -> i32 {
    if device.is_null() {
        return -1;
    }

    let dev_struct = device as *mut Camera3DeviceStruct;
    let device_arc_ptr = (*dev_struct).priv_ as *mut Arc<Mutex<AndroidCameraDevice>>;

    // Drop the device
    if !device_arc_ptr.is_null() {
        let _ = Box::from_raw(device_arc_ptr);
    }

    let _ = Box::from_raw(dev_struct);

    log::info!("Camera device closed");
    0
}

// Device ops
struct DeviceOps;

impl DeviceOps {
    const OPS: camera3_device_ops = camera3_device_ops {
        initialize: Some(camera3_device_initialize),
        configure_streams: Some(camera3_device_configure_streams),
        register_stream_buffers: None,
        process_capture_request: Some(camera3_device_process_capture_request),
        get_metadata: Some(camera3_device_get_metadata),
        dump: None,
        close: camera3_device_close,
    };
}

unsafe extern "C" fn camera3_device_initialize(_device: *mut c_void) {
    log::debug!("camera3_device_initialize");
}

unsafe extern "C" fn camera3_device_configure_streams(
    device: *mut c_void,
    streams: *mut camera3_stream_t,
    num_streams: usize,
) -> i32 {
    let dev_struct = device as *mut Camera3DeviceStruct;
    let device_arc = &*(*dev_struct).priv_ as *const Arc<Mutex<AndroidCameraDevice>>;
    let device = (*device_arc).lock().unwrap();

    let stream_slice = std::slice::from_raw_parts_mut(streams, num_streams);

    log::info!("configure_streams: {} streams", num_streams);

    let mut stored = device.streams.lock().unwrap();
    stored.clear();

    for stream in stream_slice.iter_mut() {
        // Set usage flags based on format
        match stream.format {
            HAL_PIXEL_FORMAT_RAW => {
                stream.usage |= AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN;
                stream.usage |= AHARDWAREBUFFER_USAGE_CPU_WRITE_OFTEN;
            }
            HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED => {
                stream.usage |= AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE;
                stream.usage |= AHARDWAREBUFFER_USAGE_COMPOSER_OVERLAY;
            }
            HAL_PIXEL_FORMAT_BLOB => {
                stream.usage |= AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN;
            }
            _ => {}
        }

        stored.push(camera3_stream_t {
            width: stream.width,
            height: stream.height,
            format: stream.format,
            stream_type: stream.stream_type,
            usage: stream.usage,
            max_buffers: stream.max_buffers,
            private: std::ptr::null_mut(),
        });
    }

    0
}

unsafe extern "C" fn camera3_device_process_capture_request(
    device: *mut c_void,
    request: *const camera3_capture_request_t,
) -> i32 {
    let dev_struct = device as *mut Camera3DeviceStruct;
    let device_arc = (*dev_struct).priv_ as *const Arc<Mutex<AndroidCameraDevice>>;
    let device_clone = (*device_arc).clone();

    std::thread::spawn(move || {
        let device = device_clone.lock().unwrap();
        let result = device.process_request(request);
        if result != 0 {
            log::error!("Capture request failed: {}", result);
        }
    });

    0
}

unsafe extern "C" fn camera3_device_get_metadata(
    _device: *mut c_void,
    _type: i32,
    metadata: *mut *mut c_void,
) -> i32 {
    *metadata = std::ptr::null_mut();
    0
}

// ============================================================================
// Module Definitions
// ============================================================================

static DEVICE_OPS: camera3_device_ops = DeviceOps::OPS;

#[no_mangle]
pub static HAL_MODULE_INFO_SYM: camera_module_t = camera_module_t {
    common: hardware_module_t {
        tag: 0x42424242,
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

#[no_mangle]
#[link(symbol = "HAL_MODULE_INFO_SYM")]
pub static __HAL_MODULE_INFO_SYM: camera_module_t = HAL_MODULE_INFO_SYM;

#[no_mangle]
pub unsafe extern "C" fn camera_module_init(_module: *mut hardware_module_t) -> i32 {
    hal_init()
}

// ============================================================================
// Stub Adapter (for testing)
// ============================================================================

struct StubCameraAdapter;

impl StubCameraAdapter {
    fn new() -> Box<dyn ICameraAdapter> {
        Box::new(StubCameraAdapter)
    }
}

impl ICameraAdapter for StubCameraAdapter {
    fn source_type(&self) -> CameraSourceType {
        CameraSourceType::RawCamera2
    }

    fn open(&mut self, _config: &StreamConfig) -> Result<(), String> {
        log::info!("StubAdapter::open");
        Ok(())
    }

    fn close(&mut self) {
        log::info!("StubAdapter::close");
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        log::info!("StubAdapter::start_streaming");
        Ok(())
    }

    fn stop_streaming(&mut self) {
        log::info!("StubAdapter::stop_streaming");
    }

    fn set_frame_callback(&mut self, _callback: FrameCallback) {
        log::info!("StubAdapter::set_frame_callback");
    }

    fn state(&self) -> CameraState {
        CameraState::Open
    }

    fn device_name(&self) -> &str {
        "stub_camera"
    }

    fn send_frame(&self, frame: ByteFrame) -> Result<(), String> {
        log::debug!("StubAdapter received frame: {}×{}", frame.width, frame.height);
        Ok(())
    }
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
        let stream = *cfg.streams.offset(i as isize);
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
    let frame_num = req.frame_number;
    let total_start = std::time::Instant::now();
    log::trace!("process_capture_request: frame {}", frame_num);

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

    let total_elapsed = total_start.elapsed();
    log::trace!("process_capture_request: frame {} total: {:.2}ms",
        frame_num, total_elapsed.as_secs_f64() * 1000.0);

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
    // TODO: detect physical cameras (e.g. via libcamera or V4L2)
    1
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
    info.static_camera_characteristics = std::ptr::null_mut(); // TODO: build metadata
    info.resource_cost = 50;
    info.conflicting_devices = std::ptr::null_mut();
    info.conflicting_devices_length = 0;

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
