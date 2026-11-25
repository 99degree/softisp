//! libcamera-ipa: libcamera C FFI plugin for softisp
//!
//! Implements libcamera's IPAInterface and IPASoftIspInterface,
//! bridging to the cam-ipa stable Rust API.

use std::os::raw::{c_char, c_int, c_uint, c_void};
use std::ffi::CStr;
use cam_ipa::{IpaConfig, IpaContext, IpaResult, IpaError};

// --- C-compatible structs matching libcamera headers ---

#[repr(C)]
pub struct IPAModuleInfo {
    pub module_api_version: c_int,
    pub pipeline_version: c_uint,
    pub pipeline_name: [c_char; 256],
    pub name: [c_char; 256],
}

#[repr(C)]
pub struct IPASettings {
    pub configuration_file: *const c_char,
    pub sensor_model: *const c_char,
}

#[repr(C)]
pub struct Size {
    pub width: c_uint,
    pub height: c_uint,
}

#[repr(C)]
pub struct Rectangle {
    pub x: c_int,
    pub y: c_int,
    pub width: c_uint,
    pub height: c_uint,
}

#[repr(C)]
pub struct IPACameraSensorInfo {
    pub model: *const c_char,
    pub bits_per_pixel: c_uint,
    pub cfa_pattern: c_uint,
    pub active_area_size: Size,
    pub analog_crop: Rectangle,
    pub output_size: Size,
    pub pixel_rate: u64,
    pub min_line_length: c_uint,
    pub max_line_length: c_uint,
    pub min_frame_length: c_uint,
    pub max_frame_length: c_uint,
}

#[repr(C)]
pub struct IPAConfigInfo {
    pub sensor_controls: *const c_void, // Opaque ControlInfoMap
}

#[repr(C)]
pub struct IPAInterface {
    pub _opaque: [u8; 0],
}

// --- VTable for IPASoftIspInterface ---
#[repr(C)]
pub struct IPASoftIspVTable {
    pub init: unsafe extern "C" fn(
        handle: *mut c_void,
        settings: *const IPASettings,
        fd_stats: c_int,
        fd_params: c_int,
        sensor_info: *const IPACameraSensorInfo,
        sensor_controls: *const c_void,
        ipa_controls: *mut c_void,
        ccm_enabled: *mut bool,
    ) -> c_int,
    pub start: unsafe extern "C" fn(handle: *mut c_void) -> c_int,
    pub stop: unsafe extern "C" fn(handle: *mut c_void),
    pub configure: unsafe extern "C" fn(handle: *mut c_void, config: *const IPAConfigInfo) -> c_int,
    pub configure_frame_backend: unsafe extern "C" fn(
        handle: *mut c_void,
        format: c_int,
        num_output_buffers: c_uint,
    ) -> c_int,
    pub queue_request: unsafe extern "C" fn(
        handle: *mut c_void,
        frame: c_uint,
        controls: *const c_void,
    ),
    pub compute_params: unsafe extern "C" fn(handle: *mut c_void, frame: c_uint),
    pub process_stats: unsafe extern "C" fn(
        handle: *mut c_void,
        frame: c_uint,
        buffer_id: c_uint,
        stats: *mut c_void,
    ),
    pub process_frame: unsafe extern "C" fn(
        handle: *mut c_void,
        frame: c_uint,
        buffer_id: c_uint,
        buffer_fd: c_int,
        plane_index_in: c_int,
        plane_index_out: c_int,
        width: c_int,
        height: c_int,
        results: *const c_void,
    ),
}

// --- Exported module info ---

use std::sync::OnceLock;

static IPA_MODULE_INFO: OnceLock<IPAModuleInfo> = OnceLock::new();

fn ipa_module_info() -> &'static IPAModuleInfo {
    IPA_MODULE_INFO.get_or_init(|| IPAModuleInfo {
        module_api_version: 1,
        pipeline_version: 0,
        pipeline_name: {
            let mut arr = [0u8; 256];
            let b = b"softisp";
            arr[..b.len()].copy_from_slice(b);
            arr
        },
        name: {
            let mut arr = [0u8; 256];
            let b = b"SoftISP";
            arr[..b.len()].copy_from_slice(b);
            arr
        },
    })
}

// --- Exported factory function ---

#[no_mangle]
pub extern "C" fn ipa_create() -> *mut IPAInterface {
    // Create an IpaContext and return as opaque pointer
    let config = IpaConfig::default();
    match IpaContext::new(config) {
        Ok(ctx) => Box::into_raw(Box::new(ctx)) as *mut IPAInterface,
        Err(e) => {
            log::error!("Failed to create IPA context: {}", e);
            std::ptr::null_mut()
        }
    }
}

// --- VTable implementation ---

static IPA_VTABLE: IPASoftIspVTable = IPASoftIspVTable {
    init: ipa_init,
    start: ipa_start,
    stop: ipa_stop,
    configure: ipa_configure,
    configure_frame_backend: ipa_configure_frame_backend,
    queue_request: ipa_queue_request,
    compute_params: ipa_compute_params,
    process_stats: ipa_process_stats,
    process_frame: ipa_process_frame,
};

#[repr(C)]
pub struct IPASoftIspInstance {
    pub vtable: &'static IPASoftIspVTable,
    pub handle: *mut c_void, // *mut IpaContext
}

unsafe extern "C" fn ipa_init(
    handle: *mut c_void,
    settings: *const IPASettings,
    fd_stats: c_int,
    fd_params: c_int,
    sensor_info: *const IPACameraSensorInfo,
    sensor_controls: *const c_void,
    ipa_controls: *mut c_void,
    ccm_enabled: *mut bool,
) -> c_int {
    if handle.is_null() {
        return -1;
    }
    
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    
    // Convert C types to Rust types
    let settings = convert_settings(settings);
    let sensor_info = convert_sensor_info(sensor_info);
    
    match ctx.init(&settings, &sensor_info) {
        Ok(init_result) => {
            // Copy ccm_enabled back to caller
            if !ccm_enabled.is_null() {
                unsafe { *ccm_enabled = init_result.ccm_enabled; }
            }
            // Copy ipa_controls back - simplified for MVP
            let _ = ipa_controls; // Would need proper serialization
            0
        }
        Err(e) => {
            log::error!("IPA init failed: {}", e);
            -1
        }
    }
}

unsafe extern "C" fn ipa_start(handle: *mut c_void) -> c_int {
    if handle.is_null() {
        return -1;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    match ctx.start() {
        Ok(()) => 0,
        Err(e) => {
            log::error!("IPA start failed: {}", e);
            -1
        }
    }
}

unsafe extern "C" fn ipa_stop(handle: *mut c_void) {
    if !handle.is_null() {
        let ctx = unsafe { &mut *(handle as *mut IpaContext) };
        ctx.stop();
    }
}

unsafe extern "C" fn ipa_configure(
    handle: *mut c_void,
    config: *const IPAConfigInfo,
) -> c_int {
    if handle.is_null() || config.is_null() {
        return -1;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    let config = &*(config as *const IPAConfigInfo);
    
    // Convert sensor controls from opaque pointer
    let sensor_controls = convert_control_info_map(config.sensor_controls);
    
    let ipa_config = cam_ipa::IpaConfigureInfo { sensor_controls };
    match ctx.configure(&ipa_config) {
        Ok(()) => 0,
        Err(e) => {
            log::error!("IPA configure failed: {}", e);
            -1
        }
    }
}

unsafe extern "C" fn ipa_configure_frame_backend(
    handle: *mut c_void,
    format: c_int,
    num_output_buffers: c_uint,
) -> c_int {
    if handle.is_null() {
        return -1;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    
    let ipa_format = match format {
        0 => cam_ipa::IpaFormat::RGB,
        1 => cam_ipa::IpaFormat::YUV,
        _ => cam_ipa::IpaFormat::BGRA,
    };
    
    match ctx.configure_frame_backend(ipa_format, num_output_buffers) {
        Ok(()) => 0,
        Err(e) => {
            log::error!("IPA configure_frame_backend failed: {}", e);
            -1
        }
    }
}

unsafe extern "C" fn ipa_queue_request(
    handle: *mut c_void,
    frame: c_uint,
    controls: *const c_void,
) {
    if handle.is_null() {
        return;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    
    let ctrl_list = if controls.is_null() {
        cam_ipa::ControlList::new()
    } else {
        convert_control_list(controls)
    };
    
    let _ = ctx.queue_request(frame, &ctrl_list);
}

unsafe extern "C" fn ipa_compute_params(handle: *mut c_void, frame: c_uint) {
    if handle.is_null() {
        return;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    let _ = ctx.compute_params(frame);
}

unsafe extern "C" fn ipa_process_stats(
    handle: *mut c_void,
    frame: c_uint,
    buffer_id: c_uint,
    stats: *mut c_void,
) {
    if handle.is_null() {
        return;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    
    // For MVP, stats buffer is passed as opaque - we return default metadata
    let _ = buffer_id;
    let _ = stats;
    
    // Create empty buffer for stats processing
    // In real implementation, we'd map the SharedFD buffer here
    let empty_buf: &dyn cam_ipa::Buffer = &cam_ipa::MemBuffer::new(0);
    let _ = ctx.process_stats(frame, buffer_id, empty_buf);
}

unsafe extern "C" fn ipa_process_frame(
    handle: *mut c_void,
    frame: c_uint,
    buffer_id: c_uint,
    buffer_fd: c_int,
    plane_index_in: c_int,
    plane_index_out: c_int,
    width: c_int,
    height: c_int,
    results: *const c_void,
) {
    if handle.is_null() {
        return;
    }
    let ctx = unsafe { &mut *(handle as *mut IpaContext) };
    
    let _ = buffer_id;
    let _ = plane_index_in;
    let _ = plane_index_out;
    
    // Wrap dmabuf fd as Buffer
    let mut input_buf = match cam_ipa::buffer::DmaBufBuffer::new(buffer_fd, (width * height * 2) as usize) {
        Ok(buf) => buf,
        Err(e) => {
            log::error!("Failed to create DmaBufBuffer: {}", e);
            return;
        }
    };

    let mut output_buf = cam_ipa::buffer::MemBuffer::new((width * height * 4) as usize);

    let ctrl_list = if results.is_null() {
        cam_ipa::ControlList::new()
    } else {
        convert_control_list(results)
    };

    let width = width as u32;
    let height = height as u32;

    if let Err(e) = ctx.process_frame(
        frame, buffer_id,
        &mut input_buf, &mut output_buf,
        0, 0, // plane indices
        width, height,
        &ctrl_list,
    ) {
        log::error!("IPA process_frame failed: {}", e);
    }
}

// --- Helper conversion functions ---

fn convert_settings(settings: *const IPASettings) -> cam_ipa::IpaSettings {
    if settings.is_null() {
        return cam_ipa::IpaSettings {
            config_file: None,
            sensor_model: "unknown".to_string(),
        };
    }
    
    let config_file = unsafe {
        let ptr = (*settings).configuration_file;
        if ptr.is_null() {
            None
        } else {
            CStr::from_ptr(ptr).to_string_lossy().to_string().into()
        }
    };
    
    let sensor_model = unsafe {
        CStr::from_ptr((*settings).sensor_model).to_string_lossy().to_string()
    };
    
    cam_ipa::IpaSettings { config_file, sensor_model }
}

fn convert_sensor_info(info: *const IPACameraSensorInfo) -> cam_ipa::IpaSensorInfo {
    if info.is_null() {
        return cam_ipa::IpaSensorInfo {
            model: "unknown".to_string(),
            bits_per_pixel: 10,
            cfa_pattern: 0, // RGGB
            active_area_size: cam_ipa::Size { width: 1920, height: 1080 },
            analog_crop: cam_ipa::Rectangle { x: 0, y: 0, width: 1920, height: 1080 },
            output_size: cam_ipa::Size { width: 1920, height: 1080 },
            pixel_rate: 0,
            min_line_length: 0,
            max_line_length: 0,
            min_frame_length: 0,
            max_frame_length: 0,
        };
    }
    
    unsafe {
        cam_ipa::IpaSensorInfo {
            model: CStr::from_ptr((*info).model).to_string_lossy().to_string(),
            bits_per_pixel: (*info).bits_per_pixel,
            cfa_pattern: (*info).cfa_pattern,
            active_area_size: cam_ipa::Size {
                width: (*info).active_area_size.width,
                height: (*info).active_area_size.height,
            },
            analog_crop: cam_ipa::Rectangle {
                x: (*info).analog_crop.x,
                y: (*info).analog_crop.y,
                width: (*info).analog_crop.width,
                height: (*info).analog_crop.height,
            },
            output_size: cam_ipa::Size {
                width: (*info).output_size.width,
                height: (*info).output_size.height,
            },
            pixel_rate: (*info).pixel_rate,
            min_line_length: (*info).min_line_length,
            max_line_length: (*info).max_line_length,
            min_frame_length: (*info).min_frame_length,
            max_frame_length: (*info).max_frame_length,
        }
    }
}

fn convert_control_info_map(_ptr: *const c_void) -> cam_ipa::ControlInfoMap {
    // MVP: opaque pointer, return empty map
    // Full implementation would deserialize from shared memory or use libcamera bindings
    cam_ipa::ControlInfoMap::default()
}

fn convert_control_list(_ptr: *const c_void) -> cam_ipa::ControlList {
    // MVP: opaque pointer, return empty list
    cam_ipa::ControlList::new()
}