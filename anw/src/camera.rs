//! NDK camera2 capture via raw `ndk-sys` FFI.
//!
//! The capture path is deliberately RAW16-first: the ISP pipeline consumes
//! Bayer `u16` frames, and `AIMAGE_FORMAT_RAW16` is the only NDK image format
//! that delivers exactly that. A `YUV_420_888` fallback exists for sensors that
//! do not advertise the RAW capability; there the luma plane is expanded into a
//! `GG` mosaic so the pipeline still receives Bayer-shaped input of the same
//! geometry.

use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr::null_mut;
use std::sync::atomic::{AtomicBool, Ordering};

use ndk_sys as sys;

/// Set by the framework camera callbacks and consumed by the app loop.
///
/// The callbacks run on binder threads, so they cannot close the device or
/// touch `CameraSession`; they only raise this flag. `ERROR_CAMERA_DEVICE`
/// (4) is fatal — the device must be closed and reopened or capture stays dead.
static FATAL_CAMERA_ERROR: AtomicBool = AtomicBool::new(false);

/// Consume a pending fatal camera callback.
pub fn take_fatal_camera_error() -> bool {
    FATAL_CAMERA_ERROR.swap(false, Ordering::AcqRel)
}

/// Longest capture edge accepted. Larger sensors are downscaled at capture
/// time to bound ISP latency and per-frame memory.
pub const MAX_RAW_DIM: i32 = 1920;

/// A capture geometry the sensor can actually deliver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureSize {
    /// Reader width in pixels (always even).
    pub width: i32,
    /// Reader height in pixels (always even).
    pub height: i32,
}

impl CaptureSize {
    /// Snap to even dimensions of at least 2×2.
    ///
    /// The ISP splits packed Bayer into even/odd columns, so an odd width makes
    /// the last column unreachable; an odd height breaks the 2×2 quad.
    fn even(width: i32, height: i32) -> Self {
        Self {
            width: (width & !1).max(2),
            height: (height & !1).max(2),
        }
    }

    fn area(&self) -> i64 {
        i64::from(self.width) * i64::from(self.height)
    }

    fn longest_edge(&self) -> i32 {
        self.width.max(self.height)
    }
}

/// Frames the `AImageReader` queue may hold. Three keeps the producer one
/// frame ahead without letting a stalled consumer grow the backlog.
const MAX_IMAGES: i32 = 3;

/// Bayer phase, matching the ISP's `bayer_pattern` encoding.
pub const RGGB: i32 = 0;
pub const GRBG: i32 = 1;
pub const GBRG: i32 = 2;
pub const BGGR: i32 = 3;

/// White level assumed when the sensor does not report one. A 10-bit sensor is
/// the common case among phones that omit `SENSOR_INFO_WHITE_LEVEL`.
const DEFAULT_WHITE_LEVEL: i32 = 1023;

/// Sensor properties that shape how frames must be interpreted.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorInfo {
    /// Active array width in pixels.
    pub width: i32,
    /// Active array height in pixels.
    pub height: i32,
    /// CFA phase, one of [`RGGB`], [`GRBG`], [`GBRG`], [`BGGR`].
    pub bayer_pattern: i32,
    /// Sensor orientation in degrees (0, 90, 180, 270) clockwise from
    /// the device's natural orientation. Matches
    /// `ACAMERA_SENSOR_INFO_ORIENTATION`.
    pub sensor_orientation: i32,
    /// Full-well capacity: the sample value the sensor saturates at.
    pub sensor_max: f32,
    /// `true` when the camera advertises the RAW capability.
    pub raw_supported: bool,
    /// Stream geometry to create the reader at, chosen from the camera's own
    /// advertised RAW16 configurations (see [`capture_size_candidates`]).
    pub capture: CaptureSize,
    /// Advertised RAW16 output sizes that fit the capture budget, largest
    /// first. Empty when the camera publishes no usable list.
    pub sizes: Vec<CaptureSize>,
}

impl SensorInfo {
    /// RAW16 geometries to try, best first.
    ///
    /// Falls back to the active array when the camera publishes no
    /// configuration list — a camera always accepts its own active array.
    fn raw_candidates(&self) -> Vec<CaptureSize> {
        if self.sizes.is_empty() {
            vec![self.capture]
        } else {
            self.sizes.clone()
        }
    }
}

/// Errors surfaced to the app loop as displayable strings.
pub type CamResult<T> = Result<T, String>;

/// A live capture session: open device + repeating request + image reader.
pub struct CameraSession {
    manager: *mut sys::ACameraManager,
    device: *mut sys::ACameraDevice,
    session: *mut sys::ACameraCaptureSession,
    request: *mut sys::ACaptureRequest,
    reader: *mut sys::AImageReader,
    output_target: *mut sys::ACameraOutputTarget,
    session_output: *mut sys::ACaptureSessionOutput,
    output_container: *mut sys::ACaptureSessionOutputContainer,
    bayer_pattern: i32,
    sensor_max: f32,
    raw: bool,
    sensor_width: u32,
    sensor_height: u32,
    sensor_orientation: i32,
}

// The NDK handles are owned exclusively by this struct: it is created and
// dropped on the app loop thread and never aliased anywhere else, so moving it
// across threads is sound.
unsafe impl Send for CameraSession {}

impl CameraSession {
    /// Open the first camera that yields a working capture configuration.
    pub fn open_default() -> CamResult<Self> {
        let manager = unsafe { sys::ACameraManager_create() };
        if manager.is_null() {
            return Err("ACameraManager_create returned null".into());
        }
        let mut session = Self {
            manager,
            device: null_mut(),
            session: null_mut(),
            request: null_mut(),
            reader: null_mut(),
            output_target: null_mut(),
            session_output: null_mut(),
            output_container: null_mut(),
            bayer_pattern: BGGR,
            sensor_max: DEFAULT_WHITE_LEVEL as f32,
            raw: true,
            sensor_width: 0,
            sensor_height: 0,
            sensor_orientation: 0,
        };

        let ids = session.list_camera_ids()?;
        if ids.is_empty() {
            return Err("no cameras reported by ACameraManager".into());
        }

        // First, enumerate all cameras and log their capabilities
        log::info!("=== Enumerating all cameras ===");
        let mut raw_supported_ids = Vec::new();
        let mut yuv_only_ids = Vec::new();
        for id in &ids {
            if let Ok(info) = session.read_sensor_info_for_id(id) {
                log::info!(
                    "Camera '{}': {}x{}, bayer={}, max={}, raw_supported={}",
                    id,
                    info.width,
                    info.height,
                    info.bayer_pattern,
                    info.sensor_max,
                    info.raw_supported
                );
                if info.raw_supported {
                    raw_supported_ids.push(id.clone());
                } else {
                    yuv_only_ids.push(id.clone());
                }
            }
        }

        // Try RAW16 cameras first
        let mut last_err = String::from("no camera could be opened");
        for id in &raw_supported_ids {
            match session.try_open(id) {
                Ok(()) => {
                    // `try_open` falls back to YUV internally, so the id being
                    // raw-capable says nothing about what was actually opened.
                    log::info!(
                        "Opened camera {id} ({})",
                        if session.is_raw() {
                            "RAW16"
                        } else {
                            "YUV fallback"
                        }
                    );
                    return Ok(session);
                }
                Err(e) => {
                    log::warn!("RAW16 camera '{id}' failed: {e}");
                    last_err = format!("RAW16 camera '{id}': {e}");
                }
            }
        }

        // Fall back to YUV cameras if no RAW16 camera worked
        for id in &yuv_only_ids {
            match session.try_open(id) {
                Ok(()) => {
                    log::info!("Opened YUV fallback camera: {id}");
                    return Ok(session);
                }
                Err(e) => {
                    log::warn!("YUV camera '{id}' failed: {e}");
                    last_err = format!("YUV camera '{id}': {e}");
                }
            }
        }
        Err(last_err)
    }

    /// Read sensor info for a specific camera ID without opening it.
    fn read_sensor_info_for_id(&self, id: &str) -> CamResult<SensorInfo> {
        let c_id = CString::new(id).map_err(|_| "camera id contains a NUL".to_string())?;
        self.read_sensor_info(c_id.as_ptr())
    }

    /// Camera IDs reported by the manager, as owned strings.
    fn list_camera_ids(&self) -> CamResult<Vec<String>> {
        let mut list: *mut sys::ACameraIdList = null_mut();
        check(
            unsafe { sys::ACameraManager_getCameraIdList(self.manager, &mut list) },
            "ACameraManager_getCameraIdList",
        )?;
        if list.is_null() {
            return Err("ACameraManager_getCameraIdList returned a null list".into());
        }
        let n = unsafe { (*list).numCameras } as usize;
        let ids_ptr = unsafe { (*list).cameraIds };
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let s = unsafe { CStr::from_ptr(*ids_ptr.add(i)) };
            out.push(s.to_string_lossy().into_owned());
        }
        unsafe { sys::ACameraManager_deleteCameraIdList(list) };
        Ok(out)
    }

    /// Open one camera and start a repeating request, preferring RAW16.
    fn try_open(&mut self, id: &str) -> CamResult<()> {
        let c_id = CString::new(id).map_err(|_| "camera id contains a NUL".to_string())?;

        let mut device: *mut sys::ACameraDevice = null_mut();
        let mut cb = device_callbacks();
        check(
            unsafe {
                sys::ACameraManager_openCamera(self.manager, c_id.as_ptr(), &mut cb, &mut device)
            },
            "ACameraManager_openCamera",
        )?;
        if device.is_null() {
            return Err("ACameraManager_openCamera returned a null device".into());
        }
        self.device = device;

        let info = self.read_sensor_info(c_id.as_ptr())?;
        log::info!(
            "camera '{id}': {}x{} bayer={} max={} raw={} capture={}x{}",
            info.width,
            info.height,
            info.bayer_pattern,
            info.sensor_max,
            info.raw_supported,
            info.capture.width,
            info.capture.height
        );

        let w = info.capture.width;
        let h = info.capture.height;

        if info.raw_supported {
            // Try the chosen geometry first, then every smaller advertised
            // size. A camera may advertise a RAW16 configuration that the ISP
            // pipeline still rejects (e.g. one only valid with a private
            // format pair), so a single attempt is not enough evidence that
            // RAW capture is unavailable.
            for size in info.raw_candidates() {
                if self.device.is_null() {
                    // First iteration still owns the device opened above;
                    // later ones need a fresh handle because close_partial()
                    // closed it.
                    let mut device: *mut sys::ACameraDevice = null_mut();
                    let mut cb = device_callbacks();
                    check(
                        unsafe {
                            sys::ACameraManager_openCamera(
                                self.manager,
                                c_id.as_ptr(),
                                &mut cb,
                                &mut device,
                            )
                        },
                        "ACameraManager_openCamera (retry)",
                    )?;
                    self.device = device;
                }
                match self.start_capture(
                    size.width,
                    size.height,
                    sys::AIMAGE_FORMATS::AIMAGE_FORMAT_RAW16,
                ) {
                    Ok(()) => {
                        log::info!("RAW16 accepted at {}x{}", size.width, size.height);
                        self.raw = true;
                        self.bayer_pattern = info.bayer_pattern;
                        self.sensor_max = info.sensor_max;
                        // Record the *capture* geometry, not the sensor's
                        // active array: the reader is created at this size, so
                        // frames arrive at these dimensions and the ISP chain
                        // must be built from them.
                        self.sensor_width = size.width as u32;
                        self.sensor_height = size.height as u32;
                        self.sensor_orientation = info.sensor_orientation;
                        return Ok(());
                    }
                    Err(e) => {
                        log::warn!(
                            "RAW16 rejected at {}x{} ({e}); trying a smaller size",
                            size.width,
                            size.height
                        );
                        self.close_partial();
                    }
                }
            }
        }

        // No RAW16 geometry worked: fall back to YUV at the same budgeted size.
        if self.device.is_null() {
            let mut device: *mut sys::ACameraDevice = null_mut();
            let mut cb = device_callbacks();
            check(
                unsafe {
                    sys::ACameraManager_openCamera(
                        self.manager,
                        c_id.as_ptr(),
                        &mut cb,
                        &mut device,
                    )
                },
                "ACameraManager_openCamera (retry)",
            )?;
            self.device = device;
        }

        self.start_capture(w, h, sys::AIMAGE_FORMATS::AIMAGE_FORMAT_YUV_420_888)?;
        self.raw = false;
        self.bayer_pattern = info.bayer_pattern;
        self.sensor_width = w as u32;
        self.sensor_height = h as u32;
        self.sensor_orientation = info.sensor_orientation;
        // `copy_pixels` scales the 8-bit luma plane to full `u16` range
        // (`v * 257`), so the YUV path saturates at 65535 — not at the RAW
        // white level, which would clip everything above ~4 % of full scale.
        self.sensor_max = 65535.0;
        Ok(())
    }

    /// Query static characteristics: geometry, CFA phase, white level, RAW.
    fn read_sensor_info(&self, c_id: *const c_char) -> CamResult<SensorInfo> {
        let mut chars: *mut sys::ACameraMetadata = null_mut();
        check(
            unsafe { sys::ACameraManager_getCameraCharacteristics(self.manager, c_id, &mut chars) },
            "ACameraManager_getCameraCharacteristics",
        )?;

        // ACTIVE_ARRAY_SIZE is [left, top, right, bottom].
        let array_tag = sys::acamera_metadata_tag::ACAMERA_SENSOR_INFO_ACTIVE_ARRAY_SIZE.0;
        let (width, height) = match entry_i32(chars, array_tag) {
            Some(v) if v.len() >= 4 => (v[2] - v[0], v[3] - v[1]),
            _ => (1920, 1080),
        };
        let cfa_tag = sys::acamera_metadata_tag::ACAMERA_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT.0;
        let bayer_pattern = match entry_u8(chars, cfa_tag) {
            Some(v) if !v.is_empty() => cfa_to_bayer(v[0] as i32),
            _ => BGGR,
        };
        let orientation_tag = sys::acamera_metadata_tag::ACAMERA_SENSOR_ORIENTATION.0;
        let sensor_orientation = entry_i32(chars, orientation_tag)
            .and_then(|v| v.first().copied())
            .unwrap_or(0);
        let white_tag = sys::acamera_metadata_tag::ACAMERA_SENSOR_INFO_WHITE_LEVEL.0;
        let white_level = entry_i32(chars, white_tag)
            .and_then(|v| v.first().copied())
            .unwrap_or(DEFAULT_WHITE_LEVEL);

        let raw_supported = has_raw_capability(chars);
        // Resolve the stream geometry while the metadata is still alive: the
        // configuration list is the only authority on which reader sizes the
        // camera will actually accept.
        let sizes = capture_size_candidates(width, height, &raw_capture_sizes(chars));

        unsafe { sys::ACameraMetadata_free(chars) };

        Ok(SensorInfo {
            width,
            height,
            bayer_pattern,
            sensor_orientation,
            sensor_max: white_level.max(1) as f32,
            raw_supported,
            capture: sizes[0],
            sizes,
        })
    }

    /// Create an image reader in `format` and drive a repeating preview request.
    fn start_capture(
        &mut self,
        width: i32,
        height: i32,
        format: sys::AIMAGE_FORMATS,
    ) -> CamResult<()> {
        let name = if format == sys::AIMAGE_FORMATS::AIMAGE_FORMAT_RAW16 {
            "RAW16"
        } else {
            "YUV_420_888"
        };
        let mut reader: *mut sys::AImageReader = null_mut();
        let status = unsafe {
            sys::AImageReader_new(width, height, format.0 as i32, MAX_IMAGES, &mut reader)
        };
        if status != sys::media_status_t::AMEDIA_OK || reader.is_null() {
            return Err(format!(
                "AImageReader_new({name} {width}x{height}) -> {}",
                status.0
            ));
        }
        self.reader = reader;

        let mut window: *mut sys::ACameraWindowType = null_mut();
        media_check(
            unsafe { sys::AImageReader_getWindow(self.reader, &mut window) },
            "AImageReader_getWindow",
        )?;

        check(
            unsafe { sys::ACameraOutputTarget_create(window, &mut self.output_target) },
            "ACameraOutputTarget_create",
        )?;
        check(
            unsafe { sys::ACaptureSessionOutput_create(window, &mut self.session_output) },
            "ACaptureSessionOutput_create",
        )?;
        check(
            unsafe { sys::ACaptureSessionOutputContainer_create(&mut self.output_container) },
            "ACaptureSessionOutputContainer_create",
        )?;
        check(
            unsafe {
                sys::ACaptureSessionOutputContainer_add(self.output_container, self.session_output)
            },
            "ACaptureSessionOutputContainer_add",
        )?;
        check(
            unsafe {
                sys::ACameraDevice_createCaptureRequest(
                    self.device,
                    sys::ACameraDevice_request_template::TEMPLATE_PREVIEW,
                    &mut self.request,
                )
            },
            "ACameraDevice_createCaptureRequest",
        )?;
        check(
            unsafe { sys::ACaptureRequest_addTarget(self.request, self.output_target) },
            "ACaptureRequest_addTarget",
        )?;

        let cb = session_callbacks();
        check(
            unsafe {
                sys::ACameraDevice_createCaptureSession(
                    self.device,
                    self.output_container,
                    &cb,
                    &mut self.session,
                )
            },
            "ACameraDevice_createCaptureSession",
        )?;
        check(
            unsafe {
                sys::ACameraCaptureSession_setRepeatingRequest(
                    self.session,
                    null_mut(),
                    1,
                    &mut self.request,
                    null_mut(),
                )
            },
            "ACameraCaptureSession_setRepeatingRequest",
        )?;
        log::info!("capture started: {name} {width}x{height}");
        Ok(())
    }

    /// Bayer phase for this session's frames.
    pub fn bayer_pattern(&self) -> i32 {
        self.bayer_pattern
    }

    /// `true` when frames are Bayer RAW16, `false` for the YUV fallback.
    pub fn is_raw(&self) -> bool {
        self.raw
    }

    /// Full-well capacity the sensor reported, or the 10-bit default.
    pub fn sensor_max(&self) -> f32 {
        self.sensor_max
    }

    /// Active array width of the sensor.
    pub fn sensor_width(&self) -> u32 {
        self.sensor_width
    }

    /// Active array height of the sensor.
    pub fn sensor_height(&self) -> u32 {
        self.sensor_height
    }

    /// Sensor orientation in degrees clockwise from device natural orientation.
    pub fn sensor_orientation(&self) -> i32 {
        self.sensor_orientation
    }

    /// Acquire the next queued frame and copy its pixels out as RAW `u16`.
    ///
    /// Returns `Ok(None)` when the queue is empty — the caller should redraw
    /// the previous frame rather than stall.
    pub fn next_frame(&self) -> CamResult<Option<RawFrame>> {
        let mut image: *mut sys::AImage = null_mut();
        // `acquireLatestImage` drops everything but the newest queued frame, so
        // a stalled consumer resumes on the current scene rather than working
        // through a backlog of stale ones.
        let status = unsafe { sys::AImageReader_acquireLatestImage(self.reader, &mut image) };
        if status != sys::media_status_t::AMEDIA_OK {
            // A non-OK status can still hand back an image; deleting it here is
            // what keeps the reader's buffer count from draining.
            if !image.is_null() {
                unsafe { sys::AImage_delete(image) };
            }
            if status == sys::media_status_t::AMEDIA_IMGREADER_NO_BUFFER_AVAILABLE {
                return Ok(None);
            }
            return Err(format!("AImageReader_acquireLatestImage -> {}", status.0));
        }
        if image.is_null() {
            return Ok(None);
        }
        // Every path below drops the guard, which closes the image exactly once.
        let guard = ImageGuard(image);
        self.copy_pixels(guard.0).map(Some)
    }

    /// Read plane 0 into a tightly packed `u16` Bayer buffer.
    ///
    /// RAW16 is a de-stride of little-endian samples. The YUV fallback expands
    /// the 8-bit luma plane into a `GG` mosaic, halving the non-green values so
    /// the demosaic sees a plausible chroma ratio.
    fn copy_pixels(&self, image: *mut sys::AImage) -> CamResult<RawFrame> {
        let mut width = 0i32;
        let mut height = 0i32;
        media_check(
            unsafe { sys::AImage_getWidth(image, &mut width) },
            "AImage_getWidth",
        )?;
        media_check(
            unsafe { sys::AImage_getHeight(image, &mut height) },
            "AImage_getHeight",
        )?;

        let mut data: *mut u8 = null_mut();
        let mut data_len = 0i32;
        media_check(
            unsafe { sys::AImage_getPlaneData(image, 0, &mut data, &mut data_len) },
            "AImage_getPlaneData",
        )?;
        let mut row_stride = 0i32;
        media_check(
            unsafe { sys::AImage_getPlaneRowStride(image, 0, &mut row_stride) },
            "AImage_getPlaneRowStride",
        )?;
        let mut pixel_stride = 0i32;
        media_check(
            unsafe { sys::AImage_getPlanePixelStride(image, 0, &mut pixel_stride) },
            "AImage_getPlanePixelStride",
        )?;
        if data.is_null() || width <= 0 || height <= 0 {
            return Err("image reported a null or empty plane 0".into());
        }

        let w = width as usize;
        let h = height as usize;
        let mut out = vec![0u16; w * h];

        if self.raw {
            if pixel_stride != 2 {
                return Err(format!("unexpected RAW16 pixel stride {pixel_stride}"));
            }
            let src_stride = (row_stride as usize).max(w * 2);
            for y in 0..h {
                let row = unsafe { (data as *const u8).add(y * src_stride) };
                for x in 0..w {
                    let p = unsafe { row.add(x * 2) };
                    let lo = unsafe { *p } as u16;
                    let hi = unsafe { *p.add(1) } as u16;
                    out[y * w + x] = lo | (hi << 8);
                }
            }
        } else {
            if pixel_stride != 1 {
                return Err(format!("unexpected YUV pixel stride {pixel_stride}"));
            }
            let src_stride = (row_stride as usize).max(w);
            for y in 0..h {
                let row = unsafe { (data as *const u8).add(y * src_stride) };
                for x in 0..w {
                    // 8-bit → 16-bit full scale; 257 is 0xFFFF/0xFF.
                    let v = unsafe { *row.add(x) } as u16 * 257;
                    let is_green = ((x ^ y) & 1) == 0;
                    out[y * w + x] = if is_green { v } else { v / 2 };
                }
            }
        }

        let mut ts = 0i64;
        let ts_ok = unsafe { sys::AImage_getTimestamp(image, &mut ts) };
        Ok(RawFrame {
            timestamp_ns: if ts_ok == sys::media_status_t::AMEDIA_OK {
                ts.max(0) as u64
            } else {
                0
            },
            data: out,
            width: w as u32,
            height: h as u32,
        })
    }

    /// Release everything created so far, leaving the manager intact.
    fn close_partial(&mut self) {
        if !self.session.is_null() {
            unsafe {
                sys::ACameraCaptureSession_stopRepeating(self.session);
                sys::ACameraCaptureSession_close(self.session);
            }
            self.session = null_mut();
        }
        if !self.request.is_null() {
            unsafe { sys::ACaptureRequest_free(self.request) };
            self.request = null_mut();
        }
        if !self.output_container.is_null() {
            unsafe { sys::ACaptureSessionOutputContainer_free(self.output_container) };
            self.output_container = null_mut();
        }
        if !self.session_output.is_null() {
            unsafe { sys::ACaptureSessionOutput_free(self.session_output) };
            self.session_output = null_mut();
        }
        if !self.output_target.is_null() {
            unsafe { sys::ACameraOutputTarget_free(self.output_target) };
            self.output_target = null_mut();
        }
        if !self.reader.is_null() {
            unsafe { sys::AImageReader_delete(self.reader) };
            self.reader = null_mut();
        }
        if !self.device.is_null() {
            unsafe { sys::ACameraDevice_close(self.device) };
            self.device = null_mut();
        }
    }
}

impl Drop for CameraSession {
    fn drop(&mut self) {
        self.close_partial();
        if !self.manager.is_null() {
            unsafe { sys::ACameraManager_delete(self.manager) };
            self.manager = null_mut();
        }
    }
}

/// A tightly packed Bayer frame ready for the ISP.
pub struct RawFrame {
    /// `width * height` samples, little-endian `u16` regardless of source.
    pub data: Vec<u16>,
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Capture timestamp in nanoseconds, or 0 when the sensor gave none.
    pub timestamp_ns: u64,
}

/// Closes an `AImage` exactly once when dropped.
struct ImageGuard(*mut sys::AImage);

impl Drop for ImageGuard {
    fn drop(&mut self) {
        if !self.0.is_null() {
            unsafe { sys::AImage_delete(self.0) };
            self.0 = null_mut();
        }
    }
}

/// Read a metadata entry as a slice of `i32`, or `None` when absent.
///
/// The slice borrows framework-owned memory and is valid only while `md` is
/// alive, so callers must finish with it before calling
/// [`sys::ACameraMetadata_free`].
fn entry_i32<'a>(md: *mut sys::ACameraMetadata, tag: u32) -> Option<&'a [i32]> {
    let mut entry: sys::ACameraMetadata_const_entry = unsafe { std::mem::zeroed() };
    if unsafe { sys::ACameraMetadata_getConstEntry(md, tag, &mut entry) }
        != sys::camera_status_t::ACAMERA_OK
    {
        return None;
    }
    if entry.count == 0 {
        return None;
    }
    // SAFETY: for an INT32 entry the framework points this arm at `count`
    // consecutive `i32` values owned by the metadata object.
    Some(unsafe { std::slice::from_raw_parts(entry.data.i32_, entry.count as usize) })
}

/// Read a metadata entry as a slice of `u8`, or `None` when absent.
fn entry_u8<'a>(md: *mut sys::ACameraMetadata, tag: u32) -> Option<&'a [u8]> {
    let mut entry: sys::ACameraMetadata_const_entry = unsafe { std::mem::zeroed() };
    if unsafe { sys::ACameraMetadata_getConstEntry(md, tag, &mut entry) }
        != sys::camera_status_t::ACAMERA_OK
    {
        return None;
    }
    if entry.count == 0 {
        return None;
    }
    // SAFETY: for a BYTE entry the framework points this arm at `count`
    // consecutive `u8` values owned by the metadata object.
    Some(unsafe { std::slice::from_raw_parts(entry.data.u8_, entry.count as usize) })
}

/// `true` when the camera lists `REQUEST_AVAILABLE_CAPABILITIES_RAW`.
fn has_raw_capability(md: *mut sys::ACameraMetadata) -> bool {
    let tag = sys::acamera_metadata_tag::ACAMERA_REQUEST_AVAILABLE_CAPABILITIES.0;
    let Some(caps) = entry_i32(md, tag) else {
        return false;
    };
    let raw = sys::acamera_metadata_enum_acamera_request_available_capabilities::ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_RAW;
    caps.contains(&(raw.0 as i32))
}

/// Halide pixel format code for `AIMAGE_FORMAT_RAW16` (MIPI RAW16 = 0x20).
const HAL_PIXEL_FORMAT_RAW16: i32 = 32;

/// Output geometries the sensor can actually stream, in `RAW16`.
///
/// `ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS` is an INT32 array of
/// quadruples `(format, width, height, isInput)`; only the OUTPUT
/// (`isInput == 0`) RAW16 entries describe what an `AImageReader` may be
/// created at. Requesting anything else makes `AImageReader_new` fail, which is
/// how the previous fixed-1920 path ended up with no frame source at all.
///
/// Returns an empty vec when the camera publishes no configuration list.
fn raw_capture_sizes(md: *mut sys::ACameraMetadata) -> Vec<CaptureSize> {
    let tag = sys::acamera_metadata_tag::ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS.0;
    let Some(entries) = entry_i32(md, tag) else {
        return Vec::new();
    };
    let output =
        sys::acamera_metadata_enum_acamera_scaler_available_stream_configurations::ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;
    entries
        .as_chunks::<4>()
        .0
        .iter()
        .filter(|q| q[0] == HAL_PIXEL_FORMAT_RAW16 && q[3] == output.0 as i32)
        .filter(|q| q[1] > 0 && q[2] > 0)
        .map(|q| CaptureSize::even(q[1], q[2]))
        .collect()
}

/// Sort the advertised RAW16 sizes best-first: largest area whose longest edge
/// fits [`MAX_RAW_DIM`] comes first, so a 50 MP sensor costs 1080p rather than
/// 50 MP while a 720p sensor keeps its native size.
///
/// `available` is the camera's advertised list. When the camera publishes
/// nothing usable, the active array clipped to [`MAX_RAW_DIM`] is the only
/// candidate (every camera in the wild accepts its own active array).
pub fn capture_size_candidates(
    active_width: i32,
    active_height: i32,
    available: &[CaptureSize],
) -> Vec<CaptureSize> {
    if available.is_empty() {
        return vec![fit_to_limit(CaptureSize::even(active_width, active_height))];
    }

    let mut fitting: Vec<CaptureSize> = available
        .iter()
        .copied()
        .filter(|s| s.longest_edge() <= MAX_RAW_DIM)
        .collect();
    // Largest first; ties keep the advertised order so the result is stable.
    fitting.sort_by_key(|s| -s.area());

    if fitting.is_empty() {
        // Every advertised size exceeds the budget. Return them *smallest
        // first*, and never invent a geometry.
        //
        // `ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS` is exhaustive: a
        // streamer accepts those geometries and nothing else. Inventing
        // `1920x1436` from the advertised `2592x1940` therefore cannot succeed
        // — on this device it was rejected by
        // `ACameraDevice_createCaptureSession` with status -10000 (UNKNOWN),
        // which dropped the whole capture to YUV. Overshooting the
        // `MAX_RAW_DIM` budget is the lesser evil: the ISP chain is built from
        // the real capture geometry, so only a sensor whose *smallest* RAW16
        // stream is still larger pays for it.
        let mut all: Vec<CaptureSize> = available.to_vec();
        all.sort_by_key(|s| s.area());
        return all;
    }
    fitting
}

/// Shrink `size` proportionally so its longest edge is at most [`MAX_RAW_DIM`].
fn fit_to_limit(size: CaptureSize) -> CaptureSize {
    let longest = size.longest_edge();
    if longest <= MAX_RAW_DIM {
        return size;
    }
    let scale = f64::from(MAX_RAW_DIM) / f64::from(longest);
    CaptureSize::even(
        (f64::from(size.width) * scale).round() as i32,
        (f64::from(size.height) * scale).round() as i32,
    )
}

/// Map the sensor CFA phase byte to the ISP's `bayer_pattern` encoding.
///
/// The ISP uses 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR — the same numbering the
/// camera2 `COLOR_FILTER_ARRANGEMENT` enum uses.
fn cfa_to_bayer(cfa: i32) -> i32 {
    match cfa {
        0 => RGGB,
        1 => GRBG,
        2 => GBRG,
        3 => BGGR,
        // 4 = RGB (non-Bayer) or vendor-specific: BGGR is the safe default
        // for a Bayer pipeline.
        _ => BGGR,
    }
}

/// Zeroed device-state callbacks with all handlers installed.
fn device_callbacks() -> sys::ACameraDevice_StateCallbacks {
    let mut cb: sys::ACameraDevice_StateCallbacks = unsafe { std::mem::zeroed() };
    cb.context = null_mut();
    cb.onDisconnected = Some(on_device_disconnected);
    cb.onError = Some(on_device_error);
    cb
}

/// Zeroed session-state callbacks with all handlers installed.
fn session_callbacks() -> sys::ACameraCaptureSession_stateCallbacks {
    let mut cb: sys::ACameraCaptureSession_stateCallbacks = unsafe { std::mem::zeroed() };
    cb.context = null_mut();
    cb.onClosed = Some(on_session_closed);
    cb.onReady = Some(on_session_ready);
    cb.onActive = Some(on_session_active);
    cb
}

/// Map a `camera_status_t` to a diagnostic string, or `Ok(())` on success.
fn check(status: sys::camera_status_t, what: &str) -> CamResult<()> {
    if status == sys::camera_status_t::ACAMERA_OK {
        return Ok(());
    }
    let detail = match status {
        sys::camera_status_t::ACAMERA_ERROR_CAMERA_DISCONNECTED => "CAMERA_DISCONNECTED",
        sys::camera_status_t::ACAMERA_ERROR_NOT_ENOUGH_MEMORY => "NOT_ENOUGH_MEMORY",
        sys::camera_status_t::ACAMERA_ERROR_METADATA_NOT_FOUND => "METADATA_NOT_FOUND",
        sys::camera_status_t::ACAMERA_ERROR_CAMERA_DEVICE => "CAMERA_DEVICE",
        sys::camera_status_t::ACAMERA_ERROR_CAMERA_SERVICE => "CAMERA_SERVICE",
        sys::camera_status_t::ACAMERA_ERROR_SESSION_CLOSED => "SESSION_CLOSED",
        sys::camera_status_t::ACAMERA_ERROR_INVALID_OPERATION => "INVALID_OPERATION",
        sys::camera_status_t::ACAMERA_ERROR_STREAM_CONFIGURE_FAIL => "STREAM_CONFIGURE_FAIL",
        sys::camera_status_t::ACAMERA_ERROR_CAMERA_IN_USE => "CAMERA_IN_USE",
        sys::camera_status_t::ACAMERA_ERROR_MAX_CAMERA_IN_USE => "MAX_CAMERA_IN_USE",
        sys::camera_status_t::ACAMERA_ERROR_CAMERA_DISABLED => "CAMERA_DISABLED",
        sys::camera_status_t::ACAMERA_ERROR_PERMISSION_DENIED => "PERMISSION_DENIED",
        sys::camera_status_t::ACAMERA_ERROR_UNSUPPORTED_OPERATION => "UNSUPPORTED_OPERATION",
        _ => "UNKNOWN",
    };
    Err(format!("{what} -> {detail} ({})", status.0))
}

/// `media_status_t` variant of [`check`].
fn media_check(status: sys::media_status_t, what: &str) -> CamResult<()> {
    if status == sys::media_status_t::AMEDIA_OK {
        Ok(())
    } else {
        Err(format!("{what} -> {}", status.0))
    }
}

// ── NDK callbacks ───────────────────────────────────────────────────────
// These fire on framework-owned threads and only log + raise a flag, so they
// touch no state shared with the app loop and need no locking.

fn camera_error_name(error: i32) -> &'static str {
    match error {
        1 => "ERROR_CAMERA_IN_USE",
        2 => "ERROR_MAX_CAMERAS_IN_USE",
        3 => "ERROR_CAMERA_DISABLED",
        4 => "ERROR_CAMERA_DEVICE",
        5 => "ERROR_CAMERA_SERVICE",
        _ => "ERROR_UNKNOWN",
    }
}

unsafe extern "C" fn on_device_disconnected(_ctx: *mut c_void, _device: *mut sys::ACameraDevice) {
    log::warn!("ACameraDevice: disconnected");
    FATAL_CAMERA_ERROR.store(true, Ordering::Release);
}

unsafe extern "C" fn on_device_error(
    _ctx: *mut c_void,
    _device: *mut sys::ACameraDevice,
    error: i32,
) {
    log::error!(
        "ACameraDevice: error {} ({})",
        error,
        camera_error_name(error)
    );
    FATAL_CAMERA_ERROR.store(true, Ordering::Release);
}

unsafe extern "C" fn on_session_closed(_ctx: *mut c_void, _s: *mut sys::ACameraCaptureSession) {
    log::warn!("ACameraCaptureSession: closed");
}

unsafe extern "C" fn on_session_ready(_ctx: *mut c_void, _s: *mut sys::ACameraCaptureSession) {
    log::info!("ACameraCaptureSession: ready");
}

unsafe extern "C" fn on_session_active(_ctx: *mut c_void, _s: *mut sys::ACameraCaptureSession) {
    log::info!("ACameraCaptureSession: active");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cfa_maps_to_isp_bayer_codes() {
        assert_eq!(cfa_to_bayer(0), RGGB);
        assert_eq!(cfa_to_bayer(1), GRBG);
        assert_eq!(cfa_to_bayer(2), GBRG);
        assert_eq!(cfa_to_bayer(3), BGGR);
    }

    #[test]
    fn cfa_falls_back_to_bggr() {
        // 4 = RGB filter array; anything unknown must still be in range.
        for cfa in [4, 5, 99, -1] {
            assert_eq!(cfa_to_bayer(cfa), BGGR);
        }
    }

    fn size(w: i32, h: i32) -> CaptureSize {
        CaptureSize {
            width: w,
            height: h,
        }
    }

    #[test]
    fn capture_size_is_forced_even() {
        // Odd edges cannot carry a complete 2×2 Bayer quad.
        assert_eq!(CaptureSize::even(1921, 1441), size(1920, 1440));
        assert_eq!(CaptureSize::even(1, 1), size(2, 2));
        assert_eq!(CaptureSize::even(-5, 0), size(2, 2));
    }

    #[test]
    fn prefers_largest_advertised_size_within_budget() {
        // A 50 MP sensor advertises far more than the capture budget allows;
        // the biggest that fits must win, not the sensor's full array.
        let advertised = [
            size(4000, 3000),
            size(1920, 1080),
            size(1280, 720),
            size(640, 480),
        ];
        assert_eq!(
            capture_size_candidates(4000, 3000, &advertised)[0],
            size(1920, 1080)
        );
    }

    #[test]
    fn keeps_native_size_when_camera_is_small() {
        let advertised = [size(1280, 720), size(640, 480)];
        assert_eq!(
            capture_size_candidates(1280, 720, &advertised)[0],
            size(1280, 720)
        );
    }

    #[test]
    fn candidates_are_sorted_largest_first_and_bounded() {
        let advertised = [
            size(640, 480),
            size(3840, 2160),
            size(1920, 1080),
            size(1280, 720),
        ];
        let got = capture_size_candidates(3840, 2160, &advertised);
        // 3840x2160 exceeds MAX_RAW_DIM, so it is dropped — not scaled.
        assert_eq!(got, vec![size(1920, 1080), size(1280, 720), size(640, 480)]);
        assert!(got.iter().all(|s| s.longest_edge() <= MAX_RAW_DIM));
    }

    #[test]
    fn falls_back_to_active_array_when_unadvertised() {
        // No configuration list: trust the active array, scaled to the budget.
        let got = capture_size_candidates(4000, 3000, &[]);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].longest_edge(), MAX_RAW_DIM);
        // Aspect ratio is preserved within rounding.
        let aspect = f64::from(got[0].width) / f64::from(got[0].height);
        assert!(
            (aspect - 4.0 / 3.0).abs() < 0.01,
            "aspect drifted: {aspect}"
        );
    }

    #[test]
    fn oversize_only_list_returns_advertised_sizes_smallest_first() {
        // Every advertised size is above the budget. The candidates must still
        // be geometries the camera actually advertises — a scaled invention
        // like 1920x1436 is not streamable and `createCaptureSession` rejects
        // it outright.
        let advertised = [size(4000, 3000), size(2592, 1940)];
        let got = capture_size_candidates(2592, 1940, &advertised);
        assert_eq!(got, vec![size(2592, 1940), size(4000, 3000)]);
        assert!(
            got.iter().all(|s| advertised.contains(s)),
            "every candidate must be an advertised geometry, got {got:?}"
        );
    }
}
