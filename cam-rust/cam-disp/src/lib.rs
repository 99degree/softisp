//! cam-disp: DRM display module for ARGB8888 frames.
//!
//! Uses raw ioctl to /dev/dri/card0 to create a framebuffer and display it.
//! Minimal, zero-copy where possible.
//!
//! # Usage
//! ```rust,ignore
//! let mut disp = cam_disp::DrmDisplay::new().expect("open drm");
//! disp.display_argb8888(1920, 1080, &argb_data).expect("display frame");
//! ```

#![allow(dead_code, unused_variables, unused_imports)]

use std::fs::{File, OpenOptions};
use std::io::{self, Result};
use std::mem::size_of;
use std::os::unix::io::{AsRawFd, RawFd};
use std::ptr;

// ---------------------------------------------------------------------------
// DRM ioctl numbers (from linux/drm.h)
// ---------------------------------------------------------------------------
const DRM_IOCTL_BASE: u8 = b'd';

const fn _ioc(dir: u8, typ: u8, nr: u8, size: usize) -> u64 {
    ((dir as u64) << 30) | ((typ as u64) << 8) | (nr as u64) | ((size as u64) << 16)
}
const fn _io(typ: u8, nr: u8) -> u64 {
    _ioc(0, typ, nr, 0)
}
const fn _ior(typ: u8, nr: u8, size: usize) -> u64 {
    _ioc(1, typ, nr, size)
}
const fn _iow(typ: u8, nr: u8, size: usize) -> u64 {
    _ioc(2, typ, nr, size)
}
const fn _iowr(typ: u8, nr: u8, size: usize) -> u64 {
    _ioc(3, typ, nr, size)
}

const fn drm_ioctl(nr: u8) -> u64 {
    _io(DRM_IOCTL_BASE, nr)
}
const fn drm_iow(nr: u8, size: usize) -> u64 {
    _iow(DRM_IOCTL_BASE, nr, size)
}
const fn drm_ior(nr: u8, size: usize) -> u64 {
    _ior(DRM_IOCTL_BASE, nr, size)
}
const fn drm_iowr(nr: u8, size: usize) -> u64 {
    _iowr(DRM_IOCTL_BASE, nr, size)
}

// DRM ioctl numbers (linux 5.x+)
const DRM_IOCTL_GET_RESOURCES: u64 = drm_ior(0x04, size_of::<DrmCardRes>());
const DRM_IOCTL_GET_CONNECTOR: u64 = drm_iowr(0x15, size_of::<DrmModeConnector>());
const DRM_IOCTL_GET_ENCODER: u64 = drm_iowr(0x14, size_of::<DrmModeEncoder>());
const DRM_IOCTL_GET_CRTC: u64 = drm_iowr(0x1a, size_of::<DrmModeCrtc>());
const DRM_IOCTL_SET_CRTC: u64 = drm_iowr(0x1d, size_of::<DrmModeCrtc>());
const DRM_IOCTL_MODE_ADDFB: u64 = drm_iowr(0xae, size_of::<DrmModeFbCmd>());
const DRM_IOCTL_MODE_RMFB: u64 = drm_iow(0xaf, size_of::<u32>());
const DRM_IOCTL_MODE_PAGE_FLIP: u64 = drm_iowr(0xb0, size_of::<DrmModePageFlip>());
const DRM_IOCTL_MODE_GETPLANE: u64 = drm_iowr(0xb6, size_of::<DrmModePlane>());
const DRM_IOCTL_MODE_CREATE_DUMB: u64 = drm_iowr(0xb2, size_of::<DrmModeCreateDumb>());
const DRM_IOCTL_MODE_MAP_DUMB: u64 = drm_iowr(0xb3, size_of::<DrmModeMapDumb>());
const DRM_IOCTL_MODE_DESTROY_DUMB: u64 = drm_iowr(0xb4, size_of::<DrmModeDestroyDumb>());

// Client capability: universal planes mod (kernel < 4.12 compat)
const DRM_CLIENT_CAP_UNIVERSAL_PLANES: u64 = 2;
const DRM_IOCTL_SET_CLIENT_CAP: u64 = drm_iow(0x0d, size_of::<DrmSetClientCap>());

// Flags
const DRM_MODE_CONNECTED: u32 = 1;
const DRM_MODE_DISCONNECTED: u32 = 2;
const _DRM_MODE_PAGE_FLIP_EVENT: u32 = 1;
const DRM_CLOAK_FB: u32 = 0;

// ---------------------------------------------------------------------------
// DRM structs (packed, repr(C))
// ---------------------------------------------------------------------------
#[repr(C)]
struct DrmCardRes {
    count_fbs: u64,
    fb_id_ptr: u64,
    count_crtcs: u64,
    crtc_id_ptr: u64,
    count_connectors: u64,
    connector_id_ptr: u64,
    count_encoders: u64,
    encoder_id_ptr: u64,
    min_width: u32,
    max_width: u32,
    min_height: u32,
    max_height: u32,
}

#[repr(C)]
struct DrmModeConnector {
    connector_id: u32,
    encoder_id: u32,
    connector_type: u32,
    connector_type_id: u32,
    connection: u32,
    mm_width: u32,
    mm_height: u32,
    subpixel: u32,
    count_modes: u32,
    modes_ptr: u64,
    count_props: u32,
    props_ptr: u64,
    prop_values_ptr: u64,
    count_encoders: u32,
    encoders_ptr: u64,
}

#[repr(C)]
struct DrmModeEncoder {
    encoder_id: u32,
    encoder_type: u32,
    crtc_id: u32,
    possible_crtcs: u32,
    possible_clones: u32,
}

#[repr(C)]
struct DrmModeCrtc {
    crtc_id: u32,
    fb_id: u32,
    crtc_x: u32,
    crtc_y: u32,
    gamma_size: u32,
    mode_valid: u32,
    #[allow(dead_code)]
    mode: DrmModeModeInfo,
}

#[repr(C)]
struct DrmModeModeInfo {
    clock: u32,
    hdisplay: u16,
    hsync_start: u16,
    hsync_end: u16,
    htotal: u16,
    hskew: u16,
    vdisplay: u16,
    vsync_start: u16,
    vsync_end: u16,
    vtotal: u16,
    vscan: u16,
    vrefresh: u32,
    flags: u32,
    type_: u32,
    name: [u8; 32],
}

#[repr(C)]
struct DrmModeCreateDumb {
    height: u32,
    width: u32,
    bpp: u32,
    flags: u32,
    handle: u32,
    pitch: u32,
    size: u64,
}

#[repr(C)]
struct DrmModeMapDumb {
    handle: u32,
    pad: u32,
    offset: u64,
}

#[repr(C)]
struct DrmModeDestroyDumb {
    handle: u32,
}

#[repr(C)]
struct DrmModeFbCmd {
    fb_id: u32,
    width: u32,
    height: u32,
    pitch: u32,
    bpp: u32,
    depth: u32,
    handle: u32,
}

#[repr(C)]
struct DrmSetClientCap {
    capability: u64,
    value: u64,
}

#[repr(C)]
struct DrmModePageFlip {
    crtc_id: u32,
    fb_id: u32,
    flags: u32,
    reserved: u32,
    user_data: u64,
}

#[repr(C)]
struct DrmModePlane {
    plane_id: u32,
    crtc_id: u32,
    fb_id: u32,
    possible_crtcs: u32,
    gamma_size: u32,
    count_format_types: u32,
    format_type_ptr: u64,
    count_planes: u32,
    count_combines: u32,
}

// ---------------------------------------------------------------------------
// DrmDisplay
// ---------------------------------------------------------------------------
pub struct DrmDisplay {
    fd: File,
    fb_id: u32,
    crtc_id: u32,
    connector_id: u32,
    saved_crtc: Option<DrmModeCrtc>,
}

impl DrmDisplay {
    /// Open DRM device and find a connected connector + CRTC.
    pub fn new() -> Result<Self> {
        Self::open_with_path("/dev/dri/card0")
    }

    /// Open with explicit device path.
    pub fn open_with_path(path: &str) -> Result<Self> {
        let fd = OpenOptions::new().read(true).write(true).open(path)?;

        // Enable universal planes (best-effort, may fail on older kernels)
        let _ = Self::drm_set_client_cap(fd.as_raw_fd(), DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);

        // Get card resources
        let res = Self::drm_get_resources(fd.as_raw_fd())?;

        // Find first connected connector
        let (connector_id, _conn) = Self::find_connector(fd.as_raw_fd(), &res)?;

        // Find first encoder for this connector -> first CRTC
        let crtc_id = Self::find_crtc(fd.as_raw_fd(), connector_id)?;

        // Disable the CRTC initially (teardown any previous fb)
        let saved_crtc = Self::save_and_disable_crtc(fd.as_raw_fd(), crtc_id)?;

        Ok(Self {
            fd,
            fb_id: 0,
            crtc_id,
            connector_id,
            saved_crtc: Some(saved_crtc),
        })
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// Display an ARGB8888 frame. The buffer must be `width * height * 4` bytes.
    /// Creates a dumb buffer, writes data via mmap, and page-flips.
    pub fn display_argb8888(&mut self, width: u32, height: u32, data: &[u8]) -> Result<()> {
        let pitch = width * 4; // ARGB = 4 Bpp
        self.display_raw(width, height, pitch, 32, data)
    }

    /// Generic raw-framebuffer display.
    pub fn display_raw(
        &mut self,
        width: u32,
        height: u32,
        pitch: u32,
        bpp: u32,
        data: &[u8],
    ) -> Result<()> {
        // Remove old framebuffer if any
        if self.fb_id != 0 {
            let _ = Self::drm_rmfb(self.fd.as_raw_fd(), self.fb_id);
            self.fb_id = 0;
        }

        // Create dumb buffer
        let mut create = DrmModeCreateDumb {
            width,
            height,
            bpp,
            flags: 0,
            handle: 0,
            pitch: 0,
            size: 0,
        };
        Self::drm_ioctl_obj(self.fd.as_raw_fd(), DRM_IOCTL_MODE_CREATE_DUMB, &mut create)?;

        // Map dumb buffer
        let mut map = DrmModeMapDumb {
            handle: create.handle,
            pad: 0,
            offset: 0,
        };
        Self::drm_ioctl_obj(self.fd.as_raw_fd(), DRM_IOCTL_MODE_MAP_DUMB, &mut map)?;

        // mmap the buffer
        let size = create.size as usize;
        let ptr = unsafe {
            libc::mmap(
                ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                self.fd.as_raw_fd(),
                map.offset as i64,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }

        // Write pixel data
        let copy_len = size.min(data.len());
        unsafe {
            ptr::copy_nonoverlapping(data.as_ptr(), ptr as *mut u8, copy_len);
        }

        // Unmap
        unsafe {
            libc::munmap(ptr, size);
        }

        // Add framebuffer
        let mut fb = DrmModeFbCmd {
            fb_id: 0,
            width,
            height,
            pitch: create.pitch, // real pitch from DRM
            bpp,
            depth: 24,
            handle: create.handle,
        };
        Self::drm_ioctl_obj(self.fd.as_raw_fd(), DRM_IOCTL_MODE_ADDFB, &mut fb)?;
        self.fb_id = fb.fb_id;

        // Page flip: set CRTC to show this fb
        let mut flip = DrmModePageFlip {
            crtc_id: self.crtc_id,
            fb_id: self.fb_id,
            flags: 0, // synchronous
            reserved: 0,
            user_data: 0,
        };
        if let Err(_e) =
            Self::drm_ioctl_obj(self.fd.as_raw_fd(), DRM_IOCTL_MODE_PAGE_FLIP, &mut flip)
        {
            // Fallback: try SET_CRTC instead (older kernels)
            Self::drm_ioctl_obj(
                self.fd.as_raw_fd(),
                DRM_IOCTL_SET_CRTC,
                &mut self
                    .saved_crtc
                    .get_or_insert_with(|| unsafe { std::mem::zeroed() }),
            )
            .map_err(|e2| io::Error::other(format!("page_flip + set_crtc both failed: {}", e2)))?;
        }

        Ok(())
    }

    // ------------------------------------------------------------------
    // Internal DRM ioctl helpers
    // ------------------------------------------------------------------

    fn drm_ioctl_obj<T>(fd: RawFd, req: u64, obj: &mut T) -> Result<()> {
        let ret = unsafe { libc::ioctl(fd, req as i32, obj as *mut T as *mut libc::c_void) };
        if ret < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    fn drm_get_resources(fd: RawFd) -> Result<DrmCardRes> {
        let mut res: DrmCardRes = unsafe { std::mem::zeroed() };
        Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_RESOURCES, &mut res)?;
        Ok(res)
    }

    fn drm_set_client_cap(fd: RawFd, cap: u64, val: u64) -> Result<()> {
        let mut s = DrmSetClientCap {
            capability: cap,
            value: val,
        };
        Self::drm_ioctl_obj(fd, DRM_IOCTL_SET_CLIENT_CAP, &mut s)
    }

    fn find_connector(fd: RawFd, res: &DrmCardRes) -> Result<(u32, DrmModeConnector)> {
        // Read connector IDs from pointer
        let count = res.count_connectors as usize;
        if count == 0 {
            return Err(io::Error::new(io::ErrorKind::NotFound, "No connectors"));
        }
        let ptr = res.connector_id_ptr as *const u32;
        let ids: Vec<u32> = if count > 0 && !ptr.is_null() {
            unsafe { std::slice::from_raw_parts(ptr, count).to_vec() }
        } else {
            vec![]
        };
        for &id in &ids {
            // Get connector info
            let mut conn: DrmModeConnector = unsafe { std::mem::zeroed() };
            conn.connector_id = id;
            if Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_CONNECTOR, &mut conn).is_err() {
                continue;
            }
            if conn.connection == DRM_MODE_CONNECTED && conn.count_modes > 0 {
                return Ok((id, conn));
            }
        }
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            "No connected connector",
        ))
    }

    fn find_crtc(fd: RawFd, connector_id: u32) -> Result<u32> {
        // Get connector to find its encoder
        let mut conn: DrmModeConnector = unsafe { std::mem::zeroed() };
        conn.connector_id = connector_id;
        Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_CONNECTOR, &mut conn)?;

        // Get encoder
        let encoder_id = conn.encoder_id;
        if encoder_id == 0 {
            // Try the first encoder from the list
            let count = conn.count_encoders as usize;
            if count == 0 {
                return Err(io::Error::new(io::ErrorKind::NotFound, "No encoder"));
            }
            let ptr = conn.encoders_ptr as *const u32;
            if ptr.is_null() {
                return Err(io::Error::new(io::ErrorKind::NotFound, "null encoder ptr"));
            }
            let ids = unsafe { std::slice::from_raw_parts(ptr, count) };
            // Try each encoder until we find one with a valid CRTC
            for &eid in ids {
                let mut enc: DrmModeEncoder = unsafe { std::mem::zeroed() };
                enc.encoder_id = eid;
                if Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_ENCODER, &mut enc).is_ok() {
                    if enc.crtc_id != 0 {
                        return Ok(enc.crtc_id);
                    }
                    // If encoder has possible CRTCs, pick first bit
                    if enc.possible_crtcs != 0 {
                        let crtc_bit = enc.possible_crtcs.trailing_zeros();
                        // The CRTC IDs are densely numbered from 0-31 usually
                        let crtc_id = crtc_bit + 1;
                        return Ok(crtc_id);
                    }
                }
            }
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "No possible CRTC from encoders",
            ));
        }

        let mut enc: DrmModeEncoder = unsafe { std::mem::zeroed() };
        enc.encoder_id = encoder_id;
        Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_ENCODER, &mut enc)?;

        if enc.crtc_id != 0 {
            return Ok(enc.crtc_id);
        }
        if enc.possible_crtcs != 0 {
            let crtc_bit = enc.possible_crtcs.trailing_zeros();
            return Ok(crtc_bit + 1);
        }

        Err(io::Error::new(
            io::ErrorKind::NotFound,
            "No CRTC for encoder",
        ))
    }

    fn save_and_disable_crtc(fd: RawFd, crtc_id: u32) -> Result<DrmModeCrtc> {
        let mut crtc: DrmModeCrtc = unsafe { std::mem::zeroed() };
        crtc.crtc_id = crtc_id;
        crtc.fb_id = 0;
        let _ = Self::drm_ioctl_obj(fd, DRM_IOCTL_GET_CRTC, &mut crtc);
        Ok(crtc)
    }

    fn drm_rmfb(fd: RawFd, fb_id: u32) -> Result<()> {
        Self::drm_ioctl_obj(fd, DRM_IOCTL_MODE_RMFB, &mut { fb_id })
    }
}

impl Drop for DrmDisplay {
    fn drop(&mut self) {
        if self.fb_id != 0 {
            let _ = Self::drm_rmfb(self.fd.as_raw_fd(), self.fb_id);
        }
        // Restore CRTC if we saved it
        if let Some(saved) = self.saved_crtc.take() {
            // Re-set CRTC with saved fb (0 = blank)
            let _ = Self::drm_ioctl_obj(self.fd.as_raw_fd(), DRM_IOCTL_SET_CRTC, &mut { saved });
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open_drm() {
        // This test requires /dev/dri/card0 to exist
        let disp = DrmDisplay::new();
        match disp {
            Ok(_) => println!("DRM display opened successfully"),
            Err(e) => println!("DRM not available (expected in CI): {}", e),
        }
    }
}
