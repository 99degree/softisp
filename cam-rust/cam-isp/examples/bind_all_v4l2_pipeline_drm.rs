//! Integration: V4L2 camera → ISP pipeline (UNIFIED) → DRM display.
//!
//! Flow:
//! 1. Scan `/dev/video*` for a camera device and attempt raw capture.
//! 2. If no V4L2 device, generate a synthetic 4K rainbow Bayer frame.
//! 3. Process through UNIFIED pipeline → ARGB8888 output.
//! 4. Display the output frame via DRM.
//!
//! Usage:
//!   cargo run --example bind_all_v4l2_pipeline_drm -p cam-isp --features rectifier
//!   bind_isp_drm  (if installed to ~/.cargo/bin)

use cam_disp::DrmDisplay;
use cam_isp::engine::OutputFormat as EngineOutputFormat;
use cam_isp::profile::PipelineProfile;
use cam_isp::unified_pipeline::{UnifiedConfig, UnifiedPipeline};

fn main() {
    cam_isp::init();
    println!("[1/4] ISP engine registry initialized");

    // -----------------------------------------------------------------
    // Stage 1: Acquire input frame — V4L2 or synthetic 4K Bayer.
    // -----------------------------------------------------------------
    let (width, height, raw) = try_v4l2_or_synthetic();
    println!("[2/4] Input: {}x{} ({} bytes)", width, height, raw.len(),);

    // -----------------------------------------------------------------
    // Stage 2: Build UNIFIED pipeline.
    // -----------------------------------------------------------------
    let config = UnifiedConfig {
        profile: PipelineProfile::UNIFIED,
        target_width: width,
        engine_preference: "auto".into(),
        output_format: EngineOutputFormat::Argb,
        sensor_max: 1023.0,
        ..UnifiedConfig::hd()
    };
    let mut pipeline = UnifiedPipeline::new(config).expect("Failed to build pipeline");
    println!("[3/4] UNIFIED pipeline built");

    // -----------------------------------------------------------------
    // Stage 3: Process.
    // -----------------------------------------------------------------
    let result = pipeline
        .process(&raw, width, height)
        .expect("Pipeline process failed");
    println!(
        "[3/4] Pipeline output: {}x{} format={:?} {} bytes",
        result.width,
        result.height,
        result.format,
        result.data.len()
    );

    // -----------------------------------------------------------------
    // Stage 4: DRM display.
    // -----------------------------------------------------------------
    match DrmDisplay::new() {
        Ok(mut display) => {
            display
                .display_argb8888(result.width, result.height, &result.data)
                .expect("DRM display failed");
            println!("[4/4] Displayed {}x{} via DRM", result.width, result.height);
        }
        Err(e) => {
            println!("[4/4] DRM unavailable ({e}). Pipeline output OK.");
        }
    }
}

/// Scan `/dev/video*` and attempt V4L2 capture.
/// Falls back to synthetic 4K rainbow Bayer on any failure.
fn try_v4l2_or_synthetic() -> (u32, u32, Vec<u8>) {
    let devices = scan_video_devices();
    if devices.is_empty() {
        println!("  V4L2: no /dev/video* found, using synthetic 4K");
    } else {
        for dev in &devices {
            println!("  V4L2: trying {dev}");
            match try_v4l2_capture(dev, 3840, 2160) {
                Ok((w, h, bayer)) => {
                    println!("  V4L2: captured {w}x{h} ({})", bayer.len());
                    return (w, h, bayer);
                }
                Err(e) => {
                    println!("  V4L2: {dev} failed ({e})");
                }
            }
        }
        println!("  V4L2: all devices failed, using synthetic 4K");
    }

    let (w, h) = (3840u32, 2160u32);
    (w, h, make_rainbow_bayer(w, h))
}

/// Open a V4L2 device, negotiate a capture format, and read one frame.
/// Returns `(width, height, raw_bayer_u16_le)`.
fn try_v4l2_capture(dev: &str, req_w: u32, req_h: u32) -> Result<(u32, u32, Vec<u8>), String> {
    use std::fs::OpenOptions;
    use std::os::unix::io::AsRawFd;

    // Open device
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .open(dev)
        .map_err(|e| format!("open: {e}"))?;

    let fd = file.as_raw_fd();

    // VIDIOC_QUERYCAP — verify it's a video capture device
    #[repr(C)]
    struct V4l2Capability {
        driver: [u8; 16],
        card: [u8; 32],
        bus_info: [u8; 32],
        version: u32,
        capabilities: u32,
        device_caps: u32,
        reserved: [u32; 3],
    }

    let mut cap: V4l2Capability = unsafe { std::mem::zeroed() };
    unsafe {
        ioctl_void(fd, 0x802c5600, &mut cap as *mut _ as *mut libc::c_void)
            .map_err(|e| format!("QUERYCAP: {e}"))?;
    }

    let caps = cap.device_caps | cap.capabilities;
    if caps & 0x00000001 == 0 {
        // V4L2_CAP_VIDEO_CAPTURE
        return Err("not a capture device".into());
    }

    // VIDIOC_S_FMT — request UYVY (packed 422, widely supported) or fallback
    #[repr(C)]
    struct V4l2Format {
        typ: u32,
        fmt: V4l2FormatFmt,
    }

    #[repr(C)]
    union V4l2FormatFmt {
        raw: [u8; 200],
    }

    #[repr(C)]
    #[derive(Copy, Clone)]
    struct V4l2PixFormat {
        width: u32,
        height: u32,
        pixelformat: u32,
        field: u32,
        bytesperline: u32,
        sizeimage: u32,
        colorspace: u32,
        priv_data: u32,
        flags: u32,
        henc: u32,
        quantization: u32,
        xfer_func: u32,
    }

    let mut fmt: V4l2Format = unsafe { std::mem::zeroed() };
    fmt.typ = 1; // V4L2_BUF_TYPE_VIDEO_CAPTURE
    {
        let pix = unsafe { &mut *(fmt.fmt.raw.as_mut_ptr() as *mut V4l2PixFormat) };
        pix.width = req_w;
        pix.height = req_h;
        pix.pixelformat = 0x59565955; // UYVY
        pix.field = 1; // V4L2_FIELD_NONE
    }

    // VIDIOC_S_FMT = 0xc0cc5605 (type=1, sizeof=204)
    let sfmt_req = 0xc0cc5605u64;
    unsafe {
        ioctl_obj(fd, sfmt_req, &mut fmt).map_err(|e| format!("S_FMT: {e}"))?;
    }

    let (actual_w, actual_h, pixfmt, bytesperline, sizeimage) = unsafe {
        let pix = *(fmt.fmt.raw.as_ptr() as *const V4l2PixFormat);
        (
            pix.width,
            pix.height,
            pix.pixelformat,
            pix.bytesperline,
            pix.sizeimage,
        )
    };
    println!(
        "    format: {actual_w}x{actual_h} fourcc=0x{pixfmt:08x} bpl={bytesperline} size={sizeimage}"
    );

    // VIDIOC_REQBUFS — request 1 buffer
    #[repr(C)]
    struct V4l2Reqbuffers {
        count: u32,
        typ: u32,
        memory: u32,
        capabilities: u32,
        flags: u32,
        reserved: [u32; 2],
    }

    let mut reqbufs = V4l2Reqbuffers {
        count: 1,
        typ: 1,    // V4L2_BUF_TYPE_VIDEO_CAPTURE
        memory: 1, // V4L2_MEMORY_MMAP
        capabilities: 0,
        flags: 0,
        reserved: [0; 2],
    };
    // VIDIOC_REQBUFS = 0xc0145608
    unsafe {
        ioctl_obj(fd, 0xc0145608, &mut reqbufs).map_err(|e| format!("REQBUFS: {e}"))?;
    }
    if reqbufs.count == 0 {
        return Err("REQBUFS returned 0 buffers".into());
    }

    // VIDIOC_QUERYBUF — get buffer info
    #[repr(C)]
    struct V4l2Buffer {
        index: u32,
        typ: u32,
        bytes_used: u32,
        flags: u32,
        field: u32,
        timestamp: V4l2Timeval,
        timecode: V4l2Timecode,
        sequence: u32,
        memory: u32,
        m: V4l2BufferMem,
        length: u32,
        reserved2: u32,
    }

    #[repr(C)]
    struct V4l2Timeval {
        sec: i64,
        usec: i64,
    }

    #[repr(C)]
    struct V4l2Timecode {
        typ: u32,
        flags: u32,
        frames: u8,
        seconds: u8,
        minutes: u8,
        hours: u8,
        userbits: [u8; 4],
    }

    #[repr(C)]
    union V4l2BufferMem {
        offset: u32,
        userptr: u64,
        fd: i32,
    }

    let mut buf: V4l2Buffer = unsafe { std::mem::zeroed() };
    buf.index = 0;
    buf.typ = 1; // V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = 1; // V4L2_MEMORY_MMAP

    // VIDIOC_QUERYBUF = 0xc0585609
    unsafe {
        ioctl_obj(fd, 0xc0585609, &mut buf).map_err(|e| format!("QUERYBUF: {e}"))?;
    }

    let buf_offset = unsafe { buf.m.offset };
    let buf_len = buf.length;

    // mmap the buffer
    let mmapped = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            buf_len as usize,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            fd,
            buf_offset as i64,
        )
    };
    if mmapped == libc::MAP_FAILED {
        return Err("mmap failed".into());
    }

    // VIDIOC_QBUF — enqueue buffer
    let mut qbuf = buf;
    // VIDIOC_QBUF = 0xc058560f
    unsafe {
        ioctl_obj(fd, 0xc058560f, &mut qbuf).map_err(|e| {
            libc::munmap(mmapped, buf_len as usize);
            format!("QBUF: {e}")
        })?;
    }

    // VIDIOC_STREAMON
    let mut buf_type: u32 = 1; // V4L2_BUF_TYPE_VIDEO_CAPTURE
                               // VIDIOC_STREAMON = 0x40045612
    unsafe {
        ioctl_void(fd, 0x40045612, &mut buf_type as *mut _ as *mut libc::c_void).map_err(|e| {
            libc::munmap(mmapped, buf_len as usize);
            format!("STREAMON: {e}")
        })?;
    }

    // select() on fd with 2-second timeout
    let mut fds: libc::fd_set = unsafe { std::mem::zeroed() };
    unsafe {
        libc::FD_ZERO(&mut fds);
        libc::FD_SET(fd, &mut fds);
    }
    let mut timeout = libc::timeval {
        tv_sec: 2,
        tv_usec: 0,
    };
    let sel = unsafe {
        libc::select(
            fd + 1,
            &mut fds,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            &mut timeout,
        )
    };

    if sel <= 0 {
        // VIDIOC_STREAMOFF
        let mut bt2 = 1u32;
        unsafe {
            ioctl_void(fd, 0x40045613, &mut bt2 as *mut _ as *mut libc::c_void).ok();
        }
        unsafe {
            libc::munmap(mmapped, buf_len as usize);
        }
        return Err(format!("select timeout/error: {sel}"));
    }

    // VIDIOC_DQBUF
    let mut dqbuf: V4l2Buffer = unsafe { std::mem::zeroed() };
    dqbuf.typ = 1;
    dqbuf.memory = 1;
    // VIDIOC_DQBUF = 0xc0585611
    unsafe {
        ioctl_obj(fd, 0xc0585611, &mut dqbuf).map_err(|e| {
            let mut bt2 = 1u32;
            ioctl_void(fd, 0x40045613, &mut bt2 as *mut _ as *mut libc::c_void).ok();
            libc::munmap(mmapped, buf_len as usize);
            format!("DQBUF: {e}")
        })?;
    }

    let bytes_used = dqbuf.bytes_used as usize;

    // Copy captured data
    let captured = unsafe { std::slice::from_raw_parts(mmapped as *const u8, bytes_used).to_vec() };

    // Cleanup
    unsafe {
        let mut bt2 = 1u32;
        ioctl_void(fd, 0x40045613, &mut bt2 as *mut _ as *mut libc::c_void).ok(); // STREAMOFF
        libc::munmap(mmapped, buf_len as usize);
    }

    // Convert to Bayer u16 LE
    let bayer = match pixfmt {
        0x59565955 => uyvy_to_bayer_u16(&captured, actual_w, actual_h), // UYVY
        0x56595559 => yuyv_to_bayer_u16(&captured, actual_w, actual_h), // YUYV
        0x52474230 => rggb_to_bayer_u16(&captured, actual_w, actual_h), // RGGB8
        0x32305652 => vr20_to_bayer_u16(&captured, actual_w, actual_h), // VR20 (Android)
        _ => {
            // Generic: treat raw bytes as grayscale → bayer u16
            generic_to_bayer_u16(&captured, actual_w, actual_h)
        }
    };

    Ok((actual_w, actual_h, bayer))
}

// ---------------------------------------------------------------------------
// V4L2 ioctl helpers
// ---------------------------------------------------------------------------

unsafe fn ioctl_void(
    fd: std::os::unix::io::RawFd,
    request: u64,
    arg: *mut libc::c_void,
) -> Result<(), String> {
    let ret = libc::ioctl(fd, request as i32, arg);
    if ret < 0 {
        Err(std::io::Error::last_os_error().to_string())
    } else {
        Ok(())
    }
}

unsafe fn ioctl_obj<T>(
    fd: std::os::unix::io::RawFd,
    request: u64,
    obj: *mut T,
) -> Result<(), String> {
    let ret = libc::ioctl(fd, request as i32, obj as *mut libc::c_void);
    if ret < 0 {
        Err(std::io::Error::last_os_error().to_string())
    } else {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// YUV → Bayer u16 conversions
// ---------------------------------------------------------------------------

/// UYVY (4:2:2 packed): [U0 Y0 V0 Y1] — extract Y channel as bayer sample.
fn uyvy_to_bayer_u16(data: &[u8], w: u32, h: u32) -> Vec<u8> {
    let mut out = vec![0u8; (w * h * 2) as usize];
    for y in 0..h {
        for x in 0..w {
            let row_off = (y * w * 2) as usize;
            let px = x as usize;
            // UYVY: Y is at byte offset 1, 3, 5, 7...
            let src_idx = row_off + (px & !1) * 2 + 1 + (px & 1);
            let y_val = if src_idx < data.len() {
                data[src_idx]
            } else {
                0
            };
            let sample = (y_val as u16) * 64; // scale 8→16 bit
            let out_idx = ((y * w + x) * 2) as usize;
            out[out_idx] = (sample & 0xFF) as u8;
            out[out_idx + 1] = ((sample >> 8) & 0xFF) as u8;
        }
    }
    out
}

/// YUYV: [Y0 U0 Y1 V0] — extract Y channel.
fn yuyv_to_bayer_u16(data: &[u8], w: u32, h: u32) -> Vec<u8> {
    let mut out = vec![0u8; (w * h * 2) as usize];
    for y in 0..h {
        for x in 0..w {
            let row_off = (y * w * 2) as usize;
            let px = x as usize;
            // YUYV: Y is at byte offset 0, 2, 4, 6...
            let src_idx = row_off + px;
            let y_val = if src_idx < data.len() {
                data[src_idx]
            } else {
                0
            };
            let sample = (y_val as u16) * 64;
            let out_idx = ((y * w + x) * 2) as usize;
            out[out_idx] = (sample & 0xFF) as u8;
            out[out_idx + 1] = ((sample >> 8) & 0xFF) as u8;
        }
    }
    out
}

/// Raw RGGB8: each byte → u16 shifted left by 8.
fn rggb_to_bayer_u16(data: &[u8], w: u32, h: u32) -> Vec<u8> {
    let mut out = vec![0u8; (w * h * 2) as usize];
    for i in 0..(w * h) as usize {
        let val = if i < data.len() { data[i] } else { 0 };
        let sample = (val as u16) << 8;
        out[i * 2] = (sample & 0xFF) as u8;
        out[i * 2 + 1] = ((sample >> 8) & 0xFF) as u8;
    }
    out
}

/// VR20 (Android HAL): NV21-like or YUV → extract luminance.
fn vr20_to_bayer_u16(data: &[u8], w: u32, h: u32) -> Vec<u8> {
    let mut out = vec![0u8; (w * h * 2) as usize];
    for i in 0..(w * h) as usize {
        let y_val = if i < data.len() { data[i] } else { 0 };
        let sample = (y_val as u16) * 64;
        out[i * 2] = (sample & 0xFF) as u8;
        out[i * 2 + 1] = ((sample >> 8) & 0xFF) as u8;
    }
    out
}

/// Generic: treat every 2 bytes as a little-endian u16 sample.
fn generic_to_bayer_u16(data: &[u8], w: u32, h: u32) -> Vec<u8> {
    let pixels = (w * h) as usize;
    let mut out = vec![0u8; pixels * 2];
    for i in 0..pixels {
        let b0 = if i * 2 < data.len() { data[i * 2] } else { 0 };
        let b1 = if i * 2 + 1 < data.len() {
            data[i * 2 + 1]
        } else {
            0
        };
        let val = ((b1 as u16) << 8) | (b0 as u16);
        // Ensure non-zero for pipeline
        let sample = if val == 0 { 128u16 } else { val };
        out[i * 2] = (sample & 0xFF) as u8;
        out[i * 2 + 1] = ((sample >> 8) & 0xFF) as u8;
    }
    out
}

// ---------------------------------------------------------------------------
// /dev/video* scanner
// ---------------------------------------------------------------------------

fn scan_video_devices() -> Vec<String> {
    let mut devs = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/dev") {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("video") {
                devs.push(format!("/dev/{name}"));
            }
        }
    }
    devs.sort();
    devs
}

// ---------------------------------------------------------------------------
// Synthetic 4K rainbow Bayer generator
// ---------------------------------------------------------------------------

fn make_rainbow_bayer(w: u32, h: u32) -> Vec<u8> {
    let mut buf = vec![0u8; (w * h * 2) as usize];
    for y in 0..h {
        for x in 0..w {
            let val: u16 = match (x % 2, y % 2) {
                (0, 0) => ((x * 1023 / w) as u16).min(1023),
                (1, 0) | (0, 1) => ((y * 1023 / h) as u16).min(1023),
                _ => (((x + y) * 512 / (w + h)) as u16).min(1023),
            };
            let scaled = val * 64;
            let idx = ((y * w + x) * 2) as usize;
            buf[idx] = (scaled & 0xFF) as u8;
            buf[idx + 1] = ((scaled >> 8) & 0xFF) as u8;
        }
    }
    buf
}
