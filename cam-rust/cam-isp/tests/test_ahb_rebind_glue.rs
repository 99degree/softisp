#![cfg(all(feature = "mnn", target_os = "android"))]
//! On-device regression for AHB-backed MNN session inputs.
//!
//! The test uses an INT32 Add graph so a BLOB AHardwareBuffer is treated as an
//! opaque byte container rather than an implied u8 or float tensor. The AHB is
//! bound to the session input before resize, the session is resized once, and
//! the normal inference entry point runs without resizing or uploading host
//! bytes. Add keeps the op real (Identity would be optimized away), so a
//! successful run proves the session-owned tensor reaches Vulkan execution.
//! Reading the AHB bytes from inside the shader is a separate execution detail
//! and is deliberately not asserted here.
//!
//! ```text
//! LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
//!   cargo test -p cam-isp --test test_ahb_rebind_glue --features mnn \
//!   -- --ignored --nocapture --test-threads=1
//! ```

use cam_isp::mnn_converter::convert_onnx_buffer;
use cam_isp::mnn_sys::{
    mnn_ahb_capacity_bytes, mnn_import_ahb_as_device_memory, mnn_run_true_zero_copy,
    MnnBackendType, MnnInterpreterSafe, MnnSessionSafe,
};
use cam_isp::onnx::proto::Proto;
use std::ffi::CString;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::os::raw::{c_char, c_int, c_uint, c_void};
use std::ptr::null_mut;
use std::slice;

const W: u32 = 16;
const H: u32 = 16;
const ELEMS: usize = (W as usize) * (H as usize);
const FORMAT_BLOB: c_uint = 0x21;
const FORMAT_RAW16: c_uint = 0x20;
const USAGE_CPU_RW: u64 = 3 | (3 << 4);
const USAGE_GPU_SAMPLED_IMAGE: u64 = 1 << 8;
const USAGE_GPU_DATA_BUFFER: u64 = 1 << 24;
const USAGE_CAMERA_WRITE: u64 = 1 << 17;
const ADD_BIAS: i32 = 1;
const MNN_MEMORY_AHARDWAREBUFFER: i32 = 14;
const MNN_FORWARD_VULKAN: i32 = 7;
const HALIDE_TYPE_INT: i32 = 0;
const ONNX_INT32: i32 = 6;

#[repr(C)]
#[derive(Default)]
struct AhbDesc {
    width: u32,
    height: u32,
    layers: u32,
    format: u32,
    usage: u64,
    stride: u32,
    rfu0: u32,
    rfu1: u64,
}

#[link(name = "android")]
extern "C" {
    fn AHardwareBuffer_allocate(desc: *const AhbDesc, out: *mut *mut c_void) -> c_int;
    fn AHardwareBuffer_release(buffer: *mut c_void);
    fn AHardwareBuffer_lock(
        buffer: *mut c_void,
        usage: u64,
        fence: i32,
        rect: *const c_void,
        out: *mut *mut c_void,
    ) -> c_int;
    fn AHardwareBuffer_unlock(buffer: *mut c_void, fence: *mut i32) -> c_int;
}

struct DummyAhb(*mut c_void);

impl DummyAhb {
    fn allocate_blob_bytes(bytes: u32) -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: bytes,
                    height: 1,
                    layers: 1,
                    format: FORMAT_BLOB,
                    usage: USAGE_CPU_RW | USAGE_GPU_DATA_BUFFER,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(rc, 0, "device should allocate a GPU_DATA_BUFFER BLOB AHB");
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn allocate_blob() -> Self {
        Self::allocate_blob_bytes((ELEMS * 4) as u32)
    }

    /// RAW16 private format with the usage the camera reader must request for
    /// the Vulkan `VkBuffer` import. Width x height gives exactly one packed
    /// INT32 word per graph element.
    fn allocate_raw16_sampled() -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: W,
                    height: H * 2,
                    layers: 1,
                    format: FORMAT_RAW16,
                    usage: USAGE_CPU_RW | USAGE_GPU_SAMPLED_IMAGE,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(
            rc, 0,
            "device should allocate a GPU_SAMPLED_IMAGE RAW16 AHB"
        );
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn allocate_raw16_live_usage() -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: 1600,
                    height: 1200,
                    layers: 1,
                    format: FORMAT_RAW16,
                    usage: 3 | USAGE_GPU_SAMPLED_IMAGE | USAGE_CAMERA_WRITE,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(rc, 0, "device should allocate a live-usage RAW16 AHB");
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn allocate_raw16_live_geometry() -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: 1600,
                    height: 1200,
                    layers: 1,
                    format: FORMAT_RAW16,
                    usage: USAGE_CPU_RW | USAGE_GPU_SAMPLED_IMAGE | USAGE_CAMERA_WRITE,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(rc, 0, "device should allocate a live-geometry RAW16 AHB");
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn allocate_raw16_camera_usage() -> Self {
        let mut ptr = null_mut();
        let rc = unsafe {
            AHardwareBuffer_allocate(
                &AhbDesc {
                    width: W,
                    height: H * 2,
                    layers: 1,
                    format: FORMAT_RAW16,
                    usage: USAGE_CPU_RW | USAGE_GPU_SAMPLED_IMAGE | USAGE_CAMERA_WRITE,
                    ..Default::default()
                },
                &mut ptr,
            )
        };
        assert_eq!(rc, 0, "device should allocate a camera-usage RAW16 AHB");
        assert!(!ptr.is_null());
        Self(ptr)
    }

    fn as_ptr(&self) -> *mut c_void {
        self.0
    }
}

impl Drop for DummyAhb {
    fn drop(&mut self) {
        if !self.0.is_null() {
            unsafe { AHardwareBuffer_release(self.0) };
        }
    }
}

fn raw_word(index: usize, base: u32) -> u32 {
    let high = base.wrapping_add(index as u32) & 0xffff;
    let low = 0x8000u32.wrapping_add(index as u32) & 0xffff;
    (high << 16) | low
}

fn high16(word: u32) -> u16 {
    (word >> 16) as u16
}

fn low16(word: u32) -> u16 {
    word as u16
}

fn lock_ahb(ahb: &DummyAhb) -> *mut u32 {
    let mut base = null_mut();
    assert_eq!(
        unsafe { AHardwareBuffer_lock(ahb.as_ptr(), USAGE_CPU_RW, -1, null_mut(), &mut base) },
        0,
        "dummy AHB should accept a CPU lock"
    );
    assert!(!base.is_null());
    base as *mut u32
}

fn unlock_ahb(ahb: &DummyAhb) -> Option<OwnedFd> {
    let mut fence = -1;
    assert_eq!(
        unsafe { AHardwareBuffer_unlock(ahb.as_ptr(), &mut fence) },
        0
    );
    (fence >= 0).then(|| unsafe { OwnedFd::from_raw_fd(fence) })
}

fn wait_fence(fence: &OwnedFd) {
    let mut pollfd = libc::pollfd {
        fd: fence.as_raw_fd(),
        events: 0,
        revents: 0,
    };
    assert!(unsafe { libc::poll(&mut pollfd, 1, -1) } >= 0);
}

fn fill_blob(ahb: &DummyAhb, base: u32) {
    let words = unsafe { slice::from_raw_parts_mut(lock_ahb(ahb), ELEMS) };
    for (index, slot) in words.iter_mut().enumerate() {
        *slot = raw_word(index, base);
    }
    if let Some(fence) = unlock_ahb(ahb) {
        wait_fence(&fence);
    }
}

fn check_blob(ahb: &DummyAhb, base: u32) {
    let mut mapped = null_mut();
    assert_eq!(
        unsafe { AHardwareBuffer_lock(ahb.as_ptr(), USAGE_CPU_RW, -1, null_mut(), &mut mapped) },
        0
    );
    let words = unsafe { slice::from_raw_parts(mapped as *const u32, ELEMS) };
    for (index, &value) in words.iter().enumerate() {
        assert_eq!(value, raw_word(index, base), "AHB word {index} mismatch");
    }
    if let Some(fence) = unlock_ahb(ahb) {
        wait_fence(&fence);
    }
}

fn make_int_add_session() -> (MnnInterpreterSafe, MnnSessionSafe, CString, CString) {
    let node = Proto::node("Add", &["x", "bias"], &["y"], &[]);
    let dims = vec![
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(1),
        Proto::tensor_dim_value(H as i64),
        Proto::tensor_dim_value(W as i64),
    ];
    let mut bias_raw = Vec::with_capacity(ELEMS * 4);
    for _ in 0..ELEMS {
        bias_raw.extend_from_slice(&ADD_BIAS.to_le_bytes());
    }
    let bias_dims = [1i64, 1, H as i64, W as i64];
    let bias = Proto::tensor_proto_raw_bytes("bias", &bias_raw, ONNX_INT32, &bias_dims);
    let graph = Proto::graph(
        "ahb_session_input_test",
        &[node],
        &[Proto::value_info("x", &dims, ONNX_INT32)],
        &[Proto::value_info("y", &dims, ONNX_INT32)],
        &[bias],
        &[],
    );
    let onnx = Proto::model(9, &Proto::opset("", 21), "ahb_session_input_test", &graph);
    let mnn = convert_onnx_buffer(&onnx).expect("in-memory ONNX→MNN conversion");
    let interp = MnnInterpreterSafe::from_buffer(&mnn).expect("MNN interpreter");
    let sess = interp
        .create_session(MnnBackendType::Vulkan, 1, true)
        .expect("Vulkan session");
    let x = CString::new(interp.get_input_name(&sess, 0).expect("input name")).unwrap();
    let y = CString::new("y").unwrap();
    (interp, sess, x, y)
}

fn set_input_shape(interp: &MnnInterpreterSafe, sess: &MnnSessionSafe) {
    let shape = [1, 1, H as i32, W as i32];
    interp
        .get_first_input(sess)
        .expect("session input tensor")
        .set_shape(interp.as_ptr(), sess.as_ptr(), &shape)
        .expect("set input shape");
}

fn assert_ahb_binding(interp: &MnnInterpreterSafe, sess: &MnnSessionSafe, ahb: *mut c_void) {
    let tensor = interp.get_first_input(sess).expect("session input tensor");
    let (ptr, memory_type) = tensor.device_binding().expect("device binding query");
    assert_eq!(ptr, ahb, "session input is not bound to the supplied AHB");
    assert_eq!(
        memory_type, MNN_MEMORY_AHARDWAREBUFFER,
        "session input lost its AHB memory type"
    );
}

fn assert_add_output(output: &[u32], base: u32) {
    for (index, &value) in output.iter().enumerate() {
        let source = raw_word(index, base);
        let expected = source.wrapping_add(ADD_BIAS as u32);
        assert_eq!(
            value, expected,
            "Vulkan Add did not read input word {index}"
        );
        assert_eq!(high16(value), high16(source), "packed high sample changed");
        assert_eq!(
            low16(value),
            low16(source).wrapping_add(ADD_BIAS as u16),
            "packed low sample did not change by the bias"
        );
    }
}

fn run_add(
    interp: &MnnInterpreterSafe,
    sess: &MnnSessionSafe,
    x: &CString,
    y: &CString,
    host: &[u32],
    base: u32,
    verify_values: bool,
) {
    assert_eq!(sess.get_actual_backend(), MNN_FORWARD_VULKAN);
    let shape = [1, 1, H as i32, W as i32];
    let mut output = vec![0u32; ELEMS];
    let n = unsafe {
        mnn_run_true_zero_copy(
            interp.as_ptr(),
            sess.as_ptr(),
            x.as_ptr(),
            y.as_ptr(),
            host.as_ptr() as *const c_void,
            HALIDE_TYPE_INT,
            32,
            shape.as_ptr(),
            shape.len() as i32,
            output.as_mut_ptr() as *mut f32,
            output.len() as i32,
        )
    };
    assert_eq!(n, ELEMS as i32);
    if verify_values {
        assert_add_output(&output, base);
    }
}

fn run_add_with_host(
    interp: &MnnInterpreterSafe,
    sess: &MnnSessionSafe,
    x: &CString,
    y: &CString,
    host: &[u32],
    base: u32,
) {
    run_add(interp, sess, x, y, host, base, true);
}

fn run_add_vulkan_path(
    interp: &MnnInterpreterSafe,
    sess: &MnnSessionSafe,
    x: &CString,
    y: &CString,
) {
    let ignored_host = vec![0u32; ELEMS];
    run_add(interp, sess, x, y, &ignored_host, 0, false);
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn int32_ahb_session_input_reaches_vulkan_without_host_upload() {
    let ahb = DummyAhb::allocate_blob();
    fill_blob(&ahb, 0x1000);
    check_blob(&ahb, 0x1000);
    let mut described = 0u64;
    assert_eq!(
        unsafe { mnn_ahb_capacity_bytes(ahb.as_ptr(), &mut described) },
        0
    );
    assert_eq!(described, (ELEMS * 4) as u64);

    let (interp, sess, x, y) = make_int_add_session();
    let input_handle = interp.get_first_input(&sess).expect("input").handle();
    set_input_shape(&interp, &sess);
    assert_eq!(
        unsafe {
            mnn_import_ahb_as_device_memory(
                interp.as_ptr(),
                sess.as_ptr(),
                ahb.as_ptr(),
                x.as_ptr(),
                1,
            )
        },
        0,
        "AHB should bind before resize"
    );
    sess.resize().expect("resize with AHB-bound input");

    let after = interp.get_first_input(&sess).expect("input after resize");
    assert_eq!(
        after.handle(),
        input_handle,
        "resize replaced the session-owned input tensor"
    );
    assert_ahb_binding(&interp, &sess, ahb.as_ptr());
    run_add_vulkan_path(&interp, &sess, &x, &y);
    assert_ahb_binding(&interp, &sess, ahb.as_ptr());
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn int32_session_input_ahb_can_be_rebound_without_resize() {
    let first = DummyAhb::allocate_blob();
    let second = DummyAhb::allocate_blob();
    fill_blob(&first, 0x1000);
    fill_blob(&second, 0x9000);

    let (interp, sess, x, y) = make_int_add_session();
    let input_handle = interp.get_first_input(&sess).expect("input").handle();
    set_input_shape(&interp, &sess);
    assert_eq!(
        unsafe {
            mnn_import_ahb_as_device_memory(
                interp.as_ptr(),
                sess.as_ptr(),
                first.as_ptr(),
                x.as_ptr(),
                1,
            )
        },
        0
    );
    sess.resize().expect("resize with AHB-bound input");
    assert_ahb_binding(&interp, &sess, first.as_ptr());

    sess.rebind_ahb(second.as_ptr(), x.as_ptr() as *const c_char, 0)
        .expect("targeted AHB rebind");
    let rebound = interp.get_first_input(&sess).expect("input after rebind");
    assert_eq!(
        rebound.handle(),
        input_handle,
        "rebind replaced the session tensor"
    );
    assert_ahb_binding(&interp, &sess, second.as_ptr());
    run_add_vulkan_path(&interp, &sess, &x, &y);

    sess.rebind_ahb(null_mut(), x.as_ptr() as *const c_char, 0)
        .expect("clear AHB binding");
    let (_ptr, memory_type) = rebound.device_binding().expect("device binding query");
    assert_ne!(
        memory_type, MNN_MEMORY_AHARDWAREBUFFER,
        "AHB flag was not cleared"
    );
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn int32_host_upload_add_control() {
    let host: Vec<u32> = (0..ELEMS).map(|index| raw_word(index, 0x1000)).collect();
    let (interp, sess, x, y) = make_int_add_session();
    set_input_shape(&interp, &sess);
    sess.resize().expect("resize for host control");
    run_add_with_host(&interp, &sess, &x, &y, &host, 0x1000);
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn raw16_live_usage_ahb_import_probe() {
    let ahb = DummyAhb::allocate_raw16_live_usage();
    let (interp, sess, x, _y) = make_int_add_session();
    set_input_shape(&interp, &sess);
    let rc = unsafe {
        mnn_import_ahb_as_device_memory(interp.as_ptr(), sess.as_ptr(), ahb.as_ptr(), x.as_ptr(), 1)
    };
    println!("live-usage RAW16 import rc={rc}");
    assert_eq!(rc, 0, "live-usage RAW16 AHB should import like the fixture");
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn raw16_live_geometry_ahb_import_probe() {
    let ahb = DummyAhb::allocate_raw16_live_geometry();
    let (interp, sess, x, _y) = make_int_add_session();
    set_input_shape(&interp, &sess);
    let rc = unsafe {
        mnn_import_ahb_as_device_memory(interp.as_ptr(), sess.as_ptr(), ahb.as_ptr(), x.as_ptr(), 1)
    };
    println!("live-geometry RAW16 import rc={rc}");
    assert_eq!(
        rc, 0,
        "live-geometry RAW16 AHB should import like the fixture"
    );
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn raw16_camera_usage_ahb_import_probe() {
    let ahb = DummyAhb::allocate_raw16_camera_usage();
    fill_blob(&ahb, 0x1000);
    let (interp, sess, x, _y) = make_int_add_session();
    set_input_shape(&interp, &sess);
    let rc = unsafe {
        mnn_import_ahb_as_device_memory(interp.as_ptr(), sess.as_ptr(), ahb.as_ptr(), x.as_ptr(), 1)
    };
    println!("camera-usage RAW16 import rc={rc}");
    assert_eq!(rc, 0, "camera-usage RAW16 AHB should import like a fixture");
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn raw16_sampled_image_ahb_binds_as_vulkan_buffer_input() {
    let ahb = DummyAhb::allocate_raw16_sampled();
    fill_blob(&ahb, 0x1000);

    let (interp, sess, x, y) = make_int_add_session();
    let input_handle = interp.get_first_input(&sess).expect("input").handle();
    set_input_shape(&interp, &sess);
    assert_eq!(
        unsafe {
            mnn_import_ahb_as_device_memory(
                interp.as_ptr(),
                sess.as_ptr(),
                ahb.as_ptr(),
                x.as_ptr(),
                1,
            )
        },
        0,
        "RAW16 GPU_SAMPLED_IMAGE AHB should bind like a BLOB"
    );
    sess.resize().expect("resize with RAW16 AHB-bound input");

    let after = interp.get_first_input(&sess).expect("input after resize");
    assert_eq!(
        after.handle(),
        input_handle,
        "resize replaced session tensor"
    );
    assert_ahb_binding(&interp, &sess, ahb.as_ptr());
    run_add_vulkan_path(&interp, &sess, &x, &y);
    assert_ahb_binding(&interp, &sess, ahb.as_ptr());
}

#[test]
#[ignore = "requires MNN Vulkan libraries on an Android device"]
fn undersized_blob_ahb_is_rejected() {
    let ahb = DummyAhb::allocate_blob_bytes((ELEMS * 4 - 4) as u32);
    let (interp, sess, x, _y) = make_int_add_session();
    set_input_shape(&interp, &sess);
    let rc = unsafe {
        mnn_import_ahb_as_device_memory(interp.as_ptr(), sess.as_ptr(), ahb.as_ptr(), x.as_ptr(), 1)
    };
    assert!(rc < 0, "undersized BLOB AHB must not bind to the tensor");
    let (_ptr, memory_type) = interp
        .get_first_input(&sess)
        .expect("input")
        .device_binding()
        .expect("device binding query");
    assert_ne!(memory_type, MNN_MEMORY_AHARDWAREBUFFER);
}
