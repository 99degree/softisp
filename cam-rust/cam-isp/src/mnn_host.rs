//! MNN host-tensor inference — uses mnn_run_host_tensors from the C wrapper.
//! Handles copyFromHostTensor / copyToHostTensor internally.
//! No separate C++ compilation needed — lives in mnn_wrapper.cpp.

use std::ffi::CString;
use std::time::Instant;
use log::{info, debug, error};

extern "C" {
    fn mnn_interpreter_create_from_file(path: *const std::ffi::c_char) -> *mut std::ffi::c_void;
    fn mnn_session_create(interp: *mut std::ffi::c_void, backend: i32, num_threads: i32) -> *mut std::ffi::c_void;
    fn mnn_run_host_tensors(
        interp: *mut std::ffi::c_void, sess: *mut std::ffi::c_void,
        in_data: *const f32, in_shape: *const i32, in_ndim: i32,
        out_data: *mut f32, max_out: i32,
    ) -> i32;
    fn mnn_interpreter_destroy(interp: *mut std::ffi::c_void);
    fn mnn_session_release(interp: *mut std::ffi::c_void, sess: *mut std::ffi::c_void);
}

pub struct MnnHostModel {
    interp: *mut std::ffi::c_void,
    sess: *mut std::ffi::c_void,
}

unsafe impl Send for MnnHostModel {}
unsafe impl Sync for MnnHostModel {}

impl MnnHostModel {
    pub fn load(path: &str) -> Option<Self> {
        let c_path = CString::new(path).ok()?;
        let interp = unsafe { mnn_interpreter_create_from_file(c_path.as_ptr()) };
        if interp.is_null() { return None; }
        // MNN_BACKEND_CPU = 0
        let sess = unsafe { mnn_session_create(interp, 0, 4) };
        if sess.is_null() { unsafe { mnn_interpreter_destroy(interp); } return None; }
        Some(Self { interp, sess })
    }

    pub fn run(&self, input: &[f32], shape: &[i32]) -> Option<Vec<f32>> {
        let t0 = Instant::now();
        let mut output = vec![0.0f32; 4096];
        let n = unsafe {
            mnn_run_host_tensors(
                self.interp, self.sess,
                input.as_ptr(), shape.as_ptr(), shape.len() as i32,
                output.as_mut_ptr(), output.len() as i32,
            )
        };
        if n <= 0 {
            error!("MnnHostModel::run failed: {} (input shape {:?})", n, shape);
            return None;
        }
        output.truncate(n as usize);
        debug!("MnnHostModel::run {:?} {} outputs (shape {:?})", t0.elapsed(), n, shape);
        Some(output)
    }
}

impl Drop for MnnHostModel {
    fn drop(&mut self) {
        unsafe {
            if !self.sess.is_null() { mnn_session_release(self.interp, self.sess); }
            if !self.interp.is_null() { mnn_interpreter_destroy(self.interp); }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore] // requires MobileNetV2_224.mnn at known path + LD_LIBRARY_PATH=lib/aarch64
    fn test_mobilenet_host() {
        let path = "/data/data/com.termux/files/home/MNN/benchmark/models/MobileNetV2_224.mnn";
        let m = MnnHostModel::load(path).expect("load failed");
        let input = vec![0.0f32; 3 * 224 * 224];
        let out = m.run(&input, &[1, 3, 224, 224]).expect("run failed");
        assert_eq!(out.len(), 1001, "MobileNetV2 should output 1001 classes");
        info!("MNN host-tensor inference OK: {} outputs", out.len());
    }
}