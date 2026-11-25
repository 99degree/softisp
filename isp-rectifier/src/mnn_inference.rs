//! MNN-based inference backend for the ISP Rectifier distilled model.
//!
//! This module replaces `tract-onnx` with MNN's C FFI loaded through
//! `mnn_sys`. The flow is:
//!
//! 1. ONNX model bytes (from `rectifier_model::generate_rectifier_model()` or
//!    a pre-trained `.onnx` file) are converted to MNN flatbuffer bytes via
//!    `mnn_convert_onnx_to_mnn_buffer` (libMNNConvertDeps.so, in-process).
//! 2. An `MnnInterpreter` is created from the MNN buffer.
//! 3. A Vulkan (or CPU) session is created per model.
//! 4. Per-frame, the two inputs (hist [1,256] f32 + meta [1,52] f32) are
//!    uploaded via `mnn_set_input_float`, then `mnn_session_run` executes.
//! 5. The four outputs (wb [1,3], ccm [1,9], tone [1,7], zoom [1,1]) are
//!    read back via `mnn_session_read_output`.
//!
//! This keeps `isp-rectifier` free of the `tract-onnx` dependency tree when
//! the `mnn` feature is enabled — the MNN native libs are already bundled
//! with the APK.

use crate::mnn_sys::{MnnBackendType, MnnInterpreter, MnnSession};
use crate::types::{FrameMetadata, ISPOptimizedParams};
use std::path::Path;
use std::ptr;

/// Output names declared by the rectifier ONNX graph (must match the
/// `Proto::value_info` output names in `rectifier_model.rs`).
const OUTPUT_WB: &str = "wb";
const OUTPUT_CCM: &str = "ccm";
const OUTPUT_TONE: &str = "tone";
const OUTPUT_ZOOM: &str = "zoom";

/// Input names declared by the rectifier ONNX graph.
const INPUT_HIST: &str = "hist";
const INPUT_META: &str = "meta";

/// MNN inference engine for the rectifier model.
///
/// The model is loaded once (ONNX → MNN conversion + session creation) and
/// reused for every frame. Inputs/outputs are uploaded/read back per frame.
pub struct MnnInference {
    interp: MnnInterpreter,
    sess: MnnSession,
    clamp: bool,
}

impl MnnInference {
    /// Create from ONNX model bytes (converts to MNN in-process, creates a
    /// Vulkan session).
    pub fn from_onnx_bytes(model_bytes: &[u8], clamp: bool) -> Result<Self, String> {
        use crate::mnn_sys::mnn_convert_onnx_to_mnn_buffer;
        use libc;

        // Step 1: ONNX → MNN conversion (in-process, no temp files).
        let mut out_data: *mut std::ffi::c_void = ptr::null_mut();
        let mut out_size: usize = 0;
        let mut error_msg: [std::os::raw::c_char; 1024] = [0; 1024];
        let rc = unsafe {
            mnn_convert_onnx_to_mnn_buffer(
                model_bytes.as_ptr() as *const std::ffi::c_void,
                model_bytes.len(),
                &mut out_data,
                &mut out_size,
                error_msg.as_mut_ptr() as *mut std::os::raw::c_char,
                error_msg.len(),
            )
        };
        if rc != 0 {
            let err = unsafe { std::ffi::CStr::from_ptr(error_msg.as_ptr() as *const std::os::raw::c_char) }
                .to_string_lossy()
                .into_owned();
            return Err(format!("ONNX→MNN convert failed: {}", err));
        }
        if out_data.is_null() || out_size == 0 {
            return Err("ONNX→MNN convert returned empty".into());
        }

        let mnn_slice = unsafe { std::slice::from_raw_parts(out_data as *const u8, out_size) };
        let mnn_bytes = mnn_slice.to_vec();
        unsafe {
            libc::free(out_data);
        }

        // Step 2: Create interpreter from MNN buffer.
        let interp = MnnInterpreter::from_buffer(&mnn_bytes)
            .ok_or_else(|| "Failed to create MNN interpreter from buffer".to_string())?;

        // Step 3: Create a Vulkan session.
        let sess = interp
            .create_session(MnnBackendType::Vulkan, 4, false)
            .ok_or_else(|| "Failed to create MNN Vulkan session".to_string())?;

        // Resize session for the declared input shapes so MNN allocates
        // device buffers up front.
        // Input 0: hist [1, 256] f32
        let hist_t = interp.get_input(&sess, INPUT_HIST);
        if let Some(t) = hist_t {
            let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &[1i32, 256]);
        }
        // Input 1: meta [1, 52] f32
        let meta_t = interp.get_input(&sess, INPUT_META);
        if let Some(t) = meta_t {
            let _ = t.set_shape(interp.as_ptr(), sess.as_ptr(), &[1i32, 52]);
        }

        let resize_rc = unsafe {
            crate::mnn_sys::mnn_session_resize(
                interp.interpreter_ptr(),
                sess.as_ptr(),
            )
        };
        if resize_rc != 0 {
            return Err(format!("MNN session resize failed: rc={resize_rc}"));
        }

        Ok(Self {
            interp,
            sess,
            clamp,
        })
    }

    /// Create from an ONNX model file path.
    pub fn new<P: AsRef<Path>>(model_path: P, clamp: bool) -> Result<Self, String> {
        Self::from_file(model_path, clamp)
    }

    /// Create from an ONNX model file path (alias for `new`).
    pub fn from_file<P: AsRef<Path>>(model_path: P, clamp: bool) -> Result<Self, String> {
        let bytes = std::fs::read(model_path).map_err(|e| format!("Read model: {}", e))?;
        Self::from_onnx_bytes(&bytes, clamp)
    }

    /// Run inference: hist [256] + meta [52] → ISPOptimizedParams.
    pub fn run(
        &self,
        histogram: &[f32],
        metadata: &[f32],
    ) -> Result<ISPOptimizedParams, String> {
        // Upload input 0: hist [1, 256] f32.
        let hist_t = self
            .interp
            .get_input(&self.sess, INPUT_HIST)
            .ok_or_else(|| "Input tensor 'hist' not found".to_string())?;
        if !hist_t.upload_f32_shaped(&[1, 256], histogram) {
            return Err("Failed to upload hist input".to_string());
        }

        // Upload input 1: meta [1, 52] f32.
        let meta_t = self
            .interp
            .get_input(&self.sess, INPUT_META)
            .ok_or_else(|| "Input tensor 'meta' not found".to_string())?;
        if !meta_t.upload_f32_shaped(&[1, 52], metadata) {
            return Err("Failed to upload meta input".to_string());
        }

        // Run inference.
        let run_rc = unsafe {
            crate::mnn_sys::mnn_session_run(
                self.interp.interpreter_ptr(),
                self.sess.as_ptr(),
            )
        };
        if run_rc != 0 {
            let err = unsafe { crate::mnn_sys::mnn_last_error() };
            let err = if err.is_null() {
                String::new()
            } else {
                unsafe { std::ffi::CStr::from_ptr(err) }.to_string_lossy().into_owned()
            };
            return Err(format!("MNN session run failed (rc={run_rc}): {err}"));
        }

        // Read outputs.
        let read_f32 = |name: &str, count: usize| -> Result<Vec<f32>, String> {
            let mut buf = vec![0.0f32; count];
            let n = self.interp.read_output_floats(&self.sess, name, &mut buf);
            match n {
                Some(n) if n == count => Ok(buf),
                Some(n) => Err(format!(
                    "Output '{name}' returned {n} elements, expected {count}"
                )),
                None => Err(format!(
                    "Output tensor '{}' not found or empty", name
                )),
            }
        };

        let wb = read_f32(OUTPUT_WB, 3)?;
        let ccm = read_f32(OUTPUT_CCM, 9)?;
        let tone = read_f32(OUTPUT_TONE, 7)?;
        let zoom = read_f32(OUTPUT_ZOOM, 1)?;

        let mut params = ISPOptimizedParams {
            wb_r_gain: wb[0],
            wb_g_gain: wb[1],
            wb_b_gain: wb[2],
            ccm: [
                [ccm[0], ccm[1], ccm[2]],
                [ccm[3], ccm[4], ccm[5]],
                [ccm[6], ccm[7], ccm[8]],
            ],
            tone_curve_lut: tone,
            zoom_factor: zoom[0],
        };

        if self.clamp {
            params.clamp(&crate::types::RegisterLimits::default());
        }

        Ok(params)
    }

    /// High-level: run inference from `FrameMetadata`, normalizing histogram
    /// and building the 52-dim metadata vector.
    pub fn optimize(&self, frame: &FrameMetadata) -> Result<ISPOptimizedParams, String> {
        let (hist, meta) = frame.to_feature_vector();
        self.run(&hist, &meta)
    }
}
