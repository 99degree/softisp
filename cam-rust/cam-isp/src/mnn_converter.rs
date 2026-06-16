//! MNN ONNX-to-MNN conversion via static C FFI (no subprocess).
//!
//! Calls mnn_convert_onnx_to_mnn() from mnn_convert_api.cpp, which is
//! statically linked into the final binary via build.rs + cc::Build.
//! No MNNConvert subprocess needed.

use std::ffi::CString;
use crate::mnn_sys::{MnnConvertResult, mnn_convert_onnx_to_mnn};

/// Conversion options for MNN converter
#[derive(Debug, Clone)]
pub struct MnnConvertOptions {
    pub biz_code: String,
    pub optimize_level: u8,
    pub fp16: bool,
    pub weight_quant_bits: u8,
    pub weight_quant_block: i32,
    pub save_static_model: bool,
    pub target_version: f32,
    pub transformer_fuse: bool,
    pub allow_custom_op: bool,
    pub use_gelu_approximation: bool,
    pub input_config_file: Option<String>,
}

impl Default for MnnConvertOptions {
    fn default() -> Self {
        MnnConvertOptions {
            biz_code: "MNN".to_string(),
            optimize_level: 1,
            fp16: false,
            weight_quant_bits: 0,
            weight_quant_block: -1,
            save_static_model: false,
            target_version: 0.0,
            transformer_fuse: false,
            allow_custom_op: false,
            use_gelu_approximation: true,
            input_config_file: None,
        }
    }
}

/// Convert ONNX model to MNN format via static C FFI.
///
/// No subprocess, no temp files — calls MNN::Cli::convertModel directly
/// through the statically linked mnn_convert_api.cpp wrapper.
pub fn convert_onnx_to_mnn(
    onnx_path: &str,
    mnn_path: &str,
    options: Option<&MnnConvertOptions>,
) -> Result<String, String> {
    let opts = options.cloned().unwrap_or_default();

    let c_onnx = CString::new(onnx_path).map_err(|_| "NUL in onnx_path")?;
    let c_mnn  = CString::new(mnn_path).map_err(|_| "NUL in mnn_path")?;
    let c_biz  = CString::new(opts.biz_code.as_str()).map_err(|_| "NUL in biz_code")?;

    let mut result = MnnConvertResult {
        success: 0,
        error_msg: [0; 1024],
    };

    unsafe {
        mnn_convert_onnx_to_mnn(
            c_onnx.as_ptr(),
            c_mnn.as_ptr(),
            c_biz.as_ptr(),
            opts.optimize_level as i32,
            opts.weight_quant_bits as i32,
            if opts.fp16 { 1 } else { 0 },
            &mut result,
        );
    }

    if result.success == 0 {
        Ok(format!("OK: {} -> {}", onnx_path, mnn_path))
    } else {
        // Extract error message from C string (null-terminated)
        let msg = unsafe {
            let ptr = result.error_msg.as_ptr() as *const u8;
            let mut len = 0usize;
            while len < 1024 && *ptr.add(len) != 0 { len += 1; }
            String::from_utf8_lossy(std::slice::from_raw_parts(ptr, len)).to_string()
        };
        Err(format!("MNN conversion failed: {}", msg))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_options() {
        let opts = MnnConvertOptions::default();
        assert_eq!(opts.optimize_level, 1);
        assert!(!opts.fp16);
        assert_eq!(opts.biz_code, "MNN");
    }
}