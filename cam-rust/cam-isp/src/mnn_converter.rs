//! MNN ONNX-to-MNN conversion.
//!
//! Two paths:
//! 1. **Buffer API** (`convert_onnx_buffer`): pure in-memory via
//!    `libMNNConvertDeps.so`'s C-only `mnn_convert_onnx_to_mnn_buffer`
//!    (MNN MNNConvertBuffer.cpp). The converter parses the ONNX protobuf from
//!    the buffer, runs the full optimizeLevel=2 pipeline (incl. IspChainFusion)
//!    and returns MNN bytes — no temp files, no memfd, no subprocess.
//! 2. **Subprocess** (`convert_onnx_to_mnn`): spawns `MNNConvert` binary.
//!    Retained as fallback for offline/export tooling only.

use std::process::Command;

/// Conversion options for MNN converter
#[derive(Debug, Clone)]
pub struct MnnConvertOptions {
    pub biz_code: String,
    pub optimize_level: u8,
    pub fp16: bool,
    pub preserve_input_type: bool,
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
            preserve_input_type: false,
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

/// Convert ONNX bytes to MNN bytes in-process (zero disk writes, zero memfd).
///
/// Uses `libMNNConvertDeps.so` via the pure in-memory C API
/// `mnn_convert_onnx_to_mnn_buffer` (MNN's MNNConvertBuffer.cpp) — the
/// converter parses the ONNX protobuf from the buffer, runs the full
/// optimizeLevel=2 pipeline and returns malloc'd MNN bytes. Caller owns the
/// returned `Vec<u8>`.
#[cfg(feature = "mnn")]
pub fn convert_onnx_buffer(onnx_bytes: &[u8]) -> Result<Vec<u8>, String> {
    use crate::mnn_sys::mnn_convert_onnx_to_mnn_buffer;

    if onnx_bytes.is_empty() {
        return Err("ONNX bytes are empty".into());
    }

    let mut out_data: *mut std::ffi::c_void = std::ptr::null_mut();
    let mut out_size: usize = 0;
    let mut error_msg = [0 as std::os::raw::c_char; 1024usize];
    let rc = unsafe {
        mnn_convert_onnx_to_mnn_buffer(
            onnx_bytes.as_ptr() as *const std::ffi::c_void,
            onnx_bytes.len(),
            &mut out_data,
            &mut out_size,
            error_msg.as_mut_ptr(),
            error_msg.len(),
        )
    };
    if rc != 0 {
        let err_msg = unsafe { std::ffi::CStr::from_ptr(error_msg.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        return Err(format!("ONNX→MNN buffer convert failed: {}", err_msg));
    }
    if out_data.is_null() || out_size == 0 {
        return Err("ONNX→MNN buffer convert returned empty".into());
    }

    let slice = unsafe { std::slice::from_raw_parts(out_data as *const u8, out_size) };
    let mnn_bytes = slice.to_vec();
    unsafe {
        libc::free(out_data);
    }
    Ok(mnn_bytes)
}

/// Convert MNN bytes to MNN bytes in-process (framework=MNN).
///
/// Pass1 (B test): re-runs the Pass0 .mnn through the converter with
/// framework=MNN, which applies IspChainFusion — fusing primitive ops
/// (Conv/BinaryOp/Pool/...) into isp.* custom ops carrying pre-compiled
/// SPIR-V. Caller owns the returned `Vec<u8>`.
#[cfg(feature = "mnn")]
pub fn convert_mnn_buffer(mnn_bytes: &[u8]) -> Result<Vec<u8>, String> {
    use crate::mnn_sys::{mnn_convert_mnn_buffer, MnnConvertBufferResult, MnnConvert_FreeBuffer};

    if mnn_bytes.is_empty() {
        return Err("MNN bytes are empty".into());
    }

    let mut result = MnnConvertBufferResult {
        success: 0,
        error_msg: [0 as std::os::raw::c_char; 1024usize],
        data: std::ptr::null_mut(),
        size: 0,
    };
    unsafe {
        mnn_convert_mnn_buffer(
            mnn_bytes.as_ptr() as *const std::ffi::c_void,
            mnn_bytes.len(),
            &mut result,
        );
    }
    if result.success != 0 {
        let err_msg = unsafe { std::ffi::CStr::from_ptr(result.error_msg.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        unsafe { MnnConvert_FreeBuffer(&mut result) };
        return Err(format!("MNN→MNN buffer convert failed: {}", err_msg));
    }
    if result.data.is_null() || result.size == 0 {
        unsafe { MnnConvert_FreeBuffer(&mut result) };
        return Err("MNN→MNN buffer convert returned empty".into());
    }

    let slice = unsafe { std::slice::from_raw_parts(result.data as *const u8, result.size) };
    let mnn_out = slice.to_vec();
    unsafe {
        MnnConvert_FreeBuffer(&mut result);
    }
    Ok(mnn_out)
}

/// Dump MNN bytes to the flatbuffer-JSON text via MNN::Cli::mnn2json.
///
/// The JSON has one object per op with "type" (MNN OpType enum name,
/// e.g. "Conv", "BinaryOp") and, for custom ops, "main_type": "Extra"
/// with "main.type" = the isp.* string. The test parses this in Rust
/// (serde_json) to build an abstract node graph for Pass0/Pass1
/// comparison and to verify the B-test constraint: Pass1 MNN contains
/// ONLY isp.* custom opset operations.
#[cfg(feature = "mnn")]
pub fn dump_mnn_to_json(mnn_bytes: &[u8]) -> Result<String, String> {
    use crate::mnn_sys::{
        mnn_dump_mnn_to_json, MnnConvertBufferResult, MnnConvertJsonResult, MnnConvert_FreeBuffer,
    };

    if mnn_bytes.is_empty() {
        return Err("MNN bytes are empty".into());
    }

    let mut result = MnnConvertJsonResult {
        success: 0,
        error_msg: [0 as std::os::raw::c_char; 1024usize],
        data: std::ptr::null_mut(),
        size: 0,
    };
    unsafe {
        mnn_dump_mnn_to_json(
            mnn_bytes.as_ptr() as *const std::ffi::c_void,
            mnn_bytes.len(),
            &mut result,
        );
    }
    if result.success != 0 {
        let err_msg = unsafe { std::ffi::CStr::from_ptr(result.error_msg.as_ptr()) }
            .to_string_lossy()
            .into_owned();
        unsafe { MnnConvert_FreeBuffer(&mut result as *mut _ as *mut MnnConvertBufferResult) };
        return Err(format!("MNN→JSON dump failed: {}", err_msg));
    }
    if result.data.is_null() || result.size == 0 {
        unsafe { MnnConvert_FreeBuffer(&mut result as *mut _ as *mut MnnConvertBufferResult) };
        return Err("MNN→JSON dump returned empty".into());
    }

    let s = unsafe { std::ffi::CStr::from_ptr(result.data as *const std::os::raw::c_char) }
        .to_string_lossy()
        .into_owned();
    unsafe {
        MnnConvert_FreeBuffer(&mut result as *mut _ as *mut MnnConvertBufferResult);
    }
    Ok(s)
}

/// Fallback: Convert ONNX file to MNN file via MNNConvert subprocess.
///
/// Retained for offline/export tooling. Prefer `convert_onnx_buffer` for
/// all runtime paths — it avoids disk I/O, temp files, and subprocess overhead.
pub fn convert_onnx_to_mnn(
    onnx_path: &str,
    mnn_path: &str,
    options: Option<&MnnConvertOptions>,
) -> Result<String, String> {
    let opts = options.cloned().unwrap_or_default();

    let mnn_convert = find_mnn_convert_binary()?;

    let mut cmd = Command::new(&mnn_convert);
    cmd.arg("--framework")
        .arg("ONNX")
        .arg("--modelFile")
        .arg(onnx_path)
        .arg("--MNNModel")
        .arg(mnn_path)
        .arg("--bizCode")
        .arg(&opts.biz_code);

    if opts.fp16 {
        cmd.arg("--fp16");
    }
    if opts.weight_quant_bits > 0 {
        cmd.arg("--weightQuantBits")
            .arg(opts.weight_quant_bits.to_string());
    }
    if opts.preserve_input_type {
        cmd.arg("--preserveInputType");
    }
    if !opts.use_gelu_approximation {
        cmd.arg("--geluApproximation");
    }
    if opts.allow_custom_op {
        cmd.arg("--allowCustomOp");
    }
    if opts.transformer_fuse {
        cmd.arg("--transformerFuse");
    }

    let output = cmd
        .output()
        .map_err(|e| format!("Failed to run MNNConvert: {}", e))?;

    if output.status.success() {
        if std::path::Path::new(mnn_path).exists() {
            Ok(format!("OK: {} -> {}", onnx_path, mnn_path))
        } else {
            let combined = if output.stdout.is_empty() {
                &output.stderr
            } else {
                &output.stdout
            };
            Err(format!(
                "MNNConvert succeeded but no output file: {}",
                String::from_utf8_lossy(combined)
            ))
        }
    } else {
        let combined = if output.stderr.is_empty() {
            &output.stdout
        } else {
            &output.stderr
        };
        Err(format!(
            "MNN conversion failed: {}",
            String::from_utf8_lossy(combined).trim()
        ))
    }
}

/// Find the MNNConvert binary. Checks common locations.
fn find_mnn_convert_binary() -> Result<String, String> {
    // 1. Check PATH first
    if let Ok(output) = Command::new("which").arg("MNNConvert").output() {
        if output.status.success() {
            let path = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !path.is_empty() && std::path::Path::new(&path).exists() {
                return Ok(path);
            }
        }
    }

    // 2. Check well-known locations relative to current dir / project root
    let candidates = [
        "tools/MNNConvert",
        "target/release/tools/MNNConvert",
        "target/debug/tools/MNNConvert",
        "../tools/MNNConvert",
        "/data/data/com.termux/files/home/softisp/cam-rust/tools/MNNConvert",
        "/data/data/com.termux/files/home/softisp/tools/MNNConvert",
    ];
    for p in &candidates {
        if std::path::Path::new(p).exists() {
            return Ok(p.to_string());
        }
    }

    // 3. Check relative to executable
    if let Ok(exe_path) = std::env::current_exe() {
        if let Some(exe_dir) = exe_path.parent() {
            let rel_candidates = [
                exe_dir.join("MNNConvert"),
                exe_dir.join("tools").join("MNNConvert"),
                exe_dir.join("..").join("tools").join("MNNConvert"),
                exe_dir
                    .join("..")
                    .join("..")
                    .join("tools")
                    .join("MNNConvert"),
                exe_dir
                    .join("..")
                    .join("..")
                    .join("..")
                    .join("tools")
                    .join("MNNConvert"),
            ];
            for p in &rel_candidates {
                if p.exists() {
                    return Ok(p.to_string_lossy().to_string());
                }
            }
        }
    }

    Err("MNNConvert binary not found. Install or build MNNConvert and place it in PATH.".into())
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

    // Build a trivial ONNX model (identity) and verify the pure in-memory
    // buffer conversion returns valid MNN bytes that load as an interpreter —
    // no temp files, no memfd, no /proc/self/fd.
    #[cfg(feature = "mnn")]
    #[test]
    fn test_convert_onnx_buffer_roundtrip() {
        use crate::mnn_sys::MnnInterpreterSafe;
        use crate::onnx::proto::Proto;

        // Minimal real graph: input "x" -> Identity -> output "y" (1 node;
        // empty graphs are rejected by onnx2MNNNet).
        let nodes = vec![Proto::node("Identity", &["x"], &["y"], &[])];
        let dims = vec![
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(1),
        ];
        let inputs = vec![Proto::value_info("x", &dims, 1)];
        let outputs = vec![Proto::value_info("y", &dims, 1)];
        let init = Vec::new();
        let vi = Vec::new();
        let opset = Proto::opset("", 21);
        let graph = Proto::graph("buffer_test", &nodes, &inputs, &outputs, &init, &vi);
        let onnx = Proto::model(9, &opset, "buffer_test", &graph);

        let mnn = convert_onnx_buffer(&onnx).expect("buffer ONNX→MNN conversion should succeed");
        assert!(!mnn.is_empty(), "expected non-empty MNN bytes");

        let interp =
            MnnInterpreterSafe::from_buffer(&mnn).expect("MNN interpreter should load from buffer");
        drop(interp);
    }
}
