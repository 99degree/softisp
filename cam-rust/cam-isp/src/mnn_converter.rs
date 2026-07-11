//! MNN ONNX-to-MNN conversion via MNNConvert subprocess.
//!
//! Uses MNNConvert binary as a subprocess to isolate MNN's
//! C++ global state from the inference runtime. This prevents
//! "pthread_mutex_lock on destroyed mutex" crashes caused by
//! the converter polluting MNN's global executor/backend state.

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

/// Convert ONNX model to MNN format via MNNConvert subprocess.
///
/// Uses MNNConvert binary (separate process) so the converter's MNN global
/// state does not pollute the inference runtime's MNN globals.
pub fn convert_onnx_to_mnn(
    onnx_path: &str,
    mnn_path: &str,
    options: Option<&MnnConvertOptions>,
) -> Result<String, String> {
    let opts = options.cloned().unwrap_or_default();

    let mnn_convert = find_mnn_convert_binary()?;

    let mut cmd = Command::new(&mnn_convert);
    cmd.arg("--framework").arg("ONNX")
        .arg("--modelFile").arg(onnx_path)
        .arg("--MNNModel").arg(mnn_path)
        .arg("--bizCode").arg(&opts.biz_code);

    if opts.fp16 {
        cmd.arg("--fp16");
    }
    if opts.weight_quant_bits > 0 {
        cmd.arg("--weightQuantBits").arg(opts.weight_quant_bits.to_string());
    }
    if opts.preserve_input_type {
        cmd.arg("--preserveInputType");
    }
    if !opts.use_gelu_approximation {
        cmd.arg("--geluApproximation");
    }

    let output = cmd.output()
        .map_err(|e| format!("Failed to run MNNConvert: {}", e))?;

    if output.status.success() {
        if std::path::Path::new(mnn_path).exists() {
            Ok(format!("OK: {} -> {}", onnx_path, mnn_path))
        } else {
            let combined = if output.stdout.is_empty() { &output.stderr } else { &output.stdout };
            Err(format!("MNNConvert succeeded but no output file: {}",
                String::from_utf8_lossy(combined)))
        }
    } else {
        let combined = if output.stderr.is_empty() { &output.stdout } else { &output.stderr };
        Err(format!("MNN conversion failed: {}",
            String::from_utf8_lossy(combined).trim()))
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
                exe_dir.join("..").join("..").join("tools").join("MNNConvert"),
                exe_dir.join("..").join("..").join("..").join("tools").join("MNNConvert"),
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
}
