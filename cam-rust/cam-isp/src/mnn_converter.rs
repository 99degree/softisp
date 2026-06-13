//! MNN ONNX-to-MNN converter via CLI tool (MNNConvert)

use std::path::Path;
use std::process::Command;

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

/// Convert ONNX model to MNN format using the MNNConvert CLI tool
pub fn convert_onnx_to_mnn(
    onnx_path: &str,
    mnn_path: &str,
    options: Option<&MnnConvertOptions>,
) -> Result<String, String> {
    let opts = options.cloned().unwrap_or_default();
    
    // Find MNNConvert binary
    let mnn_convert = find_mnn_convert()?;
    
    let mut cmd = Command::new(&mnn_convert);
    cmd.arg("-f").arg("ONNX")
       .arg("--modelFile").arg(onnx_path)
       .arg("--MNNModel").arg(mnn_path)
       .arg("--bizCode").arg(&opts.biz_code)
       .arg("--optimizeLevel").arg(opts.optimize_level.to_string());
    
    if opts.fp16 {
        cmd.arg("--fp16");
    }
    if opts.weight_quant_bits > 0 {
        cmd.arg("--weightQuantBits").arg(opts.weight_quant_bits.to_string());
        cmd.arg("--weightQuantBlock").arg(opts.weight_quant_block.to_string());
    }
    if opts.save_static_model {
        cmd.arg("--saveStaticModel");
    }
    if opts.target_version > 0.0 {
        cmd.arg("--targetVersion").arg(opts.target_version.to_string());
    }
    if opts.transformer_fuse {
        cmd.arg("--transformerFuse");
    }
    if opts.allow_custom_op {
        cmd.arg("--allowCustomOp");
    }
    if !opts.use_gelu_approximation {
        cmd.arg("--useGeluApproximation").arg("0");
    }
    if let Some(config) = &opts.input_config_file {
        cmd.arg("--inputConfigFile").arg(config);
    }

    let output = cmd.output()
        .map_err(|e| format!("Failed to execute MNNConvert: {}", e))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("{}\n{}", stdout, stderr);

    if output.status.success() {
        Ok(combined.trim().to_string())
    } else {
        Err(format!("MNNConvert failed: {}", combined))
    }
}

/// Find the MNNConvert binary
fn find_mnn_convert() -> Result<String, String> {
    let paths = [
        "/data/data/com.termux/files/home/MNN/build/MNNConvert",
        "/data/data/com.termux/files/home/MNN/build2/MNNConvert",
        "/data/data/com.termux/files/usr/bin/MNNConvert",
        "MNNConvert", // in PATH
    ];

    for path in &paths {
        if Path::new(path).exists() {
            return Ok(path.to_string());
        }
    }

    Err("MNNConvert not found. Please build MNN with converter enabled.".to_string())
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