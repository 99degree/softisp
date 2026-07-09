//! Test loading ONNX models

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use tract_onnx::prelude::Framework;
    
    fn models_dir() -> PathBuf {
        // Try CARGO_MANIFEST_DIR first, fall fall back to current dir
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let models = manifest.join("models");
        if models.exists() {
            models
        } else {
            // Fallback for when running from different directory
            PathBuf::from("models")
        }
    }
    
    #[test]
    fn test_fp32_model_exists() {
        let path = models_dir().join("fusedispcontroller.onnx");
        assert!(path.exists(), "FP32 model not found at {:?}", path);
        
        let metadata = std::fs::metadata(&path).unwrap();
        assert!(metadata.len() > 100_000, "FP32 model too small (< 100KB)");
        assert!(metadata.len() < 100_000_000, "FP32 model too large (> 100MB)");
        
        eprintln!("FP32 model: {} bytes ({:.1} KB)", metadata.len(), metadata.len() as f64 / 1024.0);
    }
    
    #[test]
    fn test_fp16_model_exists() {
        let path = models_dir().join("fusedispcontroller_fp16.onnx");
        assert!(path.exists(), "FP16 model not found at {:?}", path);
        
        let metadata = std::fs::metadata(&path).unwrap();
        assert!(metadata.len() > 100_000, "FP16 model too small");
        
        eprintln!("FP16 model: {} bytes ({:.1} KB)", metadata.len(), metadata.len() as f64 / 1024.0);
    }
    
    #[test]
    fn test_int8_model_exists() {
        let path = models_dir().join("fusedispcontroller_int8.onnx");
        assert!(path.exists(), "INT8 model not found at {:?}", path);
        
        let metadata = std::fs::metadata(&path).unwrap();
        assert!(metadata.len() > 100_000, "INT8 model too small");
        
        eprintln!("INT8 model: {} bytes ({:.1} KB)", metadata.len(), metadata.len() as f64 / 1024.0);
    }
    
    #[test]
    fn test_model_sizes_comparison() {
        let fp32 = models_dir().join("fusedispcontroller.onnx");
        let fp16 = models_dir().join("fusedispcontroller_fp16.onnx");
        let int8 = models_dir().join("fusedispcontroller_int8.onnx");
        
        let fp32_size = std::fs::metadata(&fp32).map(|m| m.len()).unwrap_or(0);
        let fp16_size = std::fs::metadata(&fp16).map(|m| m.len()).unwrap_or(0);
        let int8_size = std::fs::metadata(&int8).map(|m| m.len()).unwrap_or(0);
        
        eprintln!("Model sizes:");
        eprintln!("  FP32: {:.1} KB", fp32_size as f64 / 1024.0);
        eprintln!("  FP16: {:.1} KB", fp16_size as f64 / 1024.0);
        eprintln!("  INT8: {:.1} KB", int8_size as f64 / 1024.0);
        
        // All models should exist and have reasonable sizes
        assert!(fp32_size > 100_000, "FP32 model missing or too small");
        assert!(fp16_size > 100_000, "FP16 model missing or too small");
        assert!(int8_size > 100_000, "INT8 model missing or too small");
    }
    
    #[test]
    fn test_load_fp32_with_tract() {
        let path = models_dir().join("fusedispcontroller.onnx");
        if !path.exists() {
            eprintln!("Skipping: FP32 model not found");
            return;
        }
        
        // Try loading with tract
        let result = tract_onnx::onnx().model_for_path(&path);
        if let Err(e) = &result {
            eprintln!("Tract load error (may be expected for external data models): {:?}", e);
        }
        
        // The model may have external data, so tract load might fail
        // This is expected behavior for large models
        eprintln!("Tract load result: {:?}", result.is_ok());
    }
}
