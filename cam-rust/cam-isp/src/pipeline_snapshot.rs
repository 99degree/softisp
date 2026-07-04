//! PipelineSnapshot — save/restore pipeline state for fast restart.
//!
//! Captures the serialized ONNX bytes + MNN model bytes so that
//! subsequent frames don't need to re-generate or re-convert.
//! Useful for camera preview where the pipeline topology doesn't change.
//!
//! Usage:
//!   let snapshot = PipelineSnapshot::capture(&blocks, &params)?;
//!   // Later: fast restart without ONNX generation + conversion
//!   let engine = snapshot.restore_engine()?;

use std::path::{Path, PathBuf};

pub struct PipelineSnapshot {
    pub onnx_bytes: Vec<u8>,
    pub mnn_path: PathBuf,
    pub width: u32,
    pub height: u32,
    pub stage_count: usize,
}

impl PipelineSnapshot {
    /// Capture pipeline state: generate ONNX + convert to MNN.
    pub fn capture(
        onnx_bytes: &[u8],
        mnn_dir: &Path,
        width: u32,
        height: u32,
        stage_count: usize,
    ) -> Result<Self, String> {
        let mnn_path = mnn_dir.join(format!("pipeline_{}x{}.mnn", width, height));

        // Convert ONNX to MNN
        let onnx_path = mnn_dir.join(format!("pipeline_{}x{}.onnx", width, height));
        std::fs::write(&onnx_path, onnx_bytes)
            .map_err(|e| format!("write onnx: {}", e))?;

        crate::mnn_converter::convert_onnx_to_mnn(
            &onnx_path.to_string_lossy(),
            &mnn_path.to_string_lossy(),
            None,
        )?;

        Ok(Self {
            onnx_bytes: onnx_bytes.to_vec(),
            mnn_path,
            width,
            height,
            stage_count,
        })
    }

    /// Restore engine from cached MNN model (skips ONNX gen + conversion).
    pub fn restore_engine(&self) -> Result<crate::mnnengine::MnnEngine, String> {
        let mut engine = crate::mnnengine::MnnEngine::new(
            crate::mnnengine::MnnBackend::Vulkan);
        engine.set_model_path(self.mnn_path.to_string_lossy().to_string());
        Ok(engine)
    }

    /// Check if cached MNN model exists.
    pub fn is_cached(&self) -> bool {
        self.mnn_path.exists()
    }

    /// Delete cached files.
    pub fn invalidate(&self) -> Result<(), String> {
        if self.mnn_path.exists() {
            std::fs::remove_file(&self.mnn_path)
                .map_err(|e| format!("remove mnn: {}", e))?;
        }
        let onnx_path = self.mnn_path.with_extension("onnx");
        if onnx_path.exists() {
            std::fs::remove_file(&onnx_path)
                .map_err(|e| format!("remove onnx: {}", e))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_snapshot_struct() {
        let snap = PipelineSnapshot {
            onnx_bytes: vec![0u8; 100],
            mnn_path: PathBuf::from("/tmp/test.mnn"),
            width: 1920,
            height: 1080,
            stage_count: 4,
        };
        assert_eq!(snap.width, 1920);
        assert_eq!(snap.height, 1080);
        assert_eq!(snap.stage_count, 4);
        assert!(!snap.is_cached());
    }

    #[test]
    fn test_snapshot_is_cached() {
        let snap = PipelineSnapshot {
            onnx_bytes: vec![0u8; 100],
            mnn_path: PathBuf::from("/nonexistent/path.mnn"),
            width: 640,
            height: 480,
            stage_count: 2,
        };
        // File doesn't exist, so not cached
        assert!(!snap.is_cached());
    }

    #[test]
    fn test_snapshot_invalidate() {
        let snap = PipelineSnapshot {
            onnx_bytes: vec![0u8; 50],
            mnn_path: PathBuf::from("/nonexistent/path.mnn"),
            width: 640,
            height: 480,
            stage_count: 2,
        };
        assert!(!snap.is_cached());
        // invalidate is safe even if file doesn't exist
        let _ = snap.invalidate();
        // After invalidation, mnn_path should be cleared
        assert!(!snap.is_cached());
    }
}
