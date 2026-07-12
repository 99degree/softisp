//! Error types for the ISP pipeline.

use crate::pipeline::builder::PipelineError;
use std::fmt;

/// ISP pipeline error type.
///
/// This error type covers all failure modes in the ISP pipeline,
/// from configuration errors to runtime failures.
#[derive(Debug, Clone)]
pub enum IspError {
    /// Configuration or parameter error
    Config(String),
    /// Block processing failed
    BlockProcessing {
        /// Name of the block that failed
        block: String,
        /// Error message
        message: String,
    },
    /// Pipeline build or execution error
    Pipeline(String),
    /// MNN backend error
    Mnn(String),
    /// ONNX backend error
    Onnx(String),
    /// I/O error
    Io(String),
    /// Invalid input data
    InvalidInput(String),
    /// Feature not enabled
    FeatureNotEnabled(String),
    /// Model conversion error
    Conversion(String),
    /// Quantization error
    Quantization(String),
    /// Memory allocation error
    Memory(String),
    /// Timeout
    Timeout(String),
    /// Unsupported operation
    Unsupported(String),
}

impl fmt::Display for IspError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Config(msg) => write!(f, "configuration error: {}", msg),
            Self::BlockProcessing { block, message } => {
                write!(f, "block '{}' processing failed: {}", block, message)
            }
            Self::Pipeline(msg) => write!(f, "pipeline error: {}", msg),
            Self::Mnn(msg) => write!(f, "MNN error: {}", msg),
            Self::Onnx(msg) => write!(f, "ONNX error: {}", msg),
            Self::Io(msg) => write!(f, "I/O error: {}", msg),
            Self::InvalidInput(msg) => write!(f, "invalid input: {}", msg),
            Self::FeatureNotEnabled(msg) => write!(f, "feature not enabled: {}", msg),
            Self::Conversion(msg) => write!(f, "conversion error: {}", msg),
            Self::Quantization(msg) => write!(f, "quantization error: {}", msg),
            Self::Memory(msg) => write!(f, "memory error: {}", msg),
            Self::Timeout(msg) => write!(f, "timeout: {}", msg),
            Self::Unsupported(msg) => write!(f, "unsupported: {}", msg),
        }
    }
}

impl std::error::Error for IspError {}

impl From<std::io::Error> for IspError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e.to_string())
    }
}

impl From<String> for IspError {
    fn from(s: String) -> Self {
        Self::Pipeline(s)
    }
}

impl From<&str> for IspError {
    fn from(s: &str) -> Self {
        Self::Pipeline(s.to_string())
    }
}

/// Result type alias for ISP operations.
pub type IspResult<T> = Result<T, IspError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = IspError::Config("invalid width".into());
        assert_eq!(format!("{}", err), "configuration error: invalid width");
    }

    #[test]
    fn test_error_from_string() {
        let err: IspError = "something failed".into();
        assert!(matches!(err, IspError::Pipeline(_)));
    }

    #[test]
    fn test_error_from_io() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let err: IspError = io_err.into();
        assert!(matches!(err, IspError::Io(_)));
    }

    #[test]
    fn test_error_clone() {
        let err = IspError::Mnn("test".into());
        let cloned = err.clone();
        assert_eq!(format!("{}", err), format!("{}", cloned));
    }
}

impl From<PipelineError> for IspError {
    fn from(e: PipelineError) -> Self {
        match e {
            PipelineError::EmptyPipeline => Self::Pipeline("empty pipeline".into()),
            PipelineError::InvalidResolution(w, h) => {
                Self::Config(format!("invalid resolution: {}x{}", w, h))
            }
            PipelineError::ValidationFailed(issues) => {
                Self::Pipeline(format!("validation failed: {}", issues.join("; ")))
            }
            PipelineError::ComposeFailed(msg) => Self::Pipeline(msg),
            PipelineError::ConvertFailed(msg) => Self::Conversion(msg),
            PipelineError::IoError(msg) => Self::Io(msg),
        }
    }
}

impl From<crate::hdr::HdrError> for IspError {
    fn from(e: crate::hdr::HdrError) -> Self {
        match e {
            crate::hdr::HdrError::QueueFull => Self::Pipeline("HDR queue full".into()),
            crate::hdr::HdrError::FrameCount { expected, got } => {
                Self::Config(format!("HDR: need {} frames, got {}", expected, got))
            }
            crate::hdr::HdrError::SizeMismatch { w, h, ew, eh } => {
                Self::InvalidInput(format!("HDR size mismatch: {}x{} vs {}x{}", w, h, ew, eh))
            }
            crate::hdr::HdrError::Isp(msg) => Self::Pipeline(format!("HDR ISP: {}", msg)),
            crate::hdr::HdrError::Align(msg) => Self::Pipeline(format!("HDR align: {}", msg)),
            crate::hdr::HdrError::Merge(msg) => Self::Pipeline(format!("HDR merge: {}", msg)),
            crate::hdr::HdrError::Enhance(msg) => Self::Pipeline(format!("HDR enhance: {}", msg)),
            crate::hdr::HdrError::Encode(msg) => Self::Pipeline(format!("HDR encode: {}", msg)),
        }
    }
}

// Additional From implementations for common patterns
impl From<std::num::ParseIntError> for IspError {
    fn from(e: std::num::ParseIntError) -> Self {
        Self::Config(format!("parse error: {}", e))
    }
}

impl From<std::num::ParseFloatError> for IspError {
    fn from(e: std::num::ParseFloatError) -> Self {
        Self::Config(format!("parse error: {}", e))
    }
}
