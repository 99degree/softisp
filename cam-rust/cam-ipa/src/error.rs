//! Error types for cam-ipa

use thiserror::Error;

/// Result type for cam-ipa operations
pub type IpaResult<T> = Result<T, IpaError>;

/// Errors that can occur in the IPA pipeline
#[derive(Error, Debug)]
pub enum IpaError {
    #[error("Engine error: {0}")]
    Engine(#[from] cam_isp::error::IspError),

    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    #[error("Not initialized")]
    NotInitialized,

    #[error("Already initialized")]
    AlreadyInitialized,

    #[error("Buffer error: {0}")]
    Buffer(String),

    #[error("Model loading failed: {0}")]
    ModelLoad(String),

    #[error("Inference failed: {0}")]
    Inference(String),

    #[error("Callback error: {0}")]
    Callback(String),

    #[error("FFI conversion error: {0}")]
    FfiConversion(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Other: {0}")]
    Other(String),
}

impl From<String> for IpaError {
    fn from(s: String) -> Self {
        IpaError::Other(s)
    }
}

impl From<&str> for IpaError {
    fn from(s: &str) -> Self {
        IpaError::Other(s.to_string())
    }
}
