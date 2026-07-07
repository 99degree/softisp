//! Error types for the camera HAL binder interface.

use std::fmt;

/// Camera HAL binder error type.
///
/// Covers all failure modes in the AIDL binder interface,
/// from device enumeration to capture request processing.
#[derive(Debug, Clone)]
pub enum BinderError {
    /// Camera device not found
    DeviceNotFound(String),
    /// Camera device already in use
    DeviceBusy(String),
    /// Invalid stream configuration
    InvalidStreamConfig(String),
    /// Buffer allocation failed
    BufferAllocation(String),
    /// Capture request failed
    CaptureFailed(String),
    /// Session error
    Session(String),
    /// Metadata error
    Metadata(String),
    /// V4L2 camera error
    V4L2(String),
    /// ISP pipeline error
    Isp(String),
    /// Binder transaction failed
    Transaction(String),
    /// I/O error
    Io(String),
    /// Invalid parameter
    InvalidParameter(String),
    /// Feature not implemented
    NotImplemented(String),
}

impl fmt::Display for BinderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DeviceNotFound(msg) => write!(f, "device not found: {}", msg),
            Self::DeviceBusy(msg) => write!(f, "device busy: {}", msg),
            Self::InvalidStreamConfig(msg) => write!(f, "invalid stream config: {}", msg),
            Self::BufferAllocation(msg) => write!(f, "buffer allocation failed: {}", msg),
            Self::CaptureFailed(msg) => write!(f, "capture failed: {}", msg),
            Self::Session(msg) => write!(f, "session error: {}", msg),
            Self::Metadata(msg) => write!(f, "metadata error: {}", msg),
            Self::V4L2(msg) => write!(f, "V4L2 error: {}", msg),
            Self::Isp(msg) => write!(f, "ISP error: {}", msg),
            Self::Transaction(msg) => write!(f, "transaction error: {}", msg),
            Self::Io(msg) => write!(f, "I/O error: {}", msg),
            Self::InvalidParameter(msg) => write!(f, "invalid parameter: {}", msg),
            Self::NotImplemented(msg) => write!(f, "not implemented: {}", msg),
        }
    }
}

impl std::error::Error for BinderError {}

impl From<std::io::Error> for BinderError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e.to_string())
    }
}

impl From<String> for BinderError {
    fn from(s: String) -> Self {
        Self::Transaction(s)
    }
}

impl From<&str> for BinderError {
    fn from(s: &str) -> Self {
        Self::Transaction(s.to_string())
    }
}

/// Result type alias for binder operations.
pub type BinderResult<T> = Result<T, BinderError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_display() {
        let err = BinderError::DeviceNotFound("camera0".into());
        assert_eq!(format!("{}", err), "device not found: camera0");
    }

    #[test]
    fn test_error_from_string() {
        let err: BinderError = "something failed".into();
        assert!(matches!(err, BinderError::Transaction(_)));
    }

    #[test]
    fn test_error_from_io() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file not found");
        let err: BinderError = io_err.into();
        assert!(matches!(err, BinderError::Io(_)));
    }

    #[test]
    fn test_error_clone() {
        let err = BinderError::Session("test".into());
        let cloned = err.clone();
        assert_eq!(format!("{}", err), format!("{}", cloned));
    }
}
