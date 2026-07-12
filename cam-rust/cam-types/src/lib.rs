//! Core type definitions for the camera ISP pipeline.
//!
//! This module defines the fundamental data types used throughout the ISP pipeline,
//! including pixel formats, camera sources, and ISP block definitions.
//!
//! # Overview
//!
//! The ISP pipeline processes raw sensor data through a series of blocks:
//!
//! ```text
//! Raw Sensor → Unpack → Demosaic → CCM → Tone → Display
//!                ↓
//!           Bayer Pattern
//! ```

/// Frame pixel format.
///
/// Defines how pixel data is stored in memory. The ISP pipeline supports
/// both standard color formats and raw sensor formats.
///
/// # Raw Formats
///
/// - `RawSensor`: 16-bit per pixel, only lower 10 or 12 bits are valid
/// - `Raw10`: 10-bit packed (4 pixels per 5 bytes)
/// - `Raw12`: 12-bit packed (2 pixels per 3 bytes)
///
/// # Examples
///
/// ```rust,ignore
/// let format = FrameFormat::Rgba8888;
/// assert_eq!(format.bytes_per_pixel(), 4);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum FrameFormat {
    /// YUV 4:2:0 8-bit (3 planes: Y, U, V)
    Yuv420888,
    /// RGBA 8-bit (4 bytes per pixel)
    Rgba8888,
    /// RGB 8-bit (3 bytes per pixel)
    Rgb888,
    /// NCHW float tensor (for ML models)
    NchwFloat,
    /// Raw 16-bit per pixel (only lower 10 or 12 bits are valid, not packed)
    RawSensor,
    /// Raw Bayer 10-bit packed (4 pixels per 5 bytes)
    Raw10,
    /// Raw Bayer 12-bit packed (2 pixels per 3 bytes)
    Raw12,
}

/// Camera source type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum CameraSourceType {
    /// No camera source.
    None,
    /// Test stub that can emit RGB / YUV / RAW / RAW10 / RAW12.
    Stub,
    /// Android Camera HAL v3.
    AndroidHal,
    /// Linux V4L2 camera.
    V4l2,
}

/// Frame type for HDR and normal captures.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum FrameType {
    Normal,
    HdrUnder,
    HdrNeutral,
    HdrOver,
    HdrResult,
}

/// Bayer sensor color filter array pattern.
///
/// The CFA (Color Filter Array) determines the arrangement of red, green,
/// and blue filters over the sensor pixels. The most common patterns are:
///
/// ```text
/// RGGB: R G R G    GRBG: G R G R
///       G B G B          B G B G
///
/// GBRG: G B G R    BGGR: B G B G
///       R G R G          G R G R
/// ```
///
/// # Examples
///
/// ```rust,ignore
/// let pattern = BayerPattern::Rggb;
/// assert_eq!(pattern as u32, 0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum BayerPattern {
    /// Red-Green-Green-Blue (most common)
    Rggb = 0,
    /// Green-Red-Blue-Green
    Grbg = 1,
    /// Green-Blue-Red-Green
    Gbrg = 2,
    /// Blue-Green-Green-Red
    Bggr = 3,
}

/// Matrix types for coefficient passing in ISP pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum MatrixType {
    Ccm,
    Lsc,
    ToneCurve,
    WbGains,
    BlackLevel,
    AeParams,
    Histogram,
    MeanRgb,
    ExposureZones,
    AeRects,
    FaceRects,
    SceneLum,
    ColorTemp,
    Extra,
}

/// Tone mapping parameters.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ToneParams {
    pub contrast: f32,
    pub brightness: f32,
    pub gamma_recip: f32,
    pub shadow_lift: f32,
    pub highlight_roll: f32,
    pub sharpness: f32,
    pub saturation: f32,
}

impl Default for ToneParams {
    fn default() -> Self {
        Self {
            contrast: 1.0,
            brightness: 0.0,
            gamma_recip: 1.0,
            shadow_lift: 0.0,
            highlight_roll: 0.0,
            sharpness: 1.0,
            saturation: 1.0,
        }
    }
}

/// Color correction matrix parameters.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct CcmParams {
    /// 3x3 CCM matrix in row-major order.
    pub matrix: [f32; 9],
    /// Color gains [R, G, B].
    pub gains: [f32; 3],
}

impl Default for CcmParams {
    fn default() -> Self {
        Self {
            matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            gains: [1.0, 1.0, 1.0],
        }
    }
}

/// Tensor layout descriptor.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TensorLayout {
    pub dims: Vec<i64>,
    pub strides: Vec<i64>,
    pub element_size: usize,
}

/// Block definition for the ISP pipeline.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BlockDef {
    pub id: String,
    pub name: String,
    pub input_types: Vec<MatrixType>,
    pub output_types: Vec<MatrixType>,
    pub enabled: bool,
}

/// An image frame in the ISP pipeline.
///
/// Represents a single image with its pixel data and metadata.
/// Frames flow through the ISP pipeline, being transformed by each block.
///
/// # Layout
///
/// ```text
/// Frame {
///     data: [u8]      // Pixel data
///     width: u32      // Width in pixels
///     height: u32     // Height in pixels
///     format: FrameFormat  // Pixel format
///     stride: u32     // Bytes per row (may include padding)
///     timestamp_ns: u64  // Capture timestamp
/// }
/// ```
#[derive(Debug, Clone)]
pub struct Frame {
    /// Raw pixel data
    pub data: Vec<u8>,
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Pixel format
    pub format: FrameFormat,
    /// Stride (bytes per row, may include padding)
    pub stride: u32,
    /// Capture timestamp in nanoseconds since epoch (monotonic preferred).
    /// Used for gyro sync, frame prep timing, and deshake.
    pub timestamp_ns: u64,
}

impl Frame {
    pub fn new(data: Vec<u8>, width: u32, height: u32, format: FrameFormat, stride: u32) -> Self {
        Self {
            data,
            width,
            height,
            format,
            stride,
            timestamp_ns: 0,
        }
    }

    pub fn new_with_timestamp(
        data: Vec<u8>,
        width: u32,
        height: u32,
        format: FrameFormat,
        stride: u32,
        timestamp_ns: u64,
    ) -> Self {
        Self {
            data,
            width,
            height,
            format,
            stride,
            timestamp_ns,
        }
    }

    pub fn bytes_per_pixel(&self) -> u32 {
        match self.format {
            FrameFormat::Rgba8888 => 4,
            FrameFormat::Rgb888 => 3,
            FrameFormat::Yuv420888 => 1,
            FrameFormat::NchwFloat => 4,
            FrameFormat::RawSensor => 2,
            FrameFormat::Raw10 => 2,
            FrameFormat::Raw12 => 2,
        }
    }

    pub fn create_default(width: u32, height: u32, format: FrameFormat) -> Self {
        let bpp = match format {
            FrameFormat::Rgba8888 => 4,
            _ => 4,
        };
        let size = (width * height * bpp) as usize;
        Self {
            data: vec![0u8; size],
            width,
            height,
            format,
            stride: width,
            timestamp_ns: 0,
        }
    }
}

/// Trait for ISP pipeline blocks.
///
/// ISP blocks are the building blocks of the image processing pipeline.
/// Each block performs a specific transformation on the image data.
///
/// # Pipeline Flow
///
/// ```text
/// RawInput → Normalize → CFA → BLC → WB → Demosaic → CCM → Tone → Display
/// ```
///
/// # Implementing a Block
///
/// To create a custom ISP block:
///
/// ```rust,ignore
/// pub struct MyBlock {
///     params: MyParams,
/// }
///
/// impl IspBlock for MyBlock {
///     fn id(&self) -> &str { "my_block" }
///     fn name(&self) -> &str { "My Processing Block" }
///     fn input_types(&self) -> Vec<MatrixType> { vec![MatrixType::Ccm] }
///     fn output_types(&self) -> Vec<MatrixType> { vec![MatrixType::ToneCurve] }
///     fn compute(&self, coeffs: &dyn Any) -> Box<dyn Any> {
///         // Process image data
///         Box::new(output_data)
///     }
/// }
/// ```
pub trait IspBlock: Send + Sync {
    /// Unique identifier for this block type
    fn id(&self) -> &str;

    /// Human-readable name for logging
    fn name(&self) -> &str;

    /// Types of coefficients this block consumes
    fn input_types(&self) -> Vec<MatrixType>;

    /// Types of coefficients this block produces
    fn output_types(&self) -> Vec<MatrixType>;

    /// Whether this block is enabled (can be disabled at runtime)
    fn enabled(&self) -> bool {
        true
    }

    /// Compute the block's output based on input coefficients.
    ///
    /// # Arguments
    /// * `coeffs` - Input coefficients (e.g., AWB gains, CCM matrix)
    ///
    /// # Returns
    /// Boxed output data that can be passed to the next block
    fn compute(&self, coeffs: &dyn std::any::Any) -> Box<dyn std::any::Any>;
}
