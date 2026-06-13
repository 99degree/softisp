//! Core type definitions for the camera ISP pipeline.
//! Ported from com.camtypes

/// Frame pixel format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum FrameFormat {
    Yuv420888,
    Rgba8888,
    NchwFloat,
    RawSensor,
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum BayerPattern {
    Rggb = 0,
    Grbg = 1,
    Gbrg = 2,
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
#[derive(Debug, Clone)]
pub struct Frame {
    pub data: Vec<u8>,
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub stride: u32,
}

impl Frame {
    pub fn new(data: Vec<u8>, width: u32, height: u32, format: FrameFormat, stride: u32) -> Self {
        Self { data, width, height, format, stride }
    }

    pub fn bytes_per_pixel(&self) -> u32 {
        match self.format {
            FrameFormat::Rgba8888 => 4,
            FrameFormat::Yuv420888 => 1,
            FrameFormat::NchwFloat => 4,
            FrameFormat::RawSensor => 2,
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
        }
    }
}

/// Trait for ISP pipeline blocks.
pub trait IspBlock: Send + Sync {
    fn id(&self) -> &str;
    fn name(&self) -> &str;
    fn input_types(&self) -> Vec<MatrixType>;
    fn output_types(&self) -> Vec<MatrixType>;
    fn enabled(&self) -> bool { true }

    /// Compute the block's output based on input coefficients.
    fn compute(&self, coeffs: &dyn std::any::Any) -> Box<dyn std::any::Any>;
}