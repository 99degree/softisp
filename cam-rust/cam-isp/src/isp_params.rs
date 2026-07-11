//! ISP Parameters — deterministic parameters for pipeline blocks.
//!
//! All pipeline blocks receive parameters from the controller,
//! making them pure functions without internal state or decisions.

use std::collections::HashMap;

/// ISP parameters — all settings for a single frame.
#[derive(Debug, Clone)]
pub struct IspParams {
    /// Black level correction.
    pub blc: BlcParams,
    /// White balance gains.
    pub wb: WbParams,
    /// Color correction matrix.
    pub ccm: CcmParams,
    /// Tone mapping curve.
    pub tone: ToneParams,
    /// Saturation control.
    pub saturation: SaturationParams,
    /// Sharpening.
    pub sharpen: SharpenParams,
    /// Noise reduction.
    pub denoise: DenoiseParams,
    /// Lens corrections.
    pub lens: LensParams,
    /// Display/output settings.
    pub display: DisplayParams,
    /// Digital zoom factor [1.0, 4.0]. From neural model or manual.
    pub zoom: f32,
    /// VCM focus motor position [0.0, 1.0]. 0=infinity, 1=macro.
    /// Used for focus breathing compensation in GDC.
    pub vcm_position: f32,
    /// Custom parameters for extensibility.
    pub custom: HashMap<String, f32>,
}

impl Default for IspParams {
    fn default() -> Self {
        Self {
            blc: BlcParams::default(),
            wb: WbParams::default(),
            ccm: CcmParams::default(),
            tone: ToneParams::default(),
            saturation: SaturationParams::default(),
            sharpen: SharpenParams::default(),
            denoise: DenoiseParams::off(),
            lens: LensParams::default(),
            display: DisplayParams::default(),
            zoom: 1.0,
            vcm_position: 0.0,
            custom: HashMap::new(),
        }
    }
}

impl IspParams {
    /// Create identity parameters (no processing).
    pub fn identity() -> Self {
        Self {
            blc: BlcParams { r: 0.0, gr: 0.0, gb: 0.0, b: 0.0 },
            wb: WbParams { r: 1.0, g: 1.0, b: 1.0 },
            ccm: CcmParams::identity(),
            tone: ToneParams::identity(),
            saturation: SaturationParams { factor: 1.0, vibrance: 0.0 },
            sharpen: SharpenParams { amount: 0.0, radius: 1.0, threshold: 0.0 },
            denoise: DenoiseParams::off(),
            lens: LensParams::default(),
            display: DisplayParams::default(),
            zoom: 1.0,
            vcm_position: 0.0,
            custom: HashMap::new(),
        }
    }

    /// Interpolate between two parameter sets.
    pub fn lerp(&self, other: &IspParams, t: f32) -> IspParams {
        IspParams {
            blc: self.blc.lerp(&other.blc, t),
            wb: self.wb.lerp(&other.wb, t),
            ccm: self.ccm.lerp(&other.ccm, t),
            tone: self.tone.lerp(&other.tone, t),
            saturation: self.saturation.lerp(&other.saturation, t),
            sharpen: self.sharpen.lerp(&other.sharpen, t),
            denoise: self.denoise.lerp(&other.denoise, t),
            lens: self.lens.clone(),
            display: self.display.clone(),
            zoom: self.zoom + (other.zoom - self.zoom) * t,
            vcm_position: self.vcm_position + (other.vcm_position - self.vcm_position) * t,
            custom: self.custom.clone(),
        }
    }
}

/// Black level correction parameters.
#[derive(Debug, Clone)]
pub struct BlcParams {
    /// Black level for R channel.
    pub r: f32,
    /// Black level for Gr channel.
    pub gr: f32,
    /// Black level for Gb channel.
    pub gb: f32,
    /// Black level for B channel.
    pub b: f32,
}

impl Default for BlcParams {
    fn default() -> Self {
        Self { r: 64.0, gr: 64.0, gb: 64.0, b: 64.0 }
    }
}

impl BlcParams {
    pub fn new(r: f32, gr: f32, gb: f32, b: f32) -> Self {
        Self { r, gr, gb, b }
    }

    pub fn lerp(&self, other: &BlcParams, t: f32) -> BlcParams {
        BlcParams {
            r: self.r + (other.r - self.r) * t,
            gr: self.gr + (other.gr - self.gr) * t,
            gb: self.gb + (other.gb - self.gb) * t,
            b: self.b + (other.b - self.b) * t,
        }
    }
}

/// White balance parameters.
#[derive(Debug, Clone)]
pub struct WbParams {
    /// Red gain.
    pub r: f32,
    /// Green gain.
    pub g: f32,
    /// Blue gain.
    pub b: f32,
}

impl Default for WbParams {
    fn default() -> Self {
        Self { r: 1.0, g: 1.0, b: 1.0 }
    }
}

impl WbParams {
    pub fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    pub fn lerp(&self, other: &WbParams, t: f32) -> WbParams {
        WbParams {
            r: self.r + (other.r - self.r) * t,
            g: self.g + (other.g - self.g) * t,
            b: self.b + (other.b - self.b) * t,
        }
    }
}

/// Color correction matrix parameters.
#[derive(Debug, Clone)]
pub struct CcmParams {
    /// 3x3 matrix (row-major).
    pub matrix: [f32; 9],
}

impl Default for CcmParams {
    fn default() -> Self {
        Self::identity()
    }
}

impl CcmParams {
    pub fn identity() -> Self {
        Self {
            matrix: [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0],
        }
    }

    pub fn new(matrix: [f32; 9]) -> Self {
        Self { matrix }
    }

    pub fn lerp(&self, other: &CcmParams, t: f32) -> CcmParams {
        let mut matrix = [0.0; 9];
        for i in 0..9 {
            matrix[i] = self.matrix[i] + (other.matrix[i] - self.matrix[i]) * t;
        }
        CcmParams { matrix }
    }
}

/// Tone mapping parameters.
#[derive(Debug, Clone)]
pub struct ToneParams {
    /// Contrast (1.0 = neutral).
    pub contrast: f32,
    /// Brightness offset.
    pub brightness: f32,
    /// Gamma (1.0 = linear).
    pub gamma: f32,
    /// Black crush (0.0-1.0).
    pub black_crush: f32,
    /// White clip (0.0-1.0).
    pub white_clip: f32,
}

impl Default for ToneParams {
    fn default() -> Self {
        Self::identity()
    }
}

impl ToneParams {
    pub fn identity() -> Self {
        Self {
            contrast: 1.0,
            brightness: 0.0,
            gamma: 1.0,
            black_crush: 0.0,
            white_clip: 1.0,
        }
    }

    pub fn lerp(&self, other: &ToneParams, t: f32) -> ToneParams {
        ToneParams {
            contrast: self.contrast + (other.contrast - self.contrast) * t,
            brightness: self.brightness + (other.brightness - self.brightness) * t,
            gamma: self.gamma + (other.gamma - self.gamma) * t,
            black_crush: self.black_crush + (other.black_crush - self.black_crush) * t,
            white_clip: self.white_clip + (other.white_clip - self.white_clip) * t,
        }
    }
}

/// Saturation parameters.
#[derive(Debug, Clone)]
pub struct SaturationParams {
    /// Saturation factor (1.0 = neutral).
    pub factor: f32,
    /// Vibrance (0.0 = off, 1.0 = max).
    pub vibrance: f32,
}

impl Default for SaturationParams {
    fn default() -> Self {
        Self { factor: 1.0, vibrance: 0.0 }
    }
}

impl SaturationParams {
    pub fn lerp(&self, other: &SaturationParams, t: f32) -> SaturationParams {
        SaturationParams {
            factor: self.factor + (other.factor - self.factor) * t,
            vibrance: self.vibrance + (other.vibrance - self.vibrance) * t,
        }
    }
}

/// Sharpening parameters.
#[derive(Debug, Clone)]
pub struct SharpenParams {
    /// Sharpening amount (0.0 = off).
    pub amount: f32,
    /// Sharpening radius.
    pub radius: f32,
    /// Edge threshold (0.0 = sharpen all).
    pub threshold: f32,
}

impl Default for SharpenParams {
    fn default() -> Self {
        Self { amount: 0.0, radius: 1.0, threshold: 0.0 }
    }
}

impl SharpenParams {
    pub fn lerp(&self, other: &SharpenParams, t: f32) -> SharpenParams {
        SharpenParams {
            amount: self.amount + (other.amount - self.amount) * t,
            radius: self.radius + (other.radius - self.radius) * t,
            threshold: self.threshold + (other.threshold - self.threshold) * t,
        }
    }
}

/// Denoise parameters.
#[derive(Debug, Clone)]
pub struct DenoiseParams {
    /// Spatial denoise strength (0.0 = off).
    pub spatial_strength: f32,
    /// Temporal denoise strength (0.0 = off).
    pub temporal_strength: f32,
    /// Edge preservation (0.0 = none, 1.0 = max).
    pub edge_preserve: f32,
    /// Bilateral filter sigma.
    pub bilateral_sigma: f32,
}

impl DenoiseParams {
    pub fn off() -> Self {
        Self {
            spatial_strength: 0.0,
            temporal_strength: 0.0,
            edge_preserve: 0.5,
            bilateral_sigma: 3.0,
        }
    }

    pub fn lerp(&self, other: &DenoiseParams, t: f32) -> DenoiseParams {
        DenoiseParams {
            spatial_strength: self.spatial_strength + (other.spatial_strength - self.spatial_strength) * t,
            temporal_strength: self.temporal_strength + (other.temporal_strength - self.temporal_strength) * t,
            edge_preserve: self.edge_preserve + (other.edge_preserve - self.edge_preserve) * t,
            bilateral_sigma: self.bilateral_sigma + (other.bilateral_sigma - self.bilateral_sigma) * t,
        }
    }
}

/// Lens correction parameters.
#[derive(Debug, Clone, Default)]
pub struct LensParams {
    /// Vignetting correction strength.
    pub vignetting_strength: f32,
    /// Vignetting center X (0.0-1.0).
    pub vignetting_center_x: f32,
    /// Vignetting center Y (0.0-1.0).
    pub vignetting_center_y: f32,
    /// Lens distortion k1 coefficient.
    pub distortion_k1: f32,
    /// Lens distortion k2 coefficient.
    pub distortion_k2: f32,
    /// Chromatic aberration correction strength.
    pub chromatic_aberration: f32,
}

/// Display/output parameters.
#[derive(Debug, Clone)]
pub struct DisplayParams {
    /// Output format.
    pub format: OutputFormat,
    /// Output width.
    pub width: u32,
    /// Output height.
    pub height: u32,
}

impl Default for DisplayParams {
    fn default() -> Self {
        Self {
            format: OutputFormat::Rgb,
            width: 1920,
            height: 1080,
        }
    }
}

/// Output format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputFormat {
    /// RGB float.
    Rgb,
    /// RGBA uint8.
    Rgba,
    /// ARGB uint8.
    Argb,
    /// YUV420.
    Yuv420,
    /// Float16 RGB.
    Float16,
}

/// Frame statistics from controller analysis.
#[derive(Debug, Clone)]
pub struct FrameStats {
    /// Average luminance.
    pub avg_luminance: f32,
    /// Luminance histogram (256 bins).
    pub histogram: [u32; 256],
    /// Color temperature estimate (K).
    pub color_temp: f32,
    /// Noise level estimate.
    pub noise_level: f32,
    /// Sharpness score.
    pub sharpness: f32,
    /// Face detection results.
    pub faces: Vec<FaceInfo>,
    /// Scene type.
    pub scene: SceneType,
}

impl Default for FrameStats {
    fn default() -> Self {
        Self {
            avg_luminance: 0.5,
            histogram: [0; 256],
            color_temp: 5500.0,
            noise_level: 0.0,
            sharpness: 0.0,
            faces: Vec::new(),
            scene: SceneType::default(),
        }
    }
}

/// Face detection result.
#[derive(Debug, Clone)]
pub struct FaceInfo {
    /// Bounding box [x, y, width, height] (normalized 0.0-1.0).
    pub bbox: [f32; 4],
    /// Confidence score.
    pub confidence: f32,
}

/// Scene classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[derive(Default)]
pub enum SceneType {
    /// General scene.
    #[default]
    General,
    /// Portrait (face detected).
    Portrait,
    /// Landscape (low detail, bright).
    Landscape,
    /// Low light (dark, noisy).
    LowLight,
    /// High dynamic range.
    Hdr,
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isp_params_default() {
        let params = IspParams::default();
        assert_eq!(params.wb.r, 1.0);
        assert_eq!(params.tone.gamma, 1.0);
    }

    #[test]
    fn test_isp_params_identity() {
        let params = IspParams::identity();
        assert_eq!(params.blc.r, 0.0);
        assert_eq!(params.saturation.factor, 1.0);
    }

    #[test]
    fn test_isp_params_lerp() {
        let p1 = IspParams::identity();
        let mut p2 = IspParams::identity();
        p2.wb.r = 2.0;
        
        let interpolated = p1.lerp(&p2, 0.5);
        assert!((interpolated.wb.r - 1.5).abs() < 0.001);
    }

    #[test]
    fn test_blc_params() {
        let blc = BlcParams::new(64.0, 64.0, 64.0, 64.0);
        assert_eq!(blc.r, 64.0);
    }

    #[test]
    fn test_tone_params_identity() {
        let tone = ToneParams::identity();
        assert_eq!(tone.contrast, 1.0);
        assert_eq!(tone.gamma, 1.0);
    }
}
