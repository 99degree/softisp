//! ISP Rectifier - Types and Core Logic
//! 
//! This crate provides the type definitions and inference logic for the
//! distilled ISP controller model that predicts optimal ISP parameters
//! from frame metadata.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Input metadata extracted from a camera frame
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrameMetadata {
    /// 256-bin luminance histogram (normalized)
    pub histogram: Vec<u32>,
    
    /// Correlated Color Temperature in Kelvin
    pub cct: f32,
    
    /// Current white balance gains [R, G, B]
    pub wb_gains: [f32; 3],
    
    /// Auto-exposure metadata
    pub ae: AutoExposure,
    
    /// Auto-focus metadata
    pub af: AutoFocus,
    
    /// Auto-white-balance metadata
    pub awb: AutoWhiteBalance,
    
    /// Frame brightness (0.0 - 1.0)
    pub brightness: f32,
    
    /// Frame contrast (0.0 - 1.0)
    pub contrast: f32,
    
    /// Estimated noise level (0.0 - 1.0)
    pub noise_level: f32,
    
    /// Frame timestamp
    pub timestamp: u64,
}

impl FrameMetadata {
    /// Convert to feature vector for model input (267 dimensions)
    /// Layout: histogram(256) + cct(1) + wb_gains(3) + ae(2) + af(2) + other(3) = 267
    pub fn to_feature_vector(&self) -> (Vec<f32>, Vec<f32>) {
        // Histogram: normalize to fixed sum
        let mut histogram: Vec<f32> = self.histogram.iter().map(|&x| x as f32).collect();
        let sum: f32 = histogram.iter().sum();
        if sum > 0.0 {
            for v in &mut histogram {
                *v = *v / sum * 10000.0;
            }
        }
        
        // Metadata features (11 dims)
        let metadata = vec![
            self.cct / 10000.0,                    // Normalized CCT
            self.wb_gains[0], self.wb_gains[1], self.wb_gains[2],  // WB gains
            self.ae.exposure_time,                 // Exposure time
            self.ae.iso_gain,                      // ISO gain
            self.af.position,                      // Focus position
            self.af.sharpness,                     // Sharpness
            self.brightness,                       // Brightness
            self.contrast,                         // Contrast
            self.noise_level,                      // Noise level
        ];
        
        (histogram, metadata)
    }
}

/// Auto-exposure metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutoExposure {
    pub exposure_time: f32,
    pub iso_gain: f32,
    pub target_brightness: f32,
}

/// Auto-focus metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutoFocus {
    pub position: f32,
    pub sharpness: f32,
}

/// Auto-white-balance metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AutoWhiteBalance {
    pub gains: [f32; 3],
    pub confidence: f32,
}

/// Optimized ISP parameters output by the model
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ISPOptimizedParams {
    /// White balance gains [R, G, B]
    pub wb_r_gain: f32,
    pub wb_g_gain: f32,
    pub wb_b_gain: f32,
    
    /// 3x3 Color Correction Matrix
    pub ccm: [[f32; 3]; 3],
    
    /// Tone mapping curve (7 control points)
    pub tone_curve_lut: Vec<f32>,
    
    /// Digital zoom/crop factor
    pub zoom_factor: f32,
}

impl Default for ISPOptimizedParams {
    fn default() -> Self {
        Self {
            wb_r_gain: 1.0,
            wb_g_gain: 1.0,
            wb_b_gain: 1.0,
            ccm: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tone_curve_lut: vec![0.0, 0.15, 0.35, 0.5, 0.65, 0.85, 1.0],
            zoom_factor: 1.0,
        }
    }
}

/// Safe ranges for ISP register values
pub struct RegisterLimits {
    pub wb_gains: (f32, f32),
    pub ccm: (f32, f32),
    pub tone_curve: (f32, f32),
    pub zoom: (f32, f32),
}

impl Default for RegisterLimits {
    fn default() -> Self {
        Self {
            wb_gains: (0.1, 10.0),
            ccm: (-2.0, 2.0),
            tone_curve: (0.0, 1.0),
            zoom: (1.0, 4.0),
        }
    }
}

impl ISPOptimizedParams {
    /// Clamp all parameters to safe register ranges
    pub fn clamp(&mut self, limits: &RegisterLimits) {
        self.wb_r_gain = self.wb_r_gain.clamp(limits.wb_gains.0, limits.wb_gains.1);
        self.wb_g_gain = self.wb_g_gain.clamp(limits.wb_gains.0, limits.wb_gains.1);
        self.wb_b_gain = self.wb_b_gain.clamp(limits.wb_gains.0, limits.wb_gains.1);
        
        for row in &mut self.ccm {
            for val in row {
                *val = val.clamp(limits.ccm.0, limits.ccm.1);
            }
        }
        
        for val in &mut self.tone_curve_lut {
            *val = val.clamp(limits.tone_curve.0, limits.tone_curve.1);
        }
        
        self.zoom_factor = self.zoom_factor.clamp(limits.zoom.0, limits.zoom.1);
    }
}

/// ISP Hardware Register Map
/// Matches the actual register layout of the ISP pipeline
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ISPRegisters {
    // White Balance
    pub wb_r_gain: u16,  // Fixed-point Q4.12
    pub wb_g_gain: u16,
    pub wb_b_gain: u16,
    
    // Color Correction Matrix (9 registers)
    pub ccm_00: i16, pub ccm_01: i16, pub ccm_02: i16,
    pub ccm_10: i16, pub ccm_11: i16, pub ccm_12: i16,
    pub ccm_20: i16, pub ccm_21: i16, pub ccm_22: i16,
    
    // Tone Curve LUT (7 entries)
    pub tone_lut: [u16; 7],  // Fixed-point Q0.16
    
    // Zoom/Crop
    pub zoom_scale: u16,  // Fixed-point Q4.12
    pub crop_x: u16,
    pub crop_y: u16,
    pub crop_w: u16,
    pub crop_h: u16,
}

impl Default for ISPRegisters {
    fn default() -> Self {
        Self {
            wb_r_gain: 0x1000,  // 1.0 in Q4.12
            wb_g_gain: 0x1000,
            wb_b_gain: 0x1000,
            ccm_00: 0x1000, ccm_01: 0,     ccm_02: 0,
            ccm_10: 0,      ccm_11: 0x1000, ccm_12: 0,
            ccm_20: 0,      ccm_21: 0,      ccm_22: 0x1000,
            tone_lut: [0x0000, 0x2666, 0x5999, 0x8000, 0xA666, 0xD999, 0xFFFF],
            zoom_scale: 0x1000,
            crop_x: 0, crop_y: 0, crop_w: 1920, crop_h: 1080,
        }
    }
}

impl ISPRegisters {
    /// Convert optimized params to hardware register values
    pub fn from_params(params: &ISPOptimizedParams) -> Self {
        let mut regs = Self::default();
        
        // WB gains: float -> Q4.12
        regs.wb_r_gain = (params.wb_r_gain * 4096.0) as u16;
        regs.wb_g_gain = (params.wb_g_gain * 4096.0) as u16;
        regs.wb_b_gain = (params.wb_b_gain * 4096.0) as u16;
        
        // CCM: float -> Q4.12 (signed)
        regs.ccm_00 = (params.ccm[0][0] * 4096.0) as i16;
        regs.ccm_01 = (params.ccm[0][1] * 4096.0) as i16;
        regs.ccm_02 = (params.ccm[0][2] * 4096.0) as i16;
        regs.ccm_10 = (params.ccm[1][0] * 4096.0) as i16;
        regs.ccm_11 = (params.ccm[1][1] * 4096.0) as i16;
        regs.ccm_12 = (params.ccm[1][2] * 4096.0) as i16;
        regs.ccm_20 = (params.ccm[2][0] * 4096.0) as i16;
        regs.ccm_21 = (params.ccm[2][1] * 4096.0) as i16;
        regs.ccm_22 = (params.ccm[2][2] * 4096.0) as i16;
        
        // Tone curve: float -> Q0.16
        for (i, &val) in params.tone_curve_lut.iter().enumerate() {
            if i < 7 {
                regs.tone_lut[i] = (val.clamp(0.0, 1.0) * 65535.0) as u16;
            }
        }
        
        // Zoom
        regs.zoom_scale = (params.zoom_factor * 4096.0) as u16;
        
        regs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_metadata_feature_vector() {
        let metadata = FrameMetadata {
            histogram: vec![100; 256],
            cct: 5500.0,
            wb_gains: [1.2, 1.0, 0.9],
            ae: AutoExposure { exposure_time: 0.033, iso_gain: 2.0, target_brightness: 0.5 },
            af: AutoFocus { position: 0.5, sharpness: 0.8 },
            awb: AutoWhiteBalance { gains: [1.0, 1.0, 1.0], confidence: 0.9 },
            brightness: 0.6,
            contrast: 0.7,
            noise_level: 0.1,
            timestamp: 12345,
        };
        
        let (hist, meta) = metadata.to_feature_vector();
        assert_eq!(hist.len(), 256);
        assert_eq!(meta.len(), 11);
        
        // Check metadata values
        assert!((meta[0] - 0.55).abs() < 0.01);  // CCT normalized
        assert_eq!(meta[1], 1.2);  // WB R
        assert_eq!(meta[2], 1.0);  // WB G
        assert_eq!(meta[3], 0.9);  // WB B
    }
    
    #[test]
    fn test_params_clamping() {
        let mut params = ISPOptimizedParams {
            wb_r_gain: 15.0,
            wb_g_gain: -0.5,
            wb_b_gain: 2.0,
            ccm: [[3.0, -3.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tone_curve_lut: vec![-0.5, 1.5, 0.5, 0.5, 0.5, 0.5, 0.5],
            zoom_factor: 10.0,
        };
        
        let limits = RegisterLimits::default();
        params.clamp(&limits);
        
        assert_eq!(params.wb_r_gain, 10.0);
        assert_eq!(params.wb_g_gain, 0.1);
        assert_eq!(params.ccm[0][0], 2.0);
        assert_eq!(params.ccm[0][1], -2.0);
        assert_eq!(params.tone_curve_lut[0], 0.0);
        assert_eq!(params.tone_curve_lut[1], 1.0);
        assert_eq!(params.zoom_factor, 4.0);
    }
    
    #[test]
    fn test_register_conversion() {
        let params = ISPOptimizedParams {
            wb_r_gain: 1.5,
            wb_g_gain: 1.0,
            wb_b_gain: 0.8,
            ccm: [[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            tone_curve_lut: vec![0.0, 0.25, 0.5, 0.75, 1.0, 1.0, 1.0],
            zoom_factor: 1.5,
        };
        
        let regs = ISPRegisters::from_params(&params);
        
        // Check WB gains (Q4.12: 1.5 * 4096 = 6144)
        assert_eq!(regs.wb_r_gain, 6144);
        assert_eq!(regs.wb_g_gain, 4096);
        assert_eq!(regs.wb_b_gain, 3276);  // 0.8 * 4096 = 3276.8 -> 3276
        
        // Check CCM
        assert_eq!(regs.ccm_00, 4096);  // 1.0
        assert_eq!(regs.ccm_01, 409);   // 0.1 * 4096 = 409.6 -> 409
        
        // Check tone LUT
        assert_eq!(regs.tone_lut[0], 0);
        assert_eq!(regs.tone_lut[3], 49152);  // 0.75 * 65535 = 49151.25
    }
}