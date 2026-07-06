//! Post-Processing Pipeline
//!
//! Chains post-ISP operations that work on the output of the default pipeline:
//! - **EIS** (Electronic Image Stabilization) — gyro-based jitter removal
//! - **Deshake** (Software Stabilization) — block-matching motion estimation
//! - **GDC** (Geometric Distortion Correction) — lens distortion correction
//! - **HDR** (Multi-exposure merge) — under/neutral/over exposure fusion
//! - **Temporal Denoise** — frame-to-frame noise reduction
//!
//! # Architecture
//!
//! ```text
//! Default ISP Output (BGRA/RGB)
//!         │
//!         ▼
//!   ┌─────────────┐
//!   │  EIS Warp   │ ← gyro samples
//!   └──────┬──────┘
//!          │
//!          ▼
//!   ┌─────────────┐
//!   │ Deshake     │ ← block-matching motion estimation
//!   └──────┬──────┘
//!          │
//!          ▼
//!   ┌─────────────┐
//!   │  GDC Warp   │ ← lens distortion map
//!   └──────┬──────┘
//!          │
//!          ▼
//!   ┌─────────────┐
//!   │  HDR Merge  │ ← under/over exposures
//!   └──────┬──────┘
//!          │
//!          ▼
//!   ┌─────────────┐
//!   │ Temp Denoise│ ← previous frame
//!   └──────┬──────┘
//!          │
//!          ▼
//!   Final Output
//! ```
//!
//! Each stage is optional and can be enabled/disabled via `PostProcessConfig`.
//! The pipeline runs on CPU (for now) using bilinear interpolation for warps.

use log::debug;
use cam_types::FrameFormat;

use crate::eis::{EisEngine, GyroSample};
use crate::deshake::DeshakeEngine;
use crate::pipeline::{IspFrame, IspAuxOutput};

/// Configuration for the post-processing pipeline.
#[derive(Debug, Clone)]
pub struct PostProcessConfig {
    /// Enable EIS stabilization (gyro-based).
    pub eis_enabled: bool,
    /// EIS crop fraction (0.0-0.2). Default 0.1 = 10% each side.
    pub eis_crop_fraction: f32,
    /// EIS smoothing alpha (0.0-1.0). Lower = smoother but more lag.
    pub eis_smoothing_alpha: f32,

    /// Enable Deshake (software-based, no gyro needed).
    pub deshake_enabled: bool,
    /// Deshake block size (pixels). Default 32.
    pub deshake_block_size: u32,
    /// Deshake search radius (pixels). Default 16.
    pub deshake_search_radius: i32,
    /// Deshake crop fraction (0.0-0.2). Default 0.08.
    pub deshake_crop_fraction: f32,
    /// Deshake smoothing alpha (0.0-1.0). Default 0.08.
    pub deshake_smoothing_alpha: f32,

    /// Enable GDC (lens distortion correction).
    pub gdc_enabled: bool,
    /// GDC strength (0.0 = no correction, 1.0 = full correction).
    pub gdc_strength: f32,

    /// Enable HDR merge.
    pub hdr_enabled: bool,

    /// Enable temporal denoise.
    pub temporal_denoise_enabled: bool,
    /// Temporal denoise blend factor (0.0 = no denoise, 1.0 = full blend).
    pub temporal_denoise_blend: f32,

    /// Output format.
    pub output_format: FrameFormat,
}

impl Default for PostProcessConfig {
    fn default() -> Self {
        Self {
            eis_enabled: false,
            eis_crop_fraction: 0.10,
            eis_smoothing_alpha: 0.15,
            deshake_enabled: false,
            deshake_block_size: 32,
            deshake_search_radius: 16,
            deshake_crop_fraction: 0.08,
            deshake_smoothing_alpha: 0.08,
            gdc_enabled: false,
            gdc_strength: 1.0,
            hdr_enabled: false,
            temporal_denoise_enabled: false,
            temporal_denoise_blend: 0.3,
            output_format: FrameFormat::Rgba8888,
        }
    }
}

/// Post-processing pipeline state.
pub struct PostProcessPipeline {
    config: PostProcessConfig,
    eis: EisEngine,
    deshake: DeshakeEngine,
    /// Previous frame for temporal denoise.
    prev_frame: Option<Vec<u8>>,
    /// Previous frame dimensions.
    prev_dims: Option<(u32, u32)>,
    /// GDC distortion coefficients `[k1, k2, p1, p2, k3]`.
    gdc_coefficients: [f32; 5],
    /// GDC center point (normalized 0-1).
    gdc_center: (f32, f32),
    /// Frame counter.
    frame_count: u64,
}

impl PostProcessPipeline {
    /// Create a new post-processing pipeline.
    pub fn new(config: PostProcessConfig) -> Self {
        let mut eis = EisEngine::new();
        eis.enabled = config.eis_enabled;

        let mut deshake = DeshakeEngine::new();
        deshake.enabled = config.deshake_enabled;
        deshake.block_size = config.deshake_block_size;
        deshake.search_radius = config.deshake_search_radius;
        deshake.smoothing_alpha = config.deshake_smoothing_alpha;

        Self {
            config,
            eis,
            deshake,
            prev_frame: None,
            prev_dims: None,
            gdc_coefficients: [0.0; 5], // No distortion by default
            gdc_center: (0.5, 0.5),
            frame_count: 0,
        }
    }

    /// Create with default config.
    pub fn default_config() -> Self {
        Self::new(PostProcessConfig::default())
    }

    /// Enable EIS with specified parameters.
    pub fn with_eis(mut self, crop_fraction: f32, smoothing_alpha: f32) -> Self {
        self.config.eis_enabled = true;
        self.config.eis_crop_fraction = crop_fraction;
        self.config.eis_smoothing_alpha = smoothing_alpha;
        self.eis.enabled = true;
        self
    }

    /// Enable GDC with lens distortion coefficients.
    ///
    /// # Arguments
    /// * `k1, k2, k3` — radial distortion coefficients
    /// * `p1, p2` — tangential distortion coefficients
    /// * `strength` — correction strength (0.0-1.0)
    pub fn with_gdc(mut self, k1: f32, k2: f32, p1: f32, p2: f32, k3: f32, strength: f32) -> Self {
        self.config.gdc_enabled = true;
        self.config.gdc_strength = strength;
        self.gdc_coefficients = [k1, k2, p1, p2, k3];
        self
    }

    /// Enable HDR merge.
    pub fn with_hdr(mut self) -> Self {
        self.config.hdr_enabled = true;
        self
    }

    /// Enable temporal denoise.
    pub fn with_temporal_denoise(mut self, blend: f32) -> Self {
        self.config.temporal_denoise_enabled = true;
        self.config.temporal_denoise_blend = blend.clamp(0.0, 1.0);
        self
    }

    /// Set GDC center point (normalized 0-1).
    pub fn with_gdc_center(mut self, cx: f32, cy: f32) -> Self {
        self.gdc_center = (cx, cy);
        self
    }

    /// Push a gyro sample for EIS.
    pub fn push_gyro_sample(&mut self, sample: GyroSample) {
        self.eis.push_sample(sample);
    }

    /// Enable software deshake with specified parameters.
    pub fn with_deshake(mut self, block_size: u32, search_radius: i32, crop_fraction: f32, smoothing_alpha: f32) -> Self {
        self.config.deshake_enabled = true;
        self.config.deshake_block_size = block_size;
        self.config.deshake_search_radius = search_radius;
        self.config.deshake_crop_fraction = crop_fraction;
        self.config.deshake_smoothing_alpha = smoothing_alpha;
        self.deshake.enabled = true;
        self.deshake.block_size = block_size;
        self.deshake.search_radius = search_radius;
        self.deshake.smoothing_alpha = smoothing_alpha;
        self
    }

    /// Process a frame through the post-processing pipeline.
    ///
    /// Takes the output of the default ISP pipeline and applies
    /// EIS → Deshake → GDC → Temporal Denoise in sequence.
    pub fn process(&mut self, frame: &IspFrame) -> Result<IspFrame, String> {
        self.process_inner(&frame.data, frame.width, frame.height, frame)
    }

    /// Process a float tensor `[1,3,H,W]` in `[0,1]` range (FloatRgb planar format).
    ///
    /// This is the preferred input for post-processing — avoids format conversion.
    /// The float data is expected as RGB planar: [R0,R1,...,Rn, G0,G1,...,Gn, B0,B1,...,Bn].
    /// Values are in `[0,1]` range (matching FloatRgb ISP output).
    pub fn process_float(
        &mut self,
        float_data: &[f32],
        width: u32,
        height: u32,
        aux: Option<IspAuxOutput>,
        timestamp_ns: u64,
    ) -> Result<IspFrame, String> {
        let n = (width * height) as usize;
        let mut u8_rgba = vec![0u8; n * 4];
        if float_data.len() >= n * 3 {
            // Convert planar [0,1] f32 → packed u8 RGBA
            let r_plane = &float_data[0..n];
            let g_plane = &float_data[n..n * 2];
            let b_plane = &float_data[n * 2..n * 3];
            for i in 0..n {
                u8_rgba[i * 4] = (r_plane[i].clamp(0.0, 1.0) * 255.0) as u8;
                u8_rgba[i * 4 + 1] = (g_plane[i].clamp(0.0, 1.0) * 255.0) as u8;
                u8_rgba[i * 4 + 2] = (b_plane[i].clamp(0.0, 1.0) * 255.0) as u8;
                u8_rgba[i * 4 + 3] = 255;
            }
        } else {
            return Err(format!(
                "process_float: expected {} float values (3×{}×{}), got {}",
                n * 3,
                height,
                width,
                float_data.len()
            ));
        }

        let frame = IspFrame {
            data: vec![], // u8 data is passed separately to process_inner
            width,
            height,
            format: FrameFormat::Rgba8888,
            float_data: Some(float_data.to_vec()),
            aux,
            timestamp_ns,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };
        self.process_inner(&u8_rgba, width, height, &frame)
    }

    fn process_inner(
        &mut self,
        data: &[u8],
        width: u32,
        height: u32,
        frame: &IspFrame,
    ) -> Result<IspFrame, String> {
        let t_start = std::time::Instant::now();

        if !self.config.eis_enabled
            && !self.config.deshake_enabled
            && !self.config.gdc_enabled
            && !self.config.hdr_enabled
            && !self.config.temporal_denoise_enabled
        {
            // No post-processing needed
            return Ok(frame.clone());
        }

        let mut data = data.to_vec();
        let mut width = width;
        let mut height = height;
        let mut eis_compensation = None;

        // ── Stage 1: EIS Warp ──
        if self.config.eis_enabled {
            let t_eis = std::time::Instant::now();

            // Compute EIS compensation
            if let Some(comp) = self.eis.update(
                frame.timestamp_ns as i64,
                500.0, // focal length in pixels (should come from calibration)
                width,
                height,
            ) {
                eis_compensation = Some(comp);

                // Apply EIS warp
                let (new_data, new_w, new_h) = apply_eis_warp(
                    &data,
                    width,
                    height,
                    &comp,
                    self.config.eis_crop_fraction,
                )?;
                data = new_data;
                width = new_w;
                height = new_h;

                debug!("EIS warp: dx={:.1} dy={:.1} roll={:.2}° ({:.2}ms)",
                    comp[0], comp[1], comp[2],
                    t_eis.elapsed().as_secs_f64() * 1000.0);
            }
        }

        // ── Stage 2: Deshake (software stabilization) ──
        if self.config.deshake_enabled {
            let t_deshake = std::time::Instant::now();

            if let Some((new_data, new_w, new_h)) = self.deshake.update(&data, width, height) {
                data = new_data;
                width = new_w;
                height = new_h;

                let motion = self.deshake.smooth_motion();
                debug!("Deshake: dx={:.1} dy={:.1} ({:.2}ms)",
                    motion[0], motion[1],
                    t_deshake.elapsed().as_secs_f64() * 1000.0);
            }
        }

        // ── Stage 3: GDC (Lens Distortion Correction) ──
        if self.config.gdc_enabled {
            let t_gdc = std::time::Instant::now();

            let (new_data, new_w, new_h) = apply_gdc(
                &data,
                width,
                height,
                &self.gdc_coefficients,
                self.gdc_center,
                self.config.gdc_strength,
            )?;
            data = new_data;
            width = new_w;
            height = new_h;

            debug!("GDC: strength={:.2} ({:.2}ms)",
                self.config.gdc_strength,
                t_gdc.elapsed().as_secs_f64() * 1000.0);
        }

        // ── Stage 4: Temporal Denoise ──
        if self.config.temporal_denoise_enabled {
            if let Some(ref prev) = self.prev_frame {
                if let Some((pw, ph)) = self.prev_dims {
                    if pw == width && ph == height {
                        let t_td = std::time::Instant::now();

                        data = apply_temporal_denoise(
                            &data,
                            prev,
                            width,
                            height,
                            self.config.temporal_denoise_blend,
                        );

                        debug!("Temporal denoise: blend={:.2} ({:.2}ms)",
                            self.config.temporal_denoise_blend,
                            t_td.elapsed().as_secs_f64() * 1000.0);
                    }
                }
            }
        }

        // Store current frame for next temporal denoise
        if self.config.temporal_denoise_enabled {
            self.prev_frame = Some(data.clone());
            self.prev_dims = Some((width, height));
        }

        self.frame_count += 1;

        let total_ms = t_start.elapsed().as_secs_f64() * 1000.0;
        debug!("PostProcess frame {}: {}x{} → {}x{} ({:.2}ms)",
            self.frame_count, frame.width, frame.height, width, height, total_ms);

        let mut result = IspFrame::new(width, height, self.config.output_format);
        result.data = data;
        result.aux = frame.aux.clone();
        result.timestamp_ns = frame.timestamp_ns;
        result.prep_duration_ns = frame.prep_duration_ns;
        result.inference_duration_ns = frame.inference_duration_ns;
        result.total_duration_ns = frame.total_duration_ns + (total_ms * 1_000_000.0) as u64;

        // Add EIS compensation to aux output
        if let Some(comp) = eis_compensation {
            if let Some(ref mut aux) = result.aux {
                aux.eis_compensation = Some(comp);
            }
        }

        Ok(result)
    }

    /// Reset pipeline state (e.g., on camera switch).
    pub fn reset(&mut self) {
        self.eis.reset();
        self.deshake.reset();
        self.prev_frame = None;
        self.prev_dims = None;
        self.frame_count = 0;
    }

    /// Get current EIS state.
    pub fn eis(&self) -> &EisEngine {
        &self.eis
    }

    /// Get mutable EIS state.
    pub fn eis_mut(&mut self) -> &mut EisEngine {
        &mut self.eis
    }

    /// Get current Deshake state.
    pub fn deshake(&self) -> &DeshakeEngine {
        &self.deshake
    }

    /// Get mutable Deshake state.
    pub fn deshake_mut(&mut self) -> &mut DeshakeEngine {
        &mut self.deshake
    }

    /// Get frame count.
    pub fn frame_count(&self) -> u64 {
        self.frame_count
    }
}

// ============================================================================
// EIS Warp Implementation
// ============================================================================

/// Apply EIS stabilization warp to a BGRA/RGBA frame.
///
/// Returns (warped_data, new_width, new_height) after cropping.
fn apply_eis_warp(
    data: &[u8],
    width: u32,
    height: u32,
    comp: &[f32; 3],
    crop_fraction: f32,
) -> Result<(Vec<u8>, u32, u32), String> {
    let dx = comp[0];
    let dy = comp[1];
    let roll_deg = comp[2];

    // Compute crop margins
    let margin_x = (width as f32 * crop_fraction).round() as u32;
    let margin_y = (height as f32 * crop_fraction).round() as u32;

    // Check for overflow before subtraction
    if width <= 2 * margin_x || height <= 2 * margin_y {
        return Err("EIS crop too large".to_string());
    }

    // Output dimensions (after crop)
    let out_w = width - 2 * margin_x;
    let out_h = height - 2 * margin_y;

    // Apply inverse warp + crop
    let roll_rad = roll_deg * std::f32::consts::PI / 180.0;
    let cos_r = roll_rad.cos();
    let sin_r = roll_rad.sin();
    let cx = width as f32 / 2.0;
    let cy = height as f32 / 2.0;

    let bpp = 4; // BGRA/RGBA
    let mut out = vec![0u8; (out_w * out_h * bpp as u32) as usize];

    for oy in 0..out_h {
        for ox in 0..out_w {
            // Map output pixel to input pixel (inverse of compensation)
            let ix = ox as f32 + margin_x as f32;
            let iy = oy as f32 + margin_y as f32;

            // Apply inverse rotation and translation
            let xc = ix - cx;
            let yc = iy - cy;
            let sx = cos_r * xc - sin_r * yc + cx - dx;
            let sy = sin_r * xc + cos_r * yc + cy - dy;

            // Bilinear interpolation
            let pixel = bilinear_sample(data, width, height, bpp, sx, sy);
            let out_idx = ((oy * out_w + ox) * bpp as u32) as usize;
            out[out_idx..out_idx + bpp].copy_from_slice(&pixel);
        }
    }

    Ok((out, out_w, out_h))
}

// ============================================================================
// GDC (Geometric Distortion Correction) Implementation
// ============================================================================

/// Apply lens distortion correction using Brown-Conrady model.
///
/// Returns (corrected_data, width, height).
fn apply_gdc(
    data: &[u8],
    width: u32,
    height: u32,
    coefficients: &[f32; 5],
    center: (f32, f32),
    strength: f32,
) -> Result<(Vec<u8>, u32, u32), String> {
    let [k1, k2, p1, p2, k3] = *coefficients;
    let (cx, cy) = center;
    let cx_px = cx * width as f32;
    let cy_px = cy * height as f32;

    // Scale factor for normalized coordinates
    let scale = (width as f32).max(height as f32) / 2.0;

    let bpp = 4;
    let mut out = vec![0u8; data.len()];

    for y in 0..height {
        for x in 0..width {
            // Normalize to center
            let x_n = (x as f32 - cx_px) / scale;
            let y_n = (y as f32 - cy_px) / scale;

            // Radial distance
            let r2 = x_n * x_n + y_n * y_n;
            let r4 = r2 * r2;
            let r6 = r4 * r2;

            // Radial distortion
            let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

            // Tangential distortion
            let dx_tang = 2.0 * p1 * x_n * y_n + p2 * (r2 + 2.0 * x_n * x_n);
            let dy_tang = p1 * (r2 + 2.0 * y_n * y_n) + 2.0 * p2 * x_n * y_n;

            // Apply distortion (inverse for correction)
            let src_x_n = x_n * radial + dx_tang * strength;
            let src_y_n = y_n * radial + dy_tang * strength;

            // Convert back to pixel coordinates
            let src_x = src_x_n * scale + cx_px;
            let src_y = src_y_n * scale + cy_px;

            // Bilinear sample
            let pixel = bilinear_sample(data, width, height, bpp, src_x, src_y);
            let out_idx = ((y * width + x) * bpp as u32) as usize;
            out[out_idx..out_idx + bpp].copy_from_slice(&pixel);
        }
    }

    Ok((out, width, height))
}

// ============================================================================
// Temporal Denoise Implementation
// ============================================================================

/// Blend current frame with previous frame for temporal noise reduction.
fn apply_temporal_denoise(
    current: &[u8],
    previous: &[u8],
    width: u32,
    height: u32,
    blend: f32,
) -> Vec<u8> {
    let len = (width * height * 4) as usize;
    if current.len() != len || previous.len() != len {
        return current.to_vec();
    }

    let mut out = vec![0u8; len];
    let inv_blend = 1.0 - blend;

    for i in 0..len {
        let c = current[i] as f32;
        let p = previous[i] as f32;
        out[i] = (c * inv_blend + p * blend).round() as u8;
    }

    out
}

// ============================================================================
// Bilinear Interpolation
// ============================================================================

/// Sample a pixel using bilinear interpolation.
fn bilinear_sample(data: &[u8], width: u32, height: u32, bpp: usize, x: f32, y: f32) -> Vec<u8> {
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = x0 + 1;
    let y1 = y0 + 1;

    let fx = x - x0 as f32;
    let fy = y - y0 as f32;

    let sample = |sx: i32, sy: i32| -> Vec<u8> {
        if sx < 0 || sx >= width as i32 || sy < 0 || sy >= height as i32 {
            vec![0u8; bpp]
        } else {
            let idx = ((sy as u32 * width + sx as u32) * bpp as u32) as usize;
            data[idx..idx + bpp].to_vec()
        }
    };

    let p00 = sample(x0, y0);
    let p10 = sample(x1, y0);
    let p01 = sample(x0, y1);
    let p11 = sample(x1, y1);

    let mut result = vec![0u8; bpp];
    for c in 0..bpp {
        let v = p00[c] as f32 * (1.0 - fx) * (1.0 - fy)
            + p10[c] as f32 * fx * (1.0 - fy)
            + p01[c] as f32 * (1.0 - fx) * fy
            + p11[c] as f32 * fx * fy;
        result[c] = v.round().clamp(0.0, 255.0) as u8;
    }

    result
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_postprocess_config_default() {
        let config = PostProcessConfig::default();
        assert!(!config.eis_enabled);
        assert!(!config.gdc_enabled);
        assert!(!config.hdr_enabled);
        assert!(!config.temporal_denoise_enabled);
    }

    #[test]
    fn test_postprocess_pipeline_creation() {
        let pipeline = PostProcessPipeline::new(PostProcessConfig::default());
        assert_eq!(pipeline.frame_count(), 0);
    }

    #[test]
    fn test_postprocess_noop() {
        let mut pipeline = PostProcessPipeline::new(PostProcessConfig::default());
        let frame = IspFrame::new(64, 64, FrameFormat::Rgba8888);
        let result = pipeline.process(&frame).unwrap();
        assert_eq!(result.width, 64);
        assert_eq!(result.height, 64);
    }

    #[test]
    fn test_bilinear_sample_center() {
        let data = vec![100u8; 4 * 4 * 4]; // 4x4 BGRA
        let pixel = bilinear_sample(&data, 4, 4, 4, 1.5, 1.5);
        assert_eq!(pixel, vec![100, 100, 100, 100]);
    }

    #[test]
    fn test_bilinear_sample_edge() {
        let mut data = vec![0u8; 4 * 4 * 4];
        // Set corner pixel
        data[0..4].copy_from_slice(&[255, 128, 64, 32]);
        let pixel = bilinear_sample(&data, 4, 4, 4, 0.1, 0.1);
        // Should be close to corner value
        assert!(pixel[0] > 200);
    }

    #[test]
    fn test_temporal_denoise() {
        let current = vec![100u8; 16];
        let previous = vec![200u8; 16];
        let result = apply_temporal_denoise(&current, &previous, 2, 2, 0.5);
        // Should blend to ~150
        assert!((result[0] as i32 - 150).abs() < 2);
    }

    #[test]
    fn test_eis_crop_too_large() {
        let data = vec![0u8; 10 * 10 * 4];
        let comp = [0.0, 0.0, 0.0];
        let result = apply_eis_warp(&data, 10, 10, &comp, 0.6);
        assert!(result.is_err());
    }

    #[test]
    fn test_gdc_identity() {
        // Zero coefficients = no distortion
        let data = vec![128u8; 8 * 8 * 4];
        let coeffs = [0.0; 5];
        let (out, w, h) = apply_gdc(&data, 8, 8, &coeffs, (0.5, 0.5), 1.0).unwrap();
        assert_eq!(w, 8);
        assert_eq!(h, 8);
        // Output should be identical to input
        assert_eq!(out, data);
    }

    #[test]
    fn test_postprocess_with_eis() {
        let config = PostProcessConfig {
            eis_enabled: true,
            eis_crop_fraction: 0.1,
            ..Default::default()
        };
        let mut pipeline = PostProcessPipeline::new(config);

        // Push some gyro samples
        pipeline.push_gyro_sample(GyroSample {
            timestamp_ns: 1_000_000_000,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });

        let frame = IspFrame::new(100, 100, FrameFormat::Rgba8888);
        let result = pipeline.process(&frame).unwrap();

        // EIS only warps after first frame (needs prev timestamp)
        // First frame returns same dimensions
        assert_eq!(result.width, 100);
        assert_eq!(result.height, 100);
    }
}
