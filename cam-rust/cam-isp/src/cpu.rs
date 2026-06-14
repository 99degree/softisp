//! Pure Rust CPU fallback engine.
//!
//! Orchestrates the 16-stage ISP pipeline using sub-modules for pixel ops.
//!
//! Pipeline:
//!   RawInput(INT16) → Normalize(FLOAT) → DPC → Denoise → Calibration
//!   → [AF stats] → [EIS gyro] → AWB → BLC/WB → LSC → Demosaic(RGB)
//!   → [IspController feedback] → CCM → AE → Tone → FCS → LDCI
//!   → Warp → Display(UINT8 BGRA)

use std::sync::Mutex;
use std::time::Instant;

use log::{info, debug, warn, error};
use cam_types::ToneParams;

use crate::engine::{IspEngine, EngineFactory, register_engine};
use crate::pipeline::{IspBlock, IspFrame};
use crate::controller::IspController;
use crate::af::AfState;
use crate::eis::EisEngine;

use crate::demosaic::{demosaic_malvar, bayer_to_quads};
use crate::isp_ops::{
    generate_simulated_raw, apply_dpc, apply_gaussian_denoise, apply_blc_wb_raw,
    apply_lsc, apply_tone, apply_fcs, apply_ldci,
    calculate_ae_gain,
};
use crate::stats::{compute_channel_means, compute_tone_stats, compute_histogram, compute_zone_stats};
use crate::warp::{generate_identity_grid, warp_image};
use crate::simd::selector::{SimdEngine, best_backend};


// ── Engine registration ──

pub struct CpuEngine {
    loaded: bool,
    _target_width: u32,
    /// ISP controller for AWB/AE/CCM/tone parameter estimation.
    pub controller: Mutex<IspController>,
    /// AF engine for autofocus state machine.
    pub af_engine: Mutex<AfState>,
    /// EIS engine for gyro stabilization.
    pub eis_engine: Mutex<EisEngine>,
    /// Auto-selected SIMD backend (Neon, SSE2, or Scalar).
    simd: &'static dyn SimdEngine,
}

impl CpuEngine {
    pub fn new() -> Self {
        let simd = best_backend();
        info!("CpuEngine: SIMD backend selected: {}", simd.name());
        Self {
            loaded: false,
            _target_width: 0,
            controller: Mutex::new(IspController::new()),
            af_engine: Mutex::new(AfState::default()),
            eis_engine: Mutex::new(EisEngine::new()),
            simd,
        }
    }
}

impl IspEngine for CpuEngine {
    fn backend_name(&self) -> &'static str { "CPU" }
    fn priority(&self) -> i32 { 70 }
    fn is_loaded(&self) -> bool { self.loaded }

    fn build(
        &mut self,
        pipeline_head: Box<dyn IspBlock>,
        aux_blocks: Vec<Box<dyn IspBlock>>,
        _warp_block: Option<Box<dyn IspBlock>>,
        _opset_version: i64,
    ) -> Result<(), String> {
        let n_aux = aux_blocks.len();
        let head_id = pipeline_head.id();
        info!("CpuEngine::build head={} aux={} blocks", head_id, n_aux);
        // Blocks are consumed for compatibility with IspEngine trait
        // (CPU engine doesn't need block composition)
        self.loaded = true;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn process(
        &self,
        width: u32,
        height: u32,
        _stride_width: u32,
        buf: &[u8],
        _sensor_max: f32,
        target_width: u32,
        ccm_matrix: Option<&[f32; 9]>,
        _tone_params: &ToneParams,
        bayer_gains: Option<&[f32; 4]>,
        awb_gains: Option<&[f32; 3]>,
        _analog_gain: f32,
        _scene_change: f32,
        _lsc_gains: Option<&[f32]>,
        blc_values: Option<&[f32; 4]>,
        _warp_grid: Option<&[f32]>,
    ) -> Result<IspFrame, String> {
        if !self.loaded {
            error!("CpuEngine::process called before build()");
            return Err("Engine not initialized".to_string());
        }
        let t0 = Instant::now();

        debug!("CpuEngine::process start {}x{} → target {}", width, height, target_width);
        macro_rules! stage_time { ($name:expr) => { debug!("  stage {:>2}: {} = {:?}", line!(), $name, t0.elapsed()); } }
        // Per-5-lines progress tracker for hang debugging
        macro_rules! progress { () => { debug!("  prog >> line {} t={:?}", line!(), t0.elapsed()); } }

        // ── 1. RawInput: interpret as INT16 Bayer ──
        let expected = (width * height * 2) as usize;
        let raw: Vec<u16> = if buf.len() >= expected {
            buf.chunks_exact(2)
                .take((width * height) as usize)
                .map(|c| u16::from_le_bytes([c[0], c[1]]))
                .collect()
        } else {
            warn!("CpuEngine: input buffer too small ({} < {}), using simulated data",
                buf.len(), expected);
            generate_simulated_raw(width, height, buf)
        };
        let t_input = t0.elapsed();

        // ── 2. Normalize: INT16 → FLOAT [0, 1] ──
        let max_val = if _sensor_max > 0.0 { _sensor_max } else { 65535.0 };
        let mut float = vec![0.0f32; raw.len()];
        self.simd.normalize_u16_to_f32(&raw, &mut float, max_val);
        stage_time!("2. Normalize");

        // ── 2b. DPC (defective pixel correction) ──
        let dpc_data = apply_dpc(&float, width as usize, height as usize,
            _lsc_gains.map(|g| g[0]).unwrap_or(0.15));
        stage_time!("2b. DPC");

        // ── 2c. Gaussian denoise ──
        let denoised = apply_gaussian_denoise(&dpc_data, width as usize, height as usize, 0.3);
        stage_time!("2c. Denoise");

        // ── 2d. Calibration stats ──
        let calibration_stats = {
            let quads = bayer_to_quads(&denoised, width as usize, height as usize);
            crate::calibration::compute_calibration_stats(
                &quads, 4, (height as usize + 1) / 2, (width as usize + 1) / 2,
                cam_types::BayerPattern::Rggb,
            )
        };

        stage_time!("2d. Calibration");

        progress!();

        // ── 2e. AF engine ──
        let focus_metric = AfState::focus_metric_from_calibration(&calibration_stats.0);
        if let Ok(mut af) = self.af_engine.try_lock() {
            af.feed_metric(focus_metric);
            if af.scan_phase == crate::af::AfScanPhase::Idle {
                af.start_scan();
            }
            af.advance_scan();
        }

        // ── 2f. EIS ──
        let eis_frame_count = self.controller.try_lock()
            .map(|g| g.frame_count)
            .unwrap_or(0);
        let eis_compensation = match self.eis_engine.try_lock() {
            Ok(mut eis) => {
                if !eis.enabled { eis.enabled = true; }
                let t_ns = eis_frame_count as i64 * 33_333_333;
                let jitter = (eis_frame_count as f32 * 0.1).sin() * 0.02;
                eis.push_sample(crate::eis::GyroSample {
                    timestamp_ns: t_ns, x: 0.01 + jitter,
                    y: 0.02 + jitter * 0.5, z: 0.005 + jitter * 0.3,
                });
                if _warp_grid.is_some() {
                    None
                } else {
                    let fl = width as f32 * 1.2;
                    eis.update(t_ns + 33_333_333, fl, width, height)
                }
            }
            Err(_) => None
        };
        stage_time!("2f. EIS");
        let t_pre = t0.elapsed();

        progress!();

        // ── 3. AWB ──
        let awb_gains = if let Some(g) = awb_gains { *g } else {
            self.controller.try_lock()
                .map(|g| g.get_awb_gains())
                .unwrap_or([1.0, 1.0, 1.0])
        };
        let default_bayer_gains = if bayer_gains.is_some() {
            *bayer_gains.unwrap()
        } else {
            [awb_gains[0], awb_gains[1], awb_gains[1], awb_gains[2]]
        };
        let wb_gains: &[f32; 4] = bayer_gains.unwrap_or(&default_bayer_gains);

        // ── 4. BLC + WB + LSC ──
        let blc = *blc_values.unwrap_or(&[0.0, 0.0, 0.0, 0.0]);
        let blc_wb = apply_blc_wb_raw(&denoised, width as usize, height as usize, &blc, wb_gains);
        let lsc_k = _lsc_gains.and_then(|g| g.first()).copied().unwrap_or(0.15);
        let corrected = apply_lsc(&blc_wb, width as usize, height as usize, lsc_k);

        // ── 5. Demosaic → RGB ──
        let rgb = demosaic_malvar(&corrected, width as usize, height as usize, Some(&awb_gains));
        stage_time!("5. Demosaic");

        // ── 5b. Stats → IspController ──
        progress!();
        let channel_means = compute_channel_means(&rgb);
        let tone_stats = compute_tone_stats(&rgb);
        let histogram = compute_histogram(&rgb);
        let zone_stats = compute_zone_stats(&rgb, width as usize, height as usize, 6, 8);
        progress!();
        let ctrl_ccm = match self.controller.try_lock() {
            Ok(mut ctrl) => {
                if !ctrl.zone_stats_enabled { ctrl.init_zone_stats(6, 8); }
                ctrl.update_channel_stats(&channel_means);
                ctrl.update_tone_stats(&tone_stats);
                ctrl.update_histogram(&histogram);
                ctrl.update_zone_stats(&zone_stats);
                let bias = ctrl.brightness_bias;
                let _ = ctrl.compute_exposure(bias);
                ctrl.get_ccm()
            }
            Err(_) => {
                warn!("CpuEngine: controller lock contention, using fallback CCM");
                [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            }
        };

        stage_time!("5b. Stats → Ctrl");

        progress!();

        // ── 6. AE ──
        let ae_gain = if _analog_gain <= 0.0 {
            self.controller.try_lock()
                .map(|g| g.get_effective_exposure_gain())
                .unwrap_or(1.0)
        } else {
            calculate_ae_gain(&rgb)
        };

        stage_time!("6. AE");

        progress!();

        // ── 7. CCM ──
        let ccm: &[f32; 9] = ccm_matrix.unwrap_or(&ctrl_ccm);
        let ccm_applied = self.simd.apply_ccm(&rgb, ccm);

        progress!();

        // ── 8. Tone + FCS + LDCI + Warp ──
        let adjusted = self.simd.apply_ae_gain(&ccm_applied, ae_gain);
        let mut toned = apply_tone(&adjusted, _tone_params, width as usize, height as usize);
        toned = apply_fcs(&toned, width as usize, height as usize, 0.4);
        toned = apply_ldci(&toned, width as usize, height as usize, 0.3);

        // Warp (EIS or external)
        toned = {
            if let Some(comp) = eis_compensation {
                let identity = generate_identity_grid(height as usize, width as usize);
                let mut warp = identity.clone();
                let cos_r = comp[2].to_radians().cos();
                let sin_r = comp[2].to_radians().sin();
                let hf = height as f32 / 2.0;
                let wf = width as f32 / 2.0;
                for idx in (0..warp.len()).step_by(2) {
                    if idx + 1 >= warp.len() { break; }
                    let cy = warp[idx];
                    let cx = warp[idx + 1];
                    let ry = cy * cos_r - cx * sin_r;
                    let rx = cy * sin_r + cx * cos_r;
                    warp[idx]     = (ry * hf - comp[1]) / hf;
                    warp[idx + 1] = (rx * wf - comp[0]) / wf;
                }
                warp_image(&toned, &warp, height as usize, width as usize)
            } else if let Some(grid) = _warp_grid {
                warp_image(&toned, grid, height as usize, width as usize)
            } else {
                toned
            }
        };
        stage_time!("8. Tone+FCS+LDCI+Warp");
        progress!();
        let t_process = t0.elapsed();

        // ── 9. Display ──
        let out_width = if target_width > 0 { target_width } else { width };
        let out_bytes = self.simd.display_output(&toned, width as usize, height as usize, out_width as usize);
        stage_time!("9. Display");
        let t_total = t0.elapsed();

        progress!();

        // Build aux output BEFORE the Ok expression to avoid lock-in-expression issues
        // Use try_lock with fallback to avoid mysterious deadlock on Android
        let scene_category = match self.controller.try_lock() {
            Ok(guard) => Some(guard.scene_category.name().to_string()),
            Err(_) => {
                warn!("CpuEngine: controller lock contention in aux build");
                None
            }
        };
        let (af_phase, vcm_position) = match self.af_engine.try_lock() {
            Ok(guard) => (Some(guard.display_string()), Some(guard.vcm_pos)),
            Err(_) => (None, None),
        };
        let aux = Some(crate::pipeline::IspAuxOutput {
            channel_means: Some(channel_means),
            tone_stats: Some(tone_stats),
            wb_gains: Some(awb_gains),
            histogram: Some(histogram.to_vec()),
            zone_stats: Some(zone_stats),
            focus_metric: Some(focus_metric),
            cct: None,
            ae_gain: Some(ae_gain),
            calibration_stats: Some(calibration_stats.0),
            scene_category,
            af_phase,
            vcm_position,
            eis_compensation: eis_compensation,
        });

        info!("CpuEngine: frame processed ({} bytes) total={:?}", out_bytes.len(), t_total);

        Ok(IspFrame {
            width: out_width,
            height,
            format: cam_types::FrameFormat::Rgba8888,
            data: out_bytes,
            float_data: None,
            aux,
        })
    }
}

// ── Engine registration ──

/// Register the CPU engine factory.
///
/// Called by `cam_isp::init()`.
pub fn register_cpu_engine() {
    let factory = EngineFactory {
        name: "CPU",
        priority: 70,
        create_fn: Box::new(|| Box::new(CpuEngine::new()) as Box<dyn IspEngine>),
    };
    register_engine(factory);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::RawInputBlock;

    #[test]
    #[ignore = "extended — runs full ISP pipeline"]
    fn test_cpu_engine_process() {
        let mut engine = CpuEngine::new();
        assert!(engine.build(Box::new(RawInputBlock::new()), vec![], None, 21).is_ok());

        let w = 16u32;
        let h = 16u32;
        let mut raw_buf = Vec::new();
        for y in 0..h {
            for x in 0..w {
                let val: u16 = if y % 2 == 0 {
                    if x % 2 == 0 { 2000 } else { 4000 }
                } else {
                    if x % 2 == 0 { 4000 } else { 6000 }
                };
                raw_buf.extend_from_slice(&val.to_le_bytes());
            }
        }

        let params = cam_types::ToneParams::default();
        let result = engine.process(
            w, h, w, &raw_buf, 65535.0, w,
            None, &params, None, None, 1.0, 0.0, None, None, None,
        );
        assert!(result.is_ok());
        let frame = result.unwrap();
        assert_eq!(frame.width, w);
        assert_eq!(frame.height, h);
        assert!(!frame.data.is_empty());
        assert!(frame.data[0] > 0);
    }
}
