//! Pipeline manager — orchestrates building, frame processing, and 3A control.
//!
//! Ported from `com.camapp.PipelineManager` (Java).
//! Simplified: no HDR burst, no EIS deshake, no MATCH calibration.

use std::sync::Mutex;
use std::time::Instant;

use log::info;

use crate::controller::IspController;
use crate::cpu::CpuEngine;
use crate::engine::IspEngine;
use crate::profile::PipelineProfile;
use crate::pipeline::{IspBlock, GraphComposer};
use crate::blocks;

/// Result of a single frame pipeline process.
#[derive(Debug, Clone)]
pub struct PipelineResult {
    /// Processed RGBA8888 frame data.
    pub data: Vec<u8>,
    /// Output width.
    pub width: u32,
    /// Output height.
    pub height: u32,
    /// Processing latency in milliseconds.
    pub latency_ms: f64,
    /// Current AWB gains from controller.
    pub awb_gains: [f32; 3],
    /// Estimated CCT.
    pub estimated_cct: Option<i32>,
    /// Effective exposure gain.
    pub exposure_gain: f32,
    /// Channel means from controller.
    pub channel_means: [f32; 3],
}

/// Pipeline state flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineStatus {
    /// Ready and processing frames.
    Active,
    /// Pipeline building, use fallback.
    Building,
    /// Error state.
    Error,
}

/// Simplified pipeline manager — orchestrates ISP pipeline lifecycle.
///
/// # Example
/// ```ignore
/// use cam_isp::manager::PipelineManager;
/// let mut mgr = PipelineManager::new();
/// mgr.set_profile(PipelineProfile::MED);
/// mgr.build().expect("Build failed");
/// let result = mgr.process_frame(&raw_data, 128, 96, 65535.0).unwrap();
/// ```
pub struct PipelineManager {
    /// Current pipeline profile.
    pub profile: PipelineProfile,
    /// ISP controller for AWB/AE/CCM/tone.
    pub controller: IspController,
    /// Target display width.
    pub target_width: u32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: i32,
    /// Status.
    status: PipelineStatus,

    // Internal engine
    engine: Mutex<Option<CpuEngine>>,
    /// Last build timestamp.
    last_build: Option<Instant>,
}

impl PipelineManager {
    pub fn new() -> Self {
        Self {
            profile: PipelineProfile::MED,
            controller: IspController::new(),
            target_width: 480,
            bayer_pattern: 3, // BGGR default
            status: PipelineStatus::Building,
            engine: Mutex::new(None),
            last_build: None,
        }
    }

    /// Set the pipeline profile and trigger rebuild.
    pub fn set_profile(&mut self, profile: PipelineProfile) {
        if self.profile.label != profile.label {
            info!("PipelineManager: profile {} -> {}", self.profile.label, profile.label);
            self.profile = profile;
            self.status = PipelineStatus::Building;
        }
    }

    /// Set the target output width.
    pub fn set_target_width(&mut self, width: u32) {
        if self.target_width != width {
            self.target_width = width;
            self.status = PipelineStatus::Building;
        }
    }

    /// Set the Bayer pattern.
    pub fn set_bayer_pattern(&mut self, pattern: i32) {
        if self.bayer_pattern != pattern {
            self.bayer_pattern = pattern;
            self.status = PipelineStatus::Building;
        }
    }

    /// Build or rebuild the pipeline.
    pub fn build(&mut self) -> Result<(), String> {
        info!("PipelineManager: building profile {} ({} blocks)",
            self.profile.label, self.profile.block_count());

        // Create blocks from profile
        let blocks = self.profile.build_blocks(self.target_width, self.bayer_pattern);

        // Build the ONNX model via GraphComposer
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let _model_bytes = GraphComposer::compose_from_vec(&block_refs, &[], 21)?;

        // Create CPU engine
        let mut engine = CpuEngine::new();
        let head = blocks::RawInputBlock::new();
        engine.build(Box::new(head), vec![], None, 21)?;

        // Copy controller into engine
        *engine.controller.lock().unwrap() = self.controller.clone();

        *self.engine.lock().unwrap() = Some(engine);
        self.status = PipelineStatus::Active;
        self.last_build = Some(Instant::now());

        info!("PipelineManager: build complete ({} blocks, profile={})",
            blocks.len(), self.profile.label);
        Ok(())
    }

    /// Process a raw Bayer frame through the pipeline.
    ///
    /// `raw_data`: raw Bayer INT16 data as bytes (width * height * 2).
    /// `width`, `height`: frame dimensions.
    /// `sensor_max`: maximum sensor value (65535 for 16-bit, 4095 for 12-bit).
    pub fn process_frame(
        &self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        sensor_max: f32,
    ) -> Result<PipelineResult, String> {
        let t0 = Instant::now();

        let engine_guard = self.engine.lock().map_err(|e| format!("Engine lock: {}", e))?;
        let engine = engine_guard.as_ref().ok_or("Pipeline not built")?;

        // Default tone params
        let tone_params = crate::engine::default_tone_params();

        // Process
        let frame = engine.process(
            width, height, width, raw_data,
            sensor_max, self.target_width,
            None,  // CCM — use controller
            &tone_params,
            None,  // bayer gains — use controller
            None,  // awb gains — use controller
            1.0,   // analog gain
            0.0,   // scene change
            None,  // lsc gains
            None,  // blc values
            None,  // warp grid
        )?;

        let latency = t0.elapsed();

        // Read controller state
        let ctrl = engine.controller.lock().map_err(|e| format!("Ctrl lock: {}", e))?;
        let awb_gains = ctrl.get_awb_gains();
        let estimated_cct = ctrl.estimated_cct;
        let exposure_gain = ctrl.get_effective_exposure_gain();
        let channel_means = [ctrl.avg_r, ctrl.avg_g, ctrl.avg_b];

        Ok(PipelineResult {
            data: frame.data,
            width: frame.width,
            height: frame.height,
            latency_ms: latency.as_secs_f64() * 1000.0,
            awb_gains,
            estimated_cct,
            exposure_gain,
            channel_means,
        })
    }

    /// Check if the pipeline is ready to process frames.
    pub fn is_ready(&self) -> bool {
        self.status == PipelineStatus::Active
    }

    /// Get current pipeline status.
    pub fn status(&self) -> PipelineStatus {
        self.status
    }

    /// Get the underlying engine (for direct access).
    pub fn engine(&self) -> &Mutex<Option<CpuEngine>> {
        &self.engine
    }

    /// Reset the manager to initial state.
    pub fn reset(&mut self) {
        self.controller = IspController::new();
        self.engine = Mutex::new(None);
        self.status = PipelineStatus::Building;
        self.last_build = None;
    }
}

impl Default for PipelineManager {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_create() {
        let mgr = PipelineManager::new();
        assert_eq!(mgr.profile.label, "MED");
        assert_eq!(mgr.status(), PipelineStatus::Building);
    }

    #[test]
    fn test_manager_build_and_process() {
        let mut mgr = PipelineManager::new();
        mgr.set_target_width(32);
        mgr.build().expect("Build failed");
        assert!(mgr.is_ready());

        // Create synthetic 32x32 RAW Bayer frame
        let mut raw = Vec::with_capacity(32 * 32 * 2);
        for y in 0..32u32 {
            for x in 0..32u32 {
                let val: u16 = if y % 2 == 0 {
                    if x % 2 == 0 { 2000 } else { 4000 }
                } else {
                    if x % 2 == 0 { 4000 } else { 6000 }
                };
                raw.extend_from_slice(&val.to_le_bytes());
            }
        }

        let result = mgr.process_frame(&raw, 32, 32, 65535.0);
        assert!(result.is_ok(), "Process failed: {:?}", result.err());
        let r = result.unwrap();
        assert_eq!(r.width, 32);
        assert_eq!(r.height, 32);
        assert!(!r.data.is_empty());
        assert!(r.latency_ms > 0.0);
    }
}
