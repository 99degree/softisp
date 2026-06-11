//! Debug utility module.
//! Ported from com.camcore.DebugService

use log::info;

/// Debug service for pipeline telemetry.
pub struct DebugService;

impl DebugService {
    pub fn new() -> Self {
        Self
    }

    /// Log pipeline frame statistics.
    pub fn log_frame_stats(&self, frame_number: u64, width: u32, height: u32, processing_ms: f64) {
        info!("Frame[{}] {}x{} processed in {:.2}ms", frame_number, width, height, processing_ms);
    }
}
