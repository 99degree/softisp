//! Rolling statistics buffer for the ISP controller.
//!
//! ``RollingStats`` carries per-frame statistics extracted by the inference
//! pipeline (zone means, channel averages, tone distribution, histogram).
//! The ``IspController`` triple-buffers these: one slot for the engine to
//! write into, one for processing, one kept ready for the next frame.

/// Rolling statistics from a single inference pass.
///
/// Double-buffered in ``IspController`` (A/B slots) so the engine
/// can write frame N's stats while the controller reads frame N-1's.
#[derive(Debug, Clone)]
pub struct RollingStats {
    /// RGB channel means `[R, G, B]`
    pub channel_means: [f32; 3],
    /// Tone stats: [mean_lum, min_lum, max_lum, clip_frac, shadow_frac, total_px]
    pub tone_stats: [f32; 6],
    /// Coarse histogram bins `[16]`
    pub histogram: [f32; 16],
    /// Whether histogram has valid data
    pub histogram_valid: bool,
    /// Zone RGB means (flattened: rows × cols × 3)
    pub zone_stats: Vec<Vec<[f32; 3]>>,
    /// Whether zone stats are initialized
    pub zone_stats_valid: bool,
}

impl RollingStats {
    pub fn new() -> Self {
        Self {
            channel_means: [0.5, 0.5, 0.5],
            tone_stats: [0.0; 6],
            histogram: [0.0; 16],
            histogram_valid: false,
            zone_stats: Vec::new(),
            zone_stats_valid: false,
        }
    }
}

impl Default for RollingStats {
    fn default() -> Self {
        Self::new()
    }
}
