//! TemporalPlugin — complete extraction + processing bundles.

use super::extractors::*;
use super::processors::*;

/// Trait for a complete temporal plugin (extractor + processor).
pub trait TemporalPlugin: Send + Sync {
    fn extractor(&self) -> &dyn CoeffExtractor;
    fn processor(&self) -> &dyn TemporalProcessor;
    fn name(&self) -> &str;
}

/// Complete temporal denoise plugin.
pub struct TemporalDenoisePlugin {
    extractor: NoiseExtractor,
    processor: TemporalDenoiseProcessor,
}

impl TemporalDenoisePlugin {
    pub fn new(strength: f32) -> Self {
        Self {
            extractor: NoiseExtractor::new(0.1),
            processor: TemporalDenoiseProcessor::new(strength),
        }
    }
}

impl TemporalPlugin for TemporalDenoisePlugin {
    fn extractor(&self) -> &dyn CoeffExtractor {
        &self.extractor
    }
    fn processor(&self) -> &dyn TemporalProcessor {
        &self.processor
    }
    fn name(&self) -> &str {
        "TemporalDenoise"
    }
}

/// Complete motion-adaptive plugin.
pub struct MotionAdaptivePlugin {
    extractor: MotionExtractor,
    processor: MotionAdaptiveProcessor,
}

impl MotionAdaptivePlugin {
    pub fn new(motion_strength: f32, static_strength: f32) -> Self {
        Self {
            extractor: MotionExtractor::new(16),
            processor: MotionAdaptiveProcessor::new(motion_strength, static_strength),
        }
    }
}

impl TemporalPlugin for MotionAdaptivePlugin {
    fn extractor(&self) -> &dyn CoeffExtractor {
        &self.extractor
    }
    fn processor(&self) -> &dyn TemporalProcessor {
        &self.processor
    }
    fn name(&self) -> &str {
        "MotionAdaptive"
    }
}

/// Complete gain tracking plugin.
pub struct GainTrackerPlugin {
    extractor: GainExtractor,
    processor: GainCalibrationProcessor,
}

impl GainTrackerPlugin {
    pub fn new(target_gray: f32) -> Self {
        Self {
            extractor: GainExtractor::new(target_gray),
            processor: GainCalibrationProcessor,
        }
    }
}

impl TemporalPlugin for GainTrackerPlugin {
    fn extractor(&self) -> &dyn CoeffExtractor {
        &self.extractor
    }
    fn processor(&self) -> &dyn TemporalProcessor {
        &self.processor
    }
    fn name(&self) -> &str {
        "GainTracker"
    }
}

/// Complete color tracking plugin.
pub struct ColorTrackerPlugin {
    extractor: ColorExtractor,
    processor: ColorBalanceProcessor,
}

impl ColorTrackerPlugin {
    pub fn new(target_rg: f32, target_bg: f32) -> Self {
        Self {
            extractor: ColorExtractor::new(0.1),
            processor: ColorBalanceProcessor::new(target_rg, target_bg),
        }
    }
}

impl TemporalPlugin for ColorTrackerPlugin {
    fn extractor(&self) -> &dyn CoeffExtractor {
        &self.extractor
    }
    fn processor(&self) -> &dyn TemporalProcessor {
        &self.processor
    }
    fn name(&self) -> &str {
        "ColorTracker"
    }
}
