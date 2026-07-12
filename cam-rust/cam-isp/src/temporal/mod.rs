//! Temporal processing module.
//!
//! Extracts coefficients from N-1 frames and uses them to process the Nth frame.
//! Split into atomic helper classes per algorithm:
//!
//! - `FrameCoefficients` — extracted parameters from history
//! - `FrameHistory` — stores N-1 frames
//! - `TemporalFrame` — frame data wrapper
//! - `extractors` — coefficient extraction (Noise, Motion, Gain, Color)
//! - `processors` — frame processing (Denoise, MotionAdaptive, GainCalibration, ColorBalance)
//! - `plugin` — complete bundles (TemporalDenoise, MotionAdaptive, GainTracker, ColorTracker)
//! - `pipeline` — thread-wise FramePipeline

pub mod coefficients;
pub mod extractors;
pub mod frame;
pub mod pipeline;
pub mod plugin;
pub mod processors;

pub use coefficients::{FrameCoefficients, FrameHistory};
pub use extractors::{
    CoeffExtractor, ColorExtractor, GainExtractor, MotionExtractor, NoiseExtractor,
};
pub use frame::TemporalFrame;
pub use pipeline::FramePipeline;
pub use plugin::{
    ColorTrackerPlugin, GainTrackerPlugin, MotionAdaptivePlugin, TemporalDenoisePlugin,
    TemporalPlugin,
};
pub use processors::{
    ColorBalanceProcessor, GainCalibrationProcessor, MotionAdaptiveProcessor,
    TemporalDenoiseProcessor, TemporalProcessor,
};
