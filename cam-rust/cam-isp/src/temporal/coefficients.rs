//! FrameCoefficients — extracted from N-1 frames for processing the Nth frame.

use std::collections::VecDeque;

/// Coefficients extracted from N-1 frames.
#[derive(Debug, Clone)]
pub struct FrameCoefficients {
    /// Frame width.
    pub width: u32,
    /// Frame height.
    pub height: u32,
    /// Number of frames used for extraction.
    pub history_size: usize,
    /// Noise model: per-pixel variance `[H x W]`.
    pub noise_map: Vec<f32>,
    /// Motion map: per-pixel motion intensity `[H x W]`.
    pub motion_map: Vec<f32>,
    /// Gain map: per-pixel gain `[H x W]`.
    pub gain_map: Vec<f32>,
    /// Offset map: per-pixel offset `[H x W]`.
    pub offset_map: Vec<f32>,
    /// Color statistics: `[R_mean, G_mean, B_mean, R_var, G_var, B_var]`.
    pub color_stats: [f32; 6],
    /// Global motion vector `(dx, dy)`.
    pub global_motion: (f32, f32),
    /// Extraction timestamp.
    pub timestamp: std::time::Instant,
}

impl FrameCoefficients {
    /// Create empty coefficients for a given resolution.
    pub fn empty(width: u32, height: u32) -> Self {
        let size = (width * height) as usize;
        Self {
            width,
            height,
            history_size: 0,
            noise_map: vec![0.0; size],
            motion_map: vec![0.0; size],
            gain_map: vec![1.0; size],
            offset_map: vec![0.0; size],
            color_stats: [0.0; 6],
            global_motion: (0.0, 0.0),
            timestamp: std::time::Instant::now(),
        }
    }

    /// Create new coefficients with all maps.
    pub fn new(
        width: u32,
        height: u32,
        history_size: usize,
        noise_map: Vec<f32>,
        motion_map: Vec<f32>,
        gain_map: Vec<f32>,
        offset_map: Vec<f32>,
    ) -> Self {
        Self {
            width,
            height,
            history_size,
            noise_map,
            motion_map,
            gain_map,
            offset_map,
            color_stats: [0.0; 6],
            global_motion: (0.0, 0.0),
            timestamp: std::time::Instant::now(),
        }
    }

    /// Smooth coefficients with another set using exponential moving average.
    pub fn smooth_with(&mut self, other: &FrameCoefficients, alpha: f32) {
        assert_eq!(self.width, other.width);
        assert_eq!(self.height, other.height);
        let beta = 1.0 - alpha;
        for i in 0..self.noise_map.len() {
            self.noise_map[i] = self.noise_map[i] * beta + other.noise_map[i] * alpha;
            self.gain_map[i] = self.gain_map[i] * beta + other.gain_map[i] * alpha;
            self.offset_map[i] = self.offset_map[i] * beta + other.offset_map[i] * alpha;
        }
        for i in 0..6 {
            self.color_stats[i] = self.color_stats[i] * beta + other.color_stats[i] * alpha;
        }
        self.global_motion.0 = self.global_motion.0 * beta + other.global_motion.0 * alpha;
        self.global_motion.1 = self.global_motion.1 * beta + other.global_motion.1 * alpha;
    }
}

/// History buffer for storing N-1 frames.
pub struct FrameHistory {
    frames: VecDeque<super::frame::TemporalFrame>,
    max_size: usize,
}

impl FrameHistory {
    /// Create a new history buffer.
    pub fn new(max_size: usize) -> Self {
        Self {
            frames: VecDeque::with_capacity(max_size),
            max_size,
        }
    }

    /// Push a frame into history.
    pub fn push(&mut self, frame: super::frame::TemporalFrame) {
        if self.frames.len() >= self.max_size {
            self.frames.pop_front();
        }
        self.frames.push_back(frame);
    }

    /// Get the N-1 frame (second-to-last).
    pub fn prev_frame(&self) -> Option<&super::frame::TemporalFrame> {
        if self.frames.len() >= 2 {
            self.frames.get(self.frames.len() - 2)
        } else {
            None
        }
    }

    /// Get all frames.
    pub fn frames(&self) -> &VecDeque<super::frame::TemporalFrame> {
        &self.frames
    }

    /// Get history size.
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// Check if history is empty.
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }
}
