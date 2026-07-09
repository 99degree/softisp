//! Frame rate control module.
//!
//! Provides intelligent frame rate control to maintain target FPS
//! while allowing pipeline processing to adapt to HW capabilities.

use std::time::{Duration, Instant};

/// Frame rate controller for pipeline processing.
///
/// Tracks frame timing and decides whether to process or skip frames
/// to maintain target FPS without overloading the hardware.
pub struct FrameRateController {
    /// Target frames per second.
    target_fps: f64,
    /// Minimum frame interval in microseconds.
    min_frame_interval_us: u64,
    /// Maximum allowed processing time per frame in microseconds.
    max_processing_time_us: u64,
    /// Frame timing history for adaptive control.
    frame_times: VecDeque<Duration>,
    /// Maximum history size.
    max_history: usize,
    /// Last frame timestamp.
    last_frame_time: Option<Instant>,
    /// Total frames processed.
    frames_processed: u64,
    /// Total frames skipped.
    frames_skipped: u64,
    /// Adaptive mode: adjust based on HW load.
    adaptive: bool,
    /// Current load factor (0.0 = idle, 1.0 = max load).
    load_factor: f64,
}

impl FrameRateController {
    /// Create a new frame rate controller.
    ///
    /// # Arguments
    /// * `target_fps` - Target frames per second (e.g., 30.0, 60.0)
    pub fn new(target_fps: f64) -> Self {
        let min_frame_interval_us = (1_000_000.0 / target_fps) as u64;
        // Allow up to 2x frame time for processing (adaptive headroom)
        let max_processing_time_us = min_frame_interval_us * 2;

        Self {
            target_fps,
            min_frame_interval_us,
            max_processing_time_us,
            frame_times: VecDeque::with_capacity(60),
            max_history: 60,
            last_frame_time: None,
            frames_processed: 0,
            frames_skipped: 0,
            adaptive: true,
            load_factor: 0.0,
        }
    }

    /// Create a controller with fixed (non-adaptive) frame rate.
    pub fn new_fixed(target_fps: f64) -> Self {
        let min_frame_interval_us = (1_000_000.0 / target_fps) as u64;
        Self {
            target_fps,
            min_frame_interval_us,
            max_processing_time_us: min_frame_interval_us,
            frame_times: VecDeque::with_capacity(60),
            max_history: 60,
            last_frame_time: None,
            frames_processed: 0,
            frames_skipped: 0,
            adaptive: false,
            load_factor: 0.0,
        }
    }

    /// Check if a frame should be processed.
    ///
    /// Returns `true` if enough time has passed since the last frame,
    /// or if this is the first frame.
    pub fn should_process(&mut self) -> bool {
        let now = Instant::now();

        match self.last_frame_time {
            None => {
                // First frame, always process
                self.last_frame_time = Some(now);
                true
            }
            Some(last) => {
                let elapsed = now.duration_since(last);
                let elapsed_us = elapsed.as_micros() as u64;

                if elapsed_us >= self.min_frame_interval_us {
                    // Enough time has passed
                    self.last_frame_time = Some(now);
                    true
                } else {
                    // Too soon, skip frame
                    self.frames_skipped += 1;
                    false
                }
            }
        }
    }

    /// Record frame processing time.
    ///
    /// Call this after processing a frame to update timing statistics
    /// and adaptive load calculation.
    pub fn record_frame(&mut self, processing_time: Duration) {
        self.frames_processed += 1;
        self.last_frame_time = Some(Instant::now());

        // Add to history
        if self.frame_times.len() >= self.max_history {
            self.frame_times.pop_front();
        }
        self.frame_times.push_back(processing_time);

        // Update adaptive load factor
        if self.adaptive {
            self.update_load_factor();
        }
    }

    /// Update load factor based on recent frame times.
    fn update_load_factor(&mut self) {
        if self.frame_times.is_empty() {
            self.load_factor = 0.0;
            return;
        }

        // Calculate average frame time
        let total_us: u64 = self.frame_times.iter()
            .map(|d| d.as_micros() as u64)
            .sum();
        let avg_us = total_us / self.frame_times.len() as u64;

        // Load factor = actual time / max allowed time
        self.load_factor = (avg_us as f64) / (self.max_processing_time_us as f64);
        self.load_factor = self.load_factor.min(1.0);
    }

    /// Get current load factor (0.0 = idle, 1.0 = max load).
    pub fn load_factor(&self) -> f64 {
        self.load_factor
    }

    /// Check if we're under heavy load.
    pub fn is_heavy_load(&self) -> bool {
        self.load_factor > 0.8
    }

    /// Check if we're at risk of missing the target FPS.
    pub fn is_frame_drop_risk(&self) -> bool {
        if self.frame_times.is_empty() {
            return false;
        }

        // Check if recent frames are taking too long
        let recent_avg: f64 = self.frame_times.iter()
            .rev()
            .take(5)
            .map(|d| d.as_micros() as f64)
            .sum::<f64>() / 5.0;

        recent_avg > self.min_frame_interval_us as f64 * 0.9
    }

    /// Get current FPS based on recent frame times.
    pub fn current_fps(&self) -> f64 {
        if self.frame_times.is_empty() {
            return 0.0;
        }

        let total_us: u64 = self.frame_times.iter()
            .map(|d| d.as_micros() as u64)
            .sum();
        let avg_us = total_us / self.frame_times.len() as u64;

        if avg_us > 0 {
            1_000_000.0 / avg_us as f64
        } else {
            0.0
        }
    }

    /// Get target FPS.
    pub fn target_fps(&self) -> f64 {
        self.target_fps
    }

    /// Get frames processed count.
    pub fn frames_processed(&self) -> u64 {
        self.frames_processed
    }

    /// Get frames skipped count.
    pub fn frames_skipped(&self) -> u64 {
        self.frames_skipped
    }

    /// Get total frames (processed + skipped).
    pub fn total_frames(&self) -> u64 {
        self.frames_processed + self.frames_skipped
    }

    /// Get skip ratio (0.0 = no skips, 1.0 = all skipped).
    pub fn skip_ratio(&self) -> f64 {
        let total = self.total_frames();
        if total == 0 {
            0.0
        } else {
            self.frames_skipped as f64 / total as f64
        }
    }

    /// Reset statistics.
    pub fn reset(&mut self) {
        self.frame_times.clear();
        self.last_frame_time = None;
        self.frames_processed = 0;
        self.frames_skipped = 0;
        self.load_factor = 0.0;
    }

    /// Enable or disable adaptive mode.
    pub fn set_adaptive(&mut self, adaptive: bool) {
        self.adaptive = adaptive;
    }

    /// Get recommended pipeline complexity based on current load.
    ///
    /// Returns a value from 0.0 to 1.0 indicating the recommended
    /// fraction of the full pipeline to use:
    /// - 1.0 = full pipeline (low load)
    /// - 0.5 = simplified pipeline (high load)
    /// - 0.25 = minimal pipeline (critical load)
    pub fn recommended_complexity(&self) -> f64 {
        if !self.adaptive {
            return 1.0;
        }

        match self.load_factor {
            l if l < 0.5 => 1.0,       // Full pipeline
            l if l < 0.7 => 0.75,      // Slightly reduced
            l if l < 0.85 => 0.5,      // Half complexity
            l if l < 0.95 => 0.25,     // Minimal
            _ => 0.1,                   // Emergency minimal
        }
    }

    /// Get timing statistics.
    pub fn stats(&self) -> FrameRateStats {
        let avg_frame_time = if self.frame_times.is_empty() {
            Duration::ZERO
        } else {
            let total_us: u64 = self.frame_times.iter()
                .map(|d| d.as_micros() as u64)
                .sum();
            Duration::from_micros(total_us / self.frame_times.len() as u64)
        };

        let max_frame_time = self.frame_times.iter()
            .copied()
            .max()
            .unwrap_or(Duration::ZERO);

        let min_frame_time = self.frame_times.iter()
            .copied()
            .min()
            .unwrap_or(Duration::ZERO);

        FrameRateStats {
            target_fps: self.target_fps,
            current_fps: self.current_fps(),
            frames_processed: self.frames_processed,
            frames_skipped: self.frames_skipped,
            avg_frame_time,
            min_frame_time,
            max_frame_time,
            load_factor: self.load_factor,
        }
    }
}

use std::collections::VecDeque;

/// Frame rate statistics.
#[derive(Debug, Clone)]
pub struct FrameRateStats {
    /// Target FPS.
    pub target_fps: f64,
    /// Current measured FPS.
    pub current_fps: f64,
    /// Total frames processed.
    pub frames_processed: u64,
    /// Total frames skipped.
    pub frames_skipped: u64,
    /// Average frame processing time.
    pub avg_frame_time: Duration,
    /// Minimum frame processing time.
    pub min_frame_time: Duration,
    /// Maximum frame processing time.
    pub max_frame_time: Duration,
    /// Current load factor (0.0-1.0).
    pub load_factor: f64,
}

impl std::fmt::Display for FrameRateStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "FPS: {:.1}/{:.1} | Frames: {} processed, {} skipped | Load: {:.0}%% | Time: avg={:.1}ms, min={:.1}ms, max={:.1}ms",
            self.current_fps,
            self.target_fps,
            self.frames_processed,
            self.frames_skipped,
            self.load_factor,
            self.avg_frame_time.as_secs_f64() * 1000.0,
            self.min_frame_time.as_secs_f64() * 1000.0,
            self.max_frame_time.as_secs_f64() * 1000.0,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_frame_rate_controller_creation() {
        let ctrl = FrameRateController::new(30.0);
        assert_eq!(ctrl.target_fps(), 30.0);
        assert_eq!(ctrl.frames_processed(), 0);
        assert_eq!(ctrl.frames_skipped(), 0);
    }

    #[test]
    fn test_should_process_first_frame() {
        let mut ctrl = FrameRateController::new(30.0);
        assert!(ctrl.should_process());
    }

    #[test]
    fn test_should_process_timing() {
        let mut ctrl = FrameRateController::new(30.0);
        assert!(ctrl.should_process());
        ctrl.record_frame(Duration::from_millis(10));

        // Immediately after should skip
        assert!(!ctrl.should_process());

        // After frame interval should process
        thread::sleep(Duration::from_millis(35));
        assert!(ctrl.should_process());
    }

    #[test]
    fn test_load_factor() {
        let mut ctrl = FrameRateController::new(30.0);
        ctrl.record_frame(Duration::from_millis(5));
        assert!(ctrl.load_factor() < 0.5);
    }

    #[test]
    fn test_stats() {
        let mut ctrl = FrameRateController::new(60.0);
        for _ in 0..10 {
            thread::sleep(Duration::from_millis(1));
            ctrl.should_process();
            ctrl.record_frame(Duration::from_millis(1));
        }
        let stats = ctrl.stats();
        assert_eq!(stats.target_fps, 60.0);
        assert!(stats.frames_processed > 0);
    }
}
