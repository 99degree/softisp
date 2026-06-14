//! Stub camera adapter — generates synthetic test frames in multiple formats.
//!
//! Useful for testing the ISP pipeline without physical hardware.
//! Supports RGB (RGBA), YUV (NV21), RAW (Bayer), RAW10, and RAW12 output.


use std::time::{SystemTime, UNIX_EPOCH};

use cam_types::{CameraSourceType, FrameFormat};

use crate::camera::{ByteFrame, CameraState, FrameCallback, ICameraAdapter, StreamConfig};

/// Stub frame pattern type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StubPattern {
    /// Solid color (default: gray)
    Solid,
    /// Color bars (ITU-R BT.601 test pattern)
    ColorBars,
    /// Checkerboard
    Checkerboard,
    /// Gradient (top-to-bottom)
    Gradient,
    /// Bayer raw with color gradient (for raw modes)
    BayerGradient,
}

/// Stub camera adapter — generates test frames in any format.
pub struct StubAdapter {
    /// Current stream configuration.
    config: Option<StreamConfig>,
    /// Frame callback.
    callback: Option<FrameCallback>,
    /// Current state.
    state: CameraState,
    /// Device name.
    name: String,
    /// Pattern type.
    pattern: StubPattern,
    /// Frame counter (for changing patterns).
    frame_count: u64,
    /// Simulated FPS interval.
    #[allow(dead_code)]
    fps: u32,
    /// Whether to add random noise to raw frames.
    add_noise: bool,
}

impl StubAdapter {
    /// Create a new stub adapter.
    pub fn new(name: &str) -> Self {
        Self {
            config: None,
            callback: None,
            state: CameraState::Closed,
            name: name.to_string(),
            pattern: StubPattern::ColorBars,
            frame_count: 0,
            fps: 30,
            add_noise: false,
        }
    }

    /// Set the test pattern.
    pub fn set_pattern(&mut self, pattern: StubPattern) {
        self.pattern = pattern;
    }

    /// Enable/disable noise on raw frames.
    pub fn set_noise(&mut self, enable: bool) {
        self.add_noise = enable;
    }

    /// Generate a frame based on the current configuration.
    pub fn generate_frame(&mut self) -> Option<ByteFrame> {
        let cfg = self.config.as_ref()?;
        let data = self.render_frame(cfg.width, cfg.height, cfg.format);
        self.frame_count += 1;

        Some(ByteFrame {
            data,
            width: cfg.width,
            height: cfg.height,
            format: cfg.format,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        })
    }

    fn render_frame(&self, width: u32, height: u32, format: FrameFormat) -> Vec<u8> {
        let w = width as usize;
        let h = height as usize;

        match format {
            FrameFormat::Rgba8888 => self.render_rgba(w, h),
            FrameFormat::Rgb888 => self.render_rgba(w, h),   // same as RGBA but with stride=3
            FrameFormat::RawSensor => self.render_raw16(w, h),
            FrameFormat::Raw10 => vec![0u8; w * h * 2],       // raw placeholder
            FrameFormat::Raw12 => vec![0u8; w * h * 2],       // raw placeholder
            FrameFormat::Yuv420888 => self.render_nv21(w, h),
            FrameFormat::NchwFloat => vec![0u8; w * h * 4],
        }
    }

    fn render_rgba(&self, w: usize, h: usize) -> Vec<u8> {
        let mut buf = vec![0u8; w * h * 4];
        for y in 0..h {
            for x in 0..w {
                let idx = (y * w + x) * 4;
                match self.pattern {
                    StubPattern::Solid => {
                        let v = 128u8;
                        buf[idx..idx + 4].copy_from_slice(&[v, v, v, 255]);
                    }
                    StubPattern::ColorBars => {
                        let bar = (x * 8 / w) as u8;
                        let (r, g, b) = match bar {
                            0 => (255, 255, 255), // white
                            1 => (255, 255, 0),   // yellow
                            2 => (0, 255, 255),   // cyan
                            3 => (0, 255, 0),     // green
                            4 => (255, 0, 255),   // magenta
                            5 => (255, 0, 0),     // red
                            6 => (0, 0, 255),     // blue
                            _ => (0, 0, 0),        // black
                        };
                        buf[idx..idx + 4].copy_from_slice(&[r, g, b, 255]);
                    }
                    StubPattern::Checkerboard => {
                        let block = 32;
                        let toggle = ((x / block) + (y / block)) % 2 == 0;
                        let v = if toggle { 255 } else { 32 };
                        buf[idx..idx + 4].copy_from_slice(&[v, v, v, 255]);
                    }
                    StubPattern::Gradient | StubPattern::BayerGradient => {
                        let v = ((x + y) * 255 / (w + h)) as u8;
                        buf[idx..idx + 4].copy_from_slice(&[v, v, v, 255]);
                    }
                }
            }
        }
        buf
    }

    fn render_raw16(&self, w: usize, h: usize) -> Vec<u8> {
        // 16-bit Bayer (BGGR pattern)
        let mut buf = vec![0u8; w * h * 2];
        for y in 0..h {
            for x in 0..w {
                let idx = (y * w + x) * 2;
                // BGGR bayer pattern
                let bayer_val = match ((y & 1), (x & 1)) {
                    (0, 0) => self.bayer_color(x, y, w, h, 0.8),  // B
                    (0, 1) => self.bayer_color(x, y, w, h, 0.3),  // Gb
                    (1, 0) => self.bayer_color(x, y, w, h, 0.3),  // Gr
                    (1, 1) => self.bayer_color(x, y, w, h, 0.0),  // R
                    _ => unreachable!(),
                };
                let val = (bayer_val.clamp(0.0, 1.0) * 65535.0) as u16;
                buf[idx..idx + 2].copy_from_slice(&val.to_le_bytes());
            }
        }
        // Optional noise injection
        if self.add_noise {
            use std::time::{SystemTime, UNIX_EPOCH};
            let seed = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;
            let mut rng = XorShift64 { state: seed };
            for chunk in buf.chunks_exact_mut(2) {
                let val = u16::from_le_bytes([chunk[0], chunk[1]]);
                let noise = (rng.next() % 513) as i32 - 256; // ±256
                let new_val = (val as i32 + noise).clamp(0, 65535) as u16;
                chunk.copy_from_slice(&new_val.to_le_bytes());
            }
        }
        buf
    }

    fn render_nv21(&self, w: usize, h: usize) -> Vec<u8> {
        // NV21: Y plane (w*h) + UV interleaved (w*h/2)
        let y_size = w * h;
        let uv_size = w * h / 2;
        let mut buf = vec![0u8; y_size + uv_size];

        // Y plane
        for y in 0..h {
            for x in 0..w {
                let idx = y * w + x;
                match self.pattern {
                    StubPattern::ColorBars => {
                        let bar = (x * 8 / w) as u8;
                        buf[idx] = match bar {
                            0 => 235, 1 => 210, 2 => 170,
                            3 => 145, 4 => 106, 5 => 81,
                            6 => 41, _ => 16,
                        };
                    }
                    _ => {
                        buf[idx] = ((x + y) * 255 / (w + h)) as u8;
                    }
                }
            }
        }
        // UV plane (NV21: V first, then U)
        for y in (0..h).step_by(2) {
            for x in (0..w).step_by(2) {
                let idx = y_size + (y / 2) * w + x;
                buf[idx] = 128; // V
                buf[idx + 1] = 128; // U
            }
        }
        buf
    }

    /// Bayer color value with a channel bias for color target.
    fn bayer_color(&self, x: usize, y: usize, w: usize, h: usize, channel_bias: f64) -> f64 {
        // Base gradient 0..1
        let grad = (x + y) as f64 / (w + h) as f64;
        // Add channel-specific boost
        let biased = grad * (0.5 + channel_bias * 0.5);
        biased.min(1.0)
    }
}

/// Simple pseudo-random for noise injection.
struct XorShift64 {
    state: u64,
}
impl XorShift64 {
    fn next(&mut self) -> u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state as u32
    }
}

impl ICameraAdapter for StubAdapter {
    fn source_type(&self) -> CameraSourceType {
        CameraSourceType::Stub
    }

    fn open(&mut self, config: &StreamConfig) -> Result<(), String> {
        self.config = Some(config.clone());
        self.state = CameraState::Open;
        log::info!("Stub '{}' opened: {}x{} {:?}", self.name, config.width, config.height, config.format);
        Ok(())
    }

    fn close(&mut self) {
        self.state = CameraState::Closed;
        self.config = None;
        log::info!("Stub '{}' closed", self.name);
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        self.state = CameraState::Streaming;
        log::info!("Stub '{}' streaming started", self.name);
        Ok(())
    }

    fn stop_streaming(&mut self) {
        self.state = CameraState::Open;
        log::info!("Stub '{}' streaming stopped", self.name);
    }

    fn set_frame_callback(&mut self, callback: FrameCallback) {
        self.callback = Some(callback);
    }

    fn state(&self) -> CameraState {
        self.state
    }

    fn device_name(&self) -> &str {
        &self.name
    }

    fn send_frame(&self, _frame: ByteFrame) -> Result<(), String> {
        // Stub adapter doesn't accept external frames; it generates its own.
        Err("Stub adapter generates its own frames — use generate_frame()".to_string())
    }
}
