//! TemporalFrame — frame for temporal processing.

/// Frame for temporal processing.
#[derive(Debug, Clone)]
pub struct TemporalFrame {
    /// Frame data (RGB interleaved).
    pub data: Vec<f32>,
    /// Frame width.
    pub width: u32,
    /// Frame height.
    pub height: u32,
    /// Frame number.
    pub frame_number: u64,
    /// Timestamp.
    pub timestamp: std::time::Instant,
}

impl TemporalFrame {
    /// Create a new frame.
    pub fn new(data: Vec<f32>, width: u32, height: u32, frame_number: u64) -> Self {
        Self {
            data,
            width,
            height,
            frame_number,
            timestamp: std::time::Instant::now(),
        }
    }

    /// Get pixel value at (x, y, channel).
    pub fn get_pixel(&self, x: u32, y: u32, channel: usize) -> f32 {
        let idx = ((y * self.width + x) * 3 + channel as u32) as usize;
        self.data.get(idx).copied().unwrap_or(0.0)
    }

    /// Set pixel value at (x, y, channel).
    pub fn set_pixel(&mut self, x: u32, y: u32, channel: usize, value: f32) {
        let idx = ((y * self.width + x) * 3 + channel as u32) as usize;
        if idx < self.data.len() {
            self.data[idx] = value;
        }
    }

    /// Get size in bytes.
    pub fn size_bytes(&self) -> usize {
        self.data.len() * std::mem::size_of::<f32>()
    }
}
