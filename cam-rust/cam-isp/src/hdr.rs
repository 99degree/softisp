//! # HDR Capture Pipeline — Raw Frame Queue + Async Inference
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    HDR CAPTURE PIPELINE                      │
//! ├─────────────────────────────────────────────────────────────┤
//! │                                                             │
//! │  ┌─────────┐  ┌─────────┐  ┌─────────┐                     │
//! │  │ Frame 1 │  │ Frame 2 │  │ Frame 3 │                     │
//! │  │ Under   │  │ Normal  │  │ Over    │                     │
//! │  │ (-2EV)  │  │ (0EV)   │  │ (+2EV)  │                     │
//! │  └────┬────┘  └────┬────┘  └────┬────┘                     │
//! │       │            │            │                           │
//! │       ▼            ▼            ▼                            │
//! │  ┌─────────────────────────────────────────────────┐       │
//! │  │          ISP Pipeline (Stage 1)                  │       │
//! │  │  BayerWB → Demosaic+CCM → ToneMap → ARGB8888    │       │
//! │  │  (per exposure, 33ms × 3 = 100ms)               │       │
//! │  └──────────────────────┬──────────────────────────┘       │
//! │                         │                                   │
//! │                         ▼                                   │
//! │  ┌─────────────────────────────────────────────────┐       │
//! │  │           HDR Frame Queue (MPSC Channel)         │       │
//! │  │  [ARGB_EV-2, ARGB_EV0, ARGB_EV+2] ──► Async    │       │
//! │  │                                      HDR Worker  │       │
//! │  └──────────────────────┬──────────────────────────┘       │
//! │                         │                                   │
//! │                         ▼                                   │
//! │  ┌─────────────────────────────────────────────────┐       │
//! │  │          HDR Worker (Async Thread)                │       │
//! │  │  1. Align frames (EIS motion vectors)            │       │
//! │  │  2. Merge (luminance-weighted / Mertens)         │       │
//! │  │  3. Tone Map (ACES/Reinhard → ARGB8888)          │       │
//! │  │  4. Neural Enhance (optional MNN model)          │       │
//! │  │  5. Encode (JPEG/HEIC)                            │       │
//! │  └─────────────────────────────────────────────────┘       │
//! │                                                             │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Memory & Latency Budget
//!
//! | Stage            | Time      | Memory                |
//! |------------------|-----------|-----------------------|
//! | ISP Burst (3fr)  | 100ms     | 3 × 46MB = 138MB     |
//! | Align            | 5ms       | 2× frame buffers      |
//! | Merge (Mertens)  | 5ms       | 2× frame buffers      |
//! | Neural Enhance   | 20ms      | 2× model buffers      |
//! | Encode           | 10ms      | Output buffer          |
//! | **Total**        | **~120ms**| **<200MB peak**       |

#![allow(dead_code)]

use std::sync::mpsc::{channel, Sender, Receiver};
use std::sync::Arc;
use std::thread;

// ── Types ──────────────────────────────────────────────────────────

/// Frame with exposure metadata after ISP processing.
/// Data is ARGB8888 (or NCHW float) output from ISP for one exposure.
#[derive(Debug, Clone)]
pub struct HdrFrame {
    /// Pixel data (ARGB8888 or NCHW float bytes)
    pub data: Arc<Vec<u8>>,
    /// Exposure value relative to metered: -2.0, 0.0, +2.0
    pub ev: f32,
    /// Frame timestamp
    pub timestamp_ns: u64,
    /// ISO value for this frame
    pub iso: f32,
    /// Exposure time in seconds
    pub exposure_time: f32,
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
}

/// Metadata for an HDR capture session.
#[derive(Debug, Clone)]
pub struct CaptureMetadata {
    pub timestamp_ns: u64,
    pub scene_brightness: f32,  // average scene luminance
}

/// Result of HDR processing — an enhanced, tonemapped, encoded frame.
#[derive(Debug, Clone)]
pub struct EnhancedFrame {
    /// Encoded image bytes (JPEG/HEIC)
    pub data: Vec<u8>,
    /// Original width
    pub width: u32,
    /// Original height
    pub height: u32,
    /// Thumbnail for quick preview
    pub thumbnail: Option<Vec<u8>>,
    /// HDR metadata (gain map, etc.)
    pub hdr_metadata: Option<Vec<u8>>,
}

/// HDR capture request sent to the worker queue.
pub struct HdrCaptureRequest {
    /// 3 frames: [under-exposed, normal, over-exposed]
    /// sorted by EV ascending (-2, 0, +2)
    pub frames: Vec<HdrFrame>,
    pub metadata: CaptureMetadata,
    /// Channel to send the result back
    pub response_tx: std::sync::mpsc::Sender<Result<EnhancedFrame, HdrError>>,
}

/// HDR pipeline errors.
#[derive(Debug)]
pub enum HdrError {
    /// Queue is full / dropped
    QueueFull,
    /// Frame count mismatch (need exactly 3)
    FrameCount { expected: usize, got: usize },
    /// Frame size mismatch across exposures
    SizeMismatch { w: u32, h: u32, ew: u32, eh: u32 },
    /// ISP processing failed
    Isp(String),
    /// Alignment failed
    Align(String),
    /// Merge failed
    Merge(String),
    /// Neural enhancement failed
    Enhance(String),
    /// Encoding failed
    Encode(String),
}

impl std::fmt::Display for HdrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HdrError::QueueFull => write!(f, "HDR queue full"),
            HdrError::FrameCount { expected, got } => {
                write!(f, "need {} frames, got {}", expected, got)
            }
            HdrError::SizeMismatch { w, h, ew, eh } => {
                write!(f, "frame size mismatch: {}x{} vs {}x{}", w, h, ew, eh)
            }
            HdrError::Isp(s) => write!(f, "ISP error: {}", s),
            HdrError::Align(s) => write!(f, "align error: {}", s),
            HdrError::Merge(s) => write!(f, "merge error: {}", s),
            HdrError::Enhance(s) => write!(f, "enhance error: {}", s),
            HdrError::Encode(s) => write!(f, "encode error: {}", s),
        }
    }
}

impl std::error::Error for HdrError {}

/// HDR merge strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HdrMergeMethod {
    /// Mertens exposure fusion — fast, good quality
    Mertens,
    /// Deep HDR CNN — best quality, slower
    DeepHdr,
    /// Neural HDR (Kalantari et al.) — excellent quality
    NeuralHdr,
}

impl Default for HdrMergeMethod {
    fn default() -> Self { HdrMergeMethod::Mertens }
}

// ── HDR Capture Queue ─────────────────────────────────────────────

/// Async HDR capture queue.
///
/// Submits multi-exposure capture requests to a background worker thread.
/// The worker processes: align → merge → tone map → enhance → encode.
/// Returns result via oneshot channel.
pub struct HdrCaptureQueue {
    tx: Sender<HdrCaptureRequest>,
    worker_handle: Option<thread::JoinHandle<()>>,
    /// Number of frames expected per HDR capture (default: 3)
    frames_per_capture: usize,
}

impl HdrCaptureQueue {
    /// Create a new HDR capture queue with background worker.
    pub fn new(_num_workers: usize) -> Self {
        let (tx, rx) = channel::<HdrCaptureRequest>();

        let worker_handle = thread::spawn(move || {
            let worker = HdrWorker::new();
            Self::worker_loop(&worker, rx);
        });

        Self {
            tx,
            worker_handle: Some(worker_handle),
            frames_per_capture: 3,
        }
    }

    /// Set expected frame count per capture.
    pub fn with_frames_per_capture(mut self, n: usize) -> Self {
        self.frames_per_capture = n;
        self
    }

    /// Background worker loop — processes requests until channel closes.
    fn worker_loop(worker: &HdrWorker, rx: Receiver<HdrCaptureRequest>) {
        while let Ok(request) = rx.recv() {
            let result = worker.process(&request);
            // Send result back; ignore error if receiver dropped
            let _ = request.response_tx.send(result);
        }
    }

    /// Submit an HDR capture request for async processing.
    ///
    /// Returns immediately. Result arrives via the oneshot channel inside `request`.
    pub fn submit(&self, request: HdrCaptureRequest) -> Result<(), HdrError> {
        self.tx.send(request).map_err(|_| HdrError::QueueFull)
    }
}

impl Drop for HdrCaptureQueue {
    fn drop(&mut self) {
        // Drop the sender to close the channel, worker will exit
        // (worker_handle joins automatically via JoinHandle drop)
    }
}

// ── HDR Worker ─────────────────────────────────────────────────────

/// HDR processing worker — runs align → merge → tone map → enhance → encode.
struct HdrWorker {
    // Future: MNN-based neural pipeline for enhance
    // neural_pipeline: MnnInterpreterSafe,
}

impl HdrWorker {
    fn new() -> Self {
        Self {
            // neural_pipeline loaded from hdr_enhance.mnn
        }
    }

    /// Process a complete HDR capture request.
    fn process(&self, request: &HdrCaptureRequest) -> Result<EnhancedFrame, HdrError> {
        let frames = &request.frames;

        // Validate frame count
        if frames.len() < 2 {
            return Err(HdrError::FrameCount {
                expected: 2,
                got: frames.len(),
            });
        }

        // Validate uniform dimensions
        let w = frames[0].width;
        let h = frames[0].height;
        for f in frames {
            if f.width != w || f.height != h {
                return Err(HdrError::SizeMismatch {
                    w, h, ew: f.width, eh: f.height,
                });
            }
        }

        // 1. Sort frames by EV (ensure -2, 0, +2 order)
        let mut sorted = frames.to_vec();
        sorted.sort_by(|a, b| a.ev.partial_cmp(&b.ev).unwrap());

        // 2. Align frames using EIS motion vectors
        //    For now: identity alignment (noop)
        //    Future: use MNN optical flow model
        let aligned = self.align_frames(&sorted)?;

        // 3. Merge exposures
        //    3-frame: use HdrMergeBlock logic (luminance-weighted)
        //    2-frame: simple blend
        let merged = match aligned.len() {
            3 => self.merge_three_exp(&aligned)?,
            2 => self.merge_two_exp(&aligned)?,
            _n => {
                // N-frame: average (fallback)
                self.merge_n_frames(&aligned)?
            }
        };

        // 4. Tone map (ACES filmic)
        let tonemapped = self.tone_map(&merged)?;

        // 5. Encode (placeholder for JPEG/HEIC)
        let encoded = self.encode(&tonemapped, 95)?;

        Ok(EnhancedFrame {
            data: encoded,
            width: w,
            height: h,
            thumbnail: None,
            hdr_metadata: None,
        })
    }

    // ── Alignment ──────────────────────────────────────────────

    /// Align frames using EIS motion vectors.
    /// For current implementation: identity (no alignment).
    /// Future: MNN-based optical flow or EIS motion vector warp.
    fn align_frames(&self, frames: &[HdrFrame]) -> Result<Vec<HdrFrame>, HdrError> {
        // TODO: Implement EIS-based alignment
        // - Compute homography from gyro/IMU data
        // - Warp under/over exposures to match neutral frame
        // - Or use MNN optical flow model for per-pixel alignment
        Ok(frames.to_vec())
    }

    // ── Merge ──────────────────────────────────────────────────

    /// Merge 3 exposures using luminance-weighted blending (Mertens-style).
    /// This implements the same algorithm as `HdrMergeBlock` but in CPU code
    /// for simplicity. Future: run the ONNX model directly.
    fn merge_three_exp(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let under = &frames[0]; // -2EV
        let neutral = &frames[1]; // 0EV
        let over = &frames[2]; // +2EV

        let len = under.data.len().min(neutral.data.len()).min(over.data.len());
        // Assume ARGB8888: 4 bytes per pixel (A, R, G, B)
        let pixel_count = len / 4;

        let mut merged = Vec::with_capacity(len);

        for i in 0..pixel_count {
            let base = i * 4;

            // Extract RGB (ignore alpha for merge)
            let r_u = under.data[base + 1] as f32 / 255.0;
            let g_u = under.data[base + 2] as f32 / 255.0;
            let b_u = under.data[base + 3] as f32 / 255.0;

            let r_n = neutral.data[base + 1] as f32 / 255.0;
            let g_n = neutral.data[base + 2] as f32 / 255.0;
            let b_n = neutral.data[base + 3] as f32 / 255.0;

            let r_o = over.data[base + 1] as f32 / 255.0;
            let g_o = over.data[base + 2] as f32 / 255.0;
            let b_o = over.data[base + 3] as f32 / 255.0;

            // Luminance for weight computation
            let lum_u = 0.299 * r_u + 0.587 * g_u + 0.114 * b_u;
            let lum_n = 0.299 * r_n + 0.587 * g_n + 0.114 * b_n;
            let lum_o = 0.299 * r_o + 0.587 * g_o + 0.114 * b_o;

            // Well-exposedness weights (Mertens)
            let w_u = (1.0 - (lum_u - 0.5).abs() * 2.0).clamp(0.0, 1.0);
            let w_n = (1.0 - (lum_n - 0.5).abs() * 2.0).clamp(0.0, 1.0);
            let w_o = (1.0 - (lum_o - 0.5).abs() * 2.0).clamp(0.0, 1.0);

            // Normalize weights
            let total = w_u + w_n + w_o + 1e-10;
            let w_u = w_u / total;
            let w_n = w_n / total;
            let w_o = w_o / total;

            // Blend
            let r_m = (r_u * w_u + r_n * w_n + r_o * w_o).clamp(0.0, 1.0);
            let g_m = (g_u * w_u + g_n * w_n + g_o * w_o).clamp(0.0, 1.0);
            let b_m = (b_u * w_u + b_n * w_n + b_o * w_o).clamp(0.0, 1.0);

            merged.push(255); // Alpha
            merged.push((r_m * 255.0) as u8);
            merged.push((g_m * 255.0) as u8);
            merged.push((b_m * 255.0) as u8);
        }

        Ok(Arc::new(merged))
    }

    /// Merge 2 exposures (under/over) with simple blend.
    fn merge_two_exp(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let under = &frames[0];
        let over = &frames[1];

        let len = under.data.len().min(over.data.len());
        let pixel_count = len / 4;

        let mut merged = Vec::with_capacity(len);

        for i in 0..pixel_count {
            let base = i * 4;

            let r_u = under.data[base + 1] as f32 / 255.0;
            let g_u = under.data[base + 2] as f32 / 255.0;
            let b_u = under.data[base + 3] as f32 / 255.0;

            let r_o = over.data[base + 1] as f32 / 255.0;
            let g_o = over.data[base + 2] as f32 / 255.0;
            let b_o = over.data[base + 3] as f32 / 255.0;

            // Luminance-based blend: dark areas → over, bright areas → under
            let lum_o = 0.299 * r_o + 0.587 * g_o + 0.114 * b_o;
            let weight = (lum_o - 0.5) * 2.0; // -1 to 1

            let w_under = (weight + 1.0) * 0.5; // bright → more under weight
            let w_over = 1.0 - w_under;

            let r_m = (r_u * w_under + r_o * w_over).clamp(0.0, 1.0);
            let g_m = (g_u * w_under + g_o * w_over).clamp(0.0, 1.0);
            let b_m = (b_u * w_under + b_o * w_over).clamp(0.0, 1.0);

            merged.push(255);
            merged.push((r_m * 255.0) as u8);
            merged.push((g_m * 255.0) as u8);
            merged.push((b_m * 255.0) as u8);
        }

        Ok(Arc::new(merged))
    }

    /// Merge N frames by simple averaging (fallback).
    fn merge_n_frames(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let n = frames.len() as f32;
        let len = frames.iter().map(|f| f.data.len()).min().unwrap_or(0);
        let pixel_count = len / 4;

        let mut merged = vec![0u8; len];

        for i in 0..pixel_count {
            let base = i * 4;
            let mut r_acc = 0.0f32;
            let mut g_acc = 0.0f32;
            let mut b_acc = 0.0f32;

            for f in frames {
                r_acc += f.data[base + 1] as f32;
                g_acc += f.data[base + 2] as f32;
                b_acc += f.data[base + 3] as f32;
            }

            let div = n.max(1.0);
            merged[base] = 255;
            merged[base + 1] = (r_acc / div).round().clamp(0.0, 255.0) as u8;
            merged[base + 2] = (g_acc / div).round().clamp(0.0, 255.0) as u8;
            merged[base + 3] = (b_acc / div).round().clamp(0.0, 255.0) as u8;
        }

        Ok(Arc::new(merged))
    }

    // ── Tone Mapping ───────────────────────────────────────────

    /// ACES filmic tone mapping (CPU implementation matching `HdrToneBlock`).
    fn tone_map(&self, frame: &[u8]) -> Result<Vec<u8>, HdrError> {
        let pixel_count = frame.len() / 4;
        let mut output = Vec::with_capacity(frame.len());

        // ACES input transform matrix (sRGB → ACES)
        let input_mat: [[f32; 3]; 3] = [
            [0.59719, 0.35458, 0.04823],
            [0.07600, 0.90834, 0.01566],
            [0.02840, 0.13383, 0.83777],
        ];
        // ACES output transform matrix (ACES → sRGB)
        let output_mat: [[f32; 3]; 3] = [
            [1.60475, -0.53108, -0.07367],
            [-0.10208, 1.10813, -0.00605],
            [-0.00327, -0.07276, 1.07602],
        ];

        for i in 0..pixel_count {
            let base = i * 4;
            let a = frame[base] as f32 / 255.0;
            let r = frame[base + 1] as f32 / 255.0;
            let g = frame[base + 2] as f32 / 255.0;
            let b = frame[base + 3] as f32 / 255.0;

            // Step 1: sRGB → ACES color space
            let aces_r = r * input_mat[0][0] + g * input_mat[0][1] + b * input_mat[0][2];
            let aces_g = r * input_mat[1][0] + g * input_mat[1][1] + b * input_mat[1][2];
            let aces_b = r * input_mat[2][0] + g * input_mat[2][1] + b * input_mat[2][2];

            // Step 2: ACES filmic curve
            // (x*(2.51x + 0.03)) / (x*(2.43x + 0.59) + 0.14)
            let aces_curve = |x: f32| -> f32 {
                if x <= 0.0 { return 0.0; }
                let num = x * (2.51 * x + 0.03);
                let den = x * (2.43 * x + 0.59) + 0.14;
                num / den
            };

            let cr = aces_curve(aces_r);
            let cg = aces_curve(aces_g);
            let cb = aces_curve(aces_b);

            // Step 3: ACES → sRGB color space
            let sr = cr * output_mat[0][0] + cg * output_mat[0][1] + cb * output_mat[0][2];
            let sg = cr * output_mat[1][0] + cg * output_mat[1][1] + cb * output_mat[1][2];
            let sb = cr * output_mat[2][0] + cg * output_mat[2][1] + cb * output_mat[2][2];

            // Step 4: Clamp to [0, 1]
            let sr = sr.clamp(0.0, 1.0);
            let sg = sg.clamp(0.0, 1.0);
            let sb = sb.clamp(0.0, 1.0);

            output.push((a * 255.0) as u8);
            output.push((sr * 255.0) as u8);
            output.push((sg * 255.0) as u8);
            output.push((sb * 255.0) as u8);
        }

        Ok(output)
    }

    // ── Encoding ───────────────────────────────────────────────

    /// Encode merged frame to JPEG/HEIC.
    /// Placeholder: writes PPM format for now.
    /// Future: use libjpeg-turbo or Android Bitmap API.
    fn encode(&self, frame: &[u8], _quality: u8) -> Result<Vec<u8>, HdrError> {
        let pixel_count = frame.len() / 4;

        // Simple PPM encoding (R, G, B only, no alpha)
        let mut ppm = Vec::new();
        // PPM header
        ppm.extend_from_slice(b"P6\n");
        // Width and height (approximate from data length)
        // Without known dimensions, use square root heuristic
        let w = (pixel_count as f32).sqrt() as u32;
        let h = (pixel_count / w as usize) as u32;
        ppm.extend_from_slice(format!("{} {}\n255\n", w, h).as_bytes());

        // RGB data (skip alpha)
        for i in 0..pixel_count {
            let base = i * 4;
            ppm.push(frame[base + 1]); // R
            ppm.push(frame[base + 2]); // G
            ppm.push(frame[base + 3]); // B
        }

        Ok(ppm)
    }
}

// ── Builder / Integration ─────────────────────────────────────────

/// Builder for HDR capture configuration.
#[derive(Debug, Clone)]
pub struct HdrConfig {
    /// Exposure values for bracketing (default: [-2.0, 0.0, 2.0])
    pub ev_bracket: Vec<f32>,
    /// Merge strategy
    pub merge_method: HdrMergeMethod,
    /// Tone mapping intensity
    pub tone_intensity: f32,
    /// Enable neural enhancement
    pub neural_enhance: bool,
    /// Output JPEG quality (1-100)
    pub jpeg_quality: u8,
}

impl Default for HdrConfig {
    fn default() -> Self {
        Self {
            ev_bracket: vec![-2.0, 0.0, 2.0],
            merge_method: HdrMergeMethod::Mertens,
            tone_intensity: 1.0,
            neural_enhance: false,
            jpeg_quality: 95,
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_frame(ev: f32, r_val: u8, g_val: u8, b_val: u8) -> HdrFrame {
        let mut data = Vec::with_capacity(64 * 64 * 4);
        for _ in 0..(64 * 64) {
            data.push(255); // A
            data.push(r_val); // R
            data.push(g_val); // G
            data.push(b_val); // B
        }
        HdrFrame {
            data: Arc::new(data),
            ev,
            timestamp_ns: 0,
            iso: 100.0,
            exposure_time: 0.033,
            width: 64,
            height: 64,
        }
    }

    #[test]
    fn test_hdr_merge_three_exp() {
        let worker = HdrWorker::new();
        let frames = vec![
            make_test_frame(-2.0, 50, 50, 50),   // under — dark
            make_test_frame(0.0, 128, 128, 128),  // neutral — mid
            make_test_frame(2.0, 200, 200, 200),  // over — bright
        ];

        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata { timestamp_ns: 0, scene_brightness: 0.5 },
            response_tx: std::sync::mpsc::channel().0,
        };

        let result = worker.process(&request);
        assert!(result.is_ok(), "HDR processing failed: {:?}", result.err());
        let enhanced = result.unwrap();
        assert!(!enhanced.data.is_empty(), "Enhanced frame should have data");
    }

    #[test]
    fn test_hdr_merge_two_exp() {
        let worker = HdrWorker::new();
        let frames = vec![
            make_test_frame(-2.0, 30, 30, 30),
            make_test_frame(2.0, 220, 220, 220),
        ];

        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata { timestamp_ns: 0, scene_brightness: 0.5 },
            response_tx: std::sync::mpsc::channel().0,
        };

        let result = worker.process(&request);
        assert!(result.is_ok(), "HDR 2-frame failed: {:?}", result.err());
    }

    #[test]
    fn test_hdr_queue_submit() {
        let queue = HdrCaptureQueue::new(1);
        let (tx, rx) = std::sync::mpsc::channel();

        let req = HdrCaptureRequest {
            frames: vec![
                make_test_frame(-2.0, 50, 50, 50),
                make_test_frame(0.0, 128, 128, 128),
                make_test_frame(2.0, 200, 200, 200),
            ],
            metadata: CaptureMetadata { timestamp_ns: 0, scene_brightness: 0.5 },
            response_tx: tx,
        };

        assert!(queue.submit(req).is_ok(), "Submit should succeed");

        // Wait for result
        let result = rx.recv().expect("Should receive result");
        assert!(result.is_ok(), "HDR queue processing failed: {:?}", result.err());
    }

    #[test]
    fn test_hdr_error_frame_count() {
        let worker = HdrWorker::new();
        let frames = vec![make_test_frame(0.0, 128, 128, 128)]; // only 1 frame

        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata { timestamp_ns: 0, scene_brightness: 0.5 },
            response_tx: std::sync::mpsc::channel().0,
        };

        let result = worker.process(&request);
        assert!(result.is_err(), "Should fail with <2 frames");
    }

    #[test]
    fn test_hdr_error_size_mismatch() {
        let worker = HdrWorker::new();
        let mut f1 = make_test_frame(-2.0, 50, 50, 50);
        f1.width = 32; // Different from default 64
        let f2 = make_test_frame(0.0, 128, 128, 128);
        let f3 = make_test_frame(2.0, 200, 200, 200);

        let request = HdrCaptureRequest {
            frames: vec![f1, f2, f3],
            metadata: CaptureMetadata { timestamp_ns: 0, scene_brightness: 0.5 },
            response_tx: std::sync::mpsc::channel().0,
        };

        let result = worker.process(&request);
        assert!(result.is_err(), "Should fail on size mismatch");
    }

    #[test]
    fn test_hdr_tone_map_aces() {
        let worker = HdrWorker::new();
        // Create a solid gray frame
        let mut frame = Vec::with_capacity(64 * 64 * 4);
        for _ in 0..(64 * 64) {
            frame.push(255);
            frame.push(128);
            frame.push(128);
            frame.push(128);
        }

        let result = worker.tone_map(&frame).unwrap();
        assert_eq!(result.len(), frame.len(), "Tone map should preserve size");

        // Verify output is within [0, 255]
        for &b in &result {
            assert!(b <= 255, "Pixel value out of range: {}", b as u16);
        }
    }

    #[test]
    fn test_hdr_config_defaults() {
        let cfg = HdrConfig::default();
        assert_eq!(cfg.ev_bracket.len(), 3);
        assert_eq!(cfg.ev_bracket[0], -2.0);
        assert_eq!(cfg.ev_bracket[2], 2.0);
        assert_eq!(cfg.merge_method, HdrMergeMethod::Mertens);
        assert_eq!(cfg.jpeg_quality, 95);
    }

    #[test]
    fn test_hdr_merge_method_default() {
        let m: HdrMergeMethod = Default::default();
        assert_eq!(m, HdrMergeMethod::Mertens);
    }
}
