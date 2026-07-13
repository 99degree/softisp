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

use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread;

// ── Types ──────────────────────────────────────────────────────────

/// Frame with exposure metadata after ISP processing.
/// Wraps an `IspFrame` with HDR-specific exposure information.
#[derive(Debug, Clone)]
pub struct HdrFrame {
    /// The underlying ISP-processed frame (pixel data, params, aux, timestamps)
    pub frame: crate::pipeline::types::IspFrame,
    /// Exposure value relative to metered: -2.0, 0.0, +2.0
    pub ev: f32,
    /// ISO value for this frame
    pub iso: f32,
    /// Exposure time in seconds
    pub exposure_time: f32,
}

/// Metadata for an HDR capture session.
#[derive(Debug, Clone)]
pub struct CaptureMetadata {
    pub timestamp_ns: u64,
    pub scene_brightness: f32, // average scene luminance
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HdrMergeMethod {
    /// Mertens exposure fusion — fast, good quality
    #[default]
    Mertens,
    /// Deep HDR CNN — best quality, slower
    DeepHdr,
    /// Neural HDR (Kalantari et al.) — excellent quality
    NeuralHdr,
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

    /// Convenience: submit frames directly, returns a receiver for the result.
    pub fn submit_frames(
        &self,
        frames: Vec<HdrFrame>,
    ) -> Result<std::sync::mpsc::Receiver<Result<EnhancedFrame, HdrError>>, HdrError> {
        let (response_tx, response_rx) = std::sync::mpsc::channel();
        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
            response_tx,
        };
        self.submit(request)?;
        Ok(response_rx)
    }

    /// Get the expected number of frames per capture.
    pub fn frames_per_capture(&self) -> usize {
        self.frames_per_capture
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

    /// Read RGB pixel at index `i` from an HDR frame, normalised to [0,1].
    /// Handles both FloatRgb (planar NCHW f32 via `float_data`)
    /// and ARGB8888 (interleaved bytes via `data`).
    fn pixel_rgb(frame: &crate::pipeline::types::IspFrame, i: usize) -> (f32, f32, f32) {
        if let Some(ref fd) = frame.float_data {
            // NCHW planar: [R...R, G...G, B...B], each plane = H×W
            let plane = (frame.width * frame.height) as usize;
            let r = fd.get(i).copied().unwrap_or(0.0);
            let g = fd.get(plane + i).copied().unwrap_or(0.0);
            let b = fd.get(2 * plane + i).copied().unwrap_or(0.0);
            (r, g, b)
        } else {
            // Assume ARGB8888: A at base, R=base+1, G=base+2, B=base+3
            let base = i * 4;
            if base + 3 < frame.data.len() {
                let r = frame.data[base + 1] as f32 / 255.0;
                let g = frame.data[base + 2] as f32 / 255.0;
                let b = frame.data[base + 3] as f32 / 255.0;
                (r, g, b)
            } else {
                (0.0, 0.0, 0.0)
            }
        }
    }

    /// Write ARGB pixel at index `i` in the output buffer from RGB [0,1].
    fn write_argb(out: &mut [u8], i: usize, a: u8, r: f32, g: f32, b: f32) {
        let base = i * 4;
        if base + 3 < out.len() {
            out[base] = a;
            out[base + 1] = (r.clamp(0.0, 1.0) * 255.0) as u8;
            out[base + 2] = (g.clamp(0.0, 1.0) * 255.0) as u8;
            out[base + 3] = (b.clamp(0.0, 1.0) * 255.0) as u8;
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
        let w = frames[0].frame.width;
        let h = frames[0].frame.height;
        for f in frames {
            if f.frame.width != w || f.frame.height != h {
                return Err(HdrError::SizeMismatch {
                    w,
                    h,
                    ew: f.frame.width,
                    eh: f.frame.height,
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
        let encoded = self.encode(&tonemapped, 95, w, h)?;

        Ok(EnhancedFrame {
            data: encoded,
            width: w,
            height: h,
            thumbnail: None,
            hdr_metadata: None,
        })
    }

    // ── Alignment ──────────────────────────────────────────────

    /// Align frames using EIS-style global motion estimation.
    ///
    /// When gyro/IMU data is unavailable we fall back to image-based
    /// block-matching (the same translation model EIS uses to stabilise a
    /// video stream): every non-reference frame is translated to best match
    /// the reference (neutral-EV) frame. The reference frame is left
    /// untouched. This removes handshake/registration error between the
    /// under/over exposures before they are merged.
    fn align_frames(&self, frames: &[HdrFrame]) -> Result<Vec<HdrFrame>, HdrError> {
        if frames.len() < 2 {
            return Ok(frames.to_vec());
        }

        // Reference = frame whose EV is closest to neutral (0).
        let ref_idx = frames
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.ev.abs().partial_cmp(&b.1.ev.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);
        let reference = &frames[ref_idx].frame;

        let mut out = Vec::with_capacity(frames.len());
        for (i, f) in frames.iter().enumerate() {
            if i == ref_idx {
                out.push(f.clone());
                continue;
            }
            let (dx, dy) = Self::estimate_translation(&f.frame, reference, 16);
            out.push(Self::shift_frame(f, dx, dy));
        }
        Ok(out)
    }

/// Build a grayscale luminance buffer (length `w*h`) from an ISP frame.
fn frame_luma(frame: &crate::pipeline::types::IspFrame) -> Vec<f32> {
    let w = frame.width as usize;
    let h = frame.height as usize;
    let mut lum = vec![0.0f32; w * h];
    if let Some(ref fd) = frame.float_data {
        let plane = w * h;
        for i in 0..plane {
            let r = fd.get(i).copied().unwrap_or(0.0);
            let g = fd.get(plane + i).copied().unwrap_or(0.0);
            let b = fd.get(2 * plane + i).copied().unwrap_or(0.0);
            lum[i] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
        }
    } else {
        for i in 0..(w * h) {
            let base = i * 4;
            if base + 3 < frame.data.len() {
                let r = frame.data[base + 1] as f32;
                let g = frame.data[base + 2] as f32;
                let b = frame.data[base + 3] as f32;
                lum[i] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
            }
        }
    }
    lum
}

/// Sum of absolute differences between the block at `(bx,by)` in `lum_from`
/// and the same block shifted by `(dx,dy)` in `lum_to`.
fn block_sad(
    lum_from: &[f32],
    lum_to: &[f32],
    w: i32,
    h: i32,
    bx: i32,
    by: i32,
    bw: i32,
    bh: i32,
    dx: i32,
    dy: i32,
) -> f32 {
    let mut sad = 0.0f32;
    for yy in 0..bh {
        for xx in 0..bw {
            let fx = bx + xx;
            let fy = by + yy;
            let tx = bx + xx + dx;
            let ty = by + yy + dy;
            let fv = if fx >= 0 && fx < w && fy >= 0 && fy < h {
                lum_from[(fy * w + fx) as usize]
            } else {
                0.0
            };
            let tv = if tx >= 0 && tx < w && ty >= 0 && ty < h {
                lum_to[(ty * w + tx) as usize]
            } else {
                0.0
            };
            sad += (fv - tv).abs();
        }
    }
    sad
}

/// Estimate the integer `(dx, dy)` that best translates `from` onto `to` using
/// a 4×4 grid of block matches with median voting (robust to moving content).
fn estimate_translation(
    from: &crate::pipeline::types::IspFrame,
    to: &crate::pipeline::types::IspFrame,
    max_shift: i32,
) -> (i32, i32) {
    let w = from.width as i32;
    let h = from.height as i32;
    if w <= 0 || h <= 0 {
        return (0, 0);
    }
    let lum_from = Self::frame_luma(from);
    let lum_to = Self::frame_luma(to);

    let blocks = 4usize;
    let bw = (w as usize / blocks).max(1) as i32;
    let bh = (h as usize / blocks).max(1) as i32;
    if bw < 4 || bh < 4 {
        return (0, 0);
    }

    let mut dxs = Vec::new();
    let mut dys = Vec::new();
    for by in 0..(blocks as i32) {
        for bx in 0..(blocks as i32) {
            let x0 = bx * bw;
            let y0 = by * bh;
            let mut best = (0i32, 0i32);
            let mut best_sad = f32::INFINITY;
            for dy in -max_shift..=max_shift {
                for dx in -max_shift..=max_shift {
                    let s = Self::block_sad(&lum_from, &lum_to, w, h, x0, y0, bw, bh, dx, dy);
                    if s < best_sad {
                        best_sad = s;
                        best = (dx, dy);
                    }
                }
            }
            dxs.push(best.0);
            dys.push(best.1);
        }
    }
    (Self::median(&dxs), Self::median(&dys))
}

/// Median of a small integer slice (returns 0 for empty input).
fn median(v: &[i32]) -> i32 {
    if v.is_empty() {
        return 0;
    }
    let mut s = v.to_vec();
    s.sort_unstable();
    s[s.len() / 2]
}

/// Return a copy of `f` shifted by `(dx, dy)` with edge replication.
fn shift_frame(f: &HdrFrame, dx: i32, dy: i32) -> HdrFrame {
    if dx == 0 && dy == 0 {
        return f.clone();
    }
    let w = f.frame.width as i32;
    let h = f.frame.height as i32;
    let mut out = f.clone();

    if out.frame.float_data.is_none() {
        // ARGB interleaved path.
        let mut data = vec![0u8; out.frame.data.len()];
        let pw = 4;
        for y in 0..h {
            for x in 0..w {
                let sx = (x - dx).clamp(0, w - 1);
                let sy = (y - dy).clamp(0, h - 1);
                let di = ((y * w + x) * pw) as usize;
                let si = ((sy * w + sx) * pw) as usize;
                data[di..di + 4].copy_from_slice(&out.frame.data[si..si + 4]);
            }
        }
        out.frame.data = data;
    } else if let Some(ref fd) = f.frame.float_data {
        // Planar NCHW f32: 3 planes of `w*h`.
        let plane = (w * h) as usize;
        let mut new_fd = vec![0.0f32; fd.len()];
        for c in 0..3 {
            for y in 0..h {
                for x in 0..w {
                    let sx = (x - dx).clamp(0, w - 1);
                    let sy = (y - dy).clamp(0, h - 1);
                    let di = c * plane + (y * w + x) as usize;
                    let si = c * plane + (sy * w + sx) as usize;
                    new_fd[di] = fd[si];
                }
            }
        }
        out.frame.float_data = Some(new_fd);
    }
    out
}

    // ── Merge ──────────────────────────────────────────────────

    /// Merge 3 exposures using luminance-weighted blending (Mertens-style).
    fn merge_three_exp(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let under = &frames[0];
        let neutral = &frames[1];
        let over = &frames[2];

        let pixel_count = (under.frame.width * under.frame.height) as usize;
        let mut merged = vec![0u8; pixel_count * 4];

        for i in 0..pixel_count {
            let (r_u, g_u, b_u) = Self::pixel_rgb(&under.frame, i);
            let (r_n, g_n, b_n) = Self::pixel_rgb(&neutral.frame, i);
            let (r_o, g_o, b_o) = Self::pixel_rgb(&over.frame, i);

            // Luminance for weight computation
            let lum_u = 0.299 * r_u + 0.587 * g_u + 0.114 * b_u;
            let lum_n = 0.299 * r_n + 0.587 * g_n + 0.114 * b_n;
            let lum_o = 0.299 * r_o + 0.587 * g_o + 0.114 * b_o;

            // Well-exposedness weights (Mertens)
            let w_u = (1.0 - (lum_u - 0.5).abs() * 2.0).clamp(0.0, 1.0);
            let w_n = (1.0 - (lum_n - 0.5).abs() * 2.0).clamp(0.0, 1.0);
            let w_o = (1.0 - (lum_o - 0.5).abs() * 2.0).clamp(0.0, 1.0);

            let total = w_u + w_n + w_o + 1e-10;
            let w_u = w_u / total;
            let w_n = w_n / total;
            let w_o = w_o / total;

            let r_m = (r_u * w_u + r_n * w_n + r_o * w_o).clamp(0.0, 1.0);
            let g_m = (g_u * w_u + g_n * w_n + g_o * w_o).clamp(0.0, 1.0);
            let b_m = (b_u * w_u + b_n * w_n + b_o * w_o).clamp(0.0, 1.0);

            Self::write_argb(&mut merged, i, 255, r_m, g_m, b_m);
        }

        Ok(Arc::new(merged))
    }

    /// Merge 2 exposures (under/over) with simple blend.
    fn merge_two_exp(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let under = &frames[0];
        let over = &frames[1];

        let pixel_count = (under.frame.width * under.frame.height) as usize;
        let mut merged = vec![0u8; pixel_count * 4];

        for i in 0..pixel_count {
            let (r_u, g_u, b_u) = Self::pixel_rgb(&under.frame, i);
            let (r_o, g_o, b_o) = Self::pixel_rgb(&over.frame, i);

            // Luminance-based blend: dark areas → over, bright areas → under
            let lum_o = 0.299 * r_o + 0.587 * g_o + 0.114 * b_o;
            let weight = (lum_o - 0.5) * 2.0;
            let w_under = (weight + 1.0) * 0.5;
            let w_over = 1.0 - w_under;

            let r_m = (r_u * w_under + r_o * w_over).clamp(0.0, 1.0);
            let g_m = (g_u * w_under + g_o * w_over).clamp(0.0, 1.0);
            let b_m = (b_u * w_under + b_o * w_over).clamp(0.0, 1.0);

            Self::write_argb(&mut merged, i, 255, r_m, g_m, b_m);
        }

        Ok(Arc::new(merged))
    }

    /// Merge N frames by simple averaging (fallback).
    fn merge_n_frames(&self, frames: &[HdrFrame]) -> Result<Arc<Vec<u8>>, HdrError> {
        let n = frames.len() as f32;
        let pixel_count = (frames[0].frame.width * frames[0].frame.height) as usize;
        let mut merged = vec![0u8; pixel_count * 4];

        for i in 0..pixel_count {
            let mut r_acc = 0.0f32;
            let mut g_acc = 0.0f32;
            let mut b_acc = 0.0f32;

            for f in frames {
                let (r, g, b) = Self::pixel_rgb(&f.frame, i);
                r_acc += r;
                g_acc += g;
                b_acc += b;
            }

            let div = n.max(1.0);
            Self::write_argb(&mut merged, i, 255, r_acc / div, g_acc / div, b_acc / div);
        }

        Ok(Arc::new(merged))
    }

    // ── Tone Mapping ───────────────────────────────────────────

    /// ACES filmic tone mapping (CPU implementation matching `HdrToneBlock`).
    fn tone_map(&self, frame: &[u8]) -> Result<Vec<u8>, HdrError> {
        let pixel_count = frame.len() / 4;

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

        let mut output = vec![0u8; frame.len()];

        for i in 0..pixel_count {
            let base = i * 4;
            let a = frame[base] as f32 / 255.0;
            let r = frame[base + 1] as f32 / 255.0;
            let g = frame[base + 2] as f32 / 255.0;
            let b = frame[base + 3] as f32 / 255.0;

            // Step 1: sRGB → ACES
            let aces_r = r * input_mat[0][0] + g * input_mat[0][1] + b * input_mat[0][2];
            let aces_g = r * input_mat[1][0] + g * input_mat[1][1] + b * input_mat[1][2];
            let aces_b = r * input_mat[2][0] + g * input_mat[2][1] + b * input_mat[2][2];

            // Step 2: ACES filmic curve
            let aces_curve = |x: f32| -> f32 {
                if x <= 0.0 {
                    return 0.0;
                }
                let num = x * (2.51 * x + 0.03);
                let den = x * (2.43 * x + 0.59) + 0.14;
                num / den
            };

            let cr = aces_curve(aces_r);
            let cg = aces_curve(aces_g);
            let cb = aces_curve(aces_b);

            // Step 3: ACES → sRGB
            let sr = cr * output_mat[0][0] + cg * output_mat[0][1] + cb * output_mat[0][2];
            let sg = cr * output_mat[1][0] + cg * output_mat[1][1] + cb * output_mat[1][2];
            let sb = cr * output_mat[2][0] + cg * output_mat[2][1] + cb * output_mat[2][2];

            Self::write_argb(&mut output, i, (a * 255.0) as u8, sr, sg, sb);
        }

        Ok(output)
    }

    // ── Encoding ───────────────────────────────────────────────

    /// Encode merged frame to JPEG/HEIC.
    /// Placeholder: writes PPM format for now.
    /// Future: use libjpeg-turbo or Android Bitmap API.
    fn encode(
        &self,
        frame: &[u8],
        _quality: u8,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, HdrError> {
        let pixel_count = frame.len() / 4;
        if pixel_count != (width * height) as usize {
            // Try to recover: use pixel_count dimensions
            let w = (pixel_count as f32).sqrt() as u32;
            let h = (pixel_count / w.max(1) as usize) as u32;
            return self.encode(frame, _quality, w, h);
        }

        // Simple PPM encoding (R, G, B only, no alpha)
        let mut ppm = Vec::new();
        // PPM header
        ppm.extend_from_slice(b"P6\n");
        ppm.extend_from_slice(format!("{} {}\n255\n", width, height).as_bytes());

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
            frame: crate::pipeline::types::IspFrame {
                data,
                width: 64,
                height: 64,
                format: cam_types::FrameFormat::NchwFloat,
                float_data: None,
                aux: None,
                params: crate::isp_params::IspParams::default(),
                timestamp_ns: 0,
                prep_duration_ns: 0,
                inference_duration_ns: 0,
                total_duration_ns: 0,
            },
            ev,
            iso: 100.0,
            exposure_time: 0.033,
        }
    }

    #[test]
    fn test_hdr_merge_three_exp() {
        let worker = HdrWorker::new();
        let frames = vec![
            make_test_frame(-2.0, 50, 50, 50),   // under — dark
            make_test_frame(0.0, 128, 128, 128), // neutral — mid
            make_test_frame(2.0, 200, 200, 200), // over — bright
        ];

        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
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
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
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
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
            response_tx: tx,
        };

        assert!(queue.submit(req).is_ok(), "Submit should succeed");

        // Wait for result
        let result = rx.recv().expect("Should receive result");
        assert!(
            result.is_ok(),
            "HDR queue processing failed: {:?}",
            result.err()
        );
    }

    #[test]
    fn test_hdr_error_frame_count() {
        let worker = HdrWorker::new();
        let frames = vec![make_test_frame(0.0, 128, 128, 128)]; // only 1 frame

        let request = HdrCaptureRequest {
            frames,
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
            response_tx: std::sync::mpsc::channel().0,
        };

        let result = worker.process(&request);
        assert!(result.is_err(), "Should fail with <2 frames");
    }

    #[test]
    fn test_hdr_error_size_mismatch() {
        let worker = HdrWorker::new();
        let mut f1 = make_test_frame(-2.0, 50, 50, 50);
        f1.frame.width = 32; // Different from default 64
        let f2 = make_test_frame(0.0, 128, 128, 128);
        let f3 = make_test_frame(2.0, 200, 200, 200);

        let request = HdrCaptureRequest {
            frames: vec![f1, f2, f3],
            metadata: CaptureMetadata {
                timestamp_ns: 0,
                scene_brightness: 0.5,
            },
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
            let _ = b; // u8 always in [0, 255]
        }
    }

    #[test]
    fn test_hdr_align_recovers_translation() {
        // Build a pseudo-random texture. Random texture has a sharply
        // peaked autocorrelation, so block-matching's SAD minimum is unique at
        // the true shift (a checkerboard or gradient would alias onto wrong
        // periodic / rank-1 shifts).
        let w = 64u32;
        let h = 64u32;
        let mut data = vec![0u8; (w * h * 4) as usize];
        for i in 0..(w * h) {
            let x = (i % w) as u64;
            let y = (i / w) as u64;
            let v = (((x * 73856093) ^ (y * 19349663)) % 256) as u8;
            let b = i as usize * 4;
            data[b] = 255;
            data[b + 1] = v;
            data[b + 2] = v;
            data[b + 3] = v;
        }
        let reference = HdrFrame {
            frame: crate::pipeline::types::IspFrame {
                data,
                width: w,
                height: h,
                format: cam_types::FrameFormat::NchwFloat,
                float_data: None,
                aux: None,
                params: crate::isp_params::IspParams::default(),
                timestamp_ns: 0,
                prep_duration_ns: 0,
                inference_duration_ns: 0,
                total_duration_ns: 0,
            },
            ev: 0.0,
            iso: 100.0,
            exposure_time: 0.033,
        };

        // Simulate handshake by shifting the reference by (+5, +3).
        let shifted = HdrWorker::shift_frame(&reference, 5, 3);

        // The estimator must recover the inverse translation (≈ -5, -3)
        // so align_frames() can undo the shift.
        let (dx, dy) = HdrWorker::estimate_translation(&shifted.frame, &reference.frame, 16);
        assert!((dx + 5).abs() <= 2, "expected dx≈-5, got {}", dx);
        assert!((dy + 3).abs() <= 2, "expected dy≈-3, got {}", dy);
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
