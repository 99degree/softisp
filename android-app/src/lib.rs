//! `com.softisp.camera` — a native camera preview driven by the Rust ISP.
//!
//! The app has no Java or C layer: `android-activity` provides the
//! `NativeActivity` lifecycle, the NDK camera2 API is called directly through
//! `ndk-sys`, and the resulting Bayer frames go through `cam-isp` before being
//! blitted to the `ANativeWindow`. Diagnostics land in
//! `/sdcard/download/softisp-app.log`; see [`crashlog`].

pub mod camera;
pub mod crashlog;
pub mod pipeline;
pub mod preview;
pub mod storage;

use std::ffi::c_void;
use std::path::Path;
use std::time::{Duration, Instant};

use android_activity::ndk::native_window::NativeWindow;
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};
use cam_isp::dual_engine::DualGpuEngine;
use cam_isp::engine::{select_engine_by_name, IspEngine};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;
use cam_isp::{release_native_window, set_native_window};
use camera::CameraSession;
use crashlog::write_line;
use pipeline::{FrameStager, PipelineConfig};
use preview::WindowTarget;
use storage::Access;

/// Frames processed between two stat lines.
///
/// The preview runs at ~1 fps on this hardware, so the old value of 30 meant a
/// status line every half minute — and a session that ended before frame 30
/// (the camera reports an error after ~20 s) never printed one at all. That is
/// why a black preview produced no content report to compare against. Five
/// gives a line every few seconds.
const STATS_INTERVAL_FRAMES: u64 = 5;

/// How long the event loop blocks waiting for a new frame or lifecycle event.
///
/// A finite timeout keeps the preview ticking even when the camera produces
/// nothing, so the surface is repainted instead of freezing.
const POLL_TIMEOUT: Duration = Duration::from_millis(10);

/// Wall-clock budget for one capture + ISP + present pass.
///
/// The capture queue holds three frames, so exceeding this self-throttles the
/// preview instead of letting latency grow without bound.
const FRAME_BUDGET: Duration = Duration::from_millis(500);

/// Route `log::` records to the app log *and* logcat.
///
/// The app has no Java layer, and the default `log` implementation discards
/// everything, which would make the engine/block-chain diagnostics invisible.
///
/// The file half matters more than it looks: `android_logger` alone sends
/// records to logcat, which needs the READ_LOGS grant to read and whose 256 KiB
/// ring buffers drain quickly. Everything that distinguishes "the ISP failed"
/// from "the ISP succeeded but nothing was drawn" lives in those records, and
/// losing them makes the two look identical. See [`crashlog::install_logger`].
fn init_logger() {
    crashlog::install_logger();
}

/// Longest RGB edge the tail is allowed to run at.
///
/// The cosmetic stages and the display tensor all pay per RGB pixel, while the
/// Bayer stages above them cost a quarter of that per pixel. Demosaic halves
/// both edges, so this cap applies to the *post-demosaic* frame: a 2592x1940
/// sensor is 1296x970 RGB and already fits, while a 4640x3472 one is
/// 2320x1736 and gets divided by two.
const TAIL_MAX_EDGE: u32 = 1920;

/// Whether the post-CCM downscale is enabled.
///
/// **Off by default.** It is a performance measure, not a correctness one, and
/// `cam-isp/tests/probe_post_ccm_downscale.rs` is the only thing that has ever
/// exercised it — never a real capture on device. The pipeline that runs today
/// is the one every other change in this file was verified against, so the
/// preview keeps that path until the downscale has proven itself on hardware.
///
/// Flipping it on is one line. It selects an **exact integer** reduction
/// factor, because MNN derives a Resize's output extent as
///
///     output.dim[i].extent = scale[i] * (float)input.dim[i].extent   // floors
///
/// A non-integer ratio still runs (the declared shape is corrected from the
/// block's own `out_dims`), but an integer factor keeps the declared and the
/// computed geometry identical instead of relying on that.
const TAIL_DOWNSCALE_ENABLED: bool = false;

/// Choose the post-CCM (RGB tail) geometry for a sensor of `w`x`h`.
///
/// Returns `(0, 0)` when the sensor already fits: downscaling a frame that is
/// already small enough would cost a resample and lose detail for nothing.
///
/// Otherwise the factor is the largest divisor of `gcd(w, h)` that brings the
/// longest edge under [`TAIL_MAX_EDGE`]. An integer factor is not a preference
/// — it is required for the graph to run. MNN computes a Resize's extent from
/// the scale as a floor of a float multiply, so a non-integer ratio rounds to a
/// width nobody declared and the session fails (see
/// `TAIL_DOWNSCALE_ENABLED`). Dividing exactly keeps `scale == 1/k` precise and
/// the result square with both the block's own `out_dims` and the tail.
fn post_ccm_downscale(w: u32, h: u32) -> (u32, u32) {
    if w.max(h) <= TAIL_MAX_EDGE {
        return (0, 0);
    }
    let g = gcd(w, h);
    // Divisors of the gcd, ascending, so the first that fits is the mildest.
    let mut best = 0;
    for k in 1..=g {
        if g.is_multiple_of(k) && w.max(h) / k <= TAIL_MAX_EDGE {
            best = k;
            break;
        }
    }
    if best <= 1 {
        // No exact reduction fits the budget. Leave the tail at full size
        // rather than feed the graph a geometry it cannot reproduce.
        return (0, 0);
    }
    ((w / best).max(2), (h / best).max(2))
}

/// Greatest common divisor.
fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

/// Summary of a BGRA frame's brightness, for the periodic status line.
#[derive(Default)]
struct FrameStats {
    mean_r: f64,
    mean_g: f64,
    mean_b: f64,
    max: u8,
    non_black_pct: f64,
}

fn frame_stats(bgra: &[u8]) -> FrameStats {
    if bgra.len() < 4 {
        return FrameStats::default();
    }
    let pixels = bgra.len() / 4;
    let (mut sr, mut sg, mut sb) = (0u64, 0u64, 0u64);
    let mut max = 0u8;
    let mut non_black = 0usize;
    for px in bgra.as_chunks::<4>().0 {
        // BGRA: [b, g, r, a]
        sr += px[2] as u64;
        sg += px[1] as u64;
        sb += px[0] as u64;
        let brightest = px[0].max(px[1]).max(px[2]);
        max = max.max(brightest);
        if brightest > 0 {
            non_black += 1;
        }
    }
    let n = pixels as f64;
    FrameStats {
        mean_r: sr as f64 / n,
        mean_g: sg as f64 / n,
        mean_b: sb as f64 / n,
        max,
        non_black_pct: non_black as f64 * 100.0 / n,
    }
}

/// Range and mean of the raw Bayer samples.
struct RawStats {
    min: u16,
    max: u16,
    mean: f64,
}

fn raw_stats(samples: &[u16]) -> RawStats {
    if samples.is_empty() {
        return RawStats {
            min: 0,
            max: 0,
            mean: 0.0,
        };
    }
    let mut min = u16::MAX;
    let mut max = 0u16;
    let mut sum = 0u64;
    for &v in samples {
        min = min.min(v);
        max = max.max(v);
        sum += v as u64;
    }
    RawStats {
        min,
        max,
        mean: sum as f64 / samples.len() as f64,
    }
}

/// How the bytes in a [`LastFrame`] are laid out.
#[derive(Clone, Copy, PartialEq, Eq)]
enum FrameKind {
    /// Interleaved BGRA bytes, one pixel per 4 bytes.
    Bgra,
    /// Planar RGB f32 (`[1,3,H,W]` in `[0,1]`), as `DisplayBlock::FloatRgb`
    /// emits. The engine cannot hand back interleaved pixels because every u8
    /// display format needs a `Conv(1x1)` whose Vulkan kernel is zero-filled.
    PlanarRgbF32,
}

/// The last successfully processed frame, kept to repaint the surface.
type LastFrame = (Vec<u8>, u32, u32, FrameKind);

/// Present a frame regardless of how it is encoded.
fn present_frame(
    target: &WindowTarget<'_>,
    data: &[u8],
    w: u32,
    h: u32,
    kind: FrameKind,
) -> Result<usize, String> {
    match kind {
        FrameKind::Bgra => target.present(data, w, h),
        FrameKind::PlanarRgbF32 => {
            // Reinterpret the f32 payload without copying: the caller owns a
            // Vec<u8> whose alignment is only 1, so decode through
            // `from_le_bytes` rather than casting a slice.
            let floats: Vec<f32> = data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect();
            target.present_planar_rgb(&floats, w, h)
        }
    }
}

/// Brightness summary of a planar RGB f32 frame.
fn planar_rgb_stats(floats: &[f32]) -> FrameStats {
    let plane = floats.len() / 3;
    if plane == 0 {
        return FrameStats::default();
    }
    let mean = |p: &[f32]| p.iter().map(|v| *v as f64).sum::<f64>() / plane as f64 * 255.0;
    let r = mean(&floats[0..plane]);
    let g = mean(&floats[plane..2 * plane]);
    let b = mean(&floats[2 * plane..3 * plane]);
    let max = floats
        .iter()
        .cloned()
        .fold(0.0f32, f32::max)
        .mul_add(255.0, 0.0)
        .clamp(0.0, 255.0) as u8;
    let non_black = floats.iter().filter(|v| **v > 0.0).count();
    FrameStats {
        mean_r: r,
        mean_g: g,
        mean_b: b,
        max,
        non_black_pct: non_black as f64 * 100.0 / floats.len() as f64,
    }
}

/// All per-window state, dropped and rebuilt when the window changes.
struct Preview {
    engine: Box<dyn IspEngine>,
    session: Option<CameraSession>,
    stager: FrameStager,
    config: PipelineConfig,
    last_frame: Option<LastFrame>,
    /// Most recent present() error, so a persistent surface failure is logged
    /// once (and again only when the message changes) instead of per frame.
    last_present_error: Option<String>,
    frames: u64,
    /// When true, display raw camera frame (grayscale) instead of ISP output
    show_raw: bool,
}

impl Preview {
    /// Build an engine and its block chain for `target_width` pixels wide.
    ///
    /// `engine_name` is passed straight to
    /// [`cam_isp::engine::select_engine_by_name`], so `"cpu"`, `"mnn_vulkan"`
    /// or `"auto"` all work; the choice is a build-time matter of whether the
    /// `mnn` feature (and its bundled native runtime) is present.
    fn new(engine_name: &str, _target_width: u32, _target_height: u32) -> Result<Self, String> {
        let mut engine: Box<dyn IspEngine> = select_engine_by_name(engine_name)
            .ok_or_else(|| format!("no ISP engine matching '{engine_name}'"))?;
        log::info!("ISP engine: {}", engine.backend_name());

        // Try to open a camera session to read sensor capabilities.
        // If the camera permission isn't granted yet (common on first launch),
        // fall back to safe defaults so the pipeline can still be built and the
        // app can launch instead of crashing.
        let (session, has_raw, sensor_max, bayer_pattern, sensor_width, sensor_height) =
            match CameraSession::open_default() {
                Ok(s) => {
                    log::info!(
                        "Camera supports RAW16={}, sensor_max={}, bayer={}, sensor={}x{}",
                        s.is_raw(),
                        s.sensor_max(),
                        s.bayer_pattern(),
                        s.sensor_width(),
                        s.sensor_height()
                    );
                    let props = (
                        s.is_raw(),
                        s.sensor_max(),
                        s.bayer_pattern(),
                        s.sensor_width(),
                        s.sensor_height(),
                    );
                    (Some(s), props.0, props.1, props.2, props.3, props.4)
                }
                Err(e) => {
                    log::warn!("Could not open camera for capability check: {e}");
                    (None, false, 1023.0, camera::BGGR, 1600, 1200)
                }
            };

        // Determine profile based on camera capabilities
        let mut profile = if has_raw {
            log::info!("Camera supports RAW16, using RAW Bayer pipeline (fused isp.* opset)");
            PipelineProfile::FUSED
        } else {
            log::info!("Camera only supports YUV, using YUV input pipeline");
            PipelineProfile::YUV
        };

        // The session used for the capability probe is the one we capture from.
        // Opening a second session would fail — a camera device allows a
        // single active client — and the failure was silently discarded, which
        // left the preview with no frame source at all.

        // Build the pipeline for the *camera sensor* dimensions, not the
        // window dimensions.  The engine must process frames exactly as the
        // camera produces them; WindowTarget::present() scales the output to
        // fit the surface via nearest-neighbour blit.
        let pipeline_width = sensor_width.max(1);
        let pipeline_height = sensor_height.max(1);

        // Shrink the RGB tail (tone, EE, bilateral, sharpen, display) after
        // CCM. The display tensor dominates GPU memory: at 5 Mpx it is 77 MB
        // (fp32) or 38 MB (fp16) per session, and the whole cosmetic chain runs
        // on it. Downscaling here keeps demosaic full resolution — Bayer is a
        // quarter the elements — while making everything downstream cheaper.
        // Expressed in *RGB* coordinates: demosaic halves both edges, so the
        // frame entering the tail is already half the sensor's size.
        profile.post_ccm_downscale = if TAIL_DOWNSCALE_ENABLED {
            post_ccm_downscale(pipeline_width / 2, pipeline_height / 2)
        } else {
            (0, 0)
        };
        if profile.post_ccm_downscale != (0, 0) {
            log::info!(
                "post-CCM downscale: {}x{} -> {}x{}",
                pipeline_width,
                pipeline_height,
                profile.post_ccm_downscale.0,
                profile.post_ccm_downscale.1
            );
        }

        // The *graph* must be built for the same output format the engine is
        // told to expect. `profile.output_format` decides whether DisplayBlock
        // emits a Conv(1x1) permutation or the convolution-free Pow+Clip path;
        // `params.output_format` decides how the engine decodes what comes back.
        // Letting the two disagree is silent: the graph emits `[1,4,H,W]`
        // B,G,R,A planes in 0..255, the engine decodes them as `[1,3,H,W]` f32
        // in 0..1, and every pixel lands 8x too large in the wrong channel.
        profile.output_format = pipeline::OUTPUT_FORMAT;

        // Build the pipeline blocks once, then build two engines with identical chains
        // for dual-GPU ping-pong (engine A / engine B).
        let mut blocks = profile.build_blocks(pipeline_width, pipeline_height, bayer_pattern);
        if blocks.is_empty() {
            return Err(format!("profile {} produced no blocks", profile.label));
        }
        let head_a = blocks.remove(0);
        let aux_a = profile.build_aux_blocks(pipeline_height as i64, pipeline_width as i64);
        blocks.extend(aux_a);

        // Build a second identical set of blocks for the second engine.
        let mut blocks_b = profile.build_blocks(pipeline_width, pipeline_height, bayer_pattern);
        if blocks_b.is_empty() {
            return Err(format!(
                "profile {} produced no blocks (second set)",
                profile.label
            ));
        }
        let head_b = blocks_b.remove(0);
        let aux_b = profile.build_aux_blocks(pipeline_height as i64, pipeline_width as i64);
        blocks_b.extend(aux_b);

        // Tell the engine whether the chain divides by `sensor_max` up front.
        // With `normalize_at_display` there is no NormalizeBlock, so the graph
        // stays in raw sensor counts and the engine scales the display weights
        // and the clamp bounds itself.
        if let Some(mnn) = engine.as_any_mut().downcast_mut::<MnnEngine>() {
            mnn.set_normalizes_at_input(!profile.normalize_at_display);
        }

        engine
            .build(head_a, blocks, None, 21)
            .map_err(|e| format!("engine.build failed: {e}"))?;
        let engine_a = engine;

        let mut engine_b: Box<dyn IspEngine> = select_engine_by_name(engine_name)
            .ok_or_else(|| format!("no ISP engine matching '{engine_name}'"))?;
        if let Some(mnn) = engine_b.as_any_mut().downcast_mut::<MnnEngine>() {
            mnn.set_normalizes_at_input(!profile.normalize_at_display);
        }
        engine_b
            .build(head_b, blocks_b, None, 21)
            .map_err(|e| format!("engine.build failed: {e}"))?;

        let dual_engine = DualGpuEngine::new(engine_a, engine_b);
        let mut engine: Box<dyn IspEngine> = Box::new(dual_engine);

        log::info!(
            "pipeline built: profile {} at {}x{}px{} (dual GPU engine)",
            profile.label,
            pipeline_width,
            pipeline_height,
            match profile.post_ccm_downscale {
                (0, 0) => String::new(),
                (w, h) => format!(", RGB tail {w}x{h}"),
            }
        );

        Ok(Self {
            engine,
            session,
            stager: FrameStager::new(),
            config: PipelineConfig {
                sensor_max,
                bayer_pattern,
            },
            last_frame: None,
            last_present_error: None,
            frames: 0,
            show_raw: false,
        })
    }

    /// Open the camera on first use, and remember what the sensor reported.
    ///
    /// Retries on every call until it succeeds, so a transient camera
    /// open failure (e.g. sensor still initializing) doesn't leave the
    /// preview blank forever.
    fn ensure_camera(&mut self) {
        if self.session.is_some() {
            return;
        }
        match CameraSession::open_default() {
            Ok(s) => {
                let raw = s.is_raw();
                // The sensor's own white level matters: normalisation divides
                // by it, so a hardcoded 10-bit value clips a sensor that
                // actually saturates higher.
                self.config = PipelineConfig {
                    sensor_max: s.sensor_max(),
                    bayer_pattern: s.bayer_pattern(),
                };
                log::info!(
                    "camera open (raw={raw}, sensor_max={})",
                    self.config.sensor_max
                );
                self.session = Some(s);
            }
            Err(e) => {
                // Don't spam the log — only log once every ~1 second.
                static mut LAST_CAMERA_ERROR: Option<Instant> = None;
                let now = Instant::now();
                let should_log = unsafe {
                    match LAST_CAMERA_ERROR {
                        None => {
                            LAST_CAMERA_ERROR = Some(now);
                            true
                        }
                        Some(t) if now.duration_since(t).as_secs() >= 1 => {
                            LAST_CAMERA_ERROR = Some(now);
                            true
                        }
                        _ => false,
                    }
                };
                if should_log {
                    write_line("ERROR", "camera", &e);
                }
            }
        }
    }

    /// Capture one frame, run the ISP, and present it to `target`.
    ///
    /// Never breaks the loop on transient errors — if ISP fails we still
    /// repaint the last good frame so the UI never goes blank.
    fn render_once(&mut self, target: &WindowTarget<'_>) {
        self.ensure_camera();
        let Some(cam) = self.session.as_ref() else {
            return;
        };

        let started = Instant::now();
        // Per-stage marks. "frame took 650ms" says nothing actionable on its
        // own — the cost could be the capture copy, the ISP, or the blit, and
        // they have completely different fixes. These carry the split into the
        // warning line so the log answers "which stage" without a profiler.
        let mut t_capture = Duration::ZERO;
        let mut t_stage = Duration::ZERO;
        let mut t_isp = Duration::ZERO;
        let mut t_present = Duration::ZERO;
        match cam.next_frame() {
            Ok(Some(frame)) => {
                t_capture = started.elapsed();

                // What did the sensor actually deliver?
                //
                // The ISP report further down says what came *out* of the
                // pipeline; without the input's range the two are
                // indistinguishable. All-minimum samples mean the capture is
                // dark (exposure), not that the ISP crushed it.
                if self.frames == 0 || self.frames.is_multiple_of(STATS_INTERVAL_FRAMES) {
                    let raw = raw_stats(&frame.data);
                    write_line(
                        "INFO",
                        "capture",
                        &format!(
                            "raw {}x{} min {} max {} mean {:.1} (sensor_max {})",
                            frame.width,
                            frame.height,
                            raw.min,
                            raw.max,
                            raw.mean,
                            self.config.sensor_max
                        ),
                    );
                }

                let stage_t0 = Instant::now();
                let staged = self.stager.stage(&frame);
                t_stage = stage_t0.elapsed();

                if self.show_raw {
                    // Display raw camera frame as grayscale
                    let raw_bgra = self.raw_to_grayscale_bgra(&frame);
                    let present_t0 = Instant::now();
                    if let Err(e) = present_frame(
                        target,
                        &raw_bgra,
                        frame.width,
                        frame.height,
                        FrameKind::Bgra,
                    ) {
                        log::trace!("present raw: {e}");
                    }
                    t_present = present_t0.elapsed();
                    self.last_frame = Some((raw_bgra, frame.width, frame.height, FrameKind::Bgra));
                } else {
                    let params =
                        self.config
                            .params(frame.width, frame.height, staged, frame.timestamp_ns);
                    let isp_t0 = Instant::now();
                    let result = self.engine.process(&params);
                    t_isp = isp_t0.elapsed();
                    match result {
                        Ok(out) => {
                            self.frames += 1;
                            // `FloatRgb` hands back planar f32; anything else
                            // is already bytes the window can take.
                            let kind = if self.config.float_output() {
                                FrameKind::PlanarRgbF32
                            } else {
                                FrameKind::Bgra
                            };
                            // A frame that processes but cannot be shown is
                            // indistinguishable from a failed one unless the
                            // present error is recorded: this is the only step
                            // between "ISP returned pixels" and "the screen
                            // changed". It used to be a `trace!`, which the
                            // Info-level logger dropped, so a black preview
                            // logged nothing at all.
                            //
                            // Rate-limited to a change of message, so a
                            // per-frame surface error stays one line.
                            let present_t0 = Instant::now();
                            let presented =
                                present_frame(target, &out.data, out.width, out.height, kind);
                            t_present = present_t0.elapsed();
                            match presented {
                                Ok(_) => self.last_present_error = None,
                                Err(e) => {
                                    if self.last_present_error.as_deref() != Some(e.as_str()) {
                                        write_line(
                                            "ERROR",
                                            "display",
                                            &format!("present failed: {e}"),
                                        );
                                        self.last_present_error = Some(e);
                                    }
                                }
                            }
                            self.last_frame = Some((out.data, out.width, out.height, kind));
                        }
                        Err(e) => {
                            write_line("ERROR", "isp", &format!("process failed: {e}"));
                            // ISP failed — repaint the last good frame so the UI
                            // never goes blank.
                            if let Some((data, w, h, kind)) = self.last_frame.as_ref() {
                                let _ = present_frame(target, data, *w, *h, *kind);
                            }
                        }
                    }
                }
            }
            Ok(None) => {
                // Nothing queued: repaint the last good frame so the surface
                // survives instead of showing a stale/blank buffer.
                if let Some((data, w, h, kind)) = self.last_frame.as_ref() {
                    let _ = present_frame(target, data, *w, *h, *kind);
                }
            }
            Err(e) => write_line("ERROR", "camera", &e),
        }

        let elapsed = started.elapsed();
        if elapsed > FRAME_BUDGET {
            write_line(
                "WARN",
                "loop",
                &format!(
                    "frame took {}ms (capture {} / stage {} / isp {} / present {} ms)",
                    elapsed.as_millis(),
                    t_capture.as_millis(),
                    t_stage.as_millis(),
                    t_isp.as_millis(),
                    t_present.as_millis()
                ),
            );
        }
        // The first frame always reports: it is the one that answers "is the
        // picture visible at all", and a session may not survive to frame 5.
        if self.frames == 1
            || (self.frames > 0 && self.frames.is_multiple_of(STATS_INTERVAL_FRAMES))
        {
            let (w, h) = self.last_frame.as_ref().map_or((0, 0), |f| (f.1, f.2));
            // Report what the frame actually contains, not just its size.
            //
            // "N frames processed, last WxH" is satisfied by a completely black
            // buffer: the ISP can return the right length of zeros and every
            // stage above will report success. These numbers are the only way
            // to tell "the pixel work ran" from "the pixel work produced
            // nothing" without pulling the image off the device.
            let content = self
                .last_frame
                .as_ref()
                .map(|(data, _, _, kind)| match kind {
                    FrameKind::Bgra => frame_stats(data),
                    FrameKind::PlanarRgbF32 => {
                        let floats: Vec<f32> = data
                            .as_chunks::<4>()
                            .0
                            .iter()
                            .map(|b| f32::from_le_bytes(*b))
                            .collect();
                        planar_rgb_stats(&floats)
                    }
                })
                .unwrap_or_default();
            write_line(
                "INFO",
                "stats",
                &format!(
                    "{} frames processed, last {w}x{h}, rgb mean [{:.1},{:.1},{:.1}], max {}, non-black {:.1}%",
                    self.frames,
                    content.mean_r,
                    content.mean_g,
                    content.mean_b,
                    content.max,
                    content.non_black_pct
                ),
            );
        }
    }

    /// Convert raw u16 Bayer frame to grayscale BGRA for display
    fn raw_to_grayscale_bgra(&self, frame: &crate::camera::RawFrame) -> Vec<u8> {
        let w = frame.width as usize;
        let h = frame.height as usize;
        let mut bgra = vec![0u8; w * h * 4];
        // Find min/max for auto-contrast
        let mut min_val = u16::MAX;
        let mut max_val = u16::MIN;
        for &sample in &frame.data {
            min_val = min_val.min(sample);
            max_val = max_val.max(sample);
        }
        let range = (max_val.saturating_sub(min_val)).max(1) as u32;
        for (i, &sample) in frame.data.iter().enumerate() {
            let normalized = ((sample.saturating_sub(min_val)) as u32 * 255) / range;
            let gray = normalized as u8;
            let idx = i * 4;
            bgra[idx] = gray; // B
            bgra[idx + 1] = gray; // G
            bgra[idx + 2] = gray; // R
            bgra[idx + 3] = 255; // A
        }
        bgra
    }
}

/// ISP backend requested at build time.
///
/// `mnn` builds name the Vulkan MNN engine, whose runtime libraries are bundled
/// into the APK's `lib/arm64-v8a` by `runtime_libs`; without that feature the
/// crate has no MNN code compiled in, so the CPU engine is the only choice.
#[cfg(feature = "mnn")]
const ENGINE_NAME: &str = "mnn_vulkan";
#[cfg(not(feature = "mnn"))]
const ENGINE_NAME: &str = "cpu";

/// Entry point the `NativeActivity` glue spawns on its own thread.
///
/// This symbol has the Rust ABI, not the C one, and may be called again if the
/// activity is recreated.
#[unsafe(no_mangle)]
fn android_main(app: AndroidApp) {
    // The external data directory needs no runtime permission, so it is the
    // guaranteed-writable fallback when /sdcard/download is out of reach.
    let fallback = app.external_data_path();
    // File first: the logger forwards into it, so the path has to be resolved
    // before any record is emitted or the early lines would be dropped.
    crashlog::install(fallback.as_deref());
    init_logger();

    // Register the ISP engines and builtin blocks before anything selects one.
    // The registry starts empty: without this call `select_engine_by_name`
    // matches nothing, and the first frame dies with "no ISP engine matching
    // ...". It is idempotent, so a recreated activity may call it again.
    cam_isp::init();

    let sdk = AndroidApp::sdk_version();
    log::info!("softisp camera starting (sdk {sdk}, engine {ENGINE_NAME})");
    if let Some(p) = crashlog::active_path() {
        log::info!("log file: {}", p.display());
    }
    // Recording why the spec path was skipped turns a silent log-location
    // change into something the user can act on.
    if crashlog::active_path() != Some(Path::new(crashlog::LOG_PATH)) {
        write_line("WARN", "storage", &storage_note(sdk, fallback.as_deref()));
    }

    let mut exit = false;
    let mut window: Option<NativeWindow> = None;
    // Built once the window reports a size, since the block chain is sized
    // from the preview's output width.
    let mut preview: Option<Preview> = None;
    // Reused WindowTarget across frames so surface buffers are configured
    // only once per window lifetime, not every frame. Recreating it each
    // iteration re-runs set_buffers_geometry and causes the compositor to
    // drop/delay frames, so we keep it stable until the window is replaced.
    let mut target: Option<WindowTarget<'static>> = None;

    while !exit {
        app.poll_events(Some(POLL_TIMEOUT), |event| {
            match event {
                PollEvent::Main(MainEvent::Destroy) => exit = true,
                // A resize or config change re-runs InitWindow on some
                // framework versions and not on others, so all three
                // re-take the window rather than trusting one of them.
                PollEvent::Main(MainEvent::InitWindow { .. })
                | PollEvent::Main(MainEvent::WindowResized { .. })
                | PollEvent::Main(MainEvent::ConfigChanged { .. }) => {
                    if let Some(win) = app.native_window() {
                        // Hand the ANativeWindow to cam-isp so it can render
                        // output into this surface. This is called before any
                        // ISP processing happens, so the window is ready when
                        // the first frame arrives.
                        unsafe { set_native_window(win.ptr().as_ptr() as *mut c_void) };
                        window = Some(win);
                    }
                }
                PollEvent::Main(MainEvent::TerminateWindow { .. }) => {
                    // Drop the surface: release the ANativeWindow ref held by
                    // cam-isp so the surface can be destroyed cleanly.
                    release_native_window();
                    // Drop the cached WindowTarget so its NativeWindow borrow
                    // is released before the window is destroyed by the framework.
                    target = None;
                    window = None;
                }
                PollEvent::Main(MainEvent::InputAvailable) => {
                    // The preview owns the whole surface, so input has no
                    // effect. Drain the queue anyway: leaving events pending
                    // makes Android redeliver this callback every loop.
                    if let Ok(mut events) = app.input_events_iter() {
                        while events.next(|_ev| InputStatus::Handled) {}
                    }
                }
                PollEvent::Wake | PollEvent::Timeout => {}
                _ => {}
            }
        });

        if exit {
            break;
        }
        let Some(win) = window.as_ref() else {
            continue;
        };
        // Diagnostic proxy: report that we got an ANativeWindow from the
        // activity glue and what geometry it claims.
        write_line(
            "WARN",
            "window",
            &format!(
                "InitWindow callback reached: NativeWindow ptr={:p}",
                win.ptr().as_ptr()
            ),
        );
        let (w, h) = (win.width() as u32, win.height() as u32);
        write_line("WARN", "window", &format!("window geometry {w}x{h}"));
        // A freshly created surface reports 0×0 until the activity is laid
        // out. Sizing a block chain to zero pixels divides by zero inside the
        // profile builder, so wait for a real geometry instead.
        if w == 0 || h == 0 {
            continue;
        }
        if preview.is_none() {
            match Preview::new(ENGINE_NAME, w, h) {
                Ok(mut p) => {
                    // Show ISP pipeline output by default. Set show_raw = true
                    // to bypass the ISP and display grayscale raw Bayer frames
                    // instead (useful for verifying camera capture is working).
                    p.show_raw = false;
                    preview = Some(p);
                }
                Err(e) => {
                    write_line("FATAL", "engine", &e);
                    return;
                }
            }
        }
        // Create the WindowTarget once per window lifetime (not every frame).
        // Re-running set_buffers_geometry each iteration forces the compositor
        // to reallocate/reconfigure surface buffers, which causes frame drops
        // and can crash the Vulkan/Gralloc path.
        let target: &mut WindowTarget<'static> = match target.as_mut() {
            Some(t) => t,
            None => {
                let t = WindowTarget::new(win, w as i32, h as i32);
                match t {
                    Ok(t) => {
                        // SAFETY: WindowTarget<'static> borrows `win`
                        // (a &NativeWindow) which lives for the duration of the
                        // activity. NativeWindow is Send+Sync and its pointer
                        // is stable while the window is active.
                        let t = unsafe {
                            std::mem::transmute::<WindowTarget<'_>, WindowTarget<'static>>(t)
                        };
                        target.insert(t)
                    }
                    Err(e) => {
                        log::trace!("window target not ready: {e}");
                        continue;
                    }
                }
            }
        };
        preview.as_mut().unwrap().render_once(target);
    }

    let frames = preview.as_ref().map_or(0, |p| p.frames);
    log::info!("exiting after {frames} frames");
    write_line("INFO", "exit", &format!("processed {frames} frames"));
}

/// Explain, in the log, why the mandated `/sdcard/download` path was skipped.
///
/// Re-probes the spec path so the message reports the live error rather than a
/// guess, and names the concrete grant the user needs.
fn storage_note(sdk: i32, fallback: Option<&Path>) -> String {
    let spec = Path::new(crashlog::LOG_PATH);
    let (access, reason) = storage::probe(spec, false);
    let mut note = match access {
        Access::Writable => format!("{} became writable after startup", spec.display()),
        Access::Denied => {
            let why = reason.unwrap_or_else(|| "unknown reason".into());
            let grant = if sdk >= 30 {
                "grant 'All files access' for com.softisp.camera in Settings"
            } else {
                "grant android.permission.WRITE_EXTERNAL_STORAGE"
            };
            format!(
                "{} unavailable: {why}; the log went to the app's external \
                 directory — {grant} to use this path",
                spec.display()
            )
        }
    };
    if let Some(dir) = fallback {
        note.push_str(&format!(" (fallback: {})", dir.display()));
    }
    note
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn post_ccm_downscale_uses_an_exact_integer_factor() {
        // The factor must divide both RGB edges exactly. (The Resize path now
        // also survives an inexact ratio, but an exact one keeps the declared
        // and computed shapes identical instead of relying on that.)
        let (w, h) = post_ccm_downscale(2320, 1736);
        assert_eq!((w, h), (1160, 868));
        assert_eq!(2320 % 2, 0);
        assert_eq!(1736 % 2, 0);
        let src = 2320.0 / 1736.0;
        let dst = w as f64 / h as f64;
        assert!((src - dst).abs() < 1e-9, "aspect drifted: {src} vs {dst}");
    }

    #[test]
    fn post_ccm_downscale_prefers_the_mildest_factor_that_fits() {
        // 4640x3472 Bayer is 2320x1736 RGB; /2 is the first exact factor that
        // brings it under the cap.
        assert_eq!(post_ccm_downscale(2320, 1736), (1160, 868));
        // 3268x2448 Bayer is 1634x1224 RGB, which already fits -> no resample.
        assert_eq!(post_ccm_downscale(1634, 1224), (0, 0));
    }

    #[test]
    fn post_ccm_downscale_skips_sensors_with_no_exact_factor() {
        // 1921 is prime-ish with the height: no factor brings it under the cap
        // without inventing a size, so the tail stays full-resolution.
        assert_eq!(post_ccm_downscale(4000, 3001), (0, 0));
    }

    #[test]
    fn post_ccm_downscale_is_off_when_the_sensor_already_fits() {
        // Resampling a frame that already fits would cost detail for nothing.
        assert_eq!(post_ccm_downscale(1920, 1080), (0, 0));
        assert_eq!(post_ccm_downscale(1280, 720), (0, 0));
        assert_eq!(post_ccm_downscale(1920, 1920), (0, 0));
    }

    #[test]
    fn post_ccm_downscale_caps_a_tall_frame_by_its_long_edge() {
        // Portrait frame: the cap applies to the longer (vertical) edge.
        // 1940x2592 RGB shares only small factors with the cap, so this stays
        // full-size rather than inventing a geometry.
        assert_eq!(post_ccm_downscale(1940, 2592), (970, 1296));
        let (w, h) = post_ccm_downscale(1940, 2592);
        assert!(w <= super::TAIL_MAX_EDGE && h <= super::TAIL_MAX_EDGE);
    }
}

#[cfg(test)]
mod frame_stats_tests {
    use super::frame_stats;

    #[test]
    fn black_frame_reports_zeros() {
        // This is the report that makes a black preview visible: every other
        // signal (frame count, size, present()) looks healthy.
        let black = vec![0u8; 4 * 100];
        let s = frame_stats(&black);
        assert_eq!(s.max, 0);
        assert_eq!(s.non_black_pct, 0.0);
        assert_eq!(s.mean_r, 0.0);
    }

    #[test]
    fn bgra_channels_are_read_in_order() {
        // Memory order is B,G,R,A; the summary must not swap red and blue.
        let px = [10u8, 20, 30, 255];
        let s = frame_stats(&px);
        assert_eq!(s.mean_b, 10.0);
        assert_eq!(s.mean_g, 20.0);
        assert_eq!(s.mean_r, 30.0);
        assert_eq!(s.max, 30);
        assert_eq!(s.non_black_pct, 100.0);
    }

    #[test]
    fn short_buffer_is_handled() {
        assert_eq!(frame_stats(&[]).max, 0);
        assert_eq!(frame_stats(&[1, 2]).max, 0);
    }
}

#[cfg(test)]
mod raw_stats_tests {
    use super::raw_stats;

    #[test]
    fn reports_range_and_mean() {
        let s = raw_stats(&[0, 512, 1023]);
        assert_eq!(s.min, 0);
        assert_eq!(s.max, 1023);
        assert!((s.mean - 511.67).abs() < 0.1);
    }

    #[test]
    fn black_capture_is_visible_in_the_numbers() {
        // An unexposed frame is all near-zero; this is what distinguishes
        // "sensor dark" from "ISP crushed it".
        let s = raw_stats(&[1, 2, 0, 3]);
        assert_eq!(s.max, 3);
        assert!(s.mean < 2.0);
    }

    #[test]
    fn empty_is_handled() {
        assert_eq!(raw_stats(&[]).max, 0);
    }
}
