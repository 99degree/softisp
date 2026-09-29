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

use std::time::{Duration, Instant};

use android_activity::ndk::native_window::NativeWindow;
use android_activity::{AndroidApp, InputStatus, MainEvent, PollEvent};
use cam_isp::engine::{select_engine_by_name, IspEngine};

use camera::CameraSession;
use crashlog::write_line;
use pipeline::{FrameStager, PipelineConfig};
use preview::WindowTarget;

/// Frames processed between two stat lines. At 30 fps this is one line per
/// second: frequent enough to spot a stall, sparse enough to stay readable.
const STATS_INTERVAL_FRAMES: u64 = 30;

/// How long the event loop blocks waiting for a new frame or lifecycle event.
///
/// A finite timeout keeps the preview ticking even when the camera produces
/// nothing, so the surface is repainted instead of freezing.
const POLL_TIMEOUT: Duration = Duration::from_millis(100);

/// Wall-clock budget for one capture + ISP + present pass.
///
/// The capture queue holds three frames, so exceeding this self-throttles the
/// preview instead of letting latency grow without bound.
const FRAME_BUDGET: Duration = Duration::from_millis(500);

/// The last successfully processed frame, kept to repaint the surface.
type LastFrame = (Vec<u8>, u32, u32);

/// All per-window state, dropped and rebuilt when the window changes.
struct Preview {
    engine: Box<dyn IspEngine>,
    session: Option<CameraSession>,
    stager: FrameStager,
    config: PipelineConfig,
    last_frame: Option<LastFrame>,
    frames: u64,
}

impl Preview {
    fn new() -> Result<Self, String> {
        // "cpu" is explicit: the MNN backend needs its own native runtime,
        // which is not shipped inside this APK.
        let engine = select_engine_by_name("cpu").ok_or("no ISP engine available")?;
        log::info!("ISP engine: {}", engine.backend_name());
        Ok(Self {
            engine,
            session: None,
            stager: FrameStager::new(),
            config: PipelineConfig {
                sensor_max: 1023.0,
                bayer_pattern: camera::BGGR,
            },
            last_frame: None,
            frames: 0,
        })
    }

    /// Open the camera on first use, and remember what the sensor reported.
    fn ensure_camera(&mut self) {
        if self.session.is_some() {
            return;
        }
        match CameraSession::open_default() {
            Ok(s) => {
                let raw = s.is_raw();
                self.config = PipelineConfig {
                    sensor_max: 1023.0,
                    bayer_pattern: s.bayer_pattern(),
                };
                log::info!("camera open (raw={raw})");
                self.session = Some(s);
            }
            Err(e) => write_line("ERROR", "camera", &e),
        }
    }

    /// Capture one frame, run the ISP, and present it to `target`.
    fn render_once(&mut self, target: &WindowTarget<'_>) {
        self.ensure_camera();
        let Some(cam) = self.session.as_ref() else {
            return;
        };

        let started = Instant::now();
        match cam.next_frame() {
            Ok(Some(frame)) => {
                let staged = self.stager.stage(&frame);
                let params =
                    self.config
                        .params(frame.width, frame.height, staged, frame.timestamp_ns);
                match self.engine.process(&params) {
                    Ok(out) => {
                        self.frames += 1;
                        if let Err(e) = target.present(&out.data, out.width, out.height) {
                            log::trace!("present: {e}");
                        }
                        self.last_frame = Some((out.data, out.width, out.height));
                    }
                    Err(e) => write_line("ERROR", "isp", &format!("process failed: {e}")),
                }
            }
            Ok(None) => {
                // Nothing queued: repaint so the surface survives a resize
                // instead of showing a stale buffer.
                if let Some((data, w, h)) = self.last_frame.as_ref() {
                    let _ = target.present(data, *w, *h);
                }
            }
            Err(e) => write_line("ERROR", "camera", &e),
        }

        let elapsed = started.elapsed();
        if elapsed > FRAME_BUDGET {
            write_line(
                "WARN",
                "loop",
                &format!("frame took {}ms", elapsed.as_millis()),
            );
        }
        if self.frames > 0 && self.frames.is_multiple_of(STATS_INTERVAL_FRAMES) {
            let (w, h) = self.last_frame.as_ref().map_or((0, 0), |f| (f.1, f.2));
            write_line(
                "INFO",
                "stats",
                &format!("{} frames processed, last {w}x{h}", self.frames),
            );
        }
    }
}

/// Entry point the `NativeActivity` glue spawns on its own thread.
///
/// This symbol has the Rust ABI, not the C one, and may be called again if the
/// activity is recreated.
#[unsafe(no_mangle)]
fn android_main(app: AndroidApp) {
    crashlog::install();
    log::info!(
        "softisp camera starting (sdk {})",
        AndroidApp::sdk_version()
    );

    let mut preview = match Preview::new() {
        Ok(p) => p,
        Err(e) => {
            write_line("FATAL", "engine", &e);
            return;
        }
    };
    let mut exit = false;
    let mut window: Option<NativeWindow> = None;
    let mut resized = ConfigChangeTracker::default();

    while !exit {
        app.poll_events(Some(POLL_TIMEOUT), |event| {
            match event {
                PollEvent::Main(MainEvent::Destroy) => exit = true,
                PollEvent::Main(MainEvent::InitWindow { .. }) => window = app.native_window(),
                PollEvent::Main(MainEvent::TerminateWindow { .. }) => window = None,
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
        // Rotation and density changes are absorbed by the manifest's
        // `configChanges`; re-take the window so the presentation buffers are
        // allocated at the new size instead of stretching stale ones.
        if resized.changed(&app) {
            window = app.native_window();
        }
        let Some(win) = window.as_ref() else {
            continue;
        };
        let (w, h) = (win.width(), win.height());
        let target = match WindowTarget::new(win, w, h) {
            Ok(t) => t,
            Err(e) => {
                log::trace!("window not ready: {e}");
                continue;
            }
        };
        preview.render_once(&target);
    }

    let frames = preview.frames;
    log::info!("exiting after {frames} frames");
    write_line("INFO", "exit", &format!("processed {frames} frames"));
}

/// Tracks the window size so a resize re-allocates the presentation buffers.
#[derive(Default)]
struct ConfigChangeTracker {
    last: Option<(i32, i32)>,
}

impl ConfigChangeTracker {
    /// `true` when the window size changed since the previous call.
    fn changed(&mut self, app: &AndroidApp) -> bool {
        let size = app.native_window().map(|w| (w.width(), w.height()));
        let changed = self.last.is_some() && self.last != size;
        self.last = size;
        changed
    }
}
