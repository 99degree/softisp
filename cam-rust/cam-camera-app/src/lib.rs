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
use cam_isp::engine::{select_engine_by_name, IspEngine};
use cam_isp::profile::PipelineProfile;
use cam_isp::{release_native_window, set_native_window};

use camera::CameraSession;
use crashlog::write_line;
use pipeline::{FrameStager, PipelineConfig};
use preview::WindowTarget;
use storage::Access;

/// Frames processed between two stat lines. At 30 fps this is one line per
/// second: frequent enough to spot a stall, sparse enough to stay readable.
const STATS_INTERVAL_FRAMES: u64 = 30;

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

/// Route `log::` records to logcat.
///
/// The app has no Java layer, and the default `log` implementation discards
/// everything, which would make the engine/block-chain diagnostics invisible.
/// `android_logger` is a target dependency pulled in only for Android (see
/// Cargo.toml), so on the host this compiles to nothing and the tests stay
/// hermetic.
fn init_logger() {
    #[cfg(target_os = "android")]
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("softisp"),
    );
}

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
    fn new(engine_name: &str, target_width: u32, sensor_height: u32) -> Result<Self, String> {
        let mut engine: Box<dyn IspEngine> = select_engine_by_name(engine_name)
            .ok_or_else(|| format!("no ISP engine matching '{engine_name}'"))?;
        log::info!("ISP engine: {}", engine.backend_name());

        // The chain comes from the same profile the desktop tooling uses, so
        // preview and benchmarks exercise identical blocks.
        let profile = PipelineProfile::MED;
        let mut blocks = profile.build_blocks(target_width, sensor_height, camera::BGGR);
        if blocks.is_empty() {
            return Err(format!("profile {} produced no blocks", profile.label));
        }
        let head = blocks.remove(0);
        let aux = profile.build_aux_blocks(sensor_height as i64, target_width as i64);
        blocks.extend(aux);
        engine
            .build(head, blocks, None, 21)
            .map_err(|e| format!("engine.build failed: {e}"))?;
        log::info!(
            "pipeline built: profile {} at {}x{}px",
            profile.label,
            target_width,
            sensor_height
        );

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
        match cam.next_frame() {
            Ok(Some(frame)) => {
                let staged = self.stager.stage(&frame);

                if self.show_raw {
                    // Display raw camera frame as grayscale
                    let raw_bgra = self.raw_to_grayscale_bgra(&frame);
                    if let Err(e) = target.present(&raw_bgra, frame.width, frame.height) {
                        log::trace!("present raw: {e}");
                    }
                    self.last_frame = Some((raw_bgra, frame.width, frame.height));
                } else {
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
                        Err(e) => {
                            write_line("ERROR", "isp", &format!("process failed: {e}"));
                            // ISP failed — repaint the last good frame so the UI
                            // never goes blank.
                            if let Some((data, w, h)) = self.last_frame.as_ref() {
                                let _ = target.present(data, *w, *h);
                            }
                        }
                    }
                }
            }
            Ok(None) => {
                // Nothing queued: repaint the last good frame so the surface
                // survives instead of showing a stale/blank buffer.
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
    init_logger();
    crashlog::install(fallback.as_deref());

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
                    // Show raw camera feed by default to verify camera is working
                    p.show_raw = true;
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
