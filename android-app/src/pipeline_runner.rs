//! Two-worker ISP pipeline.
//!
//! `DualGpuEngine::process()` is a round-robin selector, not a pipeline: it
//! waits for engine A to finish before engine B starts, so the second engine
//! only doubled session memory. This module drives one engine per worker
//! thread instead. Camera capture is serialized behind a mutex; each finished
//! frame carries a sequence number assigned at capture time, and the
//! presenting thread reorders results before blitting them. Presentation and
//! window lifetime stay on the main thread — `ANativeWindow` is not touched
//! from a worker.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use cam_isp::engine::IspEngine;

use crate::camera::{CameraSession, RawFrame};
use crate::pipeline::{FrameStager, PipelineConfig};
use crate::{raw_stats, raw_to_grayscale_bgra, FrameKind, RawStats};

/// One finished frame, handed back to the presenting thread.
pub(crate) struct Done {
    pub(crate) seq: u64,
    pub(crate) worker: usize,
    pub(crate) raw: RawStats,
    pub(crate) capture: Duration,
    pub(crate) stage: Duration,
    pub(crate) isp: Duration,
    /// ISP failure. Reported without stalling the reorder queue: the sequence
    /// is consumed so later frames are not held behind a failed one.
    pub(crate) error: Option<String>,
    pub(crate) data: Vec<u8>,
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) kind: FrameKind,
}

/// Owns the camera, the two pinned engines, and the reorder queue.
///
/// Dropping the runner signals both workers and joins them, so engines are
/// never torn down while a `process()` call is in flight.
pub(crate) struct PipelineRunner {
    camera: Arc<Mutex<Option<CameraSession>>>,
    stop: Arc<AtomicBool>,
    show_raw: Arc<AtomicBool>,
    rx: Receiver<Done>,
    workers: Vec<JoinHandle<()>>,
    next_to_present: u64,
    pending: BTreeMap<u64, Done>,
}

/// State shared by both workers. Bundled so `worker_loop` stays below the
/// clippy argument limit without weakening any of the ownership guarantees.
struct WorkerShared {
    camera: Arc<Mutex<Option<CameraSession>>>,
    stop: Arc<AtomicBool>,
    seq: Arc<AtomicU64>,
    show_raw: Arc<AtomicBool>,
    config: PipelineConfig,
    tx: Sender<Done>,
}

impl WorkerShared {
    fn clone_for_worker(&self) -> Self {
        Self {
            camera: Arc::clone(&self.camera),
            stop: Arc::clone(&self.stop),
            seq: Arc::clone(&self.seq),
            show_raw: Arc::clone(&self.show_raw),
            config: self.config,
            tx: self.tx.clone(),
        }
    }
}

impl PipelineRunner {
    /// Start one worker per engine. `camera` may be `None` on first launch
    /// while permission is pending; `set_camera` installs it later.
    pub(crate) fn new(
        camera: Option<CameraSession>,
        engines: [Box<dyn IspEngine>; 2],
        config: PipelineConfig,
        show_raw: Arc<AtomicBool>,
    ) -> Self {
        let camera = Arc::new(Mutex::new(camera));
        let stop = Arc::new(AtomicBool::new(false));
        let seq = Arc::new(AtomicU64::new(0));
        let (tx, rx) = mpsc::channel();
        let shared = WorkerShared {
            camera: Arc::clone(&camera),
            stop: Arc::clone(&stop),
            seq,
            show_raw: Arc::clone(&show_raw),
            config,
            tx,
        };

        let workers = engines
            .into_iter()
            .enumerate()
            .map(|(i, engine)| {
                let shared = shared.clone_for_worker();
                thread::Builder::new()
                    .name(format!("isp-worker-{i}"))
                    .spawn(move || worker_loop(i, engine, shared))
                    .expect("spawn ISP worker")
            })
            .collect();

        Self {
            camera,
            stop,
            show_raw,
            rx,
            workers,
            next_to_present: 0,
            pending: BTreeMap::new(),
        }
    }

    /// Install (or replace) the camera session. Palette probe on the main
    /// thread calls this once permission has been granted.
    pub(crate) fn set_camera(&self, camera: CameraSession) {
        *self.camera.lock().unwrap() = Some(camera);
    }

    /// Drop the current session after a fatal camera callback.
    ///
    /// Workers hold the mutex only for the duration of `next_frame()`, so this
    /// waits for an in-flight capture to finish and then leaves both workers
    /// idle until `set_camera()` installs a fresh device.
    pub(crate) fn clear_camera(&self) {
        *self.camera.lock().unwrap() = None;
    }

    pub(crate) fn has_camera(&self) -> bool {
        self.camera.lock().unwrap().is_some()
    }

    /// Toggle the debug raw-grayscale preview without restarting workers.
    pub(crate) fn set_show_raw(&self, value: bool) {
        self.show_raw.store(value, Ordering::Relaxed);
    }

    /// Return the next completed frame in capture order, if one is ready.
    ///
    /// A worker that finishes second is buffered until its predecessor
    /// arrives, so the surface never shows frame N+1 before N.
    pub(crate) fn poll(&mut self) -> Option<Done> {
        while let Ok(done) = self.rx.try_recv() {
            self.pending.insert(done.seq, done);
        }
        match self.pending.remove(&self.next_to_present) {
            Some(done) => {
                self.next_to_present += 1;
                Some(done)
            }
            None => None,
        }
    }
}

impl Drop for PipelineRunner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
    }
}

/// Body of one pinned worker. At most one frame is in flight per worker, so
/// the result channel is naturally bounded to two entries.
fn worker_loop(worker: usize, engine: Box<dyn IspEngine>, shared: WorkerShared) {
    let WorkerShared {
        camera,
        stop,
        seq,
        show_raw,
        config,
        tx,
    } = shared;
    let mut stager = FrameStager::new();

    while !stop.load(Ordering::Relaxed) {
        let capture_t0 = Instant::now();
        let captured = {
            let guard = camera.lock().unwrap();
            match guard.as_ref() {
                Some(cam) => cam.next_frame(),
                None => Ok(None),
            }
        };
        let capture = capture_t0.elapsed();

        let frame: RawFrame = match captured {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                thread::sleep(Duration::from_millis(2));
                continue;
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };

        // Assigned under the camera lock above, so sequence order == capture
        // order even though the two workers finish in either order.
        let s = seq.fetch_add(1, Ordering::SeqCst);
        let raw = raw_stats(&frame.data);

        let stage_t0 = Instant::now();
        let staged = stager.stage(&frame);
        let stage = stage_t0.elapsed();

        let (data, width, height, kind, isp, error) = if show_raw.load(Ordering::Relaxed) {
            (
                raw_to_grayscale_bgra(&frame),
                frame.width,
                frame.height,
                FrameKind::Bgra,
                Duration::ZERO,
                None,
            )
        } else {
            let params = config.params(frame.width, frame.height, staged, frame.timestamp_ns);
            let isp_t0 = Instant::now();
            let result = engine.process(&params);
            let isp = isp_t0.elapsed();
            match result {
                Ok(out) => {
                    let kind = if config.float_output() {
                        FrameKind::PlanarRgbF32
                    } else {
                        FrameKind::Bgra
                    };
                    (out.data, out.width, out.height, kind, isp, None)
                }
                Err(e) => (
                    Vec::new(),
                    frame.width,
                    frame.height,
                    FrameKind::Bgra,
                    isp,
                    Some(e.to_string()),
                ),
            }
        };

        let done = Done {
            seq: s,
            worker,
            raw,
            capture,
            stage,
            isp,
            error,
            data,
            width,
            height,
            kind,
        };
        if tx.send(done).is_err() {
            break;
        }
    }
}
