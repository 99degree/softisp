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

#[cfg(feature = "direct_vulkan")]
use cam_isp::direct_vulkan::DirectVulkanEngine;
use cam_isp::engine::{IspEngine, PrepareParams};
use cam_isp::mnnengine::{MnnEngine, PreparedFrame};

use crate::ahb_input::AhbInputSlot;
use crate::ahb_output::{self, AhbOutputSlot, AhbView};
use crate::app::{raw_stats, raw_to_grayscale_bgra, FrameKind, RawStats};
use crate::camera::{CameraSession, RawFrame};
use crate::pipeline::PipelineConfig;

/// Cloneable cancellation signal shared with code outside the workers.
#[derive(Clone, Debug)]
pub struct PipelineHandle {
    stop: Arc<AtomicBool>,
}

impl PipelineHandle {
    pub(crate) fn new(stop: Arc<AtomicBool>) -> Self {
        Self { stop }
    }

    /// Request cancellation. Repeated calls have no additional effect.
    pub fn cancel(&self) {
        self.stop.store(true, Ordering::Release);
    }

    /// Whether cancellation has been requested.
    pub fn is_cancelled(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }
}

/// One finished frame, handed back to the presenting thread.
pub(crate) struct Done {
    pub(crate) seq: u64,
    pub(crate) worker: usize,
    pub(crate) raw: RawStats,
    /// Monotonic timestamp (ns) when the frame was captured from the camera.
    pub(crate) t_capture_ns: u64,
    /// Monotonic timestamp (ns) when staging finished.
    pub(crate) t_stage_ns: u64,
    /// Monotonic timestamp (ns) when MNN prepare finished.
    pub(crate) t_mnn_prepare_ns: u64,
    /// Monotonic timestamp (ns) when MNN inference finished.
    pub(crate) t_mnn_inference_ns: u64,
    /// Monotonic timestamp (ns) when MNN output finished.
    pub(crate) t_mnn_output_ns: u64,
    /// ISP failure. Reported without stalling the reorder queue: the sequence
    /// is consumed so later frames are not held behind a failed one.
    pub(crate) error: Option<String>,
    pub(crate) data: Vec<u8>,
    /// Presenter-facing view of an AHB-backed output. When `Some`, `data` is
    /// empty and the pixels live in this view's mapping.
    pub(crate) ahb: Option<AhbView>,
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
    handle: PipelineHandle,
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
        ahb_output::log_switch_state();
        let camera = Arc::new(Mutex::new(camera));
        let handle = PipelineHandle::new(Arc::new(AtomicBool::new(false)));
        let seq = Arc::new(AtomicU64::new(0));
        let (tx, rx) = mpsc::channel();
        let shared = WorkerShared {
            camera: Arc::clone(&camera),
            stop: Arc::clone(&handle.stop),
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
            handle,
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

    /// Return a cloneable handle for external cancellation.
    pub(crate) fn handle(&self) -> PipelineHandle {
        self.handle.clone()
    }

    /// Stop the pipeline and release camera/engine/frame resources.
    ///
    /// Workers finish the frame already in flight, then exit. Joining drops the
    /// engine (and any staged tensor guards) on the worker thread before the
    /// camera session is cleared here. Repeated calls are harmless.
    pub(crate) fn cancel(&mut self) {
        self.handle.cancel();
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
        while self.rx.try_recv().is_ok() {}
        self.pending.clear();
        *self.camera.lock().unwrap() = None;
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
        self.cancel();
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
    let mnn = engine.as_any().downcast_ref::<MnnEngine>();
    #[cfg(feature = "direct_vulkan")]
    let direct = engine.as_any().downcast_ref::<DirectVulkanEngine>();
    #[cfg(feature = "direct_vulkan")]
    let direct_active = direct.is_some();
    #[cfg(not(feature = "direct_vulkan"))]
    let direct_active = false;
    #[cfg(feature = "direct_vulkan")]
    let uses_ahb = mnn.is_some() || direct.is_some();
    #[cfg(not(feature = "direct_vulkan"))]
    let uses_ahb = mnn.is_some();
    let input_ahb_off = std::env::var_os("SOFTISP_INPUT_AHB_OFF").is_some()
        || std::path::Path::new("/sdcard/download/softisp-input-ahb-off").exists();
    let mut geometry: Option<(u32, u32)> = None;
    let mut prepared: Option<PreparedFrame<'_>> = None;
    let mut ahb_slot: Option<AhbOutputSlot> = None;
    let mut input_ahb_slot: Option<AhbInputSlot> = None;

    while !stop.load(Ordering::Acquire) {
        // One AHB per worker, allocated once the geometry is known and reused
        // for the session lifetime. Rebinding would leave MNN's imported
        // VkBuffer pointing at the old buffer, so a size change drops and
        // recreates the slot (and invalidates any staged prepare).
        if ahb_output::output_ahb_enabled() && uses_ahb {
            if let Some((w, h)) = geometry {
                let stale = ahb_slot
                    .as_ref()
                    .is_none_or(|s| s.width() != w || s.height() != h);
                if stale {
                    match AhbOutputSlot::new(w, h, 0) {
                        Ok(slot) => {
                            ahb_slot = Some(slot);
                            prepared = None;
                        }
                        Err(e) => log::warn!("AHB output slot unavailable: {e}"),
                    }
                }
            }
        }

        // MNN caches the imported AHB for a tensor/geometry, so the input slot
        // must stay stable until the geometry changes. A new slot invalidates
        // the prepared session; the binding is reinstalled before its resize.
        if uses_ahb && !input_ahb_off {
            if let Some((w, h)) = geometry {
                let stale = input_ahb_slot
                    .as_ref()
                    .is_none_or(|slot| !slot.matches(w, h));
                if stale {
                    match AhbInputSlot::new(w, h, 0) {
                        Ok(slot) => {
                            input_ahb_slot = Some(slot);
                            prepared = None;
                        }
                        Err(e) => {
                            input_ahb_slot = None;
                            log::warn!("AHB input slot unavailable: {e}; using host staging");
                        }
                    }
                }
            }
        }

        // Prep the next frame while the camera is still filling its buffer.
        // The controller already holds the values this frame will consume, so
        // no sensor pixels are needed for the tensor uploads or session resize.
        if prepared.is_none() {
            if let (Some(mnn), Some((w, h))) = (mnn, geometry) {
                let mut prepare_params = config.params(w, h, &[], 0);
                if let Some(slot) = input_ahb_slot.as_ref().filter(|s| s.matches(w, h)) {
                    let (binding, buf) = slot.buffers();
                    prepare_params.input_buffer = Some(buf);
                    prepare_params.input_ahb = Some(binding);
                }
                if let Some(slot) = ahb_slot.as_ref() {
                    let (binding, buf) = slot.buffers();
                    prepare_params.output_buffer = Some(buf);
                    prepare_params.output_ahb = Some(binding);
                }
                let prep_t0 = Instant::now();
                prepared = mnn.prepare(&prepare_params).ok();
                let _ = prep_t0.elapsed().as_nanos() as u64; // engine-internal timing in PreparedMeta
            }
        }
        if stop.load(Ordering::Acquire) {
            break;
        }

        let capture_t0 = Instant::now();
        let captured = {
            let guard = camera.lock().unwrap();
            match guard.as_ref() {
                Some(cam) => cam.next_frame(),
                None => Ok(None),
            }
        };
        if stop.load(Ordering::Acquire) {
            break;
        }

        let frame: RawFrame = match captured {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                if stop.load(Ordering::Acquire) {
                    break;
                }
                thread::sleep(Duration::from_millis(2));
                continue;
            }
            Err(_) => {
                if stop.load(Ordering::Acquire) {
                    break;
                }
                thread::sleep(Duration::from_millis(5));
                continue;
            }
        };
        geometry = Some((frame.width, frame.height));

        #[cfg(feature = "direct_vulkan")]
        if direct_active && mnn.is_none() && (input_ahb_slot.is_none() || ahb_slot.is_none()) {
            if let Some(slot) = ahb_slot.as_ref() {
                slot.mark_free();
            }
            log::debug!("direct Vulkan waiting for AHB slots; dropping warmup frame");
            continue;
        }

        // Assigned under the camera lock above, so sequence order == capture
        // order even though the two workers finish in either order. Every
        // assigned sequence must reach the presenter: skipping one would stall
        // `PipelineRunner::poll()` forever at that missing sequence.
        let s = seq.fetch_add(1, Ordering::SeqCst);
        let raw = raw_stats(&frame.data);

        let stage_t0 = Instant::now();
        // RAW16 camera AHBs on this device expose only sampled-image Vulkan
        // memory types and cannot be imported as storage VkBuffers. The direct
        // engine therefore uses its stable BLOB input slot instead; the single
        // de-stride copy into that slot is the last CPU step before the GPU.
        let camera_ahb = if input_ahb_off || direct_active {
            None
        } else {
            frame
                .ahb
                .as_ref()
                .map(|ahb| ahb.ptr() as *mut std::ffi::c_void)
        };
        // The `u16` samples are already little-endian bytes in memory; expose
        // them without a second widening pass. A camera AHB is bound directly
        // to MNN, so this slice is only the fallback upload source.
        let host_bytes = frame.data_bytes();
        let (staged, pooled_ahb_staged) = if camera_ahb.is_some() {
            (host_bytes, false)
        } else if uses_ahb {
            match input_ahb_slot
                .as_ref()
                .filter(|slot| slot.matches(frame.width, frame.height))
            {
                Some(slot) => match slot.stage(&frame) {
                    Ok(()) => (slot.bytes(), true),
                    Err(e) => {
                        log::warn!("input AHB staging failed: {e}; using host staging");
                        (host_bytes, false)
                    }
                },
                None => (host_bytes, false),
            }
        } else {
            (host_bytes, false)
        };

        let (data, width, height, kind, error) = if show_raw.load(Ordering::Relaxed) {
            prepared = None;
            (
                raw_to_grayscale_bgra(&frame),
                frame.width,
                frame.height,
                FrameKind::Bgra,
                None,
            )
        } else {
            let mut params = config.params(frame.width, frame.height, staged, frame.timestamp_ns);
            if let Some(ahb) = camera_ahb {
                // MediaNDK owns the frame AHB; `AhbRef` keeps it alive through
                // inference. `run_prepared` rebinds when this pointer changes.
                let capacity = frame.data.len().saturating_mul(2);
                params.input_ahb = Some(cam_types::AhbBinding::new(
                    ahb,
                    capacity,
                    cam_types::AhbFenceTokens::new(-1, -1, 0),
                ));
            } else if pooled_ahb_staged {
                if let Some(slot) = input_ahb_slot
                    .as_ref()
                    .filter(|slot| slot.matches(frame.width, frame.height))
                {
                    let (binding, buf) = slot.buffers();
                    params.input_buffer = Some(buf);
                    params.input_ahb = Some(binding);
                }
            }
            if let Some(slot) = ahb_slot.as_ref() {
                // Wait only for this worker's presenter handoff; the other
                // engine keeps running. Marking busy here prevents the GPU
                // from overwriting pixels the surface may still be reading.
                slot.wait_free();
                slot.mark_busy();
                if mnn.is_some() {
                    match slot.begin_worker_window() {
                        Ok(()) => {
                            let (binding, buf) = slot.buffers();
                            params.output_buffer = Some(buf);
                            params.output_ahb = Some(binding);
                        }
                        Err(e) => {
                            log::warn!("AHB output CPU window failed: {e}; using host copy");
                            slot.mark_free();
                            prepared = None;
                        }
                    }
                } else {
                    match slot.begin_gpu_window() {
                        Ok(()) => {
                            let (binding, _) = slot.buffers();
                            params.output_ahb = Some(binding);
                        }
                        Err(e) => {
                            log::warn!("AHB output GPU window failed: {e}; using host copy");
                            slot.mark_free();
                        }
                    }
                }
            }
            #[cfg(feature = "direct_vulkan")]
            if direct.is_some() && mnn.is_none() {
                match engine.process(&params) {
                    Ok(out) => {
                        let kind = if config.float_output() {
                            FrameKind::PlanarRgbF32
                        } else {
                            FrameKind::Bgra
                        };
                        let done = Done {
                            seq: s,
                            worker,
                            raw,
                            t_capture_ns: capture_t0.elapsed().as_nanos() as u64,
                            t_stage_ns: stage_t0.elapsed().as_nanos() as u64,
                            t_mnn_prepare_ns: 0,
                            t_mnn_inference_ns: out.inference_duration_ns,
                            t_mnn_output_ns: 0,
                            error: None,
                            data: out.data,
                            ahb: ahb_slot.as_ref().map(|slot| slot.view()),
                            width: out.width,
                            height: out.height,
                            kind,
                        };
                        let ok = tx.send(done).is_ok();
                        if !ok {
                            if let Some(slot) = ahb_slot.as_ref() {
                                slot.mark_free();
                            }
                        }
                        if !ok || stop.load(Ordering::Acquire) {
                            break;
                        }
                        continue;
                    }
                    Err(e) => {
                        if let Some(slot) = ahb_slot.as_ref() {
                            slot.mark_free();
                        }
                        let done = Done {
                            seq: s,
                            worker,
                            raw,
                            t_capture_ns: capture_t0.elapsed().as_nanos() as u64,
                            t_stage_ns: stage_t0.elapsed().as_nanos() as u64,
                            t_mnn_prepare_ns: 0,
                            t_mnn_inference_ns: 0,
                            t_mnn_output_ns: 0,
                            error: Some(e.to_string()),
                            data: Vec::new(),
                            ahb: None,
                            width: frame.width,
                            height: frame.height,
                            kind: FrameKind::Bgra,
                        };
                        let ok = tx.send(done).is_ok();
                        if !ok || stop.load(Ordering::Acquire) {
                            break;
                        }
                        continue;
                    }
                }
            }

            let expected = PrepareParams::from(&params);
            let staged_result = match (mnn, prepared.take()) {
                (Some(mnn), Some(prep)) if prep.prepare_params() == expected => {
                    Some(mnn.run_prepared(prep, &params))
                }
                (Some(mnn), Some(_)) => Some(mnn.process_staged(&params)),
                (Some(mnn), None) => Some(mnn.process_staged(&params)),
                (None, _) => None,
            };
            match staged_result {
                Some(Ok((out, pending))) => {
                    let kind = if config.float_output() {
                        FrameKind::PlanarRgbF32
                    } else {
                        FrameKind::Bgra
                    };
                    // Extract engine-internal phase timings from the returned frame.
                    let inf_ns = out.inference_duration_ns;
                    let out_ns = out
                        .aux
                        .as_ref()
                        .and_then(|a| a.timing)
                        .map(|t| t.output_meta_ns.saturating_add(t.output_copy_ns))
                        .unwrap_or(0);
                    let done = Done {
                        seq: s,
                        worker,
                        raw,
                        t_capture_ns: capture_t0.elapsed().as_nanos() as u64,
                        t_stage_ns: stage_t0.elapsed().as_nanos() as u64,
                        t_mnn_prepare_ns: 0, // engine-internal; see PreparedMeta.prep_setup_ns/prep_tensors_ns
                        t_mnn_inference_ns: inf_ns,
                        t_mnn_output_ns: out_ns,
                        error: None,
                        data: out.data,
                        ahb: ahb_slot.as_ref().map(|s| s.view()),
                        width: out.width,
                        height: out.height,
                        kind,
                    };
                    let ok = tx.send(done).is_ok();
                    if let Some(mnn) = mnn {
                        let _ = mnn.finish(pending);
                    }
                    if !ok {
                        if let Some(slot) = ahb_slot.as_ref() {
                            slot.mark_free();
                        }
                    }
                    if !ok || stop.load(Ordering::Acquire) {
                        break;
                    }
                    continue;
                }
                Some(Err(e)) => {
                    if let Some(slot) = ahb_slot.as_ref() {
                        slot.mark_free();
                    }
                    (
                        Vec::new(),
                        frame.width,
                        frame.height,
                        FrameKind::Bgra,
                        Some(e.to_string()),
                    )
                }
                None => {
                    let result = engine.process(&params);
                    match result {
                        Ok(out) => {
                            let kind = if config.float_output() {
                                FrameKind::PlanarRgbF32
                            } else {
                                FrameKind::Bgra
                            };
                            (out.data, out.width, out.height, kind, None)
                        }
                        Err(e) => (
                            Vec::new(),
                            frame.width,
                            frame.height,
                            FrameKind::Bgra,
                            Some(e.to_string()),
                        ),
                    }
                }
            }
        };

        let done = Done {
            seq: s,
            worker,
            raw,
            t_capture_ns: capture_t0.elapsed().as_nanos() as u64,
            t_stage_ns: stage_t0.elapsed().as_nanos() as u64,
            t_mnn_prepare_ns: 0,
            t_mnn_inference_ns: 0,
            t_mnn_output_ns: 0,
            error,
            data,
            ahb: None,
            width,
            height,
            kind,
        };
        if tx.send(done).is_err() || stop.load(Ordering::Acquire) {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cam_ipa::{BufferManager, ZeroPolicy};
    use cam_isp::controller::IspController;
    use cam_isp::engine::{IspEngine, ProcessParams};
    use cam_isp::error::{IspError, IspResult};
    use cam_isp::pipeline::{IspBlock, IspFrame};
    use cam_types::FrameFormat;
    use std::any::Any;
    use std::sync::atomic::{AtomicUsize, Ordering as AtomicOrdering};

    struct TrackedEngine {
        drops: Arc<AtomicUsize>,
        controller: Mutex<IspController>,
        output: Option<IspFrame>,
    }

    impl Drop for TrackedEngine {
        fn drop(&mut self) {
            self.drops.fetch_add(1, AtomicOrdering::SeqCst);
        }
    }

    impl IspEngine for TrackedEngine {
        fn backend_name(&self) -> &'static str {
            "tracked"
        }

        fn priority(&self) -> i32 {
            1
        }

        fn is_loaded(&self) -> bool {
            false
        }

        fn as_any(&self) -> &dyn Any {
            self
        }

        fn as_any_mut(&mut self) -> &mut dyn Any {
            self
        }

        fn build(
            &mut self,
            _pipeline_head: Box<dyn IspBlock>,
            _aux_blocks: Vec<Box<dyn IspBlock>>,
            _warp_block: Option<Box<dyn IspBlock>>,
            _opset_version: i64,
        ) -> IspResult<()> {
            Ok(())
        }

        fn controller(&self) -> &Mutex<IspController> {
            &self.controller
        }

        fn process(&self, _params: &ProcessParams<'_>) -> IspResult<IspFrame> {
            match &self.output {
                Some(frame) => Ok(frame.clone()),
                None => Err(IspError::Unsupported("tracked test engine".into())),
            }
        }
    }

    #[test]
    fn handle_cancel_is_idempotent_and_shared_by_clones() {
        let handle = PipelineHandle::new(Arc::new(AtomicBool::new(false)));
        let clone = handle.clone();
        assert!(!handle.is_cancelled());
        assert!(!clone.is_cancelled());
        handle.cancel();
        handle.cancel();
        clone.cancel();
        assert!(handle.is_cancelled());
        assert!(clone.is_cancelled());
    }

    #[test]
    fn cancel_joins_workers_and_releases_engine_and_camera() {
        let drops = Arc::new(AtomicUsize::new(0));
        let make_engine = || -> Box<dyn IspEngine> {
            Box::new(TrackedEngine {
                drops: Arc::clone(&drops),
                controller: Mutex::new(IspController::new()),
                output: None,
            })
        };
        let mut runner = PipelineRunner::new(
            None,
            [make_engine(), make_engine()],
            PipelineConfig {
                sensor_max: 1023.0,
                black_level: [0.0; 4],
                bayer_pattern: 0,
            },
            Arc::new(AtomicBool::new(false)),
        );
        let handle = runner.handle();
        assert!(!handle.is_cancelled());
        handle.cancel();
        handle.cancel();
        assert!(handle.is_cancelled());

        runner.cancel();
        assert!(runner.workers.is_empty());
        assert!(runner.pending.is_empty());
        assert!(runner.poll().is_none());
        assert!(!runner.has_camera());
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 2);
    }

    #[test]
    #[ignore = "on-device warm-up and cancel/restart check"]
    fn device_warmup_and_cancel_restart_no_stale_frame() {
        let mut manager = BufferManager::new();
        let warmup_t0 = Instant::now();
        let warm = manager.allocate_mem_with_policy(1600 * 1200 * 4, ZeroPolicy::FullOverwrite);
        let warmup = warmup_t0.elapsed();
        assert_eq!(warm.len(), 1600 * 1200 * 4);
        eprintln!("warm-up allocation {} bytes in {warmup:?}", warm.len());

        let drops = Arc::new(AtomicUsize::new(0));
        let config = PipelineConfig {
            sensor_max: 1023.0,
            black_level: [0.0; 4],
            bayer_pattern: 0,
        };
        let show_raw = Arc::new(AtomicBool::new(false));
        let make_engine = || -> Box<dyn IspEngine> {
            Box::new(TrackedEngine {
                drops: Arc::clone(&drops),
                controller: Mutex::new(IspController::new()),
                output: None,
            })
        };

        let mut first = PipelineRunner::new(
            None,
            [make_engine(), make_engine()],
            config,
            Arc::clone(&show_raw),
        );
        let handle = first.handle();
        handle.cancel();
        handle.cancel();
        first.cancel();
        assert!(first.poll().is_none());
        assert_eq!(drops.load(AtomicOrdering::SeqCst), 2);

        let mut frame = IspFrame::new(4, 4, FrameFormat::Rgba8888);
        frame.data.fill(0x7f);
        let params = ProcessParams::new(4, 4, &[0u8; 32]);
        let probe = TrackedEngine {
            drops: Arc::clone(&drops),
            controller: Mutex::new(IspController::new()),
            output: Some(frame),
        };
        let produced = probe.process(&params).expect("non-blank probe frame");
        assert!(produced.data.iter().any(|byte| *byte != 0));
        drop(probe);

        let mut restarted =
            PipelineRunner::new(None, [make_engine(), make_engine()], config, show_raw);
        assert!(!restarted.handle().is_cancelled());
        assert!(restarted.poll().is_none());
        restarted.pending.insert(
            0,
            Done {
                seq: 0,
                worker: 0,
                raw: RawStats {
                    min: 0,
                    max: 255,
                    mean: 127.0,
                },
                t_capture_ns: 0,
                t_stage_ns: 0,
                t_mnn_prepare_ns: 0,
                t_mnn_inference_ns: 0,
                t_mnn_output_ns: 0,
                error: None,
                data: produced.data.clone(),
                ahb: None,
                width: produced.width,
                height: produced.height,
                kind: FrameKind::Bgra,
            },
        );
        let presented = restarted.poll().expect("fresh frame after restart");
        assert_eq!(presented.seq, 0);
        assert!(presented.data.iter().any(|byte| *byte != 0));
        restarted.cancel();
        assert!(restarted.poll().is_none());
        assert!(drops.load(AtomicOrdering::SeqCst) >= 5);
    }

    #[test]
    fn pipeline_handle_registry_tracks_cancellation() {
        assert!(!crate::pipeline_cancel_requested());
        assert_eq!(crate::softisp_anw_pipeline_cancelled(), 0);
        crate::softisp_anw_cancel_pipeline();
        assert_eq!(crate::softisp_anw_pipeline_cancelled(), 0);

        let handle = PipelineHandle::new(Arc::new(AtomicBool::new(false)));
        crate::set_pipeline_handle(handle.clone());
        crate::softisp_anw_cancel_pipeline();
        crate::softisp_anw_cancel_pipeline();
        assert!(crate::pipeline_cancel_requested());
        assert_eq!(crate::softisp_anw_pipeline_cancelled(), 1);
        assert!(handle.is_cancelled());

        crate::clear_pipeline_handle();
        assert_eq!(crate::softisp_anw_pipeline_cancelled(), 0);
    }
}
