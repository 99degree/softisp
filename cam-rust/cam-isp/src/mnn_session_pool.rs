//! MNN Session Pool — parallel inference with shared interpreter.
//!
//! A pool of N sessions sharing one ``MnnInterpreterSafe``.
//! Threads acquire a slot (blocking), use it, release it back.
//!
//! # Field Drop Order
//! ``slots`` (sessions) are dropped BEFORE ``interp`` because session
//! release needs the interpreter. This is guaranteed by declaration order:
//! ``slots`` is declared first, Rust drops fields top-to-bottom.

use log::info;
use std::collections::VecDeque;
use std::sync::{Condvar, Mutex};

use crate::mnn_sys::{MnnBackendType, MnnInterpreterSafe, MnnSessionSafe, MnnTensorSafe};
use crate::upload_cache::UploadCache;

/// One session together with its cached extra-input tensor handles.
#[cfg(feature = "mnn")]
pub(crate) struct SessionSlot {
    pub(crate) sess: MnnSessionSafe,
    pub(crate) tensor_pool: Vec<(String, MnnTensorSafe)>,
    /// Last value successfully uploaded to each tensor in this session.
    /// Cleared whenever `resizeSession` replaces the backend buffers.
    pub(crate) upload_cache: UploadCache,
    /// Input geometry this session's graph is currently sized for, if any.
    ///
    /// MNN invalidates a whole session whenever an input tensor's extent
    /// changes (`Interpreter::resizeTensor` → `Session::setNeedResize`), and
    /// `Session::run()` then refuses to execute — it returns
    /// COMPUTE_SIZE_ERROR — until the session is resized again. Because the
    /// pool hands out sessions round-robin, this has to be tracked per slot:
    /// a single engine-wide "last shape" cannot tell whether *this* session
    /// still needs resizing.
    pub(crate) resized_for: Option<(u32, u32)>,
}

/// A pool of N sessions sharing one interpreter.
///
/// ⚠ Fields are dropped in declaration order. `slots` (sessions) must be
/// dropped BEFORE `interp` because session release needs the interpreter.
#[cfg(feature = "mnn")]
pub(crate) struct SessionPool {
    /// Queue of available slots (dropped first — sessions before interpreter).
    inner: Mutex<VecDeque<SessionSlot>>,
    /// Wakes a blocked `acquire()` when a slot is returned.
    cond: Condvar,
    /// Shared interpreter (owns the model). Dropped last.
    pub(crate) interp: MnnInterpreterSafe,
    /// Cached tensor names used during pool construction.
    tensor_names: Vec<String>,
}

#[cfg(feature = "mnn")]
impl SessionPool {
    /// Build a pool of up to `n` sessions sharing `interp`.
    /// `extra_names` are the non-frame input tensors (runtime-fed weights
    /// + constants) that the engine will write into before each inference.
    ///
    /// The session count is a memory budget. Every session allocates its own
    /// Vulkan buffers for the whole graph, so at 5 Mpx four of them can need
    /// several hundred MB — more than a phone will hand out. On device that
    /// does not even surface as an MNN error: the process is killed while the
    /// second session is being built, so the log stops mid-init with no panic.
    ///
    /// This therefore halves the count on failure and retries, down to one
    /// session. A single-session preview is slower but runs; a four-session one
    /// that cannot be allocated takes the app with it.
    ///
    /// Returns `Err` only when even one session cannot be created.
    pub(crate) fn new(
        interp: MnnInterpreterSafe,
        backend: MnnBackendType,
        n: usize,
        num_threads: i32,
        fp16_storage: bool,
        extra_names: Vec<String>,
    ) -> Result<Self, String> {
        let tensor_names = extra_names;
        let mut wanted = n.max(1);
        loop {
            let mut slots = VecDeque::with_capacity(wanted);
            let mut failed = false;
            for i in 0..wanted {
                let Some(sess) = interp.create_session(backend, num_threads, fp16_storage) else {
                    log::warn!(
                        "SessionPool: session {} of {} failed to allocate",
                        i + 1,
                        wanted
                    );
                    failed = true;
                    break;
                };
                let mut tensor_pool = Vec::new();
                for name in &tensor_names {
                    if let Some(t) = interp.get_input(&sess, name) {
                        tensor_pool.push((name.clone(), t));
                    }
                }
                slots.push_back(SessionSlot {
                    sess,
                    tensor_pool,
                    upload_cache: UploadCache::default(),
                    resized_for: None,
                });
            }

            if !failed {
                info!(
                    "SessionPool: {} slots created with {} extra tensors each",
                    slots.len(),
                    slots[0].tensor_pool.len()
                );
                return Ok(Self {
                    interp,
                    inner: Mutex::new(slots),
                    cond: Condvar::new(),
                    tensor_names,
                });
            }

            // Drop the partial pool before retrying: those sessions hold the
            // very GPU memory the retry needs.
            drop(slots);
            if wanted == 1 {
                return Err("could not allocate even one MNN session".to_string());
            }
            let next = (wanted / 2).max(1);
            log::warn!("SessionPool: retrying with {next} session(s) instead of {wanted}");
            wanted = next;
        }
    }

    /// Acquire a slot (blocks until one is free).
    pub(crate) fn acquire(&self) -> SessionGuard<'_> {
        let mut slots = self.inner.lock().unwrap();
        while slots.is_empty() {
            slots = self.cond.wait(slots).unwrap();
        }
        SessionGuard {
            pool: self,
            slot: Some(slots.pop_front().unwrap()),
        }
    }

    /// Return a slot to the pool.
    fn release(&self, slot: SessionSlot) {
        let mut slots = self.inner.lock().unwrap();
        slots.push_back(slot);
        self.cond.notify_one();
    }

    /// Number of sessions actually created.
    ///
    /// Can be lower than the requested count: [`Self::new`] halves it when an
    /// allocation fails, and the engine records this so its own logging and
    /// any later resize decisions match reality.
    pub(crate) fn len(&self) -> usize {
        match self.inner.lock() {
            Ok(g) => g.len(),
            Err(e) => e.into_inner().len(),
        }
    }

    /// Return the number of tensor names tracked by this pool.
    #[allow(dead_code)]
    pub(crate) fn tensor_count(&self) -> usize {
        self.tensor_names.len()
    }
}

/// RAII guard: automatically returns the slot on drop.
#[cfg(feature = "mnn")]
pub(crate) struct SessionGuard<'a> {
    pool: &'a SessionPool,
    slot: Option<SessionSlot>,
}

#[cfg(feature = "mnn")]
impl<'a> std::ops::Deref for SessionGuard<'a> {
    type Target = SessionSlot;
    fn deref(&self) -> &Self::Target {
        self.slot.as_ref().unwrap()
    }
}

#[cfg(feature = "mnn")]
impl<'a> std::ops::DerefMut for SessionGuard<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.slot.as_mut().unwrap()
    }
}

impl<'a> SessionGuard<'a> {
    /// Shared interpreter behind this slot, for staged stats readback.
    pub(crate) fn interp(&self) -> &MnnInterpreterSafe {
        &self.pool.interp
    }

    /// Tensor handles and their per-session upload cache, borrowed together.
    pub(crate) fn extra_inputs(&mut self) -> (&[(String, MnnTensorSafe)], &mut UploadCache) {
        let slot = self.slot.as_mut().unwrap();
        (&slot.tensor_pool, &mut slot.upload_cache)
    }

    /// Re-resolve the cached extra-input tensor handles from the session.
    ///
    /// `resizeSession` re-allocates the session's tensors, so every handle
    /// cached at pool construction points at freed memory. Writing the runtime
    /// weights through those stale handles silently did nothing — the values
    /// stayed at whatever the model shipped with, and for the fused CCM that
    /// meant an all-zero matrix and a black frame. Call this after any resize.
    pub(crate) fn refresh_tensors(&mut self, interp: &MnnInterpreterSafe) {
        let slot = self.slot.as_mut().unwrap();
        let names: Vec<String> = slot.tensor_pool.iter().map(|(n, _)| n.clone()).collect();
        let refreshed: Vec<(String, MnnTensorSafe)> = names
            .into_iter()
            .filter_map(|n| interp.get_input(&slot.sess, &n).map(|t| (n, t)))
            .collect();
        slot.tensor_pool = refreshed;
        slot.upload_cache.clear();
    }
}

#[cfg(feature = "mnn")]
impl<'a> Drop for SessionGuard<'a> {
    fn drop(&mut self) {
        if let Some(slot) = self.slot.take() {
            self.pool.release(slot);
        }
    }
}
