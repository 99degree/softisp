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

/// One session together with its cached extra-input tensor handles.
#[cfg(feature = "mnn")]
pub(crate) struct SessionSlot {
    pub(crate) sess: MnnSessionSafe,
    pub(crate) tensor_pool: Vec<(String, MnnTensorSafe)>,
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
    /// Build a pool of `n` sessions sharing `interp`.
    pub(crate) fn new(
        interp: MnnInterpreterSafe,
        backend: MnnBackendType,
        n: usize,
        num_threads: i32,
    ) -> Result<Self, String> {
        let extra_names = [
            "DemosaicCcmBlock/w",
            "DemosaicCcmBlock/b",
            "BayerWbBlock/gains",
            "ToneBlock/contrast",
            "ToneBlock/brightness",
            "ToneBlock/gamma_recip",
            "saturation/scale",
            "Sharpen/strength",
            "LdciBlock/strength",
            "FcsBlock/gain",
            "FcsBlock/bias",
            "NormalizeBlock/max_val",
            "Gamma/inv_gamma",
            "Gamma/min",
            "Gamma/max",
            "Gamma/lift",
            "Gamma/norm",
            "AutoContrast/lift",
            "AutoContrast/half",
            "AutoContrast/contrast_w",
            "AutoContrast/zero",
            "AutoContrast/one",
            "DisplayBlock/scale",
            "DisplayBlock/gamma_exp",
            "DisplayBlock/zero",
            "DisplayBlock/one",
        ];
        let tensor_names: Vec<String> = extra_names.iter().map(|s| s.to_string()).collect();
        let mut slots = VecDeque::with_capacity(n);
        for _ in 0..n {
            let sess = interp
                .create_session(backend, num_threads)
                .ok_or_else(|| format!("create session {} failed", slots.len()))?;
            let mut tensor_pool = Vec::new();
            for name in &tensor_names {
                if let Some(t) = interp.get_input(&sess, name) {
                    tensor_pool.push((name.clone(), t));
                }
            }
            slots.push_back(SessionSlot { sess, tensor_pool });
        }
        info!(
            "SessionPool: {} slots created with {} extra tensors each",
            n,
            slots[0].tensor_pool.len()
        );
        Ok(Self {
            interp,
            inner: Mutex::new(slots),
            cond: Condvar::new(),
            tensor_names,
        })
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

#[cfg(feature = "mnn")]
impl<'a> Drop for SessionGuard<'a> {
    fn drop(&mut self) {
        if let Some(slot) = self.slot.take() {
            self.pool.release(slot);
        }
    }
}
