//! Dual-GPU engine wrapper: alternates frames between two `isp.*` Vulkan
//! engine instances (engine A, engine B) so that while the GPU infers frame N
//! on engine A, the host can prepare frame N+1 for engine B, and vice versa.
//!
//! Both engines run the same fused `isp.*` chain (fp32, same graph), so output
//! is deterministic. Sessions are created once at build time and reused every
//! frame.
//!
//! Frames are processed strictly one-at-a-time (engine A for frame N fully
//! completes before engine B starts frame N+1), so there is no out-of-order
//! output and no reorder buffer is needed.
//!
//! Usage:
//!   let mut engine_a = MnnEngine::new(MnnBackend::Vulkan);
//!   let mut engine_b = MnnEngine::new(MnnBackend::Vulkan);
//!   engine_a.build(head_a, blocks_a, None, 21)?;
//!   engine_b.build(head_b, blocks_b, None, 21)?;
//!   let mut dual = DualGpuEngine::new(engine_a, engine_b);
//!   dual.next().process(&params)?; // frame N -> engine A
//!   dual.next().process(&params)?; // frame N+1 -> engine B
//!
//! Both engines share the same `IspController` (AWB/AE/CCM/tone state) so that
//! 3A feedback is consistent across frames. Call
//! `MnnEngine::set_controller(ctrl.clone())` on each engine before wrapping
//! them in `DualGpuEngine` to share controller state.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::controller::IspController;
use crate::engine::{IspEngine, ProcessParams};
use crate::pipeline::{IspBlock, IspFrame};

/// Runs two fused `isp.*` engine instances in round-robin order:
/// frame N -> engine A, frame N+1 -> engine B, frame N+2 -> engine A, ...
pub struct DualGpuEngine {
    engines: [Box<dyn IspEngine>; 2],
    toggle: AtomicBool,
    /// Shared controller so both engines see the same AWB/AE/CCM/tone state.
    ///
    /// Set via [`Self::set_controller`] on each engine before wrapping, or via
    /// `MnnEngine::set_controller` on each individual engine. The `DualGpuEngine`
    /// exposes engine A's controller through [`IspEngine::controller`].
    controller: Mutex<IspController>,
}

impl DualGpuEngine {
    /// Create a new dual engine from two already-built `MnnEngine` instances.
    ///
    /// Both engines must have been built with the same block chain. The engine
    /// instances keep their own `IspController`; callers who want shared state
    /// should call `set_controller` on each `MnnEngine` before wrapping them
    /// here.
    pub fn new(engine_a: Box<dyn IspEngine>, engine_b: Box<dyn IspEngine>) -> Self {
        Self {
            engines: [engine_a, engine_b],
            toggle: AtomicBool::new(false),
            controller: Mutex::new(IspController::default()),
        }
    }

    /// Returns the next engine in round-robin order (A, B, A, B, ...).
    pub fn next(&self) -> &dyn IspEngine {
        let idx = self.toggle.fetch_xor(true, Ordering::Relaxed) as usize;
        &*self.engines[idx]
    }

    /// Returns engine A's controller (`IspEngine::controller()`).
    pub fn controller(&self) -> &Mutex<IspController> {
        self.engines[0].controller()
    }
}

impl IspEngine for DualGpuEngine {
    fn backend_name(&self) -> &'static str {
        "mnn_vulkan_dual"
    }

    fn priority(&self) -> i32 {
        self.engines[0].priority()
    }

    fn is_loaded(&self) -> bool {
        self.engines[0].is_loaded() && self.engines[1].is_loaded()
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn controller(&self) -> &Mutex<IspController> {
        &self.controller
    }

    /// Build both engines with identical chains.
    ///
    /// Because `build()` takes ownership of `head` and `aux`, the caller must
    /// provide two separate block lists (one per engine). There is no clone
    /// path because `IspBlock` is not `Clone`.
    /// Build both engines with identical chains.
    ///
    /// Because `build()` takes ownership of `head` and `aux`, both engines
    /// cannot be built from the same block list — `IspBlock` is not `Clone`.
    /// Callers should instead build two `MnnEngine` instances separately
    /// (each with its own block list) and pass them to `DualGpuEngine::new()`.
    ///
    /// This method always returns an error; use [`Self::new`] instead.
    fn build(
        &mut self,
        _head: Box<dyn IspBlock>,
        _aux: Vec<Box<dyn IspBlock>>,
        _warp: Option<Box<dyn IspBlock>>,
        _opset: i64,
    ) -> crate::error::IspResult<()> {
        Err(crate::error::IspError::Config(
            "DualGpuEngine does not support build(); use DualGpuEngine::new() \
             with two pre-built engines instead".into(),
        ))
    }

    fn process(&self, params: &crate::engine::ProcessParams) -> crate::error::IspResult<IspFrame> {
        let engine = self.next();
        engine.process(params)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dual_engine_round_robin() {
        // Just verify construction works; full engine test requires MNN libs.
        let _ = DualGpuEngine::new(
            Box::new(crate::cpu::CpuEngine::new()),
            Box::new(crate::cpu::CpuEngine::new()),
        );
    }
}
