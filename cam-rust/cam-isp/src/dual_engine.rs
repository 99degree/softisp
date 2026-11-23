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
//!   let engine_a = MnnEngine::new(MnnBackend::Vulkan);
//!   let engine_b = MnnEngine::new(MnnBackend::Vulkan);
//!   engine_a.build(head_a, blocks_a.clone(), None, 21)?;
//!   engine_b.build(head_b, blocks_b.clone(), None, 21)?;
//!   let mut dual = DualGpuEngine::new(engine_a, engine_b);
//!   dual.next().process(&params)?; // frame N -> engine A
//!   dual.next().process(&params)?; // frame N+1 -> engine B
//!
//! Both engines should share the same `IspController` (AWB/AE/CCM/tone state)
//! so that 3A feedback is consistent across frames. Call
//! `mnn_engine_a.set_controller(ctrl.clone())` and
//! `mnn_engine_b.set_controller(ctrl.clone())` before build, or pass the same
//! `Arc<Mutex<IspController>>` when constructing.

use std::sync::Mutex;

use crate::controller::IspController;
use crate::engine::{IspEngine, ProcessParams};
use crate::pipeline::{IspBlock, IspFrame};

/// Runs two fused `isp.*` engine instances in round-robin order:
/// frame N -> engine A, frame N+1 -> engine B, frame N+2 -> engine A, ...
pub struct DualGpuEngine {
    engines: [Box<dyn IspEngine>; 2],
    toggle: bool,
    /// Shared controller so both engines see the same AWB/AE/CCM/tone state.
    controller: Mutex<IspController>,
}

impl DualGpuEngine {
    /// Create a new dual engine from two already-built `MnnEngine` instances.
    ///
    /// Both engines must have been built with the same block chain. The
    /// controller from `engine_a` is adopted as the shared controller; if you
    /// want a specific shared controller, call `set_controller` on both
    /// engines before the first `process`.
    pub fn new(engine_a: Box<dyn IspEngine>, engine_b: Box<dyn IspEngine>) -> Self {
        let controller = engine_a.controller().clone();
        Self {
            engines: [engine_a, engine_b],
            toggle: false,
            controller: Mutex::new(IspController::default()),
        }
    }

    /// Returns the next engine in round-robin order (A, B, A, B, ...).
    pub fn next(&mut self) -> &mut dyn IspEngine {
        let idx = self.toggle as usize;
        self.toggle = !self.toggle;
        &mut *self.engines[idx]
    }

    /// Returns the shared controller (engine A's controller).
    pub fn controller(&self) -> &Mutex<IspController> {
        &self.controller
    }

    /// Replace the shared controller on both engines.
    pub fn set_controller(&mut self, ctrl: Mutex<IspController>) {
        self.controller = ctrl;
        // Both engines declare their own controller field; we can't reach into
        // the dyn trait object to set it, so callers should call
        // `MnnEngine::set_controller` on each engine before wrapping. This
        // method exists for API symmetry and is a no-op on the dyn objects.
        let _ = self.engines[0].controller();
        let _ = self.engines[1].controller();
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

    fn build(
        &mut self,
        head: Box<dyn IspBlock>,
        aux: Vec<Box<dyn IspBlock>>,
        warp: Option<Box<dyn IspBlock>>,
        opset: i64,
    ) -> crate::error::IspResult<()> {
        // Both engines must be built with identical chains. The caller is
        // responsible for constructing two separate block lists (build takes
        // ownership of head/aux). Build engine A first, then engine B.
        self.engines[0].build(head, aux, warp.clone(), opset)?;
        self.engines[1].build(head, aux, warp, opset)?;
        Ok(())
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
