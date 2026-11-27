//! Standalone Vulkan compute engine.
//!
//! This `IspEngine` bypasses MNN. The worker supplies an input AHB containing
//! opaque little-endian RAW16 words and an output AHB for packed BGRA words.
//! Both are imported as Vulkan external-memory `VkBuffer`s and remain
//! GPU-owned for the whole dispatch: no host upload and no device→host copy.

use std::sync::{Arc, Mutex};
use std::time::Instant;

use cam_types::FrameFormat;
use log::info;

use crate::controller::IspController;
use crate::direct_vulkan_sys::DirectVulkanContext;
use crate::engine::{EngineFactory, IspEngine, ProcessParams};
use crate::error::{IspError, IspResult};
use crate::pipeline::types::IspFrame;
use crate::pipeline::IspBlock;

/// A Vulkan compute engine with no neural model and no MNN session.
pub struct DirectVulkanEngine {
    context: Option<DirectVulkanContext>,
    controller: Arc<Mutex<IspController>>,
    loaded: bool,
    mode: u32,
}

impl DirectVulkanEngine {
    pub fn new() -> Self {
        let mode = std::env::var("SOFTISP_DIRECT_VULKAN_MODE")
            .ok()
            .and_then(|v| v.parse::<u32>().ok())
            .unwrap_or(0);
        Self {
            context: DirectVulkanContext::new().ok(),
            controller: Arc::new(Mutex::new(IspController::new())),
            loaded: false,
            mode,
        }
    }

    pub fn share_controller(&mut self, controller: &Arc<Mutex<IspController>>) {
        self.controller = Arc::clone(controller);
    }

    pub fn register_factories() {
        crate::engine::register_engine(EngineFactory {
            name: "direct_vulkan_a",
            priority: 98,
            create_fn: Box::new(|| Box::new(DirectVulkanEngine::new())),
        });
        crate::engine::register_engine(EngineFactory {
            name: "direct_vulkan_b",
            priority: 97,
            create_fn: Box::new(|| Box::new(DirectVulkanEngine::new())),
        });
    }

    fn context(&self) -> IspResult<&DirectVulkanContext> {
        self.context
            .as_ref()
            .ok_or_else(|| IspError::Pipeline("direct Vulkan context was not created".to_string()))
    }
}

impl Default for DirectVulkanEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl IspEngine for DirectVulkanEngine {
    fn backend_name(&self) -> &'static str {
        "direct_vulkan"
    }

    fn priority(&self) -> i32 {
        98
    }

    fn is_loaded(&self) -> bool {
        self.loaded
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    fn build(
        &mut self,
        _pipeline_head: Box<dyn IspBlock>,
        _aux_blocks: Vec<Box<dyn IspBlock>>,
        _warp_block: Option<Box<dyn IspBlock>>,
        _opset_version: i64,
    ) -> IspResult<()> {
        if self.context.is_none() {
            self.context = Some(DirectVulkanContext::new().map_err(IspError::Pipeline)?);
        }
        self.loaded = true;
        info!("DirectVulkanEngine ready (mode={})", self.mode);
        Ok(())
    }

    fn controller(&self) -> &Mutex<IspController> {
        &self.controller
    }

    fn process(&self, p: &ProcessParams<'_>) -> IspResult<IspFrame> {
        if !self.loaded {
            return Err(IspError::Config(
                "DirectVulkanEngine::process called before build()".to_string(),
            ));
        }
        let (Some(input), Some(output)) = (p.input_ahb, p.output_ahb) else {
            return Err(IspError::InvalidInput(
                "DirectVulkanEngine requires input and output AHardwareBuffers".to_string(),
            ));
        };
        let ctx = self.context()?;
        let t0 = Instant::now();
        ctx.run_ahb(
            input.ptr,
            input.len,
            output.ptr,
            output.len,
            p.width.max(1),
            p.height.max(1),
            self.mode,
        )
        .map_err(IspError::Pipeline)?;

        let mut frame =
            IspFrame::new_external(p.width.max(1), p.height.max(1), FrameFormat::Rgba8888);
        frame.inference_duration_ns = t0.elapsed().as_nanos() as u64;
        frame.timestamp_ns = p.timestamp_ns;
        Ok(frame)
    }
}
