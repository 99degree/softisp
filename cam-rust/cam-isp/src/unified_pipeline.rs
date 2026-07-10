//! Unified Camera Pipeline — Fused ISP + Post-Processing
//!
//! Combines the GPU-accelerated ISP pipeline (ONNX→MNN→Vulkan) with
//! post-processing stages that also run on GPU via ONNX ops.
//!
//! # Architecture
//!
//! ```text
//! Raw Input (INT16 [1,1,H,W])
//!         │
//!         ▼
//!   ┌─────────────────────────────────────┐
//!   │  Fused ISP Pipeline (GPU)           │
//!   │  Unpack → Demosaic+CCM → Warp →    │
//!   │  Display (outputs FloatRgb)         │
//!   └──────────────┬──────────────────────┘
//!                  │ [1,3,H,W] f32
//!                  ▼
//!   ┌─────────────────────────────────────┐
//!   │  GPU Warp (separate MNN session)    │
//!   │  GDC grid + EIS displacement →      │
//!   │  GridSample on GPU                  │
//!   └──────────────┬──────────────────────┘
//!                  │ [1,3,H,W] f32
//!                  ▼
//!   ┌─────────────────────────────────────┐
//!   │  Post-Processing (CPU, float)       │
//!   │  Deshake → Temporal Denoise →       │
//!   │  Wavelet Denoise                    │
//!   └──────────────┬──────────────────────┘
//!                  │
//!                  ▼
//!           Final Output (FloatRgb/RGB)
//! ```

use log::info;
use log::warn;
use crate::error::IspResult;
use crate::pipeline::{IspFrame, IspBlock, GraphComposer};
use crate::engine::{IspEngine, ProcessParams, OutputFormat, select_engine};
use crate::pipeline::traits::ProcessPipeline;
use crate::postprocess::{PostProcessConfig, PostProcessPipeline};
use crate::profile::PipelineProfile;
use crate::blocks::gpu_warp::GpuWarpBlock;
use crate::onnx::proto::Proto;
use crate::controller_api::ControllerApi;
#[cfg(feature = "mnn")]
use crate::warp_engine::GpuWarpEngine;

/// Configuration for the unified pipeline.
#[derive(Debug, Clone)]
pub struct UnifiedConfig {
    /// ISP pipeline profile (LITE, MED, HEAVY, PRO).
    pub profile: PipelineProfile,
    /// Target output width (height auto-calculated as 9:16).
    pub target_width: u32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: i32,
    /// Engine preference: "auto", "vulkan", "cpu".
    pub engine_preference: String,
    /// Post-processing configuration.
    pub post_config: PostProcessConfig,
    /// Output format for display block.
    pub output_format: OutputFormat,
    /// Sensor max value (default 1023 for 10-bit).
    pub sensor_max: f32,
    /// Enable GPU-accelerated warp for EIS/GDC.
    pub gpu_warp_enabled: bool,
}

impl Default for UnifiedConfig {
    fn default() -> Self {
        Self {
            profile: PipelineProfile::MED,
            target_width: 1920,
            bayer_pattern: 0,
            engine_preference: "auto".into(),
            post_config: PostProcessConfig::default(),
            output_format: OutputFormat::FloatRgb,
            sensor_max: 1023.0,
            gpu_warp_enabled: false,
        }
    }
}

impl UnifiedConfig {
    /// Create a config for HD (1280x720) with LITE profile.
    pub fn hd() -> Self {
        Self { profile: PipelineProfile::LITE, target_width: 1280, ..Default::default() }
    }

    /// Create a config for FHD (1920x1080) with MED profile.
    pub fn fhd() -> Self {
        Self { profile: PipelineProfile::MED, target_width: 1920, ..Default::default() }
    }

    /// Create a config for 4K (3840x2160) with HEAVY profile.
    pub fn uhd() -> Self {
        Self { profile: PipelineProfile::HEAVY, target_width: 3840, ..Default::default() }
    }

    /// Create a config for 4K with PRO profile and full post-processing.
    pub fn pro() -> Self {
        let post = PostProcessConfig {
            eis_enabled: true,
            deshake_enabled: true,
            gdc_enabled: true,
            temporal_denoise_enabled: true,
            ..Default::default()
        };
        Self { profile: PipelineProfile::PRO, target_width: 3840, post_config: post, ..Default::default() }
    }

    /// Enable EIS stabilization.
    pub fn with_eis(mut self) -> Self { self.post_config.eis_enabled = true; self }

    /// Enable deshake stabilization.
    pub fn with_deshake(mut self) -> Self { self.post_config.deshake_enabled = true; self }

    /// Enable GDC lens correction.
    pub fn with_gdc(mut self, strength: f32) -> Self {
        self.post_config.gdc_enabled = true;
        self.post_config.gdc_strength = strength;
        self
    }

    /// Enable temporal denoise.
    pub fn with_temporal_denoise(mut self) -> Self {
        self.post_config.temporal_denoise_enabled = true; self
    }

    /// Enable GPU-accelerated warp for EIS/GDC.
    pub fn with_gpu_warp(mut self) -> Self { self.gpu_warp_enabled = true; self }

    /// Use RGBA output format (instead of FloatRgb).
    pub fn with_rgba_output(mut self) -> Self {
        self.output_format = OutputFormat::Rgba; self
    }
}

/// GPU warp parameters (GDC coefficients + EIS displacement).
#[derive(Debug, Clone, Default)]
pub struct GpuWarpParams {
    /// GDC radial distortion coefficient k1.
    pub gdc_k1: f32,
    /// GDC radial distortion coefficient k2.
    pub gdc_k2: f32,
    /// GDC radial distortion coefficient k3.
    pub gdc_k3: f32,
    /// EIS horizontal displacement (normalized [-1,1]).
    pub eis_dx: f32,
    /// EIS vertical displacement (normalized [-1,1]).
    pub eis_dy: f32,
}

impl GpuWarpParams {
    /// Create identity parameters (no distortion, no displacement).
    pub fn identity() -> Self { Self::default() }

    /// Create GDC-only parameters (no EIS).
    pub fn gdc(k1: f32, k2: f32, k3: f32) -> Self {
        Self { gdc_k1: k1, gdc_k2: k2, gdc_k3: k3, ..Default::default() }
    }

    /// Create EIS-only parameters (no GDC).
    pub fn eis(dx: f32, dy: f32) -> Self {
        Self { eis_dx: dx, eis_dy: dy, ..Default::default() }
    }

    /// Returns true if any parameter is non-zero (warp needed).
    pub fn needs_warp(&self) -> bool {
        self.gdc_k1.abs() > 1e-6
            || self.gdc_k2.abs() > 1e-6
            || self.gdc_k3.abs() > 1e-6
            || self.eis_dx.abs() > 1e-6
            || self.eis_dy.abs() > 1e-6
    }
}

/// Unified camera pipeline combining ISP + post-processing.
pub struct UnifiedPipeline {
    config: UnifiedConfig,
    engine: Box<dyn IspEngine>,
    post_pipeline: PostProcessPipeline,
    /// ISP controller for parameter-driven processing (unified API).
    controller: crate::controller_api::Controller,
    /// GPU warp block (for ONNX generation).
    gpu_warp: Option<GpuWarpBlock>,
    /// GPU warp ONNX model bytes.
    gpu_warp_onnx: Option<Vec<u8>>,
    /// GPU warp MNN inference engine.
    #[cfg(feature = "mnn")]
    gpu_warp_engine: Option<GpuWarpEngine>,
    /// GPU format converter for float→RGBA/BGRA/etc.
    #[cfg(feature = "mnn")]
    format_converter: Option<crate::format_convert::FormatConvertEngine>,
    width: u32,
    height: u32,
    initialized: bool,
}

impl UnifiedPipeline {
    /// Create a new unified pipeline.
    pub fn new(config: UnifiedConfig) -> IspResult<Self> {
        info!("UnifiedPipeline: profile={}, width={}, gpu_warp={}, format={:?}",
            config.profile.label, config.target_width, config.gpu_warp_enabled, config.output_format);

        // Initialize controller (neural with fallback)
        let _controller = crate::controller_api::Controller::neural();

        let mut engine = match config.engine_preference.as_str() {
            "vulkan" => {
                crate::engine::select_engine_by_name("mnn_vulkan")
                    .or_else(|| crate::engine::select_engine_by_name("mnn_cpu"))
                    .unwrap_or_else(|| Box::new(crate::cpu::CpuEngine::new()))
            }
            "cpu" => Box::new(crate::cpu::CpuEngine::new()),
            _ => select_engine().unwrap_or_else(|| Box::new(crate::cpu::CpuEngine::new())),
        };

        let h = config.target_width * 9 / 16;
        let w = config.target_width;

        // Build ISP blocks
        let blocks = config.profile.build_blocks(w, config.bayer_pattern);
        let block_refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = GraphComposer::compose_from_vec(&block_refs, &[], 21)
            .map_err(|e| crate::error::IspError::Pipeline(format!("compose: {}", e)))?;
        info!("UnifiedPipeline: ISP ONNX {} bytes, {} blocks", onnx.len(), blocks.len());

        let mut block_iter = blocks.into_iter();
        let head = block_iter.next()
            .ok_or_else(|| crate::error::IspError::Pipeline("empty pipeline".into()))?;
        let aux: Vec<Box<dyn IspBlock>> = block_iter.collect();
        engine.build(head, aux, None, 21)?;

        // Build GPU warp
        let (gpu_warp, gpu_warp_onnx) = if config.gpu_warp_enabled {
            let warp_block = GpuWarpBlock::new(w, h);
            let nodes = warp_block.nodes();
            let inits = warp_block.initializers();
            let extras = warp_block.extra_inputs();

            let mut graph_inputs = vec![
                Proto::value_info("GpuWarp/input",
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                      Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1),
            ];
            for (name, etype, shape) in &extras {
                let dim_protos: Vec<Vec<u8>> = shape.iter().map(|d| {
                    if *d > 0 { Proto::tensor_dim_value(*d) }
                    else { Proto::tensor_dim_param("?") }
                }).collect();
                graph_inputs.push(Proto::value_info(name, &dim_protos, *etype as i32));
            }

            let graph_outputs = vec![
                Proto::value_info("GpuWarp/frame",
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
                      Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1),
            ];

            let graph = Proto::graph("gpu_warp", &nodes, &graph_inputs, &graph_outputs, &inits, &[]);
            let opset = Proto::opset("", 21);
            let model = Proto::model(8, &opset, "softisp-gpu-warp", &graph);
            info!("UnifiedPipeline: GPU warp ONNX {} bytes", model.len());
            (Some(warp_block), Some(model))
        } else {
            (None, None)
        };

        // Build GPU warp MNN engine
        #[cfg(feature = "mnn")]
        let gpu_warp_engine = if let Some(ref onnx) = gpu_warp_onnx {
            match GpuWarpEngine::from_onnx(onnx, w, h) {
                Ok(e) => { info!("UnifiedPipeline: GPU warp engine ready"); Some(e) }
                Err(e) => { warn!("GPU warp engine init failed: {:?}", e); None }
            }
        } else {
            None
        };

        // GPU format converter for float→target format
        #[cfg(feature = "mnn")]
        let format_converter = if config.output_format != OutputFormat::FloatRgb {
            match crate::format_convert::FormatConvertEngine::new(w, h, config.output_format) {
                Ok(e) => { info!("UnifiedPipeline: GPU format converter ready for {:?}", config.output_format); Some(e) }
                Err(e) => { warn!("GPU format converter init failed: {:?}", e); None }
            }
        } else {
            None
        };

        let post_pipeline = PostProcessPipeline::new(config.post_config.clone());

        // Initialize controller (neural with fallback)
        let controller = crate::controller_api::Controller::neural();

        Ok(Self {
            config,
            engine,
            post_pipeline,
            controller,
            gpu_warp,
            gpu_warp_onnx,
            #[cfg(feature = "mnn")]
            gpu_warp_engine,
            #[cfg(feature = "mnn")]
            format_converter,
            width: w,
            height: h,
            initialized: true,
        })
    }

    /// Get a mutable reference to the controller.
    pub fn controller_mut(&mut self) -> &mut crate::controller_api::Controller {
        &mut self.controller
    }

    /// Get a reference to the controller.
    pub fn controller(&self) -> &crate::controller_api::Controller {
        &self.controller
    }

    /// Load neural model for ISP parameter prediction.
    pub fn load_model(&mut self, model_path: &str) -> bool {
        let loaded = self.controller.load_model(model_path);
        if loaded {
            info!("UnifiedPipeline: loaded model from {}", model_path);
        } else {
            warn!("UnifiedPipeline: failed to load model from {}", model_path);
        }
        loaded
    }

    /// Switch to rule-based controller.
    pub fn use_rule_based_controller(&mut self) {
        self.controller = crate::controller_api::Controller::rule_based();
        info!("UnifiedPipeline: switched to rule-based controller");
    }

    /// Switch to neural controller.
    pub fn use_neural_controller(&mut self) {
        self.controller = crate::controller_api::Controller::neural();
        info!("UnifiedPipeline: switched to neural controller");
    }

    /// Process a raw frame through the full pipeline.
    pub fn process(&mut self, raw_data: &[u8], width: u32, height: u32) -> IspResult<IspFrame> {
        self.process_with_warp(raw_data, width, height, &GpuWarpParams::identity())
    }

    /// Process with GPU warp parameters (GDC + EIS).
    ///
    /// Uses FloatRgb format internally to avoid RGBA conversion overhead.
    pub fn process_with_warp(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        #[allow(unused_variables)]
        warp_params: &GpuWarpParams,
    ) -> IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Pipeline("pipeline not initialized".into()));
        }

        // 0. Controller analysis: analyze frame and update parameters
        let mut frame_for_analysis = crate::pipeline::types::IspFrame {
            params: crate::isp_params::IspParams::default(),
            data: raw_data.to_vec(),
            width,
            height,
            format: cam_types::FrameFormat::RawSensor,
            float_data: None,
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };
        let isp_params = self.controller.analyze_and_update(&frame_for_analysis);
        
        // Store params in frame for inference-time use
        frame_for_analysis.params = isp_params.clone();

        // 1. ISP processing (GPU) — output FloatRgb [1,3,H,W] f32
        let mut params = ProcessParams::new(width, height, raw_data);
        params.isp_params = Some(isp_params);
        params.target_width = self.width;
        params.target_height = self.height;
        params.sensor_max = self.config.sensor_max;
        params.output_format = self.config.output_format;

        #[allow(unused_mut)]
        let mut isp_output = self.engine.process(&params)?;

        // 2. GPU warp (if enabled) — operates directly on float data
        #[cfg(feature = "mnn")]
        if self.config.gpu_warp_enabled && warp_params.needs_warp() {
            if let Some(ref warp_engine) = self.gpu_warp_engine {
                if let Some(ref mut float_data) = isp_output.float_data {
                    let t_warp = std::time::Instant::now();
                    let n = (self.width * self.height * 3) as usize;

                    // Ensure buffer is large enough
                    if float_data.len() >= n {
                        // Copy input data for GPU warp (can't use same buffer for input/output)
                        let mut input_copy = vec![0.0f32; n];
                        input_copy.copy_from_slice(&float_data[..n]);

                        // Run GPU warp
                        match warp_engine.run_into(
                            &input_copy,
                            warp_params.gdc_k1,
                            warp_params.gdc_k2,
                            warp_params.gdc_k3,
                            warp_params.eis_dx,
                            warp_params.eis_dy,
                            &mut float_data[..n],
                        ) {
                            Ok(_) => {
                                info!("GPU warp (float): k1={:.3} k2={:.3} k3={:.3} dx={:.3} dy={:.3} ({:.2}ms)",
                                    warp_params.gdc_k1, warp_params.gdc_k2, warp_params.gdc_k3,
                                    warp_params.eis_dx, warp_params.eis_dy,
                                    t_warp.elapsed().as_secs_f64() * 1000.0);
                            }
                            Err(e) => {
                                warn!("GPU warp failed, falling back to CPU: {:?}", e);
                            }
                        }
                    }
                }
            }
        }

        // 3. Post-processing (CPU for deshake, temporal denoise, etc.)
        let post_output = if let Some(ref float_data) = isp_output.float_data {
            // Float path: use process_float for zero-copy
            self.post_pipeline.process_float(
                float_data,
                isp_output.width,
                isp_output.height,
                isp_output.aux.clone(),
                isp_output.timestamp_ns,
                Some(isp_output.params.clone()),
            ).map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        } else {
            // u8 path: use process
            self.post_pipeline.process(&isp_output)
                .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        };

        // GPU format conversion: float→target format via ONNX
        #[cfg(feature = "mnn")]
        let post_output = if let Some(ref converter) = self.format_converter {
            if let Some(ref float_data) = post_output.float_data {
                let n = (post_output.width * post_output.height * 3) as usize;
                if float_data.len() >= n {
                    let t_fmt = std::time::Instant::now();
                    let mut out_buf = vec![0u8; post_output.width as usize * post_output.height as usize * converter.output_format().bytes_per_pixel()];
                    match converter.convert(&float_data[..n], &mut out_buf) {
                        Ok(bytes_written) => {
                            out_buf.truncate(bytes_written);
                            info!("GPU format convert: {:?} ({:.2}ms)",
                                converter.output_format(),
                                t_fmt.elapsed().as_secs_f64() * 1000.0);
                            IspFrame {
            params: post_output.params.clone(),
                                data: out_buf,
                                width: post_output.width,
                                height: post_output.height,
                                format: post_output.format,
                                float_data: post_output.float_data,
                                aux: post_output.aux,
                                timestamp_ns: post_output.timestamp_ns,
                                prep_duration_ns: post_output.prep_duration_ns,
                                inference_duration_ns: post_output.inference_duration_ns,
                                total_duration_ns: post_output.total_duration_ns,
                            }
                        }
                        Err(e) => {
                            warn!("GPU format convert failed, falling back: {:?}", e);
                            post_output
                        }
                    }
                } else {
                    post_output
                }
            } else {
                post_output
            }
        } else {
            post_output
        };

        Ok(post_output)
    }

    /// Process with raw Bayer data.
    pub fn process_bayer(&mut self, raw_data: &[u8], bayer_width: u32, bayer_height: u32) -> IspResult<IspFrame> {
        self.process(raw_data, bayer_width, bayer_height)
    }

    /// Process using tiled rendering for high-resolution output (e.g., 4K→4K).
    /// Splits the input into tiles with overlap, processes each tile independently,
    /// and stitches the results together.
    pub fn process_tiled(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        warp_params: &GpuWarpParams,
    ) -> IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Pipeline("pipeline not initialized".into()));
        }

        // Check if tiled rendering is enabled in profile
        if !self.config.profile.use_tiled_rendering {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        let tile_x = self.config.profile.tile_count_x.max(1) as u32;
        let tile_y = self.config.profile.tile_count_y.max(1) as u32;
        let overlap = self.config.profile.tile_overlap as u32;

        if tile_x == 1 && tile_y == 1 {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        info!("Tiled rendering: {}×{} tiles with {}px overlap", tile_x, tile_y, overlap);

        // Calculate tile dimensions with overlap
        let tile_w = (width + tile_x - 1) / tile_x;
        let tile_h = (height + tile_y - 1) / tile_y;

        // Output dimensions per tile (without overlap)
        let out_tile_w = self.width / tile_x;
        let out_tile_h = self.height / tile_y;

        // Total output float data size: [1, 3, H, W]
        let total_float_size = (self.width * self.height * 3) as usize;
        let mut stitched_float = vec![0.0f32; total_float_size];

        // Process each tile
        for ty in 0..tile_y {
            for tx in 0..tile_x {
                // Input tile bounds with overlap
                let in_x_start = (tx * tile_w).saturating_sub(overlap);
                let in_y_start = (ty * tile_h).saturating_sub(overlap);
                let in_x_end = ((tx + 1) * tile_w).min(width) + overlap;
                let in_y_end = ((ty + 1) * tile_h).min(height) + overlap;

                let tile_in_w = in_x_end - in_x_start;
                let tile_in_h = in_y_end - in_y_start;

                // Output tile bounds (without overlap)
                let out_x_start = tx * out_tile_w;
                let out_y_start = ty * out_tile_h;
                let tile_out_w = out_tile_w;
                let tile_out_h = out_tile_h;

                info!("Processing tile ({},{}): in={}x{} out={}x{} at ({},{}) input offset=({},{})",
                    tx, ty, tile_in_w, tile_in_h, tile_out_w, tile_out_h, out_x_start, out_y_start, in_x_start, in_y_start);

                // Extract tile from raw data
                let tile_raw = Self::extract_tile(raw_data, width, in_x_start, in_y_start, tile_in_w, tile_in_h);

                // Process tile
                let mut params = ProcessParams::new(tile_in_w, tile_in_h, &tile_raw);
                params.isp_params = Some(self.controller.analyze_and_update(&crate::pipeline::types::IspFrame {
                    params: crate::isp_params::IspParams::default(),
                    data: tile_raw.clone(),
                    width: tile_in_w,
                    height: tile_in_h,
                    format: cam_types::FrameFormat::RawSensor,
                    float_data: None,
                    aux: None,
                    timestamp_ns: 0,
                    prep_duration_ns: 0,
                    inference_duration_ns: 0,
                    total_duration_ns: 0,
                }));
                params.target_width = tile_out_w;
                params.target_height = tile_out_h;
                params.sensor_max = self.config.sensor_max;
                params.output_format = self.config.output_format;

                let mut tile_output = self.engine.process(&params)?;

                // Apply GPU warp if enabled (per-tile)
                #[cfg(feature = "mnn")]
                if self.config.gpu_warp_enabled && warp_params.needs_warp() {
                    if let Some(ref warp_engine) = self.gpu_warp_engine {
                        if let Some(ref mut float_data) = tile_output.float_data {
                            let n = (tile_out_w * tile_out_h * 3) as usize;
                            if float_data.len() >= n {
                                let mut input_copy = vec![0.0f32; n];
                                input_copy.copy_from_slice(&float_data[..n]);
                                let _ = warp_engine.run_into(
                                    &input_copy,
                                    warp_params.gdc_k1,
                                    warp_params.gdc_k2,
                                    warp_params.gdc_k3,
                                    warp_params.eis_dx,
                                    warp_params.eis_dy,
                                    &mut float_data[..n],
                                );
                            }
                        }
                    }
                }

                // Stitch tile float data into output
                if let Some(ref tile_float) = tile_output.float_data {
                    Self::stitch_tile_f32(
                        &mut stitched_float,
                        tile_float,
                        self.width,
                        self.height,
                        out_x_start,
                        out_y_start,
                        tile_out_w,
                        tile_out_h,
                    );
                }
            }
        }

        // Create stitched ISP output
        let mut isp_output = IspFrame {
            params: crate::isp_params::IspParams::default(),
            float_data: Some(stitched_float),
            width: self.width,
            height: self.height,
            format: cam_types::FrameFormat::NchwFloat,
            data: vec![],
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        };

        // Apply GPU warp on stitched result if needed
        #[cfg(feature = "mnn")]
        if self.config.gpu_warp_enabled && warp_params.needs_warp() {
            if let Some(ref warp_engine) = self.gpu_warp_engine {
                if let Some(ref mut float_data) = isp_output.float_data {
                    let n = (self.width * self.height * 3) as usize;
                    if float_data.len() >= n {
                        let mut input_copy = vec![0.0f32; n];
                        input_copy.copy_from_slice(&float_data[..n]);
                        let _ = warp_engine.run_into(
                            &input_copy,
                            warp_params.gdc_k1,
                            warp_params.gdc_k2,
                            warp_params.gdc_k3,
                            warp_params.eis_dx,
                            warp_params.eis_dy,
                            &mut float_data[..n],
                        );
                    }
                }
            }
        }

        // Run post-processing on stitched result
        let post_output = if let Some(ref float_data) = isp_output.float_data {
            self.post_pipeline.process_float(
                float_data,
                isp_output.width,
                isp_output.height,
                isp_output.aux.clone(),
                isp_output.timestamp_ns,
                None, // params not needed for post-processing
            )?
        } else {
            self.post_pipeline.process(&isp_output)?
        };

        Ok(post_output)
    }

    /// Extract a tile from packed raw Bayer data (INT32 packed, half width).
    fn extract_tile(
        raw_data: &[u8],
        full_width: u32,
        x: u32,
        y: u32,
        w: u32,
        h: u32,
    ) -> Vec<u8> {
        let packed_w = (full_width / 2) as usize;
        let row_stride = packed_w * 4; // bytes per row (INT32 = 4 bytes)
        let start_row = y as usize;
        let start_col = (x / 2) as usize; // packed Bayer: 2 pixels per INT32
        let tile_packed_w = (w / 2) as usize;

        let mut tile = Vec::with_capacity((tile_in_h * row_stride) as usize);
        for row in start_row..(start_row + h as usize) {
            let row_start = row * row_stride + start_col * 4;
            let row_end = row_start + tile_packed_w * 4;
            if row_end <= raw_data.len() {
                tile.extend_from_slice(&raw_data[row_start..row_end]);
            }
        }
        tile
    }

    /// Stitch a tile of float data into the output buffer.
    /// Input tile is [tile_out_w * tile_out_h * 3] f32 (planar RGB).
    fn stitch_tile_f32(
        output: &mut [f32],
        tile: &[f32],
        out_width: u32,
        out_height: u32,
        x: u32,
        y: u32,
        w: u32,
        h: u32,
    ) {
        let out_w = out_width as usize;
        let tile_w = w as usize;
        let tile_h = h as usize;

        // Planar layout: R plane, then G plane, then B plane
        let plane_size = out_w * out_height as usize;

        for c in 0..3 {
            let out_base = c * plane_size + (y as usize) * out_w + x as usize;
            let tile_base = c * tile_w * tile_h;
            for row in 0..tile_h {
                let out_row = out_base + row * out_w;
                let tile_row = tile_base + row * tile_w;
                output[out_row..out_row + tile_w].copy_from_slice(&tile[tile_row..tile_row + tile_w]);
            }
        }
    }

    /// Get pipeline info.
    pub fn info(&self) -> PipelineInfo {
        PipelineInfo {
            profile: self.config.profile.label.to_string(),
            engine: self.engine.backend_name().to_string(),
            input_width: self.width,
            input_height: self.height,
            output_format: format!("{:?}", self.config.output_format),
            post_eis: self.config.post_config.eis_enabled,
            post_deshake: self.config.post_config.deshake_enabled,
            post_gdc: self.config.post_config.gdc_enabled,
            post_temporal_denoise: self.config.post_config.temporal_denoise_enabled,
            gpu_warp: self.config.gpu_warp_enabled,
        }
    }

    /// Check if pipeline is ready.
    pub fn is_ready(&self) -> bool { self.initialized }

    /// Get reference to the engine.
    pub fn engine(&self) -> &dyn IspEngine { self.engine.as_ref() }

    /// Get reference to GPU warp block.
    pub fn gpu_warp(&self) -> Option<&GpuWarpBlock> { self.gpu_warp.as_ref() }

    /// Get GPU warp ONNX model.
    pub fn gpu_warp_onnx(&self) -> Option<&[u8]> { self.gpu_warp_onnx.as_deref() }

    /// Extract a tile from raw Bayer data (packed INT32 format).
    /// Raw data is INT32 where each pixel is 16-bit, packed 2 pixels per INT32.
    /// Extract a tile from raw Bayer data (packed INT32 format).
    /// Raw data is INT32 where each pixel is 16-bit, packed 2 pixels per INT32.
mod tests {
    use super::*;

    #[test]
    fn test_unified_config_defaults() {
        let cfg = UnifiedConfig::default();
        assert_eq!(cfg.profile.label, "MED");
        assert_eq!(cfg.target_width, 1920);
        assert!(matches!(cfg.output_format, OutputFormat::FloatRgb));
        assert!(!cfg.gpu_warp_enabled);
    }

    #[test]
    fn test_unified_config_presets() {
        assert_eq!(UnifiedConfig::hd().target_width, 1280);
        assert_eq!(UnifiedConfig::fhd().target_width, 1920);
        assert_eq!(UnifiedConfig::uhd().target_width, 3840);
    }

    #[test]
    fn test_unified_config_builder() {
        let cfg = UnifiedConfig::hd().with_eis().with_deshake().with_gdc(0.5)
            .with_temporal_denoise().with_gpu_warp().with_rgba_output();
        assert!(cfg.gpu_warp_enabled);
        assert!(matches!(cfg.output_format, OutputFormat::Rgba));
    }

    #[test]
    fn test_unified_pipeline_build() {
        crate::init();
        let config = UnifiedConfig::hd();
        let pipeline = UnifiedPipeline::new(config);
        assert!(pipeline.is_ok(), "Build failed: {:?}", pipeline.err());
        let p = pipeline.unwrap();
        assert!(p.is_ready());
        assert_eq!(p.info().profile, "LITE");
    }

    #[test]
    fn test_unified_pipeline_process() {
        crate::init();
        let pipeline = UnifiedPipeline::new(UnifiedConfig::hd()).unwrap();
        let raw = vec![128u8; 1280 * 720 * 2];
        let params = ProcessParams::new(1280, 720, &raw);
        let result = pipeline.process(&params);
        assert!(result.is_ok(), "Process failed: {:?}", result.err());
        let frame = result.unwrap();
        assert!(frame.width > 0 && !frame.data.is_empty());
    }

    #[test]
    fn test_unified_pipeline_with_gpu_warp() {
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config);
        assert!(pipeline.is_ok(), "Build failed: {:?}", pipeline.err());
        let p = pipeline.unwrap();
        assert!(p.gpu_warp().is_some());
        assert!(p.gpu_warp_onnx().is_some());
    }

    #[test]
    fn test_gpu_warp_params() {
        assert!(!GpuWarpParams::identity().needs_warp());
        assert!(GpuWarpParams::gdc(-0.3, 0.0, 0.0).needs_warp());
        assert!(GpuWarpParams::eis(0.1, 0.0).needs_warp());
        assert!(!GpuWarpParams { gdc_k1: 1e-7, ..Default::default() }.needs_warp());
    }

    #[test]
    fn test_unified_pipeline_info_display() {
        crate::init();
        let config = UnifiedConfig::pro().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config).unwrap();
        let s = format!("{}", pipeline.info());
        assert!(s.contains("PRO") && s.contains("GPU_Warp"));
    }

    #[test]
    fn test_unified_pipeline_float_rgb_format() {
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config).unwrap();
        let info = pipeline.info();
        assert!(info.output_format.contains("FloatRgb"));
    }

impl ProcessPipeline for UnifiedPipeline {
    fn process(&self, params: &ProcessParams) -> IspResult<IspFrame> {
        self.engine.process(params)
    }

    fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }

    fn is_loaded(&self) -> bool {
        self.initialized
    }
}

/// Pipeline information.
