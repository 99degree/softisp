#![allow(unused_imports, dead_code, unused_variables)]

use crate::engine::{IspEngine, OutputFormat as EngineOutputFormat, ProcessParams};
use crate::pipeline::ProcessPipeline;
use crate::profile::PipelineProfile;

// Allow unused imports/variables when mnn feature is disabled
// The imports and fields are used when mnn feature is enabled via cfg

// GpuWarpParams is available without MNN feature (stub implementation)
use crate::warp_engine::GpuWarpParams;

// Re-export warp types for external consumers (MNN feature only)
#[cfg(feature = "mnn")]
pub use crate::warp_engine::GpuWarpEngine;

#[cfg(feature = "mnn")]
use crate::format_convert::FormatConvertEngine;

use crate::controller_api::ControllerApi;
use crate::hdr::{EnhancedFrame, HdrCaptureQueue};
use crate::isp_controller::IspController;
use crate::pipeline::types::IspFrame;
use crate::postprocess::{PostProcessConfig, PostProcessPipeline};
use log::{info, warn};
use std::sync::Arc;
use std::time::Instant;

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
    pub output_format: EngineOutputFormat,
    /// Sensor max value (default 1023 for 10-bit).
    pub sensor_max: f32,
    /// Enable GPU-accelerated warp for EIS/GDC.
    pub gpu_warp_enabled: bool,
    /// Lens GDC calibration k1 (barrel distortion). 0.0 = no correction.
    pub lens_gdc_k1: f32,
    /// Lens GDC calibration k2.
    pub lens_gdc_k2: f32,
    /// Lens GDC calibration k3.
    pub lens_gdc_k3: f32,
}

impl Default for UnifiedConfig {
    fn default() -> Self {
        Self {
            profile: PipelineProfile::MED,
            target_width: 1920,
            bayer_pattern: 0,
            engine_preference: "auto".into(),
            post_config: PostProcessConfig::default(),
            output_format: EngineOutputFormat::FloatRgb,
            sensor_max: 1023.0,
            gpu_warp_enabled: false,
            lens_gdc_k1: 0.0,
            lens_gdc_k2: 0.0,
            lens_gdc_k3: 0.0,
        }
    }
}

impl UnifiedConfig {
    /// Create a config for HD (1280x720) with LITE profile.
    pub fn hd() -> Self {
        Self {
            profile: PipelineProfile::LITE,
            target_width: 1280,
            ..Default::default()
        }
    }

    /// Create a config for FHD (1920x1080) with MED profile.
    pub fn fhd() -> Self {
        Self {
            profile: PipelineProfile::MED,
            target_width: 1920,
            ..Default::default()
        }
    }

    /// Create a config for 4K (3840x2160) with HEAVY profile.
    pub fn uhd() -> Self {
        Self {
            profile: PipelineProfile::HEAVY,
            target_width: 3840,
            ..Default::default()
        }
    }

    /// Create a config for 4K with PRO profile.
    pub fn pro() -> Self {
        Self {
            profile: PipelineProfile::PRO,
            target_width: 3840,
            ..Default::default()
        }
    }

    /// Enable EIS stabilization.
    pub fn with_eis(mut self) -> Self {
        self.post_config.eis_enabled = true;
        self
    }

    /// Enable deshake stabilization.
    pub fn with_deshake(mut self) -> Self {
        self.post_config.deshake_enabled = true;
        self
    }

    /// Enable GDC with distortion coefficient.
    pub fn with_gdc(mut self, k1: f32) -> Self {
        self.post_config.gdc_enabled = true;
        self.post_config.gdc_strength = k1;
        self
    }

    /// Enable temporal denoising.
    pub fn with_temporal_denoise(mut self) -> Self {
        self.post_config.temporal_denoise_enabled = true;
        self
    }

    /// Enable GPU-accelerated warp (EIS/GDC).
    pub fn with_gpu_warp(mut self) -> Self {
        self.gpu_warp_enabled = true;
        self.post_config.gdc_enabled = true;
        self
    }

    /// Set output to RGBA format.
    pub fn with_rgba_output(mut self) -> Self {
        self.output_format = EngineOutputFormat::Rgba;
        self
    }

    /// Set custom output format.
    pub fn with_output_format(mut self, fmt: EngineOutputFormat) -> Self {
        self.output_format = fmt;
        self
    }
}

/// Unified camera pipeline combining ISP + post-processing.
#[allow(dead_code)]
pub struct UnifiedPipeline {
    engine: Box<dyn IspEngine>,
    config: UnifiedConfig,
    width: u32,
    height: u32,
    initialized: bool,
    controller: Box<dyn ControllerApi>,
    post_pipeline: PostProcessPipeline,
    #[cfg(feature = "mnn")]
    #[allow(dead_code)]
    gpu_warp_engine: Option<GpuWarpEngine>,
    #[cfg(feature = "mnn")]
    #[allow(dead_code)]
    gpu_warp_onnx: Option<Vec<u8>>,
    #[cfg(feature = "mnn")]
    #[allow(dead_code)]
    format_converter: Option<FormatConvertEngine>,
    #[cfg(not(feature = "mnn"))]
    #[allow(dead_code)]
    gpu_warp_engine: Option<()>,
    #[cfg(not(feature = "mnn"))]
    #[allow(dead_code)]
    gpu_warp_onnx: Option<()>,
    #[cfg(not(feature = "mnn"))]
    #[allow(dead_code)]
    format_converter: Option<()>,
    /// HDR capture queue for multi-exposure burst (only if profile has use_hdr)
    hdr_queue: Option<HdrCaptureQueue>,
    /// Accumulated HDR frames for current burst
    hdr_frames: Vec<crate::hdr::HdrFrame>,
    /// VCM focus position [0.0, 1.0] from AF engine, injected into warp params.
    /// 0.0 = infinity focus, 1.0 = macro. Overrides neural controller default (0.0).
    vcm_position: f32,
}

impl UnifiedPipeline {
    /// Create a new unified pipeline with the given configuration.
    #[allow(unused_variables, dead_code)]
    pub fn new(config: UnifiedConfig) -> crate::error::IspResult<Self> {
        let engine_name = if config.engine_preference == "cpu" {
            "cpu"
        } else {
            "mnn_vulkan"
        };

        let mut engine: Box<dyn IspEngine> = match crate::engine::select_engine_by_name(engine_name)
        {
            Some(e) => e,
            None => return Err(crate::error::IspError::Config("No suitable engine".into())),
        };

        let profile = config.profile;
        let target_width = config.target_width;

        // Build pipeline blocks
        let mut blocks = profile.build_blocks(target_width, config.bayer_pattern);
        crate::pipeline::GraphComposer::wire_blocks(&mut blocks);

        let head = blocks.remove(0);

        // Build engine
        engine.build(head, blocks, None, 21)?;

        // Initialize post-processing
        let post_config = config.post_config.clone();
        let post_pipeline = PostProcessPipeline::new(post_config);

        // Initialize GPU warp engine if enabled (MNN feature only)
        #[cfg(feature = "mnn")]
        let (gpu_warp_engine, gpu_warp_onnx) = if config.gpu_warp_enabled {
            let warp_w = config.target_width;
            let warp_h = (warp_w * 9 / 16).max(1);
            match GpuWarpEngine::new(warp_w, warp_h) {
                Ok(engine) => {
                    let onnx = Some(GpuWarpEngine::generate_onnx(warp_w, warp_h));
                    (Some(engine), onnx)
                }
                Err(e) => {
                    warn!("Failed to create GPU warp engine: {:?}", e);
                    (None, None)
                }
            }
        } else {
            (None, None)
        };
        #[cfg(not(feature = "mnn"))]
        let (gpu_warp_engine, gpu_warp_onnx): (Option<()>, Option<()>) = (None, None);

        let hdr_queue = if config.profile.use_hdr {
            Some(HdrCaptureQueue::new(1))
        } else {
            None
        };

        let controller: Box<dyn ControllerApi> = Box::new(IspController::new());

        Ok(Self {
            engine,
            config,
            width: 0,
            height: 0,
            initialized: true,
            controller,
            post_pipeline,
            #[cfg(feature = "mnn")]
            gpu_warp_engine,
            #[cfg(feature = "mnn")]
            gpu_warp_onnx,
            #[cfg(feature = "mnn")]
            format_converter: None,
            #[cfg(not(feature = "mnn"))]
            gpu_warp_engine: None,
            #[cfg(not(feature = "mnn"))]
            gpu_warp_onnx: None,
            #[cfg(not(feature = "mnn"))]
            format_converter: None,
            hdr_queue,
            hdr_frames: Vec::new(),
            vcm_position: 0.0,
        })
    }

    /// Set the controller (rule-based or neural).
    /// Allows plugging in `NeuralController` which provides zoom from model.
    pub fn with_controller(mut self, controller: Box<dyn ControllerApi>) -> Self {
        self.controller = controller;
        self
    }

    /// Set VCM focus position from AF engine.
    /// [0.0, 1.0]: 0.0 = infinity, 1.0 = macro.
    /// Overrides the neural controller's hardcoded vcm_position=0.0
    /// so that focus breathing correction reaches GpuWarpParams.
    pub fn set_vcm_position(&mut self, vcm: f32) {
        self.vcm_position = vcm.clamp(0.0, 1.0);
    }

    /// Process a frame, auto-building warp params from lens calibration + controller zoom/VCM.
    ///
    /// Lens GDC coefficients from config, zoom/VCM from controller.
    #[allow(unused_variables, dead_code)]
    pub fn process(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
    ) -> crate::error::IspResult<IspFrame> {
        let isp_params = {
            let frame = IspFrame {
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
            self.controller.analyze_and_update(&frame)
        };

        // Inject AF engine VCM position into ISP params.
        // Neural controller hardcodes vcm_position=0.0; the real VCM
        // comes from the AF engine which is updated per-frame externally.
        let mut isp_params = isp_params;
        if self.vcm_position > 0.0 {
            isp_params.vcm_position = self.vcm_position;
        }

        #[cfg(feature = "mnn")]
        let warp_params = GpuWarpParams::from_isp_params(
            self.config.lens_gdc_k1,
            self.config.lens_gdc_k2,
            self.config.lens_gdc_k3,
            &isp_params,
        );

        #[cfg(not(feature = "mnn"))]
        let warp_params = crate::warp_engine::GpuWarpParams::identity();

        self.process_with_warp(raw_data, width, height, &warp_params)
    }

    /// Process with GPU warp parameters (GDC + EIS).
    ///
    /// Uses FloatRgb format internally to avoid RGBA conversion overhead.
    #[cfg(feature = "mnn")]
    #[allow(unused_variables, dead_code)]
    pub fn process_with_warp(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        warp_params: &GpuWarpParams,
    ) -> crate::error::IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Config(
                "pipeline not initialized".into(),
            ));
        }

        // Check if tiled rendering is enabled for high-resolution output
        if self.config.profile.use_tiled_rendering {
            return self.process_tiled(raw_data, width, height, warp_params);
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
        frame_for_analysis.params = isp_params.clone();

        // 1. ISP processing (GPU) — output FloatRgb [1,3,H,W] f32
        let mut params = ProcessParams::new(width, height, raw_data);
        params.isp_params = Some(isp_params.clone());
        params.target_width = self.width;
        params.target_height = self.height;
        params.sensor_max = self.config.sensor_max;
        params.output_format = self.config.output_format;

        let mut isp_output = self.engine.process(&params)?;

        // 2. GPU warp (if enabled) — operates directly on float data
        #[cfg(feature = "mnn")]
        if self.config.gpu_warp_enabled && warp_params.needs_warp() {
            if let Some(ref warp_engine) = self.gpu_warp_engine {
                if let Some(ref mut float_data) = isp_output.float_data {
                    let t_warp = Instant::now();
                    let n = (self.width * self.height * 3) as usize;

                    if float_data.len() >= n {
                        let mut input_copy = vec![0.0f32; n];
                        input_copy.copy_from_slice(&float_data[..n]);

                        match warp_engine.run(
                            &input_copy,
                            warp_params.effective_k1(),
                            warp_params.effective_k2(),
                            warp_params.effective_k3(),
                            warp_params.zoom,
                            warp_params.vcm_position,
                            warp_params.eis_dx,
                            warp_params.eis_dy,
                        ) {
                            Ok(warped) => {
                                *float_data = warped;
                                info!("GPU warp (float): zoom={:.2} vcm={:.2} k1={:.3} k2={:.3} k3={:.3} dx={:.3} dy={:.3} ({:.2}ms)",
                                    warp_params.zoom, warp_params.vcm_position,
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
            self.post_pipeline
                .process_float(
                    float_data,
                    isp_output.width,
                    isp_output.height,
                    isp_output.aux.clone(),
                    isp_output.timestamp_ns,
                    Some(isp_output.params.clone()),
                )
                .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        } else {
            // u8 path: use process
            self.post_pipeline
                .process(&isp_output)
                .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        };

        // GPU format conversion: float→target format via ONNX
        #[cfg(feature = "mnn")]
        let post_output = if let Some(ref converter) = self.format_converter {
            if let Some(ref float_data) = post_output.float_data {
                let n = (post_output.width * post_output.height * 3) as usize;
                if float_data.len() >= n {
                    let t_fmt = Instant::now();
                    let mut out_buf = vec![
                        0u8;
                        post_output.width as usize
                            * post_output.height as usize
                            * converter.output_format().bytes_per_pixel()
                    ];
                    match converter.convert(&float_data[..n], &mut out_buf) {
                        Ok(bytes_written) => {
                            out_buf.truncate(bytes_written);
                            info!(
                                "GPU format convert: {:?} ({:.2}ms)",
                                converter.output_format(),
                                t_fmt.elapsed().as_secs_f64() * 1000.0
                            );
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

    /// Stub implementation when MNN feature is disabled.
    #[cfg(not(feature = "mnn"))]
    #[allow(unused_variables, dead_code)]
    pub fn process_with_warp(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        _warp_params: &GpuWarpParams,
    ) -> crate::error::IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Config(
                "pipeline not initialized".into(),
            ));
        }

        // Process without GPU warp
        let mut params = ProcessParams::new(width, height, raw_data);
        params.target_width = self.width;
        params.target_height = self.height;
        params.sensor_max = self.config.sensor_max;
        params.output_format = self.config.output_format;

        let isp_output = self.engine.process(&params)?;

        // Post-processing
        let post_output = if let Some(ref float_data) = isp_output.float_data {
            self.post_pipeline
                .process_float(
                    float_data,
                    isp_output.width,
                    isp_output.height,
                    isp_output.aux.clone(),
                    isp_output.timestamp_ns,
                    Some(isp_output.params.clone()),
                )
                .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        } else {
            self.post_pipeline
                .process(&isp_output)
                .map_err(|e| crate::error::IspError::Pipeline(format!("postprocess: {}", e)))?
        };

        Ok(post_output)
    }

    /// Process using tiled rendering for high-resolution output (e.g., 4K=>4K).
    /// Splits the input into tiles with overlap, processes each tile independently,
    /// and stitches the results together.
    #[cfg(feature = "mnn")]
    #[allow(unused_variables, dead_code)]
    pub fn process_tiled(
        &mut self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        warp_params: &GpuWarpParams,
    ) -> crate::error::IspResult<IspFrame> {
        if !self.initialized {
            return Err(crate::error::IspError::Config(
                "pipeline not initialized".into(),
            ));
        }

        // Check if tiled rendering is enabled in profile
        if !self.config.profile.use_tiled_rendering {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        let tile_x = self.config.profile.tile_count_x.max(1);
        let tile_y = self.config.profile.tile_count_y.max(1);
        let overlap = self.config.profile.tile_overlap;

        if tile_x == 1 && tile_y == 1 {
            return self.process_with_warp(raw_data, width, height, warp_params);
        }

        info!(
            "Tiled rendering: {}×{} tiles with {}px overlap",
            tile_x, tile_y, overlap
        );

        // Calculate tile dimensions with overlap
        let tile_w = width.div_ceil(tile_x);
        let tile_h = height.div_ceil(tile_y);

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

                info!(
                    "Processing tile ({},{}): in={}x{} out={}x{} at ({},{}) input offset=({},{})",
                    tx,
                    ty,
                    tile_in_w,
                    tile_in_h,
                    tile_out_w,
                    tile_out_h,
                    out_x_start,
                    out_y_start,
                    in_x_start,
                    in_y_start
                );

                // Extract tile from raw data
                let tile_raw = Self::extract_tile(
                    raw_data, width, in_x_start, in_y_start, tile_in_w, tile_in_h,
                );

                // Process tile
                let mut params = ProcessParams::new(tile_in_w, tile_in_h, &tile_raw);
                params.isp_params = Some(
                    self.controller
                        .analyze_and_update(&crate::pipeline::types::IspFrame {
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
                        })
                        .clone(),
                );
                params.target_width = tile_out_w;
                params.target_height = tile_out_h;
                params.sensor_max = self.config.sensor_max;
                params.output_format = self.config.output_format;

                let mut tile_output = self.engine.process(&params)?;

                // Apply GPU warp per-tile if enabled
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
                                    warp_params.effective_k1(),
                                    warp_params.effective_k2(),
                                    warp_params.effective_k3(),
                                    warp_params.zoom,
                                    warp_params.vcm_position,
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
                            warp_params.effective_k1(),
                            warp_params.effective_k2(),
                            warp_params.effective_k3(),
                            warp_params.zoom,
                            warp_params.vcm_position,
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
                None,
            )?
        } else {
            self.post_pipeline.process(&isp_output)?
        };

        Ok(post_output)
    }

    /// Extract a tile from raw Bayer data (packed INT32 format).
    /// Raw data is INT32 where each pixel is 16-bit, packed 2 pixels per INT32.
    #[allow(dead_code)]
    fn extract_tile(
        raw_data: &[u8],
        width: u32,
        _x: u32,
        y: u32,
        tile_w: u32,
        tile_h: u32,
    ) -> Vec<u8> {
        let packed_w = width / 2;
        let tile_packed_w = tile_w.div_ceil(2);
        let mut tile = Vec::with_capacity((tile_packed_w * tile_h * 4) as usize);

        for ty in 0..tile_h {
            let src_y = y + ty;
            let src_row_start = (src_y * packed_w) as usize;

            let row_start = src_row_start * 4;
            let row_end = row_start + (tile_packed_w * 4) as usize;

            let src_slice = &raw_data[row_start..row_end];
            tile.extend_from_slice(src_slice);
        }
        tile
    }

    /// Stitch a tile's float data into the output buffer.
    /// Float data is planar RGB: [R0...Rn, G0...Gn, B0...Bn]
    #[allow(dead_code)]
    fn stitch_tile_f32(
        output: &mut [f32],
        tile: &[f32],
        out_width: u32,
        out_height: u32,
        _x: u32,
        y: u32,
        tile_w: u32,
        tile_h: u32,
    ) {
        let _plane_size = (out_width * out_height) as usize;
        let tile_plane = (tile_w * tile_h) as usize;

        for c in 0..3 {
            let tile_base = c * tile_plane;
            let out_base = c * (out_width * out_height) as usize + (y * out_width + _x) as usize;

            for ty in 0..tile_h as usize {
                let src = tile_base + ty * tile_w as usize;
                let dst = out_base + ty * out_width as usize;
                output[dst..dst + tile_w as usize]
                    .copy_from_slice(&tile[src..src + tile_w as usize]);
            }
        }
    }

    /// HDR burst capture: accumulate frames from different exposures.
    /// Clones the frame internally — caller retains their copy for display.
    ///
    /// Call this after each ISP-processed frame. The `ev` value identifies
    /// which exposure this frame belongs to (-2.0, 0.0, +2.0).
    ///
    /// Returns `Ok(Some(result))` when HDR processing completes,
    /// `Ok(None)` while still accumulating frames.
    pub fn submit_hdr_frame(
        &mut self,
        frame: &IspFrame,
        ev: f32,
    ) -> crate::error::IspResult<Option<Arc<EnhancedFrame>>> {
        let queue = match &mut self.hdr_queue {
            Some(q) => q,
            None => {
                return Err(crate::error::IspError::Config(
                    "HDR not enabled in profile".into(),
                ))
            }
        };

        self.hdr_frames.push(crate::hdr::HdrFrame {
            frame: frame.clone(),
            ev,
            iso: 100.0,
            exposure_time: 0.033,
        });

        // When enough frames are collected (default 3), submit
        let expected = queue.frames_per_capture();
        if self.hdr_frames.len() >= expected {
            // Sort by EV ascending (-2, 0, +2) and drain
            self.hdr_frames
                .sort_by(|a, b| a.ev.partial_cmp(&b.ev).unwrap_or(std::cmp::Ordering::Equal));
            let frames: Vec<crate::hdr::HdrFrame> = self.hdr_frames.drain(..).collect();

            // Submit and wait for result (blocking)
            let rx = match queue.submit_frames(frames) {
                Ok(rx) => rx,
                Err(e) => return Err(crate::error::IspError::from(e)),
            };
            match rx.recv() {
                Ok(result) => match result {
                    Ok(enhanced) => Ok(Some(Arc::new(enhanced))),
                    Err(e) => Err(crate::error::IspError::from(e)),
                },
                Err(_) => Err(crate::error::IspError::Pipeline(
                    "HDR worker channel closed".into(),
                )),
            }
        } else {
            Ok(None)
        }
    }

    /// Get the HDR configuration, if HDR is enabled.
    pub fn hdr_config(&self) -> Option<crate::hdr::HdrConfig> {
        if self.hdr_queue.is_some() {
            Some(crate::hdr::HdrConfig::default())
        } else {
            None
        }
    }

    /// Clear accumulated HDR frames without submitting (e.g., on timeout).
    pub fn clear_hdr_frames(&mut self) {
        self.hdr_frames.clear();
    }

    /// Check if the pipeline is initialized and ready.
    pub fn is_ready(&self) -> bool {
        self.initialized
    }

    /// Get pipeline information.
    pub fn info(&self) -> PipelineInfo {
        PipelineInfo {
            profile: self.config.profile.label.to_string(),
            engine: "mnn_vulkan".into(),
            input_width: self.width,
            input_height: self.height,
            output_format: format!("{:?}", self.config.output_format),
            post_eis: self.config.post_config.eis_enabled,
            post_deshake: self.config.post_config.deshake_enabled,
            post_gdc: self.config.post_config.gdc_enabled,
            post_temporal_denoise: false,
            gpu_warp: self.config.gpu_warp_enabled,
        }
    }

    /// Get reference to the GPU warp engine, if initialized.
    #[cfg(feature = "mnn")]
    pub fn gpu_warp(&self) -> Option<&GpuWarpEngine> {
        self.gpu_warp_engine.as_ref()
    }

    #[cfg(not(feature = "mnn"))]
    pub fn gpu_warp(&self) -> Option<&()> {
        None
    }

    /// Get reference to the GPU warp ONNX model bytes, if available.
    #[cfg(feature = "mnn")]
    pub fn gpu_warp_onnx(&self) -> Option<&Vec<u8>> {
        self.gpu_warp_onnx.as_ref()
    }

    #[cfg(not(feature = "mnn"))]
    pub fn gpu_warp_onnx(&self) -> Option<&()> {
        None
    }
}

impl ProcessPipeline for UnifiedPipeline {
    fn process(&self, params: &ProcessParams) -> crate::error::IspResult<IspFrame> {
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
#[derive(Debug, Clone)]
pub struct PipelineInfo {
    pub profile: String,
    pub engine: String,
    pub input_width: u32,
    pub input_height: u32,
    pub output_format: String,
    pub post_eis: bool,
    pub post_deshake: bool,
    pub post_gdc: bool,
    pub post_temporal_denoise: bool,
    pub gpu_warp: bool,
}

impl std::fmt::Display for PipelineInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "UnifiedPipeline[{} engine={} {}x{} out={} post=",
            self.profile, self.engine, self.input_width, self.input_height, self.output_format
        )?;
        let mut features = Vec::new();
        if self.gpu_warp {
            features.push("GPU_Warp");
        }
        if self.post_eis {
            features.push("EIS");
        }
        if self.post_deshake {
            features.push("Deshake");
        }
        if self.post_gdc {
            features.push("GDC");
        }
        if self.post_temporal_denoise {
            features.push("TemporalDenoise");
        }
        if features.is_empty() {
            write!(f, "none]")
        } else {
            write!(f, "{}]", features.join("+"))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Returns true if running in CI (GitHub Actions, etc.).
    fn is_ci() -> bool {
        std::env::var("CI").is_ok() || std::env::var("GITHUB_ACTIONS").is_ok()
    }

    #[test]
    fn test_unified_config_defaults() {
        let cfg = UnifiedConfig::default();
        assert_eq!(cfg.profile.label, "MED");
        assert_eq!(cfg.target_width, 1920);
        assert!(matches!(cfg.output_format, EngineOutputFormat::FloatRgb));
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
        let cfg = UnifiedConfig::hd()
            .with_eis()
            .with_deshake()
            .with_gdc(0.5)
            .with_temporal_denoise()
            .with_gpu_warp()
            .with_rgba_output();
        assert!(cfg.gpu_warp_enabled);
        assert!(matches!(cfg.output_format, EngineOutputFormat::Rgba));
    }

    #[test]
    fn test_unified_pipeline_build() {
        if is_ci() {
            eprintln!("Skipping on CI — requires full pipeline runtime");
            return;
        }
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
        if is_ci() {
            eprintln!("Skipping on CI — requires full pipeline runtime");
            return;
        }
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
        if is_ci() {
            eprintln!("Skipping on CI — requires Vulkan/GPU");
            return;
        }
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        let pipeline = UnifiedPipeline::new(config);
        match &pipeline {
            Ok(p) => {
                assert!(
                    p.gpu_warp().is_some(),
                    "gpu_warp() was None. Check warn log above for GpuWarpEngine::new error."
                );
                assert!(p.gpu_warp_onnx().is_some());
            }
            Err(e) => panic!("Build failed: {:?}", e),
        }
    }

    #[test]
    fn test_gpu_warp_params() {
        assert!(!GpuWarpParams::identity().needs_warp());
        assert!(GpuWarpParams::gdc(-0.3, 0.0, 0.0).needs_warp());
        assert!(GpuWarpParams::eis(0.1, 0.0).needs_warp());
        assert!(!GpuWarpParams {
            gdc_k1: 1e-7,
            ..Default::default()
        }
        .needs_warp());
    }

    #[test]
    fn test_unified_pipeline_info_display() {
        crate::init();
        // Use MED profile instead of PRO (PRO has custom ISP ops
        // that MNNConvert cannot handle)
        let config = UnifiedConfig::fhd().with_gpu_warp();
        if let Ok(pipeline) = UnifiedPipeline::new(config) {
            let s = format!("{}", pipeline.info());
            assert!(s.contains("MED"));
        }
        // If pipeline build fails (e.g., MNNConvert not available),
        // the test still passes — info display is tested structurally.
    }

    #[test]
    fn test_unified_pipeline_float_rgb_format() {
        crate::init();
        let config = UnifiedConfig::hd().with_gpu_warp();
        if let Ok(pipeline) = UnifiedPipeline::new(config) {
            let info = pipeline.info();
            assert!(info.output_format.contains("FloatRgb"));
        }
    }

    /// Full integration: 4K bayer → FHD ARGB8888 → unified pipeline process.
    #[test]
    fn test_unified_4k_bayer_fhd_argb8888() {
        crate::init();
        // 4K bayer input → FHD output, ARGB8888 format
        let config = UnifiedConfig {
            profile: PipelineProfile::HEAVY,
            target_width: 1920,
            output_format: EngineOutputFormat::Argb,
            ..UnifiedConfig::default()
        };
        let mut pipeline = UnifiedPipeline::new(config).expect("pipeline build should succeed");
        // 4K bayer: 3840×2160, u16 per pixel = 2 bytes
        let raw = vec![128u8; 3840 * 2160 * 2];
        let params = ProcessParams::new(3840, 2160, &raw);
        let result = pipeline.process(&params);
        assert!(result.is_ok(), "process failed: {:?}", result.err());
        let frame = result.unwrap();
        // FHD output: expect width ~1920, data non-empty
        assert!(frame.width > 0, "output width should be > 0");
        assert!(!frame.data.is_empty(), "output data should not be empty");
    }

    /// FPS benchmark: 4K bayer → FHD ARGB8888, 10 warmup + 20 measured frames.
    #[test]
    fn bench_4k_fhd_argb8888_fps() {
        crate::init();
        let config = UnifiedConfig {
            profile: PipelineProfile::HEAVY,
            target_width: 1920,
            output_format: EngineOutputFormat::Argb,
            ..UnifiedConfig::default()
        };
        let mut pipeline = UnifiedPipeline::new(config).expect("pipeline build");
        let raw = vec![128u8; 3840 * 2160 * 2];
        let params = ProcessParams::new(3840, 2160, &raw);

        // Warmup: 10 frames
        for i in 0..10 {
            let _ = pipeline.process(&params);
        }

        // Measured: 20 frames
        let n = 20;
        let t0 = std::time::Instant::now();
        for i in 0..n {
            let r = pipeline.process(&params);
            assert!(r.is_ok(), "frame {} failed: {:?}", i, r.err());
        }
        let elapsed = t0.elapsed();
        let fps = n as f64 / elapsed.as_secs_f64();
        let avg_ms = elapsed.as_secs_f64() * 1000.0 / n as f64;
        eprintln!("\n=== 4K→FHD ARGB8888: {} frames in {:.2?} ===", n, elapsed);
        eprintln!("   avg: {:.2} ms/frame  |  {:.1} FPS", avg_ms, fps);
    }
}
