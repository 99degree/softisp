//! Main IPA context managing the full pipeline

use crate::{
    Buffer, BufferManager, ControlInfoMap, ControlList, EngineManager, EventEmitter, IpaCallbacks,
    IpaConfig, IpaConfigureInfo, IpaController, IpaError, IpaFormat, IpaInitResult, IpaResult,
    IpaSensorInfo, IpaSettings,
};
use cam_isp::{
    engine::{OutputFormat, ProcessParams},
    isp_params::IspParams,
    pipeline::IspFrame,
};
use cam_types::ToneParams;
use std::path::Path;

/// Main IPA context managing the full pipeline
pub struct IpaContext {
    config: IpaConfig,
    engine: EngineManager,
    controller: IpaController,
    buffers: BufferManager,
    events: EventEmitter,
    initialized: bool,
    started: bool,
    image_width: u32,
    image_height: u32,
    target_format: IpaFormat,
    num_output_buffers: u32,
    pending_frames: Vec<u32>,
    frame_controls: std::collections::HashMap<u32, ControlList>,
}

impl IpaContext {
    /// Create a new IPA context
    pub fn new(config: IpaConfig) -> IpaResult<Self> {
        let engine = EngineManager::new(config.clone())?;
        let controller = IpaController::new();
        let buffers = BufferManager::new();
        let events = EventEmitter::new();

        Ok(Self {
            image_width: config.image_width,
            image_height: config.image_height,
            target_format: config.format,
            num_output_buffers: config.num_output_buffers,
            config,
            engine,
            controller,
            buffers,
            events,
            initialized: false,
            started: false,
            pending_frames: Vec::new(),
            frame_controls: std::collections::HashMap::new(),
        })
    }

    /// Initialize the IPA with settings and sensor info
    pub fn init(
        &mut self,
        _settings: &IpaSettings,
        sensor_info: &IpaSensorInfo,
    ) -> IpaResult<IpaInitResult> {
        if self.initialized {
            return Err(IpaError::AlreadyInitialized.into());
        }

        // Update dimensions from sensor info
        self.image_width = sensor_info.output_size.width;
        self.image_height = sensor_info.output_size.height;

        // Build the engine pipeline
        self.engine.build()?;

        // Load models from model_dir if specified
        if let Some(model_dir) = self.config.model_dir.clone() {
            self.load_models(&model_dir)?;
        }

        self.initialized = true;

        // Return init result with default controls
        Ok(IpaInitResult {
            ipa_controls: ControlInfoMap::default(),
            ccm_enabled: true,
        })
    }

    /// Load models from directory
    fn load_models(&mut self, _model_dir: &Path) -> IpaResult<()> {
        // Model loading is handled by MnnEngine internally
        // The engine will look for models when needed
        log::info!("Model directory: {:?}", _model_dir);
        Ok(())
    }

    /// Start the IPA processing
    pub fn start(&mut self) -> IpaResult<()> {
        if !self.initialized {
            return Err(IpaError::NotInitialized.into());
        }
        if self.started {
            return Ok(());
        }

        // Start event loop
        self.events.start(IpaCallbacks::default());

        self.started = true;
        log::info!("IPA started");
        Ok(())
    }

    /// Stop the IPA processing
    pub fn stop(&mut self) {
        if !self.started {
            return;
        }

        self.events.shutdown();
        self.started = false;
        log::info!("IPA stopped");
    }

    /// Configure the IPA
    pub fn configure(&mut self, config: &IpaConfigureInfo) -> IpaResult<()> {
        // Convert ControlInfoMap to ControlList for controller
        let ctrl_list = ControlList::from_info_map(&config.sensor_controls);
        self.controller.update_from_controls(&ctrl_list);
        Ok(())
    }

    /// Configure frame backend (output format and buffer count)
    pub fn configure_frame_backend(
        &mut self,
        format: IpaFormat,
        num_buffers: u32,
    ) -> IpaResult<()> {
        self.target_format = format;
        self.num_output_buffers = num_buffers;
        Ok(())
    }

    /// Queue a request for processing
    pub fn queue_request(&mut self, frame: u32, controls: &ControlList) -> IpaResult<()> {
        if !self.started {
            return Err(IpaError::NotInitialized.into());
        }

        self.pending_frames.push(frame);
        self.frame_controls.insert(frame, controls.clone());

        // Update controller with sensor controls
        self.controller.update_from_controls(controls);

        // Emit set sensor controls callback
        self.events.emit_set_sensor_controls(controls.clone());

        Ok(())
    }

    /// Compute parameters for a frame (3A update)
    pub fn compute_params(&mut self, _frame: u32) -> IpaResult<()> {
        if !self.started {
            return Err(IpaError::NotInitialized.into());
        }

        // Create a dummy frame for controller analysis
        // In real use, this would use actual frame data
        let dummy_frame = IspFrame {
            params: IspParams::default(),
            width: self.image_width,
            height: self.image_height,
            data: vec![128; (self.image_width * self.image_height * 2) as usize],
            format: cam_types::FrameFormat::RawSensor,
            float_data: None,
            aux: None,
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
            external_data: None,
        };

        // Run 3A analysis
        let _params = self.controller.analyze_and_update(&dummy_frame);

        // Emit set ISP params callback
        self.events.emit_set_isp_params();

        Ok(())
    }

    /// Process statistics buffer
    pub fn process_stats(
        &mut self,
        _frame: u32,
        _buffer_id: u32,
        _buffer: &dyn Buffer,
    ) -> IpaResult<ControlList> {
        if !self.started {
            return Err(IpaError::NotInitialized.into());
        }

        // For now, return mock stats
        // Real implementation would run stats inference via engine
        let mut metadata = ControlList::new();
        metadata.set_f32_array("ColourGains", &[1.5, 1.2]);
        metadata.set_i32("ExposureTime", 2000);
        metadata.set_f32("AnalogueGain", 2.5);

        // Update controller with stats
        self.controller.update_from_controls(&metadata);

        // Emit metadata ready
        self.events.emit_metadata_ready(0, metadata.clone());

        Ok(metadata)
    }

    /// Process a frame through the full ISP pipeline
    pub fn process_frame(
        &mut self,
        frame: u32,
        buffer_id: u32,
        input: &mut dyn Buffer,
        output: &mut dyn Buffer,
        plane_in: i32,
        plane_out: i32,
        width: u32,
        height: u32,
        results: &ControlList,
    ) -> IpaResult<()> {
        if !self.started {
            return Err(IpaError::NotInitialized.into());
        }

        // Update controller with result controls
        self.controller.update_from_controls(results);

        // Prepare process parameters
        let mut params = ProcessParams::new(width, height, &[]);
        params.target_width = width;
        params.target_height = height;
        params.output_format = self.ipa_format_to_output_format(self.target_format);
        params.timestamp_ns = 0; // Would be set from frame timing

        // Set ISP parameters from controller
        let isp_params = self.controller.get_params();
        let awb_gains = self.controller.get_awb_gains();
        let analog_gain = self.controller.get_analog_gain();
        let ctrl_tone = isp_params.tone.clone();
        params.ccm_matrix = Some(isp_params.ccm.matrix);
        params.awb_gains = Some(awb_gains);

        // Convert cam_isp ToneParams to cam_types ToneParams
        params.tone_params = ToneParams {
            contrast: ctrl_tone.contrast,
            brightness: ctrl_tone.brightness,
            gamma_recip: 1.0 / ctrl_tone.gamma.max(0.1),
            ..ToneParams::default()
        };

        params.bayer_pattern = 0; // RGGB default
        params.analog_gain = analog_gain;
        params.isp_params = Some(isp_params);

        // Set zero-copy buffers
        params.input_buffer = Some((input.as_mut_ptr(), input.len()));
        params.output_buffer = Some((output.as_mut_ptr(), output.len()));

        // Handle AHardwareBuffer for Vulkan zero-copy
        if input.is_ahb() {
            params.input_ahb = input.ahb_ptr().map(|p| (p, input.len()));
        }
        if output.is_ahb() {
            params.output_ahb = output.ahb_ptr().map(|p| (p, output.len()));
        }

        // Process frame
        let _result = self.engine.process(params)?;

        // Emit frame done
        self.events.emit_frame_done(frame, buffer_id);

        // Remove from pending
        self.pending_frames.retain(|&f| f != frame);
        self.frame_controls.remove(&frame);

        Ok(())
    }

    /// Set callbacks for async events
    pub fn set_callbacks(&mut self, callbacks: IpaCallbacks) {
        self.events = EventEmitter::new();
        self.events.start(callbacks);
    }

    /// Convert IpaFormat to OutputFormat
    fn ipa_format_to_output_format(&self, format: IpaFormat) -> OutputFormat {
        match format {
            IpaFormat::RGB => OutputFormat::Rgb,
            IpaFormat::RGBA => OutputFormat::Rgba,
            IpaFormat::BGRA => OutputFormat::Bgra,
            IpaFormat::ARGB => OutputFormat::Argb,
            IpaFormat::ABGR => OutputFormat::Abgr,
            IpaFormat::YUV | IpaFormat::NV12 | IpaFormat::P010 => OutputFormat::Bgra,
        }
    }

    /// Get current configuration
    pub fn config(&self) -> &IpaConfig {
        &self.config
    }

    /// Check if initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Check if started
    pub fn is_started(&self) -> bool {
        self.started
    }
}

impl Drop for IpaContext {
    fn drop(&mut self) {
        self.stop();
    }
}
