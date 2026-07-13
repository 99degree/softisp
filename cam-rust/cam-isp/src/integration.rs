//! HAL-ISP Integration Module
//!
//! Provides complete integration between:
//! - Camera HAL (Android Camera2 / V4L2)
//! - ISP Pipeline (CPU / MNN-Vulkan / ONNX Runtime)
//! - Memory Management (CMA / ION / memfd / CMA)
//! - Zero-copy buffer passing

use crate::engine::{select_engine, select_engine_by_name, IspEngine, ProcessParams};
use crate::mnn_buffer::cma_buffer::{BufferAlignment, BufferUsage, CMAAllocator, CMABufferManager, CMABuffer};
use log::{error, info, warn};
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

// Type alias for frame processor callback
pub type IspFrameProcessor = Box<dyn Fn(&[u8], u32, u32, i32) -> Result<Vec<u8>, String> + Send + Sync>;

// ─── Zero-Copy Buffer Management ───────────────────────────────────────────

/// Camera input buffer (raw Bayer from MIPI/V4L2)
#[derive(Debug, Clone)]
pub struct CameraInputBuffer {
    pub ptr: *mut u8,
    pub size: usize,
    pub width: u32,
    pub height: u32,
    pub bits_per_pixel: u8,
    pub cma_buffer: Option<Arc<crate::mnn_buffer::cma_buffer::CMABuffer>>,
}

/// ISP output buffer (processed RGB/YUV)
#[derive(Debug, Clone)]
pub struct IspOutputBuffer {
    pub ptr: *mut u8,
    pub size: usize,
    pub width: u32,
    pub height: u32,
    pub format: i32, // HAL_PIXEL_FORMAT_*
    pub cma_buffer: Option<Arc<crate::mnn_buffer::cma_buffer::CMABuffer>>,
}

/// Zero-copy buffer manager for camera ↔ ISP ↔ MNN
pub struct ZeroCopyBufferManager {
    cma_manager: CMABufferManager,
}

impl ZeroCopyBufferManager {
    /// Create new zero-copy buffer manager
    pub fn new(allocator: CMAAllocator, alignment: BufferAlignment) -> Self {
        let cma_manager = CMABufferManager::with_settings(allocator, alignment);
        Self {
            cma_manager,
        }
    }

    /// Allocate camera input buffer (raw Bayer)
    pub fn allocate_camera_input(
        &self,
        name: &str,
        width: u32,
        height: u32,
        bytes_per_pixel: u32,
    ) -> Result<CameraInputBuffer, String> {
        let size = (width * height * bytes_per_pixel) as usize;
        let usage = vec![BufferUsage::Camera, BufferUsage::CPU];

        let cma_buffer = self.cma_manager.allocate(name, size, &usage)
            .map_err(|e| format!("CMA allocation failed: {}", e))?;

        Ok(CameraInputBuffer {
            ptr: cma_buffer.as_ptr(),
            size,
            width,
            height,
            bits_per_pixel: (bytes_per_pixel * 8) as u8,
            cma_buffer: Some(cma_buffer),
        })
    }

    /// Allocate ISP output buffer (processed RGB/YUV)
    pub fn allocate_isp_output(
        &self,
        name: &str,
        width: u32,
        height: u32,
        bytes_per_pixel: u32,
    ) -> Result<IspOutputBuffer, String> {
        let size = (width * height * bytes_per_pixel) as usize;
        let usage = vec![BufferUsage::ISP, BufferUsage::CPU];

        let cma_buffer = self.cma_manager.allocate(name, size, &usage)
            .map_err(|e| format!("CMA allocation failed: {}", e))?;

        Ok(IspOutputBuffer {
            ptr: cma_buffer.as_ptr(),
            size,
            width,
            height,
            format: 0x1, // HAL_PIXEL_FORMAT_RGBA_8888
            cma_buffer: Some(cma_buffer),
        })
    }

    /// Get CMA manager for advanced operations
    pub fn cma_manager(&self) -> &CMABufferManager {
        &self.cma_manager
    }
}

// ─── Camera ISP Service ────────────────────────────────────────────────────

/// ISP service configuration
#[derive(Debug, Clone)]
pub struct CameraIspConfig {
    /// ISP engine name ("cpu", "mnn", "auto")
    pub engine: String,
    /// Sensor max value (e.g., 1023 for 10-bit)
    pub sensor_max: f32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR)
    pub bayer_pattern: i32,
    /// Target output format
    pub output_format: i32,
    /// Enable zero-copy buffers
    pub zero_copy: bool,
}

impl Default for CameraIspConfig {
    fn default() -> Self {
        Self {
            engine: "auto".to_string(),
            sensor_max: 1023.0,
            bayer_pattern: 0,
            output_format: 0x1, // RGBA_8888
            zero_copy: true,
        }
    }
}

/// High-level Camera ISP Service
pub struct CameraIspService {
    engine: Box<dyn IspEngine>,
    config: CameraIspConfig,
}

impl CameraIspService {
    /// Create new camera ISP service
    pub fn new(
        engine: &str,
        sensor_max: f32,
        bayer_pattern: i32,
    ) -> Result<Self, String> {
        let engine: Box<dyn IspEngine> = if engine == "auto" {
            select_engine().ok_or("No ISP engine available")?
        } else {
            select_engine_by_name(engine).ok_or_else(|| format!("ISP engine '{}' not found", engine))?
        };
        let engine_name = engine.backend_name().to_string();

        let config = CameraIspConfig {
            engine: engine_name.to_lowercase(),
            sensor_max,
            bayer_pattern,
            output_format: 0x1,
            zero_copy: true,
        };

        let mut service = Self {
            engine,
            config,
        };

        // Build a minimal pipeline for CPU engine if it's not yet loaded
        // (CPU engine requires build() to be called before process())
        if service.config.engine == "cpu" {
            if let Some(cpu_engine) = service.engine.as_any_mut().downcast_mut::<crate::cpu::CpuEngine>() {
                use crate::blocks::{RawInputBlock, NormalizeBlock, CfaBlock, BlcBlock, BayerWbBlock, DemosaicBlock, CcmBlock, ToneBlock, DisplayBlock};
                use crate::pipeline::IspBlock;

                let mut blocks: Vec<Box<dyn crate::pipeline::IspBlock>> = vec![
                    Box::new(RawInputBlock::new()),
                    Box::new(NormalizeBlock::new()),
                    Box::new(CfaBlock::new()),
                    Box::new(BlcBlock::new()),
                    Box::new(BayerWbBlock::new()),
                    Box::new(DemosaicBlock::new(0)),
                    Box::new(CcmBlock::new()),
                    Box::new(ToneBlock::new()),
                    Box::new(DisplayBlock::new(64)),
                ];

                let head = blocks.remove(0);
                let aux = blocks;
                let _ = cpu_engine.build(head, aux, None, 21);
            }
        }

        Ok(service)
    }

    /// Process a raw frame through the ISP pipeline
    pub fn process_raw_frame(
        &self,
        raw_data: &[u8],
        width: u32,
        height: u32,
        bayer_pattern: i32,
    ) -> Result<Vec<u8>, String> {
        let mut params = crate::engine::ProcessParams::new(width, height, raw_data);
        params.sensor_max = self.config.sensor_max;
        params.bayer_pattern = bayer_pattern;
        params.timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        let frame = self.engine.process(&params).map_err(|e| e.to_string())?;
        Ok(frame.data)
    }

    /// Process raw frame with zero-copy buffers
    pub fn process_raw_frame_zc(
        &mut self,
        input: &mut CameraInputBuffer,
        output: &mut IspOutputBuffer,
    ) -> Result<(), String> {
        let mut params = crate::engine::ProcessParams::new(input.width, input.height,
            unsafe { std::slice::from_raw_parts(input.ptr, input.size) });
        params.sensor_max = self.config.sensor_max;
        params.bayer_pattern = self.config.bayer_pattern;
        params.timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        let frame = self.engine.process(&params).map_err(|e| e.to_string())?;

        // Copy output to zero-copy buffer
        if frame.data.len() <= output.size {
            unsafe {
                std::ptr::copy_nonoverlapping(
                    frame.data.as_ptr(),
                    output.ptr,
                    frame.data.len(),
                );
            }
            Ok(())
        } else {
            Err(format!("Output buffer too small: need {}, have {}", frame.data.len(), output.size))
        }
    }

    /// Get the underlying engine for advanced use
    pub fn engine(&self) -> &dyn IspEngine {
        self.engine.as_ref()
    }
}

// ─── Android HAL ISP Bridge ────────────────────────────────────────────────

/// Bridge between Android Camera HAL and ISP pipeline
pub struct AndroidHalIspBridge {
    /// ISP service
    isp_service: Mutex<Option<CameraIspService>>,
    /// Current frame processor
    frame_processor: Mutex<Option<IspFrameProcessor>>,
}

impl AndroidHalIspBridge {
    /// Create new Android HAL ISP bridge
    pub fn new() -> Self {
        Self {
            isp_service: Mutex::new(None),
            frame_processor: Mutex::new(None),
        }
    }

    /// Initialize ISP service
    pub fn init(&self, engine: &str, sensor_max: f32, bayer_pattern: i32) -> Result<(), String> {
        let service = CameraIspService::new(engine, sensor_max, bayer_pattern)?;
        *self.isp_service.lock().unwrap() = Some(service);
        Ok(())
    }

    /// Set frame processor callback (called from Android HAL on new frame)
    pub fn set_frame_processor(&self, processor: IspFrameProcessor) {
        *self.frame_processor.lock().unwrap() = Some(processor);
    }

    /// Process a camera frame from Android HAL
    pub fn process_android_frame(
        &self,
        data: &[u8],
        width: u32,
        height: u32,
        format: i32,
    ) -> Result<Vec<u8>, String> {
        // Check if we have a custom processor
        if let Some(ref proc) = *self.frame_processor.lock().unwrap() {
            return (proc)(data, width, height, format);
        }

        // Use built-in ISP service
        if let Some(ref service) = *self.isp_service.lock().unwrap() {
            service.process_raw_frame(data, width, height, 0)
        } else {
            Err("ISP service not initialized".into())
        }
    }
}

impl Default for AndroidHalIspBridge {
    fn default() -> Self {
        Self::new()
    }
}

// ─── V4L2 ISP Bridge ──────────────────────────────────────────────────────

/// Bridge between V4L2 camera and ISP pipeline
pub struct V4l2IspBridge {
    /// ISP service
    isp_service: Mutex<Option<CameraIspService>>,
    /// Sensor specification
    sensor: Mutex<Option<SensorSpec>>,
}

/// Sensor specification for raw capture
#[derive(Debug, Clone)]
pub struct SensorSpec {
    pub name: String,
    pub width: u32,
    pub height: u32,
    pub bayer_pattern: BayerPattern,
    pub bit_depth: u8,
    pub frame_rate_hz: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BayerPattern {
    Rggb = 0,
    Grbg = 1,
    Gbrg = 2,
    Bggr = 3,
}

impl V4l2IspBridge {
    /// Create new V4L2 ISP bridge
    pub fn new() -> Self {
        Self {
            isp_service: Mutex::new(None),
            sensor: Mutex::new(None),
        }
    }

    /// Set sensor specification
    pub fn set_sensor(&self, spec: SensorSpec) {
        *self.sensor.lock().unwrap() = Some(spec);
    }

    /// Initialize ISP service
    pub fn init(&self, engine: &str, sensor_max: f32, bayer_pattern: i32) -> Result<(), String> {
        let service = CameraIspService::new(engine, sensor_max, bayer_pattern)?;
        *self.isp_service.lock().unwrap() = Some(service);
        Ok(())
    }

    /// Process a single already-captured V4L2 frame through the ISP.
    ///
    /// Real V4L2 capture is performed by `cam-hal-linux` (see
    /// `cam_hal_linux::v4l2::adapter`); this bridge only runs the ISP stage.
    pub fn process_frame(
        &self,
        raw_data: &[u8],
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, String> {
        let sensor = self.sensor.lock().unwrap().clone()
            .ok_or("Sensor not configured")?;

        let isp_service = self.isp_service.lock().unwrap();
        if let Some(ref service) = *isp_service {
            service.process_raw_frame(
                raw_data,
                width,
                height,
                sensor.bayer_pattern as i32,
            )
        } else {
            Err("ISP service not initialized".into())
        }
    }

    /// Process a batch of captured frames, accumulating statistics.
    pub fn process_batch(
        &self,
        frames: &[(Vec<u8>, u32, u32)],
    ) -> Result<BridgeStats, String> {
        let mut stats = BridgeStats::default();
        for (raw_data, w, h) in frames {
            match self.process_frame(raw_data, *w, *h) {
                Ok(_) => {
                    stats.frames_captured += 1;
                    stats.frames_processed += 1;
                }
                Err(e) => {
                    error!("Frame processing failed: {}", e);
                    stats.frames_dropped += 1;
                    if stats.frames_dropped > 5 {
                        return Err(e);
                    }
                }
            }
        }
        Ok(stats)
    }
}

impl Default for V4l2IspBridge {
    fn default() -> Self {
        Self::new()
    }
}

/// Bridge statistics
#[derive(Debug, Default, Clone)]
pub struct BridgeStats {
    pub frames_captured: u64,
    pub frames_processed: u64,
    pub frames_dropped: u64,
    pub avg_capture_us: f64,
    pub avg_processing_us: f64,
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cpu_engine_select() {
        crate::init();
        match crate::engine::select_engine_by_name("cpu") {
            Some(e) => println!("CPU engine found: {}", e.backend_name()),
            None => panic!("CPU engine NOT found"),
        }
    }

    #[test]
    fn test_camera_isp_service_creation() {
        crate::init();
        let service = CameraIspService::new("cpu", 1023.0, 0);
        match service {
            Ok(_) => println!("Service created OK"),
            Err(e) => panic!("Service creation failed: {}", e),
        }
    }

    #[test]
    fn test_camera_isp_service_process() {
        crate::init();
        let service = CameraIspService::new("cpu", 1023.0, 0).unwrap();
        let data = vec![128u8; 64 * 48 * 2];
        let result = service.process_raw_frame(&data, 64, 48, 0);
        if let Err(ref e) = result {
            eprintln!("ISP Error: {}", e);
        }
        assert!(result.is_ok());
    }

    #[test]
    fn test_zero_copy_buffer_manager() {
        let manager = ZeroCopyBufferManager::new(
            CMAAllocator::Malloc,
            BufferAlignment::Page,
        );

        let input = manager.allocate_camera_input("test_in", 1920, 1080, 2).unwrap();
        assert!(!input.ptr.is_null());
        assert_eq!(input.size, 1920 * 1080 * 2);

        let output = manager.allocate_isp_output("test_out", 1920, 1080, 4).unwrap();
        assert!(!output.ptr.is_null());
        assert_eq!(output.size, 1920 * 1080 * 4);
    }

    #[test]
    fn test_android_hal_bridge() {
        crate::init();
        let bridge = AndroidHalIspBridge::new();
        bridge.init("cpu", 1023.0, 0).unwrap();

        let data = vec![128u8; 64 * 48 * 2];
        let result = bridge.process_android_frame(&data, 64, 48, 0);
        assert!(result.is_ok());
    }

    #[test]
    fn test_v4l2_bridge() {
        crate::init();
        let bridge = V4l2IspBridge::new();
        bridge.set_sensor(SensorSpec {
            name: "test".into(),
            width: 1920,
            height: 1080,
            bayer_pattern: BayerPattern::Rggb,
            bit_depth: 10,
            frame_rate_hz: 30,
        });
        bridge.init("cpu", 1023.0, 0).unwrap();
    }
}