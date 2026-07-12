//! Bridge between camera adapter and ISP pipeline.
//!
//! Creates a `FrameProcessor` that routes raw camera frames through
//! `IspEngine` and can be attached to `AndroidCameraAdapter`.

use cam_isp::engine::select_engine;
use log::info;
use std::sync::Arc;
use std::sync::Mutex;

/// Frame processing function type (matches AndroidCameraAdapter expectation).
pub type FrameProcessor =
    Arc<dyn Fn(&[u8], u32, u32, i32) -> Result<Vec<u8>, String> + Send + Sync>;

/// Build a default ISP pipeline for raw-to-RGBA conversion and return
/// a `FrameProcessor` closure — no Android dependency.
///
/// Pipeline chain:
///   RawInput → Normalize → CFA → BLC → BayerWB → Demosaic → CCM → Tone → Display
pub fn create_isp_processor(
    width: u32,
    height: u32,
    target_width: u32,
) -> Result<FrameProcessor, cam_isp::error::IspError> {
    use cam_isp::blocks;

    // 1. Build default block chain
    let raw_input = Box::new(blocks::RawInputBlock::new());
    let normalize = Box::new(blocks::NormalizeBlock::new());
    let cfa = Box::new(blocks::CfaBlock::new());
    let blc = Box::new(blocks::BlcBlock::new());
    let bayer_wb = Box::new(blocks::BayerWbBlock::new());
    let demosaic = Box::new(blocks::DemosaicBlock::new(2)); // GBRG pattern
    let ccm = Box::new(blocks::CcmBlock::new());
    let tone = Box::new(blocks::ToneBlock::new());
    let display = Box::new(blocks::DisplayBlock::new(target_width));

    let aux: Vec<Box<dyn cam_isp::pipeline::IspBlock>> =
        vec![normalize, cfa, blc, bayer_wb, demosaic, ccm, tone, display];

    // 2. Select and build engine
    let mut engine = select_engine().ok_or_else(|| "No ISP engine available".to_string())?;

    info!(
        "Building ISP pipeline with engine: {}, target {}x{}",
        engine.backend_name(),
        width,
        height
    );

    engine.build(raw_input, aux, None, 21)?;

    // 3. Wrap engine in Arc<Mutex<>> for shared ownership with the closure
    let engine = Arc::new(Mutex::new(Some(engine)));

    // 4. Create processor closure
    #[allow(unused_imports)]
    use cam_isp::engine::IspEngine;
    let processor: FrameProcessor = Arc::new(
        move |data: &[u8], w: u32, h: u32, _fmt: i32| -> Result<Vec<u8>, String> {
            let proc_start = std::time::Instant::now();
            let mut guard = engine.lock().map_err(|e| format!("Lock failed: {}", e))?;
            let eng = guard.as_mut().ok_or_else(|| "Engine taken".to_string())?;
            let mut params = cam_isp::engine::ProcessParams::new(w, h, data);
            params.target_width = target_width;
            params.sensor_max = 65535.0;
            params.timestamp_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0);
            let result = eng.process(&params).map_err(|e| e.to_string())?;
            let proc_elapsed = proc_start.elapsed();
            log::trace!(
                "ISP process: {}x{} -> {} bytes in {:.2}ms",
                w,
                h,
                result.data.len(),
                proc_elapsed.as_secs_f64() * 1000.0
            );
            Ok(result.data)
        },
    );

    info!("ISP FrameProcessor created for {}x{}", width, height);
    Ok(processor)
}

/// Attach an ISP processor to an AndroidCameraAdapter.
/// Only available when `cam-hal-android` is enabled (android feature).
#[cfg(feature = "android")]
pub fn attach_isp_to_android_adapter(
    adapter: &mut ::cam_hal_android::adapter::AndroidCameraAdapter,
    width: u32,
    height: u32,
    target_width: u32,
) -> Result<(), String> {
    let processor = create_isp_processor(width, height, target_width).map_err(|e| e.to_string())?;
    adapter.set_processor(processor);
    info!("ISP processor attached to AndroidCameraAdapter");
    Ok(())
}
