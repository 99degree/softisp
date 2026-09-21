//! Bridge: dummy Bayer in pipeline output to ANativeWindow display.
//!
//! Wraps ISP engine end-to-end with synthetic input and renders
//! the resulting frame to the cached ANativeWindow from `jni.rs`.
use jni::prelude::*;
use std::sync::OnceLock;

use crate::engine::{CpuEngine, IspEngine, ProcessParams};
use crate::pipeline::IspFrame;

/// Lazy singleton ISP engine (CPU backend).
static ENGINE: OnceLock<std::sync::Mutex<Option<Box<dyn IspEngine>>>> = OnceLock::new();

/// Get or lazily initialize the ISP engine.
fn engine() -> std::sync::MutexGuard<'static, Option<Box<dyn IspEngine>>> {
    ENGINE
        .get_or_init(|| {
            let mut eng = CpuEngine::new();
            let _ = eng.build(
                Box::new(crate::blocks::DisplayBlock::new(0)),
                vec![],
                None,
                14,
            );
            Some(Box::new(eng) as Box<dyn IspEngine>)
        })
        .lock()
        .expect("engine mutex poisoned")
        .clone()
}

/// Generate dummy Bayer RGGB frame (high-frequency pattern).
pub fn dummy_bayer_rggb(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    buf.extend_from_slice(&crate::synth_bayer::bayer_stress(width, height));
    let expected = (width * height * 2) as usize;
    if buf.len() < expected {
        let extra = vec![0x80u8, 0x80u8; expected - buf.len()];
        buf.extend_from_slice(&extra);
    }
    buf
}

/// Process a dummy frame through the ISP pipeline.
pub fn process_dummy_frame(
    width: u32,
    height: u32,
) -> crate::error::IspResult<IspFrame> {
    let bayer = dummy_bayer_rggb(width, height);
    let params = ProcessParams::new(width, height, &bayer);
    let eng = engine();
    eng.process(&params)
}

/// Render an IspFrame to the cached ANativeWindow (placeholder — actual
/// buffer swap happens on the Java side via Surface.lockCanvas).
pub fn render_frame_to_window(_frame: &IspFrame) -> jni::errors::Result<()> {
    let _env = crate::jni::attach_jvm().ok_or_else(|| {
        jni::errors::Error::from(jni::errors::LastError::from(
            "Failed to attach JVM for rendering",
        ))
    })?;
    let _window = crate::jni::native_window().ok_or_else(|| {
        jni::errors::Error::from(jni::errors::LastError::from(
            "No native window set",
        ))
    })?;
    // Frame data is available; actual draw is JNI-side.
    Ok(())
}

/// Full pipeline step: dummy Bayer → process → render.
pub fn pipeline_step(width: u32, height: u32) -> crate::error::IspResult<()> {
    let frame = process_dummy_frame(width, height)?;
    render_frame_to_window(&frame)?;
    Ok(())
}
