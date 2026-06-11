//! Android Camera HAL Binder Service.
//!
//! Registers as `android.hardware.camera.provider.ICameraProvider` with the
//! Android service manager. Provides camera device binders that can use V4L2
//! capture via cam-hal-linux.
//!
//! ## Building
//! ```bash
//! cargo build --features android -p cam-binder
//! ```
//! Requires Android NDK with `libbinder_ndk.so`.

#[cfg(feature = "android")]
pub mod provider;
#[cfg(feature = "android")]
pub mod device;
#[cfg(feature = "android")]
pub mod session;

use log::info;

#[cfg(feature = "android")]
use rsbinder::{ServiceManager, Binder};

/// Register the camera HAL service with the Android service manager.
#[cfg(feature = "android")]
pub fn register_service() -> Result<(), String> {
    let provider = provider::CameraProviderService::new();
    let binder = Binder::new(provider);
    ServiceManager::add_service("media.camera", binder)
        .map_err(|e| format!("Failed to register camera service: {:?}", e))?;
    info!("Camera HAL service registered as 'media.camera'");
    Ok(())
}

#[cfg(not(feature = "android"))]
pub fn register_service() -> Result<(), String> {
    info!("Binder service registration skipped (android feature disabled)");
    Ok(())
}
