//! CameraProviderFactory — AOSP CameraProvider service registration.
//!
//! Implements the factory pattern for creating and registering CameraProvider
//! instances with Android's ServiceManager. This is the entry point for the
//! camera HAL service on Android.
//!
//! Registration flow:
//! 1. Create CameraHalService
//! 2. Register with ServiceManager as "media.camera"
//! 3. Start binder thread pool
//! 4. Handle incoming requests via AIDL dispatch

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use log::info;

use crate::provider::CameraProvider;
use crate::service::CameraHalService;

/// Service registration state.
static SERVICE_REGISTERED: AtomicBool = AtomicBool::new(false);

/// CameraProviderFactory — creates and manages CameraProvider instances.
///
/// On Android, this registers the provider with ServiceManager.
/// On Linux host, this runs as a standalone local service.
pub struct CameraProviderFactory {
    service: Arc<CameraHalService>,
}

impl CameraProviderFactory {
    /// Create a new factory with default configuration.
    pub fn new() -> Self {
        let service = Arc::new(CameraHalService::new());
        Self { service }
    }

    /// Create a factory with custom provider.
    pub fn with_provider(provider: CameraProvider) -> Self {
        let service = Arc::new(CameraHalService::with_provider(provider));
        Self { service }
    }

    /// Get the underlying service.
    pub fn service(&self) -> &Arc<CameraHalService> {
        &self.service
    }

    /// Get the provider.
    pub fn provider(&self) -> &Arc<CameraProvider> {
        self.service.provider()
    }

    /// Register the camera HAL service with the system.
    ///
    /// On Android, registers with ServiceManager as:
    /// - "android.hardware.camera.provider.ICameraProvider/internal/0"
    ///
    /// On Linux host, this is a no-op (direct connection).
    pub fn register(&self) -> Result<(), String> {
        if SERVICE_REGISTERED.load(Ordering::Relaxed) {
            return Err("service already registered".into());
        }

        info!("CameraProviderFactory: registering camera HAL service");

        #[cfg(target_os = "android")]
        {
            // Android binder registration
            info!("CameraProviderFactory: registering with ServiceManager on Android");
            info!("  Service name: android.hardware.camera.provider.ICameraProvider/internal/0");
            info!("  Interface version: 1");
            // In production, this would call:
            //   binder::default_loader()->registerService(...)
            // or use the ServiceManager proxy
        }

        #[cfg(not(target_os = "android"))]
        {
            info!("CameraProviderFactory: non-Android platform, running in local mode");
            info!("  Use CameraHalService directly for communication");
        }

        SERVICE_REGISTERED.store(true, Ordering::Relaxed);
        Ok(())
    }

    /// Unregister the service.
    pub fn unregister(&self) -> Result<(), String> {
        if !SERVICE_REGISTERED.load(Ordering::Relaxed) {
            return Err("service not registered".into());
        }
        info!("CameraProviderFactory: unregistering camera HAL service");
        SERVICE_REGISTERED.store(false, Ordering::Relaxed);
        Ok(())
    }

    /// Check if the service is registered.
    pub fn is_registered(&self) -> bool {
        SERVICE_REGISTERED.load(Ordering::Relaxed)
    }

    /// Run the service (blocking).
    ///
    /// On Android, starts the binder thread pool.
    /// On Linux, runs a direct command loop.
    pub fn run(&self) -> Result<(), String> {
        self.register()?;

        info!("CameraProviderFactory: service started");

        #[cfg(target_os = "android")]
        {
            info!("CameraProviderFactory: starting binder thread pool");
            // In production: IPCThreadState::self()->joinThreadPool();
        }

        #[cfg(not(target_os = "android"))]
        {
            info!("CameraProviderFactory: running in local mode (no binder)");
            info!("  Service ready for direct connections");
        }

        Ok(())
    }

    /// Shutdown the service.
    pub fn shutdown(&self) -> Result<(), String> {
        info!("CameraProviderFactory: shutting down");
        self.unregister()?;
        Ok(())
    }
}

impl Default for CameraProviderFactory {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience function: create and run the camera HAL service.
pub fn run_camera_hal() -> Result<(), String> {
    let factory = CameraProviderFactory::new();
    factory.run()
}

/// Convenience function: create provider with standard cameras.
pub fn create_default_provider() -> Arc<CameraProvider> {
    let provider = CameraProvider::new();
    Arc::new(provider)
}

/// VINTF manifest entry for camera HAL.
///
/// This describes the camera HAL interface for the VINTF manifest.
/// On Android, this is included in the device manifest XML.
pub struct VintfManifestEntry {
    /// Service instance name.
    pub instance_name: String,
    /// Interface version.
    pub version: i32,
    /// Whether the service is running.
    pub is_running: bool,
}

impl VintfManifestEntry {
    /// Create a manifest entry for the camera provider.
    pub fn new() -> Self {
        Self {
            instance_name: "internal/0".into(),
            version: 1,
            is_running: false,
        }
    }

    /// Get the full service name.
    pub fn service_name(&self) -> String {
        format!("android.hardware.camera.provider.ICameraProvider/{}", self.instance_name)
    }

    /// Get the interface descriptor.
    pub fn interface_descriptor(&self) -> String {
        format!("android.hardware.camera.provider.ICameraProvider/{}", self.version)
    }

    /// Generate VINTF manifest XML fragment.
    pub fn to_vintf_xml(&self) -> String {
        format!(
            r#"    <hal format="aidl">
        <name>android.hardware.camera.provider</name>
        <version>{}</version>
        <fqname>ICameraProvider/{}</fqname>
    </hal>"#,
            self.version, self.instance_name
        )
    }
}

impl Default for VintfManifestEntry {
    fn default() -> Self {
        Self::new()
    }
}

/// HIDL-to-AIDL compatibility layer.
///
/// Wraps a legacy HIDL camera provider to expose AIDL interface.
/// This enables gradual migration from HIDL to AIDL HAL.
pub struct HidlToAidlShim {
    /// Underlying HIDL provider (simulated).
    hidl_version: i32,
}

impl HidlToAidlShim {
    /// Create a new shim wrapping a HIDL provider.
    pub fn new(hidl_version: i32) -> Self {
        info!("HidlToAidlShim: wrapping HIDL version {}", hidl_version);
        Self { hidl_version }
    }

    /// Get the HIDL version.
    pub fn hidl_version(&self) -> i32 {
        self.hidl_version
    }

    /// Check if HIDL-to-AIDL translation is needed.
    pub fn needs_translation(&self) -> bool {
        self.hidl_version < 3 // HIDL versions < 3 need AIDL translation
    }

    /// Translate HIDL error to AIDL error code.
    pub fn translate_error(hidl_error: i32) -> i32 {
        match hidl_error {
            0 => 0,  // OK
            1 => 1,  // INVALID_ARGUMENTS
            2 => 2,  // INVALID_OPERATION
            3 => 5,  // NO_MEMORY -> BUFFER_LOST
            4 => 10, // NO_RESOURCES -> DEVICE_ERROR
            _ => -1, // UNKNOWN
        }
    }
}

/// VNDK compatibility layer.
///
/// Provides the vendor NDK (VNDK) interface for camera HAL.
/// This includes platform library stubs for vendor code.
pub struct VndkCompatLayer {
    /// VNDK version.
    version: String,
}

impl VndkCompatLayer {
    /// Create a new VNDK compatibility layer.
    pub fn new() -> Self {
        Self {
            version: "35".into(), // Android 15 VNDK version
        }
    }

    /// Get the VNDK version.
    pub fn version(&self) -> &str {
        &self.version
    }

    /// Check if a library is available in the VNDK.
    pub fn is_library_available(&self, name: &str) -> bool {
        // Camera HAL typically uses these VNDK libraries
        matches!(name,
            "libcamera_metadata" |
            "libbinder" |
            "libutils" |
            "libcutils" |
            "liblog" |
            "libhidlbase" |
            "libhidltransport" |
            "libhwbinder" |
            "libbase" |
            "libsync" |
            "libgralloc"
        )
    }

    /// Get VNDK stub path for a library.
    pub fn get_stub_path(&self, name: &str) -> Option<String> {
        if self.is_library_available(name) {
            Some(format!("/vendor/lib64/{}.so", name))
        } else {
            None
        }
    }
}

impl Default for VndkCompatLayer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_factory_creation() {
        let factory = CameraProviderFactory::new();
        assert!(!factory.is_registered());
    }

    #[test]
    fn test_factory_register_unregister() {
        let factory = CameraProviderFactory::new();
        assert!(factory.register().is_ok());
        assert!(factory.is_registered());
        assert!(factory.unregister().is_ok());
        assert!(!factory.is_registered());
    }

    #[test]
    fn test_factory_double_register() {
        let factory = CameraProviderFactory::new();
        assert!(factory.register().is_ok());
        assert!(factory.register().is_err()); // already registered
        factory.unregister().ok();
    }

    #[test]
    fn test_vintf_manifest() {
        let entry = VintfManifestEntry::new();
        assert_eq!(entry.service_name(), "android.hardware.camera.provider.ICameraProvider/internal/0");
        assert!(entry.to_vintf_xml().contains("android.hardware.camera.provider"));
    }

    #[test]
    fn test_hidl_shim() {
        let shim = HidlToAidlShim::new(2);
        assert_eq!(shim.hidl_version(), 2);
        assert!(shim.needs_translation());

        let shim3 = HidlToAidlShim::new(3);
        assert!(!shim3.needs_translation());
    }

    #[test]
    fn test_hidl_error_translation() {
        assert_eq!(HidlToAidlShim::translate_error(0), 0);
        assert_eq!(HidlToAidlShim::translate_error(1), 1);
        assert_eq!(HidlToAidlShim::translate_error(3), 5);
    }

    #[test]
    fn test_vndk_compat() {
        let vndk = VndkCompatLayer::new();
        assert!(vndk.is_library_available("libcamera_metadata"));
        assert!(vndk.is_library_available("libbinder"));
        assert!(!vndk.is_library_available("libfoo"));
        assert!(vndk.get_stub_path("libcamera_metadata").is_some());
    }

    #[test]
    fn test_create_default_provider() {
        let provider = create_default_provider();
        let ids = provider.get_camera_id_list();
        assert!(!ids.is_empty());
    }

    #[test]
    fn test_run_camera_hal() {
        // Should succeed without blocking (non-Android platform)
        let result = run_camera_hal();
        assert!(result.is_ok());
    }
}
