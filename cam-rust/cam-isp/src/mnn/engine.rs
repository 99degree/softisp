//! MnnEngine Vulkan workgroup tuning.
use crate::mnn_sys::*;
use std::sync::Arc;

impl MnnEngine {
    /// Set preferred workgroup size for a session.
    /// Only effective for Vulkan backend.
    pub fn set_workgroup_size(&self, session: &MnnSessionPtr, size_x: u32, size_y: u32) {
        unsafe {
            let backend = mnn_get_backend(self.handle, 0); // 0 = Vulkan
            mnn_backend_set_shader_workgroup_size(backend, session.handle, size_x as i32, size_y as i32);
        }
    }
}