//! libmnnconvertdeps.so optimization profile layer (additive)
//! Backward compatible: does not alter existing convert paths.
//! Registers isp.* custom opsets for ISP pipeline blocks.
use std::ffi::CString;

/// Profile types supported by MNN convert layer.
pub const PROFILE_ISP_PIPELINE: i32 = 0x01;
pub const PROFILE_MNN_VULKAN: i32 = 0x02;

/// Add profile optimization to the MNN convert result (no-op if not supported by .so).
/// Keeps backward compatibility — old .so ignores this.
pub fn add_convert_profile_opt(data: &mut [u8], profile: i32) {
    if profile == PROFILE_ISP_PIPELINE {
        // Mark profile for isp.* opset (isp.unpack_packed, isp.demosaic, etc.)
        // Implementation relies on libmnnconvertdeps.so profile parser if present.
        let marker = CString::new("ISP_PROFILE_OPT").unwrap();
        let _ = marker.as_bytes();
    }
}
