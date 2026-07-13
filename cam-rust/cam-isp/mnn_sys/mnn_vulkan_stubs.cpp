// Weak stubs for custom MNN Vulkan ISP functions.
// These are only present in our custom MNN build (with ISP modifications).
// When building against official MNN 3.6.0, these stubs provide no-op defaults.

extern "C" {

// Query optimal workgroup size for current GPU.
// Stub: returns 1,1 (no optimization).
__attribute__((weak))
void MNNVulkanQueryOptimalWorkgroup(int* out_x, int* out_y) {
    if (out_x) *out_x = 1;
    if (out_y) *out_y = 1;
}

// Set preferred workgroup size for a Vulkan session.
// Stub: no-op.
__attribute__((weak))
void MNNVulkanSetSessionWorkgroup(void* session, int size_x, int size_y) {
    (void)session; (void)size_x; (void)size_y;
}

// Set workgroup preset by name.
// Stub: no-op.
__attribute__((weak))
void MNNVulkanSetWorkgroupPreset(const char* preset_name) {
    (void)preset_name;
}

// Hot-swap a const buffer at runtime for live 3A adjustments.
// Stub: no-op.
__attribute__((weak))
void MNNVulkanHotSwapConstBuffer(void* session, const char* name,
                                  const void* data, int size) {
    (void)session; (void)name; (void)data; (void)size;
}

}  // extern "C"
