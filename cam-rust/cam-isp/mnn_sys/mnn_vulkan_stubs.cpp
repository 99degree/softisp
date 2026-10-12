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

// Ensure BinaryAddToEltwise merge pass registration is initialized.
// This symbol is referenced by MNNConvertDeps via PostConverter.cpp.
// The converter's CMake builds with -fvisibility=hidden, so the symbol
// from MNN_Express (built with default visibility) may not be visible.
// Provide a STRONG C++ definition here (with C++ linkage so it gets the
// correct mangling _Z34ensureBinaryAddToEltwiseRegisteredv) so the
// dynamic linker can resolve it for MNNConvertDeps at load time.
// The actual pass body is a no-op (see MNN express/BinaryAddToEltwise.cpp:110-118)
// because the ISP pass0 pipeline never emits Int8ToFloat+Convert+ADD patterns.
__attribute__((visibility("default")))
void ensureBinaryAddToEltwiseRegistered() {
    // Intentional no-op - the merge pass is not needed for ISP pass0
}

// Also provide the namespaced version (C++ linkage) for direct calls
namespace MNN {
namespace Express {
void ensureBinaryAddToEltwiseRegistered() {
    // Intentional no-op - the merge pass is not needed for ISP pass0
}
} // namespace Express
} // namespace MNN
