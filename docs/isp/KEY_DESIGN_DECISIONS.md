# Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| `SimdEngine` trait + `OnceLock` | Zero-cost dispatch, no compile-time flags |
| `#[target_feature]` for fp16 | Compiles on any CPU, runtime `is_aarch64_feature_detected!` gates |
| Triple-buffered RollingStats | Lock-free: engine writes while controller processes |
| ONNX dual-registration | Graph inputs overridable at runtime (CCM/tone/WB) |
| pack_rgba INT32 default | Single i32 per pixel = efficient memory + chaining |
| Debug-level per-frame logs | No info-level noise from frame timing |
| `mnn_buffer` behind feature flag | Isolates pre-existing compilation errors |
| LDCI split: `isp.ldci` (6-op, clip) / `isp.ldci_a` (4-op, no clip) | Independent exact patterns → independent SPIR-V shaders → independent tuning |
