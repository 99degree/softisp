# Project Status

## Status (2026-07-21)

SoftISP is a complete camera ISP pipeline in Rust with Vulkan GPU acceleration via MNN.

### Verified (from `cargo test --lib -p cam-isp`)
- **813 lib tests pass**, 0 failed, 2 ignored
- **0 warnings** on `cargo clippy --workspace` (per-crate loop)
- MNN runtime libs present: `cam-rust/lib/aarch64-v8a/libMNN.so`, `libMNN_Vulkan.so`, `libvulkan.so`, `libMNNConvertDeps.so`

### Backend Availability

| Feature | Crate Flag | Status |
|---------|------------|--------|
| CPU (default) | — | ✅ Production |
| MNN Vulkan | `mnn` | ✅ Libraries present, needs `LD_LIBRARY_PATH` |
| ONNX Runtime | `ort` | ⚠️ `libonnxruntime.so` not installed |

### Key Deliverables
- 52 ISP blocks (input, BLC/WB, demosaic, CCM, tone, EE, LDCI, FCS, warp, stats, display)
- 5 pipeline profiles: LITE, MED, HEAVY, PRO, UNIFIED
- Dual controller: rule-based + neural (FiLM-ISP, ONNX)
- SIMD backends: NeonDotprod, NeonFp16, Neon, Scalar
- Triple-buffered RollingStats feedback loop
- pack_rgba INT32 default output
- MNNConvert static C FFI (no subprocess)
- ONNX TypeProto/TensorShapeProto encoding fix (prevents MNN DCE)
- WarpGridBlock grid + shading LUT as runtime `extra_inputs()`

### Performance (MNN Vulkan, Snapdragon-class)
- HEAVY profile 4K→4K: **79.8 FPS** avg (2394 frames / 30 s, P99 129.1 ms)
- UNIFIED profile 4K→4K (tiled 2×2): **141.2 FPS** avg (4236 frames / 30 s, P99 8.7 ms)

### Not Ported / Blocked
- ORT inference: needs `libonnxruntime.so`
- Android Camera2 NDK: needs NDK device
- HDR merge / AiDetect: needs ONNX models + runtime
- YUV processing: not supported in CpuEngine
