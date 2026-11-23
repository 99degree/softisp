# Progress

## ✅ Done
- All Java/Kotlin → Rust (48+ modules)
- SIMD backend (NeonDotprod/NeonFp16/Neon/Scalar)
- AdaptiveDownscaleBlock (fit/crop/pad, EIS margin)
- Triple-buffered RollingStats
- Fused orientation in DisplayBlock (rot90/180/270/hflip/vflip)
- Tone fused into DemosaicCcmBlock Conv
- pack_rgba INT32 default output
- MNNConvert static C FFI (no subprocess)
- All MNN libraries updated from `~/MNN/build`, including `libMNN_GL.so`
- MNN Vulkan GPU backend working (dlopen plugin + ICD loader fix)
- ONNX `TypeProto` / `TensorShapeProto` encoding fix (proto.rs): correct oneof submessage + Dimension repeat prevents MNN DCE of the ISP graph
- WarpGridBlock grid + shading LUT moved to runtime `extra_inputs()` (graph inputs, not initializers) — defeats MNN fold-to-identity
- `synth_bayer` module (5 generators) for stress-test inputs with high spatial frequency
- Pipeline tests: **813 pass**, 0 failed, 2 ignored
- HEAVY profile 4K→4K MNN stress test: **79.8 FPS** avg (2394 frames / 30s, P99 129.1ms)
- UNIFIED profile 4K→4K MNN stress test: **141.2 FPS** avg (4236 frames / 30s, P99 8.7ms, tiled 2×2)
- MNN converter: LDCI split — `isp.ldci` (6-op Pool+Clip) and `isp.ldci_a` (4-op Pool no clip) as independent exact patterns with own SPIR-V shaders
- MNN converter: removed 40+ dead `try*` helpers (~2500 lines), all ISP matching via exact pattern tables only

## ⏳ In Progress
- Numeric agreement ONNX stats ↔ software stats at FHD
- 30fps target at FHD (HEAVY 4K path: 79.8 FPS; UNIFIED 4K path: 141.2 FPS with tiles)

## 🚧 Blocked
- GPU zero-copy: needs device memory
- V4L2 on Android: `rscam` crate platform check
- 256-bin histogram ONNX block: OneHot → 2GB+ at FHD
- Content-aware inpainting: external ML models

## Not Ported

| Feature | Reason |
|---------|--------|
| ORT inference | Needs `libonnxruntime.so` |
| MNN full inference | ~~Needs `libMNN.so`~~ ✅ Working on Vulkan GPU |
| Android Camera2 NDK | Needs NDK device |
| HDR merge, ONNX AI blocks (AiDetect) | Needs ONNX models + runtime |
| YUV processing | Not supported in CpuEngine |
