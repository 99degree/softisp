# MNN Vulkan GPU Backend — Working

MNN runs on **Vulkan (Adreno GPU)** with real inference timing:

```
$ bench_mnn_gpu .mnn_bench_16958.mnn 5 50 2
  actual backend: Vulkan (7)
  avg:   17.949 ms  (55.7 FPS)
  model: 10.9 GFLOP, 5.8 MB
```

## Two issues fixed

1. **MNN_SEP_BUILD plugin loading**: MNN compiles backends as separate `.so` files. Each backend self-registers via a static initializer (`gResistor → MNNInsertExtraRuntimeCreator()`), but this only fires when the `.so` is `dlopen`'d. `libMNN.so` does NOT auto-load plugins. Fix: `ensure_vulkan_loaded()` calls `dlopen("libMNN_Vulkan.so")` in `mnn_session_create()`.

2. **`libvulkan.so` (ICD loader)**: MNN's `vulkan_wrapper.cpp` does `dlopen("libvulkan.so")` for driver function pointers. On Android the system driver is at `/system/lib64/libvulkan.so` but Termux doesn't search there. Fix: copied it to `cam-rust/lib/aarch64-v8a/`.

Both are required — loading `libMNN_Vulkan.so` without `libvulkan.so` still fails silently.

## Runtime libs in `cam-rust/lib/aarch64-v8a/`

| Library | Source | Purpose |
|---------|--------|---------|
| `libMNN.so` | `~/MNN/build` | MNN core |
| `libMNN_Vulkan.so` | `~/MNN/build` | Vulkan backend plugin |
| `libvulkan.so` | `/system/lib64/` | Android Vulkan ICD loader |
| `libMNNConvertDeps.so` | `~/MNN/build` | ONNX→MNN converter deps |
