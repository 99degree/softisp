# MNNConvert — Static C FFI (No Subprocess)

`mnn_sys/mnn_convert_api.cpp` exposes `mnn_convert_onnx_to_mnn()` linking `MNN::Cli::convertModel()` directly. Compiled as `.a` via `cc::Build` in `build.rs`. No fork/exec, no `MNNConvert` binary needed — single `.so` dep (`libMNNConvertDeps.so`).
