# Canonical native libraries

This is the only repository directory that should contain runtime `.so`/`.a`
dependencies for MNN and ONNX Runtime on aarch64-v8a.

## Contents

- `libMNN.so` — canonical MNN runtime (Vulkan/OpenCL/OpenGL/profile enabled).
- `libMNNConvertDeps.so` — canonical MNN converter dependency shared library.
- `libonnxruntime.so` — canonical ONNX Runtime shared library.
- `libc++_shared.so` — Android NDK C++ runtime.

Build scripts copy newer artifacts from `~/MNN/build_vk/...` into this directory,
but the checked-in copies remain the fallback when the host `~/MNN` tree is not
available.
