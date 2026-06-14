# Pre-built Libraries for Android (aarch64)

This directory contains pre-built libraries for the camera ISP pipeline.

## MNN Libraries

- **libMNN.so** - Main MNN inference library (CPU backend)
- **libMNN_Vulkan.so** - MNN Vulkan backend
- **libMNN_CL.so** - MNN OpenCL backend  
- **libMNN_GL.so** - MNN OpenGL backend
- **libMNN_Express.so** - MNN Express API
- **libMNNConvertDeps.so** - MNN converter dependencies

## ONNX Runtime

- **libonnxruntime.so** - ONNX Runtime inference library

## Standard Libraries

- **libc++_shared.so** - C++ standard library (for Android NDK)

## Vulkan

- **libvulkan.so**, **libvulkan.so.1** - Vulkan loader
- **libvulkan_freedreno.so** - Adreno Vulkan driver
- **vulkan.adreno.so** - Adreno Vulkan implementation

## Custom

- **libmnn_wrapper.a** - Static library with MNN C++ wrapper for Rust FFI
- **mnn_wrapper.o** - Object file

## Building

These libraries were built from:
- MNN: https://github.com/alibaba/MNN
- ONNX Runtime: https://github.com/microsoft/onnxruntime

To rebuild:
```bash
# Build MNN
cd ~/MNN
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DMNN_BUILD_SHARED_LIBS=ON ..
make -j4

# Build ONNX Runtime
# (See onnxruntime build instructions)

# Copy to this directory
cp ~/MNN/build/libMNN* ./cam-rust/lib/aarch64/
```

## Usage

Set LD_LIBRARY_PATH to include this directory:
```bash
export LD_LIBRARY_PATH=/path/to/softisp/cam-rust/lib/aarch64:$LD_LIBRARY_PATH
cargo run --example pipeline -p cam-isp --features "mnn"
```
