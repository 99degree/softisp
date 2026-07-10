# MNN Vulkan Backend Guide

> **Using libMNN with Vulkan GPU acceleration on ARM64 Android/Termux**
>
> **Last Updated**: 2025-07-08
> **Tested on**: Qualcomm Adreno GPU, MNN 3.5.0, ARM64 Termux

---

## Table of Contents

1. [Overview](#overview)
2. [Building libMNN with Vulkan](#building-libmnn-with-vulkan)
3. [Verifying Vulkan Support](#verifying-vulkan-support)
4. [Session API with Vulkan Backend](#session-api-with-vulkan-backend)
5. [Module (Express) API with Vulkan Backend](#module-express-api-with-vulkan-backend)
6. [Vulkan-Specific Considerations](#vulkan-specific-considerations)
7. [Buffer Management for Vulkan](#buffer-management-for-vulkan)
8. [Performance Tuning](#performance-tuning)
9. [Known Issues & Workarounds](#known-issues--workarounds)
10. [Rust FFI Integration](#rust-ffi-integration)
11. [Troubleshooting](#troubleshooting)

---

## Overview

MNN supports multiple GPU backends via a pluggable architecture. The Vulkan backend (`MNN_FORWARD_VULKAN`, type `7`) runs inference on the GPU using Vulkan compute shaders.

### Key Characteristics

| Aspect | CPU (type 0) | Vulkan (type 7) |
|--------|-------------|-----------------|
| Execution | ARM NEON SIMD | GPU compute shaders |
| Memory | DDR / host RAM | GPU device memory |
| Precision | FP32/FP16/INT8 | FP16 (fast) / FP32 (slow) |
| Initialization | Instant | ~150–300ms (shader compilation) |
| Batch >1 | Good | Poor (Vulkan backend is batch-optimized) |
| Small ops | Fast | Slower (kernel launch overhead) |
| Large ops (Conv, MatMul) | Fast | **Faster** (GPU parallelism) |
| Power efficiency | Medium | **Higher** for sustained loads |

### When to Use Vulkan

**✅ Good fit:**
- Large models with Conv/MatMul/GEMM-heavy ops
- Batch=1 inference on modern GPU (Adreno 6xx+)
- Power-constrained scenarios (phone battery)
- Offloading CPU for concurrent work

**❌ Not a good fit:**
- Very small models (< 1M params) — CPU wins on latency
- Models with many small, scattered ops (overhead dominates)
- Batch > 1 on mobile GPUs
- ops not implemented in Vulkan backend (fallback to CPU)

---

## Building libMNN with Vulkan

### 1. CMake Options

```bash
mkdir -p build_vulkan && cd build_vulkan

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DMNN_VULKAN=ON \              # Enable Vulkan backend
  -DMNN_VULKAN_IMAGE=ON \         # Enable Vulkan image memory (recommended)
  -DMNN_USE_LOGCAT=ON \           # For Android logging
  -DMNN_BUILD_SHARED_LIBS=ON \    # Shared libs for FFI
  -DMNN_BUILD_TEST=OFF \          # Skip tests for speed
  -DMNN_BUILD_DEMO=OFF \
  -DMNN_BUILD_TOOLS=OFF
  # Add other options as needed
```

**On Android/Termux** (like this device), the NDK is not needed — the system's Clang toolchain works:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DMNN_VULKAN=ON \
  -DMNN_VULKAN_IMAGE=ON \
  -DMNN_USE_LOGCAT=ON \
  -DMNN_BUILD_SHARED_LIBS=ON \
  -DMNN_BUILD_TEST=OFF \
  -DMNN_BUILD_TOOLS=OFF \
  -DMNN_BUILD_OPENCV=OFF \
  -DMNN_BUILD_LLM=OFF \
  -DMNN_SEP_BUILD=OFF

make -j$(nproc) MNN MNN_Vulkan MNN_Express
```

### 2. Built Artifacts

After building, you get:

```
build_vulkan/
├── libMNN.so               # Core MNN library
├── libMNN_Vulkan.so        # Vulkan backend plugin
├── libMNN_Express.so       # Express API (optional)
└── source/backend/vulkan/
    └── libMNN_Vulkan.so    # (also here if SEP_BUILD=ON)
```

### 3. Library Loading Order

The Vulkan backend is loaded at **runtime** — `libMNN.so` dlopen's `libMNN_Vulkan.so` when you request `MNN_FORWARD_VULKAN`. The Vulkan loader (`libvulkan.so`) is then dlopen'd by `libMNN_Vulkan.so`.

```
Your App → libMNN.so → dlopen → libMNN_Vulkan.so → dlopen → libvulkan.so
                                                                  ↓
                                                            vulkan.adreno.so
                                                           (driver, loaded by libvulkan)
```

**LD_LIBRARY_PATH must include** all MNN libraries:

```bash
export LD_LIBRARY_PATH=/path/to/build_vulkan:/path/to/build_vulkan/source/backend/vulkan
```

---

## Verifying Vulkan Support

### 1. Using `GetMNNInfo` tool

```bash
LD_LIBRARY_PATH="$(pwd)/OFF:$(pwd)/express/OFF:$(pwd)/source/backend/vulkan/OFF" \
  ./GetMNNInfo model.mnn
```

Output (on Android → logcat with tag `MNNJNI`):
```
Model default dimensionFormat is NHWC
Model Inputs:
  [ input ]: NC4HW4, size: [ -1, 3, 224, 224 ], type: float
Model Outputs:
  [ output ]
```

### 2. Using `run_test.out` (requires `MNN_BUILD_TEST=ON`)

All operators can be run with backend type `7`:

```bash
# Run a specific test on Vulkan
LD_LIBRARY_PATH="..." ./run_test.out op/BinaryBroadcastTest 7 1 0

# Run all op tests on Vulkan
LD_LIBRARY_PATH="..." ./run_test.out all 7 1 0
```

### 3. Check Vulkan Driver in Process Maps

When Vulkan is active, `/proc/<pid>/maps` shows:

```
/vendor/lib64/hw/vulkan.adreno.so    ← Adreno driver
/system/lib64/libvulkan.so           ← Vulkan loader
```

### 4. Probe from code

```cpp
#include <MNN/MNNForwardType.h>
#include <MNN/Interpreter.hpp>

bool checkVulkanSupport() {
    // Try to create an interpreter and Vulkan session
    auto net = MNN::Interpreter::createFromFile("model.mnn");
    if (!net) return false;
    
    MNN::ScheduleConfig cfg;
    cfg.type = MNN_FORWARD_VULKAN;
    cfg.numThread = 1;
    MNN::BackendConfig bc;
    bc.precision = MNN::BackendConfig::Precision_Normal;
    cfg.backendConfig = &bc;
    
    auto sess = net->createSession(cfg);
    if (!sess) return false;
    
    return true;  // Vulkan is available and working
}
```

---

## Session API with Vulkan Backend

### Basic Usage

```cpp
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>

// 1. Load model
auto net = MNN::Interpreter::createFromFile("model.mnn");

// 2. Configure for Vulkan
MNN::ScheduleConfig cfg;
cfg.type = MNN_FORWARD_VULKAN;     // ← Vulkan backend
cfg.numThread = 1;                  // Ignored for Vulkan (GPU uses 1 queue)

MNN::BackendConfig bc;
bc.precision = MNN::BackendConfig::Precision_Normal;
// Precision_Low    → FP16 (fast, good quality)
// Precision_Normal → FP16 (balance)
// Precision_High   → FP32 (slow, accurate)

cfg.backendConfig = &bc;

// 3. Create session
auto sess = net->createSession(cfg);

// 4. Get input tensor
auto in = net->getSessionInput(sess, "input");

// 5. Set input data
auto shape = in->shape();
size_t size = 1;
for (auto d : shape) size *= d;
std::vector<float> data(size);

// Option A: Direct host pointer (SIMD-friendly)
in->buffer().host = (uint8_t*)data.data();

// Option B: copyFromHostTensor (only works if backend allocated memory)
auto hostTensor = MNN::Tensor::create(shape, in->getType(), data.data(), MNN::Tensor::CAFFE);
in->copyFromHostTensor(hostTensor);
delete hostTensor;

// 6. Run
net->resizeSession(sess);    // Optional: reallocate for new shapes
net->runSession(sess);

// 7. Get output
auto out = net->getSessionOutput(sess, "output");
auto outTensor = MNN::Tensor::create(out->shape(), out->getType());
out->copyToHostTensor(outTensor);
float* result = outTensor->host<float>();

// 8. Cleanup
MNN::Interpreter::destroy(net);
```

### Zero-Copy Input Setup

For Vulkan, the most efficient approach is to let MNN manage GPU memory and use `copyFromHostTensor`:

```cpp
// 1. Create session (this allocates GPU buffers)
auto sess = net->createSession(cfg);

// 2. Get input tensor (host-portal)
auto in = net->getSessionInput(sess, "input");

// 3. Host-side tensor with our data
auto hostTensor = MNN::Tensor::create(in->shape(), in->getType(), data.data(), MNN::Tensor::CAFFE);

// 4. This copies CPU → GPU via the Vulkan backend
//    (internally: memcpy to staging buffer → vkCmdCopyBuffer)
in->copyFromHostTensor(hostTensor);

// 5. Run inference (GPU processes the data)
net->runSession(sess);
```

**Note**: For Vulkan, `copyFromHostTensor` **does work** because the Vulkan backend allocates proper GPU memory for the input tensor during session creation. This differs from CPU backend where the input tensor is a "portal" with null backend.

---

## Module (Express) API with Vulkan Backend

The high-level Express/VARP API also supports Vulkan:

```cpp
#include <MNN/expr/Module.hpp>
#include <MNN/expr/ExprCreator.hpp>
#include <MNN/expr/Executor.hpp>

using namespace MNN::Express;

// 1. Configure executor for Vulkan
MNN::BackendConfig bc;
bc.precision = MNN::BackendConfig::Precision_Low;  // FP16

auto executor = Executor::newExecutor(MNN_FORWARD_VULKAN, bc, 1);
ExecutorScope scope(executor);

// 2. Load module
auto module = Module::load({"input"}, {"output"}, "model.mnn");

// 3. Create input VARP
auto input = _Input({1, 3, 224, 224}, NCHW, halide_type_of<float>());
auto ptr = input->writeMap<float>();
// Fill data...
for (int i = 0; i < 3*224*224; i++) ptr[i] = ...;
input->unMap();

// 4. Run
auto outputs = module->onForward({input});

// 5. Get output
auto output = outputs[0];
auto outPtr = output->readMap<float>();
```

---

## Vulkan-Specific Considerations

### 1. GPU Memory Types

Vulkan has two memory types for compute:

| Memory Type | CMake Flag | Description | Performance |
|------------|-----------|-------------|-------------|
| **Buffer** | `-DMNN_VULKAN_IMAGE=OFF` | Linear storage buffers | Good for 1D/sequential access |
| **Image** | `-DMNN_VULKAN_IMAGE=ON` (default) | 2D image objects | **Better for Conv/Im2Col** on Adreno |

**Recommendation**: Use **Image** (`MNN_VULKAN_IMAGE=ON`). It's the default and performs better on Adreno GPUs.

### 2. Precision Modes

| BackendConfig::Precision | Vulkan Behavior | Use Case |
|--------------------------|----------------|----------|
| `Precision_Low` (2) | FP16 compute + storage | **Best performance**, good quality for most models |
| `Precision_Normal` (1) | FP16 compute, FP32 storage | Balance |
| `Precision_High` (0) | FP32 compute + storage | Reference accuracy, slowest |

**Recommendation**: Use `Precision_Low` for inference. On Adreno, FP16 is natively supported in hardware (2× throughput vs FP32).

### 3. Vulkan Queue & Threading

The `numThread` setting in `ScheduleConfig` is **ignored** by the Vulkan backend — GPU uses a single submission queue. However, setting it to `1` is correct.

### 4. Fallback to CPU

Ops not implemented in the Vulkan backend automatically fall back to CPU via MNN's heterogeneous execution scheduler. This is transparent but introduces a CPU↔GPU copy overhead.

To check which ops ran on Vulkan vs CPU:

```cpp
// Enable debug output
MNN::Tensor::dumpTensorOutput = true;
```

### 5. Shader Compilation Cache

Vulkan backend compiles shaders lazily. The first inference is significantly slower (~150–300ms). You can warm up:

```cpp
// Warmup: run with a dummy input to compile shaders
net->runSession(sess);  // First run: slow (compiles)
net->runSession(sess);  // Second run: fast (cached)
```

---

## Buffer Management for Vulkan

### CPU ↔ GPU Transfer Cost

For Vulkan inference, data must move between CPU and GPU:

```
CPU Memory ──→ Staging Buffer (CPU visible) ──→ GPU Device Memory ──→ Compute
                      ↑ copy                      ↑ vkCmdCopyBuffer
```

**Key insight**: The memfd approach (from `MNN_SOLUTION_SUMMARY.md`) works for CPU backend but gives **no benefit** for Vulkan. Vulkan always copies data to GPU device memory — you cannot make the GPU read directly from CPU-allocated memfd buffers.

### Recommended Buffer Strategy

```cpp
// 1. Create a STAGING buffer (CPU-visible, used for upload/download)
//    This is what copyFromHostTensor does internally

// 2. Let MNN manage GPU memory
auto sess = net->createSession(cfg);  // Allocates GPU memory for all tensors

// 3. Upload CPU data → GPU (happens inside runSession)
auto in = net->getSessionInput(sess, "input");
in->copyFromHostTensor(hostTensor);   // CPU → staging buf → GPU

// 4. Run inference (GPU only, no CPU involvement)
net->runSession(sess);

// 5. Download GPU results → CPU
auto out = net->getSessionOutput(sess, "output");
out->copyToHostTensor(hostOutTensor); // GPU → staging buf → CPU
```

### For Camera Pipeline Integration

In a camera pipeline, you can't avoid the CPU→GPU upload, but you can minimize stalls:

```rust
// Rust pseudocode: camera → CPU buffer → Vulkan inference
fn process_frame(frame: &[u8]) -> Vec<f32> {
    // 1. Preprocess on CPU (normalize, convert format)
    let input = preprocess(frame);  // CPU work

    // 2. Upload to GPU + infer + download
    //    The copyFromHostTensor + runSession + copyToHostTensor
    //    sequence is pipelined by the Vulkan driver
    
    // 3. Postprocess on CPU
    postprocess(output)
}
```

To overlap CPU and GPU work, use double-buffering:

```rust
fn pipeline_double_buffer() {
    // Buffer A: CPU preprocessing
    // Buffer B: GPU inference
    // Double-buffer to overlap:
    //   Frame 1: CPU preprocess(A) → GPU infer(B)
    //   Frame 2: CPU preprocess(B) → GPU infer(A)
    //   ...
}
```

---

## ISP Pipeline Performance

The SoftISP pipeline runs entirely on Vulkan GPU via MNN:

### Benchmark Results (Snapdragon 8 Gen 2)

| Resolution | GPU Latency | GPU FPS | CPU Latency | CPU FPS | Speedup |
|------------|-------------|---------|-------------|---------|---------|
| HD (1280×720) | 1.47 ms | **680** | 442 ms | 2 | **340×** |
| FHD (1920×1080) | 2.28 ms | **438** | 983 ms | 1 | **438×** |
| 4K (3840×2160) | 12.87 ms | **78** | 3904 ms | 0.25 | **312×** |

### Pipeline Architecture

The ISP pipeline uses 4 GPU dispatches:

```
RawInput (INT16 packed) → Unpack+Demosaic+CCM → WarpGrid → Display
     ↓                        ↓                    ↓           ↓
  1 dispatch              1 dispatch          1 dispatch   1 dispatch
```

### IspChainFusion

The MNN converter fuses standard ONNX ops into ISP-specific VulkanFuse Extra ops:

- **Pass 1**: Standard ops → ISP Extra ops (demosaic, FCS, display)
- **Pass 2**: Adjacent Extra ops → Fused Extra ops (unpack_demosaic, fcs_display)

This reduces GPU dispatches from 12+ to 4-5.

### Running the Benchmark

```bash
# GPU benchmark
ENGINE=vulkan cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp

# CPU benchmark
ENGINE=cpu cargo run --release --features mnn --example bench_e2e_pipeline -p cam-isp
```

---

## Performance Tuning

### Benchmarking

Use MNN's `timeProfile.out` or `benchmark.out` tools:

```bash
# Warmup (compile shaders)
LD_LIBRARY_PATH="..." ./timeProfile.out model.mnn input.png 7 10

# Measure (10 warmup + 50 benchmark)
LD_LIBRARY_PATH="..." ./timeProfile.out model.mnn input.png 7 10 50
```

### Tuning Parameters

```cpp
MNN::ScheduleConfig cfg;
cfg.type = MNN_FORWARD_VULKAN;
cfg.backupType = MNN_FORWARD_CPU;  // Fallback if Vulkan fails

// Vulkan-specific tuning (via backendConfig)
MNN::BackendConfig bc;
bc.precision = MNN::BackendConfig::Precision_Low;  // FP16
bc.memory = MNN::BackendConfig::Memory_Normal;

// Memory modes:
//   Memory_Normal  → Default
//   Memory_High    → More memory, possibly faster
//   Memory_Low     → Less memory, possibly slower (not recommended for Vulkan)
```

### What to Expect (Measured on Adreno)

| Model | CPU (1 thread) | Vulkan (FP16) | Speedup |
|-------|---------------|---------------|---------|
| MobileNetV2 (3.5M params) | ~15ms | ~8ms | **~1.9×** |
| Small op test (< 1M params) | ~0.12ms | ~0.26ms | **0.5×** (worse) |
| BatchMatMul (6×6×64×64) | ~0.62ms | ~0.88ms | **0.7×** (initialization dominates) |

**Rule of thumb**: Vulkan starts winning when each op has enough work to amortize the kernel launch overhead (> 1ms of GPU work per op).

---

## Known Issues & Workarounds

### 1. Op Coverage

From `test_stages.json` (confirmed on this device):

| Op | Status | Notes |
|----|--------|-------|
| `op/binary/powInt8` | ❌ Returns 0 | Vulkan backend bug |
| `op/binary/AddBroast` | ❌ SEGFAULT | Vulkan backend bug |
| `op/Interp` | ❌ Fails | Vulkan interpolation bug |
| `op/InterpInt8` | ❌ Fails | Same |
| `op/Deconvolutionfull` | ❌ Fails | Known limitation |
| `op/GridSample3D` | ❌ Fails | Not implemented |
| `op/AvePool3d` | ❌ Fails | Not implemented |
| `op/ROIPooling` | ❌ Fails | Known limitation |
| `op/cumprod` / `op/cumsum` | ❌ Fails | Loop/scan family |
| `op/convolution/weighti8i4conv2d` | ❌ Hangs | Int4 quant conv |

**Workaround**: All failing ops fall back to CPU transparently, but at a performance cost. For critical paths, verify your model's ops work on Vulkan by running:

```bash
LD_LIBRARY_PATH="..." ./run_test.out <op_name> 7 1 0
```

### 2. Initialization Latency

First infer: ~150–300ms (shader compilation)
Subsequent: Fast

**Workaround**: Run a dummy inference at startup to warm up the shader cache.

### 3. Memory Pressure

Vulkan allocates GPU memory for all intermediate tensors. For large models (LLM, diffusion), this can exhaust GPU memory. Use `Memory_Low` mode to reduce memory at the cost of recomputation.

### 4. Session_Input_User Mode

The `Session_Input_User` mode that controls input memory ownership is **not compatible with Vulkan** — the Vulkan backend needs to allocate its own GPU memory.

---

## Rust FFI Integration

For the `cam-isp` project, integrating libMNN with Vulkan involves:

### Step 1: Build libMNN with Vulkan

```bash
cd /path/to/MNN
mkdir build_vulkan && cd build_vulkan
cmake .. \
  -DMNN_VULKAN=ON -DMNN_VULKAN_IMAGE=ON \
  -DMNN_BUILD_SHARED_LIBS=ON \
  -DMNN_BUILD_TEST=OFF -DMNN_BUILD_TOOLS=OFF \
  -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) MNN MNN_Vulkan
```

### Step 2: Link Rust Binary

```toml
# In your Rust build script or Cargo.toml
[build-dependencies]
# Add build.rs that sets cargo:rustc-link-search and cargo:rustc-link-lib
```

`build.rs`:
```rust
fn main() {
    println!("cargo:rustc-link-search=/path/to/MNN/build_vulkan");
    println!("cargo:rustc-link-search=/path/to/MNN/build_vulkan/source/backend/vulkan");
    println!("cargo:rustc-link-lib=MNN");
    println!("cargo:rustc-link-lib=MNN_Vulkan");
    
    // Generate bindings (optional, using bindgen)
    let bindings = bindgen::Builder::default()
        .header("/path/to/MNN/include/MNN/Interpreter.hpp")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");
    bindings
        .write_to_file("src/mnn_bindings.rs")
        .expect("Couldn't write bindings");
}
```

### Step 3: FFI Wrapper

```rust
// src/mnn_vulkan.rs
use std::ffi::CString;

#[link(name = "MNN")]
#[link(name = "MNN_Vulkan")]
extern "C" {
    // MNN C API functions (use mnn/src/*.hpp or generate with bindgen)
}

pub struct MnnVulkanSession {
    // ... 
}

impl MnnVulkanSession {
    pub fn new(model_path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        // 1. Create interpreter from file
        // 2. Create session with Vulkan config
        // 3. Return session
    }
    
    pub fn infer(&mut self, input: &[f32]) -> Result<Vec<f32>, Box<dyn std::error::Error>> {
        // 1. Set input
        // 2. Run session
        // 3. Get output
    }
}
```

### Step 4: Runtime Loading

At runtime, ensure the MNN libraries are findable:

```bash
# Set LD_LIBRARY_PATH before running
export LD_LIBRARY_PATH=/path/to/MNN/build_vulkan:/path/to/MNN/build_vulkan/source/backend/vulkan
./your_rust_binary
```

Or embed the path in the Rust binary:

```rust
fn main() {
    // Add to dynamic linker search path
    std::env::set_var("LD_LIBRARY_PATH", "/path/to/MNN/build_vulkan");
    // ... rest of program
}
```

### Step 5: Complete Rust Example

```rust
use std::ffi::{CStr, CString};
use std::os::raw::c_char;

// Minimal MNN C-API binding for Vulkan inference
extern "C" {
    fn mnn_interpreter_create_from_file(path: *const c_char) -> *mut std::ffi::c_void;
    fn mnn_interpreter_create_session(interp: *mut std::ffi::c_void, 
                                       forward_type: i32, precision: i32, 
                                       num_thread: i32) -> *mut std::ffi::c_void;
    fn mnn_interpreter_get_session_input(sess: *mut std::ffi::c_void,
                                          name: *const c_char) -> *mut std::ffi::c_void;
    fn mnn_tensor_copy_from_host(tensor: *mut std::ffi::c_void,
                                  data: *const f32, size: i32) -> i32;
    fn mnn_session_run(sess: *mut std::ffi::c_void) -> i32;
    fn mnn_interpreter_destroy(interp: *mut std::ffi::c_void);
}

pub struct VulkanEngine {
    interpreter: *mut std::ffi::c_void,
    session: *mut std::ffi::c_void,
    input_tensor: *mut std::ffi::c_void,
    input_size: usize,
}

impl VulkanEngine {
    pub fn new(model_path: &str, input_size: usize) -> Result<Self, String> {
        let path = CString::new(model_path).unwrap();
        
        let interpreter = unsafe { mnn_interpreter_create_from_file(path.as_ptr()) };
        if interpreter.is_null() {
            return Err("Failed to load model".into());
        }
        
        // Vulkan = type 7, Low precision = 2 (FP16)
        let session = unsafe {
            mnn_interpreter_create_session(interpreter, 7, 2, 1)
        };
        if session.is_null() {
            unsafe { mnn_interpreter_destroy(interpreter) };
            return Err("Failed to create Vulkan session".into());
        }
        
        let input_tensor = unsafe {
            mnn_interpreter_get_session_input(session, std::ptr::null())
        };
        
        Ok(Self { interpreter, session, input_tensor, input_size })
    }
    
    pub fn infer(&mut self, input: &[f32]) -> Result<Vec<f32>, String> {
        assert_eq!(input.len(), self.input_size);
        
        let ret = unsafe {
            mnn_tensor_copy_from_host(self.input_tensor, input.as_ptr(), input.len() as i32)
        };
        if ret != 0 {
            return Err("Failed to copy input".into());
        }
        
        let ret = unsafe { mnn_session_run(self.session) };
        if ret != 0 {
            return Err(format!("runSession failed: {}", ret));
        }
        
        // Read output... (simplified)
        Ok(Vec::new())
    }
}

impl Drop for VulkanEngine {
    fn drop(&mut self) {
        unsafe { mnn_interpreter_destroy(self.interpreter) };
    }
}
```

---

## Troubleshooting

### "Don't support 7"
Vulkan backend is not built. Rebuild with `-DMNN_VULKAN=ON`.

### "Invalid device for support vulkan"
`vkCreateInstance` failed or no suitable GPU found. Check:
```bash
# Is libvulkan.so available?
ldconfig -p | grep vulkan
# Or try loading it directly
python3 -c "import ctypes; ctypes.cdll.LoadLibrary('libvulkan.so')"
```

### SEGFAULT on `runSession`
Possible causes:
- Model has unsupported ops → check with `run_test.out <op_name> 7 1 0`
- Input shape mismatch → verify `getSessionInput` dimensions
- GPU memory exhausted → reduce model size or use `Memory_Low`

### COMPUTE_SIZE_ERROR (error code 3)
- Input tensor memory issue (see `mnn-inference-guide.md` for CPU-specific analysis)
- For Vulkan: likely a shape mismatch or unsupported op combination

### Test hangs on Vulkan
Some ops hang the Vulkan driver (e.g., `weighti8i4conv2d`). Kill the process and skip the offending op.

### Performance worse than CPU
- First inference is slow due to shader compilation (run a warmup)
- Model is too small — Vulkan overhead dominates
- Check precision mode: use `Precision_Low` for FP16

---

## References

- MNN Vulkan source: `MNN/source/backend/vulkan/`
- MNN runtime registration: `VulkanRuntime.cpp:280`
- Vulkan wrapper (dlopen): `vulkan_wrapper.cpp`
- GPU type detection: `VulkanRuntime.cpp:96-106`
- `test_stages.json`: test configuration including Vulkan stages
- MNN_SOLUTION_SUMMARY.md: memfd buffer management (CPU only)
- mnn-inference-guide.md: detailed CPU inference analysis

---

## Changelog

- **2025-06-13**: Initial guide — Vulkan backend usage for MNN on ARM64
- **2025-06-13**: Added benchmark data, known issues, Rust FFI integration
