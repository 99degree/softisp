# Multi-Frontend IPA Architecture for softisp

## Executive Summary

**Goal**: Create a stable, reusable Rust IPA core (`cam-ipa`) with multiple frontend adapters for different camera stacks:
- **libcamera-ipa** — libcamera pipeline handler (C FFI)
- **jni** — Android JNI bridge for Java/Kotlin apps
- **anw** — ANativeWindow buffer consumer for Android display
- **cli** — standalone polyglot ELF for 4K frame generation, 3A/neural processing, and V4L2 capture (training data prep)

All frontends consume the same `cam-ipa` Rust API, ensuring consistent ISP processing across platforms.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      cam-ipa (CORE, rlib)                       │
│  IpaContext (lifecycle) │ EngineManager (MnnEngine) │ IpaController │
│  IpaPipeline (blocks)   │ BufferManager (zero-copy) │ EventEmitter │
│                         │ NeuralController (isp-rectifier, opt-in)│
└──────────────────────────┬──────────────────────────────────────┘
         ┌─────────────────┼─────────────────┬─────────────────────┐
         ▼                 ▼                 ▼                     ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐ ┌─────────────┐
│libcamera-ipa  │ │     jni       │ │     anw       │ │     cli     │
│  (cdylib)     │ │   (cdylib)    │ │   (cdylib)    │ │ (cdylib+bin)│
│               │ │               │ │               │ │               │
│• IPAInterface │ │• JNI_OnLoad   │ │• ANativeWindow│ │• 4K frame   │
│• IPASoftIsp   │ │• Java API     │ │• BufferQueue  │ │• 3A + neural│
│• SharedFD→dmab│ │• AHardwareBuf │ │• EGL/GPU      │ │• V4L2 capt. │
│   uf→DmaBuf   │ │               │               │ │• training prep│
└───────────────┘ └───────────────┘ └───────────────┘ └─────────────┘
```

---

## Crate Structure

| Crate | Type | Path | Purpose |
|-------|------|------|---------|
| `cam-ipa` | rlib | `cam-rust/cam-ipa/` | Stable Rust IPA API (core) |
| `libcamera-ipa` | rlib+cdylib | `libcamera-ipa/` | libcamera C FFI plugin |
| `jni` | rlib+cdylib | `jni/` | Android JNI bridge |
| `anw` | rlib+cdylib | `anw/` | ANativeWindow consumer |
| `cli` | rlib+cdylib+bin | `cli/` | 4K generator, 3A/neural, V4L2 capture |

All crates link statically via `rlib` or dynamically via `cdylib`. They share `cam-ipa` as the single dependency.

---

## cam-ipa: Stable Rust API

### Public Types (`types.rs`)

```rust
pub struct IpaConfig {
    pub model_dir: Option<PathBuf>,      // ONNX/MNN model directory
    pub image_width: u32,
    pub image_height: u32,
    pub format: IpaFormat,
    pub num_output_buffers: u32,
    pub backend: EngineBackend,
    pub enable_dual_gpu: bool,
    pub stats_interval: u32,
    pub neural_enabled: bool,            // enable NeuralController (isp-rectifier)
}

pub enum IpaFormat { RGB, RGBA, BGRA, ARGB, ABGR, YUV, NV12, P010 }
pub enum EngineBackend { MnnVulkan, Cpu, OnnxRuntime }

pub struct IpaSettings {
    pub config_file: Option<String>,
    pub sensor_model: String,
}

pub struct IpaSensorInfo {
    pub model: String,
    pub bits_per_pixel: u32,
    pub cfa_pattern: u32,
    pub active_area_size: Size,
    pub analog_crop: Rectangle,
    pub output_size: Size,
    pub pixel_rate: u64,
    pub min_line_length: u32,
    pub max_line_length: u32,
    pub min_frame_length: u32,
    pub max_frame_length: u32,
}

pub struct IpaCallbacks {
    pub metadata_ready: Option<Box<dyn Fn(u32, &ControlList) + Send + Sync>>,
    pub frame_done: Option<Box<dyn Fn(u32, u32) + Send + Sync>>,
    pub set_sensor_controls: Option<Box<dyn Fn(&ControlList) + Send + Sync>>,
    pub set_isp_params: Option<Box<dyn Fn() + Send + Sync>>,
}

pub struct ControlList { /* serde-serializable key→value map */ }
pub struct ControlInfoMap { /* metadata about available controls */ }
pub struct Size { pub width: u32, pub height: u32 }
pub struct Rectangle { pub x: i32, pub y: i32, pub width: u32, pub height: u32 }
```

### Core API (`context.rs`)

```rust
pub struct IpaContext { /* private */ }

impl IpaContext {
    pub fn new(config: IpaConfig) -> IpaResult<Self>;
    pub fn init(&mut self, _settings: &IpaSettings, sensor_info: &IpaSensorInfo) -> IpaResult<IpaInitResult>;
    pub fn start(&mut self) -> IpaResult<()>;
    pub fn stop(&mut self);
    pub fn configure(&mut self, config: &IpaConfigureInfo) -> IpaResult<()>;
    pub fn configure_frame_backend(&mut self, format: IpaFormat, num_buffers: u32) -> IpaResult<()>;
    
    // Frame processing
    pub fn queue_request(&mut self, frame: u32, controls: &ControlList) -> IpaResult<()>;
    pub fn compute_params(&mut self, frame: u32) -> IpaResult<()>;
    pub fn process_stats(&mut self, frame: u32, buffer_id: u32, buffer: &dyn Buffer) -> IpaResult<ControlList>;
    pub fn process_frame(&mut self, frame: u32, buffer_id: u32,
                         input: &mut dyn Buffer, output: &mut dyn Buffer,
                         plane_in: i32, plane_out: i32,
                         width: u32, height: u32,
                         results: &ControlList) -> IpaResult<()>;
    
    // Callbacks
    pub fn set_callbacks(&mut self, callbacks: IpaCallbacks);
    
    // Status
    pub fn is_initialized(&self) -> bool;
    pub fn is_started(&self) -> bool;
}

// Buffer abstraction for zero-copy
pub trait Buffer: Send + Sync {
    fn as_ptr(&self) -> *const u8;
    fn as_mut_ptr(&mut self) -> *mut u8;
    fn len(&self) -> usize;
    fn fd(&self) -> Option<RawFd>;
    fn is_ahb(&self) -> bool { false }
    fn ahb_ptr(&self) -> Option<*mut c_void> { None }
}
```

### Internal Modules

| Module | Purpose |
|--------|---------|
| `engine_manager.rs` | Wraps `MnnEngine` (Vulkan), handles build/process, FP16 storage |
| `controller.rs` | `IpaController` — wraps `cam_isp::controller::IspController` for 3A (AWB/AE/CCM/tone) |
| `buffer.rs` | `Buffer` trait + `DmaBufBuffer` (Linux dmabuf/memfd mmap + Android SharedFD), `AhwBuffer` (Android AHB), `MemBuffer` (testing), `create_memfd_dmabuf()` |
| `events.rs` | `EventEmitter` — thread-safe callback channel |
| `error.rs` | `IpaError`, `IpaResult<T>` |
| `types.rs` | Public types (IpaConfig, IpaFormat, ControlList, etc.) |

### Neural Processor (Optional)

When `cam-ipa` is built with `cam-isp/rectifier` feature (default), `IpaContext` can use `cam_isp::neural_controller::NeuralController` instead of the rule-based `IspController`. The neural controller wraps `isp_rectifier::OptimizedInference` with automatic fallback to rule-based when the model is unavailable.

- Set `neural_enabled: true` in `IpaConfig` to opt in
- Model path from `IpaConfig.model_dir` or `SOFTISP_MODEL_DIR` env var
- Falls back gracefully if model load fails

---

## libcamera-ipa: Frontend 1

Implements libcamera's `IPAInterface` and `IPASoftIspInterface` via C FFI.

### Exports (matching libcamera headers)

```c
struct IPAModuleInfo {
    int moduleAPIVersion;      // = 1
    uint32_t pipelineVersion;  // = 0
    char pipelineName[256];    // "softisp"
    char name[256];            // "SoftISP"
};

extern "C" IPAInterface* ipaCreate();

// VTable for IPASoftIspInterface
int32_t init(...);
int32_t start();
void stop();
int32_t configure(IPAConfigInfo*);
int32_t configureFrameBackend(Format, uint32);
void queueRequest(uint32, ControlList*);
void computeParams(uint32);
void processStats(uint32, uint32, ControlList*);
void processFrame(uint32, uint32, SharedFD, int32, int32, int32, int32, ControlList*);
```

### Bridge Strategy
- `ipaCreate()` → `Box::into_raw(Box::new(IpaContext::new(...)))` as `*mut c_void`
- `SharedFD` → extract raw fd → wrap in `DmaBufBuffer` (mmap'd dmabuf)
- `ControlList`/`ControlInfoMap` → JSON-serializable maps
- `Format` RGB/YUV → `IpaFormat` → `OutputFormat`

---

## jni: Frontend 2

Exposes Java/Kotlin API for Android apps.

### Java API Surface

```kotlin
class IpaSession(config: IpaConfig) {
    fun init(settings, sensorInfo): IpaInitResult
    fun start(); fun stop(); fun configure(...); fun configureFrameBackend(...)
    fun queueRequest(frame, controls); fun computeParams(frame)
    fun processStats(frame, bufferId, hardwareBuffer): ControlList
    fun processFrame(frame, bufferId, input, output, planeIn, planeOut, width, height, results)
    fun setMetadataReadyListener(...); fun setFrameDoneListener(...)
}
```

### JNI Implementation
- `JNI_OnLoad` registers native methods
- `IpaSession` holds `*mut IpaContext` as `jlong` (Send/Sync wrapper)
- `HardwareBuffer` → `AHardwareBuffer*` → `AhwBuffer` → Vulkan `input_ahb`/`output_ahb`
- Callbacks → Java `MethodID` invocation

---

## anw: Frontend 3

Consumes ANativeWindow buffer queue for display/recording.

```rust
pub struct AnwConsumer {
    window: ANativeWindow,
    ipa: IpaContext,
    buffer_queue: BufferQueue,
}

impl AnwConsumer {
    pub fn new(window, ipa_config) -> IpaResult<Self>;
    pub fn start(&mut self) -> IpaResult<()>;
    pub fn stop(&mut self);
    pub fn set_frame_callback(&mut self, cb: Box<dyn Fn(&dyn Buffer) + Send>);
}
```

Flow: dequeue buffer → wrap in `AnwBuffer` → `ipa.process_frame()` → queue back to ANativeWindow.

---

## cli: Frame Generator & 3A/Neural Test Tool (Frontend 4)

**Polyglot ELF** — runs directly (`./libcam_ipa_test_cli.so`) and is dlopen-able (exports `ipa_test_main`, `my_entry`).

### Capabilities

| Capability | Description |
|------------|-------------|
| **4K frame generator** | Synthetic 16-bit Bayer pattern at 3840×2160 (default) or configurable resolution |
| **V4L2 capture** | `cam-hal-linux` v4l2 feature — captures real frames from `/dev/video*` via `rscam` |
| **3A processing** | AWB/AE/CCM/tone via `IspController` (rule-based) or `NeuralController` (opt-in) |
| **Neural processor** | `isp-rectifier` distilled ONNX model with rule-based fallback |
| **Zero-copy** | Linux `memfd_create` + mmap (`DmaBufBuffer`); Android `SharedFD` + `AhwBuffer` |
| **Training data prep** | V4L2 per-frame stats → CMOS sensor characteristics → training datasets |

### Build

```bash
# Synthetic frames only (default)
cargo build -p cam-ipa-test-cli

# With V4L2 capture (Linux only)
cargo build -p cam-ipa-test-cli --features v4l2
```

### Run

```bash
# Direct execution (polyglot ELF)
./target/debug/libcam_ipa_test_cli.so

# Or dlopen and call ipa_test_main()
```

### Frame Acquisition Priority

1. V4L2 device (`/dev/video0`) if feature enabled and device found
2. Synthetic 16-bit Bayer pattern (always available fallback)

### Output

- Processed BGRA frame written to `output_frame.bgra` for inspection
- Per-frame metadata via `metadata_ready` callback (AWB gains, exposure, gain)
- Frame done via `frame_done` callback

---

## Buffer Abstraction (Key for Zero-Copy)

```
Linux:  dmabuf fd → DmaBufBuffer (mmap) → ProcessParams::input_buffer/output_buffer
Android: SharedFD → DmaBufBuffer (mmap) OR AHardwareBuffer → AhwBuffer → input_ahb/output_ahb (Vulkan)
Test:   MemBuffer (host memory, no zero-copy)
Linux test: memfd_create → DmaBufBuffer (create_memfd_dmabuf())
```

---

## Build Configuration

### Workspace Cargo.toml

```toml
members = [
    "isp-rectifier",
    "cam-rust/cam-types",
    "cam-rust/cam-isp",
    "cam-rust/cam-hal",
    "cam-rust/cam-hal-android",
    "cam-rust/cam-hal-linux",
    "cam-rust/cam-core",
    "cam-rust/cam-onnx",
    "cam-rust/cam-motion",
    "cam-rust/cam-binder",
    "cam-rust/cam-disp",
    "cam-rust/cam-ipa",
    "libcamera-ipa",
    "jni",
    "anw",
    "cli",
    "android-app",
]
```

### Individual Cargo.toml (crate-type = rlib + cdylib for all)

```toml
# cam-rust/cam-ipa/Cargo.toml
[lib] crate-type = ["rlib"]

# libcamera-ipa/Cargo.toml
[lib] crate-type = ["rlib", "cdylib"]

# jni/Cargo.toml
[lib] crate-type = ["rlib", "cdylib"]

# anw/Cargo.toml
[lib] crate-type = ["rlib", "cdylib"]

# cli/Cargo.toml
[lib] crate-type = ["rlib", "cdylib"]
[[bin]] name = "ipa_test_cli"
```

---

## Implementation Phases

### Phase 1: cam-ipa Core ✅
- Crate structure, Cargo.toml, error types
- Public types, Buffer trait, DmaBufBuffer (Linux memfd + Android SharedFD)
- IpaContext lifecycle, EngineManager (MnnEngine), IpaController (IspController)
- EventEmitter, unit tests

### Phase 2: libcamera-ipa Frontend ✅
- C structs matching libcamera headers, VTable, ipaCreate/IPA_MODULE_INFO exports
- Bridge: SharedFD → DmaBufBuffer, ControlList opaque handling

### Phase 3: JNI Frontend ✅ (stub)
- JNI_OnLoad, native methods, Send/Sync wrapper for *mut IpaContext

### Phase 4: ANativeWindow Frontend ✅ (stub)
- ANativeWindow consumer skeleton

### Phase 5: CLI Frame Generator ✅
- 4K synthetic Bayer generation
- V4L2 capture via cam-hal-linux (feature-gated)
- 3A + neural processing through cam-ipa
- Polyglot ELF (direct exec + dlopen)
- Output frame to file

---

## Reuse from softisp

| Component | Source | Usage |
|-----------|--------|-------|
| MnnEngine | `cam-isp/src/mnnengine.rs` | EngineManager backend (Vulkan) |
| DualGpuEngine | `cam-isp/src/dual_engine.rs` | Optional dual-GPU ping-pong |
| IspController | `cam-isp/src/controller.rs` | 3A (AWB/AE/CCM/tone/scene) |
| NeuralController | `cam-isp/src/neural_controller.rs` | isp-rectifier neural + fallback |
| Pipeline blocks | `cam-isp/src/pipeline/` | Unpack→BLC→WB→Demosaic→CCM→Tone→Denoise→Display |
| ProcessParams | `cam-isp/src/engine.rs` | Zero-copy input/output buffers + AHB |
| OutputFormat | `cam-isp/src/engine.rs` | Format negotiation |
| cam-hal-linux | `cam-rust/cam-hal-linux/` | V4L2 capture (rscam) |
| cam-types | `cam-rust/cam-types/` | Shared types (FrameFormat, ToneParams) |

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| ABI stability across frontends | cam-ipa stable Rust API; frontends thin wrappers |
| Thread safety | Mutex/RwLock for shared state; EventEmitter channel for callbacks |
| Buffer lifetime | Buffer trait Send+Sync; frontends manage platform lifetimes |
| Model loading | SOFTISP_MODEL_DIR env; neural falls back to rule-based |
| Android NDK linking | Each frontend builds with aarch64-linux-android target |
| V4L2 not available | Synthetic frame fallback (always works) |

---

## Success Criteria

1. **cam-ipa**: Builds as rlib, unit tests pass, API documented
2. **libcamera-ipa**: Produces `libipa_softisp.so`, loads in libcamera-99degree
3. **jni**: `libsoftisp_jni.so`, callable from Kotlin
4. **anw**: `libsoftisp_anw.so`, consumes ANativeWindow buffers
5. **cli**: 4K frame generation + V4L2 capture + 3A/neural processing; polyglot ELF
6. **Integration**: All frontends produce identical ISP output for same input
7. **Performance**: Vulkan path >30 FPS at 4K (target hardware)
