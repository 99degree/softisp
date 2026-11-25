# Multi-Frontend IPA Architecture for softisp

## Executive Summary

**Goal**: Create a stable, reusable Rust IPA core (`cam-ipa`) with multiple frontend adapters for different camera stacks:
- **libcamera-ipa** — libcamera pipeline handler (C FFI)
- **jni** — Android JNI bridge for Java/Kotlin apps
- **anw** — ANativeWindow buffer consumer for Android display

All frontends consume the same `cam-ipa` Rust API, ensuring consistent ISP processing across platforms.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      cam-ipa (CORE)                             │
│  ┌─────────────┐ ┌──────────────┐ ┌────────────┐ ┌──────────┐  │
│  │ IpaContext  │ │ EngineManager│ │IpaController│ │IpaPipeline│ │
│  │ (lifecycle) │ │ (MnnEngine)  │ │ (3A: AWB/AE)│ │ (blocks)  │  │
│  └─────────────┘ └──────────────┘ └────────────┘ └──────────┘  │
│  ┌─────────────┐ ┌──────────────┐                              │
│  │BufferManager│ │ EventEmitter │                              │
│  │ (zero-copy) │ │ (callbacks)  │                              │
│  └─────────────┘ └──────────────┘                              │
└──────────────────────────┬──────────────────────────────────────┘
                           │
         ┌─────────────────┼─────────────────┐
         ▼                 ▼                 ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│libcamera-ipa  │ │     jni       │ │     anw       │
│  (cdylib)     │ │   (cdylib)    │ │   (cdylib)    │
│               │ │               │ │               │
│• IPAInterface │ │• JNI_OnLoad   │ │• ANativeWindow│
│• IPASoftIsp   │ │• Java API     │ │• BufferQueue  │
│• SharedFD     │ │• AHardwareBuf │ │• EGL/GPU      │
└───��───────────┘ └───────────────┘ └───────────────┘
```

---

## Crate Structure

| Crate | Type | Path | Purpose |
|-------|------|------|---------|
| `cam-ipa` | rlib | `cam-rust/cam-ipa/` | Stable Rust IPA API |
| `libcamera-ipa` | cdylib | `libcamera-ipa/` | libcamera C FFI plugin |
| `jni` | cdylib | `jni/` | Android JNI bridge |
| `anw` | cdylib | `anw/` | ANativeWindow consumer |

---

## cam-ipa: Stable Rust API (Phase 1)

### Public Types (`types.rs`)

```rust
// Configuration
pub struct IpaConfig {
    pub model_dir: Option<PathBuf>,
    pub image_width: u32,
    pub image_height: u32,
    pub format: IpaFormat,
    pub num_output_buffers: u32,
    pub backend: EngineBackend,
    pub enable_dual_gpu: bool,
}

pub enum IpaFormat { RGB, YUV, RGBA, BGRA, NV12, P010 }
pub enum EngineBackend { MnnVulkan, Cpu, OnnxRuntime }

// Settings from camera stack
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

// Callbacks (type-erased for frontend flexibility)
pub struct IpaCallbacks {
    pub metadata_ready: Option<Box<dyn Fn(u32, &ControlList) + Send + Sync>>,
    pub frame_done: Option<Box<dyn Fn(u32, u32) + Send + Sync>>,
    pub set_sensor_controls: Option<Box<dyn Fn(&ControlList) + Send + Sync>>,
    pub set_isp_params: Option<Box<dyn Fn() + Send + Sync>>,
}

// Control types (simplified for stable API)
pub struct ControlList { /* opaque, serde-serializable */ }
pub struct ControlInfoMap { /* opaque */ }
pub struct Size { pub width: u32, pub height: u32 }
pub struct Rectangle { pub x: i32, pub y: i32, pub width: u32, pub height: u32 }
```

### Core API (`context.rs`)

```rust
pub struct IpaContext { /* private */ }

impl IpaContext {
    pub fn new(config: IpaConfig) -> IpaResult<Self>;
    pub fn init(&mut self, settings: &IpaSettings, sensor_info: &IpaSensorInfo) -> IpaResult<IpaInitResult>;
    pub fn start(&mut self) -> IpaResult<()>;
    pub fn stop(&mut self);
    pub fn configure(&mut self, config: &IpaConfigureInfo) -> IpaResult<()>;
    pub fn configure_frame_backend(&mut self, format: IpaFormat, num_buffers: u32) -> IpaResult<()>;
    
    // Frame processing
    pub fn queue_request(&mut self, frame: u32, controls: &ControlList) -> IpaResult<()>;
    pub fn compute_params(&mut self, frame: u32) -> IpaResult<()>;
    pub fn process_stats(&mut self, frame: u32, buffer_id: u32, buffer: &dyn Buffer) -> IpaResult<ControlList>;
    pub fn process_frame(&mut self, frame: u32, buffer_id: u32, 
                         input: &dyn Buffer, output: &mut dyn Buffer,
                         plane_in: i32, plane_out: i32,
                         width: u32, height: u32,
                         results: &ControlList) -> IpaResult<()>;
    
    // Callbacks
    pub fn set_callbacks(&mut self, callbacks: IpaCallbacks);
}

// Buffer abstraction for zero-copy
pub trait Buffer: Send + Sync {
    fn as_ptr(&self) -> *const u8;
    fn as_mut_ptr(&mut self) -> *mut u8;
    fn len(&self) -> usize;
    fn fd(&self) -> Option<RawFd>;  // For dmabuf/AHB import
}
```

### Internal Modules

- `engine_manager.rs` — Wraps `MnnEngine`/`DualGpuEngine`, handles build/process
- `controller.rs` — Wraps `IspController` for 3A (AWB/AE/CCM/tone)
- `pipeline.rs` — Builds softisp block chain (Unpack→BLC→WB→Demosaic→CCM→Tone→Denoise→Display)
- `buffer.rs` — `BufferManager` for dmabuf/AHB/memfd handling
- `events.rs` — `EventEmitter` for thread-safe callback invocation
- `error.rs` — `IpaError`, `IpaResult<T>`

---

## libcamera-ipa: Frontend 1 (Phase 2)

Implements libcamera's `IPAInterface` and `IPASoftIspInterface` via C FFI.

### Exports (matching libcamera headers)

```c
// IPAModuleInfo (exported symbol)
struct IPAModuleInfo {
    int moduleAPIVersion;      // = 1
    uint32_t pipelineVersion;  // = 0
    char pipelineName[256];    // "softisp"
    char name[256];            // "SoftISP"
};

extern "C" IPAInterface* ipaCreate();

// VTable for IPASoftIspInterface
int32_t init(IPASettings*, SharedFD fdStats, SharedFD fdParams,
             IPACameraSensorInfo*, ControlInfoMap*,
             ControlInfoMap* ipaControls, bool* ccmEnabled);
int32_t start();
void stop();
int32_t configure(IPAConfigInfo*);
int32_t configureFrameBackend(Format format, uint32_t num_output_buffers);
void queueRequest(uint32_t frame, ControlList* controls);
void computeParams(uint32_t frame);
void processStats(uint32_t frame, uint32_t bufferId, ControlList* stats);
void processFrame(uint32_t frame, uint32_t bufferId, SharedFD bufferFd,
                  int32_t planeIndexIn, int32_t planeIndexOut,
                  int32_t width, int32_t height, ControlList* results);
```

### Bridge Strategy
- `ipaCreate()` → `Box::into_raw(Box::new(IpaContext::new(...)))` as `*mut c_void`
- VTable functions cast handle to `&mut IpaContext` and delegate
- `SharedFD` → extract raw fd → wrap in `DmaBufBuffer` implementing `Buffer` trait
- `ControlList`/`ControlInfoMap` → opaque pointers + accessor FFI (MVP) or JSON serialization

---

## jni: Frontend 2 (Phase 3)

Exposes Java/Kotlin API for Android apps.

### Java API Surface

```kotlin
// com.softisp.ipa.IpaSession
class IpaSession(config: IpaConfig) {
    fun init(settings: IpaSettings, sensorInfo: IpaSensorInfo): IpaInitResult
    fun start()
    fun stop()
    fun configure(config: IpaConfigureInfo)
    fun configureFrameBackend(format: IpaFormat, numBuffers: Int)
    
    // Async frame processing
    fun queueRequest(frame: Int, controls: ControlList)
    fun computeParams(frame: Int)
    fun processStats(frame: Int, bufferId: Int, hardwareBuffer: HardwareBuffer): ControlList
    fun processFrame(frame: Int, bufferId: Int,
                     input: HardwareBuffer, output: HardwareBuffer,
                     planeIn: Int, planeOut: Int,
                     width: Int, height: Int,
                     results: ControlList)
    
    // Callbacks
    fun setMetadataReadyListener(listener: (frame: Int, metadata: ControlList) -> Unit)
    fun setFrameDoneListener(listener: (frame: Int, bufferId: Int) -> Unit)
}
```

### JNI Implementation
- `JNI_OnLoad` registers native methods
- `IpaSession` holds `*mut IpaContext` as `jlong`
- `HardwareBuffer` → `AHardwareBuffer*` → `AhwBuffer` implementing `Buffer` trait
- Callbacks → Java `MethodID` invocation on appropriate thread

---

## anw: Frontend 3 (Phase 4)

Consumes ANativeWindow buffer queue for display/recording.

### API

```rust
pub struct AnwConsumer {
    window: ANativeWindow,
    ipa: IpaContext,
    buffer_queue: BufferQueue,
}

impl AnwConsumer {
    pub fn new(window: ANativeWindow, ipa_config: IpaConfig) -> IpaResult<Self>;
    pub fn start(&mut self) -> IpaResult<()>;
    pub fn stop(&mut self);
    pub fn set_frame_callback(&mut self, cb: Box<dyn Fn(&dyn Buffer) + Send>);
}
```

### Flow
1. `ANativeWindow` dequeues buffer → `ANativeWindowBuffer`
2. Wrap in `AnwBuffer` implementing `Buffer` trait
3. Call `ipa.process_frame()` with input/output buffers
4. Queue processed buffer back to `ANativeWindow` for display

---

## Buffer Abstraction (Key for Zero-Copy)

```rust
// In cam-ipa/src/buffer.rs

pub trait Buffer: Send + Sync {
    fn as_ptr(&self) -> *const u8;
    fn as_mut_ptr(&mut self) -> *mut u8;
    fn len(&self) -> usize;
    fn fd(&self) -> Option<RawFd>;
    fn is_ahb(&self) -> bool { false }
    fn ahb_ptr(&self) -> Option<*mut c_void> { None }
}

// Implementations per frontend:
// - DmaBufBuffer (libcamera-ipa): wraps dmabuf fd, mmap on demand
// - AhwBuffer (jni, anw): wraps AHardwareBuffer*, import to Vulkan via mnn_import_ahb_as_device_memory
// - MemfdBuffer (testing): wraps memfd_create
```

---

## Build Configuration

### Workspace Cargo.toml additions

```toml
members = [
    "cam-rust/cam-ipa",
    "libcamera-ipa",
    "jni",
    "anw",
    # ... existing
]
```

### Individual Cargo.toml

```toml
# cam-rust/cam-ipa/Cargo.toml
[package]
name = "cam-ipa"
crate-type = ["rlib"]

# libcamera-ipa/Cargo.toml
[package]
name = "libcamera-ipa"
crate-type = ["cdylib"]
[dependencies]
cam-ipa = { path = "../cam-rust/cam-ipa" }

# jni/Cargo.toml
[package]
name = "softisp-jni"
crate-type = ["cdylib"]
[dependencies]
cam-ipa = { path = "../cam-rust/cam-ipa" }
jni = "0.21"
ndk-sys = { version = "0.6", features = ["media"] }

# anw/Cargo.toml
[package]
name = "softisp-anw"
crate-type = ["cdylib"]
[dependencies]
cam-ipa = { path = "../cam-rust/cam-ipa" }
ndk-sys = { version = "0.6", features = ["native_window"] }
```

---

## Implementation Phases

### Phase 1: cam-ipa Core (Week 1-2)
- [ ] Crate structure, Cargo.toml, error types
- [ ] Public types: IpaConfig, IpaSettings, IpaSensorInfo, IpaFormat, IpaCallbacks, ControlList
- [ ] Buffer trait + DmaBufBuffer/AhwBuffer implementations
- [ ] IpaContext with full lifecycle (new, init, start, stop, configure)
- [ ] EngineManager wrapping MnnEngine build/process
- [ ] IpaController wrapping IspController
- [ ] IpaPipeline building softisp block chain
- [ ] EventEmitter for callbacks
- [ ] Unit tests for each component

### Phase 2: libcamera-ipa Frontend (Week 2-3)
- [ ] Crate structure, Cargo.toml
- [ ] C struct definitions matching libcamera headers exactly
- [ ] VTable implementation for IPASoftIspInterface
- [ ] ipaCreate / ipaModuleInfo exports
- [ ] Bridge: SharedFD → DmaBufBuffer, ControlList opaque handling
- [ ] Callback registration → EventEmitter
- [ ] Build verification: `nm -D libipa_softisp.so`

### Phase 3: JNI Frontend (Week 3-4)
- [ ] Crate structure, Cargo.toml with jni/ndk-sys
- [ ] JNI_OnLoad, native method registration
- [ ] IpaSession Java class + native peer
- [ ] HardwareBuffer → AhwBuffer bridge
- [ ] Callback → Java listener invocation
- [ ] Example Android app integration

### Phase 4: ANativeWindow Frontend (Week 4)
- [ ] Crate structure, Cargo.toml with ndk-sys native_window
- [ ] ANativeWindow consumer loop
- [ ] Buffer queue management
- [ ] EGL/Vulkan interop for display
- [ ] Integration with android-app crate

---

## Reuse from softisp

| Component | Source | Usage |
|-----------|--------|-------|
| MnnEngine | `cam-isp/src/mnnengine.rs` | EngineManager backend |
| DualGpuEngine | `cam-isp/src/dual_engine.rs` | Optional dual-GPU mode |
| IspController | `cam-isp/src/isp_controller.rs` | 3A (AWB/AE/CCM/tone) |
| Pipeline blocks | `cam-isp/src/pipeline/` | IpaPipeline builder |
| ProcessParams | `cam-isp/src/engine.rs` | Zero-copy buffer passing |
| OutputFormat | `cam-isp/src/engine.rs` | Format negotiation |
| cam-types | `cam-rust/cam-types/` | Shared types |

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| ABI stability across frontends | cam-ipa uses stable Rust API; frontends are thin wrappers |
| Thread safety | cam-ipa uses `Mutex`/`RwLock`; EventEmitter uses channel for cross-thread callbacks |
| Buffer lifetime | Buffer trait enforces `Send + Sync`; frontends manage platform lifetimes |
| Model loading | cam-ipa reads `SOFTISP_MODEL_DIR` env; frontends ensure models deployed |
| Android NDK linking | Each frontend builds with appropriate target; shared cam-ipa is rlib |

---

## Success Criteria

1. **cam-ipa**: Builds as rlib, all unit tests pass, API documented
2. **libcamera-ipa**: Produces `libipa_softisp.so` with correct exports, loads in libcamera
3. **jni**: Produces `libsoftisp_jni.so`, callable from Kotlin, processes frames
4. **anw**: Produces `libsoftisp_anw.so`, consumes ANativeWindow buffers, displays output
5. **Integration**: All frontends produce identical ISP output for same input
6. **Performance**: Vulkan path achieves >30 FPS at 4K on target hardware