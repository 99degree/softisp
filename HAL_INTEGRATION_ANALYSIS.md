# SoftISP: HAL Integration & Performance Optimization TODO

## 1. HAL ↔ Pipeline Parameter Mapping

### Covered Parameters

| Android Camera3 HAL Metadata Tag | ISP Pipeline Param | Source File | Status |
|---|---|---|---|
| `ANDROID_SENSOR_BLACK_LEVEL_PATTERN` | `IspParams.blc` (R,Gr,Gb,B) | `isp_params.rs` | ✅ Mapped |
| `ANDROID_COLOR_CORRECTION_GAINS` | `IspParams.wb` (R,G,B gains) | `isp_params.rs` | ✅ Mapped |
| `ANDROID_COLOR_CORRECTION_MATRIX` / `ANDROID_COLOR_CORRECTION_TRANSFORM` | `IspParams.ccm` (3×3 matrix) | `isp_params.rs` | ✅ Mapped |
| `ANDROID_TONEMAP_CURVE_BLUE/GREEN/RED` | `IspParams.tone.curve_lut` (7 control pts) | `isp_params.rs` | ✅ Mapped |
| `ANDROID_TONEMAP_GAMMA`, `ANDROID_TONEMAP_CONTRAST_CURVE` | `IspParams.tone.gamma`, contrast | `isp_params.rs` | ✅ Mapped |
| `ANDROID_SHARPENING_STRENGTH` | `IspParams.sharpen.amount` | `isp_params.rs` | ✅ Mapped |
| `ANDROID_NOISE_REDUCTION_STRENGTH` | `IspParams.denoise.spatial_strength` | `isp_params.rs` | ✅ Mapped |
| `ANDROID_LENS_DISTORTION` k1/k2 | `IspParams.lens.distortion_k1/k2` | `isp_params.rs` | ✅ Mapped |
| `ANDROID_SENSOR_EXPOSURE_TIME`, `ANDROID_SENSOR_SENSITIVITY` | **No AE param in IspParams** | — | ❌ Missing |
| `ANDROID_LENS_SHADING_MAP` | **No LSC param** | — | ❌ Missing |
| `ANDROID_CONTROL_AE_MODE`, `ANDROID_CONTROL_AWB_MODE`, `ANDROID_CONTROL_AF_MODE` | **No 3A mode routing** | — | ❌ Missing |
| `ANDROID_STATISTICS_FACE_RECTANGLES` | `FrameStats.faces` | `isp_params.rs` | ⚠️ Collected but unused |
| OIS/Gyro integration | No IMU data path | — | ❌ Missing |
| `ANDROID_SENSOR_TIMESTAMP` | `IspFrame.timestamp_ns` | `pipeline/types.rs` | ✅ Mapped |
| `ANDROID_SENSOR_ROLLING_SHUTTER_SKEW` | Not used | — | ❌ Missing |

### Entry Points for Metadata

- **HAL service**: `cam-binder/src/cam_hal_service.rs` — receives camera3 capture requests
- **HAL device ops**: `cam-hal-android/src/lib.rs` — `device_process_capture_request()` receives `settings` pointer but **discards it**
- **Camera adapter**: `cam-hal-android/src/adapter.rs` — `AndroidCameraAdapter` passes raw frames only
- **HAL bridge**: `cam-binder/src/hal_bridge.rs` — `HardwareBufferBridge` for zero-copy (not connected to pipeline)
- **Controller API**: `cam-isp/src/controller_api.rs` — `ControllerApi` trait for 3A computation
- **Neural controller**: `cam-isp/src/neural_controller.rs` — neural 3A model (267-in → 20-out)

### Current HAL Data Flow (broken)

```
camera3_capture_request_t
├── settings (camera_metadata_t*)    ──→ DISCARDED ❌
│     ├── AE results
│     ├── AWB results
│     └── AF results
└── stream_buffer (AHardwareBuffer*) ──→ AndroidCameraAdapter
                                          └── raw bytes → ISP pipeline
                                                          └── ControllerApi
                                                               └── recomputes 3A from frame stats
                                                                    (ignoring HAL metadata)
```

### Desired HAL Data Flow

```
camera3_capture_request_t
├── settings ──→ MetadataParser ──→ IspParams
│     ├── AE    → blc, exposure     │
│     ├── AWB   → wb, ccm           ├── unifies HAL + computed params
│     └── AF    → lens              │
│                                   │
└── stream_buffer ──→ AHardwareBufferBridge ──→ MNN Vulkan (zero-copy)
                                                   │
                                                   └── IspParams → blocks
```

---

## 2. Performance Baseline (4K→FHD, MNN Vulkan)

### Current Timing (Release Build, Phone 1 — SD720G / Adreno 618)

```
Steady State: 25.4 ms/frame (39.3 FPS)
├── tensor_assign (sess.resize cached + param upload)   0.3 ms  (1%)
│     └── sess.resize() (shape cached)                 ~13 µs
│     └── set_extra_inputs (ISP params)                 ~0.3 ms
└── inference (mnn_run_with_output)                     23.0 ms (91%)
      └── Vulkan GPU kernels                            ~18 ms  (after warm)
      └── Tensor setup/readback                         ~5 ms

First frame:  35.6 ms (session pool create + shader compile)
Warm frames:   3 required for stable timing
```

### Burst Headroom (30 FPS = 33.3 ms interval)

- **Single frame**: 30.9 ms → **2.4 ms spare** (not enough for burst)
- **Burst 2**: 61.8 ms out of 66.6 ms → **4.8 ms spare**
- **Burst 3**: 92.7 ms out of 100 ms → **7.3 ms spare**

---

## 3. Optimization Opportunities

### [P0] Skip Redundant `sess.resize()` — *Immediate, ~30 min*

**Problem**: `MnnEngine` calls `sess.resize()` every frame (7.3ms) even though the input tensor shape never changes at fixed resolution.

**Fix**: Cache tensor shapes at engine build time. Only `resize()` on resolution change. This is safe because the session pool already allocates fixed-size sessions.

**Savings**: **7.3 ms/frame** (24% of total)

**After**: 30.9 → **23.6 ms/frame** (42 FPS, **9.7 ms spare** at 30 FPS)

### [P0] HAL Metadata Routing — *High impact, 1 week*

**Problem**: `device_process_capture_request()` in `cam-hal-android/src/lib.rs` receives `camera3_capture_request_t.settings` (camera_metadata_t*) but never parses it.

**Fix**: 
1. Add a `camera_metadata` parser (or bind to Android's libcamera_metadata)
2. Extract AE/AWB/AF results → populate `IspParams`
3. Skip redundant frame-analysis in `ControllerApi` when HAL metadata is fresh

**Savings**: Eliminates redundant 3A computation (~1ms) + aligns SoftISP with Android camera stack

### [P1] Async Double-Buffering — *Medium, ~2 days*

**Pattern**: While Vulkan processes frame N, CPU prepares frame N+1 (copy RAW into next session slot + compute params).

```
Frame N:   [CPU prep] [GPU infer]
Frame N+1:              [CPU prep] [GPU infer]
                         └── hidden latency
```

**Savings**: Overlaps 5-7ms of CPU work with GPU inference

### [P1] MNN Vulkan Workgroup Tuning — *Medium, ~4 hours*

**Problem**: Default Vulkan workgroup size may not be optimal for Adreno GPU.

**Fix**: Call `MNNVulkanQueryOptimalWorkgroup` at init and set via `MNNVulkanSetWorkgroupPreset("Adreno")` or `MNNVulkanSetSessionWorkgroup`.

**Savings**: **2-4 ms** inference speedup (10-20%)

### [P2] MNN Session Warmup at Init — *Low, ~2 hours*

**Problem**: First frame takes 35.6ms (session pool init + shader compilation) vs 30.9ms steady state.

**Fix**: Run a dummy frame through all sessions at init time (pre-warm shader cache).

**Savings**: First-frame latency penalty eliminated (critical for camera startup latency)

### [P2] Pipeline Block Fusion — *High effort, ~1 week*

**Problem**: 7+ separate ONNX blocks → multiple kernel launches, intermediate tensor readback.

**Fix**: Fuse `FcsBlock` + `LdciBlock` + `EeBlock` + `DisplayBlock` into a single ONNX model. This removes intermediate tensor round-trips.

**Savings**: **1-3 ms** from reduced kernel launch overhead

### [P2] AHardwareBuffer Zero-Copy — *High effort, ~2 weeks*

**Problem**: RAW data is copied from heap `Vec<u8>` into MNN tensor (7.5ms tensor_assign). With AHardwareBuffer (gralloc), MNN can use the GPU fd directly via Vulkan external memory.

**Fix**: Connect `HardwareBufferBridge` (cam-binder) → MNN Vulkan `MNNVulkanSetConstBuffer` with GPU memory fd.

**Savings**: **7.5 ms** tensor_assign eliminated entirely

### [P3] Adaptive Resolution Scaling

**Idea**: On heavy scenes (or thermal throttle), reduce input resolution for some blocks (e.g., LDCI, EE at half-res). This trades quality for speed when needed.

---

## 4. Consolidated Optimization Path

### Phase 1 — Quick Wins (this week)

| # | Task | Files | Est. Savings | Effort |
|---|---|---|---|---|
| 1 | Cache sess shapes, skip redundant resize | `mnnengine.rs` | **7.3 ms** | 30 min |
| 2 | Add workgroup tuning for Adreno | `mnnengine.rs`, `mnn_sys.rs` | **2-4 ms** | 4 hr |
| 3 | Session pool warmup at init | `mnnengine.rs`, `engine.rs` | Eliminate first-frame penalty | 2 hr |

**Phase 1 target**: **18-20 ms/frame** (50+ FPS, **13-15 ms spare**)

### Phase 2 — HAL Integration (next sprint)

| # | Task | Files | Est. Savings | Effort |
|---|---|---|---|---|
| 4 | Parse camera_metadata in `device_process_capture_request` | `cam-hal-android/src/lib.rs` | Architectural | 1 week |
| 5 | Route AE/AWB/AF → IspParams | `controller_api.rs`, `isp_params.rs` | Architectural | 2 days |
| 6 | Implement async double-buffering | `mnnengine.rs`, `unified_pipeline.rs` | **5-7 ms hidden** | 2 days |

### Phase 3 — Zero-Copy & Fusing (longer term)

| # | Task | Files | Est. Savings | Effort |
|---|---|---|---|---|
| 7 | Connect AHardwareBuffer → MNN Vulkan external memory | `hal_bridge.rs`, `mnnengine.rs` | **7.5 ms** | 2 weeks |
| 8 | Fuse adjacent blocks into single ONNX | `pipeline/build.rs`, `profile.rs` | **1-3 ms** | 1 week |
| 9 | Adaptive resolution scaling | `profile.rs`, `auto_profile.rs` | Variable | 1 week |

---

## 5. Burst Capacity Projection

| Scenario | Frame Time | 30fps Spare | Burst 3 (99ms) Spare | Max Burst Depth |
|---|---|---|---|---|
| **Current** (after resize caching) | 25.4 ms | 7.9 ms | 23.8 ms | 3 frames |
| **Phase 1** (workgroup tuning) | 20 ms | 13.3 ms | 39 ms | 5 frames |
| **Phase 2** (async: hidden prep) | 20 ms | 13.3 ms | ~60 ms | 5+ frames |
| **Phase 3** (zero-copy) | 12 ms | 21.3 ms | 63 ms | 8 frames |

---

## 6. Files Requiring Changes

### Performance
- `cam-isp/src/mnnengine.rs` — sess.resize caching, workgroup tuning, pool warmup
- `cam-isp/src/mnn_sys.rs` — FFI for `MNNVulkanQueryOptimalWorkgroup`, `MNNVulkanSetWorkgroupPreset`
- `cam-isp/src/pipeline/build.rs` — ONNX model fusion
- `cam-isp/src/profile.rs` — block selection for fused models

### HAL Integration
- `cam-hal-android/src/lib.rs` — parse camera_metadata_t in `device_process_capture_request`
- `cam-hal-android/src/adapter.rs` — metadata → IspParams routing
- `cam-binder/src/hal_bridge.rs` — connect HardwareBufferBridge to MNN inference
- `cam-isp/src/controller_api.rs` — accept external 3A params from HAL
- `cam-isp/src/isp_params.rs` — add exposure/sensitivity fields

### Build & Config
- `.cargo/config.toml` — keep LD_PRELOAD runner (Android linker namespace)
- `build.rs` — aarch64-v8a path (done)
- `lib/.gitignore` — add `*.so` to prevent committing prebuilt binaries
