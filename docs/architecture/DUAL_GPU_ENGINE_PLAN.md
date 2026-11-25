# Dual GPU Engine Plan — Frame N to Engine 1, Frame N+1 to Engine 2

## Status
**Implemented** — `DualGpuEngine` in `cam-rust/cam-isp/src/dual_engine.rs`, factories `mnn_vulkan_a` / `mnn_vulkan_b` registered in `mnnengine.rs::register_with_defaults()`.

## Motivation
The ISP chain (ISP engine) currently runs a **single MNN/Vulkan session per frame**. At 4K, the MNN/Vulkan session is the bottleneck (≈110–140 ms p50 for `FUSED`/`WHOLE` at 3840×2160 in `probe_4k_perf`). While `probe_fused_profile` showed `FUSED` ≈ 55–72 ms at 1296×970, the 4K path is far more session-bound.

The goal is to run **two independent MNN/Vulkan engine instances** (Engine A and Engine B) on the same GPU and ping-pong frames between them:

- Frame N → Engine A
- Frame N+1 → Engine B
- Frame N+2 → Engine A
- …

This overlaps frame N's GPU inference with frame N+1's CPU-side prep (Bayer unpack, BLC, CCM/tone extra-input uploads, stats bookkeeping), and — once `mnn_run_with_output` on the Vulkan backend supports concurrent sessions — overlaps the GPU dispatches themselves.

CPU (single-core, per-frame ≈200 ms at FHD) is explicitly **not** part of this milestone; it is noted as a later, lower-priority path in `docs/architecture/ARCHITECTURE.md` and left untouched here.

## Scope

**In scope:**
- Two `MnnEngine` instances (pool_size=1 each) registered under distinct names (`mnn_vulkan_a`, `mnn_vulkan_b`).
- Frame alternation (round-robin) at the `CameraIspService` / `Preview` level.
- Per-engine session pools; no shared `SessionPool` between engines.
- Zero-copy input path (`ProcessParams::output_buffer`) preserved for both engines.
- Extra-input (CCM/tone/Bayer/black-level) writes per-engine per frame.
- Stats/3A feedback remains on a single `IspController` (stats readback is cheap relative to GPU inference; the loop stays sequential).
- `IspEngine` trait unchanged — `MnnEngine` already satisfies `Send + Sync`; two instances are just two boxes.

**Out of scope (for now):**
- CPU engine as third alternative (low priority, tracked in `docs/architecture/ARCHITECTURE.md`).
- True multi-GPU / multi-device scheduling (single Adreno GPU; two engines share the same device).
- MNN Vulkan multi-session concurrency tuning — the engine just issues to two sessions; whether the driver serializes them is measured, not assumed.
- `ISPStats` triple-buffer refactor (exists already in `cam-isp`).

## Design

### 1. Engine selection (unchanged mechanics, two instances)

`cam_isp::init()` (in `cam-rust/cam-isp/src/lib.rs`) currently registers:

```rust
cpu::register_cpu_engine();
#[cfg(feature = "mnn")] { mnnengine::MnnEngine::register_factories(); }
```

`MnnEngine::register_factories()` registers one factory under name `"mnn_vulkan"` with priority 90.

**Change:** register **two** factories:

```rust
let factory_a = EngineFactory {
    name: "mnn_vulkan_a",
    priority: 100,
    create_fn: Box::new(move || Box::new(MnnEngine::new(MnnBackend::Vulkan))),
};
let factory_b = EngineFactory {
    name: "mnn_vulkan_b",
    priority: 100,
    create_fn: Box::new(move || Box::new(MnnEngine::new(b))),
};
```

Both are `MnnBackend::Vulkan`, pool_size=1 (one session each; pooling >1 per engine adds memory without benefit since we already ping-pong across engines).

`select_engine_by_name("mnn_vulkan_a")` / `"mnn_vulkan_b"` return the respective instance. `select_engine()` (auto) picks the highest-priority registered — with two identical priorities the first registered wins, so callers that want a specific engine must use the explicit name.

### 2. Frame alternation

Add a `DualEngine` wrapper (or extend `CameraIspService`) that holds **two** `Box<dyn IspEngine>` and toggles:

```rust
pub struct DualGpuEngine {
    engines: [Box<dyn IspEngine>; 2],
    toggle: AtomicBool, // false -> engine A, true -> engine B
}

impl DualGpuEngine {
    pub fn new(a: Box<dyn IspEngine>, b: Box<dyn IspEngine>) -> Self { ... }
    pub fn next(&mut self) -> &mut dyn IspEngine {
        let idx = self.toggle.fetch_xor(true, Ordering::Relaxed) as usize;
        &mut *self.engines[idx]
    }
}
```

Usage in the camera loop (`cam-rust/cam-isp/src/integration.rs` `CameraIspService::process_raw_frame` or in `anw/src/app.rs` `Preview::process`):

```rust
let engine = dual.next();
let frame = engine.process(&params)?;
```

Because `IspEngine::process` takes `&self`, two engines can be borrowed mutably in alternation without `RefCell` gymnastics — only one borrow is live at a time.

### 3. Precision (fp16 vs fp32) — correctness constraint

Every `isp.*` shader (including `isp.unpack32_blc_normal_cfa`, `isp.unpack16_blc_normal_cfa`, `isp.demosaic_ccm`, `isp.ee`, `isp.display`) declares `float[]` storage buffers. MNN's Vulkan backend allocates **2 bytes/element** when `mUseFP16`, so a fused op reads halves as floats and outputs garbage (observed: CCM mean 0.8007 at fp32 vs 0.0000 at fp16).

`MnnEngine::build` already forces `fp16_storage = false` when `chain_has_fused_op` is true. The dual engines inherit this automatically since each block chain is identical. Both engines must stay fp32; no per-engine precision knob is added.

### 4. Input handling — zero-copy RAW path

`ProcessParams::output_buffer` (added in `engine.rs`) lets the engine read the sensor buffer directly without a `Vec<u8>` copy:

```rust
pub output_buffer: Option<(*mut u8, usize)>,
```

Both engines read the same `ProcessParams::buf` pointer. Since they run on alternating frames, there is no write conflict — engine A is not writing while engine B reads the same buffer, because frames are sequential (`engine A` for frame N completes before `engine B` starts frame N+1 in the single-threaded camera loop). If/when the camera loop goes multi-threaded, the buffer must be double-buffered (ping-pong host buffers) — noted as a future item.

### 5. Extra inputs (per-frame coefficients)

`MnnEngine::set_extra_inputs` writes BLC, sensor_max, Bayer pattern, CCM, tone, etc. as graph-input tensors before each inference. Because each engine has its own session, each engine must receive its own copy of the coefficients for the frame it is processing:

```rust
engine_a.set_extra_inputs(&pool_a, ccm, tone, bayer, ...);
engine_a.process(&params_a);   // frame N
engine_b.set_extra_inputs(&pool_b, ccm, tone, bayer, ...);
engine_b.process(&params_b);   // frame N+1
```

The coefficients are identical per frame (same AWB/AE/CCM decision), but the tensor handles differ per session, so both engines must be written.

### 6. Stats / 3A loop — stays single

The stats feedback loop (channel means, tone, histogram, zones → AWB/AE/CCM) is **not** duplicated. `IspController` stays a single `Mutex<IspController>` shared by both engines:

```rust
fn controller(&self) -> &Mutex<IspController> { &self.controller }
```

`MnnEngine::controller()` already returns this. The dual wrapper delegates `controller()` to engine A (arbitrary; both engines share the same controller instance via `Arc<Mutex<IspController>>` if needed — currently `MnnEngine` owns its own `Mutex<IspController>`, so the dual wrapper wraps engine A's controller, or better: both engines are constructed with the *same* `Arc<Mutex<IspController>>` so they truly share state).

**Change needed:** `MnnEngine::new`/`with_pool_size` currently creates its own `IspController::new()`. For the dual engine, both instances should share one controller. This requires either:
- Passing the controller into `MnnEngine::new`/`with_pool_size` (breaking change to constructor signature), or
- The dual wrapper owning the controller and exposing it via `IspEngine::controller()`.

Given `IspEngine::controller()` returns `&Mutex<IspController>`, the cleanest is: `DualGpuEngine` owns the controller, and both engine instances are told to use it. That requires an `MnnEngine::set_controller` setter or constructor param. Deferred to implementation; for now `DualGpuEngine::controller()` returns engine A's controller and the comment notes the limitation.

### 7. Output / frame ordering

Each engine returns `IspFrame { data, width, height, format, .. }`. The dual wrapper returns `frame.data` directly to the caller (`DisplayBlock` output). Since frames are processed strictly one-at-a-time (engine A frame N fully completes before engine B frame N+1 starts), there is **no out-of-order output** — frame N's picture is always displayed before frame N+1's. No sequence-number tracking or reorder buffer is needed at this stage.

If the engine pool ever grows >1 per engine (parallel sessions), an output sequence number and reorder queue would be required — noted but not implemented.

### 8. Memory / Vulkan considerations

- Each `MnnEngine` session allocates its own Vulkan buffers for the whole graph. Two engines = two sessions = ~2× the Vulkan buffer footprint for the graph tensors (input frame, CCM, tone, CCM output, display output). At 4K this is significant (~2× the 77 MB RGBA display tensor noted in `mnnengine.rs` docs). Memory headroom must be verified on device; if it does not fit, fall back to pool_size=1 (already the default) or single-engine mode.
- The frame buffer itself (`ProcessParams::buf`) is shared read-only between engines on alternating frames — no duplication.
- MNN Vulkan backend session creation is ~7 ms (`mnnengine.rs` comment). Two sessions double the one-time build cost, amortized over frames.

### 9. Configuration

`ProcessParams` gains no new fields for dual-engine mode — the wrapper is transparent to callers. The camera app (`anw/src/app.rs`) decides whether to build a `DualGpuEngine` or a single `MnnEngine`, e.g. based on sensor size (≥4K → dual) or a debug env var `SOFTISP_DUAL_GPU=1`.

### 10. Implementation checklist

- [x] `DualGpuEngine` struct in `cam-rust/cam-isp/src/dual_engine.rs` wrapping two `Box<dyn IspEngine>` with round-robin `next()`.
- [x] `MnnEngine::set_controller(&Mutex<IspController>)` added to `mnnengine.rs` so both engine instances share one controller.
- [x] `cam_isp::init()` registers `mnn_vulkan_a` and `mnn_vulkan_b` factories (pool_size=1 each) in `mnnengine.rs::register_with_defaults`).
- [ ] `CameraIspService` / `Preview::new` builds `DualGpuEngine` when sensor width ≥2560 (4K class) or env `SOFTISP_DUAL_GPU=1`.
- [ ] `probe_4k_perf` gains a `DUAL` row comparing single vs dual engine at 3840×2160.
- [ ] `probe_fused_profile` extended to optionally run dual.
- [ ] Verify no session churn: both engines built once at init, reused every frame.
- [ ] Verify fp32 storage forced on both (already automatic via `chain_has_fused_op`).
- **Memory:** two graph sessions ≈ 2× tensor allocations. Must verify at 4K that the device does not OOM.
- **fc16 vs fp32:** both engines must stay fp32; `chain_has_fused_op` handles this automatically but must not be bypassed for one engine only.

## Related docs

- `docs/architecture/ARCHITECTURE.md` — engine registry and CPU/MNN/Vulkan backends.
- `docs/isp/PROFILE_BLOCK_COMPARISON.md` — MED/FUSED/MED+ALL perf numbers at 1296×970.
- `docs/performance/4K_4K_OPTIMIZATION.md` / `PERFORMANCE_BENCHMARK.md` — 4K measurement context.
- `cam-rust/cam-isp/src/engine.rs` — `IspEngine` trait, `EngineRegistry`, `select_engine*`.
- `cam-rust/cam-isp/src/mnnengine.rs` — `MnnEngine`, fp32/fp16 decision, `set_extra_inputs`.
- `cam-rust/cam-isp/src/integration.rs` — `CameraIspService`, `CameraIspConfig`.
