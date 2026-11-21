# INT16/FLOAT32 RAW Input Path for `isp.unpack*_blc_normal_cfa` — Design Summary

**Author:** softisp engineering  
**Scope:** `cam-rust/cam-isp/` + `MNN/source/backend/vulkan/` + `MNN/source/shape/` + `MNN/source/core/`  
**Status:** implemented & unit-tested (Rust), compiled SPIR-V (MNN), pending APK validation on device

---

## 1. Problem Statement

The ISP chain's head block reads the raw Bayer sensor plane and converts it into the four-plane CFA quad `[1,4,H/2,W/2]` that every downstream stage (demosaic → CCM → LDCI → EE → display) consumes.

Before this work the head was one of:

| Head | Graph input | Blocks in chain | Host work |
|------|-------------|-----------------|-----------|
| Packed INT32 (`isp.unpack_blc_normal_cfa`) | `[1,2,H,W/2]` INT32 | RawInputPackedBlock → UnpackBlock → NormalizeBlock → BlcBlock → CfaBlockPacked | host splits u16 pairs into even/odd i32 lanes; Cast INT32→FLOAT32 |
| Native INT16 (`RawInput16Block`+`UnpackCfaBlock`) | `[1,1,H,W]` INT16 | RawInput16Block → UnpackCfaBlock (Conv) → NormalizeBlock → BlcBlock | none (INT16 passed verbatim), but **4 separate GPU dispatches** |

The MNN converter upcasts a declared-INT16 input to FLOAT32, so a "native INT16" head still ends up as FLOAT32 inside MNN while the engine passes a u16 buffer — a type mismatch that silently wastes memory (2× buffer size per session at 5 Mpx ≈ 19 MB/session).

Every head also pays **five full-frame host↔GPU round trips** just to arrive at the same `[1,4,H/2,W/2]` quad the fused shader could produce in one dispatch.

---

## 2. Solution Overview

Add two fused ISP ops to the MNN Vulkan backend and matching Rust blocks, so `FUSE_STAGES::ALL` (the `FUSED`/`UNIFIED`/`MED-fused` profiles) emits **one** `isp.unpack*_blc_normal_cfa` Extra dispatch instead of five separate blocks:

| Op | Graph input | Shader reads | GPU dispatch |
|----|-------------|--------------|--------------|
| `isp.unpack32_blc_normal_cfa` | `[1,1,H,W]` FLOAT32 | engine converts u16→f32 (`RawFloat`), then shader divides by `sensor_max`, subtracts per-colour BLC, extracts Bayer quad | 1 dispatch |
| `isp.unpack16_blc_normal_cfa` | `[1,1,H,W]` INT16 | engine passes u16 verbatim (`RawInt`/`SplitInt16`), shader converts INT16→FLOAT32 + above | 1 dispatch |

`isp.unpack_blc_normal_cfa` (packed INT32) remains as the legacy fallback for models that declare `[1,2,H,W/2]` INT32.

The head-selection order in `profile_builder::build_blocks` is:

```
isp.pipeline (whole_pipeline)   >  checked first, subsumes everything
isp.unpack32_blc_normal_cfa     >  if InputMode is RawFloat (host already converted to f32)
isp.unpack16_blc_normal_cfa     >  if InputMode is RawInt/SplitInt16 (shader converts)
isp.unpack_blc_normal_cfa       >  packed INT32 fallback
RawInput16Block+UnpackCfa       >  primitive INT16 path (force_input16)
RawInputPackedBlock+...         >  primitive packed INT32 path (use_unpack)
RawInputBlock+Normalize+CfaBlock > legacy FLOAT path
```

---

## 3. What Was Built

### 3.1 MNN shaders (compiled to SPIR-V, headers regenerated)

| File | Binding | Purpose |
|------|---------|---------|
| `isp_spv/isp_unpack16_blc_normal_cfa.comp` + `.spv` + `_spv.h` | `binding 0 = Uniforms` (W/H/sensor_max/gain/BLC[4]/bayer_pattern), `binding 3 = InBuf INT16`, `binding 4 = OutBuf float[4]`, `binding 5 = BlcBuf`, `binding 6 = MaxBuf`, `binding 7 = BayerBuf` | Converts INT16→FLOAT32, normalizes, subtracts BLC, unpacks 2×2 Bayer sites into planar quad |
| `isp_spv/isp.unpack32_blc_normal_cfa.comp` + `.spv` + `_spv.h` | same bindings, but `binding 3 = InBuf FLOAT32` | Host already converted u16→f32; shader does divide+BLC+quad extract |

**Why bindings 3/4 instead of 1/2:** MNN's `VulkanFuse` scans every `isp_*.spv` at op construction (`scanSpvBindings`) and builds a universal descriptor map: `binding 0 = params`, `binding 3 = input frame`, `binding 4 = output frame`, `binding 5.. = metadata`. Of the 80 shipped `isp_*.spv`, 66 already declare `InBuf@3/OutBuf@4`. Declaring our new shaders the same way keeps them consistent with every other ISP op — declaring 1/2 would make them read the `isp.pipeline` DEBUG TAP and write into CONFIG, which is exactly the historical bug this comment block documents.

### 3.2 MNN shape inference (`ShapeIspExtra.cpp`)

Added `isp.unpack16_blc_normal_cfa` / `isp.unpack32_blc_normal_cfa` to the existing `isp.unpack_blc` / `isp.unpack_blc_normal_cfa` shape rule, with a critical distinction:

| Op family | Input declared | Output width derivation |
|-----------|---------------|-------------------------|
| `isp.unpack_blc(_normal_cfa)` | `[1,2,H,W/2]` packed words/row | dim[3] **is** W/2 → output `[1,4,H/2,W/2]` |
| `isp.unpack16/32_blc_normal_cfa` | `[1,1,H,W]` one sample per pixel | dim[3] **is** W → output `[1,4,H/2,W/2]` (must halve W) |

Without this rule MNN's generic Extra fallback copies the input shape to every output, so downstream ops (CCM, demosaic) see `[1,1,H,W]` instead of `[1,4,H/2,W/2]`, and `Session::resize` fails with `COMPUTE_SIZE_ERROR` (error code 3) — **the exact crash the user saw ("black screen", apk installed but no pipeline output).**

### 3.3 MNN opset registration (`IspOpsetValidate.hpp`)

Registered both new op names so MNN's Extra-op allowlist accepts them at graph load.

### 3.4 MNN lookup table (`IspSpvLookup.hpp/.cpp`)

Added extern declarations and map entries for both new op names → their SPIR-V headers, plus `#include`s in the `.cpp`.

### 3.5 Rust blocks (`cam-isp/src/blocks/`)

| File | Block | `input_elem_type` | Op emitted |
|------|-------|-------------------|------------|
| `normalize_blc_unpack16_cfa.rs` | `NormalizeBlcUnpack16Cfa` | 5 (INT16) | `isp.unpack16_blc_normal_cfa` |
| `normalize_blc_unpack32_cfa.rs` | `NormalizeBlcUnpack32Cfa` | 2 (FLOAT32) | `isp.unpack32_blc_normal_cfa` |

Both export `with_concrete_dims`, `with_sensor_max`, `with_blc_offsets`, `with_bayer_pattern`, `with_opset_mode`, plus `nodes()` (Custom/MnnIspMin → fused Extra node; Primitive → Div→Sub→Conv fallback), `extra_input_defaults()` (BLC/sensor_max/bayer_pattern tensors), and `is_graph_input_placeholder_for()` (true under MnnIspMin so GraphComposer doesn't declare a second boundary block).

### 3.6 Profile & builder wiring (`profile.rs`, `profile_builder.rs`)

- `FuseStages` gained `unpack32_blc_normal_cfa` (in `ALL`, off in `NONE`).
- `FuseStages::fused_head_shader_paths()` documents the shader-path order and that the actual choice is made by `MnnEngine::InputMode`, not by the enum alone.
- `profile_builder::build_blocks` checks `unpack32_blc_normal_cfa` before `unpack16_blc_normal_cfa` before `unpack_blc_normal_cfa`.
- `FUSED`, `WHOLE`, `UNIFIED` profiles all use `FuseStages::ALL`, so the fused head is active by default for fused profiles.

---

## 4. Why the Bug Happened (root cause)

**Symptom:** APK builds and installs, app launches, but the ISP pipeline produces no output — a black screen. Logcat / engine logs show:

```
MNN inference failed: n=-3 ... runSession failed: MNN error code=3 (COMPUTE_SIZE_ERROR)
```

**Root cause:** MNN's `Session::resize` calls the Extra-op **shape computer** (`ShapeIspExtra`) before allocating GPU buffers. The shape computer declares each output's dimensions. For `isp.unpack16_blc_normal_cfa` / `isp.unpack32_blc_normal_cfa` there was **no shape rule at all** — these op names were new and absent from `ShapeIspExtra.cpp`. MNN's generic fallback for an unrecognized Extra op **copies the input tensor's shape to every output**:

- Input declared by `NormalizeBlcUnpack16Cfa::input_value_info()`: `[1,1,H,W]` (e.g. `[1,1,1080,1920]`)
- Fallback-declared output: `[1,1,1080,1920]` (1 channel, full resolution)

But the fused shader writes a **4-plane quad at quarter resolution**: `[1,4,540,960]`. The Vulkan backend then binds an output tensor whose declared size (1080×1920×1 element) does not match what the SPIR-V shader expects (540×960×4 elements). The descriptor-set layout / command-buffer mismatch is detected during pipeline encoding → `COMPUTE_SIZE_ERROR` → `Session::run` aborts → no frame is produced → black screen.

**The fix:** add an explicit shape rule in `ShapeIspExtra.cpp` so `isp.unpack16_blc_normal_cfa` / `isp.unpack32_blc_normal_cfa` output `[1,4,H/2,W/2]` float, and account for the fact that these ops read a **full row** (one sample per pixel) while the packed `isp.unpack_blc_normal_cfa` reads **packed words** (dim[3] already = W/2). The output width is therefore:

```
outW = full_row_input ? W / 2 : W   // W/2 for INT16/FLOAT32, W for packed INT32
```

This is the same class of bug that was previously fixed for `isp.crop_blc_norm_cfa` (comment in VulkanFuse.cpp lines 652-653) and `isp.cfa` — every fused ISP op needs its own shape contract because MNN cannot infer "half resolution, 4 channels" from a raw tensor shape alone.

---

## 5. How `raw_int16` pairs with `raw_f32` for path selection

The user wants `raw_int16` and `raw_f32` as explicit input-path settings that select the correct fused head. Here is how they map:

| App setting | Engine `InputMode` | Graph input dtype | MNN shader | Rust block | Host work |
|-------------|-------------------|-------------------|------------|------------|-----------|
| `raw_int16` | `RawInt` / `SplitInt16` | INT16 (`input_elem_type()==5`) | `isp.unpack16_blc_normal_cfa` (bindings 3=INT16) | `NormalizeBlcUnpack16Cfa` | passes u16 verbatim; shader converts INT16→FLOAT32 |
| `raw_f32` | `RawFloat` / `SplitFloat` | FLOAT32 (`input_elem_type()==2`) | `isp.unpack32_blc_normal_cfa` (bindings 3=FLOAT32) | `NormalizeBlcUnpack32Cfa` | engine widens u16→f32 once; shader divides+BLC+unpacks |

`MnnEngine::build()` infers `InputMode` from the converted model's declared input dtype (lines 1483-1557 of `mnnengine.rs`):

- `split && is_float` → `SplitFloat` (channels==2, FLOAT32)
- `split && bits==16` → `SplitInt16` (channels==2, INT16 kept by converter via `--preserveInputType`)
- `split && otherwise` → `SplitInt32` (packed INT32)
- `!split && is_float` → `RawFloat` (channels==1, FLOAT32)
- `!split && otherwise` → `RawInt` (channels==1, INT16)

`profile_builder` then picks the fused head by checking `fuse_stages.unpack32_blc_normal_cfa` before `unpack16_blc_normal_cfa`. Because both are in `FuseStages::ALL`, the deciding factor at runtime is the **element type the engine measured**, not a separate app flag:

```
if InputMode is RawFloat / SplitFloat → isp.unpack32_blc_normal_cfa
elif InputMode is RawInt / SplitInt16 → isp.unpack16_blc_normal_cfa
else                                 → isp.unpack_blc_normal_cfa (packed INT32)
```

If the app wants an explicit toggle independent of model dtype, add a `ProcessParams::raw_input_mode` enum (`RawInt16` / `RawF32`) and pass it into `MnnEngine::build()` to override the inferred `InputMode`. The two fused blocks already accept either dtype through their `input_elem_type()` and `input_value_info()`, so no Rust-side change is needed beyond wiring that enum through `ProcessParams` → `MnnEngine` → head selection.

---

## 6. Memory impact

At 5 Mpx (3840×2160):

| Convention | Bytes/pixel | Per session | 4 sessions |
|------------|-------------|-------------|------------|
| Packed INT32 (`[1,2,H,W/2]`) | 4 | 32 MB | 128 MB |
| Native INT16 kept (`[1,1,H,W]`) | 2 | 16 MB | 64 MB |
| Native INT16 widened to f32 | 4 | 32 MB | 128 MB |

The INT16 fused head halves frame-buffer memory versus the packed INT32 path **and** collapses five dispatches into one — both reduce host↔GPU traffic and peak session memory.

---

## 7. Verification status

- `cargo clippy -p cam-isp --lib --features rectifier -- -D warnings`: 0 errors
- `cargo test -p cam-isp --lib --features rectifier`: 813 passed, 0 failed (includes 4 new `fused_head_*` tests)
- MNN shaders compile with `glslangValidator`; SPIR-V headers regenerated; binding scan confirms `InBuf@3/OutBuf@4` for both new shaders
- `ShapeIspExtra.cpp` updated; MNN C++ side not yet recompiled on device (no NDK toolchain in this environment)
- APK built via `cargo apk2 build -p cam-camera-app`; device install/test pending

---

## 8. Remaining work for device validation

1. Rebuild `libMNN_Vulkan.so` from the updated MNN source (new shaders + shape rule + opset registration) and re-copy to `cam-rust/lib/aarch64-v8a/`.
2. Rebuild APK (`cargo apk2 build -p cam-camera-app`), install, run.
3. Confirm engine log shows `head: isp.unpack16_blc_normal_cfa` (or `...32...`) and `runSession` no longer returns `COMPUTE_SIZE_ERROR`.
4. Compare frame time against the MED primitive baseline (expect ~2.5× speedup at 4K from head fusion alone, per earlier probe data).

---

## 9. Files changed

**softisp repo:**
- `cam-rust/cam-isp/src/blocks/normalize_blc_unpack16_cfa.rs` (new)
- `cam-rust/cam-isp/src/blocks/normalize_blc_unpack32_cfa.rs` (new)
- `cam-rust/cam-isp/src/blocks/mod.rs` (module + re-exports)
- `cam-rust/cam-isp/src/profile.rs` (`FuseStages` fields, `ALL`/`NONE`, `fused_head_shader_paths()`)
- `cam-rust/cam-isp/src/profile_builder.rs` (head-selection order)
- `cam-rust/cam-isp/tests/probe_4k_perf.rs` (warmup error reporting)

**MNN repo:**
- `source/backend/vulkan/buffer/execution/isp_spv/isp_unpack16_blc_normal_cfa.comp` / `.spv` / `_spv.h` (new)
- `source/backend/vulkan/buffer/execution/isp_spv/isp_unpack32_blc_normal_cfa.comp` / `.spv` / `_spv.h` (new)
- `source/backend/vulkan/buffer/execution/IspSpvLookup.hpp/.cpp` (lookup entries)
- `source/shape/ShapeIspExtra.cpp` (shape rule for both new ops, full-row vs packed width handling)
- `source/core/IspOpsetValidate.hpp` (opset allowlist entries)
