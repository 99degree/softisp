# Verified FHD build — 2026-11-15

Pin record for the state that produced a visible, correctly-scaled preview on
device. Two repositories have to agree for an APK to work, and only one of them
is this repo, so the other is recorded here.

Branch: `verified_fhd_20261115`
softisp commit: `7ab9dc9ae8142a5b8bfaf31b4284917474be28d9` (2026-11-15 23:48 +0800)

---

## MNN

The prebuilt libraries in `cam-rust/lib/aarch64-v8a/` are **not** stock MNN.
They are built from a local fork with uncommitted changes, so the commit alone
does not describe them — both parts are pinned here.

| | |
|---|---|
| Repository | `https://github.com/99degree/MNN.git` |
| Branch | `master` |
| Commit | `30d31c1275c5981325f407f8f39c1acab0d83069` |
| Describe | `3.6.1-249-g30d31c127` |
| Commit date | 2026-11-12 20:12:38 +0800 |
| Commit subject | `fix(isp): isp.display_rgba_f16 packs halves with packHalf2x16, not float16_t` |

### Uncommitted delta on top of that commit

Four files are modified in the working tree and are required for the shipped
`libMNN_Vulkan.so`. They are not in the commit above.

1. `source/backend/vulkan/buffer/execution/VulkanFuse.cpp`
   `types.resize(maxIndex+1)` → `types.assign(maxIndex+1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)`.
   `VkDescriptorType`'s zero value is `VK_DESCRIPTOR_TYPE_SAMPLER`, so every
   binding not written by an `input`/`const` attribute kept SAMPLER while its
   SPIR-V declared a storage buffer. `isp.pipeline` hits this exactly: bindings
   1 (`DEBUG TAP`) and 2 (`CONFIG`) are written by neither attribute loop, so
   the descriptor layout disagreed with the shader and the driver rejected the
   op.

2. `source/backend/vulkan/buffer/execution/isp_spv/isp_raw_dummy.comp`
   Added a per-pixel hash mode (`mode = 1`) alongside the existing smooth ramp.
   Independent pixels make a broken stage show as a flat field, which is what
   finally isolated the zeroed EE stage.

3/4. `isp_spv/isp_raw_dummy.spv` and `isp_spv/isp_raw_dummy_spv.h` — the
   recompiled shader and its embedded header, generated from (2).

> To reproduce: check out `30d31c127`, re-apply the four-file delta, and rebuild.
> The delta is not committed anywhere, so it exists only in
> `~/MNN` on this machine.

### Build configuration

Built at `~/MNN/build_isp` with CMake:

| Option | Value |
|---|---|
| `ANDROID_ABI` | `arm64-v8a` |
| `CMAKE_BUILD_TYPE` | `Release` |
| `CMAKE_TOOLCHAIN_FILE` | `android-sdk/ndk/29.0.14206865/build/cmake/android.toolchain.cmake` |
| `MNN_VULKAN` | `ON` |
| `MNN_SEP_BUILD` | `ON` (backends are separate `.so` plugins) |
| `MNN_BUILD_CONVERTER` | `ON` |
| `MNN_ARM82` | `ON` |
| `MNN_LOW_MEMORY` | `ON` |
| `MNN_OPENCL` / `MNN_OPENGL` | `OFF` |
| `MNN_BUILD_SHARED_LIBS` | `ON` |
| `MNN_BUILD_TEST` / `MNN_BUILD_TRAIN` | `OFF` |

`MNN_SEP_BUILD=ON` matters: `libMNN.so` does not auto-load its backends, so
`mnn_session_create()` `dlopen`s `libMNN_Vulkan.so` explicitly.

### Shipped libraries

`cam-rust/lib/aarch64-v8a/`, copied from `~/MNN/build_isp/`:

| File | Bytes | MD5 |
|---|---|---|
| `libMNN.so` | 3 233 488 | `09523ac5b02d7ef4cc41d25610853aaf` |
| `libMNN_Express.so` | 988 136 | `b637912ccec04427fbb5e7ce0c6194df` |
| `libMNN_Vulkan.so` | 2 763 545 | `8038e6f6a1119f80d5a77e811a6f52f6` |
| `libMNNConvertDeps.so` | 6 293 649 | `4059719ebe473ddda378c2f655716baf` |

`libMNN.so` and `libMNN_Express.so` are byte-identical to the build output.
The other two are **post-processed**: Android has no versioned soname, so

```bash
patchelf --replace-needed libvulkan.so.1 libvulkan.so libMNN_Vulkan.so
patchelf --replace-needed libvulkan.so.1 libvulkan.so libMNNConvertDeps.so
```

is what turns `b1aff3c5…` into `8038e6f6…` and `e2d257ef…` into `4059719e…`.
All four must come from the *same* build directory; mixing versions makes the
converter or the backend fail to load.

`libvulkan.so` (the ICD loader) is copied from `/system/lib64/libvulkan.so`.
Termux does not search there, so MNN's `vulkan_wrapper.cpp` cannot find it
otherwise.

---

## What this state was verified to do

- RAW16 accepted at 2592x1940, 4 MNN sessions created on the Vulkan backend.
- MED profile builds and runs; the frame reports a non-zero mean, max 255.
- Preview visible, letterboxed to the surface instead of stretched
  (1296x970 4:3 into a 1080x2264 portrait surface).

### Fixes that made the picture appear

Each was independently sufficient to keep the preview black:

1. `NormalizeBlock/max_val` hardcoded to 65535 while the sensor white level is
   1023 → white pixels normalised to 0.016.
2. Display `Conv(1×1)` weights declared as graph inputs; MNN's Vulkan
   convolution reads kernels from the operator and zero-fills when absent →
   alpha 255, RGB 0.
3. The `FloatRgb` identity path emitted `gamma_exp`/`zero`/`one` as node inputs
   without declaring them → MNN dropped them, optimizer SIGSEGV.
4. `profile.output_format` (graph) and `ProcessParams::output_format` (decoder)
   were independent; a mismatch decoded 4 planes as 3.
5. `EeBlock` defaulted to the `MnnIspMin` opset, emitting an `isp.ee` extra op
   that MNN's converter did not rewrite in the profile's block list → output
   read as zeros. Now emits a plain `Conv`.
6. `present()` mapped each axis independently, stretching 4:3 into 0.48.

---

## Toolchain

| | |
|---|---|
| Device | Android SDK 36, arm64-v8a, Adreno GPU |
| NDK | `29.0.14206865` |
| Rust | pinned by `rust-toolchain.toml` (stable) |
| APK profile | `dev` with `opt-level = 2` (`cargo apk2 build -p cam-camera-app`) |
