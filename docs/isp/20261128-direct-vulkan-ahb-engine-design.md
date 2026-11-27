# Direct Vulkan AHB/VkBuffer passthrough engine design

## Symptom / motivation

The MNN AHB binding path proves that a session-owned tensor can carry an
`AHardwareBuffer` device pointer and reach a Vulkan backend, but it does not
prove that live camera pixels survive the path. The production shim still
reports black frames, and the failure is spread across MNN session wrapping,
tensor format handling, and Vulkan buffer import.

The user requested a smaller architecture that can replace MNN later:

1. Bypass MNN entirely in a testable engine.
2. Consume the RAW16 `AHardwareBuffer` as an opaque byte container through a
   `VkBuffer`, never a sampled `VkImage`.
3. Run a trivial Vulkan compute passthrough or grayscale shader.
4. Write the result into a `VkBuffer`-backed output `AHardwareBuffer`.
5. Hand that output buffer to Android presentation (`ASurfaceTransaction`) or a
   host-visible readback for verification.

## Decisions

- RAW16 pairs are packed into one 32-bit word. The shader masks each 16-bit
  lane and shifts by 16; no float reinterpretation of the raw word is used.
- BLOB and private RAW16 AHBs are opaque byte containers. Vulkan `VkBuffer` has
  no pixel format, so shader layout is the only interpretation.
- The compute shader accepts `uint` words and writes packed BGRA `uint` words.
  It supports two modes:
  - `mode=0`: passthrough, both RAW16 samples expanded to BGRA.
  - `mode=1`: grayscale, both samples converted to grey BGRA.
- The direct engine never calls MNN, `tryResizeSession()`, or
  `Tensor::setDevicePtr()`. It owns its Vulkan device, pipeline, and fences.
- The engine exposes a device context per worker, mirroring the two-worker
  `IspEngine` layout. Context creation is fail-soft: if Vulkan or AHB import is
  unavailable, `process()` returns a normal `IspError` instead of panicking.
- AHB import uses `VK_ANDROID_external_memory_android_hardware_buffer` and
  binds the imported `VkDeviceMemory` to a `VkBuffer` with dedicated allocation.
  No CPU `AHardwareBuffer_lock()`/`unlock()` occurs inside GPU execution; the
  caller may open a CPU window only before submission or after fence wait.
- The engine is selected by a new `direct_vulkan` feature in `anw`, leaving the
  default `mnn_vulkan` path unchanged.

## Layering

```text
anw/app.rs
  ENGINE_NAME = "direct_vulkan" when feature direct_vulkan is enabled
      |
      v
cam_isp::engine registry
      |
      v
DirectVulkanEngine : IspEngine
      |
      v
direct_vulkan.rs FFI
      |
      v
direct_vulkan.cpp (Vulkan compute, AHB external memory)
      |
      +--> input RAW16 AHB / staged BLOB AHB -> VkBuffer
      +--> output BLOB AHB -> VkBuffer
      +--> compute shader -> fence wait -> no host copy on AHB path
```

The existing `AhbDisplay` remains the presentation sink. It needs no Vulkan
context and consumes the output AHB handle directly.

## Scope of the first change

- Add the C++ Vulkan compute module and its Rust FFI.
- Register a `DirectVulkanEngine` behind `cam-isp/direct_vulkan`.
- Wire `anw` so the direct engine gets the same AHB slots as MNN and returns the
  output AHB view to the presenter.
- Add an ignored on-device regression that allocates RAW16/BLOB AHB fixtures,
  writes a known pattern, runs the engine, and verifies non-black output bytes.
- Keep MNN code and its temporary diagnostics unchanged by this feature.

## Verification

1. `cargo fmt --all -- --check`
2. `cargo check -p softisp-anw --features "mnn,direct_vulkan"`
3. `cargo clippy -p cam-isp --lib --features "rectifier,direct_vulkan" -- -D warnings`
4. Focused on-device test:
   ```bash
   LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
     cargo test -p cam-isp --test test_direct_vulkan_ahb --features direct_vulkan \
     -- --ignored --nocapture --test-threads=1
   ```
5. Production shim with `direct_vulkan` selected: no MNN warnings, no black
   frame stats, and no forbidden diagnostic strings in the log.

## Status

Implemented behind `cam-isp/direct_vulkan` and `anw/direct_vulkan`, selected by
`ENGINE_NAME=direct_vulkan`. The C++ module owns one Vulkan device/pipeline per
worker, caches imported AHBs by pointer, and rewrites descriptors only when the
input or output buffer changes. There is no MNN session, no host upload, no
`AHardwareBuffer_lock()`/`unlock()` during dispatch, and no `tryResizeSession()`.

### Supported AHB sources

A device probe confirmed the gralloc contract that blocked the first version:

- `BLOB` + `GPU_DATA_BUFFER` imports as a `VkBuffer` on Adreno 618.
- `RAW16` + `GPU_SAMPLED_IMAGE` reports AHB Vulkan properties (`allocationSize`,
  memory types `0x18`) but `vkAllocateMemory` fails for both plain and dedicated
  chains with `VkResult=5`. It also has no Vulkan image format
  (`format=0 externalFormat=0`), so it cannot be a `VkImage` either.
- `RAW16` + `GPU_DATA_BUFFER` is rejected by gralloc 4.0 before Vulkan sees it.
- Requesting only `CPU_READ_OFTEN` on the ImageReader does not change this: the
  resulting RAW16 AHB still fails `VkBuffer` import.
- `VK_EXT_external_memory_dma_buf` is not exposed by the device, so the fd
  import path is unavailable.

Live MediaNDK frames therefore cannot be imported as Vulkan memory by any
supported route. The direct engine uses the worker-owned BLOB input slot. The
single de-stride copy into that slot is the last CPU step; from that point the
frame is a GPU-owned `VkBuffer`. The output is an `R8G8B8A8_UNORM` AHB with
`GPU_FRAMEBUFFER` usage, bound as a storage `VkBuffer`. SurfaceFlinger reads it
through a hardware overlay path (no CPU software blit), which cut presentation
from ~60–80 ms to ~28 ms and end-to-end frame time roughly in half. See
`20261128-direct-vulkan-latency-analysis.md`.

### Verification

`cargo fmt --all -- --check`, `cargo check -p softisp-anw --features
"mnn,direct_vulkan"`, and clippy for `softisp-anw --all-features` and
`cam-isp --lib --features "rectifier,direct_vulkan"` pass. The ignored on-device
test `test_direct_vulkan_ahb.rs` allocates two BLOB AHBs, dispatches twice
through the same imported buffers, and verifies non-black BGRA output. A
production shim run with `ENGINE_NAME=direct_vulkan` reports non-black frames
(`rgb mean ~22`, `max 255`, `non-black 100.0%`) with no MNN warnings, no AHB
import failure, and no AHB handoff warnings. `shim-debug.sh verify` hashes all
eight uploaded libraries identically.

Two production defects were fixed during verification:

1. The warmup-frame drop consumed a `seq` value without publishing a `Done`,
   so `PipelineRunner::poll()` waited forever at that missing sequence. The
   drop now happens before sequence assignment.
2. The output-slot handoff used a 1M-yield spin loop and warned on normal
   presenter backpressure. It now sleeps on a `Condvar`, preserving the busy
   flag that prevents GPU overwrite while removing the false tear warning.
