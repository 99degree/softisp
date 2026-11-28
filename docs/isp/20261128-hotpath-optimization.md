# Direct Vulkan hot-path optimization

## Question

Is there any way to shorten the remaining preview critical path after the
graphics-output AHB change (~18 fps at 1600x1200 RAW16)?

## Live measurement (instrumented build, before this change)

Earlier `20261128-direct-vulkan-latency-analysis.md` reported `wait avg=13.6ms`.
A fresh instrumented run with per-stage timers showed that number was stale:

| stage | observed | owner |
|-------|----------|-------|
| acquire (`next_frame`) | 5.2–14.7 ms | worker, under camera mutex |
| `raw_stats` | 0.5–1.5 ms | worker |
| de-stride + stage | 0.7–1.6 ms | worker |
| Vulkan dispatch | 3.5–5.9 ms | worker |
| `slot.wait_free()` | **120–150 ms** | worker, presenter handoff |
| `LATP read` (`AHardwareBuffer_unlock/lock`) | 0.7–1.9 ms | presenter |
| `LATP present` (`present_ahb_raw`) | 13–28 ms | presenter |

`[loop]` durations alternated between ~96–99 ms `media_ndk_capture` frames and
~31–34 ms frames. The dominant, unexpected cost was the worker waiting for the
single output slot. `present_ahb_raw()` itself was only 12–28 ms, so the extra
~100 ms happened between presentation and the slot becoming free again.

### Root cause hypothesis

`AhbView::begin_cpu_read()` was called on **every** present. It unlocks and
re-locks the same AHB the surface is still reading, forcing an extra
producer/consumer synchronization round trip. Combined with one output slot per
worker, that delayed SurfaceFlinger's release callback and stalled the next
frame behind it.

## User proposal: write the output AHB as a `VkImage`

The output AHB is `R8G8B8A8_UNORM` + `GPU_FRAMEBUFFER`. Unlike the RAW16 camera
AHB (which has `format=0`, no storage usage and cannot be Vulkan memory), the
graphics output AHB is a first-class Vulkan image.

A one-shot probe on device confirmed it:

```text
IMAGE-PROBE ok=true fmt=37 ext=28 feat=0x7dd83 storage=0x7dd83
types=0x18 reqMask=0x18 mask=0x18 reqSize=0 ahbSize=7680000
allocated=3 bind=0
```

`fmt=37` is `VK_FORMAT_R8G8B8A8_UNORM`, and `storage=0x7dd83` includes
`VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT`. The image imports, allocates and binds
successfully.

## RAW16 → explicit `R16_*` format check

The commonly cited AHB→Vulkan mapping (`AHARDWAREBUFFER_FORMAT_RAW16` →
`VK_FORMAT_R16_UINT`/`R16_UNORM`) was tested explicitly rather than relying on
the driver-reported format. A synthetic RAW16 AHB (`GPU_SAMPLED_IMAGE`) was
probed with `vkCreateImage` + dedicated AHB import for each of `R16_UINT`,
`R16_UNORM` and `R16_SFLOAT`:

```text
fmt=74 (R16_UINT)  feat=0x20000 storage=0x1cc83 mask=0x18 alloc=0 bind=0
fmt=70 (R16_UNORM) feat=0x20000 storage=0x1dd83 mask=0x18 alloc=0 bind=0
fmt=76 (R16_SFLOAT) feat=0x20000 storage=0x1dd83 mask=0x18 alloc=0 bind=0
```

All three create the image and compute the right memory-type mask, but
`vkAllocateMemory` with the AHB import chain fails on this Adreno 618. So even
forcing the "correct" Vulkan format cannot make the camera RAW16 AHB a Vulkan
image here. The one CPU de-stride copy stays.

The remaining zero-copy suggestion — extract the AHB's dma_buf fd with
`AHardwareBuffer_getNativeHandle()` and import it with
`VK_EXT_external_memory_dma_buf` — is also unavailable. The device reports:

```text
external memory extensions: khr_external_memory_fd=1 ext_external_memory_dma_buf=0
```

`VK_KHR_external_memory_fd` alone only imports driver-owned opaque fds, not
foreign dma_buf fds, so the fd path cannot be used either.

The last suggested route — an external-format sampled image with an identity
`VkSamplerYcbcrConversion` — needs a non-zero `externalFormat` ID. The same probe
reports `ahb_format=0 ahb_external=0`, and
`ext_external_memory_host=0`. There is no external format to chain, no host
pointer import, and no dma_buf import on the vendor ICD. The CPU de-stride copy
into the BLOB AHB is the only viable input path with the vendor driver.

## Mesa Turnip comparison

Termux now carries `mesa-vulkan-icd-freedreno` and a generic `libvulkan.so`, so
the same probes were repeated against Mesa Turnip 26.3.1
(`/data/data/com.termux/files/usr/share/vulkan/icd.d/freedreno_icd.aarch64.json`).

Mesa exposes `VK_EXT_external_memory_dma_buf` (the vendor ICD does not), but it
does **not** expose `VK_ANDROID_external_memory_android_hardware_buffer`. That
means the existing AHB-backed `DirectVulkanContext` cannot be created under
Mesa: a dedicated standalone probe was added so the dma_buf route can be tested
without the AHB extension.

`raw16_ahb_dmabuf_import` in `test_direct_vulkan_image.rs` extracts the AHB's
native fd, imports it as a `VkBuffer` through `VK_KHR_external_memory_fd` +
`VK_EXT_external_memory_dma_buf`, and binds memory:

```text
RAW16 dma_buf AHB import: ok=true device=Turnip Adreno (TM) 618 vendor=5143
fdProps=0 mask=0x7 createBuffer=0 reqMask=0x7 mask=0x7
chosen=0 alloc=1 bind=0
```

So the zero-copy **input** path is viable on Mesa, while the vendor ICD forces
the one CPU de-stride copy. Production selection remains on the vendor AHB path;
the Mesa probe is a capability/run comparison only. The AHB-backed storage-image
output test is skipped under Mesa because that ICD lacks the AHB extension
required to import the display AHB.

## Applied changes

1. **`dv_run_ahb_image()` / `DirectVulkanContext::run_ahb_image()`.** A second
   compute pipeline (`raw16_bgra_image.comp`) writes packed BGRA through a
   `writeonly image2D` bound to the output AHB's `VkImage`/`VkImageView`. No CPU
   mapping is opened on the output at all. Selected by
   `SOFTISP_DIRECT_VULKAN_IMAGE_OUTPUT` (default on; `0`/`false` restores the
   old `VkBuffer` path).
2. **`dv_probe_image_ahb()` / `DirectVulkanContext::probe_image_ahb()`** and
   **`dv_probe_raw16_image_ahb()` / `probe_raw16_image_ahb()`.** Kept as public
   diagnostics so perf/capability tests can query any AHB instead of one-shot
   log lines in the worker. The standalone
   **`probe_dmabuf_ahb()` / `dv_probe_dmabuf_ahb_global()`** performs the same
   capability check for ICDs that expose dma_buf but not AHB external memory.
3. **Presenter CPU window only when needed.** `app.rs` no longer calls
   `begin_cpu_read()` on every AHB frame. SurfaceFlinger consumes the AHB
   handle; a CPU mapping is opened only on frames that emit a stats line
   (`STATS_INTERVAL_FRAMES = 5`). `repaint_last()` now re-submits the AHB handle
   through `AhbDisplay` as well, instead of falling back to a CPU blit from a
   mapping that may not exist.
4. **Temporary `LATW`/`LATP`/`IMAGE-PROBE`/`[loop]`-threshold instrumentation
   removed** from the production hot path. The capability and perf methods stay
   in the public API and are exercised by an on-device test.

## Perf unit test

`cam-rust/cam-isp/tests/test_direct_vulkan_image.rs` allocates a BLOB input AHB
and an RGBA_8888 + `GPU_FRAMEBUFFER` output AHB, probes the storage-image import,
runs 32 dispatches, and checks the output image is non-black. It is `#[ignore]`
because it needs a device:

```bash
LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
  cargo test -p cam-isp --test test_direct_vulkan_image --features direct_vulkan \
  -- --ignored --nocapture --test-threads=1
```

Result on device (Adreno 618, 64x48): 32 dispatches in 10.6–13.3 ms → **0.33–0.42
ms per dispatch**. Output-image probe returned `ok=true`; the RAW16 explicit
format probe returned `ok=false` for all three `R16_*` candidates. Against Mesa
Turnip the AHB-backed tests print a skip line (no AHB extension) while
`raw16_ahb_dmabuf_import` reports a successful dma_buf import. To force Mesa:

```bash
VK_ICD_FILENAMES=/data/data/com.termux/files/usr/share/vulkan/icd.d/freedreno_icd.aarch64.json \
LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a:/data/data/com.termux/files/usr/lib \
  cargo test -p cam-isp --test test_direct_vulkan_image --features direct_vulkan \
  -- --ignored --nocapture --test-threads=1
```

## Remaining work

- Measure the full preview on device with the image path selected; compare
  `slot.wait_free()` against the earlier 120–150 ms.
- If the slot wait is still large, add a second output slot per worker so the
  GPU can start frame N+1 while SurfaceFlinger still owns N's AHB.
- The input still needs one CPU de-stride copy into the BLOB AHB; the RAW16
  camera AHB cannot be imported as Vulkan memory on this device (see
  `20261128-raw16-ahb-image-copy-probe.md`).
