# RAW16 camera AHB fd zero-copy path

## Symptom

With the built-in Adreno 618 hwvulkan HAL, RAW16 camera AHBs cannot be imported
as `VkBuffer`/`VkImage` external memory. Explicit `R16_*` AHB image imports fail
at `vkAllocateMemory` with `VK_ERROR_TOO_MANY_OBJECTS`, the HAL exposes
`VK_KHR_external_memory_fd` but not `VK_EXT_external_memory_dma_buf`, and the
AHB reports no external format. The direct-Vulkan path therefore kept one CPU
de-stride copy into a worker-owned BLOB AHB.

## Reproduction

1. Allocate a RAW16 `AHardwareBuffer` with `AHARDWAREBUFFER_FORMAT_RAW16` and
   `AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE`.
2. Run `raw16_ahb_explicit_vulkan_format_import`,
   `raw16_ahb_opaque_fd_import`, and `raw16_ahb_dmabuf_import` from
   `cam-rust/cam-isp/tests/test_direct_vulkan_image.rs` against the built-in HAL.
3. Observe explicit-format import failure, opaque-fd import failure, and
   missing dma_buf extension.

## Root cause

The camera AHB's native handle exposes a dma_buf fd. Vulkan drivers that expose
only `VK_KHR_external_memory_fd` treat fds as driver-opaque export tokens and
reject the camera's dma_buf fd; the built-in Adreno HAL does not advertise
`VK_EXT_external_memory_dma_buf` at all. A dedicated probe confirmed the
opaque-fd experiment fails on this device:

```text
RAW16 opaque_fd AHB import: ok=false opaque_fd device=Adreno (TM) 618 vendor=5143;
fdProps=0 mask=0x18; createBuffer=0; reqMask=0x18 mask=0x18; chosen=4294967295 alloc=0 bind=0
```

Mesa Turnip does advertise the dma_buf extension, so the same AHB fd imports
successfully there. The extension set, not the AHB layout, is the blocker on
the vendor driver.

## Fix

The direct Vulkan context now enables `VK_KHR_external_memory_fd` and
`VK_EXT_external_memory_dma_buf` when the selected device exposes them, queries
`vkGetMemoryFdPropertiesKHR` and
`vkGetBufferMemoryRequirements2KHR`, and caches one imported `VkBuffer` per
camera AHB fd. The new `raw16_bgra_strided.comp` shader reads each `u16` sample
through the AHB's byte stride and writes packed BGRA to the existing AHB-backed
storage `VkImage`, so neither side is opened by the CPU during execution.

`AHardwareBuffer_describe()` is API 26+, while this C++ module still compiles
against older API headers, so the plane layout is read from the gralloc native
handle. The layout words were confirmed with a 80x60 RAW16 probe before the
strided run was wired up.

Runtime selection is conservative:

- `SOFTISP_DIRECT_VULKAN_FD_INPUT=0|false` keeps the BLOB de-stride fallback.
- `SOFTISP_RAW_AHB_SKIP_CPU` (or the matching marker file) is still required for
  the camera layer to hand over the media AHB without a CPU read.
- Drivers without `VK_EXT_external_memory_dma_buf` keep the existing
  BLOB-input/storage-image path; the default engine remains `mnn_vulkan`.

The `AHardwareBuffer_lock`/`unlock` calls remain confined to pre-run staging or
post-fence presentation windows. No host upload is performed on the fd path.

## Verification

`raw16_ahb_fd_image_run` is an ignored on-device test. It fills a RAW16 AHB,
imports its fd, runs 32 strided dispatches into an RGBA_8888 AHB-backed storage
image, waits on the fence, and checks for non-black pixels.

With the project loader and Turnip 710/720/722 v4.7 unified:

```text
HAL ready: .../libvulkan_freedreno.so id=vulkan
fd image output: 32 dispatches in 18.272969ms (0.571 ms/run)
```

With Turnip 26.3.1 EMULATOR the same test measured `0.895 ms/run`. With the
built-in Adreno HAL it correctly skips because the dma_buf extension is absent.
`cargo test -p cam-isp --test test_direct_vulkan_image --features direct_vulkan --no-run`
is clean, and the focused `cam-isp` lib suite remains 863 passed / 0 failed /
2 ignored with `rectifier,direct_vulkan`.

A hot reload of the pushed core verified all nine uploaded libraries with
`shim-debug.sh verify`; the app started `direct_vulkan_a` with
`fd_input=true`. The loader SONAME is now `libvulkan.so`, and the rebuilt shim
APK preloads that library before the core. Install
`target/debug/apk/softisp-test.apk` once to route the core's `DT_NEEDED` to the
project loader rather than Android's system loader.

## Outcome

`OPAQUE_FD` is rejected and is retained only as a capability probe. The viable
zero-copy input route is `VK_EXT_external_memory_dma_buf` on Turnip-class
drivers, with the BLOB de-stride path retained for the built-in Adreno driver.
