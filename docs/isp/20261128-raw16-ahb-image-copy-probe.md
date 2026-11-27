# RAW16 AHB → VkImage → VkBuffer copy probe

## Symptom / motivation

The direct Vulkan engine imports the worker-owned BLOB input AHB as a
`VkBuffer`. Live MediaNDK RAW16 AHBs cannot be imported as storage buffers on
this device, so one CPU de-stride copy into the BLOB slot remains before the
GPU. The user proposed using `vkCmdCopyImageToBuffer` on the MediaNDK AHB: bind
the RAW16 AHB as a `VkImage` and copy its bytes into the compute shader's input
`VkBuffer` entirely on the GPU.

## Result

The proposal was implemented as a temporary probe with a synthetic RAW16 AHB
and then with the live MediaNDK AHB. It does **not** work on this device. The
failure happens before `vkCmdCopyImageToBuffer` can be recorded.

`vkGetAndroidHardwareBufferPropertiesANDROID` reports, for both the synthetic
RAW16 AHB and the live camera AHB:

```text
vkFormat=0 (VK_FORMAT_UNDEFINED)
externalFormat=0
formatFeatures=0x20000 (MIDPOINT_CHROMA_SAMPLES only)
allocationSize=3842048 (live) / 4096 (fixture)
memoryTypeBits=0x18
```

There is no Vulkan format and no external format that maps the RAW16 AHB into
an image. Forcing `VK_FORMAT_R16_UNORM`, `VK_FORMAT_R16_UINT`, and
`VK_FORMAT_R16_SNORM` in `VkImageCreateInfo` does not help: `vkCreateImage`
succeeds, but memory import fails for every candidate memory type with
`VkResult=5`.

Live production log:

```text
LIVE-IMAGE-PROBE FAIL: all image formats failed:
  format=0:  AHB image import failed: vkFormat=0 external=0 features=0x20000 ...
  format=70: AHB image import failed: vkFormat=0 external=0 features=0x20000 ...
  format=74: AHB image import failed: vkFormat=0 external=0 features=0x20000 ...
  format=71: AHB image import failed: vkFormat=0 external=0 features=0x20000 ...
```

`vkCmdCopyImageToBuffer` requires an image with `VK_IMAGE_USAGE_TRANSFER_SRC_BIT`
and a defined or external image format. Neither exists for this AHB, so no
copy command is legal. Adding `AHARDWAREBUFFER_USAGE_GPU_FRAMEBUFFER` to the
synthetic allocation does not change the reported format or the import result.

## Follow-up: `vkCmdCopyBuffer` from the AHB as a `VkBuffer`

The same device was probed with `vkCmdCopyBuffer` as the user suggested: import
the AHB as external-memory `VkBuffer`, then GPU-copy it into a plain
host-visible destination buffer. Result:

```text
CASE RAW16+GPU_SAMPLED_IMAGE (camera-like):
  Err("AHB import failed: req=512 ahb=4096 types=0x18")
CASE BLOB+GPU_DATA_BUFFER (worker slot):
  Ok("vkCmdCopyBuffer ok: nonzero=512/512")
```

The camera-like RAW16 AHB cannot be imported as a `VkBuffer` either, so there
is no source buffer for `vkCmdCopyBuffer`. The BLOB worker slot can, but the
CPU already writes directly into that buffer — inserting a GPU copy would add
work, not remove it.

## Corrective finding: VkBuffer import requires `GPU_DATA_BUFFER`

`vkCmdCopyBuffer` itself is format-agnostic, but the source has to be a
`VkBuffer` bound to imported AHB memory. The
`VK_ANDROID_external_memory_android_hardware_buffer` spec only permits binding
an AHB to a `VkBuffer` when the buffer was created with
`AHARDWAREBUFFER_USAGE_GPU_DATA_BUFFER`. Camera RAW16 AHBs carry
`GPU_SAMPLED_IMAGE`; gralloc rejects `GPU_DATA_BUFFER` for private RAW16 on this
device. Therefore:

- `RAW16 + GPU_SAMPLED_IMAGE` → no `VkBuffer` import (confirmed: `req=512
  ahb=4096 types=0x18`, all allocations fail).
- The `raw16_sampled_image_ahb_binds_as_vulkan_buffer_input` MNN test only checks
  that the tensor's `MNN_MEMORY_AHARDWAREBUFFER` flag survives `resize()`; it
  does not prove `vkAllocateMemory` succeeded, so it is not evidence against
  this rule.

## Live experiment: Camera HAL rejects `GPU_DATA_BUFFER` on RAW16

Added `AHARDWAREBUFFER_USAGE_GPU_DATA_BUFFER` to the `AImageReader_newWithUsage`
flags for the RAW16 stream and rebuilt. `AImageReader_newWithUsage` itself
succeeded (no fallback warning), but the frame flow died: no `CAM-AHB` frame
logs, no loop frames, and the app exited after ~34 s.

`logcat` shows the exact cause:

```text
E Gralloc4: non-BLOB pixel format with GPU_DATA_BUFFER usage is not supported prior to gralloc 4.1
E BufferQueueProducer: dequeueBuffer: createGraphicBuffer failed
E Camera3-OutputStream: getBufferLockedCommon: Stream 0:
    Can't dequeue next output buffer: Invalid argument (-22)
D ACameraDevice: Device error received, code 3 ...
E softisp::camera: ACameraDevice: error 4 (ERROR_CAMERA_DEVICE)
```

This device runs gralloc 4.0. A non-BLOB format (RAW16 = 0x20) combined with
`GPU_DATA_BUFFER` is explicitly unsupported, so the camera stream cannot
produce an AHB that `vkBindBufferMemory` would accept as a `VkBuffer`. The
zero-copy RAW16 → `vkCmdCopyBuffer` path is impossible on this hardware.

## Three-layer isolation (synthetic fixture, current device)

Probed `AHardwareBuffer_allocate` with correct RAW16 geometry (1600×1200) and
queried `vkGetAndroidHardwareBufferPropertiesANDROID` for each successful
allocation:

```text
BLOB+DATA (worker output)          allocate OK usage=0x1000033 ahbMemoryTypes=0x18
RAW16+CPU only                     allocate OK usage=0x33       ahbMemoryTypes=0x18
RAW16+CAMERA_OUTPUT                allocate OK usage=0x80000000000003 ahbMemoryTypes=0x18
RAW16+GPU_SAMPLED (camera-like)    allocate OK usage=0x103      ahbMemoryTypes=0x18
RAW16+GPU_DATA                     AHardwareBuffer_allocate rc=1
RAW16+GPU_DATA+SAMPLED             AHardwareBuffer_allocate rc=1
RAW16+GPU_DATA+CAMERA_OUTPUT       AHardwareBuffer_allocate rc=1
```

Isolation:

1. **Gralloc is the primary gate.** `RAW16 + GPU_DATA_BUFFER` fails at
   `AHardwareBuffer_allocate` itself (`rc=1`), before Vulkan or the Camera HAL
   is involved. There is no AHB to import, so `memoryTypeBits` is never even
   reported. This matches the live `Gralloc4: non-BLOB pixel format with
   GPU_DATA_BUFFER usage is not supported prior to gralloc 4.1` log.
2. **Vulkan is a secondary gate for `GPU_SAMPLED_IMAGE`-only RAW16.** The
   driver reports `ahbMemoryTypes=0x18` and
   `bufferMemoryTypes=0x18` (non-zero intersection), but the actual
   `vkAllocateMemory` with the external-memory import chain still fails with
   `VkResult=5` (`VK_ERROR_OUT_OF_DEVICE_MEMORY`). The driver checks the AHB's
   usage flags at allocation time, not just at property-query time.
3. **Camera HAL is also a gate** (from the live experiment), but it is moot
   because Gralloc already prevents creating the buffer.

Conclusion: `AHardwareBuffer_lock()` + `memcpy()` into a regular host-visible
`VkBuffer` is the only spec-compliant path on this device.

## `CPU_READ_OFTEN`-only ImageReader (user suggestion)

Changed `AImageReader_newWithUsage` to request only
`AHARDWAREBUFFER_USAGE_CPU_READ_OFTEN` (no `GPU_SAMPLED_IMAGE`). Live run was
clean: ~19 fps, non-black frames, no errors. The camera AHB descriptor became:

```text
format=0x20 usage=0x20003 1600x1200 stride=1600
```

But a synthetic `RAW16 + CPU_READ_OFTEN` AHB still fails `VkBuffer` import:
all memory-type allocations fail. The usage flag on the reader does not change
the fact that private RAW16 is not Vulkan-buffer-importable on this device.

## DMA-BUF fd path (user suggestion)

`VK_EXT_external_memory_dma_buf` is **not exposed** by the Vulkan device on
this hardware (Adreno 618). The probe reports:

```text
RAW16+CPU_R+GPU_SAMPLED: Err("VK_EXT_external_memory_dma_buf not exposed by device")
BLOB+CPU_RW+GPU_DATA:    Err("VK_EXT_external_memory_dma_buf not exposed by device")
```

Only `VK_ANDROID_external_memory_android_hardware_buffer` is present, which
requires `GPU_DATA_BUFFER` for `VkBuffer` import — unavailable for camera RAW16.

## Why the premise does not hold

"MediaNDK returns RAW16, therefore it is a VkImage" is not true for Vulkan.
RAW16 is an Android gralloc format (`AHARDWAREBUFFER_FORMAT_RAW16 = 0x20`) with
no Vulkan equivalent on this driver. `GPU_SAMPLED_IMAGE` allows the buffer to
be used as a sampled image in OpenGL ES / gralloc terms, but Vulkan reports
`VK_FORMAT_UNDEFINED` with `externalFormat=0`, so there is no `VkImage` to copy
from. MNN's buffer import works only because BLOB AHBs are byte containers;
the RAW16 AHB is not byte-addressable as Vulkan device memory either.

## Consequence

The direct Vulkan engine keeps its current design:

- Worker-owned BLOB AHB input, imported once as a `VkBuffer`.
- One CPU de-stride copy from the MediaNDK RAW16 plane into that BLOB slot.
- GPU compute shader reads the packed `uint` words.
- Zero-copy AHB output, written directly by Vulkan.

True camera-AHB input zero-copy would require the camera to deliver a buffer
format Vulkan can import as a storage buffer (for example BLOB with
`GPU_DATA_BUFFER`), which Camera2 RAW16 capture does not provide on this device.
A future path would need vendor support for RAW16 as a Vulkan-external buffer or
a pre-bound capture target that is itself Vulkan-importable.

## Verification

The temporary probe code and live log instrumentation were removed after this
finding. The working direct Vulkan path was rebuilt and reverified with clean
non-black production frames.
