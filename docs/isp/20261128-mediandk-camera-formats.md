# MediaNDK image formats and this device's camera outputs

## MediaNDK `AIMAGE_FORMATS` (NDK r29, `media/NdkImage.h`)

These are the only format values MediaNDK `AImageReader` accepts. `AImageReader_new()`
supports API 24 values; `AImageReader_newWithUsage()` is API 26;
`AImageReader_newWithDataSpace()` is API 34.

| Value | Name | Layout |
|-------|------|--------|
| `0x1` | `AIMAGE_FORMAT_RGBA_8888` | 4 B/px, 1 plane |
| `0x2` | `AIMAGE_FORMAT_RGBX_8888` | 4 B/px, 1 plane, X unused |
| `0x3` | `AIMAGE_FORMAT_RGB_888` | 3 B/px, 1 plane |
| `0x4` | `AIMAGE_FORMAT_RGB_565` | 2 B/px, 1 plane |
| `0x16` | `AIMAGE_FORMAT_RGBA_FP16` | 8 B/px, 1 plane, half floats |
| `0x20` | `AIMAGE_FORMAT_RAW16` | 16-bit Bayer, 2 B/px |
| `0x22` | `AIMAGE_FORMAT_PRIVATE` | implementation-defined; no `AImage` plane access |
| `0x23` | `AIMAGE_FORMAT_YUV_420_888` | 3 planes, 8-bit 4:2:0 (Y, U, V) |
| `0x24` | `AIMAGE_FORMAT_RAW_PRIVATE` | sensor raw, opaque |
| `0x25` | `AIMAGE_FORMAT_RAW10` | 10-bit Bayer, packed |
| `0x26` | `AIMAGE_FORMAT_RAW12` | 12-bit Bayer, packed |
| `0x2C` | `AIMAGE_FORMAT_RAW14` | 14-bit Bayer, packed |
| `0x100` | `AIMAGE_FORMAT_JPEG` | compressed, single BLOB plane |
| `0x101` | `AIMAGE_FORMAT_DEPTH_POINT_CLOUD` | xyz + confidence floats |
| `0x20203859` | `AIMAGE_FORMAT_Y8` | 8-bit grayscale, fourcc `Y8  ` |
| `0x44363159` | `AIMAGE_FORMAT_DEPTH16` | 16-bit depth + confidence, fourcc `Y16D` |
| `0x48454946` | `AIMAGE_FORMAT_HEIC` | HEIF, fourcc `HEIF` |
| `0x69656963` | `AIMAGE_FORMAT_DEPTH_JPEG` | JPEG + XMP depth, fourcc `ciei` |

## `AImageReader_newWithUsage` compatibility (NDK header)

The header states exactly two allowed usage combinations:

- All non-`PRIVATE` formats: `CPU_READ_RARELY` or `CPU_READ_OFTEN`.
- `RGBA_8888` only: `VIDEO_ENCODE` and/or `GPU_SAMPLED_IMAGE`.

`GPU_DATA_BUFFER` is **not** listed for any MediaNDK format. This is the header-level
reason the direct Vulkan engine cannot import a camera AHB as a `VkBuffer`: no
camera reader can be opened with that usage, and gralloc rejects it for private
RAW formats anyway.

## What this device's camera actually advertises

One-shot diagnostic (removed after capture) read
`ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS` from the active camera's static
metadata. Every entry is an output stream; there are no input streams.

| Format | Name | Sizes |
|--------|------|-------|
| `0x22` | `PRIVATE` | 33 sizes, 176x144 … 4640x3472 |
| `0x23` | `YUV_420_888` | 33 sizes, 176x144 … 4640x3472 |
| `0x100` | `JPEG` | 32 sizes, 320x240 … 4640x3472 |
| `0x25` | `RAW10` | 4640x3472 only |
| `0x20` | `RAW16` | 4640x3472 only |
| `0x24` | `RAW_PRIVATE` | 4640x3472 only |

Not advertised by the camera: `RGBA_8888`, `RGBX_8888`, `RGB_888`, `RGB_565`,
`RGBA_FP16`, `RAW12`, `RAW14`, `DEPTH16`, `DEPTH_POINT_CLOUD`, `Y8`, `HEIC`,
`DEPTH_JPEG`. Those are `AImageReader`-side formats, not camera output formats
on this device.

The three raw formats (`RAW10`, `RAW16`, `RAW_PRIVATE`) advertise only the full
4640x3472 sensor array; the pipeline still opens a 1600x1200 RAW16 reader and the
HAL accepts it.

## Vulkan importability cross-reference

| Camera format | Vulkan route | Result |
|---------------|--------------|--------|
| `JPEG` / BLOB | AHB → `VkBuffer` with `GPU_DATA_BUFFER` | works (worker-owned output slot uses this) |
| `RAW16` | AHB → `VkBuffer` (`GPU_SAMPLED_IMAGE`) | `vkAllocateMemory` fails, `VkResult=5` |
| `RAW16` | AHB → `VkImage` | no format (`format=0 externalFormat=0`) |
| `RAW16` | `GPU_DATA_BUFFER` | gralloc 4.0 rejects `rc=1` |
| `RAW16` | CPU-only reader usage | still fails `VkBuffer` import |
| `RAW16` | DMA-BUF fd | `VK_EXT_external_memory_dma_buf` absent |
| `PRIVATE` | `AImage` planes | no CPU plane access; no GPU usage allowed |
| `YUV_420_888` | per-plane AHB | reader usage limited to `CPU_READ_*`; no GPU flags |
| `RAW10`, `RAW_PRIVATE` | AHB | opaque, same gralloc/Vulkan gates as RAW16 |

See `20261128-raw16-ahb-image-copy-probe.md` for the raw probe evidence.

## Consequence

There is no camera output format on this device that can be opened by MediaNDK
with Vulkan-consumable usage. The only Vulkan-importable AHBs are worker-owned
BLOB buffers with `GPU_DATA_BUFFER`, so the one CPU copy from the camera plane
into that BLOB is unavoidable on this hardware. `RGBA_8888` is the sole format
the NDK permits with `GPU_SAMPLED_IMAGE`, but the camera does not offer it.
