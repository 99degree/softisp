# Direct Vulkan latency analysis

## Symptom

Live production logs after the direct Vulkan engine is enabled show roughly
580–700 ms per frame at 1600x1200:

```text
[loop] frame took 608.6 ms (media_ndk_capture 272.2 / input_stage 266.1 /
       mnn_prepare 0.0 / mnn_inference 2.2 / mnn_output 0.0 / present 68.1 ms)
```

The GPU pass itself is ~2 ms. Almost all wall time is in `media_ndk_capture`
(~260 ms) and `input_stage` (~250 ms), with `present` (~60–70 ms) third. The
user wants lower latency and a best-effort real-time pipeline.

## Reproduction

1. `DIRECT_VULKAN=1 ./build-test-libs.sh --core-only`
2. `./push-test-libs.sh`
3. `./shim-debug.sh reload` (Termux foregrounded; camera app foregrounded for frames)
4. Read `/sdcard/download/softisp-app.log`, grep `[loop]`.

## Instrumented breakdown

Temporary `LAT2`/`LATP` logs (removed after measurement) decomposed the opaque
`media_ndk_capture`/`input_stage` numbers. Per-frame averages:

| stage        | BLOB output | RGBA8888 output |
|--------------|-------------|-----------------|
| `next`       | 13.0 ms     | 13.0 ms         |
| `copy`       | 1.0 ms      | 1.0 ms          |
| `proc`       | 4.0 ms      | 4.0 ms          |
| `wait`       | 150–170 ms  | 13.6 ms         |
| `present`    | 60–80 ms    | 27.9 ms         |
| `read`       | 1.3 ms      | 1.3 ms          |

Key findings:

- The `media_ndk_capture`/`input_stage` inflated numbers in `[loop]` were mostly
  the output-slot `wait_free()` blocked on the presenter. `pipeline_ms` in that
  log is a sum of overlapping worker timings, not true end-to-end wall time.
- `next` (camera acquire) is ~13 ms; the CPU de-stride `copy` is ~1 ms; the GPU
  `proc` is ~4 ms. None of these are the bottleneck.
- The dominant costs are output backpressure (`wait`) and SurfaceFlinger
  presentation (`present`).

## Fix: graphics-format output AHB

The direct Vulkan output slot was a `BLOB` AHB. BLOB buffers cannot be scanned
out as graphics buffers, so SurfaceFlinger copies them on the CPU. Switching
the direct Vulkan path to an `R8G8B8A8_UNORM` AHB with
`GPU_FRAMEBUFFER` usage lets the compositor take a zero-copy hardware path.
The Vulkan driver still imports the graphics AHB as a linear `VkBuffer` on this
device, so the compute shader writes straight into it unchanged.

Scoped to direct Vulkan only (`new_graphics`), gated by the `direct_vulkan`
feature; the default `mnn_vulkan` path keeps its oversized BLOB slot.

Result: ~18 fps (240 frames in 13.4 s) vs ~10.6 fps before. `wait` dropped
from 150–170 ms to ~14 ms and `present` from 60–80 ms to ~28 ms.

The first graphics-output attempt failed with `AHardwareBuffer_allocate rc=-22`
because `AHardwareBuffer_Desc.layers` was left at zero. Setting `layers = 1`
fixed it.

## Input side remains CPU-staged

RAW16 camera AHBs have no Vulkan image format (`format=0 externalFormat=0`) and
cannot carry `GPU_DATA_BUFFER` (gralloc 4.0 rejects the combination), so they
cannot be imported as `VkImage` or `VkBuffer`. Details in
`20261128-raw16-ahb-image-copy-probe.md`. The ~1 ms CPU de-stride copy into a
worker-owned BLOB AHB remains; it is not a bottleneck.

## Status

Resolved for the dominant costs. Graphics-format output AHB landed on the
direct Vulkan path and was verified on device with clean non-black frames.
