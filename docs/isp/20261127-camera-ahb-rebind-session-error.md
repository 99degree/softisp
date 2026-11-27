# Camera AHB rebind leaves MNN session in COMPUTE_SIZE_ERROR

## Symptom

On `dev/input-ahb-stable-slot` at `01f4a902`, the first MediaNDK frame after
startup completes; every subsequent frame fails:

```
ERROR [cam_isp::mnnengine] MNN inference failed: n=-3
  (input=1600x1200, output_name='DisplayBlock/frame', path=raw_zero_copy)
ERROR [cam_isp::mnnengine]   MNN reported: runSession failed: MNN error code=3
```

MNN `ErrorCode 3` is `COMPUTE_SIZE_ERROR`. `Session::runWithCallBack()`
returns it only when `mNeedResize` is still true, i.e. a resize started but
did not complete.

## Reproduction

```bash
./build-test-libs.sh --core-only
./push-test-libs.sh
am start --activity-clear-task --activity-single-top \
  -n com.softisp.camera/com.softisp.camera.ShimActivity
./shim-debug.sh reload
./shim-debug.sh logs
```

The first completed frame logs `rgb mean [0.0,0.0,0.0]` because the output CPU
read window is stale; the next frame onward fails inference before presentation.

## Root cause chain

1. `prepare()` binds the worker-owned pooled input AHB and records its pointer
   in `PreparedMeta::input_ahb_ptr`.
2. The first real frame carries a MediaNDK AHB with a different pointer.
   `run_prepared()` calls `mnn_import_ahb_as_device_memory()` on the frame input
   tensor, then `MnnSessionSafe::resize_for_rebind()`.
3. `mnn_session_resize_for_rebind()` calls
   `Interpreter::tryResizeSession(sess, needRelloc=1)`. That sets
   `Session::mNeedMalloc = true` and calls `Session::resize()`.
4. `mnn_session_resize_for_rebind()` ignores the `ErrorCode` returned by
   `tryResizeSession()`. If `allocMemory()` fails, `Session::resize()` leaves
   `mNeedResize = true`, so the session is unrunnable. The wrapper still returns
   0 to Rust, so no warning is emitted and the next `runSession()` fails with
   error 3.

After propagating the code, the device run reports
`Session rebind resize failed: rc=-1`. MNN error 1 is `OUT_OF_MEMORY`; the
session-wide realloc cannot import a newly swapped camera AHB.

### MNN AHB tensor API check

The suggested "cast the AHB into a new tensor" API does not exist in public
MNN:

- `Tensor::create(shape, type, void* data, dimType)` wraps a **host** pointer.
  A Vulkan session still copies that host buffer into its own device memory.
- `Tensor::createDevice(shape, type, dimType)` creates a device tensor with no
  backing memory; it does not accept an AHardwareBuffer.
- `Tensor::setDevicePtr(ptr, MNN_MEMORY_AHARDWAREBUFFER)` is the only AHB hook.
  It sets `buffer().flags` and `buffer().device` on an existing tensor, which is
  why the current wrapper applies it to `getSessionInput()`.
- There is no public `setSessionInput`/`replaceInput` API: `Interpreter::runSession()`
  calls `Session::run()`, which uses the session's own tensors returned by
  `getInput()`. A separately created tensor cannot be substituted into
  `runSession()`.

The Vulkan fork imports the AHB in `VulkanBackend::onAcquire()` and caches the
resulting `VkBuffer` in `TensorUtils::getSharedMem(tensor)`. When the AHB pointer
changes, the tensor must be re-acquired. A full `tryResizeSession(..., 1)` is
the wrong granularity: it reallocates the entire pipeline and fails with OOM.
`Interpreter::getBackend(session, tensor)` plus
`Backend::onReleaseBuffer()` / `Backend::onAcquireBuffer()` can re-acquire just
that tensor without a session resize. Session inputs use
`Backend::DYNAMIC_SEPERATE` storage.

## Relevant source

- `anw/src/pipeline_runner.rs`: prepares with pooled AHB, rebinds camera AHB
  when a frame arrives.
- `cam-rust/cam-isp/src/mnnengine.rs`: `PreparedMeta::input_ahb_ptr` and
  `run_prepared()` rebind logic.
- `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp`: `mnn_rebind_ahb_device_memory()`
  changes the session-owned tensor's device pointer and re-acquires only that
  tensor.
- `MNN/source/core/Interpreter.cpp`: `runSession()` calls `Session::run()`;
  `tryResizeSession(sess, 1)` sets `mNeedMalloc` then calls `Session::resize()`.
- `MNN/source/core/Session.cpp`: `runWithCallBack()` returns
  `COMPUTE_SIZE_ERROR` when `mNeedResize` is true; `resize()` clears
  `mNeedResize` only after `allocMemory()` succeeds.

## Resolution

`mnn_session_resize_for_rebind()` was removed. The new glue API
`mnn_rebind_ahb_device_memory()` applies `setDevicePtr()` to the existing
session input tensor, calls `onReleaseBuffer()` to drop the cached Vulkan import,
and calls `onAcquireBuffer()` with `Backend::DYNAMIC_SEPERATE` to import exactly
that AHB. No pipeline tensor is resized, so `mNeedResize` is never set and the
session remains runnable.

If the re-acquire fails, the glue clears the AHB flag and re-acquires
backend-owned memory before returning the error. Rust therefore falls back to a
host upload into a tensor MNN will actually read; it cannot skip the upload on
stale AHB flags. The capacity check uses four bytes per expected input element:
`VulkanBackend::getTensorSize()` allocates four bytes for integer tensors and
for high-precision float tensors.

Do not restore the black-frame regression by skipping host staging based on
`frame.ahb.is_some()`.

## Follow-up failure: camera RAW16 AHB cannot back a Vulkan buffer tensor

After the session-resize bug was removed, device logs showed
`onAcquireBuffer failed for 'raw_frame' (rc=-6)`. Source inspection explains it:

- The MNN model input is a 32-bit integer tensor. This fork's
  `VulkanBackend::getTensorSize()` allocates four bytes per element (rounded to
  four lanes), while a MediaNDK RAW16 plane is two bytes per pixel.
- `anw` was computing `AhbBinding::len` from the de-strided host buffer as
  `frame.data.len() * 2`, which reports four bytes per pixel even when the real
  `AHardwareBuffer` is only two bytes per pixel. The Rust capacity gate therefore
  passed a buffer that Vulkan could not import.
- Camera image AHBs are normally allocated with `GPU_SAMPLED_IMAGE`, not
  `GPU_DATA_BUFFER`; MNN's `VulkanBufferFromAHB::create()` imports an AHB as a
  `VkBuffer`, so a sampled-image-only buffer is not a valid source even when its
  size matches.

The glue now exposes `mnn_ahb_capacity_bytes()` so `run_prepared()` validates the
real descriptor before rebinding. `cam-rust/cam-isp/tests/test_ahb_rebind_glue.rs`
reproduces both cases without opening a camera: a dummy RAW16 sampled-image AHB
must be rejected, while a BLOB `GPU_DATA_BUFFER` AHB must import and clear
cleanly. This is an Android device test because it needs real
`AHardwareBuffer` allocation and MNN's Vulkan backend:

```bash
LD_LIBRARY_PATH=$PWD/cam-rust/lib/aarch64-v8a \
  cargo test -p cam-isp --test test_ahb_rebind_glue --features mnn \
  -- --ignored --nocapture --test-threads=1
```

## MNN side: external-memory extensions were probed but never enabled

The dummy BLOB `GPU_DATA_BUFFER` AHB still failed with
`vkGetAndroidHardwareBufferPropertiesANDROID unavailable`. Code trace:

- `third_party/MNN/source/backend/vulkan/component/VulkanBuffer.cpp` resolves
  that entry point with `vkGetDeviceProcAddr()`.
- `third_party/MNN/source/backend/vulkan/component/VulkanDevice.cpp` enumerated
  `VK_KHR_external_memory`,
  `VK_ANDROID_external_memory_android_hardware_buffer`,
  `VK_KHR_dedicated_allocation`, `VK_KHR_get_memory_requirements2`, and
  `VK_KHR_bind_memory2` into `availableDeviceExtensions`, but never pushed them
  into `VkDeviceCreateInfo::ppEnabledExtensionNames`.
- `third_party/MNN/source/backend/vulkan/component/VulkanInstance.cpp` also did
  not enable `VK_KHR_external_memory_capabilities`.

When `third_party/MNN` is present, softisp uses that local checkout; otherwise
it falls back to `MNN_DIR` or `$HOME/MNN`. The two Vulkan setup files enable the
probed extension chain when present. Rebuild with `./build-test-libs.sh` (no
`--core-only`) after changing MNN sources; the fast core-only path keeps stale
`libMNN_Vulkan.so`.

## Device result after the extension fix

The dummy BLOB `GPU_DATA_BUFFER` test now passes, including rebind after
resize. The real MediaNDK frame still fails:

```
input AHB rebind failed: rc=-6; rebind: onAcquireBuffer failed for 'raw_frame'
(flags=14, device=...); tensor code=0 bits=32 elems=960000;
ahb format=32 usage=0x20003 1600x1200 layers=1 bytes=7680000;
restored backend-owned memory; using host upload
```

Capacity is not the cause: 7,680,000 bytes is already twice MNN's 3,840,000-byte
Vulkan requirement for 960,000 32-bit elements. The remaining cause is the AHB
contract itself:

- `AHARDWAREBUFFER_FORMAT_BLOB` (`0x21`) is the format that corresponds to
  Vulkan buffers (`hardware_buffer.h:111-117`).
- `AHARDWAREBUFFER_USAGE_GPU_DATA_BUFFER` (`1UL << 24`) **requires** BLOB format
  (`hardware_buffer.h:366-370`).
- The camera AHB is private RAW16 format `0x20` with usage `0x20003`, i.e. CPU
  read plus camera write; it has neither BLOB format nor `GPU_DATA_BUFFER`.

`VulkanBufferFromAHB::create()` imports AHBs as a `VkBuffer` using
`VkImportAndroidHardwareBufferInfoANDROID`; it cannot import an AHB with no GPU
usage. The conclusion that the **VkBuffer import** failed for that observed
buffer is proven; the stronger claim that the AHB bytes themselves cannot be
reinterpreted was too broad. The AHB is memory and the RAW16 bytes can be
recast in Vulkan. A later device probe refined this result: private RAW16 with
`GPU_SAMPLED_IMAGE` imports through the same `VkBuffer` path, so the missing GPU
usage bit — not the RAW16 format — was the blocker. Until the reader grants that
usage, rebind failure keeps the host upload path rather than presenting stale
or zero data.

## Follow-up failure: every frame is black after the extension fix

### Symptom

On the same branch with rebuilt MNN libs and `/sdcard/download/softisp-output-ahb-off`
present, the app reports 45 frames with:

```
rgb mean [0.0,0.0,0.0], max 0, non-black 0.0%
```

while each capture reports non-zero Bayer samples, e.g.
`worker 1 raw 600x800 min 15 max 1023 mean 92.2`. The only inference warnings
are the expected AHB rebind failures; `MNN inference failed` is absent and
`mnnengine`'s existing `black frame` probe never fires. With output AHB enabled
the same zero output is observed, so presentation is not the first isolation
point.

### Reproduction

```bash
touch /sdcard/download/softisp-output-ahb-off
am start -n com.termux/com.termux.app.TermuxActivity
./shim-debug.sh reload
sleep 18
./shim-debug.sh logs
grep -E '\[stats\]|input AHB rebind|black frame' .tmp/shim-debug/app.log
```

### Current state of the diagnosis

`frame_stats()` reads `Done::data` when no output AHB view exists, so the zero
mean means the host buffer copied from MNN is zero or empty, not that the
surface blit failed. The missing `black frame` warning implies the copied
`data_out` slice is empty, because that probe only runs for non-empty output.
The next investigation must distinguish an empty output slice from zero bytes
written into a non-empty slice, and verify that the host fallback uploads
`p.buf` to the session input after the failed AHB re-acquire.

### Final resolution

The black-frame regression from `1c6908f6` was reverted and merged to `master`
as `c5c59f96`. The remaining MNN input failure is environmental, not a tensor
contract bug: live MediaNDK RAW16 AHBs have neither `BLOB` format nor
`GPU_DATA_BUFFER` usage, so `VulkanBufferFromAHB::create()` cannot import them
as a `VkBuffer`. The targeted rebind works with worker-owned BLOB slots; when
the camera AHB fails the re-acquire, the glue restores backend memory and the
host upload path presents non-black pixels. `AHardwareBuffer_describe()` must be
declared with a `void` return in Rust FFI; the previous `int` signature produced
a false `rc=-2` capacity failure.

The clean production proof is now the standalone `direct_vulkan` engine
(`docs/isp/20261128-direct-vulkan-ahb-engine-design.md`): it removes MNN from
the data path entirely and runs a Vulkan compute shader on BLOB AHB-backed
storage buffers. It uses the same worker-owned BLOB input slot because the
camera RAW16 AHB cannot be used as a Vulkan storage buffer on this device.

The first production runs of that engine stalled because the warmup frame
consumed a presentation sequence without publishing a result; the presenter
waited at the missing sequence while the worker spun on its busy output slot.
Moving the warmup drop before sequence assignment fixed the stall. Replacing
the busy-wait with a condition variable removed the misleading tear warning;
the busy flag still gates GPU writes until the presenter releases the slot.

### Verbose trace

Enable `/sdcard/download/softisp-verbose`, keep `/sdcard/download/softisp-output-ahb-off`,
and reload. The `pipeline stage=infer_done` line reports MNN's returned element
count; `pipeline stage=write_input` reports the staged host byte count and input
mode. `HOST-DIAG in` samples the pointer actually handed to `mnn_run_*`, so a
non-zero Bayer capture with a zero input sample isolates the loss to the
MediaNDK AHB mapping or the host upload. `HOST-DIAG out` reports RGB-only
maximum because the BGRA alpha byte is always 255 and would otherwise mask an
all-black result. These are temporary measurements and must be removed after
the root cause is fixed.

### Controlled AHB fixture

`cam-rust/cam-isp/tests/test_ahb_rebind_glue.rs` allocates dummy AHBs but
had never written sample data into them. A zero-filled fixture cannot prove
that an imported AHB carries pixels. Fill the RAW16 plane with a repeated
10-bit pattern (`0..1023`), pack the same pattern into BLOB words, and add an
identity-session test that imports the BLOB AHB before resize and verifies the
pattern survives the GPU run. This separates an import failure from a data
path failure without requiring a camera.

The first device run with `HOST-DIAG in` shows the pointer handed to MNN is
non-zero Bayer data (first 32 bytes are plausible 10-bit samples), while
`HOST-DIAG out` reports `rgb_max=0`. The loss is therefore downstream of host
staging. Before the first camera AHB arrives, the input tensor is still bound
to the worker-owned pooled AHB from `prepare()`; no host upload occurs for
those frames. To isolate that binding from the later camera-AHB rebind path,
add a temporary `softisp-input-ahb-off` switch that leaves `input_ahb` unset
in both `prepare()` and the per-frame params. If the output becomes non-black
with the switch on, the pooled input binding is the regression; if it stays
black, the fault is in the MNN/Vulkan path shared by both inputs.

### Proven root cause: initial AHB "import" never imports device memory

The deterministic A/B test isolates the pooled-input failure without the app:

1. `host_upload_identity_preserves_raw16_pattern` passes: a host-uploaded
   pattern survives the Vulkan identity session.
2. `gpu_data_buffer_ahb_input_carries_raw16_pattern` fails with output word 0
   equal to zero. The CPU mapping of the dummy BLOB AHB contains the pattern,
   so the fixture and CPU copy are correct.
3. `gpu_data_buffer_ahb_binding_state_probe` passes valid host bytes while the
   input tensor is AHB-bound. The output is still zero. Therefore the AHB flag
   is retained and the wrapper's host upload is skipped.
4. `AHardwareBuffer_unlock()` returns no release fence for this BLOB AHB, so a
   missing CPU→GPU release fence is not the cause.
5. With `MNN_ISP_TRACE=1`, the initial-import path emits no
   `VulkanBufferFromAHB` trace lines at all. The targeted rebind path emits
   `[ZERO-COPY] ahbProps ... selected=3 flags=0x1` and
   `[ZERO-COPY] AHB imported ok size=1024 memType=3`.

The exact mechanism is:

- `cam-rust/cam-isp/mnn_sys/mnn_wrapper.cpp:1022` calls
  `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` and returns success,
  but it never calls `Backend::onAcquireBuffer()`. The log string at line 1026
  says "imported" even though only tensor metadata changed.
- `mnn_run_true_zero_copy()` at line 753 sees
  `flags == MNN_MEMORY_AHARDWAREBUFFER` and skips `copyFromHostTensor()`.
- No `VkBuffer` was imported for that input, so Vulkan reads whatever device
  memory was associated with the tensor, not the AHB contents.

This is the first regression point: every pooled-input frame is black even
before a MediaNDK AHB arrives. A later camera-AHB pointer change calls
`MnnSessionSafe::rebind_ahb()`, which does perform the real Vulkan import and
therefore does not have this particular defect, though the camera AHB format
still prevents that import from succeeding.

The clean fix is not to restore an unconditional host upload. It is to make
every AHB binding actually acquire backend device memory, and to keep the host
upload only for the genuine fallback path where AHB import fails and the flag
is cleared. The unit test must bind a filled BLOB AHB after the session is
resized through the targeted rebind API and assert that identity output matches
the CPU-mapped bytes.

### Targeted rebind imports the AHB but identity output is still zero

Changing the test to call `MnnSessionSafe::rebind_ahb()` after resize makes the
MNN Vulkan import actually run: the trace emits `ahbProps size=4096
typeBits=0x18 selected=3 flags=0x1` and `AHB imported ok size=1024 memType=3`.
The imported memory type is host-coherent but not host-visible, so the CPU
mapping probe in `VulkanBufferFromAHB::create()` is skipped. The identity output
is nevertheless still all zeros. A second `resizeSession()` after the rebind,
intended to rebuild `_InsertCopy()` without a host-copy wrapper for the now
Vulkan-resident input, also produces zeros.

This eliminates "no Vulkan import" as the only cause. Two possibilities remain
and must be separated by trace, not guessed:

1. The executed Vulkan command reads a different tensor/wrapper whose bytes are
   still zero, even though the session input itself was rebound.
2. The imported AHB `VkBuffer` is selected but its CPU-written bytes are not
   visible through that import.

Temporary `MNN_ISP_TRACE` logs in `VulkanBackend::getBuffer()` and
`VulkanBackend::onCopyBuffer()` must show the AHB buffer handle at execute and
readback time. They must be removed after the root cause is fixed.

### Proven final mechanism: MNN wraps the AHB input away

The trace for the rebind identity run shows:

```
[PIPELINE] execute: total=1 vk=1 cpu=0
[CMD] Raster backend=7 exec=0x...
[ZERO-COPY] gpu2host src flags=0 dev=0x... vk=0x... off=0 bytes=1024
```

There is no `[ZERO-COPY] getBuffer AHB ...` line during execute and no
`[ZERO-COPY] host2gpu ...` line. The tensor read back by
`copyToHostTensor()` is therefore a normal Vulkan tensor whose flags are zero,
not the AHB-bound session input. `mnn_run_true_zero_copy()` skips the only host
upload because the session input carries the AHB flag, while `_InsertCopy()`
had already replaced that input with a normal Vulkan wrapper because
`WrapExecution::needWrap()` only inspected backend type and not
`MNN_MEMORY_AHARDWAREBUFFER`.

MNN creates sessions with `Session_Input_Inside` by default, so
`Pipeline::_allocForTensor()` can acquire session inputs during resize. The
clean fix is to make `WrapExecution::needWrap()` return false for a Vulkan
AHB-flagged input. Then the original session input remains in
`iter.workInputs`, `_allocTensor()` calls `VulkanBackend::onAcquireBuffer()`
and imports the AHB, and the Vulkan op reads the imported buffer directly. This
removes the host-copy wrapper without adding a per-frame session resize.

Even after keeping the AHB input unwrapped, no AHB import appeared in the
trace. `Pipeline::_allocTensor()` skips `onAcquireBuffer()` whenever
`getDescribeOrigin(tensor)->mem` is already non-null. Session inputs are
created with host-side memory, and `Tensor::setDevicePtr()` only changes
`buffer().flags` and `buffer().device`; it does not release that memory object.
`Backend::onReleaseBuffer()` is the supported operation that clears it. The
wrapper's initial AHB bind therefore needs to release the existing memory
object and cached shared memory before returning, so the upcoming resize calls
`VulkanBackend::onAcquireBuffer()` and imports the AHB exactly once.

### Actual blocker: VulkanRaster fast path bypasses AHB device buffers

The latest trace is decisive:

```
[RUN] in=0x... flags=14 host=0x0 device=0x... hostbuf=0x...
[ZERO-COPY] AHB imported ok size=1024 memType=3
[PIPELINE] execute: total=1 vk=1 cpu=0
[ZERO-COPY] getBuffer t=0x... flags=0 dev=0x...   (output/readback only)
```

The session input is correctly marked `MNN_MEMORY_AHARDWAREBUFFER`, and the
rebind successfully imports it into a Vulkan buffer. Yet `getBuffer()` is never
called for that input. `VulkanRaster::onEncodeFast()` reads the source through
`VulkanBackend::getTensorBuffer()`, which blindly casts `tensor->deviceId()` to
`VulkanBuffer*`. An AHB-backed tensor stores a `VulkanAhbMemObj` in
`TensorUtils::getSharedMem()` instead, so the raster blit binds a stale
backend-owned buffer and copies zeros. The fix is to make VulkanRaster's input
bindings use `VulkanBackend::getBuffer()`, the only accessor that resolves AHB
memory objects.

## Narrowed scope: tensor format decides whether a wrap/resize is needed

The user narrowed the remaining work: internal Vulkan execution details are out
of scope. The only required proof is that an AHB-backed buffer can be bound as a
session-owned tensor input and reaches a Vulkan execution path. The output AHB
story is separate.

The simplified on-device regression reuses an INT32 Add graph with a non-zero
BLOB AHB. Binding before resize keeps the exact session tensor and its
`MNN_MEMORY_AHARDWAREBUFFER` flag. A host-upload control proves the graph is a
valid non-identity probe: the output is `input + bias` with packed RAW16 words.
With the AHB bound, the run still reaches Vulkan (`BinaryOp`, backend 7) and the
flag survives, but the readback contains only the bias:

```text
host upload:  output word 0 = input word 0 + 1
AHB input:    output word 0 = 1
```

So the narrowed binding/rebinding proof is satisfied: the session-owned tensor
is the bound tensor and Vulkan executes a real Add on it. Why the shader reads
zeros from the imported buffer is now an internal-execution detail and outside
the agreed scope. The user's format hypothesis remains plausible but is not
needed to close this task: a GPU-only BLOB AHB has no MNN data format of its own;
MNN assigns a `dimensionFormat`, and a mismatch can cause
`Pipeline::_InsertCopy()` / `WrapExecution` to use a different tensor.

Relevant source already identified:

- `third_party/MNN/source/core/WrapExecution.cpp:20` `needWrap()` only compares
  backend type and CPU pack/format; it does not inspect `buffer().flags`.
- `third_party/MNN/source/core/Pipeline.cpp:147` `_allocTensor()` calls
  `TensorUtils::setLinearLayout()` before `onAcquireBuffer()`.
- `third_party/MNN/source/core/TensorUtils.cpp:140`
  `setLinearLayout()` rounds channel extent to 4 for `NC4HW4`.
- `third_party/MNN/source/core/TensorUtils.cpp:81` the wrap tensor inherits
  `mMidFormat`, not necessarily the external tensor's format.

### Tensor type must preserve the raw 32-bit pattern

The user clarified that the BLOB AHB must not be represented as a `float32`
tensor. A RAW16 plane packed into 32-bit words is an opaque bit pattern; once
MNN treats it as `float32`, any arithmetic changes the IEEE-754 value and the
original raw bits cannot be recovered. The fixture must therefore use an
integer tensor:

- `include/MNN/HalideRuntime.h:61-63`: Halide type codes are
  `halide_type_int = 0`, `halide_type_uint = 1`, `halide_type_float = 2`.
- `tools/converter/source/onnx/onnxConverter.cpp:96-97` copies the ONNX tensor
  dtype into `InputParam::dtype` and sets `dformat = MNN_DATA_FORMAT_NCHW`.
- `tools/converter/source/onnx/onnxOpConverter.cpp:279-288` maps ONNX INT32 and
  UINT32 to MNN `DataType_DT_INT32`, so a 32-bit AHB word keeps its bit pattern.
- `source/backend/vulkan/buffer/backend/VulkanBackend.cpp:344-349`
  `getTensorSize()` allocates four bytes per element for integer tensors and
  rounds the element count to four lanes.
- `source/backend/vulkan/buffer/execution/VulkanBinary.cpp:110-128,213`
  selects `glsl_binary_int_*` when the input type code is `halide_type_int`, so
  integer Add/Sub is a real Vulkan operation rather than an optimized-away
  identity.

The user added the packing detail: RAW16 is a half-width integer format, so
two RAW16 samples are packed into one 32-bit word and split later with a 16-bit
shift. That is the workaround for GPUs that do not support INT16/UINT16. MNN
converter maps both ONNX INT32 and UINT32 to MNN INT32 (`DataType_DT_INT32`),
which preserves the 32-bit pattern even when the ONNX source type is unsigned.

### BLOB bytes are not a u8 tensor

The remaining concern is whether treating `AHARDWAREBUFFER_FORMAT_BLOB` as an
INT32 tensor is a hack to align AHB with MNN. It is not a format conversion:
BLOB is an opaque byte container, and `VulkanBufferFromAHB::create()` imports it
as a `VkBuffer`, which has no pixel format at all. The AHB descriptor only tells
the importer how many bytes exist; the MNN tensor type tells the shader how to
interpret those bytes.

MNN cannot represent UINT32 here:

- `Tensor::setType()` has no `DataType_DT_UINT32` case; ONNX UINT32 is converted
  to `DataType_DT_INT32`.
- `VulkanBackend::onAcquire()` asserts the Halide code is `float` or `int`, so a
  `halide_type_uint` tensor is rejected before import.
- Therefore `INT32` is the only bit-preserving 32-bit integer type available.
  The signedness is irrelevant for raw-word transport; unpacking masks to
  16-bit lanes before shifting, so sign extension cannot leak into a sample.

Choosing a pixel format such as RGBA8888 instead of BLOB would not make the
tensor u8. It would only change allocation layout/stride and could make a
`VkBuffer` import invalid. BLOB is the correct storage format for packed raw
words; the hardcoded `0x21` is confined to the test fixture, while production
paths describe the real camera AHB.

To make this contract explicit, `mnn_import_ahb_as_device_memory()` and
`mnn_rebind_ahb_device_memory()` now compare
`AHardwareBuffer_describe()` capacity against `Tensor::usize()`. An
undersized BLOB is rejected instead of silently binding a tensor whose Vulkan
buffer is larger than the imported memory. The on-device regression includes
that negative case, plus a host-upload control that proves the INT32 Add model
and converter preserve the packed RAW16 word pattern.

### Camera RAW16 AHB must carry GPU_SAMPLED_IMAGE usage

The last production gap is that the MediaNDK reader currently creates its
buffers without GPU usage. The observed camera AHB is private RAW16
(`format=0x20`) with usage `0x20003`, width 1600, height 1200, and 3,840,000
bytes. That is exactly two bytes per pixel, so the byte capacity matches the
packed INT32 tensor (`w*h/2` INT32 words). The failure is not the format or the
size: `VulkanBufferFromAHB::create()` queries only the AHB allocation size and
memory type bits, then imports the buffer with
`VkImportAndroidHardwareBufferInfoANDROID`. An AHB with no GPU usage cannot be
imported at all.

A direct allocation probe established the usable combination:

```text
BLOB + GPU_DATA_BUFFER        rc=0
RAW16 + GPU_SAMPLED_IMAGE     rc=0
RAW16 + GPU_DATA_BUFFER       rc=1
RAW16 + sampled + data buffer rc=1
```

The device gralloc rejects `GPU_DATA_BUFFER` for private RAW16, so requesting
that bit would keep the fallback path. However, a RAW16 AHB carrying
`GPU_SAMPLED_IMAGE` imports successfully through MNN's existing `VkBuffer`
path: the on-device `raw16_sampled_image_ahb_binds_as_vulkan_buffer_input`
regression binds it before resize, keeps the session tensor handle and AHB
flag, and reaches the Vulkan Add execution.

The fix has two parts:

1. Create the RAW16 `AImageReader` with `AImageReader_newWithUsage()` and
   request `CPU_READ_OFTEN | GPU_SAMPLED_IMAGE`. Keep `AImageReader_new()` as a
   fallback because a camera HAL may reject the usage request.
2. Only bind a MediaNDK AHB directly when `AHardwareBuffer_describe()` confirms
   `GPU_SAMPLED_IMAGE`, the plane is tightly packed, and capacity is at least
   the tensor byte size. Otherwise the existing host-staging path remains
   valid.

`AHARDWAREBUFFER_FORMAT_BLOB` stays test-fixture-only; production keeps the
real RAW16 format descriptor and treats its bytes as opaque packed words.

## Current regression after MNN trace cleanup (2026-11-27)

After rebuilding the trace-free MNN fork and pushing the matching shim libs,
`./shim-debug.sh verify` still reports `md5 verification: PASS (8 libraries)`,
and the focused on-device AHB suite still passes 5/5. The production app is not
yet acceptable:

```text
input AHB rebind failed: rc=-6; rebind: AHB import failed for 'raw_frame';
restored backend memory; using host upload
INFO [stats] 95 frames processed, last 600x800,
rgb mean [0.0,0.0,0.0], max 0, non-black 0.0%
```

No `HOST-DIAG`, `AHB-DIAG`, `[RUN]`, `[mnn_run]`, or `MNN-ISP-*` line appears in
the current run, so this is not stale instrumentation. The next diagnosis must
separate two failures: (1) why a `GPU_SAMPLED_IMAGE` RAW16 AHB that imports in
the fixture fails on the live camera buffer, and (2) why the documented host
fallback still produces a black output. Do not commit this branch until both are
resolved.

### Working hypothesis for the black fallback

The per-frame host source is not the obvious suspect: `copy_pixels()` reports
non-zero Bayer samples, `data_bytes()` exposes the same little-endian `u16`
memory, and `run_prepared()` selects `p.buf` when a camera AHB frame does not
set `input_buffer`. The remaining branch point is the input tensor's
`buffer().flags` immediately before `mnn_run_*`: the wrapper skips
`copyFromHostTensor()` whenever `MNN_MEMORY_AHARDWAREBUFFER` is still set. A
failed targeted rebind is supposed to clear that flag and restore backend-owned
memory, but `prepare()` can also install an AHB binding before a resize. The
next step is a short-lived debug log of the pre-run device binding and the first
staged host word. It must be removed once the branch is diagnosed; the permanent
proof belongs in this document and in a focused regression test.

The first new evidence comes from `logcat -s MNNJNI:*`, not the shim's own log:
the Vulkan backend repeatedly prints `Vulkan AHB import failed for tensor
device=0x... size=3840000`. That is the `VulkanBufferFromAHB::create()` failure
inside `VulkanBackend::onAcquire()`, not a capacity failure in the softisp glue.
The failure reason still needs to be resolved: the fixture's synthetic RAW16
`GPU_SAMPLED_IMAGE` AHB imports, while the live camera AHB of the same nominal
format/usage and byte size does not. The next diagnostic must capture the live
AHB descriptor in the shim log and the exact failing Vulkan call in the MNN fork.
