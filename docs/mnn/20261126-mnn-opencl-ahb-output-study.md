# MNN fork OpenCL backend: AHardwareBuffer / zero-copy output study (2026-11-26)

## Scope and verdict

Fork studied: `~/MNN` at `b2f05a51`. Repo: `~/softisp`. Read-only study; no source files were changed.

**Verdict: OpenCL does not add any output-device-info capability over Vulkan.** It does have a real, Android-only AHardwareBuffer (AHB) import path, based on vendor OpenCL extensions, but:

- it is **not** an `onAcquire`/tensor-allocation path — it runs from the copy helpers (`_allocHostBuffer`), so it is driven by explicit host↔device copy calls (the MNN **Express** `Variable::setDevicePtr` / `Variable::copyToDevicePtr` flow in the fork's own test), not by binding a session tensor and running inference;
- it **cannot hand the caller a `cl_mem`, a `cl_context`, or an `AHardwareBuffer*` back** — there is no OpenCL equivalent of `MNNVulkanTensorContent` / `getDeviceInfo` / `MNNMetalGetTensorContent`;
- `Tensor::getDeviceInfo(dst, MNN_FORWARD_OPENCL)` is **always false** in this fork;
- softisp does not build or expose OpenCL at all (`MNN_OPENCL=OFF`, no `libMNN_OpenCL.so`, no OpenCL route in `MnnBackend`), so the capability is unreachable from the app today.

Net: for softisp's AHB output ambition, the current Vulkan fork path (`VulkanBackend::onAcquire` + `VulkanBufferFromAHB` + `VulkanBackend::onGetTensorInfo`) is strictly more capable and is already the supported route.

## OpenCL tensor info / mapping / sync path with citations

### AHB import actually exists — but in copy helpers, not allocation

- `source/backend/opencl/core/OpenCLBackend.cpp:785-832` — `_allocHostBuffer(length, tensor)`:
  - checks `tensor->buffer().flags == MNN_MEMORY_AHARDWAREBUFFER` under `#ifdef __ANDROID__` (`:786`, `:791-792`);
  - **Mali**: `clImportMemoryARM` with `CL_IMPORT_TYPE_ANDROID_HARDWARE_BUFFER_ARM` / `CL_IMPORT_MEMORY_WHOLE_ALLOCATION_ARM` (`:796-798`);
  - **Adreno**: `CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM` with `cl_mem_ahardwarebuffer_host_ptr.ahb_ptr` (`:800-805`);
  - other GPUs / unsupported device → error (`:807-819`);
  - wraps the imported `cl::Buffer` in `CLSharedMemReleaseBuffer` and stores it in `TensorUtils::setSharedMem()` (`:794-806`; class at `OpenCLBackend.hpp:270-288`);
  - normal tensors use a private `CL_MEM_ALLOC_HOST_PTR` staging buffer (`:823-829`).
- Capability detection: `source/backend/opencl/core/runtime/OpenCLRuntime.cpp:209-214` requires `cl_arm_import_memory_android_hardware_buffer` (Mali) or `cl_qcom_android_ahardwarebuffer_host_ptr` (Adreno). It is compiled only when `MNN_USE_LIB_WRAPPER` is defined (`source/backend/opencl/CMakeLists.txt:25-28`, Android case). Symbol plumbing: `OpenCLWrapper.hpp:182`, `:184-187`; `OpenCLWrapper.cpp:125-127`, `:131-137`, `:232-235`, `:814-823`.
- Output direction: `OpenCLBackend.cpp:945-995` (`copyFromDevice`) and `:998-1005` / `:1044-1106` (`copyToDevice`), AHB routing in `:1108-1131` (`copyBetweenDevice` passes `srcMemtype`/`dstMemtype` at `:1126-1128`); dispatch in `:1157-1173`.
- GPU-side conversion kernel: `source/backend/opencl/core/BufferConvertor.cpp:574-669` (`convertBetweenAHDandCLmem`): RGBA8 `gl_to_cl`/`cl_to_gl` and YUV420 `yuv_to_cl`/`cl_to_yuv` (`:592-623`), kernel args include the imported AHB `cl::Buffer` (`:632-649`), enqueue + `event.wait()` (`:663-667`). No CPU memcpy.
- **Contradiction with docs:** `MNN/VULKAN_ZEROCOPY_ARCHITECTURE.md:446-498` says the import is in `onAcquire` with stale line numbers. In the actual source, `OpenCLBackend::onAcquire` (`OpenCLBackend.cpp:448-586`) never reads `buffer().flags` and unconditionally writes a pool-allocated `cl::Buffer`/`cl::Image` into `buffer().device` (`:495`, `:504`, `:513`, `:521`, `:532`, `:567`, `:576`, `:585`). Binding an AHB to a session tensor before resize is therefore overwritten, unlike Vulkan.
- The fork's own working example uses Express: `tools/cpp/GpuInterTest.cpp:378-412` (`_Input` + `setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` on input, `copyToDevicePtr(...)` on output), with AHB allocation at `:177-211`. `docs/faq.md:245-249` says the same: `setDevicePtr` for input, `copyToDevicePtr` for output, and notes OpenCL supports OpenCL/OpenGL memory I/O.
- `copyToDevicePtr` is **Express-only** (`include/MNN/expr/Expr.hpp:119-120`; `express/Expr.cpp:591-613` builds a temp tensor, calls the backend's `onCopyBuffer`, then `wait`). Plain `include/MNN/Tensor.hpp` has no `copyToDevicePtr` (`:296-310` only map/unmap/wait/setDevicePtr).
- Format limits: only `AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM` and `AHARDWAREBUFFER_FORMAT_Y8Cb8Cr8_420` (`BufferConvertor.cpp:604-623`; same claim in `VULKAN_ZEROCOPY_ARCHITECTURE.md:578-586`). No F32/RGBA16 path, and no Linux dma-buf precedent.

### Tensor info: no OpenCL export API

- `source/core/Tensor.cpp:510-519`: `getDeviceInfo(dst, type)` returns false when backend type differs, otherwise calls `Backend::onGetTensorInfo`.
- `source/core/Backend.hpp:228-230`: default `onGetTensorInfo` returns false. Global search finds overrides only in `source/backend/vulkan/buffer/backend/VulkanBackend.cpp:840-850` and `source/backend/metal/MetalBackend.mm:729`; **OpenCL has none**.
- Public header `include/MNN/MNNSharedContext.h`: `MNNVulkanTensorContent` (`:27-34` — `VkBuffer`, size, offset, type, mask), `MNNMetalTensorContent` + `MNNMetalGetTensorContent` (`:44-54`), but **no OpenCL tensor-content struct or getter**. The only OpenCL-related public item is `MNNDeviceContext::contextPtr` (`:57-70`), used at session creation to select/reuse an OpenCL context (`source/backend/opencl/core/OpenCLBackend.cpp:38-44`), not to export a tensor.
- Therefore `getDeviceInfo(dst, MNN_FORWARD_OPENCL)` (`MNNForwardType.h:33`) is never useful: either the backend type mismatches (false) or OpenCL inherits the base stub (false). `getDeviceInfo(dst, MNN_FORWARD_VULKAN)` returns `MNNVulkanTensorContent` as above.

### map/unmap/sync

- Core fallback: `Tensor::map()` calls `onSync(READ,false)` then `onMapTensor`; if no backend override, it `malloc`s and does `onCopyBuffer` into it (`source/core/Tensor.cpp:427-461`); `unmap()` mirrors that for writes (`:463-488`). `wait()` delegates to `onSync` (`:489-496`).
- OpenCL overrides both map hooks: `source/backend/opencl/core/OpenCLBackend.cpp:1212-1271` (SVM fast path / malloc + `onCopyBuffer` fallback) and `:1273-1316` (unmap). Thus `Tensor::map()` on an OpenCL output returns a **separate SVM/malloc buffer with a copy**, never the AHB's CPU mapping and never the raw `cl_mem`.
- OpenCL `onSync` is just `queue.finish()` when `toCpu` is true (`:884-889`); it ignores `mtype` and the tensor. Vulkan does not override `onMapTensor`/`onUnmapTensor` at all and `onSync` always calls `_finish()` (`source/backend/vulkan/buffer/backend/VulkanBackend.cpp:829-832`). For an AHB output, OpenCL's visibility guarantee comes from `event.wait()` inside `convertBetweenAHDandCLmem` (`BufferConvertor.cpp:666`), not from `Tensor::wait`. softisp's `output->wait(MAP_TENSOR_READ, true)` (`mnn_wrapper.cpp:776`) would only flush the OpenCL queue; it would not copy the result into the AHB.

## Public vs internal API accessibility from softisp's wrapper

- softisp compiles `mnn_wrapper.cpp` with **only** `build.include(&mnn_include)` (`cam-rust/cam-isp/build.rs:357-375`; include-dir selection `:625-636`; header priority documented `:14-26`). It links `MNN`, `MNN_Vulkan`, `MNN_Express` (`:421-449`) and copies only Vulkan companion libs (`:337-355`). There is no `MNN_CL` linkage and no `source/` include path.
- `OpenCLBackend` lives in `source/backend/opencl/core/OpenCLBackend.hpp:86-206`, includes internal headers (`core/Backend.hpp`, `MNN_generated.h`, `backend/opencl/core/BufferPool.hpp`, etc. at `:12-25`) and is not exported in `include/MNN`. Its `getBufferPool()` (`:116-118`), `CLRuntime`, `openCLBuffer()`/`openCLImage()` (`source/backend/opencl/core/OpenCLRunningUtils.hpp:102-110`), and `CLSharedMemReleaseBuffer` are all internal. No `extern "C"`/`MNN_PUBLIC` OpenCL accessor exists anywhere under `source/backend/opencl` (search yielded none).
- Consequence: an external C++ wrapper compiled against `include/MNN` only **cannot** retrieve a `cl_mem`, `cl_context`, device buffer size/offset, or the imported AHB from an output tensor. It can only use `Tensor::buffer().device`/`flags` (`include/MNN/Tensor.hpp:164-169`, `:210-212`) and the public `setDevicePtr`/`map`/`wait` calls — none of which export an OpenCL buffer.
- Partial public route that would work for OpenCL (not used by softisp): mimic `Variable::copyToDevicePtr` with public `Tensor` APIs — construct a temp `MNN::Tensor`, `setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)`, then `output->copyToHostTensor(&temp)` (`source/core/Tensor.cpp:187-193`) — which reaches OpenCL `copyBetweenDevice` at `OpenCLBackend.cpp:1126-1128`. That writes pixels into the AHB but still gives the caller no way to *query* the device buffer; the caller reads its own AHB CPU mapping.

## Comparison table vs Vulkan

| Aspect | OpenCL (this fork) | Vulkan (this fork, buffer backend) |
|---|---|---|
| AHB import | Yes, Android only, vendor extensions; Mali `clImportMemoryARM` / Adreno `CL_MEM_EXT_HOST_PTR_QCOM` (`OpenCLBackend.cpp:785-819`) | Yes, Android AHB + Linux dma-buf (`VulkanBackend.cpp:357-383`; `VulkanBuffer.cpp:126-224`) |
| Import hook | `_allocHostBuffer` from explicit copy helpers (`OpenCLBackend.cpp:785`, `:975`, `:1084`, `:1118`) | `onAcquire` on tensor allocation (`VulkanBackend.cpp:351-383`) |
| `onAcquire` checks `MNN_MEMORY_AHARDWAREBUFFER` | **No** — overwrites `device` from pool (`OpenCLBackend.cpp:448-586`) | **Yes** — `getAhbMem` / per-tensor `VulkanAhbMemObj` (`VulkanBackend.cpp:290-336`, `:357-382`) |
| Per-tensor ownership of external buffer | `CLSharedMemReleaseBuffer` via `setSharedMem` (`OpenCLBackend.hpp:270-288`) | `VulkanAhbMemObj` + shared buffer via `setSharedMem` (`VulkanBackend.cpp:295-313`) |
| Output path into AHB | Express `copyToDevicePtr` → `cl_to_gl`/`cl_to_yuv` GPU kernel, blocking `event.wait()` (`BufferConvertor.cpp:604-667`; `Expr.cpp:591-613`) | Direct GPU write into imported `VkBuffer`; `wait(MAP_TENSOR_READ,true)` → `_finish()` (`VulkanBackend.cpp:829-832`) |
| Caller can retrieve device/buffer info | **No** — no OpenCL struct/getter, no AHB export; `getDeviceInfo` always false | **Yes** — `getDeviceInfo(dst, MNN_FORWARD_VULKAN)` → `MNNVulkanTensorContent` (`VulkanBackend.cpp:840-850`; `MNNSharedContext.h:27-34`) |
| `Tensor::map()` on GPU output | Overrides with SVM/malloc + copy (`OpenCLBackend.cpp:1212-1271`) | No override; core malloc + `onCopyBuffer` (`Tensor.cpp:448-460`) |
| Supported AHB formats | RGBA8, YUV420 only (`BufferConvertor.cpp:604-623`) | RGBA8 / F32 storage-buffer byte path; no YUV420 (`VULKAN_ZEROCOPY_ARCHITECTURE.md:578-586`) |
| Build state in this checkout | `MNN_OPENCL:BOOL=OFF` (`~/MNN/build_isp/CMakeCache.txt:495`); no `libMNN_OpenCL.so` | `MNN_VULKAN:BOOL=ON`, `MNN_VULKAN_IMAGE:BOOL=OFF`; `libMNN_Vulkan.so` built and linked |

## Relevance to softisp / recommendations

- softisp's inference route is Vulkan-only: `MnnBackend` has a single `Vulkan` variant and explicitly excludes OpenCL (`cam-rust/cam-isp/src/mnnengine.rs:163-185`). The wrapper enum does list `MNN_BACKEND_OPENCL = 3` and maps it to `MNN_FORWARD_OPENCL` (`mnn_sys/mnn_wrapper.h:30-37`; `mnn_wrapper.cpp:110-111`), but that enum value is not reachable from the engine. OpenCL appears only in comments/examples/tests (`engine.rs:311`, `auto_profile.rs:37`, `examples/bench_hd.rs:27`, `tests/test_mnn_engine.rs:768-774`).
- The existing AHB FFI is Vulkan-oriented: `mnn_import_ahb_as_device_memory` calls `Tensor::setDevicePtr(ahb, MNN_MEMORY_AHARDWAREBUFFER)` (`mnn_wrapper.cpp:811-854`) and `mnn_run_ahb_output` checks the flag then `wait()` (`:749-781`). In the **current fork** that Vulkan path is now real (`VulkanBackend.cpp:357-383`), so the old "Vulkan ignores setDevicePtr" diagnosis is stale.
- Do **not** switch to OpenCL expecting a superior output-info path: there is none. If AHB output is needed on OpenCL, it would require (a) building `MNN_CL` with `MNN_OPENCL=ON`, (b) using or emulating the Express copy flow, (c) vendor extension support and RGBA8/YUV420/NC4HW4 layout compatibility, and (d) new wrapper code — a larger change than the already-present Vulkan route.
- Keep using Vulkan + `getDeviceInfo(MNN_FORWARD_VULKAN)` if a native buffer handle must be handed to another consumer (e.g. media/Gralloc); OpenCL offers no equivalent.

### Contradictions with prior findings / fork docs to fix later

- `docs/overview/ROADMAP.md:227-233`, `:238-250`, `:301-350` and `docs/isp/20261125-ahb-buffer-pool-and-zero-copy-analysis.md:441-448` state Vulkan does not consume `setDevicePtr`/`buffer().flags` and that "Only the OpenCL backend consumes `setDevicePtr`". **False for the current fork** — `VulkanBackend.cpp:357-383` and `:315-336` do exactly that.
- `MNN/VULKAN_ZEROCOPY_ARCHITECTURE.md:446-498` describes OpenCL AHB import as happening in `onAcquire` with stale line numbers. **False** — import is in `_allocHostBuffer` driven by copy helpers; `onAcquire` ignores the flag.
- `docs/overview/ROADMAP.md:315` cites `OpenCLBackend.cpp:747` / `VulkanBackend.cpp:218`; current lines are `:785-832` and `:351-383`.

## Open questions / on-device validation needs

- Is `libMNN_Vulkan.so` in the shipped APK built from this fork with `VK_ANDROID_external_memory_android_hardware_buffer` enabled and `MNN_VULKAN_IMAGE=OFF`? (Here: `MNN_VULKAN_IMAGE=OFF`; extension enablement still needs device-side confirmation.)
- Does `mnn_run_ahb_output`'s `wait(MAP_TENSOR_READ,true)` actually make the imported `VkBuffer` writes visible to `AHardwareBuffer_lock()` on Adreno/Mali, or is an explicit fence/`AHardwareBuffer_unlock` fence handoff required?
- Does the ISP output tensor's NC4HW4/F16 layout match the caller's AHB format/stride (RGBA8 linear) well enough to avoid a reformat kernel? OpenCL's kernel handles RGBA8/YUV420 explicitly; Vulkan writes raw bytes into a storage buffer.
- Would `Tensor::getDeviceInfo(...)` be useful to softisp at all? It returns `VkBuffer`/size/offset, **not** `AHardwareBuffer*`, so a consumer needing an AHB must own the AHB it imported and pass it alongside the tensor info.
- OpenCL route remains unvalidated on device: no `libMNN_OpenCL.so` is built or shipped, so its AHB path cannot be exercised without a separate MNN build and an app-side backend selection change.
