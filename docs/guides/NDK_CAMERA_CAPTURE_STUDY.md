# NDK Camera Capture — Study Notes

Study of the Android NDK Camera2 API + Media SDK (`AImageReader`) client-side
frame capture pattern, and how it maps to the `softisp` project.

> **Scope:** This covers the **app-side client API** (`ACameraManager`,
> `AImageReader`, `ACaptureRequest`). This is distinct from the **HAL
> implementation** that `cam-hal-android` provides (which Android's
> `cameraserver` daemon loads and calls into).

---

## 1. Permission Model (NDK app)

| Concern | NDK capability | Mechanism |
|---------|---------------|-----------|
| **Check** permission | ✅ API 31+ | `APermissionManager_checkPermission("android.permission.CAMERA", pid, uid)` → `PERMISSION_MANAGER_STATUS_GRANTED` |
| **Request** permission | ❌ Not possible natively | Must use Java/Kotlin `ActivityCompat.requestPermissions()` before starting native backend |
| **Declare** permission | ✅ Manifest | `<uses-permission android:name="android.permission.CAMERA" />` in `AndroidManifest.xml` |

**Key constraint:** Native code cannot pop a permission dialog. The Java/Kotlin
`Activity` must request `CAMERA` permission, receive `onRequestPermissionsResult`,
then call into native init (`initNativeCamera()`).

**softisp relevance:** Any Android demo app using `softisp` for on-device ISP
must wrap the native pipeline in a thin Java `Activity` that handles the
permission handshake first.

---

## 2. Media SDK — `AImageReader` (frame sink)

`AImageReader` allocates a gralloc-backed buffer queue and exposes an
`ANativeWindow` handle. The camera streams frames into this window; the reader
hands them to a listener callback.

```cpp
#include <media/NdkImageReader.h>
AImageReader* reader = nullptr;
ANativeWindow* window = nullptr;
AImageReader_new(1920, 1080, AIMAGE_FORMAT_YUV_420_888, 2, &reader); // 2 = max buffered
AImageReader_getWindow(reader, &window);   // ← ANativeWindow* for camera target
AImageReader_ImageListener listener{
    .context = nullptr,
    .onImageAvailable = [](void*, AImageReader* r){ processCapturedFrame(r); }
};
AImageReader_setImageListener(reader, &listener);
```

**Format notes:**
- `AIMAGE_FORMAT_YUV_420_888` → planes: [0]=Y (full res), [1]=U, [2]=V, with
  per-plane row strides.
- `AIMAGE_FORMAT_RGBA_8888` → single plane, tight packing.
- The `ANativeWindow*` from `AImageReader_getWindow()` is the **same type**
  our `vulkan_display::VulkanDisplay::new_from_window()` consumes.

**softisp relevance:** This `ANativeWindow*` is the bridge between camera
capture and our two existing modules:
- Input side: `AImageReader` window → camera frames
- Output side: `vulkan_display` window → rendered ISP result

---

## 3. NDK Camera — `ACameraManager` (device control)

```cpp
#include <camera/NdkCameraManager.h>
ACameraManager* mgr = ACameraManager_create();
ACameraIdList* ids = nullptr;
ACameraManager_getCameraIdList(mgr, &ids);          // enumerate
const char* id = ids->cameraIds[0];                 // pick back camera
ACameraDevice_StateCallbacks cb{ /* onDisconnected, onError */ };
ACameraManager_openCamera(mgr, id, &cb, &device);   // open
ACameraManager_deleteCameraIdList(ids);
```

**softisp relevance:** `cam-hal-android/src/lib.rs` already defines the
**server-side** mirror of these types (`camera3_device_ops`, `camera_module_t`,
`HAL_MODULE_INFO_SYM`). The NDK `ACameraManager` is the **client** that talks
to `cameraserver`, which proxies (via binder) to our HAL's `camera3_device_ops`.

---

## 4. Capture Session — `ACameraCaptureSession`

```cpp
ACaptureSessionOutputContainer* container = nullptr;
ACaptureSessionOutput* output = nullptr;
ACameraOutputTarget* target = nullptr;
ACaptureRequest* req = nullptr;
ACameraCaptureSession* session = nullptr;

ACaptureSessionOutputContainer_create(&container);
ACaptureSessionOutput_create(readerWindow, &output);          // bind AImageReader window
ACaptureSessionOutputContainer_add(container, output);
ACameraDevice_createCaptureSession(device, container, &cb, &session);
ACameraDevice_createCaptureRequest(device, TEMPLATE_PREVIEW, &req);
ACameraOutputTarget_create(readerWindow, &target);
ACaptureRequest_addTarget(req, target);
ACameraCaptureSession_setRepeatingRequest(session, nullptr, 1, &req, nullptr);
```

**softisp relevance:** `cam-binder/src/types.rs` already defines `TEMPLATE_PREVIEW`
(=1) and the AIDL `CaptureRequest`/`StreamConfig` parcelables for the binder
layer. The NDK `ACaptureRequest` is the **client** mirror of those AIDL types.

---

## 5. Pixel Access — `AImage`

```cpp
void processCapturedFrame(AImageReader* reader) {
    AImage* img = nullptr;
    if (AImageReader_acquireLatestImage(reader, &img) != AMEDIA_OK) return;
    int32_t planes = 0;
    AImage_getNumberOfPlanes(img, &planes);          // 3 for YUV_420_888
    uint8_t* y = nullptr; int32_t yLen = 0, yStride = 0;
    AImage_getPlaneData(img, 0, &y, &yLen);          // direct CPU ptr
    AImage_getPlaneRowStride(img, 0, &yStride);
    // ... feed y[] to ISP ...
    AImage_delete(img);                              // release back to queue
}
```

**softisp relevance:** The Y-plane bytes are exactly what `cam-isp`'s
RAW/Bayer input path expects. Combined with `cam-binder/src/hal_bridge.rs`
(`HardwareBufferBridge::lock_for_inference` for zero-copy AHardwareBuffer), we
have two frame-ingest paths:
- **CPU path:** `AImage_getPlaneData` → `Vec<u8>` → `cam-isp` (hermetic, testable)
- **Zero-copy path:** `AHardwareBuffer` lock → host ptr → MNN `set_tensor_host`

---

## 6. CMake / Link Dependencies

```
target_link_libraries(native_lib
    camera2ndk   # ACameraManager, ACaptureRequest, etc.
    mediandk     # AImageReader, AImage
    android      # ANativeWindow, ANativeActivity
    log)
```

Minimum API level: **24** (Camera2 NDK), **26** for `AHardwareBuffer`.

**softisp relevance:** `cam-isp/build.rs` already links `mediandk`. A native
capture demo would add `camera2ndk` + `android` to the link set.

---

## 7. Mapping to `softisp` Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Android App (Java Activity)                                     │
│   • Requests CAMERA permission                                    │
│   • Loads native lib, calls init()                               │
└───────────────────────────┬─────────────────────────────────────┘
                              │ JNI / native entry
┌─────────────────────────────▼──────────────────────────────────┐
│  NDK Camera CLIENT (ACameraManager + AImageReader)  ← AI Mode    │
│   • opens camera, creates AImageReader window                    │
│   • onImageAvailable → AImage → Y-plane bytes                   │
└───────────────┬───────────────────────────┬─────────────────────┘
                │                            │
        ┌───────▼────────┐          ┌────────▼─────────┐
        │  cam-isp       │          │  vulkan_display  │
        │  (processing)  │          │  (output)        │
        └───────┬────────┘          └────────▲─────────┘
                │                            │
        ┌───────▼────────┐                   │
        │  cam-binder    │   AHardwareBuffer │
        │  hal_bridge    │ ← zero-copy ──────┘
        └───────┬────────┘
                │ binder (ICameraProvider/Device/Session)
┌───────────────▼────────────────────────────────────────────────┐
│  cam-hal-android  (HAL server: camera3_device_ops)   ← exists   │
│   • loaded by cameraserver, proxies to actual sensor             │
└─────────────────────────────────────────────────────────────────┘
```

### What exists vs. what's missing

| Component | Status | Location |
|-----------|--------|----------|
| HAL server (`camera3_device_ops`) | ✅ Implemented | `cam-hal-android/src/lib.rs` |
| Binder AIDL types (`CaptureRequest`, `StreamConfig`) | ✅ Implemented | `cam-binder/src/types.rs` |
| `CameraProvider` enumeration | ⚠️ Stub on Android | `cam-binder/src/provider.rs` |
| AHardwareBuffer ↔ MNN bridge | ✅ Implemented | `cam-binder/src/hal_bridge.rs` |
| `VulkanDisplay` (output) | ✅ Implemented | `cam-hal-android/src/vulkan_display.rs` |
| **NDK Camera CLIENT (`ACameraManager` + `AImageReader`)** | ❌ **Missing** | — |
| Java permission wrapper | ❌ Missing | — |

---

## 8. Gap Analysis & Next Steps

The AI Mode conversation describes the **client-side capture** layer that is
currently absent from `softisp`. To build an end-to-end Android demo
(camera → ISP → display), we need:

1. **Client-side capture module** (new crate or module in `cam-hal-android`):
   - Wrap `ACameraManager` + `AImageReader` in safe Rust (`unsafe` FFI block)
   - Provide `CameraCapture::new()` → `on_frame(|y_plane: &[u8]| {...})`
   - Link `camera2ndk` + `mediandk` (add to `build.rs` or Cargo deps)

2. **Java `Activity` shim** (for permission + JNI entry):
   - `AndroidManifest.xml` with `CAMERA` permission
   - `requestPermissions()` before native init
   - `System.loadLibrary("softisp_capture")`

3. **Wire capture → ISP → display**:
   - `AImage` Y-plane → `cam-isp` `process_raw_frame()`
   - ISP output → `VulkanDisplay::new_from_window()` (or AImageReader window)

4. **Hermetic CPU-path test** (no camera needed):
   - Reuse existing `generate_test_frame()` from `cam-isp/examples/camera_isp.rs`
   - Feed test pattern through capture→ISP→display chain on host

### Where to implement

- **New crate `cam-capture`** (cleanest separation): NDK Camera client API
  in safe Rust. Depends on `cam-types`, `cam-isp`, `cam-hal-android`.
- **Or module in `cam-hal-android`**: but that crate is the HAL server —
  mixing client API there is confusing. A separate crate is preferred.

### Build constraints

- NDK target `aarch64-linux-android` (not host Termux)
- `minSdk 24` for camera2ndk, `26` for AHardwareBuffer
- Feature-gated (`#[cfg(target_os = "android")]`) so host tests still pass
