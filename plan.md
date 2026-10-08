# Rust JNI — no CMake
- Remove externalNativeBuild / cmake path from android-app/build.gradle.kts
- Delete cpp/ CMakeLists.txt, SoftispJni.cpp, AndroidHalBridge.cpp
- libcam_isp.so produced by cam-isp (cabi feature); loaded by SoftispJni.java System.loadLibrary("cam_isp")
- No Android CMake needed; JNI live in Rust cabi.rs

## Revised 2026-08-27 (after AOSP/Camera2/NDK study)
- Direct AOSP Camera2 NDK (API 24+): ACameraManager, ACameraDevice, ACaptureRequest, ACameraCaptureSession, ANativeWindow / AImageReader — implemented in AndroidCameraBridge direct stub
- Hybrid architecture confirmed: Java SurfaceView/permissions → native C++ (native-lib.cpp + CMake: camera2ndk + mediandk + android + log) → AImageReader (CPU frame processing via MNN Express) → ANativeWindow (preview render via MNN CV::ImageProcess / lockUnlockAndPost)
- Full MNN ISP pipeline (BLC, WB, YUV→RGB via _Interp/_Concat) + renderTensorToWindow applied; disconnect raw HAL preview loop (only ImageReader output) per AOSP migration guide
- All edits to production paths only; /tmp never used; build blocked at device SDK layer (not stub/code), APK assembly requires matching device SDK level
- JNI alignment verified: 17 Rust softisp_ exports → Java native; cabi + rectifier preserved
