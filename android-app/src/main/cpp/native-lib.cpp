#include <jni.h>
#include <android/log.h>
#include <android/native_window_jni.h>
#include "softisp_api.h"

#define LOG_TAG "SoftispNative"

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_setPreviewSurface(
    JNIEnv *env, jobject /*this*/, jobject surface) {
  if (surface == NULL) {
    softisp_release_native_window();
    return;
  }

  ANativeWindow *window = ANativeWindow_fromSurface(env, surface);
  if (window != NULL) {
    softisp_set_native_window(reinterpret_cast<void *>(window));
  } else {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_fromSurface failed");
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_releasePreviewSurface(
    JNIEnv * /*env*/, jobject /*this*/) {
  softisp_release_native_window();
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_startNativeCamera(
    JNIEnv *env, jobject /*this*/, jobject surface) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "startNativeCamera");
  if (surface == NULL) {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "NULL surface");
    return;
  }

  ANativeWindow *window = ANativeWindow_fromSurface(env, surface);
  if (window != NULL) {
    softisp_set_native_window(reinterpret_cast<void *>(window));
    ANativeWindow_setBuffersGeometry(window, 1280, 720, WINDOW_FORMAT_RGBA_8888);
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "SUCCESS");
  } else {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "fail");
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_stopNativeCamera(
    JNIEnv * /*env*/, jobject /*this*/) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "release");
  softisp_release_native_window();
}
