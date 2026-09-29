#include <jni.h>
#include <android/log.h>
#include <android/native_window_jni.h>
#include <android/surface.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <android/bitmap.h>
#include "softisp_api.h"

#define LOG_TAG "SoftispNative"
#define CRASH_LOG_PATH "/sdcard/download/softisp-app.log"

static void softisp_log_to_file(const char *msg) {
    int fd = open(CRASH_LOG_PATH, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd >= 0) {
        write(fd, msg, strlen(msg));
        close(fd);
    }
}

static void crash_handler(int sig, siginfo_t *info, void *ucontext) {
    char buf[512];
    snprintf(buf, sizeof(buf),
             "Softisp crash: signal=%d code=%d addr=%p\n",
             sig, info->si_code, info->si_addr);
    softisp_log_to_file(buf);
    // Re-raise with default handler so system also records the crash.
    signal(sig, SIG_DFL);
    raise(sig);
}

static void install_crash_handler(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = crash_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGILL, &sa, NULL);
    sigaction(SIGFPE, &sa, NULL);
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    install_crash_handler();
    return JNI_VERSION_1_6;
}

static void draw_test_pattern(ANativeWindow *window) {
    if (window == NULL) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "draw_test_pattern: window is null");
        return;
    }

    int width = ANativeWindow_getWidth(window);
    int height = ANativeWindow_getHeight(window);
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "draw_test_pattern: window size %dx%d", width, height);

    if (width <= 0 || height <= 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "draw_test_pattern: invalid size %dx%d", width, height);
        return;
    }

    ANativeWindow_Buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    int rc = ANativeWindow_lock(window, &buffer, NULL);
    if (rc != 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_lock failed: %d", rc);
        return;
    }
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ANativeWindow locked: format=%d stride=%d height=%d",
                        buffer.format, buffer.stride, buffer.height);

    // Draw a simple gradient pattern to verify drawing works
    if (buffer.format == WINDOW_FORMAT_RGBA_8888) {
        uint32_t *pixels = static_cast<uint32_t *>(buffer.bits);
        for (int y = 0; y < buffer.height; ++y) {
            for (int x = 0; x < buffer.width; ++x) {
                // Simple gradient: red increases with x, green increases with y, blue is constant
                uint8_t r = (x * 255) / buffer.width;
                uint8_t g = (y * 255) / buffer.height;
                uint8_t b = 128;
                uint8_t a = 255;
                pixels[y * buffer.stride + x] = (a << 24) | (b << 16) | (g << 8) | r;
            }
        }
        __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "Test pattern drawn (RGBA_8888)");
    } else {
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG, "Unsupported window format: %d", buffer.format);
    }

    rc = ANativeWindow_unlockAndPost(window);
    if (rc != 0) {
        __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_unlockAndPost failed: %d", rc);
    } else {
        __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "Test pattern posted to window");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_setPreviewSurface(
    JNIEnv *env, jobject /*this*/, jobject surface) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "setPreviewSurface enter");
  if (surface == NULL) {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "setPreviewSurface surface is null");
    softisp_release_native_window();
    return;
  }

  ANativeWindow *window = ANativeWindow_fromSurface(env, surface);
  if (window != NULL) {
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ANativeWindow acquired from Surface");
    softisp_set_native_window(reinterpret_cast<void *>(window));
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "softisp native window set");
    draw_test_pattern(window);
  } else {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_fromSurface failed");
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_releasePreviewSurface(
    JNIEnv * /*env*/, jobject /*this*/) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "releasePreviewSurface");
  softisp_release_native_window();
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_startNativeCamera(
    JNIEnv *env, jobject /*this*/, jobject surface) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "startNativeCamera enter");
  if (surface == NULL) {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "NULL surface");
    return;
  }

  ANativeWindow *window = ANativeWindow_fromSurface(env, surface);
  if (window != NULL) {
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "ANativeWindow acquired in startNativeCamera");
    softisp_set_native_window(reinterpret_cast<void *>(window));
    int rc = ANativeWindow_setBuffersGeometry(window, 1280, 720, WINDOW_FORMAT_RGBA_8888);
    if (rc == 0) {
      __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "Buffer geometry set 1280x720 RGBA_8888");
    } else {
      __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "setBuffersGeometry failed: %d", rc);
    }
    draw_test_pattern(window);
    __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "startNativeCamera SUCCESS");
  } else {
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_fromSurface failed");
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_stopNativeCamera(
    JNIEnv * /*env*/, jobject /*this*/) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "stopNativeCamera");
  softisp_release_native_window();
}
