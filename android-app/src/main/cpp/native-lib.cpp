#include <jni.h>
#include <android/log.h>
#include <android/native_window_jni.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
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

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_setPreviewSurface(
    JNIEnv *env, jobject /*this*/, jobject surface) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "setPreviewSurface enter");
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
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "releasePreviewSurface");
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
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "ANativeWindow_fromSurface failed");
  }
}

extern "C" JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_stopNativeCamera(
    JNIEnv * /*env*/, jobject /*this*/) {
  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, "stopNativeCamera");
  softisp_release_native_window();
}
