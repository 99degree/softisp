#include <jni.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <android/log.h>

// Declare the Rust init function
extern void cam_isp_init(void);

#define LOG_TAG "SoftispJni"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
    (void)vm;
    (void)reserved;
    LOGI("JNI_OnLoad");
    cam_isp_init();
    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
Java_com_softisp_camera_SoftispJni_setPreviewSurface(JNIEnv* env, jclass clazz, jobject surfaceTexture) {
    LOGI("setPreviewSurface called");
    // TODO: Implement actual native window setting
    (void)env;
    (void)clazz;
    (void)surfaceTexture;
}