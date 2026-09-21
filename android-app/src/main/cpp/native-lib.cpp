#include <jni.h>
#include <android/native_window_jni.h>
#include <android/log.h>
static ANativeWindow* w=0;
#define LOG_TAG "SoftispNative"
extern "C" JNIEXPORT void JNICALL Java_com_softisp_camera_SoftispJni_startNativeCamera(JNIEnv* env,jobject,jobject s){__android_log_print(ANDROID_LOG_INFO,LOG_TAG,"load");if(s==NULL){__android_log_print(ANDROID_LOG_ERROR,LOG_TAG,"NULL");return;}w=ANativeWindow_fromSurface(env,s);if(w){__android_log_print(ANDROID_LOG_INFO,LOG_TAG,"SUCCESS");ANativeWindow_setBuffersGeometry(w,1280,720,WINDOW_FORMAT_RGBA_8888);}else __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,"fail");}
extern "C" JNIEXPORT void JNICALL Java_com_softisp_camera_SoftispJni_stopNativeCamera(JNIEnv*,jobject){if(w){__android_log_print(ANDROID_LOG_INFO,LOG_TAG,"release");ANativeWindow_release(w);w=0;}}
