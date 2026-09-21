# ProGuard rules for softisp JNI
-keep class com.softisp.camera.** { *; }
-keep interface com.softisp.camera.** { *; }
-keepclassmembers class * implements com.softisp.camera.CameraProcessor {
    public void processFrame(byte[], int, int, int);
}

# JNI method preservation
-keepclassmembers class com.softisp.camera.SoftispJni {
    public static native byte[] composeOnnx(int, int, java.lang.String);
    public static native byte[] composeMnn(int, int, java.lang.String);
    public static native long createProfile();
    public static native void freeProfile(long);
    public static native int profileSetFlag(long, java.lang.String, int);
    public static native int profileSetDemosaicQuality(long, int);
    public static native int profileSetRotateMode(long, int);
    public static native int profileSetOutputFormat(long, int);
    public static native int profileSetEisMargin(long, double);
    public static native int profileSetStatsDownscaleMax(long, int);
    public static native int profileSetPipelineDownscaleTarget(long, int);
    public static native int profileSetTiling(long, int, int, int);
    public static native byte[] composeOnnxWithProfile(int, int, long);
    public static native byte[] composeMnnWithProfile(int, int, long);
    public static native int setOpsetMode(int);
    public static native int clearOpsetMode();
    public static native java.lang.String getVersion();
    public static native java.lang.String getErrorString(int);
    public static native boolean isLibraryAvailable();
}

# Keep all native method signatures (no void* types in ProGuard)
-keepclassmembers class * {
    public static native byte[] composeOnnx(int, int, java.lang.String);
    public static native byte[] composeMnn(int, int, java.lang.String);
    public static native long createProfile();
    public static native void freeProfile(long);
    public static native int profileSetFlag(long, java.lang.String, int);
    public static native int profileSetDemosaicQuality(long, int);
    public static native int profileSetRotateMode(long, int);
    public static native int profileSetOutputFormat(long, int);
    public static native int profileSetEisMargin(long, double);
    public static native int profileSetStatsDownscaleMax(long, int);
    public static native int profileSetPipelineDownscaleTarget(long, int);
    public static native int profileSetTiling(long, int, int, int);
    public static native byte[] composeOnnxWithProfile(int, int, long);
    public static native byte[] composeMnnWithProfile(int, int, long);
    public static native int setOpsetMode(int);
    public static native int clearOpsetMode();
    public static native java.lang.String getVersion();
    public static native java.lang.String getErrorString(int);
    public static native boolean isLibraryAvailable();
}