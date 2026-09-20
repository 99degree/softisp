package com.softisp.camera;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.util.Log;

public class SoftispJni {
    static { System.loadLibrary("cam_isp"); }

    // ========================================================================
    // Model Composition API (matches Rust cabi exports)
    // ========================================================================

    /**
     * Compose an ONNX model for the given profile and resolution.
     * 
     * @param width Image width in pixels
     * @param height Image height in pixels
     * @param profile Profile name: "LITE", "MED", "HEAVY", "PRO", "UNIFIED", "HDR", "TEST"
     * @return Serialized ONNX ModelProto as byte array, or empty array on error
     */
    public static native byte[] composeOnnx(int width, int height, String profile);

    // ========================================================================
    // Custom Profile Management
    // ========================================================================

    /**
     * Create a new custom pipeline profile with default values.
     * 
     * @return Opaque handle (pointer) to the profile, or 0 on failure.
     *         Must be freed with {@link #profileFree(long)}.
     */
    public static native long profileCreate();

    /**
     * Free a custom pipeline profile handle.
     * 
     * @param handle Handle returned by {@link #profileCreate()}
     */
    public static native void profileFree(long handle);

    /**
     * Free a buffer returned by {@link #composeOnnx} or {@link #composeOnnxWithProfile}.
     *
     * @param bytes Buffer to free
     * @param len Buffer length
     */
    public static native void free(byte[] bytes, int len);

    /**
     * Set a boolean feature flag on a custom profile.
     * 
     * @param handle Profile handle from {@link #profileCreate()}
     * @param flag Flag name (e.g., "use_fcs", "use_ldci", "use_ee", "use_warp",
     *             "use_unsharp", "use_lsc", "use_hdr", "use_zone_stats",
     *             "use_channel_means", "use_tone_stats", "use_histogram",
     *             "use_bilateral", "use_saturation", "use_vignetting",
     *             "use_colorspace", "use_gamma", "use_sharpen",
     *             "use_wavelet_denoise", "use_auto_contrast", "use_normalize",
     *             "use_tiled_rendering", "use_unpack", "force_input16")
     * @param value Non-zero to enable, 0 to disable
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetFlag(long handle, String flag, int value);

    /**
     * Set the demosaic quality on a custom profile.
     * 
     * @param handle Profile handle
     * @param quality {@link DemosaicQuality#STANDARD}, {@link DemosaicQuality#HQ_LINEAR}, or {@link DemosaicQuality#EDGE}
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetDemosaicQuality(long handle, int quality);

    /**
     * Set the rotation mode on a custom profile.
     * 
     * @param handle Profile handle
     * @param mode 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetRotateMode(long handle, int mode);

    /**
     * Set the output pixel format on a custom profile.
     * 
     * @param handle Profile handle
     * @param format One of {@link OutputFormat} constants
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetOutputFormat(long handle, int format);

    /**
     * Set the EIS margin fraction on a custom profile.
     * 
     * @param handle Profile handle
     * @param margin Margin fraction (e.g., 0.05 for 5% border)
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetEisMargin(long handle, double margin);

    /**
     * Set the stats downscale max dimension on a custom profile.
     * 
     * @param handle Profile handle
     * @param maxPx Maximum dimension in pixels for stats computation (0 = auto)
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetStatsDownscaleMax(long handle, int maxPx);

    /**
     * Set the pipeline downscale target on a custom profile.
     * 
     * @param handle Profile handle
     * @param maxPx Maximum dimension in pixels for pipeline (0 = no downscale)
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetPipelineDownscaleTarget(long handle, int maxPx);

    /**
     * Set tiled rendering parameters on a custom profile.
     * 
     * @param handle Profile handle
     * @param tileCountX Number of tiles in X dimension
     * @param tileCountY Number of tiles in Y dimension
     * @param overlap Overlap in pixels between tiles
     * @return {@link SoftispError#OK} on success
     */
    public static native int profileSetTiling(long handle, int tileCountX, int tileCountY, int overlap);

    /**
     * Compose an ONNX model using a custom pipeline profile.
     * 
     * @param width Image width
     * @param height Image height
     * @param handle Custom profile handle
     * @return Serialized ONNX ModelProto as byte array
     */
    public static native byte[] composeOnnxWithProfile(int width, int height, long handle);


    // ========================================================================
    // Global Opset Mode
    // ========================================================================

    /**
     * Set the global opset mode for ONNX emission.
     * 
     * @param mode {@link OpsetMode#PRIMITIVE} or {@link OpsetMode#CUSTOM}
     * @return {@link SoftispError#OK} on success
     */
    public static native int setOpsetMode(int mode);

    /**
     * Clear the global opset mode override.
     * 
     * @return {@link SoftispError#OK} on success
     */
    public static native int clearOpsetMode();

    // ========================================================================
    // Library Info
    // ========================================================================

    /**
     * Get the softisp library version string.
     * 
     * @return Version string (e.g., "cam-isp 0.1.0 GPL-3.0-or-later")
     */
    public static native String version();

    /**
     * Sets the preview surface for camera display.
     * The native code will create an ANativeWindow from the SurfaceTexture.
     *
     * @param surfaceTexture The SurfaceTexture from TextureView
     */
    public static native void setPreviewSurface(SurfaceTexture surfaceTexture);

    /**
     * Get a human-readable error string for an error code.
     * 
     * @param code Error code from any native function
     * @return Error description string
     */
    public static native String errorString(int code);

    public boolean isLibraryAvailable() {
        try {
            System.loadLibrary("cam_isp");
            return true;
        } catch (UnsatisfiedLinkError e) {
            return false;
        }
    }

    public static synchronized SoftispJni getInstance(Context context) {
        return new SoftispJni();
    }

    // ========================================================================
    // Constants
    // ========================================================================

    public static class SoftispError {
        public static final int OK = 0;
        public static final int ERR_INVALID_INPUT = 1;
        public static final int ERR_COMPOSE_FAILED = 2;
        public static final int ERR_CONVERT_FAILED = 3;
        public static final int ERR_UNKNOWN_PROFILE = 4;
    }

    public static class OpsetMode {
        public static final int PRIMITIVE = 0;  // Standard ONNX primitives
        public static final int CUSTOM = 1;     // Custom op types (for MNN conversion)
    }

    public static class DemosaicQuality {
        public static final int STANDARD = 0;   // Standard bilinear/edge-aware
        public static final int HQ_LINEAR = 1;  // High-quality linear interpolation
        public static final int EDGE = 2;       // Edge-directed demosaic
    }

    public static class OutputFormat {
        public static final int FLOAT_RGB = 0;       // f32 [0,1], 12 B/px
        public static final int FLOAT_BGRA = 1;      // f32 [0,255], 16 B/px
        public static final int PACKED_RGB = 2;      // INT32 packed, 2 B/px (2 pixels/INT32)
        public static final int BGRA = 3;            // u8 BGRA, 4 B/px
        public static final int RGBA = 4;            // u8 RGBA, 4 B/px
        public static final int ARGB = 5;            // u8 ARGB, 4 B/px
        public static final int ABGR = 6;            // u8 ABGR, 4 B/px
        public static final int RGB = 7;             // u8 RGB, 3 B/px
        public static final int BGR = 8;             // u8 BGR, 3 B/px
        public static final int FLOAT16_RGB = 9;     // f16 [0,255], 6 B/px
        public static final int FLOAT16_BGRA = 10;   // f16 [0,255], 8 B/px
    }

    public static class ProfileFlag {
        public static final String USE_UNPACK = "use_unpack";
        public static final String FORCE_INPUT16 = "force_input16";
        public static final String USE_FCS = "use_fcs";
        public static final String USE_LDCI = "use_ldci";
        public static final String USE_EE = "use_ee";
        public static final String USE_BAD_PIXEL = "use_bad_pixel";
        public static final String USE_LOCAL_CONTRAST = "use_local_contrast";
        public static final String USE_UNSHARP = "use_unsharp";
        public static final String USE_LSC = "use_lsc";
        public static final String USE_WARP = "use_warp";
        public static final String USE_HDR = "use_hdr";
        public static final String USE_ZONE_STATS = "use_zone_stats";
        public static final String USE_CHANNEL_MEANS = "use_channel_means";
        public static final String USE_TONE_STATS = "use_tone_stats";
        public static final String USE_HISTOGRAM = "use_histogram";
        public static final String USE_BILATERAL = "use_bilateral";
        public static final String USE_SATURATION = "use_saturation";
        public static final String USE_VIGNETTING = "use_vignetting";
        public static final String USE_COLORSPACE = "use_colorspace";
        public static final String USE_GAMMA = "use_gamma";
        public static final String USE_SHARPEN = "use_sharpen";
        public static final String USE_WAVELET_DENOISE = "use_wavelet_denoise";
        public static final String USE_AUTO_CONTRAST = "use_auto_contrast";
        public static final String USE_NORMALIZE = "use_normalize";
        public static final String USE_TILED_RENDERING = "use_tiled_rendering";
    }

    public static class BuiltinProfile {
        public static final String LITE = "LITE";
        public static final String MED = "MED";
        public static final String HEAVY = "HEAVY";
        public static final String PRO = "PRO";
        public static final String UNIFIED = "UNIFIED";
        public static final String HDR = "HDR";
        public static final String TEST = "TEST";
    }

    // ========================================================================
    // Camera Processor Interface
    // ========================================================================

    public interface CameraProcessor {
        void processFrame(byte[] data, int width, int height, int format);
        void onFrameProcessed(byte[] processedData, int width, int height, int format);
        void onError(String error);
    }
}