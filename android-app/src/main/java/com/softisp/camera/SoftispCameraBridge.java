package com.softisp.camera;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import androidx.annotation.NonNull;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

/**
 * Android Camera2 bridge that integrates with softisp JNI for ISP processing.
 * 
 * This class handles:
 * - Camera device management
 * - Frame capture and delivery to softisp JNI
 * - Model composition and loading
 * - Buffer management for zero-copy where possible
 */
public class SoftispCameraBridge {
    private static final String TAG = "SoftispCameraBridge";
    
    // Camera parameters
    private static final int MAX_IMAGES = 2;
    private static final int IMAGE_FORMAT = ImageFormat.YUV_420_888;
    
    private final Context context;
    private final CameraManager cameraManager;
    private final String cameraId;
    private final CameraProcessor frameProcessor;
    private final HandlerThread cameraThread;
    private final Handler cameraHandler;
    private final Semaphore cameraLock = new Semaphore(1);
    
    private CameraDevice cameraDevice;
    private ImageReader imageReader;
    private CaptureRequest.Builder captureRequestBuilder;
    private Size previewSize;
    private int sensorOrientation;
    private boolean isProcessing;
    
    // Softisp model handles
    private byte[] currentOnnxModel;
    private byte[] currentMnnModel;
    private long customProfileHandle = 0;
    
    public interface CameraProcessor {
        void onFrameProcessed(byte[] processedData, int width, int height, int format, long timestamp);
        void onError(String error);
        void onModelComposed(String modelType, int width, int height, int modelSize);
    }
    
    public SoftispCameraBridge(Context context, String cameraId, CameraProcessor processor) {
        this.context = context.getApplicationContext();
        this.cameraId = cameraId;
        this.frameProcessor = processor;
        this.cameraManager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
        
        // Create dedicated thread for camera operations
        this.cameraThread = new HandlerThread("SoftispCameraThread");
        this.cameraThread.start();
        this.cameraHandler = new Handler(cameraThread.getLooper());
        
        initializeCamera();
    }
    
    private void initializeCamera() {
        cameraHandler.post(() -> {
            try {
                CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(cameraId);
                
                // Get sensor orientation
                Integer orientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
                sensorOrientation = (orientation != null) ? orientation : 0;
                
                // Get available stream configurations
                StreamConfigurationMap streamMap = characteristics.get(
                    CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
                
                if (streamMap == null) {
                    throw new IllegalStateException("No stream configuration map for camera " + cameraId);
                }
                
                // Choose optimal preview size (prefer 1920x1080 or closest)
                Size[] outputSizes = streamMap.getOutputSizes(ImageReader.class);
                previewSize = chooseOptimalSize(outputSizes, 1920, 1080);
                
                if (previewSize == null) {
                    previewSize = outputSizes[0];
                }
                
                Log.i(TAG, "Selected preview size: " + previewSize.getWidth() + "x" + previewSize.getHeight());
                
                // Create ImageReader for frame processing
                imageReader = ImageReader.newInstance(
                    previewSize.getWidth(),
                    previewSize.getHeight(),
                    IMAGE_FORMAT,
                    MAX_IMAGES);
                imageReader.setOnImageAvailableListener(onImageAvailableListener, cameraHandler);
                
                // Pre-compose the default model
                composeDefaultModel();
                
            } catch (CameraAccessException e) {
                Log.e(TAG, "Failed to initialize camera", e);
                if (frameProcessor != null) {
                    frameProcessor.onError("Camera init failed: " + e.getMessage());
                }
            }
        });
    }
    
    private Size chooseOptimalSize(Size[] sizes, int width, int height) {
        if (sizes == null || sizes.length == 0) return null;
        
        List<Size> validSizes = Arrays.asList(sizes);
        validSizes.sort((s1, s2) -> {
            int area1 = s1.getWidth() * s1.getHeight();
            int area2 = s2.getWidth() * s2.getHeight();
            return Integer.compare(area2, area1); // Larger first
        });
        
        // Choose the first size that is at least the requested size
        for (Size size : validSizes) {
            if (size.getWidth() >= width && size.getHeight() >= height) {
                return size;
            }
        }
        
        // If no size meets the requirement, return the largest size
        return validSizes.get(0);
    }
    
    private void composeDefaultModel() {
        // Compose HEAVY profile model for default resolution
        // This runs on the camera thread to avoid blocking UI
        cameraHandler.post(() -> {
            try {
                String profile = SoftispJni.BuiltinProfile.HEAVY;
                currentOnnxModel = SoftispJni.composeOnnx(
                    previewSize.getWidth(), previewSize.getHeight(), profile);
                
                if (currentOnnxModel.length > 0) {
                    Log.i(TAG, "Composed ONNX model: " + currentOnnxModel.length + " bytes");
                    if (frameProcessor != null) {
                        frameProcessor.onModelComposed("ONNX", previewSize.getWidth(), 
                            previewSize.getHeight(), currentOnnxModel.length);
                    }
                    
                } else {
                    Log.w(TAG, "Failed to compose ONNX model");
                }
            } catch (Exception e) {
                Log.e(TAG, "Error composing model", e);
            }
        });
    }
    
    /**
     * Start camera preview and frame processing.
     * 
     * @param surface Surface for preview display
     */
    public void startPreview(Surface surface) {
        cameraHandler.post(() -> {
            if (cameraDevice != null) {
                Log.w(TAG, "Camera already opened");
                return;
            }

            try {
                try {
                    cameraLock.acquire();
                } catch (InterruptedException e) {
                    Log.w(TAG, "Camera lock interrupted", e);
                    Thread.currentThread().interrupt();
                    return;
                }

                cameraManager.openCamera(cameraId, new CameraDevice.StateCallback() {
                    @Override
                    public void onOpened(@NonNull CameraDevice camera) {
                        cameraLock.release();
                        cameraDevice = camera;
                        Log.i(TAG, "Camera opened successfully — preview visible");
                        // Preview visible: session set on preview surface
                        createCaptureSession(surface);
                    }
                    
                    @Override
                    public void onDisconnected(@NonNull CameraDevice camera) {
                        cameraLock.release();
                        camera.close();
                        cameraDevice = null;
                        Log.i(TAG, "Camera disconnected");
                    }
                    
                    @Override
                    public void onError(@NonNull CameraDevice camera, int error) {
                        cameraLock.release();
                        camera.close();
                        cameraDevice = null;
                        Log.e(TAG, "Camera error: " + error);
                        if (frameProcessor != null) {
                            frameProcessor.onError("Camera error: " + error);
                        }
                    }
                }, cameraHandler);
                
            } catch (CameraAccessException e) {
                cameraLock.release();
                Log.e(TAG, "Failed to open camera", e);
                if (frameProcessor != null) {
                    frameProcessor.onError("Failed to open camera: " + e.getMessage());
                }
            }
        });
    }
    
    private void createCaptureSession(Surface surface) {
        if (cameraDevice == null) return;
        
        try {
            List<Surface> surfaces = Arrays.asList(surface, imageReader.getSurface());
            
            cameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull android.hardware.camera2.CameraCaptureSession session) {
                    try {
                        // Create capture request for preview + image reader
                        captureRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                        captureRequestBuilder.addTarget(surface);
                        captureRequestBuilder.addTarget(imageReader.getSurface());
                        
                        // Set auto-exposure and auto-white-balance
                        captureRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE, 
                            CaptureRequest.CONTROL_AE_MODE_ON_AUTO_FLASH);
                        captureRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE,
                            CaptureRequest.CONTROL_AWB_MODE_AUTO);
                        
                        // Start repeating capture
                        session.setRepeatingRequest(captureRequestBuilder.build(),
                            new CameraCaptureSession.CaptureCallback() {
                                @Override
                                public void onCaptureCompleted(@NonNull CameraCaptureSession session,
                                                             @NonNull CaptureRequest request,
                                                             @NonNull TotalCaptureResult result) {
                                    // Capture completed - stats available in result
                                }
                            }, cameraHandler);
                        
                        Log.i(TAG, "Capture session configured and started");
                        isProcessing = true;
                        
                    } catch (CameraAccessException e) {
                        Log.e(TAG, "Failed to configure capture session", e);
                    }
                }
                
                @Override
                public void onConfigureFailed(@NonNull android.hardware.camera2.CameraCaptureSession session) {
                    Log.e(TAG, "Failed to configure capture session");
                }
            }, cameraHandler);
            
        } catch (CameraAccessException e) {
            Log.e(TAG, "Failed to create capture session", e);
        }
    }
    
    /**
     * Stop camera preview and release resources.
     */
    public void stopPreview() {
        cameraHandler.post(() -> {
            isProcessing = false;
            
            if (cameraDevice != null) {
                try {
                    cameraDevice.close();
                } catch (Exception e) {
                    Log.w(TAG, "Error closing camera", e);
                }
                cameraDevice = null;
            }
            
            if (imageReader != null) {
                imageReader.close();
                imageReader = null;
            }
            
            Log.i(TAG, "Camera preview stopped");
        });
    }
    
    /**
     * Switch to a different camera.
     */
    public void switchCamera(String newCameraId) {
        stopPreview();
        // Note: In a real app, you'd recreate the bridge with the new camera ID
    }
    
    /**
     * Set a custom pipeline profile for model composition.
     * 
     * @param profileHandle Handle from SoftispJni.profileCreate()
     */
    public void setCustomProfile(long profileHandle) {
        this.customProfileHandle = profileHandle;
        recomposeModel();
    }
    
    /**
     * Recompose the model with current settings.
     */
    private void recomposeModel() {
        if (previewSize == null) return;
        
        cameraHandler.post(() -> {
            if (customProfileHandle != 0) {
                currentOnnxModel = SoftispJni.composeOnnxWithProfile(
                    previewSize.getWidth(), previewSize.getHeight(), customProfileHandle);
            } else {
                currentOnnxModel = SoftispJni.composeOnnx(
                    previewSize.getWidth(), previewSize.getHeight(), 
                    SoftispJni.BuiltinProfile.HEAVY);
            }
            
            if (currentOnnxModel.length > 0) {
                if (frameProcessor != null) {
                    frameProcessor.onModelComposed("ONNX", previewSize.getWidth(), 
                        previewSize.getHeight(), currentOnnxModel.length);
                }
            }
        });
    }
    
    /**
     * Get the currently loaded ONNX model.
     */
    public byte[] getOnnxModel() {
        return currentOnnxModel;
    }
    
    /**
     * Get the currently loaded MNN model.
     */
    public byte[] getMnnModel() {
        return currentMnnModel;
    }
    
    /**
     * Get the preview size.
     */
    public Size getPreviewSize() {
        return previewSize;
    }
    
    /**
     * Check if camera is currently processing frames.
     */
    public boolean isProcessing() {
        return isProcessing;
    }
    
    /**
     * Release all resources.
     */
    public void release() {
        stopPreview();
        
        if (customProfileHandle != 0) {
            SoftispJni.profileFree(customProfileHandle);
            customProfileHandle = 0;
        }
        
        cameraHandler.removeCallbacksAndMessages(null);
        cameraThread.quitSafely();
        
        try {
            cameraThread.join(1000);
        } catch (InterruptedException e) {
            Log.w(TAG, "Camera thread join interrupted", e);
        }
        
        Log.i(TAG, "SoftispCameraBridge released");
    }
    
    // ========================================================================
    // Image processing callback
    // ========================================================================
    
    private final ImageReader.OnImageAvailableListener onImageAvailableListener = 
        new ImageReader.OnImageAvailableListener() {
            @Override
            public void onImageAvailable(ImageReader reader) {
                if (!isProcessing || frameProcessor == null) return;
                
                Image image = reader.acquireLatestImage();
                if (image == null) return;
                
                try {
                    // Convert YUV_420_888 to raw Bayer data for softisp
                    // Note: softisp expects raw Bayer (INT16) input
                    // For YUV input, we'd need a conversion step or use a different pipeline
                    
                    // For now, we'll extract the Y plane as grayscale and 
                    // use it as a proxy - in production this would need proper
                    // raw Bayer capture (typically via PRIVATE format or raw stream)
                    
                    Image.Plane[] planes = image.getPlanes();
                    if (planes.length == 0) return;
                    
                    // Get Y plane (luminance)
                    Image.Plane yPlane = planes[0];
                    ByteBuffer yBuffer = yPlane.getBuffer();
                    int width = image.getWidth();
                    int height = image.getHeight();
                    int rowStride = yPlane.getRowStride();
                    int pixelStride = yPlane.getPixelStride();
                    
                    // Create byte array for the Y plane
                    byte[] yData = new byte[width * height];
                    yBuffer.rewind();
                    
                    if (pixelStride == 1 && rowStride == width) {
                        yBuffer.get(yData);
                    } else {
                        // Handle strided layout
                        byte[] rowData = new byte[rowStride];
                        for (int y = 0; y < height; y++) {
                            yBuffer.position(y * rowStride);
                            yBuffer.get(rowData, 0, Math.min(rowStride, yBuffer.remaining()));
                            System.arraycopy(rowData, 0, yData, y * width, width);
                        }
                    }
                    
                    // Process through softisp (requires Bayer input)
                    // For YUV input, we need to either:
                    // 1. Convert YUV to Bayer (not trivial)
                    // 2. Use a YUV-compatible pipeline profile
                    // 3. Capture in RAW format instead
                    
                    // For this demo, we'll pass the Y plane as-is and let
                    // the softisp pipeline handle it (it expects INT16 Bayer)
                    // In production, use CameraDevice.TEMPLATE_STILL_CAPTURE with RAW target
                    
                    long timestamp = image.getTimestamp();
                    
                    // Call frame processor with the data
                    // Note: This is a simplified path - real implementation would
                    // use the MNN runtime to run inference on the composed model
                    frameProcessor.onFrameProcessed(yData, width, height, 
                        ImageFormat.YUV_420_888, timestamp);
                    
                } finally {
                    image.close();
                }
            }
        };
}