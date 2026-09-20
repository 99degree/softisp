package com.softisp.camera;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.ImageReader;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Surface;
import android.view.TextureView;

import java.util.Arrays;
import java.util.List;

/**
 * View that displays camera preview and processes frames via SoftISP JNI.
 * Uses TextureView for preview display and AndroidCameraBridge for camera2 integration.
 */
public class SoftispPreviewView extends TextureView implements TextureView.SurfaceTextureListener {
    private static final String TAG = "SoftispPreviewView";
    
    private Context context;
    private String cameraId;
    private AndroidCameraBridge androidCameraBridge;
    private Surface previewSurface;
    private boolean isPreviewStarted = false;
    
    public SoftispPreviewView(Context context) {
        super(context);
        this.context = context;
        init();
    }
    
    public SoftispPreviewView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.context = context;
        init();
    }
    
    public SoftispPreviewView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        this.context = context;
        init();
    }
    
    private void init() {
        setSurfaceTextureListener(this);
        initializeCameraId();
    }
    
    private void initializeCameraId() {
        CameraManager cameraManager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
        try {
            for (String id : cameraManager.getCameraIdList()) {
                CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(id);
                Integer lensFacing = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (lensFacing != null && lensFacing == CameraCharacteristics.LENS_FACING_BACK) {
                    cameraId = id;
                    Log.i(TAG, "Selected back camera: " + cameraId);
                    return;
                }
            }
        } catch (CameraAccessException e) {
            Log.e(TAG, "Failed to get camera list", e);
        }
        Log.w(TAG, "No back camera found, using first available");
        try {
            CameraManager manager = (CameraManager) context.getSystemService(Context.CAMERA_SERVICE);
            String[] ids = manager.getCameraIdList();
            if (ids.length > 0) {
                cameraId = ids[0];
            }
        } catch (CameraAccessException e) {
            Log.e(TAG, "Failed to get camera list", e);
        }
    }
    
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int width, int height) {
        Log.i(TAG, "Surface texture available: " + width + "x" + height);
        previewSurface = new Surface(surfaceTexture);
        startCameraPreview();
    }
    
    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int width, int height) {
        Log.i(TAG, "Surface texture size changed: " + width + "x" + height);
        // Reset the preview surface with new dimensions
        if (previewSurface != null) {
            previewSurface.release();
        }
        previewSurface = new Surface(surfaceTexture);
        if (androidCameraBridge != null && isPreviewStarted) {
            androidCameraBridge.stopPreview();
            startCameraPreview();
        }
    }
    
    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
        Log.i(TAG, "Surface texture destroyed");
        stopCameraPreview();
        if (previewSurface != null) {
            previewSurface.release();
            previewSurface = null;
        }
        return true;
    }
    
    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {
        // Called every frame - optional for performance monitoring
    }
    
    private void startCameraPreview() {
        if (cameraId == null || previewSurface == null) {
            Log.e(TAG, "Cannot start preview: cameraId=" + cameraId + ", previewSurface=" + (previewSurface != null ? "valid" : "null"));
            return;
        }
        
        try {
            // Initialize the camera bridge if not done
            if (androidCameraBridge == null) {
                androidCameraBridge = new AndroidCameraBridge(context, cameraId, null);
                // Note: Frame processor will be set by the activity via setFrameProcessor()
            }
            
            // Start the preview with our surface for display and the bridge's image reader for processing
            androidCameraBridge.startPreview(previewSurface);
            isPreviewStarted = true;
            Log.i(TAG, "Camera preview started");
        } catch (Exception e) {
            Log.e(TAG, "Failed to start camera preview", e);
        }
    }
    
    private void stopCameraPreview() {
        if (androidCameraBridge != null && isPreviewStarted) {
            androidCameraBridge.stopPreview();
            isPreviewStarted = false;
            Log.i(TAG, "Camera preview stopped");
        }
    }
    
    /**
     * Sets the frame processor for processing camera frames via SoftISP.
     * @param processor The camera processor to use for frame processing
     */
    public void setFrameProcessor(SoftispJni.CameraProcessor processor) {
        if (androidCameraBridge != null) {
            androidCameraBridge.setFrameProcessor(processor);
        }
    }
    
    /**
     * Releases all resources associated with the camera preview.
     * Should be called when the view is no longer needed.
     */
    public void release() {
        stopCameraPreview();
        if (androidCameraBridge != null) {
            androidCameraBridge.release();
            androidCameraBridge = null;
        }
    }
}