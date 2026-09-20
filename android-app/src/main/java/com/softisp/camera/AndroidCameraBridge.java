package com.softisp.camera;

import android.content.Context;

public class AndroidCameraBridge {
    private Context context;
    private String cameraId;
    private SoftispJni.CameraProcessor frameProcessor;

    public AndroidCameraBridge(Context context, String cameraId, SoftispJni.CameraProcessor processor) {
        this.context = context;
        this.cameraId = cameraId;
        this.frameProcessor = processor;
    }

    public void setFrameProcessor(SoftispJni.CameraProcessor processor) {
        this.frameProcessor = processor;
    }

    public void startPreview() {
        // To be implemented: start the frame processing pipeline
    }

    public void stopPreview() {
        // To be implemented: stop the frame processing pipeline
    }

    public void release() {
        this.frameProcessor = null;
    }
}