package com.softisp.camera;

public interface CameraProcessor {
    void processFrame(byte[] data, int width, int height, int format);
    void onFrameProcessed(byte[] processedData, int width, int height, int format);
    void onError(String error);
}
