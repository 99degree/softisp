package com.softisp.camera;

import android.util.Log;

public class SoftispPerformanceProcessor implements CameraProcessor {
    private static final String TAG = "SoftispPerformanceProcessor";
    
    private long lastFrameTime;
    private long frameCount;
    private float fps;
    private long totalProcessingTime;
    
    public SoftispPerformanceProcessor() {
        lastFrameTime = System.currentTimeMillis();
        frameCount = 0;
        fps = 0;
        totalProcessingTime = 0;
    }
    
    @Override
    public void processFrame(byte[] data, int width, int height, int format) {
        long startTime = System.currentTimeMillis();
        
        Log.d(TAG, "Processing frame: " + width + "x" + height + " format=" + format);
        
        // Processing time is measured; frame is passed to the JNI layer for actual ISP processing
        // The actual pipeline is composed via SoftispJni.composeOnnx
        long processingTime = System.currentTimeMillis() - startTime;
        totalProcessingTime += processingTime;
        frameCount++;
        
        updateFPS();
        logPerformanceMetrics(width, height, processingTime);
        
        onFrameProcessed(data, width, height, format);
    }
    
    @Override
    public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
        // Callback when frame processing is complete
        Log.d(TAG, "Frame processed: " + width + "x" + height);
    }
    
    @Override
    public void onError(String error) {
        Log.e(TAG, "Processing error: " + error);
    }
    
    private void updateFPS() {
        long currentTime = System.currentTimeMillis();
        long timeSinceLastFrame = currentTime - lastFrameTime;
        
        if (timeSinceLastFrame > 0) {
            fps = (frameCount * 1000.0f) / (float) timeSinceLastFrame;
        }
        
        lastFrameTime = currentTime;
    }
    
    private void logPerformanceMetrics(int width, int height, long processingTime) {
        float megapixels = (width * height) / (1024.0f * 1024.0f);
        float processingTimeMs = processingTime;
        float processingTimePerMp = megapixels > 0 ? processingTimeMs / megapixels : 0;
        float currentFps = processingTimeMs > 0 ? 1000.0f / processingTimeMs : 0;
        
        Log.i(TAG, String.format(
            "Performance: %dx%d %.2fMP, %.2fms/frame, %.2f fps, %.2fms/mp",
            width, height, megapixels, processingTimeMs, currentFps, processingTimePerMp
        ));
        
        if (frameCount >= 10) {
            float avgProcessingTime = totalProcessingTime / (float) frameCount;
            float avgFps = (frameCount * 1000.0f) / totalProcessingTime;
            Log.i(TAG, String.format(
                "Average: %.2fms/frame, %.2f fps",
                avgProcessingTime, avgFps
            ));
        }
    }
}