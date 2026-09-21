package com.softisp.camera;

import android.content.Context;
import android.util.AttributeSet;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.util.concurrent.CopyOnWriteArrayList;

public class CameraFrameProcessorView extends SurfaceView implements SurfaceHolder.Callback {
    private static final String TAG = "CameraFrameProcessorView";
    
    private SurfaceHolder surfaceHolder;
    private CameraProcessor frameProcessor;
    private SoftispCameraApplication application;
    private Context context;
    
    public CameraFrameProcessorView(Context context) {
        super(context);
        initView(context);
    }
    
    public CameraFrameProcessorView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initView(context);
    }
    
    public CameraFrameProcessorView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        initView(context);
    }
    
    private void initView(Context context) {
        this.context = context;
        surfaceHolder = getHolder();
        surfaceHolder.addCallback(this);
        surfaceHolder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
        
        application = SoftispCameraApplication.getInstance();
    }
    
    public void setFrameProcessor(CameraProcessor processor) {
        this.frameProcessor = processor;
    }
    
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        // Surface created - initialize camera processing
    }
    
    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        // Surface changed - adjust processing parameters
        if (frameProcessor != null) {
            // Notify the processor of the new dimensions
            // In production, this would call the JNI layer with the actual camera data
            byte[] dummyData = new byte[width * height * 4]; // Dummy data for demonstration
            frameProcessor.processFrame(dummyData, width, height, format);
        }
    }
    
    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        // Surface destroyed - cleanup resources
    }
    
    public void processCameraFrame(byte[] data, int width, int height, int format) {
        if (frameProcessor != null) {
            frameProcessor.processFrame(data, width, height, format);
        }
    }
}