package com.softisp.camera;

import android.Manifest;
import android.app.Service;
import android.content.Intent;
import android.os.Binder;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.util.Log;

public class SoftispCameraService extends Service {
    private static final String TAG = "SoftispCameraService";
    private static final int MAX_BUFFER_SIZE = 64 * 1024 * 1024; // 64MB

    private SoftispCameraApplication application;
    private SoftispJni.CameraProcessor activeProcessor;
    private android.media.ImageReader imageReader;
    private Handler handler;

    public SoftispCameraService() {
        super();
    }

    @Override
    public void onCreate() {
        super.onCreate();
        application = SoftispCameraApplication.getInstance();
        handler = new Handler(Looper.getMainLooper());

        Log.i(TAG, "Softisp Camera Service created");
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (intent != null) {
            String action = intent.getAction();
            if ("PROCESS_FRAME".equals(action)) {
                processFrameFromIntent(intent);
            }
        }
        return START_STICKY;
    }

    private void processFrameFromIntent(Intent intent) {
        byte[] data = intent.getByteArrayExtra("frame_data");
        int width = intent.getIntExtra("frame_width", 0);
        int height = intent.getIntExtra("frame_height", 0);
        int format = intent.getIntExtra("frame_format", 0);

        if (data != null && width > 0 && height > 0) {
            if (activeProcessor != null) {
                activeProcessor.processFrame(data, width, height, format);
            }
        }
    }

    public void setFrameProcessor(SoftispJni.CameraProcessor processor) {
        this.activeProcessor = processor;
    }

    public static class CameraProcessorBinder extends Binder {
        private final SoftispCameraService service;

        public CameraProcessorBinder(SoftispCameraService service) {
            this.service = service;
        }

        public SoftispCameraService getService() {
            return service;
        }
    }

    private final IBinder binder;

    {
        binder = new CameraProcessorBinder(this);
    }

    @Override
    public IBinder onBind(Intent intent) {
        return binder;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        return super.onUnbind(intent);
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (activeProcessor != null) {
            activeProcessor = null;
        }
        Log.i(TAG, "Softisp Camera Service destroyed");
    }
}