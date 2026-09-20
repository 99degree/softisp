package com.softisp.camera;

import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.widget.Button;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.common.util.concurrent.ListenableFuture;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class CameraActivity extends AppCompatActivity {
    private static final String TAG = "CameraActivity";
    private static final int CAMERA_PERMISSION_REQUEST_CODE = 100;

    private SoftispPreviewView previewView;
    private Button captureButton;
    private Button switchCameraButton;
    private SoftispJni.CameraProcessor frameProcessor;
    private ExecutorService cameraExecutor;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);
        // Request camera permission at runtime (fixes crash)
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) { ActivityCompat.requestPermissions(this, new String[]{android.Manifest.permission.CAMERA}, 100); }

        // Initialize executor service
        cameraExecutor = Executors.newSingleThreadExecutor();

        // Initialize frame processor
        frameProcessor = new SoftispJni.CameraProcessor() {
            @Override
            public void processFrame(byte[] data, int width, int height, int format) {
                // Process the frame on the camera executor
                cameraExecutor.execute(() -> {
                    // Call back to the main thread
                    runOnUiThread(() -> {
                        Toast.makeText(CameraActivity.this,
                            "Frame processed: " + width + "x" + height, Toast.LENGTH_SHORT).show();
                    });
                });
            }

            @Override
            public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
                // Callback when frame processing is complete
            }

            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(CameraActivity.this, "Error: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        };

        // Initialize views
        previewView = findViewById(R.id.preview_view);
        captureButton = findViewById(R.id.capture_button);
        // switchCameraButton = findViewById(R.id.switch_camera_button); // Not in layout

        // Set the frame processor on the preview view
        if (previewView != null) {
            previewView.setFrameProcessor(frameProcessor);
        }

        // Check camera permissions
        if (allPermissionsGranted()) {
            // Camera permission already granted, preview view will start when surface is ready
        } else {
            ActivityCompat.requestPermissions(this,
                new String[]{android.Manifest.permission.CAMERA},
                CAMERA_PERMISSION_REQUEST_CODE);
        }

        // Set up button listeners
        captureButton.setOnClickListener(v -> {
            // Take a photo (this would call the JNI layer)
            Toast.makeText(CameraActivity.this, "Capture not implemented", Toast.LENGTH_SHORT).show();
        });
    }

    private boolean allPermissionsGranted() {
        return ContextCompat.checkSelfPermission(this, android.Manifest.permission.CAMERA) ==
               PackageManager.PERMISSION_GRANTED;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE && grantResults.length > 0 &&
            grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            // Permission granted, preview view will start when surface is ready
        } else {
            Toast.makeText(this, "Camera permission required", Toast.LENGTH_LONG).show();
            finish();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (cameraExecutor != null) {
            cameraExecutor.shutdown();
        }
        // Note: previewView releases its own resources in its release() method,
        // but we don't call it here as the view's lifecycle is managed by the activity.
        // If needed, we could call previewView.release() but it's handled automatically.
    }
}