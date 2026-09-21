package com.softisp.camera;

import android.os.Bundle;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MainActivity";
    
    private CameraFrameProcessorView previewView;
    private CameraProcessor compositeProcessor;
    private List<CameraProcessor> activeProcessors;
    private Button enableCameraButton;
    private Button toggleProcessingButton;
    private FrameLayout previewContainer;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        
        // Initialize components
        initUIComponents();
        setupFrameProcessorChain();
        
        Toast.makeText(this, "SoftISP Camera App initialized", Toast.LENGTH_SHORT).show();
    }
    
    private void initUIComponents() {
        previewContainer = findViewById(R.id.preview_container);
        enableCameraButton = findViewById(R.id.enable_camera_button);
        toggleProcessingButton = findViewById(R.id.toggle_processing_button);
        
        // Create preview view and add to container
        previewView = new CameraFrameProcessorView(this);
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            ViewGroup.LayoutParams.MATCH_PARENT);
        previewContainer.addView(previewView, params);
        
        // Set button click listeners
        enableCameraButton.setOnClickListener(v -> toggleCameraProcessing());
        toggleProcessingButton.setOnClickListener(v -> toggleProcessing());
    }
    
    private void setupFrameProcessorChain() {
        // Create a chain of processors
        activeProcessors = new ArrayList<>();
        
        // Create a composite processor that chains multiple processors
        compositeProcessor = new CameraProcessor() {
            @Override
            public void processFrame(byte[] data, int width, int height, int format) {
                // Process through each processor in sequence
                for (CameraProcessor processor : activeProcessors) {
                    processor.processFrame(data, width, height, format);
                }
            }
            
            @Override
            public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
                // Callback when frame processing is complete
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(MainActivity.this, "Error: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        };
        
        previewView.setFrameProcessor(compositeProcessor);
    }
    
    private void toggleCameraProcessing() {
        if (previewView != null) {
            // Initialize the softisp JNI engine with default settings
            SoftispCameraApplication app = SoftispCameraApplication.getInstance();
            // app.initializeSoftispEngine("auto", 1023.0f, 0); // Method doesn't exist
            
            // Enable camera processing
            if (enableCameraButton.getText().equals("Enable Camera")) {
                enableCameraButton.setText("Disable Camera");
                // In production, this would start the Camera2 API and call the JNI layer
                Toast.makeText(this, "Camera enabled", Toast.LENGTH_SHORT).show();
            } else {
                enableCameraButton.setText("Enable Camera");
                // In production, this would stop the camera
                Toast.makeText(this, "Camera disabled", Toast.LENGTH_SHORT).show();
            }
        }
    }
    
    private void toggleProcessing() {
        if (activeProcessors.isEmpty()) {
            // Add default processors
            activeProcessors.add(createDebugProcessor());
            activeProcessors.add(createPerformanceProcessor());
            toggleProcessingButton.setText("Remove Processors");
            Toast.makeText(this, "Processors enabled", Toast.LENGTH_SHORT).show();
        } else {
            // Remove processors
            activeProcessors.clear();
            toggleProcessingButton.setText("Add Processors");
            Toast.makeText(this, "Processors disabled", Toast.LENGTH_SHORT).show();
        }
    }
    
    private CameraProcessor createDebugProcessor() {
        return new CameraProcessor() {
            @Override
            public void processFrame(byte[] data, int width, int height, int format) {
                // Debug processing - in production this would call the JNI layer
                runOnUiThread(() -> {
                    Toast.makeText(MainActivity.this,
                        "Debug: Processing " + width + "x" + height + " frame",
                        Toast.LENGTH_SHORT).show();
                });
            }
            
            @Override
            public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
                // Callback when frame processing is complete
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(MainActivity.this, "Error: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        };
    }
    
    private CameraProcessor createPerformanceProcessor() {
        return new CameraProcessor() {
            @Override
            public void processFrame(byte[] data, int width, int height, int format) {
                // Performance processing - in production this would call the JNI layer
                // For demonstration, we'll just log
                android.util.Log.i(TAG, "Performance: Processing " + width + "x" + height + " frame");
            }
            
            @Override
            public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
                // Callback when frame processing is complete
            }
            
            @Override
            public void onError(String error) {
                runOnUiThread(() -> {
                    Toast.makeText(MainActivity.this, "Error: " + error, Toast.LENGTH_SHORT).show();
                });
            }
        };
    }
    
    @Override
    protected void onDestroy() {
        super.onDestroy();
        activeProcessors.clear();
        compositeProcessor = null;
        previewView = null;
    }
}