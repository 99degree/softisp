package com.softisp.camera;
import android.widget.Toast;
import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.Handler;
import android.os.Looper;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
public class SoftispDebugProcessor implements CameraProcessor {
    private static final String TAG = "SoftispDebugProcessor";
    
    private Context context;
    private Handler uiHandler;
    private Paint debugPaint;
    private Bitmap debugIcon;
    
    public SoftispDebugProcessor(Context context) {
        this.context = context;
        this.uiHandler = new Handler(Looper.getMainLooper());
        initializeDebugGraphics(context);
    }
    
    private void initializeDebugGraphics(Context context) {
        // Initialize debug visualization graphics
        debugPaint = new Paint();
        debugPaint.setColor(Color.RED);
        debugPaint.setStrokeWidth(2.0f);
        debugPaint.setAntiAlias(true);
        
        // Try to load debug icon from assets
        try {
            AssetManager assetManager = context.getAssets();
            InputStream inputStream = assetManager.open("icons/debug_icon.png");
            debugIcon = BitmapFactory.decodeStream(inputStream);
            inputStream.close();
        } catch (IOException e) {
            // If no icon available, create a simple one
            debugIcon = Bitmap.createBitmap(32, 32, Bitmap.Config.ARGB_8888);
            Canvas canvas = new Canvas(debugIcon);
            Paint redPaint = new Paint(); redPaint.setColor(Color.RED); canvas.drawRect(0, 0, 32, 32, redPaint);
        }
    }
    
    @Override
    public void processFrame(byte[] data, int width, int height, int format) {
        // Process and visualize the frame for debugging
        uiHandler.post(() -> {
            // In production, this would call the JNI layer to process the frame
            // For debugging, we'll just show a simple visualization
            Bitmap bitmap = visualizeDebugData(data, width, height, format);
            
            // Display debug information
            String debugInfo = String.format("Debug: %dx%d %s", width, height, getFormatString(format));
            Toast.makeText(context, debugInfo, Toast.LENGTH_SHORT).show();
        });
    }
    
    @Override
    public void onFrameProcessed(byte[] processedData, int width, int height, int format) {
        // Callback when frame processing is complete
        uiHandler.post(() -> {
            Toast.makeText(context, "Frame processed successfully", Toast.LENGTH_SHORT).show();
        });
    }
    
    @Override
    public void onError(String error) {
        // Handle processing errors
        uiHandler.post(() -> {
            Toast.makeText(context, "Error: " + error, Toast.LENGTH_LONG).show();
        });
    }
    
    private Bitmap visualizeDebugData(byte[] data, int width, int height, int format) {
        // Create a debug visualization of the frame data
        Bitmap bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);
        
        // Draw debug overlays based on format
        switch (format) {
            case 0: // YUV
                drawYuvDebugOverlay(canvas, data, width, height);
                break;
            case 1: // RGBA
                drawRgbDebugOverlay(canvas, data, width, height);
                break;
            case 2: // RAW Bayer
                drawBayerDebugOverlay(canvas, data, width, height);
                break;
            default:
                drawGenericDebugOverlay(canvas, data, width, height, format);
                break;
        }
        
        // Draw debug icon and info
        drawDebugInfo(canvas, width, height);
        
        return bitmap;
    }
    
    private void drawYuvDebugOverlay(Canvas canvas, byte[] data, int width, int height) {
        // Draw YUV debug overlay
        Paint yPaint = new Paint();
        yPaint.setColor(Color.GREEN);
        yPaint.setAlpha(128);
        
        // Draw Y channel histogram
        int[] histogram = calculateHistogram(data, width, height, 0);
        drawHistogram(canvas, histogram, Color.GREEN, width, height);
        
        // Draw U channel average
        float uAvg = calculateChannelAverage(data, width, height, 1);
        drawChannelValue(canvas, uAvg, Color.GREEN, "U Avg: " + String.format("%.2f", uAvg));
    }
    
    private void drawRgbDebugOverlay(Canvas canvas, byte[] data, int width, int height) {
        // Draw RGB debug overlay
        int redPixels = countColorPixels(data, width, height, 0, 0, 0);
        int greenPixels = countColorPixels(data, width, height, 0, 1, 0);
        int bluePixels = countColorPixels(data, width, height, 0, 2, 0);
        
        float redPercent = (redPixels * 100.0f) / (width * height);
        float greenPercent = (greenPixels * 100.0f) / (width * height);
        float bluePercent = (bluePixels * 100.0f) / (width * height);
        
        // Draw color bars
        drawColorBar(canvas, redPercent, Color.RED, "R: " + String.format("%.1f%%", redPercent));
        drawColorBar(canvas, greenPercent, Color.GREEN, "G: " + String.format("%.1f%%", greenPercent));
        drawColorBar(canvas, bluePercent, Color.BLUE, "B: " + String.format("%.1f%%", bluePercent));
    }
    
    private void drawBayerDebugOverlay(Canvas canvas, byte[] data, int width, int height) {
        // Draw Bayer pattern debug overlay
        Paint bayerPaint = new Paint();
        bayerPaint.setColor(Color.YELLOW);
        bayerPaint.setAlpha(128);
        
        // Simple Bayer pattern visualization
        for (int y = 0; y < height; y += 2) {
            for (int x = 0; x < width; x += 2) {
                int index = (y * width + x) * 2;
                if (index + 1 < data.length) {
                    int bayerValue = ((data[index] & 0xFF) + (data[index + 1] & 0xFF)) / 2;
                    int normalizedValue = (bayerValue * 255) / 1023; // Assuming 10-bit Bayer
                    int color = Color.HSVToColor(255, new float[]{normalizedValue / 255.0f * 360, 1, 1});
                    Paint pt = new Paint(); pt.setColor(color); canvas.drawPoint(x, y, pt);
                }
            }
        }
        
        // Draw Bayer pattern legend
        drawBayerPatternLegend(canvas, width, height);
    }
    
    private void drawGenericDebugOverlay(Canvas canvas, byte[] data, int width, int height, int format) {
        // Draw generic debug overlay
        Paint genericPaint = new Paint();
        genericPaint.setColor(Color.CYAN);
        genericPaint.setAlpha(128);
        
        // Draw frame statistics
        int frameSize = data.length;
        float frameSizeMB = frameSize / (1024.0f * 1024.0f);
        drawText(canvas, "Frame Size: " + String.format("%.2f MB", frameSizeMB), 10, 30);
        drawText(canvas, "Resolution: " + width + "x" + height, 10, 60);
        drawText(canvas, "Format: " + getFormatString(format), 10, 90);
    }
    
    private void drawDebugInfo(Canvas canvas, int width, int height) {
        // Draw debug icon and information
        if (debugIcon != null) {
            int iconSize = 64;
            int iconX = width - iconSize - 10;
            int iconY = height - iconSize - 10;
            canvas.drawBitmap(debugIcon, iconX, iconY, null);
        }
        
        drawText(canvas, "SoftISP Debug", width - 200, height - 40);
        drawText(canvas, "JNI Integration", width - 150, height - 20);
    }
    
    private void drawHistogram(Canvas canvas, int[] histogram, int color, int width, int height) {
        // Draw histogram as a simple graph
        int graphWidth = width / 2;
        int graphHeight = 100;
        int graphX = width / 2;
        int graphY = height - graphHeight - 50;
        
        float maxValue = getMaxHistogramValue(histogram);
        if (maxValue > 0) {
            for (int i = 0; i < histogram.length && i < graphWidth; i++) {
                float barHeight = (histogram[i] / maxValue) * graphHeight;
                int barX = graphX + i;
                int barY = graphY - (int) barHeight;
                canvas.drawRect(barX, barY, barX + 2, graphY, new Paint(color));
            }
        }
    }
    
    private void drawColorBar(Canvas canvas, float percentage, int color, String label) {
        int barWidth = (int) (percentage * 5);
        int barHeight = 20;
        int x = 10;
        int y = canvas.getHeight() - 60;
        
        // Draw background
        canvas.drawRect(x, y, x + 100, y + barHeight, new Paint(Color.GRAY));
        
        // Draw colored bar
        canvas.drawRect(x, y, x + barWidth, y + barHeight, new Paint(color));
        
        // Draw label
        drawText(canvas, label, x + 110, y + 15);
    }
    
    private void drawChannelValue(Canvas canvas, float value, int color, String label) {
        int x = 10;
        int y = canvas.getHeight() - 30;
        
        Paint textPaint = new Paint();
        textPaint.setColor(color);
        canvas.drawText(label, x, y, textPaint);
    }
    
    private void drawText(Canvas canvas, String text, float x, float y) {
        Paint textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(14);
        canvas.drawText(text, x, y, textPaint);
    }
    
    private float calculateChannelAverage(byte[] data, int width, int height, int channel) {
        // Calculate average value for a specific channel
        long sum = 0;
        int count = 0;
        
        if (channel < 4) {
            for (int i = 0; i < data.length; i += 4) {
                sum += (data[i + channel] & 0xFF);
                count++;
            }
        }
        
        return count > 0 ? sum / (float) count : 0;
    }
    
    private int countColorPixels(byte[] data, int width, int height, int r, int g, int b) {
        // Count pixels matching specific RGB values
        int count = 0;
        
        for (int i = 0; i < data.length; i += 4) {
            int red = data[i] & 0xFF;
            int green = data[i + 1] & 0xFF;
            int blue = data[i + 2] & 0xFF;
            
            if (red == r * 255 && green == g * 255 && blue == b * 255) {
                count++;
            }
        }
        
        return count;
    }
    
    private int[] calculateHistogram(byte[] data, int width, int height, int channel) {
        // Calculate histogram for a specific channel
        int[] histogram = new int[256];
        
        if (channel < 4) {
            for (int i = 0; i < data.length; i += 4) {
                int value = (data[i + channel] & 0xFF);
                histogram[value]++;
            }
        }
        
        return histogram;
    }
    
    private float getMaxHistogramValue(int[] histogram) {
        float max = 0;
        for (int value : histogram) {
            if (value > max) {
                max = value;
            }
        }
        return max;
    }
    
    private void drawBayerPatternLegend(Canvas canvas, int width, int height) {
        // Draw Bayer pattern legend
        int legendX = width - 150;
        int legendY = height - 120;
        
        Paint legendPaint = new Paint();
        legendPaint.setColor(Color.BLACK);
        legendPaint.setTextSize(12);
        
        canvas.drawText("Bayer Pattern:", legendX, legendY, legendPaint);
        canvas.drawText("RGGB: Red at top-left", legendX, legendY + 20, legendPaint);
        canvas.drawText("BGGR: Blue at top-left", legendX, legendY + 40, legendPaint);
        canvas.drawText("GBRG: Green at top-left", legendX, legendY + 60, legendPaint);
        canvas.drawText("GBRG: Green at top-left", legendX, legendY + 80, legendPaint);
    }
    
    private String getFormatString(int format) {
        switch (format) {
            case 0: return "YUV_420_888";
            case 1: return "RGBA_8888";
            case 2: return "RAW10";
            case 3: return "RAW12";
            case 4: return "PRIVATE";
            default: return "UNKNOWN";
        }
    }
}