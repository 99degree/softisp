package com.softisp.camera;

import android.app.NativeActivity;
import android.content.Intent;
import android.os.Bundle;
import android.view.WindowManager;

/**
 * NativeActivity subclass used only by the test-only shim APK.
 *
 * The stock android.app.NativeActivity does not override onNewIntent(), so a
 * second {@code am start} on an existing singleTask activity leaves
 * {@code getIntent()} pointing at the original launcher intent. Calling
 * setIntent() here makes the native control watcher see each new command.
 *
 * The window flags also let a control intent bring the preview to the screen
 * while the device is locked; that path is still under study (see the shim
 * experiment doc).
 */
public class ShimActivity extends NativeActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        applyWakeFlags();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        applyWakeFlags();
    }

    private void applyWakeFlags() {
        getWindow().addFlags(
                WindowManager.LayoutParams.FLAG_SHOW_WHEN_LOCKED
                        | WindowManager.LayoutParams.FLAG_TURN_SCREEN_ON
                        | WindowManager.LayoutParams.FLAG_DISMISS_KEYGUARD
                        | WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }
}
