package com.softisp.camera;

import android.content.Context;
import android.util.Log;

public class SoftispCameraApplication extends android.app.Application {
    private static final String TAG = "SoftispCameraApp";
    private static SoftispCameraApplication instance;

    @Override
    public void onCreate() {
        super.onCreate();
        instance = this;
        SoftispJni.getInstance(this);
    }

    public static SoftispCameraApplication getInstance() {
        return instance;
    }
}