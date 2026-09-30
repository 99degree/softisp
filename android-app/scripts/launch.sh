#!/data/data/com.termux/files/usr/bin/sh
# Launch com.softisp.camera through the platform `monkey` binary.
#
# `am start` cannot be used from Termux: the platform `am` refuses it
# ("package=com.android.shell does not belong to uid=…") and the Termux `am`
# wrapper is a no-op stub. `monkey` runs as the shell user and goes through
# ActivityTaskManager, which is allowed to start a launcher activity.
#
# It needs a real PATH because monkey is a shell wrapper around app_process.
export PATH=/system/bin:/system/xbin:/product/bin:/vendor/bin
export ANDROID_DATA=/data
export ANDROID_ROOT=/system
exec /system/bin/monkey -p com.softisp.camera -c android.intent.category.LAUNCHER 1
