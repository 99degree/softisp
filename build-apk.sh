#!/data/data/com.termux/files/usr/bin/bash
# Rebuild the SoftISP camera APK with the pinned JDK 11 + NDK r29 toolchain.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"

cd "$(dirname "$0")/cam-rust"
cargo apk2 build -p cam-camera-app
