#!/data/data/com.termux/files/usr/bin/bash
# Rebuild the SoftISP camera APK with the pinned JDK 11 + NDK r29 toolchain.
#
# MNN is a subproject: its .so files are bundled into the APK, so build them
# first and copy them into cam-rust/lib/<abi>/ before the APK build. This
# guarantees the APK ships current MNN libraries rather than stale ones.
set -euo pipefail

export JAVA_HOME=/data/data/com.termux/files/usr/lib/jvm/java-11-openjdk
export ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865"
export ANDROID_HOME="$HOME/android-sdk"

# 1. Build MNN Vulkan libs and copy all .so into cam-rust/lib/aarch64-v8a/
./scripts/build_mnn_libs.sh arm64-v8a

# 2. Build the APK with cargo-apk2 (default debug profile).
cd "$(dirname "$0")/cam-rust"
cargo apk2 build -p cam-camera-app
