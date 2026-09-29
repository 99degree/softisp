# Package a Rust cdylib into an installable APK.
#
# The native side of com.softisp.camera lives in cam-rust/cam-camera-app; this
# Makefile only drives `cargo apk2`, which compiles the crate, generates
# AndroidManifest.xml from the crate's [package.metadata.android] table, and
# signs the result. There is no C or C++ in the build.
#
#   make            build cam-rust/cam-rust/target/release/apk/softisp-camera.apk
#   make push       install the APK on the connected device
#   make run        install, then launch the camera activity
#   make logs       tail the crash log the app writes to the app's log dir
#   make clean      remove build output
#
# Host requirements (all satisfied on this Termux device):
#   * cargo + cargo-apk2
#   * ANDROID_SDK_ROOT pointing at an SDK with platform android-34 + build-tools
#   * ANDROID_NDK_HOME pointing at an NDK
# On Termux the NDK's own clang/llvm-* binaries are x86_64 and cannot run here,
# so the NDK toolchain dir is shimmed with symlinks to the Termux equivalents
# and build-tools are shimmed the same way. `toolchain` does that automatically.

CARGO        ?= cargo
CARGO_APK2   ?= cargo-apk2
ADAPTER      := $(shell command -v $(CARGO_APK2) 2>/dev/null)

TARGET       ?= aarch64-linux-android
PROFILE      ?= release

WORKSPACE    := $(abspath ../cam-rust)
APK_DIR      := $(WORKSPACE)/target/$(PROFILE)/apk
APK_NAME     := softisp-camera
APK          := $(APK_DIR)/$(APK_NAME).apk

# `cargo apk2` resolves the NDK host tools through ANDROID_NDK_ROOT, and dexes
# the generated R.java with whatever JDK JAVA_HOME points at. Its bundled R8
# refuses class files above major version 55 (Java 11), and a JDK 17 javac
# still emits major 61, so the JDK is pinned to whatever is at most 17 here.
# This overrides any inherited JAVA_HOME, which on Termux defaults to 21.
NDK          ?= $(HOME)/android-sdk/ndk/29.0.14206865
SDK_ROOT     ?= $(HOME)/android-sdk
JVM_ROOT     := /data/data/com.termux/files/usr/lib/jvm
JAVA_HOME    := $(firstword $(wildcard $(JVM_ROOT)/java-11-openjdk $(JVM_ROOT)/java-17-openjdk))
ifeq ($(JAVA_HOME),)
  $(error No JDK 11 or 17 found under $(JVM_ROOT); cargo-apk2's R8 cannot dex newer class files)
endif
ANDROID_NDK_ROOT  ?= $(NDK)
export ANDROID_NDK_ROOT
export JAVA_HOME
export HOST = aarch64-unknown-linux-android

.PHONY: all build toolchain push run logs uninstall clean check-tests

all: build

build: toolchain
	@echo "==> cargo apk2 build --release --target $(TARGET)"
	$(CARGO) apk2 build --release --target $(TARGET) \
		--manifest-path $(WORKSPACE)/Cargo.toml -p cam-camera-app
	@echo "==> built $(APK)"
	@ls -l $(APK)

# Termux hosts are aarch64, so the NDK's prebuilt/linux-x86_64 binaries are the
# wrong architecture. ndk-build2 picks that directory from $HOST, so shim it with
# symlinks to the Termux LLVM tools, and do the same for build-tools, whose
# aapt/aapt2/zipalign are also x86_64. Everything here is idempotent.
toolchain:
	@BINDIR=$(NDK)/toolchains/llvm/prebuilt/linux-x86_64/bin; \
	for t in clang clang++ llvm-ar llvm-readelf llvm-strip llvm-objcopy llvm-objdump llvm-nm; do \
		if [ -e /data/data/com.termux/files/usr/bin/$$t ] && [ ! -L $$BINDIR/$$t ]; then \
			mv $$BINDIR/$$t $$BINDIR/$$t.ndk-x86_64 2>/dev/null || true; \
		fi; \
		if [ -e /data/data/com.termux/files/usr/bin/$$t ]; then \
			ln -sfn /data/data/com.termux/files/usr/bin/$$t $$BINDIR/$$t; \
		fi; \
	done; \
	ANDROIDDIR=$(NDK)/toolchains/llvm/prebuilt/android; \
	if [ -d $(NDK)/toolchains/llvm/prebuilt/linux-x86_64 ]; then \
		mkdir -p $$ANDROIDDIR; \
		for e in bin lib include sysroot AndroidVersion.txt BUILD_INFO; do \
			[ -e $(NDK)/toolchains/llvm/prebuilt/linux-x86_64/$$e ] && \
				ln -sfn ../linux-x86_64/$$e $$ANDROIDDIR/$$e; \
		done; \
	fi; \
	BT=$$SDK_ROOT/build-tools; \
	NEWBT=$$BT/34.0.0-termux; \
	if [ ! -d $$NEWBT ]; then cp -a $$BT/34.0.0 $$NEWBT; fi; \
	for t in aapt aapt2 zipalign apksigner; do \
		if [ -e /data/data/com.termux/files/usr/bin/$$t ]; then \
			ln -sfn /data/data/com.termux/files/usr/bin/$$t $$NEWBT/$$t; \
		fi; \
	done; \
	echo "==> toolchain shims ready"

push: build
	@echo "==> installing $(APK_NAME)"
	@mkdir -p /data/local/tmp
	cp $(APK) /data/local/tmp/$(APK_NAME).apk
	pm install -r /data/local/tmp/$(APK_NAME).apk || adb install -r /data/local/tmp/$(APK_NAME).apk

run: push
	am start -n com.softisp.camera/android.app.NativeActivity || \
		adb shell am start -n com.softisp.camera/android.app.NativeActivity

logs:
	@tail -f /sdcard/download/softisp-app.log 2>/dev/null || \
		tail -f $(HOME)/storage/shared/Download/softisp-app.log 2>/dev/null || \
		echo "no crash log found yet"

uninstall:
	pm uninstall com.softisp.camera || adb uninstall com.softisp.camera

check-tests:
	cd $(WORKSPACE) && cargo test -p cam-camera-app --lib

clean:
	cd $(WORKSPACE) && cargo clean -p cam-camera-app
	rm -f /data/local/tmp/$(APK_NAME).apk
