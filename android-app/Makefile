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

TARGET       ?= aarch64-linux-android
PROFILE      ?= release
# ISP backend. `mnn` selects the Vulkan MNN engine and bundles libMNN*.so plus
# the libvulkan.so ICD loader into the APK; `cpu` leaves the pipeline on the
# NEON/SIMD backend and produces a much smaller APK.
ENGINE       ?= mnn
FEATURE      := $(if $(filter cpu,$(ENGINE)),,mnn)

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

.PHONY: all build toolchain verify-manifest push run logs uninstall clean check-tests

all: build

build: toolchain
	@echo "==> cargo apk2 build --release --target $(TARGET) (engine: $(ENGINE))"
	$(CARGO) apk2 build --release --target $(TARGET) $(if $(FEATURE),--features $(FEATURE),) \
		--manifest-path $(WORKSPACE)/Cargo.toml -p cam-camera-app
	@echo "==> built $(APK)"
	@ls -l $(APK)
	@$(SDK_ROOT)/build-tools/34.0.0-termux/aapt list $(APK) 2>/dev/null | grep '\.so' || unzip -l $(APK) | grep '\.so'
	@$(MAKE) --no-print-directory verify-manifest

# cargo-apk2 1.4.2 serialises two `<application>` fields without the quick-xml
# `@` attribute prefix (ndk-build2 bug), so the manifest is supplied whole via
# `manifest_override`. This asserts the packaged result, because a manifest that
# parses but is missing `lib_name` produces an app that installs and then dies
# in `NativeActivity.onCreate` with "Unable to find native library main" — an
# app that looks installed but never starts.
#
# `lib_name` must sit on `<activity>`, not `<application>`: NativeActivity reads
# `getActivityInfo(...).metaData`, which never carries application-level entries.
# aapt's xmltree indents by nesting depth, so the placement is asserted by
# checking the entry appears after `<activity>` at a deeper indent.
verify-manifest:
	@AAPT=$(SDK_ROOT)/build-tools/34.0.0-termux/aapt; \
	$$AAPT dump xmltree $(APK) AndroidManifest.xml > /dev/null 2>&1 || { \
		echo "==> aapt unavailable; skipping manifest assertions"; exit 0; \
	}; \
	dump=$$($$AAPT dump xmltree $(APK) AndroidManifest.xml 2>/dev/null); \
	fail=0; \
	for pat in \
		'package="com.softisp.camera"' \
		'android.app.NativeActivity' \
		'android.intent.category.LAUNCHER' \
		'uses-sdk' \
		'minSdkVersion' \
		'targetSdkVersion' \
		'android.permission.CAMERA' \
		'android.permission.MANAGE_EXTERNAL_STORAGE'; do \
		echo "$$dump" | grep -q "$$pat" || { echo "MISSING: $$pat"; fail=1; }; \
	done; \
	act=$$(echo "$$dump" | awk '/E: activity/{found=1} found'); \
	if [ -z "$$act" ]; then \
		echo "MISSING: <activity>"; fail=1; \
	elif ! echo "$$act" | grep -q 'android.app.lib_name'; then \
		echo "MISSING: android.app.lib_name on <activity> (an <application>-level"; \
		echo "         entry is invisible to NativeActivity and yields libname=main)"; \
		fail=1; \
	elif ! echo "$$act" | grep -q 'cam_camera_app'; then \
		echo "MISSING: cam_camera_app value for android.app.lib_name"; fail=1; \
	fi; \
	if [ $$fail -ne 0 ]; then echo "==> manifest check FAILED"; exit 1; fi; \
	echo "==> manifest OK"

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
	BT=$(SDK_ROOT)/build-tools; \
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

# The spec path first, then the app's external directory, which is where the
# log lands when the storage permission has not been granted. Termux can read
# both, so no adb round trip is needed.
LOG_PATHS := /sdcard/download/softisp-app.log \
             /storage/emulated/0/Android/data/com.softisp.camera/files/softisp-app.log

logs:
	@found=0; \
	for p in $(LOG_PATHS); do \
		if [ -f "$$p" ]; then echo "==> $$p"; tail -f "$$p"; found=1; break; fi; \
	done; \
	if [ $$found -eq 0 ]; then echo "no log found yet; checked: $(LOG_PATHS)"; fi

uninstall:
	pm uninstall com.softisp.camera || adb uninstall com.softisp.camera

check-tests:
	cd $(WORKSPACE) && cargo test -p cam-camera-app --lib

clean:
	cd $(WORKSPACE) && cargo clean -p cam-camera-app
	rm -f /data/local/tmp/$(APK_NAME).apk
