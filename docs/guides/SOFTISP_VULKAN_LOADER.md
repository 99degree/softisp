# SoftISP Vulkan loader

SoftISP packages its own `libvulkan.so` for the compute-only Vulkan path. The
loader is a generated full entry-point passthrough; it selects one of two
backends at load time:

- **Adreno**: load a hwvulkan HAL that exports `HMI` (`hw_module_t`), open the
  HAL device, and call its `vkGetInstanceProcAddr` directly. This bypasses the
  Android loader's ICD filtering and lets MNN/direct-Vulkan use the vendor
  driver for AHB/storage-image compute.
- **Other platforms**: `dlopen` the system `/system/lib64/libvulkan.so` and
  forward all entry points to it. The existing SIMD de-stride copy into a
  worker-owned BLOB AHB and Vulkan buffer/image remains unchanged.

## Source and generation

- Generated C++: `vulkan_loader/softisp_vulkan_loader.cpp`
- Generator: `scripts/gen_vulkan_loader.py`
- NDK source: `$ANDROID_NDK_ROOT/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/vulkan/{vulkan_core.h,vulkan_android.h}`

Regenerate with:

```bash
ANDROID_NDK_ROOT="$HOME/android-sdk/ndk/29.0.14206865" python3 scripts/gen_vulkan_loader.py
```

Do not hand-edit the generated file; change the generator and regenerate.

## Packaging

`anw/build.rs` compiles the generated source with NDK `clang++` into
`cam-rust/lib/aarch64-v8a/libvulkan.so` and copies it to
`cam-rust/lib/arm64-v8a/`. The shared object's SONAME is `libvulkan.so`, so the
core's `DT_NEEDED` resolves to the staged project loader. Both directories are
sources for cargo-apk2's `runtime_libs`, so the APK and shim-test staging both
receive the loader.

When the build host has `/vendor/lib64/hw/vulkan.adreno.so`, the build script
also stages it as `libsoftisp_vulkan_adreno.so` in both runtime-lib directories.
An app-private copy is required because the app's namespace cannot `dlopen` the
vendor path directly. The staged HAL is not committed to git; it is copied at
build time.

`build-apk.sh` verifies that the APK's `libvulkan.so` hash matches the build
output and, on Adreno build hosts, that the staged HAL hash matches too.
`push-test-libs.sh` pushes the project loader; `shim-debug.sh verify` hashes it
along with the core, `libc++_shared.so`, and all MNN runtime libraries.
`softisp-shim` preloads `libvulkan.so` before the core so hot-reload tests use
the project loader instead of Android's system loader. This requires a one-time
rebuild and user-side install of `target/debug/apk/softisp-test.apk` after
changing that dependency list.

## Selection and overrides

The loader checks in order:

1. `SOFTISP_VULKAN_PLATFORM` — set to a string containing `adreno` or `qcom`
   to force the Adreno HAL path, or `generic` to force the system loader.
2. Android properties `ro.hardware.vulkan`, `ro.hardware`, and
   `ro.board.platform` — any value containing `adreno` or `qcom` selects Adreno.
3. Non-Adreno platforms use the system loader.

On Adreno:

- `SOFTISP_VULKAN_DRIVER=<path>` overrides the HAL path.
- Otherwise the loader tries `libsoftisp_vulkan_adreno.so` from the app's
  native library directory, then `/vendor/lib64/hw/vulkan.adreno.so`.
- `SOFTISP_HWVULKAN_DEVICE_ID` overrides the HAL device id; the default is
  `vulkan`. Some Turnip builds require `vk0`.

Any HAL-open failure falls back to the system loader instead of aborting.

## Scope

This loader implements the full Vulkan 1.0/1.1 core and Android entry-point set
generated from the NDK headers, but it does **not** implement Android WSI or
swapchain translation. It is valid for compute/AHB probes, MNN Vulkan
inference, and the direct-Vulkan RAW16→BGRA path. General graphics/present use
must keep using the Android system loader.

The direct-Vulkan engine imports the camera AHB fd through
`VK_EXT_external_memory_dma_buf` when the selected driver exposes it, and keeps
the BLOB de-stride fallback otherwise. Built-in Adreno does not expose that
extension; Turnip does. See `docs/isp/20261128-raw16-fd-zero-copy.md`.

## Verification

```bash
cargo fmt --all -- --check
cargo clippy -p softisp-anw --all-features --no-deps -- -D warnings
cargo test -p cam-isp --lib --features "rectifier,direct_vulkan" --no-fail-fast
cargo test -p softisp-anw --lib --no-default-features --features direct_vulkan --no-fail-fast
DIRECT_VULKAN=1 ./build-test-libs.sh --core-only
./push-test-libs.sh
./shim-debug.sh reload
./shim-debug.sh verify
```

Device-only probes (ignored by default):

```bash
BIN=$(ls target/debug/build/cam-isp/*/out/test_direct_vulkan_image-* | head -1)
for t in raw16_ahb_explicit_vulkan_format_import \
         raw16_ahb_dmabuf_import \
         raw16_ahb_opaque_fd_import \
         raw16_ahb_fd_image_run \
         probe_and_run_ahb_storage_image; do
  LD_LIBRARY_PATH="$PWD/cam-rust/lib/arm64-v8a:/system/lib64:/vendor/lib64" \
    timeout 60 "$BIN" --ignored --nocapture --test-threads=1 "$t"
done
```
