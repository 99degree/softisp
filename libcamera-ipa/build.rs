fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    eprintln!("[libcamera-ipa] build: main entry for libcamera IPA plugin (target={target_os})");

    #[cfg(target_os = "android")]
    {
        // The prebuilt MNN shared objects reference versioned platform symbols
        // (`__android_log_print@LIBLOG`), while the NDK sysroot ships an
        // unversioned liblog stub — so lld refuses to leave them unresolved when
        // this crate links. The device's /system/lib64/liblog.so does define
        // LIBLOG, so the reference resolves at load time.
        println!("cargo:rustc-link-arg=-Wl,--allow-shlib-undefined");
        println!("cargo:rustc-link-arg=-Wl,-rpath,/system/lib64");
        println!("cargo:rustc-link-arg=-Wl,-rpath,/apex/com.android.runtime/lib64/bionic");
    }
}
