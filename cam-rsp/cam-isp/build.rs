fn main() {
    // Emit rerun-if-changed for all env vars that affect the build
    println!("cargo:rerun-if-env-changed=MNN_DIR");
    println!("cargo:rerun-if-env-changed=MNN_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=MNN_LIB_DIR");
    println!("cargo:rerun-if-env-changed=MNN_CONVERT_DIR");
    println!("cargo:rerun-if-env-changed=ANDROID_NDK_HOME");

    #[cfg(feature = "ort")]
    link_onnxruntime();

    #[cfg(feature = "mnn")]
    link_mnn();

    #[cfg(feature = "mnn")]
    link_mnnconvert();

    // Link JNI library for Android with JNI feature
    #[cfg(all(target_os = "android", feature = "jni"))]
    link_jni();

    // Emit NDK linker flags for Android targets
    setup_ndk_linker();
}

/// Link JNI library for Android.
fn link_jni() {
    println!("cargo:warning=Linking JNI library");
    let abi_dir = abi_dir();
    let jni_src = Path::new("src/jni.c");
    if jni_src.exists() {
        println!("cargo:rerun-if-changed={}", jni_src.display());
        let out_dir = PathBuf::from(std::env::var("OUT_DIR").unwrap());
        let mut build = cc::Build::new();
        build
            .file(jni_src)
            // We are compiling C, not C++
            .flag_if_supported("-std=c99")
            .include("src"); // in case we have headers in src
        setup_cc_for_android(&mut build);
        build.compile("jni");

        let src = out_dir.join("libjni.a");
        let dst = abi_dir.join("libjni.a");
        if src.exists() {
            std::fs::copy(&src, &dst).ok();
            println!("cargo:rerun-if-changed={}", dst.display());
        }
    }

    println!("cargo:rustc-link-search=native={}", abi_dir.display());
    println!("cargo:rustc-link-lib=static=jni");
}