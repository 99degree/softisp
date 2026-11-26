//! APK launcher shim.
//!
//! Android starts this cdylib's `android_main` through the stock
//! `android.app.NativeActivity`. It locates the real application cdylib built
//! by `anw/` (`libsoftisp.so`) in the uploaded runtime directory, copies it and
//! its MNN/C++ dependencies into anonymous memfds, loads them with
//! `android_dlopen_ext`, and forwards the live `AndroidApp` to the core's
//! `android_main`. No library is copied into app-private storage, so the
//! uploaded files are the only runtime source.
//!
//! Every search root is mirrored to logcat and to
//! `/sdcard/download/softisp-app.log` (falling back to app external data),
//! including on failure, so a missing core is diagnosable either way.

use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::fs::{self, File, OpenOptions};
use std::io::{Seek, SeekFrom, Write};
use std::os::fd::AsRawFd;
#[cfg(target_os = "android")]
use std::os::fd::FromRawFd;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

use android_activity::AndroidApp;
#[cfg(not(target_os = "android"))]
use libc::dlopen;
use libc::{dlclose, dlerror, dlsym};
use log::{error, info, warn};

#[cfg(target_os = "android")]
use jni::{objects::JObject, refs::Global, sys::jobject, JavaVM};

const CORE_NAMES: &[&str] = &["libsoftisp.so", "libsoftisp_core.so", "libsoftisp_hot.so"];
const RUNTIME_DEPENDENCIES: &[&str] = &[
    "libc++_shared.so",
    "libsoftisp_cpp_shared.so",
    "libMNN.so",
    "libMNN_Express.so",
    "libMNN_Vulkan.so",
    "libMNNConvertDeps.so",
];
const RTLD_NOW: c_int = 2;
const RTLD_LOCAL: c_int = 0;
const PRIMARY_LOG: &str = "/sdcard/download/softisp-app.log";

#[cfg(target_os = "android")]
const ANDROID_DLEXT_USE_LIBRARY_FD: u64 = 0x10;
#[cfg(target_os = "android")]
const ANDROID_DLEXT_FORCE_LOAD: u64 = 0x40;

#[cfg(target_os = "android")]
#[repr(C)]
struct AndroidDlextInfo {
    flags: u64,
    reserved_addr: *mut c_void,
    reserved_size: usize,
    relro_fd: c_int,
    library_fd: c_int,
    library_fd_offset: i64,
    library_namespace: *mut c_void,
}

#[cfg(target_os = "android")]
unsafe extern "C" {
    fn android_dlopen_ext(
        filename: *const c_char,
        flags: c_int,
        info: *const AndroidDlextInfo,
    ) -> *mut c_void;
}
// Dedicated action so a launcher tap or another app's intent can never be
// mistaken for a debug command.
const INTENT_ACTION: &str = "com.softisp.camera.action.CONTROL";
const INTENT_EXTRA: &str = "softisp_cmd";

type AndroidMain = unsafe extern "Rust" fn(AndroidApp);

/// Streaming MD5 so every loaded library can be fingerprinted without pulling
/// the 129 MB core into a second in-memory copy.
mod md5 {
    use std::fs::File;
    use std::io::Read;
    use std::path::Path;

    const S: [u32; 64] = [
        7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5,
        9, 14, 20, 5, 9, 14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10,
        15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
    ];
    const K: [u32; 64] = [
        0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613,
        0xfd469501, 0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193,
        0xa679438e, 0x49b40821, 0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d,
        0x02441453, 0xd8a1e681, 0xe7d3fbc8, 0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
        0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a, 0xfffa3942, 0x8771f681, 0x6d9d6122,
        0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70, 0x289b7ec6, 0xeaa127fa,
        0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665, 0xf4292244,
        0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
        0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb,
        0xeb86d391,
    ];

    struct Digest {
        state: [u32; 4],
        len: u64,
        block: [u8; 64],
        used: usize,
    }

    impl Digest {
        fn new() -> Self {
            Self {
                state: [0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476],
                len: 0,
                block: [0; 64],
                used: 0,
            }
        }

        fn compress(&mut self, block: &[u8; 64]) {
            let mut m = [0u32; 16];
            for (i, word) in m.iter_mut().enumerate() {
                let j = i * 4;
                *word = u32::from_le_bytes([block[j], block[j + 1], block[j + 2], block[j + 3]]);
            }
            let [mut a, mut b, mut c, mut d] = self.state;
            for i in 0..64 {
                let (f, g) = match i / 16 {
                    0 => ((b & c) | (!b & d), i),
                    1 => ((d & b) | (!d & c), (5 * i + 1) % 16),
                    2 => (b ^ c ^ d, (3 * i + 5) % 16),
                    _ => (c ^ (b | !d), (7 * i) % 16),
                };
                let mixed = f
                    .wrapping_add(a)
                    .wrapping_add(K[i])
                    .wrapping_add(m[g])
                    .rotate_left(S[i]);
                a = d;
                d = c;
                c = b;
                b = b.wrapping_add(mixed);
            }
            self.state[0] = self.state[0].wrapping_add(a);
            self.state[1] = self.state[1].wrapping_add(b);
            self.state[2] = self.state[2].wrapping_add(c);
            self.state[3] = self.state[3].wrapping_add(d);
        }

        fn update(&mut self, mut input: &[u8]) {
            self.len = self.len.wrapping_add(input.len() as u64);
            if self.used > 0 {
                let take = (64 - self.used).min(input.len());
                self.block[self.used..self.used + take].copy_from_slice(&input[..take]);
                self.used += take;
                input = &input[take..];
                if self.used == 64 {
                    let block = self.block;
                    self.compress(&block);
                    self.used = 0;
                }
            }
            while input.len() >= 64 {
                let mut block = [0u8; 64];
                block.copy_from_slice(&input[..64]);
                self.compress(&block);
                input = &input[64..];
            }
            self.block[..input.len()].copy_from_slice(input);
            self.used = input.len();
        }

        fn finish(mut self) -> [u8; 16] {
            let bits = self.len.wrapping_mul(8).to_le_bytes();
            let mut tail = [0u8; 128];
            tail[..self.used].copy_from_slice(&self.block[..self.used]);
            tail[self.used] = 0x80;
            let pad_end = if self.used < 56 { 64 } else { 128 };
            tail[pad_end - 8..pad_end].copy_from_slice(&bits);
            self.compress(&tail[..64].try_into().unwrap());
            if pad_end == 128 {
                self.compress(&tail[64..].try_into().unwrap());
            }
            let mut out = [0u8; 16];
            for (i, word) in self.state.iter().enumerate() {
                out[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
            }
            out
        }
    }

    pub fn hex_of_file(path: &Path) -> std::io::Result<(String, u64)> {
        let mut file = File::open(path)?;
        let mut digest = Digest::new();
        let mut buf = [0u8; 64 * 1024];
        let mut bytes = 0u64;
        loop {
            let read = file.read(&mut buf)?;
            if read == 0 {
                break;
            }
            bytes += read as u64;
            digest.update(&buf[..read]);
        }
        let raw = digest.finish();
        let mut hex = String::with_capacity(32);
        for byte in raw {
            hex.push_str(&format!("{byte:02x}"));
        }
        Ok((hex, bytes))
    }

    #[cfg(test)]
    pub fn hex_of_bytes(input: &[u8]) -> String {
        let mut digest = Digest::new();
        digest.update(input);
        let raw = digest.finish();
        let mut hex = String::with_capacity(32);
        for byte in raw {
            hex.push_str(&format!("{byte:02x}"));
        }
        hex
    }
}

fn init_logcat() {
    #[cfg(target_os = "android")]
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("softisp-shim"),
    );
}

#[derive(Clone)]
struct ShimLog {
    path: PathBuf,
}

impl ShimLog {
    fn new(app: &AndroidApp) -> Self {
        Self {
            path: resolve_log_path(app),
        }
    }

    fn line(&self, level: &str, tag: &str, message: &str) {
        let line = format!("{level} [{tag}] {message}\n");
        if let Ok(mut file) = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.path)
        {
            let _ = file.write_all(line.as_bytes());
        }
        match level {
            "FATAL" | "ERROR" => error!("[{tag}] {message}"),
            "WARN" => warn!("[{tag}] {message}"),
            _ => info!("[{tag}] {message}"),
        }
    }
}

fn resolve_public_path(app: &AndroidApp, name: &str) -> PathBuf {
    let primary = Path::new(PRIMARY_LOG)
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(name);
    if OpenOptions::new()
        .create(true)
        .append(true)
        .open(&primary)
        .is_ok()
    {
        return primary;
    }
    app.external_data_path()
        .map(|dir| dir.join(name))
        .unwrap_or(primary)
}

fn resolve_log_path(app: &AndroidApp) -> PathBuf {
    resolve_public_path(app, "softisp-app.log")
}

/// The APK native library directory is resolved from the mapped path of this
/// shim, not hard-coded: Android app directories contain a random suffix.
fn self_library_dir() -> Option<PathBuf> {
    let maps = fs::read_to_string("/proc/self/maps").ok()?;
    for line in maps.lines() {
        if !line.contains("libsoftisp_shim.so") {
            continue;
        }
        let path = line.split_whitespace().nth(5)?;
        return Path::new(path).parent().map(Path::to_path_buf);
    }
    None
}

fn push_unique(list: &mut Vec<PathBuf>, path: PathBuf) {
    if !list.contains(&path) {
        list.push(path);
    }
}

fn search_roots(external_data: Option<PathBuf>, native_dir: Option<&Path>) -> Vec<PathBuf> {
    let mut roots = Vec::new();
    for dir in ["/sdcard/download", "/sdcard/Download"] {
        push_unique(&mut roots, PathBuf::from(dir));
    }
    if let Some(dir) = external_data {
        push_unique(&mut roots, dir);
    }
    if let Some(dir) = native_dir {
        push_unique(&mut roots, dir.to_path_buf());
    }
    push_unique(&mut roots, PathBuf::from("."));
    roots
}

fn format_roots(roots: &[PathBuf]) -> String {
    roots
        .iter()
        .map(|path| path.display().to_string())
        .collect::<Vec<_>>()
        .join(" | ")
}

fn find_core(roots: &[PathBuf]) -> Option<PathBuf> {
    for dir in roots {
        for name in CORE_NAMES {
            let candidate = dir.join(name);
            if candidate.is_file() {
                return Some(candidate);
            }
        }
    }
    None
}

fn find_library(dir: &Path, name: &str) -> Option<PathBuf> {
    let candidate = dir.join(name);
    candidate.is_file().then_some(candidate)
}

struct LoadedLibrary {
    _file: Option<File>,
    handle: *mut c_void,
}

impl Drop for LoadedLibrary {
    fn drop(&mut self) {
        unsafe { dlclose(self.handle) };
    }
}

fn dlerror_text() -> String {
    let ptr = unsafe { dlerror() };
    if ptr.is_null() {
        "no dlerror detail".to_string()
    } else {
        unsafe { CStr::from_ptr(ptr.cast()) }
            .to_string_lossy()
            .into_owned()
    }
}

#[cfg(target_os = "android")]
const MFD_CLOEXEC: libc::c_uint = 0x0001;

#[cfg(target_os = "android")]
fn memfd_library(path: &Path, source: &mut File, log: &ShimLog) -> Option<File> {
    let name = CString::new(format!(
        "softisp_{}",
        path.file_name().unwrap_or_default().to_string_lossy()
    ))
    .ok()?;
    let fd = unsafe { libc::memfd_create(name.as_ptr(), MFD_CLOEXEC) };
    if fd < 0 {
        log.line(
            "FATAL",
            "shim",
            &format!("memfd_create for {} failed: {fd}", path.display()),
        );
        return None;
    }
    let mut memfd = unsafe { File::from_raw_fd(fd) };
    if let Err(err) = std::io::copy(source, &mut memfd) {
        log.line(
            "FATAL",
            "shim",
            &format!("cannot copy {} into memfd: {err}", path.display()),
        );
        return None;
    }
    if let Err(err) = memfd.seek(SeekFrom::Start(0)) {
        log.line(
            "FATAL",
            "shim",
            &format!("cannot rewind memfd for {}: {err}", path.display()),
        );
        return None;
    }
    log.line(
        "INFO",
        "shim",
        &format!("loaded {} into anonymous memfd", path.display()),
    );
    Some(memfd)
}

#[cfg(target_os = "android")]
fn dlopen_fd(path: &Path, identity: &Path, file: &File, log: &ShimLog) -> Option<*mut c_void> {
    let name = CString::new(identity.to_string_lossy().as_bytes()).ok()?;
    let info = AndroidDlextInfo {
        flags: ANDROID_DLEXT_USE_LIBRARY_FD | ANDROID_DLEXT_FORCE_LOAD,
        reserved_addr: std::ptr::null_mut(),
        reserved_size: 0,
        relro_fd: -1,
        library_fd: file.as_raw_fd(),
        library_fd_offset: 0,
        library_namespace: std::ptr::null_mut(),
    };
    unsafe { dlerror() };
    let handle = unsafe { android_dlopen_ext(name.as_ptr(), RTLD_NOW | RTLD_LOCAL, &info) };
    if handle.is_null() {
        log.line(
            "FATAL",
            "shim",
            &format!(
                "memfd-backed dlopen {} as {} failed: {}",
                path.display(),
                identity.display(),
                dlerror_text()
            ),
        );
        return None;
    }
    Some(handle)
}

fn load_library(path: &Path, identity: &Path, log: &ShimLog) -> Option<LoadedLibrary> {
    log.line(
        "INFO",
        "shim",
        &format!("loading {} as {}", path.display(), identity.display()),
    );
    let file = match File::open(path) {
        Ok(file) => file,
        Err(err) => {
            log.line(
                "FATAL",
                "shim",
                &format!("cannot open {} for loading: {err}", path.display()),
            );
            return None;
        }
    };

    #[cfg(target_os = "android")]
    {
        let mut source = file;
        let memfd = memfd_library(path, &mut source, log)?;
        let handle = dlopen_fd(path, identity, &memfd, log)?;
        Some(LoadedLibrary {
            _file: Some(memfd),
            handle,
        })
    }

    #[cfg(not(target_os = "android"))]
    {
        drop(file);
        let name = CString::new(identity.to_string_lossy().as_bytes()).ok()?;
        unsafe { dlerror() };
        let handle = unsafe { dlopen(name.as_ptr(), RTLD_NOW | RTLD_LOCAL) };
        if handle.is_null() {
            log.line(
                "FATAL",
                "shim",
                &format!("dlopen {} failed: {}", path.display(), dlerror_text()),
            );
            return None;
        }
        Some(LoadedLibrary {
            _file: None,
            handle,
        })
    }
}

fn android_main_from_handle(
    handle: *mut c_void,
    path: &Path,
    log: &ShimLog,
) -> Option<AndroidMain> {
    unsafe { dlerror() };
    let symbol = unsafe { dlsym(handle, c"android_main".as_ptr()) };
    if symbol.is_null() {
        log.line(
            "FATAL",
            "shim",
            &format!(
                "dlsym(android_main) in {} failed: {}",
                path.display(),
                dlerror_text()
            ),
        );
        unsafe { dlclose(handle) };
        return None;
    }
    Some(unsafe { std::mem::transmute::<*mut c_void, AndroidMain>(symbol) })
}

fn fatal_not_found(log: &ShimLog, roots: &[PathBuf]) {
    let mut message = format!(
        "core library not found (looked for {}); searched:",
        CORE_NAMES.join(", ")
    );
    for root in roots {
        message.push_str(&format!("\n  {}", root.display()));
    }
    log.line("FATAL", "shim", &message);
}

/// Fingerprint the libraries in the directory they are loaded from. The caller
/// compares these values with `md5sum` of its own build outputs, so a stale
/// upload is detectable from the log alone.
fn log_library_md5s(dir: &Path, log: &ShimLog) {
    let mut paths = Vec::new();
    match fs::read_dir(dir) {
        Ok(entries) => {
            for entry in entries.flatten() {
                let name = entry.file_name();
                let name = name.to_string_lossy();
                let path = entry.path();
                if path.is_file() && name.ends_with(".so") && name != "libsoftisp_shim.so" {
                    paths.push(path);
                }
            }
        }
        Err(err) => {
            log.line(
                "FATAL",
                "shim",
                &format!("cannot list {}: {err}", dir.display()),
            );
            return;
        }
    }
    paths.sort();
    for path in paths {
        match md5::hex_of_file(&path) {
            Ok((digest, bytes)) => {
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                log.line(
                    "INFO",
                    "shim",
                    &format!("md5 {name} {digest} {bytes} bytes"),
                );
            }
            Err(err) => log.line(
                "FATAL",
                "shim",
                &format!("md5 {} failed: {err}", path.display()),
            ),
        }
    }
}

/// Remove `.so` files left by older shim builds that copied libs into the
/// app-private data directory. The current shim never writes there, but this
/// keeps a reload from carrying stale snapshots forward.
fn remove_legacy_staged_libs(dir: Option<&Path>, log: &ShimLog) {
    let Some(dir) = dir else {
        return;
    };
    let entries = match fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(_) => return,
    };
    let mut removed = 0;
    for entry in entries.flatten() {
        let path = entry.path();
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if !path.is_file() || !name.ends_with(".so") || name == "libsoftisp_shim.so" {
            continue;
        }
        match fs::remove_file(&path) {
            Ok(()) => removed += 1,
            Err(err) => log.line(
                "WARN",
                "shim",
                &format!("cannot remove legacy staged {}: {err}", path.display()),
            ),
        }
    }
    log.line(
        "INFO",
        "shim",
        &format!(
            "removed {removed} legacy staged .so files from {}",
            dir.display()
        ),
    );
}

#[cfg(target_os = "android")]
type IntentReader = dyn Fn() -> Option<String> + Send + Sync;

#[cfg(target_os = "android")]
fn intent_reader(app: &AndroidApp, log: &ShimLog) -> Option<Box<IntentReader>> {
    let vm = unsafe { JavaVM::from_raw(app.vm_as_ptr().cast()) };
    let raw_activity = app.activity_as_ptr() as jobject;
    let activity: Option<Global<JObject<'static>>> = vm
        .attach_current_thread::<_, _, jni::errors::Error>(|env| {
            let activity = unsafe { JObject::from_raw(env, raw_activity) };
            env.new_global_ref(activity)
        })
        .ok();
    let Some(activity) = activity else {
        log.line("WARN", "shim", "cannot retain Activity ref; control off");
        return None;
    };

    Some(Box::new(move || {
        vm.attach_current_thread(|env| {
            let intent = env
                .call_method(
                    activity.as_obj(),
                    jni::jni_str!("getIntent"),
                    jni::jni_sig!("()Landroid/content/Intent;"),
                    &[],
                )?
                .l()?;
            if intent.is_null() {
                return Ok::<_, jni::errors::Error>(None);
            }
            let action = env
                .call_method(
                    &intent,
                    jni::jni_str!("getAction"),
                    jni::jni_sig!("()Ljava/lang/String;"),
                    &[],
                )?
                .l()?;
            if action.is_null() {
                return Ok::<_, jni::errors::Error>(None);
            }
            let action = jni::objects::JString::cast_local(env, action)?;
            if action.try_to_string(env)? != INTENT_ACTION {
                return Ok::<_, jni::errors::Error>(None);
            }
            let key = env.new_string(INTENT_EXTRA)?;
            let value = env
                .call_method(
                    &intent,
                    jni::jni_str!("getStringExtra"),
                    jni::jni_sig!("(Ljava/lang/String;)Ljava/lang/String;"),
                    &[(&key).into()],
                )?
                .l()?;
            if value.is_null() {
                return Ok::<_, jni::errors::Error>(None);
            }
            let value = jni::objects::JString::cast_local(env, value)?;
            let text = value.try_to_string(env)?;
            let remove_key = env.new_string(INTENT_EXTRA)?;
            env.call_method(
                &intent,
                jni::jni_str!("removeExtra"),
                jni::jni_sig!("(Ljava/lang/String;)V"),
                &[(&remove_key).into()],
            )?;
            Ok::<_, jni::errors::Error>(Some(text))
        })
        .ok()
        .flatten()
    }))
}

/// Intent-driven control for the external debug loop. A dedicated
/// `com.softisp.camera.action.CONTROL` action with extra `softisp_cmd`
/// reaches this watcher through `ShimActivity.onNewIntent`; no shell
/// force-stop permission is needed.
#[cfg(target_os = "android")]
fn spawn_control_watcher(
    app: &AndroidApp,
    log: ShimLog,
    core_handle: *mut c_void,
    legacy_dir: Option<PathBuf>,
) {
    let Some(read_command) = intent_reader(app, &log) else {
        return;
    };
    log.line(
        "INFO",
        "shim",
        "control watcher armed (intent action ready)",
    );
    let core_handle = core_handle as usize;
    let vm = unsafe { JavaVM::from_raw(app.vm_as_ptr().cast()) };
    let raw_activity = app.activity_as_ptr() as jobject;
    let activity: Option<Global<JObject<'static>>> = vm
        .attach_current_thread::<_, _, jni::errors::Error>(|env| {
            let activity = unsafe { JObject::from_raw(env, raw_activity) };
            env.new_global_ref(activity)
        })
        .ok();

    thread::spawn(move || loop {
        let Some(command) = read_command() else {
            thread::sleep(Duration::from_millis(200));
            continue;
        };
        let command = command.trim().to_ascii_lowercase();
        match command.as_str() {
            "clean" => {
                log.line("INFO", "shim", "control intent: clean legacy staged libs");
                remove_legacy_staged_libs(legacy_dir.as_deref(), &log);
            }
            "ping" => log.line("INFO", "shim", "control intent: ping"),
            "cancel" => {
                log.line("INFO", "shim", "control intent: cancel pipeline");
                unsafe { dlerror() };
                let symbol = unsafe {
                    dlsym(
                        core_handle as *mut c_void,
                        c"softisp_anw_cancel_pipeline".as_ptr(),
                    )
                };
                if symbol.is_null() {
                    log.line(
                        "WARN",
                        "shim",
                        &format!("softisp_anw_cancel_pipeline not found: {}", dlerror_text()),
                    );
                } else {
                    let cancel: unsafe extern "C" fn() =
                        unsafe { std::mem::transmute::<*mut c_void, _>(symbol) };
                    unsafe { cancel() };
                    log.line("INFO", "shim", "pipeline cancellation requested");
                }
            }
            "stop" | "kill" | "quit" | "exit" => {
                log.line(
                    "INFO",
                    "shim",
                    &format!("control intent '{command}'; finishing activity"),
                );
                remove_legacy_staged_libs(legacy_dir.as_deref(), &log);
                let Some(activity) = activity.as_ref() else {
                    std::process::exit(0);
                };
                let result = vm.attach_current_thread(|env| {
                    env.call_method(
                        activity.as_obj(),
                        jni::jni_str!("finish"),
                        jni::jni_sig!("()V"),
                        &[],
                    )?;
                    Ok::<(), jni::errors::Error>(())
                });
                if let Err(err) = result {
                    log.line("WARN", "shim", &format!("Activity.finish() failed: {err}"));
                } else {
                    log.line("INFO", "shim", "Activity.finish() requested");
                }
                thread::sleep(Duration::from_secs(2));
                log.line("WARN", "shim", "forced exit after stop intent");
                std::process::exit(0);
            }
            _ => log.line(
                "WARN",
                "shim",
                &format!("unknown control intent value '{command}'"),
            ),
        }
    });
}

#[unsafe(no_mangle)]
fn android_main(app: AndroidApp) {
    init_logcat();
    let log = ShimLog::new(&app);
    log.line("INFO", "shim", "launcher started");

    let native_dir = self_library_dir();
    let roots = search_roots(app.external_data_path(), native_dir.as_deref());
    log.line(
        "INFO",
        "shim",
        &format!("core search roots: {}", format_roots(&roots)),
    );

    if native_dir.is_none() {
        log.line(
            "WARN",
            "shim",
            "cannot resolve APK native library dir from /proc/self/maps",
        );
    }
    #[cfg(target_os = "android")]
    remove_legacy_staged_libs(app.internal_data_path().as_deref(), &log);

    let Some(source) = find_core(&roots) else {
        fatal_not_found(&log, &roots);
        return;
    };
    log.line("INFO", "shim", &format!("found core {}", source.display()));

    let load_dir = source
        .parent()
        .filter(|dir| !dir.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    log.line(
        "INFO",
        "shim",
        &format!("loading runtime libraries from {}", load_dir.display()),
    );
    log_library_md5s(load_dir, &log);

    let mut loaded = Vec::new();
    for name in RUNTIME_DEPENDENCIES {
        let Some(path) = find_library(load_dir, name) else {
            log.line(
                "FATAL",
                "shim",
                &format!(
                    "missing runtime dependency {name} next to {}",
                    source.display()
                ),
            );
            return;
        };
        let identity = native_dir
            .as_deref()
            .map(|dir| dir.join(name))
            .unwrap_or_else(|| path.clone());
        let Some(library) = load_library(&path, &identity, &log) else {
            return;
        };
        log.line("INFO", "shim", &format!("loaded dependency {name}"));
        loaded.push(library);
    }

    let core_name = source.file_name().unwrap_or_default();
    let core_identity = native_dir
        .as_deref()
        .map(|dir| dir.join(core_name))
        .unwrap_or_else(|| source.clone());
    let Some(core) = load_library(&source, &core_identity, &log) else {
        return;
    };
    let core_handle = core.handle;
    let Some(entry) = android_main_from_handle(core_handle, &source, &log) else {
        return;
    };
    loaded.push(core);

    #[cfg(target_os = "android")]
    spawn_control_watcher(&app, log.clone(), core_handle, app.internal_data_path());
    #[cfg(not(target_os = "android"))]
    let _ = core_handle;

    std::env::set_var("SOFTISP_SHIM_CHAINED", "1");
    log.line(
        "INFO",
        "shim",
        &format!("forwarding android_main to {}", source.display()),
    );
    unsafe { entry(app) };
}

#[cfg(test)]
mod tests {
    use super::{format_roots, md5::hex_of_bytes, search_roots, CORE_NAMES, RUNTIME_DEPENDENCIES};
    use std::path::PathBuf;

    #[test]
    fn md5_matches_known_vectors() {
        assert_eq!(hex_of_bytes(b""), "d41d8cd98f00b204e9800998ecf8427e");
        assert_eq!(hex_of_bytes(b"abc"), "900150983cd24fb0d6963f7d28e17f72");
        assert_eq!(
            hex_of_bytes(b"message digest"),
            "f96b697d7cb7938d525a2f31aaf161d0"
        );
        assert_eq!(
            hex_of_bytes(b"abcdefghijklmnopqrstuvwxyz"),
            "c3fcd3d76192e4007dfb496cca67e13b"
        );
    }

    #[test]
    fn md5_handles_multi_block_input() {
        assert_eq!(
            hex_of_bytes(&[b'a'; 1_000]),
            "cabe45dcc9ae5b66ba86600cca6b8ba8"
        );
    }

    #[test]
    fn search_roots_prefers_uploaded_core_over_apk() {
        let external = PathBuf::from("/sdcard/Android/data/com.softisp.camera/files");
        let native = PathBuf::from("/data/app/com.softisp.camera/lib/arm64");
        let roots = search_roots(Some(external.clone()), Some(&native));

        assert_eq!(roots[0], PathBuf::from("/sdcard/download"));
        assert_eq!(roots[1], PathBuf::from("/sdcard/Download"));
        assert_eq!(roots[2], external);
        assert_eq!(roots[3], native);
        assert_eq!(roots[4], PathBuf::from("."));
    }

    #[test]
    fn search_roots_deduplicates_native_path() {
        let native = PathBuf::from("/data/app/com.softisp.camera/lib/arm64");
        let roots = search_roots(None, Some(&native));
        assert_eq!(roots.len(), 4);
        assert_eq!(roots[0], PathBuf::from("/sdcard/download"));
        assert_eq!(roots[2], native);
        assert_eq!(roots.iter().filter(|path| *path == &native).count(), 1);
    }

    #[test]
    fn runtime_dependencies_are_loaded_before_core() {
        assert!(!RUNTIME_DEPENDENCIES
            .iter()
            .any(|name| CORE_NAMES.contains(name)));
        let mnn = RUNTIME_DEPENDENCIES
            .iter()
            .position(|name| *name == "libMNN.so")
            .unwrap();
        let convert = RUNTIME_DEPENDENCIES
            .iter()
            .position(|name| *name == "libMNNConvertDeps.so")
            .unwrap();
        assert!(mnn < convert);
    }

    #[test]
    fn core_names_cover_anw_and_hotload_outputs() {
        assert_eq!(CORE_NAMES.len(), 3);
        assert!(CORE_NAMES.contains(&"libsoftisp.so"));
        assert!(CORE_NAMES.contains(&"libsoftisp_core.so"));
        assert!(CORE_NAMES.contains(&"libsoftisp_hot.so"));
    }

    #[test]
    fn format_roots_is_readable_on_one_line() {
        let roots = vec![PathBuf::from("/a"), PathBuf::from("/b")];
        assert_eq!(format_roots(&roots), "/a | /b");
    }
}
