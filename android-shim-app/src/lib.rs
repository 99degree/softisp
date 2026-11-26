//! APK launcher shim.
//!
//! Android starts this cdylib's `android_main` through the stock
//! `android.app.NativeActivity`. It locates the real application cdylib built
//! by `anw/` (`libsoftisp.so`), stages it and its MNN dependencies into
//! app-private storage, and forwards the live `AndroidApp` to the core's
//! `android_main`. Rebuilding the ISP/MNN core then needs no APK repack.
//!
//! Every search root is mirrored to logcat and to
//! `/sdcard/download/softisp-app.log` (falling back to app-private storage),
//! including on failure, so a missing core is diagnosable either way.

use std::ffi::{c_int, c_void, CStr, CString};
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, SystemTime};

use android_activity::AndroidApp;
use libc::{dlclose, dlerror, dlopen, dlsym};
use log::{error, info, warn};

#[cfg(target_os = "android")]
use jni::{objects::JObject, refs::Global, sys::jobject, JavaVM};

const CORE_NAMES: &[&str] = &["libsoftisp.so", "libsoftisp_core.so", "libsoftisp_hot.so"];
const STAGED_NAME: &str = "libsoftisp.so";
const RTLD_NOW: c_int = 2;
const RTLD_LOCAL: c_int = 0;
const PRIMARY_LOG: &str = "/sdcard/download/softisp-app.log";
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

fn signature(path: &Path) -> Option<(u64, SystemTime)> {
    let meta = fs::metadata(path).ok()?;
    Some((meta.len(), meta.modified().ok()?))
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

fn search_roots(
    external_data: Option<PathBuf>,
    native_dir: Option<&Path>,
    staged_dir: &Path,
) -> Vec<PathBuf> {
    let mut roots = Vec::new();
    push_unique(&mut roots, staged_dir.to_path_buf());
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

/// Copy MNN/C++ dependencies next to the staged core so its `$ORIGIN` rpath
/// resolves after `dlopen`. The test APK ships none of them, so they are taken
/// from the same roots as the core (normally `/sdcard/download`). Core names
/// are skipped: an old APK copy must never shadow the pushed update.
fn sync_dependencies(roots: &[PathBuf], staged_dir: &Path, log: &ShimLog) {
    let mut copied = 0;
    for dir in roots {
        let entries = match fs::read_dir(dir) {
            Ok(entries) => entries,
            Err(_) => continue,
        };
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if !name.ends_with(".so")
                || name == "libsoftisp_shim.so"
                || CORE_NAMES.iter().any(|core| *core == name.as_ref())
            {
                continue;
            }
            let source = entry.path();
            let dest = staged_dir.join(name.as_ref());
            if signature(&source) == signature(&dest) {
                continue;
            }
            if fs::copy(&source, &dest).is_ok() {
                copied += 1;
            }
        }
    }
    log.line(
        "INFO",
        "shim",
        &format!(
            "staged {copied} dependency libs into {} from: {}",
            staged_dir.display(),
            format_roots(roots)
        ),
    );
}

fn copy_core(source: &Path, staged: &Path, log: &ShimLog) -> bool {
    if source == staged && signature(source).is_some() {
        return true;
    }
    if signature(source) == signature(staged) {
        return true;
    }
    match fs::copy(source, staged) {
        Ok(bytes) => {
            log.line(
                "INFO",
                "shim",
                &format!(
                    "staged core {} -> {} ({bytes} bytes)",
                    source.display(),
                    staged.display()
                ),
            );
            true
        }
        Err(err) => {
            log.line(
                "FATAL",
                "shim",
                &format!(
                    "cannot stage core {} -> {}: {err}",
                    source.display(),
                    staged.display()
                ),
            );
            false
        }
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

fn load_android_main(staged: &Path, log: &ShimLog) -> Option<(AndroidMain, *mut c_void)> {
    let path = CString::new(staged.to_string_lossy().as_bytes()).ok()?;
    unsafe { dlerror() };
    let handle = unsafe { dlopen(path.as_ptr(), RTLD_NOW | RTLD_LOCAL) };
    if handle.is_null() {
        log.line(
            "FATAL",
            "shim",
            &format!("dlopen {} failed: {}", staged.display(), dlerror_text()),
        );
        return None;
    }
    unsafe { dlerror() };
    let symbol = unsafe { dlsym(handle, c"android_main".as_ptr()) };
    if symbol.is_null() {
        log.line(
            "FATAL",
            "shim",
            &format!(
                "dlsym(android_main) in {} failed: {}",
                staged.display(),
                dlerror_text()
            ),
        );
        unsafe { dlclose(handle) };
        return None;
    }
    Some((
        unsafe { std::mem::transmute::<*mut c_void, AndroidMain>(symbol) },
        handle,
    ))
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

/// Fingerprint every library that is about to be loaded. The caller compares
/// these values with `md5sum` of its own build outputs, so a stale core on the
/// device is detectable from the log alone.
fn log_library_md5s(staged_dir: &Path, log: &ShimLog) {
    let mut paths = vec![staged_dir.join(STAGED_NAME)];
    if let Ok(entries) = fs::read_dir(staged_dir) {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if name.ends_with(".so") && name != STAGED_NAME {
                paths.push(entry.path());
            }
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
fn spawn_control_watcher(app: &AndroidApp, log: ShimLog, core_handle: *mut c_void) {
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

    let Some(internal) = app.internal_data_path() else {
        log.line(
            "FATAL",
            "shim",
            "no app internal data path; cannot stage core",
        );
        return;
    };
    let staged_dir = internal;
    if let Err(err) = fs::create_dir_all(&staged_dir) {
        log.line(
            "FATAL",
            "shim",
            &format!("cannot create {}: {err}", staged_dir.display()),
        );
        return;
    }

    let native_dir = self_library_dir();
    let roots = search_roots(app.external_data_path(), native_dir.as_deref(), &staged_dir);
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
    sync_dependencies(&roots, &staged_dir, &log);

    let Some(source) = find_core(&roots) else {
        fatal_not_found(&log, &roots);
        return;
    };
    log.line("INFO", "shim", &format!("found core {}", source.display()));

    let staged = staged_dir.join(STAGED_NAME);
    if !copy_core(&source, &staged, &log) {
        return;
    }
    log_library_md5s(&staged_dir, &log);
    let Some((entry, core_handle)) = load_android_main(&staged, &log) else {
        return;
    };
    #[cfg(target_os = "android")]
    spawn_control_watcher(&app, log.clone(), core_handle);
    #[cfg(not(target_os = "android"))]
    let _ = core_handle;

    std::env::set_var("SOFTISP_SHIM_CHAINED", "1");
    log.line(
        "INFO",
        "shim",
        &format!("forwarding android_main to {}", staged.display()),
    );
    unsafe { entry(app) };
}

#[cfg(test)]
mod tests {
    use super::{format_roots, md5::hex_of_bytes, search_roots, CORE_NAMES};
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
    fn search_roots_prioritises_staged_update_over_apk() {
        let staged = PathBuf::from("/data/user/0/com.softisp.camera/files");
        let external = PathBuf::from("/sdcard/Android/data/com.softisp.camera/files");
        let native = PathBuf::from("/data/app/com.softisp.camera/lib/arm64");
        let roots = search_roots(Some(external.clone()), Some(&native), &staged);

        assert_eq!(roots[0], staged);
        assert_eq!(roots[1], PathBuf::from("/sdcard/download"));
        assert_eq!(roots[2], PathBuf::from("/sdcard/Download"));
        assert_eq!(roots[3], external);
        assert_eq!(roots[4], native);
        assert_eq!(roots[5], PathBuf::from("."));
    }

    #[test]
    fn search_roots_deduplicates_native_and_staged_paths() {
        let staged = PathBuf::from("/data/app/com.softisp.camera/lib/arm64");
        let roots = search_roots(None, Some(&staged), &staged);
        assert_eq!(roots.len(), 4);
        assert_eq!(roots[0], staged);
        assert!(!roots.iter().skip(1).any(|path| path == &staged));
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
