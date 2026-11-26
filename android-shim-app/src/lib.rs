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
use std::time::SystemTime;

use android_activity::AndroidApp;
use libc::{dlclose, dlerror, dlopen, dlsym};
use log::{error, info, warn};

const CORE_NAMES: &[&str] = &["libsoftisp.so", "libsoftisp_core.so", "libsoftisp_hot.so"];
const STAGED_NAME: &str = "libsoftisp.so";
const RTLD_NOW: c_int = 2;
const RTLD_LOCAL: c_int = 0;
const PRIMARY_LOG: &str = "/sdcard/download/softisp-app.log";

type AndroidMain = unsafe extern "Rust" fn(AndroidApp);

fn init_logcat() {
    #[cfg(target_os = "android")]
    android_logger::init_once(
        android_logger::Config::default()
            .with_max_level(log::LevelFilter::Info)
            .with_tag("softisp-shim"),
    );
}

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

fn resolve_log_path(app: &AndroidApp) -> PathBuf {
    let primary = PathBuf::from(PRIMARY_LOG);
    if OpenOptions::new()
        .create(true)
        .append(true)
        .open(&primary)
        .is_ok()
    {
        return primary;
    }
    app.external_data_path()
        .map(|dir| dir.join("softisp-app.log"))
        .unwrap_or(primary)
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

fn load_android_main(staged: &Path, log: &ShimLog) -> Option<AndroidMain> {
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
    let Some(entry) = load_android_main(&staged, &log) else {
        return;
    };

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
    use super::{format_roots, search_roots, CORE_NAMES};
    use std::path::PathBuf;

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
