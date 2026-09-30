//! Crash / diagnostic log.
//!
//! The app has no Java layer, so a Rust panic would otherwise be invisible
//! outside logcat. Every error and every panic is appended to a file so a
//! crashed camera preview can be diagnosed after the fact.
//!
//! The mandated location is `/sdcard/download/softisp-app.log`. Under scoped
//! storage (API 29+) that directory is only reachable with
//! `MANAGE_EXTERNAL_STORAGE`, a special-access grant the user makes in Settings
//! and which a `NativeActivity` — having no Activity object to drive a dialog —
//! cannot request for itself. So [`install`] walks a candidate list and keeps
//! the first one it can actually open:
//!
//! 1. `/sdcard/download/softisp-app.log` — the spec path, used when the
//!    permission has been granted.
//! 2. `/sdcard/Download/softisp-app.log` and
//!    `/storage/emulated/0/Download/softisp-app.log` — the same directory under
//!    its other spellings, in case the lowercase form is ever not folded onto
//!    it.
//! 3. `<external data dir>/softisp-app.log` — the app-specific external
//!    directory, which needs no permission at all, so there is always somewhere
//!    to write. Whichever path won is recorded as the first line of the file.
//!
//! The file is size-capped: once it exceeds [`MAX_LOG_BYTES`] it is rotated to
//! `softisp-app.log.1` (single generation) before new data is appended.

use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};

/// Primary log file — the path mandated by the app spec.
pub const LOG_PATH: &str = "/sdcard/download/softisp-app.log";

/// Name of the rotated generation, written next to the active log.
pub const LOG_ROTATED_NAME: &str = "softisp-app.log.1";

/// Rotation threshold. At roughly 120 bytes per line this is ~4,000 lines.
pub const MAX_LOG_BYTES: u64 = 512 * 1024;

static ACTIVE_PATH: OnceLock<PathBuf> = OnceLock::new();
static LOG_READY: AtomicBool = AtomicBool::new(false);
static WRITE_FAILED: AtomicBool = AtomicBool::new(false);
static FILE_LOCK: Mutex<()> = Mutex::new(());
static APP_START_MS: AtomicU64 = AtomicU64::new(0);

/// Timestamp `SystemTime::now()` as milliseconds since the Unix epoch.
///
/// Returns 0 if the clock is before the epoch (never on Android, but the
/// fallible API demands a total function).
pub fn now_millis() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Format `now_millis()` as a compact stamp: `MM-DD HH:MM:SS.mmm`.
///
/// Uses UTC so the log is self-describing without a timezone database.
pub fn timestamp(ms: u64) -> String {
    let secs = ms / 1000;
    let sub_ms = ms % 1000;
    let tod = secs % 86_400;

    // Walk whole years off the epoch so February 29 is handled, then index
    // into a cumulative month table. The loop runs once per elapsed year, so
    // it is a handful of iterations even in the far future.
    const CUM: [u32; 12] = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
    let mut year = 1970u64;
    let mut day_of_year = secs / 86_400;
    loop {
        let len = if is_leap(year) { 366 } else { 365 };
        if day_of_year < len {
            break;
        }
        day_of_year -= len;
        year += 1;
    }
    let day_of_year = day_of_year as u32;

    let leap = is_leap(year);
    let month = (0..12)
        .rev()
        .find(|&i| day_of_year >= CUM[i] + u32::from(leap && i >= 2))
        .unwrap_or(11);
    let month_start = CUM[month] + u32::from(leap && month >= 2);
    format!(
        "{:02}-{:02} {:02}:{:02}:{:02}.{:03}",
        month + 1,
        day_of_year - month_start + 1,
        tod / 3600,
        (tod / 60) % 60,
        tod % 60,
        sub_ms
    )
}

/// Gregorian leap-year test for a full four-digit year.
fn is_leap(year: u64) -> bool {
    year.is_multiple_of(4) && !year.is_multiple_of(100) || year.is_multiple_of(400)
}

/// Milliseconds since [`install`] was called, for elapsed-time columns.
pub fn elapsed_ms() -> u64 {
    now_millis().saturating_sub(APP_START_MS.load(Ordering::Relaxed))
}

/// The file currently being written, or `None` before [`install`] succeeds.
pub fn active_path() -> Option<&'static Path> {
    ACTIVE_PATH.get().map(PathBuf::as_path)
}

/// Files to try, in preference order.
///
/// `fallback_dir` is the app's external data directory, which is always
/// writable and therefore the last resort. See the module docs for why the
/// first entries need a runtime-granted permission.
pub fn candidate_paths(fallback_dir: Option<&Path>) -> Vec<PathBuf> {
    let mut out = vec![
        PathBuf::from(LOG_PATH),
        PathBuf::from("/sdcard/Download/softisp-app.log"),
        PathBuf::from("/storage/emulated/0/Download/softisp-app.log"),
    ];
    if let Some(dir) = fallback_dir {
        out.push(dir.join("softisp-app.log"));
    }
    out
}

/// Try each candidate in turn and keep the first that can be opened for append.
fn resolve_log_path(candidates: &[PathBuf]) -> Option<PathBuf> {
    for path in candidates {
        // Best effort: the parent may not exist yet, and creating it can fail
        // even in cases where writing the file would have succeeded.
        if let Some(dir) = path.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        match OpenOptions::new().create(true).append(true).open(path) {
            Ok(_) => return Some(path.clone()),
            Err(e) => log::warn!("crashlog: {} unwritable: {e}", path.display()),
        }
    }
    None
}

/// Install the process-wide log file and the panic hook.
///
/// Idempotent: a second call is a no-op, so `install()` is safe to invoke from
/// both the entry point and early error paths. `fallback_dir` is normally the
/// activity's external data directory, used when `/sdcard/download` is not
/// reachable.
pub fn install(fallback_dir: Option<&Path>) {
    if LOG_READY.swap(true, Ordering::SeqCst) {
        return;
    }
    APP_START_MS.store(now_millis(), Ordering::Relaxed);

    let candidates = candidate_paths(fallback_dir);
    match resolve_log_path(&candidates) {
        Some(path) => {
            let _ = ACTIVE_PATH.set(path.clone());
            let ver = env!("CARGO_PKG_VERSION");
            write_line("INFO", "boot", &format!("log file: {}", path.display()));
            write_line("INFO", "boot", &format!("softisp camera {ver} starting"));
        }
        None => {
            // Nothing is writable. The panic hook is still installed so the
            // message reaches logcat, but there is no file to append to.
            eprintln!(
                "softisp: no writable log path; tried {}",
                candidates
                    .iter()
                    .map(|p| p.display().to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
    }

    install_panic_hook();
}

fn install_panic_hook() {
    let previous = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        write_line("FATAL", "panic", &panic_message(info));
        write_line(
            "FATAL",
            "panic",
            "backtrace follows in logcat under tag 'softisp'",
        );
        previous(info);
    }));
}

/// Render a `PanicHookInfo` as a single line.
fn panic_message(info: &std::panic::PanicHookInfo<'_>) -> String {
    if let Some(s) = info.payload().downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = info.payload().downcast_ref::<String>() {
        s.clone()
    } else {
        let loc = info
            .location()
            .map(|l| format!("{}:{}:{}", l.file(), l.line(), l.column()))
            .unwrap_or_else(|| "<unknown location>".into());
        format!("non-string panic payload at {loc}")
    }
}

/// Write one record, rotating first if the log has outgrown [`MAX_LOG_BYTES`].
pub fn write_line(level: &str, tag: &str, msg: &str) {
    if !LOG_READY.load(Ordering::SeqCst) || WRITE_FAILED.load(Ordering::SeqCst) {
        return;
    }
    let Some(path) = ACTIVE_PATH.get() else {
        return;
    };
    let Ok(_guard) = FILE_LOCK.lock() else {
        return;
    };

    if rotate_if_needed(path).is_err() {
        WRITE_FAILED.store(true, Ordering::SeqCst);
        return;
    }

    let line = format!(
        "{} {:5} [{}] {:>7}ms {}\n",
        timestamp(now_millis()),
        level,
        tag,
        elapsed_ms(),
        msg.replace('\n', " ")
    );

    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(path) {
        let _ = f.write_all(line.as_bytes());
        let _ = f.flush();
    } else {
        WRITE_FAILED.store(true, Ordering::SeqCst);
    }
}

/// Move an oversized log aside so the next append starts a fresh file.
fn rotate_if_needed(path: &Path) -> std::io::Result<()> {
    let too_big = match std::fs::metadata(path) {
        Ok(m) => m.len() >= MAX_LOG_BYTES,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(e) => return Err(e),
    };
    if !too_big {
        return Ok(());
    }
    // A single rotated generation is enough: the previous one is discarded.
    let rotated = path.with_file_name(LOG_ROTATED_NAME);
    let _ = std::fs::remove_file(&rotated);
    std::fs::rename(path, rotated)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join("softisp-crashlog-test");
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    #[test]
    fn timestamp_renders_expected_shape() {
        // 2026-01-02T03:04:05.006Z → "01-02 03:04:05.006"
        assert_eq!(timestamp(1_767_323_045_006), "01-02 03:04:05.006");
    }

    #[test]
    fn timestamp_handles_leap_day() {
        // 2024-03-01T00:00:00.000Z (2024 is a leap year)
        assert_eq!(timestamp(1_709_251_200_000), "03-01 00:00:00.000");
    }

    #[test]
    fn timestamp_handles_non_leap_year() {
        // 2023-03-01T00:00:00.000Z
        assert_eq!(timestamp(1_677_628_800_000), "03-01 00:00:00.000");
    }

    #[test]
    fn is_leap_matches_gregorian_rules() {
        assert!(is_leap(2024));
        assert!(!is_leap(2023));
        assert!(!is_leap(2100)); // century, not leap
        assert!(is_leap(2000)); // 400-year rule
    }

    #[test]
    fn spec_path_is_the_first_candidate() {
        let cands = candidate_paths(None);
        assert_eq!(cands[0], PathBuf::from(LOG_PATH));
        assert!(
            cands
                .iter()
                .any(|p| p == Path::new("/sdcard/Download/softisp-app.log")),
            "expected the capitalised spelling too"
        );
    }

    #[test]
    fn fallback_dir_is_appended_last() {
        let fb = Path::new("/data/data/com.softisp.camera/files");
        assert_eq!(
            candidate_paths(Some(fb)).last().unwrap(),
            &fb.join("softisp-app.log")
        );
    }

    #[test]
    fn resolve_skips_unwritable_candidates() {
        let good = scratch("resolve-good.log");
        let _ = std::fs::remove_file(&good);
        // A path under a *file* can never be created, so it always fails.
        let blocker = scratch("resolve-blocker");
        std::fs::write(&blocker, b"x").unwrap();
        let impossible = blocker.join("nested/app.log");

        assert_eq!(
            resolve_log_path(&[impossible, good.clone()]),
            Some(good.clone()),
            "the first writable candidate must win"
        );
        let _ = std::fs::remove_file(&good);
        let _ = std::fs::remove_file(&blocker);
    }

    #[test]
    fn resolve_returns_none_when_everything_fails() {
        let blocker = scratch("resolve-none-blocker");
        std::fs::write(&blocker, b"x").unwrap();
        assert!(resolve_log_path(&[blocker.join("a/b/c.log")]).is_none());
        let _ = std::fs::remove_file(&blocker);
    }

    #[test]
    fn rotate_skips_missing_file() {
        let p = scratch("rotate-missing.log");
        let _ = std::fs::remove_file(&p);
        assert!(rotate_if_needed(&p).is_ok());
    }

    #[test]
    fn rotate_leaves_small_file_alone() {
        let p = scratch("rotate-small.log");
        std::fs::write(&p, vec![b'x'; 64]).unwrap();
        assert!(rotate_if_needed(&p).is_ok());
        assert!(p.exists(), "a small file must not be rotated");
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn rotate_moves_oversized_file() {
        let p = scratch("rotate-big.log");
        let rotated = p.with_file_name(LOG_ROTATED_NAME);
        let _ = std::fs::remove_file(&rotated);
        // MAX_LOG_BYTES is a const, so pad to exactly the threshold: `>=` must
        // treat that as oversized.
        std::fs::write(&p, vec![b'x'; MAX_LOG_BYTES as usize]).unwrap();
        assert!(rotate_if_needed(&p).is_ok());
        assert!(!p.exists(), "oversized file must be moved aside");
        assert!(rotated.exists(), "rotated generation must exist");
        let _ = std::fs::remove_file(&rotated);
    }
}
