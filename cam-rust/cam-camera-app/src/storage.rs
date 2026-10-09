//! Where the app is allowed to write, and how to find out.
//!
//! `/sdcard/download` is the mandated log location but not a location every
//! Android release lets an app touch. This module answers one question — can
//! the app write there right now? — and reports it in a form that can go
//! straight into the crash log.
//!
//! The rules, in order of authority:
//!
//! * **API 29 and below** — `WRITE_EXTERNAL_STORAGE` is a runtime permission. A
//!   `NativeActivity` has no Activity object to show the dialog, so it must be
//!   granted out of band:
//!   `adb shell pm grant com.softisp.camera android.permission.WRITE_EXTERNAL_STORAGE`.
//! * **API 30 and above** — scoped storage is mandatory. The only route to the
//!   public Download directory is the "all files access" special access
//!   (`MANAGE_EXTERNAL_STORAGE`), which only the user can grant from Settings:
//!   Settings → Apps → SoftISP Camera → All files access.
//!
//! Rather than modelling that matrix, [`probe`] just tries to open the target
//! for append. The result is the ground truth for the running device, and it
//! keeps working if a future release changes the rules again.

use std::fs::OpenOptions;
use std::io::Write;
use std::path::Path;

/// The outcome of testing whether a path is writable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// The file was created or opened for append.
    Writable,
    /// The path could not be opened; the probe carries a reason.
    Denied,
}

impl Access {
    /// A short, log-friendly explanation of a [`Access::Denied`] result.
    pub fn denied_reason(err: &std::io::Error) -> String {
        use std::io::ErrorKind::*;
        let hint = match err.kind() {
            PermissionDenied => {
                "no storage permission — grant WRITE_EXTERNAL_STORAGE (API <= 29) or \
                 'All files access' in Settings (API >= 30)"
            }
            NotFound => "the parent directory does not exist",
            _ => "",
        };
        if hint.is_empty() {
            format!("{err}")
        } else {
            format!("{err} ({hint})")
        }
    }
}

/// Test whether `path` can be opened for append, creating it if absent.
///
/// A zero-byte probe file is not left behind when `keep` is false: the caller
/// is normally about to write real data, and an empty artefact in the user's
/// Download directory is just litter. `keep` forces the file to be left in
/// place, for the case where the caller will not write anything itself.
pub fn probe(path: &Path, keep: bool) -> (Access, Option<String>) {
    if let Some(dir) = path.parent() {
        if let Err(e) = std::fs::create_dir_all(dir) {
            return (Access::Denied, Some(Access::denied_reason(&e)));
        }
    }
    let existed = path.exists();
    match OpenOptions::new().create(true).append(true).open(path) {
        Ok(mut f) => {
            if !existed && !keep {
                // Leaving a zero-byte file behind would look like a real (empty)
                // log; drop it again so the caller decides when to create it.
                let _ = std::fs::remove_file(path);
            } else if !existed {
                let _ = f.write_all(b"");
            }
            (Access::Writable, None)
        }
        Err(e) => (Access::Denied, Some(Access::denied_reason(&e))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join("softisp-storage-test");
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(name)
    }

    #[test]
    fn writable_path_is_reported_writable() {
        let p = scratch("ok.log");
        let _ = std::fs::remove_file(&p);
        let (access, err) = probe(&p, false);
        assert_eq!(access, Access::Writable);
        assert!(err.is_none());
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn probe_does_not_leave_an_empty_file_behind() {
        let p = scratch("no-litter.log");
        let _ = std::fs::remove_file(&p);
        probe(&p, false);
        assert!(!p.exists(), "probe must clean up after itself");
    }

    #[test]
    fn keep_preserves_the_file() {
        let p = scratch("kept.log");
        let _ = std::fs::remove_file(&p);
        probe(&p, true);
        assert!(p.exists(), "keep=true must leave the file");
        let _ = std::fs::remove_file(&p);
    }

    #[test]
    fn unwritable_parent_is_reported_denied() {
        // A *file* as the parent directory can never be created.
        let blocker = scratch("blocker");
        std::fs::write(&blocker, b"x").unwrap();
        let (access, err) = probe(&blocker.join("nested/app.log"), false);
        assert_eq!(access, Access::Denied);
        assert!(err.is_some(), "a denial must carry a reason");
        let _ = std::fs::remove_file(&blocker);
    }

    #[test]
    fn missing_parent_is_created() {
        let base = scratch("mkdir-parent");
        let _ = std::fs::remove_dir_all(&base);
        let p = base.join("deep/app.log");
        assert_eq!(probe(&p, false).0, Access::Writable);
        let _ = std::fs::remove_dir_all(&base);
    }
}
