//! Logger module for the camera pipeline.
//!
//! Automatically selects:
//! - Android logcat when building with `android` feature
//! - Standard env_logger otherwise
//!
//! Usage:
//! ```rust,ignore
//! use cam_core::logger::init_logger;
//! init_logger(true); // verbose = debug level
//! ```

use log::{info, LevelFilter};
use std::sync::Once;

static INIT: Once = Once::new();

/// Initialize the logger for Android (logcat) or standard output.
///
/// * `verbose` — if true, sets level to Debug; otherwise Info.
/// * `tag` — Android log tag (ignored on non-Android platforms).
pub fn init_logger(verbose: bool) {
    init_logger_with_tag(verbose, "cam_hal_rust")
}

/// Initialize logger with a custom Android log tag.
pub fn init_logger_with_tag(verbose: bool, tag: &str) {
    INIT.call_once(|| {
        let level = if verbose { LevelFilter::Debug } else { LevelFilter::Info };

        #[cfg(feature = "android")]
        {
            android_logger::init_once(
                android_logger::Config::default()
                    .with_max_level(level)
                    .with_tag(tag),
            );
            info!("Android logger initialized (verbose={}, tag={})", verbose, tag);
        }

        #[cfg(not(feature = "android"))]
        {
            let _ = tag;
            env_logger::Builder::new()
                .filter_level(level)
                .format_timestamp_millis()
                .init();
            info!("Logger initialized (verbose={})", verbose);
        }
    });
}
