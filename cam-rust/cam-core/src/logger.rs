//! Logger module for the camera pipeline.
//! Ported from com.camcore.Logger

use log::{info, LevelFilter};
use std::sync::Once;

static INIT: Once = Once::new();

/// Initialize the logger for Android or standard output.
pub fn init_logger(verbose: bool) {
    INIT.call_once(|| {
        let level = if verbose { LevelFilter::Debug } else { LevelFilter::Info };
        env_logger::Builder::new()
            .filter_level(level)
            .format_timestamp_millis()
            .init();
        info!("Logger initialized (verbose={})", verbose);
    });
}
