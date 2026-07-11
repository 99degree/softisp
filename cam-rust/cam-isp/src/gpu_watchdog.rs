//! GPU Hang Watchdog
//!
//! Monitors GPU execution time and triggers fallback to CPU if timeout exceeded.
//! Prevents application hangs when GPU drivers misbehave.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// GPU execution state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpuState {
    /// GPU is idle
    Idle,
    /// GPU is executing
    Executing,
    /// GPU appears hung
    Hung,
    /// GPU timed out, fallback to CPU
    Timeout,
}

/// Watchdog configuration
#[derive(Debug, Clone)]
pub struct WatchdogConfig {
    /// Maximum allowed GPU execution time before considering it hung
    pub timeout: Duration,
    /// Interval between health checks
    pub check_interval: Duration,
    /// Number of consecutive timeouts before triggering fallback
    pub max_timeouts: u32,
    /// Enable automatic CPU fallback
    pub auto_fallback: bool,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            timeout: Duration::from_secs(5),           // 5 seconds
            check_interval: Duration::from_millis(100), // 100ms
            max_timeouts: 3,
            auto_fallback: true,
        }
    }
}

/// GPU hang watchdog
pub struct GpuWatchdog {
    config: WatchdogConfig,
    state: Arc<AtomicU64>,  // GpuState as u64
    start_time: Arc<AtomicU64>,  // Instant as u64 (nanos since epoch)
    timeout_count: Arc<AtomicU32>,
    fallback_requested: Arc<AtomicBool>,
    running: Arc<AtomicBool>,
}

impl GpuWatchdog {
    /// Create a new watchdog with the given configuration
    pub fn new(config: WatchdogConfig) -> Self {
        Self {
            config,
            state: Arc::new(AtomicU64::new(GpuState::Idle as u64)),
            start_time: Arc::new(AtomicU64::new(0)),
            timeout_count: Arc::new(AtomicU32::new(0)),
            fallback_requested: Arc::new(AtomicBool::new(false)),
            running: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Start monitoring GPU execution
    pub fn begin_execution(&self) {
        self.state.store(GpuState::Executing as u64, Ordering::SeqCst);
        self.start_time.store(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            Ordering::SeqCst,
        );
    }

    /// Mark GPU execution as complete
    pub fn end_execution(&self) {
        self.state.store(GpuState::Idle as u64, Ordering::SeqCst);
        self.timeout_count.store(0, Ordering::SeqCst);
    }

    /// Get current GPU state
    pub fn state(&self) -> GpuState {
        match self.state.load(Ordering::SeqCst) as u32 {
            0 => GpuState::Idle,
            1 => GpuState::Executing,
            2 => GpuState::Hung,
            3 => GpuState::Timeout,
            _ => GpuState::Idle,
        }
    }

    /// Check if CPU fallback is requested
    pub fn fallback_requested(&self) -> bool {
        self.fallback_requested.load(Ordering::SeqCst)
    }

    /// Clear fallback request (e.g., after successful CPU execution)
    pub fn clear_fallback(&self) {
        self.fallback_requested.store(false, Ordering::SeqCst);
        self.timeout_count.store(0, Ordering::SeqCst);
    }

    /// Run the watchdog in a background thread
    pub fn start_monitoring(&self) {
        if self.running.swap(true, Ordering::SeqCst) {
            return; // Already running
        }

        let state = self.state.clone();
        let start_time = self.start_time.clone();
        let timeout_count = self.timeout_count.clone();
        let fallback_requested = self.fallback_requested.clone();
        let running = self.running.clone();
        let config = self.config.clone();

        std::thread::spawn(move || {
            while running.load(Ordering::SeqCst) {
                std::thread::sleep(config.check_interval);

                if state.load(Ordering::SeqCst) != GpuState::Executing as u64 {
                    continue;
                }

                // Check execution time
                let start_ns = start_time.load(Ordering::SeqCst);
                let now_ns = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;
                
                let elapsed = Duration::from_nanos(now_ns.saturating_sub(start_ns));

                if elapsed > config.timeout {
                    state.store(GpuState::Hung as u64, Ordering::SeqCst);
                    
                    let count = timeout_count.fetch_add(1, Ordering::SeqCst) + 1;
                    log::warn!(
                        "GPU hang detected! Execution time: {:?}, timeout count: {}/{}",
                        elapsed, count, config.max_timeouts
                    );

                    if count >= config.max_timeouts && config.auto_fallback {
                        fallback_requested.store(true, Ordering::SeqCst);
                        state.store(GpuState::Timeout as u64, Ordering::SeqCst);
                        log::error!("GPU fallback to CPU requested after {} timeouts", count);
                    }
                }
            }
        });
    }

    /// Stop the watchdog
    pub fn stop_monitoring(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

impl Drop for GpuWatchdog {
    fn drop(&mut self) {
        self.stop_monitoring();
    }
}

/// Execute a GPU operation with watchdog protection
pub fn execute_with_watchdog<F, T>(
    watchdog: &GpuWatchdog,
    gpu_op: F,
    fallback_op: impl FnOnce() -> T,
) -> T
where
    F: FnOnce() -> T,
{
    if watchdog.fallback_requested() {
        log::info!("Using CPU fallback due to previous GPU timeout");
        return fallback_op();
    }

    watchdog.begin_execution();
    let result = gpu_op();
    watchdog.end_execution();

    if watchdog.fallback_requested() {
        log::warn!("GPU operation triggered timeout, will use CPU fallback next time");
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_watchdog_initial_state() {
        let watchdog = GpuWatchdog::new(WatchdogConfig::default());
        assert_eq!(watchdog.state(), GpuState::Idle);
        assert!(!watchdog.fallback_requested());
    }

    #[test]
    fn test_watchdog_execution_cycle() {
        let watchdog = GpuWatchdog::new(WatchdogConfig::default());
        watchdog.begin_execution();
        assert_eq!(watchdog.state(), GpuState::Executing);
        watchdog.end_execution();
        assert_eq!(watchdog.state(), GpuState::Idle);
    }

    #[test]
    fn test_execute_with_watchdog_success() {
        let watchdog = GpuWatchdog::new(WatchdogConfig::default());
        let result = execute_with_watchdog(
            &watchdog,
            || 42,
            || 0,
        );
        assert_eq!(result, 42);
    }

    #[test]
    fn test_execute_with_watchdog_fallback() {
        let watchdog = GpuWatchdog::new(WatchdogConfig::default());
        watchdog.clear_fallback();
        
        // Simulate timeout by directly setting fallback
        watchdog.fallback_requested.store(true, Ordering::SeqCst);
        
        let result = execute_with_watchdog(
            &watchdog,
            || 42,
            || 0,
        );
        assert_eq!(result, 0); // Fallback was used
    }
}
