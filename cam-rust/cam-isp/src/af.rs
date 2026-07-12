//! AF (Autofocus) Engine — coarse→fine software scan with VCM/LSC integration.
//!
//! Ported from `com.camcore.isp.pipeline.controller.AfEngine` (Java, 367 lines).
//!
//! Provides:
//! - AF modes: ContinuousPicture, ContinuousVideo, Auto, Macro, Manual, Off
//! - Coarse→fine scan lifecycle (start → advance → feed metric → settle)
//! - VCM↔diopter conversion
//! - Focus metric extraction from calibration data
//! - Peak detection with neighbor smoothing

/// VCM actuator range constants (0-1023).
pub mod vcm {
    pub const MIN: i32 = 0;
    pub const MAX: i32 = 1023;
    pub const RANGE: i32 = MAX - MIN; // 1024
    pub const COARSE_STEP: i32 = 64; // ~16 coarse positions
    pub const FINE_RADIUS: i32 = 48; // fine scan ±48 around peak
    pub const FINE_STEP: i32 = 16; // fine scan step size
}

/// AF mode enum — matches Android Camera2 AF modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfMode {
    ContinuousPicture,
    ContinuousVideo,
    Auto,
    Macro,
    Manual,
    Off,
}

impl AfMode {
    pub fn from_name(s: &str) -> Self {
        match s {
            "CONTINUOUS_PICTURE" | "AF-C" => Self::ContinuousPicture,
            "CONTINUOUS_VIDEO" | "AF-V" => Self::ContinuousVideo,
            "AUTO" | "AF-A" => Self::Auto,
            "MACRO" | "AF-M" => Self::Macro,
            "MANUAL" | "AF-MAN" => Self::Manual,
            _ => Self::Off,
        }
    }

    pub fn display_name(&self) -> &str {
        match self {
            Self::ContinuousPicture => "AF-C",
            Self::ContinuousVideo => "AF-V",
            Self::Auto => "AF-A",
            Self::Macro => "AF-M",
            Self::Manual => "AF-MAN",
            Self::Off => "AF-OFF",
        }
    }
}

/// Phase of the software AF coarse→fine scan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfScanPhase {
    Idle,
    CoarseScan,
    FineScan,
    Settled,
}

impl AfScanPhase {
    pub fn display(&self) -> &str {
        match self {
            Self::Idle => "",
            Self::CoarseScan => "COARSE",
            Self::FineScan => "FINE",
            Self::Settled => "✓DONE",
        }
    }
}

/// AF state — container for all autofocus state machine fields.
#[derive(Debug, Clone)]
pub struct AfState {
    /// Current AF mode.
    pub mode: AfMode,
    /// Current scan phase.
    pub scan_phase: AfScanPhase,
    /// VCM positions for the current scan (coarse or fine).
    pub scan_positions: Vec<i32>,
    /// Current index into scan_positions.
    pub scan_step: usize,
    /// Total steps in current scan.
    pub scan_total: usize,
    /// Per-position focus metrics collected so far.
    pub scan_metrics: Vec<(i32, f32)>,
    /// VCM position of the coarse peak.
    pub scan_coarse_peak: i32,
    /// Best VCM position found across all scans.
    pub scan_best_pos: i32,
    /// Best focus metric found.
    pub scan_best_metric: f32,
    /// Current VCM position.
    pub vcm_pos: i32,
    /// AF state code (0=INACTIVE, 1=SCAN, 2=FOCUSED, 3=AF_SCAN, 4=LOCKED, 5=UNFOCUSED, 6=PASSIVE_UNFOC).
    pub af_state: i32,
    /// Whether AF is locked.
    pub af_locked: bool,
    /// Focus distance in diopters.
    pub focus_distance: f32,
}

impl Default for AfState {
    fn default() -> Self {
        Self {
            mode: AfMode::ContinuousPicture,
            scan_phase: AfScanPhase::Idle,
            scan_positions: Vec::new(),
            scan_step: 0,
            scan_total: 0,
            scan_metrics: Vec::new(),
            scan_coarse_peak: -1,
            scan_best_pos: -1,
            scan_best_metric: -1.0,
            vcm_pos: -1,
            af_state: 0,
            af_locked: false,
            focus_distance: 0.0,
        }
    }
}

impl AfState {
    /// Convert AF state code to string label.
    pub fn state_label(state: i32) -> &'static str {
        match state {
            0 => "INACTIVE",
            1 => "SCAN",
            2 => "FOCUSED",
            3 => "AF_SCAN",
            4 => "LOCKED",
            5 => "UNFOCUSED",
            6 => "PASSIVE_UNFOC",
            _ => "",
        }
    }

    /// One-line AF status string for UI overlay.
    pub fn display_string(&self) -> String {
        if self.scan_phase != AfScanPhase::Idle {
            let pct = self.scan_step * 100 / self.scan_total.max(1);
            return format!(
                "AF {} {}% vcm={}",
                self.scan_phase.display(),
                pct,
                self.scan_best_pos,
            );
        }
        let state_str = Self::state_label(self.af_state);
        if self.af_locked {
            format!("🔒 AF-LOCK {} {}", self.mode.display_name(), state_str)
        } else {
            format!(
                "AF {} {} dist={:.2}m",
                self.mode.display_name(),
                state_str,
                self.focus_distance
            )
        }
    }

    // ── VCM ↔ diopter conversion ──

    /// Convert VCM actuator position (0-1023) to focus distance in diopters.
    pub fn vcm_to_diopters(vcm_pos: i32) -> f32 {
        if vcm_pos <= vcm::MIN {
            return 0.0; // infinity
        }
        let n = (vcm_pos - vcm::MIN) as f32 / vcm::RANGE as f32;
        // n = 1 - 1/(1 + d*0.5)  →  d = (1/(1-n) - 1) / 0.5
        if n >= 0.999 {
            20.0
        } else {
            ((1.0 / (1.0 - n)) - 1.0) / 0.5
        }
    }

    /// Convert focus distance in diopters to VCM position.
    pub fn diopters_to_vcm(diopters: f32) -> i32 {
        if diopters <= 0.0 {
            return vcm::MIN;
        }
        let n = (1.0 - 1.0 / (1.0 + diopters * 0.5)).clamp(0.0, 1.0);
        ((vcm::MIN as f32 + n * vcm::RANGE as f32) as i32).clamp(vcm::MIN, vcm::MAX)
    }

    // ── Focus metric ──

    /// Compute a focus metric from calibration data.
    ///
    /// The calibration data should contain quad-level variances at indices `[4:8]`.
    /// Higher variance = more high-frequency detail = better focus.
    /// Returns the mean of 4 quad variances.
    pub fn focus_metric_from_calibration(calibration_data: &[f32]) -> f32 {
        if calibration_data.len() < 22 {
            return 0.0;
        }
        let var_sum =
            calibration_data[4] + calibration_data[5] + calibration_data[6] + calibration_data[7];
        var_sum / 4.0
    }

    // ── Scan lifecycle ──

    /// Build the list of coarse VCM positions for a full-range scan.
    fn coarse_positions() -> Vec<i32> {
        let mut pos = Vec::new();
        let mut p = vcm::MIN;
        while p <= vcm::MAX {
            pos.push(p);
            p += vcm::COARSE_STEP;
        }
        pos
    }

    /// Build the list of fine VCM positions around a peak.
    fn fine_positions(peak: i32) -> Vec<i32> {
        let lo = (peak - vcm::FINE_RADIUS).max(vcm::MIN);
        let hi = (peak + vcm::FINE_RADIUS).min(vcm::MAX);
        let mut pos = Vec::new();
        let mut p = lo;
        while p <= hi {
            pos.push(p);
            p += vcm::FINE_STEP;
        }
        pos.sort_unstable();
        pos.dedup();
        pos
    }

    /// Start a software coarse→fine AF scan.
    pub fn start_scan(&mut self) {
        let positions = Self::coarse_positions();
        self.scan_positions = positions;
        self.scan_step = 0;
        self.scan_total = self.scan_positions.len();
        self.scan_metrics.clear();
        self.scan_phase = AfScanPhase::CoarseScan;
        self.scan_coarse_peak = -1;
        self.scan_best_pos = -1;
        self.scan_best_metric = -1.0;
        log::info!(
            "AF scan START: coarse {} steps [{}, {}]",
            self.scan_total,
            self.scan_positions.first().unwrap_or(&0),
            self.scan_positions.last().unwrap_or(&0),
        );
    }

    /// Advance the AF scan state machine and return the next VCM position.
    ///
    /// Call once per frame before pipeline processing, after the previous
    /// frame's metric has been fed via `feed_metric()`.
    ///
    /// Returns the next VCM position to set, or `None` if the scan is complete.
    pub fn advance_scan(&mut self) -> Option<i32> {
        match self.scan_phase {
            AfScanPhase::Idle => return None,

            AfScanPhase::CoarseScan => {
                if self.scan_step >= self.scan_positions.len() {
                    // Coarse scan done — find peak, start fine scan
                    let peak = self.find_peak_position();
                    self.scan_coarse_peak = peak;
                    if peak < 0 {
                        log::warn!("Coarse scan: no peak found (all metrics zero?)");
                        self.scan_phase = AfScanPhase::Idle;
                        return None;
                    }
                    let fine = Self::fine_positions(peak);
                    self.scan_positions = fine;
                    self.scan_step = 0;
                    self.scan_total = self.scan_positions.len();
                    self.scan_metrics.clear();
                    self.scan_phase = AfScanPhase::FineScan;
                    log::info!(
                        "Coarse peak at VCM={}, fine scan {} steps around it",
                        peak,
                        self.scan_total
                    );
                    // Fall through to fine scan
                } else {
                    let pos = self.scan_positions[self.scan_step];
                    self.vcm_pos = pos;
                    return Some(pos);
                }
            }

            AfScanPhase::FineScan => {
                if self.scan_step >= self.scan_positions.len() {
                    // Fine scan done — settle
                    let best = self.find_best_position();
                    self.scan_best_pos = best;
                    self.scan_phase = AfScanPhase::Settled;
                    log::info!(
                        "AF scan DONE: best VCM={} (metric={:.2})",
                        best,
                        self.scan_best_metric
                    );
                    self.vcm_pos = best;
                    return None;
                } else {
                    let pos = self.scan_positions[self.scan_step];
                    self.vcm_pos = pos;
                    return Some(pos);
                }
            }

            AfScanPhase::Settled => return None,
        }

        // Fall-through from coarse→fine transition
        if self.scan_phase == AfScanPhase::FineScan && self.scan_step < self.scan_positions.len() {
            let pos = self.scan_positions[self.scan_step];
            self.vcm_pos = pos;
            return Some(pos);
        }

        None
    }

    /// Feed the focus metric for the current frame.
    ///
    /// Call AFTER pipeline processing, with the metric from
    /// `focus_metric_from_calibration()`.
    pub fn feed_metric(&mut self, metric: f32) {
        if self.scan_phase == AfScanPhase::Idle {
            return;
        }
        if self.scan_step >= self.scan_positions.len() {
            return;
        }

        let pos = self.scan_positions[self.scan_step];
        self.scan_metrics.push((pos, metric));

        if metric > self.scan_best_metric {
            self.scan_best_metric = metric;
            self.scan_best_pos = pos;
        }

        self.scan_step += 1;
    }

    // ── Peak detection ──

    /// Find the VCM position with the highest focus metric.
    fn find_peak_position(&self) -> i32 {
        self.scan_metrics
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|&(pos, _)| pos)
            .unwrap_or(-1)
    }

    /// Find the best (highest metric) position, smoothed with neighbors.
    fn find_best_position(&self) -> i32 {
        if self.scan_metrics.is_empty() {
            return -1;
        }

        // Sort by position
        let mut sorted = self.scan_metrics.clone();
        sorted.sort_by_key(|&(pos, _)| pos);

        // Smooth: average each position's metric with neighbors
        let smoothed: Vec<(i32, f32)> = sorted
            .iter()
            .enumerate()
            .map(|(i, &(pos, val))| {
                let mut sum = val;
                let mut count = 1;
                if i > 0 {
                    sum += sorted[i - 1].1;
                    count += 1;
                }
                if i < sorted.len() - 1 {
                    sum += sorted[i + 1].1;
                    count += 1;
                }
                (pos, sum / count as f32)
            })
            .collect();

        smoothed
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|&(pos, _)| pos)
            .unwrap_or(sorted.last().map(|&(pos, _)| pos).unwrap_or(-1))
    }

    // ── Hardware AF passthrough ──

    /// Update AF state from camera adapter.
    pub fn update_from_adapter(
        &mut self,
        adapter_mode: &str,
        adapter_state: i32,
        adapter_focus_distance: f32,
    ) {
        self.mode = AfMode::from_name(adapter_mode);
        self.af_state = adapter_state.clamp(-1, 6);
        self.focus_distance = adapter_focus_distance.max(0.0);
        self.af_locked = adapter_state == 4;
    }

    /// Sanitize AF parameters.
    pub fn sanitize(&mut self) -> bool {
        let mut clamped = false;
        if self.af_state < -1 {
            self.af_state = -1;
            clamped = true;
        }
        if self.af_state > 6 {
            self.af_state = 6;
            clamped = true;
        }
        if self.focus_distance < 0.0 {
            self.focus_distance = 0.0;
            clamped = true;
        }
        if clamped {
            log::warn!(
                "AF params clamped: state={} dist={:.2}",
                self.af_state,
                self.focus_distance
            );
        }
        clamped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let af = AfState::default();
        assert_eq!(af.mode, AfMode::ContinuousPicture);
        assert_eq!(af.scan_phase, AfScanPhase::Idle);
        assert_eq!(af.af_state, 0);
    }

    #[test]
    fn test_vcm_to_diopters() {
        // Infinity: VCM_MIN → 0 diopters
        assert!((AfState::vcm_to_diopters(vcm::MIN) - 0.0).abs() < 0.01);
        // Mid range: VCM ≈ 512 → ~0.5 diopters
        let d = AfState::vcm_to_diopters(512);
        assert!(d > 0.0 && d < 10.0, "VCM=512 → {} diopters", d);
        // Max: VCM_MAX → ~20 diopters
        let d = AfState::vcm_to_diopters(vcm::MAX);
        assert!((d - 20.0).abs() < 1.0, "VCM_MAX → {} diopters", d);
    }

    #[test]
    fn test_diopters_to_vcm_roundtrip() {
        let d = 0.5;
        let vcm = AfState::diopters_to_vcm(d);
        let d2 = AfState::vcm_to_diopters(vcm);
        assert!(
            (d - d2).abs() < 0.1,
            "Roundtrip: {} → VCM={} → {}",
            d,
            vcm,
            d2
        );
    }

    #[test]
    fn test_focus_metric() {
        let mut data = vec![0.0f32; 24];
        data[4] = 10.0;
        data[5] = 20.0;
        data[6] = 30.0;
        data[7] = 40.0;
        let metric = AfState::focus_metric_from_calibration(&data);
        assert!(
            (metric - 25.0).abs() < 0.01,
            "Metric should be 25.0, got {}",
            metric
        );
    }

    #[test]
    fn test_focus_metric_short_array() {
        let data = [1.0f32; 4];
        let metric = AfState::focus_metric_from_calibration(&data);
        assert!(
            (metric - 0.0).abs() < 0.01,
            "Short array should return 0, got {}",
            metric
        );
    }

    #[test]
    fn test_coarse_positions() {
        let positions = AfState::coarse_positions();
        assert!(!positions.is_empty());
        assert_eq!(positions[0], 0);
        assert!(*positions.last().unwrap() >= vcm::MAX - vcm::COARSE_STEP);
    }

    #[test]
    fn test_scan_lifecycle() {
        let mut af = AfState::default();

        // Start scan
        af.start_scan();
        assert_eq!(af.scan_phase, AfScanPhase::CoarseScan);
        assert!(af.scan_total > 0);

        // Advance through coarse scan, feeding metrics
        while let Some(pos) = af.advance_scan() {
            assert_eq!(pos, af.vcm_pos);
            // Simulate focus metric: parabolic peak at VCM=512
            let metric = 100.0 - ((pos as f32 - 512.0) / 100.0).powi(2);
            af.feed_metric(metric);
        }

        // Should settle on best position near 512
        assert_eq!(af.scan_phase, AfScanPhase::Settled);
        assert!(
            (af.scan_best_pos as f32 - 512.0).abs() < 100.0,
            "Best pos should be near 512, got {}",
            af.scan_best_pos
        );
        assert!(af.scan_best_metric > 0.0);
    }

    #[test]
    fn test_state_display_string() {
        let af = AfState::default();
        let s = af.display_string();
        assert!(s.contains("AF-C") || s.contains("AF"));
        assert!(s.contains("INACTIVE"));
    }

    #[test]
    fn test_sanitize() {
        let mut af = AfState::default();
        af.af_state = -5;
        af.focus_distance = -1.0;
        assert!(af.sanitize());
        assert_eq!(af.af_state, -1);
        assert!((af.focus_distance - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_fine_positions() {
        let positions = AfState::fine_positions(512);
        assert!(!positions.is_empty());
        // All positions should be within ±48 of 512
        for &p in &positions {
            assert!(p >= 464 && p <= 560, "Position {} out of fine range", p);
        }
    }

    #[test]
    fn test_find_peak() {
        let mut af = AfState::default();
        af.scan_metrics = vec![(0, 10.0), (64, 50.0), (128, 100.0), (192, 80.0)];
        let peak = af.find_peak_position();
        assert_eq!(peak, 128);
    }

    #[test]
    fn test_find_best_smoothed() {
        let mut af = AfState::default();
        af.scan_metrics = vec![(0, 10.0), (64, 50.0), (128, 100.0), (192, 80.0)];
        let best = af.find_best_position();
        // After neighbor smoothing, position 192 has the best
        // neighborhood average (100+80)/2 = 90 vs (50+100+80)/3 = 76.67
        assert!(
            best == 192 || best == 128,
            "Best should be 128 or 192, got {}",
            best
        );
    }
}
