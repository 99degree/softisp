//! In-memory stores for camera characteristics and learner observations.
//!
//! Provides:
//! - CameraCharacteristicsStore — static camera metadata (CCM, black level, etc.)
//! - LearnerStore — observation ring buffer with CCT-bin indexing
//!
//! These replace Android SQLite (LearnerDb) for pure-Rust operation.
//! The learner can collect observations in memory, fit regression models,
//! and optionally export/import for persistence across sessions.
//!
//! ## Disk persistence
//! LearnerStore supports file-based persistence (CSV format):
//! - `save(path)` writes current observations, trimming oldest if >1 MB
//! - `load(path)` reads observations from a previously saved CSV
//! - `set_persistence_path()` enables auto-save/auto-trim on add()

use std::collections::HashMap;

/// Maximum file size for learner persistence (1 MB).
/// When saving, observations are trimmed oldest-first until the
/// serialized CSV fits under this limit.
pub const MAX_PERSISTENCE_FILE_BYTES: u64 = 1024 * 1024;

// ═══════════════════════════════════════════════════════════════════════════
// CameraCharacteristicsStore
// ═══════════════════════════════════════════════════════════════════════════

/// Static camera characteristics that vary per sensor/camera.
///
/// These would normally come from Camera2 characteristics metadata.
/// For the learner/calibration system they serve as ground-truth targets
/// when `useHardwareReference` is enabled.
#[derive(Debug, Clone)]
pub struct CameraCharacteristics {
    /// Camera identifier (e.g., "0", "1", or "back", "front").
    pub camera_id: String,
    /// Sensor sensitivity: ISO range [min, max].
    pub iso_range: [f32; 2],
    /// Exposure time range in nanoseconds [min, max].
    pub exposure_range_ns: [i64; 2],
    /// Sensor physical size in mm [width, height].
    pub sensor_size_mm: [f32; 2],
    /// Focal lengths in mm (may be multiple for zoom lenses).
    pub focal_lengths: Vec<f32>,
    /// Sensor Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: u8,
    /// Sensor orientation in degrees (0, 90, 180, 270).
    pub orientation: i32,
    /// Hardware level (0=LIMITED, 1=FULL, 2=LEVEL_3).
    pub hardware_level: u8,

    // ── ISP calibration targets (learned per sensor) ──

    /// Black level per Bayer channel [R, Gr, Gb, B].
    pub black_level: [f32; 4],
    /// White level (sensor saturation).
    pub white_level: f32,
    /// Default CCM matrix (3x3 row-major) for D65 illuminant.
    pub default_ccm: [f32; 9],
    /// AWB gains for D65 illuminant [R, G, B].
    pub d65_awb: [f32; 3],
    /// AWB gains for A illuminant [R, G, B].
    pub a_awb: [f32; 3],
    /// Sensor gray world R/G and B/G ratios (for AWB sanity checking).
    pub sensor_gray_rg: f32,
    pub sensor_gray_bg: f32,
}

impl Default for CameraCharacteristics {
    fn default() -> Self {
        Self {
            camera_id: "default".into(),
            iso_range: [100.0, 3200.0],
            exposure_range_ns: [1_000_000, 1_000_000_000], // 1ms to 1s
            sensor_size_mm: [6.4, 4.8],
            focal_lengths: vec![4.0],
            bayer_pattern: 0,
            orientation: 0,
            hardware_level: 1,
            black_level: [64.0; 4],
            white_level: 1023.0,
            default_ccm: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            d65_awb: [1.0, 1.0, 1.0],
            a_awb: [1.5, 1.0, 0.8],
            sensor_gray_rg: 1.0,
            sensor_gray_bg: 1.0,
        }
    }
}

/// In-memory store for camera characteristics.
#[derive(Debug, Clone)]
pub struct CameraCharacteristicsStore {
    /// Per-camera characteristics.
    cameras: HashMap<String, CameraCharacteristics>,
}

impl Default for CameraCharacteristicsStore {
    fn default() -> Self {
        Self::new()
    }
}

impl CameraCharacteristicsStore {
    pub fn new() -> Self {
        Self { cameras: HashMap::new() }
    }

    /// Register a camera's characteristics.
    pub fn register(&mut self, chars: CameraCharacteristics) {
        let id = chars.camera_id.clone();
        self.cameras.insert(id, chars);
    }

    /// Get characteristics for a specific camera.
    pub fn get(&self, camera_id: &str) -> Option<&CameraCharacteristics> {
        self.cameras.get(camera_id)
    }

    /// Get or create default characteristics for a camera.
    pub fn get_or_default(&mut self, camera_id: &str) -> CameraCharacteristics {
        self.cameras.get(camera_id).cloned().unwrap_or_else(|| {
            let mut chars = CameraCharacteristics::default();
            chars.camera_id = camera_id.to_string();
            self.cameras.insert(camera_id.to_string(), chars.clone());
            chars
        })
    }

    /// Update black level for a camera (from calibration).
    pub fn set_black_level(&mut self, camera_id: &str, bl: [f32; 4]) {
        if let Some(c) = self.cameras.get_mut(camera_id) {
            c.black_level = bl;
        }
    }

    /// Update white level for a camera.
    pub fn set_white_level(&mut self, camera_id: &str, wl: f32) {
        if let Some(c) = self.cameras.get_mut(camera_id) {
            c.white_level = wl;
        }
    }

    /// Remove a camera.
    pub fn remove(&mut self, camera_id: &str) {
        self.cameras.remove(camera_id);
    }

    /// Number of cameras stored.
    pub fn len(&self) -> usize {
        self.cameras.len()
    }

    /// Whether the store is empty.
    pub fn is_empty(&self) -> bool {
        self.cameras.is_empty()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LearnerStore
// ═══════════════════════════════════════════════════════════════════════════

/// A single observation for the calibration learner.
///
/// Matches StatsLearner.Observation from the Java codebase.
#[derive(Debug, Clone)]
pub struct LearnerObservation {
    /// Pipeline RGB channel means [0..1].
    pub r: f32,
    pub g: f32,
    pub b: f32,
    /// Pipeline mean luminance [0..1].
    pub lum: f32,
    /// Pipeline estimated CCT in Kelvin.
    pub cct: u32,
    /// R gain from HW AWB (or our AWB if HW unavailable).
    pub hw_awb_rg: f32,
    /// B gain from HW AWB (or our AWB if HW unavailable).
    pub hw_awb_bg: f32,
    /// CCM diagonal R element from HW (or ours).
    pub hw_ccm_diag_r: f32,
    /// CCM diagonal G element from HW (or ours).
    pub hw_ccm_diag_g: f32,
    /// CCM diagonal B element from HW (or ours).
    pub hw_ccm_diag_b: f32,
    /// Display gamma from HW (or ours).
    pub hw_gamma: f32,
    /// Exposure time in milliseconds.
    pub hw_exp_ms: f32,
    /// Analog gain.
    pub hw_analog_gain: f32,
    /// LSC polynomial coefficient k1.
    pub lsc_k1: f32,
    /// LSC polynomial coefficient k2.
    pub lsc_k2: f32,
    /// Bayer pattern (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR).
    pub bayer_pattern: u8,
    /// Monotonic frame counter.
    pub frame_idx: u64,
    /// Timestamp in milliseconds.
    pub timestamp_ms: u64,
}

impl LearnerObservation {
    /// Convert to RegressionModel Observation for model fitting.
    pub fn to_regression_obs(&self) -> crate::regression::Observation {
        crate::regression::Observation {
            r: self.r,
            g: self.g,
            b: self.b,
            lum: self.lum,
            cct: self.cct,
            hw_awb_rg: self.hw_awb_rg,
            hw_awb_bg: self.hw_awb_bg,
            hw_ccm_diag_r: self.hw_ccm_diag_r,
            hw_ccm_diag_g: self.hw_ccm_diag_g,
            hw_ccm_diag_b: self.hw_ccm_diag_b,
            hw_gamma: self.hw_gamma,
            lsc_k1: self.lsc_k1,
            lsc_k2: self.lsc_k2,
            hw_exp_ms: self.hw_exp_ms,
            hw_analog_gain: self.hw_analog_gain,
        }
    }
}

/// In-memory ring buffer for learner observations with CCT-bin indexing.
#[derive(Debug, Clone)]
pub struct LearnerStore {
    /// Maximum number of observations to retain.
    max_observations: usize,
    /// Ring buffer of observations (sorted by frame_idx).
    observations: Vec<LearnerObservation>,
    /// CCT-bin index: bin_key → observation indices.
    cct_bins: HashMap<u32, Vec<usize>>,
    /// Frame counter for ordering.
    next_frame_idx: u64,
    /// Camera identifier associated with this store.
    camera_id: String,
    /// Optional file path for disk persistence.
    persistence_path: Option<String>,
}

impl LearnerStore {
    /// Create a new learner store with a given capacity.
    pub fn new(camera_id: &str, max_observations: usize) -> Self {
        Self {
            max_observations,
            observations: Vec::with_capacity(max_observations.min(100)),
            cct_bins: HashMap::new(),
            next_frame_idx: 0,
            camera_id: camera_id.to_string(),
            persistence_path: None,
        }
    }

    /// Camera ID.
    pub fn camera_id(&self) -> &str {
        &self.camera_id
    }

    /// Number of observations stored.
    pub fn len(&self) -> usize {
        self.observations.len()
    }

    /// Whether the store is empty.
    pub fn is_empty(&self) -> bool {
        self.observations.is_empty()
    }

    /// Maximum capacity.
    pub fn capacity(&self) -> usize {
        self.max_observations
    }

    /// Compute the CCT bin key (500K bins matching FastPredictor).
    fn cct_bin_key(cct: u32) -> u32 {
        let clamped = cct.clamp(2000, 8000 - 1);
        ((clamped - 2000) / 500) * 500 + 2000
    }

    /// Add an observation.
    pub fn add(&mut self, mut obs: LearnerObservation) {
        obs.frame_idx = self.next_frame_idx;
        self.next_frame_idx += 1;

        // Evict oldest if at capacity
        self.evict_if_full();

        // Index by CCT bin
        let bin = Self::cct_bin_key(obs.cct);
        let idx = self.observations.len();
        self.cct_bins.entry(bin).or_default().push(idx);

        self.observations.push(obs);

        // Auto-save to disk if persistence is enabled
        if self.persistence_path.is_some() {
            let _ = self.save();
        }
    }

    /// Get all observations for a given CCT bin.
    pub fn get_by_cct_bin(&self, cct: u32) -> Vec<&LearnerObservation> {
        let bin = Self::cct_bin_key(cct);
        match self.cct_bins.get(&bin) {
            Some(indices) => indices.iter()
                .filter_map(|&i| self.observations.get(i))
                .collect(),
            None => Vec::new(),
        }
    }

    /// Get all observations.
    pub fn all(&self) -> &[LearnerObservation] {
        &self.observations
    }

    /// Get the most recent observation.
    pub fn latest(&self) -> Option<&LearnerObservation> {
        self.observations.last()
    }

    /// Get the oldest observation.
    pub fn oldest(&self) -> Option<&LearnerObservation> {
        self.observations.first()
    }

    /// Get observations sorted by frame index (ascending).
    pub fn sorted(&self) -> Vec<&LearnerObservation> {
        let mut sorted: Vec<&LearnerObservation> = self.observations.iter().collect();
        sorted.sort_by_key(|o| o.frame_idx);
        sorted
    }

    /// Convert all observations to RegressionModel observations.
    pub fn to_regression_observations(&self) -> Vec<crate::regression::Observation> {
        self.observations.iter().map(|o| o.to_regression_obs()).collect()
    }

    /// Number of distinct CCT bins populated.
    pub fn populated_bin_count(&self) -> usize {
        self.cct_bins.len()
    }

    // ── Persistence (disk) ──

    /// Enable disk persistence to the given file path.
    /// Observations are auto-saved to this path after every `add()`.
    /// If the file exists, observations are loaded from it.
    /// Returns the number of observations loaded, or 0 if file didn't exist.
    pub fn set_persistence_path(&mut self, path: &str) -> std::io::Result<usize> {
        let loaded = if std::path::Path::new(path).exists() {
            self.load(path)?
        } else {
            0
        };
        self.persistence_path = Some(path.to_string());
        Ok(loaded)
    }

    /// Save observations to the configured persistence path.
    /// Trims oldest observations if the CSV exceeds `MAX_PERSISTENCE_FILE_BYTES`.
    pub fn save(&self) -> std::io::Result<()> {
        let path = match &self.persistence_path {
            Some(p) => p.clone(),
            None => return Ok(()),
        };
        self.save_to(&path)
    }

    /// Save observations to a specific file path.
    /// Trims oldest observations if the CSV would exceed `MAX_PERSISTENCE_FILE_BYTES`.
    pub fn save_to(&self, path: &str) -> std::io::Result<()> {
        let csv = self.to_csv();
        let bytes = csv.as_bytes();

        if bytes.len() as u64 > MAX_PERSISTENCE_FILE_BYTES {
            // Need to trim — create a trimmed copy and save that
            let mut trimmed = self.clone();
            while trimmed.estimate_csv_size() > MAX_PERSISTENCE_FILE_BYTES as usize
                && trimmed.len() > 1
            {
                trimmed.remove_oldest();
            }
            let trimmed_csv = trimmed.to_csv();
            std::fs::write(path, trimmed_csv.as_bytes())?;
        } else {
            std::fs::write(path, bytes)?;
        }
        Ok(())
    }

    /// Load observations from a CSV file.
    /// Returns the number of observations loaded.
    pub fn load(&mut self, path: &str) -> std::io::Result<usize> {
        let content = std::fs::read_to_string(path)?;
        if content.trim().is_empty() {
            return Ok(0);
        }
        let count = self.from_csv(&content);
        Ok(count)
    }

    /// Estimate the byte size of the CSV output without allocating the full string.
    /// Used for pre-trimming before save.
    fn estimate_csv_size(&self) -> usize {
        // Each observation line is roughly 120 bytes + 20 for the header
        if self.observations.is_empty() {
            return 80; // header only
        }
        100 + self.observations.len() * 120
    }

    /// Remove the oldest observation.
    fn remove_oldest(&mut self) {
        if self.observations.is_empty() {
            return;
        }
        // Remove from CCT-bin index
        if let Some(oldest) = self.observations.first() {
            let bin = Self::cct_bin_key(oldest.cct);
            if let Some(indices) = self.cct_bins.get_mut(&bin) {
                indices.retain(|&i| i != 0);
            }
        }
        self.observations.remove(0);
        // Shift all indices in cct_bins
        for indices in self.cct_bins.values_mut() {
            for i in indices.iter_mut() {
                *i = i.saturating_sub(1);
            }
        }
    }

    /// Evict oldest observations if at capacity.
    fn evict_if_full(&mut self) {
        while self.observations.len() >= self.max_observations {
            self.remove_oldest();
        }
    }

    /// Clear all observations.
    pub fn clear(&mut self) {
        self.observations.clear();
        self.cct_bins.clear();
        self.next_frame_idx = 0;
    }

    // ── Export / Import ──

    /// Export observations as a CSV string.
    ///
    /// Format: frame_idx,timestamp,scene,cct,lum,r,g,b,
    ///         hw_awb_rg,hw_awb_bg,hw_ccm_r,hw_ccm_g,hw_ccm_b,
    ///         hw_gamma,exp_ms,analog_gain,lsc_k1,lsc_k2
    pub fn to_csv(&self) -> String {
        let mut csv = String::new();
        csv.push_str("frame_idx,timestamp,cct,lum,r,g,b,hw_awb_rg,hw_awb_bg,hw_ccm_r,hw_ccm_g,hw_ccm_b,hw_gamma,exp_ms,analog_gain,lsc_k1,lsc_k2\n");
        for obs in &self.observations {
            csv.push_str(&format!(
                "{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.2},{:.4},{:.6},{:.6}\n",
                obs.frame_idx, obs.timestamp_ms, obs.cct, obs.lum, obs.r, obs.g, obs.b,
                obs.hw_awb_rg, obs.hw_awb_bg,
                obs.hw_ccm_diag_r, obs.hw_ccm_diag_g, obs.hw_ccm_diag_b,
                obs.hw_gamma, obs.hw_exp_ms, obs.hw_analog_gain,
                obs.lsc_k1, obs.lsc_k2,
            ));
        }
        csv
    }

    /// Parse a CSV line into a `LearnerObservation`.
    ///
    /// Expected columns (17):
    /// frame_idx,timestamp,cct,lum,r,g,b,
    /// hw_awb_rg,hw_awb_bg,hw_ccm_r,hw_ccm_g,hw_ccm_b,
    /// hw_gamma,exp_ms,analog_gain,lsc_k1,lsc_k2
    pub fn from_csv_line(line: &str) -> Option<LearnerObservation> {
        let parts: Vec<&str> = line.trim().split(',').collect();
        if parts.len() < 17 {
            return None;
        }
        let idx: u64 = parts[0].parse().ok()?;
        let ts: u64 = parts[1].parse().ok()?;
        let cct: u32 = parts[2].parse().ok()?;
        let lum: f32 = parts[3].parse().ok()?;
        let r: f32 = parts[4].parse().ok()?;
        let g: f32 = parts[5].parse().ok()?;
        let b: f32 = parts[6].parse().ok()?;
        let hw_rg: f32 = parts[7].parse().ok()?;
        let hw_bg: f32 = parts[8].parse().ok()?;
        let hw_ccm_r: f32 = parts[9].parse().ok()?;
        let hw_ccm_g: f32 = parts[10].parse().ok()?;
        let hw_ccm_b: f32 = parts[11].parse().ok()?;
        let hw_gamma: f32 = parts[12].parse().ok()?;
        let exp_ms: f32 = parts[13].parse().ok()?;
        let analog: f32 = parts[14].parse().ok()?;
        let lsc_k1: f32 = parts[15].parse().ok()?;
        let lsc_k2: f32 = parts[16].parse().ok()?;

        Some(LearnerObservation {
            r, g, b, lum, cct,
            hw_awb_rg: hw_rg,
            hw_awb_bg: hw_bg,
            hw_ccm_diag_r: hw_ccm_r,
            hw_ccm_diag_g: hw_ccm_g,
            hw_ccm_diag_b: hw_ccm_b,
            hw_gamma,
            hw_exp_ms: exp_ms,
            hw_analog_gain: analog,
            lsc_k1, lsc_k2,
            bayer_pattern: 0,
            frame_idx: idx,
            timestamp_ms: ts,
        })
    }

    /// Import observations from a CSV string.
    /// Returns the number of observations imported.
    pub fn from_csv(&mut self, csv: &str) -> usize {
        let mut count = 0;
        for line in csv.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("frame_idx") {
                continue; // skip header and empty lines
            }
            if let Some(obs) = Self::from_csv_line(trimmed) {
                self.add(obs);
                count += 1;
            }
        }
        count
    }

    // ── Summary stats ──

    /// Compute scene diversity score (0..1) based on number of distinct CCT bins.
    pub fn diversity_score(&self) -> f32 {
        if self.observations.is_empty() {
            return 0.0;
        }
        let max_bins = ((8000 - 2000) / 500 + 1) as f32; // 13 bins
        (self.cct_bins.len() as f32 / max_bins).clamp(0.0, 1.0)
    }

    /// Mean CCT across all observations.
    pub fn mean_cct(&self) -> f32 {
        if self.observations.is_empty() {
            return 5500.0;
        }
        let sum: u64 = self.observations.iter().map(|o| o.cct as u64).sum();
        sum as f32 / self.observations.len() as f32
    }

    /// Mean luminance across all observations.
    pub fn mean_lum(&self) -> f32 {
        if self.observations.is_empty() {
            return 0.5;
        }
        self.observations.iter().map(|o| o.lum as f64).sum::<f64>() as f32 / self.observations.len() as f32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_obs(cct: u32, lum: f32) -> LearnerObservation {
        LearnerObservation {
            r: 0.5, g: 0.5, b: 0.5, lum, cct,
            hw_awb_rg: 1.5, hw_awb_bg: 1.2,
            hw_ccm_diag_r: 1.0, hw_ccm_diag_g: 1.0, hw_ccm_diag_b: 1.0,
            hw_gamma: 2.2, hw_exp_ms: 10.0, hw_analog_gain: 1.0,
            lsc_k1: 0.01, lsc_k2: -0.0001,
            bayer_pattern: 0, frame_idx: 0, timestamp_ms: 0,
        }
    }

    #[test]
    fn test_learner_store_add_and_count() {
        let mut store = LearnerStore::new("test_cam", 100);
        assert!(store.is_empty());
        assert_eq!(store.len(), 0);

        store.add(make_obs(5500, 0.5));
        assert_eq!(store.len(), 1);
        assert!(!store.is_empty());
    }

    #[test]
    fn test_learner_store_max_capacity() {
        let mut store = LearnerStore::new("test_cam", 5);
        for _ in 0..10 {
            store.add(make_obs(5500, 0.5));
        }
        assert_eq!(store.len(), 5); // should be capped at 5
    }

    #[test]
    fn test_cct_bin_index() {
        let mut store = LearnerStore::new("test_cam", 100);
        store.add(make_obs(5500, 0.5));
        store.add(make_obs(5500, 0.6));
        store.add(make_obs(3000, 0.3));

        let bin_55 = store.get_by_cct_bin(5500);
        assert_eq!(bin_55.len(), 2);

        let bin_30 = store.get_by_cct_bin(3000);
        assert_eq!(bin_30.len(), 1);

        assert_eq!(store.populated_bin_count(), 2);
    }

    #[test]
    fn test_latest_oldest() {
        let mut store = LearnerStore::new("test_cam", 100);
        let obs1 = make_obs(5500, 0.5);
        let mut obs2 = make_obs(5500, 0.6);
        obs2.timestamp_ms = 100;
        store.add(obs1);
        store.add(obs2);

        assert!(store.latest().is_some());
        assert!((store.latest().unwrap().lum - 0.6).abs() < 0.01);
        assert!((store.oldest().unwrap().lum - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_sorted() {
        let mut store = LearnerStore::new("test_cam", 100);
        let mut obs1 = make_obs(5500, 0.5);
        obs1.timestamp_ms = 50;
        let mut obs2 = make_obs(5500, 0.6);
        obs2.timestamp_ms = 10;
        let mut obs3 = make_obs(5500, 0.3);
        obs3.timestamp_ms = 30;
        store.add(obs1);
        store.add(obs2);
        store.add(obs3);

        // frame_idx is auto-assigned by add(): 0, 1, 2
        let sorted = store.sorted();
        assert_eq!(sorted[0].frame_idx, 0);
        assert_eq!(sorted[1].frame_idx, 1);
        assert_eq!(sorted[2].frame_idx, 2);
    }

    #[test]
    fn test_csv_roundtrip() {
        let mut store = LearnerStore::new("test_cam", 100);
        store.add(make_obs(5500, 0.5));
        store.add(make_obs(3000, 0.2));

        let csv = store.to_csv();
        assert!(csv.contains("5500"));
        assert!(csv.contains("3000"));

        let mut store2 = LearnerStore::new("test_cam", 100);
        let count = store2.from_csv(&csv);
        assert_eq!(count, 2);
        assert_eq!(store2.len(), 2);
    }

    #[test]
    fn test_diversity_score() {
        let mut store = LearnerStore::new("test_cam", 100);
        assert!((store.diversity_score() - 0.0).abs() < 0.01);

        store.add(make_obs(5500, 0.5));
        store.add(make_obs(5500, 0.6));
        // Single bin → low diversity
        assert!(store.diversity_score() > 0.0 && store.diversity_score() < 0.2);

        store.add(make_obs(3000, 0.3));
        store.add(make_obs(6500, 0.7));
        // Multiple bins → higher diversity
        assert!(store.diversity_score() > 0.15);
    }

    #[test]
    fn test_clear() {
        let mut store = LearnerStore::new("test_cam", 100);
        store.add(make_obs(5500, 0.5));
        store.add(make_obs(3000, 0.3));
        assert_eq!(store.len(), 2);

        store.clear();
        assert_eq!(store.len(), 0);
        assert_eq!(store.populated_bin_count(), 0);
    }

    // ── CameraCharacteristicsStore tests ──

    #[test]
    fn test_camera_chars_store() {
        let mut store = CameraCharacteristicsStore::new();
        assert!(store.is_empty());

        let chars = CameraCharacteristics {
            camera_id: "back".into(),
            iso_range: [50.0, 6400.0],
            ..Default::default()
        };
        store.register(chars);

        assert_eq!(store.len(), 1);
        let fetched = store.get("back");
        assert!(fetched.is_some());
        assert!((fetched.unwrap().iso_range[1] - 6400.0).abs() < 0.01);
    }

    #[test]
    fn test_camera_chars_get_or_default() {
        let mut store = CameraCharacteristicsStore::new();
        let chars = store.get_or_default("front");
        assert_eq!(chars.camera_id, "front");
        assert_eq!(store.len(), 1);
    }

    #[test]
    fn test_camera_chars_update() {
        let mut store = CameraCharacteristicsStore::new();
        store.register(CameraCharacteristics {
            camera_id: "cam".into(),
            ..Default::default()
        });

        store.set_black_level("cam", [128.0; 4]);
        let chars = store.get("cam").unwrap();
        assert!((chars.black_level[0] - 128.0).abs() < 0.01);
    }

    // ── Persistence tests ──

    #[test]
    fn test_persistence_roundtrip() {
        let mut store = LearnerStore::new("persist_cam", 100);
        store.add(make_obs(5500, 0.5));
        store.add(make_obs(3000, 0.2));
        store.add(make_obs(6500, 0.8));
        assert_eq!(store.len(), 3);

        let tmp = std::env::temp_dir().join("learner_test.csv");
        let path = tmp.to_str().unwrap().to_string();
        store.save_to(&path).expect("Save should succeed");

        let mut loaded = LearnerStore::new("persist_cam", 100);
        let count = loaded.load(&path).expect("Load should succeed");
        assert_eq!(count, 3);
        assert_eq!(loaded.len(), 3);

        // Clean up
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_persistence_empty_store() {
        let store = LearnerStore::new("empty", 100);
        let tmp = std::env::temp_dir().join("learner_empty.csv");
        let path = tmp.to_str().unwrap().to_string();
        store.save_to(&path).expect("Save empty should succeed");

        let mut loaded = LearnerStore::new("empty", 100);
        let count = loaded.load(&path).expect("Load empty should succeed");
        assert_eq!(count, 0);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_persistence_auto_trim() {
        let max_obs = MAX_PERSISTENCE_FILE_BYTES as usize / 120 + 10; // enough to overflow 1MB
        let mut store = LearnerStore::new("trim_test", max_obs);

        // Fill with enough observations to exceed 1 MB
        for i in 0..5000 {
            let mut obs = make_obs(5500, 0.5);
            obs.frame_idx = i;
            store.add(obs);
        }

        // Save should trim to fit under 1 MB
        let tmp = std::env::temp_dir().join("learner_trim.csv");
        let path = tmp.to_str().unwrap().to_string();
        store.save_to(&path).expect("Save trim should succeed");

        let metadata = std::fs::metadata(&path).expect("File should exist");
        assert!(
            metadata.len() <= MAX_PERSISTENCE_FILE_BYTES + 512,
            "File size {} exceeds limit {}",
            metadata.len(), MAX_PERSISTENCE_FILE_BYTES
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn test_set_persistence_path_creates_file() {
        let mut store = LearnerStore::new("auto_persist", 50);
        let tmp = std::env::temp_dir().join("learner_auto.csv");
        let path = tmp.to_str().unwrap().to_string();

        // Set persistence path (file doesn't exist yet)
        let loaded = store.set_persistence_path(&path).expect("set_persistence_path");
        assert_eq!(loaded, 0); // nothing to load

        // Add observations — auto-saves after each
        store.add(make_obs(5500, 0.5));
        store.add(make_obs(3000, 0.3));

        // Verify file was created
        assert!(tmp.exists(), "Persistence file should exist after add");
        let content = std::fs::read_to_string(&path).expect("Read should work");
        assert!(content.contains("5500"));
        assert!(content.contains("3000"));

        // Clean up
        let _ = std::fs::remove_file(&path);
    }
}
