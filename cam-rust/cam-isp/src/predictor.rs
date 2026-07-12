//! FastPredictor — per-CCT-bin averaging predictor for ISP parameters.
//!
//! Ported from `FastPredictor.kt` (Java, 241 lines).
//!
//! ## Algorithm
//! Observations are binned by CCT (2000K–8000K, 500K bins).
//! For each bin, maintains running averages of HW ISP parameters.
//! Prediction: find the nearest populated bin → return its averages.
//!
//! ## Properties
//! - O(1) update, O(1) predict — runs every frame if desired
//! - Converges quickly (~10–20 observations per bin)
//! - Confidence = fraction of bins populated (scene diversity)

use std::collections::HashMap;

const CCT_BIN_SIZE: i32 = 500;
const CCT_MIN: i32 = 2000;
const CCT_MAX: i32 = 8000;

/// Running average statistics for one CCT bin.
#[derive(Debug, Clone)]
struct BinStats {
    count: u32,
    sum_awb_rg: f64,
    sum_awb_bg: f64,
    sum_gamma: f64,
    sum_lsc_k1: f64,
    sum_lsc_k2: f64,
    sum_target_lum: f64,
    sum_max_iso: f64,
}

impl BinStats {
    fn new() -> Self {
        Self {
            count: 0,
            sum_awb_rg: 0.0,
            sum_awb_bg: 0.0,
            sum_gamma: 0.0,
            sum_lsc_k1: 0.0,
            sum_lsc_k2: 0.0,
            sum_target_lum: 0.0,
            sum_max_iso: 0.0,
        }
    }

    fn observe(
        &mut self,
        awb_rg: f64,
        awb_bg: f64,
        gamma: f64,
        lsc_k1: f64,
        lsc_k2: f64,
        target_lum: f64,
        max_iso: f64,
    ) {
        self.count += 1;
        self.sum_awb_rg += awb_rg;
        self.sum_awb_bg += awb_bg;
        self.sum_gamma += gamma;
        self.sum_lsc_k1 += lsc_k1;
        self.sum_lsc_k2 += lsc_k2;
        self.sum_target_lum += target_lum;
        self.sum_max_iso += max_iso;
    }

    fn avg_awb_rg(&self) -> f32 {
        (self.sum_awb_rg / self.count as f64) as f32
    }
    fn avg_awb_bg(&self) -> f32 {
        (self.sum_awb_bg / self.count as f64) as f32
    }
    fn avg_gamma(&self) -> f32 {
        (self.sum_gamma / self.count as f64) as f32
    }
    fn avg_lsc_k1(&self) -> f32 {
        (self.sum_lsc_k1 / self.count as f64) as f32
    }
    fn avg_lsc_k2(&self) -> f32 {
        (self.sum_lsc_k2 / self.count as f64) as f32
    }
    fn avg_target_lum(&self) -> f32 {
        (self.sum_target_lum / self.count as f64) as f32
    }
    fn avg_max_iso(&self) -> f32 {
        (self.sum_max_iso / self.count as f64) as f32
    }
}

/// Predicted ISP parameters for a given CCT.
#[derive(Debug, Clone, Copy)]
pub struct Prediction {
    pub awb_rg: f32,
    pub awb_bg: f32,
    pub gamma: f32,
    pub lsc_k1: f32,
    pub lsc_k2: f32,
    pub target_lum: f32,
    pub max_iso: f32,
}

/// Fast, lightweight ISP parameter predictor using per-CCT-bin averaging.
#[derive(Debug, Clone)]
pub struct FastPredictor {
    /// All bins: CCT bin key → BinStats.
    bins: HashMap<i32, BinStats>,
    /// Total observations seen.
    total_observations: u32,
}

impl Default for FastPredictor {
    fn default() -> Self {
        Self::new()
    }
}

impl FastPredictor {
    pub fn new() -> Self {
        Self {
            bins: HashMap::new(),
            total_observations: 0,
        }
    }

    /// Number of distinct CCT bins populated.
    pub fn populated_bin_count(&self) -> usize {
        self.bins.len()
    }

    /// Whether we have enough data for a meaningful prediction.
    pub fn is_ready(&self) -> bool {
        self.total_observations >= 10
    }

    /// Confidence based on bin diversity (0..1).
    pub fn confidence(&self) -> f32 {
        if self.total_observations < 10 {
            return 0.0;
        }
        let max_bins = ((CCT_MAX - CCT_MIN) / CCT_BIN_SIZE + 1) as f32;
        (self.populated_bin_count() as f32 / max_bins).clamp(0.1, 0.95)
    }

    // ── CCT bin key ──

    /// Compute the bin key for a given CCT.
    fn cct_bin(cct: i32) -> i32 {
        let clamped = cct.clamp(CCT_MIN, CCT_MAX - 1);
        ((clamped - CCT_MIN) / CCT_BIN_SIZE) * CCT_BIN_SIZE + CCT_MIN
    }

    // ── Observe ──

    /// Record one observation into the appropriate CCT bin.
    pub fn observe(
        &mut self,
        cct: i32,
        awb_rg: f32,
        awb_bg: f32,
        gamma: f32,
        lsc_k1: f32,
        lsc_k2: f32,
        target_lum: f32,
        max_iso: f32,
    ) {
        let bin = Self::cct_bin(cct);
        let stats = self.bins.entry(bin).or_insert_with(BinStats::new);
        stats.observe(
            awb_rg as f64,
            awb_bg as f64,
            gamma as f64,
            lsc_k1 as f64,
            lsc_k2 as f64,
            target_lum as f64,
            max_iso as f64,
        );
        self.total_observations += 1;
    }

    // ── Predict ──

    /// Predict ISP parameters for the given CCT by finding the nearest bin.
    pub fn predict(&self, cct: i32) -> Option<Prediction> {
        if !self.is_ready() {
            return None;
        }
        let bin_key = self.find_nearest_bin(cct)?;
        let s = self.bins.get(&bin_key)?;
        Some(Prediction {
            awb_rg: s.avg_awb_rg().clamp(0.5, 3.0),
            awb_bg: s.avg_awb_bg().clamp(0.5, 3.0),
            gamma: s.avg_gamma().clamp(1.0, 4.0),
            lsc_k1: s.avg_lsc_k1().clamp(-0.1, 0.1),
            lsc_k2: s.avg_lsc_k2().clamp(-0.001, 0.001),
            target_lum: s.avg_target_lum().clamp(0.1, 0.6),
            max_iso: s.avg_max_iso().clamp(200.0, 6400.0),
        })
    }

    /// Find the nearest populated bin (CCT-wise) to the query.
    fn find_nearest_bin(&self, cct: i32) -> Option<i32> {
        if self.bins.is_empty() {
            return None;
        }
        let query = Self::cct_bin(cct);
        // Exact match
        if self.bins.contains_key(&query) {
            return Some(query);
        }
        // Nearest neighbor search
        self.bins
            .keys()
            .min_by_key(|&key| (key - query).abs())
            .copied()
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.bins.clear();
        self.total_observations = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let fp = FastPredictor::new();
        assert!(!fp.is_ready());
        assert_eq!(fp.populated_bin_count(), 0);
        assert_eq!(fp.total_observations, 0);
        assert!((fp.confidence() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_observe() {
        let mut fp = FastPredictor::new();
        fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        assert_eq!(fp.total_observations, 1);
        assert_eq!(fp.populated_bin_count(), 1);
    }

    #[test]
    fn test_not_ready_before_10_obs() {
        let mut fp = FastPredictor::new();
        for _ in 0..9 {
            fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        }
        assert!(!fp.is_ready());
        assert!(fp.predict(5500).is_none());
    }

    #[test]
    fn test_ready_after_10_obs() {
        let mut fp = FastPredictor::new();
        for _ in 0..10 {
            fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        }
        assert!(fp.is_ready());
        let pred = fp.predict(5500);
        assert!(pred.is_some());
        let p = pred.unwrap();
        assert!((p.awb_rg - 1.5).abs() < 0.01);
        assert!((p.gamma - 2.2).abs() < 0.01);
    }

    #[test]
    fn test_nearest_bin() {
        let mut fp = FastPredictor::new();
        // Add observations at 5500K only
        for _ in 0..10 {
            fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        }
        // Predict for 5600K (same bin)
        let pred = fp.predict(5600);
        assert!(pred.is_some());
        // Predict for 8000K (furthest bin)
        let pred_far = fp.predict(8000);
        assert!(pred_far.is_some());
    }

    #[test]
    fn test_multi_bin() {
        let mut fp = FastPredictor::new();
        // 10 obs at 3000K
        for _ in 0..10 {
            fp.observe(3000, 2.0, 0.8, 2.0, 0.02, -0.0002, 0.4, 3200.0);
        }
        // 10 obs at 6500K
        for _ in 0..10 {
            fp.observe(6500, 1.3, 1.5, 2.4, -0.01, 0.0001, 0.3, 800.0);
        }
        assert_eq!(fp.populated_bin_count(), 2);
        assert!(fp.confidence() > 0.0);

        // Predict for warm CCT
        let warm = fp.predict(3000).unwrap();
        assert!((warm.awb_rg - 2.0).abs() < 0.1);
        assert!((warm.max_iso - 3200.0).abs() < 200.0);

        // Predict for cool CCT
        let cool = fp.predict(6500).unwrap();
        assert!((cool.awb_rg - 1.3).abs() < 0.1);
        assert!((cool.max_iso - 800.0).abs() < 200.0);
    }

    #[test]
    fn test_reset() {
        let mut fp = FastPredictor::new();
        for _ in 0..10 {
            fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        }
        assert!(fp.is_ready());
        fp.reset();
        assert!(!fp.is_ready());
        assert_eq!(fp.total_observations, 0);
    }

    #[test]
    fn test_confidence_increases_with_diversity() {
        let mut fp = FastPredictor::new();
        // Single bin → low confidence
        for _ in 0..10 {
            fp.observe(5500, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
        }
        let c1 = fp.confidence();

        // Multiple bins → higher confidence
        for cct in (2000..8000).step_by(1000) {
            for _ in 0..10 {
                fp.observe(cct, 1.5, 1.2, 2.2, 0.01, -0.0001, 0.35, 1600.0);
            }
        }
        let c2 = fp.confidence();
        assert!(
            c2 > c1,
            "More bins should give higher confidence ({} > {})",
            c2,
            c1
        );
    }
}
