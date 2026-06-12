//! RegressionModel — OLS linear regression for ISP parameter prediction.
//!
//! Ported from `RegressionModel.kt` (Java, 237 lines).
//!
//! Features: [log(luminance), R/G ratio, B/G ratio, CCT/10000]
//! Targets: AWB R/G, B/G, CCM diagonal R/G/B, gamma, LSC k1/k2,
//!          exposure target luminance, exposure max ISO
//!
//! Fitted via closed-form OLS (ordinary least squares) with Gauss-Jordan
//! elimination for matrix inversion.
//!
//! ## Usage
//! ```ignore
//! let mut model = cam_isp::regression::RegressionModel::new();
//! // model.fit_all(&observations);
//! let pred = model.predict_rgb(0.3, 0.5, 0.2, 5500);
//! ```

/// Number of features: [log_lum, rg_ratio, bg_ratio, cct_norm]
const N_FEATURES: usize = 4;
/// Number of weights per target: bias + N_FEATURES = 5
const N_WEIGHTS: usize = N_FEATURES + 1;

/// Feature vector [log(lum), R/G, B/G, CCT/10000].
#[derive(Debug, Clone, Copy)]
pub struct Features {
    pub log_lum: f64,
    pub rg_ratio: f64,
    pub bg_ratio: f64,
    pub cct_norm: f64,
}

impl Features {
    /// Build feature vector from raw pixel stats.
    /// `lum` is mean luminance in [0, 1].
    /// `cct` is estimated color temperature in Kelvin.
    pub fn from_rgb_lum_cct(r: f32, g: f32, b: f32, lum: f32, cct: u32) -> Self {
        let rg = if g > 0.001 { (r / g) as f64 } else { 1.0 };
        let bg = if g > 0.001 { (b / g) as f64 } else { 1.0 };
        let log_lum = if lum > 0.001 {
            (lum as f64).ln()
        } else {
            -6.9 // ln(0.001)
        };
        let cct_norm = cct as f64 / 10000.0;
        Self { log_lum, rg_ratio: rg, bg_ratio: bg, cct_norm }
    }

    /// Return as array for matrix operations.
    fn to_array(&self) -> [f64; N_FEATURES] {
        [self.log_lum, self.rg_ratio, self.bg_ratio, self.cct_norm]
    }
}

/// Fitted weights for one target parameter: [bias, w_loglum, w_rg, w_bg, w_cct].
pub type Weights = [f64; N_WEIGHTS];

/// OLS linear regression model for ISP parameter calibration.
#[derive(Debug, Clone)]
pub struct RegressionModel {
    pub is_ready: bool,
    pub awb_rg: Weights,
    pub awb_bg: Weights,
    pub ccm_diag_r: Weights,
    pub ccm_diag_g: Weights,
    pub ccm_diag_b: Weights,
    pub gamma: Weights,
    pub lsc_k1: Weights,
    pub lsc_k2: Weights,
    pub target_lum: Weights,
    pub max_iso: Weights,
}

impl Default for RegressionModel {
    fn default() -> Self {
        Self::new()
    }
}

impl RegressionModel {
    /// Create a new model with default (no-op) weights.
    pub fn new() -> Self {
        Self {
            is_ready: false,
            awb_rg: default_weights(1.0),
            awb_bg: default_weights(1.0),
            ccm_diag_r: default_weights(1.0),
            ccm_diag_g: default_weights(1.0),
            ccm_diag_b: default_weights(1.0),
            gamma: default_weights(2.2),
            lsc_k1: default_weights(0.01),
            lsc_k2: default_weights(-0.0001),
            target_lum: default_weights(0.35),
            max_iso: default_weights(1600.0),
        }
    }

    /// Reset model to unready state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Predict a single target from features using fitted weights.
    /// y = w[0] + sum(w[i+1] * features[i])
    pub fn predict(features: &Features, weights: &Weights) -> f64 {
        let arr = features.to_array();
        let mut y = weights[0];
        for i in 0..N_FEATURES {
            y += weights[i + 1] * arr[i];
        }
        y
    }

    /// Predict for a single observation tuple.
    pub fn predict_rgb(&self, r: f32, g: f32, b: f32, cct: u32) -> Prediction {
        let lum = 0.299 * r as f64 + 0.587 * g as f64 + 0.114 * b as f64;
        let features = Features::from_rgb_lum_cct(r, g, b, lum as f32, cct);

        Prediction {
            awb_rg: Self::predict(&features, &self.awb_rg) as f32,
            awb_bg: Self::predict(&features, &self.awb_bg) as f32,
            ccm_diag_r: Self::predict(&features, &self.ccm_diag_r) as f32,
            ccm_diag_g: Self::predict(&features, &self.ccm_diag_g) as f32,
            ccm_diag_b: Self::predict(&features, &self.ccm_diag_b) as f32,
            gamma: Self::predict(&features, &self.gamma) as f32,
            lsc_k1: Self::predict(&features, &self.lsc_k1) as f32,
            lsc_k2: Self::predict(&features, &self.lsc_k2) as f32,
            target_lum: Self::predict(&features, &self.target_lum) as f32,
            max_iso: Self::predict(&features, &self.max_iso) as f32,
        }
    }

    /// Fit all 10 target parameters from observations.
    pub fn fit_all(&mut self, observations: &[Observation]) {
        if observations.len() < 5 {
            return; // Need at least 5 observations for 5 weights
        }

        let features: Vec<Features> = observations.iter().map(|obs| {
            Features::from_rgb_lum_cct(obs.r, obs.g, obs.b, obs.lum, obs.cct)
        }).collect();

        let targets_awb_rg: Vec<f64> = observations.iter().map(|o| o.hw_awb_rg as f64).collect();
        let targets_awb_bg: Vec<f64> = observations.iter().map(|o| o.hw_awb_bg as f64).collect();
        let targets_ccm_r: Vec<f64> = observations.iter().map(|o| o.hw_ccm_diag_r as f64).collect();
        let targets_ccm_g: Vec<f64> = observations.iter().map(|o| o.hw_ccm_diag_g as f64).collect();
        let targets_ccm_b: Vec<f64> = observations.iter().map(|o| o.hw_ccm_diag_b as f64).collect();
        let targets_gamma: Vec<f64> = observations.iter().map(|o| o.hw_gamma as f64).collect();
        let targets_lsc_k1: Vec<f64> = observations.iter().map(|o| o.lsc_k1 as f64).collect();
        let targets_lsc_k2: Vec<f64> = observations.iter().map(|o| o.lsc_k2 as f64).collect();
        let targets_target_lum: Vec<f64> = observations.iter().map(|o| {
            let exp_norm = (o.hw_exp_ms * o.hw_analog_gain) / 33.0;
            (o.lum as f64 * exp_norm as f64).clamp(0.1, 0.6)
        }).collect();
        let targets_max_iso: Vec<f64> = observations.iter().map(|o| {
            (o.hw_analog_gain as f64 * 100.0).clamp(200.0, 6400.0)
        }).collect();

        self.awb_rg = ordinary_least_squares(&features, &targets_awb_rg);
        self.awb_bg = ordinary_least_squares(&features, &targets_awb_bg);
        self.ccm_diag_r = ordinary_least_squares(&features, &targets_ccm_r);
        self.ccm_diag_g = ordinary_least_squares(&features, &targets_ccm_g);
        self.ccm_diag_b = ordinary_least_squares(&features, &targets_ccm_b);
        self.gamma = ordinary_least_squares(&features, &targets_gamma);
        self.lsc_k1 = ordinary_least_squares(&features, &targets_lsc_k1);
        self.lsc_k2 = ordinary_least_squares(&features, &targets_lsc_k2);
        self.target_lum = ordinary_least_squares(&features, &targets_target_lum);
        self.max_iso = ordinary_least_squares(&features, &targets_max_iso);

        self.is_ready = true;
    }
}

/// A single observation for model fitting.
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub lum: f32,
    pub cct: u32,
    pub hw_awb_rg: f32,
    pub hw_awb_bg: f32,
    pub hw_ccm_diag_r: f32,
    pub hw_ccm_diag_g: f32,
    pub hw_ccm_diag_b: f32,
    pub hw_gamma: f32,
    pub lsc_k1: f32,
    pub lsc_k2: f32,
    pub hw_exp_ms: f32,
    pub hw_analog_gain: f32,
}

/// Predicted ISP parameters for a single query.
#[derive(Debug, Clone, Copy)]
pub struct Prediction {
    pub awb_rg: f32,
    pub awb_bg: f32,
    pub ccm_diag_r: f32,
    pub ccm_diag_g: f32,
    pub ccm_diag_b: f32,
    pub gamma: f32,
    pub lsc_k1: f32,
    pub lsc_k2: f32,
    pub target_lum: f32,
    pub max_iso: f32,
}

// ── OLS implementation ──

/// Ordinary least squares via Gauss-Jordan elimination.
///
/// w = (X'X)^(-1) · X'y
///
/// Features are N_FEATURES per row, with an implicit bias column of 1s.
/// Returns N_WEIGHTS weights: [bias, w_loglum, w_rg, w_bg, w_cct].
fn ordinary_least_squares(features: &[Features], targets: &[f64]) -> Weights {
    let n = features.len();
    let k = N_FEATURES;
    let m = k + 1; // +1 for bias term

    // Build X'X and X'y
    let mut xtx = vec![vec![0.0f64; m]; m];
    let mut xty = vec![0.0f64; m];

    for i in 0..n {
        let row = features[i].to_array();
        let y = targets[i];

        // Bias term: feature = 1
        xtx[0][0] += 1.0;
        xty[0] += y;

        for j in 0..k {
            xtx[0][j + 1] += row[j];
            xtx[j + 1][0] += row[j];
            xty[j + 1] += row[j] * y;
            for l in 0..k {
                xtx[j + 1][l + 1] += row[j] * row[l];
            }
        }
    }

    // Gauss-Jordan elimination on augmented matrix [X'X | I]
    let mut mat = xtx.clone();
    let mut inv = vec![vec![0.0f64; m]; m];
    for i in 0..m {
        inv[i][i] = 1.0;
    }

    for col in 0..m {
        let mut pivot = mat[col][col];
        if pivot.abs() < 1e-12 {
            pivot = 1e-12;
        }
        for j in 0..m {
            mat[col][j] /= pivot;
            inv[col][j] /= pivot;
        }
        for row in 0..m {
            if row == col {
                continue;
            }
            let factor = mat[row][col];
            for j in 0..m {
                mat[row][j] -= factor * mat[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }

    // weights = inv * xty
    let mut w = [0.0f64; N_WEIGHTS];
    for i in 0..m {
        for j in 0..m {
            w[i] += inv[i][j] * xty[j];
        }
    }
    w
}

/// Build default weights: bias = `default_val`, all other coefficients = 0.
fn default_weights(default_val: f64) -> Weights {
    let mut w = [0.0f64; N_WEIGHTS];
    w[0] = default_val;
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let model = RegressionModel::new();
        assert!(!model.is_ready);
        assert!((model.awb_rg[0] - 1.0).abs() < 1e-6);
        assert!((model.gamma[0] - 2.2).abs() < 1e-6);
    }

    #[test]
    fn test_reset() {
        let mut model = RegressionModel::new();
        model.is_ready = true;
        model.awb_rg = [2.0, 0.0, 0.0, 0.0, 0.0];
        model.reset();
        assert!(!model.is_ready);
        assert!((model.awb_rg[0] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_feature_vector() {
        let f = Features::from_rgb_lum_cct(0.5, 0.3, 0.2, 0.3, 5500);
        // log(0.3) ≈ -1.204
        assert!((f.log_lum - (-1.204)).abs() < 0.01, "log_lum={}", f.log_lum);
        // R/G = 0.5/0.3 ≈ 1.667
        assert!((f.rg_ratio - 1.667).abs() < 0.01, "rg_ratio={}", f.rg_ratio);
        // B/G = 0.2/0.3 ≈ 0.667
        assert!((f.bg_ratio - 0.667).abs() < 0.01, "bg_ratio={}", f.bg_ratio);
        // CCT/10000 = 0.55
        assert!((f.cct_norm - 0.55).abs() < 1e-6, "cct_norm={}", f.cct_norm);
    }

    #[test]
    fn test_predict_default() {
        let model = RegressionModel::new();
        let pred = model.predict_rgb(0.5, 0.3, 0.2, 5500);
        // Default weights should give default predictions
        assert!((pred.awb_rg - 1.0).abs() < 0.01);
        assert!((pred.gamma - 2.2).abs() < 0.01);
        assert!((pred.lsc_k1 - 0.01).abs() < 0.001);
    }

    #[test]
    fn test_fit_perfect_linear() {
        // Generate perfect linear data: target = 2 + 3*log_lum + 0.5*rg - bg + 4*cct_norm
        let mut observations = Vec::new();
        for i in 0..20 {
            let cct = 2000 + i * 300;
            let lum = 0.05 + i as f32 * 0.03;
            let r = 0.3 + i as f32 * 0.02;
            let g = 0.3;
            let b = 0.3 - i as f32 * 0.01;
            let features = Features::from_rgb_lum_cct(r, g, b, lum, cct as u32);
            let true_val = 2.0 + 3.0 * features.log_lum + 0.5 * features.rg_ratio
                - features.bg_ratio + 4.0 * features.cct_norm;
            observations.push(Observation {
                r, g, b, lum,
                cct: cct as u32,
                hw_awb_rg: true_val as f32,
                hw_awb_bg: 1.0,
                hw_ccm_diag_r: 1.0,
                hw_ccm_diag_g: 1.0,
                hw_ccm_diag_b: 1.0,
                hw_gamma: 2.2,
                lsc_k1: 0.01,
                lsc_k2: -0.0001,
                hw_exp_ms: 10.0,
                hw_analog_gain: 1.0,
            });
        }

        let mut model = RegressionModel::new();
        model.fit_all(&observations);
        assert!(model.is_ready);

        // The fitted weights should closely approximate the true coefficients
        // True: y = 2 + 3*log_lum + 0.5*rg - bg + 4*cct
        let pred = model.predict_rgb(0.5, 0.3, 0.2, 5500);
        let expected: f64 = {
            let f = Features::from_rgb_lum_cct(0.5, 0.3, 0.2, 0.3, 5500);
            2.0 + 3.0 * f.log_lum + 0.5 * f.rg_ratio - f.bg_ratio + 4.0 * f.cct_norm
        };
        assert!(
            (pred.awb_rg as f64 - expected).abs() < 0.5,
            "pred={:.4} expected={:.4}", pred.awb_rg, expected
        );
    }

    #[test]
    fn test_not_enough_observations() {
        let mut model = RegressionModel::new();
        let obs = vec![Observation {
            r: 0.5, g: 0.3, b: 0.2, lum: 0.3, cct: 5500,
            hw_awb_rg: 1.5, hw_awb_bg: 1.2, hw_ccm_diag_r: 1.0,
            hw_ccm_diag_g: 1.0, hw_ccm_diag_b: 1.0, hw_gamma: 2.2,
            lsc_k1: 0.01, lsc_k2: -0.0001, hw_exp_ms: 10.0, hw_analog_gain: 1.0,
        }];
        model.fit_all(&obs);
        assert!(!model.is_ready, "Need ≥5 observations");
    }

    #[test]
    fn test_ols_basic() {
        // Simple 1-feature: y = 2 + 3*x
        let features = vec![
            Features { log_lum: 0.0, rg_ratio: 0.5, bg_ratio: 0.5, cct_norm: 0.5 },
            Features { log_lum: 0.0, rg_ratio: 0.0, bg_ratio: 0.0, cct_norm: 0.0 },
            Features { log_lum: 0.0, rg_ratio: 1.0, bg_ratio: 1.0, cct_norm: 1.0 },
            Features { log_lum: 0.0, rg_ratio: 0.3, bg_ratio: 0.3, cct_norm: 0.3 },
            Features { log_lum: 0.0, rg_ratio: 0.8, bg_ratio: 0.8, cct_norm: 0.8 },
        ];
        let targets: Vec<f64> = features.iter().map(|f| {
            1.0 + 0.5 * f.rg_ratio + 0.3 * f.bg_ratio - 0.2 * f.cct_norm
        }).collect();

        let w = ordinary_least_squares(&features, &targets);
        // w[0] ≈ 1.0, w[2] ≈ 0.5, w[3] ≈ 0.3, w[4] ≈ -0.2
        assert!((w[0] - 1.0).abs() < 0.3, "bias={:.4}", w[0]);
        assert!((w[2] - 0.5).abs() < 0.2, "w_rg={:.4}", w[2]);
    }
}
