//! Zone-based statistics processing.
//!
//! Per-zone RGB statistics for multi-illuminant AWB, CCT estimation,
//! and scene analysis.
//!
//! Extracted from `controller.rs`.

use crate::controller::IspController;

impl IspController {
    /// Initialize zone stats grid with given dimensions.
    /// Call this once before feeding zone stats.
    pub fn init_zone_stats(&mut self, rows: usize, cols: usize) {
        self.zone_rows = rows;
        self.zone_cols = cols;
        let n = rows * cols;

        // Build center-weighted zone weights
        let mut w = vec![0.0f32; n];
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                let cc = c >= cols / 2 - 1 && c <= cols / 2;
                let cr = r >= rows / 2 - 1 && r <= rows / 2;
                w[i] = if cr && cc {
                    4.0
                } else if cr || cc {
                    2.0
                } else {
                    1.0
                };
            }
        }
        let sum: f32 = w.iter().sum();
        if sum > 0.0 {
            for v in w.iter_mut() {
                *v /= sum;
            }
        }
        self.zone_weight = w;

        // Initialize per-zone arrays
        self.zone_rgb = vec![vec![[0.0f32; 3]; cols]; rows];
        self.zone_lum = vec![vec![0.0f32; cols]; rows];
        self.zone_cct = vec![vec![5500.0f32; cols]; rows];
        self.smoothed_zone_cct = vec![vec![5500.0f32; cols]; rows];
        self.zone_cct_initialized = false;
        self.zone_stats_enabled = true;
    }

    /// Update zone stats from per-zone RGB means.
    /// `zone_stats` should have length zone_rows * zone_cols * 3,
    /// with RGB values in `[0, 1]` interleaved per zone (row-major).
    pub fn update_zone_stats(&mut self, zone_stats: &[f32]) {
        if !self.zone_stats_enabled {
            return;
        }
        let expected = self.zone_rows * self.zone_cols * 3;
        if zone_stats.len() < expected {
            return;
        }

        let min_rgb = 0.001;
        let mut idx = 0;
        let mut warm_c = 0;
        let mut mid_c = 0;
        let mut cool_c = 0;
        let mut valid_z = 0;
        let mut t_r = 0.0f64;
        let mut t_g = 0.0f64;
        let mut t_b = 0.0f64;
        let mut t_w = 0.0f64;

        self.zone_rgb.clear();
        self.zone_lum.clear();
        self.zone_cct.clear();

        for r in 0..self.zone_rows {
            let mut rgb_row = Vec::with_capacity(self.zone_cols);
            let mut lum_row = Vec::with_capacity(self.zone_cols);
            let mut cct_row = Vec::with_capacity(self.zone_cols);

            for c in 0..self.zone_cols {
                let rz = zone_stats[idx].max(min_rgb);
                let gz = zone_stats[idx + 1].max(min_rgb);
                let bz = zone_stats[idx + 2].max(min_rgb);
                idx += 3;

                rgb_row.push([rz, gz, bz]);
                let y = 0.299 * rz + 0.587 * gz + 0.114 * bz;
                lum_row.push(y);

                let w_idx = r * self.zone_cols + c;
                let w = self.zone_weight.get(w_idx).copied().unwrap_or(1.0) as f64;
                t_r += rz as f64 * w;
                t_g += gz as f64 * w;
                t_b += bz as f64 * w;
                t_w += w;

                // Compute per-zone CCT
                if gz > min_rgb && rz > min_rgb && bz > min_rgb {
                    let rg = rz / gz;
                    let bg = bz / gz;
                    let cct_val = IspController::estimate_cct(rg, bg) as f32;
                    cct_row.push(cct_val);

                    if self.zone_cct_initialized {
                        let prev = self.smoothed_zone_cct[r][c];
                        self.smoothed_zone_cct[r][c] = prev + (cct_val - prev) * 0.3;
                    } else {
                        self.smoothed_zone_cct[r][c] = cct_val;
                    }
                    let smoothed = self.smoothed_zone_cct[r][c];
                    if smoothed < 4000.0 {
                        warm_c += 1;
                    } else if smoothed > 5500.0 {
                        cool_c += 1;
                    } else {
                        mid_c += 1;
                    }
                    valid_z += 1;
                } else {
                    cct_row.push(5500.0);
                }
            }

            self.zone_rgb.push(rgb_row);
            self.zone_lum.push(lum_row);
            self.zone_cct.push(cct_row);
        }

        self.zone_cct_initialized = true;

        // ── Zone-weighted AWB ──
        if t_w > 0.001 {
            let wr = (t_r / t_w) as f32;
            let wg = (t_g / t_w) as f32;
            let wb = (t_b / t_w) as f32;
            if wg > min_rgb {
                let r_gain = (wg * self.target_r / wr).clamp(0.5, 3.0);
                let b_gain = (wg * self.target_b / wb).clamp(0.5, 3.0);
                let alpha_awb = self.awb_alpha();
                self.awb_gains[0] += (r_gain - self.awb_gains[0]) * alpha_awb;
                self.awb_gains[2] += (b_gain - self.awb_gains[2]) * alpha_awb;
            }
        }

        // ── Multi-illuminant cluster detection ──
        if valid_z >= 6 {
            let total = (warm_c + mid_c + cool_c) as f32;
            let max_c = warm_c.max(mid_c).max(cool_c);
            self.dominant_cct_cluster = if max_c == warm_c {
                Some(0)
            } else if max_c == mid_c {
                Some(1)
            } else {
                Some(2)
            };

            let mut sorted = [warm_c, mid_c, cool_c];
            sorted.sort_by(|a, b| b.cmp(a));
            if sorted.len() >= 2
                && (sorted[1] as f32 / total) > 0.20
                && (sorted[0] as f32) < 0.80 * total
            {
                self.dominant_cct_cluster = Some(-1);
            }
            self.dominant_cluster_fraction = max_c as f32 / total;
        }

        log::debug!(
            "ZoneStats: w={} m={} c={} cluster={:?} frac={:.2}",
            warm_c,
            mid_c,
            cool_c,
            self.dominant_cct_cluster,
            self.dominant_cluster_fraction
        );
    }
}
