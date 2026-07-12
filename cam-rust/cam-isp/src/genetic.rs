//! GeneticOptimizer — genetic algorithm for ISP parameter calibration.
//!
//! Ported from `GeneticOptimizer.kt` (Java, 230 lines).
//!
//! Evolves a population of `[Chromosome]`s toward lower fitness (error vs.
//! Camera2 HW ISP observations). Uses tournament selection, blend crossover,
//! bounded mutation, and elitism.

use rand::Rng;
use rand::SeedableRng;

use crate::store::LearnerObservation;

/// A candidate ISP parameter set.
#[derive(Debug, Clone, Copy)]
pub struct Chromosome {
    /// Offset from 1.0 for AWB R/G (e.g., -0.1 → 0.9).
    pub awb_rg_offset: f32,
    /// Offset from 1.0 for AWB B/G (e.g., 0.2 → 1.2).
    pub awb_bg_offset: f32,
    /// CCM diagonal R.
    pub ccm_diag_r: f32,
    /// CCM diagonal G.
    pub ccm_diag_g: f32,
    /// CCM diagonal B.
    pub ccm_diag_b: f32,
    /// Display gamma.
    pub gamma: f32,
    /// Tone contrast.
    pub contrast: f32,
    /// Tone brightness offset.
    pub brightness: f32,
}

impl Chromosome {
    /// AWB R gain: 1 + offset.
    pub fn awb_rg(&self) -> f32 {
        1.0 + self.awb_rg_offset
    }
    /// AWB B gain: 1 + offset.
    pub fn awb_bg(&self) -> f32 {
        1.0 + self.awb_bg_offset
    }

    /// Human-readable parameter string.
    pub fn to_param_string(&self) -> String {
        format!(
            "awbRg={:.3} awbBg={:.3} ccm=[{:.3},{:.3},{:.3}] γ={:.2} c={:.2} b={:.3}",
            self.awb_rg(),
            self.awb_bg(),
            self.ccm_diag_r,
            self.ccm_diag_g,
            self.ccm_diag_b,
            self.gamma,
            self.contrast,
            self.brightness,
        )
    }
}

/// Result of a genetic optimization run.
#[derive(Debug, Clone)]
pub struct GeneticResult {
    /// Best chromosome found.
    pub best: Chromosome,
    /// Best fitness (lower = better).
    pub fitness: f32,
    /// Number of generations evolved.
    pub generations: u32,
    /// Best fitness per generation (for convergence tracking).
    pub history: Vec<f32>,
}

/// Genetic algorithm optimizer for ISP parameters.
#[derive(Debug, Clone)]
pub struct GeneticOptimizer {
    /// Number of generations to evolve.
    pub generations: u32,
    /// Population size per generation.
    pub population_size: u32,
    /// Mutation probability per gene.
    pub mutation_rate: f32,
    /// Tournament size for selection.
    pub tournament_size: u32,
    /// Elitism count (keep N best).
    pub elitism: u32,
    /// Seed for reproducible runs (None = random).
    pub seed: Option<u64>,
}

impl Default for GeneticOptimizer {
    fn default() -> Self {
        Self {
            generations: 200,
            population_size: 100,
            mutation_rate: 0.15,
            tournament_size: 5,
            elitism: 2,
            seed: None,
        }
    }
}

impl GeneticOptimizer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Run genetic optimization.
    ///
    /// Returns the best chromosome and its fitness.
    /// Returns a no-op chromosome with fitness = MAX if < 15 observations.
    pub fn optimize(&self, observations: &[LearnerObservation]) -> GeneticResult {
        if observations.len() < 15 {
            return GeneticResult {
                best: Chromosome {
                    awb_rg_offset: 0.0,
                    awb_bg_offset: 0.0,
                    ccm_diag_r: 1.0,
                    ccm_diag_g: 1.0,
                    ccm_diag_b: 1.0,
                    gamma: 2.2,
                    contrast: 1.0,
                    brightness: 0.0,
                },
                fitness: f32::MAX,
                generations: 0,
                history: Vec::new(),
            };
        }

        let mut rng = match self.seed {
            Some(s) => rand::rngs::StdRng::seed_from_u64(s),
            None => rand::rngs::StdRng::from_entropy(),
        };

        let mut history = Vec::with_capacity(self.generations as usize);
        let mut population: Vec<Chromosome> = (0..self.population_size)
            .map(|_| random_chromosome(&mut rng))
            .collect();
        let mut best_ever = population[0];
        let mut best_fitness = f32::MAX;

        for _gen in 0..self.generations {
            let fitnesses: Vec<f32> = population
                .iter()
                .map(|c| fitness(c, observations))
                .collect();

            let gen_best_idx = fitnesses
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .map(|(i, _)| i)
                .unwrap_or(0);
            let gen_best_fit = fitnesses[gen_best_idx];
            history.push(gen_best_fit);

            if gen_best_fit < best_fitness {
                best_fitness = gen_best_fit;
                best_ever = population[gen_best_idx];
            }

            // Build next population
            let mut sorted: Vec<(usize, &Chromosome, f32)> = population
                .iter()
                .enumerate()
                .zip(fitnesses.iter())
                .map(|((i, c), f)| (i, c, *f))
                .collect();
            sorted.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());

            let mut new_pop = Vec::with_capacity(self.population_size as usize);

            // Elitism: keep best N
            for i in 0..self.elitism.min(self.population_size) {
                new_pop.push(*sorted[i as usize].1);
            }

            while (new_pop.len() as u32) < self.population_size {
                let p1 = tournament_select(&population, &fitnesses, self.tournament_size, &mut rng);
                let p2 = tournament_select(&population, &fitnesses, self.tournament_size, &mut rng);
                let child = crossover(&p1, &p2, &mut rng);
                new_pop.push(mutate(&child, self.mutation_rate, &mut rng));
            }

            population = new_pop;
        }

        GeneticResult {
            best: best_ever,
            fitness: best_fitness,
            generations: self.generations,
            history,
        }
    }

    /// Export optimized characteristics as JSON string.
    pub fn export_json(&self, observations: &[LearnerObservation]) -> String {
        let result = self.optimize(observations);
        let c = result.best;

        let hw_gamma: Vec<f32> = observations.iter().map(|o| o.hw_gamma).collect();
        let hw_rg: Vec<f32> = observations.iter().map(|o| o.hw_awb_rg).collect();
        let hw_bg: Vec<f32> = observations.iter().map(|o| o.hw_awb_bg).collect();

        format!(
            r#"{{"version":1,"observations":{},"learned":{{
"awbRgOffset":{:.4},"awbBgOffset":{:.4},
"ccmDiagR":{:.4},"ccmDiagG":{:.4},"ccmDiagB":{:.4},
"gamma":{:.4},"contrast":{:.4},"brightness":{:.4},"fitness":{:.6}
}},"stats_summary":{{
"hwGamma_mean":{:.4},"hwGamma_std":{:.4},
"hwAwbRg_mean":{:.4},"hwAwbBg_mean":{:.4}
}}}}
"#,
            observations.len(),
            c.awb_rg_offset,
            c.awb_bg_offset,
            c.ccm_diag_r,
            c.ccm_diag_g,
            c.ccm_diag_b,
            c.gamma,
            c.contrast,
            c.brightness,
            result.fitness,
            mean(&hw_gamma),
            std_dev(&hw_gamma),
            mean(&hw_rg),
            mean(&hw_bg),
        )
    }
}

// ── Private helpers ──

fn random_chromosome<R: Rng>(rng: &mut R) -> Chromosome {
    Chromosome {
        awb_rg_offset: rng.gen_range(-0.3..0.3),
        awb_bg_offset: rng.gen_range(-0.3..0.3),
        ccm_diag_r: rng.gen_range(0.8..1.4),
        ccm_diag_g: rng.gen_range(0.8..1.4),
        ccm_diag_b: rng.gen_range(0.8..1.4),
        gamma: rng.gen_range(1.0..3.0),
        contrast: rng.gen_range(0.6..1.4),
        brightness: rng.gen_range(-0.1..0.1),
    }
}

/// Fitness: weighted sum of absolute errors between predicted and observed HW params.
/// Lower = better.
fn fitness(chr: &Chromosome, data: &[LearnerObservation]) -> f32 {
    if data.is_empty() {
        return f32::MAX;
    }
    let awb_rg = chr.awb_rg();
    let awb_bg = chr.awb_bg();
    let mut err = 0.0f64;
    for obs in data {
        err += (awb_rg - obs.hw_awb_rg).abs() as f64 * 2.0;
        err += (awb_bg - obs.hw_awb_bg).abs() as f64 * 2.0;
        err += (chr.ccm_diag_r - obs.hw_ccm_diag_r).abs() as f64;
        err += (chr.ccm_diag_g - obs.hw_ccm_diag_g).abs() as f64;
        err += (chr.ccm_diag_b - obs.hw_ccm_diag_b).abs() as f64;
        err += (chr.gamma - obs.hw_gamma).abs() as f64 * 3.0;
    }
    err += (chr.contrast - 1.0).abs() as f64 * 0.1;
    err += chr.brightness.abs() as f64 * 0.1;
    (err / data.len() as f64) as f32
}

/// Tournament selection: randomly pick `size` individuals, return the best.
fn tournament_select<R: Rng>(
    pop: &[Chromosome],
    fitnesses: &[f32],
    size: u32,
    rng: &mut R,
) -> Chromosome {
    let n = pop.len();
    let mut best_idx = rng.gen_range(0..n);
    for _ in 1..size {
        let idx = rng.gen_range(0..n);
        if fitnesses[idx] < fitnesses[best_idx] {
            best_idx = idx;
        }
    }
    pop[best_idx]
}

/// Blend crossover (BLX-α).
fn crossover<R: Rng>(a: &Chromosome, b: &Chromosome, rng: &mut R) -> Chromosome {
    let alpha = 0.3;
    fn blend(va: f32, vb: f32, alpha: f32, rng: &mut impl Rng) -> f32 {
        let lo = va.min(vb) - alpha * (va - vb).abs();
        let hi = va.max(vb) + alpha * (va - vb).abs();
        if (hi - lo).abs() < f32::EPSILON {
            return (va + vb) / 2.0;
        }
        rng.gen_range(lo..hi)
    }
    Chromosome {
        awb_rg_offset: blend(a.awb_rg_offset, b.awb_rg_offset, alpha, rng),
        awb_bg_offset: blend(a.awb_bg_offset, b.awb_bg_offset, alpha, rng),
        ccm_diag_r: blend(a.ccm_diag_r, b.ccm_diag_r, alpha, rng),
        ccm_diag_g: blend(a.ccm_diag_g, b.ccm_diag_g, alpha, rng),
        ccm_diag_b: blend(a.ccm_diag_b, b.ccm_diag_b, alpha, rng),
        gamma: blend(a.gamma, b.gamma, alpha, rng),
        contrast: blend(a.contrast, b.contrast, alpha, rng),
        brightness: blend(a.brightness, b.brightness, alpha, rng),
    }
}

/// Bounded mutation: randomly perturb each gene with probability `rate`.
fn mutate<R: Rng>(chr: &Chromosome, rate: f32, rng: &mut R) -> Chromosome {
    fn mutate_val(v: f32, range: f32, rate: f32, rng: &mut impl Rng) -> f32 {
        if rng.gen::<f32>() >= rate {
            return v;
        }
        v + (rng.gen::<f32>() - 0.5) * range * 2.0
    }
    Chromosome {
        awb_rg_offset: mutate_val(chr.awb_rg_offset, 0.1, rate, rng).clamp(-0.5, 0.5),
        awb_bg_offset: mutate_val(chr.awb_bg_offset, 0.1, rate, rng).clamp(-0.5, 0.5),
        ccm_diag_r: mutate_val(chr.ccm_diag_r, 0.05, rate, rng).clamp(0.6, 1.6),
        ccm_diag_g: mutate_val(chr.ccm_diag_g, 0.05, rate, rng).clamp(0.6, 1.6),
        ccm_diag_b: mutate_val(chr.ccm_diag_b, 0.05, rate, rng).clamp(0.6, 1.6),
        gamma: mutate_val(chr.gamma, 0.2, rate, rng).clamp(1.0, 4.0),
        contrast: mutate_val(chr.contrast, 0.05, rate, rng).clamp(0.5, 1.5),
        brightness: mutate_val(chr.brightness, 0.02, rate, rng).clamp(-0.2, 0.2),
    }
}

fn mean(data: &[f32]) -> f32 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f32>() / data.len() as f32
}

fn std_dev(data: &[f32]) -> f32 {
    if data.len() < 2 {
        return 0.0;
    }
    let m = mean(data);
    let variance = data.iter().map(|v| (v - m).powi(2)).sum::<f32>() / (data.len() - 1) as f32;
    variance.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_obs(
        awb_rg: f32,
        awb_bg: f32,
        ccm_r: f32,
        ccm_g: f32,
        ccm_b: f32,
        gamma: f32,
    ) -> LearnerObservation {
        LearnerObservation {
            r: 0.5,
            g: 0.5,
            b: 0.5,
            lum: 0.5,
            cct: 5500,
            hw_awb_rg: awb_rg,
            hw_awb_bg: awb_bg,
            hw_ccm_diag_r: ccm_r,
            hw_ccm_diag_g: ccm_g,
            hw_ccm_diag_b: ccm_b,
            hw_gamma: gamma,
            hw_exp_ms: 10.0,
            hw_analog_gain: 1.0,
            lsc_k1: 0.01,
            lsc_k2: -0.0001,
            bayer_pattern: 0,
            frame_idx: 0,
            timestamp_ms: 0,
        }
    }

    #[test]
    fn test_chromosome_convenience() {
        let c = Chromosome {
            awb_rg_offset: 0.2,
            awb_bg_offset: -0.1,
            ccm_diag_r: 1.0,
            ccm_diag_g: 1.0,
            ccm_diag_b: 1.0,
            gamma: 2.2,
            contrast: 1.0,
            brightness: 0.0,
        };
        assert!((c.awb_rg() - 1.2).abs() < 0.01);
        assert!((c.awb_bg() - 0.9).abs() < 0.01);
        let s = c.to_param_string();
        assert!(s.contains("awbRg=1.200"));
    }

    #[test]
    fn test_fitness_perfect_match() {
        let chr = Chromosome {
            awb_rg_offset: 0.2,
            awb_bg_offset: -0.1,
            ccm_diag_r: 1.1,
            ccm_diag_g: 1.0,
            ccm_diag_b: 0.9,
            gamma: 2.2,
            contrast: 1.0,
            brightness: 0.0,
        };
        let obs = vec![make_obs(1.2, 0.9, 1.1, 1.0, 0.9, 2.2)];
        let f = fitness(&chr, &obs);
        // Perfect match → only regularization terms remain
        assert!(
            f < 0.02,
            "Perfect match fitness should be near 0, got {}",
            f
        );
    }

    #[test]
    fn test_fitness_poor_match() {
        let chr = Chromosome {
            awb_rg_offset: 0.0,
            awb_bg_offset: 0.0,
            ccm_diag_r: 1.0,
            ccm_diag_g: 1.0,
            ccm_diag_b: 1.0,
            gamma: 2.2,
            contrast: 1.0,
            brightness: 0.0,
        };
        let obs = vec![make_obs(2.0, 0.5, 1.5, 1.2, 0.6, 3.5)];
        let f = fitness(&chr, &obs);
        assert!(
            f > 0.5,
            "Poor match fitness should be significant, got {}",
            f
        );
    }

    #[test]
    fn test_not_enough_observations() {
        let opt = GeneticOptimizer::new();
        let obs = vec![make_obs(1.0, 1.0, 1.0, 1.0, 1.0, 2.2)];
        let result = opt.optimize(&obs);
        assert!((result.fitness - f32::MAX).abs() < 0.1);
        assert_eq!(result.generations, 0);
    }

    #[test]
    fn test_optimization_converges() {
        let opt = GeneticOptimizer {
            generations: 50,
            population_size: 30,
            mutation_rate: 0.2,
            tournament_size: 3,
            elitism: 2,
            seed: Some(42),
        };

        // Perfectly consistent observations: all have same HW params
        let obs: Vec<LearnerObservation> = (0..20)
            .map(|_| make_obs(1.3, 1.1, 1.05, 0.98, 1.02, 2.4))
            .collect();

        let result = opt.optimize(&obs);
        assert!(result.generations > 0);
        assert!(!result.history.is_empty());

        // The best fitness should be low (near-perfect match possible)
        // Allow some slack since GA might not find exact global optimum
        assert!(
            result.fitness < 0.15,
            "Should converge close to target, got fitness={}",
            result.fitness
        );

        // AWB R should be near 1.3
        assert!(
            (result.best.awb_rg() - 1.3).abs() < 0.15,
            "awb_rg={:.3} should be near 1.3",
            result.best.awb_rg()
        );
    }

    #[test]
    fn test_deterministic_seed() {
        let opt = GeneticOptimizer {
            generations: 30,
            population_size: 20,
            seed: Some(12345),
            ..Default::default()
        };
        let obs: Vec<LearnerObservation> = (0..20)
            .map(|_| make_obs(1.3, 1.1, 1.05, 0.98, 1.02, 2.4))
            .collect();

        let result1 = opt.optimize(&obs);
        let result2 = opt.optimize(&obs);
        assert!((result1.fitness - result2.fitness).abs() < 0.001);
    }

    #[test]
    fn test_export_json() {
        let opt = GeneticOptimizer {
            generations: 10,
            population_size: 10,
            seed: Some(99),
            ..Default::default()
        };
        let obs: Vec<LearnerObservation> = (0..15)
            .map(|_| make_obs(1.3, 1.1, 1.05, 0.98, 1.02, 2.4))
            .collect();

        let json = opt.export_json(&obs);
        assert!(json.contains("awbRgOffset"));
        assert!(json.contains("ccmDiagR"));
        assert!(json.contains("fitness"));
    }

    #[test]
    fn test_tournament_selection() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let pop = vec![
            Chromosome {
                awb_rg_offset: 0.0,
                awb_bg_offset: 0.0,
                ccm_diag_r: 1.0,
                ccm_diag_g: 1.0,
                ccm_diag_b: 1.0,
                gamma: 2.2,
                contrast: 1.0,
                brightness: 0.0,
            },
            Chromosome {
                awb_rg_offset: 0.5,
                awb_bg_offset: 0.5,
                ccm_diag_r: 1.5,
                ccm_diag_g: 1.5,
                ccm_diag_b: 1.5,
                gamma: 3.0,
                contrast: 1.5,
                brightness: 0.2,
            },
        ];
        let fitnesses = vec![0.1, 2.0];
        // The best (fitness 0.1) should be selected more often
        let mut best_count = 0;
        for _ in 0..100 {
            let selected = tournament_select(&pop, &fitnesses, 2, &mut rng);
            if (selected.awb_rg_offset - 0.0).abs() < 0.01 {
                best_count += 1;
            }
        }
        assert!(best_count > 50); // Should win >50% of tournaments
    }
}
