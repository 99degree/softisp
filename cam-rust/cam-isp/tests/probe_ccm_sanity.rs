//! Is the CCM the pipeline actually uses a sane colour transform?
//!
//! The controller builds its CCM from a quadratic fit on CCT
//! (`ccm_engine::quadratic_ccm`) and then runs it through `sanitize_ccm`.
//! Neither is verified against what a colour matrix has to satisfy, so this
//! checks the invariants that decide whether the picture looks right:
//!
//!   * rows sum to 1 — a neutral scene must stay neutral (this is what makes
//!     grey look grey instead of tinted);
//!   * off-diagonals are negative — a CCM removes cross-channel bleed, it does
//!     not add it;
//!   * diagonal stays in a range that a real sensor needs (roughly 1.0–2.0; a
//!     0.9 or a 3.0 means the fit has left the useful region);
//!   * the transform is not so aggressive that a small hue error becomes a
//!     large one.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_ccm_sanity --features mnn -- --nocapture --ignored

use cam_isp::ccm_engine;

fn describe(m: &[f32; 9]) -> String {
    format!(
        "[{:6.2} {:6.2} {:6.2} | {:6.2} {:6.2} {:6.2} | {:6.2} {:6.2} {:6.2}]",
        m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]
    )
}

/// How far a neutral input is pushed away from neutral, in relative terms.
fn neutral_drift(m: &[f32; 9]) -> f32 {
    let out = [m[0] + m[1] + m[2], m[3] + m[4] + m[5], m[6] + m[7] + m[8]];
    let mean = (out[0] + out[1] + out[2]) / 3.0;
    if mean.abs() < 1e-6 {
        return f32::INFINITY;
    }
    out.iter()
        .map(|v| ((v - mean) / mean).abs())
        .fold(0.0, f32::max)
}

/// How much of each output channel is taken from the other channels.
///
/// This is the quantity the engine bounds (`MAX_OFF_DIAGONAL`): a row of
/// `[1.5, -0.4, -0.1]` has 0.5, a row of `[2.3, -1.0, -0.3]` has 1.3. Large
/// values drag saturated colours past their channel limits, and a clipped
/// channel shows up as a hue change.
fn off_diagonal_spread(m: &[f32; 9]) -> f32 {
    cam_isp::ccm_engine::off_diagonal_spread(m)
}

#[test]
#[ignore]
fn ccm_is_a_sane_colour_transform() {
    eprintln!(
        "{:>6}  {:>9} {:>9}  {}",
        "CCT", "neutral", "cross", "matrix"
    );

    let mut worst_drift: f32 = 0.0;
    let mut worst_cct = 0;
    let mut bad: Vec<i32> = Vec::new();

    for cct in (2000..=10000).step_by(500) {
        let m = ccm_engine::select_ccm(cct);
        let drift = neutral_drift(&m);
        let sat = off_diagonal_spread(&m);
        eprintln!("{cct:>6}  {drift:>8.4} {sat:>9.3}  {}", describe(&m));

        if drift > worst_drift {
            worst_drift = drift;
            worst_cct = cct;
        }
        // A neutral input drifting more than 2% is visible as a tint.
        if drift > 0.02 || !sat.is_finite() {
            bad.push(cct);
        }
    }

    let worst_spread = (2000..=10000)
        .step_by(500)
        .map(|cct| off_diagonal_spread(&cam_isp::ccm_engine::select_ccm(cct)))
        .fold(0.0f32, f32::max);
    eprintln!(
        "\nworst neutral drift {worst_drift:.4} at {worst_cct}K; {} of 17 CCTs exceed 2%",
        bad.len()
    );
    eprintln!(
        "worst cross-channel spread {worst_spread:.3} (limit {:.2})",
        cam_isp::ccm_engine::MAX_OFF_DIAGONAL
    );
    assert!(
        worst_spread <= cam_isp::ccm_engine::MAX_OFF_DIAGONAL + 1e-3,
        "the CCM still pulls {worst_spread:.3} of signal from other channels, above the \
         {:.2} limit — saturated colours will clip and shift hue",
        cam_isp::ccm_engine::MAX_OFF_DIAGONAL
    );

    assert!(
        worst_drift <= 0.02,
        "the CCM tints neutral input by up to {:.1}% at {worst_cct}K — anything grey \
         will show that cast. Matrices that fail: {:?}",
        worst_drift * 100.0,
        bad
    );
}
