//! Does folding white balance into `isp.demosaic_ccm` change the picture?
//!
//! WB is a per-plane scalar and the fused demosaic already multiplies the CFA quad
//! by a 3x4 matrix, so the two are one step — but "obviously equivalent" is how the
//! tone-double-apply bug got in. This compares the two spellings pixel for pixel.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_wb_fused --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 1296;
const H: u32 = 970;

fn render(wb_fused: bool) -> Option<(Vec<f32>, f64)> {
    let mut profile = PipelineProfile::FUSED;
    profile.output_format = OutputFormat::FloatRgb;
    profile.opset_mode = 2;
    profile.fuse_stages = if wb_fused {
        FuseStages { ..FuseStages::ALL }
    } else {
        // The separate-WB arm must disable the fused demosaic+CCM op too:
        // block presence is driven by `demosaic_ccm`, not `wb_into_demosaic`,
        // so only clearing the latter would leave both arms without a WB block
        // and make the probe compare a frame with itself.
        FuseStages {
            demosaic_ccm: false,
            wb_into_demosaic: false,
            ..FuseStages::ALL
        }
    };

    let mut blocks = profile.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

    // A patterned frame: a flat field is neutral, so a WB mix-up is invisible.
    let raw = cam_isp::synth_bayer::bayer_color_diffuse(W, H);
    let mut params = ProcessParams::new(W, H, &raw);
    params.sensor_max = 1023.0;
    params.target_width = W;
    params.target_height = H;
    params.tone_params = cam_isp::engine::default_tone_params();
    // A deliberately non-neutral balance, so a lost or misordered gain shows.
    params.bayer_pattern = 2; // GRBG
    params.output_format = OutputFormat::FloatRgb;

    for _ in 0..5 {
        engine.process(&params).ok()?;
    }
    let mut times = Vec::new();
    let mut out = None;
    for _ in 0..9 {
        let f = engine.process(&params).ok()?;
        times.push(f.inference_duration_ns as f64 / 1e6);
        out = Some(
            f.data
                .as_chunks::<4>()
                .0
                .iter()
                .map(|b| f32::from_le_bytes(*b))
                .collect::<Vec<f32>>(),
        );
    }
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some((out?, times[times.len() / 2]))
}

#[test]
#[ignore]
fn wb_fold_matches() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let (separate, ms_sep) = render(false).expect("separate WB failed");
    let (folded, ms_folded) = render(true).expect("folded WB failed");

    assert_eq!(
        separate.len(),
        folded.len(),
        "the two spellings disagree about the output size"
    );

    let mean = |v: &[f32]| v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    let mut sum = 0.0f64;
    let mut peak = 0.0f64;
    for (a, b) in separate.iter().zip(folded.iter()) {
        let d = (*a as f64 - *b as f64).abs();
        sum += d;
        peak = peak.max(d);
    }
    let dm = sum / separate.len() as f64;

    eprintln!("\n{:<22} {:>9} {:>8}", "white balance", "ms", "mean");
    eprintln!(
        "{:<22} {ms_sep:>9.1} {:>8.4}",
        "separate block",
        mean(&separate)
    );
    eprintln!(
        "{:<22} {ms_folded:>9.1} {:>8.4}",
        "folded into demosaic",
        mean(&folded)
    );
    eprintln!("difference: mean|Δ| {dm:.6}  peak|Δ| {peak:.6}");

    assert!(mean(&folded) > 0.05, "the folded chain rendered nothing");
    assert!(
        dm < 0.002 && peak < 0.02,
        "folding white balance changed the picture (mean|Δ| {dm:.5}, peak|Δ| {peak:.5})"
    );
}
