//! Does the shared statistics downscale hold the invariant, and the numbers?
//!
//! Every statistics block now reads one pooled tensor instead of each shrinking the
//! frame for itself. Two things have to be true: the chain never grows between
//! stages (`check_extent_monotonicity`), and the values the controller receives are
//! the same as at full resolution.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_stats_cascade --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::pipeline::GraphComposer;
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

fn build(stats_max: u32) -> Option<(Vec<String>, Vec<String>)> {
    let mut p = PipelineProfile::FUSED;
    p.opset_mode = 2;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);

    // The chain as the composer sees it: the pipeline, then the aux branches.
    let refs: Vec<&dyn cam_isp::pipeline::IspBlock> = std::iter::once(head.as_ref())
        .chain(blocks.iter().map(|b| b.as_ref()))
        .chain(aux.iter().map(|b| b.as_ref()))
        .collect();

    let extents = GraphComposer::check_extent_monotonicity(&refs);
    let ids: Vec<String> = aux
        .iter()
        .map(|b| format!("{} {:?}", b.id(), b.declared_extents()))
        .collect();
    Some((extents, ids))
}

fn render(stats_max: u32) -> Option<([f32; 3], f64)> {
    let mut p = PipelineProfile::FUSED;
    p.output_format = OutputFormat::FloatRgb;
    p.opset_mode = 2;
    p.stats_downscale_max = stats_max;

    let mut blocks = p.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = p.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

    let raw = cam_isp::synth_bayer::bayer_color_diffuse(W, H);
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &raw,
        sensor_max: 1023.0,
        target_width: W,
        target_height: H,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: 2,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: None,
        input_buffer: None,
        output_buffer: None,
        input_ahb: None,
        output_ahb: None,
    };
    for _ in 0..5 {
        engine.process(&params).ok()?;
    }
    let f = engine.process(&params).ok()?;
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len().max(1) as f64;
    Some((
        f.aux
            .as_ref()?
            .calibration_stats
            .map(|c| [c[0], c[1], c[2]])?,
        mean,
    ))
}

#[test]
#[ignore]
fn stats_cascade() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    for stats_max in [0u32, 648, 324] {
        let tag = if stats_max == 0 {
            "full".to_string()
        } else {
            format!("{stats_max} edge")
        };
        match build(stats_max) {
            Some((issues, ids)) => {
                eprintln!("\n== {tag}: {} aux blocks ==", ids.len());
                for i in &ids {
                    eprintln!("   {i}");
                }
                if issues.is_empty() {
                    eprintln!("   extents: monotonic");
                } else {
                    for i in &issues {
                        eprintln!("   VIOLATION: {i}");
                    }
                }
            }
            None => eprintln!("{tag}: build failed"),
        }
        if let Some((cal, mean)) = render(stats_max) {
            eprintln!(
                "   frame mean {mean:.4}, calibration quad means [{:.4}, {:.4}, {:.4}]",
                cal[0], cal[1], cal[2]
            );
        } else {
            eprintln!("   render failed");
        }
    }
}
