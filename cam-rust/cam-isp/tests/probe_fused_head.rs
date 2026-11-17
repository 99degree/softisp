//! Does the fused head replace unpack+normalize+blc+cfa without changing the
//! frame?
//!
//! `FuseStages::unpack_blc_normal_cfa` swaps five head blocks — `RawInputPacked
//! + Unpack + Normalize + Blc + CfaPacked` — for one `isp.unpack_blc_normal_cfa`
//! dispatch, and suppresses the two BLC blocks so the black level is not
//! subtracted twice. The rest of the chain is untouched, so the two renders must
//! agree.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_fused_head --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::profile::{FuseStages, PipelineProfile};

const W: u32 = 512;
const H: u32 = 384;

fn render(fused_head: bool) -> Result<(Vec<u8>, f64), String> {
    let mut profile = PipelineProfile::MED;
    profile.output_format = OutputFormat::FloatRgb;
    // MnnIspMin is what makes the allowlist meaningful; only the head is on it.
    profile.opset_mode = if fused_head { 2 } else { 0 };
    profile.fuse_stages = FuseStages {
        unpack_blc_normal_cfa: fused_head,
        ..FuseStages::NONE
    };

    let mut blocks = profile.build_blocks(W, H, 2);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")
        .ok_or_else(|| "mnn_vulkan engine not registered".to_string())?;
    engine
        .build(head, blocks, None, 21)
        .map_err(|e| format!("build: {e}"))?;

    let sample: u16 = 600;
    let mut buf = vec![0u8; (W as usize) * (H as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: &buf,
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
    };

    engine.process(&params).map_err(|e| format!("first: {e}"))?;
    let f = engine
        .process(&params)
        .map_err(|e| format!("second: {e}"))?;
    Ok((f.data, f.inference_duration_ns as f64 / 1e6))
}

fn floats(data: &[u8]) -> Vec<f32> {
    data.as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect()
}

#[test]
#[ignore]
fn fused_head_matches_the_primitive_head() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let (prim, prim_ms) = render(false).expect("primitive head failed");
    let (fused, fused_ms) = render(true).expect("fused head failed");

    let pv = floats(&prim);
    let fv = floats(&fused);
    eprintln!(
        "primitive {:>7.1} ms  {} values\nfused     {:>7.1} ms  {} values",
        prim_ms,
        pv.len(),
        fused_ms,
        fv.len()
    );

    assert_eq!(
        pv.len(),
        fv.len(),
        "the fused head changed the frame geometry ({} vs {} values)",
        pv.len(),
        fv.len()
    );

    let mean = |v: &[f32]| v.iter().map(|x| *x as f64).sum::<f64>() / v.len() as f64;
    let (pm, fm) = (mean(&pv), mean(&fv));
    eprintln!("means: primitive {pm:.4}  fused {fm:.4}");
    assert!(
        fm > 0.05,
        "fused head rendered a blank frame (mean {fm:.4})"
    );

    let mut sum = 0.0f64;
    let mut peak = 0.0f64;
    for (a, b) in pv.iter().zip(fv.iter()) {
        let d = (*a as f64 - *b as f64).abs();
        sum += d;
        if d > peak {
            peak = d;
        }
    }
    let dm = sum / pv.len() as f64;
    eprintln!("difference mean|Δ| {dm:.5}  peak|Δ| {peak:.5}");

    assert!(
        dm < 0.02 && peak < 0.25,
        "the fused head does not reproduce the primitive head \
         (mean|Δ| {dm:.4}, peak|Δ| {peak:.4})"
    );
}
