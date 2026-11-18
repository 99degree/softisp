//! MED vs FUSED vs WHOLE, on real patterns, with the 3A loop held open.
//!
//! A flat grey field is a weak test: it has no chroma to lose, so a colour cast or
//! a box-filter demosaic looks identical to an edge-directed one. This renders all
//! three chains from the same four synthetic patterns with the *same* explicit
//! parameters (`isp_params`, so the controller never moves them), reports what each
//! produces, and writes the frames as PNGs to `<repo>/frames/compare/` so they can
//! be looked at rather than inferred from statistics.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_profile_compare --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::engine::{IspEngine, OutputFormat, ProcessParams};
use cam_isp::isp_params::{BlcParams, CcmParams, IspParams, ToneParams, WbParams};
use cam_isp::mnnengine::{MnnBackend, MnnEngine};
use cam_isp::profile::PipelineProfile;

const W: u32 = 1296;
const H: u32 = 970;

struct Frame {
    label: &'static str,
    ms: f64,
    values: Vec<f32>,
    w: u32,
    h: u32,
}

impl Frame {
    fn mean(&self) -> f64 {
        self.values.iter().map(|v| *v as f64).sum::<f64>() / self.values.len().max(1) as f64
    }
    /// Plane stride, derived from the data rather than from the requested
    /// geometry: a fused chain decides its own output size, and indexing past the
    /// end of the buffer makes the per-channel means nonsense.
    fn plane(&self) -> usize {
        self.values.len() / 3
    }

    /// Per-channel means, so a cast is a vector rather than a hunch.
    fn means_rgb(&self) -> [f64; 3] {
        let plane = self.plane();
        let n = plane.max(1) as f64;
        let mut m = [0.0f64; 3];
        for c in 0..3 {
            let lo = c * plane;
            let hi = (lo + plane).min(self.values.len());
            m[c] = self.values[lo..hi].iter().map(|v| *v as f64).sum::<f64>() / n;
        }
        m
    }
    fn stddev(&self) -> f64 {
        let m = self.mean();
        let var = self
            .values
            .iter()
            .map(|v| (*v as f64 - m).powi(2))
            .sum::<f64>()
            / self.values.len().max(1) as f64;
        var.sqrt()
    }
    /// Pearson correlation of two frames' samples.
    ///
    /// Requires identical size: a chain that renders at a different geometry is
    /// not comparable sample-for-sample.
    fn correlate(&self, other: &Frame) -> Option<f64> {
        if self.values.len() != other.values.len() {
            eprintln!(
                "      (not comparable: {} values vs {} — {}x{} vs {}x{})",
                self.values.len(),
                other.values.len(),
                self.w,
                self.h,
                other.w,
                other.h
            );
            return None;
        }
        let a = &self.values;
        let b = &other.values;
        let n = a.len() as f64;
        let ma = self.mean();
        let mb = other.mean();
        let mut num = 0.0;
        let mut da = 0.0;
        let mut db = 0.0;
        for (x, y) in a.iter().zip(b.iter()) {
            let dx = *x as f64 - ma;
            let dy = *y as f64 - mb;
            num += dx * dy;
            da += dx * dx;
            db += dy * dy;
        }
        let den = (da * db).sqrt();
        if den <= 0.0 {
            None
        } else {
            Some(num / den)
        }
    }
    fn write_png(&self, dir: &std::path::Path, tag: &str) {
        let plane = self.plane();
        let mut rgb = vec![0u8; plane * 3];
        for i in 0..plane {
            for c in 0..3 {
                let v = self.values.get(c * plane + i).copied().unwrap_or(0.0);
                rgb[i * 3 + c] = (v * 255.0).clamp(0.0, 255.0) as u8;
            }
        }
        let path = dir.join(format!("{tag}.png"));
        if image::save_buffer(&path, &rgb, self.w, self.h, image::ColorType::Rgb8).is_ok() {
            eprintln!("      wrote {}", path.display());
        }
    }
}

fn fixed_params() -> IspParams {
    let mut p = IspParams::default();
    // A mild daylight balance and a real (non-identity) matrix, so the colour path
    // is exercised rather than bypassed.
    p.blc = BlcParams {
        r: 64.0,
        gr: 64.0,
        gb: 64.0,
        b: 64.0,
    };
    p.wb = WbParams {
        r: 1.9,
        g: 1.0,
        b: 1.6,
    };
    p.ccm = CcmParams {
        matrix: [
            1.32, -0.26, -0.06, //
            -0.14, 1.24, -0.10, //
            0.02, -0.30, 1.28,
        ],
    };
    // `isp_params::ToneParams` carries the display gamma, not the shader's
    // reciprocal, and has no saturation field: saturation travels with the
    // controller's own ToneParams, which this probe deliberately holds still.
    p.tone = ToneParams {
        contrast: 1.2,
        brightness: 0.05,
        gamma: 2.2,
        ..Default::default()
    };
    p
}

fn render(profile: PipelineProfile, raw: &[u8], pattern: i32) -> Option<Frame> {
    let mut profile = profile;
    profile.output_format = OutputFormat::FloatRgb;
    let label = profile.label;

    let mut blocks = profile.build_blocks(W, H, pattern);
    let head = blocks.remove(0);
    let aux = profile.build_aux_blocks(H as i64, W as i64);
    blocks.extend(aux);

    let mut engine = MnnEngine::with_pool_size(MnnBackend::Vulkan, 2);
    engine.build(head, blocks, None, 21).ok()?;

    let params = ProcessParams {
        width: W,
        height: H,
        stride_width: W,
        buf: raw,
        sensor_max: 1023.0,
        target_width: W,
        target_height: H,
        ccm_matrix: None,
        tone_params: cam_isp::engine::default_tone_params(),
        bayer_gains: None,
        awb_gains: None,
        bayer_pattern: pattern,
        analog_gain: 1.0,
        scene_change: 0.0,
        lsc_gains: None,
        blc_values: None,
        warp_grid: None,
        warp_shading: None,
        output_format: OutputFormat::FloatRgb,
        timestamp_ns: 0,
        isp_params: Some(fixed_params()),
    };

    for _ in 0..5 {
        engine.process(&params).ok()?;
    }
    let mut times = Vec::new();
    let mut last = None;
    for _ in 0..9 {
        let f = engine.process(&params).ok()?;
        times.push(f.inference_duration_ns as f64 / 1e6);
        last = Some(f);
    }
    let f = last?;
    times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    Some(Frame {
        label: match label {
            "MED" => "MED",
            "FUSED" => "FUSED",
            "WHOLE" => "WHOLE",
            _ => "?",
        },
        ms: times[times.len() / 2],
        values: f
            .data
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .collect(),
        w: f.width,
        h: f.height,
    })
}

#[test]
#[ignore]
fn compare_profiles() {
    let _ = env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .is_test(false)
        .try_init();
    cam_isp::init();

    let out_dir = std::path::PathBuf::from("../frames/compare");
    let _ = std::fs::create_dir_all(&out_dir);

    let patterns: [(&str, fn(u32, u32) -> Vec<u8>); 4] = [
        ("grey", |w, h| {
            // A flat mid field: the weakest test, and the one every earlier
            // measurement used.
            let mut v = vec![0u8; (w * h * 2) as usize];
            for c in v.chunks_exact_mut(2) {
                let s: u16 = 512;
                c[0] = (s & 0xFF) as u8;
                c[1] = (s >> 8) as u8;
            }
            v
        }),
        ("rainbow", cam_isp::synth_bayer::bayer_rainbow),
        ("diffuse", cam_isp::synth_bayer::bayer_color_diffuse),
        ("stress", cam_isp::synth_bayer::bayer_stress),
    ];

    for (tag, gen) in patterns {
        let raw = gen(W, H);
        eprintln!("\n════ {tag} {W}x{H} ════");
        eprintln!(
            "{:<7} {:>8} {:>9} {:>9} {:>18} {:>9}",
            "chain", "ms", "mean", "stddev", "rgb means", "vs MED"
        );

        let med = render(PipelineProfile::MED, &raw, 2);
        let Some(med) = med else {
            eprintln!("MED failed on {tag}");
            continue;
        };
        med.write_png(&out_dir, &format!("{tag}-med"));

        let mut rows = vec![(
            med.label,
            med.ms,
            med.mean(),
            med.stddev(),
            med.means_rgb(),
            1.0,
        )];
        for profile in [PipelineProfile::FUSED, PipelineProfile::WHOLE] {
            match render(profile, &raw, 2) {
                Some(f) => {
                    let corr = f.correlate(&med).unwrap_or(f64::NAN);
                    f.write_png(&out_dir, &format!("{tag}-{}", f.label.to_lowercase()));
                    eprintln!(
                        "      {} emitted {}x{} ({} values)",
                        f.label,
                        f.w,
                        f.h,
                        f.values.len()
                    );
                    rows.push((f.label, f.ms, f.mean(), f.stddev(), f.means_rgb(), corr));
                }
                None => eprintln!("{} failed on {tag}", profile.label),
            }
        }
        for (label, ms, mean, sd, rgb, corr) in rows {
            eprintln!(
                "{label:<7} {ms:>8.1} {mean:>9.4} {sd:>9.4}  [{:>5.3},{:>5.3},{:>5.3}] {corr:>9.4}",
                rgb[0], rgb[1], rgb[2]
            );
        }
    }

    eprintln!(
        "\n\"vs MED\" is the Pearson correlation of the two frames' samples.\n\
         1.0 = the same structure; below ~0.9 a chain is rendering something else."
    );
}
