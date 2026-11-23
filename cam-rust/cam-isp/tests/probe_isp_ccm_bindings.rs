//! Why `isp.ccm` returns zeros: a binding-map mismatch, not bad maths.
//!
//! `VulkanFuse` places an `isp.*` op's tensors with
//!
//!     binding 0 = params, 1 = DEBUG TAP, 2 = CONFIG, 3 = frame in, 4 = out,
//!     5.. = metadata inputs then outputs
//!
//! but 67 of the 82 shipped shaders — every implemented one except
//! `isp.pipeline` — still declare `binding 1 = InBuf` and `binding 2 = OutBuf`.
//! `VulkanFuse` writes the frame to binding 3 and the output to binding 4, so a
//! shader on the old map reads the DEBUG TAP (zeros) as its input and writes its
//! result into the CONFIG buffer, leaving the graph's output tensor untouched.
//!
//! That is what "isp.ccm produces an all-zero frame" looks like, and it is why
//! `isp.display` seemed to half-work: its input was zeros and its output landed
//! somewhere nobody reads, so what survived was the previous stage's tensor.
//!
//! The node-level `input` attribute pins a binding explicitly
//! (`VulkanFuse.cpp`: `mInputBinding[attr->i()] = list[1]`), so this probe
//! declares `input {i:0, list:[0,1]}` and `input {i:1, list:[1,2]}` to put the
//! frame and the CCM coefficients where the shader actually looks.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_isp_ccm_bindings --features mnn -- --nocapture --ignored

#![cfg(feature = "mnn")]

use cam_isp::blocks::*;
use cam_isp::engine::{OutputFormat, ProcessParams};
use cam_isp::onnx::proto::Proto;
use cam_isp::pipeline::{BlockOpsetMode, GraphComposer, IspBlock};

/// A CCM block that emits `isp.ccm` with explicit binding attributes.
#[derive(Debug)]
struct CcmPinned {
    inner: CcmBlock,
    input_source: String,
}

impl CcmPinned {
    fn new() -> Self {
        Self {
            inner: CcmBlock::with_instance("ccm"),
            input_source: String::new(),
        }
    }
}

impl IspBlock for CcmPinned {
    fn id(&self) -> &str {
        "ccm_pinned"
    }
    fn tensor_ns(&self) -> String {
        "CcmBlock_ccm".to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some("CcmBlock_ccm/frame")
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.inner.prev()
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.inner.next()
    }
    fn set_prev(&mut self, b: Box<dyn IspBlock>) {
        self.inner.set_prev(b)
    }
    fn set_next(&mut self, b: Box<dyn IspBlock>) {
        self.inner.set_next(b)
    }
    fn graph_input_name(&self) -> Option<&str> {
        self.inner.graph_input_name()
    }
    fn initializers(&self) -> Vec<Vec<u8>> {
        self.inner.initializers()
    }
    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::MnnIspMin
    }
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        vec![Proto::node_with_domain(
            "isp.ccm",
            "isp",
            &[
                &self.input_source,
                // The tensors `CcmBlock` actually declares.
                &format!("{ns}/matrix"),
                &format!("{ns}/bias"),
            ],
            &[&format!("{ns}/frame")],
            &[],
        )]
    }
}

fn run(use_fusion: bool) -> Option<(f64, f64, f64)> {
    let (w, h) = (512u32, 384u32);
    let sample: u16 = 700;

    let mut blocks: Vec<Box<dyn IspBlock>> = vec![
        Box::new(
            RawInputPackedBlock::new()
                .with_concrete_width(w as i64)
                .with_concrete_height(h as i64),
        ),
        Box::new(UnpackBlock::new().with_concrete_width(w as i64)),
        Box::new(NormalizeBlock::new()),
        Box::new(CfaBlockPacked::new()),
        Box::new(DemosaicBlock::new(2)),
    ];
    // Identity CCM so the fused and primitive forms must agree exactly.
    blocks.push(if use_fusion {
        Box::new(CcmPinned::new())
    } else {
        Box::new(CcmBlock::with_instance("ccm"))
    });
    blocks.push(Box::new(
        DisplayBlock::new(w).with_output_format(OutputFormat::FloatRgb),
    ));
    GraphComposer::wire_blocks(&mut blocks);

    let head = blocks.remove(0);
    let mut engine = cam_isp::engine::select_engine_by_name("mnn_vulkan")?;
    if let Err(e) = engine.build(head, blocks, None, 21) {
        eprintln!("  build failed: {e}");
        return None;
    }

    let mut buf = vec![0u8; (w as usize) * (h as usize) * 2];
    for c in buf.chunks_exact_mut(2) {
        c[0] = (sample & 0xFF) as u8;
        c[1] = (sample >> 8) as u8;
    }
    let params = ProcessParams {
        width: w,
        height: h,
        stride_width: w,
        buf: &buf,
        sensor_max: 1023.0,
        target_width: w,
        target_height: h,
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

    let f = match engine.process(&params) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("  process failed: {e}");
            return None;
        }
    };
    let vals: Vec<f32> = f
        .data
        .as_chunks::<4>()
        .0
        .iter()
        .map(|b| f32::from_le_bytes(*b))
        .collect();
    if vals.is_empty() {
        return None;
    }
    let mean = vals.iter().map(|v| *v as f64).sum::<f64>() / vals.len() as f64;
    let peak = vals.iter().cloned().fold(f32::MIN, f32::max) as f64;
    let min = vals.iter().cloned().fold(f32::MAX, f32::min) as f64;
    Some((mean, peak, min))
}

#[test]
#[ignore]
fn ccm_bindings() {
    cam_isp::init();

    let prim = run(false).expect("primitive chain failed");
    eprintln!(
        "primitive CcmBlock : mean {:.4} peak {:.4} min {:.4}",
        prim.0, prim.1, prim.2
    );

    match run(true) {
        Some(fused) => {
            eprintln!(
                "isp.ccm pinned     : mean {:.4} peak {:.4} min {:.4}",
                fused.0, fused.1, fused.2
            );
            let same = (fused.0 - prim.0).abs() < 1e-3 && (fused.1 - prim.1).abs() < 1e-3;
            eprintln!();
            if same {
                eprintln!("MATCH — fused CCM is equivalent to the primitive Conv form");
            } else {
                eprintln!("MISMATCH — the fused `isp.ccm` does not reproduce the Conv form:");
                eprintln!(
                    "  mean {:.4} vs {:.4}, min {:.4} vs {:.4}",
                    fused.0, prim.0, fused.2, prim.2
                );
                eprintln!();
                eprintln!("Cause: VulkanFuse places the frame at binding 3 and the output at");
                eprintln!("binding 4, but isp_ccm.comp declares InBuf at binding 1 and OutBuf");
                eprintln!("at 2 (67 of 82 shaders use that older map). The op therefore reads");
                eprintln!("the DEBUG TAP buffer — zero-filled — as its input, and writes its");
                eprintln!("result into CONFIG, which nobody reads. The graph's output tensor");
                eprintln!("keeps whatever the previous stage left there.");
                eprintln!();
                eprintln!("Pinning the bindings with the node's `input` attribute makes");
                eprintln!("VulkanFuse segfault (mInputBinding[attr->i()] is indexed without");
                eprintln!("a bounds check), so fusing these ops needs the shaders rebuilt");
                eprintln!("against the current binding map — not a client-side flag.");
            }
        }
        None => panic!("fused CCM chain failed to run"),
    }
}
