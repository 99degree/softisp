//! Does the graph declare a block's *second* output?
//!
//! `OpType_Extra`'s shape computer sizes `outputs[0]` and leaves any further
//! outputs copying the *input* shape — so an op that writes a second tensor needs
//! the graph to declare it, and MNN needs a shape rule for it. This checks the
//! half softisp owns: that the composer registers every output a block reports.
//!
//! Run:
//!   cargo test -p cam-isp --test probe_multi_output -- --nocapture

use cam_isp::pipeline::{GraphComposer, IspBlock};

/// A block with two outputs, and a consumer of the second one.
#[derive(Debug)]
struct TwoOut {
    id: &'static str,
    frame: &'static str,
    aux: &'static str,
    input: String,
    prev: Option<Box<dyn IspBlock>>,
    next: Option<Box<dyn IspBlock>>,
}

impl IspBlock for TwoOut {
    fn id(&self) -> &str {
        self.id
    }
    fn tensor_ns(&self) -> String {
        self.id.to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(self.frame)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }
    fn set_prev(&mut self, b: Box<dyn IspBlock>) {
        self.prev = Some(b);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }
    fn set_next(&mut self, b: Box<dyn IspBlock>) {
        self.next = Some(b);
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame.to_string(), self.aux.to_string()]
    }
    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![]
    }
    fn graph_input_name(&self) -> Option<&str> {
        Some(&self.input)
    }
    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(cam_isp::onnx::proto::Proto::value_info(
            &self.input,
            &[
                cam_isp::onnx::proto::Proto::tensor_dim_value(1),
                cam_isp::onnx::proto::Proto::tensor_dim_value(1),
                cam_isp::onnx::proto::Proto::tensor_dim_value(8),
                cam_isp::onnx::proto::Proto::tensor_dim_value(8),
            ],
            1,
        ))
    }
    fn graph_output_names(&self) -> Vec<&str> {
        vec![self.frame, self.aux]
    }
    fn graph_output_value_infos(&self) -> Vec<Vec<u8>> {
        vec![
            cam_isp::onnx::proto::Proto::value_info(
                self.frame,
                &[
                    cam_isp::onnx::proto::Proto::tensor_dim_value(1),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(3),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(8),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(8),
                ],
                1,
            ),
            cam_isp::onnx::proto::Proto::value_info(
                self.aux,
                &[
                    cam_isp::onnx::proto::Proto::tensor_dim_value(1),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(4),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(2),
                    cam_isp::onnx::proto::Proto::tensor_dim_value(2),
                ],
                1,
            ),
        ]
    }
}

#[test]
fn second_output_is_registered() {
    cam_isp::init();

    let mut a = Box::new(TwoOut {
        id: "two_out",
        frame: "TwoOut/frame",
        aux: "TwoOut/stat",
        input: "raw".to_string(),
        prev: None,
        next: None,
    });
    a.set_input_source("raw");
    let refs: Vec<&dyn IspBlock> = vec![a.as_ref()];
    let onnx = GraphComposer::compose_from_vec(&refs, &[], 21).expect("compose");

    for name in ["TwoOut/frame", "TwoOut/stat"] {
        let present = onnx.windows(name.len()).any(|w| w == name.as_bytes());
        assert!(present, "{name} is missing from the graph");
    }
}
