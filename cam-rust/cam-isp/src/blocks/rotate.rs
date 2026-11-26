//! RotateBlock — standalone orientation stage for the display frame.
//!
//! Applies `rotate_mode` while the frame is still float RGB `[1,3,H,W]`, before
//! `DisplayBlock` packs it into the display format. Only multiples of 90° are
//! supported: a 90°/270° turn is one `Transpose`, and every flip is a `Gather`
//! with a reversed index initializer. Both are primitive ONNX ops with no
//! custom `isp.*` shader and no float sampling grid, so arbitrary-angle warps
//! are explicitly out of scope here and belong in a later GPU warp stage.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Step {
    Transpose,
    HFlip,
    VFlip,
}

/// Orientation transform: 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
#[derive(Debug)]
pub struct RotateBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub rotate_mode: i32,
    /// Concrete input geometry, `(in_h, in_w)`.
    pub in_h: i64,
    pub in_w: i64,
}

impl RotateBlock {
    pub fn new(rotate_mode: i32, in_h: i64, in_w: i64) -> Self {
        Self {
            id: "rotate".into(),
            prev: None,
            next: None,
            frame_tensor: "RotateBlock/frame".into(),
            input_source: String::new(),
            rotate_mode,
            in_h,
            in_w,
        }
    }

    pub fn out_dims(&self) -> (i64, i64) {
        if self.swaps_dims() {
            (self.in_w, self.in_h)
        } else {
            (self.in_h, self.in_w)
        }
    }

    fn swaps_dims(&self) -> bool {
        matches!(self.rotate_mode, 1 | 3)
    }

    /// Decomposition shared by node emission, initializers, and the permutation
    /// tests. Keeping one source of truth prevents the graph and its declared
    /// geometry from drifting apart.
    fn steps(&self) -> Vec<Step> {
        match self.rotate_mode {
            1 => vec![Step::Transpose, Step::HFlip],
            2 => vec![Step::HFlip, Step::VFlip],
            3 => vec![Step::Transpose, Step::VFlip],
            4 => vec![Step::HFlip],
            5 => vec![Step::VFlip],
            _ => vec![],
        }
    }

    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("RotateBlock/input")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }

    fn flip_indices(len: i64) -> Vec<i64> {
        if len <= 0 {
            return vec![];
        }
        (0..len).rev().collect()
    }
}

impl IspBlock for RotateBlock {
    fn id(&self) -> &str {
        &self.id
    }

    fn tensor_ns(&self) -> String {
        "RotateBlock".to_string()
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }

    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }

    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev = Some(block);
    }

    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }

    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next = Some(block);
    }

    fn input_tensors(&self) -> Vec<String> {
        vec![self.effective_input()]
    }

    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.effective_input(),
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(self.in_h),
                Proto::tensor_dim_value(self.in_w),
            ],
            1,
        ))
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let (out_h, out_w) = self.out_dims();
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(out_h),
                Proto::tensor_dim_value(out_w),
            ],
            1,
        ))
    }

    fn declared_extents(&self) -> Option<((i64, i64), (i64, i64))> {
        Some(((self.in_h, self.in_w), self.out_dims()))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let steps = self.steps();
        if steps.is_empty() {
            return vec![Proto::node(
                "Identity",
                &[&self.effective_input()],
                &[&self.frame_tensor],
                &[],
            )];
        }

        let ns = self.tensor_ns();
        let mut prev = self.effective_input();
        let mut nodes = Vec::with_capacity(steps.len());
        for (i, step) in steps.iter().enumerate() {
            let out = if i + 1 == steps.len() {
                self.frame_tensor.clone()
            } else {
                format!("{ns}/step{i}")
            };
            nodes.push(match step {
                Step::Transpose => Proto::node(
                    "Transpose",
                    &[&prev],
                    &[&out],
                    &[Proto::attribute_ints("perm", &[0, 1, 3, 2])],
                ),
                Step::HFlip => Proto::node(
                    "Gather",
                    &[&prev, &format!("{ns}/hflip_indices")],
                    &[&out],
                    &[Proto::attribute_int("axis", 3)],
                ),
                Step::VFlip => Proto::node(
                    "Gather",
                    &[&prev, &format!("{ns}/vflip_indices")],
                    &[&out],
                    &[Proto::attribute_int("axis", 2)],
                ),
            });
            prev = out;
        }
        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut h = self.in_h;
        let mut w = self.in_w;
        let mut out = Vec::new();
        for step in self.steps() {
            match step {
                Step::Transpose => std::mem::swap(&mut h, &mut w),
                Step::HFlip => out.push(Proto::tensor_proto_int64(
                    &format!("{ns}/hflip_indices"),
                    &Self::flip_indices(w),
                )),
                Step::VFlip => out.push(Proto::tensor_proto_int64(
                    &format!("{ns}/vflip_indices"),
                    &Self::flip_indices(h),
                )),
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn op_type(node: &[u8]) -> &str {
        assert_eq!(node[0], 0x22, "NodeProto.op_type must be field 4");
        let mut i = 1;
        let mut len = 0usize;
        let mut shift = 0;
        loop {
            let b = node[i];
            i += 1;
            len |= ((b & 0x7f) as usize) << shift;
            if b & 0x80 == 0 {
                break;
            }
            shift += 7;
        }
        std::str::from_utf8(&node[i..i + len]).unwrap()
    }

    fn apply_steps(mut img: Vec<Vec<i32>>, steps: &[Step]) -> Vec<Vec<i32>> {
        for step in steps {
            match step {
                Step::Transpose => {
                    let h = img.len();
                    let w = img[0].len();
                    let mut out = vec![vec![0; h]; w];
                    for y in 0..h {
                        for x in 0..w {
                            out[x][y] = img[y][x];
                        }
                    }
                    img = out;
                }
                Step::HFlip => {
                    for row in &mut img {
                        row.reverse();
                    }
                }
                Step::VFlip => img.reverse(),
            }
        }
        img
    }

    #[test]
    fn rotate_output_dims_follow_mode() {
        for (mode, expected) in [
            (0, (480, 640)),
            (1, (640, 480)),
            (2, (480, 640)),
            (3, (640, 480)),
            (4, (480, 640)),
            (5, (480, 640)),
        ] {
            assert_eq!(
                RotateBlock::new(mode, 480, 640).out_dims(),
                expected,
                "rotate_mode {mode}"
            );
        }
    }

    #[test]
    fn rotation_uses_only_transpose_and_gather() {
        for mode in 0..=5 {
            let mut block = RotateBlock::new(mode, 480, 640);
            block.set_input_source("tone/frame");
            let nodes = block.nodes();
            let actual: Vec<&str> = nodes.iter().map(|n| op_type(n)).collect();
            let expected: &[&str] = match mode {
                0 => &["Identity"],
                1 => &["Transpose", "Gather"],
                2 => &["Gather", "Gather"],
                3 => &["Transpose", "Gather"],
                4 => &["Gather"],
                5 => &["Gather"],
                _ => unreachable!(),
            };
            assert_eq!(actual, expected, "rotate_mode {mode}");
            assert!(
                !actual.iter().any(|t| *t == "GridSample" || *t == "Slice"),
                "rotate_mode {mode} must not use sampling or Slice"
            );
        }
    }

    #[test]
    fn flip_indices_are_initializers_not_runtime_inputs() {
        for (mode, expected_count) in [(0, 0), (1, 1), (2, 2), (3, 1), (4, 1), (5, 1)] {
            let block = RotateBlock::new(mode, 480, 640);
            assert_eq!(
                block.initializers().len(),
                expected_count,
                "rotate_mode {mode}"
            );
            assert!(block.extra_inputs().is_empty(), "rotate_mode {mode}");
        }
    }

    #[test]
    fn rotation_permutation_matches_expected_pixel_mapping() {
        let in_h = 5i64;
        let in_w = 7i64;
        for mode in 0..=5 {
            let block = RotateBlock::new(mode, in_h, in_w);
            let img = (0..in_h)
                .map(|y| (0..in_w).map(|x| (y * in_w + x) as i32).collect())
                .collect::<Vec<Vec<i32>>>();
            let out = apply_steps(img, &block.steps());
            let (out_h, out_w) = block.out_dims();
            assert_eq!(out.len(), out_h as usize, "mode {mode}");
            assert_eq!(out[0].len(), out_w as usize, "mode {mode}");

            for (oy, ox) in [(0, 0), (out_h - 1, out_w - 1), (out_h / 2, out_w / 2)] {
                let (sy, sx) = match mode {
                    0 => (oy, ox),
                    1 => (in_h - 1 - ox, oy),
                    2 => (in_h - 1 - oy, in_w - 1 - ox),
                    3 => (ox, in_w - 1 - oy),
                    4 => (oy, in_w - 1 - ox),
                    5 => (in_h - 1 - oy, ox),
                    _ => unreachable!(),
                };
                assert_eq!(
                    out[oy as usize][ox as usize],
                    (sy * in_w + sx) as i32,
                    "mode {mode} at ({oy},{ox})"
                );
            }
        }
    }
}
