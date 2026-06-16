//! DisplayBlock — final format conversion + orientation transform fused.
//!
//! Converts float [0,1] RGB → float [0,255] BGRA and optionally applies
//! rotation/flip (90°/180°/270°/hflip/vflip) in the same ONNX subgraph.
//! Fusing saves one full-frame memory pass vs a separate RotateBlock.
//!
//! Rotation modes (i32): 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
//! Constants defined in crate::profile::ROTATE_*.

use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct DisplayBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub target_width: u32,
    /// Orientation transform: 0=none, 1=rot90, 2=rot180, 3=rot270, 4=hflip, 5=vflip.
    pub rotate_mode: i32,
    /// Concrete input dims (set via with_concrete_dims) for Slice initializers.
    pub in_h: Option<i64>,
    pub in_w: Option<i64>,
}

impl DisplayBlock {
    pub fn new(target_width: u32) -> Self {
        Self {
            id: "display".into(),
            prev: None,
            next: None,
            frame_tensor: "DisplayBlock/frame".into(),
            input_source: String::new(),
            target_width,
            rotate_mode: 0,
            in_h: None,
            in_w: None,
        }
    }

    /// Set the rotation/flip mode (use `crate::profile::ROTATE_*` constants).
    pub fn with_rotate(mut self, mode: i32) -> Self {
        self.rotate_mode = mode;
        self
    }

    /// Set concrete input dimensions for computing Slice flip initializers.
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.in_h = Some(h);
        self.in_w = Some(w);
        self
    }

    /// Whether this mode swaps height and width (90° / 270°).
    fn swaps_dims(&self) -> bool {
        matches!(self.rotate_mode, 1 | 3) // ROTATE_ROT90 | ROTATE_ROT270
    }

    /// Whether this mode applies a horizontal flip.
    fn needs_hflip(&self) -> bool {
        matches!(self.rotate_mode, 1 | 2 | 4) // ROT90 | ROT180 | HFLIP
    }

    /// Whether this mode applies a vertical flip.
    fn needs_vflip(&self) -> bool {
        matches!(self.rotate_mode, 2 | 3 | 5) // ROT180 | ROT270 | VFLIP
    }

    /// Output dimensions after rotation.
    fn out_dims(&self) -> (i64, i64) {
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => {
                if self.swaps_dims() { (w, h) } else { (h, w) }
            }
            _ => (0, 0),
        }
    }

    /// Returns true if no rotation/flip transform is requested.
    fn is_identity(&self) -> bool {
        self.rotate_mode == 0
    }
}

impl IspBlock for DisplayBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "DisplayBlock".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    fn input_tensors(&self) -> Vec<String> { vec![self.input_source.clone()] }
    fn output_tensors(&self) -> Vec<String> { vec![self.frame_tensor.clone()] }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => Some(Proto::value_info(&self.input_source,
                &[Proto::tensor_dim_value(1), Proto::tensor_dim_param("C"),
                  Proto::tensor_dim_value(h), Proto::tensor_dim_value(w)], 1)),
            _ => Some(Proto::value_info(&self.input_source,
                &[Proto::tensor_dim_value(1), Proto::tensor_dim_param("C"),
                  Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1)),
        }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => {
                let (oh, ow) = if self.swaps_dims() { (w, h) } else { (h, w) };
                Some(Proto::value_info(&self.frame_tensor,
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_param("C"),
                      Proto::tensor_dim_value(oh), Proto::tensor_dim_value(ow)], 1))
            }
            _ => Some(Proto::value_info(&self.frame_tensor,
                &[Proto::tensor_dim_value(1), Proto::tensor_dim_param("C"),
                  Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1)),
        }
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let final_output = &self.frame_tensor;

        // Fast path: no rotation → identity (rename tensor, zero cost)
        if self.is_identity() {
            return vec![
                Proto::node("Identity", &[&self.input_source], &[final_output], &[]),
            ];
        }

        let mut nodes: Vec<Vec<u8>> = Vec::new();
        // Keep intermediate tensor names alive across the function scope
        let mut tensor_pool: Vec<String> = Vec::new();
        let mut prev = self.input_source.as_str();

        // 90° or 270°: transpose to swap H ↔ W
        if self.swaps_dims() {
            let transposed = format!("{}/transposed", ns);
            nodes.push(Proto::node("Transpose",
                &[prev], &[&transposed],
                &[Proto::attribute_ints("perm", &[0, 1, 3, 2])]));
            tensor_pool.push(transposed);
            prev = tensor_pool.last().unwrap().as_str();
        }

        // Horizontal flip on W (axis 3)
        if self.needs_hflip() {
            let hflipped = if self.needs_vflip() {
                format!("{}/hflipped", ns)
            } else {
                self.frame_tensor.clone()
            };
            let starts = format!("{}/hflip_starts", ns);
            let ends = format!("{}/hflip_ends", ns);
            let axes = format!("{}/hflip_axes", ns);
            let steps = format!("{}/hflip_steps", ns);
            nodes.push(Proto::node("Slice",
                &[prev, &starts, &ends, &axes, &steps],
                &[&hflipped], &[]));
            tensor_pool.push(hflipped);
            prev = tensor_pool.last().unwrap().as_str();
        }

        // Vertical flip on H (axis 2)
        if self.needs_vflip() {
            let vflipped = format!("{}/vflipped", ns);
            let starts = format!("{}/vflip_starts", ns);
            let ends = format!("{}/vflip_ends", ns);
            let axes = format!("{}/vflip_axes", ns);
            let steps = format!("{}/vflip_steps", ns);
            nodes.push(Proto::node("Slice",
                &[prev, &starts, &ends, &axes, &steps],
                &[&vflipped], &[]));
            tensor_pool.push(vflipped);
            prev = tensor_pool.last().unwrap().as_str();
        }

        // Final: identity rename to output tensor
        if prev != final_output {
            nodes.push(Proto::node("Identity", &[prev], &[final_output], &[]));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // No scale Mul needed — output is float [0,1] directly.
        // Consumer (display driver or engine readback) multiplies by 255.
        let mut inits: Vec<Vec<u8>> = Vec::new();

        if self.is_identity() { return inits; }

        let (h, w) = match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => (h, w),
            _ => return inits, // no concrete dims → no flip initializers
        };

        // For 90/270, transpose swaps dims so flip axes apply post-swap
        let (flip_h, flip_w) = if self.swaps_dims() { (w, h) } else { (h, w) };

        if self.needs_hflip() && flip_w > 0 {
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_starts", ns), &[flip_w - 1]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_axes", ns), &[3]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_steps", ns), &[-1]));
        }

        if self.needs_vflip() && flip_h > 0 {
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_starts", ns), &[flip_h - 1]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_axes", ns), &[2]));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_steps", ns), &[-1]));
        }

        inits
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> { vec![] }
}
