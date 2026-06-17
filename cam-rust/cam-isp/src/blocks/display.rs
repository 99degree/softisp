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
    /// INT32 packed RGBA (Mul+ReduceSum+Cast). Unreliable on OpenCL.
    pub pack_rgba: bool,
    /// BGRA float [0,255] via Conv(1×1): does channel swap + mul(255) + alpha
    /// in one ONNX op. Rust to_bgra becomes trivial f32→u8 truncation.
    pub bg4a: bool,
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
            pack_rgba: false,
            bg4a: false,
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
    /// Enable packed RGBA output: Mul(255) + Mul(weights) + ReduceSum + Cast(INT32).
    /// Output is [1,1,H,W] INT32 where each value = R*65536 + G*256 + B.
    pub fn with_pack_rgba(mut self, enable: bool) -> Self {
        self.pack_rgba = enable;
        self
    }
    /// Enable BGRA float [0,255] output via Conv(1×1).
    /// Does channel swap (RGB→BGR), multiply by 255, and adds alpha=255.
    /// Output: [1,4,H,W] FLOAT [0,255] — Rust only needs f32→u8 truncation.
    pub fn with_bg4a(mut self, enable: bool) -> Self {
        self.bg4a = enable;
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

    /// Returns true if no rotation/flip transform is requested
    /// AND no output format conversion (bg4a/pack_rgba) is needed.
    fn is_identity(&self) -> bool {
        self.rotate_mode == 0 && !self.bg4a && !self.pack_rgba
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
        let ch = if self.pack_rgba { 1 } else if self.bg4a { 4 } else { 3 };
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => {
                let (oh, ow) = if self.swaps_dims() { (w, h) } else { (h, w) };
                Some(Proto::value_info(&self.frame_tensor,
                    &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(ch),
                      Proto::tensor_dim_value(oh), Proto::tensor_dim_value(ow)], 1))
            }
            _ => Some(Proto::value_info(&self.frame_tensor,
                &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(ch),
                  Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1)),
        }
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let scale = format!("{}/scale", ns);
        let final_output = &self.frame_tensor;

        // Fast path: no rotation → scale by 1.0 (trivial, all backends)
        if self.is_identity() {
            return vec![
                Proto::node("Mul", &[&self.input_source, &scale], &[final_output], &[]),
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

        // Final: scale, optionally pack via bg4a (Conv 1×1) or pack_rgba (INT32)
        if self.bg4a {
            // BGRA float [0,255] output via Conv(1×1):
            //   - Channel swap RGB→BGR
            //   - Multiply by 255
            //   - Alpha = 255
            // Weight [4,3,1,1]:  [[0,0,255], [0,255,0], [255,0,0], [0,0,0]]
            // Bias [4]:         [0, 0, 0, 255]
            let conv_w = format!("{}/bg4a_w", ns);
            let conv_b = format!("{}/bg4a_b", ns);
            let conv_out = format!("{}/bg4a_out", ns);
            // Only add Conv if we need to actually transform (not identity)
            if prev != final_output || true {
                nodes.push(Proto::node("Conv", &[prev, &conv_w, &conv_b], &[&conv_out],
                    &[Proto::attribute_ints("kernel_shape", &[1, 1]),
                      Proto::attribute_int("group", 1)]));
                tensor_pool.push(conv_out);
                prev = tensor_pool.last().unwrap().as_str();
            }
            if prev != final_output {
                nodes.push(Proto::node("Identity", &[prev], &[final_output], &[]));
            }
        } else if self.pack_rgba {
            // pack_rgba: Mul(255) -> Mul(weights [65536,256,1]) -> ReduceSum -> Cast(INT32)
            // Output: [1,1,H,W] INT32, each value = R*65536+G*256+B
            let scale_255 = format!("{}/scale_255", ns);
            let scaled = format!("{}/scaled", ns);
            let pack_w = format!("{}/pack_w", ns);
            let wweighted = format!("{}/wweighted", ns);
            let rsum = format!("{}/rsum", ns);
            let cast_out = format!("{}/cast", ns);
            // 1. Mul by 255
            nodes.push(Proto::node("Mul", &[prev, &scale_255], &[&scaled], &[]));
            // 2. Mul with weights [65536, 256, 1] — broadcast across H,W
            nodes.push(Proto::node("Mul", &[&scaled, &pack_w], &[&wweighted], &[]));
            // 3. ReduceSum along channel axis → [1,1,H,W]
            nodes.push(Proto::node("ReduceSum", &[&wweighted], &[&rsum],
                &[Proto::attribute_ints("axes", &[1]), Proto::attribute_int("keepdims", 1)]));
            // 4. Cast to INT32
            nodes.push(Proto::node("Cast", &[&rsum], &[&cast_out], &[Proto::attribute_int("to", 6)])); // INT32
            if &cast_out != final_output {
                nodes.push(Proto::node("Identity", &[&cast_out], &[final_output], &[]));
            }
        } else {
            // Float [0,1] output
            if prev != final_output {
                nodes.push(Proto::node("Mul", &[prev, &scale], &[final_output], &[]));
            }
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Scale by 1.0 — trivial on all backends, avoids Identity op issues on Vulkan
        // Output is float [0,1] directly. Consumer (display) multiplies by 255.
        let mut inits = vec![
            Proto::tensor_proto_float_scalar(&format!("{}/scale", ns), 1.0),
        Proto::tensor_proto_float_scalar(&format!("{}/scale_255", ns), 255.0),
        // Pack weights: [65536, 256, 1] for R*65536+G*256+B
        Proto::tensor_proto_float(&format!("{}/pack_w", ns), &[3], &[65536.0, 256.0, 1.0]),
        ];

        if self.bg4a {
            // Conv(1×1) weight [4,3,1,1]:  B=0*R+0*G+255*B,  G=0*R+255*G+0*B,
            // R=255*R+0*G+0*B,  A=(bias)
            // Layout: [oc, ic, kh, kw] = [4, 3, 1, 1]
            // Order: oc0[B](ic0,ic1,ic2), oc1[G](ic0,ic1,ic2), oc2[R], oc3[A]
            inits.push(Proto::tensor_proto_float(
                &format!("{}/bg4a_w", ns),
                &[4, 3, 1, 1],
                &[0.0, 0.0, 255.0,   // oc0 (B): B = 255*B_in
                  0.0, 255.0, 0.0,   // oc1 (G): G = 255*G_in
                  255.0, 0.0, 0.0,   // oc2 (R): R = 255*R_in
                  0.0, 0.0, 0.0]));  // oc3 (A): 0 (bias handles alpha=255)
            // Bias [4]: B=0, G=0, R=0, A=255
            inits.push(Proto::tensor_proto_float(
                &format!("{}/bg4a_b", ns),
                &[4],
                &[0.0, 0.0, 0.0, 255.0]));
        }

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
