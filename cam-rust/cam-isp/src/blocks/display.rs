//! DisplayBlock — final format conversion + orientation transform fused.
//!
//! The output format is selected at build time via `with_output_format()`.
//! The ONNX subgraph produces the exact byte layout requested — the engine
//! just reads the raw output buffer, no Rust-side conversion needed.
//!
//! Format → ONNX subgraph:
//! - `FloatRgb`:  identity Mul(1.0) → `[1,3,H,W]` f32 RGB `[0,1]`
//! - `FloatBgra`: Conv(1×1) BGR permutation + alpha=255 → `[1,4,H,W]` f32 BGRA `[0,255]`
//! - `PackedRgb`: adjacent-pixel pack → `[1,1,H,W/2]` INT32
//!   word = (pixel0.R << 8) | pixel0.G | (pixel1.B << 24) | (pixel1.A << 16)
//! - `Bgra`:      same Conv as FloatBgra → `[1,4,H,W]` f32 BGRA `[0,255]`
//! - `Rgba`:      Conv(1×1) identity permutation + alpha=255 → `[1,4,H,W]` f32 RGBA `[0,255]`
//! - `Argb`:      Conv(1×1) alpha-first permutation → `[1,4,H,W]` f32 ARGB `[0,255]`
//! - `Abgr`:      Conv(1×1) ABGR permutation → `[1,4,H,W]` f32 ABGR `[0,255]`
//! - `Rgb`:       Conv(1×1) identity permutation, no alpha → `[1,3,H,W]` f32 RGB `[0,255]`
//! - `Bgr`:       Conv(1×1) BGR permutation, no alpha → `[1,3,H,W]` f32 BGR `[0,255]`
//!
//! Rotation/flip modes fused via Transpose + Slice before the format node.

use crate::engine::OutputFormat;
use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// DisplayBlock — final format conversion + tone mapping.
///
/// Converts float32 RGB `[0,1]` → RGBA/ARGB/AGBR `[0,255]` via Conv(1×1)
/// with channel permutation weights. Supports sRGB gamma encoding.
/// Output format selected at build time via `.rgba()`, `.argb()`, `.agbr()`.
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
    /// Output pixel format — determines the ONNX subgraph (Conv, Mul, Cast, etc.).
    pub output_format: OutputFormat,
    /// PackedRgb stores two RGBA pixels per INT32 word:
    /// lower 16 bits = pixel0.R|G, upper 16 bits = pixel1.B|A.
    pub pack_two_pixels: bool,
    /// Optional float output tensor name for post-processing pipeline.
    /// When set, the block produces an additional `[1,3,H,W]` float32 tensor
    /// in `[0,1]` range that can be consumed by PostProcessPipeline.
    pub postprocess_output: Option<String>,
    /// Workgroup tuning for Vulkan dispatch: (size_x, size_y).
    /// Default (0,0) = MNN auto-tune; recommended: (32,8) for 4K.
    pub workgroup_size: (u32, u32),
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
            output_format: OutputFormat::default(),
            pack_two_pixels: true,
            postprocess_output: None,
            workgroup_size: (0, 0), // auto-tune
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

    /// Select the output pixel format. The ONNX subgraph is built to produce
    /// bytes in exactly this layout — engine reads them raw, no conversion.
    pub fn with_output_format(mut self, fmt: OutputFormat) -> Self {
        self.output_format = fmt;
        self
    }
    /// Enable post-processing output: additional `[1,3,H,W]` float32 tensor in `[0,1]` range.
    /// When enabled, the block produces a second output tensor named `postprocess_output`
    /// that can be consumed directly by PostProcessPipeline.
    pub fn with_postprocess_output(mut self, enable: bool) -> Self {
        if enable {
            self.postprocess_output = Some("DisplayBlock/postprocess".to_string());
        } else {
            self.postprocess_output = None;
        }
        self
    }

    /// Set custom post-process output tensor name.
    pub fn with_postprocess_tensor(mut self, name: &str) -> Self {
        self.postprocess_output = Some(name.to_string());
        self
    }
    /// Output is `[1,1,H,W/2]` INT32:
    /// lower 16 bits = pixel0.R|G, upper 16 bits = pixel1.B|A.
    /// Equivalent to `.with_output_format(OutputFormat::PackedRgb)`.
    pub fn with_pack_rgba(mut self, enable: bool) -> Self {
        if enable {
            self.output_format = OutputFormat::PackedRgb;
            self.pack_two_pixels = true;
        }
        self
    }

    /// Override adjacent-pixel packing for PackedRgb.
    pub fn with_packed_two_pixels(mut self, enable: bool) -> Self {
        self.pack_two_pixels = enable;
        self
    }
    /// Enable BGRA float `[0,255]` output via Conv(1×1) (FloatBgra format).
    /// Equivalent to `.with_output_format(OutputFormat::FloatBgra)`.
    pub fn with_bg4a(mut self, enable: bool) -> Self {
        if enable {
            self.output_format = OutputFormat::FloatBgra;
        }
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
        self.rotate_mode == 0 && self.output_format == OutputFormat::FloatRgb
    }

    fn packed_output_width(&self, width: i64) -> i64 {
        if self.output_format == OutputFormat::PackedRgb && self.pack_two_pixels {
            (width / 2).max(1)
        } else {
            width
        }
    }

    fn can_pack_two_pixels(&self) -> bool {
        self.output_format == OutputFormat::PackedRgb
            && self.pack_two_pixels
            && self.in_h.is_some()
            && self.in_w.is_some()
    }

    /// Return the effective input tensor name: `input_source` if set,
    /// otherwise `graph_input_name()`.
    fn effective_input(&self) -> String {
        if self.input_source.is_empty() {
            self.graph_input_name()
                .unwrap_or("DisplayBlock/frame")
                .to_string()
        } else {
            self.input_source.clone()
        }
    }
}

impl DisplayBlock {
    /// Set Vulkan shader workgroup size: (size_x, size_y).
    pub fn workgroup(mut self, size_x: u32, size_y: u32) -> Self {
        self.workgroup_size = (size_x, size_y);
        self
    }

    /// RGBA output: `[1,4,H,W]` float32 `[0,255]`.
    pub fn rgba(self) -> Self {
        self.with_output_format(OutputFormat::Rgba)
    }

    /// ARGB output: `[1,4,H,W]` float32 `[0,255]`.
    pub fn argb(self) -> Self {
        self.with_output_format(OutputFormat::Argb)
    }

    /// AGRB (ABGR) output: `[1,4,H,W]` float32 `[0,255]`.
    pub fn agbr(self) -> Self {
        self.with_output_format(OutputFormat::Abgr)
    }
}

impl IspBlock for DisplayBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "DisplayBlock".to_string()
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
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        let mut tensors = vec![self.frame_tensor.clone()];
        if let Some(ref pp) = self.postprocess_output {
            tensors.push(pp.clone());
        }
        tensors
    }

    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => Some(Proto::value_info(
                &self.effective_input(),
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_param("C"),
                    Proto::tensor_dim_value(h),
                    Proto::tensor_dim_value(w),
                ],
                1,
            )),
            _ => Some(Proto::value_info(
                &self.effective_input(),
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_param("C"),
                    Proto::tensor_dim_param("H"),
                    Proto::tensor_dim_param("W"),
                ],
                1,
            )),
        }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let ch = self.output_format.channel_count() as i64;
        let elem = self.output_format.onnx_elem_type();
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => {
                let (oh, ow) = if self.swaps_dims() { (w, h) } else { (h, w) };
                let packed_ow = self.packed_output_width(ow);
                Some(Proto::value_info(
                    &self.frame_tensor,
                    &[
                        Proto::tensor_dim_value(1),
                        Proto::tensor_dim_value(ch),
                        Proto::tensor_dim_value(oh),
                        Proto::tensor_dim_value(packed_ow),
                    ],
                    elem,
                ))
            }
            _ => Some(Proto::value_info(
                &self.frame_tensor,
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(ch),
                    Proto::tensor_dim_param("H"),
                    Proto::tensor_dim_param("W"),
                ],
                elem,
            )),
        }
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let scale = format!("{}/scale", ns);
        let final_output = &self.frame_tensor;

        // Fast path: no rotation, FloatRgb output → Pow+Clip matching R6
        // R6 detects: BinaryOp(POW,exp≈1/2.4) + optional Clip(0,1) → isp.display
        // The isp.display shader does gamma correction + BCS in one dispatch.
        if self.is_identity() {
            let gamma_exp = format!("{}/gamma_exp", ns);
            let gamma = format!("{}/gamma", ns);
            let zero = format!("{}/zero", ns);
            let one = format!("{}/one", ns);
            return vec![
                Proto::node(
                    "Pow",
                    &[&self.effective_input(), &gamma_exp],
                    &[&gamma],
                    &[],
                ),
                Proto::node("Clip", &[&gamma, &zero, &one], &[final_output], &[]),
            ];
        }

        let mut nodes: Vec<Vec<u8>> = Vec::new();
        // Keep intermediate tensor names alive across the function scope
        let mut tensor_pool: Vec<String> = Vec::new();
        let mut prev = self.effective_input();

        // 90° or 270°: transpose to swap H ↔ W
        if self.swaps_dims() {
            let transposed = format!("{}/transposed", ns);
            nodes.push(Proto::node(
                "Transpose",
                &[&prev],
                &[&transposed],
                &[Proto::attribute_ints("perm", &[0, 1, 3, 2])],
            ));
            tensor_pool.push(transposed);
            prev = tensor_pool.last().unwrap().clone();
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
            nodes.push(Proto::node(
                "Slice",
                &[&prev, &starts, &ends, &axes, &steps],
                &[&hflipped],
                &[],
            ));
            tensor_pool.push(hflipped);
            prev = tensor_pool.last().unwrap().clone();
        }

        // Vertical flip on H (axis 2)
        if self.needs_vflip() {
            let vflipped = format!("{}/vflipped", ns);
            let starts = format!("{}/vflip_starts", ns);
            let ends = format!("{}/vflip_ends", ns);
            let axes = format!("{}/vflip_axes", ns);
            let steps = format!("{}/vflip_steps", ns);
            nodes.push(Proto::node(
                "Slice",
                &[&prev, &starts, &ends, &axes, &steps],
                &[&vflipped],
                &[],
            ));
            tensor_pool.push(vflipped);
            prev = tensor_pool.last().unwrap().clone();
        }

        // Final: apply output format conversion
        use OutputFormat::*;
        match self.output_format {
            FloatRgb => {
                // Identity — Mul(1.0) for float [0,1] RGB
                if prev != *final_output {
                    nodes.push(Proto::node("Mul", &[&prev, &scale], &[final_output], &[]));
                }
            }
            Float16Rgb => {
                // Float RGB [0,1] → Cast(FLOAT→FLOAT16)
                let f32_out = format!("{}/f32", ns);
                nodes.push(Proto::node("Mul", &[&prev, &scale], &[&f32_out], &[]));
                tensor_pool.push(f32_out);
                let f32_ref = tensor_pool.last().unwrap().as_str();
                nodes.push(Proto::node(
                    "Cast",
                    &[f32_ref],
                    &[final_output],
                    &[Proto::attribute_int("to", 10)],
                )); // 10 = FLOAT16
            }
            Float16Bgra => {
                // Float BGRA [0,255] → Cast(FLOAT→FLOAT16)
                let conv_w = format!("{}/conv_w", ns);
                let conv_b = format!("{}/conv_b", ns);
                let conv_out = format!("{}/conv_out", ns);
                let f32_out = format!("{}/f32", ns);
                nodes.push(Proto::node(
                    "Conv",
                    &[&prev, &conv_w, &conv_b],
                    &[&conv_out],
                    &[
                        Proto::attribute_ints("kernel_shape", &[1, 1]),
                        Proto::attribute_int("group", 1),
                    ],
                ));
                tensor_pool.push(conv_out);
                nodes.push(Proto::node(
                    "Identity",
                    &[tensor_pool.last().unwrap()],
                    &[&f32_out],
                    &[],
                ));
                tensor_pool.push(f32_out);
                nodes.push(Proto::node(
                    "Cast",
                    &[tensor_pool.last().unwrap()],
                    &[final_output],
                    &[Proto::attribute_int("to", 10)],
                )); // 10 = FLOAT16
            }
            FloatBgra | Bgra | Rgba | Argb | Abgr | Rgb | Bgr => {
                // Conv(1×1) with format-specific weight permutation + scale(255) + alpha bias.
                // Output is always float [0,255] (u8 truncation happens in the consumer).
                let conv_w = format!("{}/conv_w", ns);
                let conv_b = format!("{}/conv_b", ns);
                let conv_out = format!("{}/conv_out", ns);
                nodes.push(Proto::node(
                    "Conv",
                    &[&prev, &conv_w, &conv_b],
                    &[&conv_out],
                    &[
                        Proto::attribute_ints("kernel_shape", &[1, 1]),
                        Proto::attribute_int("group", 1),
                    ],
                ));
                tensor_pool.push(conv_out);
                prev = tensor_pool.last().unwrap().clone();
                if prev != *final_output {
                    nodes.push(Proto::node("Identity", &[&prev], &[final_output], &[]));
                }
            }
            PackedRgb => {
                if self.can_pack_two_pixels() {
                    let pack_w = format!("{}/pack_pair_w", ns);
                    let pack_b = format!("{}/pack_pair_b", ns);
                    let conv_out = format!("{}/pack_conv_out", ns);
                    let cast_out = format!("{}/cast", ns);

                    nodes.push(Proto::node(
                        "Conv",
                        &[&prev, &pack_w, &pack_b],
                        &[&conv_out],
                        &[
                            Proto::attribute_ints("kernel_shape", &[1, 2]),
                            Proto::attribute_ints("strides", &[1, 2]),
                            Proto::attribute_ints("pads", &[0, 0, 0, 0]),
                            Proto::attribute_int("group", 1),
                        ],
                    ));
                    nodes.push(Proto::node(
                        "Cast",
                        &[&conv_out],
                        &[&cast_out],
                        &[Proto::attribute_int("to", 6)],
                    ));
                    if &cast_out != final_output {
                        nodes.push(Proto::node("Identity", &[&cast_out], &[final_output], &[]));
                    }
                } else {
                    // Legacy per-pixel pack: Mul(255) -> Mul(weights [65536,256,1]) -> ReduceSum -> Cast(INT32)
                    // Output: [1,1,H,W] INT32, each value = R*65536+G*256+B.
                    let scale_255 = format!("{}/scale_255", ns);
                    let scaled = format!("{}/scaled", ns);
                    let pack_w = format!("{}/pack_w", ns);
                    let wweighted = format!("{}/wweighted", ns);
                    let rsum = format!("{}/rsum", ns);
                    let cast_out = format!("{}/cast", ns);
                    nodes.push(Proto::node("Mul", &[&prev, &scale_255], &[&scaled], &[]));
                    nodes.push(Proto::node("Mul", &[&scaled, &pack_w], &[&wweighted], &[]));
                    nodes.push(Proto::node(
                        "ReduceSum",
                        &[&wweighted],
                        &[&rsum],
                        &[
                            Proto::attribute_ints("axes", &[1]),
                            Proto::attribute_int("keepdims", 1),
                        ],
                    ));
                    nodes.push(Proto::node(
                        "Cast",
                        &[&rsum],
                        &[&cast_out],
                        &[Proto::attribute_int("to", 6)],
                    ));
                    if &cast_out != final_output {
                        nodes.push(Proto::node("Identity", &[&cast_out], &[final_output], &[]));
                    }
                }
            }
        }

        // Add post-process output tensor if enabled
        if let Some(ref pp_tensor) = self.postprocess_output {
            // Post-process output is [1,3,H,W] float32 in [0,1] range
            // Take the RGB channels from the input (before any format conversion)
            let pp_scale = format!("{}/pp_scale", ns);
            // Input is [1,3,H,W] float [0,1] — just multiply by 1.0 (identity)
            // This gives us a clean float tensor for post-processing
            nodes.push(Proto::node(
                "Mul",
                &[&self.effective_input(), &pp_scale],
                &[pp_tensor],
                &[],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        // Scale 1.0 — used by non-identity format conversion paths
        let mut inits = vec![Proto::tensor_proto_float_scalar(
            &format!("{}/scale", ns),
            1.0,
        )];

        // Format-specific initializers
        use OutputFormat::*;
        match self.output_format {
            FloatRgb | Float16Rgb => {
                // For FloatRgb identity path: Pow(1/2.4) + Clip(0,1) → R6 matches
                // gamma_exp = 1/2.4 for sRGB gamma correction
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/gamma_exp", ns),
                    1.0 / 2.4,
                ));
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/zero", ns),
                    0.0,
                ));
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/one", ns),
                    1.0,
                ));
            }
            PackedRgb => {
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/scale_255", ns),
                    255.0,
                ));
                if self.can_pack_two_pixels() {
                    match (self.in_h, self.in_w) {
                        (Some(_), Some(_)) => {}
                        _ => return inits,
                    };
                    inits.push(Proto::tensor_proto_float(
                        &format!("{}/pack_pair_w", ns),
                        &[1, 4, 1, 2],
                        &[256.0, 0.0, 1.0, 0.0, 0.0, 16777216.0, 0.0, 16777216.0],
                    ));
                    inits.push(Proto::tensor_proto_float(
                        &format!("{}/pack_pair_b", ns),
                        &[1],
                        &[0.0],
                    ));
                } else {
                    inits.push(Proto::tensor_proto_float(
                        &format!("{}/pack_w", ns),
                        &[3, 1, 1],
                        &[65536.0, 256.0, 1.0],
                    ));
                }
            }
            fmt @ (FloatBgra | Float16Bgra | Bgra | Rgba | Argb | Abgr | Rgb | Bgr) => {
                // Conv(1×1): channel permutation + scale(255) + alpha bias
                let oc = fmt.channel_count(); // 3 for Rgb/Bgr, 4 for others
                let (weights, bias): (Vec<f32>, Vec<f32>) = match fmt {
                    FloatBgra | Float16Bgra | Bgra => (
                        // oc0(B)=255*B, oc1(G)=255*G, oc2(R)=255*R, oc3(A)=0
                        vec![
                            0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        ],
                        vec![0.0, 0.0, 0.0, 255.0],
                    ),
                    Rgba => (
                        // oc0(R)=255*R, oc1(G)=255*G, oc2(B)=255*B, oc3(A)=0
                        vec![
                            255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0,
                        ],
                        vec![0.0, 0.0, 0.0, 255.0],
                    ),
                    Argb => (
                        // oc0(A)=0, oc1(R)=255*R, oc2(G)=255*G, oc3(B)=255*B
                        vec![
                            0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0,
                        ],
                        vec![255.0, 0.0, 0.0, 0.0],
                    ),
                    Abgr => (
                        // oc0(A)=0, oc1(B)=255*B, oc2(G)=255*G, oc3(R)=255*R
                        vec![
                            0.0, 0.0, 0.0, 0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0,
                        ],
                        vec![255.0, 0.0, 0.0, 0.0],
                    ),
                    Rgb => (
                        // oc0(R)=255*R, oc1(G)=255*G, oc2(B)=255*B
                        vec![255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0],
                        vec![0.0, 0.0, 0.0],
                    ),
                    Bgr => (
                        // oc0(B)=255*B, oc1(G)=255*G, oc2(R)=255*R
                        vec![0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0],
                        vec![0.0, 0.0, 0.0],
                    ),
                    _ => unreachable!(),
                };
                let w_shape: Vec<i64> = vec![oc as i64, 3, 1, 1];
                let b_shape: Vec<i64> = vec![oc as i64];
                inits.push(Proto::tensor_proto_float(
                    &format!("{}/conv_w", ns),
                    &w_shape,
                    &weights,
                ));
                inits.push(Proto::tensor_proto_float(
                    &format!("{}/conv_b", ns),
                    &b_shape,
                    &bias,
                ));
            }
        }

        if self.is_identity() {
            return inits;
        }

        let (h, w) = match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => (h, w),
            _ => return inits, // no concrete dims → no flip initializers
        };

        // For 90/270, transpose swaps dims so flip axes apply post-swap
        let (flip_h, flip_w) = if self.swaps_dims() { (w, h) } else { (h, w) };

        if self.needs_hflip() && flip_w > 0 {
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_starts", ns),
                &[flip_w - 1],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_ends", ns),
                &[-1],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_axes", ns),
                &[3],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/hflip_steps", ns),
                &[-1],
            ));
        }

        if self.needs_vflip() && flip_h > 0 {
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_starts", ns),
                &[flip_h - 1],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_ends", ns),
                &[-1],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_axes", ns),
                &[2],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/vflip_steps", ns),
                &[-1],
            ));
        }

        // Add post-process scale initializer
        if self.postprocess_output.is_some() {
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/pp_scale", ns),
                1.0,
            ));
        }

        inits
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        let mut extras = Vec::new();
        // scale is only used in nodes() for FloatRgb/FP16Rgb/postprocess paths
        if self.is_identity()
            || self.output_format == OutputFormat::Float16Rgb
            || self.postprocess_output.is_some()
        {
            extras.push((format!("{}/scale", ns), 1, vec![1]));
        }
        // gamma_exp, zero, one only for FloatRgb identity path
        if self.is_identity() {
            extras.push((format!("{}/gamma_exp", ns), 1, vec![1]));
            extras.push((format!("{}/zero", ns), 1, vec![1]));
            extras.push((format!("{}/one", ns), 1, vec![1]));
        }
        extras
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_display_id() {
        assert_eq!(DisplayBlock::new(1920).id(), "display");
    }

    #[test]
    fn test_display_with_output_format() {
        let b = DisplayBlock::new(1920).with_output_format(OutputFormat::Rgba);
        assert_eq!(b.target_width, 1920);
    }

    #[test]
    fn test_display_emit_onnx_rgb() {
        let mut b = DisplayBlock::new(1920).with_output_format(OutputFormat::FloatRgb);
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // FloatRgb emits at least one node
        assert!(!nodes.is_empty());
    }

    #[test]
    fn test_display_emit_onnx_rgba() {
        let mut b = DisplayBlock::new(1920).with_output_format(OutputFormat::Rgba);
        b.set_input_source("in/frame");
        let nodes = b.nodes();
        // Rgba uses Conv(1x1) for channel permutation
        assert!(!nodes.is_empty());
    }

    #[test]
    fn test_display_tensor_ns() {
        let b = DisplayBlock::new(640);
        assert!(!b.tensor_ns().is_empty());
    }

    #[test]
    fn test_display_initializers() {
        let b = DisplayBlock::new(640);
        let inits = b.initializers();
        assert_eq!(inits.len(), 3);
    }
}
