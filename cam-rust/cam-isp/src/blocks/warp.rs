//! WarpGridBlock — EIS + GDC via GridSample //!
//! Emits ONNX GridSample node + optional camera orientation rotation.
//! Requires MNN Vulkan backend for GPU-accelerated sampling.
//!
//! GridSample node specs:
//! - Input: [1,3,H,W] float32 RGB image tensor
//! - Grid: [1,OH,OW,2] float32 sampling grid in [-1,1] range
//! - Mode: LINEAR, CLAMP
//! - Target: [1,3,OH,OW] out
//!
use crate::pipeline::IspBlock;
use crate::onnx::proto::Proto;

pub struct WarpGridBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    pub target_width: u32,
    pub target_height: u32,
    pub output_width: u32,
    pub output_height: u32,
    pub rotate_mode: i32,
    pub grid_initializer: Option<Vec<u8>>, // [OH,OW,2] float32 sampling grid
    pub bcs: Option<(f32, f32, f32)>,     // Brightness, Contrast, Saturation overrides
}

impl WarpGridBlock {
    pub fn new(target_width: u32, target_height: u32) -> Self {
        Self {
            id: "warp_grid".to_string(),
            prev: None,
            next: None,
            frame_tensor: "WarpGrid/frame".to_string(),
            input_source: String::new(),
            target_width,
            target_height,
            output_width: target_width,
            output_height: target_height,
            rotate_mode: 0,
            grid_initializer: None,
            bcs: None
        }
    }

    /// Set upscale/letterbox output resolution.
    pub fn with_output_resolution(mut self, width: u32, height: u32) -> Self {
        self.output_width = width;
        self.output_height = height;
        self
    }

    /// Set orientation rotation/flip mode.
    pub fn with_rotate(mut self, mode: i32) -> Self {
        self.rotate_mode = mode;
        self
    }

    /// Set sampling grid initializer (ONNX variable).
    pub fn with_grid(mut self, grid: Option<Vec<f32>>) -> Self {
        if let Some(g) = grid {
            let initializer = Proto::tensor_proto_float(
                &format!("{}/grid", self.tensor_ns()),
                &[1, self.output_height as i64, self.output_width as i64, 2],
                &g
            );
            self.grid_initializer = Some(initializer);
        }
        self
    }

    /// Set BCS display tuning: brightness, contrast, saturation.
    pub fn with_bcs(mut self, brightness: f32, contrast: f32, saturation: f32) -> Self {
        self.bcs = Some((brightness, contrast, saturation));
        self
    }

    /// Whether transposition is needed to swap H↔W.
    fn swaps_dims(&self) -> bool {
        matches!(self.rotate_mode, 1 | 3) // ROTATE_ROT90 || ROTATE_ROT270
    }

    /// Whether mode requires horizontal flip on W (axis=3).
    fn needs_hflip(&self) -> bool {
        matches!(self.rotate_mode, 1 | 2 | 4) // ROTATE_ROT90 || ROTATE_ROT180 || HFLIP
    }

    /// Whether mode requires vertical flip on H (axis=2).
    fn needs_vflip(&self) -> bool {
        matches!(self.rotate_mode, 2 | 3 | 5) // ROTATE_ROT180 || ROTATE_ROT270 || VFLIP
    }
}

impl IspBlock for WarpGridBlock {
    fn id(&self) -> &str { &self.id }
    fn tensor_ns(&self) -> String { "WarpGrid".to_string() }
    fn frame_tensor(&self) -> Option<&str> { Some(&self.frame_tensor) }
    fn input_source(&self) -> Option<&str> { Some(&self.input_source) }
    fn set_input_source(&mut self, name: &str) { self.input_source = name.to_string(); }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> { self.prev.as_ref() }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) { self.prev = Some(block); }
    fn next(&self) -> Option<&Box<dyn IspBlock>> { self.next.as_ref() }
    fn set_next(&mut self, block: Box<dyn IspBlock>) { self.next = Some(block); }
    
    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }
    
    fn graph_output_name(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    
    fn input_value_info(&self) -> Option<Vec<u8>> {
        Some(Proto::value_info(
            &self.input_source,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(3),
              Proto::tensor_dim_param("H"), Proto::tensor_dim_param("W")], 1))
    }
    
    fn output_value_info(&self) -> Option<Vec<u8>> {
        let ch = 3;
        Some(Proto::value_info(
            &self.frame_tensor,
            &[Proto::tensor_dim_value(1), Proto::tensor_dim_value(ch),
              Proto::tensor_dim_value(self.output_height as i64),
              Proto::tensor_dim_value(self.output_width as i64)], 1))
    }
    
    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut nodes = Vec::new();

        // Sampling grid: [1,OH,OW,2]
        let grid_input = self.grid_initializer.clone()
            .map_or_else(|| format!("{}/grid_init", ns), |_| format!("{}/grid", ns));

        // GridSample node
        let grid_node = Proto::node(
            "GridSample", 
            &[&self.input_source, &grid_input], 
            &[&format!("{}/grid_sampled", ns)],
            &[
                Proto::attribute_string("mode", "LINEAR"),
                Proto::attribute_string("padding_mode", "CLAMP"),
                Proto::attribute_string("align_corners", "false")
            ]
        );
        nodes.push(grid_node);

        let mut prev = format!("{}/grid_sampled", ns);

        // Rotate/flip via Transpose + Slice
        if self.swaps_dims() {
            nodes.push(Proto::node("Transpose", &[&prev], &[&format!("{}/transposed", ns)],
                                  &[Proto::attribute_ints("perm", &[0, 1, 3, 2])]));
            prev = format!("{}/transposed", ns);
        }

        if self.needs_hflip() {
            nodes.push(Proto::node("Slice", 
                 &[&prev, &format!("{}/hflip_starts", ns),
                   &format!("{}/hflip_ends", ns),
                   &format!("{}/hflip_axes", ns),
                   &format!("{}/hflip_steps", ns)],
                 &[&format!("{}/hflipped", ns)], &[]));
            prev = format!("{}/hflipped", ns);
        }
        
        if self.needs_vflip() {
            nodes.push(Proto::node("Slice",
                 &[&prev, &format!("{}/vflip_starts", ns),
                   &format!("{}/vflip_ends", ns),
                   &format!("{}/vflip_axes", ns),
                   &format!("{}/vflip_steps", ns)],
                 &[&self.frame_tensor], &[]));
        } else if prev != self.frame_tensor {
            nodes.push(Proto::node("Identity", &[&prev], &[&self.frame_tensor], &[]));
        }

        // BCS display layer fused via Mul/Add
        if let Some((bright, contr, _sat)) = self.bcs {
            let scope = format!("{}/bcs", ns);
            let _ = bright;
            let _ = contr;
            let bright_add = format!("{}/bright_add", scope);
            let bright_ident = format!("{}/bright_out", scope);
            let contr_mul = format!("{}/contr_mul", scope);
            let contr_out = format!("{}/contr_out", scope);
            let luma_vec = format!("{}/luma_w", scope);
            let luma_out = format!("{}/luma_out", scope);
            let final_out = format!("{}/final", scope);

            // Brightness: add scalar
            nodes.push(Proto::node("Add", &[&self.frame_tensor, &bright_add], &[&bright_ident], &[]));
            // Contrast: sub(0.5) mul(const) add(0.5)
            nodes.push(Proto::node("Sub", &[&bright_ident, &format!("{}/half", scope)], 
                                          &[&contr_mul], &[]));
            nodes.push(Proto::node("Mul", &[&contr_mul, &format!("{}/contr_w", scope)], 
                                          &[&contr_out], &[]));
            nodes.push(Proto::node("Add", &[&contr_out, &format!("{}/half", scope)], 
                                          &[&luma_vec], &[]));
            // Saturation: mix(luma, img * const)
            nodes.push(Proto::node("Mul", &[&luma_vec, &luma_out], &[&luma_vec], &[]));
            nodes.push(Proto::node("Mul", &[&luma_vec, &format!("{}/sat_w", scope)], 
                                          &[&final_out], &[]));
            nodes.push(Proto::node("Identity", &[&final_out], &[&self.frame_tensor], &[]));
        }
    
        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits = Vec::new();

        // Sampling grid initializer (if provided)
        if let Some(init) = &self.grid_initializer {
            inits.push(init.clone());
        }
        
        // Rotate/flip slicing boundaries
        if self.swaps_dims() {
            return inits;
        }
        
        let (height, width) = (self.output_height as i64, self.output_width as i64);
        
        if self.needs_hflip() && width > 0 {
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_starts", ns), &[width-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_axes", ns), &[3]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/hflip_steps", ns), &[-1]));
        }
        
        if self.needs_vflip() && height > 0 {
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_starts", ns), &[height-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_ends", ns), &[-1]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_axes", ns), &[2]));
            inits.push(Proto::tensor_proto_int64(&format!("{}/vflip_steps", ns), &[-1]));
        }
        
        // Brightness/Contrast/Saturation constants
        if let Some((bright, contr, sat)) = self.bcs {
            let scope = format!("{}/bcs", ns);
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/bright_add", scope), bright));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/half", scope), 0.5));
            inits.push(Proto::tensor_proto_float_scalar(&format!("{}/contr_w", scope), contr));
            inits.push(Proto::tensor_proto_float_scalar(&scope,                // dummy
                                                       bright));
            // Saturation weights
            inits.push(Proto::tensor_proto_float(
                &format!("{}/luma_w", scope),
                &[3],
                &[0.299, 0.587, 0.114]
            ));
            let sat_w: Vec<f32> = match sat {
                v if v >= 0.0 => vec![1.0, v, v],
                _ => vec![1.0, 1.4, 0.5]  // test split
            };
            inits.push(Proto::tensor_proto_float(
                &format!("{}/sat_w", scope),
                &[3],
                &sat_w
            ));
        }
        
        inits
    }
}