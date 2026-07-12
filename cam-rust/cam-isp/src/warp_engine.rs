//! GPU Warp Engine — runs GDC+EIS warp via separate MNN session.
//!
//! Takes the GPU warp ONNX model and runs it as a separate inference pass
//! after the main ISP pipeline. Parameters (k1, k2, k3, eis_x, eis_y)
//! are set per-frame before inference.

#[cfg(feature = "mnn")]
use crate::error::IspResult;
#[cfg(feature = "mnn")]
use crate::mnn::mnn_sys::{MnnBackendType, MnnInterpreterSafe, MnnSessionSafe};
#[cfg(feature = "mnn")]
use log::info;

/// Parameters for GPU warp (GDC + EIS).
#[derive(Debug, Clone, Copy, Default)]
pub struct GpuWarpParams {
    /// GDC radial distortion coefficient k1
    pub gdc_k1: f32,
    /// GDC radial distortion coefficient k2
    pub gdc_k2: f32,
    /// GDC radial distortion coefficient k3
    pub gdc_k3: f32,
    /// EIS horizontal displacement in normalized [-1,1] coords
    pub eis_dx: f32,
    /// EIS vertical displacement in normalized [-1,1] coords
    pub eis_dy: f32,
    /// Digital zoom factor [1.0, 4.0]. GDC strength scales inversely.
    pub zoom: f32,
    /// VCM focus motor position [0.0, 1.0]. 0=infinity, 1=macro.
    /// Focus breathing changes effective focal length, affecting distortion.
    pub vcm_position: f32,
}

impl GpuWarpParams {
    /// Default breathing factor: ~15% focal length change from infinity to macro.
    /// This is lens-specific; calibrate per module for production.
    const BREATHING: f32 = 0.15;

    /// Create identity (no warp) parameters.
    pub fn identity() -> Self {
        Self {
            gdc_k1: 0.0,
            gdc_k2: 0.0,
            gdc_k3: 0.0,
            eis_dx: 0.0,
            eis_dy: 0.0,
            zoom: 1.0,
            vcm_position: 0.0,
        }
    }

    /// Create GDC parameters with given k1, k2, k3.
    pub fn gdc(k1: f32, k2: f32, k3: f32) -> Self {
        Self {
            gdc_k1: k1,
            gdc_k2: k2,
            gdc_k3: k3,
            eis_dx: 0.0,
            eis_dy: 0.0,
            zoom: 1.0,
            vcm_position: 0.0,
        }
    }

    /// Create EIS parameters with given displacement.
    pub fn eis(dx: f32, dy: f32) -> Self {
        Self {
            gdc_k1: 0.0,
            gdc_k2: 0.0,
            gdc_k3: 0.0,
            eis_dx: dx,
            eis_dy: dy,
            zoom: 1.0,
            vcm_position: 0.0,
        }
    }

    /// Set digital zoom factor. GDC strength scales inversely.
    pub fn with_zoom(mut self, zoom: f32) -> Self {
        self.zoom = zoom.max(1.0);
        self
    }

    /// Set VCM focus motor position [0.0, 1.0].
    /// Affects effective focal length via focus breathing.
    pub fn with_vcm(mut self, pos: f32) -> Self {
        self.vcm_position = pos.clamp(0.0, 1.0);
        self
    }

    /// Create warp params from ISP controller params.
    /// Extracts zoom and VCM position for GDC correction.
    pub fn from_isp_params(
        gdc_k1: f32,
        gdc_k2: f32,
        gdc_k3: f32,
        isp: &crate::isp_params::IspParams,
    ) -> Self {
        Self {
            gdc_k1,
            gdc_k2,
            gdc_k3,
            eis_dx: 0.0,
            eis_dy: 0.0,
            zoom: isp.zoom.max(1.0),
            vcm_position: isp.vcm_position.clamp(0.0, 1.0),
        }
    }

    /// Combined focal factor: accounts for both digital zoom and
    /// focus breathing (VCM position).
    ///   zoom=1.0, vcm=0.0 → factor=1.0 (wide, infinity)
    ///   zoom=2.0, vcm=0.5 → factor=2.0 * 1.075 = 2.15
    pub fn focal_factor(&self) -> f32 {
        self.zoom * (1.0 + self.vcm_position * Self::BREATHING)
    }

    /// Effective k1 accounting for digital zoom and focus breathing.
    /// Distortion magnitude is inversely proportional to effective
    /// focal length: at higher zoom or closer focus (VCM→1.0),
    /// the cropped/breathing FOV has less visible distortion.
    pub fn effective_k1(&self) -> f32 {
        self.gdc_k1 / self.focal_factor()
    }

    /// Effective k2.
    pub fn effective_k2(&self) -> f32 {
        self.gdc_k2 / self.focal_factor()
    }

    /// Effective k3.
    pub fn effective_k3(&self) -> f32 {
        self.gdc_k3 / self.focal_factor()
    }

    /// Check if any warp is needed (non-identity).
    pub fn needs_warp(&self) -> bool {
        self.gdc_k1.abs() > 1e-6
            || self.gdc_k2.abs() > 1e-6
            || self.gdc_k3.abs() > 1e-6
            || self.eis_dx.abs() > 1e-6
            || self.eis_dy.abs() > 1e-6
    }
}

/// GPU warp engine — separate MNN session for GDC+EIS warp.
#[cfg(feature = "mnn")]
pub struct GpuWarpEngine {
    session: MnnSessionSafe,
    interp: MnnInterpreterSafe,
    width: u32,
    height: u32,
    initialized: bool,
}

/// Generate a unique temp file path for release builds (cleaned up after use).
/// Debug dump: save bytes to disk if debug_assertions are enabled.
#[cfg(feature = "mnn")]
fn debug_dump_onnx(name: &str, onnx: &[u8]) {
    if cfg!(debug_assertions) {
        let path = format!("{}.onnx", name);
        let _ = std::fs::write(&path, onnx);
        log::info!("Dumped ONNX to {}", path);
    }
}

#[cfg(feature = "mnn")]
impl GpuWarpEngine {
    /// Create warp engine from ONNX bytes.
    ///
    /// Architecture (same-process, no disk writes):
    ///   \[init\]  build ONNX (Rust, no MNN) → convert via mnn_convert_onnx_buffer
    ///   \[exec\]  load MNN from buffer → create session → run inference
    pub fn from_onnx(onnx: &[u8], width: u32, height: u32) -> IspResult<Self> {
        info!(
            "GpuWarpEngine: creating from {} byte ONNX for {}×{}",
            onnx.len(),
            width,
            height
        );

        // Debug: dump ONNX to disk for inspection
        debug_dump_onnx("warp_gdc", onnx);

        // Convert ONNX→MNN via same-process buffer API (libMNNConvertDeps.so)
        // Uses memfd internally — zero disk writes on Linux.
        let mut result = crate::mnn_sys::MnnConvertBufferResult {
            success: 0,
            error_msg: [0i8; 1024usize],
            data: std::ptr::null_mut(),
            size: 0,
        };
        unsafe {
            crate::mnn_sys::mnn_convert_onnx_buffer(
                onnx.as_ptr() as *const std::ffi::c_void,
                onnx.len(),
                &mut result,
            );
        }
        if result.success != 0 {
            let err_msg = unsafe { std::ffi::CStr::from_ptr(result.error_msg.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            return Err(crate::error::IspError::Conversion(format!(
                "convert warp onnx: {}",
                err_msg
            )));
        }

        // Copy MNN model data into safe Rust Vec
        let mnn_bytes = if !result.data.is_null() && result.size > 0 {
            let slice =
                unsafe { std::slice::from_raw_parts(result.data as *const u8, result.size) };
            let owned = slice.to_vec();
            // Debug: dump MNN to disk for inspection
            if cfg!(debug_assertions) {
                let path = "warp_gdc.mnn";
                let _ = std::fs::write(path, &owned);
                log::info!("Dumped MNN to {}", path);
            }
            unsafe {
                crate::mnn_sys::MnnConvert_FreeBuffer(&mut result);
            }
            owned
        } else {
            return Err(crate::error::IspError::Conversion(
                "convert returned empty buffer".into(),
            ));
        };

        let interp = MnnInterpreterSafe::from_buffer(&mnn_bytes)
            .ok_or_else(|| crate::error::IspError::Mnn("warp model load fail".into()))?;

        // Create session (CPU for warp — lightweight)
        let session = interp
            .create_session(MnnBackendType::Cpu, 2)
            .ok_or_else(|| crate::error::IspError::Mnn("warp session create fail".into()))?;

        info!("GpuWarpEngine: initialized {}×{}", width, height);

        Ok(Self {
            interp,
            session,
            width,
            height,
            initialized: true,
        })
    }

    /// Create a new warp engine with a default ONNX model for the given dimensions.
    pub fn new(width: u32, height: u32) -> IspResult<Self> {
        let onnx = Self::generate_onnx(width, height);
        Self::from_onnx(&onnx, width, height)
    }

    /// Generate ONNX model for GDC+EIS warp with full GPU grid computation.
    ///
    /// All ISP parameters flow through MNN tensors. The grid is computed
    /// entirely in ONNX arithmetic ops — zero CPU involvement.
    /// MNN Executor fuses these ops into efficient GPU kernels.
    ///
    /// Input tensors (per-frame runtime):
    ///   GpuWarp/input   [1,3,H,W]  — frame to warp
    ///   GpuWarp/gdc_k1  [1]        — radial distortion k1
    ///   GpuWarp/gdc_k2  [1]        — radial distortion k2
    ///   GpuWarp/gdc_k3  [1]        — radial distortion k3
    ///   GpuWarp/zoom    [1]        — digital zoom [1.0, 4.0]
    ///   GpuWarp/vcm     [1]        — focus motor position [0, 1]
    ///   GpuWarp/eis_dx  [1,1,H,W]  — EIS X displacement
    ///   GpuWarp/eis_dy  [1,1,H,W]  — EIS Y displacement
    ///
    /// Output tensor:
    ///   GpuWarp/frame   [1,3,H,W]  — warped frame
    ///
    /// Computation (all GPU, all ONNX ops):
    ///   focal_factor = zoom * (1 + vcm * BREATHING)
    ///   effective_k1 = gdc_k1 / focal_factor
    ///   ...
    ///   r² = grid_x² + grid_y²  →  r⁴ → r⁶
    ///   denom = 1 + k1*r² + k2*r⁴ + k3*r⁶
    ///   inv_denom = 1/denom
    ///   grid = concat(grid_x*inv + eis_dx, grid_y*inv + eis_dy)
    ///   GridSample(input, grid)
    pub fn generate_onnx(width: u32, height: u32) -> Vec<u8> {
        use crate::onnx::proto::Proto;

        let h = height as i64;
        let w = width as i64;
        let ns = "GpuWarp";

        // ── Runtime inputs ──
        let inputs = vec![
            Proto::value_info(
                &format!("{}/input", ns),
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(3),
                    Proto::tensor_dim_value(h),
                    Proto::tensor_dim_value(w),
                ],
                1,
            ),
            Proto::value_info(&format!("{}/gdc_k1", ns), &[Proto::tensor_dim_value(1)], 1),
            Proto::value_info(&format!("{}/gdc_k2", ns), &[Proto::tensor_dim_value(1)], 1),
            Proto::value_info(&format!("{}/gdc_k3", ns), &[Proto::tensor_dim_value(1)], 1),
            Proto::value_info(&format!("{}/zoom", ns), &[Proto::tensor_dim_value(1)], 1),
            Proto::value_info(&format!("{}/vcm", ns), &[Proto::tensor_dim_value(1)], 1),
            Proto::value_info(
                &format!("{}/eis_dx", ns),
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(h),
                    Proto::tensor_dim_value(w),
                ],
                1,
            ),
            Proto::value_info(
                &format!("{}/eis_dy", ns),
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_value(h),
                    Proto::tensor_dim_value(w),
                ],
                1,
            ),
        ];

        // ── Output ──
        let outputs = vec![Proto::value_info(
            &format!("{}/frame", ns),
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ],
            1,
        )];

        // ── Grid computation nodes ──
        let mut nodes = Vec::new();

        // Helper: format names
        let gx = format!("{}/grid_x", ns);
        let gy = format!("{}/grid_y", ns);
        let zero = format!("{}/zero", ns);
        let one = format!("{}/one", ns);
        let breath = format!("{}/breathing", ns);
        let shape4 = format!("{}/shape4", ns);

        // 1) Broadcast coords to [1,1,H,W] via Add(zero)
        let gx2d = format!("{}/gx2d", ns);
        nodes.push(Proto::node("Add", &[&gx, &zero], &[&gx2d], &[]));
        let gy2d = format!("{}/gy2d", ns);
        nodes.push(Proto::node("Add", &[&gy, &zero], &[&gy2d], &[]));

        // 2) focal_factor = zoom * (1 + vcm * breathing)
        let vcm_term = format!("{}/vcm_term", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&format!("{}/vcm", ns), &breath],
            &[&vcm_term],
            &[],
        ));
        let base = format!("{}/base", ns);
        nodes.push(Proto::node("Add", &[&one, &vcm_term], &[&base], &[]));
        let focal = format!("{}/focal", ns);
        nodes.push(Proto::node(
            "Mul",
            &[&format!("{}/zoom", ns), &base],
            &[&focal],
            &[],
        ));

        // 3) effective_k = k / focal_factor
        let mut mk = |i: u32| {
            let k = format!("{}/gdc_k{}", ns, i);
            let eff = format!("{}/eff_k{}", ns, i);
            nodes.push(Proto::node("Div", &[&k, &focal], &[&eff], &[]));
            eff
        };
        let ek1 = mk(1);
        let ek2 = mk(2);
        let ek3 = mk(3);

        // 4) r² = gx² + gy²
        let gx_sq = format!("{}/gx_sq", ns);
        nodes.push(Proto::node("Mul", &[&gx2d, &gx2d], &[&gx_sq], &[]));
        let gy_sq = format!("{}/gy_sq", ns);
        nodes.push(Proto::node("Mul", &[&gy2d, &gy2d], &[&gy_sq], &[]));
        let r2 = format!("{}/r2", ns);
        nodes.push(Proto::node("Add", &[&gx_sq, &gy_sq], &[&r2], &[]));

        // 5) r⁴ = r²²
        let r4 = format!("{}/r4", ns);
        nodes.push(Proto::node("Mul", &[&r2, &r2], &[&r4], &[]));

        // 6) r⁶ = r⁴ * r²
        let r6 = format!("{}/r6", ns);
        nodes.push(Proto::node("Mul", &[&r4, &r2], &[&r6], &[]));

        // 7) denom = 1 + k1*r² + k2*r⁴ + k3*r⁶
        let k1r2 = format!("{}/k1r2", ns);
        nodes.push(Proto::node("Mul", &[&ek1, &r2], &[&k1r2], &[]));
        let k2r4 = format!("{}/k2r4", ns);
        nodes.push(Proto::node("Mul", &[&ek2, &r4], &[&k2r4], &[]));
        let k3r6 = format!("{}/k3r6", ns);
        nodes.push(Proto::node("Mul", &[&ek3, &r6], &[&k3r6], &[]));
        let t1 = format!("{}/t1", ns);
        nodes.push(Proto::node("Add", &[&one, &k1r2], &[&t1], &[]));
        let t2 = format!("{}/t2", ns);
        nodes.push(Proto::node("Add", &[&t1, &k2r4], &[&t2], &[]));
        let denom = format!("{}/denom", ns);
        nodes.push(Proto::node("Add", &[&t2, &k3r6], &[&denom], &[]));

        // 8) inv_denom = 1/denom
        let inv = format!("{}/inv", ns);
        nodes.push(Proto::node("Div", &[&one, &denom], &[&inv], &[]));

        // 9) gdc + eis: final = gdc_grid * inv_denom + eis
        let gx_gdc = format!("{}/gx_gdc", ns);
        nodes.push(Proto::node("Mul", &[&gx2d, &inv], &[&gx_gdc], &[]));
        let gy_gdc = format!("{}/gy_gdc", ns);
        nodes.push(Proto::node("Mul", &[&gy2d, &inv], &[&gy_gdc], &[]));
        let fx = format!("{}/fx", ns);
        nodes.push(Proto::node(
            "Add",
            &[&gx_gdc, &format!("{}/eis_dx", ns)],
            &[&fx],
            &[],
        ));
        let fy = format!("{}/fy", ns);
        nodes.push(Proto::node(
            "Add",
            &[&gy_gdc, &format!("{}/eis_dy", ns)],
            &[&fy],
            &[],
        ));

        // 10) Concat [fx, fy] → [1,1,H,W,2] → Reshape to [1,H,W,2]
        let grid5d = format!("{}/g5d", ns);
        nodes.push(Proto::node(
            "Concat",
            &[&fx, &fy],
            &[&grid5d],
            &[Proto::attribute_int("axis", -1)],
        ));
        let grid4d = format!("{}/grid", ns);
        nodes.push(Proto::node("Reshape", &[&grid5d, &shape4], &[&grid4d], &[]));

        // 11) GridSample
        let sampled = format!("{}/sampled", ns);
        nodes.push(Proto::node(
            "GridSample",
            &[&format!("{}/input", ns), &grid4d],
            &[&sampled],
            &[
                Proto::attribute_string("mode", "bilinear"),
                Proto::attribute_string("padding_mode", "zeros"),
                Proto::attribute_int("align_corners", 0),
            ],
        ));

        // 12) Identity → output
        nodes.push(Proto::node(
            "Identity",
            &[&sampled],
            &[&format!("{}/frame", ns)],
            &[],
        ));

        // ── Constants / initializers ──
        let grid_x_data: Vec<f32> = (0..w as usize)
            .map(|x| 2.0 * x as f32 / (w - 1) as f32 - 1.0)
            .collect();
        let grid_y_data: Vec<f32> = (0..h as usize)
            .map(|y| 2.0 * y as f32 / (h - 1) as f32 - 1.0)
            .collect();

        let initializers = vec![
            Proto::tensor_proto_float_scalar(&zero, 0.0),
            Proto::tensor_proto_float_scalar(&one, 1.0),
            Proto::tensor_proto_float_scalar(&breath, 0.15),
            Proto::tensor_proto_float(&gx, &[1, 1, 1, w], &grid_x_data),
            Proto::tensor_proto_float(&gy, &[1, 1, h, 1], &grid_y_data),
            Proto::tensor_proto_int64(&shape4, &[1, h, w, 2i64]),
        ];

        let graph = Proto::graph(
            "GpuWarpGraph",
            &nodes,
            &inputs,
            &outputs,
            &[],
            &initializers,
        );
        let opset = Proto::opset("", 21);
        Proto::model(8, &opset, "softisp", &graph)
    }

    /// Run warp inference into a pre-allocated output buffer.
    ///
    /// All ISP params flow through MNN tensors (not Rust function args).
    /// Grid is computed entirely by MNN on GPU — zero CPU involvement.
    ///
    /// `frame`: RGB planar float data [1,3,H,W] in [0,1] range.
    /// `k1,k2,k3`: GDC radial distortion coefficients.
    /// `zoom`: digital zoom factor [1.0, 4.0].
    /// `vcm`: focus motor position [0.0, 1.0].
    /// `eis_dx, eis_dy`: EIS displacement in normalized [-1,1] coords.
    ///     These are broadcast across the frame (same displacement for all pixels).
    ///     For per-pixel displacement, pass a [1,1,H,W] grid directly.
    /// `out`: pre-allocated output buffer (must be H*W*3 elements).
    ///
    /// Returns slice of `out` with warped data.
    pub fn run_into<'a>(
        &self,
        frame: &[f32],
        k1: f32,
        k2: f32,
        k3: f32,
        zoom: f32,
        vcm: f32,
        eis_dx: f32,
        eis_dy: f32,
        out: &'a mut [f32],
    ) -> crate::error::IspResult<&'a mut [f32]> {
        if !self.initialized {
            return Err(crate::error::IspError::Config(
                "warp engine not init".into(),
            ));
        }

        let h = self.height as usize;
        let w = self.width as usize;
        let n = h * w;

        if out.len() < n * 3 {
            return Err(crate::error::IspError::InvalidInput(format!(
                "output buffer too small: {} < {}",
                out.len(),
                n * 3
            )));
        }

        // ── Set frame input tensor ──
        if let Some(input) = self.interp.get_first_input(&self.session) {
            let _ = input.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, 3, h as i32, w as i32],
            );
            if let Some(bytes) = input.as_bytes_mut() {
                let copy_len = (frame.len() * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        frame.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }

        // ── Set ISP param tensors (all [1] scalars) ──
        let params: [(&str, f32); 5] = [
            ("GpuWarp/gdc_k1", k1),
            ("GpuWarp/gdc_k2", k2),
            ("GpuWarp/gdc_k3", k3),
            ("GpuWarp/zoom", zoom),
            ("GpuWarp/vcm", vcm),
        ];
        for (name, val) in &params {
            if let Some(t) = self.interp.get_input(&self.session, name) {
                let _ = t.set_shape(self.interp.as_ptr(), self.session.as_ptr(), &[1]);
                if let Some(bytes) = t.as_bytes_mut() {
                    unsafe {
                        std::ptr::copy_nonoverlapping(
                            val as *const f32 as *const u8,
                            bytes.as_mut_ptr(),
                            4,
                        );
                    }
                }
            }
        }

        // ── Set EIS displacement tensors (uniform scalar broadcast to [1,1,H,W]) ──
        // For simplicity, pass uniform scalar values that MNN broadcasts.
        // The ONNX model has eis_dx/eis_dy as [1,1,H,W] tensors; here we create
        // a uniform grid where every pixel gets the same displacement.
        let eis_grid_x = vec![eis_dx; n];
        let eis_grid_y = vec![eis_dy; n];
        if let Some(t) = self.interp.get_input(&self.session, "GpuWarp/eis_dx") {
            let _ = t.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, 1, h as i32, w as i32],
            );
            if let Some(bytes) = t.as_bytes_mut() {
                let copy_len = (n * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        eis_grid_x.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }
        if let Some(t) = self.interp.get_input(&self.session, "GpuWarp/eis_dy") {
            let _ = t.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, 1, h as i32, w as i32],
            );
            if let Some(bytes) = t.as_bytes_mut() {
                let copy_len = (n * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        eis_grid_y.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }

        // ── Resize + Run ──
        self.session
            .resize()
            .map_err(|e| crate::error::IspError::Mnn(format!("warp resize: {}", e)))?;
        self.session
            .run()
            .map_err(|e| crate::error::IspError::Mnn(format!("warp inference: {}", e)))?;

        // ── Read output into caller's buffer ──
        let output = self
            .interp
            .get_first_output(&self.session)
            .ok_or_else(|| crate::error::IspError::Mnn("warp output missing".into()))?;
        let out_bytes = output
            .as_bytes()
            .ok_or_else(|| crate::error::IspError::Mnn("warp output null".into()))?;

        let copy_bytes = (n * 3 * 4).min(out_bytes.len()).min(out.len() * 4);
        unsafe {
            std::ptr::copy_nonoverlapping(
                out_bytes.as_ptr(),
                out.as_mut_ptr() as *mut u8,
                copy_bytes,
            );
        }

        Ok(&mut out[..n * 3])
    }

    /// Run warp inference (allocates output buffer).
    pub fn run(
        &self,
        frame: &[f32],
        k1: f32,
        k2: f32,
        k3: f32,
        zoom: f32,
        vcm: f32,
        eis_dx: f32,
        eis_dy: f32,
    ) -> crate::error::IspResult<Vec<f32>> {
        let n = (self.width * self.height * 3) as usize;
        let mut out = vec![0.0f32; n];
        self.run_into(frame, k1, k2, k3, zoom, vcm, eis_dx, eis_dy, &mut out)?;
        Ok(out)
    }
}

#[cfg(test)]
#[cfg(feature = "mnn")]
mod tests {
    use super::*;

    #[test]
    fn test_gpu_warp_engine_build() {
        crate::init();

        // Build a simple warp ONNX (GridSample only, grid computed on CPU)
        let w = 64u32;
        let h = 64u32;
        let onnx = GpuWarpEngine::generate_onnx(w, h);

        // Create engine
        let engine = GpuWarpEngine::from_onnx(&onnx, w, h);
        assert!(engine.is_ok(), "Build failed: {:?}", engine.err());
    }
}
