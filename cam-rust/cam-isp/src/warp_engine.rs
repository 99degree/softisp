//! GPU Warp Engine — runs GDC+EIS warp via separate MNN session.
//!
//! Takes the GPU warp ONNX model and runs it as a separate inference pass
//! after the main ISP pipeline. Parameters (k1, k2, k3, eis_x, eis_y)
//! are set per-frame before inference.

#[cfg(feature = "mnn")]
use log::info;
#[cfg(feature = "mnn")]
use crate::error::IspResult;
#[cfg(feature = "mnn")]
use crate::mnn::mnn_sys::{
    MnnInterpreterSafe, MnnSessionSafe,
    MnnBackendType,
};
#[cfg(feature = "mnn")]
use crate::mnn::mnn_converter::{convert_onnx_to_mnn, MnnConvertOptions};

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
    pub fn from_isp_params(gdc_k1: f32, gdc_k2: f32, gdc_k3: f32,
                           isp: &crate::isp_params::IspParams) -> Self {
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

#[cfg(feature = "mnn")]
impl GpuWarpEngine {
    /// Create warp engine from ONNX bytes.
    pub fn from_onnx(onnx: &[u8], width: u32, height: u32) -> IspResult<Self> {
        info!("GpuWarpEngine: creating from {} byte ONNX for {}×{}", onnx.len(), width, height);

        // Write ONNX to temp file, convert to MNN
        let pid = std::process::id();
        let tid = std::thread::current().id();
        let stamp = std::time::UNIX_EPOCH.elapsed().unwrap_or_default().as_nanos();
        let on = format!(".warp_{}_{}_{:?}.onnx", pid, stamp, tid);
        let mn = on.replace(".onnx", ".mnn");
        std::fs::write(&on, onnx)
            .map_err(|e| crate::error::IspError::Io(format!("write onnx: {}", e)))?;

        let opts = MnnConvertOptions::default();
        convert_onnx_to_mnn(&on, &mn, Some(&opts))
            .map_err(|e| crate::error::IspError::Conversion(format!("convert warp: {}", e)))?;
        let _ = std::fs::remove_file(&on);

        // Load MNN model from buffer (avoids C++ static init issues with from_file)
        let mnn_bytes = std::fs::read(&mn)
            .map_err(|e| crate::error::IspError::Io(format!("read mnn: {}", e)))?;
        let _ = std::fs::remove_file(&mn);

        let interp = MnnInterpreterSafe::from_buffer(&mnn_bytes)
            .ok_or_else(|| crate::error::IspError::Mnn("warp model load fail".into()))?;

        // Create session (CPU for warp — lightweight)
        let session = interp.create_session(MnnBackendType::Cpu, 2)
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

    /// Generate a default ONNX model for GDC+EIS warp.
    ///
    /// Simple GridSample model: takes frame + grid as inputs.
    /// Grid is pre-computed on CPU in `run_into()` for reliability.
    ///
    /// Inputs:
    ///   GpuWarp/input  [1,3,H,W]  — frame to warp
    ///   GpuWarp/grid   [1,H,W,2]  — normalized sampling grid [-1,1]
    ///
    /// Output:
    ///   GpuWarp/frame  [1,3,H,W]  — warped frame
    pub fn generate_onnx(width: u32, height: u32) -> Vec<u8> {
        use crate::onnx::proto::Proto;

        let h = height as i64;
        let w = width as i64;

        // Graph inputs
        let inputs = vec![
            Proto::value_info("GpuWarp/input", &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ], 1),
            Proto::value_info("GpuWarp/grid", &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
                Proto::tensor_dim_value(2),
            ], 1),
        ];

        // Graph output
        let outputs = vec![
            Proto::value_info("GpuWarp/frame", &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(3),
                Proto::tensor_dim_value(h),
                Proto::tensor_dim_value(w),
            ], 1),
        ];

        // Single GridSample node
        let nodes = vec![
            Proto::node("GridSample", &["GpuWarp/input", "GpuWarp/grid"], &["GpuWarp/frame"], &[
                Proto::attribute_string("mode", "bilinear"),
                Proto::attribute_string("padding_mode", "zeros"),
                Proto::attribute_int("align_corners", 0),
            ]),
        ];

        let graph = Proto::graph("GpuWarpGraph", &nodes, &inputs, &outputs, &[], &[]);
        let opset = Proto::opset("", 21);
        Proto::model(8, &opset, "softisp", &graph)
    }

    /// Run warp inference into a pre-allocated output buffer.
    ///
    /// `frame`: RGB planar float data [1,3,H,W] in [0,1] range.
    /// `k1,k2,k3`: GDC radial distortion coefficients.
    /// `eis_x,eis_y`: EIS displacement in normalized [-1,1] coords.
    /// `out`: pre-allocated output buffer (must be H*W*3 elements).
    ///
    /// Returns slice of `out` with warped data.
    pub fn run_into<'a>(
        &self,
        frame: &[f32],
        k1: f32,
        k2: f32,
        k3: f32,
        eis_x: f32,
        eis_y: f32,
        out: &'a mut [f32],
    ) -> crate::error::IspResult<&'a mut [f32]> {
        if !self.initialized {
            return Err(crate::error::IspError::Config("warp engine not init".into()));
        }

        let h = self.height as usize;
        let w = self.width as usize;
        let n = h * w;

        if out.len() < n * 3 {
            return Err(crate::error::IspError::InvalidInput(
                format!("output buffer too small: {} < {}", out.len(), n * 3)));
        }

        // Set main frame input
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

        // Compute GDC+EIS grid on CPU
        // Grid layout: [1, H, W, 2] where grid[y][x] = [gdc_x + eis_x, gdc_y + eis_y]
        let mut grid = vec![0.0f32; n * 2];
        for y in 0..h {
            let ny = 2.0 * y as f32 / (h - 1) as f32 - 1.0;
            for x in 0..w {
                let nx = 2.0 * x as f32 / (w - 1) as f32 - 1.0;
                let r2 = nx * nx + ny * ny;
                let r4 = r2 * r2;
                let r6 = r4 * r2;
                let denom = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
                let inv_denom = 1.0 / denom;
                let idx = y * w + x;
                grid[idx * 2] = nx * inv_denom + eis_x;
                grid[idx * 2 + 1] = ny * inv_denom + eis_y;
            }
        }

        // Set grid input tensor
        if let Some(grid_tensor) = self.interp.get_input(&self.session, "GpuWarp/grid") {
            let _ = grid_tensor.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, h as i32, w as i32, 2],
            );
            if let Some(bytes) = grid_tensor.as_bytes_mut() {
                let copy_len = (grid.len() * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        grid.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }

        // Resize + Run
        self.session.resize()
            .map_err(|e| crate::error::IspError::Mnn(format!("warp resize: {}", e)))?;
        self.session.run()
            .map_err(|e| crate::error::IspError::Mnn(format!("warp inference: {}", e)))?;

        // Read output directly into caller's buffer
        let output = self.interp.get_first_output(&self.session)
            .ok_or_else(|| crate::error::IspError::Mnn("warp output missing".into()))?;
        let out_bytes = output.as_bytes()
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
        eis_x: f32,
        eis_y: f32,
    ) -> crate::error::IspResult<Vec<f32>> {
        let n = (self.width * self.height * 3) as usize;
        let mut out = vec![0.0f32; n];
        self.run_into(frame, k1, k2, k3, eis_x, eis_y, &mut out)?;
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

