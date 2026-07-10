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

/// GPU warp engine — separate MNN session for GDC+EIS warp.
#[cfg(feature = "mnn")]
pub struct GpuWarpEngine {
    interp: MnnInterpreterSafe,
    session: MnnSessionSafe,
    width: u32,
    height: u32,
    initialized: bool,
}

#[cfg(feature = "mnn")]
impl GpuWarpEngine {
    /// Create warp engine from ONNX bytes.
    pub fn from_onnx(onnx: &[u8], width: u32, height: u32) -> IspResult<Self> {
        info!("GpuWarpEngine: creating from {} byte ONNX for {}×{}", onnx.len(), width, height);

        // Write ONNX to temp file
        let on = format!(".warp_temp_{}.onnx", std::process::id());
        let mn = on.replace(".onnx", ".mnn");
        std::fs::write(&on, onnx)
            .map_err(|e| crate::error::IspError::Io(format!("write onnx: {}", e)))?;

        // Convert ONNX → MNN
        let opts = MnnConvertOptions::default();
        convert_onnx_to_mnn(&on, &mn, Some(&opts))
            .map_err(|e| crate::error::IspError::Conversion(format!("convert warp: {}", e)))?;
        let _ = std::fs::remove_file(&on);

        // Load MNN model
        let interp = MnnInterpreterSafe::from_file(&mn)
            .ok_or_else(|| crate::error::IspError::Mnn("warp model load fail".into()))?;
        let _ = std::fs::remove_file(&mn);

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

        // Set GDC coefficients
        self.set_const_f32("GpuWarp/gdc_k1", k1);
        self.set_const_f32("GpuWarp/gdc_k2", k2);
        self.set_const_f32("GpuWarp/gdc_k3", k3);

        // Set EIS grid
        let eis_grid_x: Vec<f32> = vec![eis_x; n];
        let eis_grid_y: Vec<f32> = vec![eis_y; n];
        self.set_const_f32_grid("GpuWarp/eis_x", &eis_grid_x, h, w);
        self.set_const_f32_grid("GpuWarp/eis_y", &eis_grid_y, h, w);

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

    /// Set a scalar float constant as input tensor.
    fn set_const_f32(&self, name: &str, value: f32) {
        if let Some(tensor) = self.interp.get_input(&self.session, name) {
            let _ = tensor.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1],
            );
            if let Some(bytes) = tensor.as_bytes_mut() {
                if bytes.len() >= 4 {
                    unsafe {
                        *(bytes.as_mut_ptr() as *mut f32) = value;
                    }
                }
            }
        }
    }

    /// Set a 4D float grid as input tensor.
    fn set_const_f32_grid(&self, name: &str, data: &[f32], h: usize, w: usize) {
        if let Some(tensor) = self.interp.get_input(&self.session, name) {
            let _ = tensor.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, 1, h as i32, w as i32],
            );
            if let Some(bytes) = tensor.as_bytes_mut() {
                let copy_len = (data.len() * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        data.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }
    }
}

#[cfg(test)]
#[cfg(feature = "mnn")]
mod tests {
    use super::*;
    use crate::blocks::gpu_warp::GpuWarpBlock;
    use crate::pipeline::GraphComposer;
    use crate::pipeline::IspBlock;
    use crate::onnx::proto::Proto;

    #[test]
    fn test_gpu_warp_engine_build() {
        crate::init();

        // Build warp ONNX
        let w = 64u32;
        let h = 64u32;
        let block = GpuWarpBlock::new(w, h);

        let nodes = block.nodes();
        let inits = block.initializers();
        let extras = block.extra_inputs();

        // Build graph inputs
        let mut graph_inputs = vec![
            Proto::value_info(
                "GpuWarp/input",
                &[Proto::tensor_dim_value(1),
                  Proto::tensor_dim_value(3),
                  Proto::tensor_dim_param("H"),
                  Proto::tensor_dim_param("W")], 1),
        ];
        for (name, etype, shape) in &extras {
            let dim_protos: Vec<Vec<u8>> = shape.iter().map(|d| {
                if *d > 0 {
                    Proto::tensor_dim_value(*d)
                } else {
                    Proto::tensor_dim_param("?")
                }
            }).collect();
            graph_inputs.push(Proto::value_info(name, &dim_protos, *etype as i32));
        }

        let graph_outputs = vec![
            Proto::value_info(
                "GpuWarp/frame",
                &[Proto::tensor_dim_value(1),
                  Proto::tensor_dim_value(3),
                  Proto::tensor_dim_param("H"),
                  Proto::tensor_dim_param("W")], 1),
        ];

        let graph = Proto::graph(
            "gpu_warp", &nodes, &graph_inputs, &graph_outputs, &inits, &[]);
        let opset = Proto::opset("", 21);
        let onnx = Proto::model(8, &opset, "test-warp", &graph);

        // Create engine
        let engine = GpuWarpEngine::from_onnx(&onnx, w, h);
        assert!(engine.is_ok(), "Build failed: {:?}", engine.err());
    }
}
