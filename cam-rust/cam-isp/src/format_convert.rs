//! FormatConvertEngine — GPU format conversion via ONNX.
//!
//! Wraps a small DisplayBlock-style ONNX model to convert float RGB
//! to RGBA/BGRA/ARGB/Float16/etc on GPU via MNN Vulkan, avoiding
//! CPU-side format conversion loops.
//!
//! # Architecture
//!
//! ```text
//! Input:  [1, 3, H, W] f32 RGB planar [0,1]
//! Output: [1, 4, H, W] f32 RGBA/ARGB/BGRA [0,255]
//!         or [1, 3, H, W] f32 RGB [0,255]
//!         or [1, 3, H, W] f16 Float16RGB [0,1]
//! ```

#[cfg(feature = "mnn")]
use crate::engine::OutputFormat;
#[cfg(feature = "mnn")]
use crate::onnx::proto::Proto;
use log::info;
#[cfg(feature = "mnn")]
use std::os::unix::io::RawFd;

/// GPU-accelerated format converter.
///
/// Builds a standalone ONNX model from DisplayBlock's conversion pattern
/// and runs it through MNN Vulkan.
#[cfg(feature = "mnn")]
pub struct FormatConvertEngine {
    interp: crate::mnn::MnnInterpreterSafe,
    session: crate::mnn::MnnSessionSafe,
    width: u32,
    height: u32,
    output_format: OutputFormat,
    initialized: bool,
}

#[cfg(feature = "mnn")]
impl FormatConvertEngine {
    /// Create format converter for given dimensions and output format.
    pub fn new(
        width: u32,
        height: u32,
        output_format: OutputFormat,
    ) -> crate::error::IspResult<Self> {
        use crate::mnn_converter::convert_onnx_buffer;
        use crate::mnnengine::{MnnBackendType, MnnInterpreterSafe};

        info!(
            "FormatConvertEngine: {}×{} → {:?}",
            width, height, output_format
        );

        // Build ONNX model using DisplayBlock's pattern
        let onnx = build_format_convert_onnx(width, height, output_format);

        // Convert ONNX→MNN and load it through a memfd so the model never
        // round-trips through the Rust heap. Falls back to the heap buffer
        // path if the memfd conversion is unavailable on this platform.
        let interp = match crate::mnn_converter::convert_onnx_memfd(&onnx) {
            Ok(fd) => {
                let interp = MnnInterpreterSafe::from_fd(fd).ok_or_else(|| {
                    crate::error::IspError::Mnn("fmt conv model load fail".into())
                })?;
                // MNN has parsed the flatbuffer into memory; the fd is no longer needed.
                let _ = unsafe { libc::close(fd) };
                interp
            }
            Err(_) => {
                let mnn_bytes = convert_onnx_buffer(&onnx).map_err(|e| {
                    crate::error::IspError::Conversion(format!("convert fmt conv: {}", e))
                })?;
                MnnInterpreterSafe::from_buffer(&mnn_bytes)
                    .ok_or_else(|| crate::error::IspError::Mnn("fmt conv model load fail".into()))?
            }
        };

        let session = interp
            .create_session(MnnBackendType::Cpu, 2)
            .ok_or_else(|| crate::error::IspError::Mnn("fmt conv session fail".into()))?;

        info!("FormatConvertEngine: ready ({} bytes ONNX)", onnx.len());

        Ok(Self {
            interp,
            session,
            width,
            height,
            output_format,
            initialized: true,
        })
    }

    /// Convert float RGB planar to target format.
    ///
    /// `input`: RGB planar [R₀..Rₙ, G₀..Gₙ, B₀..Bₙ] f32 [0,1].
    /// `output`: pre-allocated output buffer.
    ///
    /// Returns number of bytes written to output.
    pub fn convert(&self, input: &[f32], output: &mut [u8]) -> crate::error::IspResult<usize> {
        if !self.initialized {
            return Err(crate::error::IspError::Config("fmt conv not init".into()));
        }

        let h = self.height as usize;
        let w = self.width as usize;
        let n = h * w;

        // Set input tensor
        if let Some(tensor) = self.interp.get_first_input(&self.session) {
            let _ = tensor.set_shape(
                self.interp.as_ptr(),
                self.session.as_ptr(),
                &[1, 3, h as i32, w as i32],
            );
            if let Some(bytes) = tensor.as_bytes_mut() {
                let copy_len = (input.len() * 4).min(bytes.len());
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        input.as_ptr() as *const u8,
                        bytes.as_mut_ptr(),
                        copy_len,
                    );
                }
            }
        }

        // Run
        self.session
            .resize()
            .map_err(|e| crate::error::IspError::Mnn(format!("fmt conv resize: {}", e)))?;
        self.session
            .run()
            .map_err(|e| crate::error::IspError::Mnn(format!("fmt conv run: {}", e)))?;

        // Read output
        let out_tensor = self
            .interp
            .get_first_output(&self.session)
            .ok_or_else(|| crate::error::IspError::Mnn("fmt conv output missing".into()))?;
        let out_bytes = out_tensor
            .as_bytes()
            .ok_or_else(|| crate::error::IspError::Mnn("fmt conv output null".into()))?;

        let bpp = self.output_format.bytes_per_pixel();
        let expected = n * bpp;
        let copy_len = out_bytes.len().min(output.len()).min(expected);
        output[..copy_len].copy_from_slice(&out_bytes[..copy_len]);

        Ok(copy_len)
    }

    /// Get output format.
    pub fn output_format(&self) -> OutputFormat {
        self.output_format
    }
}

/// Build a standalone ONNX model for format conversion.
///
/// Uses the same Conv(1×1)/Mul/Cast pattern as DisplayBlock.
#[cfg(feature = "mnn")]
fn build_format_convert_onnx(w: u32, h: u32, fmt: OutputFormat) -> Vec<u8> {
    use OutputFormat::*;

    let input = "fmt_conv/input";
    let output_name = "fmt_conv/output";
    let ns = "FmtConv";

    let mut nodes: Vec<Vec<u8>> = Vec::new();
    let mut inits: Vec<Vec<u8>> = Vec::new();
    let prev = input.to_string();

    // Input: [1, 3, H, W] f32
    let vi = Proto::value_info(
        input,
        &[
            Proto::tensor_dim_value(1),
            Proto::tensor_dim_value(3),
            Proto::tensor_dim_value(h as i64),
            Proto::tensor_dim_value(w as i64),
        ],
        1,
    ); // 1 = FLOAT

    match fmt {
        FloatRgb | Float16Rgb => {
            // Identity: just pass through (Mul 1.0)
            let scale = format!("{}/scale", ns);
            inits.push(Proto::tensor_proto_float_scalar(&scale, 1.0));
            if prev != output_name {
                nodes.push(Proto::node("Mul", &[&prev, &scale], &[output_name], &[]));
            }
            if fmt == Float16Rgb {
                // Cast FLOAT→FLOAT16
                nodes.push(Proto::node(
                    "Cast",
                    &[input],
                    &[output_name],
                    &[Proto::attribute_int("to", 10)],
                ));
            }
        }
        FloatBgra | Float16Bgra | Bgra | Rgba | Argb | Abgr | Rgb | Bgr => {
            // Conv(1×1) channel permutation + scale(255) + alpha bias
            let conv_w = format!("{}/conv_w", ns);
            let conv_b = format!("{}/conv_b", ns);
            let conv_out = format!("{}/conv_out", ns);
            let oc = fmt.channel_count();
            let (weights, bias): (Vec<f32>, Vec<f32>) = match fmt {
                FloatBgra | Float16Bgra | Bgra => (
                    vec![
                        0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    ],
                    vec![0.0, 0.0, 0.0, 255.0],
                ),
                Rgba => (
                    vec![
                        255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0,
                    ],
                    vec![0.0, 0.0, 0.0, 255.0],
                ),
                Argb => (
                    vec![
                        0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0,
                    ],
                    vec![255.0, 0.0, 0.0, 0.0],
                ),
                Abgr => (
                    vec![
                        0.0, 0.0, 0.0, 0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0,
                    ],
                    vec![255.0, 0.0, 0.0, 0.0],
                ),
                Rgb => (
                    vec![255.0, 0.0, 0.0, 0.0, 255.0, 0.0, 0.0, 0.0, 255.0],
                    vec![0.0, 0.0, 0.0],
                ),
                Bgr => (
                    vec![0.0, 0.0, 255.0, 0.0, 255.0, 0.0, 255.0, 0.0, 0.0],
                    vec![0.0, 0.0, 0.0],
                ),
                _ => unreachable!(),
            };
            inits.push(Proto::tensor_proto_float(
                &conv_w,
                &[oc as i64, 3, 1, 1],
                &weights,
            ));
            inits.push(Proto::tensor_proto_float(&conv_b, &[oc as i64], &bias));
            nodes.push(Proto::node(
                "Conv",
                &[input, &conv_w, &conv_b],
                &[&conv_out],
                &[
                    Proto::attribute_ints("kernel_shape", &[1, 1]),
                    Proto::attribute_int("group", 1),
                ],
            ));

            if fmt == Float16Bgra {
                // Cast FLOAT→FLOAT16
                nodes.push(Proto::node(
                    "Cast",
                    &[&conv_out],
                    &[output_name],
                    &[Proto::attribute_int("to", 10)],
                ));
            } else {
                nodes.push(Proto::node("Identity", &[&conv_out], &[output_name], &[]));
            }
        }
        PackedRgb => {
            // Mul(255) + Cast(FLOAT→INT32) — simplified pack
            let scale = format!("{}/scale", ns);
            let scale_255 = format!("{}/scale_255", ns);
            let cast_out = format!("{}/cast", ns);
            inits.push(Proto::tensor_proto_float_scalar(&scale_255, 255.0));
            inits.push(Proto::tensor_proto_float_scalar(&scale, 1.0));
            nodes.push(Proto::node("Mul", &[input, &scale_255], &[&cast_out], &[]));
            nodes.push(Proto::node(
                "Cast",
                &[&cast_out],
                &[output_name],
                &[Proto::attribute_int("to", 6)],
            )); // 6 = INT32
        }
    }

    let bpp = fmt.channel_count() as i64;
    let graph = Proto::graph(
        "FormatConvert",
        &nodes,
        &[vi],
        &[Proto::value_info(
            output_name,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_value(bpp),
                Proto::tensor_dim_value(h as i64),
                Proto::tensor_dim_value(w as i64),
            ],
            fmt.onnx_elem_type(),
        )],
        &inits,
        &[],
    );

    // Opset 13
    let opset = Proto::opset("", 13);

    Proto::model(11, &opset, "cam_rust_fmt_convert", &graph)
}

#[cfg(all(test, feature = "mnn"))]
mod tests {
    use super::*;
    use crate::engine::OutputFormat;

    /// Returns true if running in CI (GitHub Actions, etc.).
    fn is_ci() -> bool {
        std::env::var("CI").is_ok() || std::env::var("GITHUB_ACTIONS").is_ok()
    }

    #[test]
    fn test_format_convert_onnx_rgba() {
        if is_ci() {
            eprintln!("Skipping on CI — requires MNN Vulkan runtime");
            return;
        }
        let onnx = build_format_convert_onnx(1920, 1080, OutputFormat::Rgba);
        // Should produce a valid ONNX model
        assert!(
            onnx.len() > 100,
            "ONNX model too small: {} bytes",
            onnx.len()
        );
    }

    #[test]
    fn test_format_convert_onnx_argb() {
        let onnx = build_format_convert_onnx(640, 480, OutputFormat::Argb);
        assert!(onnx.len() > 100);
    }

    #[test]
    fn test_format_convert_onnx_float16() {
        let onnx = build_format_convert_onnx(320, 240, OutputFormat::Float16Rgb);
        assert!(onnx.len() > 100);
    }

    #[test]
    fn test_format_convert_onnx_rgb() {
        let onnx = build_format_convert_onnx(1280, 720, OutputFormat::Rgb);
        assert!(onnx.len() > 100);
    }

    #[test]
    fn test_format_convert_onnx_bgr() {
        let onnx = build_format_convert_onnx(1280, 720, OutputFormat::Bgr);
        assert!(onnx.len() > 100);
    }

    #[test]
    fn test_format_convert_onnx_abgr() {
        let onnx = build_format_convert_onnx(1280, 720, OutputFormat::Abgr);
        assert!(onnx.len() > 100);
    }
}
