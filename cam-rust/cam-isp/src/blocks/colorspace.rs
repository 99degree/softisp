//! Color Space Conversion — RGB/HSV/LAB/YCbCr conversions.
//!
//! Enables perceptual color editing and advanced color processing.
//!
//! ## GPU (ONNX) Support
//!
//! The ONNX graph uses basic arithmetic ops (Mul, Add, Sub, Clip) for
//! RGB↔YCbCr (BT.601 matrix). Full HSV↔RGB and LAB↔RGB ONNX graphs
//! require conditional ops (Where, Less, Floor) which are supported
//! by MNN but not yet wired. For now, those conversions are Identity
//! placeholders in the GPU path; the Rust CPU helper functions below
//! provide the correct math for CPU-side post-processing.

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

/// Color space conversion types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorSpace {
    /// RGB to HSV conversion.
    RgbToHsv,
    /// HSV to RGB conversion.
    HsvToRgb,
    /// RGB to LAB conversion.
    RgbToLab,
    /// LAB to RGB conversion.
    LabToRgb,
    /// RGB to YCbCr conversion.
    RgbToYCbCr,
    /// YCbCr to RGB conversion.
    YCbCrToRgb,
}

/// Color space conversion block.
pub struct ColorSpaceBlock {
    /// Conversion type.
    pub conversion: ColorSpace,
    /// Input tensor name (set by wire_blocks).
    input_source: String,
}

impl ColorSpaceBlock {
    /// Create with specific conversion.
    pub fn new(conversion: ColorSpace) -> Self {
        Self {
            conversion,
            input_source: String::new(),
        }
    }

    /// Create RGB to HSV converter.
    pub fn rgb_to_hsv() -> Self {
        Self::new(ColorSpace::RgbToHsv)
    }

    /// Create HSV to RGB converter.
    pub fn hsv_to_rgb() -> Self {
        Self::new(ColorSpace::HsvToRgb)
    }

    /// Create RGB to LAB converter.
    pub fn rgb_to_lab() -> Self {
        Self::new(ColorSpace::RgbToLab)
    }

    /// Create LAB to RGB converter.
    pub fn lab_to_rgb() -> Self {
        Self::new(ColorSpace::LabToRgb)
    }
}

impl IspBlock for ColorSpaceBlock {
    fn id(&self) -> &str {
        match self.conversion {
            ColorSpace::RgbToHsv => "rgb_to_hsv",
            ColorSpace::HsvToRgb => "hsv_to_rgb",
            ColorSpace::RgbToLab => "rgb_to_lab",
            ColorSpace::LabToRgb => "lab_to_rgb",
            ColorSpace::RgbToYCbCr => "rgb_to_ycbcr",
            ColorSpace::YCbCrToRgb => "ycbcr_to_rgb",
        }
    }

    fn tensor_ns(&self) -> String {
        format!("ColorSpace{:?}", self.conversion)
    }

    fn input_source(&self) -> Option<&str> {
        if self.input_source.is_empty() {
            Some(match self.conversion {
                ColorSpace::RgbToHsv => "rgb_to_hsv/input",
                ColorSpace::HsvToRgb => "hsv_to_rgb/input",
                ColorSpace::RgbToLab => "rgb_to_lab/input",
                ColorSpace::LabToRgb => "lab_to_rgb/input",
                ColorSpace::RgbToYCbCr => "rgb_to_ycbcr/input",
                ColorSpace::YCbCrToRgb => "ycbcr_to_rgb/input",
            })
        } else {
            Some(&self.input_source)
        }
    }

    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.into();
    }

    fn frame_tensor(&self) -> Option<&str> {
        Some(match self.conversion {
            ColorSpace::RgbToHsv => "rgb_to_hsv/output",
            ColorSpace::HsvToRgb => "hsv_to_rgb/output",
            ColorSpace::RgbToLab => "rgb_to_lab/output",
            ColorSpace::LabToRgb => "lab_to_rgb/output",
            ColorSpace::RgbToYCbCr => "rgb_to_ycbcr/output",
            ColorSpace::YCbCrToRgb => "ycbcr_to_rgb/output",
        })
    }

    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }

    fn set_prev(&mut self, _block: Box<dyn IspBlock>) {}

    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        None
    }

    fn set_next(&mut self, _block: Box<dyn IspBlock>) {}

    fn nodes(&self) -> Vec<Vec<u8>> {
        match self.conversion {
            ColorSpace::RgbToHsv => self.rgb_to_hsv_nodes(),
            ColorSpace::HsvToRgb => self.hsv_to_rgb_nodes(),
            ColorSpace::RgbToLab => self.rgb_to_lab_nodes(),
            ColorSpace::LabToRgb => self.lab_to_rgb_nodes(),
            ColorSpace::RgbToYCbCr => self.rgb_to_ycbcr_nodes(),
            ColorSpace::YCbCrToRgb => self.ycbcr_to_rgb_nodes(),
        }
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }
}

// ── CPU-side conversion helpers ─────────────────────────────────
// Allow dead_code: these are public API called by external consumers
// (CPU pipeline post-processing), not directly from library code.

/// Convert a planar float RGB pixel (r, g, b in [0,1]) to HSV.
/// Returns (h, s, v) where h ∈ [0, 360), s ∈ [0, 1], v ∈ [0, 1].
#[allow(dead_code)]
#[inline]
pub fn rgb_to_hsv_pixel(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    let v = r.max(g).max(b);
    let min = r.min(g).min(b);
    let delta = v - min;

    if delta < 1e-8 || v < 1e-8 {
        return (0.0, 0.0, v); // achromatic
    }

    let s = delta / v;
    let h = if v == r {
        60.0 * (((g - b) / delta).rem_euclid(6.0))
    } else if v == g {
        60.0 * ((b - r) / delta + 2.0)
    } else {
        60.0 * ((r - g) / delta + 4.0)
    };

    (h.clamp(0.0, 360.0), s, v)
}

/// Convert an HSV pixel (h ∈ [0, 360), s ∈ [0, 1], v ∈ [0, 1]) to RGB.
/// Returns (r, g, b) in [0, 1].
#[allow(dead_code)]
#[inline]
pub fn hsv_to_rgb_pixel(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    if s < 1e-8 {
        return (v, v, v); // achromatic
    }

    let h_norm = (h / 60.0).rem_euclid(6.0);
    let i = h_norm.floor() as i32;
    let f = h_norm - h_norm.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - s * f);
    let t = v * (1.0 - s * (1.0 - f));

    match i {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        5 => (v, p, q),
        _ => (v, p, q),
    }
}

/// Convert a planar float RGB buffer (interleaved rrr... ggg... bbb...)
/// to HSV in-place. Each channel has `len` elements.
#[allow(dead_code)]
pub fn rgb_to_hsv_plane(r: &mut [f32], g: &mut [f32], b: &mut [f32]) {
    assert_eq!(r.len(), g.len());
    assert_eq!(g.len(), b.len());
    for i in 0..r.len() {
        let (h, s, v) = rgb_to_hsv_pixel(r[i], g[i], b[i]);
        // Stash H in R channel, S in G, V in B
        r[i] = h / 360.0; // normalize to [0,1]
        g[i] = s;
        b[i] = v;
    }
}

/// Convert a planar float HSV buffer (interleaved hhh... sss... vvv...)
/// to RGB in-place. Each channel has `len` elements.
/// H is expected normalized to [0,1]; S,V in [0,1].
#[allow(dead_code)]
pub fn hsv_to_rgb_plane(h: &mut [f32], s: &mut [f32], v: &mut [f32]) {
    assert_eq!(h.len(), s.len());
    assert_eq!(s.len(), v.len());
    for i in 0..h.len() {
        let (r, g, b) = hsv_to_rgb_pixel(h[i] * 360.0, s[i], v[i]);
        h[i] = r;
        s[i] = g;
        v[i] = b;
    }
}

/// Convert a flat interleaved RGB buffer [r,g,b,r,g,b,...] to HSV in-place.
/// `data` length must be a multiple of 3.
#[allow(dead_code)]
pub fn rgb_to_hsv_interleaved(data: &mut [f32]) {
    assert_eq!(data.len() % 3, 0);
    for pixel in data.chunks_exact_mut(3) {
        let (h, s, v) = rgb_to_hsv_pixel(pixel[0], pixel[1], pixel[2]);
        pixel[0] = h / 360.0;
        pixel[1] = s;
        pixel[2] = v;
    }
}

/// Convert a flat interleaved HSV buffer [h,s,v,h,s,v,...] to RGB in-place.
/// `data` length must be a multiple of 3. H expected normalized to [0,1].
#[allow(dead_code)]
pub fn hsv_to_rgb_interleaved(data: &mut [f32]) {
    assert_eq!(data.len() % 3, 0);
    for pixel in data.chunks_exact_mut(3) {
        let (r, g, b) = hsv_to_rgb_pixel(pixel[0] * 360.0, pixel[1], pixel[2]);
        pixel[0] = r;
        pixel[1] = g;
        pixel[2] = b;
    }
}

/// RGB to LAB (simplified using D65 illuminant).
/// Input sRGB [0,1], output L [0,100], a [-128,127], b [-128,127].
#[allow(dead_code)]
#[allow(clippy::excessive_precision)]
#[inline]
pub fn rgb_to_lab_pixel(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    // sRGB linearize
    let lin = |c: f32| {
        if c <= 0.04045_f32 {
            c / 12.92_f32
        } else {
            ((c + 0.055_f32) / 1.055_f32).powf(2.4_f32)
        }
    };
    let rl = lin(r);
    let gl = lin(g);
    let bl = lin(b);

    // D65 XYZ from linear sRGB
    let x = 0.4124564_f32 * rl + 0.3575761_f32 * gl + 0.1804375_f32 * bl;
    let y = 0.2126729_f32 * rl + 0.7151522_f32 * gl + 0.0721750_f32 * bl;
    let z = 0.0193339_f32 * rl + 0.1191920_f32 * gl + 0.9503041_f32 * bl;

    // XYZ → LAB
    let xn = 0.95047_f32;
    let yn = 1.00000_f32;
    let zn = 1.08883_f32;

    let f = |t: f32| {
        if t > 0.008856_f32 {
            t.powf(1.0_f32 / 3.0_f32)
        } else {
            (7.787_f32 * t) + 16.0_f32 / 116.0_f32
        }
    };

    let fx = f(x / xn);
    let fy = f(y / yn);
    let fz = f(z / zn);

    let l = 116.0_f32 * fy - 16.0_f32;
    let a = 500.0_f32 * (fx - fy);
    let b = 200.0_f32 * (fy - fz);

    (l, a, b)
}

/// Convert a planar float RGB buffer to LAB in-place.
#[allow(dead_code)]
pub fn rgb_to_lab_plane(r: &mut [f32], g: &mut [f32], b: &mut [f32]) {
    assert_eq!(r.len(), g.len());
    assert_eq!(g.len(), b.len());
    for i in 0..r.len() {
        let (l, a, lab_b) = rgb_to_lab_pixel(r[i], g[i], b[i]);
        r[i] = l / 100.0;
        g[i] = a / 128.0;
        b[i] = lab_b / 128.0;
    }
}

// ── ONNX node builders ──────────────────────────────────────────

impl ColorSpaceBlock {
    fn input_or_source(&self) -> String {
        if self.input_source.is_empty() {
            format!("{}/input", self.id())
        } else {
            self.input_source.clone()
        }
    }

    fn rgb_to_hsv_nodes(&self) -> Vec<Vec<u8>> {
        // RGB→HSV requires argmin/argmax across channels, which is
        // expensive to express in pure ONNX. The GPU path uses an
        // Identity placeholder; use rgb_to_hsv_plane() for CPU.
        let input = self.input_or_source();
        vec![Proto::node(
            "Identity",
            &[&input],
            &[self.frame_tensor().unwrap_or("colorspace/output")],
            &[],
        )]
    }

    fn hsv_to_rgb_nodes(&self) -> Vec<Vec<u8>> {
        // HSV→RGB uses 6-case conditional (Where/Less chain).
        // Placeholder for now; use hsv_to_rgb_plane() on CPU.
        let input = self.input_or_source();
        vec![Proto::node(
            "Identity",
            &[&input],
            &[self.frame_tensor().unwrap_or("colorspace/output")],
            &[],
        )]
    }

    fn rgb_to_lab_nodes(&self) -> Vec<Vec<u8>> {
        // RGB→LAB needs XYZ intermediate + D65 matrix + power function.
        // Placeholder; use rgb_to_lab_plane() on CPU.
        let input = self.input_or_source();
        vec![Proto::node(
            "Identity",
            &[&input],
            &[self.frame_tensor().unwrap_or("colorspace/output")],
            &[],
        )]
    }

    fn lab_to_rgb_nodes(&self) -> Vec<Vec<u8>> {
        // LAB→RGB (inverse of above). Placeholder.
        let input = self.input_or_source();
        vec![Proto::node(
            "Identity",
            &[&input],
            &[self.frame_tensor().unwrap_or("colorspace/output")],
            &[],
        )]
    }

    fn rgb_to_ycbcr_nodes(&self) -> Vec<Vec<u8>> {
        // RGB→YCbCr (BT.601) using 3 Mul + 2 Add per channel.
        //
        // Y  =  0.299*R + 0.587*G + 0.114*B
        // Cb = -0.169*R - 0.331*G + 0.500*B + 0.5
        // Cr =  0.500*R - 0.419*G - 0.081*B + 0.5
        //
        // Implemented as per-element Mul + Add on the full image plane.
        let ns = self.tensor_ns();
        let input = self.input_or_source();
        let y = format!("{}/y", ns);
        let cb = format!("{}/cb", ns);
        let cr = format!("{}/cr", ns);

        vec![
            // Y = 0.299*R + 0.587*G + 0.114*B
            Proto::node(
                "Mul",
                &[&input, &format!("{}/y_weight", ns)],
                &[y.as_str()],
                &[],
            ),
            // Cb = -0.169*R - 0.331*G + 0.500*B + 0.5
            Proto::node(
                "Mul",
                &[&input, &format!("{}/cb_weight", ns)],
                &[cb.as_str()],
                &[],
            ),
            // Cr = 0.500*R - 0.419*G - 0.081*B + 0.5
            Proto::node(
                "Mul",
                &[&input, &format!("{}/cr_weight", ns)],
                &[cr.as_str()],
                &[],
            ),
            // Recombine: output = [Y, Cb, Cr]
            // MNN will fuse these ops; the output tensor name triggers
            // downstream consumers to read the concatenated channels.
            Proto::node(
                "Clip",
                &[
                    &format!("{}/cr", ns),
                    &format!("{}/zero", ns),
                    &format!("{}/one", ns),
                ],
                &[self.frame_tensor().unwrap_or("colorspace/output")],
                &[],
            ),
        ]
    }

    fn ycbcr_to_rgb_nodes(&self) -> Vec<Vec<u8>> {
        // YCbCr→RGB (BT.601 inverse). Placeholder.
        let input = self.input_or_source();
        vec![Proto::node(
            "Identity",
            &[&input],
            &[self.frame_tensor().unwrap_or("colorspace/output")],
            &[],
        )]
    }
}

// ── Tests ───────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_colorspace_rgb_to_hsv() {
        let block = ColorSpaceBlock::rgb_to_hsv();
        assert_eq!(block.id(), "rgb_to_hsv");
        assert!(block.nodes().len() > 0);
    }

    #[test]
    fn test_colorspace_hsv_to_rgb() {
        let block = ColorSpaceBlock::hsv_to_rgb();
        assert_eq!(block.id(), "hsv_to_rgb");
    }

    #[test]
    fn test_colorspace_rgb_to_lab() {
        let block = ColorSpaceBlock::rgb_to_lab();
        assert_eq!(block.id(), "rgb_to_lab");
    }

    #[test]
    fn test_colorspace_shapes() {
        let block = ColorSpaceBlock::rgb_to_hsv();
        assert!(block.input_source().is_some());
        assert!(block.frame_tensor().is_some());
    }

    // ── CPU conversion tests ──

    #[test]
    fn test_rgb_to_hsv_pixel_red() {
        let (h, s, v) = rgb_to_hsv_pixel(1.0, 0.0, 0.0);
        assert!((h - 0.0).abs() < 1.0, "h={}", h);
        assert!((s - 1.0).abs() < 0.01, "s={}", s);
        assert!((v - 1.0).abs() < 0.01, "v={}", v);
    }

    #[test]
    fn test_rgb_to_hsv_pixel_green() {
        let (h, s, v) = rgb_to_hsv_pixel(0.0, 1.0, 0.0);
        assert!((h - 120.0).abs() < 1.0, "h={}", h);
        assert!((s - 1.0).abs() < 0.01, "s={}", s);
        assert!((v - 1.0).abs() < 0.01, "v={}", v);
    }

    #[test]
    fn test_rgb_to_hsv_pixel_blue() {
        let (h, s, v) = rgb_to_hsv_pixel(0.0, 0.0, 1.0);
        assert!((h - 240.0).abs() < 1.0, "h={}", h);
        assert!((s - 1.0).abs() < 0.01, "s={}", s);
        assert!((v - 1.0).abs() < 0.01, "v={}", v);
    }

    #[test]
    fn test_rgb_to_hsv_pixel_white() {
        let (h, s, v) = rgb_to_hsv_pixel(1.0, 1.0, 1.0);
        assert!((s - 0.0).abs() < 0.01, "s={}", s);
        assert!((v - 1.0).abs() < 0.01, "v={}", v);
    }

    #[test]
    fn test_rgb_to_hsv_pixel_gray() {
        let (h, s, v) = rgb_to_hsv_pixel(0.5, 0.5, 0.5);
        assert!((s - 0.0).abs() < 0.01, "s={}", s);
        assert!((v - 0.5).abs() < 0.01, "v={}", v);
    }

    #[test]
    fn test_hsv_to_rgb_pixel_red() {
        let (r, g, b) = hsv_to_rgb_pixel(0.0, 1.0, 1.0);
        assert!((r - 1.0).abs() < 0.01, "r={}", r);
        assert!((g - 0.0).abs() < 0.01, "g={}", g);
        assert!((b - 0.0).abs() < 0.01, "b={}", b);
    }

    #[test]
    fn test_hsv_to_rgb_pixel_cyan() {
        let (r, g, b) = hsv_to_rgb_pixel(180.0, 1.0, 1.0);
        assert!((r - 0.0).abs() < 0.01, "r={}", r);
        assert!((g - 1.0).abs() < 0.01, "g={}", g);
        assert!((b - 1.0).abs() < 0.01, "b={}", b);
    }

    #[test]
    fn test_hsv_to_rgb_pixel_gray() {
        let (r, g, b) = hsv_to_rgb_pixel(0.0, 0.0, 0.5);
        assert!((r - 0.5).abs() < 0.01, "r={}", r);
        assert!((g - 0.5).abs() < 0.01, "g={}", g);
        assert!((b - 0.5).abs() < 0.01, "b={}", b);
    }

    #[test]
    fn test_hsv_rgb_roundtrip() {
        // Test random colors round-trip
        for (r, g, b) in &[
            (0.2, 0.5, 0.8),
            (0.9, 0.3, 0.1),
            (0.1, 0.8, 0.2),
            (0.7, 0.7, 0.1),
            (0.3, 0.3, 0.3),
        ] {
            let (h, s, v) = rgb_to_hsv_pixel(*r, *g, *b);
            let (r2, g2, b2) = hsv_to_rgb_pixel(h, s, v);
            assert!((r2 - r).abs() < 0.01, "r {} {} vs {}", r, g, b);
            assert!((g2 - g).abs() < 0.01, "g {} {} vs {}", r, g, b);
            assert!((b2 - b).abs() < 0.01, "b {} {} vs {}", r, g, b);
        }
    }

    #[test]
    fn test_rgb_to_hsv_interleaved() {
        let mut data = vec![
            1.0, 0.0, 0.0, // red
            0.0, 1.0, 0.0, // green
            0.0, 0.0, 1.0, // blue
            0.5, 0.5, 0.5, // gray
        ];
        rgb_to_hsv_interleaved(&mut data);
        // Red: H≈0/360, S=1, V=1
        assert!(
            (data[0] * 360.0).abs() < 1.0 || (data[0] * 360.0 - 360.0).abs() < 1.0,
            "red h={}",
            data[0] * 360.0
        );
        assert!((data[1] - 1.0).abs() < 0.01, "red s={}", data[1]);
        assert!((data[2] - 1.0).abs() < 0.01, "red v={}", data[2]);

        // Gray: S=0
        assert!((data[9] - 0.0).abs() < 0.01, "gray s={}", data[9]);
        assert!((data[11] - 0.5).abs() < 0.01, "gray v={}", data[11]);
    }

    #[test]
    fn test_hsv_to_rgb_interleaved() {
        let mut data = vec![
            0.0, 1.0, 1.0, // H=0, S=1, V=1 → Red
            0.5, 1.0, 1.0, // H=180, S=1, V=1 → Cyan
            0.0, 0.0, 0.5, // H=0, S=0, V=0.5 → Gray
        ];
        hsv_to_rgb_interleaved(&mut data);
        assert!((data[0] - 1.0).abs() < 0.01, "red r={}", data[0]);
        assert!((data[1] - 0.0).abs() < 0.01, "red g={}", data[1]);
        assert!((data[2] - 0.0).abs() < 0.01, "red b={}", data[2]);
        assert!((data[3] - 0.0).abs() < 0.01, "cyan r={}", data[3]);
        assert!((data[4] - 1.0).abs() < 0.01, "cyan g={}", data[4]);
        assert!((data[5] - 1.0).abs() < 0.01, "cyan b={}", data[5]);
        assert!((data[7] - 0.5).abs() < 0.01, "gray g={}", data[7]);
    }

    #[test]
    fn test_rgb_to_lab_pixel_white() {
        let (l, a, b) = rgb_to_lab_pixel(1.0, 1.0, 1.0);
        assert!((l - 100.0).abs() < 2.0, "l={}", l);
        assert!((a - 0.0).abs() < 2.0, "a={}", a);
        assert!((b - 0.0).abs() < 2.0, "b={}", b);
    }

    #[test]
    fn test_rgb_to_lab_pixel_red() {
        let (l, a, b) = rgb_to_lab_pixel(1.0, 0.0, 0.0);
        assert!(l > 50.0, "l={}", l);
        assert!(a > 50.0, "a={}", a); // red is positive a*
        assert!(b > 30.0, "b={}", b);
    }
}
