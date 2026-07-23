//! Synthetic Bayer pattern generators for benchmarks and stress tests.
//!
//! All functions return packed LE u16 bytes ready for `ProcessParams::new()`.
//! Patterns exercise the full ISP pipeline (demosaic, CCM, tone, sharpen, etc.)
//! with high spatial frequency and wide dynamic range.

/// HSV hue (0..6) → linear (r,g,b) each in 0..1.
/// 0=red, 1=yellow, 2=green, 3=cyan, 4=blue, 5=magenta, 6=red.
fn hue_to_rgb(hue: f32) -> (f32, f32, f32) {
    let h = hue % 6.0;
    let f = h - h.floor();
    let q = 1.0 - f;
    let t = f;
    match h.floor() as i32 {
        0 => (1.0, t, 0.0),
        1 => (q, 1.0, 0.0),
        2 => (0.0, 1.0, t),
        3 => (0.0, q, 1.0),
        4 => (t, 0.0, 1.0),
        5 => (1.0, 0.0, q),
        _ => (1.0, 0.0, 0.0),
    }
}

/// Generate a constant-value Bayer frame.
///
/// All pixels get the same u16 value per Bayer site:
/// R=2000, G=4000, B=6000 (RGGB pattern).
pub fn bayer_constant(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let val: u16 = match (y & 1, x & 1) {
                (0, 0) => 2000,
                (0, 1) => 4000,
                (1, 0) => 4000,
                (1, 1) => 6000,
                _ => 0,
            };
            buf.extend_from_slice(&val.to_le_bytes());
        }
    }
    buf
}

/// Generate a rainbow hue-sweep Bayer frame.
///
/// Hue rotates across x (red → yellow → green → cyan → blue → magenta → red).
/// A vertical sine envelope adds diffuse brightness variation.
/// High spatial frequency (XOR bit pattern) prevents MNN graph collapse.
///
/// Peak value ≈ 12000 (within 14-bit sensor range).
pub fn bayer_rainbow(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let hue = (x as f32 / width as f32) * 6.0;
            let (r0, g0, b0) = hue_to_rgb(hue);

            let vy = (y as f32 / height as f32) * std::f32::consts::PI;
            let envelope = 0.4 + 0.6 * (0.5 + 0.5 * vy.sin());

            let mx = ((x ^ y) & 0x1F) as f32 / 31.0;
            let detail = 0.6 + 0.4 * mx;

            let scale = envelope * detail * 12000.0;
            let val = match (y & 1, x & 1) {
                (0, 0) => (r0 * scale).clamp(0.0, 16383.0),
                (0, 1) => ((r0 + g0) * 0.5 * scale).clamp(0.0, 16383.0),
                (1, 0) => ((g0 + b0) * 0.5 * scale).clamp(0.0, 16383.0),
                (1, 1) => (b0 * scale).clamp(0.0, 16383.0),
                _ => 0.0,
            };
            buf.extend_from_slice(&(val as u16).to_le_bytes());
        }
    }
    buf
}

/// Generate a maze-like high-frequency Bayer frame.
///
/// Concentric XOR patterns create dense edges that stress demosaic,
/// sharpening, and edge-enhancement blocks.  Brightness ramps diagonally.
///
/// Peak value ≈ 14000 (14-bit range).
pub fn bayer_maze(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            // Diagonal gradient
            let diag = ((x + y) as f32 / (width + height) as f32).clamp(0.0, 1.0);

            // Concentric ring pattern (alternating bright/dark)
            let dx = x as f32 - width as f32 * 0.5;
            let dy = y as f32 - height as f32 * 0.5;
            let ring = ((dx * dx + dy * dy).sqrt() * 0.5).sin();

            // XOR checker for high-freq detail
            let checker = ((x ^ y) & 0x0F) as f32 / 15.0;

            let base = (0.3 + 0.7 * diag) * (0.5 + 0.5 * ring) * (0.7 + 0.3 * checker);
            let scale = base * 14000.0;

            // Assign per Bayer site with slight color tint
            let val = match (y & 1, x & 1) {
                (0, 0) => (scale * 1.1).clamp(0.0, 16383.0), // R warm
                (0, 1) => scale.clamp(0.0, 16383.0),         // G
                (1, 0) => scale.clamp(0.0, 16383.0),         // G
                (1, 1) => (scale * 0.9).clamp(0.0, 16383.0), // B cool
                _ => 0.0,
            };
            buf.extend_from_slice(&(val as u16).to_le_bytes());
        }
    }
    buf
}

/// Generate a color-diffuse Bayer frame.
///
/// Smooth gradients across both axes with independent R/G/B patterns
/// simulating a color chart.  Multiple overlapping sine waves create
/// smooth diffuse color regions that exercise CCM and tone mapping.
///
/// Peak value ≈ 13000 (14-bit range).
pub fn bayer_color_diffuse(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let nx = x as f32 / width as f32;
            let ny = y as f32 / height as f32;

            // Independent frequency per channel (floor at 0.05 so no pixel is zero)
            let r_wave = 0.05 + 0.95 * (nx * 3.0 + ny * 1.5).sin() * 0.5 + 0.5;
            let g_wave = 0.05 + 0.95 * (nx * 2.0 - ny * 2.5 + 1.0).sin() * 0.5 + 0.5;
            let b_wave = 0.05 + 0.95 * (nx * 1.0 + ny * 4.0 + 2.0).sin() * 0.5 + 0.5;

            // Smooth diffuse blending
            let blend = 0.4 + 0.6 * (nx * std::f32::consts::PI).sin().abs();
            let scale = blend * 13000.0;

            let val = match (y & 1, x & 1) {
                (0, 0) => (r_wave * scale).clamp(0.0, 16383.0),
                (0, 1) => ((r_wave + g_wave) * 0.5 * scale).clamp(0.0, 16383.0),
                (1, 0) => ((g_wave + b_wave) * 0.5 * scale).clamp(0.0, 16383.0),
                (1, 1) => (b_wave * scale).clamp(0.0, 16383.0),
                _ => 0.0,
            };
            buf.extend_from_slice(&(val as u16).to_le_bytes());
        }
    }
    buf
}

/// Combined stress pattern: rainbow + maze + diffuse.
///
/// Overlays all three patterns for maximum spatial frequency and dynamic
/// range.  Best for stress testing the full pipeline end-to-end.
///
/// Peak value ≈ 16000 (near 14-bit max).
pub fn bayer_stress(width: u32, height: u32) -> Vec<u8> {
    let mut buf = Vec::with_capacity((width * height * 2) as usize);
    for y in 0..height {
        for x in 0..width {
            let nx = x as f32 / width as f32;
            let ny = y as f32 / height as f32;

            // Rainbow hue sweep
            let hue = nx * 6.0;
            let (r0, g0, b0) = hue_to_rgb(hue);

            // Vertical envelope (diffuse)
            let vy = ny * std::f32::consts::PI;
            let envelope = 0.4 + 0.6 * (0.5 + 0.5 * vy.sin());

            // Maze XOR detail
            let mx = ((x ^ y) & 0x1F) as f32 / 31.0;
            let maze = 0.6 + 0.4 * mx;

            // Concentric ring (additional high-freq)
            let dx = x as f32 - width as f32 * 0.5;
            let dy = y as f32 - height as f32 * 0.5;
            let ring = 0.8 + 0.2 * ((dx * dx + dy * dy).sqrt() * 0.3).sin();

            // Diagonal gradient
            let diag = 0.7 + 0.3 * ((nx + ny) * std::f32::consts::PI).sin();

            let scale = envelope * maze * ring * diag * 14000.0 + 100.0; // floor 100 to avoid zero
                                                                         // Floor each channel at 0.1 so Bayer sites never reach exactly 0.0
            let r1 = r0.max(0.1);
            let g1 = ((r0 + g0) * 0.5).max(0.1);
            let g2 = ((g0 + b0) * 0.5).max(0.1);
            let b1 = b0.max(0.1);
            let val = match (y & 1, x & 1) {
                (0, 0) => (r1 * scale).clamp(0.0, 16383.0),
                (0, 1) => (g1 * scale).clamp(0.0, 16383.0),
                (1, 0) => (g2 * scale).clamp(0.0, 16383.0),
                (1, 1) => (b1 * scale).clamp(0.0, 16383.0),
                _ => 0.0,
            };
            buf.extend_from_slice(&(val as u16).to_le_bytes());
        }
    }
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bayer_constant_sizes() {
        let buf = bayer_constant(128, 72);
        assert_eq!(buf.len(), 128 * 72 * 2);
        // First pixel (0,0) = R site = 2000
        let v = u16::from_le_bytes([buf[0], buf[1]]);
        assert_eq!(v, 2000);
    }

    #[test]
    fn test_bayer_rainbow_peak_range() {
        let buf = bayer_rainbow(1920, 1080);
        assert_eq!(buf.len(), 1920 * 1080 * 2);
        let max_val = buf
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .max()
            .unwrap();
        // Peak should be in upper range but below 16-bit max
        assert!(max_val > 8000, "max_val={max_val}, expected > 8000");
        assert!(max_val <= 16383, "max_val={max_val}, expected <= 16383");
    }

    #[test]
    fn test_bayer_maze_has_variation() {
        let buf = bayer_maze(256, 256);
        let vals: Vec<u16> = buf
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        let min = *vals.iter().min().unwrap();
        let max = *vals.iter().max().unwrap();
        assert!(max - min > 1000, "maze range too narrow: {min}..{max}");
    }

    #[test]
    fn test_bayer_color_diffuse_nonzero() {
        let buf = bayer_color_diffuse(640, 480);
        assert_eq!(buf.len(), 640 * 480 * 2);
        // No pixel should be zero (diffuse patterns have non-zero min)
        let min_val = buf
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .min()
            .unwrap();
        assert!(min_val > 0, "min_val=0, expected nonzero");
    }

    #[test]
    fn test_bayer_stress_full_range() {
        let buf = bayer_stress(320, 240);
        let vals: Vec<u16> = buf
            .chunks_exact(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        let min = *vals.iter().min().unwrap();
        let max = *vals.iter().max().unwrap();
        assert!(min > 0, "stress pattern has zero pixels");
        assert!(max > 10000, "stress pattern max too low: {max}");
        assert!(max <= 16383, "stress pattern exceeds 14-bit: {max}");
    }
}
