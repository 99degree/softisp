//! Demosaic — Malvar (2004) gradient-based directional demosaicing.
//!
//! Ported from `cpu.rs` to reduce file size. Contains all demosaic-related
//! functions: Malvar demosaic, interpolate helpers, and Bayer-to-quad conversion.

/// Malvar gradient-based demosaic: raw CFA → RGB.
///
/// Implements the algorithm from:
///   Malvar, He, and Cutler, "High-Quality Linear Interpolation for
///   Demosaicing of Bayer-Patterned Color Images" (2004).
///
/// Works on BGGR-packed float data (single channel of size H×W).
/// Output is interleaved RGB float (H×W×3).
pub(crate) fn demosaic_malvar(
    cfa: &[f32],
    width: usize,
    height: usize,
    _awb: Option<&[f32; 3]>,
) -> Vec<f32> {
    let total = width * height;
    let mut rgb = vec![0.0f32; total * 3];

    // Phase 1: Interpolate green at red and blue pixels
    for y in 0..height {
        for x in 0..width {
            let is_red = (y % 2 == 1) && (x % 2 == 1);
            let is_blue = (y % 2 == 0) && (x % 2 == 0);
            let is_green_row = y % 2 == 0;
            let is_green_col = x % 2 == 1;

            let cfa_val = cfa[y * width + x];

            if is_red {
                let g = interpolate_green_at_rb(cfa, width, height, x, y, false);
                set_rgb(&mut rgb, x, y, width, cfa_val, g, 0.0);
            } else if is_blue {
                let g = interpolate_green_at_rb(cfa, width, height, x, y, true);
                set_rgb(&mut rgb, x, y, width, 0.0, g, cfa_val);
            } else {
                // Green pixel (Gr or Gb)
                if is_green_row && is_green_col {
                    // Gr (green in red row)
                    let r = interpolate_red_at_green(cfa, width, height, x, y);
                    let b = interpolate_blue_at_green(cfa, width, height, x, y);
                    set_rgb(&mut rgb, x, y, width, r, cfa_val, b);
                } else {
                    // Gb (green in blue row)
                    let r = interpolate_red_at_green(cfa, width, height, x, y);
                    let b = interpolate_blue_at_green(cfa, width, height, x, y);
                    set_rgb(&mut rgb, x, y, width, r, cfa_val, b);
                }
            }
        }
    }

    // Phase 2: Interpolate R and B at green pixels (already done above)
    // Phase 3: Interpolate R at B and B at R
    for y in 0..height {
        for x in 0..width {
            let is_red = (y % 2 == 1) && (x % 2 == 1);
            let is_blue = (y % 2 == 0) && (x % 2 == 0);
            let idx = (y * width + x) * 3;
            let g = rgb[idx + 1];

            if is_red {
                let b = interpolate_blue_at_red(cfa, width, height, x, y, g);
                rgb[idx + 2] = b;
            } else if is_blue {
                let r = interpolate_red_at_blue(cfa, width, height, x, y, g);
                rgb[idx] = r;
            }
        }
    }

    rgb
}

/// Interpolate green at a red or blue pixel using directional gradients.
fn interpolate_green_at_rb(
    cfa: &[f32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    _is_blue: bool,
) -> f32 {
    // Compute horizontal and vertical gradients
    let left = if x > 0 {
        cfa[y * width + (x - 1)]
    } else {
        cfa[y * width + x]
    };
    let right = if x + 1 < width {
        cfa[y * width + (x + 1)]
    } else {
        cfa[y * width + x]
    };
    let up = if y > 0 {
        cfa[(y - 1) * width + x]
    } else {
        cfa[y * width + x]
    };
    let down = if y + 1 < height {
        cfa[(y + 1) * width + x]
    } else {
        cfa[y * width + x]
    };

    let center = cfa[y * width + x];
    let dh = (center - left).abs() + (center - right).abs();
    let dv = (center - up).abs() + (center - down).abs();

    if dh < dv {
        // Horizontal interpolation
        let g2 = if x > 0 {
            cfa[y * width + (x - 1)]
        } else {
            cfa[y * width + x]
        };
        let g4 = if x + 1 < width {
            cfa[y * width + (x + 1)]
        } else {
            cfa[y * width + x]
        };
        (g2 + g4) / 2.0
    } else if dv < dh {
        // Vertical interpolation
        let g1 = if y > 0 {
            cfa[(y - 1) * width + x]
        } else {
            cfa[y * width + x]
        };
        let g3 = if y + 1 < height {
            cfa[(y + 1) * width + x]
        } else {
            cfa[y * width + x]
        };
        (g1 + g3) / 2.0
    } else {
        // Equal gradients: average all four
        let g1 = if y > 0 {
            cfa[(y - 1) * width + x]
        } else {
            cfa[y * width + x]
        };
        let g2 = if x > 0 {
            cfa[y * width + (x - 1)]
        } else {
            cfa[y * width + x]
        };
        let g3 = if y + 1 < height {
            cfa[(y + 1) * width + x]
        } else {
            cfa[y * width + x]
        };
        let g4 = if x + 1 < width {
            cfa[y * width + (x + 1)]
        } else {
            cfa[y * width + x]
        };
        (g1 + g2 + g3 + g4) / 4.0
    }
}

/// Interpolate red at a blue pixel using color differences with the green channel.
fn interpolate_red_at_blue(
    cfa: &[f32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    _g: f32,
) -> f32 {
    // R at B: average of 4 diagonal neighbors
    let mut sum = 0.0f32;
    let mut cnt = 0u32;
    let neighbors = [(-1, -1), (-1, 1), (1, -1), (1, 1)];
    for &(dx, dy) in &neighbors {
        let nx = x as isize + dx;
        let ny = y as isize + dy;
        if nx >= 0 && nx < width as isize && ny >= 0 && ny < height as isize {
            sum += cfa[ny as usize * width + nx as usize];
            cnt += 1;
        }
    }
    if cnt > 0 {
        sum / cnt as f32
    } else {
        0.0
    }
}

/// Interpolate blue at a red pixel using color differences with the green channel.
fn interpolate_blue_at_red(
    cfa: &[f32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
    _g: f32,
) -> f32 {
    // B at R: average of 4 diagonal neighbors
    let mut sum = 0.0f32;
    let mut cnt = 0u32;
    let neighbors = [(-1, -1), (-1, 1), (1, -1), (1, 1)];
    for &(dx, dy) in &neighbors {
        let nx = x as isize + dx;
        let ny = y as isize + dy;
        if nx >= 0 && nx < width as isize && ny >= 0 && ny < height as isize {
            sum += cfa[ny as usize * width + nx as usize];
            cnt += 1;
        }
    }
    if cnt > 0 {
        sum / cnt as f32
    } else {
        0.0
    }
}

/// Interpolate red at a green pixel.
fn interpolate_red_at_green(
    cfa: &[f32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> f32 {
    // R at G in BGGR: vertical neighbors for Gr (even row, odd col)
    // horizontal neighbors for Gb (odd row, even col)
    let is_gr = y.is_multiple_of(2) && x % 2 == 1; // Gr: green in red row
    let mut sum = 0.0f32;
    let mut cnt = 0u32;

    if is_gr {
        // Vertical neighbors
        if y > 0 {
            sum += cfa[(y - 1) * width + x];
            cnt += 1;
        }
        if y + 1 < height {
            sum += cfa[(y + 1) * width + x];
            cnt += 1;
        }
    } else {
        // Horizontal neighbors
        if x > 0 {
            sum += cfa[y * width + (x - 1)];
            cnt += 1;
        }
        if x + 1 < width {
            sum += cfa[y * width + (x + 1)];
            cnt += 1;
        }
    }

    if cnt > 0 {
        sum / cnt as f32
    } else {
        0.0
    }
}

/// Interpolate blue at a green pixel.
fn interpolate_blue_at_green(
    cfa: &[f32],
    width: usize,
    height: usize,
    x: usize,
    y: usize,
) -> f32 {
    // B at G in BGGR: horizontal neighbors for Gr, vertical for Gb
    let is_gr = y.is_multiple_of(2) && x % 2 == 1;
    let mut sum = 0.0f32;
    let mut cnt = 0u32;

    if is_gr {
        // Horizontal neighbors
        if x > 0 {
            sum += cfa[y * width + (x - 1)];
            cnt += 1;
        }
        if x + 1 < width {
            sum += cfa[y * width + (x + 1)];
            cnt += 1;
        }
    } else {
        // Vertical neighbors
        if y > 0 {
            sum += cfa[(y - 1) * width + x];
            cnt += 1;
        }
        if y + 1 < height {
            sum += cfa[(y + 1) * width + x];
            cnt += 1;
        }
    }

    if cnt > 0 {
        sum / cnt as f32
    } else {
        0.0
    }
}

/// Set an RGB pixel in the output buffer.
fn set_rgb(rgb: &mut [f32], x: usize, y: usize, width: usize, r: f32, g: f32, b: f32) {
    let idx = (y * width + x) * 3;
    if idx + 2 < rgb.len() {
        rgb[idx] = r;
        rgb[idx + 1] = g;
        rgb[idx + 2] = b;
    }
}

/// Split flat raw Bayer data into 4-channel quad format for calibration stats.
///
/// Takes a flat `[H*W]` float array of raw Bayer pixels (RGGB pattern)
/// and produces a flat `[4 * ((H+1)/2) * ((W+1)/2)]` array where each of
/// the 4 channels represents one Bayer quadrant.
pub(crate) fn bayer_to_quads(bayer: &[f32], w: usize, h: usize) -> Vec<f32> {
    let qh = h.div_ceil(2);
    let qw = w.div_ceil(2);
    let mut quads = vec![0.0f32; 4 * qh * qw];

    for y in 0..h {
        let qy = y / 2;
        for x in 0..w {
            let qx = x / 2;
            let chan = ((y & 1) << 1) | (x & 1); // 0=TL=R, 1=TR=Gr, 2=BL=Gb, 3=BR=B
            let src = y * w + x;
            let dst = chan * (qh * qw) + qy * qw + qx;
            if src < bayer.len() && dst < quads.len() {
                quads[dst] = bayer[src];
            }
        }
    }

    quads
}
