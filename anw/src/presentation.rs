//! Presentation of processed frames into an `ANativeWindow`.
//!
//! The window buffer is locked, filled with the ISP's BGRA output, and posted.
//! Scaling is nearest-neighbour: preview is a live view, and a nearest-neighbour
//! blit costs a fraction of a resample while staying trivially correct for
//! arbitrary aspect ratios.

use android_activity::ndk::hardware_buffer_format::HardwareBufferFormat;
use android_activity::ndk::native_window::NativeWindow;

/// The NDK's `ARect`, which is what `NativeWindow::lock` expects for its
/// dirty-bounds argument.
type Rect = android_activity::ndk_sys::ARect;

/// Bytes per pixel in the `R8G8B8A8_UNORM` buffer the window is configured for.
const BYTES_PER_PIXEL: usize = 4;

/// A window configured for RGBA8888 buffers.
pub struct WindowTarget<'a> {
    window: &'a NativeWindow,
}

/// Where a source frame lands in the window buffer.
///
/// Both axes used to be mapped independently (`x * src_w / dst_w`,
/// `y * src_h / dst_h`), which stretches any frame whose aspect differs from
/// the surface's. A 4:3 sensor frame on a 1080x2264 portrait surface came out
/// 2.8x too tall. The frame now keeps its aspect and the leftover area is
/// painted black — the same letterbox a camera app shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Viewport {
    left: usize,
    top: usize,
    width: usize,
    height: usize,
}

impl Viewport {
    /// Largest `src_w`×`src_h`-shaped rectangle that fits in `dst_w`×`dst_h`.
    fn fit(dst_w: usize, dst_h: usize, src_w: usize, src_h: usize) -> Self {
        if dst_w == 0 || dst_h == 0 || src_w == 0 || src_h == 0 {
            return Self {
                left: 0,
                top: 0,
                width: 0,
                height: 0,
            };
        }
        // Compare as a ratio to avoid overflowing on huge dimensions.
        let width = if dst_w * src_h <= dst_h * src_w {
            // Surface is narrower than the frame: full width, reduced height.
            dst_w
        } else {
            // Surface is wider: full height, reduced width.
            dst_h * src_w / src_h
        };
        let height = if width == dst_w {
            dst_w * src_h / src_w
        } else {
            dst_h
        };
        let width = width.max(1).min(dst_w);
        let height = height.max(1).min(dst_h);
        Self {
            left: (dst_w - width) / 2,
            top: (dst_h - height) / 2,
            width,
            height,
        }
    }

    fn right(&self) -> usize {
        self.left + self.width
    }

    fn bottom(&self) -> usize {
        self.top + self.height
    }

    fn area(&self) -> usize {
        self.width * self.height
    }

    fn empty(&self) -> bool {
        self.width == 0 || self.height == 0
    }
}

impl<'a> WindowTarget<'a> {
    /// Reject geometries the platform cannot allocate buffers for.
    fn geometry_check(width: i32, height: i32) -> Result<(), String> {
        if width <= 0 || height <= 0 {
            return Err(format!("invalid window geometry {width}x{height}"));
        }
        Ok(())
    }

    /// Configure `window` to provide `width`×`height` RGBA8888 buffers.
    pub fn new(window: &'a NativeWindow, width: i32, height: i32) -> Result<Self, String> {
        Self::geometry_check(width, height)?;
        window
            .set_buffers_geometry(width, height, Some(HardwareBufferFormat::R8G8B8A8_UNORM))
            .map_err(|e| format!("set_buffers_geometry({width}x{height}) -> {e}"))?;
        Ok(Self { window })
    }

    /// Blit planar RGB (`[1,3,H,W]` f32 in `[0,1]`) into the window.
    ///
    /// The graph emits planar RGB because every u8 display format needs a
    /// `Conv(1x1)` to permute channels, and on Vulkan that Conv's kernel is
    /// zero-filled (the backend reads weights from the operator, not from a
    /// graph input) — the picture comes back black with a 255 alpha plane.
    /// Converting here costs one pass over the frame and cannot be wrong.
    ///
    /// Returns the number of destination pixels written.
    pub fn present_planar_rgb(&self, src: &[f32], src_w: u32, src_h: u32) -> Result<usize, String> {
        let src_w = src_w as usize;
        let src_h = src_h as usize;
        if src_w == 0 || src_h == 0 {
            return Err("source frame has a zero dimension".into());
        }
        let plane = src_w * src_h;
        if src.len() < plane * 3 {
            return Err(format!(
                "planar RGB too small: {} f32 for {src_w}x{src_h}, need {}",
                src.len(),
                plane * 3
            ));
        }
        let (r_plane, rest) = src.split_at(plane);
        let (g_plane, rest) = rest.split_at(plane);
        let b_plane = &rest[..plane];

        let mut dirty = Rect {
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
        };
        let mut guard = self
            .window
            .lock(Some(&mut dirty))
            .map_err(|e| format!("ANativeWindow_lock -> {e}"))?;

        let dst_w = guard.width();
        let dst_h = guard.height();
        let stride = guard.stride();
        if dst_w == 0 || dst_h == 0 || stride < dst_w {
            return Err(format!(
                "window buffer unusable: {dst_w}x{dst_h} stride {stride}"
            ));
        }
        let bits = guard.bits() as *mut u8;
        if bits.is_null() {
            return Err("window buffer has a null pixel pointer".into());
        }

        let viewport = Viewport::fit(dst_w, dst_h, src_w, src_h);
        if viewport.empty() {
            return Ok(0);
        }
        let scale = 255.0f32;

        for y in viewport.top..viewport.bottom() {
            let sy = (y - viewport.top) * src_h / viewport.height;
            let src_row = sy * src_w;
            let dst_row = unsafe { bits.add(y * stride * BYTES_PER_PIXEL) };
            for x in viewport.left..viewport.right() {
                let sx = (x - viewport.left) * src_w / viewport.width;
                let p = src_row + sx;
                // The window format is R8G8B8A8: store R,G,B,A in that order.
                let r = (r_plane[p] * scale).clamp(0.0, 255.0) as u8;
                let g = (g_plane[p] * scale).clamp(0.0, 255.0) as u8;
                let b = (b_plane[p] * scale).clamp(0.0, 255.0) as u8;
                let d = x * BYTES_PER_PIXEL;
                unsafe {
                    *dst_row.add(d) = r;
                    *dst_row.add(d + 1) = g;
                    *dst_row.add(d + 2) = b;
                    *dst_row.add(d + 3) = 255;
                }
            }
        }

        // Bars: the frame keeps its own aspect, so whatever the viewport does
        // not cover has to be painted or it shows stale buffer contents.
        let bars = dst_w * dst_h - viewport.area();
        if bars > 0 {
            self.paint_bars(bits, stride, dst_w, dst_h, viewport);
        }
        Ok(viewport.area())
    }

    /// Blit `src` (BGRA, `src_w`×`src_h`) into the window, scaling to fit.
    ///
    /// Returns the number of destination pixels written.
    pub fn present(&self, src: &[u8], src_w: u32, src_h: u32) -> Result<usize, String> {
        let src_w = src_w as usize;
        let src_h = src_h as usize;
        if src_w == 0 || src_h == 0 {
            return Err("source frame has a zero dimension".into());
        }
        let needed = src_w * src_h * BYTES_PER_PIXEL;
        if src.len() < needed {
            return Err(format!(
                "source frame too small: {} bytes for {src_w}x{src_h} BGRA, need {needed}",
                src.len()
            ));
        }

        // A resize may race this call; the dirty bounds let the compositor
        // redraw only the region that changed.
        let mut dirty = Rect {
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
        };
        let mut guard = self
            .window
            .lock(Some(&mut dirty))
            .map_err(|e| format!("ANativeWindow_lock -> {e}"))?;

        let dst_w = guard.width();
        let dst_h = guard.height();
        let stride = guard.stride();
        if dst_w == 0 || dst_h == 0 || stride < dst_w {
            return Err(format!(
                "window buffer unusable: {dst_w}x{dst_h} stride {stride}"
            ));
        }
        let bits = guard.bits() as *mut u8;
        if bits.is_null() {
            return Err("window buffer has a null pixel pointer".into());
        }

        let viewport = Viewport::fit(dst_w, dst_h, src_w, src_h);
        if viewport.empty() {
            return Ok(0);
        }

        for y in viewport.top..viewport.bottom() {
            let sy = (y - viewport.top) * src_h / viewport.height;
            let src_row = sy * src_w * BYTES_PER_PIXEL;
            let dst_row = unsafe { bits.add(y * stride * BYTES_PER_PIXEL) };
            for x in viewport.left..viewport.right() {
                let sx = (x - viewport.left) * src_w / viewport.width;
                let s = src_row + sx * BYTES_PER_PIXEL;
                // The ISP emits BGRA; the window format is R8G8B8A8.
                let d = x * BYTES_PER_PIXEL;
                unsafe {
                    *dst_row.add(d) = src[s + 2];
                    *dst_row.add(d + 1) = src[s + 1];
                    *dst_row.add(d + 2) = src[s];
                    *dst_row.add(d + 3) = 255;
                }
            }
        }

        let bars = dst_w * dst_h - viewport.area();
        if bars > 0 {
            self.paint_bars(bits, stride, dst_w, dst_h, viewport);
        }
        Ok(viewport.area())
    }

    /// Fill the area outside `viewport` with opaque black.
    ///
    /// The window buffer is reused across frames, so an uncovered column would
    /// otherwise keep showing whatever the previous frame put there.
    fn paint_bars(
        &self,
        bits: *mut u8,
        stride: usize,
        dst_w: usize,
        dst_h: usize,
        viewport: Viewport,
    ) {
        let row = |y: usize| unsafe { bits.add(y * stride * BYTES_PER_PIXEL) };
        for y in 0..dst_h {
            let inside = y >= viewport.top && y < viewport.bottom();
            if !inside {
                // Whole row is a bar.
                let dst_row = row(y);
                unsafe {
                    std::ptr::write_bytes(dst_row, 0, dst_w * BYTES_PER_PIXEL);
                }
                continue;
            }
            // Side bars only.
            if viewport.left > 0 {
                unsafe {
                    std::ptr::write_bytes(row(y), 0, viewport.left * BYTES_PER_PIXEL);
                }
            }
            let right_bar = dst_w - viewport.right();
            if right_bar > 0 {
                unsafe {
                    std::ptr::write_bytes(
                        row(y).add(viewport.right() * BYTES_PER_PIXEL),
                        0,
                        right_bar * BYTES_PER_PIXEL,
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometry_rejects_non_positive_edges() {
        assert!(WindowTarget::geometry_check(0, 100).is_err());
        assert!(WindowTarget::geometry_check(100, 0).is_err());
        assert!(WindowTarget::geometry_check(100, -1).is_err());
    }

    #[test]
    fn geometry_accepts_positive_edges() {
        assert!(WindowTarget::geometry_check(1, 1).is_ok());
        assert!(WindowTarget::geometry_check(1920, 1080).is_ok());
    }
}

#[cfg(test)]
mod viewport_tests {
    use super::Viewport;

    #[test]
    fn landscape_frame_on_portrait_surface_bars_top_and_bottom() {
        // The device case: 1296x970 into 1080x2264. Aspect must be preserved,
        // so the frame keeps full width and is centred vertically. Filling the
        // surface instead is the 2.8x vertical stretch this fixes.
        let v = Viewport::fit(1080, 2264, 1296, 970);
        assert_eq!(v.left, 0, "full width, no side bars");
        assert_eq!(v.width, 1080);
        assert_eq!(v.height, 1080 * 970 / 1296);
        assert_eq!(v.top, (2264 - v.height) / 2);
        assert_eq!(v.right(), 1080);
        assert_eq!(v.bottom(), v.top + v.height);
    }

    #[test]
    fn aspect_ratio_is_preserved() {
        for (dw, dh, sw, sh) in [
            (1080usize, 2264usize, 1296usize, 970usize),
            (1920, 1080, 1296, 970),
            (1000, 1000, 4000, 3000),
            (640, 480, 640, 480),
        ] {
            let v = Viewport::fit(dw, dh, sw, sh);
            let src = sw as f64 / sh as f64;
            let dst = v.width as f64 / v.height as f64;
            assert!(
                (src - dst).abs() < 0.01,
                "{sw}x{sh} into {dw}x{dh}: aspect {src:.3} became {dst:.3}"
            );
        }
    }

    #[test]
    fn viewport_never_exceeds_the_surface() {
        let v = Viewport::fit(1080, 2264, 1296, 970);
        assert!(v.right() <= 1080);
        assert!(v.bottom() <= 2264);
        assert!(v.area() < 1080 * 2264, "there must be bars to paint");
    }

    #[test]
    fn matching_aspect_fills_the_surface() {
        let v = Viewport::fit(640, 480, 1280, 960);
        assert_eq!((v.left, v.top, v.width, v.height), (0, 0, 640, 480));
        assert_eq!(v.area(), 640 * 480);
    }

    #[test]
    fn degenerate_geometry_is_empty_not_panicking() {
        assert!(Viewport::fit(0, 100, 10, 10).empty());
        assert!(Viewport::fit(100, 100, 0, 10).empty());
    }
}
