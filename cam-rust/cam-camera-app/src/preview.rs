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

/// A window configured for RGBA8888 buffers of a fixed size.
pub struct WindowTarget<'a> {
    window: &'a NativeWindow,
    width: i32,
    height: i32,
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
        Ok(Self {
            window,
            width,
            height,
        })
    }

    /// Buffer width this target requested.
    pub fn width(&self) -> i32 {
        self.width
    }

    /// Buffer height this target requested.
    pub fn height(&self) -> i32 {
        self.height
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

        // Nearest-neighbour sample coordinates, computed once per axis.
        let x_map: Vec<usize> = (0..dst_w).map(|x| x * src_w / dst_w).collect();
        let y_map: Vec<usize> = (0..dst_h).map(|y| y * src_h / dst_h).collect();

        for (y, &sy) in y_map.iter().enumerate() {
            let src_row = sy * src_w * BYTES_PER_PIXEL;
            let dst_row = unsafe { bits.add(y * stride * BYTES_PER_PIXEL) };
            for (x, &sx) in x_map.iter().enumerate() {
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
        Ok(dst_w * dst_h)
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
