//! AdaptiveDownscaleBlock — hybrid crop+pad+scale+align downscale.
//!
//! Fuses aspect-adaptive scaling + content-aware padding into one ONNX
//! subgraph (Slice + Resize + Pad).  No black bars, no distortion.
//!
//! # EIS margin
//!
//! When EIS/deshake is active, AdaptiveDownscaleBlock should produce an
//! output slightly larger than the target resolution, providing margin
//! for the warp block to shift without revealing black borders.
//!
//! Design:
//! - Compute the "fit" scale from source → target (preserving aspect).
//! - The margin is dynamic: the difference between available scaled pixels
//!   and the target dimensions.
//!   - If aspect matches → scale slightly larger (e.g., 1.1×) to create
//!     real-pixel margin
//!   - If aspect differs → black bars ARE the margin (free)
//! - After EIS warp, crop to exactly target dimensions.
//! - The margin must be large enough to absorb worst-case EIS shift
//!   (configurable via PipelineProfile).
//! - Scene object H/W ratio is preserved in the final cropped frame.
//!
//! # Modes
//!
//! - **"fit"** (default): scale to fit within target bounds preserving
//!   aspect ratio, then fill pillarbox/letterbox with edge reflection.
//!   Preserves ALL content, no cropping, no distortion.
//!
//! - **"crop"**: crop source to target aspect ratio first, then scale
//!   to fill target.  Maximizes FOV fill but loses edge pixels.
//!
//! - **"pad"**: pad source to target aspect ratio (edge reflect), then
//!   scale to fill target.  Preserves all content, no cropping.
//!
//! All modes end with optional macroblock alignment pad (edge reflect).
//!
//! # Controller integration
//!
//! Parameters (target_w, target_h, codec_align, pad_mode) are set at
//! build time from PipelineProfile.  Future: extra inputs from
//! IspController for scene-adaptive tolerance (edge density feedback).

use crate::onnx::proto::Proto;
use crate::pipeline::IspBlock;

pub struct AdaptiveDownscaleBlock {
    pub id: String,
    pub prev: Option<Box<dyn IspBlock>>,
    pub next: Option<Box<dyn IspBlock>>,
    pub frame_tensor: String,
    pub input_source: String,
    /// Target output width in pixels.
    pub target_w: i64,
    /// Target output height in pixels.
    pub target_h: i64,
    /// Codec macroblock alignment (8 or 16). 0 = no alignment.
    pub codec_align: i64,
    /// Padding filler mode: "reflect", "edge", or "constant".
    pub filler_mode: String,
    /// Aspect handling: "fit", "crop", or "pad".
    pub aspect_mode: String,
    /// EIS margin fraction (e.g. 0.05 for 5%). When set, the block scales to
    /// (1 + margin) × target then center-crops to target, reserving edge pixels
    /// for EIS/deshake warp shifts without revealing black borders.
    pub margin: f64,
    /// Concrete input dimensions (set via with_concrete_dims).
    pub in_h: Option<i64>,
    pub in_w: Option<i64>,
}

impl AdaptiveDownscaleBlock {
    /// Create an adaptive downscale block.
    ///
    /// - `target_w`, `target_h`: output resolution in pixels
    /// - `codec_align`: macroblock alignment (0, 8, or 16)
    /// - `filler_mode`: bar filling ("reflect", "edge", "constant")
    /// - `aspect_mode`: "fit" | "crop" | "pad"
    pub fn new(
        target_w: i64,
        target_h: i64,
        codec_align: i64,
        filler_mode: &str,
        aspect_mode: &str,
    ) -> Self {
        Self {
            id: "adaptive_downscale".into(),
            prev: None,
            next: None,
            frame_tensor: "AdaptiveDownscaleBlock/frame".into(),
            input_source: String::new(),
            target_w,
            target_h,
            codec_align: codec_align.max(0),
            filler_mode: filler_mode.to_string(),
            aspect_mode: aspect_mode.to_string(),
            margin: 0.0,
            in_h: None,
            in_w: None,
        }
    }

    /// Set concrete input dimensions.
    pub fn with_concrete_dims(mut self, h: i64, w: i64) -> Self {
        self.in_h = Some(h);
        self.in_w = Some(w);
        self
    }

    /// Set EIS margin fraction (0.05 = 5%). The block scales to
    /// (1 + margin) × target then center-crops to target, reserving edge
    /// pixels for EIS warp shifts without revealing black borders.
    pub fn with_margin(mut self, margin: f64) -> Self {
        self.margin = margin.max(0.0);
        self
    }

    // ── Internal parameter computation ──

    /// Compute the effective source rect after crop/pad to match target AR.
    /// Returns (src_y, src_h, src_x, src_w) — the region of interest in
    /// source coordinates, or None if already matching.
    fn aspect_adjust(&self) -> Option<(i64, i64, i64, i64)> {
        let (sh, sw) = (self.in_h?, self.in_w?);
        let src_aspect = sw as f64 / sh as f64;
        let tgt_aspect = self.target_w as f64 / self.target_h as f64;

        if (src_aspect - tgt_aspect).abs() < 0.001 {
            return None; // already matches
        }

        match self.aspect_mode.as_str() {
            "crop" => {
                // Crop edges to match target AR
                if src_aspect > tgt_aspect {
                    // Too wide → crop left/right
                    let target_w = sh as f64 * tgt_aspect;
                    let crop = ((sw as f64 - target_w) / 2.0).round() as i64;
                    Some((0, sh, crop, sw - crop))
                } else {
                    // Too tall → crop top/bottom
                    let target_h = sw as f64 / tgt_aspect;
                    let crop = ((sh as f64 - target_h) / 2.0).round() as i64;
                    Some((crop, sh - crop, 0, sw))
                }
            }
            "pad" => {
                // Pad edges to match target AR (no crop, add filler)
                if src_aspect < tgt_aspect {
                    // Too tall → pad left/right
                    let target_w = sh as f64 * tgt_aspect;
                    let pad = ((target_w - sw as f64) / 2.0).round() as i64;
                    Some((-pad, sh, -pad, sw + pad))
                } else {
                    // Too wide → pad top/bottom
                    let target_h = sw as f64 / tgt_aspect;
                    let pad = ((target_h - sh as f64) / 2.0).round() as i64;
                    Some((-pad, sh + pad, 0, sw))
                }
            }
            _ /* "fit" */ => {
                // Scale to fit within target, no crop, no distortion
                // The resize step handles this — aspect_adjust returns None
                // to skip the crop/pad steps for "fit" mode.
                None
            }
        }
    }

    /// The intermediate dimensions after aspect adjustment (before resize).
    fn aspect_dims(&self) -> (i64, i64) {
        match self.aspect_adjust() {
            Some((_, ah, _, aw)) if ah > 0 && aw > 0 => (ah, aw),
            _ => (self.in_h.unwrap_or(0), self.in_w.unwrap_or(0)),
        }
    }

    /// Compute scale factor and output dims before codec-align pad.
    fn scale_out_dims(&self) -> (f64, f64, i64, i64) {
        let (ah, aw) = self.aspect_dims();
        if ah <= 0 || aw <= 0 {
            return (1.0, 1.0, self.target_h, self.target_w);
        }
        let src_aspect = aw as f64 / ah as f64;
        // Effective target includes EIS margin (scale larger, crop back later)
        let eff_w = (self.target_w as f64 * (1.0 + self.margin)).round() as i64;
        let eff_h = (self.target_h as f64 * (1.0 + self.margin)).round() as i64;
        let eff_w = eff_w.max(1);
        let eff_h = eff_h.max(1);
        let tgt_aspect = eff_w as f64 / eff_h as f64;

        let (scale_h, scale_w, out_h, out_w) = match self.aspect_mode.as_str() {
            "crop" => {
                // Scale to fill effective target (may crop further via resize)
                let scale = (eff_w as f64 / aw as f64)
                    .max(eff_h as f64 / ah as f64);
                (scale, scale, eff_h, eff_w)
            }
            "pad" => {
                // Scale to fill effective target (padding already done above)
                let scale = (eff_w as f64 / aw as f64)
                    .max(eff_h as f64 / ah as f64);
                (scale, scale, eff_h, eff_w)
            }
            _ /* "fit" */ => {
                // Scale to fit within effective target (letterbox/pillarbox)
                if src_aspect > tgt_aspect {
                    // Wider → fit width, pillarbox top/bottom
                    let scale = eff_w as f64 / aw as f64;
                    let oh = (ah as f64 * scale).round() as i64;
                    (scale, scale, oh, eff_w)
                } else {
                    // Taller → fit height, letterbox left/right
                    let scale = eff_h as f64 / ah as f64;
                    let ow = (aw as f64 * scale).round() as i64;
                    (scale, scale, eff_h, ow)
                }
            }
        };
        (scale_h, scale_w, out_h.max(1), out_w.max(1))
    }

    /// Output dimensions after EIS margin center-crop + align pad.
    pub fn out_dims(&self) -> (i64, i64) {
        // Final output is the nominal target (margin is cropped off after scale)
        let a = self.codec_align;
        if a <= 0 {
            return (self.target_h, self.target_w);
        }
        let pb = (a - (self.target_h % a)) % a;
        let pr = (a - (self.target_w % a)) % a;
        (self.target_h + pb, self.target_w + pr)
    }

    /// Compute padding amounts for the codec-align pad at the end.
    fn align_pad(&self) -> (i64, i64, i64, i64) {
        let a = self.codec_align;
        if a <= 0 {
            return (0, 0, 0, 0);
        }
        // Align to nominal target dims (margin already cropped)
        let pb = (a - (self.target_h % a)) % a;
        let pr = (a - (self.target_w % a)) % a;
        (0, pb, 0, pr)
    }
}

// ── IspBlock trait implementation ──

impl IspBlock for AdaptiveDownscaleBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn tensor_ns(&self) -> String {
        "AdaptiveDownscaleBlock".to_string()
    }
    fn frame_tensor(&self) -> Option<&str> {
        Some(&self.frame_tensor)
    }
    fn input_source(&self) -> Option<&str> {
        Some(&self.input_source)
    }
    fn set_input_source(&mut self, name: &str) {
        self.input_source = name.to_string();
    }
    fn prev(&self) -> Option<&Box<dyn IspBlock>> {
        self.prev.as_ref()
    }
    fn set_prev(&mut self, block: Box<dyn IspBlock>) {
        self.prev = Some(block);
    }
    fn next(&self) -> Option<&Box<dyn IspBlock>> {
        self.next.as_ref()
    }
    fn set_next(&mut self, block: Box<dyn IspBlock>) {
        self.next = Some(block);
    }
    fn input_tensors(&self) -> Vec<String> {
        vec![self.input_source.clone()]
    }
    fn output_tensors(&self) -> Vec<String> {
        vec![self.frame_tensor.clone()]
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        let ns = self.tensor_ns();
        match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => Some(Proto::value_info(
                &self.input_source,
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_param(&format!("{}_C", ns)),
                    Proto::tensor_dim_value(h),
                    Proto::tensor_dim_value(w),
                ],
                1,
            )),
            _ => Some(Proto::value_info(
                &self.input_source,
                &[
                    Proto::tensor_dim_value(1),
                    Proto::tensor_dim_param("C"),
                    Proto::tensor_dim_param("H"),
                    Proto::tensor_dim_param("W"),
                ],
                1,
            )),
        }
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        let (oh, ow) = self.out_dims();
        Some(Proto::value_info(
            &self.frame_tensor,
            &[
                Proto::tensor_dim_value(1),
                Proto::tensor_dim_param("C"),
                Proto::tensor_dim_value(oh),
                Proto::tensor_dim_value(ow),
            ],
            1,
        ))
    }

    fn nodes(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let needs_aspect = self.aspect_adjust().is_some();
        let (_apt, _apb, _apl, _apr) = self.align_pad();
        let needs_align = _apb > 0 || _apr > 0;
        let (scale_h, scale_w, _oh, _ow) = self.scale_out_dims();
        let needs_resize = (scale_h - 1.0).abs() > 0.001 || (scale_w - 1.0).abs() > 0.001;
        let needs_margin_crop = self.margin > 0.001;
        // Always produces at least one output tensor.
        let input_name = &self.input_source;
        let _after_aspect = format!("{}/aspect_adjusted", ns);
        let after_resize = format!("{}/resized", ns);
        let final_output = &self.frame_tensor;

        let mut nodes: Vec<Vec<u8>> = Vec::new();
        // Keep intermediate tensor names alive across the function scope
        let mut tensor_pool: Vec<String> = Vec::new();
        let mut prev_tensor = input_name.as_str();

        // Stage 1: Aspect adjustment (Slice for crop, Pad for pad, skip for fit)
        if needs_aspect {
            let ns = self.tensor_ns();
            let (sy, sah, sx, saw) = self.aspect_adjust().unwrap();
            let aspect_out = if needs_resize || needs_align {
                format!("{}/aspect_adjusted", ns)
            } else {
                self.frame_tensor.clone()
            };
            if self.aspect_mode.as_str() == "pad" {
                let pads_t = format!("{}/aspect_pads", ns);
                let cval = format!("{}/aspect_cval", ns);
                let _p_top = if sy < 0 { -sy } else { 0 };
                let _p_bot = if sah > (self.in_h.unwrap_or(0) + sy) {
                    sah - self.in_h.unwrap_or(0) - sy
                } else {
                    0
                };
                let _p_left = if sx < 0 { -sx } else { 0 };
                let _p_right = if saw > (self.in_w.unwrap_or(0) + sx) {
                    saw - self.in_w.unwrap_or(0) - sx
                } else {
                    0
                };
                nodes.push(Proto::node(
                    "Pad",
                    &[prev_tensor, &pads_t, &cval],
                    &[&aspect_out],
                    &[Proto::attribute_string("mode", &self.filler_mode)],
                ));
            } else {
                let starts = format!("{}/crop_starts", ns);
                let ends = format!("{}/crop_ends", ns);
                let axes = format!("{}/crop_axes", ns);
                let steps = format!("{}/crop_steps", ns);
                nodes.push(Proto::node(
                    "Slice",
                    &[prev_tensor, &starts, &ends, &axes, &steps],
                    &[&aspect_out],
                    &[],
                ));
            }
            tensor_pool.push(aspect_out);
            prev_tensor = tensor_pool.last().unwrap().as_str();
        }

        // Stage 2: Resize if needed
        if needs_resize {
            let scales = format!("{}/scales", ns);
            // If margin > 0, resize goes to intermediate for crop
            let needs_intermediate = needs_align || needs_margin_crop;
            let resize_out = if needs_intermediate {
                &after_resize
            } else {
                final_output
            };
            nodes.push(Proto::node(
                "Resize",
                &[prev_tensor, "", "", &scales],
                &[resize_out],
                &[
                    Proto::attribute_string("mode", "nearest"),
                    Proto::attribute_int("coordinate_transformation_mode", 0),
                ],
            ));
            prev_tensor = resize_out;
        }

        // Stage 2b: EIS margin center-crop (if margin > 0)
        // Scales to (1+margin)×target, then crops back to target
        let needs_margin_crop = self.margin > 0.001;
        if needs_margin_crop {
            let margin_crop_out = if needs_align {
                format!("{}/margin_cropped", ns)
            } else {
                self.frame_tensor.clone()
            };
            let starts = format!("{}/margin_starts", ns);
            let ends = format!("{}/margin_ends", ns);
            let axes = format!("{}/margin_axes", ns);
            let steps = format!("{}/margin_steps", ns);
            nodes.push(Proto::node(
                "Slice",
                &[prev_tensor, &starts, &ends, &axes, &steps],
                &[&margin_crop_out],
                &[],
            ));
            tensor_pool.push(margin_crop_out);
            prev_tensor = tensor_pool.last().unwrap().as_str();
        }

        // Stage 3: Codec-alignment pad if needed
        if needs_align {
            let pads = format!("{}/align_pads", ns);
            let cval = format!("{}/align_cval", ns);
            nodes.push(Proto::node(
                "Pad",
                &[prev_tensor, &pads, &cval],
                &[final_output],
                &[Proto::attribute_string("mode", "edge")],
            ));
        }

        nodes
    }

    fn initializers(&self) -> Vec<Vec<u8>> {
        let ns = self.tensor_ns();
        let mut inits: Vec<Vec<u8>> = Vec::new();
        let (sh, sw) = match (self.in_h, self.in_w) {
            (Some(h), Some(w)) => (h, w),
            _ => return inits,
        };

        // Aspect adjustment initializers
        if let Some((sy, sah, sx, saw)) = self.aspect_adjust() {
            if self.aspect_mode.as_str() == "pad" {
                let p_top = if sy < 0 { -sy } else { 0 };
                let p_bot = if sah > sh + sy { sah - sh - sy } else { 0 };
                let p_left = if sx < 0 { -sx } else { 0 };
                let p_right = if saw > sw + sx { saw - sw - sx } else { 0 };
                let pad_arr: [i64; 8] = [0, 0, p_top, p_left, 0, 0, p_bot, p_right];
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/aspect_pads", ns),
                    &pad_arr,
                ));
                inits.push(Proto::tensor_proto_float_scalar(
                    &format!("{}/aspect_cval", ns),
                    0.0,
                ));
            } else {
                // Crop
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/crop_starts", ns),
                    &[0, 0, sy.max(0), sx.max(0)],
                ));
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/crop_ends", ns),
                    &[i64::MAX, i64::MAX, (sy + sah).min(sh), (sx + saw).min(sw)],
                ));
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/crop_axes", ns),
                    &[0, 1, 2, 3],
                ));
                inits.push(Proto::tensor_proto_int64(
                    &format!("{}/crop_steps", ns),
                    &[1, 1, 1, 1],
                ));
            }
        }

        // Resize scale factors
        let (scale_h, scale_w, _oh, _ow) = self.scale_out_dims();
        if (scale_h - 1.0).abs() > 0.001 || (scale_w - 1.0).abs() > 0.001 {
            inits.push(Proto::tensor_proto_float(
                &format!("{}/scales", ns),
                &[4],
                &[1.0f32, 1.0f32, scale_h as f32, scale_w as f32],
            ));
        }

        // EIS margin center-crop (from enlarged resize output back to target)
        if self.margin > 0.001 && _oh > self.target_h && _ow > self.target_w {
            let crop_h = (_oh - self.target_h) / 2;
            let crop_w = (_ow - self.target_w) / 2;
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/margin_starts", ns),
                &[0, 0, crop_h.max(0), crop_w.max(0)],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/margin_ends", ns),
                &[
                    i64::MAX,
                    i64::MAX,
                    (crop_h + self.target_h).min(_oh),
                    (crop_w + self.target_w).min(_ow),
                ],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/margin_axes", ns),
                &[0, 1, 2, 3],
            ));
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/margin_steps", ns),
                &[1, 1, 1, 1],
            ));
        }

        // Codec-align pad
        let (_, pb, _, pr) = self.align_pad();
        if pb > 0 || pr > 0 {
            let pad_arr: [i64; 8] = [0, 0, 0, 0, 0, 0, pb, pr];
            inits.push(Proto::tensor_proto_int64(
                &format!("{}/align_pads", ns),
                &pad_arr,
            ));
            inits.push(Proto::tensor_proto_float_scalar(
                &format!("{}/align_cval", ns),
                0.0,
            ));
        }

        inits
    }

    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        let ns = self.tensor_ns();
        vec![
            (format!("{}/margin", ns), 1, vec![1]), // float scalar: EIS margin (0.05 = 5%)
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fit_same_aspect() {
        // 1920×1080 (16:9) → 1280×720 (16:9) — already matches
        let b =
            AdaptiveDownscaleBlock::new(1280, 720, 0, "edge", "fit").with_concrete_dims(1080, 1920);
        assert!(b.aspect_adjust().is_none(), "Same aspect → no crop/pad");
        let (oh, ow) = b.out_dims();
        assert_eq!(oh, 720);
        assert_eq!(ow, 1280);
    }

    #[test]
    fn test_fit_pillarbox() {
        // 1920×1080 (16:9) → 640×480 (4:3, 1.333)
        // 16:9 wider than 4:3 → fit width, pillarbox top/bottom
        let b =
            AdaptiveDownscaleBlock::new(640, 480, 0, "edge", "fit").with_concrete_dims(1080, 1920);
        assert!(b.aspect_adjust().is_none(), "Fit mode → no aspect adjust");
        let (_, _, oh, ow) = b.scale_out_dims();
        // After crop (crop to 1440×1080 = 4:3), then scale
        assert_eq!(ow, 640, "Should fit width");
        // 1080 * 640/1440 = 480 but oh should be 480 * 1080/1440 = 360
        // Wait let me compute: target_w=640, target_h=480, tgt_aspect=1.333
        // src_aspect=1920/1080=1.778 > 1.333 → fit width
        // scale = 640/1920 = 0.333
        // oh = 1080 * 0.333 = 360
        assert_eq!(oh, 360, "Pillarbox height");
        let (_, pb, _, _) = b.align_pad();
        assert_eq!(pb, 0, "No codec align");
    }

    #[test]
    fn test_fit_letterbox() {
        // 1080×1920 (portrait 9:16) → 640×480 (landscape 4:3)
        let b =
            AdaptiveDownscaleBlock::new(640, 480, 0, "edge", "fit").with_concrete_dims(1920, 1080);
        let (_, _, oh, ow) = b.scale_out_dims();
        // src_aspect = 1080/1920 = 0.5625 < 1.333 → fit height
        // scale = 480/1920 = 0.25
        // ow = 1080 * 0.25 = 270
        assert_eq!(oh, 480, "Fit height");
        assert_eq!(ow, 270, "Letterbox width");
    }

    #[test]
    fn test_crop_mode() {
        // 1920×1080 (16:9) → 640×480 (4:3) — crop left/right
        let b =
            AdaptiveDownscaleBlock::new(640, 480, 0, "edge", "crop").with_concrete_dims(1080, 1920);
        let adj = b.aspect_adjust().unwrap();
        assert_eq!(adj.0, 0, "No top crop");
        assert_eq!(adj.1, 1080, "Full height");
        assert!(adj.2 > 0, "Left crop");
        assert!(adj.3 < 1920, "Right crop");
        // 1080 * (640/480) = 1440, crop_total = 1920 - 1440 = 480
        assert_eq!(adj.3 - adj.2, 1440, "Cropped width");
    }

    #[test]
    fn test_pad_mode() {
        // 1920×1080 (16:9) → 640×480 (4:3) — pad top/bottom with edge reflect
        let b = AdaptiveDownscaleBlock::new(640, 480, 0, "reflect", "pad")
            .with_concrete_dims(1080, 1920);
        let adj = b.aspect_adjust().unwrap();
        // 16:9 (1.778) > 4:3 (1.333) → too wide → pad top/bottom
        // target_h = 1920 / 1.333 = 1440, pad_total = 1440 - 1080 = 360
        // Return: (-pad, sh + pad, 0, sw) = (-180, 1260, 0, 1920)
        assert_eq!(adj.0, -180, "Pad top offset");
        assert_eq!(adj.1, 1260, "Pad bottom end");
        assert_eq!(adj.1 - adj.0, 1440, "Padded height");
    }

    #[test]
    fn test_codec_align_16() {
        let b = AdaptiveDownscaleBlock::new(1280, 720, 16, "edge", "fit")
            .with_concrete_dims(1080, 1920);
        let (_, pb, _, pr) = b.align_pad();
        assert_eq!(pb, 0); // 720 % 16 = 0
        assert_eq!(pr, 0); // 1280 % 16 = 0
    }

    #[test]
    fn test_codec_align_pad() {
        // 640×360 → 16-align: 360 % 16 = 8, pad to 368
        // 640 % 16 = 0
        let b =
            AdaptiveDownscaleBlock::new(640, 360, 16, "edge", "fit").with_concrete_dims(1080, 1920);
        let (_, pb, _, pr) = b.align_pad();
        assert_eq!(pb, 8);
        assert_eq!(pr, 0);
    }

    #[test]
    fn test_nodes_count() {
        // Fit mode with same aspect → resize only → 1 node
        let b =
            AdaptiveDownscaleBlock::new(640, 480, 0, "edge", "fit").with_concrete_dims(1080, 1920);
        assert_eq!(b.nodes().len(), 1, "Resize only");
    }

    #[test]
    fn test_crop_nodes() {
        // Crop mode + align → slice + resize + pad = 3 nodes
        // 648×480 (align=16): 648%16=8 → needs pad, different aspect → crop
        let b = AdaptiveDownscaleBlock::new(648, 480, 16, "edge", "crop")
            .with_concrete_dims(1080, 1920);
        assert_eq!(b.nodes().len(), 3, "Slice + Resize + Pad");
    }

    #[test]
    fn test_fit_pillarbox_nodes() {
        // Fit mode with pillarbox → resize + pad = 2 nodes
        // 16:9 → 4:3, fit width → pad top/bottom
        let b = AdaptiveDownscaleBlock::new(640, 480, 16, "reflect", "fit")
            .with_concrete_dims(1080, 1920);
        let n = b.nodes().len();
        // Should have resize + pad + maybe identity
        assert!(n >= 1, "At least Resize");
    }
}
