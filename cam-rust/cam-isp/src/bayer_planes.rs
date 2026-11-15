//! One place that maps a Bayer pattern to what each CFA plane contains.
//!
//! After the CFA stage the frame is `[1, 4, H/2, W/2]`, one plane per quad
//! position in *physical* order: TL, TR, BL, BR. Which colour sits in which
//! plane depends on the sensor's pattern, and several stages need to know that:
//!
//!   * AWB applies per-colour gains and must index them by plane;
//!   * lens shading / black level tables are per colour;
//!   * the demosaic's channel permutation already handles it internally.
//!
//! Getting this wrong is a colour swap that no amount of 3A can fix: with a
//! GBRG sensor the planes are `G, B, R, G`, and treating them as `R, Gr, Gb, B`
//! puts the red gain on a green plane and the blue gain on red — which is
//! exactly the "blue looks yellow" cast this module exists to prevent.

use cam_types::BayerPattern;

/// Which channel each CFA plane holds, in plane order TL, TR, BL, BR.
///
/// Channel indices follow the RGB convention used everywhere downstream:
/// 0 = red, 1 = green, 2 = blue.
pub fn plane_channels(pattern: i32) -> [usize; 4] {
    match pattern {
        // RGGB: TL=R, TR=G, BL=G, BR=B
        0 => [0, 1, 1, 2],
        // GRBG: TL=G, TR=R, BL=B, BR=G
        1 => [1, 0, 2, 1],
        // GBRG: TL=G, TR=B, BL=R, BR=G
        2 => [1, 2, 0, 1],
        // BGGR: TL=B, TR=G, BL=G, BR=R
        _ => [2, 1, 1, 0],
    }
}

/// The same mapping from the shared enum, for callers that have one.
pub fn plane_channels_of(pattern: BayerPattern) -> [usize; 4] {
    plane_channels(pattern as i32)
}

/// Reorder three per-colour gains `[R, G, B]` into the four plane slots.
///
/// Both greens receive the green gain, so `[R, G, B]` becomes
/// `[g[plane[0]], g[plane[1]], g[plane[2]], g[plane[3]]]`.
pub fn gains_by_plane(gains_rgb: &[f32; 3], pattern: i32) -> [f32; 4] {
    let planes = plane_channels(pattern);
    [
        gains_rgb[planes[0]],
        gains_rgb[planes[1]],
        gains_rgb[planes[2]],
        gains_rgb[planes[3]],
    ]
}

/// Fold per-plane values into `[R, G, B]` for the four plane slots.
///
/// The two green planes are averaged — that is what a channel mean over a Bayer
/// quad is worth, and it is what AWB expects.
pub fn planes_to_rgb(values: &[f32; 4], pattern: i32) -> [f32; 3] {
    let planes = plane_channels(pattern);
    let mut out = [0.0f32; 3];
    let mut greens = (0.0f32, 0u32);
    for (plane, &ch) in planes.iter().enumerate() {
        if ch == 1 {
            greens.0 += values[plane];
            greens.1 += 1;
        } else {
            out[ch] = values[plane];
        }
    }
    out[1] = if greens.1 > 0 {
        greens.0 / greens.1 as f32
    } else {
        0.0
    };
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mapping must be a permutation of one R, one B and two G — for every
    /// pattern. A typo here is a colour swap, so it is checked structurally
    /// rather than by eyeballing the table.
    #[test]
    fn every_pattern_has_one_red_one_blue_two_greens() {
        for pattern in 0..4 {
            let planes = plane_channels(pattern);
            let reds = planes.iter().filter(|c| **c == 0).count();
            let greens = planes.iter().filter(|c| **c == 1).count();
            let blues = planes.iter().filter(|c| **c == 2).count();
            assert_eq!(reds, 1, "pattern {pattern}: {planes:?}");
            assert_eq!(greens, 2, "pattern {pattern}: {planes:?}");
            assert_eq!(blues, 1, "pattern {pattern}: {planes:?}");
        }
    }

    #[test]
    fn rggb_matches_the_documented_order() {
        assert_eq!(plane_channels(0), [0, 1, 1, 2], "RGGB planes are R,G,G,B");
    }

    #[test]
    fn gbrg_is_not_the_same_as_rggb() {
        // The device reports pattern 2. If this ever equals RGGB's mapping, the
        // table has regressed to "assume RGGB", which is the bug this module
        // exists to avoid.
        assert_ne!(plane_channels(2), plane_channels(0));
        assert_eq!(plane_channels(2), [1, 2, 0, 1], "GBRG planes are G,B,R,G");
    }

    #[test]
    fn gains_follow_the_planes() {
        // Gains that would correct a red cast: boost R, cut B.
        let gains = [1.4f32, 1.0, 0.7];
        for pattern in 0..4 {
            let mapped = gains_by_plane(&gains, pattern);
            let planes = plane_channels(pattern);
            for (plane, &ch) in planes.iter().enumerate() {
                assert!(
                    (mapped[plane] - gains[ch]).abs() < 1e-6,
                    "pattern {pattern} plane {plane} holds channel {ch} but got {}",
                    mapped[plane]
                );
            }
        }
    }

    #[test]
    fn round_trip_preserves_rgb_order() {
        // A quad whose planes are exactly the channel values must fold back to
        // the same RGB triple, whatever the pattern.
        for pattern in 0..4 {
            let planes = plane_channels(pattern);
            let values: [f32; 4] = std::array::from_fn(|p| (planes[p] + 1) as f32);
            let rgb = planes_to_rgb(&values, pattern);
            assert!((rgb[0] - 1.0).abs() < 1e-6, "pattern {pattern}: {rgb:?}");
            assert!((rgb[1] - 2.0).abs() < 1e-6, "pattern {pattern}: {rgb:?}");
            assert!((rgb[2] - 3.0).abs() < 1e-6, "pattern {pattern}: {rgb:?}");
        }
    }

    #[test]
    fn greens_are_averaged() {
        let planes = plane_channels(0); // RGGB: planes 1 and 2 are green
        assert_eq!(planes[1], 1);
        assert_eq!(planes[2], 1);
        let rgb = planes_to_rgb(&[0.1, 0.4, 0.6, 0.9], 0);
        assert!((rgb[1] - 0.5).abs() < 1e-6, "greens 0.4 and 0.6 -> 0.5");
    }
}
