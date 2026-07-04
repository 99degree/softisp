//! Scene classification based on luminance and color temperature.
//!
//! Ported from `StatsLearner.kt` (Java) — scene classification logic.
//!
//! Classifies a scene into one of:
//! - Dark (very low light)
//! - Indoor (moderate light, any CCT)
//! - Sunset/Sunrise (low light with warm CCT)
//! - Outdoor (natural daylight)
//! - Bright (very high light)
//! - Unknown

/// Scene category for scene-adaptive ISP tuning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneCategory {
    Dark,
    Indoor,
    SunriseSunset,
    Outdoor,
    Bright,
    Unknown,
}

impl SceneCategory {
    /// Return the name as a static string.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Dark => "DARK",
            Self::Indoor => "INDOOR",
            Self::SunriseSunset => "SUNRISE_SUNSET",
            Self::Outdoor => "OUTDOOR",
            Self::Bright => "BRIGHT",
            Self::Unknown => "UNKNOWN",
        }
    }

    /// Classify a scene from mean luminance and estimated CCT.
    ///
    /// `lum` is the mean luminance in [0, 1].
    /// `cct` is the estimated color temperature in Kelvin (can be 0 if unknown).
    pub fn classify(lum: f32, cct: u32) -> Self {
        if !lum.is_finite() || lum < 0.03 {
            return Self::Dark;
        }
        if (0.03..=0.25).contains(&lum) && (2000..=4000).contains(&cct) {
            return Self::SunriseSunset;
        }
        if (0.03..=0.25).contains(&lum) {
            return Self::Indoor;
        }
        if lum > 0.5 {
            return Self::Bright;
        }
        if lum >= 0.25 {
            return Self::Outdoor;
        }
        Self::Unknown
    }

    /// Classify from luminance only (when CCT is unavailable).
    pub fn classify_lum_only(lum: f32) -> Self {
        if !lum.is_finite() || lum < 0.03 {
            Self::Dark
        } else if lum <= 0.25 {
            Self::Indoor
        } else if lum > 0.5 {
            Self::Bright
        } else {
            Self::Outdoor
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dark() {
        assert_eq!(SceneCategory::classify(0.01, 5000), SceneCategory::Dark);
        assert_eq!(SceneCategory::classify(0.029, 5500), SceneCategory::Dark);
    }

    #[test]
    fn test_sunrise_sunset() {
        assert_eq!(SceneCategory::classify(0.1, 3000), SceneCategory::SunriseSunset);
        assert_eq!(SceneCategory::classify(0.03, 2000), SceneCategory::SunriseSunset);
        assert_eq!(SceneCategory::classify(0.25, 3500), SceneCategory::SunriseSunset);
    }

    #[test]
    fn test_indoor() {
        assert_eq!(SceneCategory::classify(0.1, 5500), SceneCategory::Indoor);
        assert_eq!(SceneCategory::classify(0.2, 10000), SceneCategory::Indoor);
    }

    #[test]
    fn test_bright() {
        assert_eq!(SceneCategory::classify(0.6, 5500), SceneCategory::Bright);
        assert_eq!(SceneCategory::classify(0.9, 3000), SceneCategory::Bright);
    }

    #[test]
    fn test_outdoor() {
        assert_eq!(SceneCategory::classify(0.3, 5500), SceneCategory::Outdoor);
        assert_eq!(SceneCategory::classify(0.4, 6000), SceneCategory::Outdoor);
    }

    #[test]
    fn test_unknown() {
        assert_eq!(SceneCategory::classify(f32::NAN, 0), SceneCategory::Dark);
    }

    #[test]
    fn test_lum_only() {
        assert_eq!(SceneCategory::classify_lum_only(0.01), SceneCategory::Dark);
        assert_eq!(SceneCategory::classify_lum_only(0.1), SceneCategory::Indoor);
        assert_eq!(SceneCategory::classify_lum_only(0.3), SceneCategory::Outdoor);
        assert_eq!(SceneCategory::classify_lum_only(0.6), SceneCategory::Bright);
    }

    #[test]
    fn test_edge_case() {
        assert_eq!(SceneCategory::classify(0.03, 1500), SceneCategory::Indoor);
        assert_eq!(SceneCategory::classify(0.03, 4500), SceneCategory::Indoor);
    }
}
