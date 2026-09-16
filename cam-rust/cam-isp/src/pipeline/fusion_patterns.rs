use crate::pipeline::IspBlock;

pub struct IspFusionPatterns;

impl IspFusionPatterns {
    /// Pattern 1: Normalize + BLC50 + UnpackCfa -> NormalizeBlcUnpackCfa
    /// This fuses three common operations into one for efficiency.
    pub fn pattern_normalize_blc_unpack_cfa() -> Vec<&'static str> {
        vec!["normalize", "blc50", "unpack_cfa"]
    }

    /// Pattern 2: CFA + DemosaicCcm -> DemosaicCcm
    /// CFA (sensor data extraction) + DemosaicCcm (demosaicing + CCM) fused
    pub fn pattern_cfa_demosaic_ccm() -> Vec<&'static str> {
        vec!["cfa", "demosaic_ccm"]
    }

    /// Pattern 3: UnpackCfa + Blc -> UnpackCfaWithBlc
    /// UnpackCfa with optional BLC fusion (already handled by DemosaicCcm)
    pub fn pattern_unpack_cfa_blc() -> Vec<&'static str> {
        vec!["unpack_cfa", "blc"]
    }

    /// Pattern 4: UnpackCfa -> UnpackCfa (identity pass-through)
    /// Used for pass-through optimization
    pub fn pattern_unpack_cfa_only() -> Vec<&'static str> {
        vec!["unpack_cfa"]
    }

    /// Pattern 5: Normalize -> Normalize (identity)
    /// Simple normalization pass-through
    pub fn pattern_normalize_only() -> Vec<&'static str> {
        vec!["normalize"]
    }

    /// Match a sequence of block IDs to a fused block kind.
    /// Returns the fused block's `IspBlock::id()` value (e.g. "isp.unpack_blc")
    /// on exact match, or `None` if no pattern matches.
    pub fn match_fused(block_ids: &[&str]) -> Option<&'static str> {
        match block_ids {
            ["normalize", "blc50", "unpack_cfa"] => Some("isp.unpack_blc"),
            ["normalize", "blc50", "unpack_cfa", "cfa", "demosaic_ccm"] => {
                Some("isp.unpack_packed")
            }
            ["normalize", "blc", "unpack_cfa", "cfa"] => Some("isp.unpack_packed"),
            ["normalize", "blc", "unpack_cfa"] => Some("isp.unpack_blc"),
            ["normalize", "blc", "cfa", "demosaic_ccm"] => Some("isp.unpack_packed"),
            ["normalize", "blc", "cfa"] => Some("isp.unpack_packed"),
            ["normalize", "blc"] => Some("isp.unpack_blc"),
            ["normalize", "unpack_cfa"] => Some("isp.unpack_packed"),
            ["normalize", "blc50"] => Some("isp.unpack_blc"),
            ["blc50", "unpack_cfa"] => Some("isp.unpack_blc"),
            ["unpack_cfa", "cfa", "demosaic_ccm"] => Some("isp.unpack_packed"),
            ["unpack_cfa", "cfa"] => Some("isp.unpack_packed"),
            ["unpack_cfa", "demosaic_ccm"] => Some("isp.unpack_packed"),
            ["cfa", "demosaic_ccm"] => Some("isp.demosaic_ccm"),
            ["cfa", "demosaic"] => Some("isp.demosaic_a"),
            ["cfa", "bilateral"] => Some("isp.demosaic_edge"),
            ["bilateral", "ee"] => Some("isp.ee"),
            ["sharpen", "bilateral"] => Some("isp.ee"),
            ["fcs", "ee"] => Some("isp.ee"),
            ["ldci", "ee"] => Some("isp.ee"),
            ["ee", "fcs"] => Some("isp.ee"),
            ["ee", "ldci"] => Some("isp.ee"),
            ["awb", "wb"] => Some("isp.awb"),
            ["ae", "wb"] => Some("isp.ae"),
            ["ae", "awb"] => Some("isp.ae"),
            ["fcs", "tone"] => Some("isp.tone"),
            ["ldci", "tone"] => Some("isp.tone"),
            ["ee", "tone"] => Some("isp.tone"),
            ["vignetting", "lsc"] => Some("isp.lsc"),
            ["lsc", "colorspace"] => Some("isp.colorspace"),
            ["warp", "gamma"] => Some("isp.gamma"),
            ["display", "cfa"] => Some("isp.display"),
            ["cfa", "bilateral"] => Some("isp.demosaic_edge"),
            ["fcs", "ldci"] => Some("isp.ldci"),
            ["fcs", "ldci", "ee"] => Some("isp.ee"),
            ["normalize"] => Some("isp.normalize"),
            ["blc50"] => Some("isp.unpack_blc"),
            ["blc"] => Some("isp.unpack_blc"),
            ["unpack_cfa"] => Some("isp.unpack_packed"),
            ["cfa"] => Some("isp.pyramid"),
            ["cfa", "bayer_wb"] => Some("isp.fcs"),
            ["bayer_wb"] => Some("isp.awb"),
            ["ccm"] => Some("isp.fcs"),
            ["demosaic_ccm"] => Some("isp.demosaic_ccm"),
            ["demosaic"] => Some("isp.demosaic_debayer"),
            ["demosaic_interp"] => Some("isp.demosaic_interp"),
            ["fcs"] => Some("isp.fcs"),
            ["ee"] => Some("isp.ee"),
            ["ldci"] => Some("isp.ldci"),
            ["ldci_a"] => Some("isp.ldci_a"),
            ["lsc"] => Some("isp.lsc"),
            ["tone"] => Some("isp.tone"),
            ["gamma"] => Some("isp.gamma"),
            ["vignetting"] => Some("isp.vignetting"),
            ["auto_contrast"] => Some("isp.auto_contrast"),
            ["bilateral"] => Some("isp.bilateral"),
            ["warp"] => Some("isp.warp"),
            ["display"] => Some("isp.display"),
            ["colorspace"] => Some("isp.colorspace"),
            ["saturation"] => Some("isp.fcs"),
            ["sharpen"] => Some("isp.ee"),
            ["wavelet_denoise"] => Some("isp.demosaic_edge"),
            ["pyramid"] => Some("isp.pyramid"),
            ["resize"] => Some("isp.interp"),
            ["adaptive_downscale"] => Some("isp.interp"),
            ["temporal_denoise"] => Some("isp.fcs"),
            ["hdr_tone"] => Some("isp.tone"),
            ["wb"] => Some("isp.awb"),
            ["ae"] => Some("isp.ae"),
            ["af_focus"] => Some("isp.af_focus"),
            ["calibration"] => Some("isp.calibration"),
            ["histogram"] => Some("isp.histogram"),
            ["tone_stats"] => Some("isp.tone_stats"),
            ["cal_stats"] => Some("isp.calib_stats"),
            _ => None,
        }
    }

    /// Check if a sequence of block IDs matches any fused pattern.
    /// Returns the fused block's `IspBlock::id()` value, or None.
    /// Delegates to `match_fused`.
    pub fn check_fused(block_ids: &[&str]) -> Option<&'static str> {
        Self::match_fused(block_ids)
    }
}
