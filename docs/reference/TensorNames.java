package com.camcore.isp.pipeline;

/**
 * Java wrapper for TensorNames constants.
 * Generated from TensorNames.kt - DO NOT EDIT MANUALLY.
 * This class provides the same tensor name constants for JNI header generation.
 */
public final class TensorNames {

    // =========================================================================
    // INPUT BLOCKS
    // =========================================================================

    /** Raw sensor frame: N4HW INT16 (4 channels: R, Gr, Gb, B) */
    public static final String RAW_INPUT = "RAWINPUTBLOCK/FRAME";
    public static final int RAW_INPUT_ELEM_TYPE = 4;  // ONNX TensorProto.INT16

    /** Normalized raw frame: N4HW FLOAT (output of NormalizeBlock) */
    public static final String NORMALIZE_OUTPUT = "NORMALIZEBLOCK/OUTPUT";
    public static final String NORMALIZE_SENSOR_MAX = "NORMALIZEBLOCK/SENSOR_MAX";
    public static final String NORMALIZE_BLACK_LEVEL = "NORMALIZEBLOCK/BLACK_LEVEL";
    public static final int NORMALIZE_OUTPUT_ELEM_TYPE = 1;  // FLOAT

    /** DemosaicG2Ccm runtime CCM tensor: row-major float[9] (identity fallback when absent). */
    public static final String DEMOSAIC_G2_CCM_MATRIX = "DEMOSAIC_G2_CCM/CCM_MATRIX";
    public static final int DEMOSAIC_G2_CCM_ELEM_TYPE = 1;  // FLOAT

    /** CCT-based CCM interpolation output: 3x3 matrix [9] FLOAT */
    public static final String CCM_INTERP_MATRIX = "CCM_INTERP/CCM_MATRIX";
    public static final int CCM_INTERP_MATRIX_ELEM_TYPE = 1;  // FLOAT

    /** Bayer pattern index (0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR) — used by demosaic/CFA blocks */
    public static final String BAYER_PATTERN = "BAYER/PATTERN";
    public static final int BAYER_PATTERN_ELEM_TYPE = 4;  // INT16

    // =========================================================================
    // PROCESSING BLOCKS - Black Level Correction
    // =========================================================================

    /** BLC block output: N4HW FLOAT (after black level subtraction) */
    public static final String BLC_OUTPUT = "BLCBLOCK/OUTPUT";
    /** BLC block tiled pattern: [H, W, 4] FLOAT - per-pixel black level */
    public static final String BLC_PATTERN = "BLCBLOCK/BLC";
    public static final int BLC_PATTERN_ELEM_TYPE = 1;  // FLOAT

    /** Crop BLC block tiled pattern (for cropped sensor region) */
    public static final String CROP_BLC_TILED = "CROPBLCBLOCK/BLC_TILED";
    public static final int CROP_BLC_TILED_ELEM_TYPE = 1;

    /** RawBLC block tiled pattern (alternate BLC implementation) */
    public static final String RAW_BLC_TILED = "RAWBLCBLOCK/BLC_TILED";
    public static final int RAW_BLC_TILED_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - White Balance
    // =========================================================================

    /** Bayer WB gains: [4] FLOAT (R, Gr, Gb, B) */
    public static final String BAYER_WB_GAINS = "BAYERWBBLOCK/BAYER_GAINS";
    public static final int BAYER_WB_GAINS_ELEM_TYPE = 1;

    /** Bayer WB output: N4HW FLOAT */
    public static final String BAYER_WB_OUTPUT = "BAYERWBBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Color Correction
    // =========================================================================

    /** CCM 3x3 matrix: [9] FLOAT (row-major) */
    public static final String CCM_MATRIX = "CCMBLOCK/CCM_MATRIX";
    public static final int CCM_MATRIX_ELEM_TYPE = 1;

    /** CCM bias: [3] FLOAT */
    public static final String CCM_BIAS = "CCMBLOCK/CCM_BIAS";
    public static final int CCM_BIAS_ELEM_TYPE = 1;

    /** CCM output: N3HW FLOAT (RGB) */
    public static final String CCM_OUTPUT = "CCMBLOCK/OUTPUT";

    /** Demosaic+CCM combined weight matrix: [9] FLOAT */
    public static final String DEMOSAIC_CCM_WEIGHTS = "DEMOSAICCCMBLOCK/W";
    public static final String DEMOSAIC_CCM_BIAS = "DEMOSAICCCMBLOCK/B";
    public static final int DEMOSAIC_CCM_WEIGHTS_ELEM_TYPE = 1;
    public static final int DEMOSAIC_CCM_BIAS_ELEM_TYPE = 1;

    /** Demosaic+CCM output: N3HW FLOAT */
    public static final String DEMOSAIC_CCM_OUTPUT = "DEMOSAICCCMBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Tone Mapping / Gamma
    // =========================================================================

    /** Tone curve LUT: [256] FLOAT */
    public static final String TONE_LUT = "TONEBLOCK/LUT";
    public static final int TONE_LUT_ELEM_TYPE = 1;

    /** Tone block local parameters */
    public static final String TONE_CONTRAST = "TONEBLOCK/CONTRAST";
    public static final int TONE_CONTRAST_ELEM_TYPE = 1;
    public static final String TONE_BRIGHTNESS = "TONEBLOCK/BRIGHTNESS";
    public static final int TONE_BRIGHTNESS_ELEM_TYPE = 1;
    public static final String TONE_SHADOW_LIFT = "TONEBLOCK/SHADOW_LIFT";
    public static final int TONE_SHADOW_LIFT_ELEM_TYPE = 1;
    public static final String TONE_HIGHLIGHT_ROLL = "TONEBLOCK/HIGHLIGHT_ROLL";
    public static final int TONE_HIGHLIGHT_ROLL_ELEM_TYPE = 1;
    public static final String TONE_GAMMA_RECIP = "TONE_CURVE/GAMMA_RECIP";
    public static final int TONE_GAMMA_RECIP_ELEM_TYPE = 1;
    /** Gamma intermediate Pow output (pre-Clip): single producer of frameTensor */
    public static final String TONE_GAMMAD = "TONE_CURVE/GAMMAD";
    public static final int TONE_GAMMAD_ELEM_TYPE = 1;
    /** GammaBlock baked uniform vector [3]: {W, H, inv_gamma} (W/H hot-swapped). */
    public static final String TONE_GAMMA_WH = "TONE_CURVE/GAMMA_WH";
    public static final int TONE_ELEM_TYPE = 1;

    /** Tone block output: N3HW FLOAT */
    public static final String TONE_OUTPUT = "TONEBLOCK/OUTPUT";

    /**
     * ToneBlock S-curve intermediate tensors and scalar constants.
     *
     * These used to be built inline as "${tensorNs}/min" etc. inside
     * ToneBlock — a string literal per use, unable to be referenced by any
     * other component (the JNI/model tooling, a test, or a future fused
     * shader). Declaring them here keeps every graph tensor name in the one
     * registry, which is the project's naming contract.
     */
    public static final String TONE_SUB1   = "TONE_BLOCK/sub1";       // 1 - x
    public static final String TONE_SQ1    = "TONE_BLOCK/sq1";        // (1-x)^2
    public static final String TONE_LIFT_MUL = "TONE_BLOCK/lift_mul"; // lift * (1-x)^2
    public static final String TONE_LIFTED = "TONE_BLOCK/lifted";     // x + lift*(1-x)^2
    public static final String TONE_SQ2    = "TONE_BLOCK/sq2";        // y^2
    public static final String TONE_ROLL_MUL = "TONE_BLOCK/roll_mul"; // rolloff * y^2
    public static final String TONE_ROLLED = "TONE_BLOCK/rolled";     // y - rolloff*y^2
    public static final String TONE_MUL    = "TONE_BLOCK/mul";        // y * contrast
    public static final String TONE_ADD    = "TONE_BLOCK/add";        // + brightness
    public static final String TONE_RELU   = "TONE_BLOCK/relu";       // ReLU
    public static final String TONE_POW    = "TONE_BLOCK/pow";        // ^ gamma_recip

    /** Scalar constants for the S-curve (min, max, one, two). */
    public static final String TONE_MIN = "TONE_BLOCK/min";
    public static final String TONE_MAX = "TONE_BLOCK/max";
    public static final String TONE_ONE = "TONE_BLOCK/one";
    public static final String TONE_TWO = "TONE_BLOCK/two";

    /** Tone RGB LUT (3 separate curves): [3, 256] FLOAT */
    public static final String TONE_RGB_LUT = "TONEBLOCK/RGB_LUT";
    public static final int TONE_RGB_LUT_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Lens Shading Correction
    // =========================================================================

    /** LSC gain grid: [grid_h, grid_w, 4] FLOAT */
    public static final String LSC_GRID = "LSCBLOCK/GAIN_GRID";
    /** LSC gain grid tiled (full sensor): [1, 1, H, W] FLOAT - used by MNN path */
    public static final String LSC_GRID_TILED = "LSCBLOCK/GAIN_GRID_TILED";
    public static final int LSC_GRID_TILED_ELEM_TYPE = 1;
    public static final int LSC_GRID_ELEM_TYPE = 1;

    /** LSC output: N4HW FLOAT */
    public static final String LSC_OUTPUT = "LSCBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Vignetting / Shading
    // =========================================================================

    /** Vignetting map: [H, W] FLOAT */
    public static final String VIGNETTING_MAP = "VIGNETTINGBLOCK/GAIN_MAP";
    public static final int VIGNETTING_MAP_ELEM_TYPE = 1;

    /** Vignetting output: N4HW FLOAT */
    public static final String VIGNETTING_OUTPUT = "VIGNETTINGBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - YUV Domain
    // =========================================================================

    /** YUV brightness adjustment */
    public static final String YUV_BRIGHTNESS = "YUVBCCBLOCK/YUV_BRIGHTNESS";
    public static final int YUV_BRIGHTNESS_ELEM_TYPE = 1;
    public static final String YUV_CONTRAST = "YUVBCCBLOCK/YUV_CONTRAST";
    public static final int YUV_CONTRAST_ELEM_TYPE = 1;
    public static final String YUV_SATURATION = "YUVSATBLOCK/YUV_SATURATION";
    public static final int YUV_SATURATION_ELEM_TYPE = 1;
    public static final int YUV_ELEM_TYPE = 1;

    /** YUV output: N3HW FLOAT */
    public static final String YUV_OUTPUT = "YUVBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Noise Reduction
    // =========================================================================

    /** NR strength: [1] FLOAT */
    public static final String NR_STRENGTH = "NRBLOCK/STRENGTH";
    public static final int NR_STRENGTH_ELEM_TYPE = 1;

    /** NR output: N4HW FLOAT */
    public static final String NR_OUTPUT = "NRBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Noise Reduction
    // =========================================================================

    /** Sharpen strength: [1] FLOAT */
    public static final String SHARPEN_STRENGTH = "SHARPENBLOCK/STRENGTH";
    public static final int SHARPEN_STRENGTH_ELEM_TYPE = 1;

    /** Saturation strength */
    public static final String SATURATION_STRENGTH = "SATURATION/SATURATION";
    public static final int SATURATION_STRENGTH_ELEM_TYPE = 1;

    /** EE (edge enhancement) gain */
    public static final String EE_GAIN = "EEBLOCK/EE_GAIN";
    public static final int EE_GAIN_ELEM_TYPE = 1;

    /** FCS gain */
    public static final String FCS_GAIN = "FCSBLOCK/FCS_GAIN";
    public static final int FCS_GAIN_ELEM_TYPE = 1;

    /** LDCI strength */
    public static final String LDCI_STRENGTH = "LDCIBLOCK/STRENGTH";
    public static final int LDCI_STRENGTH_ELEM_TYPE = 1;

    /** Sharpen output: N3HW FLOAT */
    public static final String SHARPEN_OUTPUT = "SHARPENBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Algorithm / Gamma
    // =========================================================================

    /** Algo gamma exposure gain */
    public static final String ALGO_GAMMA_EXPOSURE_GAIN = "ALGO_GAMMA/EXPOSURE_GAIN";
    public static final String ALGO_GAMMA_ANALOG_GAIN = "ALGO_GAMMA/ANALOG_GAIN";
    public static final int ALGO_GAMMA_ANALOG_GAIN_ELEM_TYPE = 1;
    public static final String ALGO_GAMMA_SCENE_CHANGE = "ALGO_GAMMA/SCENE_CHANGE";
    public static final int ALGO_GAMMA_SCENE_CHANGE_ELEM_TYPE = 1;
    /** Algo gamma coefficients — runtime per-frame input tensors so the controller
     *  can tune the gamma curve every frame instead of baking it at build time.
     *  Each is a separate float[1] scalar (NOT packed into one vector) so each
     *  scalar keeps its own shape and name, matching the AE precedent. */
    public static final String ALGO_GAMMA_CONTRAST      = "ALGO_GAMMA/CONTRAST";
    public static final int    ALGO_GAMMA_CONTRAST_ELEM_TYPE   = 1;
    public static final String ALGO_GAMMA_BRIGHTNESS     = "ALGO_GAMMA/BRIGHTNESS";
    public static final int    ALGO_GAMMA_BRIGHTNESS_ELEM_TYPE = 1;
    public static final String ALGO_GAMMA_GAMMA_BASE     = "ALGO_GAMMA/GAMMA_BASE";
    public static final int    ALGO_GAMMA_GAMMA_BASE_ELEM_TYPE = 1;
    public static final String ALGO_GAMMA_K1             = "ALGO_GAMMA/K1";
    public static final int    ALGO_GAMMA_K1_ELEM_TYPE   = 1;
    public static final String ALGO_GAMMA_K2             = "ALGO_GAMMA/K2";
    public static final int    ALGO_GAMMA_K2_ELEM_TYPE   = 1;
    public static final String ALGO_GAMMA_GMIN           = "ALGO_GAMMA/GMIN";
    public static final int    ALGO_GAMMA_GMIN_ELEM_TYPE = 1;
    public static final String ALGO_GAMMA_GMAX           = "ALGO_GAMMA/GMAX";
    public static final int    ALGO_GAMMA_GMAX_ELEM_TYPE = 1;
    public static final int    ALGO_GAMMA_ELEM_TYPE = 1;
    public static final String ALGO_GAMMA_OUTPUT = "ALGO_GAMMA/GAMMA_OUTPUT";
    public static final int ALGO_GAMMA_OUTPUT_ELEM_TYPE = 1;

    /** Algo AE (Auto Exposure) */
    public static final String ALGO_AE_EXPOSURE_GAIN = "ALGO_AE/EXPOSURE_GAIN";
    public static final String ALGO_AE_MEAN_LUMINANCE = "ALGO_AE/MEAN_LUMINANCE";
    public static final String ALGO_AE_TARGET_LUMINANCE = "ALGO_AE/TARGET_LUMINANCE";
    public static final int ALGO_AE_ELEM_TYPE = 1;
    /**
     * Legacy packed AE stats [1,3] = [exposure_gain, lum_mean, target_luminance].
     * SUPERSEDED: AlgoAeBlock now emits three separate float[1] tensors
     * (ALGO_AE_EXPOSURE_GAIN / MEAN_LUMINANCE / TARGET_LUMINANCE) so each scalar
     * keeps its own shape and name. Kept registered for older composed graphs.
     */
    public static final String ALGO_AE_STATS = "ALGO_AE/STATS";
    public static final int ALGO_AE_STATS_ELEM_TYPE = 1;

    /** Algo AWB (Auto White Balance) */
    public static final String ALGO_AWB_GAINS = "ALGO_AWB/AWB_GAINS";
    public static final int ALGO_AWB_GAINS_ELEM_TYPE = 1;

    /** Algo CCT (Correlated Color Temperature) */
    public static final String ALGO_CCT_CCT = "ALGO_CCT/CCT";
    public static final int ALGO_CCT_ELEM_TYPE = 1;

    /** AWB gains */
    public static final String AWB_GAINS = "AWBBLOCK/AWB_GAINS";
    public static final int AWB_GAINS_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - HDR Merge
    // =========================================================================

    /** HDR merge neutral frame */
    public static final String HDR_NEUTRAL = "HDRMERGEBLOCK/NEUTRAL";
    public static final int HDR_NEUTRAL_ELEM_TYPE = 1;
    public static final String HDR_OVER = "HDRMERGEBLOCK/OVER";
    public static final int HDR_OVER_ELEM_TYPE = 1;
    public static final String HDR_DISP_UNDER = "HDRMERGEBLOCK/DISP_UNDER";
    public static final int HDR_DISP_UNDER_ELEM_TYPE = 1;
    public static final String HDR_DISP_OVER = "HDRMERGEBLOCK/DISP_OVER";
    public static final int HDR_DISP_OVER_ELEM_TYPE = 1;
    public static final int HDR_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Chromatic Aberration
    // =========================================================================

    /** CA grid R channel */
    public static final String CA_GRID_R = "CHROMATICABERRATIONBLOCK/GRID_R";
    public static final int CA_GRID_R_ELEM_TYPE = 1;
    public static final String CA_GRID_B = "CHROMATICABERRATIONBLOCK/GRID_B";
    public static final int CA_GRID_B_ELEM_TYPE = 1;
    public static final int CA_ELEM_TYPE = 1;

    /** Hue offset for YUV saturation block */
    public static final String HUE_OFFSET = "YUVSATBLOCK/HUE_OFFSET";
    public static final int HUE_OFFSET_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Downscale
    // =========================================================================

    /** Downscale sizes tensor (int64[4]: [1, 3, H, W]) */
    public static final String DOWNSCALE_SIZES = "DOWNSCALEBLOCK/SIZES";
    public static final int DOWNSCALE_SIZES_ELEM_TYPE = 7;  // INT64

    /** Downscale HW tensor (float[4]: {W,H,outW,outH}) — runtime per-frame input tensor,
     *  so the controller can change output resolution per frame instead of baking it at build time. */
    public static final String DOWNSCALE_HW = "DOWNSCALE_BLOCK/HW";
    public static final int DOWNSCALE_HW_ELEM_TYPE = 1;  // FLOAT

    /** Demosaic HW tensor (float[4]: {W,H,pad,pad}) — runtime per-frame input tensor
     *  for isp.demosaic_g2 / isp.demosaic_malvar so the controller can set W/H
     *  per frame instead of baking a zeroed const attr. Shared by both demosaic
     *  variants under a common name. */
    public static final String DEMOSAIC_BLOCK_HW = "DEMOSAIC_BLOCK/HW";
    public static final int DEMOSAIC_BLOCK_HW_ELEM_TYPE = 1;  // FLOAT

    /** Malvar demosaic weights tensor (float[60]: [3,4,5,5]) — runtime per-frame
     *  input tensor for isp.demosaic_malvar on binding 3. */
    public static final String DEMOSAIC_MALVAR_WEIGHTS = "DEMOSAIC_MALVAR_BLOCK/WEIGHTS";
    public static final int DEMOSAIC_MALVAR_WEIGHTS_ELEM_TYPE = 1;  // FLOAT

    /** Bayer pattern tensor (float[1]: 0=RGGB, 1=GRBG, 2=GBRG, 3=BGGR) — runtime
     *  per-frame input tensor shared by demosaic variants so the shader can rotate
     *  input channels to the correct RGGB kernel orientation. */
    public static final String DEMOSAIC_BAYER_PATTERN = "DEMOSAIC_BLOCK/BAYER_PATTERN";
    public static final int DEMOSAIC_BAYER_PATTERN_ELEM_TYPE = 1;  // FLOAT

    // =========================================================================
    // PROCESSING BLOCKS - Focus
    // =========================================================================

    /** Focus metric output: float[1] */
    public static final String FOCUS_METRIC = "FOCUS/METRIC";
    public static final int FOCUS_METRIC_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Histogram
    // =========================================================================

    /** Histogram bins: float[16] */
    public static final String HISTOGRAM_BINS = "HISTOGRAM/BINS";
    public static final int HISTOGRAM_BINS_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Bilateral / Auto Contrast / Local Contrast
    // =========================================================================

    /** Bilateral strength */
    public static final String BILATERAL_STRENGTH = "BILATERAL/STRENGTH";
    public static final int BILATERAL_STRENGTH_ELEM_TYPE = 1;
    public static final String AUTO_CONTRAST_STRENGTH = "AUTO_CONTRAST/STRENGTH";
    public static final int AUTO_CONTRAST_STRENGTH_ELEM_TYPE = 1;
    public static final String AUTO_CONTRAST_BRIGHTNESS = "AUTO_CONTRAST/BRIGHTNESS";
    public static final int AUTO_CONTRAST_BRIGHTNESS_ELEM_TYPE = 1;
    public static final String UNSHARP_SHARPNESS = "UNSHARP/SHARPNESS";
    public static final int UNSHARP_SHARPNESS_ELEM_TYPE = 1;
    public static final String LOCAL_CONTRAST_STRENGTH = "LOCAL_CONTRAST/STRENGTH";
    public static final int LOCAL_CONTRAST_STRENGTH_ELEM_TYPE = 1;
    public static final int CONTRAST_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Calibration
    // =========================================================================

    /** Calibration stats output: float[24] */
    public static final String CALIBRATION_STATS = "CALIBRATION/STATS";
    public static final int CALIBRATION_STATS_ELEM_TYPE = 1;

    /** Unified stats output: float[9] */
    public static final String UNIFIED_STATS = "UNIFIED_STATS/STATS";
    public static final int UNIFIED_STATS_ELEM_TYPE = 1;

    /** Zone stats output: float[zoneRows*zoneCols*3] */
    public static final String ZONE_STATS = "ZONE_STATS/STATS";
    public static final int ZONE_STATS_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Temporal Denoise
    // =========================================================================

    /** TNR previous frame */
    public static final String TNR_PREV_FRAME = "TEMPORALDENOISEBLOCK/PREV_FRAME";
    public static final int TNR_PREV_FRAME_ELEM_TYPE = 1;

    /** TNR gain */
    public static final String TNR_GAIN = "TEMPORALDENOISEBLOCK/TNR_GAIN";
    public static final int TNR_GAIN_ELEM_TYPE = 1;

    /** TNR threshold */
    public static final String TNR_THRESH = "TEMPORALDENOISEBLOCK/TNR_THRESH";
    public static final int TNR_THRESH_ELEM_TYPE = 1;

    /** TNR diff max */
    public static final String TNR_DIFF_MAX = "TEMPORALDENOISEBLOCK/DIFF_MAX";
    public static final int TNR_DIFF_MAX_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - NLM Denoise
    // =========================================================================

    /** NLM h parameter */
    public static final String NLM_H = "NLMBLOCK/H";
    public static final int NLM_H_ELEM_TYPE = 1;

    /** CEH clip limit */
    public static final String CEH_CLIP_LIMIT = "CEHBLOCK/CLIP_LIMIT";
    public static final int CEH_CLIP_LIMIT_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Wavelet Denoise
    // =========================================================================

    /** Wavelet threshold */
    public static final String WAVELET_THRESH = "WAVELETDENOISEBLOCK/THRESH";
    public static final int WAVELET_THRESH_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - BNF Denoise
    // =========================================================================

    /** BNF intensity sigma */
    public static final String BNF_INTENSITY_SIGMA = "BNFBLOCK/INTENSITY_SIGMA";
    public static final int BNF_INTENSITY_SIGMA_ELEM_TYPE = 1;

    /** BNF spatial sigma */
    public static final String BNF_SPATIAL_SIGMA = "BNFBLOCK/SPATIAL_SIGMA";
    public static final int BNF_SPATIAL_SIGMA_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - BadPixelLscBayerWb (fused)
    // =========================================================================

    /** Hot pixel mask for BadPixelLscBayerWbBlock */
    public static final String BAD_PIXEL_LSC_BAYER_WB_HOT_MASK = "BADPIXELLSCBAYERWBBLOCK/HOT_MASK";
    public static final int BAD_PIXEL_LSC_BAYER_WB_HOT_MASK_ELEM_TYPE = 2;  // UINT8

    /** LSC gain table for BadPixelLscBayerWbBlock */
    public static final String BAD_PIXEL_LSC_BAYER_WB_LSC_TABLE = "BADPIXELLSCBAYERWBBLOCK/LSC_TABLE";
    public static final int BAD_PIXEL_LSC_BAYER_WB_LSC_TABLE_ELEM_TYPE = 1;

    /** Bayer WB gains for BadPixelLscBayerWbBlock */
    public static final String BAD_PIXEL_LSC_BAYER_WB_BAYER_GAINS = "BADPIXELLSCBAYERWBBLOCK/BAYER_GAINS";
    public static final int BAD_PIXEL_LSC_BAYER_WB_BAYER_GAINS_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - CCT Interpolation
    // =========================================================================

    /** CCT input for CCM interpolation */
    public static final String CCT_IN = "CCMINTERPBLOCK/CCT";
    public static final int CCT_IN_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Hot Pixel Mask
    // =========================================================================

    /** Hot pixel mask for BadPixelBlock */
    public static final String HOT_PIXEL_MASK = "BADPIXELBLOCK/HOT_MASK";
    public static final int HOT_PIXEL_MASK_ELEM_TYPE = 2;  // UINT8

    // =========================================================================
    // PROCESSING BLOCKS - CNF Denoise
    // =========================================================================

    /** CNF threshold */
    public static final String CNF_THRESHOLD = "CNFBLOCK/THRESHOLD";
    public static final int CNF_THRESHOLD_ELEM_TYPE = 1;

    /** CNF red gain */
    public static final String CNF_R_GAIN = "CNFBLOCK/R_GAIN";
    public static final int CNF_R_GAIN_ELEM_TYPE = 1;

    /** CNF blue gain */
    public static final String CNF_B_GAIN = "CNFBLOCK/B_GAIN";
    public static final int CNF_B_GAIN_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Color Space
    // =========================================================================

    /** ColorSpace custom matrix */
    public static final String COLORSPACE_MATRIX = "COLORSPACEBLOCK/MATRIX";
    public static final int COLORSPACE_MATRIX_ELEM_TYPE = 1;

    // =========================================================================
    // PROCESSING BLOCKS - Demosaic
    // =========================================================================

    /** Demosaic output: N3HW FLOAT */
    public static final String DEMOSAIC_OUTPUT = "DEMOSAICBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - HDR Merge / Warp
    // =========================================================================

    /** HDR merge weight: [1] FLOAT */
    public static final String HDR_MERGE_WEIGHT = "HDRMERGEBLOCK/WEIGHT";
    public static final int HDR_MERGE_WEIGHT_ELEM_TYPE = 1;

    /** Warp grid: [H, W, 2] FLOAT (normalized coords) */
    public static final String WARP_GRID = "WARPBLOCK/GRID";
    public static final int WARP_GRID_ELEM_TYPE = 1;

    /** HDR max value for RefToneBlock */
    public static final String HDR_MAX = "REFTONEBLOCK/HDR_MAX";
    public static final int HDR_MAX_ELEM_TYPE = 1;

    /** HDR output: N3HW FLOAT */
    public static final String HDR_OUTPUT = "HDRMERGEBLOCK/OUTPUT";

    /** Warp output: N3HW FLOAT */
    public static final String WARP_OUTPUT = "WARPBLOCK/OUTPUT";

    // =========================================================================
    // PROCESSING BLOCKS - Statistics
    // =========================================================================

    /** AE stats grid: [grid_h, grid_w] FLOAT */
    public static final String AE_STATS = "AEBLOCK/AE_STATS";
    public static final int AE_STATS_ELEM_TYPE = 1;

    /** AWB stats: [4] FLOAT (R, Gr, Gb, B means) */
    public static final String AWB_STATS = "AWBBLOCK/AWB_STATS";
    public static final int AWB_STATS_ELEM_TYPE = 1;

    /** AF stats: [num_windows] FLOAT */
    public static final String AF_STATS = "AFBLOCK/AF_STATS";
    public static final int AF_STATS_ELEM_TYPE = 1;

    // =========================================================================
    // OUTPUT BLOCKS
    // =========================================================================

    /** Final display frame: N3HW UINT8 (RGBA) or N3HW FLOAT */
    public static final String DISPLAY_FRAME = "DISPLAYBLOCK/FRAME";
    public static final int DISPLAY_FRAME_ELEM_TYPE = 2;  // UINT8 by default
    public static final int DISPLAY_FRAME_ELEM_TYPE_FLOAT = 1;  // FLOAT variant

    /** Unified stats frame (for telemetry) */
    public static final String UNIFIED_STATS_FRAME = "UNIFIED_STATS/FRAME";

    // =========================================================================
    // SPECIAL / INTERNAL TENSORS
    // =========================================================================

    /** ISP metadata blob (opaque): [1] UINT8 */
    public static final String ISP_META = "ISP_META/DATA";
    public static final int ISP_META_ELEM_TYPE = 2;

    /** Frame index / sequence number */
    public static final String FRAME_INDEX = "FRAME_INDEX";
    public static final int FRAME_INDEX_ELEM_TYPE = 4;  // INT16

    // =========================================================================
    // MNN-SPECIFIC TENSOR NAMES (used when emitting direct isp.* ops)
    // =========================================================================

    /** MNN direct-emit ISP block namespace prefix */
    public static final String MNN_ISP_PREFIX = "ISP.";

    /** MNN BLC op output */
    public static final String MNN_BLC_OUTPUT = "ISP.BLC/OUTPUT";

    /** MNN CCM op output */
    public static final String MNN_CCM_OUTPUT = "ISP.CCM/OUTPUT";

    /** MNN Tone op output */
    public static final String MNN_TONE_OUTPUT = "ISP.TONE/OUTPUT";

    /** MNN Demosaic op output */
    public static final String MNN_DEMOSAIC_OUTPUT = "ISP.DEMOSAIC/OUTPUT";

    /** MNN BayerWB op output */
    public static final String MNN_BAYER_WB_OUTPUT = "ISP.BAYERWB/OUTPUT";

    // =========================================================================
    // HELPER FUNCTIONS
    // =========================================================================


    // =========================================================================
    // BLOCK FRAME OUTPUT TENSORS (mechanical: one per IspBlock class)
    // =========================================================================

    /** AafBlock frame output (auto-derived from block tensorNs). */
    public static final String AAF_BLOCK_FRAME = "AAF_BLOCK/FRAME";
    /** Reserved stats-demosaic feedback name. The pipeline now runs EXACTLY ONE
     *  demosaic per profile (LITE=G2, MED=EDGE, HEAVY/PRO=MALVAR) and the
     *  aux/stats blocks read that same output, so nothing produces
     *  "STATS_DEMOSAIC/FRAME" any more. Kept as the neutral default for the
     *  graph-internal stats feedback constraint. */
    public static final String STATS_DEMOSAIC_FRAME = "STATS_DEMOSAIC/FRAME";
    /** AutoContrastBlock frame output (auto-derived from block tensorNs). */
    public static final String AUTO_CONTRAST_BLOCK_FRAME = "AUTO_CONTRAST_BLOCK/FRAME";
    /** BadPixelBlock frame output (auto-derived from block tensorNs). */
    public static final String BAD_PIXEL_BLOCK_FRAME = "BAD_PIXEL_BLOCK/FRAME";
    /** BadPixelLscBayerWbBlock frame output (auto-derived from block tensorNs). */
    public static final String BAD_PIXEL_LSC_BAYER_WB_BLOCK_FRAME = "BAD_PIXEL_LSC_BAYER_WB_BLOCK/FRAME";
    /** BayerWbBlock frame output (auto-derived from block tensorNs). */
    public static final String BAYER_WB_BLOCK_FRAME = "BAYER_WB_BLOCK/FRAME";
    /** BilateralBlock frame output (auto-derived from block tensorNs). */
    public static final String BILATERAL_BLOCK_FRAME = "BILATERAL_BLOCK/FRAME";
    /** BnfBlock frame output (auto-derived from block tensorNs). */
    public static final String BNF_BLOCK_FRAME = "BNF_BLOCK/FRAME";
    /** CalibrationBlock frame output (auto-derived from block tensorNs). */
    public static final String CALIBRATION_BLOCK_FRAME = "CALIBRATION/FRAME";
    /** CcmInterpBlock frame output (auto-derived from block tensorNs). */
    public static final String CCM_INTERP_BLOCK_FRAME = "CCM_INTERP_BLOCK/FRAME";
    /** CehBlock frame output (auto-derived from block tensorNs). */
    public static final String CEH_BLOCK_FRAME = "CEH_BLOCK/FRAME";
    /** CfaBlock frame output (auto-derived from block tensorNs). */
    public static final String CFA_BLOCK_FRAME = "CFA_BLOCK/FRAME";
    /** ChromaticAberrationBlock frame output (auto-derived from block tensorNs). */
    public static final String CHROMATIC_ABERRATION_BLOCK_FRAME = "CHROMATIC_ABERRATION_BLOCK/FRAME";
    /** ChromaticAberrationMinBlock frame output (auto-derived from block tensorNs). */
    public static final String CHROMATIC_ABERRATION_MIN_BLOCK_FRAME = "CHROMATIC_ABERRATION_MIN_BLOCK/FRAME";
    /** CnfBlock frame output (auto-derived from block tensorNs). */
    public static final String CNF_BLOCK_FRAME = "CNF_BLOCK/FRAME";
    /** ColorSpaceBlock frame output (auto-derived from block tensorNs). */
    public static final String COLOR_SPACE_BLOCK_FRAME = "COLOR_SPACE_BLOCK/FRAME";
    /** CropBlcBlock frame output (auto-derived from block tensorNs). */
    public static final String CROP_BLC_BLOCK_FRAME = "CROP_BLC_BLOCK/CROP_OUT";
    /** CropBlcNormCfaBlock frame output (auto-derived from block tensorNs). */
    public static final String CROP_BLC_NORM_CFA_BLOCK_FRAME = "CROP_BLC_NORM_CFA_BLOCK/CROP_OUT";

    /** CropBlock frame output (auto-derived from block tensorNs). */
    public static final String CROP_BLOCK_FRAME = "CROP_BLOCK/FRAME";
    /** CscBlock frame output (auto-derived from block tensorNs). */
    public static final String CSC_BLOCK_FRAME = "CSCBLOCK/FRAME";
    /** DemosaicCcmBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_CCM_BLOCK_FRAME = "DEMOSAIC_CCM_BLOCK/FRAME";
    /** DemosaicEdgeBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_EDGE_BLOCK_FRAME = "DEMOSAIC_EDGE_BLOCK/FRAME";
    /** DemosaicEdgeCcmBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_EDGE_CCM_BLOCK_FRAME = "DEMOSAIC_EDGE_CCM_BLOCK/FRAME";
    /** DemosaicG2CcmBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_G2_CCM_BLOCK_FRAME = "DEMOSAIC_G2_CCM_BLOCK/FRAME";
    /** DemosaicHqLinearBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_HQ_LINEAR_BLOCK_FRAME = "DEMOSAIC_HQ_LINEAR_BLOCK/FRAME";
    /** DemosaicLinearBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_LINEAR_BLOCK_FRAME = "DEMOSAIC_LINEAR_BLOCK/FRAME";
    /** DemosaicMalvarBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_MALVAR_BLOCK_FRAME = "DEMOSAIC_MALVAR_BLOCK/FRAME";
    /** DemosaicMalvarCcmBlock frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_MALVAR_CCM_BLOCK_FRAME = "DEMOSAIC_MALVAR_CCM_BLOCK/FRAME";
    /** DemosaicQuality frame output (auto-derived from block tensorNs). */
    public static final String DEMOSAIC_QUALITY_FRAME = "DEMOSAIC_QUALITY/FRAME";
    /** DownscaleBlock frame output (auto-derived from block tensorNs). */
    public static final String DOWNSCALE_BLOCK_FRAME = "DOWNSCALE_BLOCK/FRAME";
    /** EeBlock frame output (auto-derived from block tensorNs). */
    public static final String EE_BLOCK_FRAME = "EE_BLOCK/FRAME";
    /** FastEeBlock frame output (auto-derived from block tensorNs). */
    public static final String FAST_EE_BLOCK_FRAME = "FAST_EE_BLOCK/FRAME";
    /** FcsBlock frame output (auto-derived from block tensorNs). */
    public static final String FCS_BLOCK_FRAME = "FCS_BLOCK/FRAME";
    /** FocusBlock frame output (auto-derived from block tensorNs). */
    public static final String FOCUS_BLOCK_FRAME = "FOCUS/FRAME";
    /** GammaBlock frame output (auto-derived from block tensorNs). */
    public static final String GAMMA_BLOCK_FRAME = "GAMMA_BLOCK/FRAME";
    /** GpuWarpBlock frame output (auto-derived from block tensorNs). */
    public static final String GPU_WARP_BLOCK_FRAME = "GPU_WARP_BLOCK/FRAME";
    /** HdrMergeBlock frame output (auto-derived from block tensorNs). */
    public static final String HDR_MERGE_BLOCK_FRAME = "HDR_MERGE_BLOCK/FRAME";
    /** HistogramBlock frame output (auto-derived from block tensorNs). */
    public static final String HISTOGRAM_BLOCK_FRAME = "HISTOGRAM/FRAME";
    /** IdentityBlock frame output (auto-derived from block tensorNs). */
    public static final String IDENTITY_BLOCK_FRAME = "IDENTITY_BLOCK/FRAME";
    /** LaplacianPyramidBlock frame output (auto-derived from block tensorNs). */
    public static final String LAPLACIAN_PYRAMID_BLOCK_FRAME = "LAPLACIAN_PYRAMID_BLOCK/FRAME";
    /** LdciBlock frame output (auto-derived from block tensorNs). */
    public static final String LDCI_BLOCK_FRAME = "LDCI_BLOCK/FRAME";
    /** LocalContrastBlock frame output (auto-derived from block tensorNs). */
    public static final String LOCAL_CONTRAST_BLOCK_FRAME = "LOCAL_CONTRAST/FRAME";
    /** LscBlock frame output (auto-derived from block tensorNs). */
    public static final String LSC_BLOCK_FRAME = "LSC_BLOCK/FRAME";
    /** LscMinBlock frame output (auto-derived from block tensorNs). */
    public static final String LSC_MIN_BLOCK_FRAME = "LSC_MIN_BLOCK/FRAME";

    /**
     * `isp.lsc_min` radial coefficients float[4] = {k1, k2, vcmNorm, strength}.
     *
     * A NAMED tensor (not a baked const) because the ELEMENTWISE patch writes
     * W/H into binding 0's first two floats every frame — per-frame
     * coefficients must not live there. Unfed (all zero) the shader's gain
     * collapses to exactly 1.0, which is the documented passthrough.
     */
    public static final String LSC_STRENGTH = "LSC_MIN_BLOCK/STRENGTH";

    /**
     * `isp.pipeline` STAGE MASK, float[1] bitfield: which stages of the fused op
     * RUN. Bit set = runs. All-zero (or unfed) = the FULL chain.
     *
     * This is the "slim the op down until the frame is recognizable" control:
     * drop one stage at a time and the point where the picture becomes
     * recognizable identifies the stage that was destroying it.
     *   bit 0 DEMOSAIC   area-average the CFA footprint (else point-sample)
     *   bit 1 NORMALIZE  divide the raw plane by sensorMax
     *   bit 2 BLC        subtract the per-plane black level
     *   bit 3 WB         apply the per-plane white balance
     *   bit 4 CCM        apply the 3x3 colour matrix
     *   bit 5 LIFT       S-curve shadow lift
     *   bit 6 GAMMA      display encode pow(v, gamma_recip)
     *   bit 7 LETTERBOX  fit+centre (else fill the output grid edge to edge)
     */
    public static final String PIPELINE_STAGES = "PIPELINEBLOCK/STAGES";
    public static final int PIPELINE_STAGES_ELEM_TYPE = 1;  // FLOAT

    // ── DEBUG TAP ────────────────────────────────────────────────────────────
    // Every isp.* op writes a FIXED-SIZE snapshot of its OWN output into a
    // dedicated buffer, exposed as a real graph output so it is readable from
    // Kotlin. Fixed 64x64 float (16 KB) regardless of frame size, so one reader
    // handles every op and the cost is constant.
    //
    // Layout (see DebugTap.kt / the shader's DBG_* defines):
    //   [0]=schema  [1]=output element count  [2]=W  [3]=H  [4]=planes
    //   [64..] = the 64x64 payload, row-major
    // All-zero payload means the op never ran.
    //
    // ONE name is used for all ops because only ONE op's tap is read at a time,
    // and the reader correlates the payload with the op by the [5] op-id field
    // and by the log line the block emits. Giving each op its own name would
    // multiply 64-element graph outputs for no benefit.
    public static final String DEBUG_TAP = "ISPTAP/FRAME";
    public static final int DEBUG_TAP_ELEM_TYPE = 1;  // FLOAT
    /** Element count of the tap tensor, header + payload. */
    public static final int DEBUG_TAP_FLOATS = 64 + 64 * 64;

    // Stage bits — MUST match the ST_* #defines in isp_pipeline.comp.
    public static final int STAGE_DEMOSAIC  = 1;
    public static final int STAGE_NORMALIZE = 2;
    public static final int STAGE_BLC       = 4;
    public static final int STAGE_WB        = 8;
    public static final int STAGE_CCM       = 16;
    public static final int STAGE_LIFT      = 32;
    public static final int STAGE_GAMMA     = 64;
    public static final int STAGE_LETTERBOX = 128;
    /**
     * INVERTED bit: when SET, the fused op does NO resampling — it takes the
     * centre outW x outH of the source and maps it 1:1. The op is always
     * reducing (2592x1944 -> 480x360 is 5.4x), and that reduction is the prime
     * suspect for "no longer looks like the source", so being able to switch it
     * off entirely is the decisive test. Unlike the other bits, this one being
     * SET REMOVES work.
     */
    public static final int STAGE_NOSCALE = 256;
    /** Every stage — the value the shader uses when the tensor is unfed. */
    public static final int STAGE_ALL = 255;   // every FORWARD bit; NOSCALE stays OFF
    /** Valid-bit mask for the whole field (forward bits + the inverted one). */
    public static final int STAGE_FIELD = 511;
    public static final int LSC_STRENGTH_ELEM_TYPE = 1;  // FLOAT
    /** NchwToRgbaBlock frame output (auto-derived from block tensorNs). */
    public static final String NCHW_TO_RGBA_BLOCK_FRAME = "NCHW_TO_RGBA_BLOCK/FRAME";
    /** NhwcToArgbBlock frame output (auto-derived from block tensorNs). */
    public static final String NHWC_TO_ARGB_BLOCK_FRAME = "NHWC_TO_ARGB_BLOCK/FRAME";
    /** NlmBlock frame output (auto-derived from block tensorNs). */
    public static final String NLM_BLOCK_FRAME = "NLM_BLOCK/FRAME";
    /** NormalizeBlock frame output (auto-derived from block tensorNs). */
    public static final String NORMALIZE_BLOCK_FRAME = "NORMALIZE_BLOCK/FRAME";
    /** PipelineVerifier frame output (auto-derived from block tensorNs). */
    public static final String PIPELINE_VERIFIER_FRAME = "PIPELINE_VERIFIER/FRAME";
    /** RawBlcBlock frame output (auto-derived from block tensorNs). */
    public static final String RAW_BLC_BLOCK_FRAME = "RAW_BLC_BLOCK/FRAME";
    /** RawBlcMinBlock frame output (auto-derived from block tensorNs). */
    public static final String RAW_BLC_MIN_BLOCK_FRAME = "RAW_BLC_MIN_BLOCK/FRAME";
    /** RawProcessingBlock frame output (auto-derived from block tensorNs). */
    public static final String RAW_PROCESSING_BLOCK_FRAME = "RAW_PROCESSING_BLOCK/FRAME";
    /** RefBayerWbBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_BAYER_WB_BLOCK_FRAME = "REF_BAYER_WB_BLOCK/FRAME";
    /** RefCcmBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_CCM_BLOCK_FRAME = "REF_CCM_BLOCK/FRAME";
    /** RefCnfBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_CNF_BLOCK_FRAME = "REF_CNF_BLOCK/FRAME";
    /** RefCscBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_CSC_BLOCK_FRAME = "REF_CSC_BLOCK/FRAME";
    /** RefDemosaicBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_DEMOSAIC_BLOCK_FRAME = "REF_DEMOSAIC_BLOCK/FRAME";
    /** RefToneBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_TONE_BLOCK_FRAME = "REF_TONE_BLOCK/FRAME";
    /** RefYuvBccBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_YUV_BCC_BLOCK_FRAME = "REF_YUV_BCC_BLOCK/FRAME";
    /** RefYuvSatBlock frame output (auto-derived from block tensorNs). */
    public static final String REF_YUV_SAT_BLOCK_FRAME = "REF_YUV_SAT_BLOCK/FRAME";
    /** SaturationBlock frame output (auto-derived from block tensorNs). */
    public static final String SATURATION_BLOCK_FRAME = "SATURATION_BLOCK/FRAME";
    /** SinkBlock frame output (auto-derived from block tensorNs). */
    public static final String SINK_BLOCK_FRAME = "SINK_BLOCK/FRAME";
    /** StatsBlock frame output (auto-derived from block tensorNs). */
    public static final String STATS_BLOCK_FRAME = "STATS/FRAME";
    /** TemporalDenoiseBlock frame output (auto-derived from block tensorNs). */
    public static final String TEMPORAL_DENOISE_BLOCK_FRAME = "TEMPORAL_DENOISE_BLOCK/FRAME";
    /** TestPatternBlock frame output (auto-derived from block tensorNs). */
    public static final String TEST_PATTERN_BLOCK_FRAME = "TEST_PATTERN_BLOCK/FRAME";
    /** PatternOverlayBlock output (known-pattern window overlaid on the live frame). */
    public static final String PATTERN_OVERLAY_BLOCK_FRAME = "PATTERN_OVERLAY_BLOCK/FRAME";
    /** ToneBlock frame output (auto-derived from block tensorNs). */
    public static final String TONE_BLOCK_FRAME = "TONE_BLOCK/FRAME";
    /** ToneCurveBlock frame output (auto-derived from block tensorNs). */
    public static final String TONE_CURVE_BLOCK_FRAME = "TONE_CURVE_BLOCK/FRAME";
    /** ToneStatsBlock frame output (auto-derived from block tensorNs). */
    public static final String TONE_STATS_BLOCK_FRAME = "TONE_STATS/FRAME";
    /** UnsharpBlock frame output (auto-derived from block tensorNs). */
    public static final String UNSHARP_BLOCK_FRAME = "UNSHARP_BLOCK/FRAME";
    /** VignettingBlock frame output (auto-derived from block tensorNs). */
    public static final String VIGNETTING_BLOCK_FRAME = "VIGNETTING_BLOCK/FRAME";
    /** VignettingMinBlock frame output (auto-derived from block tensorNs). */
    public static final String VIGNETTING_MIN_BLOCK_FRAME = "VIGNETTING_MIN_BLOCK/FRAME";
    /** WarpBlock frame output (auto-derived from block tensorNs). */
    public static final String WARP_BLOCK_FRAME = "WARP_BLOCK/FRAME";
    /** WaveletDenoiseBlock frame output (auto-derived from block tensorNs). */
    public static final String WAVELET_DENOISE_BLOCK_FRAME = "WAVELET_DENOISE_BLOCK/FRAME";
    /** WbGainsBlock frame output (auto-derived from block tensorNs). */
    public static final String WB_GAINS_BLOCK_FRAME = "WB_GAINS_BLOCK/FRAME";
    /** YuvBccBlock frame output (auto-derived from block tensorNs). */
    public static final String YUV_BCC_BLOCK_FRAME = "YUV_BCC_BLOCK/FRAME";
    /** YuvSatBlock frame output (auto-derived from block tensorNs). */
    public static final String YUV_SAT_BLOCK_FRAME = "YUV_SAT_BLOCK/FRAME";
    /** ZoneStatsBlock frame output (auto-derived from block tensorNs). */
    public static final String ZONE_STATS_BLOCK_FRAME = "ZONE_STATS/FRAME";

    /** All known tensor names for validation */
    public static final java.util.Set<String> ALL_TENSOR_NAMES = java.util.Set.of(RAW_INPUT,
        NORMALIZE_OUTPUT,
        NORMALIZE_SENSOR_MAX,
        NORMALIZE_BLACK_LEVEL,
        BLC_OUTPUT,
        BLC_PATTERN,
        RAW_BLC_TILED,
        CROP_BLC_TILED,
        BAYER_WB_GAINS,
        BAYER_WB_OUTPUT,
        CCM_MATRIX,
        CCM_OUTPUT,
        DEMOSAIC_CCM_WEIGHTS,
        DEMOSAIC_CCM_BIAS,
        DEMOSAIC_CCM_OUTPUT,
        TONE_LUT,
        TONE_OUTPUT,
        TONE_RGB_LUT,
        TONE_CONTRAST,
        TONE_BRIGHTNESS,
        TONE_SHADOW_LIFT,
        TONE_HIGHLIGHT_ROLL,
        TONE_GAMMA_RECIP,
        TONE_GAMMAD,
        TONE_GAMMA_WH,
        LSC_GRID,
        LSC_GRID_TILED,
        LSC_OUTPUT,
        VIGNETTING_MAP,
        VIGNETTING_OUTPUT,
        NR_STRENGTH,
        NR_OUTPUT,
        SHARPEN_STRENGTH,
        SHARPEN_OUTPUT,
        EE_GAIN,
        FCS_GAIN,
        LDCI_STRENGTH,
        DEMOSAIC_OUTPUT,
        DEMOSAIC_G2_CCM_MATRIX,
        CCM_INTERP_MATRIX,
        BAYER_PATTERN,
        HDR_MERGE_WEIGHT,
        WARP_GRID,
        HDR_OUTPUT,
        WARP_OUTPUT,
        HDR_NEUTRAL,
        HDR_OVER,
        HDR_DISP_UNDER,
        HDR_DISP_OVER,
        AE_STATS,
        AWB_STATS,
        AF_STATS,
        ALGO_AE_EXPOSURE_GAIN,
        ALGO_AE_MEAN_LUMINANCE,
        ALGO_AE_TARGET_LUMINANCE,
        ALGO_AE_STATS,
        ALGO_AWB_GAINS,
        ALGO_CCT_CCT,
        ALGO_GAMMA_OUTPUT,
        UNIFIED_STATS,
        DISPLAY_FRAME,
        UNIFIED_STATS_FRAME,
        ISP_META,
        FRAME_INDEX,
        MNN_BLC_OUTPUT,
        MNN_CCM_OUTPUT,
        MNN_TONE_OUTPUT,
        MNN_DEMOSAIC_OUTPUT,
        MNN_BAYER_WB_OUTPUT,
        YUV_BRIGHTNESS,
        YUV_CONTRAST,
        YUV_SATURATION,
        ALGO_GAMMA_EXPOSURE_GAIN,
        ALGO_GAMMA_ANALOG_GAIN,
        ALGO_GAMMA_SCENE_CHANGE,
        AWB_GAINS,
        CA_GRID_R,
        CA_GRID_B,
        BILATERAL_STRENGTH,
        AUTO_CONTRAST_STRENGTH,
        AUTO_CONTRAST_BRIGHTNESS,
        UNSHARP_SHARPNESS,
        LOCAL_CONTRAST_STRENGTH,
        TNR_PREV_FRAME,
        AAF_BLOCK_FRAME,
        STATS_DEMOSAIC_FRAME,
        AUTO_CONTRAST_BLOCK_FRAME,
        BAD_PIXEL_BLOCK_FRAME,
        BAD_PIXEL_LSC_BAYER_WB_BLOCK_FRAME,
        BAYER_WB_BLOCK_FRAME,
        BILATERAL_BLOCK_FRAME,
        BNF_BLOCK_FRAME,
        CALIBRATION_BLOCK_FRAME,
        CCM_INTERP_BLOCK_FRAME,
        CEH_BLOCK_FRAME,
        CFA_BLOCK_FRAME,
        CHROMATIC_ABERRATION_BLOCK_FRAME,
        CHROMATIC_ABERRATION_MIN_BLOCK_FRAME,
        CNF_BLOCK_FRAME,
        COLOR_SPACE_BLOCK_FRAME,
        CROP_BLC_BLOCK_FRAME,
        CROP_BLC_NORM_CFA_BLOCK_FRAME,
        CROP_BLOCK_FRAME,
        CSC_BLOCK_FRAME,
        DEMOSAIC_CCM_BLOCK_FRAME,
        DEMOSAIC_EDGE_BLOCK_FRAME,
        DEMOSAIC_EDGE_CCM_BLOCK_FRAME,
        DEMOSAIC_G2_CCM_BLOCK_FRAME,
        DEMOSAIC_HQ_LINEAR_BLOCK_FRAME,
        DEMOSAIC_LINEAR_BLOCK_FRAME,
        DEMOSAIC_MALVAR_BLOCK_FRAME,
        DEMOSAIC_MALVAR_CCM_BLOCK_FRAME,
        DEMOSAIC_QUALITY_FRAME,
        DOWNSCALE_BLOCK_FRAME,
        EE_BLOCK_FRAME,
        FAST_EE_BLOCK_FRAME,
        FCS_BLOCK_FRAME,
        FOCUS_BLOCK_FRAME,
        GAMMA_BLOCK_FRAME,
        GPU_WARP_BLOCK_FRAME,
        HDR_MERGE_BLOCK_FRAME,
        HISTOGRAM_BLOCK_FRAME,
        IDENTITY_BLOCK_FRAME,
        LAPLACIAN_PYRAMID_BLOCK_FRAME,
        LDCI_BLOCK_FRAME,
        LOCAL_CONTRAST_BLOCK_FRAME,
        LSC_BLOCK_FRAME,
        LSC_MIN_BLOCK_FRAME,
        LSC_STRENGTH,
        PIPELINE_STAGES,
        DEBUG_TAP,
        NCHW_TO_RGBA_BLOCK_FRAME,
        NHWC_TO_ARGB_BLOCK_FRAME,
        NLM_BLOCK_FRAME,
        NORMALIZE_BLOCK_FRAME,
        PIPELINE_VERIFIER_FRAME,
        RAW_BLC_BLOCK_FRAME,
        RAW_BLC_MIN_BLOCK_FRAME,
        RAW_PROCESSING_BLOCK_FRAME,
        REF_BAYER_WB_BLOCK_FRAME,
        REF_CCM_BLOCK_FRAME,
        REF_CNF_BLOCK_FRAME,
        REF_CSC_BLOCK_FRAME,
        REF_DEMOSAIC_BLOCK_FRAME,
        REF_TONE_BLOCK_FRAME,
        REF_YUV_BCC_BLOCK_FRAME,
        REF_YUV_SAT_BLOCK_FRAME,
        SATURATION_BLOCK_FRAME,
        SINK_BLOCK_FRAME,
        STATS_BLOCK_FRAME,
        TEMPORAL_DENOISE_BLOCK_FRAME,
        TEST_PATTERN_BLOCK_FRAME,
        PATTERN_OVERLAY_BLOCK_FRAME,
        TONE_BLOCK_FRAME,
        TONE_CURVE_BLOCK_FRAME,
        TONE_STATS_BLOCK_FRAME,
        UNSHARP_BLOCK_FRAME,
        VIGNETTING_BLOCK_FRAME,
        VIGNETTING_MIN_BLOCK_FRAME,
        WARP_BLOCK_FRAME,
        WAVELET_DENOISE_BLOCK_FRAME,
        WB_GAINS_BLOCK_FRAME,
        YUV_BCC_BLOCK_FRAME,
        YUV_SAT_BLOCK_FRAME,
        ZONE_STATS_BLOCK_FRAME);

    /** Get element type for a known tensor name, or -1 if unknown */
    public static int getElemType(String name) {
        return switch (name) {
            case RAW_INPUT -> RAW_INPUT_ELEM_TYPE;
            case NORMALIZE_SENSOR_MAX -> NORMALIZE_OUTPUT_ELEM_TYPE;
            case NORMALIZE_BLACK_LEVEL -> NORMALIZE_OUTPUT_ELEM_TYPE;
            case BLC_PATTERN -> BLC_PATTERN_ELEM_TYPE;
            case DOWNSCALE_HW -> DOWNSCALE_HW_ELEM_TYPE;
            case DEMOSAIC_BAYER_PATTERN -> DEMOSAIC_BAYER_PATTERN_ELEM_TYPE;
            case RAW_BLC_TILED -> RAW_BLC_TILED_ELEM_TYPE;
            case CROP_BLC_TILED -> CROP_BLC_TILED_ELEM_TYPE;
            case BAYER_WB_GAINS -> BAYER_WB_GAINS_ELEM_TYPE;
            case CCM_MATRIX -> CCM_MATRIX_ELEM_TYPE;
            case DEMOSAIC_CCM_WEIGHTS -> DEMOSAIC_CCM_WEIGHTS_ELEM_TYPE;
            case TONE_LUT -> TONE_LUT_ELEM_TYPE;
            case TONE_RGB_LUT -> TONE_RGB_LUT_ELEM_TYPE;
            case TONE_CONTRAST -> TONE_ELEM_TYPE;
            case TONE_BRIGHTNESS -> TONE_ELEM_TYPE;
            case TONE_SHADOW_LIFT -> TONE_ELEM_TYPE;
            case TONE_HIGHLIGHT_ROLL -> TONE_ELEM_TYPE;
            case TONE_GAMMA_RECIP -> TONE_ELEM_TYPE;
            case TONE_GAMMAD -> TONE_GAMMAD_ELEM_TYPE;
            case LSC_STRENGTH -> LSC_STRENGTH_ELEM_TYPE;
            case PIPELINE_STAGES -> PIPELINE_STAGES_ELEM_TYPE;
            case DEBUG_TAP -> DEBUG_TAP_ELEM_TYPE;
            case LSC_GRID -> LSC_GRID_ELEM_TYPE;
            case LSC_GRID_TILED -> LSC_GRID_TILED_ELEM_TYPE;
            case VIGNETTING_MAP -> VIGNETTING_MAP_ELEM_TYPE;
            case NR_STRENGTH -> NR_STRENGTH_ELEM_TYPE;
            case SHARPEN_STRENGTH -> SHARPEN_STRENGTH_ELEM_TYPE;
            case SATURATION_STRENGTH -> SATURATION_STRENGTH_ELEM_TYPE;
            case EE_GAIN -> EE_GAIN_ELEM_TYPE;
            case FCS_GAIN -> FCS_GAIN_ELEM_TYPE;
            case LDCI_STRENGTH -> LDCI_STRENGTH_ELEM_TYPE;
            case HDR_MERGE_WEIGHT -> HDR_MERGE_WEIGHT_ELEM_TYPE;
            case WARP_GRID -> WARP_GRID_ELEM_TYPE;
            case AE_STATS -> AE_STATS_ELEM_TYPE;
            case AWB_STATS -> AWB_STATS_ELEM_TYPE;
            case AF_STATS -> AF_STATS_ELEM_TYPE;
            case UNIFIED_STATS -> UNIFIED_STATS_ELEM_TYPE;
            case DISPLAY_FRAME -> DISPLAY_FRAME_ELEM_TYPE;
            case ISP_META -> ISP_META_ELEM_TYPE;
            case FRAME_INDEX -> FRAME_INDEX_ELEM_TYPE;
            case CCM_INTERP_MATRIX -> CCM_INTERP_MATRIX_ELEM_TYPE;
            case DEMOSAIC_G2_CCM_MATRIX -> DEMOSAIC_G2_CCM_ELEM_TYPE;
            default -> -1;
        };
    }

    /** Check if a tensor name is a known ISP tensor */
    public static boolean isKnownTensor(String name) {
        return ALL_TENSOR_NAMES.contains(name);
    }

    /** Validate that all provided names are known */
    public static java.util.List<String> validateNames(java.lang.Iterable<String> names) {
        java.util.List<String> result = new java.util.ArrayList<>();
        for (String name : names) {
            if (!ALL_TENSOR_NAMES.contains(name)) {
                result.add(name);
            }
        }
        return result;
    }

    // Prevent instantiation
    private TensorNames() {}
}