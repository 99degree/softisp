//! Motion processing module for the ISP pipeline.
//! Ported from com.camcore.isp.motion

use cam_types::Frame;

/// Motion estimation result.
#[derive(Debug, Clone)]
pub struct MotionVectors {
    pub global: (f32, f32),  // (dx, dy)
    pub local: Vec<(f32, f32)>,
    pub confidence: f32,
}

/// Motion compensator for handshake stabilization (EIS).
pub struct MotionCompensator {
    accumulated_vectors: Vec<MotionVectors>,
}

impl MotionCompensator {
    pub fn new() -> Self {
        Self {
            accumulated_vectors: Vec::new(),
        }
    }

    /// Estimate motion between two consecutive frames.
    pub fn estimate_motion(
        &self,
        _prev_frame: &Frame,
        _curr_frame: &Frame,
    ) -> MotionVectors {
        // TODO: Implement feature-based or optical flow motion estimation.
        MotionVectors {
            global: (0.0, 0.0),
            local: vec![],
            confidence: 0.0,
        }
    }

    /// Apply motion compensation to warp the image.
    pub fn compensate(
        &self,
        _frame: &mut Frame,
        _vectors: &MotionVectors,
    ) {
        // TODO: Apply affine transformation for EIS.
    }
}
