//! Test heavy pipeline profiling with MNN
//! Uses profiler-enabled libMNN.so (MNN_PIPELINE_PROFILE=ON)

#[cfg(test)]
mod tests {
    use crate::engine::*;
    use crate::pipeline::*;
    use std::time::Instant;

    #[test]
    #[ignore = "Heavy test for profiling"]
    fn test_heavy_pipeline_profiler() {
        // Create a test pipeline with multiple blocks
        let mut pipeline = Pipeline::new();
        
        // Add a heavy pipeline similar to the real ISP pipeline
        // RawInput -> UnpackCfa -> Demosaic -> CCM -> Tone -> EE -> LDCI -> Display
        pipeline.add_block(crate::blocks::RawInputBlock::new("raw_input"));
        pipeline.add_block(crate::blocks::UnpackCfaBlock::new("unpack_cfa"));
        pipeline.add_block(crate::blocks::DemosaicCcmBlock::new("demosaic_ccm"));
        pipeline.add_block(crate::blocks::ToneBlock::new("tone"));
        pipeline.add_block(crate::blocks::EeBlock::new("ee"));
        pipeline.add_block(crate::blocks::LdciBlock::new("ldci"));
        pipeline.add_block(crate::blocks::DisplayBlock::new("display"));

        // Create a test frame
        let width = 1920;
        let height = 1080;
        let frame = crate::Frame::new_raw(width, height, crate::FrameFormat::BayerRGGB16);

        // Create engine with MNN backend
        let backend = BackendType::MNN;
        let mut engine = CpuEngine::new(backend);

        // Process the frame
        let start = Instant::now();
        let result = engine.process(&pipeline, &frame);
        let elapsed = start.elapsed();

        println!("Pipeline processed in: {:?}", elapsed);
        assert!(result.is_ok());

        // TODO: Access profiler data via FFI
        // For now, the profiler data is printed by MNN internally
        // when MNN_PIPELINE_PROFILE=ON
    }
}
