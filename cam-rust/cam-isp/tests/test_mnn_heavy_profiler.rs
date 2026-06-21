//! Test heavy pipeline profiling with MNN
//! Uses profiler-enabled libMNN.so (MNN_PIPELINE_PROFILE=ON)

#[cfg(test)]
mod tests {

    #[test]
    #[ignore = "Heavy test for profiling"]
    fn test_heavy_pipeline_profiler() {
        // Create a test pipeline with multiple blocks
        use cam_isp::pipeline::GraphComposer;
        use cam_isp::blocks;

        let unpack = blocks::UnpackCfaBlock::new();
        let demosaic = blocks::DemosaicCcmBlock::new(0);  // RGGB bayer pattern
        let tone = blocks::ToneBlock::new();
        let ee = blocks::EeBlock::new();
        let ldci = blocks::LdciBlock::new();
        let display = blocks::DisplayBlock::new(1920);   // FHD target width

        let blocks_vec: Vec<&dyn cam_isp::pipeline::IspBlock> = vec![
            &unpack, &demosaic, &tone, &ee, &ldci, &display,
        ];

        // Compose ONNX graph
        println!("Composing ONNX pipeline...");
        let _onnx = GraphComposer::compose_from_vec(&blocks_vec, &[], 15)
            .expect("Failed to compose ONNX pipeline");

        println!("Heavy pipeline composed successfully");
    }
}
