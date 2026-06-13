fn main() {
    use cam_isp::pipeline::{IspBlock, GraphComposer};
    use cam_isp::blocks::*;
    use cam_isp::profile::PipelineProfile;
    
    // Test subsets of HEAVY profile
    // 1. Full HEAVY
    let blocks = PipelineProfile::HEAVY.build_blocks(8, 2);
    let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
    GraphComposer::compose_from_vec(&refs, &[], 16).unwrap();
    
    // 2. HEAVY without unsharp
    // 3. HEAVY without unsharp and ldci
}
