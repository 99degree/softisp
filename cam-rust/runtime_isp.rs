#![allow(unused)]
use cam_isp::{BayerPattern, blocks::{UnpackBlock, BayerProcBlock, DemosaicBlock, DisplayBlock}, pipeline::PipelineBuilder};

fn main() {
    let mut pipeline = PipelineBuilder::new();
    pipeline.add_block(Box::new(UnpackBlock::new().with_bayer_pattern(BayerPattern::RGGB)))
            .add_block(Box::new(DemosaicBlock::new()))
            .add_block(Box::new(DisplayBlock::new(1920)));
    
    println!("✅ Runtime ISP pipeline ready");
}
