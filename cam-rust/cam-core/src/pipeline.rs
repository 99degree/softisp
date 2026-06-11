//! Pipeline manager for the ISP.
//! Ported from com.camcore.isp.IspProcessor

use log::info;
use cam_isp::pipeline::IspBlock;
use cam_isp::blocks;

/// Pipeline configuration.
#[derive(Clone)]
pub struct PipelineConfig {
    pub target_width: u32,
    pub opset_version: i64,
    pub bayer_pattern: i32,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            target_width: 1280,
            opset_version: 16,
            bayer_pattern: 2, // GBRG (common on Qualcomm)
        }
    }
}

/// Manages the ISP pipeline chain.
pub struct PipelineManager {
    pub config: PipelineConfig,
    chain: Option<Vec<Box<dyn IspBlock>>>,
    head: Option<Box<dyn IspBlock>>,
}

impl PipelineManager {
    pub fn new(config: PipelineConfig) -> Self {
        Self {
            config,
            chain: None,
            head: None,
        }
    }

    /// Build the default ISP pipeline chain.
    pub fn build_default_chain(&mut self) -> Result<(), String> {
        let raw_input = Box::new(blocks::RawInputBlock::new());
        let normalize = Box::new(blocks::NormalizeBlock::new());
        let cfa = Box::new(blocks::CfaBlock::new());
        let blc = Box::new(blocks::BlcBlock::new());
        let bayer_wb = Box::new(blocks::BayerWbBlock::new());
        let demosaic = Box::new(blocks::DemosaicBlock::new(self.config.bayer_pattern));
        let ccm = Box::new(blocks::CcmBlock::new());
        let tone = Box::new(blocks::ToneBlock::new());
        let display = Box::new(blocks::DisplayBlock::new(self.config.target_width));

        let blocks: Vec<Box<dyn IspBlock>> = vec![
            raw_input,
            normalize,
            cfa,
            blc,
            bayer_wb,
            demosaic,
            ccm,
            tone,
            display,
        ];

        self.chain = Some(blocks);
        info!("Built pipeline chain: {} blocks", self.chain.as_ref().map(|c| c.len()).unwrap_or(0));
        Ok(())
    }

    /// Get the pipeline head block.
    pub fn get_head(&mut self) -> Option<Box<dyn IspBlock>> {
        self.chain.as_mut().and_then(|chain| {
            if chain.is_empty() {
                return None;
            }
            // Take ownership of the first block
            Some(chain.remove(0))
        })
    }

    /// Get the full chain of blocks.
    pub fn get_blocks(&mut self) -> Vec<Box<dyn IspBlock>> {
        self.chain.take().unwrap_or_default()
    }
}
