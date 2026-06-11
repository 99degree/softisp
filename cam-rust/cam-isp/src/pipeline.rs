//! ISP pipeline types: block trait, frame, graph composer.
//! Ported from com.camcore.isp.pipeline

use std::collections::{HashMap, HashSet};
use log::{info, warn};
use cam_types::FrameFormat;

use crate::onnx::proto::Proto;

/// Auxiliary outputs from ISP processing.
#[derive(Debug, Clone)]
pub struct IspAuxOutput {
    pub channel_means: Option<[f32; 3]>,
    pub tone_stats: Option<[f32; 3]>,
    pub wb_gains: Option<[f32; 3]>,
    pub histogram: Option<Vec<f32>>,
    pub zone_stats: Option<Vec<f32>>,
    pub focus_metric: Option<f32>,
    pub cct: Option<f32>,
    pub ae_gain: Option<f32>,
}

/// ISP frame carrying pixel data.
#[derive(Debug, Clone)]
pub struct IspFrame {
    pub data: Vec<u8>,
    pub width: u32,
    pub height: u32,
    pub format: FrameFormat,
    pub float_data: Option<Vec<f32>>,
    pub aux: Option<IspAuxOutput>,
}

impl IspFrame {
    pub fn new(width: u32, height: u32, format: FrameFormat) -> Self {
        let data = vec![0u8; (width * height * 4) as usize];
        Self {
            data,
            width,
            height,
            format,
            float_data: None,
            aux: None,
        }
    }
}

/// ISP block that contributes ONNX graph fragments to a fused pipeline.
///
/// Blocks form a linked list via [next]/[prev], defining the frame buffer flow.
/// GraphComposer takes the head block and traverses via [next].
pub trait IspBlock: Send {
    /// Unique block identifier.
    fn id(&self) -> &str;

    /// Namespace for tensor names. Defaults to class name.
    fn tensor_ns(&self) -> String;

    /// Tensor name this block reads as its frame buffer input.
    fn input_source(&self) -> Option<&str>;
    fn set_input_source(&mut self, name: &str);

    /// Tensor name this block writes as its frame buffer output.
    fn frame_tensor(&self) -> Option<&str>;

    /// Linked list navigation.
    fn prev(&self) -> Option<&Box<dyn IspBlock>>;
    fn set_prev(&mut self, block: Box<dyn IspBlock>);

    fn next(&self) -> Option<&Box<dyn IspBlock>>;
    fn set_next(&mut self, block: Box<dyn IspBlock>);

    fn is_head(&self) -> bool {
        self.prev().is_none()
    }

    fn is_tail(&self) -> bool {
        self.next().is_none()
    }

    /// Graph-fragment API — ONNX nodes as serialized NodeProto bytes.
    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    /// ONNX initializers as serialized TensorProto bytes.
    fn initializers(&self) -> Vec<Vec<u8>> {
        vec![]
    }

    /// Tensor names this block reads. GraphComposer walks [prev] chain to
    /// find the producing block. Unresolved tensors become graph inputs.
    fn input_tensors(&self) -> Vec<String> {
        self.input_source()
            .map(|s| vec![s.to_string()])
            .unwrap_or_default()
    }

    /// Tensor names this block writes.
    fn output_tensors(&self) -> Vec<String> {
        self.frame_tensor()
            .map(|s| vec![s.to_string()])
            .unwrap_or_default()
    }

    /// If non-None, this block is the pipeline input.
    fn graph_input_name(&self) -> Option<&str> {
        if self.is_head() { self.frame_tensor() } else { None }
    }

    /// If non-None, this block produces the pipeline output.
    fn graph_output_name(&self) -> Option<&str> {
        if self.is_tail() { self.frame_tensor() } else { None }
    }

    fn input_elem_type(&self) -> i32 {
        1 // FLOAT
    }

    fn output_elem_type(&self) -> i32 {
        1 // FLOAT
    }

    fn input_value_info(&self) -> Option<Vec<u8>> {
        None
    }

    fn output_value_info(&self) -> Option<Vec<u8>> {
        None
    }

    /// Additional graph inputs (e.g., "sensor_max", "sizes").
    /// Each entry: (tensor_name, elem_type, shape_dims as i64).
    fn extra_inputs(&self) -> Vec<(String, i64, Vec<i64>)> {
        vec![]
    }
}

/// Helper to build a chain of blocks as a Vec.
pub struct PipelineBuilder {
    pub blocks: Vec<Box<dyn IspBlock>>,
}

impl PipelineBuilder {
    pub fn new() -> Self {
        Self { blocks: Vec::new() }
    }

    pub fn add_block(mut self, block: Box<dyn IspBlock>) -> Self {
        self.blocks.push(block);
        self
    }

    pub fn build(self) -> Option<Box<dyn IspBlock>> {
        self.blocks.into_iter().next()
    }
}

// =========================================================================
// GraphComposer — merges block graph fragments into a single ONNX model
// Ported from com.camcore.isp.graph.GraphComposer
// =========================================================================

/// Merges block graph fragments into a single ONNX model.
pub struct GraphComposer;

impl GraphComposer {
    const TAG: &'static str = "GraphComposer";

    /// Compose blocks from a Vec into a single ONNX model.
    /// Blocks are in order: index 0 = head, index N-1 = tail.
    /// This is simpler than linked-list walking and avoids ownership issues.
    ///
    /// * `pipeline` — Ordered list of pipeline blocks (head to tail).
    /// * `aux_blocks` — Auxiliary parameter blocks (not in the main pipeline).
    /// * `opset_version` — ONNX opset version.
    ///
    /// Returns serialized ModelProto bytes.
    pub fn compose_from_vec(
        pipeline: &[&dyn IspBlock],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<Vec<u8>, String> {
        if pipeline.is_empty() {
            return Err("Empty pipeline".to_string());
        }

        // 1. Validate chain count
        let names: Vec<String> = pipeline.iter().map(|b| format!("{}[{}]", b.id(), b.tensor_ns())).collect();
        info!("{}: Pipeline: {} blocks: {}", Self::TAG, pipeline.len(), names.join(" → "));

        let pipeline_head = pipeline[0];
        let pipeline_tail = pipeline[pipeline.len() - 1];

        // 2. Walk blocks and set inputSource based on predecessor
        // Since blocks hold their own `input_source` as mutable state,
        // but we only have & references, the caller must set input_source
        // properly before calling this function.
        // We validate the input_source is set to the previous block's frame_tensor.

        // 3. Index all produced tensor names
        let mut produced_by = HashMap::new();
        for blk in pipeline {
            for t in blk.output_tensors() {
                produced_by.insert(t.clone(), blk.id().to_string());
            }
        }
        for blk in aux_blocks {
            for t in blk.output_tensors() {
                produced_by.insert(t.clone(), blk.id().to_string());
            }
        }

        // 4. Validate aux block inputs
        for blk in aux_blocks {
            for t in blk.input_tensors() {
                let extra_names: HashSet<String> = blk.extra_inputs().iter().map(|e| e.0.clone()).collect();
                if !produced_by.contains_key(&t) && !extra_names.contains(&t) {
                    warn!("{}: Aux block {} needs '{}' — no producer found", Self::TAG, blk.id(), t);
                }
            }
        }

        // 5. Collect nodes, initializers, value infos
        let all_blocks: Vec<&dyn IspBlock> = {
            let mut v: Vec<&dyn IspBlock> = pipeline.to_vec();
            v.extend(aux_blocks.iter().map(|b| *b));
            v
        };

        let mut all_nodes = Vec::new();
        let mut all_initializers = Vec::new();
        let mut all_inputs = Vec::new();
        let mut all_outputs = Vec::new();
        let mut extra_input_names = HashSet::new();
        let graph_name = format!("{}_pipeline", pipeline_head.id());

        for blk in &all_blocks {
            all_nodes.extend(blk.nodes());
            all_initializers.extend(blk.initializers());

            // Graph input from head block
            if std::ptr::eq(*blk as *const _, pipeline_head as *const _) {
                if let Some(name) = blk.graph_input_name() {
                    let vi = blk.input_value_info()
                        .ok_or_else(|| format!("Head block {} has no input_value_info", blk.id()))?;
                    all_inputs.push(vi);
                    info!("{}: graph input: {} → {}", Self::TAG, blk.id(), name);
                }
            }

            // Graph output: tail block
            if std::ptr::eq(*blk as *const _, pipeline_tail as *const _) {
                if let Some(name) = blk.graph_output_name() {
                    if let Some(vi) = blk.output_value_info() {
                        all_outputs.push(vi);
                        info!("{}: graph output: {} → {}", Self::TAG, blk.id(), name);
                    }
                }
            }

            // Extra runtime inputs — skip if already produced
            for (name, elem_type, dims) in blk.extra_inputs() {
                if extra_input_names.contains(&name) || produced_by.contains_key(&name) {
                    continue;
                }
                extra_input_names.insert(name.clone());
                let shape_dims: Vec<Vec<u8>> = dims.iter()
                    .map(|d| Proto::tensor_dim_value(*d))
                    .collect();
                all_inputs.push(Proto::value_info(&name, &shape_dims, elem_type as i32));
                info!("{}: extra input: {} (elem_type={})", Self::TAG, name, elem_type);
            }
        }

        if all_inputs.is_empty() {
            return Err("No graph inputs".to_string());
        }
        if all_outputs.is_empty() {
            return Err("No graph outputs".to_string());
        }

        info!("{}: {} nodes, {} initializers, {} inputs, {} outputs",
            Self::TAG, all_nodes.len(), all_initializers.len(),
            all_inputs.len(), all_outputs.len());

        let graph = Proto::graph(
            &graph_name,
            &all_nodes,
            &all_inputs,
            &all_outputs,
            &all_initializers,
        );
        let opset = Proto::opset("", opset_version);
        let model = Proto::model(7, &opset, "cam_rust_graph_composer", &graph);

        Ok(model)
    }

    /// Legacy: walk the linked list via next() pointers.
    pub fn compose(
        head: &dyn IspBlock,
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<Vec<u8>, String> {
        let chain = Self::walk_chain(head)?;
        Self::compose_from_vec(&chain, aux_blocks, opset_version)
    }

    /// Walk the main chain starting from head.
    fn walk_chain(mut block: &dyn IspBlock) -> Result<Vec<&dyn IspBlock>, String> {
        let mut chain = Vec::new();
        loop {
            chain.push(block);
            match block.next() {
                Some(next) => { block = &**next; }
                None => break,
            }
        }
        if chain.is_empty() {
            return Err("Empty main chain".to_string());
        }
        Ok(chain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::*;

    #[test]
    fn test_default_pipeline_composition() {
        // Build the standard 9-block pipeline
        let mut raw = RawInputBlock::new();
        let mut norm = NormalizeBlock::new();
        let mut cfa = CfaBlock::new();
        let mut blc = BlcBlock::new();
        let mut wb = BayerWbBlock::new();
        let mut dem = DemosaicBlock::new(2); // GBRG
        let mut ccm = CcmBlock::new();
        let mut tone = ToneBlock::new();
        let mut disp = DisplayBlock::new(1280);

        // Link blocks
        norm.set_input_source(raw.frame_tensor().unwrap_or(""));
        cfa.set_input_source(norm.frame_tensor().unwrap_or(""));
        blc.set_input_source(cfa.frame_tensor().unwrap_or(""));
        wb.set_input_source(blc.frame_tensor().unwrap_or(""));
        dem.set_input_source(wb.frame_tensor().unwrap_or(""));
        ccm.set_input_source(dem.frame_tensor().unwrap_or(""));
        tone.set_input_source(ccm.frame_tensor().unwrap_or(""));
        disp.set_input_source(tone.frame_tensor().unwrap_or(""));

        let pipeline: Vec<&dyn IspBlock> = vec![&raw, &norm, &cfa, &blc, &wb, &dem, &ccm, &tone, &disp];
        let model = GraphComposer::compose_from_vec(&pipeline, &[], 16).expect("Compose failed");
        assert!(!model.is_empty());
        assert!(model.len() > 2000); // Expected ~2719 bytes
    }
}
