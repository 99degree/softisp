//! ISP pipeline types: block trait, frame, graph composer.
//!
//! This module defines the core types for the ISP processing pipeline:
//! - `IspBlock` trait: interface for processing blocks
//! - `IspFrame`: output frame from the pipeline
//! - `GraphComposer`: builds ONNX models from block chains
//!
//! # Pipeline Architecture
//!
//! ```text
//! Raw Input → Block1 → Block2 → ... → BlockN → Output
//!     ↓
//!   Stats (AE, AWB, etc.)
//! ```

use cam_types::FrameFormat;
use log::{info, warn};
use std::collections::{HashMap, HashSet};

use crate::isp_params::IspParams;
use crate::onnx::proto::Proto;

/// Auxiliary outputs from ISP processing.
///
/// Contains statistics and metadata extracted during pipeline processing.
/// These are used for 3A (AE, AWB, AF) control and diagnostics.
///
/// # Usage
///
/// ```rust,ignore
/// let frame = engine.process(&params)?;
/// if let Some(aux) = frame.aux_output {
///     println!("AE gain: {:?}", aux.ae_gain);
///     println!("AWB gains: {:?}", aux.wb_gains);
/// }
/// ```
#[derive(Debug, Clone, Default)]
pub struct IspAuxOutput {
    /// RGB channel means for AWB statistics
    pub channel_means: Option<[f32; 3]>,
    /// Tone curve statistics
    pub tone_stats: Option<[f32; 3]>,
    /// White balance gains [R, G, B]
    pub wb_gains: Option<[f32; 3]>,
    /// RGB histogram (256 bins per channel)
    pub histogram: Option<Vec<f32>>,
    /// Zone statistics for AE metering
    pub zone_stats: Option<Vec<f32>>,
    /// Focus metric (higher = sharper)
    pub focus_metric: Option<f32>,
    /// Correlated color temperature (Kelvin)
    pub cct: Option<f32>,
    /// Auto-exposure gain
    pub ae_gain: Option<f32>,
    /// Calibration statistics `[24]` from quad-level Bayer analysis.
    pub calibration_stats: Option<[f32; 24]>,
    /// Scene classification for adaptive ISP.
    pub scene_category: Option<String>,
    /// AF phase display string.
    pub af_phase: Option<String>,
    /// Current VCM lens position.
    pub vcm_position: Option<i32>,
    /// EIS compensation `[dx_px, dy_px, roll_deg]`.
    pub eis_compensation: Option<[f32; 3]>,
}

/// ISP frame carrying pixel data.
///
/// Output from `IspEngine::process()`. Contains the processed image
/// and optional auxiliary outputs (statistics, metadata).
///
/// # Layout
///
/// ```text
/// IspFrame {
///     data: [u8]           // Pixel data (format depends on OutputFormat)
///     width: u32           // Width in pixels
///     height: u32          // Height in pixels
///     format: FrameFormat  // Pixel format
///     float_data: Option   // Raw float data (if FloatRgb/FloatBgra)
///     aux: Option          // Statistics and metadata
///     timestamp_ns: u64    // Sensor capture timestamp
///     *_duration_ns: u64   // Performance timing
/// }
/// ```
#[derive(Debug, Clone)]
pub struct IspFrame {
    /// Processed pixel data
    pub data: Vec<u8>,
    /// Width in pixels
    pub width: u32,
    /// Height in pixels
    pub height: u32,
    /// Pixel format of output data
    pub format: FrameFormat,
    /// Raw float data (only for FloatRgb/FloatBgra formats)
    pub float_data: Option<Vec<f32>>,
    /// Auxiliary outputs (statistics, metadata)
    pub aux: Option<IspAuxOutput>,
    /// Per-frame ISP parameters from controller (WB, CCM, tone, etc.)
    pub params: IspParams,
    /// Capture timestamp from sensor (ns since epoch, monotonic).
    pub timestamp_ns: u64,
    /// Frame preparation duration (ns) - time from capture to inference input ready.
    pub prep_duration_ns: u64,
    /// Inference duration (ns).
    pub inference_duration_ns: u64,
    /// Total pipeline duration (ns).
    pub total_duration_ns: u64,
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
            params: IspParams::default(),
            timestamp_ns: 0,
            prep_duration_ns: 0,
            inference_duration_ns: 0,
            total_duration_ns: 0,
        }
    }
}

/// ISP block that contributes ONNX graph fragments to a fused pipeline.
///
/// Blocks form a linked list via `[next]`/`[prev]`, defining the frame buffer flow.
/// GraphComposer takes the head block and traverses via `[next]`.
#[allow(clippy::borrowed_box)]
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

    /// Tensor names this block reads. GraphComposer walks `[prev]` chain to
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
        if self.is_head() {
            self.frame_tensor()
        } else {
            None
        }
    }

    /// If non-None, this block produces the pipeline output.
    /// Default: tail only. Override in specific blocks (stats, aux hooks) to always output.
    fn graph_output_name(&self) -> Option<&str> {
        if self.is_tail() {
            self.frame_tensor()
        } else {
            None
        }
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

    /// Auxiliary blocks this block depends on.
    ///
    /// Returns ids of aux blocks (e.g. `"fcs"`, `"ldci"`, `"ee"`)
    /// that must be inserted after this block in the pipeline.
    /// `build_blocks()` collects signals from all main blocks and only
    /// includes aux blocks that are both requested AND profile-allowed.
    fn signals_aux(&self) -> Vec<String> {
        vec![]
    }
}

/// Helper to build a chain of blocks as a Vec.
pub struct PipelineBuilder {
    pub blocks: Vec<Box<dyn IspBlock>>,
}

impl Default for PipelineBuilder {
    fn default() -> Self {
        Self::new()
    }
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

/// Statistics about a pipeline.
#[derive(Debug, Clone)]
pub struct PipelineStats {
    pub block_count: usize,
    pub block_names: Vec<String>,
    pub total_nodes: usize,
    pub total_initializers: usize,
    pub onnx_bytes: usize,
    pub estimated_flops: u64,
    pub estimated_memory_bytes: u64,
}

impl std::fmt::Display for PipelineStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Pipeline: {} blocks ({}), {} ops, {} params, {} bytes ONNX, {:.1} MFLOPs, {:.1} KB",
            self.block_count,
            self.block_names.join(" → "),
            self.total_nodes,
            self.total_initializers,
            self.onnx_bytes,
            self.estimated_flops as f64 / 1e6,
            self.estimated_memory_bytes as f64 / 1024.0,
        )
    }
}

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
        eprintln!(
            "GraphComposer::compose_from_vec: pipeline len={}, aux len={}",
            pipeline.len(),
            aux_blocks.len()
        );
        if pipeline.is_empty() {
            return Err("Empty pipeline".to_string());
        }

        // 1. Validate chain count
        let names: Vec<String> = pipeline
            .iter()
            .map(|b| format!("{}[{}]", b.id(), b.tensor_ns()))
            .collect();
        info!(
            "{}: Pipeline: {} blocks: {}",
            Self::TAG,
            pipeline.len(),
            names.join(" → ")
        );

        let pipeline_head = pipeline[0];
        let pipeline_tail = pipeline[pipeline.len() - 1];
        eprintln!(
            "GraphComposer::compose_from_vec: head={}, tail={}",
            pipeline_head.id(),
            pipeline_tail.id()
        );

        // 2. Walk blocks and set inputSource based on predecessor
        // Since blocks hold their own `input_source` as mutable state,
        // but we only have & references, the caller must set input_source
        // properly before calling this function.
        // We validate the input_source is set to the previous block's frame_tensor.

        // Validate pipeline (warnings only — callers may not call wire_blocks)
        let issues = Self::validate_pipeline(pipeline);
        if !issues.is_empty() {
            warn!(
                "{}: Pipeline validation ({} issues): {}",
                Self::TAG,
                issues.len(),
                issues.join("; ")
            );
        }

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
                let extra_names: HashSet<String> =
                    blk.extra_inputs().iter().map(|e| e.0.clone()).collect();
                if !produced_by.contains_key(&t) && !extra_names.contains(&t) {
                    warn!(
                        "{}: Aux block {} needs '{}' — no producer found",
                        Self::TAG,
                        blk.id(),
                        t
                    );
                }
            }
        }

        // 5. Collect nodes, initializers, value infos
        let all_blocks: Vec<&dyn IspBlock> = {
            let mut v: Vec<&dyn IspBlock> = pipeline.to_vec();
            v.extend(aux_blocks.iter().copied());
            v
        };

        // Collect all graph output names up front so we can skip their value_infos
        let mut graph_output_names: HashSet<String> = HashSet::new();
        for blk in &all_blocks {
            if let Some(name) = blk.graph_output_name() {
                graph_output_names.insert(name.to_string());
            }
        }

        let mut all_nodes = Vec::new();
        let mut all_initializers = Vec::new();
        let mut graph_inputs = Vec::new();
        let mut all_outputs = Vec::new();
        let mut value_infos = Vec::new();
        let mut extra_input_names = HashSet::new();
        let graph_name = format!("{}_pipeline", pipeline_head.id());

        for blk in &all_blocks {
            all_nodes.extend(blk.nodes());
            all_initializers.extend(blk.initializers());

            // Intermediate tensor value info for type inference (→ field 13)
            // Only for non-Identity blocks
            let is_identity = blk.id() == "normalize"
                || blk.id() == "cfa"
                || blk.id() == "blc"
                || blk.id() == "wb"
                || blk.id() == "ccm"
                || blk.id() == "tone"
                || blk.id() == "demosaic";
            if !is_identity {
                // Add value_info for all output tensors (except graph input/output)
                for tname in blk.output_tensors() {
                    let is_input = pipeline_head.graph_input_name().is_some_and(|n| n == tname);
                    let is_output = graph_output_names.contains(&tname);
                    if !is_input && !is_output {
                        value_infos.push(Proto::value_info(
                            &tname,
                            &[
                                Proto::tensor_dim_param("N"),
                                Proto::tensor_dim_param("C"),
                                Proto::tensor_dim_param("H"),
                                Proto::tensor_dim_param("W"),
                            ],
                            blk.output_elem_type(),
                        ));
                    }
                }
                // Add value_info for all input tensors that have a producer
                for tname in blk.input_tensors() {
                    if produced_by.contains_key(&tname) && !tname.is_empty() {
                        let is_input = pipeline_head.graph_input_name().is_some_and(|n| n == tname);
                        if !is_input {
                            value_infos.push(Proto::value_info(
                                &tname,
                                &[
                                    Proto::tensor_dim_param("N"),
                                    Proto::tensor_dim_param("C"),
                                    Proto::tensor_dim_param("H"),
                                    Proto::tensor_dim_param("W"),
                                ],
                                blk.input_elem_type(),
                            ));
                        }
                    }
                }
            }

            // Graph input from head block (→ field 11)
            if std::ptr::eq(*blk as *const _, pipeline_head as *const _) {
                if let Some(name) = blk.graph_input_name() {
                    let vi = blk.input_value_info().ok_or_else(|| {
                        format!("Head block {} has no input_value_info", blk.id())
                    })?;
                    graph_inputs.push(vi);
                    info!("{}: graph input: {} → {}", Self::TAG, blk.id(), name);
                }
            }

            // Graph output: all non-head blocks register outputs to prevent MNN DCE.
            // A single-block graph (head == tail) must still register its output.
            let is_tail = std::ptr::eq(*blk as *const _, pipeline_tail as *const _);
            let is_head = std::ptr::eq(*blk as *const _, pipeline_head as *const _);
            if let Some(name) = blk.graph_output_name() {
                if let Some(vi) = blk.output_value_info() {
                    if is_tail {
                        all_outputs.insert(0, vi);
                    } else if !is_head {
                        all_outputs.push(vi);
                    }
                    info!("{}: graph output: {} → {}", Self::TAG, blk.id(), name);
                }
            } else if is_tail && !is_head {
                if let Some(vi) = blk.output_value_info() {
                    all_outputs.insert(0, vi);
                    info!(
                        "{}: graph output (tail): {} → {}",
                        Self::TAG,
                        blk.id(),
                        blk.frame_tensor().unwrap_or("?")
                    );
                }
            }

            // Extra runtime inputs — add to graph inputs even if also an initializer
            // (ONNX allows a tensor in both field 5 initializers and field 11 inputs;
            // the input value overrides the initializer at runtime.)
            for (name, elem_type, dims) in blk.extra_inputs() {
                if extra_input_names.contains(&name) {
                    continue; // Already registered (dedup across blocks)
                }
                extra_input_names.insert(name.clone());
                let shape_dims: Vec<Vec<u8>> =
                    dims.iter().map(|d| Proto::tensor_dim_value(*d)).collect();
                let vi = Proto::value_info(&name, &shape_dims, elem_type as i32);
                value_infos.push(vi.clone());
                graph_inputs.push(vi);
                info!(
                    "{}: extra input: {} (elem_type={})",
                    Self::TAG,
                    name,
                    elem_type
                );
            }
        }

        if graph_inputs.is_empty() {
            return Err("No graph inputs".to_string());
        }
        if all_outputs.is_empty() {
            return Err("No graph outputs".to_string());
        }

        info!(
            "{}: {} nodes, {} initializers, {} graph inputs, {} outputs, {} value_infos",
            Self::TAG,
            all_nodes.len(),
            all_initializers.len(),
            graph_inputs.len(),
            all_outputs.len(),
            value_infos.len()
        );

        let graph = Proto::graph(
            &graph_name,
            &all_nodes,
            &graph_inputs,
            &all_outputs,
            &all_initializers,
            &value_infos,
        );
        let opset = Proto::opset("", opset_version);
        let model = Proto::model(11, &opset, "cam_rust_graph_composer", &graph);

        Ok(model)
    }

    /// Legacy: walk the linked list via next() pointers.
    pub fn compose(
        head: &dyn IspBlock,
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<Vec<u8>, String> {
        let chain = Self::walk_chain(head)?;
        eprintln!("GraphComposer::compose: chain length = {}", chain.len());
        eprintln!(
            "GraphComposer::compose: aux_blocks length = {}",
            aux_blocks.len()
        );
        // If the chain only has the head and there are aux_blocks,
        // treat aux_blocks as part of the main pipeline
        if chain.len() == 1 && !aux_blocks.is_empty() {
            let mut full_chain = chain;
            full_chain.extend_from_slice(aux_blocks);
            eprintln!(
                "GraphComposer::compose: using full_chain length = {}",
                full_chain.len()
            );
            Self::compose_from_vec(&full_chain, &[], opset_version)
        } else {
            eprintln!("GraphComposer::compose: using chain + aux_blocks");
            Self::compose_from_vec(&chain, aux_blocks, opset_version)
        }
    }

    /// Walk the main chain starting from head.
    fn walk_chain(mut block: &dyn IspBlock) -> Result<Vec<&dyn IspBlock>, String> {
        let mut chain = Vec::new();
        loop {
            chain.push(block);
            match block.next() {
                Some(next) => {
                    block = &**next;
                }
                None => break,
            }
        }
        if chain.is_empty() {
            return Err("Empty main chain".to_string());
        }
        Ok(chain)
    }

    /// Wire each block's input_source to the previous block's frame_tensor.
    /// Must be called before `compose_from_vec`.
    pub fn wire_blocks(blocks: &mut [Box<dyn IspBlock>]) {
        // Set the head block's input_source to its graph_input_name
        if let Some(name) = blocks[0].graph_input_name().map(|s| s.to_string()) {
            blocks[0].set_input_source(&name);
        }
        for i in 1..blocks.len() {
            let prev = blocks[i - 1].frame_tensor().unwrap_or("--").to_string();
            blocks[i].set_input_source(&prev);
        }
    }

    /// Validate pipeline before ONNX emission.
    /// Returns list of warnings/errors.
    pub fn validate_pipeline(pipeline: &[&dyn IspBlock]) -> Vec<String> {
        let mut issues = Vec::new();

        if pipeline.is_empty() {
            issues.push("Pipeline is empty".into());
            return issues;
        }

        // Check for duplicate block IDs
        let mut seen_ids = std::collections::HashSet::new();
        for blk in pipeline {
            if !seen_ids.insert(blk.id()) {
                issues.push(format!("Duplicate block ID: '{}'", blk.id()));
            }
        }

        // Check input_source is set for all blocks except the first
        for (i, blk) in pipeline.iter().enumerate() {
            if i > 0 {
                if let Some(src) = blk.input_source() {
                    if src.is_empty() {
                        issues.push(format!("Block '{}' has empty input_source", blk.id()));
                    }
                } else {
                    issues.push(format!("Block '{}' has no input_source", blk.id()));
                }
            }
        }

        // Check tensor connectivity: each block's input should be produced by a predecessor
        let mut produced: std::collections::HashSet<String> = std::collections::HashSet::new();
        for (i, blk) in pipeline.iter().enumerate() {
            if i > 0 {
                if let Some(src) = blk.input_source() {
                    if !produced.contains(src) && !src.is_empty() {
                        issues.push(format!(
                            "Block '{}' input '{}' not produced by any predecessor",
                            blk.id(),
                            src
                        ));
                    }
                }
            }
            for t in blk.output_tensors() {
                produced.insert(t);
            }
        }

        // Check for empty output tensors
        for blk in pipeline {
            if blk.output_tensors().is_empty() {
                issues.push(format!("Block '{}' has no output tensors", blk.id()));
            }
        }

        issues
    }

    /// Validate pipeline and generate auto-fix suggestions.
    pub fn validate_with_fixes(pipeline: &[&dyn IspBlock]) -> (Vec<String>, Vec<String>) {
        let issues = Self::validate_pipeline(pipeline);
        let mut fixes = Vec::new();

        for issue in &issues {
            if issue.contains("empty input_source") {
                fixes.push(
                    "Set input_source to predecessor's frame_tensor via wire_blocks()".into(),
                );
            }
            if issue.contains("not produced by any predecessor") {
                fixes.push(
                    "Add missing block before this one to produce the required tensor".into(),
                );
            }
            if issue.contains("Duplicate block ID") {
                fixes.push("Remove duplicate block or give it a unique ID".into());
            }
            if issue.contains("Pipeline is empty") {
                fixes.push("Add at least an UnpackBlock and DisplayBlock".into());
            }
        }

        (issues, fixes)
    }

    /// Compose with auto-wiring and return pipeline stats.
    pub fn compose_auto(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<(Vec<u8>, PipelineStats), String> {
        // Auto-wire input sources
        Self::wire_blocks(blocks);

        // Collect stats before composing
        let block_names: Vec<String> = blocks.iter().map(|b| b.id().to_string()).collect();
        let total_nodes: usize = blocks.iter().map(|b| b.nodes().len()).sum();
        let total_inits: usize = blocks.iter().map(|b| b.initializers().len()).sum();

        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let onnx = Self::compose_from_vec(&refs, aux_blocks, opset_version)?;

        let stats = PipelineStats {
            block_count: blocks.len(),
            block_names,
            total_nodes,
            total_initializers: total_inits,
            onnx_bytes: onnx.len(),
            estimated_flops: 0,
            estimated_memory_bytes: 0,
        };

        Ok((onnx, stats))
    }

    /// Full pipeline composition: validate → auto-wire → compose → stats.
    /// Returns (onnx_bytes, stats, validation_issues).
    pub fn compose_full(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<(Vec<u8>, PipelineStats, Vec<String>), String> {
        // Auto-wire
        Self::wire_blocks(blocks);

        // Validate
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let issues = Self::validate_pipeline(&refs);

        // Compute FLOPs/memory estimates before mutable borrow
        let (flops, _) = Self::pipeline_flops_estimate(&refs, 1920, 1080);
        let (mem, _) = Self::pipeline_memory_estimate(&refs, 1920, 1080);
        drop(refs);

        // Compose
        let (onnx, mut stats) = Self::compose_auto(blocks, aux_blocks, opset_version)?;

        // Auto-populate estimates
        stats.estimated_flops = flops;
        stats.estimated_memory_bytes = mem;

        Ok((onnx, stats, issues))
    }

    /// Full pipeline composition with specific resolution for accurate estimates.
    /// FLOPs and memory estimates use the given width/height instead of default 1080p.
    pub fn compose_full_at(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
        w: u32,
        h: u32,
    ) -> Result<(Vec<u8>, PipelineStats, Vec<String>), String> {
        // Auto-wire
        Self::wire_blocks(blocks);

        // Validate
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let issues = Self::validate_pipeline(&refs);

        // Compute FLOPs/memory at requested resolution
        let (flops, _) = Self::pipeline_flops_estimate(&refs, w, h);
        let (mem, _) = Self::pipeline_memory_estimate(&refs, w, h);
        drop(refs);

        // Compose
        let (onnx, mut stats) = Self::compose_auto(blocks, aux_blocks, opset_version)?;

        stats.estimated_flops = flops;
        stats.estimated_memory_bytes = mem;

        Ok((onnx, stats, issues))
    }

    /// Generate a text report of the pipeline.
    pub fn pipeline_report(blocks: &[&dyn IspBlock]) -> String {
        let mut lines = Vec::new();
        lines.push(format!("Pipeline Report ({} blocks)", blocks.len()));
        lines.push("─".repeat(60));

        for (i, blk) in blocks.iter().enumerate() {
            let nodes = blk.nodes().len();
            let inits = blk.initializers().len();
            let inputs = blk.input_tensors().join(", ");
            let outputs = blk.output_tensors().join(", ");
            lines.push(format!(
                "  {:2}. {:<20} nodes={:<3} inits={:<2} [{}] → [{}]",
                i + 1,
                blk.id(),
                nodes,
                inits,
                inputs,
                outputs
            ));
        }

        lines.push("─".repeat(60));
        let total_nodes: usize = blocks.iter().map(|b| b.nodes().len()).sum();
        let total_inits: usize = blocks.iter().map(|b| b.initializers().len()).sum();
        lines.push(format!(
            "  Total: {} ops, {} initializers",
            total_nodes, total_inits
        ));

        lines.join("\n")
    }

    /// One-line pipeline summary.
    pub fn pipeline_summary(blocks: &[&dyn IspBlock]) -> String {
        let total_nodes: usize = blocks.iter().map(|b| b.nodes().len()).sum();
        let names: Vec<&str> = blocks.iter().map(|b| b.id()).collect();
        format!(
            "{} blocks ({} ops): {}",
            blocks.len(),
            total_nodes,
            names.join(" → ")
        )
    }

    /// Compose ONNX + convert to MNN in one call.
    /// Returns (onnx_bytes, mnn_path, stats).
    #[cfg(feature = "mnn")]
    pub fn compose_and_convert(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
        mnn_dir: &std::path::Path,
        name: &str,
    ) -> Result<(Vec<u8>, std::path::PathBuf, PipelineStats), String> {
        let (onnx, stats, issues) = Self::compose_full(blocks, aux_blocks, opset_version)?;
        if !issues.is_empty() {
            eprintln!("Warning: {} validation issues", issues.len());
            for issue in &issues {
                eprintln!("  - {}", issue);
            }
        }

        let mnn_path = mnn_dir.join(format!("{}.mnn", name));

        let mnn_bytes = crate::mnn_converter::convert_onnx_buffer(&onnx)
            .map_err(|e| format!("convert: {}", e))?;
        std::fs::write(&mnn_path, &mnn_bytes).map_err(|e| format!("write mnn: {}", e))?;

        Ok((onnx, mnn_path, stats))
    }

    /// Benchmark ONNX emission: generate + measure time.
    pub fn compose_benchmark(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<(Vec<u8>, PipelineStats, f64), String> {
        let t0 = std::time::Instant::now();
        let (onnx, stats, _issues) = Self::compose_full(blocks, aux_blocks, opset_version)?;
        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
        Ok((onnx, stats, elapsed_ms))
    }

    /// Estimate FLOPs for the pipeline at given resolution.
    /// Returns (total_flops, per_block_flops).
    pub fn pipeline_flops_estimate(
        blocks: &[&dyn IspBlock],
        w: u32,
        h: u32,
    ) -> (u64, Vec<(String, u64)>) {
        let pixels = (w as u64) * (h as u64);
        let mut total = 0u64;
        let mut per_block = Vec::new();

        for blk in blocks {
            let flops = match blk.id() {
                // Conv: 2 * out_ch * in_ch * kH * kW * pixels
                id if id.starts_with("demosaic") => 9 * pixels, // 3x3 conv per channel
                id if id.starts_with("ccm") => 18 * pixels,     // 3x3 matrix * 3 channels
                id if id.starts_with("warp") => 5 * pixels,     // bilinear sample + grid
                id if id.starts_with("chromatic") => 15 * pixels, // 3x GridSample
                id if id.starts_with("sharpen") => 8 * pixels,  // AvgPool + Sub + Add
                id if id.starts_with("colorspace") => 18 * pixels, // 3x3 matrix
                id if id.starts_with("gamma") => 6 * pixels,    // Log + Mul + Exp
                id if id.starts_with("auto_contrast") => 4 * pixels, // Sub + Mul + Add
                id if id.starts_with("display") => 3 * pixels,  // scale + clamp
                id if id.starts_with("unpack") => 2 * pixels,   // scale + channel extract
                _ => pixels,                                    // default: 1 flop/pixel
            };
            total += flops;
            per_block.push((blk.id().to_string(), flops));
        }

        (total, per_block)
    }

    /// Estimate memory usage (bytes) for the pipeline at given resolution.
    /// Returns (total_bytes, per_block_bytes).
    pub fn pipeline_memory_estimate(
        blocks: &[&dyn IspBlock],
        w: u32,
        h: u32,
    ) -> (u64, Vec<(String, u64)>) {
        let f32_size = 4u64;
        let pixels = (w as u64) * (h as u64);
        let mut total = 0u64;
        let mut per_block = Vec::new();

        for blk in blocks {
            // Each block typically has one [1,3,H,W] output = 3*H*W*4 bytes
            // Some blocks (unpack) produce 4ch, some (demosaic) produce 3ch
            let channels: u64 = match blk.id() {
                id if id.starts_with("display") => 4, // RGBA
                id if id.starts_with("noise") => 1,   // noise map
                _ => 3,                               // RGB
            };
            let output_bytes = channels * pixels * f32_size;
            // Initializer data
            let init_bytes: u64 = blk.initializers().iter().map(|i| i.len() as u64).sum();
            let block_total = output_bytes + init_bytes;
            total += block_total;
            per_block.push((blk.id().to_string(), block_total));
        }

        (total, per_block)
    }

    /// Full pipeline benchmark: validate + auto-wire + compose + measure time.
    pub fn compose_full_benchmark(
        blocks: &mut [Box<dyn IspBlock>],
        aux_blocks: &[&dyn IspBlock],
        opset_version: i64,
    ) -> Result<(Vec<u8>, PipelineStats, Vec<String>, f64), String> {
        let t0 = std::time::Instant::now();
        let (onnx, stats, issues) = Self::compose_full(blocks, aux_blocks, opset_version)?;
        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
        Ok((onnx, stats, issues, elapsed_ms))
    }

    /// Generate a complete pipeline analysis report.
    pub fn compose_report(blocks: &[&dyn IspBlock], w: u32, h: u32) -> String {
        let mut lines = Vec::new();
        lines.push(format!("Pipeline Analysis ({}×{})", w, h));
        lines.push("═".repeat(60));
        lines.push("".to_string());

        // Structure
        lines.push("Structure:".into());
        lines.push(Self::pipeline_report(blocks));
        lines.push("".into());

        // Summary
        lines.push("Summary:".into());
        lines.push(format!("  {}", Self::pipeline_summary(blocks)));
        lines.push("".into());

        // FLOPs
        let (flops, flops_detail) = Self::pipeline_flops_estimate(blocks, w, h);
        lines.push(format!("Compute: {:.1} MFLOPs", flops as f64 / 1e6));
        for (name, f) in &flops_detail {
            lines.push(format!("  {:<20} {:.1} MFLOPs", name, *f as f64 / 1e6));
        }
        lines.push("".into());

        // Memory
        let (mem, mem_detail) = Self::pipeline_memory_estimate(blocks, w, h);
        lines.push(format!("Memory:  {:.1} KB output", mem as f64 / 1024.0));
        for (name, m) in &mem_detail {
            lines.push(format!("  {:<20} {:.1} KB", name, *m as f64 / 1024.0));
        }

        lines.join("\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blocks::*;

    #[test]
    fn test_default_pipeline_composition() {
        // Build the standard 9-block pipeline
        let raw = RawInputBlock::new();
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

        let pipeline: Vec<&dyn IspBlock> =
            vec![&raw, &norm, &cfa, &blc, &wb, &dem, &ccm, &tone, &disp];
        let model = GraphComposer::compose_from_vec(&pipeline, &[], 16).expect("Compose failed");
        assert!(!model.is_empty());
        assert!(model.len() > 2000); // Expected ~2719 bytes
    }

    #[test]
    fn test_validate_empty_pipeline() {
        let issues = GraphComposer::validate_pipeline(&[]);
        assert_eq!(issues.len(), 1);
        assert!(issues[0].contains("empty"));
    }

    #[test]
    fn test_validate_with_fixes_empty() {
        let (issues, fixes) = GraphComposer::validate_with_fixes(&[]);
        assert_eq!(issues.len(), 1);
        assert!(!fixes.is_empty());
        assert!(fixes[0].contains("UnpackBlock"));
    }

    #[test]
    fn test_validate_with_fixes_duplicate() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let (issues, fixes) = GraphComposer::validate_with_fixes(&refs);
        assert!(issues.iter().any(|i| i.contains("Duplicate")));
        assert!(!fixes.is_empty());
    }

    #[test]
    fn test_validate_good_pipeline() {
        let _unpack = UnpackBlock::new().with_concrete_dims(480, 640);
        let _demosaic = DemosaicCcmBlock::new(0);
        let _display = DisplayBlock::new(640);
        // Wire them
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let issues = GraphComposer::validate_pipeline(&refs);
        assert!(
            issues.is_empty(),
            "good pipeline should have no issues: {:?}",
            issues
        );
    }

    #[test]
    fn test_validate_empty_input_source() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)), // input_source not set
            Box::new(DisplayBlock::new(640)),
        ];
        // Only wire first->second, skip second->third
        let first_tensor = blocks[0].frame_tensor().unwrap().to_string();
        blocks[1].set_input_source(&first_tensor);
        // blocks[2] input_source is empty by default
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let issues = GraphComposer::validate_pipeline(&refs);
        assert!(!issues.is_empty(), "should detect empty input_source");
    }

    #[test]
    fn test_compose_auto_wires_and_composes() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        let (onnx, stats) = GraphComposer::compose_auto(&mut blocks, &[], 16).unwrap();
        assert!(!onnx.is_empty());
        assert_eq!(stats.block_count, 3);
        assert_eq!(stats.block_names, vec!["unpack", "demosaic_ccm", "display"]);
        assert!(stats.total_nodes > 0);
        assert!(stats.onnx_bytes > 200);
        println!("Stats: {}", stats);
    }

    #[test]
    fn test_pipeline_stats_display() {
        let stats = PipelineStats {
            block_count: 2,
            block_names: vec!["a".into(), "b".into()],
            total_nodes: 10,
            total_initializers: 5,
            onnx_bytes: 1024,
            estimated_flops: 5_000_000,
            estimated_memory_bytes: 2048 * 1024,
        };
        let s = format!("{}", stats);
        assert!(s.contains("2 blocks"));
        assert!(s.contains("10 ops"));
        assert!(s.contains("1024 bytes"));
        assert!(s.contains("5.0 MFLOPs"));
        assert!(s.contains("2048.0 KB"));
    }

    #[test]
    fn test_compose_full_validates_and_composes() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        let (onnx, stats, issues) = GraphComposer::compose_full(&mut blocks, &[], 16).unwrap();
        assert!(!onnx.is_empty());
        assert_eq!(stats.block_count, 3);
        assert!(
            issues.is_empty(),
            "wired pipeline should have no issues: {:?}",
            issues
        );
        assert!(stats.estimated_flops > 0, "FLOPs should be auto-populated");
        assert!(
            stats.estimated_memory_bytes > 0,
            "memory should be auto-populated"
        );
    }

    #[test]
    fn test_compose_full_at_4k() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(2160, 3840)),
            Box::new(DemosaicCcmBlock::new(2)),
            Box::new(DisplayBlock::new(3840)),
        ];
        let (onnx, stats, _) =
            GraphComposer::compose_full_at(&mut blocks, &[], 16, 3840, 2160).unwrap();
        assert!(!onnx.is_empty());
        assert!(stats.estimated_flops > 0);
        println!(
            "4K: {} MFLOPs, {} KB",
            stats.estimated_flops / 1_000_000,
            stats.estimated_memory_bytes / 1024
        );
    }

    #[test]
    fn test_pipeline_report() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let report = GraphComposer::pipeline_report(&refs);
        assert!(report.contains("Pipeline Report"));
        assert!(report.contains("3 blocks"));
        assert!(report.contains("unpack"));
        assert!(report.contains("display"));
        println!("{}", report);
    }

    #[test]
    fn test_pipeline_summary() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let summary = GraphComposer::pipeline_summary(&refs);
        assert!(summary.contains("3 blocks"));
        assert!(summary.contains("unpack"));
        assert!(summary.contains("display"));
        println!("Summary: {}", summary);
    }

    #[test]
    fn test_compose_benchmark_returns_time() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        let (onnx, stats, elapsed_ms) =
            GraphComposer::compose_benchmark(&mut blocks, &[], 16).unwrap();
        assert!(!onnx.is_empty());
        assert!(elapsed_ms >= 0.0);
        assert!(stats.block_count == 3);
        println!("Emission: {:.2} ms, {} bytes", elapsed_ms, onnx.len());
    }

    #[test]
    fn test_pipeline_flops_estimate() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(1920)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let (total, per_block) = GraphComposer::pipeline_flops_estimate(&refs, 1920, 1080);
        assert!(total > 0);
        assert_eq!(per_block.len(), 3);
        println!("FHD FLOPs: {} ({:.1} MFLOPs)", total, total as f64 / 1e6);
    }

    #[test]
    fn test_pipeline_memory_estimate() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(1920)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let (total, per_block) = GraphComposer::pipeline_memory_estimate(&refs, 1920, 1080);
        assert!(total > 0);
        assert_eq!(per_block.len(), 3);
        println!("FHD memory: {} ({:.1} KB)", total, total as f64 / 1024.0);
    }

    #[test]
    fn test_compose_report() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(1080, 1920)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(1920)),
        ];
        GraphComposer::wire_blocks(&mut blocks);
        let refs: Vec<&dyn IspBlock> = blocks.iter().map(|b| b.as_ref()).collect();
        let report = GraphComposer::compose_report(&refs, 1920, 1080);
        assert!(report.contains("Pipeline Analysis"));
        assert!(report.contains("1920×1080"));
        assert!(report.contains("Compute:"));
        assert!(report.contains("Memory:"));
        println!("{}", report);
    }

    #[test]
    fn test_compose_full_benchmark() {
        let mut blocks: Vec<Box<dyn IspBlock>> = vec![
            Box::new(UnpackBlock::new().with_concrete_dims(480, 640)),
            Box::new(DemosaicCcmBlock::new(0)),
            Box::new(DisplayBlock::new(640)),
        ];
        let (onnx, stats, issues, elapsed) =
            GraphComposer::compose_full_benchmark(&mut blocks, &[], 16).unwrap();
        assert!(!onnx.is_empty());
        assert_eq!(stats.block_count, 3);
        assert!(issues.is_empty());
        assert!(elapsed >= 0.0);
        println!("Full benchmark: {:.2} ms, {} bytes", elapsed, onnx.len());
    }
}
