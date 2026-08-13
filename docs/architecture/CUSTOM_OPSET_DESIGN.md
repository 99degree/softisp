# ONNX Custom Opset Support — Design Analysis

## 1. Problem Statement

Today every `IspBlock` emits **standard ONNX primitive ops only** (Conv, Mul, Add,
Sub, GridSample, ...). The mapping from those primitives to MNN's fused `isp.*`
Extra ops happens **inside the MNN converter** (Pass 1 / `IspChainFusion`) during
ONNX→MNN conversion — it is opaque to the Rust pipeline, invisible in the ONNX
graph, and not declared anywhere in the block API.

Consequences:
- A block cannot declare the *semantics* of its fragment; only its primitive
  decomposition.
- Tooling outside MNN (ONNX Runtime, netron, shape inference) sees only generic
  primitives.
- There is no first-class way to attach block-level metadata (precision,
  validation rules, Vulkan workgroup hints) to the model in a backend-agnostic
  form.

## 2. Proposed Notation

Use **short opset names** in Rust/config, mapped to **ONNX custom domains**:

| Short name | ONNX domain | Meaning |
|---|---|---|
| `isp` | `isp` | Base SoftISP custom opset (Vulkan-fused primitives) |
| `isp.ml` | `isp.ml` | Neural / learned operators (future rectifier-style blocks) |

**Rule:** `<domain>::<OpName>` in Rust/config becomes the op_type string
in ONNX.  The domain separator is `::` (double colon), matching ONNX's
existing domain-qualified op convention.

### Examples
```
isp::GpuWarpGrid
isp::ArgBConvert
isp::Fcs
isp.ml::RectifierHead
```

In ONNX protobuf:
```protobuf
// Opset imports
OperatorSetIdProto { domain: "isp"; version: 1 }
OperatorSetIdProto { domain: "isp.ml"; version: 1 }
OperatorSetIdProto { domain: ""; version: 20 }          // standard ONNX

// Node using custom op
NodeProto {
  input: ["input", "k1", "k2", "k3"]
  output: ["grid"]
  name: "gpu_warp_grid"
  op_type: "isp::GpuWarpGrid"   // domain-prefixed
  attribute: ...
}
```

## 3. Architecture

### 3.1 Rust API

Extend `IspBlock` with optional custom-opset metadata — **fully backward compatible**
because every new method has a default returning the empty/no-op case.

```rust
pub trait IspBlock: Send {
    // ── existing methods unchanged ──

    /// Optional custom ONNX op types this block emits, one per entry in
    /// `nodes()`.  When `Some`, each `&str` is the domain-prefixed op_type
    /// for the corresponding node (e.g. `"isp::Fcs"`).  Default `None`
    /// means all nodes are standard ONNX primitive ops.
    fn custom_op_types(&self) -> Option<&[&str]> {
        None
    }

    /// Custom opsets this block requires.  Each entry is `(domain, version)`.
    /// GraphComposer deduplicates across blocks and emits one
    /// `OperatorSetIdProto` per unique `(domain, version)`.  Default empty
    /// means the block only uses the standard ONNX opset.
    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![]
    }

    /// Whether this block's ONNX fragment uses custom or primitive ops.
    /// - `Primitive` (default): `custom_op_types()` is metadata only; the
    ///   emitted ONNX uses standard ONNX op types.  GraphComposer may lower
    ///   custom names to primitives before MNN conversion.
    /// - `Custom`: the block emits custom op types in ONNX; the consumer
    ///   must understand them.
    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::Primitive
    }
}

/// Whether a block's ONNX fragment uses custom or primitive ops.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockOpsetMode {
    /// Emit standard primitives; custom names are metadata only.
    Primitive,
    /// Emit custom op types in ONNX; consumer must understand them.
    Custom,
}
```

### 3.2 Proto helpers

New helpers in `crate::onnx::proto::Proto`:

```rust
impl Proto {
    /// Encode an ONNX opset import with a non-empty domain.
    pub fn opset_domain(domain: &str, version: i64) -> Vec<u8> {
        // OperatorSetIdProto: field 1 = domain (string), field 2 = version (int64)
        let mut buf = Self::string(1, domain);
        buf.extend_from_slice(&Self::int64(2, version));
        buf
    }

    /// Encode a NodeProto with an optional domain-prefixed op_type.
    /// The `domain_op` string is the full op_type as it should appear in the
    /// ONNX graph (e.g. "isp::Fcs", "isp.ml::RectifierHead").
    pub fn custom_node(
        domain_op: &str,
        inputs: &[&str],
        outputs: &[&str],
        attrs: &[Vec<u8>],
    ) -> Vec<u8> {
        // Store the full domain-prefixed string as op_type.
        // MNN's customized onnx.proto parser uses field 4 for op_type,
        // which is unchanged; only the string content gains a prefix.
        Self::node(domain_op, inputs, outputs, attrs)
    }

    /// Encode a ModelProto with multiple opset imports.
    /// Field 8 is `repeated OperatorSetIdProto`, so each opset entry is
    /// encoded as a separate length-delimited message in the same field.
    pub fn model_multi_opset(
        ir_version: i64,
        opsets: &[&[u8]],
        producer: &str,
        graph: &[u8],
    ) -> Vec<u8> {
        let mut buf = Self::int64(1, ir_version);
        buf.extend_from_slice(&Self::string(2, producer));
        buf.extend_from_slice(&Self::raw_bytes(7, graph));
        for opset in opsets {
            buf.extend_from_slice(&Self::raw_bytes(8, opset));
        }
        buf
    }
}
```

### 3.3 GraphComposer changes

`GraphComposer::compose_from_vec` now:
1. Collects all custom opsets from all blocks via `custom_opsets()`.
2. Deduplicates by `(domain, version)`.
3. Emits one `OperatorSetIdProto` per unique custom domain plus the standard ONNX opset.
4. The `opset_mode()` method controls whether custom op types are emitted or lowered.

```rust
// Collect custom opsets from all blocks and emit one
// OperatorSetIdProto per unique (domain, version).
let mut custom_domains: std::collections::HashSet<(String, i64)> =
    std::collections::HashSet::new();
for blk in &all_blocks {
    for (domain, version) in blk.custom_opsets() {
        custom_domains.insert((domain.clone(), version));
    }
}
let mut opset_imports: Vec<Vec<u8>> = vec![Proto::opset("", opset_version)];
for (domain, version) in &custom_domains {
    opset_imports.push(Proto::opset_domain(domain, *version));
}
let opset_refs: Vec<&[u8]> = opset_imports.iter().map(|o| o.as_slice()).collect();
let model = Proto::model_multi_opset(
    11,
    &opset_refs,
    "cam_rust_graph_composer",
    &graph,
);
```

### 3.4 Block-level examples

#### GpuWarpBlock (hypothetical custom-op variant)

```rust
impl IspBlock for GpuWarpBlock {
    // ... existing methods ...

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&[
            "isp::BroadcastAdd",   // grid_x + zero
            "isp::BroadcastAdd",   // grid_y + zero
            "isp::Mul",            // gx^2
            "isp::Mul",            // gy^2
            "isp::Add",            // r2
            "isp::Mul",            // r4
            "isp::Mul",            // r6
            "isp::GdcDenom",       // 1 + k1*r2 + ...
            "isp::GdcGrid",        // inv_denom, compose with EIS
            "isp::GridSample",     // final sample
        ])
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::Custom
    }
}
```

The block emits **identical tensor topology** (same names, same shapes) but the
op_type strings now carry domain semantics.  Downstream consumers that do not
understand `isp` can still treat them as opaque custom ops.

#### FcsBlock (standard → custom)

```rust
impl IspBlock for FcsBlock {
    // ...

    fn custom_opsets(&self) -> Vec<(String, i64)> {
        vec![("isp".to_string(), 1)]
    }

    fn custom_op_types(&self) -> Option<&[&str]> {
        Some(&["isp::Fcs"])
    }

    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::Custom
    }

    // nodes() emits 1 custom node directly:
    fn nodes(&self) -> Vec<Vec<u8>> {
        vec![Proto::custom_node(
            "isp::Fcs",
            &[&self.input_source, "gain", "bias"],
            &[&self.frame_tensor],
            &[],
        )]
    }
}
```

### 3.5 Backward compatibility

| Aspect | Strategy |
|---|---|
| Existing blocks | `custom_op_types()` defaults to `None`; `custom_opsets()` defaults to `Vec::new()`. Zero changes required. |
| Existing ONNX output | When no block declares custom opsets, only the standard ONNX opset is emitted — byte-identical to today. |
| MNN conversion | Custom domain ops pass through MNN's ONNX parser as opaque `Custom` ops. Primitive-only blocks are unaffected. |
| ONNX Runtime | ORT treats unknown custom ops as opaque if an `CustomOp` kernel is registered; otherwise it errors at session creation — this is expected behavior for truly custom ops. |

## 4. MNN Compatibility

### 4.1 Current MNN behavior

MNN uses a **customized** `onnx.proto` where:
- `NodeProto.op_type` is field 4 (matches our `Proto::node()`)
- `OperatorSetIdProto` is field 8 of ModelProto (matches our `Proto::opset()`)
- The converter treats op types it does not recognize as **custom ops** and emits
  them as `OpType_Custom` in MNN's flatbuffer.

This means a domain-prefixed op like `"isp::GpuWarpGrid"` will:
1. Be parsed by MNN's ONNX frontend as an unrecognized op → `Custom`.
2. Survive Pass 0 / Pass 1 as a `Custom` op **unless** a fusion rule matches it.
3. Require a runtime kernel registered under that exact name in the MNN Vulkan
   backend.

### 4.2 Two viable modes

#### Mode A — Custom-op all the way (long-term)

The block emits `isp::Fcs` directly.  MNN requires:
- An MNN-side `CustomOp` registration for `isp.Fcs` (or whatever the
  runtime-qualified name is).
- A Vulkan compute shader compiled into `libMNN_Vulkan.so`.

This is the **cleanest** semantically but requires coordinated MNN-fork work.

#### Mode B — Lowering pass (short-term, backward compatible)

The block **optionally** declares its preferred custom op type, but the
SoftISP pipeline **lowers** it to standard primitives **before** handing the
ONNX bytes to MNN.  The custom op_type is present in the emitted ONNX for
tooling/inspection, but the MNN-bound bytes use primitives.

Implementation:
```rust
pub enum BlockOpsetMode {
    /// Emit custom op types in ONNX; consumer must understand them.
    Custom,
    /// Emit standard primitives; custom names are comments/metadata only.
    Primitive,
}

impl IspBlock {
    /// Choose whether this block's ONNX fragment uses custom or primitive ops.
    /// Default: Primitive.
    fn opset_mode(&self) -> BlockOpsetMode {
        BlockOpsetMode::Primitive
    }
}
```

`GraphComposer` gets a `lower_custom_ops: bool` flag.  When true, custom node
op_types are mapped back to their primitive equivalents before emission.

This lets SoftISP adopt the notation **without** requiring an immediate MNN
runtime change.

### 4.3 Recommended path

1. **Phase 1 — Notation only (no MNN changes):** Add the trait methods, emit
   `OperatorSetIdProto(domain="isp", version=1)`, keep node op_types as
   standard primitives.  Custom opset presence is metadata/validation only.
2. **Phase 2 — Selective lowering:** Add `BlockOpsetMode::Custom`.  High-value
   blocks (Fcs, GpuWarp grid kernel, Display ARGB convert) emit a single fused
   custom op; the GraphComposer lowers it to the primitive chain for MNN.
3. **Phase 3 — MNN runtime:** Port the lowered primitive chains as MNN `Extra`
   ops into the custom MNN fork.  Wire `isp::Fcs` → `isp.fcs` shader.
4. **Phase 4 — Full custom ops:** Blocks emit opaque `isp::*` ops; MNN
   runtime dispatches to Vulkan kernels.

## 5. Concrete Impact on Existing Blocks

| Block | Primitive ops today | Hypothetical custom op | Notes |
|---|---|---|---|
| `FcsBlock` | `Mul` + `Add` + `Clip` | `isp::Fcs` | Single fused semantic; easy first target |
| `GpuWarpBlock` grid | 20+ `Add/Mul/Div/Concat` | `isp::GdcGrid` | Large reduction in node count |
| `DisplayBlock` ARGB | `Conv(1x1)` + `Mul(255)` + `Add(alpha)` | `isp::ArgBConvert` | Replaces `isp.argb_convert` Extra |
| `DisplayBlock` YUV420 | `Conv` + `Reshape` + `Mul/Add` | `isp::Yuv420Convert` | Replaces `isp.yuv420_convert` Extra |
| `UnpackCfaBlock` | `Cast/SpaceToDepth/Conv` | `isp::UnpackCfa` | Replaces `isp.unpack` Extra |
| `DemosaicCcmBlock` | `Conv(1x1)` | `isp::DemosaicCcm` | Same as today's fused Conv |
| `ToneStatsBlock` | 16 ops | `isp::ToneStats` | Encapsulates complex pattern |
| `CoarseHistogramBlock` | 23–95 ops | `isp::Histogram` | Variable-bin encapsulation |

## 6. Risks & Mitigations

| Risk | Mitigation |
|---|---|
| MNN ONNX parser rejects domain-prefixed op types | Phase 1 metadata-only mode; validate with `convert_mnn_buffer` test. |
| Existing `IspChainFusion` patterns break | Custom ops bypass fusion tables because they are already fused at the block level. |
| ONNX Runtime users see opaque custom ops | Document that `isp::*` requires SoftISP runtime or lowering. |
| Version skew between domains | Each domain versioned independently; block declares minimum version it needs. |
| Protobuf field numbering conflicts | Custom opset uses separate `OperatorSetIdProto` entries; no field overlap. |

## 7. Summary

The proposed design adds **optional** custom-opset support to every `IspBlock`
via three new trait methods with no-op defaults.  Blocks may:
1. Declare required custom domains via `custom_opsets()`.
2. Replace standard op_type strings with `isp::*` names via `custom_op_types()`.
3. Emit `OperatorSetIdProto(domain="isp", version=1)` in the ONNX model.

The short Rust notation `isp::Foo` maps directly to ONNX domain `"isp"` with
op_type `"Foo"`.  The `isp.ml::*` sub-domain is reserved for neural/learned
operators.  A two-phase implementation lets the codebase adopt the
notation immediately while deferring MNN runtime custom-op registration to a
follow-up phase.
