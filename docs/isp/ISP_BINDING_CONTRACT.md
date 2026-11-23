docs(isp): what the fixed `isp.*` bindings mean for softisp

Answering "is the fixed binding all MNN's internal arrangement, or does it bind
us too?" — it is internal, and softisp never sees a binding number. But it does
constrain us, through one thing we do control: **the order of a fused op's node
inputs**.

Where the layout actually lives:

  * `IspSpvLookup` embeds each shader's SPIR-V. The bindings are
    `OpDecorate`d inside that binary — they are not in the ONNX and not in the
    C++ wrapper.
  * The converter never touches bindings: zero occurrences of "binding" in
    `IspOnnxOps.cpp` and `onnxOpConverter.cpp`. Its whole job is
    `createVulkanFuseOp`, which copies the op's attributes and preserves
    `expr->inputs()` **in order**.
  * `VulkanFuse` turns that order into numbers:
    `mInputBinding[i] = (i == 0) ? frame : kFirstExtra + (i - 1)`.

So the chain is: our ONNX node input order → converter preserves it →
VulkanFuse numbers it → shader's compiled-in binding receives it. Only the last
link is fixed, and we do not read or write it. `libMNNConvertDeps.so` is not an
adapter that could translate anything: its C++ side mentions bindings zero times.

Two consequences worth knowing:

1. **The numbers are not the contract; the order is.** A shader moving its frame
   from binding 3 to 1 is MNN's business — as long as `VulkanFuse` numbers the
   slots to match, a graph that declares the same input order keeps working.
   That is exactly what the in-progress `VulkanFuse` change in `~/MNN` does (frames
   back to 1/2, metadata at 3, with `isp.pipeline` keeping the reserved slots),
   and it needed no softisp change at all — `CcmBlock` still emits
   `[frame, matrix, bias]` and would still land on the same shader variables.

2. **The one thing that would bind us is a fused op MNN does not have.** The
   input order is only meaningful against a known shader. `NormalizeBlcUnpackCfa`
   emits `isp.unpack_blc` (`src/blocks/normalize_blc_unpack_cfa.rs:186`), which
   is not in MNN's whitelist (`IspOpsetValidate.hpp`), has no `.spv`, and has no
   entry in `IspSpvLookup`. Nothing starts it today — no profile references the
   block — but if it were enabled, `known=0` would make VulkanFuse reject the op
   and the stage would silently fall back.

Nothing to change in softisp for the bindings themselves.
