// isp_opset.h — ISP Opset API
//
// Each op is a standalone OpType_Extra routed through VulkanFuseCreator.
// Ops are composed in sequence to form the ISP pipeline:
//
//   Bayer RAW [INT32 1x1xHxW]
//       │
//       ▼
//   isp.unpack_blc — Bayer→RGGB + Black Level Correction
//       ▼
//   isp.demosaic  — Hamilton-Adams demosaic + CCM
//       ▼
//   isp.fcs       — Flat Field / Shading Correction
//       ▼
//   isp.ee        — Edge Enhancement (unsharp mask)
//       ▼
//   isp.ldci      — Local Dynamic Contrast
//       ▼
//   isp.display   — Brightness/Contrast/Saturation + Gamma
//       ▼
//   Display-ready RGB [FLOAT 1x3xHxW]
//
// ── Op Attribute Schema ──────────────────────────────────────────────
// Every ISP op shares this attribute layout:
//
//   spirv:    Blob[INT8]  — SPIR-V bytecode
//   output_shape: Blob[INT32] — output tensor dimensions [N,C,H,W]
//   global_size:  Blob[INT32] — [W,H,1] threads
//   group_size:   Blob[INT32] — [1,1,1] (overridden by optimized_dispatch)
//   optimized_dispatch: bool — set true to skip auto-tuning
//   input[0]: List[i] — [io_type=0, binding=1] input tensor→binding 1
//   input[1]: List[i] — [io_type=1, binding=2] output tensor→binding 2
//   const:    Blob[FLOAT] + b — uniform data at binding 0 (b=false → SSBO)
//
// ── Pipeline Assembly ───────────────────────────────────────────────
// IspPipelineBuilder composes ops into a sequential flatbuffer model.
//
// Tensor reuse (in-place mode):
//   Element-wise stages (FCS, Display) can reuse the input tensor's buffer
//   as output by setting inplace=true. This eliminates a separate output
//   buffer allocation and the associated GPU write-to-new-location.
//
//   Neighbor-read stages (EE, LDCI) and size-changing stages (Unpack,
//   Demosaic) MUST NOT use inplace mode.
//
//   Memory savings with 6-stage ISP (4K→FHD):
//     Without inplace: 7 tensors, 191 MB peak
//     With inplace (stages 2,5): 5 tensors, 130 MB peak (-32%)
//
#ifndef ISP_OPSET_H
#define ISP_OPSET_H

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <MNN/Interpreter.hpp>
#include "schema/current/MNN_generated.h"

namespace isp {

// ── Opset Type Strings ──────────────────────────────────────────────
static const char* const kOpUnpackBlc       = "isp.unpack_blc";
static const char* const kOpDemosaic       = "isp.demosaic_ccm";
static const char* const kOpDemosaicNoscale = "isp.demosaic_noscale";
static const char* const kOpFcs            = "isp.fcs";
static const char* const kOpEe             = "isp.ee";
static const char* const kOpLdci           = "isp.ldci";
static const char* const kOpDisplay        = "isp.display";

// ── Op Descriptor ───────────────────────────────────────────────────
struct OpDesc {
    const char*      type;          // opset type string
    std::vector<int> output_shape;  // [N,C,H,W]
    std::vector<int> global_size;   // [W,H,1]
    std::vector<float> uniforms;    // const buffer data (floats)
};

// ── Default Op Configuration ────────────────────────────────────────
// These match the 8×8 test pattern (input 8×8 Bayer).
// For production, swap w/2→W/2, h/2→H/2 etc.
inline OpDesc UnpackBlc(int w, int h) {
    return {
        kOpUnpackBlc,
        {1, 4, h/2, w/2},          // RGGB 4 channels at half res
        {w/2, h/2, 1},             // one thread per 2×2 block
        {float(w), float(h),
         float(w/2), float(h/2),
         1023.0f,                   // sensor_max
         0.0f, 0.0f, 0.0f, 0.0f,   // blc offsets
         1.0f, 1.0f, 1.0f, 1.0f}   // wb gains
    };
}

inline OpDesc DemosaicCcm(int w, int h) {
    return {
        kOpDemosaic,
        {1, 3, h, w},              // RGB 3 channels at full res
        {w, h, 1},                 // one thread per pixel
        {float(w/2), float(h/2),   // input dimensions (RGGB quadrants)
         float(w), float(h),       // output dimensions
         1023.0f,                   // sensor_max
         1.0f, 0.0f, 0.0f,         // identity CCM row 0
         0.0f, 1.0f, 0.0f,         // identity CCM row 1
         0.0f, 0.0f, 1.0f,         // identity CCM row 2
         0.0f, 0.0f, 0.0f, 0.0f}   // pad[4]
    };
}

// Same-res demosaic: 4-channel RGGB → 3-channel RGB at same W×H.
// Input RGGB is already normalized [0,1] from unpack_blc.
// sensor_max=1.0 because unpack already divided by sensor_max.
inline OpDesc DemosaicNoscale(int w, int h) {
    return {
        kOpDemosaicNoscale,
        {1, 3, h, w},              // RGB 3 channels at same res
        {w, h, 1},
        {float(w), float(h),
         1.0f,                      // sensor_max=1.0 (input already normalized)
         1.0f, 0.0f, 0.0f,         // identity CCM row 0
         0.0f, 1.0f, 0.0f,         // identity CCM row 1
         0.0f, 0.0f, 1.0f,         // identity CCM row 2
         0.0f, 0.0f, 0.0f, 0.0f}   // pad[4]
    };
}

inline OpDesc Fcs(int w, int h, float strength = 1.0f) {
    return {
        kOpFcs,
        {1, 3, h, w},
        {w, h, 1},
        {float(w), float(h),
         strength,                  // gain
         0.0f,                      // center_th (unused)
         0.0f,                      // suppression=0 → passthrough
         0.0f, 0.0f, 0.0f, 0.0f}   // pad[4]
    };
}

inline OpDesc Ee(int w, int h, float strength = 0.5f, float threshold = 0.01f) {
    return {
        kOpEe,
        {1, 3, h, w},
        {w, h, 1},
        {float(w), float(h),
         strength,
         threshold,
         0.0f, 0.0f, 0.0f, 0.0f}   // pad[4]
    };
}

inline OpDesc Ldci(int w, int h, float strength = 0.5f, float radius = 1.0f) {
    return {
        kOpLdci,
        {1, 3, h, w},
        {w, h, 1},
        {float(w), float(h),
         strength,
         radius,
         0.0f, 0.0f, 0.0f, 0.0f}   // pad[4]
    };
}

inline OpDesc Display(int w, int h, float gamma = 2.2f) {
    return {
        kOpDisplay,
        {1, 3, h, w},
        {w, h, 1},
        {float(w), float(h),
         0.0f,                      // brightness
         1.0f,                      // contrast
         1.0f,                      // saturation
         gamma,                     // gamma (2.2=sRGB)
         0.0f, 0.0f}                // pad[2]
    };
}

// ── Pipeline Builder (with tensor pooling) ──────────────────────────
// Assembles a sequence of ISP ops into a MNN flatbuffer model.
// Tracks tensor indices dynamically to support in-place reuse.
//
class IspPipelineBuilder {
public:
    IspPipelineBuilder()
        : mFbb(new flatbuffers::FlatBufferBuilder(1024*1024)) {
        mNet.reset(new MNN::NetT);
        mNet->bizCode = "IspPipeline";
        // tensor_0 is always the session input
        ensureTensor(0);
    }

    // Add an ISP stage.
    // inplace=true: output tensor reuses input tensor's buffer (element-wise ops only).
    // inplace=false: allocates a new tensor for output (required for ops
    //   with neighbor reads or size changes).
    //
    void addStage(const OpDesc& desc, const std::vector<int8_t>& spirv,
                  bool inplace = false) {
        int inIdx  = mLastOutput;                // input = previous stage's output
        int outIdx = inplace ? inIdx : inIdx + 1; // output = new or reuse

        // Allocate tensor names up to max index needed
        ensureTensor(std::max(inIdx, outIdx));

        auto op = std::unique_ptr<MNN::OpT>(new MNN::OpT);
        op->type = MNN::OpType_Extra;
        op->main.type = MNN::OpParameter_Extra;
        op->main.value = new MNN::ExtraT();
        auto* extra = static_cast<MNN::ExtraT*>(op->main.value);
        extra->type = desc.type;
        op->inputIndexes.push_back(inIdx);
        op->outputIndexes.push_back(outIdx);

        auto addA = [&](const char* key, std::function<void(MNN::AttributeT*)> fn) {
            std::unique_ptr<MNN::AttributeT> a(new MNN::AttributeT);
            a->key = key; fn(a.get());
            extra->attr.push_back(std::move(a));
        };

        // SPIR-V
        addA("spirv", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT8;
            a->tensor->int8s = spirv;
        });
        // Output shape
        addA("output_shape", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = desc.output_shape;
        });
        // Global size
        addA("global_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = desc.global_size;
        });
        // Group size + optimized dispatch
        addA("group_size", [&](MNN::AttributeT* a) {
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_INT32;
            a->tensor->int32s = {1, 1, 1};
        });
        addA("optimized_dispatch", [&](MNN::AttributeT* a) {
            a->b = true;
        });
        // Bindings: input→binding 1, output→binding 2
        addA("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {0, 1};
        });
        addA("input", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->list.reset(new MNN::ListValueT);
            a->list->i = {1, 2};
        });
        // Const buffer at binding 0, SSBO (b=false)
        addA("const", [&](MNN::AttributeT* a) {
            a->i = 0;
            a->tensor.reset(new MNN::BlobT);
            a->tensor->dataType = MNN::DataType_DT_FLOAT;
            a->tensor->float32s = desc.uniforms;
            a->b = false;  // SSBO (std430, tight packing)
        });

        mNet->oplists.push_back(std::move(op));
        mLastOutput = outIdx;
    }

    // Finalize and return the flatbuffer data
    const uint8_t* build(size_t* out_size) {
        auto offset = MNN::Net::Pack(*mFbb, mNet.get());
        mFbb->Finish(offset);
        *out_size = mFbb->GetSize();
        return mFbb->GetBufferPointer();
    }

    // Name of the final output tensor (for getSessionOutput)
    std::string outputTensorName() const {
        return "tensor_" + std::to_string(mLastOutput);
    }

    // Total tensor count in the model
    int tensorCount() const {
        return (int)mNet->tensorName.size();
    }

private:
    void ensureTensor(int idx) {
        while ((int)mNet->tensorName.size() <= idx)
            mNet->tensorName.push_back("tensor_" + std::to_string(mNet->tensorName.size()));
    }

    int mLastOutput = 0;   // last written tensor index
    std::unique_ptr<flatbuffers::FlatBufferBuilder> mFbb;
    std::unique_ptr<MNN::NetT> mNet;
};

} // namespace isp

#endif // ISP_OPSET_H
