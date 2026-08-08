// ExactPattern.h — Match descriptor for ISP chain fusion.
//
// Defines the ExactPattern struct consumed by the auto-generated pattern
// tables in isp_fusion_patterns.h.  Separated so both the header and
// IspChainFusion.cpp can include it without circular dependencies.
#pragma once

#include <MNN/MNNDefine.h>
#include <MNN_generated.h>
#include <TensorflowOp_generated.h>

#include <cstdint>
#include <vector>

namespace MNN {

// =====================================================================
// ExactPattern — lightweight match descriptor consumed by the three
// tables in isp_fusion_patterns.h.
//
// The generated header uses five distinct constructor signatures
// (determined by positional arg count and type at positions 7/8):
//   6  args : (chain, ce, ci, type, key, namedKey)
//   7a args : (chain, ce, ci, type, key, namedKey, BinaryOpOperation)   — binOp
//   7b args : (chain, ce, ci, type, key, namedKey, bool)               — noFuse
//   8a args : (chain, ce, ci, type, key, namedKey, nullptr_t, int)     — cw only
//   8b args : (chain, ce, ci, type, key, namedKey, BinaryOpOperation, bool) — binOp+noFuse
// =====================================================================

struct ExactPattern {
    std::vector<OpType> chain;
    int constElems;        // -1 = any
    int constIndex;        // -1 = any
    const char* ispType;   // Extra-op type string, e.g. "isp.fcs"
    const char* typeKey;   // engine / SPIR-V key
    const char* namedKey;  // float-vector key or nullptr
    int binOp;             // BinaryOpOperation value, -1 = any
    int convWeightElems;   // -1 = any
    bool noFuse;           // guard: consumed but not replaced

    // 6-arg base
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(-1), noFuse(false) {}

    // 7-arg with BinaryOpOperation
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 BinaryOpOperation bo)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(static_cast<int>(bo)), convWeightElems(-1), noFuse(false) {}

    // 7-arg with noFuse bool (algo_cct guard pattern)
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 bool nf)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(-1), noFuse(nf) {}

    // 8-arg nullptr_t + convWeightElems
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 std::nullptr_t, int cw)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(cw), noFuse(false) {}

    // 8-arg BinaryOpOperation + noFuse
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 BinaryOpOperation bo, bool nf)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(static_cast<int>(bo)), convWeightElems(-1), noFuse(nf) {}
};

} // namespace MNN
