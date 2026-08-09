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
#include <utility>
#include <vector>

namespace MNN {

// =====================================================================
// ExactPattern — lightweight match descriptor consumed by the three
// tables in isp_fusion_patterns.h.
//
// Constructor signatures used by the generated tables:
//   6  args : (chain, ce, ci, type, key, namedKey)
//   7a args : (chain, ce, ci, type, key, namedKey, BinaryOpOperation)   — binOp
//   7b args : (chain, ce, ci, type, key, namedKey, bool)               — noFuse
//   8a args : (chain, ce, ci, type, key, namedKey, nullptr_t, int)     — cw only
//   8b args : (chain, ce, ci, type, key, namedKey, BinaryOpOperation, bool) — binOp+noFuse
//   9  args : (chain, ce, ci, type, key, namedKey, nullptr_t, int, bool) — cw + reqConn
//  10  args : (chain, ce, ci, type, key, namedKey, nullptr_t, int, bool, constVals)
//  Ctor4    : (chain, ce, ci, type, key, namedKey, bot, true,
//              inputTrace, inputMustBeInput, nextOpType, chainBinOps, nextBinOp, constVals)
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
    bool requireConnectivity; // false for tree-shaped DAGs (calibration, histogram, tone_stats)

    // Ctor4 profile-variant fields
    std::vector<int> inputTrace;         // producer OpType per input (-1 = any)
    std::vector<int> inputMustBeInput;   // input indices that must be graph Input
    int nextOpType;                      // required OpType of the op after the chain (-1 = any)
    std::vector<std::pair<int, int>> chainBinOps;  // (chainPos, BinOp) requirements
    int nextBinOp;                       // required BinaryOp sub-type of the next op (-1 = any)
    std::vector<float> constVals;        // expected const blob float values (1e-4 tolerance)

    // 6-arg base
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(-1), noFuse(false), requireConnectivity(true),
          nextOpType(-1), nextBinOp(-1) {}

    // 7-arg with BinaryOpOperation
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 BinaryOpOperation bo)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(static_cast<int>(bo)), convWeightElems(-1), noFuse(false),
          requireConnectivity(true),
          nextOpType(-1), nextBinOp(-1) {}

    // 7-arg with noFuse bool (algo_cct guard pattern)
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 bool nf)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(-1), noFuse(nf), requireConnectivity(true),
          nextOpType(-1), nextBinOp(-1) {}

    // 8-arg nullptr_t + convWeightElems
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 std::nullptr_t, int cw)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(cw), noFuse(false), requireConnectivity(true),
          nextOpType(-1), nextBinOp(-1) {}

    // 8-arg BinaryOpOperation + noFuse
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 BinaryOpOperation bo, bool nf)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(static_cast<int>(bo)), convWeightElems(-1), noFuse(nf),
          requireConnectivity(true),
          nextOpType(-1), nextBinOp(-1) {}

    // 9-arg: nullptr_t + convWeightElems + requireConnectivity
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 std::nullptr_t, int cw, bool rc)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(cw), noFuse(false),
          requireConnectivity(rc),
          nextOpType(-1), nextBinOp(-1) {}

    // Ctor4: profile variant with inputTrace, inputMustBeInput, nextOpType,
    //        chainBinOps, nextBinOp, constVals
    //   ExactPattern({chain}, ce, ci, type, key, namedKey, bot, true,
    //                 {inputTrace}, {inputMustBeInput}, nextOpType,
    //                 {{chainPos, binOp}, ...}, nextBinOp, {constVals})
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 BinaryOpOperation bo, bool /*nf*/,
                 std::vector<int> itr, std::vector<int> imbi,
                 int notype,
                 std::vector<std::pair<int, int>> cbo,
                 int nbo,
                 std::vector<float> cv)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(static_cast<int>(bo)), convWeightElems(-1), noFuse(true),
          requireConnectivity(true),
          inputTrace(std::move(itr)), inputMustBeInput(std::move(imbi)),
          nextOpType(notype), chainBinOps(std::move(cbo)),
          nextBinOp(nbo), constVals(std::move(cv)) {}

    // Ctor4-lite: profile variant without binOp, using nullptr
    //   ExactPattern({chain}, ce, ci, type, key, namedKey, nullptr,
    //                 {inputTrace}, {inputMustBeInput}, nextOpType,
    //                 {{chainPos, binOp}, ...}, nextBinOp, {constVals})
    ExactPattern(std::vector<OpType> ch, int ce, int ci,
                 const char* t, const char* k, const char* nk,
                 std::nullptr_t,
                 std::vector<int> itr, std::vector<int> imbi,
                 int notype,
                 std::vector<std::pair<int, int>> cbo,
                 int nbo,
                 std::vector<float> cv)
        : chain(std::move(ch)), constElems(ce), constIndex(ci),
          ispType(t), typeKey(k), namedKey(nk),
          binOp(-1), convWeightElems(-1), noFuse(false),
          requireConnectivity(true),
          inputTrace(std::move(itr)), inputMustBeInput(std::move(imbi)),
          nextOpType(notype), chainBinOps(std::move(cbo)),
          nextBinOp(nbo), constVals(std::move(cv)) {}
};

} // namespace MNN
