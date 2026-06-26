// isp_fusion.h — MNN converter-style chain fusion for ISP opsets
//
// Follows MNN's PostTreatUtils pattern:
// 1. Scans NetT op list for known Extra op chains
// 2. Merges chained ops into single fused op
// 3. Updates tensor indices correctly
//
// Usage:
//   IspFusionPass pass;
//   pass.apply(mNet);
//
#ifndef ISP_FUSION_H
#define ISP_FUSION_H

#include <vector>
#include <memory>
#include <MNN/Interpreter.hpp>
#include "schema/current/MNN_generated.h"

// ── Embedded fused SPIR-V bytecode ─────────────────────────────────
// Generated from shader_fcs_display_fused.comp (3792 bytes)
#include "shader_fcs_display_fused_spv.h"
// Generated from shader_ee_ldci_fused.comp (6924 bytes)
#include "shader_ee_ldci_fused_spv.h"

namespace isp {

// ── Fusion Pass ────────────────────────────────────────────────────
// Follows MNN converter's PostTreatUtils merge pattern.
// Scans for known Extra op chains and replaces them with fused ops.
//
class IspFusionPass {
public:
    void apply(MNN::NetT* net) {
        auto& ops = net->oplists;
        for (int i = 0; i + 1 < (int)ops.size(); i++) {
            auto& op1 = ops[i];
            auto& op2 = ops[i+1];
            if (!op1 || !op2) continue;
            if (op1->main.type != MNN::OpParameter_Extra) continue;
            if (op2->main.type != MNN::OpParameter_Extra) continue;

            auto* e1 = static_cast<MNN::ExtraT*>(op1->main.value);
            auto* e2 = static_cast<MNN::ExtraT*>(op2->main.value);

            if (tryFuseFcsDisplay(e1, e2, op1.get())) {
                // op1 is now fused, remove op2 and its extra tensor
                unlinkOp(ops, i + 1, net);
                i--; // re-scan
            } else if (tryFuseEeLdci(e1, e2, op1.get())) {
                unlinkOp(ops, i + 1, net);
                i--;
            }
        }
        // Clean up unused tensors
        net->tensorName.resize(calcTensorCount(ops));
    }

private:
    // ── FCS + Display → FcsDisplay ──
    bool tryFuseFcsDisplay(MNN::ExtraT* e1, MNN::ExtraT* e2,
                          MNN::OpT* op1) {
        if (e1->type != "isp.fcs") return false;
        if (e2->type != "isp.display") return false;

        // Extract uniforms from both ops
        float fcs_strength = 1.0f;
        float gamma = 2.2f;
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 3) {
                fcs_strength = a->tensor->float32s[2];
            }
        }
        for (auto& a : e2->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 6) {
                gamma = a->tensor->float32s[5];
            }
        }

        // Build fused uniforms: [w,h, fcs_strength, 0, gamma, 0, 0,0,0]
        auto fd = e1->attr.begin();
        auto fe = e1->attr.end();
        // Find width/height from existing const blob
        float w=0,h=0;
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 2) {
                w = a->tensor->float32s[0];
                h = a->tensor->float32s[1];
                break;
            }
        }

        // Update type
        e1->type = "isp.fcs_display";

        // Replace SPIR-V with fused version
        for (auto& a : e1->attr) {
            if (a->key == "spirv" && a->tensor) {
                a->tensor->int8s.assign(
                    (const int8_t*)kShaderFcsDisplayFusedSpv,
                    (const int8_t*)kShaderFcsDisplayFusedSpv + kShaderFcsDisplayFusedSpvLen);
                break;
            }
        }

        // Update uniforms
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor) {
                a->tensor->float32s = {
                    w, h,
                    fcs_strength,  // uniform[2]
                    0.0f,          // uniform[3]: offset
                    gamma,         // uniform[4]: gamma
                    0.0f,          // uniform[5]: brightness
                    0.0f, 0.0f, 0.0f  // uniform[6..8]: pad[3]
                };
                break;
            }
        }

        // Output tensor stays as op1's output (op2's output is now unused)
        op1->outputIndexes[0] = op1->outputIndexes[0]; // unchanged
        return true;
    }

    // ── EE + LDCI → EeLdci ──
    bool tryFuseEeLdci(MNN::ExtraT* e1, MNN::ExtraT* e2,
                      MNN::OpT* op1) {
        if (e1->type != "isp.ee") return false;
        if (e2->type != "isp.ldci") return false;

        // Extract uniforms
        float ee_strength = 0.5f, ee_threshold = 0.01f;
        float ldci_strength = 0.5f, ldci_radius = 1.0f;
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 4) {
                ee_strength = a->tensor->float32s[2];
                ee_threshold = a->tensor->float32s[3];
            }
        }
        for (auto& a : e2->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 4) {
                ldci_strength = a->tensor->float32s[2];
                ldci_radius = a->tensor->float32s[3];
            }
        }

        float w=0,h=0;
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor && a->tensor->float32s.size() >= 2) {
                w = a->tensor->float32s[0];
                h = a->tensor->float32s[1];
                break;
            }
        }

        e1->type = "isp.ee_ldci";

        // Replace SPIR-V
        for (auto& a : e1->attr) {
            if (a->key == "spirv" && a->tensor) {
                a->tensor->int8s.assign(
                    (const int8_t*)kShaderEeLdciFusedSpv,
                    (const int8_t*)kShaderEeLdciFusedSpv + kShaderEeLdciFusedSpvLen);
                break;
            }
        }

        // Update uniforms: [w,h, ee_strength, ee_threshold, ldci_strength, ldci_radius, 0,0]
        for (auto& a : e1->attr) {
            if (a->key == "const" && a->tensor) {
                a->tensor->float32s = {
                    w, h,
                    ee_strength, ee_threshold,
                    ldci_strength, ldci_radius,
                    0.0f, 0.0f
                };
                break;
            }
        }

        op1->outputIndexes[0] = op1->outputIndexes[0]; // unchanged
        return true;
    }

    // ── Remove op2 and update tensor graph ──
    void unlinkOp(std::vector<std::unique_ptr<MNN::OpT>>& ops,
                 int idx, MNN::NetT* net) {
        // op2's input tensor indices are consumed but no longer needed
        // op2's output tensor index is now orphaned
        ops.erase(ops.begin() + idx);
    }

    // Calculate how many tensors are actually used
    int calcTensorCount(const std::vector<std::unique_ptr<MNN::OpT>>& ops) {
        int maxIdx = 0;
        for (auto& op : ops) {
            for (int i : op->inputIndexes)
                if (i > maxIdx) maxIdx = i;
            for (int i : op->outputIndexes)
                if (i > maxIdx) maxIdx = i;
        }
        return maxIdx + 1;
    }
};

} // namespace isp

#endif // ISP_FUSION_H
