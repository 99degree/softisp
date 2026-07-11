# softisp Project Improvement Roadmap

## Project Status Summary

| Component | Status | Maturity |
|-----------|--------|----------|
| ISP Pipeline (Rust) | ✅ Production | 95% |
| FiLM-ISP Model | ✅ Trained | 90% |
| Teacher Models | ✅ Integrated | 85% |
| ONNX Export | ✅ Working | 90% |
| INT8 QAT | ✅ Working | 85% |
| MNN/Vulkan Backend | ✅ Production | 90% |
| Training Pipeline | ⚠️ Partial | 60% |
| Data Pipeline | ⚠️ Partial | 40% |
| Documentation | ⚠️ Partial | 50% |
| CI/CD | ❌ Missing | 0% |
| Benchmarks | ⚠️ Partial | 40% |

---

## Priority 1: Critical (Blockers for Production)

### 1.1 Data Pipeline - **CRITICAL**
- [ ] **Real training data collection pipeline**
  - [ ] DNG frame ingestion from target sensors (IMX, OV, Samsung)
  - [ ] Metadata extraction (EXIF + 3A stats) sidecar JSON generation
  - [ ] Automated teacher model inference pipeline
  - [ ] Dataset versioning & schema validation
  - [ ] Train/val/test split with stratification by scene/lighting
  
- [ ] **Data augmentation pipeline**
  - [ ] Histogram noise injection (Poisson + Gaussian)
  - [ ] Metadata jitter (exposure, WB, ISO)
  - [ ] Synthetic scene generation for edge cases
  - [ ] Domain randomization for robustness

- [ ] **Dataset versioning & lineage**
  - [ ] DVC or similar for data versioning
  - [ ] Schema validation (JSON schema for metadata)
  - [ ] Data cards / datasheets for each dataset version

### 1.2 Training Pipeline Hardening
- [ ] **Distillation loss improvements**
  - [ ] Add hue-preserving CCM loss (determinant ~1, trace ~3)
  - [ ] Perceptual tone loss (LPIPS or SSIM on rendered patches)
  - [ ] Confidence-weighted loss (teacher uncertainty weighting)
  
- [ ] **QAT improvements**
  - [ ] Per-channel quantization for sensitive layers
  - [ ] Learned quantization parameters
  - [ ] Mixed-precision search (HAQ)

- [ ] **Training infrastructure**
  - [ ] Distributed training support (DDP)
  - [ ] Mixed precision (AMP) with loss scaling
  - [ ] Gradient accumulation for large effective batch
  - [ ] WandB/TensorBoard integration
  - [ ] Model checkpointing with best-val tracking

### 1.3 Evaluation & Benchmarks
- [ ] **Per-skin-tone evaluation** (Fitzpatrick I-VI)
- [ ] **Low-light benchmark** (ELD, LOL, synthetic)
- [ ] **Latency profiling** on target hardware (A78, X1, Gen2)
- [ ] **Memory profiling** (peak RSS, VRAM)
- [ ] **Power profiling** (mW per frame)
- [ ] Regression test suite (golden images)

---

## Priority 2: High Impact

### 2.1 FiLM-ISP Model Improvements
- [ ] **FiLM architecture upgrades**
  - [ ] Hypernetwork for FiLM params (condition on full metadata)
  - [ ] Multi-head FiLM (separate γ/β per ISP block)
  - [ ] Conditional LayerNorm instead of FiLM

- [ ] **Input representation**
  - [ ] Learned histogram embedding (1D CNN instead of raw bins)
  - [ ] Metadata encoder upgrade (Transformer vs MLP)
  - [ ] Joint histogram+metadata cross-attention

- [ ] **Output heads**
  - Uncertainty heads (aleatoric uncertainty per output)
  - Constrained outputs (CCM determinant ≈ 1, WB gains > 0)
  - Tone curve monotonicity constraint

### 2.2 Data Quality
- [ ] **Teacher model ensemble**
  - Uncertainty quantification from teacher disagreement
  - Active learning: prioritize samples with high teacher disagreement
  
- [ ] **Real data collection**
  - Target: 50k+ diverse frames (indoor/outdoor/night/portrait/landscape)
  - Stratified by: lighting, scene, skin tone, sensor type
  - Partner with device vendors for proprietary data

### 2.3 Two-Stage ISP Architecture
- [ ] **Stage 2 Model** (async enhancement)
  - Denoising (NAFNet / Restormer)
  - Super-resolution (x2, x4)
  - HDR merge (Mertens + neural refine)
  - Night mode (multi-frame align + merge)
- [ ] Async pipeline integration (Stage 1 → queue → Stage 2)

---

## Priority 3: Medium Impact

### 3.1 Deployment & Operations
- [ ] **CI/CD Pipeline**
  - GitHub Actions / GitLab CI
  - Rust: cargo test, clippy, fmt, audit
  - Python: pytest, black, mypy, ruff
  - ONNX model validation (shape, opset, inference test)
  - Benchmark regression detection

- [ ] **Model Registry**
  - MLflow or DVC for model versioning
  - Model cards auto-generated from training metadata
  - Staging → Production promotion workflow

- [ ] **Monitoring**
  - Inference latency histograms (p50/p95/p99)
  - Accuracy drift detection (statistical tests)
  - Data drift detection (input distribution shift)

### 3.2 Documentation
- [ ] API reference (rustdoc + pdoc)
- [ ] Architecture decision records (ADRs)
- [ ] Hardware integration guide (HAL interface)
- [ ] Model card template (Google model card format)
- [ ] Security audit (unsafe Rust audit, supply chain)

### 3.3 Teacher Models
- [ ] CCMNet: Update to latest architecture
- [ ] Time-Aware AWB: Multi-camera support
- [ ] Neural ISP Tuning: Add more ISP parameters
- [ ] Teacher distillation: Feature matching loss (intermediate features)

---

## Quick Wins (1-2 days each)

| Task | Effort | Impact |
|------|--------|--------|
| Add Fitzpatrick skin tone evaluation | 1 day | High |
| Add low-light benchmark (LOL/ELD) | 1 day | High |
| CI/CD with GitHub Actions | 2 days | High |
| Model card auto-generation | 1 day | Medium |
| Per-skin-tone evaluation script | 1 day | High |
| INT8 calibration dataset generation | 1 day | High |
| Golden image regression tests | 2 days | High |
| WandB/TensorBoard integration | 1 day | Medium |

---

## Technical Debt

| Area | Issue | Fix |
|------|-------|-----|
| `distill_model.py` | Single file, 3000+ lines | Split into modules |
| `distill_model.py` | Hardcoded paths/config | Config-driven |
| `distill_model.py` | No type hints | Add full typing |
| `mnnengine.rs` | Large file, mixed concerns | Split modules |
| `mnnengine.rs` | No async inference API | Add async API |
| Tests | Sparse coverage | Target 80% coverage |
| `distill_model.py` | No distributed training | Add DDP support |

---

## Hardware Validation Matrix

| Device | SoC | ISP | Status |
|--------|-----|-----|--------|
| Pixel 7/8 | Tensor G2/G3 | Google ISP | ❌ Untested |
| Galaxy S23/24 | Exynos/Snapdragon | Samsung ISP | ❌ Untested |
| Xiaomi 13/14 | Snapdragon 8 Gen 2/3 | QTI ISP | ❌ Untested |
| iPhone 15/16 | A17/A18 | Apple ISP | N/A (closed) |
| Reference | Snapdragon 8 Gen 2 | QTI ISP | ⚠️ Partial |

---

## Long-term Research

- [ ] **Self-supervised pre-training** (MAE on RAW histograms)
- [ ] **Domain adaptation** (sim-to-real for synthetic data)
- [ ] **Neural ISP blocks** (learnable demosaic/denoise)
- [ ] **Online adaptation** (few-shot per-device calibration)
- [ ] **Diffusion-based HDR/denoise** (quality ceiling)
- [ ] **Neural ISP search** (NAS for ISP block config)

---

## Quick Wins (Do This Week)

```bash
# 1. Add Fitzpatrick evaluation
python -c "
import numpy as np
from sklearn.metrics import mean_squared_error
# Add per-Fitzpatrick-type evaluation
"

# 2. Add low-light benchmark
# python -m pytest tests/test_lowlight.py

# 3. Setup GitHub Actions
# .github/workflows/ci.yml

# 4. Model card generator
python scripts/generate_model_card.py --model fusedispcontroller_int8.onnx
```

---

## Success Metrics (Quarterly)

| Metric | Current | Target (Q1) | Target (Q2) |
|--------|---------|-------------|-------------|
| Dark skin WB ΔE | 0.018 | < 0.010 | < 0.005 |
| Dark skin hue ΔE | 0.022 | < 0.012 | < 0.008 |
| Low-light WB ΔE | 0.018 | < 0.012 | < 0.008 |
| False alarm rate | 8.2% | < 3% | < 1% |
| INT8 latency (A78) | 1.05ms | < 0.8ms | < 0.5ms |
| Model size (INT8) | 5.5 MB | < 4 MB | < 3 MB |
| Training data | 5k | 50k | 200k |
| Fitzpatrick coverage | 0 types | 3 types | 6 types |
| CI/CD | 0% | 100% | 100% |
