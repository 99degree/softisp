# ISP Rectifier Makefile
# Common development workflows

.PHONY: help install install-dev test test-rust lint lint-rust format format-rust \
        train export quantize validate check benchmark clean docker-build docker-run \
        download-models check-system

# Default target
help:
	@echo "ISP Rectifier - Development Workflows"
	@echo ""
	@echo "Python:"
	@echo "  install         Install Python dependencies"
	@echo "  install-dev     Install development dependencies"
	@echo "  test            Run Python tests"
	@echo "  lint            Run Python linting (ruff, mypy)"
	@echo "  format          Format Python code (ruff format)"
	@echo ""
	@echo "Rust:"
	@echo "  test-rust       Run Rust tests"
	@echo "  lint-rust       Run Rust linting (clippy)"
	@echo "  format-rust     Format Rust code (rustfmt)"
	@echo "  build-rust      Build Rust crate in release mode"
	@echo ""
	@echo "ML Pipeline:"
	@echo "  download-models Download teacher model weights"
	@echo "  collect-data    Collect teacher dataset from DNG frames"
	@echo "  train           Train distilled student model"
	@echo "  export          Export ONNX model"
	@echo "  quantize        Quantize ONNX to INT8/FP16"
	@echo "  validate        Validate quantized models"
	@echo "  check           Check thresholds (CI/CD)"
	@echo "  benchmark       Benchmark inference latency"
	@echo ""
	@echo "System:"
	@echo "  check-system    Run system health checks"
	@echo "  clean           Clean build artifacts"
	@echo ""
	@echo "Docker:"
	@echo "  docker-build    Build Docker image"
	@echo "  docker-run      Run Docker container"
	@echo ""

# Python dependencies
install:
	cd isp-rectifier && pip install -r requirements.txt

install-dev:
	cd isp-rectifier && pip install -r requirements.txt
	pip install pytest pytest-cov mypy ruff pre-commit
	pre-commit install

# Testing
test:
	cd isp-rectifier && python -m pytest tests/ -v --cov=src --cov-report=term-missing

test-rust:
	cd isp-rectifier && cargo test --all-features

# Linting
lint:
	cd isp-rectifier && ruff check src/ scripts/ tests/
	cd isp-rectifier && mypy src/ --ignore-missing-imports

lint-rust:
	cd isp-rectifier && cargo clippy --all-targets --all-features -- -D warnings

# Formatting
format:
	cd isp-rectifier && ruff format src/ scripts/ tests/
	cd isp-rectifier && ruff check --fix src/ scripts/ tests/

format-rust:
	cd isp-rectifier && cargo fmt --all

# Build
build-rust:
	cd isp-rectifier && cargo build --release --features onnx-runtime

# ML Pipeline
download-models:
	cd isp-rectifier && python scripts/download_models.py --all --output-dir models --verify

collect-data:
	@echo "Usage: make collect-data INPUT_DIR=/path/to/dngs METADATA_DIR=/path/to/json OUTPUT_DIR=teacher_dataset"
	@echo "Or use synthetic data: python isp-rectifier/collect_teacher_dataset_production.py --synthetic 10000 --output teacher_dataset"

collect-data-real:
	cd isp-rectifier && python collect_teacher_dataset_production.py \
		--input-dir $(INPUT_DIR) \
		--metadata-dir $(METADATA_DIR) \
		--output-dir $(OUTPUT_DIR) \
		--ccmnet-weights models/ccmnet.pth \
		--awb-weights models/time_aware_awb.pth \
		--isp-tuning-weights models/neural_isp_tuning.pth

collect-data-synthetic:
	cd isp-rectifier && python collect_teacher_dataset_production.py --synthetic 10000 --output teacher_dataset

train:
	cd isp-rectifier && python distill_model.py --train \
		--dataset teacher_dataset/teacher_dataset.npz \
		--epochs 100 \
		--batch-size 32 \
		--lr 1e-3 \
		--checkpoint-dir checkpoints

train-gpu:
	cd isp-rectifier && python distill_model.py --train \
		--dataset teacher_dataset/teacher_dataset.npz \
		--epochs 100 \
		--batch-size 64 \
		--lr 1e-3 \
		--device cuda \
		--checkpoint-dir checkpoints

export:
	cd isp-rectifier && python distill_model.py --export \
		--model checkpoints/best_model.pth \
		--output fusedispcontroller.onnx

quantize:
	cd isp-rectifier && python quantize.py fusedispcontroller.onnx

validate:
	cd isp-rectifier && python batch_validate_quantization.py \
		--fp32_model fusedispcontroller.onnx \
		--quant_model fusedispcontroller_int8.onnx \
		--samples 1000

validate-fp16:
	cd isp-rectifier && python batch_validate_quantization.py \
		--fp32_model fusedispcontroller.onnx \
		--quant_model fusedispcontroller_fp16.onnx \
		--samples 1000

check:
	cd isp-rectifier && python check_thresholds.py --strict

benchmark:
	cd isp-rectifier && python scripts/benchmark.py \
		fusedispcontroller.onnx \
		fusedispcontroller_int8.onnx \
		fusedispcontroller_fp16.onnx \
		--runs 1000 --warmup 50

# System checks
check-system:
	cd isp-rectifier && python scripts/check_system.py

# Clean
clean:
	cd isp-rectifier && rm -rf __pycache__ .pytest_cache .mypy_cache .ruff_cache
	cd isp-rectifier && rm -rf checkpoints teacher_dataset *.onnx *.npz MODEL_CARD.md plots/
	cd isp-rectifier && cargo clean

# Docker
docker-build:
	docker build -t isp-rectifier:latest -f isp-rectifier/Dockerfile isp-rectifier

docker-run:
	docker run --rm -it --gpus all \
		-v $(PWD)/isp-rectifier:/app/isp-rectifier \
		isp-rectifier:latest

docker-train:
	docker run --rm --gpus all \
		-v $(PWD)/isp-rectifier:/app/isp-rectifier \
		isp-rectifier:latest \
		bash -c "cd /app/isp-rectifier && make train-gpu"

# CI helpers
ci-test: install-dev test test-rust lint lint-rust

ci-train: download-models collect-data-synthetic train export quantize validate check

ci-full: ci-test ci-train