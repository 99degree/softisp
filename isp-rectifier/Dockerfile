# Dockerfile for ISP Rectifier
# Multi-stage build: train -> export -> runtime

# =============================================================================
# Stage 1: Build Environment (Python + PyTorch + ONNX)
# =============================================================================
FROM nvidia/cuda:12.1-devel-ubuntu22.04 AS builder

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 \
    python3.10-dev \
    python3-pip \
    git \
    wget \
    cmake \
    build-essential \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Set Python
RUN ln -s /usr/bin/python3.10 /usr/bin/python

WORKDIR /app

# Install Python dependencies
COPY pyproject.toml requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy source
COPY . .

# =============================================================================
# Stage 2: Training (run separately, not in final image)
# =============================================================================
FROM builder AS trainer

# This stage is for CI/CD training only
# Run: docker build --target trainer -t isp-rectifier-trainer .
# Then: docker run --gpus all isp-rectifier-trainer

ENTRYPOINT ["python", "distill_model.py"]

# =============================================================================
# Stage 3: Rust Build (for ONNX inference library)
# =============================================================================
FROM rust:1.75-slim AS rust-builder

WORKDIR /app

# Install system deps for Rust
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy Cargo files first for caching
COPY Cargo.toml Cargo.lock ./
COPY src ./src

# Build release
RUN cargo build --release --features onnx-runtime

# =============================================================================
# Stage 4: Runtime Image (minimal)
# =============================================================================
FROM nvidia/cuda:12.1-runtime-ubuntu22.04 AS runtime

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    libgomp1 \
    libopencv-core4.5 \
    libopencv-imgproc4.5 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy ONNX models
COPY --from=builder /app/fusedispcontroller.onnx ./models/
COPY --from=builder /app/fusedispcontroller_int8.onnx ./models/
COPY --from=builder /app/fusedispcontroller_fp16.onnx ./models/
COPY --from=builder /app/MODEL_CARD.md ./models/
COPY --from=builder /app/schema.json ./models/

# Copy Rust binary
COPY --from=rust-builder /app/target/release/isp-rectifier ./bin/

# Copy Python inference script (optional)
COPY --from=builder /app/requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt && rm requirements.txt
COPY --from=builder /app/inference.py ./

# Environment
ENV MODEL_PATH=/app/models/fusedispcontroller_int8.onnx
ENV RUST_LOG=info

# Non-root user
RUN useradd -m -u 1000 ispuser && chown -R ispuser:ispuser /app
USER ispuser

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD ./bin/isp-rectifier --health-check || exit 1

# Entry point
ENTRYPOINT ["./bin/isp-rectifier"]

# =============================================================================
# Stage 5: Development Image (with all tools)
# =============================================================================
FROM builder AS dev

# Install dev tools
RUN pip install --no-cache-dir \
    pytest \
    pytest-cov \
    mypy \
    ruff \
    pre-commit \
    ipython \
    jupyter

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /app

CMD ["bash"]