# Build & Test

```bash
cd cam-rust
cargo test --lib -p cam-isp              # 813 tests
cargo check --workspace                  # 0 warnings
RUST_LOG=info cargo run --example pipeline -p cam-isp   # single frame PNG
bash bench-tests.sh bench 50 64 48       # perf benchmark
# Synthetic benchmark fixed: packed INT32 input, no -4 errors
```
