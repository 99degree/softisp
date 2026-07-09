# Changelog

All notable changes to ISP Rectifier will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Initial project structure with PyTorch distillation pipeline
- Teacher dataset collection from DNG frames
- ONNX export with INT8/FP16 quantization
- Batch validation with statistical analysis
- CI/CD thresholds enforcement
- Rust inference crate with tract-onnx
- Register injection layer for ISP hardware
- Comprehensive test suite
- Docker support for training and inference
- GitHub Actions CI/CD workflows

### Changed
- N/A

### Deprecated
- N/A

### Removed
- N/A

### Fixed
- N/A

### Security
- N/A

---

## [0.1.0] - 2024-01-15

### Added
- Initial release of ISP Rectifier
- Distilled student model architecture
- Teacher dataset collection scripts
- ONNX export with dynamic axes
- INT8 dynamic quantization
- FP16 conversion
- Batch validation with statistical reporting
- CI/CD threshold checking
- Rust crate with tract-onnx inference
- Register injection with safety clamping
- Docker multi-stage build
- GitHub Actions CI/CD pipeline

---

## Version History

| Version | Date | Description |
|---------|------|-------------|
| 0.1.0 | 2024-01-15 | Initial release |

## Release Checklist

When releasing a new version:

- [ ] Update version in `Cargo.toml` and `pyproject.toml`
- [ ] Update `CHANGELOG.md`
- [ ] Run full test suite: `make ci-full`
- [ ] Build and test Docker image: `make docker-build && make docker-run`
- [ ] Create git tag: `git tag vX.Y.Z`
- [ ] Push tag: `git push origin vX.Y.Z`
- [ ] GitHub Actions will build and publish release
- [ ] Verify release artifacts on GitHub Releases
- [ ] Update documentation if needed

## Breaking Changes Policy

- Major version (X.0.0): Breaking changes allowed
- Minor version (0.X.0): New features, backwards compatible
- Patch version (0.0.X): Bug fixes only

When introducing breaking changes:
1. Add deprecation warnings first
2. Provide migration guide
3. Bump major version
4. Communicate clearly in release notes

## Supported Versions

| Version | Status | Support End |
|---------|--------|-------------|
| 0.1.x | Active | 2025-01-15 |