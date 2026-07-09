# Contributing to ISP Rectifier

Thank you for your interest in contributing! This document outlines the process for contributing to the ISP Rectifier project.

## Code of Conduct

By participating in this project, you agree to abide by our [Code of Conduct](CODE_OF_CONDUCT.md). Please be respectful and inclusive in all interactions.

## How to Contribute

### 1. Reporting Bugs

- Check existing issues first to avoid duplicates
- Use the bug report template when creating a new issue
- Include:
  - Clear, descriptive title
  - Steps to reproduce
  - Expected vs actual behavior
  - Environment details (OS, Python/Rust versions, GPU if applicable)
  - Relevant logs/error messages

### 2. Suggesting Features

- Open a feature request issue
- Describe the problem you're solving
- Explain the proposed solution
- Consider backwards compatibility

### 3. Code Contributions

#### Prerequisites

- Python 3.10+
- Rust 1.75+
- ONNX Runtime
- NVIDIA GPU (for CUDA training, optional)

#### Development Setup

```bash
# Clone the repository
git clone https://github.com/your-org/softisp.git
cd softisp/isp-rectifier

# Python dependencies
pip install -r requirements.txt
pip install -e ".[dev]"
pre-commit install

# Rust
cargo build --features onnx-runtime

# Verify setup
make check-system
```

#### Making Changes

1. **Fork the repository** and create a feature branch:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Follow coding standards**:
   - Python: Follow PEP 8, use type hints
   - Rust: Follow rustfmt, use clippy suggestions
   - Add tests for new functionality

3. **Run tests and linting**:
   ```bash
   make test
   make lint
   make test-rust
   make lint-rust
   ```

4. **Commit with clear messages**:
   ```
   feat: add FP16 quantization support
   
   - Add FP16 conversion in quantize.py
   - Update validation script for FP16 models
   - Update documentation
   ```

5. **Push and create a Pull Request**

#### Pull Request Guidelines

- Keep PRs focused and atomic
- Update documentation for user-facing changes
- Add tests for new functionality
- Ensure CI passes
- Respond to review feedback promptly

### 4. Model Contributions

If you have trained models to contribute:

1. Ensure models are trained on properly licensed data
2. Provide training configuration and dataset statistics
3. Include validation results
4. Submit via model repository or release artifacts

## Development Workflow

### Python Code Style

- Line length: 100 characters
- Type hints required for public APIs
- Docstrings for public functions (Google style)
- Use `ruff` for linting and formatting

```bash
# Auto-fix
ruff check --fix .
ruff format .
```

### Rust Code Style

- Follow standard rustfmt
- Clippy warnings treated as errors
- Use `anyhow` for error handling
- Document public APIs with `///`

```bash
cargo fmt --all
cargo clippy --all-targets --all-features -- -D warnings
```

### Git Commit Convention

Follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>[optional scope]: <description>

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Formatting, no code changes
- `refactor`: Code restructuring
- `perf`: Performance improvement
- `test`: Adding tests
- `chore`: Maintenance tasks

### Running CI Locally

```bash
# Full CI pipeline
make ci-full

# Individual steps
make install-dev
make test
make test-rust
make lint
make lint-rust
make train
make export
make quantize
make validate
make check
```

## Testing

### Python Tests

```bash
# All tests with coverage
pytest tests/ -v --cov=src --cov-report=html

# Specific test
pytest tests/test_inference.py::test_feature_vector -v
```

### Rust Tests

```bash
# All tests
cargo test --all-features

# Specific test
cargo test test_feature_vector -- --nocapture
```

## Documentation

- Update docstrings for API changes
- Update README for new features
- Add examples for complex functionality
- Keep CHANGELOG updated

## Release Process

1. Update version in `Cargo.toml` and `pyproject.toml`
2. Update CHANGELOG.md
3. Create git tag: `git tag v1.0.0`
3. Push tag: `git push origin v1.0.0`
4. GitHub Actions will build and publish release

## Getting Help

- 📖 Check the [documentation](docs/)
- 💬 Join our [Discord/Slack] (link TBD)
- 🐛 Open an [issue](https://github.com/your-org/softisp/issues)
- 📧 Email: team@example.com

## Recognition

Contributors are recognized in:
- CONTRIBUTORS.md
- Release notes
- Project README

Thank you for contributing! 🎉