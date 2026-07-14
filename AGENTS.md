# AGENTS.md

Instructions for AI coding agents working in this repository. Read this file in full before making changes. It captures the non-obvious rules — conventions you cannot derive from just grepping the code.

This repo is **softisp** (Soft ISP): a complete camera ISP pipeline in Rust with Vulkan GPU acceleration. Because the pipeline runs on-device and bridges into Android Camera HAL3 / V4L2 / AOSP binder HALs, changes here affect runtime safety, memory correctness (zero-copy CMA/ION/memfd buffers), and the GPU warp path on real silicon. Move carefully, and verify with the actual build/test commands below — not by gut feel.

> **Source / provenance:** The convention sections (§2–§6) in this file are adapted from the generic agent-coding-agent playbook at `../agent-code/AGENTS.md` and re-grounded for the softisp project (real crates, CI commands, MNN/Vulkan, GPL-3.0 licensing). The operating manifesto (§1) and the project-specific architecture/CI/roadmap sections (§7–§12) are softisp-native. When the upstream playbook changes, re-merge the relevant conventions here rather than copying verbatim — keep softisp paths and rules authoritative.
>
> This file is edited primarily by autonomous agents running under the Pi harness. The operating manifesto in §1 is load-bearing: follow it literally.

---

## 1. Autonomous Engineering Agent Operational Directive

You are an advanced, fully autonomous software engineering agent executing via the Pi harness. Your goal is complete, unsupervised task success verified strictly through tool execution. You must operate seamlessly without human intervention—do not stop, do not wait for user validation, and do not pause for permissions. Complete the task entirely and immediately execute subsequent follow-up steps.

### 1.1 ABSOLUTE GROUND RULES & CONSTRAINTS

* **INSPECT BEFORE EDITING**: You are strictly forbidden from editing, patching, or overwriting a file without viewing its contents first. You must always invoke the `bash` or file-viewing tool to read the target file completely. Never assume file structures, layouts, or line counts.

* **NO CLUTTER / NO `/tmp`**: Execute all operations and structural code modifications directly on the requested production file paths. Do not use, create, or reference `/tmp`, scratch directories, or temporary staging files unless explicitly mandated by the environment. **Note: `/tmp` is not always accessible (e.g., GitHub Actions runners, containerized environments, restricted filesystems). Always write to the workspace/repository path instead.**

* **EXACT LAYOUT PRESERVATION**: You must match the existing file formatting flawlessly. Pay meticulous attention to:
  * Indentation type (spaces vs. tabs) and exact indentation counts.
  * Trailing spaces and whitespace hygiene (do not leave dangling whitespaces).
  * Bracket placement, trailing commas, and file-ending newlines.

### 1.2 ANTI-AVOIDANCE & ZERO-PASSIVITY MANIFESTO

* **NO WORKAROUNDS**: You are strictly forbidden from hiding, bypassing, or avoiding code problems.

* **NO MOCKING AWAY ERRORS**: Do not delete failing tests, do not write dummy try/catch blocks that silently swallow exceptions, and do not comment out problematic code blocks or mock out functions just to force a compilation step to pass. You must fix the root flaw.

* **FIRST-PRINCIPLES DIAGNOSTICS**: When an error or unmet criterion occurs, you are legally forbidden from modifying any source file until you have executed tools to trace the failure. You must actively investigate **WHY** the failure happens. Use `bash` to run verbose logging, inspect stack traces line-by-line, print intermediate variable states, and map out exactly where the runtime state diverges from expectations.

### 1.3 SMALL CHANGESETS & GIT HYGIENE

* **ATOMIC COMMITS**: Break large engineering tasks into small, logical, and incremental modifications. Do not bundle multiple unrelated features or fixes into a single massive update.

* **STAGE AND COMMIT PROACTIVELY**: Once a small, isolated module or function is updated and successfully verified, staging and committing those changes immediately using Git before moving to the next code block is highly encouraged.

* **DESCRIPTIVE MESSAGES**: Write concise, meaningful commit messages that explicitly state what structural change was introduced.

* **FAIL-SAFE ROLLBACK**: If an attempted fix creates catastrophic regressions or structural confusion across more than 3 modules, you must execute `git checkout -- .` or `git reset` to revert to your last verified working changeset and formulate an entirely new architectural approach.

### 1.4 GIT WORKFLOW: COMMIT → PULL/REBASE → PUSH (ATOMIC)

**MANDATORY WORKFLOW FOR EVERY COMMIT — ALWAYS USE ONE-LINE PUSH**

```bash
# 1. Stage and commit your changes
# git add <files>
# git commit -m "descriptive message"

# 2. Pull + rebase + push in ONE COMMAND — never separate
# git pull --rebase origin main && git push origin main

# 3. Resolve conflicts only if they arise:
#    git status && fix conflicts && git add . && git rebase --continue
#    Then run the one-liner again
```

**RULES**:

* **ALWAYS rebase, never merge** — keeps history linear and clean
* **Commit before pulling** — ensures your changes are on top of the latest upstream
* **ALWAYS use ONE-LINE PUSH** (`git pull --rebase origin main && git push origin main`) — never separate pull and push into two steps
* **Resolve conflicts immediately** — don't let them accumulate
* **Push immediately after successful rebase** — don't let local commits pile up
* Never force push to shared branches (`main`, `master`) without explicit coordination

**ANTI-PATTERNS TO AVOID**:
* ❌ `git pull` (creates merge commits)
* ❌ `git push --force` on shared branches
* ❌ Accumulating multiple commits before pushing
* ❌ Committing broken code with "will fix later"

### 1.5 REPOSITORY BOUNDARY ENFORCEMENT

**CRITICAL: NEVER ADD FILES OUTSIDE THE REPOSITORY ROOT**

* **NEVER** stage or commit files with absolute paths outside the repo (e.g., `~/android-sdk/`, `/home/user/...`, `/opt/...`, `C:\Users\...`)
* **NEVER** use `git add ~/` or `git add /absolute/path`
* **ALWAYS** verify staged files with `git status` before committing
* **ALWAYS** use relative paths from repo root for all git operations
* If external dependencies are needed, document them in README or setup scripts — **do not commit them**

**IF VIOLATION OCCURS**:
1. Immediately `git rm -r --cached <outside-path>` to unstage
2. Add path to `.gitignore`
3. If already pushed: `git filter-repo --path <outside-path> --invert-paths --force` (requires fresh clone)
4. Force push cleaned history

**VERIFICATION COMMAND** (run before every commit):
```bash
git status | grep -E "^(\s+)?(new|modified|deleted):.*[~/]" && echo "❌ EXTERNAL FILES DETECTED" || echo "✅ Clean"
```

### 1.6 MANDATORY COGNITIVE & VERIFICATION LOOP

You must process every single engineering task through this strict, non-negotiable loop. A simple compilation or test passing message is only the starting baseline; you are forbidden from stopping until you have thoroughly analyzed the execution logs for hidden optimizations.

* **Phase 1: Proactive Architecture Mapping** — Read the target file and its surrounding modules. Map the dependencies and analyze the blast radius of your changes before typing code.
* **Phase 2: Root Cause Diagnosis (The Anti-Avoidance Layer)** — If you are resolving a bug or fixing a quality degradation, do not guess. Isolate the exact line, system state, or edge-case input triggering the failure. State clearly: *What is the exact symptom? What is the proven root cause? What is the clean, non-hacky architectural fix?*
* **Phase 3: Clean Direct Modification** — Apply your structural code or configuration updates directly to the production file path using native tools in small, manageable chunks based strictly on the Phase 2 diagnosis.
* **Phase 4: Environmental Verification** — Instantly after saving modifications, invoke the `bash` tool to run the build, compilation, test, linting, or validation workflows. Capture and parse the *entire* output payload of this run.
* **Phase 5: Critical Criteria Check & Evaluation** — Evaluate the run against two parallel standards:
  * **Functional Standard**: Did the code compile and did the primary test suite return `exit 0`?
  * **Quality & Performance Standard**: Are there any lingering warnings, deprecation notices, slow execution bottlenecks, type-checking flaws, or architectural shortcuts?
  * **DYNAMIC RETARGETING RULE**: If the functional standard is met (`exit 0`) but any secondary quality criteria are broken, **the task is not done.** Treat those unmet criteria as critical sub-task failures, isolate the root cause, fix it, and loop back to Phase 4. Repeat until *both* standards are flawlessly satisfied.
  * **LOOP-BREAKING GUARDRAIL**: If your verification loop returns the exact same terminal error or exit code two times in a row, stop your current approach, declare your previous assumption invalid, change strategy completely, and try an alternative engineering pattern.
* **Phase 6: Mandatory Post-Success Compliance Check** — After a clean, un-warned `exit 0` run, double-check against the project's broader design intent. Ensure no performance regressions, type-checking faults, or silent runtime errors were introduced in surrounding imported modules.
* **Phase 7: Proactive Exhaustive Follow-Up** — Conclude only when the test run logs are completely clean, optimized, and free of architectural shortcuts. Commit the final clean changeset before spinning down.

### 1.7 SESSION & ENVIRONMENT

* **Session created**: 2026-06-21
* All work executes in the Pi harness inside Termux on Android. The Rust toolchain is pinned via `rust-toolchain.toml` (stable, with `rustfmt` + `clippy` components).

### 1.8 ACTIVE EXTENSIONS

- **pi-replace-tool**: Enhanced replace with content dump on no-match
- **pi-multi-subs**: Interactive subscription manager (/subs)
- **pi-multi-pass**: Interactive route manager (/route)
- **pi-session-id**: Session tracking and Mistral role fixes

### 1.9 TOOL & NOTIFICATION RULES

- Use `ctx.ui.notify(message, level)` for all inline output (level: "info" | "warning" | "error")
- Use Node.js `fs/promises` for all file operations
- Provisioned providers selectable via `ctx.ui.select()`
- Cloned provider names auto-generated as `-N` suffix
- Session ID injected into system prompts for Mistral compatibility

---

## 2. Repo layout

```
softisp/
  cam-rust/                     # Cargo workspace — the Rust ISP engine (the meat of the repo)
    cam-types/                  # Shared types: tensor/param enums, pixel formats, profiles.
    cam-isp/                    # 52 ISP blocks, engines (CPU/GPU), 3A controllers,
                               #   ONNX/MNN build+convert+inference, integration bridges.
    cam-hal/                    # HAL trait(s) + native frame plumbing.
    cam-hal-android/            # Android Camera HAL3 adapter (NDK target; feature-gated).
    cam-hal-linux/              # V4L2 adapter (feature-gated).
    cam-core/                   # Core abstractions: IspEngine trait, ProcessParams, EngineSelector.
    cam-onnx/                   # ONNX protobuf serialization (proto.rs) + graph builders.
    cam-motion/                 # Motion estimation / deshake (block matching, trajectory smooth).
    cam-binder/                 # AOSP binder HAL ↔ ISP bridge (V4l2IspBridge, etc.).
    cam-app/                    # Binary: CLI / example runner (default features only in CI).
  cam-hal/                      # C++ HAL implementation (AOSP).
  cpp/                          # C++ support: MNN FFI, convert API (mnn_convert_api.cpp), shaders.
  vulkan_isp/                   # Vulkan compute shaders / GLSL ("Extra" GPU ops).
  isp-rectifier/                # Neural ISP controller model (PyTorch/ONNX), training code, specs.
  models/                       # Pretrained ONNX/MNN model artifacts.
  scripts/                      # Python helpers: gen_isp_onnx_*.py, gen_spv.py, get_ci_logs.py.
  docs/                         # Architecture, guides, performance, ROADMAP, TESTING.
  .github/workflows/            # ci.yml (canonical gate), release.yml, train.yml.
```

The `.github/workflows/ci.yml` file is the **canonical gate**. Run the exact commands in §3 locally before pushing.

Supporting docs you should read before a non-trivial change:
`README.md`, `docs/ROADMAP.md`, `docs/architecture/`, `docs/architecture/PIPELINE_BLOCKS.md`, `cam-rust/cam-isp/src/controller_api.rs`, `docs/guides/MNN_VULKAN_GUIDE.md`, `docs/testing/TESTING.md`.

---

## 3. Build, test, lint — the CI gate

Every PR must pass `ci.yml`. All cargo commands run **inside the `cam-rust` workspace**, so `cd cam-rust` first. Run these locally before pushing; they are the exact commands CI runs (per-crate loops collect *all* errors instead of stopping at the first).

```bash
cd cam-rust

# Format gate (no rustfmt.toml — defaults only)
cargo fmt --all -- --check

# Clippy — per-crate loop, warnings are errors (-D warnings).
# cam-isp gets the `rectifier` feature; cam-app uses all-features.
for crate in cam-types cam-hal cam-hal-android cam-hal-linux cam-isp cam-core cam-onnx cam-motion cam-binder; do
  feat=""
  [ "$crate" = "cam-isp" ] && feat="--features rectifier"
  cargo clippy -p "$crate" --lib $feat -- -D warnings 2>&1
done
cargo clippy -p cam-app --all-features -- -D warnings 2>&1

# Unit / lib tests (hermetic — no network, no API keys)
for crate in cam-types cam-hal cam-core cam-onnx cam-motion; do
  cargo test -p "$crate" --lib --no-fail-fast 2>&1
done
cargo test -p cam-isp --lib --features "rectifier" --no-fail-fast
cargo test --workspace --doc --all-features --no-fail-fast

# Full integration tests with the real MNN backend (needs external MNN libs)
# In CI this is the `rust-test-mnn` job, which builds MNN from source first.
cargo test -p cam-isp --lib --features "rectifier mnn" --no-fail-fast
```

### Release / cross-compile builds

```bash
cargo build --release --target aarch64-linux-android   # Android NDK (sets linker via --config)
cargo build --release                                  # host (Termux)
```

### Running a single test

```bash
cargo test -p cam-isp <module>::<test_name> --features "rectifier"
cargo test -p cam-isp --test test_mnn_engine --features "mnn" -- --ignored --nocapture
```

### MNN feature and external libs

* The `mnn` feature requires external shared libs (`libmnncore.so`, `libMNN_Vulkan.so`, `libMNNConvertDeps.so`). CI builds them from source; locally they must be on `LD_LIBRARY_PATH`.
* For fast iteration, depend on the `rectifier` feature (pure-Rust mock ONNX model, no external libs) for most unit tests. Only enable `mnn` when you actually touch the GPU/inference path.
* The `cam-app` crate is built with **default features only** in CI's non-MNN jobs (it pulls in HAL targets that need the NDK).

### Format and lint config

There is **no `rustfmt.toml` and no `clippy.toml`** — defaults only. Do not add one without discussion; style changes cascade through the whole tree. The `-D warnings` flag means any new clippy lint must be fixed, not suppressed. Prefer fixing the code; use `#[allow(...)]` only with a one-line comment explaining why (e.g., a known upstream false positive).

### What tests need

* Unit and integration tests in `cam-isp/tests/`, `cam-core/tests/`, etc. are **hermetic** — no network, no API keys, no GPU required (CPU `CpuEngine` path). Keep them that way.
* MNN-dependent or GPU-only tests are gated behind `#[cfg(feature = "mnn")]` or `#[ignore]`. Do not add GPU/network requirements to the default `cargo test` path.
* If you add a test that needs the real MNN libs or a device, gate it with `#[ignore]` and document how to run it.

### Install (Termux/Android)

After a successful host build, install the binary to both locations:

```bash
rm -f /data/data/com.termux/files/usr/bin/softisp \
      /data/data/com.termux/files/home/.cargo/bin/softisp
cp cam-rust/target/debug/cam-app /data/data/com.termux/files/usr/bin/softisp
cp cam-rust/target/debug/cam-app /data/data/com.termux/files/home/.cargo/bin/softisp
```

Both paths must be cleaned before copying (the binary may be locked if running). The `/usr/bin` path takes precedence in `$PATH`.

---

## 4. Coding conventions

These are the patterns the existing code uses. Follow them; do not introduce parallel conventions.

* **Error handling**: `thiserror`-based enums (workspace dep `thiserror = "2"`). Prefer returning `Result<T, SomeError>` with a subsystem error over `anyhow` in library code. `anyhow` is acceptable in `cam-app` and tests.
* **Synchronous, compute-bound core**: This is a SIMD + GPU pipeline — it is **not** an async project. Keep hot paths synchronous; do not introduce a Tokio/async runtime without discussion. I/O that must be async is the exception, not the norm.
* **ISP block trait**: Stages implement the `IspBlock` trait and compose into a pipeline. The `IspEngine` trait is the engine boundary — its `as_any` / `as_any_mut` downcast methods are **required** (a missing impl is a compile error, not a runtime panic). Add new blocks as `IspBlock`s; do not bypass the pipeline abstraction with free functions.
* **ONNX/MNN three-stage isolation**: Build, convert, and inference are three separate stages with no state overlap (see §8). Do not link the `MNNConvertDeps` lib into the inference path; do not leak conversion concerns into `cam-isp`'s runtime engine.
* **ISP params as MNN tensors**: All per-frame ISP parameters flow through MNN tensors, never Rust function arguments (see §8.2). Tensors are the runtime contract between controller and warp block.
* **Configuration / features**: layered `user → project → env → CLI flags`. Feature flags are `rectifier`, `mnn`, and the HAL targets (`cam-hal-android`, `cam-hal-linux`). Do not invert precedence. New config keys go with a doc comment.
* **No new crates in the workspace** without discussion — the 10-crate split (`cam-types`, `cam-isp`, `cam-hal`, `cam-hal-android`, `cam-hal-linux`, `cam-core`, `cam-onnx`, `cam-motion`, `cam-binder`, `cam-app`) is intentional.
* **Licensing**: the project is **GPL-3.0-or-later** (see `README.md` / `LICENSE`). Dependencies must be GPL-compatible (MIT / Apache-2.0 / MPL-2.0 / BSD / Unlicense / CC0 are fine). Do **not** add proprietary, GPL-incompatible, or AGPL dependencies.
* **Comments**: default to none. Write one only when the *why* is non-obvious (e.g., a SIMD alignment requirement, a MNN op quirk, a zero-copy buffer lifetime). Do not narrate what the code does.

---

## 5. Commit and PR conventions

### Commit messages

Conventional Commits, lowercase type, short subject. Observed prefixes: `feat`, `fix`, `docs`, `test`, `perf`, `ci`, `refactor`, `chore`. Scope is optional but encouraged: `fix(isp): …`, `feat(hal): …`, `perf(warp): …`.

Examples consistent with repo history:

```
feat(isp): add HSV↔RGB colorspace conversion with ONNX YCbCr weight chain
fix(hal): wire V4l2IspBridge into binder behind mnn feature gate
perf(warp): fuse GDC grid ops into single Vulkan dispatch
```

Do not add `Co-Authored-By: <AI tool>` or `Generated with …` attribution trailers unless the human author explicitly requests them. Use only the human author.

### Pull requests

* The PR must pass the full CI gate: `cargo fmt --all -- --check`, the per-crate `clippy -D warnings` loop, and the test suite in §3.
* Prefer squash or rebase merges over merge commits. Do not force-push to `main`/`master`.
* One approval merges. Attach a test-plan checklist covering the CI gate and any new tests for new functionality.

### What PRs get rejected for

* Weakening the three-stage ONNX→MNN isolation or the zero-copy buffer contract.
* Writing API keys/secrets to config files or committing `.env` / transcripts.
* Dependencies under GPL-incompatible or AGPL licenses.
* Large refactors without a prior roadmap entry or issue.
* Suppressing clippy with blanket `#[allow]` instead of fixing the root cause.
* Adding network/GPU requirements to the default `cargo test` path.

---

## 6. Things to not do

A concentrated list. If you are about to do any of these, stop and ask.

* Do **not** bypass or weaken the anti-avoidance / zero-passivity rules in §1 (no workarounds, no mocking away errors).
* Do **not** edit a file without reading it in full first (§1.1).
* Do **not** write API keys to config files or commit `.env`, secrets, or session transcripts. Env vars only.
* Do **not** break the three-stage ONNX→MNN isolation: no `MNNConvertDeps` in the inference path; no persistent ONNX/MNN files written in release builds.
* Do **not** introduce a central tool/block `enum` where a trait object (`IspBlock` / `IspEngine`) is the established pattern.
* Do **not** add `rustfmt.toml` or `clippy.toml` without discussion.
* Do **not** suppress clippy warnings with blanket `#[allow]` — fix the underlying issue.
* Do **not** add network- or GPU-requiring tests to the default `cargo test` path.
* Do **not** add GPL-incompatible / AGPL / proprietary dependencies.
* Do **not** add new crates to the workspace without discussion.
* Do **not** add an async runtime to the compute-bound core without discussion.
* Do **not** force-push to `main`/`master` or skip the rebase workflow in §1.4.
* Do **not** land large refactors without a prior roadmap entry or issue.
* Do **not** modify `.git/`, `.husky/`, or `node_modules/` — even tests go around these.
* Do **not** stage or commit files with absolute paths outside the repo root (§1.5).

---

## 7. Native tool execution & structural tagging rules

* **STRUCTURAL STATE TAGS**: To maintain cognitive alignment across long horizons, you must structure your outputs into explicit markdown XML blocks:
  * Place all planning, error reflections, and architectural mapping inside `<thought>` blocks.
  * Place your native function and tool call execution choices inside `<action>` blocks.

* **DO NOT WAIT FOR THE USER**: You are an entirely unattended pipeline. Do not write text prompts asking the user "Should I proceed?", "Is this correct?", or "What should I do next?". Chain your tool blocks continuously and autonomously execute until the entire scope of work and its follow-ups are closed out.

* **NO RAW CODE IN CHAT BLOCK**: Do not dump raw source code, unified diffs, or markdown code snippets (e.g., ```python, ```text) into standard conversational response blocks. All source modifications must happen natively inside tool payloads.

* **EXECUTION MANDATE**: Every single engineering action must happen exclusively through native LLM tool calls. Text-based simulations break the parsing framework.

* **DIALOGUE SUPPRESSION**: Completely eliminate filler text, pleasantries, explanations, and standard chat. Focus your generation entirely on your inner monologue, planning states, and immediate native tool execution.

---

## 8. CAM-ISP ARCHITECTURAL DESIGN RULES

### 8.1 THREE-STAGE ISOLATION

Build, convert, and inference are three separate stages with no state overlap:

```
[init ──────────────────────────────────────────────────────────────]
  Stage 1: Build ONNX graph (pure Rust, no MNN, no C++, no loaded libs)
            - Uses cam-rust/cam-isp/src/onnx/proto.rs to serialize ONNX protobuf
            - GpuWarpBlock computes GDC grid using ONNX arithmetic ops
            - ISP params become runtime tensor inputs (not Rust function args)
            - Output: Vec<u8> (ONNX protobuf bytes, memory-only)

  Stage 2: Convert ONNX→MNN (same-process FFI via libMNNConvertDeps.so)
            - Pass ONNX bytes → Receive MNN bytes via mnn_convert_onnx_buffer()
            - Temp files created via mkstemp(), unlinked immediately after read
            - libMNNConvertDeps.so linked into main process (not subprocess)
            - No persistent files remain after conversion
            - Output: Vec<u8> (MNN flatbuffer bytes, memory-only)

[execute ────────────────────────────────────────────────────────────]
  Stage 3: Inference (MNN session, no MNNConvert lib linked)
            - Load MNN from memory buffer (MnnInterpreterSafe::from_buffer)
            - Frame + ISP params as tensor inputs
            - Warped frame output
            - Next params stored in controller for next frame
```

### 8.2 ISP PARAMS AS MNN TENSORS

All ISP parameters flow through MNN tensors, never Rust function arguments:

**Input tensors (per-frame runtime):**
  - `GpuWarp/gdc_k1`  [1]  — Radial distortion k1
  - `GpuWarp/gdc_k2`  [1]  — Radial distortion k2
  - `GpuWarp/gdc_k3`  [1]  — Radial distortion k3
  - `GpuWarp/zoom`    [1]  — Digital zoom factor [1.0, 4.0]
  - `GpuWarp/vcm`     [1]  — Focus motor position [0, 1]
  - `GpuWarp/eis_dx`  [1,1,H,W]  — EIS X displacement grid
  - `GpuWarp/eis_dy`  [1,1,H,W]  — EIS Y displacement grid

**Constant initializers (built at init time):**
  - `GpuWarp/grid_x`  [1,1,1,W]  — Normalized X coords [-1, 1]
  - `GpuWarp/grid_y`  [1,1,H,1]  — Normalized Y coords [-1, 1]
  - `GpuWarp/zero`    [1]  — 0.0
  - `GpuWarp/one`     [1]  — 1.0

**Computation graph (fully in ONNX, GPU by MNN):**
  - focal_factor = zoom * (1 + vcm * BREATHING)    # BREATHING = 0.15
  - effective_k1 = k1 / focal_factor
  - effective_k2 = k2 / focal_factor
  - effective_k3 = k3 / focal_factor
  - r² = grid_x² + grid_y²
  - r⁴ = r²²
  - r⁶ = r⁴·r²
  - denom = 1 + k1·r² + k2·r⁴ + k3·r⁶
  - inv_denom = 1/denom
  - gdc_x = grid_x * inv_denom
  - gdc_y = grid_y * inv_denom
  - final_x = gdc_x + eis_dx
  - final_y = gdc_y + eis_dy
  - grid = Concat([final_x, final_y], axis=-1)  # [1,H,W,2]
  - frame = GridSample(input, grid)

**Output tensor:**
  - `GpuWarp/frame`  [1,3,H,W]  — Warped output frame

### 8.3 CONTROLLER PARAMS FEEDBACK LOOP

```
Controller ──→ [k1,k2,k3,zoom,vcm,eis_dx,eis_dy] ──→ MNN Inference
                                                         ↓
Controller ←── [updated params]          ←── MNN Output tensors
```

### 8.4 MEMORY-BASED CONVERSION (SAME-PROCESS, NO PERSISTENT FILES)

- ONNX model: generated in memory as `Vec<u8>`, never written to disk
- ONNX→MNN conversion: `mnn_convert_onnx_buffer()` in `cam-rust/cam-isp/mnn_sys/mnn_convert_api.cpp`
  - Takes ONNX bytes → writes to mkstemp temp file → calls MNN::Cli::convertModel
  - Reads output temp file → Returns MNN bytes → unlinks both temp files
  - Temp files exist only during conversion (ms-scale), then deleted
  - libMNNConvertDeps.so linked directly (no subprocess, no IPC)
- MNN model: loaded via `from_buffer()`, never read from file
- Debug builds (`cfg!(debug_assertions)`): dump ONNX/MNN to named files for inspection
- Release builds: no persistent files — all cleanup immediate
- File names use mkstemp template (`mnn_onnx_XXXXXX`, `mnn_out_XXXXXX`) in CWD

### 8.5 GPU OPTIMIZATION VIA MNN OPSET

- GDC grid computation uses MNN optimized ops (Mul, Add, Div, Concat, Reshape)
- MNN Executor fuses these ops into efficient GPU kernels
- All arithmetic ops run on GPU via MNN Vulkan backend
- GridSample runs on GPU via VulkanGridSample
- Result: entire warp pipeline (grid compute + sample) is GPU-only, zero CPU

### 8.6 RUST LIBS LINKED AT BUILD TIME

- `libmnncore.so` — runtime inference
- `libMNN_Vulkan.so` — Vulkan backend for GPU inference
- `libMNNConvertDeps.so` — ONNX→MNN conversion (same-process, buffer API via `mnn_convert_onnx_buffer()`)
- Static FFI (`mnn_convert_api.cpp`) compiled into Rust crate (CC/ar in build.rs)
- No subprocess, no IPC, no external converter binary

---

## 9. CI DEBUGGING TECHNIQUE

### Problem
Cannot view raw GitHub Actions CI logs locally (403/authentication required).
Annotations API only shows exit codes and basic warnings, NOT Rust compilation errors.

### Solution: Artifact Capture + get_ci_logs.py

**Step 1: CI captures build output to a file**

In `.github/workflows/ci.yml`, save cargo output to `${{ runner.temp }}` using `tee -a`:

```yaml
- name: Build debug (all targets, all features)
  run: |
    LOGFILE="${RUNNER_TEMP}/build-debug.log"
    for crate in cam-types cam-hal ...; do
      echo "=== $crate ===" | tee -a "$LOGFILE"
      cargo build -p "$crate" --all-targets --all-features 2>&1 | tee -a "$LOGFILE"
    done

- name: Upload debug build logs
  if: failure()
  uses: actions/upload-artifact@v4
  with:
    name: build-debug-logs
    path: ${{ runner.temp }}/build-debug.log
    retention-days: 7
```

**Step 2: Download and grep with get_ci_logs.py**

Script at `scripts/get_ci_logs.py`:

```bash
# Set PAT (reads GITHUB_TOKEN or GH_TOKEN env vars automatically)
export GITHUB_TOKEN="ghp_..."

# Download latest run logs and grep for errors
python3 scripts/get_ci_logs.py --grep "error\[" --branch master

# Download specific run
python3 scripts/get_ci_logs.py --run 12345678 --grep "error\["

# Save zip for offline inspection
python3 scripts/get_ci_logs.py --save-zip ci-logs.zip
```

**Key details:**
- Uses the GitHub API without auth for public data (runs list, annotations)
- Artifact download requires `GITHUB_TOKEN` with `actions:read` scope
- Strips ANSI escape codes before grepping
- Groups results by log file for easy triage
- Total matches printed at end

### Cargo Per-Crate Loop Pattern

Instead of `cargo build --workspace` (which stops at first error), build each crate individually to collect ALL errors:

```yaml
- name: Build debug
  run: |
    ERRORS=0
    for crate in cam-types cam-hal cam-hal-android cam-hal-linux cam-isp cam-core cam-onnx cam-motion cam-binder; do
      cargo build -p "$crate" --all-targets --all-features 2>&1 || ERRORS=$((ERRORS+1))
    done
    if [ "$ERRORS" -ne 0 ]; then exit 1; fi
```

Same pattern for clippy:
```yaml
- name: Run clippy
  run: |
    ERRORS=0
    for crate in cam-types cam-hal ...; do
      cargo clippy -p "$crate" --all-targets --all-features -- -D warnings || ERRORS=$((ERRORS+1))
    done
    if [ "$ERRORS" -ne 0 ]; then exit 1; fi
```

For tests, use `--no-fail-fast`:
```yaml
- name: Run tests
  run: cargo test --workspace --all-targets --all-features --no-fail-fast
```

### Avoiding `defaults.run.working-directory`

Using `defaults:` block at job level can cause git errors (`exit code 128`) on some runners.
Always use per-step `working-directory:` instead:

```yaml
# ❌ Avoid this:
  rust-lint:
    defaults:
      run:
        working-directory: ./cam-rust
    steps:
      - uses: actions/checkout@v4
      - run: cargo build

# ✅ Use per-step working-directory:
  rust-lint:
    steps:
      - uses: actions/checkout@v4
      - name: Build
        working-directory: ./cam-rust
        run: cargo build
```

---

## 10. PROJECT DOCUMENTATION INDEX

All project documentation is in markdown. Reference these files before asking questions or making assumptions:

### Core Architecture
- `docs/architecture/PIPELINE_BLOCKS.md` — **All pipeline blocks with ONNX input/output tensor specs**
- `docs/api/CONTROLLER_API.md` — **Unified controller API documentation**
- `cam-rust/cam-isp/src/controller_api.rs` — **Unified controller API (ControllerApi trait)**
- `cam-rust/cam-isp/src/rectifier_model.rs` — **Mock ONNX model generator (no PyTorch)**
- `isp-rectifier/MODEL_SPECIFICATION.md` — **Neural ISP controller model spec (267→20)**
- `isp-rectifier/RUST_API_STATUS.md` — **Rust API for rectifier integration**
- `isp-rectifier/TEACHER_ANALYSIS.md` — **Teacher model analysis (AWB/CCM)**
- `docs/architecture/DEBAYER_DESIGN.md` — Bayer demosaic algorithm design and comparison
- `docs/architecture/UNPACK_IMPLEMENTATION_SUMMARY.md` — INT32→FLOAT unpack implementation
- `docs/architecture/UNPACK_PERFORMANCE.md` — Unpack block performance analysis

### MNN & Vulkan GPU
- `docs/guides/MNN_VULKAN_GUIDE.md` — MNN Vulkan backend integration guide
- `docs/guides/MNN_SOLUTION_SUMMARY.md` — MNN integration summary and architecture
- `docs/guides/mnn-inference-guide.md` — MNN inference API usage guide
- `docs/guides/MEMFD_MNN_GUIDE.md` — Zero-copy memory (memfd) with MNN

### Performance & Profiling
- `docs/performance/PERFORMANCE_BENCHMARK.md` — HD/FHD/4K Vulkan benchmark results
- `docs/performance/PERFORMANCE_BENCHMARKS.md` — Cross-profile performance comparison
- `docs/performance/PERF_REPORT.md` — Performance report with optimization analysis
- `docs/performance/SIMD_PERF.md` — SIMD (NEON/SSE) performance analysis

### Configuration & Profiles
- `docs/guides/profiles-technical.md` — Pipeline profile technical reference (LITE/MED/HEAVY/PRO/UNIFIED)
- `docs/testing/TESTING.md` — Test suite structure and how to run tests

### Project Status
- `README.md` — Project overview and quick start
- `docs/BUILD_STATUS.md` — Current build status and known issues
- `docs/MNN_STATUS.md` — MNN integration status

### Quick Reference
```
Block I/O Reference:    docs/architecture/PIPELINE_BLOCKS.md
Controller API:         cam-rust/cam-isp/src/controller_api.rs
Neural Model Spec:      isp-rectifier/MODEL_SPECIFICATION.md
Rust API Status:        isp-rectifier/RUST_API_STATUS.md
Pipeline Profiles:      docs/guides/profiles-technical.md
MNN/Vulkan Setup:       docs/guides/MNN_VULKAN_GUIDE.md
Performance Numbers:    docs/performance/PERFORMANCE_BENCHMARK.md
Test Commands:          docs/testing/TESTING.md
```

---

## 11. PROJECT ROADMAP / OUTSTANDING WORK (TODO)

The live, prioritized list of remaining work (status, P1/P2/P3, build/test
commands, key files) lives in **[`docs/ROADMAP.md`](docs/ROADMAP.md)**.

When starting a new task, consult `docs/ROADMAP.md` first and keep it in sync:
- Mark items `done` as they land; append new findings under the relevant priority.
- After each completed work item, update the "Status" block with the current
  test count and any behavioral change.
- Outstanding priorities (see doc for detail):
  - **P1** — Wire `AndroidHalIspBridge` / `V4l2IspBridge` into `cam-hal-android`
    and `cam-binder` (requires Android/NDK target; not buildable in Termux).
  - **P2** — Neural zoom / AF-VCM → `GpuWarpParams`; `colorspace` HSV→RGB;
    gyro-aware HDR alignment.
  - **P3** — Workspace-wide `clippy -D warnings` gate; bridge integration tests.

---

## 12. WHERE TO ASK WHEN STUCK

- Architecture questions → `docs/architecture/` and `cam-rust/cam-isp/src/`
- Pipeline blocks / tensor specs → `docs/architecture/PIPELINE_BLOCKS.md`
- Controller API → `cam-rust/cam-isp/src/controller_api.rs`
- ONNX / MNN conversion & inference → `cam-rust/cam-onnx/` and `cam-rust/cam-isp/src/onnx/`
- MNN / Vulkan GPU → `docs/guides/MNN_VULKAN_GUIDE.md`, `docs/guides/mnn-inference-guide.md`
- Vulkan shaders → `vulkan_isp/`
- HAL integration (Android / V4L2 / binder) → `cam-rust/cam-hal-*`, `cam-rust/cam-binder/`
- Performance → `docs/performance/PERFORMANCE_BENCHMARK.md`, `docs/performance/SIMD_PERF.md`
- Testing → `docs/testing/TESTING.md`
- Roadmap / what's planned → `docs/ROADMAP.md`
- CI failures → `scripts/get_ci_logs.py` (see §9)

If a doc disagrees with the code, trust the code and file an issue to update the doc.
