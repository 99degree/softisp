# Direct Vulkan feature leaks into unified pipeline tests

## Symptom

`cargo test -p cam-isp --lib --features "rectifier,direct_vulkan" --no-fail-fast`
fails two unified-pipeline tests:

- `unified_pipeline::tests::test_unified_force_input16`
- `unified_pipeline::tests::test_unified_4k_bayer_fhd_argb8888`

Both fail with `InvalidInput("DirectVulkanEngine requires input and output AHardwareBuffers")`.

## Reproduction

```bash
cargo test -p cam-isp --lib --features "rectifier,direct_vulkan" --no-fail-fast
```

## Root cause

`DirectVulkanEngine` registered factories at priority 98/97 and reported
`priority() == 98`. `UnifiedConfig::default()` uses `engine_preference = "auto"`,
so enabling the `direct_vulkan` feature made `select_engine()` choose the
direct engine ahead of MNN/CPU in every `UnifiedPipeline` test. Those tests pass
plain `ProcessParams` with no `AHardwareBuffer`s, so the direct engine correctly
rejected them.

The loader integration change did not alter this selection logic; it only made
the feature get compiled in a new build/test combination.

## Fix

- `DirectVulkanEngine::priority()` now returns 0, so `select_engine()` skips it
  during `auto` selection while the factories remain registered for explicit
  selection.
- `select_engine_by_name()` no longer applies the auto-selection `priority() >
  0` gate to explicitly named engines, so `direct_vulkan` still resolves when
  requested by name.

## Findings

- Auto selection remains MNN/CPU; direct Vulkan is opt-in only.
- Named engine lookup is the contract for the direct path and must stay
  independent of auto priority.
