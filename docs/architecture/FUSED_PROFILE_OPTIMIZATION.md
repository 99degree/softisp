# Optimising the FUSED profile: what the measurements say

> **Scope**: where the frame time actually goes in `PipelineProfile::FUSED` and in
> `PipelineProfile::WHOLE`, which of the remaining costs are structural versus
> incidental, and what to change next — ordered by measured impact, not by how
> interesting the change is.
>
> **Basis**: probes in `cam-rust/cam-isp/tests/` that are runnable on the device.
> Every number below comes from one of them; nothing here is extrapolated from a
> smaller frame unless it says so.

---

## 1. Current state

Measured end to end — raw Bayer in, planar RGB out, 8 frames warmup, `p50` of
20–30 frames.

**3840×2160** (`PERF_FRAMES=30 cargo test -p cam-isp --test probe_4k_perf --features mnn -- --ignored --nocapture`)

| Profile | Blocks | p50 | p95 | FPS |
|---|---|---|---|---|
| MED (primitives) | 19 | 530.0 ms | 596.1 ms | 1.9 |
| FUSED | 12 | 322.9 ms | 417.6 ms | 3.1 |
| FUSED, statistics off | 12 | 306.3 ms | 387.9 ms | 3.3 |
| WHOLE (`isp.pipeline`) | 2 | 139.2 ms | 159.0 ms | 7.2 |

**1920×1080**

| Profile | Blocks | p50 | p95 | FPS |
|---|---|---|---|---|
| MED (primitives) | 19 | 249.1 ms | 300.4 ms | 4.0 |
| FUSED | 12 | 175.7 ms | 187.8 ms | 5.7 |
| WHOLE | 2 | 104.2 ms | 145.0 ms | 9.6 |

Host vs GPU, from `probe_4k_prep`:

```
prep  (CPU staging)     25.45 ms   (6%)
infer (MNN call)       433.08 ms  (96%)
total                  452.76 ms
```

**26 ms of host work against 433 ms inside MNN.** Every host-side lever in
`MNN_BUFFER_LIFECYCLE_AND_PERF.md` (the per-command `fopen`, session rebuilds,
`_finish()` syncs) is bounded by that 6%, so none of them is where the 4K frame
goes.

---

## 2. What the FUSED chain actually contains

`probe_op_inventory` counts op types in the composed 4K graph:

```
FUSED (stats full): 32 counted nodes
  Add          2      ReduceMax    3
  AveragePool  1      ReduceMean   6
  Clip         1      ReduceMin    3
  Concat       2      ReduceSum    2
  Conv         1      Size         1
  Div          1      Sub          2
  Greater      1      Mul          5
  Less         1
```

Only **one Conv** and **one Clip** — the image chain is genuinely fused. The 31
remaining nodes are the statistics branches. The image path is:

```
isp.unpack_blc_normal_cfa → isp.demosaic_ccm → tone → isp.ee
  → saturation → isp.display
```

four of those are single `isp.*` dispatches; `tone` and `saturation` are primitive
`Mul`/`Add`/`Clip` chains.

The statistics are:

```
              ┌─ zone_stats     AveragePool → ReduceMean/Min/Max → Concat
aux_hook_src ─┼─ channel_means  ReduceMean
  (or pooled) ├─ tone_stats     Conv(1×1 luma) → Mean/Min/Max/Greater/Less/Sum/Size → Concat
              └─ calibration    Mul → ReduceMean/Sub → ReduceMean → Concat
```

---

## 3. Where the 4K time goes

`probe_4k_stats_shared` sweeps the statistics lever with a single pool feeding all
four consumers (25 frames each):

| Statistics source | Aux blocks | p50 | Frame mean |
|---|---|---|---|
| full resolution | 4 | 438.6 ms | 0.7718 |
| pooled to 1080 | 5 | 307.4 ms | 0.7729 |
| pooled to 540 | 5 | 270.7 ms | 0.7740 |
| pooled to 384 | 5 | 305.2 ms | 0.7753 |

**~168 ms of a 439 ms frame — about 38% — is the statistics alone.** The calibration
quad means are `[0.9963, 1.0000, 0.9987]` at full resolution and `[0.9962, 1.0000,
0.9987]` pooled, i.e. identical to four decimals: this is a cheaper input, not an
approximation, because a mean is exact under a box filter.

`probe_4k_stages` attributes the rest, and is honest about its own noise:

```
no stats            225.3 ms  (67% of the chain)
no tone stats       105.6 ms  (31%)
no zone stats        82.4 ms  (24%)
no channel means     40.6 ms  (12%)
no saturation        33.7 ms  (10%)
no ee              -104.4 ms  (noise)
```

`no ee` coming out *negative* is the tell: `probe_4k_drift` measures the same
configuration twice, one after the other, and gets **437.1 ms then 298.0 ms** — a
139 ms spread. So at 4K only the *direction* of each row is trustworthy, not the
magnitude. The fusion verification work was therefore done at 1296×970 and the
per-stage attribution should be redone interleaved before being quoted.

---

## 4. Ranked opportunities

### P0 — delete the unconditional trace in `Pipeline::execute`

Already identified in `MNN_BUFFER_LIFECYCLE_AND_PERF.md`. Bounded by the 6% host
budget above, so it is at most ~25 ms at 4K — but it is also unconditional `fopen`
per command per frame on a phone, and free to remove.

### P1 — make `tone` and `saturation` one `isp.*` dispatch

They are the last two primitive stages in the image path, and both are per-pixel
functions of a pixel's own RGB with nothing between them:

```
tone:        Mul(exposure) → Mul(contrast) → Add(brightness) → Clip
saturation:  Mul(scale)
```

Five elementwise nodes over a 3-plane 4K frame. At 1296×970 `probe_tail_stage_cost`
measured tone at **−14.5 ms** (i.e. free) and saturation at **+0.5 ms**, so the win
here is dispatch count, not arithmetic. That is worth roughly 2 dispatches and two
full-frame round trips out of twelve.

Verification plan: one shader `isp_tone_sat`, stage mask as a coefficient tensor,
compared against the two-block form on the pattern set (`probe_profile_compare`),
which already distinguishes a real change from a flat-field artifact.

### P2 — give `isp.pipeline` (WHOLE) a real demosaic

`WHOLE` is **3.8× faster than MED at 4K** and 2.4× at FHD, on 2 blocks, and it gives
up exactly three things: the 3A readback, the edge-directed demosaic, and the EIS
warp. The demosaic is the one with a measurable cost to the picture (mean 0.6019
against MED's 0.6243 on a flat field).

Two sub-options, in order of effort:

1. **Keep the box filter, add a stage bit for a 3×3 edge-aware pass** inside the
   shader's RGB tail. The footprint average already produces a good luma; an
   unsharp on the luma plane would recover most of the perceived sharpness.
2. **Add the 3A readback** by publishing the area-summed channel means from the
   per-pixel thread. This needs a workgroup reduction the shader does not have.
   A cheaper route is a second tiny dispatch over the *output* grid, which is 1/16
   the pixels of the input — that is what `isp.cfa_downscale` was, and it fits.

Until one of those lands, `WHOLE` stays a fast preview path and `FUSED` stays the
default.

### P3 — stop re-allocating the session per geometry

`MNN_BUFFER_LIFECYCLE_AND_PERF.md` §P1/§P2. The pipeline rebuilds a session when the
frame size changes and uses `Session_Memory_Collect` rather than a cache. On a
camera that never changes size this should never fire — worth logging a warning if
it ever does, because it would explain a one-off stall rather than a slow average.

### P4 — fp16 for the parts of the graph that can take it

Every `isp.*` shader declares `float[]`, so a chain containing one forces fp32
storage for the whole session (`chain_has_fused_op`). `probe_fp16_storage` measured
what an fp16 variant of the shaders would be worth and found **nothing**: MED fp32
104.2 ms vs fp16 122.4 ms at 1296×970, i.e. inside the noise, because the arithmetic
would still be fp32 and only the buffers would halve.

So this is not a lever for FPS. It is a lever for *peak GPU memory*, which matters
because four fp32 sessions at 2592×1940 do not fit on this device.

### P5 — the invariant, now checked

`GraphComposer::check_extent_monotonicity` enforces that no stage reads more pixels
than its predecessor produced. That is not an optimisation — it is what stops a
block deriving its geometry from the wrong size and silently producing wrong
statistics, which is exactly what `ZoneStatsBlock`'s `h / rows` kernel does when
handed the sensor size instead of the pooled one. It runs during composition.

---

## 5. Things that look like optimisations and are not

| Approach | Why not |
|---|---|
| fp16 tensor storage | measured no faster (`probe_fp16_storage`); correctness risk for zero gain |
| Separately downscaling per statistics block | four traversals of a 100 MB tensor; the shared pool is 38% cheaper |
| A second `isp.*` dispatch for each stats reduction | the placeholder shaders (`isp.zone_stats`, `isp.tone_stats`, `isp.stats`) are copy-shims — enabling them silently returns the frame instead of the statistics |
| Removing the identity bridges / eliding no-ops | already done; 19 of 33 nodes were `Transpose(perm=[0,1,2,3])` |
| Merging the statistics into `isp.pipeline` | needs a workgroup reduction the shader does not have; see P2.2 |
| Tensor aliasing between ops | halves peak memory, zero latency — the pool already recycles |

---

## 6. How to reproduce

```bash
cd cam-rust

# throughput, all profiles
PERF_FRAMES=30 cargo test -p cam-isp --test probe_4k_perf --features mnn -- --ignored --nocapture

# the statistics lever
PERF_FRAMES=25 cargo test -p cam-isp --test probe_4k_stats_shared --features mnn -- --ignored --nocapture

# attribution — read the drift probe first, it says how much to trust these
PERF_FRAMES=20 cargo test -p cam-isp --test probe_4k_drift --features mnn -- --ignored --nocapture
PERF_FRAMES=20 cargo test -p cam-isp --test probe_4k_stages --features mnn -- --ignored --nocapture

# host vs GPU
PERF_FRAMES=20 cargo test -p cam-isp --test probe_4k_prep --features mnn -- --ignored --nocapture

# what the graph is made of
cargo test -p cam-isp --test probe_op_inventory --features mnn -- --nocapture
```

`PERF_W` / `PERF_H` override the geometry on all of them.
