# BayerWbBlock: design goal and runtime contract

`BayerWbBlock` is the ISP stage that applies white balance **while the image is
still a Bayer quad**, before demosaic mixes the four sensor sites into RGB. It
is small, but it sits at the boundary between sensor geometry, the controller,
and three different graph shapes (primitive, fused demosaic+CCM, whole
pipeline), so a misunderstanding here shows up as a colour cast that no later
stage can repair.

## 1. What the block is for

A RAW sensor does not output RGB. It outputs a mosaic where each pixel sits
behind one colour filter. After the CFA stage the frame is
`[1, 4, H/2, W/2]`: four planes, one per physical quad position
(top-left, top-right, bottom-left, bottom-right).

White balance is a per-colour scalar:

```text
corrected_site = raw_site * gain[colour_of_site]
```

Doing this **before** demosaic is important:

- each plane still contains exactly one colour, so the gain is exact;
- after demosaic each RGB pixel is a mixture of neighbouring sites, so the
  same correction becomes an approximation;
- the CCM and tone stages then operate on already-neutral RGB.

`BayerWbBlock` is therefore not a colour matrix and not a controller. It is the
gate that turns raw sensor channel imbalance into a neutral CFA quad.

## 2. The four-plane ordering is sensor-dependent

The block works in **plane order**, not in `[R,G,B]` order. The mapping comes
from `cam-rust/cam-isp/src/bayer_planes.rs` and uses Android's sensor colour
filter arrangement codes:

| Code | Pattern | Plane order TL, TR, BL, BR |
|---|---|---|
| 0 | RGGB | R, G, G, B |
| 1 | GRBG | G, R, B, G |
| 2 | GBRG | G, B, R, G |
| 3 | BGGR | B, G, G, R |

`IspController` produces three gains `[R,G,B]`. The engine expands them with
`bayer_planes::gains_by_plane()` before writing any tensor. Passing
`[R,G,B]` straight into the four tensor slots is only correct for RGGB; on this
device's GBRG sensor it puts the red gain on a green site and swaps the colour
cast permanently.

## 3. What the block emits

`BayerWbBlock` has exactly one frame output: `BayerWbBlock/frame`, shaped
`[1, 4, H, W]`. Its intermediate `BayerWbBlock/wb_applied` is an internal graph
node and is not a controller-visible output.

It has **no CCM metadata output**. There is no tensor such as
`BayerWbBlock/awb_metadata` or `BayerWbBlock/ccm_hint`. The controller does not
consume a WB block result. This is deliberate: the statistics tap feeding the
controller is placed **before** this block (`aux_hook_src/out`, after BLC and
before LSC/WB/demosaic), so AWB measures the scene rather than its own output.
Reading a post-WB tensor would make the loop chase itself.

The runtime inputs are:

| Tensor | Shape | Meaning |
|---|---|---|
| `BayerWbBlock/gains` | `[1,4,1,1]` | per-plane gains |
| `BayerWbBlock/zero` | scalar | lower clip bound |
| `BayerWbBlock/one` | scalar | upper clip bound |

The graph is `Mul(input, gains) -> Clip(0, 1) -> frame`.

`BayerWbBlock/gains` is the stable **logical** name for the 3A decision. The
engine always attempts that write first, even when the block was fused away,
then falls back to a physical sink. `DemosaicCcmBlock/w` is not a WB gain: it is
the whole `CCM × demosaic × WB` matrix the shader consumes. Renaming these
suffixes to `wb_gain` would be clearer but must be done across every block and
shader binding, so it is tracked as a P3 follow-up in `docs/overview/ROADMAP.md`.

### Identity special case

`BayerWbBlock::new()` uses identity gains. When all four gains are `1.0`, the
block emits a single `Identity` node and declares no extra inputs. This keeps
MNN's ISP-fusion chain from being broken by a no-op `Mul`.

Primitive profiles take this path: the profile builder constructs
`BayerWbBlock::new()` with identity gains. The live gains are not baked into
the graph; they are written from the controller every frame. The FUSED profile
goes one step further and omits `BayerWbBlock` entirely (see §5), so a converted
production session usually has **no** `BayerWbBlock/gains` tensor. That absence
is the signal that the gains must be folded into a surviving downstream tensor.

## 4. Where the live gains come from

The production app passes no explicit `bayer_gains`, `awb_gains`, or
`ccm_matrix` in `ProcessParams`. The controller is the source:

```text
stats tap (pre-WB CFA means)
    -> planes_to_rgb()
    -> IspController::update_channel_stats()
         AWB gains [R,G,B]
         CCT estimate
         select/smooth/sanitize CCM [3x3]
    -> controller state
```

`MnnEngine::process()` snapshots that state **before** inference, maps the AWB
gains to plane order, and writes the tensors for the current frame. The stats
of the current frame are read **after** inference and update the controller
for the next frame. So the loop is:

```text
frame N-1 stats -> controller -> frame N tensors -> frame N inference -> frame N stats -> ...
```

This is what "the controller emits the correspondent CCM tensor value for the
next frame" means in this codebase. There is no separate WB metadata tensor to
carry it.

## 5. Folding WB into the fused demosaic+CCM

The user's design intuition is correct: on the fused path `BayerWbBlock` is
**not needed**, because `DemosaicCcmBlock` already has runtime extra inputs
that can carry the white-balance decision. Its `Conv` weight is `[3,4]`, i.e.
one column per CFA plane, so a per-plane WB gain is exactly a column scale:

```text
fused_w[i,j] = Σ_k ccm[i,k] * demosaic[k,j] * wb_plane[j]
fused_b[i]   = -Σ_j fused_w[i,j] * blc[j]      (when BLC is still in raw counts)
```

Here `i` is the output RGB channel, `k` is the demosaic source channel, and `j`
is the CFA plane. `DemosaicCcmBlock::fuse_weights_ext()` implements this
algebra, and `profile_builder.rs` omits `BayerWbBlock` when
`fuse_stages.wb_into_demosaic` is allowed with the MNN ISP opset. On the actual
production chain the fused RAW head has already divided by `sensor_max` and
subtracted BLC, so `MnnEngine::set_extra_inputs()` writes the
already-normalized form without dividing again.

`BayerWbBlock` therefore remains useful only for primitive/unfused graphs, where
demosaic and CCM are separate and no other tensor can absorb a per-plane scale
before demosaic mixes the sites. That is why the block still exists: it is the
primitive-era spelling of the same operation, not an extra stage required by the
fused one.

`MnnEngine::set_extra_inputs()` chooses the sink from the **converted session's
input names**, because that is the ground truth after MNN's converter merges or
removes blocks:

| Session inputs | Where WB goes |
|---|---|
| `BayerWbBlock/gains` exists | write gains there; fold nothing |
| `DemosaicCcmBlock/w` exists | fold into the fused 3×4 weights |
| `DemosaicBlock/w` exists | fold into the primitive demosaic columns |
| only `CcmBlock_ccm/matrix` remains | last-resort per-colour scale on CCM |

Names that are absent are logged once per build at debug level. They are not
errors: they say which primitive was merged away. Logging them every frame was
itself a measurable cost and hid the tensor that actually carried the value.

## 6. Why the earlier "missing tensor" warnings mattered

The old per-frame lines:

```text
BayerWbBlock/gains: no such tensor in the session
CcmBlock_ccm/matrix: no such tensor in the session
```

were not proof that the graph ran with zeros. They were evidence that the
engine was targeting primitive tensor names in a fused graph. The fix is not to
silence them, but to:

1. select the surviving sink from the session;
2. write the controller's AWB + CCM into that sink;
3. report absent names once, with the reason.

## 7. Practical invariants

- AWB statistics must be read from the **pre-WB** tap.
- Gains must be expanded from `[R,G,B]` to plane order using the reported
  sensor pattern.
- When a real `BayerWbBlock/gains` input exists, do not also fold WB into
  demosaic/CCM — that would apply it twice.
- When it does not exist, WB must be folded somewhere; dropping it leaves a
  permanent cast.
- The controller's CCM and the WB fold both come from the same snapshot, so
  they describe the same frame generation.

These invariants are covered by `bayer_planes.rs` tests and by the fused-sink
selection in `mnnengine.rs`. They are the reason a "small" white-balance block
must be studied together with the controller and the fused demosaic stage.
