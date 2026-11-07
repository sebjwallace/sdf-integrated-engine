# 5. Thin parts tangling, and sharp edges

## The problem

These are the weakest spots in the contact code. Everything here passes its checks,
but the overlaps are larger than anywhere else:

| Case | Deepest brief overlap | Check limit |
|---|---|---|
| Eight anchors and four rings tangling (spawned clear) | about 7 to 15 mm, varying with how finely it is measured | 30 mm |
| Sharp 45° edge dropped onto a sharp 45° edge | about 11 to 19 mm | 30 mm |
| Plate edge onto plate edge | about 3 to 9 mm | 20 mm |
| Round bars crossing | about 6 to 10 mm | 20 mm |

These are brief peaks during impacts. Nothing stays stuck, energy does not rise, and
nothing escapes.

## Read this before starting

**Check starting positions first.** Two alarming results in this area were tests that
created bodies already inside each other:

- A "5 cm end-on overlap" between a peg and the anchor was the anchor spawned speared
  on the peg. Started clear, it lands end-on with 0 mm overlap.
- A "4 cm overlap" in the anchor stress test was two anchors spawned overlapping by
  35 mm.

Measure overlap at frame 0 before anything else.

## How contact works for thin parts

Each body has surface samples: 32, or 64 for thin bodies (`thinSpacing`). Thin parts
can fall between samples, so `collideOverlap` also runs `deepestOverlap()`, which
searches directly for the deepest point shared by both bodies, starting from a few
likely places. It stops after two starting points find nothing (`if (!hits.length &&
misses >= 2) break`), to stay fast.

## Already tried

Trying more starting points and never giving up early:

| Strategy | Worst overlap in the pile | Physics cost |
|---|---|---|
| Current: 2 starts, give up after 2 | 13.7 mm | 19 ms/step |
| 4 starts, try all | 10.6 mm | 45 ms/step |
| 2 starts, try all | 9.3 mm | 37 ms/step |

A modest gain for 2 to 2.5 times the cost, so it was not adopted. Note that the
anchors in that run included one pair that started overlapping, so treat the numbers
as relative.

## Ideas worth trying

1. **Find the sweep's equivalent for overlaps.** Tangling usually happens while
   bodies are moving. Searching from where two bodies were in contact last frame, and
   following the contact as they move, is cheaper than searching afresh.
2. **More samples where a shape is thin or sharp, not everywhere.** The SDF can show
   where: points where the distance changes direction quickly, or where the shape is
   thin.
3. **Better depth for edge-on-edge contacts.** The depth estimate assumes surfaces
   meet head-on, and only partly corrects for angled faces (`overlapContact`).

## Done when

- The worst brief overlaps in the table roughly halve, without physics cost rising by
  more than about 20%.
- All existing checks still pass.
