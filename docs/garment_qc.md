# Garment QC: one edge, one length, one verdict

A QC station for a garment line. One side of the live camera feed is marked
`FOLD FACES THIS SIDE`. The operator folds the shirt so the dimension being
checked becomes a single straight edge, turns that edge to face the mark, and
the station reports its length and says PASS or FAIL. Then the next step. Four
to six steps and the shirt is done.

```
step 1 of 4  ->  measure  ->  PASS/FAIL  ->  Next  ->  step 2 of 4  ->  ...
                                                  ...  Next shirt   ->  step 1
```

One rule covers every step: **fold the measurement into one straight edge, and
show that edge to the mark.**

One family of dimensions cannot obey it, because they are not on the outline of
the cloth at all: a neckline, a collar seam, a placket. No fold turns those into
a silhouette, so there is nothing for the segmentation to find and nothing to
present to the mark. Those are `"kind": "curve"` steps, the operator points at
the seam and the machine finds it between the points, and
[garment_curves.md](garment_curves.md) is what they do instead. And there is a
plainer one: `"kind": "distance"` is two taps on the held picture and the
straight line between them, through the calibration, with nothing looked for
and nothing corrected — a ruler laid on the picture, for a dimension that is
simply two landmarks apart. And `"kind": "perimeter"` is the whole outer edge
of the shirt laid flat, read in two halves joined at two clips because a shirt
is bigger than the view; [garment_perimeter.md](garment_perimeter.md) is that
one. Everything below is the folded edge.

This is a layer on top of the existing measurement pipeline. Segmentation,
calibration, sub-pixel edge refinement, the settling state machine and
`PlaneModel::pxToMm` are untouched; every millimetre still comes out of them.

| file | what it does |
| --- | --- |
| `include/verimeasure/edge.hpp`, `src/edge.cpp` | finds the edge facing the mark and measures its length |
| `include/verimeasure/template.hpp`, `src/template.cpp` | reads and writes a style's ordered step list |
| `include/verimeasure/qc.hpp`, `src/qc.cpp` | judges one length against one step, and appends the audit line |
| `include/verimeasure/station_page.hpp` | the operator page: step, picture, verdict, Next |
| `tests/qc_check.cpp` | offline harness and selftest, no camera needed |
| `templates/style-42-polo-m.json`, `templates/refs/` | a worked style and its reference pictures |

## Why the whole garment is never measured

Nothing here interprets the shirt. It does not detect the garment, identify its
parts, or need it segmented as a whole. Only the one folded edge has to be in
view.

That is the point. The field of view on this bench is 728 x 485 mm and the shirt
is bigger than that, so any approach that needs the whole garment inside the
picture cannot work. A 490 mm edge fits easily. The garment can hang off every
side of the frame and the reading is unaffected, because the reading is the
length of one line.

It also removes the operator's alignment problem entirely. There is no distance
from a reference to get right, no origin to align to; only an orientation. The
edge is measured corner to corner wherever it happens to sit on the mat, so the
mat may slide or turn without moving the number. `qc_check --selftest` measures
the same fold at five positions and orientations and requires them to agree.

## How the edge is found

`measureEdge` takes the refined outline, already in millimetres, and does three
things.

**Find every edge of the boundary.** Runs are grown a point at a time using
running second moments, and the cost is the length of the edges rather than the
square of the outline.

The fit is carried along the edge rather than stretched across the whole of it.
It looks at the last `edge_tangent_window_mm` of accepted boundary and no
further, because **a garment edge is not a straight line and never was**. A hem
sweeps, a shoulder slopes, a chest line crosses a body wider at the bottom than
the top. On the shirts this was written for that is 13 to 15 mm of sweep across
half a metre of edge, and a fit stretched across the whole of it counts that
curvature as its own error and gives up partway along. That is not a subtle
effect: it read a 540 mm hem as 444 mm and a 430 mm shoulder as 362 mm, both
with a perfect straightness score, because the flat part of a swept edge really
is flat and really is short. A fit that sees 40 mm at a time finds a curved edge
straight everywhere it looks, and follows it the whole way.

The fit is also the run's own rather than a fixed corridor, which is what lets a
run follow a fold laid a few degrees off the rail. A lay square to the mark to
better than a degree is not something a pair of hands does at a bench; a fold
three degrees out crosses 35 mm of depth over its own length, and a search that
worked inside a fixed depth corridor measured only as much of it as the corridor
admitted.

**What ends a run is turning, not curving.** The tangent is compared against the
one the run started with, and the run ends where the edge has turned past
`edge_turn_limit_deg`. A hem sweeping 15 mm over half a metre turns about 13
degrees end to end. An armhole, a side seam, or the corner between two edges
turns far more than that within a few millimetres. So the run ends at the
corner — which is also where the seam a tape is measured to actually is.

Points that disagree with the fit do not end it either. A thread, a nick, a care
label lying across the fold, a speck of the mat that segmented as cloth — each
is a few points on a boundary of several thousand, and a search that stops at
the first of them does not measure a fold with a thread over the middle of it,
it measures the longer of the two halves the thread cut it into. So the run
steps over anything further than `edge_defect_reject_mm` off the fit, and stops
only where the boundary walks away *along* the edge instead of handing it back
within `edge_defect_bridge_mm`. A defect sits across the edge; a corner does
not.

**Take the near one.** Every straight stretch long enough to be a fold gets a
depth: how far into the picture it sits along the direction the machine looks.
The shallowest of them says where the near side of the picture is, and the
longest stretch within `edge_band_mm` of that is the fold.

This is a question about stretches, not about points, and that is deliberate. A
single point can be a thread hanging past the fold, and a near band read from
the nearest *point* either hides the fold behind that thread or admits only the
part of it that the thread's own depth happens to reach.

**Report the chord between its two corners.** The winning run is refitted as an
arc — a parabola in the edge's own frame, which is indistinguishable from a
circle to well under a tenth of a millimetre at the curvatures a garment
actually has — whatever is not the edge is dropped, and the arc is fitted again.

The answer is the **chord**, never the arc length. A measurement is what a tape
laid flat between two seams reads, and a tape does not follow the sweep of a
hem. Curvature is recorded beside the number as `bow_mm`, not added to it.

The corners are the **seams**, found the way a floor finds them.

Nobody measuring a garment measures to where the hem stops being flat. They find
the seam at one end, the seam at the other, and pull a tape straight between
them. The seam is where the hem and the side seam meet — and cloth rounds that
meeting into a curve, so neither edge actually reaches it. The eye carries both
on until they cross. That is what is automated here: the fitted arc is carried
on from where the cloth stopped being flat, the edge *leaving* this one is found
and fitted, and the two are crossed.

Measuring to where the cloth stops being flat instead reads short by roughly
twice the radius the corner is rounded through — on a 20 mm rounding, 13 mm off
a half-metre measurement, which is most of a tolerance band spent on nothing.

**The guard is the whole design.** The edge leaving this one has to be straight
over a real length and has to point at least `edge_corner_min_angle_deg` away
from this one. A side seam does both and the crossing is well conditioned. A
fold does neither — a fold does not turn a corner, the cloth simply carries on,
or doubles back on itself — so the two directions are nearly parallel, the
crossing is anywhere or nowhere, and **no corner is invented**.

And behind the guard sits a hard bound: **neither end of the measurement may lie
past the cloth**. The extent of the boundary along the fitted edge is computed
first, and the crossing refines inside it rather than reaching past it. Whatever
the geometry says about where two edges would have met, the garment stops where
it stops, and a line drawn onto bare mat is a line measuring the mat. An
operator watching the end of the span sit on the grid has no reason to believe
any of the rest of it, and they are right not to.

That guard was what an earlier version lacked. It walked a fixed distance
outward looking for the corner, found no corner on a fold because there is none
there, ran to its cap at both ends, and added a centimetre the garment did not
have. A correction that only ever adds is the dangerous kind: a garment rejected
for a centimetre it never had is a garment rejected for nothing. `reach_mm` is
on every reading for exactly that reason, and `edge_corner_reach_mm` set to zero
turns the correction off and measures the cloth as it lies.

### Refusing a lay it cannot read

Four guards. Each one, let through, gives a confidently wrong number rather
than a slightly worse one, and there is no second opinion downstream.

**Nothing straight enough to be a fold**, or shorter than
`edge_min_length_mm`.

**The edge bows more than `edge_max_bow_fraction` of its own length.** The
curvature belongs to the garment, so the arc follows it and the chord across it
is still the measurement. What this catches is a fold gone wrong, where the
cloth between the corners has gone slack and the corners are no longer sitting
on the seams. It is a share rather than a millimetre count because 14 mm of
sweep means one thing across half a metre of chest and another across a 230 mm
sleeve. Behind it sits `edge_coverage_fraction` as a backstop: the band of cloth
lying along the arc is compared against the chord, and on a lay presented
properly they are the same length.

**The fold runs off the picture.** An edge that ends within
`edge_clip_margin_mm` of the border of the working area did not end there
because the garment did. What comes back is straight, repeatable, and exactly as
long as the view — the one failure that looks precisely like a measurement. The
rail it was laid against is not counted as a border, because lying against it is
what the step asked for.

**Not calibrated**, so there are no millimetres.

A refusal is a FAIL, not a skipped step. Letting the operator move on would let
a garment pass on the strength of the steps that happened to work, which is the
one failure mode a QC station must not have. Re-laying is the Retake button.

### What the shape numbers mean

`straightness_mm` is **roughness**: the RMS of the boundary about the arc fitted
through it. It is a measure of the cloth — bunching, a wrinkle riding over the
fold, a boundary that will not settle.

`bow_mm` is **sweep**: how far that arc departs from its own chord at the worst
point. It is not an error and nothing is subtracted for it. A garment edge is
curved by design and the chord is the measurement regardless.

`traced_mm` is the edge **followed** rather than crossed: the length along the
arc rather than across it. Which of the two is the measurement is a question
about what the tech pack means, not about accuracy — on a real garment they
differ by about a millimetre in five hundred — and
`edge_measure_along_curve` is where it is answered. Both are in the log either
way.

`reach_mm` is the fourth, and it is none of the above: it is how much of the reading was
**reached for** past the ends of the edge, to put the corners where the seams
would have met. It is a correction rather than a measurement and it only ever
adds, which is exactly why it is written down. On a sewn corner it is worth two
or three millimetres. If it is ever a large share of the number, the correction
is wrong for these garments — turn `edge_corner_reach_mm` down, to zero if the
folds present square ends of their own, and read the cloth as it lies.

Keeping the first two apart is the point. Before, one number carried both, and a
hem doing exactly what a hem does was indistinguishable from a fold that had
gone slack — so the limit had to be set loose enough for the first, which made
it useless against the second.

### Straightness is recorded even when it passes

Folding well is a training matter, not the machine's job. But a worker having a
bad day produces a slightly bowed fold, and without a number written down that
silently becomes a wrong measurement on a passed shirt. So `straightness_mm`
goes into the log on every reading. A bench whose folds are drifting shows up
there first, and nowhere else.

## The style file

An ordered list of steps, in the order the operator works through them.

```json
{
  "style": "style-42-polo-M",
  "units": "mm",
  "steps": [
    {
      "name": "half_chest",
      "target_mm": 490,
      "tol_minus_mm": -12.7,
      "tol_plus_mm": 12.7,
      "height_mm": 3.2,
      "reference_image": "refs/half_chest.svg",
      "instruction": "Fold along the chest so the line from one side seam to the other is one clean straight edge, and turn that edge to face the mark."
    }
  ]
}
```

Tolerances are absolute and two-sided rather than a percentage, because that is
what a tech pack writes: a half inch on a body length and an eighth on a
shoulder are 1.8% and 25% of their own targets, so no single proportional rule
stands in for both. They are stored separately because tech packs write
one-sided bands. A file that writes both limits positive means a symmetric band
and is saying so clumsily, so the signs are normalised rather than trusted —
taking it at face value would quietly fail every good garment.

`units` may be `mm`, `cm` or `in`. Any field named `*_mm` is millimetres
whatever `units` says. Lengths written as strings may use the fractional inches
a tech pack uses: `"27 1/2"`, `"-1/8"`.

### `height_mm`, and why every step needs one

How far the cloth's top face sits above the mat. The camera sees that face, not
the mat, and a face nearer the lens images larger — so without this every
reading is a fixed percentage long. The correction is `(d - h) / d` applied to
the outline itself, so it reaches every measurement rather than one length.

It is the largest correctable error this bench has left, and it is invisible:
nothing looks wrong, every reading is plausible, and every one of them is long.

The size of it, at the 697 mm working distance measured in September 2026:

| step | length | `height_mm` | was reading long by |
|---|---|---|---|
| `sleeve_length` | 230 mm | 1.6 | 0.5 mm |
| `shoulder_width` | 430 mm | 1.6 | 1.0 mm |
| `body_length` | 710 mm | 1.6 | 1.6 mm |
| `half_chest` | 510 mm | 3.2 | 2.3 mm |
| `bottom_hem` | 535 mm | 3.2 | 2.5 mm |

**The numbers in the shipped styles are estimates and you should replace them.**
They are the ply count each fold produces — which the step's own instruction
tells you — times an assumed **0.8 mm per ply** for a mid-weight knit:

* A shirt laid flat is front plus back: **2 plies, 1.6 mm**. That covers
  `sleeve_length`, `shoulder_width` and `body_length`.
* A hem is turned up and stitched, and a fold across the chest doubles a shirt
  that is already two plies: **4 plies, 3.2 mm**. That covers `bottom_hem` and
  `half_chest`.

To correct them, measure one ply with a caliper — or ten plies and divide by ten,
which is easier and better — and rescale. If your cloth is 0.55 mm rather than
0.8, every height here is 0.7 times what it says.

Getting it wrong by a millimetre costs about a millimetre on the longest step, so
this is worth a caliper rather than a guess. Note which way the failure runs: a
height set too *high* over-corrects and the reading comes back **short**, which
is indistinguishable from a garment out of spec. When in doubt, under-estimate.

`curve` and `distance` steps use it too. They read a seam or two points straight
off the picture through the plane model rather than through an outline, so the
correction is carried by the model instead: the same `(d - h) / d` about the
same point, which the seam is then found and measured through. A 280 mm
neckline on a two-ply lay (1.6 mm at 700 mm) reads 0.64 mm shorter than it did
when these steps ignored the height. The height is the one an edge step would
use: the step's own `height_mm`, or `--object-height-mm` when the step has none.
None of the shipped styles puts a height on one of these steps, so without that
flag they read as they always have until one is added. A `collar` step does not
use it.

The correction needs a working distance, and the one the calibration guesses for
itself is not good enough — see `estimateCameraDistance`, which explains why a
squarely mounted camera makes that guess *worse*. Pass `--focal-length-mm` once
with the figure printed on the lens: it is saved in `bench.json` and used on
every start after. `--camera-distance-mm` takes a tape measure instead, for one
run. With neither, the station falls back on the guess and says so loudly at
startup — and with heights set, a guess that is too short over-corrects and
every long step reads short.

`reference_image` is resolved against the templates directory and served to the
page by step number, so nothing a browser sends decides what gets read off disk.
`.svg`, `.jpg`, `.png` and `.webp` all work.

The four in `templates/refs/` are two-panel diagrams: on the left the point of
measure on a flat shirt, which is what the number means; on the right the fold,
with the edge presented to the mark and the arrow running along it. Both panels
matter — an operator shown only the fold has to work out for themselves which
line is wanted. Replace them with photographs when the bench is set up; keep
both views.

Targets and tolerances are editable from the operator page and written straight
back to the file, always in millimetres.

## Operator page

* Which step, out of how many, pinned so it cannot scroll out of view.
* The reference picture and the instruction that goes with it.
* PASS or FAIL, large, with the reading, the allowed band and the deviation.
* **Back**, **Retake**, **Next**, **Next shirt**. Space or Enter is Next, `r` is
  Retake, `n` is Next shirt, because a hand is usually holding fabric.
* The measured edge is drawn on the held frame in green, with its two corners
  marked, so the operator can see what the number came from rather than trusting
  that the machine found the right line.
* Everything drawn on the feed belongs to the step it was drawn for. Moving to
  another step by any route, or pressing **Retake**, clears it: the outline, the
  measured edge, a seam, a distance and its number, the points.
* The next fold does not need a clear mat. On a step still waiting for its
  reading the machine keeps holding the last reading, so pointing still works on
  a lay left as it was, until the garment on the mat is plainly a different lay
  — cloth gone from where the old lay was, or a large piece opened out beside it.
  Then the new lay is outlined as it settles and measured for this step. The lay
  the last step was measured on is never measured again for the new one: left
  alone, touched and left as it was, with a hand resting on it or an arm swept
  across it, it stays held and nothing is filed. Lifting the shirt clear of the
  mat still starts over, as it always did.
* A step that already has its verdict is not measured again by refolding on it:
  going **Back** to look at one is not a request for a new reading. **Retake** is.
* **Retake** on a folded-edge step: lay the fold again. When the reading being
  retaken is that step's own, straightening the fold where it lies is enough. On
  a pointing step it takes a new picture of the garment as it lies now, and the
  operator points again. The thresholds are the `relay_*` settings in
  `config.hpp`; the station's console prints how much cloth moved each time it
  lets a reading go or holds it again.

## Audit log

`--results PATH` appends one JSON line per step attempt.

```json
{"timestamp":"2026-09-03T16:31:07","style":"style-42-polo-M","garment":3,
 "step":"half_chest","step_index":4,"step_count":4,"measured":true,
 "measured_mm":491.62,"deviation_mm":1.62,"straightness_mm":0.34,
 "target_mm":490.0,"tol_minus_mm":-12.7,"tol_plus_mm":12.7,
 "low_mm":477.3,"high_mm":502.7,"result":"PASS","measurement":{ ... }}
```

`measured` distinguishes a step that was read and rejected from one that could
not be read at all.

## Running it

```
verimeasure_station --template templates/style-42-polo-m.json --scan-from bottom --results qc.jsonl
```

Then open `http://localhost:8000`. `--host 0.0.0.0` puts the page on a tablet at
the bench. `--scan-from` takes `left`, `right`, `top` or `bottom` and must be
the same side every shift; it is what the mark shows.

The default is `bottom`. The view is wider than it is tall and so is a shirt, so
a rail along the bottom is the one that leaves room for a full span: the fold
lies across the long axis of the picture and fits. A rail down one side leaves
only the height of the frame to reach across, and a half chest runs off the top
and bottom of the view where only part of it can be read.

Clear the mat and press **Recalibrate** at the start of a shift: a mat that has
changed height or tilt has changed scale, and no scheme survives that.

### The reach

The length of the rail is the hard limit on what the bench can measure, and the
station says what it is at startup and whenever the style changes. A target
longer than the reach is not a tolerance problem: the fold runs off the picture
and the reading is the width of the view.

The working area runs `roi_extrapolation_pitches` past the last grid line the
calibration found, stopping `roi_frame_margin_px` short of the border of the
picture. It used to stop one pitch *inside* the grid, which made the reach a
property of how far the printed mat happened to reach rather than of what the
camera can see: on the bench this was written for that cost 59 mm of rail and
put a 710 mm body length out of range at any tolerance. Extrapolating the plane
model this far is safe — it is a global fit to hundreds of lattice points, not
an interpolation between neighbouring ones — but it does bring the edges of the
picture into the watched area, so anything moving out there (a window, a
doorway) now wants to be out of frame or behind the camera. Set the value
negative to inset as before.

Offline, with no camera:

```
qc_check --selftest
qc_check --outline lay.json --template templates/style-42-polo-m.json --step 1
```

## Tuning

| field | default | what it does |
| --- | --- | --- |
| `scan_from` | `bottom` | which side of the picture the fold is presented to |
| `edge_band_mm` | 40 | how deep past the first fabric the edge may lie. Wide enough that a fold need not be perfectly square to the camera, narrow enough that an edge further in cannot be picked instead. |
| `edge_straightness_mm` | 3.0 | how far the edge may wander from straight, RMS |
| `edge_min_length_mm` | 60 | shorter than this is not a fold, it is a coincidence |
| `edge_defect_reject_mm` | 7.5 | how far one boundary point may sit off the fit before it is a thread rather than the edge |
| `edge_tangent_window_mm` | 40 | how much of the edge the fit sees at a time while it follows it. Small enough that a swept edge is straight everywhere it looks |
| `edge_turn_limit_deg` | 30 | how far the edge may turn end to end before it has become another edge. A hem turns about 13 degrees; an armhole turns far more within millimetres |
| `edge_corner_reach_mm` | 40 | how far out the crossing of the two edges may be before it is not believed. A bound on a computed answer, not a distance walked. Zero measures the cloth as it lies |
| `edge_corner_min_angle_deg` | 30 | how far apart the two edges must point before their crossing means anything. This is what tells a seam from a fold |
| `edge_corner_search_mm` | 120 | how much boundary past the end of the edge to look through for the edge leaving it |
| `edge_measure_along_curve` | false | follow the edge instead of crossing it. Both lengths are logged either way |
| `edge_max_bow_fraction` | 0.05 | how far the edge may sweep, as a share of its own length, before the corners cannot be trusted |
| `edge_coverage_fraction` | 0.75 | how much of the cloth lying along the edge has to read as that one edge |
| `edge_defect_bridge_mm` | 12 | how far the edge may advance while a defect interrupts it. Past this the boundary has left the edge rather than crossed it |
| `edge_clip_margin_mm` | 3.0 | how close to the border of the working area an edge may end before it reads as running past it |
| `roi_extrapolation_pitches` | 4.0 | how far past the last detected grid line the station may work. Negative insets instead |
| `roi_frame_margin_px` | 8 | how far short of the border of the picture the working area stops |
