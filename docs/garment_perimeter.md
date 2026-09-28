# Full perimeter: the whole outer edge, in two halves

A shirt laid flat is bigger than this bench's view, so its outer perimeter
cannot be read from one picture. A `"kind": "perimeter"` step reads it in two
halves and adds them up at two clips.

```
clip the neck and the hem  ->  one half in view, clips in the band  ->  reading
    ->  lift, turn end for end, the other half the same way  ->  reading  ->  total
```

| file | what it does |
| --- | --- |
| `include/verimeasure/perimeter.hpp`, `src/perimeter.cpp` | the clips, the cut, the fitted and traced lengths, the guide |
| `src/qc.cpp` | `judgePerimeter`: the same tolerance rule as every other step |
| `apps/verimeasure_station.cpp`, `include/verimeasure/station_page.hpp` | the guide on the feed, the halves, the strip on the panel |
| `tests/perimeter_check.cpp` | selftest on a rendered shirt, and replay of a station's refused half |
| `templates/refs/full_perimeter.svg` | the step's reference picture |

Nothing in the engine, the segmentation, the calibration, or any other kind of
step changed. A half is measured from the refined outline the engine already
produces for every reading, with the fold-height correction already in it.

## At the bench

1. **Clip the middle.** Lay the shirt face up with the sleeves out, fold it
   lengthwise to find the middle, and put a clip on the middle of the back
   neck and one on the middle of the hem. Where exactly they go changes how
   the total splits between the two halves, not the total.
2. **First half.** Lay one half on the mat with both clips inside the green
   band along the bottom of the view, and the other half flat past the bottom
   edge. It can be folded back on itself past the edge if the bench is short
   there, as long as none of it hangs and pulls. The station reads it when
   the lay holds still.
3. **Second half.** Lift the shirt clear of the mat - that is what tells the
   engine to take a new reading - turn it end for end, and lay the other half
   the same way. The collar is now at the other end of the band.
4. **The verdict** lands once both halves are in: the total, and each half.

Retake, or any move to another step or shirt, empties both halves. A half read
on one take is never added to a half of another.

Arriving at this step with the shirt still on the mat from the step before, or
after a Retake, the first half is read as soon as it is laid, without the mat
being cleared first (see "Operator page" in garment_qc.md). Between the two
halves the shirt still has to be lifted clear.

### Why clips

The halves have to be cut at the same point of the cloth both times. If they
were cut at a line on the feed, every millimetre the shirt lay off that line
would add or remove two millimetres of perimeter, one at the neck and one at
the hem - and nothing on a hem marks its middle. A clip travels with the cloth,
so it marks the same point in both pictures however the shirt was laid. The
station cuts each outline at the clip's centre, on a straight bridge across
the few millimetres of outline the clip covers.

### Why the bottom

The join has to run along an edge of the view at least as long as the cut.
Splitting a shirt means cutting along the body (about 710 mm) or across it
(at least the body's width, about 535 mm), and the view's short edges are
under 500 mm. Of the long edges the bottom is the longer, because the
calibration puts the mat about 4 degrees off square to the camera:

| edge of the working area | length |
| --- | --- |
| bottom | 758 mm |
| top | 723 mm |
| left | 492 mm |
| right | 497 mm |

Along the bottom a 710 mm half has about 20 mm to spare inside 10 mm end
margins; along the top, almost none. `perimeter_join_side` can move it.

### The guide on the feed

On a perimeter step the rail is not drawn. Instead:

- **the band**, tinted green, 15 to 60 mm in from the join side. Both clips go
  in it.
- **the margin**, dashed, 10 mm in from the other three sides. Where the cloth
  is inside it, that stretch is drawn red. The margin is advice: a half is
  refused only when its outline actually touches the border.
- **a ring on each clip found**, green inside the band and amber outside.
- **one line on the panel** saying what to put right, most urgent first:
  cloth inside a margin, clips missing or too many, clips outside the band.

It is worked out every frame from the engine's live outline and a colour
search for the clips.

## What a half measures

Two lengths, both logged.

**Fitted** (the default) is what a pattern draws: smooth seam lines that meet
at sharp corners. The outline is split at its corners - where it turns at least
`perimeter_corner_min_turn_deg` within `perimeter_corner_window_mm` - each edge
between them is fitted as a smooth curve, and the edges either side of each
corner are carried on until they cross, the way the folded-edge steps find a
seam corner. The cloth rounds every corner, so this matters:

| corner | turn | the cloth rounds it to | the cloth is shorter by |
| --- | --- | --- | --- |
| underarm, sleeve at 45 degrees | 135 degrees | 20 mm radius | about 50 mm |
| hem corner | 90 degrees | 5 mm radius | about 2 mm |

That is why the measuring interval was not made the reference: tuning an
interval until shirts agree with a pattern would use the cloth's wobble to make
up for the corners it rounds off, and the right value would change with every
style and sleeve angle.

Corners too gentle to cross, like the shoulder point, barely change the sum and
are followed. A crossing that is not believable - too little edge either side,
edges too nearly parallel, a crossing behind either edge or further out than
`perimeter_corner_reach_mm` - is refused and the cloth followed instead.
`reach_mm` on every reading says how much crossing added.

**Notches and spikes are bridged, not followed.** Where the outline turns hard
and turns back again within `perimeter_defect_span_mm` - a thread, a label, a
printed line of the mat the segmentation followed into the cloth - it went
somewhere and came back, and the garment's edge ran straight on underneath.
Each half logs how many it bridged.

**Traced** is the cloth followed, measured every `perimeter_trace_step_mm`
(5 mm). The raw outline is not used: it scatters about 0.15 mm from point to
point, and every wiggle adds length - on a real lay the same edge measured
6% longer point to point than at 5 mm steps. `"measure": "traced"` on the step
judges this length instead.

## Refusals

A half that cannot be read is about the lay, not the garment: it is said on
the panel and in the log, and nothing is filed. The operator lifts the shirt
and lays it again.

| refusal | what it means |
| --- | --- |
| no clips, one clip, three or more | exactly two clips have to be on the edge of the cloth |
| both ways round run off the picture | part of this half is past a border too: lay one half fully inside |
| a clip too close to the edge of the picture | the outline either side of a clip has to be in view to cut it |
| only part of the other half in view | the shirt does not reach the join edge, and what lies either side of the clips is not two halves of one shirt |
| the second half lies the same way round as the first | the same half twice would add up to a confident wrong number; turn the shirt end for end |

The last is judged by where the point furthest from the clips lies - the
sleeve, on a T-shirt, which sits near the collar end - and is only applied
when both halves say clearly which end the collar was at.

A shirt small enough to lie wholly in view, clear of every edge, is read in
one picture: the clips split its outline into both halves at once.

Every refused half leaves the picture and everything it was read with behind
as `perimeter-fail.png` and `perimeter-fail.json`, and prints the command to
replay it:

```
perimeter_check --case perimeter-fail.json --draw perimeter-out.png --verbose
```

## The style file

```json
{
  "name": "full_perimeter",
  "target_mm": 3000,
  "tol_minus_mm": -150,
  "tol_plus_mm": 150,
  "kind": "perimeter",
  "height_mm": 1.6,
  "reference_image": "refs/full_perimeter.svg",
  "instruction": "..."
}
```

| step field | default | what it decides |
| --- | --- | --- |
| `"kind": "perimeter"` | | this is a perimeter step |
| `"measure"` | `"fitted"` | `fitted`, as a pattern draws it, or `traced`, along the cloth |
| `"height_mm"` | | the fold height, as on an edge step: two plies for a flat lay |
| `"clip_hue_deg"` | the bench's | the clips' colour as two hues in degrees, such as `[40, 70]`, for a garment the bench's clips would not stand out on |

The tester style carries one at the end. **Its target of 3000 mm is a
placeholder**: set it from the pattern.

### Using the pattern as the reference

Plot the flat outline 1:1 on paper and cut it out. It has an exact perimeter,
no thickness and nothing to stretch, so measuring it in two halves with the
clips tests the joins, the calibration out at the edges of the view and the
corner finding in one go. The flat outline is not one pattern piece: assemble
it from the front, back and sleeves, with stitch lines and no seam allowances,
the hem at its fold, and the neck at the rib's outer edge rather than the
neckline seam - the way the camera sees the shirt.

Do not tune anything against the pattern on production shirts. A batch that
runs 1% large would simply be tuned into agreement.

## What was measured

**`perimeter_check --selftest`**, 40 checks, on a rendered T-shirt whose
pattern perimeter is 2983.46 mm and whose cloth perimeter is 2870.69 mm:

| check | result |
| --- | --- |
| fitted total against the pattern | +0.75 mm |
| traced total against the cloth | -2.17 mm |
| the same lay read five times | 1.7 mm spread, about 0.7 mm standard deviation |
| clips moved up to 30 mm along the edge, six lays each, averages | 0.38 mm spread |
| shifted and turned up to 2 degrees | 0.85 mm spread |
| a notch 5.5 mm deep, and a spike 8 mm, on a 500 mm edge | bridged, within 0.2 mm |

**Through the station itself**, camera-free, with the real calibration and a
real clear-mat frame, and the same shirt painted on in two halves: 2974.6 mm
from 1486.5 + 1488.2 mm, against 2976.6 mm expected once the 1.6 mm fold
height is taken off. The refusals, Retake, and the feed all behaved as above.

## Where it can go wrong

- **The lay.** How far the sleeves are spread and how flat the cloth is
  smoothed moves the total more than anything here. The instruction and the
  reference picture should pin both down.
- **The brightness gain.** The engine's illumination gain is a median over the
  whole working area, and a half shirt covers about two thirds of it, so the
  gain sits at its 2.0 clamp during this step. On the replay frame that cut a
  notch about 1 mm deep into the cloth edge where a printed line of the mat runs
  under it; at a gain of 1 the notch was gone. The notch was bridged, but it is
  worth fixing in the engine, for every step.
- **The outer ring of the view.** A half reaches the edges and corners of the
  frame, where repeat calibrations have disagreed by 2 to 7 mm.
- **Garments bigger than a half of the view.** A body longer than about
  725 mm, or long sleeves, needs more than two pictures, which this step does
  not do yet.

## Tuning

| field | default | what it does |
| --- | --- | --- |
| `perimeter_join_side` | `bottom` | the side the band runs along, past which the other half lies |
| `perimeter_band_near_mm`, `perimeter_band_far_mm` | 15, 60 | how far in from that side the clips have to sit |
| `perimeter_margin_mm` | 10 | the margin the guide asks to be kept clear on the other sides |
| `perimeter_border_touch_px` | 10 | how close to the border an outline point counts as touching it |
| `perimeter_clip_hue_lo_deg`, `perimeter_clip_hue_hi_deg` | 40, 70 | the clips' colour, yellow by default |
| `perimeter_clip_sat_min`, `perimeter_clip_val_min` | 110, 90 | how strong a colour a clip is |
| `perimeter_clip_area_min_mm2`, `perimeter_clip_area_max_mm2` | 30, 3000 | how big a clip looks from above |
| `perimeter_clip_clear_mm` | 6 | how far round each clip the outline is taken out and bridged |
| `perimeter_clip_on_edge_mm` | 4 | how close to the outline a clip has to be to count as on the edge |
| `perimeter_trace_step_mm` | 5 | the traced length's measuring step |
| `perimeter_corner_window_mm`, `perimeter_corner_min_turn_deg` | 30, 30 | what counts as a corner |
| `perimeter_corner_curvature_deg_per_mm` | 1 | how sharply the outline has to curve to be part of a corner |
| `perimeter_corner_fit_mm`, `perimeter_corner_fit_min_mm` | 40, 15 | how much edge either side of a corner is fitted to cross it |
| `perimeter_corner_reach_mm` | 80 | how far out a crossing may lie |
| `perimeter_edge_tolerance_mm`, `perimeter_edge_piece_min_mm` | 1, 40 | how far an edge may wander from one smooth curve before it is fitted in pieces |
| `perimeter_defect_span_mm` | 40 | how long a notch or spike may be and still be bridged |
