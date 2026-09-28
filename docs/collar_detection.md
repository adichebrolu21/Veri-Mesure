# Collar detection: draw a box, the picture does the rest

A curve step ([garment_curves.md](garment_curves.md)) takes the operator's
pointing as the start of the reading. The snap corrects that pointing across the
seam and cannot correct it along the seam, so the two ends of a neckline are
wherever a finger or a tap put them. Which of two lines a few millimetres apart
gets measured is also left to the pointing. Those are the two places a person is
still inside that measurement, and no amount of model accuracy takes them out.

A collar step takes the person out of both.

```
box  ->  the collar band, stretch by stretch  ->  its two edges, read sub-sample
     ->  each run to the edge of the cloth  ->  the step's "seam" picks one  ->  measure
```

**The box is a region, not a position.** It says where the collar is and nothing
else. Everything the reading carries comes from the picture:

| | from |
| --- | --- |
| where the line lies | `snapCurve`, sub-sample, exactly as a curve step; an attachment seam is then put in the bottom of its groove, where the stitching is |
| which line | the collar band, found as two crisp lines a band's width apart with plain cloth between them; the style file's `"seam"` names one of its two edges |
| where it starts and stops | where that line meets the edge of the cloth. On a shirt laid flat the collar's seam runs to the shirt's edge at both shoulders, and that meeting point is the shoulder point |

So the property this step has to hold is the one a measurement needs: two
operators drawing two different boxes round the same collar get the same number.
`collar_check --selftest` draws the box twelve different ways and requires it.

| file | what it does |
| --- | --- |
| `include/verimeasure/collar.hpp`, `src/collar.cpp` | the box, the band, the reads, the ends, the choice |
| `src/qc.cpp` | `judgeCollar`: `judgeCurve` on the line measured, so the two kinds of step cannot drift into different verdict rules |
| `apps/verimeasure_station.cpp`, `include/verimeasure/station_page.hpp` | the box on the feed, the collar strip, the log |
| `tests/collar_check.cpp` | selftest on a rendered collar, and replay of a real frame or a station's case |
| `templates/refs/collar_detection.svg` | the step's reference picture |

Nothing in the curve, edge, engine, segmentation or calibration code changed.
The collar module calls `snapCurve` and `fitCircleRadius` as they are, and every
other kind of step behaves exactly as it did.

## How it is found

**Which way the collar opens.** The box is drawn round a collar that hangs from
the edge of the shirt, so one side of the box has mat along it. That is the side
the collar opens toward, read off the box's own four sides. Taking the direction
from the mat to the cloth as an average over the whole box was tried first. On
the real shirt the mat under one sloping shoulder tipped it by twenty-five
degrees and put every guess half outside the box.

**Guesses.** Half-ellipses standing on the edge of the cloth near either side of
the box and rising toward the far side, at four sizes. This is the only place
the box's size and position reach, and they reach it only as somewhere to start.
They are half-ellipses, not arcs of a circle, because a collar laid flat is a
horseshoe with its legs square to the edge. A circle's legs lean inward, so down
one leg the guess sat across the band, and the snap returned the attachment seam
over the top and the opening's edge down one side.

**The band, stretch by stretch.** Along a line one of the guesses held to (the
*frame*), the picture is profiled across the collar in 20 mm stretches, 40 mm
either side. In each stretch every line is found, as a change of shade (a
*step*) or a stitch or groove (a *ridge*). Each step is asked how wide it is: the
edge of a piece of cloth or a seam changes over a millimetre, shading over
several. A **band** is two crisp lines 8 to 35 mm apart with nothing crisp
between them as strong as they are. A dynamic program then picks one band per
stretch, all the way round, so that from one stretch to the next the band's width
barely changes and its place moves no faster than the frame can stray.

Each part of that replaced something that failed on the real collar:

- **One profile averaged round the whole collar** found the lines, but which
  lines depended on which line the guesses had held to, and that depended on the
  box. The same collar read **264 to 306 mm from eight boxes**.
- **"Two groups of lines a band's width apart"** paired a seam with a line of
  shading 15 mm outside it, where the body bunches up beside the collar. That
  line runs parallel to the collar all the way round and reaches the edge of the
  cloth at both ends, exactly as the seam does. It is a line in every sense but
  one, and that one is width: it is soft.
- **A frame line that strays** from one edge of the band to the other carries
  every line found along it with it. Stretch by stretch, a straying frame only
  moves where each stretch finds the band.

Near the shoulders, the opening's edge runs beside the back of the collar and is
faint, so the last stretch or two may show no band. The seam is as strong there
as anywhere, so the run is carried on by the seam alone: a crisp line within
2 mm of where the seam was.

**Each edge is read.** The band's offsets are joined into a guide for each edge,
and past the last stretch the guide is carried on along its own curve, a
quadratic fitted to its last 30 mm, to the edge of the cloth. It is not carried
along the frame: past the last stretch nothing has been found, and a frame that
turned there took the shoulder point with it. Each guide is then snapped with a
narrow corridor (`collar_corridor_mm`, 6 mm), which holds the line it was given
and not the lines beside it.

**Looked at again until nothing changes.** The band is found again along the
outer edge just read, and read again, until the band's outer edge, looked for
along the line read, *is* that line all the way round (within 1.5 mm in every
stretch) and right down to both shoulder points. Only then is a reading taken.
Looking once more was not enough: a frame that stopped short on one leg had its
edge carried down the frame instead of the collar, and the look along that edge
found the band everywhere except where it had gone wrong. On a rendered collar
that read 13 mm short of one shoulder at full confidence. Each frame gets three
looks; failing that, the next frame is tried, and after that the reading is
refused.

**The seam is put in its groove.** An attachment seam is a groove: the band
turns down into it on one side, the body comes up out of it on the other, and
the stitching is at the bottom. The snap reads a step where it finds one, and the
band side of a groove is a step, half a millimetre short of the stitching. Half a
millimetre across a seam is a millimetre and a half along a neckline, every time
and always short. So the seam's line is moved onto the bottom of the groove,
**point by point**. At the shoulder the groove and the edge of a crease where the
collar folds under run a couple of millimetres apart, and a line moved by one
average amount followed either of them over its last centimetre depending on
the box. The groove is found as a *narrow* dark line, which a crease is not. The
opening's edge is not moved: it is the edge of the cloth, and the shadow it
throws is a dark line that is not the edge.

**The ends are where the cloth ends.** Each end of the line is walked to where
it leaves the cloth. The outline the station's reading was taken from gives
where that is to within a few millimetres; without an outline, as in
`collar_check`, pale cloth is told from the dark mat by brightness. It is only a
guide. Glow off pale cloth pushes an outline several millimetres out onto the
mat, and a collar measured to that outline would read long at both ends. So the
last centimetre of the line (less its last 2 mm, where the groove runs out into
the edge) is fitted straight and carried to the steepest fall from cloth to mat
along it, found on the picture to a fraction of a sample.

**A dark garment is read from a brightened picture.** Every line is held to
floors in gray levels as well as to the picture's own noise - two a millimetre
for an edge, the snap's `curve_min_contrast`, two and a half for a groove - and
on a black shirt the whole collar sits between one and nine gray levels, its
edges one or two apart. When the middle gray level of the cloth in the box is
under `collar_dark_below` (24), the collar is read from the picture scaled until
the cloth sits at `collar_dark_level` (80), scaled from the frame's colour before
it was rounded to whole levels. On the rendered collar below at a thirtieth of
its brightness this took the self-test from 25 checks failed of 40 to 5 of 59,
and at a twelfth from 18 of 58 to 2 of 59. A cloth brighter than the trigger is
read from the same picture as always, to the bit.

**What it does not read: a polo collar.** The band this looks for is a crew
neck's rib, 8 to 35 mm wide (`collar_band_min_mm`, `collar_band_max_mm`), running
unbroken from one shoulder to the other. A polo laid face up has its collar
folded down as a leaf about 50 mm wide, leaning in to the placket and broken
there, with the back of the collar across the bottom of the opening. No pair of
crisp lines a band's width apart runs round it, so no band is found; the lines
that were found are offered for a tap (`choose`). On the black polo this was
tried on, the one line offered runs up the inside of each leaf and across the
top of the care label rather than the placket, and ends where the back of the
collar meets the base - not a neckline. Reading a polo needs its own rule for
which line it measures and where it starts and stops.

## What it measures

`"measure": "arc"` is the length along the chosen edge between the two shoulder
points. `"chord"` is the straight line between them, the neck width. Both are
logged either way, with the neck drop (`bow_mm`: how far the line dips from its
chord) and the best-fit radius. `"seam": "outer"` (the default) is the
attachment seam; `"inner"` is the opening's own edge, from where it meets the
edge of the cloth at one shoulder to the other.

`"measure": "around"` is the whole collar: the band's outer edge over the
front from one shoulder point to the other, and on across the back of the neck
to where it started. A round neck laid flat face up shows its back as the edge
of the shirt between the two shoulder points - the back of the collar is the
top of the garment, lying along the silhouette - so that is where the back is
read. It is taken from the cloth's boundary between the shoulder points (the
short way; the long way is the rest of the shirt), then put on the picture
every half millimetre where the cloth falls to the mat, exactly as the
shoulder points are, and smoothed over a few millimetres, because the boundary
is only a guide: glow off pale cloth pushes an outline out onto the mat and the
steps of a mask are length that is not there. An edge more than half as long
again as the straight line between the shoulder points has left the neck and
is refused. The log keeps `arc_mm` (the front) and `back_mm` (the back); the
reading is the two added up. Only along the outer edge: a style asking for
`"inner"` all the way round is refused when it is loaded. On the station the
whole collar is drawn in green, front and back, and the band's inner edge - the
opening, found and not measured - is drawn dotted.

The tech pack's target is used as a gate and nothing else: a line more than a
quarter away from it is not the collar the style describes, and it is refused
rather than measured. It never chooses between lines. For `"around"` the target
is the whole collar's, front and back.

## Refusals

| refusal | what it means | filed as |
| --- | --- | --- |
| `box` | the box is smaller than a collar, holds no edge of the shirt, or cuts the collar (the line to be measured, or the band found so far, runs out of it) | nothing: the box is the operator's, not the garment's. Draw it again |
| `collar` | no band could be found and followed to both shoulders, or the edge read is not within a quarter of the target | FAIL, as for a seam: a collar that cannot be seen has not been measured |
| `choose` | no band could be found, but lines were: they are drawn, and a tap on one takes it | nothing until tapped. The tap chooses which line and nothing else; every number is still the line's own |

After a reading, a tap changes nothing. Which edge is measured is the step's
rule, not the operator's.

Every refusal leaves the held frame, the box, the outline and the step behind as
`collar-fail.png` and `collar-fail.json`, and prints the command to replay it:

```
collar_check --case collar-fail.json --draw out.png
```

## At the station

On a collar step, once the reading holds, the feed takes a drag. The operator
drags a box round the whole collar, past both shoulders, with a little of the mat
beyond the edge of the shirt inside it. The station draws the box, both edges of
the band, and the edge measured in green with its two shoulder points and its
number. The collar strip on the panel lists both edges with their lengths and
says which one the step measures. **Clear the box** takes it away. Retake, and
any move between steps, clears it with the rest of the feed.

A box takes about 0.7 to 1 s to read on this PC, and the feed does not move
while it does. The page says "Reading the collar…" straight away.

## What was measured

**On the real collar**, the salmon jersey in the `a5c3f61` capture, with eight
boxes (tight, loose, very loose, shifted each way, tall): the outer edge read
**286.59 to 286.68 mm**, a spread of 0.09 mm, with both shoulder points steady
to about a quarter of a millimetre. The band was found in 14 of 14 stretches,
12 to 16 mm wide. There is no tape reading of that collar to compare against.
The tester style's target for it is 280. That frame's calibration had been
replaced by the time this was written, so the millimetres could be about 1% out.

**On the rendered collar**, with a known seam of 286.47 mm between known
shoulder points:

| check | result |
| --- | --- |
| twelve boxes, including loose, off-centre, tall and very loose | spread 0.13 mm |
| the reading against the true length | 287.2 mm, +0.7 |
| the line read, against the seam | 0.08 mm RMS |
| laid neck up, neck down, neck to the side, eight degrees askew | all within 1 mm |
| the opening's edge, and the chord | within 0.5 mm |
| with the station's outline, and without it | agree to 0.03 mm |

The +0.7 mm is mostly the two ends sitting a quarter of a millimetre past the
rendered edge of the cloth, where a printed grid line of the mat runs along it.

**The whole collar (`"around"`)**, on the same rendered collar, whose back is
the straight edge of the cloth between the feet and so exactly the chord:
463.39 mm against a true 462.18 (front +0.7 as above, back 176.21 against
175.72, the same ends), the back 0.23 mm RMS from the edge of the cloth, six
boxes within 0.02 mm, with and without the outline within 0.03 mm, neck to the
side within 1.4 mm. Through a camera-free station, box to verdict: 463.76 mm,
the same from a much looser box. On the real white round neck on the blue cloth
(the `collar-fail.png` committed in `3bd6159`, whose own box cut the top of the
collar off - redrawn round the whole collar): **461.1 mm = 279.9 over the front +
181.2 across the back**. Seven boxes read 461.05 to 462.06 mm: six within
0.3 mm, and one loose box 1.0 mm long because the right shoulder point landed
1.4 mm further along the edge of the shirt. The front alone moved only 0.45 mm
for it, since a shoulder point sliding along an edge the seam meets squarely
barely changes the seam - but the back runs along that same edge and takes the
whole of the slide. That is the number to watch in a gauge study of this
step.

## Where it fails, and what to check on a bench

- **It needs the collar laid so its seam runs to the edge of the shirt**: face
  up, collar flattened, both shoulders on the mat. A collar whose ends are folded
  under, or whose shoulders run off the picture, has no shoulder point to find,
  and the reading is refused.
- **It needs a band.** A self-fabric binding with no groove and no change of
  shade has no edges for the band test to find. It is refused rather than
  measured, and lines are offered for a tap when any were found.
- **The ends are refined on the picture for a pale shirt on this dark mat.** A
  dark shirt falls back to the outline's edge, and the outline's glow and shadow
  errors come with it.
- **Gauge R&R.** The claim is that the operator is out of the reading, so test
  exactly that: three operators, one garment, five boxes each, and the standard
  deviation. On the real collar above, eight boxes spread 0.09 mm. What is left
  is how the collar is laid, which no machine can take out. Pulled flat it is
  longer.

## Tuning

| field | default | what it does |
| --- | --- | --- |
| `collar_corridor_mm` | 6 | how far either side of each edge the final read searches. Narrow, to hold the line the band put it on and not the topstitch beside it |
| `collar_edge_reach_mm` | 12 | how far a line may be carried on to meet the edge of the cloth. One that has to go further does not get there, and is not a collar line |
| `collar_edge_search_mm` | 8 | how far either side of the outline's edge the picture is searched for where the cloth really ends |
| `collar_band_min_mm`, `collar_band_max_mm` | 8, 35 | how far apart a band's two edges may be. Closer is a seam and its stitch; further is usually a seam and a fold |

And per step, in the style file:

| step field | default | what it decides |
| --- | --- | --- |
| `"kind": "collar"` | | this is a collar step |
| `"seam"` | `"outer"` | which edge of the band: `outer`, the attachment seam, or `inner`, the opening's edge |
| `"measure"` | `curve_measure_along_arc` | `arc` along the edge, `chord` straight between the shoulder points, or `around` the whole collar - the outer edge over the front and the edge of the shirt across the back |
