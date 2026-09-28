# Garment curves: point at the seam, measure the cloth

Some dimensions cannot be folded into an edge.

[garment_qc.md](garment_qc.md) describes the whole of the rest of this bench: the operator
folds the dimension into one straight edge, presents it to the rail, and the
station measures it. That works because every one of those dimensions is on the
**outline** of the cloth, and the outline is what a background difference
against the empty mat finds.

A neckline is not on the outline. The collar meets the body somewhere in the
middle of the shirt; the neck opening shows the back panel rather than the mat.
There is no fold that turns that seam into a silhouette, and the segmentation
sees nothing there at any resolution. The same is true of a collar seam, a
placket, a pocket line, and an armhole left unfolded.

So for those, the operator says where it is.

```
step 5 of 5  ->  lay flat  ->  point x3  ->  snap to seam  ->  PASS/FAIL  ->  Next
```

**The pointing is not the measurement.** That sentence is the whole design and
everything below is a consequence of it.

| file | what it does |
| --- | --- |
| `include/verimeasure/curve.hpp`, `src/curve.cpp` | builds the corridor, finds the seam in it, measures it |
| `include/verimeasure/finger.hpp`, `src/finger.cpp` | finds the fingertip and decides when a point has been made |
| `src/qc.cpp` | `judgeCurve`, the same verdict rules the folded edge gets |
| `tests/curve_check.cpp` | offline harness and selftest, no camera needed |
| `templates/refs/neck_line.svg` | a worked curve step and its reference picture |

## Why the trace is not the answer

The obvious thing to build is a finger tracker that reports where the finger
went. It would be wrong, and the numbers say by how much.

The plane model is a homography onto the mat. A fingertip is about ten
millimetres above the mat, and a point at height `h` sitting `r` from the
optical axis is displaced by `r·h/D`. On this bench — 0.1378 mm per pixel on a
2.4 µm sensor, so a magnification of 57 and a working distance near 935 mm on a
16 mm lens — that is **4.0 mm at the edge of the frame** and zero at the middle,
so a straight trace comes back visibly bent. Track the top of the finger instead
of its contact patch and it is 7 mm.

Then add the rest. The tip of a finger seen from above is ambiguous by most of a
centimetre, because the cloth is touched under the pad and the silhouette
carries on past it. A hand shakes. And a finger dragged along ribbed jersey
**stretches the thing being measured**, which no amount of image processing
recovers.

A traced path is a reading of the operator, not of the garment. It would also
destroy the property this bench is built on — that the number is a fact about
the cloth, so the mat can slide and two operators agree.

## What the pointing is for

It names a corridor.

```
seeds (+/- 10 mm)  ->  corridor +/- 15 mm  ->  snap to the seam  ->  measure
```

Inside the corridor the seam is found from the picture, sub-sample, by the same
kind of gradient search `refineContourSubpixel` runs along an outline normal.
The pointing has to be good to about a centimetre. The answer is good to a
fraction of a millimetre.

Every error above lands in the first arrow and dies there. Parallax, tip
ambiguity, shake, a tablet touch four display pixels out — all of it is inside
the corridor, and none of it reaches the number. That gap is why a **bare
finger** is enough, and why the "sensor on the finger" this started as is not
needed: there is nothing for it to improve. An IMU drifts metres over a
three-second trace, a magnetic tracker hates a steel bench, and anything
wireless has to be synchronised with a camera that is already looking straight
at the hand. If a physical aid is ever wanted, it should be passive — a printed
fiducial on a thimble, no electronics — and it would buy reliability of
detection, not accuracy of measurement.

There are two different accuracies here, and it is worth separating them because
the first version of this got it wrong by not doing so.

**Finding a seam that is alone in its corridor** tolerates a centimetre of
pointing: the measured RMS distance from the found path to a seam pointed at
10 mm off is 0.04 mm.

The corridor is sized by **the middle point, not the ends**. Pointing at an end
of a seam is easy: it is a corner, there is something to aim at, and it costs a
few millimetres. Pointing at the deepest part of an arc is nothing like as easy,
because there is no landmark there at all — the operator is estimating where the
middle of a curve ought to be. On the first real jersey this met, the two ends
landed on the collar seam and the middle point sat **25 mm** above it. With the
15 mm corridor it started life with, the seam was outside the search for the
whole middle of its run and there was simply nothing to find. It is 30 mm now.

That point also sets the *shape*, because the arc passes exactly through all
three: an error there bends the whole corridor rather than shifting it.

**Choosing between seams that lie near each other** is a different question, and
the pointing is the only thing that can answer it. A neckline has several lines
within a centimetre - the neck opening, the binding edge, the attachment seam, a
coverstitch - and nothing in the picture knows which the tech pack means. The
pointing has to be good to roughly half the spacing to the nearest line it must
not pick, which is about 3 mm on a real neckline. Worse than that and the
machine either takes the neighbour or, when the two are genuinely
indistinguishable, refuses.

`curve_check --selftest` is that claim, taken apart: it points at a seam of known
length off by up to a centimetre, six different ways, and requires the readings
to agree.

## Draw the seam, don't tap at it

Three taps was the first way of doing this, and it half worked. Two of the three
are easy — an end of a seam is a corner, there is something to aim at. **The
middle one never was.** At the deepest part of an arc there is no landmark at
all; the operator is estimating where the middle of a curve ought to be. On the
first real jersey this met, the two ends landed on the collar seam and the
middle point sat 16 mm above it, in plain cloth. And because the arc passes
exactly through all three, being wrong there *bends the whole corridor* rather
than shifting it.

So: **hold still at one end until the ring closes, draw along the seam, and then
take your hand away.** What the finger did in between is the path. On the tablet
it is simply a drag: finger down, along, up.

**Every wait in that is counted in frames, and there are two of them.** This
bench delivers 1.9 frames a second — a wire limit, sixty megabytes a frame down
one gigabit, see below — and at that rate a wait expressed in seconds is either
one sighting or several seconds of the operator looking at a ring. Starting
takes `finger_dwell_frames` frames within `finger_dwell_radius_mm`, or
`finger_dwell_seconds`, whichever is longer: two frames is the fewest that can
tell a hand that stopped from a hand passing through, and at 1.9 fps that is a
second. Ending takes `finger_trace_lift_frames` frames with no hand in the
picture — two, another second — because a camera looking straight down cannot
see a finger *lift*; the finger is still there, a few millimetres higher, so what
it watches for is the hand leaving, and for two frames rather than one because a
hand crossing its own shadow drops out for a frame and comes back. Nothing in
between ends a trace. A hand drawing a seam slows almost to a stop all the time —
at a corner, at the deepest part of an arc, whenever the operator looks up — and
any still-hold short enough to be worth having ended the trace halfway along the
collar for no reason the operator could see. The three-second still-hold that
used to be the other way to finish is off (`finger_trace_end_seconds` is zero);
a bench where the operator cannot lift clear of the picture can turn it back on.

**And a point the hand could not have reached is not a point.** The tip is found
afresh on every frame from a blob and a rule, and a blob can be wrong — a fold
that lifted when the hand brushed it, the corner of a shadow, the elbow on a
frame where the hand went dark. One such point is a spike a hundred millimetres
off the seam, and the corridor is built along the path, so a spike in the path
is a spike in the corridor that no snapping recovers from. A tip further from
the last believed one than `finger_max_speed_mm_s` allows in the time since it —
three hundred millimetres a second, against the thirty to a hundred a hand
draws at — is not recorded, and the ring on the feed goes hollow for that frame.
After `finger_lost_frames` of it in a row the new place is believed: two frames
agreeing on it is the hand really being there, and a tracker that will never
accept a big move is one that can be wedged forever.

**The frame rate is the wire.** 5472 × 3648 × 3 bytes is 60 MB a frame, and a
gigabit link carries about two of those a second. Switching the camera to an
8-bit Bayer format and debayering on the PC would cut the traffic to a third
and roughly triple the rate — the camera setup prints exactly this at startup —
but it changes what every measurement is made from, and the Bayer phase has to
be right or the sub-pixel edge fit suffers. It is a separate change. Everything
in the trace is designed to be right at two frames a second, and a trace of a
neckline at that rate is ten to fifteen points, each of which has to be on the
fingertip; the corridor does the rest.

A traced path has no weak point. It is near the seam everywhere, because the
operator was looking at the seam the whole way along. What it carries instead is
the hand — a point every fifteen millimetres or so at whatever rate the camera
runs, each shaking by a millimetre or two — and that is noise, which averages
away. The 16 mm was not noise; it was one estimate, and no amount of averaging
would have helped it.

**And tracing costs nothing in occlusion**, which is what makes it work at all.
The seam is read from the frame held *before* the hand came in, so a finger
lying across the seam it is tracing hides nothing that is going to be measured.
The operator can hover rather than touch — which is also the end of the
objection that a finger dragged along ribbed jersey stretches the thing being
measured.

A trace is smoothed before it becomes a corridor, and with a running **quadratic**
rather than a line. Averaging out the hand needs a window spanning several of its
points — forty or fifty millimetres — and a straight line fitted across fifty
millimetres of a 77 mm neckline cuts the corner by four. That is the curve being
flattened to remove the hand that drew it. A quadratic follows curvature exactly
to second order, so the same window costs nothing: it takes the shake and leaves
the arc.

The window has to be long enough to actually remove a hand, and that is worth
more than it sounds, because the corridor is built **along** the traced path.
Whatever wiggle survives in the path is wiggle in the ribbon's own geometry, and
the search that follows can move across the corridor but never along it — so a
seam found perfectly still comes back measured on a crooked ruler, longer than it
is, every time and only ever longer. It is the same bias `curve_smooth_mm`
exists to remove one level down, and it is larger here because a hand is noisier
than a sub-pixel fit. On a 254 mm neckline traced with 3 mm of shake: **30 mm of
window read 262, 45 mm read 254.1**, and four traces of one seam agreed to 2.0 mm
at 30 and to 0.5 mm at 45. Past about 70 the fit begins cutting the corner
instead and the arc turns short. `curve_trace_smooth_mm` sits at 50.

Single taps still work, and the station still accepts them.

## Finding the neckline without being pointed at

The pointing is where the reading's remaining error lives: a snapped seam is
repeatable to a fraction of a millimetre along its length, and its two *ends*
are wherever the operator put them, to a few millimetres each, along the seam
where the picture cannot correct them. The picture has the ends too. So on a
curve step (`"find": "auto"`, the default) the machine now tries first, the
moment the reading holds, and asks the operator only when it is not sure.

**The corners come from the outline.** A shirt laid with its neck toward the
mark has the neck as a notch in the refined outline the reading was just taken
from — or, when the collar is flattened and the back panel fills the opening,
a shallow dip where the collar's ends meet the shoulder line. Either way the
notch is the convexity defect on the mark side that opens toward it (the armpits
open the other way), and its two corners are where the outline *leaves* the line
across its mouth: the intersection of the departing edge with that line, which
is what a corner is and what a shoulder point is. Walking in from the hull's
own vertices matters, because when the shoulder runs level into a sleeve the
hull spans the whole shirt.

**The depth comes from the picture, not the target.** The tech pack's number
is the garment as designed and was wrong by a third on the first real collar;
and the collar it described was a horseshoe, a hundred millimetres deep between
corners a hundred and fifty apart, past a semicircle. So corridors are laid at
every depth a neck can have, from a boat neck at a sixth of the width to a
horseshoe at four fifths, twenty millimetres apart — what the rigid model can
bow to from either side — and every one is snapped. Each distinct line that
reads is kept (`findSeams`). On that collar there were three: the back band's
edge at 23 mm, and the collar's two edges 19 mm apart at 80 and 99 mm.

**The choosing is a rule the step states, never a guess.** Nothing in the
picture says which of three real, strong, equally readable lines the tech pack
means. `chooseSeam` uses the target only as a *gate* — a seam more than a
quarter away from it is not the seam the tech pack is describing — and never as
a preference, because picking whichever length is nearest the target would bias
every garment toward PASS. What is left decides: one seam is the answer; two a
band's width apart are the collar's edges, and the step's `"seam"` says which —
`outer`, the attachment seam, is the larger horseshoe, the one that dips
*deeper* from the chord across the mouth; `inner` is the opening's own edge.
Anything else, the machine says what it found ("155, 245 and 285 mm, cannot
tell which is meant"), draws all of them thin on the feed, and waits for the
operator to point. A refusal in this path is never filed as a FAIL: nobody has
pointed at anything yet.

**What it costs and where it fails.** Six snaps of one corridor each, about a
third of a second once per hold. It needs the neck toward the mark, a
calibration that is current, and a target that is roughly right if it is to
choose alone. Its ends are the outline's corners, which are the shoulder points
only when the collar is laid so its ends sit at the shoulder seams; a collar
whose legs run down past them reads long by the legs. Checking the ends
against the shoulder seams themselves is the next thing to build.
`curve_check --neck --frame FRAME [--target MM] --draw out.png` runs the whole
thing on a saved frame and draws every seam it found.

## Two points, a ruler

There is one step kind that does none of the above, on purpose. `"kind":
"distance"` is two taps on the held picture — or one drag, press at one end and
release at the other — and the reading is the straight line between them in
millimetres, through the calibration, distortion and all. No seam is looked for
and nothing is snapped or corrected: the number is what a ruler laid on the
picture between those two places would say. It exists for the dimensions that
are simply two landmarks apart with no seam between them for a corridor to
follow — a neck width from shoulder point to shoulder point across the opening,
a placket length — and as the plainest possible check of the calibration
against a tape.

Because nothing is corrected, its accuracy is the operator's aim on the feed. The
feed is the frame at a quarter scale, so a screen pixel is four on the sensor
and about half a millimetre on the mat; a careful tap is good to about that. Both
points are logged in pixels and in millimetres so a disputed reading can be put
back on the picture, and the line stays drawn on the feed with its number until
the next first tap. The finger tracker is not armed on a distance step: the
points are wherever the mouse put them and nothing on the feed follows a hand.

**A drag is a tape.** Hold the button down at one end, follow the curve, let go
at the other, and the reading is the length *along what was drawn*. The browser
draws the line as it is drawn — the station does not hear about a drag until the
button comes up, and a curve traced blind is not the curve — and the station
replaces it with the measured path a moment later. Two taps and one drag are the
two halves of one tool: the straight-edge and the tape.

The one thing done to a drawn path is the one thing that has to be. A path drawn
by hand is jittery at the scale of a screen pixel, and jitter never averages out
of an arc length: every wobble lengthens the segment it sits on, so a jittery
path reads long — always long, never short. Measured: a neckline drawn with ±0.6
mm of jitter at a point every two millimetres reads **262.4 mm as drawn** and
**255.2 mm after the fit**, against 254.4 true. The fit is the same running
quadratic the corridor uses on a hand-drawn trace, over `curve_trace_smooth_mm`,
and what it costs is a rounded corner: a sharp V is cut by a couple of
millimetres. A step whose tech pack wants the span across a drawn curve says
`"measure": "chord"`. The drawn points go in the log as they came, so the
smoothing can be re-run against a changed setting.

**When the ruler disagrees with a tape, read the mat.** The click-to-millimetre
path is the calibration and nothing else, and the mat's printed grid is a known
length lying in the picture. `curve_check --ruler --frame FRAME --seeds-px
--seeds "x,y;x,y"` is the station's ruler offline: give it two grid crossings
and it should read what the grid says. On the bench this was written on, a
23-cell column of the grid read 224.9 mm for 230 — the calibration was two days
old and the geometry had moved — which is a recalibration, not a software
change: shirt off, Recalibrate, read the grid again.

## Three taps, not two

Two taps name the ends of the seam. They do not describe it.

A scoop neck 180 mm across the shoulder points dips **75 mm** below the chord
joining them. A corridor built around that chord contains no neckline along most
of its length: the search would find bare cloth, or worse, find the shoulder
seam and measure that instead. The selftest asserts exactly this, so nobody
optimises the interaction down to two taps later without being told why it was
three.

Three points are the fewest that describe a curve rather than its ends, so
`seedPath` fits the circle through them and follows that arc. A garment neckline
is close enough to circular that the arc through three taps lands within a few
millimetres of the seam the whole way along — which is what the corridor is
sized for. Four or more are taken as a trace and followed as they lie; two give
the straight line between them, which is right for a seam that really is
straight.

Taps rather than a drag, and it is not only about speed. A drag presses along
the seam and stretches it; three taps touch three places and lift.

## How the seam is found

**The corridor is rectified first.** It is sampled into a straight ribbon —
one row per `curve_sample_mm` along the seed path, one column per
`curve_sample_mm` across it — so everything after that point works on a small
rectangular image in which the seam runs top to bottom.

Two things follow from that, and both matter. Every parameter becomes a length
on the mat rather than a count of pixels, so nothing here is tied to this lens
or this working distance. And "along the seam" becomes a direction the code can
actually use, which the next paragraph depends on entirely.

**It is smoothed along the seam far harder than across it.** This is the one
step that separates a seam from cloth, and it works because a seam is the only
thing in the corridor that *runs*. Knit texture is a millimetre across and goes
nowhere; a crease wanders; a speck is a speck. Averaging `curve_along_mm` of the
seam's own length lifts it clear of all of them, and costs nothing across, where
the accuracy lives.

**Both kinds of line are looked for.** Where a rib meets a body the shade
changes and the seam is a **step**, which the first derivative across the
corridor finds. Where a seam is topstitched into cloth of its own colour there
is no step at all, only the stitching: a **ridge**, and the first derivative is
exactly zero at the middle of one with a peak on either flank. A step detector
pointed at a stitched seam does not find it badly — it finds the two sides of it
and reports whichever flank scored, about a millimetre out, in a direction
nobody can see. So the second derivative is computed too, and it is the mirror
image: it peaks in the middle of a ridge and passes through zero at a step. A
ridge counts only where there is no step to explain it, which is what stops the
flanks of an ordinary step outscoring the step.

**Strength is measured against the corridor's own noise, and against a floor.**
Robust, because a coverstitch sets the maximum of any corridor it appears in and
a scale taken from that maximum makes the seam beside it look like nothing. And
floored at `curve_min_contrast` gray levels, because a scale taken *only* from
noise divides by whatever little noise it finds — on smooth cloth the tails of a
feature then clear the threshold just as its middle does, the evidence goes flat
for a couple of millimetres either side of the seam, and inside a flat band the
only thing left with an opinion is the pointing. That is the failure this whole
file exists to prevent, arriving through the back door, and it cost 1.4 mm of
accuracy before the floor was added.

**Then compressed, not clipped.** Once a line is clearly a line, being brighter
must not make it more correct — the strongest of a neckline's four lines is
almost never the one the tech pack means. But clipping flat has the same
flat-band problem as above. `x/(1+x)` flattens the difference between a strong
line and a merely clear one to about 0.08, against a pointing worth 0.5, while
staying strictly increasing everywhere — so a line still has a peak, and the
peak is still exactly where the picture puts it.

### Which line, then where — and never both at once

This is the part the first version got wrong, and the way it failed is worth
recording because it looked like success.

That version searched for a smooth path, penalising it for turning quickly. Such
a path pays for the **rate** at which it crosses the corridor and never for how
far it has gone, so drifting one sample sideways every twentieth row costs
almost nothing and arrives, four hundred rows later, on a different line
entirely. On a rendered neckline it did exactly that: one reading spent 57% of
its length on the attachment seam and 36% on the neck opening six millimetres
away, another spent 74% on a coverstitch. Each came back smooth, confident, and
wrong by a centimetre and a half of arc.

A seam is not an arbitrary smooth path. It is **one line**. So the search is in
two stages that cannot be collapsed into one.

**Stage one chooses the line, once, for the whole length,** over a rigid family:

```
offset(u) = a0 + a1·u + a2·(2u² − 1),     u from −1 to +1 along the corridor
```

a shift, a tilt and a bow relative to the arc through the taps — exactly the
freedom a circle fitted to three slightly wrong points needs in order to become
the right circle, and no more. A model that spends part of its length on a
second line now pays for the part that does not fit, over the whole run, instead
of drifting between them for free. This stage sees the compressed evidence *and*
the pointing.

**Stage two follows the cloth,** free again but only within `curve_follow_mm` of
the line it was given: wide enough for every wobble a garment seam has, far too
narrow to reach the next line along. **The pointing takes no part in this
stage.** It has already done its work, which was to say which line; where that
line lies is the picture's to answer. Leaving the prior switched on here would
let a tap drag the reading a fraction of a millimetre off the seam it had
correctly chosen, and two operators pointing differently at the same seam must
get the same number.

**The pointing enters as a cost, and it is capped.** Sitting away from the
middle of the corridor costs the square of how far, reaching the cap at
`curve_prior_mm`. Square because a square is flat at the bottom — near the
middle it has no opinion, so the picture alone places the seam. Capped because
an uncapped cost grows without limit and a tap a centimetre out puts a *lone*
seam beyond reach, which is a worse failure than the one the prior prevents. The
cap's working range is 0.50 to 0.60, measured: below it a lone seam 10 mm off
the pointing stops being found, above it two lines 4 mm apart stop being told
apart.

**Then to better than a sample.** A parabola through the response either side of
the chosen column puts the seam where that response actually peaks — on whichever
of the two kinds of line won there, so a step is located on the step and a ridge
on the ridge. It uses neither the compressed evidence nor the prior.

**And the path is fitted, not followed.** The samples each carry a few
hundredths of a millimetre of noise. That is nothing for the chord, which is two
points, and a great deal for the arc, which is a thousand of them added end to
end: noise that averages to nothing still lengthens every segment, so a
fortieth of a millimetre of jitter inflates a 200 mm neckline by four. It only
ever adds, which makes it the same kind of bias this pipeline already refuses in
`minAreaRect`. A running line fit over `curve_smooth_mm` removes it, and what it
removed is reported as `roughness_mm` rather than discarded.

### Cutting the reading back to what was pointed at

The end of the measurement is the foot of the operator's tap **on the seam that
was found** — the nearest point of it to where they pointed. So the corridor is
snapped over a longer run than the reading needs, and trimmed afterwards.

Without that step the ends sit wherever the arc through the taps happens to
stop, and that arc is fitted to taps that are off the seam — so a tap displaced
*across* the seam tilts the whole corridor and slides its ends *along* it. On a
scoop neck pointed at 10 mm off, that alone moved the reading by **1.75 mm**
between one pointing and the next: most of what a half-inch band allows a gauge
to spend on itself, spent on nothing. With the trim it is **0.52 mm**.

### What the snap cannot fix

It corrects across the seam. It cannot correct along it.

A tap that slid 5 mm along the seam moves that end of the measurement 5 mm,
because nothing in the picture knows where a neckline is supposed to stop —
only the operator does. The selftest measures this rather than hiding it: with
taps sliding up to 6 mm along, the path stays on the seam to 0.04 mm RMS and the
*reading* spreads by about 13 mm.

That is the honest limit of this design, and it is bounded by the pointing
rather than multiplied by it. The fix, when it is wanted, is the one already
written for folded edges: `adjacentEdge` in `edge.cpp` finds where two edges
would have crossed, and a shoulder point is exactly that — the shoulder seam
meeting the neckline. Snapping the ends to that corner would take the operator
out of the answer entirely.

### Refusing a corridor it cannot read

Each of these, let through, gives a confidently wrong number rather than a
slightly worse one.

**No seam there at all.** The search always returns a best path. On a corridor
with nothing in it that path is smooth, plausible, drawn on the operator's
screen and completely invented. The evidence is normalised against the level the
corridor sits at anyway rather than against its own maximum — a ratio against
its own maximum could not tell the two apart, because the best path through
noise is near the maximum of that noise by construction. An empty corridor
scores near zero and is refused. On a white rib against a white body this is
what will refuse the reading, and refusing is correct: a collar that cannot be
seen has not been measured.

**The snap went somewhere else.** `drift_mm` past `curve_max_drift_mm` means the
machine did not correct the pointing, it found a different feature — the
shoulder seam, the edge of a print — and is now measuring one thing while the
operator watches another.

**The pointing did not actually choose anything.** `margin` is how much better
the line that was measured was than the best line elsewhere in the corridor, and
it is the guard that earns its keep. A double-needle coverstitch is two parallel
lines a few millimetres apart, and an operator pointing between them has said
nothing about which. Both are real, both are strong, both are the same distance
from the taps, and the winner is then decided by nothing at all - so the reading
is repeatable within one pointing and jumps by the spacing between garments.
This is the only failure here that survives every other check: the path is
smooth, the confidence is full and the drift is small, whichever of the two it
picked.

`curve_min_margin` is **zero by default, so the number is reported and never
refuses** — and that is worth being blunt about. The failure is real and nothing
else here can see it, but the only numbers this threshold was ever set against
were rendered ones, and on the first real garment it met it refused a lay whose
seam had been found at full confidence. A limit calibrated on synthetic cloth
and never on a bench is not a limit, it is a guess with an error message, and
this one guessed in the direction that stops work. Run a shift, watch what
`margin` reads on good lays and on ambiguous ones, and set it between them. The
mechanism is tested and ready; the threshold is yours to find.

**The corridor runs off the picture.** Same rule as the folded edge, and the
same reason: what comes back is smooth, repeatable and exactly as long as the
view.

**Shorter than `curve_min_length_mm`**, or **not calibrated**.

A refusal is a FAIL, not a skipped step — identical to the folded edge, and for
the identical reason. Pointing again is the Retake button, and a refusal leaves
the points on screen rather than throwing them away, because the usual cause is
one tap in the wrong place.

## Finding the finger

The hand is found by differencing against the **settled scene** — the garment
lying on the mat — and not against the empty mat. That is the difference between
a picture with an arm in it and a picture with a shirt and an arm merged into
one blob, which has no arm in it to find.

**And it is differenced by colour, not by brightness.** This one is not a
refinement; it is the difference between working and not. A hand held over pale
cloth under a bench light throws a shadow that reaches *further across the cloth
than the hand does*, lies in the same direction as the arm, and touches the same
border. Anything that asks only how much the brightness changed finds that
shadow, takes its far corner for a fingertip, and puts the point a couple of
centimetres past where anybody pointed — which is exactly what it did on the
first real garment. A shadow scales all three channels together and leaves the
colour vector's direction alone; skin does not. `segmentObject` already knew
that, so the pointer uses it rather than a second foreground test of its own.

**And with the hand's thresholds, not the garment's.** The garment segmentation
is tuned to see cloth against a dark mat and to ignore the cloth's own shadow,
and against those thresholds a brown hand over a salmon shirt *is* a shadow.
Measured on the real held frame: the shirt's body is RGB (202, 134, 122), a
brown hand about (150, 95, 70), and the two differ by five degrees of colour
angle against the twelve the garment test wants, at seventy percent of the
brightness against a test that only believes "darker" below thirty-five. So the
hand was invisible wherever it was over the shirt and visible only where it
crossed the mat or the red print — and the tip landed where the forearm met the
hem, 150 to 250 mm from the finger, or jumped to whatever fragment of the hand
happened to lie over something dark. That is the saw-tooth trace: the points
were wherever the hand was *visible*, not where the fingertip was.

What actually separates skin from a shadow is not how dark it is but *how* it
is dark. A shadow is the same light, less of it — every channel scales by the
same factor. Skin over cloth is a different reflectance, and it darkens the blue
channel far more than the red: a channel-ratio spread near 0.17 on this shirt,
against under 0.05 for a real shadow. That test already exists as
`shadow_uniformity`, and it holds; the two gates in front of it are opened for
the hand (`finger_chroma_angle_threshold`, `finger_intensity_ratio_low`). A
composited hand on the real frame then lands to **0.1–0.7 mm** in brown and
dark skin at three places on the neckline; light skin, whose shaded edges over
pale salmon really are the colour of shaded salmon, to 1.5–16 mm.

One thing no colour test can see: dark skin lying over the dark, warm shadow a
hem throws, or over a fold. Same brightness, same hue. Those places are narrow —
a fold is five to fifteen millimetres — but the wrist crosses exactly one of
them where the arm comes over the hem, and a hand that has come loose from its
arm touches no border and is not a hand as far as the tracker is concerned. The
mask is closed over `finger_bridge_mm` so the hand stays attached. Closing fills
concavities and never extends a convex end, so it cannot invent a fingertip.

`curve_check --hand-probe --frame FRAME` paints a hand onto a real held frame
in three skin tones and reports where the tracker puts the tip — the test to
run when a bench meets a garment in a new colour.

**Does the moving blob touch the border?** A hand is attached to an arm and an
arm comes in from outside the picture. A moving thing that touches no border is
a shadow, a draught in a hanging sleeve, or the garment settling further — and
each of those, taken for a fingertip, puts a point somewhere nobody pointed.

**Which end of it is the finger?** The point furthest from where the arm crosses
the border, measured **along the arm** and taken from the **thin** part of it.

Both halves of that were learned the hard way, and both are about the one thing a
curve step asks the operator to do. Nobody re-aims a whole arm at every
millimetre of a neckline: the forearm stays where it is and the wrist bends. So
*furthest in a straight line* — which is what this used to be — stops meaning the
fingertip almost immediately. Past about forty degrees of wrist the far side of
the hand is further from the border than the finger is, the tip steps back onto
the knuckles, and it stays there for the rest of the trace. What the operator
draws along the seam then gets recorded as the hand wandering behind it: shorter,
blunter, close to straight, and *not refused*, because a path drawn by a hand
rather than a fingertip is still a perfectly smooth path. On a rendered neckline
at sixty degrees the reported tip sat 48 mm from the finger and 29 mm from the
knuckle.

Along the arm — a chamfer distance from the border pixels, inside the blob —
none of that can happen: the fingertip is the last thing reached however the
wrist is turned, because the finger is attached to the hand and the hand to the
arm.

The thinness is the second half, and a palm is why. A palm is 85 mm across and a
finger 17, so with the wrist turned right over the far corner of the palm can
still be *further along the arm* than the fingertip — not by much, and not
always, which is the worst way for a rule to be wrong. A fingertip is not merely
the far end of the arm; it is the far end of the thin part of it, and thin is
something the picture can be asked directly. The distance transform reads eight
or nine millimetres down a finger and forty across a palm, and there is nothing
in between to confuse. When no part of the blob is that thin — a closed fist —
it falls back to the far end of the arm, and the setback cap below bounds what
that can cost.

Measured on a hand drawn at the size of a hand, tracked around a 254 mm
neckline: the traced path sits **0.3 to 0.6 mm** off the seam at every wrist
angle from −60° to +100°, and keeps the arc's 75 mm bow to within 0.3 mm. It
costs 5 ms a frame on this bench's decimated mask.

**And which blob is the arm?** The one the tip was in last frame, if it still
reaches a border; otherwise the largest. A hand does not teleport between
frames, and size alone is not enough to keep hold of it — a sleeve settling, a
sheet of paper nudged by the wrist, and the tip steps across to whatever has
become the biggest thing touching an edge, taking the trace with it.

**Then take the parallax out.** This is the largest single error in pointing and
the only one that is not random. A fingertip 10 mm above the mat images
displaced *outward* from the middle of the picture, by the radius times the
height over the working distance — 4.5 mm at the edge of this frame, none at the
centre, and smooth in between. Smoothing a trace will average away shake all day
and never touch it, because it is the same push in the same direction every
time. Worse, it is a bias of the same size as the gap between a seam and its
coverstitch, so it decides which of the two gets measured.

Scaling the tip back toward the middle by `(D − h) / D` is the same correction
`measure.cpp` applies to a thick object's size, applied to a single point.
Measured on a bench-sized field it takes **4.50 mm down to 1.15 mm**. It needs
the working distance, which comes from the lens focal length — so **pass
`--focal-length-mm`**; the station derives the distance at startup and says out
loud whether it could. Without it the correction is skipped rather than guessed,
because a correction applied with a guessed distance is a second error rather
than the removal of a first.

**Then back off by half a finger — a finger, not a hand.**  The furthest point of the silhouette is the
leading edge of the fingertip; the cloth is touched under the pad, which from
above sits at the centre of the rounding. How far back is measured rather than
assumed — the distance transform of the blob is the distance to its nearest
edge, so its largest value near the tip *is* that fingertip's half-width,
whether the operator is using a finger, a thumb or a gloved hand. 
How far back is measured from the picture rather than assumed, and measuring it
over the wrong window is not a small error. The window used to be a fraction of
how far the arm reached into the frame, which on a real arm is seventy
millimetres — so what got measured was the width of the **hand**, thirty-five
millimetres of fist, and the tip was duly set back into the middle of it. It
tracked the hand rather than the finger and the trace wandered around the
knuckles. The window is a fingertip's own size now, and the setback is capped at
a fingertip's own half-width, which is what bounds the damage when somebody
points with a closed fist and there is no fingertip in the picture to find.

It is worth doing even though the corridor would forgive all of it, because the
offset points along whatever direction the hand happens to lie: partly across
the seam, where the snap removes it, and partly *along* it, where nothing can.

**And back off along the *finger*, not along the arm.** Those are the same
direction only while the wrist is straight, which on a curve step it mostly is
not. The window that measures how wide the fingertip is gives the direction for
free — its centre of mass sits behind the leading edge, down the middle of the
finger, because the finger is the only thing in the window. Aimed along the arm
instead, a bent wrist turns the setback sideways, and a centimetre applied at
right angles to the finger is a centimetre straight across the seam.

**A point lands on a dwell, not a button**, because the other hand is holding
fabric. Hold still for `finger_dwell_seconds` within `finger_dwell_radius_mm`
**and for at least `finger_dwell_frames` frames**, and the point is taken — as
the mean over the whole dwell, not the last position, since the frames of a hold
average out most of the shake for nothing.

The frame count matters as much as the time. A wall-clock hold on its own is not
a hold that anybody checked: this bench runs at 1.9 frames per second, where
four tenths of a second is barely one frame. The tip counted as held still on
the strength of having been seen once, a point landed about every second the
hand stayed near the seam, and a step wanting three of them collected thirteen.
Counting frames makes the requirement self-adjusting — the four tenths on a
camera keeping up, a deliberate second on one that is not, and never a
commitment made on a single sighting.

**A full set of points stays full.** A read that works clears them, so a full set
means the last one was refused, and the answer to a refusal is to point again
rather than to add a fourth point to three that failed. Past three, `seedPath`
stops fitting an arc and follows the polyline through every point instead, so
the corridor zigzags across the garment and there is genuinely no seam anywhere
along it — a correct refusal, and a completely unhelpful one.

**And the points belong to the garment that was on the mat.** When the reading
stops being held, they go. A point kept across that is a point on a shirt that
is no longer there, and it draws on the feed over bare mat. A ring closes on the feed while it is being held: a point that lands
with no warning feels like a machine deciding things on its own. Sweeping a hand
across the mat to reach something commits nothing, which is what lets the
operator work over the shirt without scattering points behind them.

## The tablet is the other way in, and the first one to trust

The station already streams the held frame to a tablet. Tapping that picture
puts a point down, and by the time it reaches `addSeed` it is the same thing a
fingertip produces: millimetres on the mat.

It is worth having for its own sake — no parallax, no occlusion, nothing
touching the cloth — and it is the right thing to commission with. It exercises
the corridor, the snap, the guards, the verdict and the log without any of the
hand tracking being involved, which answers the question that actually decides
whether this feature works on a given bench: **can the snap see your seams?** If
a white-on-white collar refuses, no pointer would have helped.

Taps are sent as a fraction of the way across the picture and down it, never as
pixels. The feed is scaled on the station and again by whatever screen it lands
on, and a coordinate that had to agree with both would be wrong on the first
bench with a different monitor.

## The style file

A curve step is an ordinary step with two more fields.

```json
{
  "name": "neck_line",
  "kind": "curve",
  "measure": "arc",
  "target_mm": 254.0,
  "tol_minus_mm": -10.0,
  "tol_plus_mm": 10.0,
  "reference_image": "refs/neck_line.svg",
  "instruction": "Lay the shirt flat and face up with the collar flattened down. Point at one shoulder point, then the bottom of the neckline, then the other shoulder point."
}
```

`kind` is `edge` when absent, so every style file that exists keeps working
untouched, and a file that never uses a curve step is written back byte for
byte.

`measure` is `arc` along the seam or `chord` across it, and **it is not
optional in practice**. On a fold the two differ by about a millimetre in five
hundred and the bench has one global setting for it. On a neckline the arc runs
254 mm against a 180 mm chord — forty percent — so "neck width" and "neckline
seam length" are different measurements of the same seam, and a file that does
not say which has not specified the step. Both lengths are in the log either
way.

Tolerances, units, fractional inches and editing from the bench all work exactly
as they do for a folded edge; `judgeCurve` and `judgeStep` share one copy of the
verdict arithmetic, so the two kinds of step cannot drift into being judged by
different rules.

## Operator page

The panel gains a pointing strip on a curve step and loses it again on every
other kind, so nothing changes for a bench that never uses one. It shows how
many points are down out of how many are wanted, what to do next, and whether
the step reports the seam followed or the span across it — the operator has to
know which, because on a neckline it is a forty percent difference.

The feed takes a crosshair cursor and a live border while it is accepting taps,
and only while a reading is held: before that the picture is still moving and a
point on it is a point on nothing in particular.

**Nothing from the folded edge is drawn on a curve step.** The green span is the
brightest thing on the feed and it reads as the reading, so leaving it up while
the operator points at a neckline — stale from the step before, or an answer to
a question nobody asked — is worse than drawing nothing.

On the picture itself: the taps where they actually landed, the seam that was
found from them, and a ring that closes around the fingertip as the dwell is
held. Drawing the taps *and* the seam is deliberate — this is the one
measurement on the bench that the operator is inside, so as well as checking
that the machine found a line, they have to be able to check it is the line they
meant. That is not something a number can show them.

**Undo point**, **Clear**, and **Read the seam** for a seam wanting a different
number of points than the step expects. Otherwise the third tap is the
measurement and no button is needed.

## Audit log

One JSON line per attempt, as before, with `"kind":"curve"` and the numbers a
curve carries.

```json
{"timestamp":"2026-09-10T16:31:07","style":"innerverse tester","garment":3,
 "step":"neck_line","kind":"curve","measured":true,"measured_mm":254.36,
 "deviation_mm":0.36,"straightness_mm":0.01,"bow_mm":75.03,
 "chord_mm":180.04,"arc_mm":254.36,"radius_mm":91.53,
 "confidence":1.00,"drift_mm":7.60,"margin":0.13,
 "target_mm":254.0,"result":"PASS","curve":{ ... }}
```

Those are the real figures off `--write-sample`, pointed at up to 9 mm off the
seam: a 254.41 mm arc read as 254.36, a 180.02 mm chord read as 180.04, and a
91.50 mm radius read as 91.53. The confidence saturates at 1.00 because a
rendered seam is the cleanest thing in its own corridor; cloth reads lower, and
what that number is on **your** cloth is the thing to go and find out.

Both lengths are recorded whichever one was judged, so a run can be re-read
against the other definition if the tech pack turns out to have meant the other
one, rather than re-measured.

`confidence`, `drift_mm` and `margin` are on every line, passed or failed, for
the reason `straightness_mm` is: a bench whose collars are becoming unreadable,
whose operators are pointing further and further off, or which is drifting
toward lays where the pointing no longer chooses, shows up there first and
nowhere else. A small `margin` on a PASS is the warning that the bench is close
to the one edge that no other number here can see.

`radius_mm` is a cheap sanity check that costs nothing to record — a neckline is
very nearly circular, and a reading whose radius is not is a reading of
something else.

## Running it

Add a curve step to the style and run the station as normal. `--scan-from` still
applies to the folded-edge steps and is ignored by curve steps, which do not use
the rail at all.

Offline, with no camera:

```
curve_check --selftest
curve_check --frame lay.png --seeds "120,40;170,95;220,40" --calibration calibration.json
curve_check --write-sample .
```

`--write-sample` renders a neckline of known length with the calibration that
goes with it, and prints the command to read it back. It is a working example of
the second form, and a known-good pair to compare a real capture against when
one is refused and it is not obvious why.

**And a refused read leaves its own case behind.** The station writes the held
picture to `curve-fail.png` and logs the exact `curve_check --frame` line for
it, points and all. The bench that finds this feature wanting is not the one it
was written on, and a screenshot of a red FAIL says nothing about why; the whole
case can be re-run offline as many times as it takes, against a changed setting
or a changed build, with no garment and no camera in the room. One file,
overwritten, so a bench that refuses all morning does not fill its disk with
mornings. Points read off that picture with a viewer are in pixels; `--seeds-px`
turns them into millimetres through the calibration, and `--draw out.png` draws
the snapped seam back onto the frame — which is the only way to know it took the
attachment seam and not the topstitch beside it, because the numbers cannot say.

**And every trace leaves behind what the tracker saw.** The snap can be
replayed from a held picture; the trace cannot, because what the tracker gets
wrong is what it *sees*, and the frame with the hand in it is gone by the time
anyone asks. So the station writes `pointing-debug/` at the end of every trace —
not only a refused one, because the trace that matters is the one that looked
like a saw-tooth on the screen and was accepted anyway: the settled scene, the
last frame with the hand in it, the mask it made of the two, and where in that
mask it put the tip, all at the decimation the tracker works at, with the trace
in millimetres. `curve_check --pointing pointing-debug` runs that frame back
through the same `locate()` the bench ran, with this build's settings, prints
where the tip lands now against where it landed then, and draws the mask over
the frame. One directory, overwritten.

The second form is how a bench is commissioned: capture one frame of a real
garment, read three points off the grid, and see whether the seam is found and
at what confidence. If it refuses on your fabrics, that is the answer about this
feature on this line, and it is worth having before anybody builds a jig.

## What to check before trusting it on a line

The selftest proves the geometry. It cannot prove anything about your cloth or
your operators, and two things are worth measuring on the bench itself.

**Can the snap see your seams?** Run `curve_check --frame` over a dozen captures
spanning the worst cases — white rib on white body, a print near the neckline, a
collar that will not lie flat — and read the confidence. This is the question
that decides whether the feature works here at all.

**Gauge R&R.** Three operators, one garment, five repeats each, and report the
standard deviation. A half-inch tolerance band is 25.4 mm wide and a gauge is
usually asked to spend no more than a tenth of it on itself, so **2.5 mm** is the
number to beat. On rendered cloth with several lines in the corridor the snap
itself contributes 0.16 mm of spread; the rest of the budget is the operators'
along-seam pointing, which is where it will be won or lost, and which the corner
crossing described above is the way to buy back.

**And watch `margin`.** A line of readings whose margins are drifting toward
zero is a line whose garments are about to start being measured on the wrong
stitch. Nothing else in the log can see it coming.

## Tuning

| field | default | what it does |
| --- | --- | --- |
| `curve_corridor_mm` | 30 | how far either side of the pointing the seam is looked for. The budget for everything the pointing gets wrong, and sized by the middle point rather than the ends — that is the one with no landmark to aim at |
| `curve_sample_mm` | 0.25 | how finely the corridor is sampled, along and across. In millimetres, so nothing here is tied to a lens |
| `curve_blur_mm` | 0.6 | smoothing across the corridor. About the width of the feature, so the search does not lock onto one stitch and lose the next |
| `curve_along_mm` | 3.0 | smoothing along the seam. The one step that separates a seam from cloth: a seam is the only thing in the corridor that runs |
| `curve_evidence_sigmas` | 4.0 | how far above the corridor's own noise a feature saturates. Robust, so a coverstitch cannot make the seam beside it look like nothing |
| `curve_min_contrast` | 2.0 | and how much of the picture it must change across itself, in gray levels, whatever the noise is. Without this floor the tails of a strong seam go flat and the pointing places the reading |
| `curve_prior_mm` | 3.0 | how far off the pointing a line must be before the pointing outweighs it. This is what chooses between parallel lines |
| `curve_follow_mm` | 1.2 | how far the seam may wander from the line chosen for it. Wide enough for any garment wobble, too narrow to reach the next line |
| `curve_min_margin` | 0 | how much better the chosen line must be than the best other line. Zero reports the number without refusing on it; set it from your own bench's margins |
| `curve_smoothness` | 0.35 | what a step sideways has to buy while following the chosen line |
| `curve_smooth_mm` | 4.0 | how much of the seam the running fit sees. Stops per-sample noise inflating the arc, which it otherwise does in one direction only |
| `curve_min_confidence` | 0.18 | how much of the corridor's gradient the path must carry to count as a seam. What refuses a corridor with nothing in it |
| `curve_max_drift_mm` | 12 | how far the snap may move off the pointing before it has found something else |
| `curve_min_length_mm` | 40 | shorter than this is two taps in the same place |
| `curve_seed_count` | 3 | how many points before the seam is read. Three describes a curve; two only its ends |
| `curve_measure_along_arc` | true | the fallback when a step does not say. The step should say |
| `finger_border_margin_px` | 3 | how far in the border test reaches. A blob touching no border is not an arm |
| `finger_min_area_fraction` | 0.0015 | the smallest moving thing that could be a hand |
| `finger_chroma_angle_threshold` | 0.008 | how much colour change makes a pixel part of the hand, as 1 − cos of the angle between it and the settled scene. The garment's 0.020 cannot see brown skin on salmon cloth, which differs by five degrees |
| `finger_intensity_ratio_low` | 0.85 | how much darker than the settled scene counts as the hand. The garment's 0.35 was tuned to ignore the cloth's own shadow; skin over cloth is about 0.7. The shadow test (`shadow_uniformity`) is what still separates the two |
| `finger_intensity_ratio_high` | 1.15 | and how much brighter |
| `finger_bridge_mm` | 20 | how wide a gap in the hand mask is closed over. Dark skin over the hem's shadow or a fold is invisible to any colour test, and the wrist crosses exactly there; unbridged, the hand comes loose from its arm and stops being a hand |
| `finger_dwell_seconds` | 0.4 | how long the tip holds still before the point is taken and the trace starts. Whichever of this and `finger_dwell_frames` is longer wins |
| `finger_dwell_frames` | 2 | and how many frames of it. Two is the fewest that can tell a hand that stopped from one passing through; at 1.9 fps that is a second |
| `finger_dwell_radius_mm` | 6.0 | and how still |
| `finger_height_mm` | 10 | how high a pointing fingertip sits above the mat, and so how much parallax to remove. Needs a working distance; pass `--focal-length-mm` |
| `finger_tip_look_mm` | 15 | how much of the hand around its far end counts as the fingertip when measuring how wide that fingertip is, and the window whose centre gives the finger's own direction for the setback. A fraction of the arm's length instead of this measured the fist |
| `finger_tip_setback_max_mm` | 10 | the most the tip may be moved back down the finger, and the width at or under which a part of the arm is thin enough to be a finger. Bounds the damage when there is no fingertip to find |
| `finger_trace_step_mm` | 2.0 | how far the tip must move before another point of the trace is kept |
| `finger_trace_lift_frames` | 2 | how many frames with no hand in them end a trace. A camera looking down cannot see a finger lift, so what ends a trace is the hand leaving; two rather than one because a hand crossing its own shadow drops out for a frame |
| `finger_trace_end_seconds` | 0 | how long the tip may stand still before that ends a trace; zero is never. Off because hold-draw-lift is one wait and hold-draw-hold is two, and a drawing hand pauses constantly |
| `finger_max_speed_mm_s` | 300 | the fastest a hand draws, so the furthest the tip may move between frames before the move is a lost finger rather than a stroke. Scaled by the real interval between the frames |
| `finger_lost_frames` | 2 | how many frames in a row the tip may be somewhere it could not have got to before that place is believed |
| `curve_trace_smooth_mm` | 50 | how much of a hand-drawn trace is fitted, with a running quadratic, before it becomes the corridor's centre. The corridor is built along the trace, so leftover shake is a crooked ruler that only ever reads long: 30 mm read a 254 mm seam as 262, 45 as 254.1 |

And two per step, in the style file rather than the config, because they are about the garment:

| step field | default | what it decides |
| --- | --- | --- |
| `"find"` | `"auto"` | whether a curve step is found by the machine from the outline when the reading holds, falling back to pointing when unsure, or `"point"`, waiting for the operator every time |
| `"seam"` | `"outer"` | which edge of a collar band the step means when the machine finds the band: `outer` is the attachment seam, the larger horseshoe; `inner` is the opening's own edge, a band's width inside it |
| `finger_rearm_seconds` | 0.8 | so one steady finger cannot lay down twenty points in the same place |
