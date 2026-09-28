// Collar detection: the operator draws a box, the picture does the rest.
//
// A curve step (curve.hpp) takes the operator's pointing as the start of the
// reading: the snap corrects it ACROSS the seam and cannot correct it ALONG
// the seam, so the two ends of a neckline are wherever a finger or a tap put
// them, and the choice between two lines a few millimetres apart is the
// pointing's too. Those are the two places a person is still inside that
// measurement.
//
// This step takes the person out of both.
//
//     box  ->  the collar's lines, from the picture  ->  each run to the edge
//          of the cloth  ->  the step's rule picks the edge  ->  measure
//
// The box is the whole of the operator's contribution, and it is a region, not
// a position. It says "the collar is in here" and nothing else. Every number
// the reading carries is taken from the picture:
//
//   WHERE the line lies  - snapCurve, sub-sample, exactly as a curve step; an
//                          attachment seam then put in the bottom of its
//                          groove, where the stitching is.
//   WHICH line           - the collar band is found stretch by stretch round
//                          the collar as two crisp lines a band's width apart
//                          with plain cloth between them, and the style
//                          file's "seam" names one of its two edges.
//   WHERE it ends        - where that line meets the edge of the cloth. A
//                          collar's seam runs from the shoulder, round the
//                          front and back up to the other shoulder, and on a
//                          shirt laid flat both of its ends reach the
//                          garment's silhouette. That meeting point IS the
//                          shoulder point, and it is found on the picture
//                          rather than aimed at.
//   THE BACK             - for a step that measures the whole collar
//                          ("around"), the back of the neck: the edge of the
//                          shirt between the two shoulder points, followed on
//                          the picture the way the shoulder points are found.
//
// So the property to hold is the one a measurement needs: two operators
// drawing two different boxes around the same collar get the same number.
// collar_check --selftest draws the box a dozen different ways and requires
// exactly that.
//
// What the box still does is refuse. A line that runs out of the box means the
// box cut the collar, and a reading taken from a cut collar would depend on
// where the cut was - so it is refused rather than measured, and the operator
// draws the box again.

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/curve.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

// One line the picture offered inside the box, run from the edge of the cloth
// on one side to the edge of the cloth on the other.
struct CollarLine {
    std::vector<cv::Point2d> path_mm;  // edge of the shirt to edge of the shirt
    double arc_mm = 0.0;               // along it
    double chord_mm = 0.0;             // straight between its two ends
    double depth_mm = 0.0;             // how far it dips from that chord: the neck drop
    double radius_mm = 0.0;            // best-fit circle, zero when too straight to mean one

    // From snapCurve: how much of the corridor's gradient the line carries,
    // and how much better it was than the best other line beside it.
    double confidence = 0.0;
    double margin = 0.0;
    double roughness_mm = 0.0;

    // What kind of line it is across itself. One is a change of shade - the
    // edge of a piece of cloth, where the band ends and the body or the
    // opening begins. Zero is a stitch line: a thin ridge with the same cloth
    // either side. A band's edges are the former; a topstitch running beside
    // the attachment seam is the latter, and telling them apart is what stops
    // the topstitch being measured as the seam.
    double step_share = 0.0;

    // How far outside the first line found it runs, on average, in
    // millimetres. Positive is away from the neck opening. Orders the lines
    // from the opening outward, which is the order a band is built in.
    double offset_mm = 0.0;

    bool meets_edge = false;  // both ends reach the edge of the cloth
    bool inside_box = false;  // the whole of it lies inside the box

    // For a step that measures the whole collar: the back of the neck, along
    // the edge of the shirt from this line's last point to its first, and its
    // length. Read with the line, so a line taken with a tap brings its own.
    std::vector<cv::Point2d> back_path_mm;
    double back_mm = 0.0;
    // What the reading made of it: "inner" or "outer" for the band's two
    // edges; empty for a line offered only because no band could be found.
    std::string role;
};

// What the step asks for, out of the style file.
struct CollarRequest {
    // Which edge of the band: "outer", the attachment seam where the band
    // meets the body, or "inner", the neck opening's own edge.
    std::string edge = "outer";
    // Along the edge, or straight between its two ends.
    bool along_arc = true;
    // The whole collar ("measure": "around"): the band's outer edge over the
    // front from one shoulder point to the other, and on from there along the
    // edge of the shirt across the back of the neck to where it started. The
    // back of a round neck laid flat IS the edge of the shirt between the
    // shoulder points, so that is where it is read. Only on the outer edge.
    bool around = false;
    // The tech pack's number. A GATE on plausibility and nothing else: a line
    // more than a quarter away from it is not the line the tech pack
    // describes. Never a preference among what is left - choosing whichever
    // line is nearest the target would bias every garment toward PASS.
    double target_mm = 0.0;
};

struct CollarResult {
    bool ok = false;
    std::string error;
    // Why a reading was not taken, in the terms of what to do about it:
    //   "box"    - the box: it cut the collar, or holds no collar. Draw it again.
    //   "choose" - no band could be found, but lines were: they are drawn, and
    //              a tap on one takes it, every number still its own.
    //   "collar" - nothing in the box could be read as a collar.
    std::string refusal;

    cv::Rect2d box_px;
    std::vector<cv::Point2d> box_mm;  // the box's four corners on the mat

    // The band's two edges as read, the opening's first - or, when no band
    // could be found, the lines that were, for a tap to choose between.
    std::vector<CollarLine> lines;
    int chosen = -1;
    std::string how;  // how the line was chosen, or why it could not be

    std::string edge = "outer";
    bool along_arc = true;
    bool around = false;

    // The reading: the chosen line's.
    double arc_mm = 0.0;
    double chord_mm = 0.0;
    double depth_mm = 0.0;
    cv::Point2d end_mm[2];  // where it meets the edge of the cloth: the shoulder points

    // With `around`: the back of the neck, along the edge of the shirt from
    // the second shoulder point to the first, so the chosen line and this one
    // close into the whole collar. Empty otherwise.
    std::vector<cv::Point2d> back_path_mm;
    double back_mm = 0.0;

    double measured_mm() const { return around ? arc_mm + back_mm : along_arc ? arc_mm : chord_mm; }

    // The box's first guesses at where the collar lies, kept for drawing when
    // a reading has to be looked into. They are where the search started and
    // no part of the reading.
    std::vector<std::vector<cv::Point2d>> guesses_mm;
    // ...and the line each one was snapped to, refused or not.
    std::vector<std::vector<cv::Point2d>> first_mm;
    // What each stage found, one line each: for the log when a reading is
    // refused, and for collar_check. Not part of the reading.
    std::vector<std::string> notes;

    // The chosen line as a curve reading, so the verdict is taken by exactly
    // the arithmetic a curve step is judged by (judgeCollar in qc.hpp).
    CurveResult asCurve() const;
    vjson::Value toJson() const;
};

// Find the collar inside `boxPx` and read the edge the request names.
//
// `frame` is the held picture, full resolution. `outlineMm` is the garment's
// boundary on the mat when one is known - the station passes the refined
// outline its reading was taken from - and may be empty, in which case the
// cloth is told from the mat by brightness inside the box. Either way it is
// only a guide: where each line actually meets the edge of the cloth is found
// on the picture itself, because an outline pushed a few millimetres out by
// glow off the cloth would otherwise lengthen the reading by the same amount
// at both ends.
bool detectCollar(const cv::Mat& frame, const PlaneModel& model,
                  const std::vector<cv::Point2d>& outlineMm, const cv::Rect2d& boxPx,
                  const CollarRequest& req, const Config& cfg, CollarResult& out);

// Take the line nearest a tap, when the rule could not choose.
//
// The tap chooses WHICH of the lines already read is meant and nothing more:
// every number is still the line's own, from the picture, the same whoever
// tapped and wherever near it they tapped. Returns false when no line lies
// within `reachMm` of the tap.
bool pickCollarLine(CollarResult& r, const cv::Point2d& tapMm, double reachMm = 8.0);

}  // namespace verimeasure
