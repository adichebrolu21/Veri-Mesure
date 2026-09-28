// Measure a seam the operator pointed at.
//
// Everything in edge.hpp measures the outline of the cloth: a fold presented to
// the rail, found by differencing the picture against the empty mat. That
// covers a hem, a shoulder, a chest, a sleeve - every dimension that can be
// folded into a silhouette.
//
// A neckline cannot. It is a seam in the middle of the garment: the collar
// meets the body somewhere inside the shirt, the neck opening shows the back
// panel rather than the mat, and a background difference sees nothing there at
// any resolution. Neither does a collar seam, a placket, or an armhole left
// unfolded. For those the machine has to be told which curve is wanted, and
// there is nothing in the picture that can tell it.
//
// So the operator points. Two or three taps, or a finger drawn along the seam.
//
//     seeds (+/- 10 mm)  ->  corridor  ->  snap to the seam  ->  measure
//
// The pointing is NOT the measurement, and keeping those two apart is the whole
// of this file.
//
// A fingertip sits about ten millimetres above the mat while the plane model is
// a homography onto the mat, so at the edge of this frame that height alone
// displaces it four millimetres - and the displacement grows from the centre
// outward, so a straight trace comes back bent. The tip itself is ambiguous by
// most of a centimetre under an occluding nail. A hand shakes. A finger dragged
// along ribbed jersey stretches the very thing being measured. Report the
// traced path as the answer and the number describes the operator.
//
// What the taps actually do is name a corridor. Inside it the seam is found
// from the picture, sub-sample, by the same kind of gradient search that
// refineContourSubpixel uses on an outline. The pointing has to be good to a
// centimetre; the answer is good to a fraction of a millimetre. Two operators
// pointing differently get the same number, which is the property that makes
// this a measurement at all.
//
// The corridor is rectified before anything looks at it: sampled into a
// straight ribbon in millimetres, so many along the seed path by so many
// across. Every parameter is then a length on the mat rather than a count of
// pixels, and the search itself becomes a choice of one column per row, which
// cannot loop, cannot double back, and cannot leave the corridor.

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

struct CurveResult {
    bool ok = false;
    std::string error;

    // The two measurements, both always filled, because which one the tech pack
    // means is a question about the garment and not about the machine.
    //
    // On a fold these differ by about a millimetre in five hundred and the
    // choice hardly matters. On a neckline the arc runs ten percent longer than
    // the chord across it, so the choice is the difference between a pass and a
    // reject. "Neck width" is the chord between two shoulder points; "armhole
    // curved" and a neckline seam length are the arc. The style file says
    // which, per step.
    double chord_mm = 0.0;  // straight between the two ends
    double arc_mm = 0.0;    // along the seam

    // How far the seam departs from its own chord, at the worst point. On a
    // neckline this is most of the shape rather than an error, and it is what
    // separates a collar that has been laid flat from one still standing up.
    double bow_mm = 0.0;
    // The radius of the circle the seam best fits, or zero when it is straight
    // enough that no circle means anything. A neckline is very nearly circular,
    // so this is a cheap check that the machine followed a neckline and not the
    // edge of a print.
    double radius_mm = 0.0;
    // What the running fit removed: the RMS of the snapped samples about the
    // smoothed path. This is the cloth and the picture - stitching, texture,
    // a boundary that will not settle - and it is recorded on every reading
    // whether or not it is bad enough to matter, for the same reason
    // straightness_mm is.
    double roughness_mm = 0.0;

    // How much of the corridor's own gradient the chosen path actually carries,
    // from zero to one.
    //
    // The number that decides whether there was a seam there at all. A corridor
    // with nothing in it still returns a best path: smooth, plausible, drawn on
    // the operator's screen, and invented. On a white rib against a white body
    // this is what will refuse the reading, and refusing is correct - a collar
    // that cannot be seen has not been measured.
    double confidence = 0.0;
    // How far the snapped seam ended up from the path that was pointed at, at
    // the worst point.
    //
    // The honesty number for this layer, the way reach_mm is for the folded
    // edge. Small is the system working: the operator pointed roughly and the
    // machine corrected them. Large means the machine did not correct the
    // pointing, it found something else - the shoulder seam, the edge of a
    // print, a fold in the mat - and is now measuring one thing while the
    // operator watches another.
    double drift_mm = 0.0;
    // How much better the line that was measured was than the best line
    // somewhere else in the corridor.
    //
    // The number that says whether the pointing actually chose anything. A
    // neckline can carry a coverstitch two millimetres outside the seam it
    // holds down, and a tap worth three millimetres cannot tell those apart -
    // not because the search is poor but because the information is not there.
    // Near zero means the machine picked one of two equally good answers by
    // accident, and the honest response is to ask for the point again rather
    // than to measure the topstitch on one shirt and the seam on the next.
    double margin = 0.0;

    // How much of the corridor lay outside the picture. A seam whose corridor
    // runs off the frame has not been read, it has been cropped.
    double clipped_fraction = 0.0;

    std::vector<cv::Point2d> path_mm;   // the seam as found, end to end
    std::vector<cv::Point2d> seeds_mm;  // what the operator actually pointed at

    // Which of the two lengths this reading reports, per the step.
    bool along_arc = true;
    double measured_mm() const { return along_arc ? arc_mm : chord_mm; }

    vjson::Value toJson() const;
};

// The path the seeds describe, before any snapping: what the corridor is built
// around.
//
// Two seeds give the straight line between them. Three give the circular arc
// through them, which matters more than it sounds - a neckline dips seventy to
// ninety millimetres below the chord joining its two shoulder points, so a
// corridor built around that chord contains no neckline at all, and three taps
// are the fewest that can describe the curve rather than its ends. Four or more
// are taken as a trace and followed as they lie.
//
// `extendMm` carries the path on past both end seeds, along the curve it is
// already following. Scaffolding rather than measurement: the seam is snapped
// over the longer run and the reading is then trimmed back to the ends, which
// is what lets an end land where the operator pointed rather than where the
// corridor happened to stop.
//
// `traceSmoothMm` is how much of a traced path is fitted before it is used,
// which matters only for four seeds or more - the hand that drew them is
// removed over that span and the curve is not. Ignored for two or three, where
// there is no hand in the path to remove. snapCurve passes the style file's
// curve_trace_smooth_mm; see that setting for what the length is worth.
//
// Returns false when the seeds cannot describe a path: fewer than two, or all
// of them in the same place.
bool seedPath(const std::vector<cv::Point2d>& seedsMm, double sampleMm,
              std::vector<cv::Point2d>& pathMm, double extendMm = 0.0,
              double traceSmoothMm = 50.0);

// Find the seam inside the corridor around the seeds, and measure it.
//
// `frame` is the picture the reading is taken from, full resolution, colour or
// gray. Not the foreground score the segmentation produces: that is evidence
// about where the cloth is against the mat, and a seam is inside the cloth
// where the score is flat.
bool snapCurve(const cv::Mat& frame, const PlaneModel& model,
               const std::vector<cv::Point2d>& seedsMm, const Config& cfg, CurveResult& out);

// A path the operator drew on the picture, measured as it was drawn.
//
// The tape-measure step, as the two-point ruler is the straight-edge step: the
// operator holds the mouse down and follows a curve, and the reading is the
// length along what they drew. Nothing is looked for in the picture and nothing
// is snapped - the number is the drawn path, through the calibration, and its
// accuracy is the operator's hand on the mouse.
//
// What IS done to it is the one thing that has to be. A path drawn by hand is
// jittery at the scale of a screen pixel, and jitter never averages out of an
// arc length: every wobble lengthens the segment it sits on, and a path with a
// millimetre of it every few millimetres reads several percent long, always
// long, never short. So the path is fitted with the same running quadratic the
// corridor centre uses on a hand-drawn trace, over the same window, which takes
// the jitter and leaves the curve - see curve_trace_smooth_mm for what that
// costs on a corner, which is the only thing it costs.
struct PathResult {
    bool ok = false;
    std::string error;
    double chord_mm = 0.0;  // straight between the ends
    double arc_mm = 0.0;    // along the smoothed path
    double bow_mm = 0.0;    // how far the path departs from its own chord, at most
    std::vector<cv::Point2d> path_mm;   // the path as measured: smoothed and resampled
    std::vector<cv::Point2d> drawn_mm;  // what was drawn
};
bool measurePath(const std::vector<cv::Point2d>& drawnMm, const Config& cfg, PathResult& out);

// -- finding the neckline without being pointed at --
//
// Everything above takes the pointing as given and works out from there. The
// pointing is also where the reading's remaining error lives: a seam snapped
// from the picture is repeatable to a fraction of a millimetre across its
// length, and its two ENDS are wherever the operator put them, to a few
// millimetres each, along the seam where nothing in the picture can correct
// them. So the biggest thing left to take out of the operator's hands is the
// ends - and the picture has them.
//
// A shirt laid flat has its neck as a notch in its own outline: a concavity
// between two convex corners, and the corners are the shoulder points, which
// is the landmark the tech pack measures a neckline between. The outline is
// already found, refined to a fraction of a pixel, for every held reading. Its
// notch gives the two ends; the tech pack's own length gives how deep the seam
// must dip between them; and snapCurve, handed those as if they had been
// pointed at, does what it always does. On a good garment nobody touches the
// picture. When any of it is unsure, the step falls back to pointing, exactly
// as before.
//
// The target length is used to SIZE the corridor and to check the answer, and
// for nothing else. It must never choose between lines: picking whichever
// line's length is nearest the target would bias every garment toward PASS,
// which is the one thing a QC bench must not do. Which line is the seam stays
// with the picture and the margin guard.
struct NecklineFind {
    bool ok = false;
    std::string error;
    cv::Point2d shoulder_mm[2];         // where the neckline meets the shoulder line, on the outline
    double chord_mm = 0.0;              // between them
    double silhouette_depth_mm = 0.0;   // how far the outline dips between them
    double expected_depth_mm = 0.0;     // how far the seam ought to dip, from the target; zero without one
    std::vector<cv::Point2d> seeds_mm;  // what to hand snapCurve, as if pointed at
    std::string how;                    // "outline" - the notch itself; "arc" - through the expected depth

    // Every corridor worth trying, most likely first; seeds_mm is the first.
    //
    // The target says roughly how deep the seam dips, and roughly is the
    // word: the tech pack's number is the garment as designed, the chord is
    // between two corners of an outline, and the seam on the bench can dip
    // twenty millimetres more or less than the arithmetic says. That is more
    // than the search's rigid model can bow, so one corridor at one depth
    // finds nothing and refuses, correctly. Several corridors at depths around
    // the expected one, each tried in turn, let the PICTURE say which depth -
    // by how much seam each one finds, never by which length is nearest the
    // target.
    std::vector<std::vector<cv::Point2d>> candidates;
};

// `outlineMm` is the garment's boundary, closed, in millimetres. `towardMark`
// points from the garment to the side of the bench the neck was laid toward,
// which is the one thing the machine cannot tell from the shape. `targetArcMm`
// is the tech pack's length along the seam, or zero when the step measures the
// chord or has no target, in which case the notch itself is the corridor.
bool findNeckline(const std::vector<cv::Point2d>& outlineMm, const cv::Point2d& towardMark,
                  double targetArcMm, NecklineFind& out);

// One seam the picture offered between the corners, out of several.
//
// A neck is not one line. Between the shoulder corners of a flat shirt there
// are typically three or four - the back neck band's edge, the collar's
// attachment seam, the opening's own edge fifteen millimetres inside it - and
// every one of them is a real, strong, smooth curve that a corridor at the
// right depth reads with full confidence. Nothing in the picture says which
// the tech pack means. So the picture is asked for ALL of them, and the
// choosing is done afterwards, by a rule the step states.
struct SeamCandidate {
    CurveResult curve;
    double depth_mm = 0.0;              // how far it dips from the corners' chord
    std::vector<cv::Point2d> seeds_mm;  // the corridor it was found in
};

// Snap every corridor the finder offered and keep each distinct seam that
// read, shallowest first. Two corridors landing on the same line count once.
std::vector<SeamCandidate> findSeams(const cv::Mat& frame, const PlaneModel& model,
                                     const NecklineFind& find, const Config& cfg);

// Which of them is the step's seam.
//
// `targetMm` gates: a seam more than a quarter away from the tech pack's
// number is not the one the tech pack is describing, and is dropped. That is
// the only use the target has here - a gate on plausibility, never a
// preference among what is left. `prefer` is the step's convention for a
// collar band, whose two edges are a band's width apart and equally strong:
// "outer" is the attachment seam, the larger horseshoe, which dips DEEPER
// from the chord across the mouth; "inner" is the opening's own edge, the
// shallower. One seam left is the answer. Two a band's width apart are the
// band, and the convention picks. Anything else is not a choice the machine
// gets to make, and `why` says what it found so the operator can point.
bool chooseSeam(const std::vector<SeamCandidate>& seams, double targetMm, bool alongArc,
                const std::string& prefer, std::size_t& chosen, std::string& why);

// Radius of the circle that best fits a run of points, by Taubin's method.
// Returns false when the points are too straight for a circle to mean anything,
// which is the usual answer on anything but a neckline or a collar.
bool fitCircleRadius(const std::vector<cv::Point2d>& pts, double& radiusMm,
                     cv::Point2d& centreMm);

}  // namespace verimeasure
