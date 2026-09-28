// Measure the folded edge.
//
// The operator folds the garment so the dimension being checked becomes a
// single straight edge, and presents that edge to a fixed side of the camera.
// The machine looks in from that side, finds the first straight edge it meets,
// follows it to its corners, and reports the length between them. That length
// is the measurement.
//
// Nothing here interprets the garment. It does not need the whole shirt in
// frame, or segmented, or identified; only the one edge has to be in view. That
// is what makes a 490 mm measurement possible on a bench whose field of view is
// smaller than the shirt.

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "verimeasure/calibration.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/measure.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

// The direction the machine looks in, as a unit vector in millimetres, taken
// from the side of the picture named in cfg.scan_from.
bool scanDirection(const RoiBox& roi, const PlaneModel& model, const std::string& from,
                   cv::Point2d& out);

// How long the rail is, in millimetres: the reach of the bench, and the hard
// limit on what it can measure. A target longer than this is not a tolerance
// problem, it is a measurement the rig cannot take.
double railLengthMm(const RoiBox& roi, const PlaneModel& model, const std::string& from);

// The four corners of the working area in millimetres, in order. An edge that
// ends on one of its sides ran off the picture rather than ending there.
std::vector<cv::Point2d> workingAreaMm(const RoiBox& roi, const PlaneModel& model);

struct EdgeResult {
    bool ok = false;
    std::string error;

    // The edge was found and it was straight, but it reached the border of the
    // picture, so what was measured is the view and not the garment.
    bool clipped = false;

    double length_mm = 0.0;  // the measurement: corner to corner along the fold

    // How rough the edge is: the RMS of the boundary about the arc fitted
    // through it. Recorded on every reading whether or not it is bad enough to
    // refuse, because a fold that is drifting is the first sign of a bench
    // going wrong and it is invisible otherwise.
    double straightness_mm = 0.0;

    // How far that arc departs from its own chord, at the worst point.
    //
    // Not an error and not subtracted from anything: a garment edge is curved
    // by design and the measurement is the chord regardless, which is what a
    // tape laid between two seams reads. It is here because it is the number
    // that separates a hem doing what a hem does from a fold that has gone
    // slack, and because a line of them across a shift says whether the folding
    // is holding up.
    double bow_mm = 0.0;

    // How much of length_mm was reached for past the ends of the edge itself,
    // to put the corners where the seams would have met.
    //
    // Recorded because it is a correction rather than a reading, it only ever
    // adds, and a bench where it is quietly adding twenty millimetres to every
    // step is a bench reporting the correction and not the garment. If this is
    // ever a large share of the number, edge_corner_reach_mm is wrong for these
    // garments and wants turning down - to zero, if the folds present square
    // ends of their own.
    double reach_mm = 0.0;

    // The edge along the whole of its length, as it actually lies, in
    // millimetres. This is the thing that was measured, and the thing the feed
    // draws, so what the operator sees is the fabric traced rather than a
    // straight line thrown over the top of it. First and last points are the
    // two corners.
    std::vector<cv::Point2d> path_mm;

    // The length of that trace, following the edge instead of crossing it.
    // Recorded beside length_mm rather than instead of it: which of the two is
    // the measurement is a question about what the tech pack means, and
    // cfg.edge_measure_along_curve is where it is answered.
    double traced_mm = 0.0;

    cv::Point2d a_mm{0.0, 0.0};  // the two corners
    cv::Point2d b_mm{0.0, 0.0};

    vjson::Value toJson() const;
};

// Find and measure the folded edge in an outline already in millimetres.
// `scan` is the direction the machine looks in. `borderMm`, when given, is the
// working area the outline was cut out of, so an edge that ran off the picture
// can be reported as such rather than as a short garment.
bool measureEdge(const std::vector<cv::Point2d>& outlineMm, const cv::Point2d& scan,
                 const Config& cfg, EdgeResult& out,
                 const std::vector<cv::Point2d>* borderMm = nullptr);

// The same, from a finished measurement. The outline goes through the plane
// model first, so the length travels the same pixels-to-millimetres path
// everything else does and cannot carry a scale of its own.
bool measureEdge(const MeasureResult& result, const PlaneModel& model, const cv::Point2d& scan,
                 const Config& cfg, EdgeResult& out, const RoiBox* working = nullptr);

}  // namespace verimeasure
