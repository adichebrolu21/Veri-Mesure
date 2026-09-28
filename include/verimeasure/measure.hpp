// Sub-pixel edge refinement and metric measurement.

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

struct SettledInfo {
    double area_spread_fraction = 0.0;
    double centroid_movement_px = 0.0;
};

struct MeasureResult {
    bool calibrated = false;

    // Calibrated output, millimetres.
    double length_mm = 0.0;
    double breadth_mm = 0.0;
    double area_mm2 = 0.0;
    double perimeter_mm = 0.0;
    cv::Point2d centre_mm{0.0, 0.0};
    double angle_deg = 0.0;
    double parallax_correction = 1.0;
    std::string fit_method;
    std::vector<cv::Point2d> box_mm;

    bool has_edge_fit = false;
    double side_fit_rms_mm = 0.0;
    double opposite_side_delta_mm[2]{0.0, 0.0};
    double corner_squareness_deg = 0.0;
    std::vector<cv::Point2d> corners_mm;

    bool has_saturation = false;
    double saturated_fraction = 0.0;

    // Uncalibrated output, pixels.
    double length_px = 0.0;
    double breadth_px = 0.0;

    // The refined boundary this result was measured from, in full-frame pixels.
    //
    // Carried so a layer above can ask questions of the shape that a length and
    // a breadth cannot answer, which is what garment QC needs: where the collar
    // is, where the armhole is. It is deliberately not serialised. Several
    // hundred sub-pixel points per garment would swamp the results file, and
    // nothing downstream of the log wants them.
    std::vector<cv::Point2d> outline_px;

    // Added when a measurement completes.
    bool finalised = false;
    int object_number = 0;
    int frames_averaged = 0;
    bool has_settled = false;
    SettledInfo settled;
    std::string timestamp;

    // Serialised in the same key order the Python original emitted.
    vjson::Value toJson() const;
};

struct RectFit {
    double length_mm = 0.0;
    double breadth_mm = 0.0;
    std::vector<cv::Point2d> corners_mm;
    double side_fit_rms_mm = 0.0;
    double opposite_side_delta_mm[2]{0.0, 0.0};
    double corner_squareness_deg = 0.0;
};

// Move every boundary point to the sub-pixel peak of the gradient magnitude
// along its own outward normal. This removes the systematic bias a fixed binary
// threshold introduces, where the edge lands wherever the threshold happens to
// cut the intensity ramp, and where a blown-out highlight pushes it outward.
std::vector<cv::Point2d> refineContourSubpixel(const cv::Mat& edgeEvidence,
                                               const std::vector<cv::Point>& contour,
                                               const Config& cfg);

// Fit the four sides and intersect them, instead of taking the minimum-area
// rectangle.
//
// minAreaRect has to enclose every point, so it is an extremum statistic: any
// outward noise on the boundary pushes it out and nothing pushes it back. That
// shows up as a constant positive bias, the same fraction of a millimetre on a
// 60 mm part as on a 300 mm one. Fitting each side to the hundreds of boundary
// points that belong to it averages the noise down instead of chasing its worst
// excursion.
//
// Returns false when the outline is not usefully rectangular, so blobs and
// rounded parts fall back to the enclosing rectangle.
bool fitRectangleMm(const std::vector<cv::Point2d>& pointsMm, const Config& cfg, RectFit& out);

// Convert boundary points to millimetres first, then fit in millimetre space.
// Doing it the other way round, fitting in pixels and scaling afterwards, only
// works if the pixel grid is square and uniform, which it is not.
MeasureResult measureInMm(const std::vector<cv::Point2d>& contourPxFull, const PlaneModel& model,
                          const Config& cfg);

}  // namespace verimeasure
