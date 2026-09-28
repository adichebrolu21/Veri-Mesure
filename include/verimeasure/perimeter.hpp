// The whole outer edge of a garment laid flat, taken in two halves.
//
// A shirt is bigger than the view, so its perimeter cannot be read from one
// picture. The operator clips the middle of the back neck and the middle of the
// hem, lays one half on the mat with the clips in a band along the bottom edge
// of the view and the other half flat past it, and the station reads that half:
// the outline from one clip, round the half, to the other. Then the shirt is
// lifted, turned end for end, and the other half is laid the same way. The two
// readings add up to the perimeter.
//
// The clips are what make the join exact. A clip travels with the cloth, so it
// marks the same point in both pictures however the shirt was laid, and the
// station cuts both outlines there. Cutting at a line on the feed instead would
// put every millimetre of misplacement on the total twice, once at the neck
// and once at the hem, and nothing on a hem marks its middle.
//
// What a half measures is what a pattern draws: smooth seam lines meeting at
// sharp corners. The cloth rounds every corner, by tens of millimetres at an
// underarm, so each edge is fitted and neighbouring edges are carried on until
// they cross - the way the folded-edge steps find a seam corner. The length
// along the cloth itself, measured every perimeter_trace_step_mm, is recorded
// beside it as the traced length.
//
// Nothing here is used by any other kind of step, and nothing in the engine,
// the segmentation or the calibration changed for it: a half is measured from
// the refined outline the engine already produces for every reading.

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

// The sides of the working area, in the order the guide lists them.
enum class PerimeterSide { Left = 0, Top = 1, Right = 2, Bottom = 3 };

bool parsePerimeterSide(const std::string& name, PerimeterSide& side);
const char* perimeterSideName(PerimeterSide side);

// The colour of the operator's clips.
struct ClipColour {
    double hue_lo_deg = 40.0;  // 0-360; a low bound above the high one wraps through red
    double hue_hi_deg = 70.0;
    int sat_min = 110;         // 0-255
    int val_min = 90;          // 0-255
};

// The bench's clip colour, from the configuration.
ClipColour clipColourFrom(const Config& cfg);

// A clip, as found on the picture.
struct PerimeterClip {
    cv::Point2d px{0.0, 0.0};           // centre, full-resolution pixels of the raw picture
    cv::Rect box_px;                    // where it lies, the same pixels
    std::vector<cv::Point> contour_px;  // its outline, the same pixels
    double area_mm2 = 0.0;
};

// Every clip-coloured blob of a clip's size inside the working area.
//
// Found on a decimated copy, like the engine's live pass. With `refine` each
// one is then outlined again at full resolution, so its centre is good to a
// small fraction of a millimetre: that centre is where the halves are joined.
std::vector<PerimeterClip> findClips(const cv::Mat& frameBgr, const RoiBox& roi,
                                     const PlaneModel& model, const ClipColour& colour,
                                     const Config& cfg, bool refine);

// A corner of a half's outline, and what was done at it.
struct PerimeterCorner {
    cv::Point2d at_mm{0.0, 0.0};  // the crossing when crossed, else where the cloth turns
    double turn_deg = 0.0;         // how far the outline turns there
    bool crossed = false;          // the two edges were carried on until they met
    double reach_mm = 0.0;         // what crossing added over following the cloth
};

// One half: the outline from one clip, round the garment, to the other.
struct PerimeterHalf {
    double fitted_mm = 0.0;  // edges fitted, corners crossed: what a pattern draws
    double traced_mm = 0.0;  // the cloth followed, every perimeter_trace_step_mm
    double reach_mm = 0.0;   // how much of fitted_mm crossing the corners added
    int corners = 0;         // corners found
    int crossed = 0;         // corners crossed
    // Notches and spikes bridged straight across rather than followed: a
    // thread, a label, a printed line of the mat the outline was pulled along.
    int defects = 0;
    std::vector<PerimeterCorner> corner_list;

    // What was measured, as a path: smoothed, with each crossed corner drawn to
    // its crossing. The first and last points are where the clips cut it.
    std::vector<cv::Point2d> path_mm;
    std::vector<cv::Point2d> path_px;  // the same on the raw picture, for the feed
    std::vector<cv::Point2d> corner_px;  // the corners crossed, on the raw picture
    cv::Point2d cut_px[2];             // the two cuts on the raw picture, along the join side

    // Where along the line between the two cuts the point furthest from it
    // lies, from 0 at the first cut to 1 at the second. On a T-shirt that is
    // the sleeve, which sits near the collar end, so it says which end of the
    // band the collar was at: the check that the shirt really was turned.
    double far_u = 0.5;

    vjson::Value toJson() const;
};

// What one picture held.
struct PerimeterCapture {
    bool ok = false;
    std::string error;
    // The whole garment was in view, clear of every edge: both halves were
    // read from this one picture and the step needs no second.
    bool whole = false;
    int halves = 0;  // 1, or 2 when whole
    PerimeterHalf half[2];
    std::vector<PerimeterClip> clips;  // the two on the edge of the shirt
    vjson::Value toJson() const;
};

// Read one picture: normally one half, cut at the two clips.
//
// `outlinePx` is the engine's refined boundary as a MeasureResult carries it:
// full-resolution pixels, with the fold-height correction already applied as
// a scaling by `correction` about `centre`. The topology - where the outline
// meets the border, which parts belong to which clip - is worked out on the
// picture as it is, and the lengths on the corrected outline, so the height
// correction reaches every millimetre exactly as it does on an edge step.
bool measurePerimeterCapture(const std::vector<cv::Point2d>& outlinePx, double correction,
                             const cv::Point2d& centre, const cv::Mat& frameBgr,
                             const PlaneModel& model, const RoiBox& roi,
                             const ClipColour& colour, const Config& cfg, PerimeterCapture& out);

// The same, from a finished reading.
bool measurePerimeterCapture(const MeasureResult& result, const cv::Mat& frameBgr,
                             const PlaneModel& model, const RoiBox& roi,
                             const ClipColour& colour, const Config& cfg, PerimeterCapture& out);

// Measure a half already cut out, in millimetres: the fitted and traced
// lengths, the corners and the path. Exposed for the harness.
void measurePerimeterPath(const std::vector<cv::Point2d>& pathMm, const Config& cfg,
                          PerimeterHalf& out);

// Whether the second half was laid the other way round from the first.
//
// Lifting the shirt and laying it down again without turning it puts the same
// half in view twice, and the total comes out as twice one half: a confident
// wrong number. The far point of a half - the sleeve, on a T-shirt - sits near
// the collar end, so turning the shirt end for end moves it to the other end
// of the band. Returns false, with the reason, only when both halves say
// clearly that the collar was at the same end; a garment whose far point says
// nothing either way is let through.
bool turnedEndForEnd(const PerimeterHalf& first, const PerimeterHalf& second, std::string& why);

// The two halves, joined.
struct PerimeterResult {
    bool ok = false;
    std::string error;
    bool traced = false;  // judged on the traced length instead of the fitted one
    bool whole = false;   // both halves came from one picture
    PerimeterHalf half[2];
    double fitted_mm = 0.0;
    double traced_mm = 0.0;
    double reach_mm = 0.0;
    double difference_mm = 0.0;  // first half minus second, on the judged length
    int corners = 0;
    int crossed = 0;

    double measured_mm() const { return traced ? traced_mm : fitted_mm; }
    double halfMm(int k) const { return traced ? half[k].traced_mm : half[k].fitted_mm; }
    vjson::Value toJson() const;
};

PerimeterResult joinPerimeter(const PerimeterHalf& first, const PerimeterHalf& second,
                              bool traced, bool whole);

// -- the guide on the live feed --

// Where the band and the margins are, on the raw picture, for drawing.
struct PerimeterGuideLines {
    PerimeterSide join = PerimeterSide::Bottom;
    std::vector<cv::Point2d> band_near;  // the band's edge nearest the join side
    std::vector<cv::Point2d> band_far;   // and its far edge
    std::vector<cv::Point2d> margin[4];  // the keep-clear line on each other side; empty on the join side
};

PerimeterGuideLines perimeterGuideLines(const PlaneModel& model, const RoiBox& roi,
                                        const Config& cfg);

// How the lay looks right now, from the live outline, for the operator.
struct PerimeterGuide {
    bool garment = false;           // something is on the mat
    bool margin[4] = {false, false, false, false};  // cloth inside that side's margin
    // Where along each side the cloth is inside its margin, as two points on
    // the margin line, for drawing that stretch red. Empty when clear.
    std::vector<cv::Point2d> touch[4];
    bool reaches_join = false;      // the cloth runs past the join side
    std::vector<PerimeterClip> clip_list;  // clips on the edge of the cloth
    std::vector<bool> in_band;      // one per clip
    int clips_in_band = 0;
    bool ok = false;                // nothing for the operator to put right
    std::string message;            // what to do next, in words
};

// `liveOutlinePx` is the engine's live contour mapped back to full-resolution
// pixels of the raw picture.
void evaluatePerimeterGuide(const std::vector<cv::Point2d>& liveOutlinePx,
                            const cv::Mat& frameBgr, const PlaneModel& model, const RoiBox& roi,
                            const ClipColour& colour, const Config& cfg, PerimeterGuide& out);

}  // namespace verimeasure
