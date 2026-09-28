// Quality control: one step, one edge, one verdict.
//
// This layer sits on top of the measurement core and changes nothing in it. It
// takes an edge length already in millimetres and a step already loaded, and
// says pass or fail.
//
// One rule matters and it is deliberate. An edge that could not be read is a
// FAIL, not a skipped step. Silently letting the operator move on would let a
// garment through on the strength of the steps that happened to work, which is
// the one failure mode a QC station must not have. If the fold was bad the
// operator re-folds and measures again; the retake is a button, not a loophole.

#pragma once

#include <string>

#include "verimeasure/collar.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/curve.hpp"
#include "verimeasure/edge.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/measure.hpp"
#include "verimeasure/perimeter.hpp"
#include "verimeasure/plane_model.hpp"
#include "verimeasure/template.hpp"

namespace verimeasure {

struct StepResult {
    // What was asked.
    std::string style;
    std::string step;
    int index = 0;  // zero based; the display adds one
    int total = 0;
    double target_mm = 0.0;
    double tol_minus_mm = 0.0;
    double tol_plus_mm = 0.0;

    // What happened.
    bool measured = false;  // false when the edge could not be read at all
    double measured_mm = 0.0;
    double deviation_mm = 0.0;  // measured minus target, signed
    bool pass = false;
    std::string reason;  // empty on a pass; why it failed otherwise

    // How straight the fold was, and how far it curved. Recorded on every
    // reading, not just the bad ones: a fold that is drifting is the first sign
    // of a bench going wrong, and it is invisible unless somebody writes it
    // down. The curvature is the garment's own and is not an error, but a shift
    // where it climbs is a shift where the folding is going.
    double straightness_mm = 0.0;
    double bow_mm = 0.0;
    double reach_mm = 0.0;
    double traced_mm = 0.0;

    // -- a curve step --
    //
    // "edge" or "curve", so a line of the audit log says which kind of reading
    // it was without having to look the style file up. The two are judged the
    // same way against the same tolerances, and they are not the same
    // measurement: one is a property of the cloth alone, and the other has an
    // operator's two taps in it. A log that could not tell them apart could not
    // answer the first question anybody asks of a disputed reading.
    std::string kind = "edge";
    double chord_mm = 0.0;  // across the seam
    double arc_mm = 0.0;    // along it
    // A collar measured all the way round: the back of the neck, from one
    // shoulder point to the other along the edge of the shirt. The reading is
    // arc_mm and this added together. Zero on every other step.
    double back_mm = 0.0;
    double radius_mm = 0.0;
    // How much of the corridor's gradient the seam that was measured carried,
    // and how far the machine had to move off the operator's pointing to find
    // it. On the reading whether it passed or failed, because a bench where the
    // confidence is sliding is a bench whose collars are about to stop being
    // readable, and nothing else would show it.
    double confidence = 0.0;
    double drift_mm = 0.0;
    // How much better the line that was measured was than the best other line
    // in the corridor. Near zero means the pointing chose nothing and the
    // machine would have picked one of two answers by accident; the reading is
    // refused at that point, so a small value on a PASS is the warning that the
    // bench is close to that edge.
    double margin = 0.0;

    // -- a perimeter step --
    //
    // The two halves the total was added up from, on the length that was
    // judged, and the other of the two lengths for the whole: `traced_mm` is
    // the cloth followed and `fitted_mm` the pattern's, whichever the step
    // judged. `reach_mm` is what crossing the corners added.
    double first_half_mm = 0.0;
    double second_half_mm = 0.0;
    double fitted_mm = 0.0;

    int garment = 0;  // which shirt this belongs to, within the run
    std::string timestamp;

    double low_mm() const { return target_mm + tol_minus_mm; }
    double high_mm() const { return target_mm + tol_plus_mm; }

    // The same limits and the same deviation, as a share of what was asked for.
    //
    // Tolerances are authored in millimetres, because that is how a tech pack
    // writes them and turning them into percentages in the file would throw
    // away the lopsided, absolute rule the pack actually states. But a bare
    // "+4.1 mm" tells an operator nothing on its own: it means one thing on a
    // 60 mm shoulder and another on a 490 mm chest. So the status is reported
    // as a percentage of the target, where the number carries its own scale,
    // while the file and the audit log keep the millimetres they were given.
    //
    // A step with no target has no percentage to speak of. The caller checks
    // has_percent() and falls back to millimetres rather than being handed a
    // divide by zero dressed up as a reading.
    bool has_percent() const { return target_mm > 1e-9; }
    double deviation_pct() const { return has_percent() ? deviation_mm / target_mm * 100.0 : 0.0; }
    double tol_minus_pct() const { return has_percent() ? tol_minus_mm / target_mm * 100.0 : 0.0; }
    double tol_plus_pct() const { return has_percent() ? tol_plus_mm / target_mm * 100.0 : 0.0; }

    vjson::Value toJson() const;

    // One line for the operator display and the console.
    std::string headline() const;
};

// Judge an edge that has already been measured.
StepResult judgeStep(const Template& t, int index, const EdgeResult& edge);

// The same, for a seam the operator pointed at.
//
// Which of the curve's two lengths is judged comes from the step: "arc" along
// the seam, "chord" across it. That choice is the step's to make and not this
// layer's, because on a neckline the two differ by forty percent and picking
// the wrong one fails every good garment in the run.
//
// A curve that could not be read is a FAIL, exactly as an unreadable edge is,
// and for the same reason: a garment that passed on the strength of the steps
// that happened to work has not passed. Pointing again is the Retake button.
StepResult judgeCurve(const Template& t, int index, const CurveResult& curve);

// The same, for a collar found inside a box the operator drew.
//
// Judged by judgeCurve itself, on the line the collar step measured, so the
// two can never come to disagree about what a tolerance means; the record is
// marked "collar", because a line of the audit log has to say whether a
// person chose the seam or the picture did. A collar that could not be read
// is a FAIL, as a seam is.
StepResult judgeCollar(const Template& t, int index, const CollarResult& collar);

// Judge a distance step: the straight line between two points the operator
// put down on the picture, in millimetres on the mat.
//
// Nothing is looked for in the picture and nothing is corrected - the reading
// IS the two points - so the only thing that can refuse it is two taps on the
// same place, which is not a distance. Judged against the step's tolerances by
// the same rule as an edge and a curve, and logged as kind "distance" so a
// line of the audit trail says it was the operator's ruler and not the
// machine's.
StepResult judgeDistance(const Template& t, int index, const cv::Point2d& aMm,
                         const cv::Point2d& bMm);

// Judge a distance step taken as a drawn path: the operator held the mouse down
// and followed a curve. Measured along the path as drawn (smoothed, see
// measurePath) unless the step says "chord", in which case straight between
// its ends. Logged as kind "distance" like the two-point form, with both
// lengths on the line.
StepResult judgePath(const Template& t, int index, const PathResult& path);

// Judge a perimeter step: the two halves, joined at the clips. Which length is
// judged is the step's "measure", "fitted" unless it says "traced". A
// perimeter that could not be put together is a FAIL, as an unreadable edge
// is; a half the operator has to lay again is not filed at all, because it is
// about the lay and not the garment.
StepResult judgePerimeter(const Template& t, int index, const PerimeterResult& perimeter);

// The whole judgement, from a finished measurement to a verdict.
//
// This is the hook the capture flow calls once a measurement has finalised. It
// runs on the result the engine already produced and measures nothing of its
// own, so nothing here can disagree with what the station reported.
StepResult runStepQc(const MeasureResult& result, const PlaneModel* model,
                     const cv::Point2d& scan, const Template& t, int index, const Config& cfg,
                     const RoiBox* working = nullptr);

// Append one record to the JSON Lines audit log. Returns false when it could
// not be written: a QC station that has quietly stopped recording is worth
// saying out loud rather than discovering at the end of a shift.
bool appendResultLine(const std::string& path, const vjson::Value& record);

}  // namespace verimeasure
