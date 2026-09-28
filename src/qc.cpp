#include "verimeasure/qc.hpp"

#include <cmath>
#include <cstdio>

#include "verimeasure/calibration.hpp"

namespace verimeasure {
namespace {

std::string mmText(double v) { return vjson::formatFixed(v, 2); }

// A percentage, always signed, always to two places. Signed because the whole
// point of the figure is which side of the target the garment came out on, and
// an unsigned "1.24%" next to a "-1.30%" limit reads as though it passed.
std::string pctText(double v) {
    const std::string body = vjson::formatFixed(v, 2);
    return ((v >= 0.0 && body[0] != '-') ? "+" : "") + body + "%";
}

// Everything both kinds of step do before they know what they measured.
// Returns false when the index names no step, having filled in the refusal.
bool openStep(const Template& t, int index, StepResult& r) {
    r.style = t.style;
    r.index = index;
    r.total = static_cast<int>(t.steps.size());
    r.timestamp = timestampIso();

    if (index < 0 || index >= static_cast<int>(t.steps.size())) {
        r.step = "unknown";
        r.reason = "there is no such step in this style";
        return false;
    }

    const Step& s = t.steps[static_cast<std::size_t>(index)];
    r.step = s.name;
    r.kind = s.kind;
    r.target_mm = s.target_mm;
    r.tol_minus_mm = s.tol_minus_mm;
    r.tol_plus_mm = s.tol_plus_mm;
    return true;
}

// The verdict itself is taken in millimetres, against the absolute limits the
// tech pack states. Only the words that explain it are a percentage, so
// rounding a percentage can never decide a pass or a fail.
//
// Shared between the folded edge and the traced curve deliberately. They are
// different measurements read in completely different ways, and a bench that
// judged them by two separately maintained copies of this arithmetic would
// eventually judge them by two different rules.
void applyTolerance(StepResult& r, const Step& s) {
    const double overshoot = r.has_percent() ? 100.0 / s.target_mm : 0.0;
    if (r.deviation_mm < s.tol_minus_mm) {
        r.pass = false;
        const double by = s.tol_minus_mm - r.deviation_mm;
        r.reason = r.has_percent()
                       ? "under by " + vjson::formatFixed(by * overshoot, 2) + "% past the " +
                             pctText(r.tol_minus_pct()) + " limit"
                       : "under by " + mmText(by) + " mm past the " + mmText(s.tol_minus_mm) +
                             " mm limit";
    } else if (r.deviation_mm > s.tol_plus_mm) {
        r.pass = false;
        const double by = r.deviation_mm - s.tol_plus_mm;
        r.reason = r.has_percent()
                       ? "over by " + vjson::formatFixed(by * overshoot, 2) + "% past the " +
                             pctText(r.tol_plus_pct()) + " limit"
                       : "over by " + mmText(by) + " mm past the +" + mmText(s.tol_plus_mm) +
                             " mm limit";
    } else {
        r.pass = true;
    }
}

}  // namespace

vjson::Value StepResult::toJson() const {
    vjson::Value o = vjson::Value::object();
    o.set("timestamp", vjson::Value::str(timestamp));
    o.set("style", vjson::Value::str(style));
    o.set("garment", vjson::Value::integer(garment));
    o.set("step", vjson::Value::str(step));
    o.set("step_index", vjson::Value::integer(index + 1));
    o.set("step_count", vjson::Value::integer(total));

    // A step that was never read is not the same as one that was read and
    // rejected, and an audit trail has to be able to tell them apart.
    o.set("kind", vjson::Value::str(kind));
    o.set("measured", vjson::Value::boolean(measured));
    if (measured) {
        o.set("measured_mm", vjson::Value::real(measured_mm));
        o.set("deviation_mm", vjson::Value::real(deviation_mm));
        o.set("straightness_mm", vjson::Value::real(straightness_mm));
        o.set("bow_mm", vjson::Value::real(bow_mm));
        o.set("reach_mm", vjson::Value::real(reach_mm));
        o.set("traced_mm", vjson::Value::real(traced_mm));

        // What a curve carries that an edge does not: both of its lengths, so
        // the log can be re-read against the other definition if the tech pack
        // turns out to have meant the other one; the radius, which says whether
        // the machine followed a neckline or the edge of a print; and the two
        // numbers that describe how well the pointing went.
        // A distance step is the straight line between two points and nothing
        // else: the chord IS the reading, under the name a curve uses for the
        // same length.
        if (kind == "distance") {
            o.set("chord_mm", vjson::Value::real(chord_mm));
            o.set("arc_mm", vjson::Value::real(arc_mm));
        }
        if (kind == "curve") {
            o.set("chord_mm", vjson::Value::real(chord_mm));
            o.set("arc_mm", vjson::Value::real(arc_mm));
            o.set("radius_mm", vjson::Value::real(radius_mm));
            o.set("confidence", vjson::Value::real(confidence));
            o.set("drift_mm", vjson::Value::real(drift_mm));
            o.set("margin", vjson::Value::real(margin));
        }
        // A collar carries a curve's numbers less the drift, which is how far
        // the machine moved off a pointing - and nothing was pointed at.
        if (kind == "collar") {
            o.set("chord_mm", vjson::Value::real(chord_mm));
            o.set("arc_mm", vjson::Value::real(arc_mm));
            if (back_mm > 0.0) o.set("back_mm", vjson::Value::real(back_mm));
            o.set("radius_mm", vjson::Value::real(radius_mm));
            o.set("confidence", vjson::Value::real(confidence));
            o.set("margin", vjson::Value::real(margin));
        }
        // A perimeter carries both of its lengths and the two halves it was
        // added up from, so a disputed reading can be taken apart again.
        if (kind == "perimeter") {
            o.set("fitted_mm", vjson::Value::real(fitted_mm));
            o.set("first_half_mm", vjson::Value::real(first_half_mm));
            o.set("second_half_mm", vjson::Value::real(second_half_mm));
            o.set("halves_differ_mm", vjson::Value::real(first_half_mm - second_half_mm));
        }
    }
    o.set("target_mm", vjson::Value::real(target_mm));
    o.set("tol_minus_mm", vjson::Value::real(tol_minus_mm));
    o.set("tol_plus_mm", vjson::Value::real(tol_plus_mm));
    o.set("low_mm", vjson::Value::real(low_mm()));
    o.set("high_mm", vjson::Value::real(high_mm()));

    // The same figures as a share of the target, which is how the status is
    // shown. Written alongside the millimetres rather than instead of them: an
    // audit trail that only kept percentages could not be checked back against
    // the tech pack the tolerances came from.
    o.set("has_percent", vjson::Value::boolean(has_percent()));
    if (has_percent()) {
        if (measured) o.set("deviation_pct", vjson::Value::real(deviation_pct()));
        o.set("tol_minus_pct", vjson::Value::real(tol_minus_pct()));
        o.set("tol_plus_pct", vjson::Value::real(tol_plus_pct()));
    }
    o.set("result", vjson::Value::str(pass ? "PASS" : "FAIL"));
    if (!reason.empty()) o.set("reason", vjson::Value::str(reason));
    return o;
}

std::string StepResult::headline() const {
    if (!measured) return step + ": not measured, " + reason;
    // The reading stays in millimetres, because that is the measurement. What
    // is said about it, how far off target and how far off is allowed, is a
    // percentage, so the same sentence means the same thing on every step.
    std::string line = step + ": " + mmText(measured_mm) + " mm, " + mmText(target_mm) +
                       " mm target";
    if (has_percent()) {
        line += ", " + pctText(deviation_pct()) + " of target, allowed " +
                pctText(tol_minus_pct()) + " / " + pctText(tol_plus_pct());
    } else {
        line += ", " + mmText(tol_minus_mm) + " / +" + mmText(tol_plus_mm) + " mm";
    }
    // On a curve, the number that says whether the seam was really there and
    // the one that says how far the machine had to go to find it. Both on
    // every line, because a bench where either is drifting is a bench about to
    // start arguing about tolerances instead.
    if (kind == "curve") {
        line += " [chord " + mmText(chord_mm) + ", arc " + mmText(arc_mm) + ", confidence " +
                vjson::formatFixed(confidence * 100.0, 0) + "%, drift " + mmText(drift_mm) +
                ", margin " + vjson::formatFixed(margin, 2) + "]";
        if (!pass) line += ", " + reason;
        return line;
    }
    if (kind == "collar") {
        line += " [chord " + mmText(chord_mm) + ", arc " + mmText(arc_mm) + ", neck drop " + mmText(bow_mm) +
                ", confidence " + vjson::formatFixed(confidence * 100.0, 0) + "%]";
        if (!pass) line += ", " + reason;
        return line;
    }
    if (kind == "perimeter") {
        line += " [halves " + mmText(first_half_mm) + " + " + mmText(second_half_mm) + ", fitted " +
                mmText(fitted_mm) + ", traced " + mmText(traced_mm) + ", corner reach " + mmText(reach_mm) + "]";
        if (!pass) line += ", " + reason;
        return line;
    }

    // The two numbers that say what kind of edge this was, so a shift where
    // the folding is drifting shows up in the log rather than as a run of
    // arguments about tolerances.
    if (bow_mm > 0.05 || reach_mm > 0.05) {
        line += " [bow " + mmText(bow_mm) + ", traced " + mmText(traced_mm) +
                ", corner reach " + mmText(reach_mm) + "]";
    }
    if (!pass) line += ", " + reason;
    return line;
}

StepResult judgeStep(const Template& t, int index, const EdgeResult& edge) {
    StepResult r;
    if (!openStep(t, index, r)) return r;
    const Step& s = t.steps[static_cast<std::size_t>(index)];

    if (!edge.ok) {
        r.measured = false;
        r.pass = false;
        r.reason = edge.error.empty() ? "the edge could not be read" : edge.error;
        return r;
    }

    r.measured = true;
    r.measured_mm = edge.length_mm;
    r.deviation_mm = edge.length_mm - s.target_mm;
    r.straightness_mm = edge.straightness_mm;
    r.bow_mm = edge.bow_mm;
    r.reach_mm = edge.reach_mm;
    r.traced_mm = edge.traced_mm;

    applyTolerance(r, s);
    return r;
}

StepResult judgeCurve(const Template& t, int index, const CurveResult& curve) {
    StepResult r;
    if (!openStep(t, index, r)) return r;
    const Step& s = t.steps[static_cast<std::size_t>(index)];
    r.kind = "curve";

    if (!curve.ok) {
        r.measured = false;
        r.pass = false;
        r.reason = curve.error.empty() ? "the seam could not be read" : curve.error;
        return r;
    }

    // Which length the step asked for. The style file decides; the fallback is
    // whatever snapCurve was configured with, which it has already applied.
    const bool alongArc =
        s.measure.empty() ? curve.along_arc : (s.measure == "arc");

    r.measured = true;
    r.measured_mm = alongArc ? curve.arc_mm : curve.chord_mm;
    r.deviation_mm = r.measured_mm - s.target_mm;
    r.chord_mm = curve.chord_mm;
    r.arc_mm = curve.arc_mm;
    r.radius_mm = curve.radius_mm;
    r.confidence = curve.confidence;
    r.drift_mm = curve.drift_mm;
    r.margin = curve.margin;
    // The two shape numbers a curve shares with an edge, under the names the
    // log already uses for them, so one field means one thing across both kinds
    // of step: how rough the boundary was, and how far it departs from its own
    // chord. `traced_mm` is the arc either way.
    r.straightness_mm = curve.roughness_mm;
    r.bow_mm = curve.bow_mm;
    r.traced_mm = curve.arc_mm;

    applyTolerance(r, s);
    return r;
}

StepResult judgeCollar(const Template& t, int index, const CollarResult& collar) {
    StepResult r = judgeCurve(t, index, collar.asCurve());
    r.kind = "collar";
    r.drift_mm = 0.0;
    if (!collar.ok && r.reason.empty()) r.reason = "the collar could not be read";
    // The whole collar: judged on the front and the back together, which
    // judgeCurve, knowing only a line, cannot add up.
    if (collar.ok && r.measured && index >= 0 && index < static_cast<int>(t.steps.size()) &&
        t.steps[static_cast<std::size_t>(index)].measure == "around") {
        r.back_mm = collar.back_mm;
        r.measured_mm = collar.arc_mm + collar.back_mm;
        r.deviation_mm = r.measured_mm - t.steps[static_cast<std::size_t>(index)].target_mm;
        r.traced_mm = r.measured_mm;
        r.pass = false;
        r.reason.clear();
        applyTolerance(r, t.steps[static_cast<std::size_t>(index)]);
    }
    return r;
}

StepResult judgeDistance(const Template& t, int index, const cv::Point2d& aMm,
                         const cv::Point2d& bMm) {
    StepResult r;
    if (!openStep(t, index, r)) return r;
    const Step& s = t.steps[static_cast<std::size_t>(index)];
    r.kind = "distance";

    const double mm = std::hypot(bMm.x - aMm.x, bMm.y - aMm.y);
    // Two taps on one place are a slip, not a dimension. A millimetre is well
    // under anything a tech pack asks for and well over a double tap.
    if (!(mm > 1.0)) {
        r.measured = false;
        r.pass = false;
        r.reason = "the two points are in the same place";
        return r;
    }

    r.measured = true;
    r.measured_mm = mm;
    r.deviation_mm = mm - s.target_mm;
    // The chord IS the reading; recorded under the name a curve uses for the
    // same length so one field means one thing across the kinds of step. The
    // arc of a straight line is its chord.
    r.chord_mm = mm;
    r.arc_mm = mm;
    r.traced_mm = mm;
    applyTolerance(r, s);
    return r;
}

StepResult judgePath(const Template& t, int index, const PathResult& path) {
    StepResult r;
    if (!openStep(t, index, r)) return r;
    const Step& s = t.steps[static_cast<std::size_t>(index)];
    r.kind = "distance";

    if (!path.ok) {
        r.measured = false;
        r.pass = false;
        r.reason = path.error.empty() ? "the path could not be measured" : path.error;
        return r;
    }

    // A drawn path is measured along itself. That is what drawing it meant;
    // a step that wants the span across it says "chord".
    const bool alongArc = s.measure != "chord";
    r.measured = true;
    r.measured_mm = alongArc ? path.arc_mm : path.chord_mm;
    r.deviation_mm = r.measured_mm - s.target_mm;
    r.chord_mm = path.chord_mm;
    r.arc_mm = path.arc_mm;
    r.bow_mm = path.bow_mm;
    r.traced_mm = path.arc_mm;
    applyTolerance(r, s);
    return r;
}

StepResult judgePerimeter(const Template& t, int index, const PerimeterResult& perimeter) {
    StepResult r;
    if (!openStep(t, index, r)) return r;
    const Step& s = t.steps[static_cast<std::size_t>(index)];
    r.kind = "perimeter";

    if (!perimeter.ok) {
        r.measured = false;
        r.pass = false;
        r.reason = perimeter.error.empty() ? "the perimeter could not be read" : perimeter.error;
        return r;
    }

    r.measured = true;
    r.measured_mm = perimeter.measured_mm();
    r.deviation_mm = r.measured_mm - s.target_mm;
    r.fitted_mm = perimeter.fitted_mm;
    r.traced_mm = perimeter.traced_mm;
    r.reach_mm = perimeter.reach_mm;
    r.first_half_mm = perimeter.halfMm(0);
    r.second_half_mm = perimeter.halfMm(1);
    applyTolerance(r, s);
    return r;
}

StepResult runStepQc(const MeasureResult& result, const PlaneModel* model,
                     const cv::Point2d& scan, const Template& t, int index, const Config& cfg,
                     const RoiBox* working) {
    EdgeResult edge;
    if (!model) {
        edge.ok = false;
        edge.error = "not calibrated, so there are no millimetres to judge";
    } else {
        measureEdge(result, *model, scan, cfg, edge, working);
    }
    return judgeStep(t, index, edge);
}

bool appendResultLine(const std::string& path, const vjson::Value& record) {
    if (path.empty()) return true;
    std::FILE* out = std::fopen(path.c_str(), "a");
    if (!out) return false;
    const std::string line = record.dump();
    const bool ok = std::fprintf(out, "%s\n", line.c_str()) > 0;
    std::fclose(out);
    return ok;
}

}  // namespace verimeasure
