#include "verimeasure/edge.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace verimeasure {
namespace {

// The run has to see this much of the edge before its own fit is worth
// anything. Below it the direction is set by a handful of neighbouring points
// and would happily lock onto a thread.
constexpr int kSeedPoints = 8;
constexpr double kSeedMm = 10.0;

// How far the boundary may retreat back along the edge before the corner is
// behind us. Comfortably above the noise the sub-pixel refinement leaves and
// comfortably below any real corner.
constexpr double kCornerRetreatMm = 2.0;

// Running second moments, so the fit can be carried along the edge one point at
// a time, with points joining at the front and leaving at the back, and its
// direction read back in constant time as it goes.
struct Moments {
    int n = 0;
    double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;

    void add(const cv::Point2d& p) {
        ++n;
        sx += p.x;
        sy += p.y;
        sxx += p.x * p.x;
        sxy += p.x * p.y;
        syy += p.y * p.y;
    }

    void remove(const cv::Point2d& p) {
        --n;
        sx -= p.x;
        sy -= p.y;
        sxx -= p.x * p.x;
        sxy -= p.x * p.y;
        syy -= p.y * p.y;
    }

    cv::Point2d centroid() const {
        const double inv = n ? 1.0 / n : 0.0;
        return cv::Point2d(sx * inv, sy * inv);
    }

    void covariance(double& cxx, double& cxy, double& cyy) const {
        const double inv = n ? 1.0 / n : 0.0;
        const double mx = sx * inv, my = sy * inv;
        cxx = sxx * inv - mx * mx;
        cxy = sxy * inv - mx * my;
        cyy = syy * inv - my * my;
    }

    // RMS distance of the points from their own best-fit line, measured
    // perpendicular to it. The smaller eigenvalue of the covariance, so it does
    // not blow up on an edge that happens to lie near vertical in the grid.
    double straightness() const {
        if (n < 3) return 0.0;
        double cxx, cxy, cyy;
        covariance(cxx, cxy, cyy);
        const double half = 0.5 * (cxx + cyy);
        const double diff = 0.5 * (cxx - cyy);
        return std::sqrt(std::max(0.0, half - std::sqrt(diff * diff + cxy * cxy)));
    }

    // The direction the points run in: the major axis of their covariance.
    bool direction(cv::Point2d& dir) const {
        if (n < 2) return false;
        double cxx, cxy, cyy;
        covariance(cxx, cxy, cyy);
        const double half = 0.5 * (cxx + cyy);
        const double diff = 0.5 * (cxx - cyy);
        const double major = half + std::sqrt(diff * diff + cxy * cxy);
        // Either column of the adjugate spans the eigenspace; take whichever is
        // better conditioned, so an edge lying square to the grid is no worse a
        // case than one lying across it.
        cv::Point2d v(cxy, major - cxx);
        double len = std::hypot(v.x, v.y);
        const cv::Point2d w(major - cyy, cxy);
        const double lenW = std::hypot(w.x, w.y);
        if (lenW > len) {
            v = w;
            len = lenW;
        }
        if (len < 1e-12) return false;
        dir = cv::Point2d(v.x / len, v.y / len);
        return true;
    }
};

// Perpendicular distance from p to the line through c running along dir.
inline double offLine(const cv::Point2d& p, const cv::Point2d& c, const cv::Point2d& dir) {
    return (p.y - c.y) * dir.x - (p.x - c.x) * dir.y;
}

// Least squares y = A + B t + C t^2, with t already scaled into [-1, 1] so the
// normal equations stay well conditioned whatever the edge is made of.
bool fitQuadratic(const std::vector<double>& t, const std::vector<double>& y, double& A, double& B,
                  double& C) {
    const std::size_t n = t.size();
    if (n < static_cast<std::size_t>(kSeedPoints) || n != y.size()) return false;

    double s0 = static_cast<double>(n), s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    double u0 = 0, u1 = 0, u2 = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double a = t[k], a2 = a * a;
        s1 += a;
        s2 += a2;
        s3 += a2 * a;
        s4 += a2 * a2;
        u0 += y[k];
        u1 += a * y[k];
        u2 += a2 * y[k];
    }

    const auto det3 = [](double a, double b, double c, double d, double e, double f, double g,
                         double h, double i) {
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    };

    const double det = det3(s0, s1, s2, s1, s2, s3, s2, s3, s4);
    if (std::abs(det) < 1e-9 * std::max(1.0, s0 * s2 * s4)) return false;

    A = det3(u0, s1, s2, u1, s2, s3, u2, s3, s4) / det;
    B = det3(s0, u0, s2, s1, u1, s3, s2, u2, s4) / det;
    C = det3(s0, s1, u0, s1, s2, u1, s2, s3, u2) / det;
    return true;
}

// One stretch of the boundary the search believes is a single edge.
struct Run {
    std::size_t start = 0;  // where on the outline it begins
    std::size_t walk = 0;   // boundary points it spans, defects included
    double length = 0.0;    // end to end
    double depth = 0.0;     // how far into the picture it sits, on average
};

// Scratch the growth loop reuses, so following an edge costs no allocations.
struct Scratch {
    std::vector<std::size_t> window;
};

// Grow the longest single edge of the boundary starting at `s`.
//
// The fit is carried along the edge rather than stretched across the whole of
// it: it looks at the last `windowMm` of accepted boundary and no further, so a
// garment edge that curves gently is straight everywhere the fit can see and
// the run follows it the whole way. A fit stretched across the whole run
// instead accumulates the curvature into its residual and stops partway, which
// is what read a 540 mm hem as 444 mm.
//
// What ends a run is turning, not curving. The tangent is compared against the
// one the run started with, and the run stops when the edge has turned past
// `turnLimit`. A hem sweeping 15 mm over half a metre turns about 13 degrees end
// to end. An armhole, a side seam, or the corner between two edges turns far
// more than that within a few millimetres, so that is where the edge ends -
// which is also where the seam the tape is measured to actually is.
//
// Points that disagree with the fit are not the end of it either. A thread, a
// nick, a label lying across the fold: the run steps over each of these and
// carries on, and stops only where the boundary walks away along the edge
// instead of handing it back.
Run growRun(const std::vector<cv::Point2d>& pts, const std::vector<double>& depth, std::size_t s,
            double reject, double bridgeMm, double windowMm, double turnLimit, Scratch& scratch) {
    const std::size_t n = pts.size();
    const double bridgeArcMm = 10.0 * bridgeMm;
    // The boundary is a chain, resampled at a fraction of a millimetre. A step
    // bigger than the largest defect the run would step over is not a step
    // along it, it is two pieces of outline that happen to line up, and a run
    // that spans one measures the gap.
    const double gapMm = std::max(1.0, bridgeMm);

    Run best;
    best.start = s;

    std::vector<std::size_t>& window = scratch.window;
    window.clear();
    window.push_back(s);
    std::size_t head = 0;

    Moments m;
    m.add(pts[s]);
    double sumDepth = depth[s];
    int accepted = 1;
    cv::Point2d lastKept = pts[s];
    cv::Point2d previous = pts[s];
    cv::Point2d dir(0.0, 0.0);
    cv::Point2d startDir(0.0, 0.0);
    bool haveDir = false;
    double defectArc = 0.0;

    for (std::size_t k = 1; k < n; ++k) {
        const std::size_t i = (s + k) % n;
        const cv::Point2d& p = pts[i];
        const double stepArc = std::hypot(p.x - previous.x, p.y - previous.y);
        previous = p;
        if (stepArc > gapMm) break;

        if (haveDir && std::abs(offLine(p, m.centroid(), dir)) > reject) {
            defectArc += stepArc;
            const double along =
                std::abs((p.x - lastKept.x) * dir.x + (p.y - lastKept.y) * dir.y);
            if (along > bridgeMm || defectArc > bridgeArcMm) break;
            continue;
        }

        m.add(p);
        window.push_back(i);
        sumDepth += depth[i];
        ++accepted;
        defectArc = 0.0;
        lastKept = p;

        // Retire boundary that is further back than the fit is allowed to see.
        while (window.size() - head > static_cast<std::size_t>(kSeedPoints)) {
            const cv::Point2d& oldest = pts[window[head]];
            if (std::hypot(p.x - oldest.x, p.y - oldest.y) <= windowMm) break;
            m.remove(oldest);
            ++head;
        }

        const double span = std::hypot(p.x - pts[window[head]].x, p.y - pts[window[head]].y);
        if (m.n < kSeedPoints || (!haveDir && span < kSeedMm)) continue;
        if (!m.direction(dir)) break;
        // The major axis has no sign of its own; point it the way the run is
        // travelling, so the turn below is a turn and not an ambiguity.
        if ((p.x - pts[window[head]].x) * dir.x + (p.y - pts[window[head]].y) * dir.y < 0.0) {
            dir = cv::Point2d(-dir.x, -dir.y);
        }
        if (!haveDir) {
            startDir = dir;
            haveDir = true;
        }

        const double along = std::min(1.0, std::max(-1.0, startDir.x * dir.x + startDir.y * dir.y));
        if (std::acos(along) > turnLimit) break;

        const double len = std::hypot(p.x - pts[s].x, p.y - pts[s].y);
        if (len > best.length) {
            best.length = len;
            best.walk = k + 1;
            best.depth = sumDepth / accepted;
        }
    }

    return best;
}

// The edge as an arc, and the measurement as the chord across it.
struct Fit {
    cv::Point2d centre{0.0, 0.0};
    cv::Point2d dir{1.0, 0.0};
    cv::Point2d nrm{0.0, 1.0};
    double A = 0.0, B = 0.0, C = 0.0;  // y = A + B t + C t^2, t = x / h
    double h = 1.0;

    cv::Point2d a{0.0, 0.0};  // the two corners
    cv::Point2d b{0.0, 0.0};
    double length = 0.0;      // the chord between them: the measurement
    double roughness = 0.0;   // RMS of the boundary about the arc
    double bow = 0.0;         // how far the arc departs from its own chord
    double reach = 0.0;       // how much of the length the corners were reached for
    double traced = 0.0;      // the edge followed rather than crossed
    std::vector<cv::Point2d> path;  // the edge as it lies, corner to corner

    double x(const cv::Point2d& p) const {
        return (p.x - centre.x) * dir.x + (p.y - centre.y) * dir.y;
    }
    double y(const cv::Point2d& p) const {
        return (p.x - centre.x) * nrm.x + (p.y - centre.y) * nrm.y;
    }
    double curve(double xx) const {
        const double t = xx / h;
        return A + B * t + C * t * t;
    }
    double off(const cv::Point2d& p) const { return y(p) - curve(x(p)); }
    // The direction the edge is running at x, in the picture's own frame.
    cv::Point2d tangent(double xx) const {
        const double slope = (B + 2.0 * C * (xx / h)) / h;
        const double len = std::sqrt(1.0 + slope * slope);
        return cv::Point2d((dir.x + nrm.x * slope) / len, (dir.y + nrm.y * slope) / len);
    }
    cv::Point2d at(double xx) const {
        const double yy = curve(xx);
        return cv::Point2d(centre.x + dir.x * xx + nrm.x * yy,
                           centre.y + dir.y * xx + nrm.y * yy);
    }
};

// The edge that leaves this one at `end`, if there is one.
//
// Walked outward along the boundary, past however far the corner is rounded
// through, looking for the first stretch that is straight over a real length
// and lies across this edge rather than along it. Both conditions matter. The
// straightness is what stops a piece of the rounding being mistaken for the
// seam; the angle is what tells a seam from a fold, and on a fold this finds
// nothing and says so, which is the whole point of it.
bool adjacentEdge(const std::vector<cv::Point2d>& pts, std::size_t end, int step, const Fit& fit,
                  double tol, double searchMm, double fitMm, double minAngle,
                  cv::Point2d& linePt, cv::Point2d& lineDir) {
    const std::size_t n = pts.size();
    const cv::Point2d anchor = pts[end];

    std::vector<cv::Point2d> walk;
    walk.reserve(256);
    for (std::size_t k = 1; k < n; ++k) {
        const std::size_t i = step > 0 ? (end + k) % n : (end + n - k) % n;
        const cv::Point2d& p = pts[i];
        if (std::hypot(p.x - anchor.x, p.y - anchor.y) > searchMm) break;
        walk.push_back(p);
    }
    if (walk.size() < 24) return false;

    const double limit = std::cos(minAngle);
    for (std::size_t s = 0; s + 12 < walk.size(); ++s) {
        Moments m;
        double span = 0.0;
        for (std::size_t k = s; k < walk.size(); ++k) {
            span = std::hypot(walk[k].x - walk[s].x, walk[k].y - walk[s].y);
            if (span > fitMm) break;
            m.add(walk[k]);
        }
        if (m.n < 12 || span < fitMm * 0.9) continue;
        // Held to a good deal tighter than the edge itself. A window that still
        // has a little of the rounding in it passes an ordinary straightness
        // test while leaning by a degree or two, and a degree of lean on the
        // seam moves the crossing several millimetres.
        if (m.straightness() > 0.35 * tol) continue;
        cv::Point2d d;
        if (!m.direction(d)) continue;
        if (std::abs(d.x * fit.dir.x + d.y * fit.dir.y) > limit) continue;
        linePt = m.centroid();
        lineDir = d;
        return true;
    }
    return false;
}

// The definitive reading of one run: fit the arc, drop what is not the edge,
// fit again, and take the answer from what is left.
//
// The answer is the CHORD between the two corners, never the arc length. A
// measurement is what a tape laid flat between two seams reads, and a tape does
// not follow the sweep of a hem. Curvature is recorded beside the number, not
// added to it.
bool describeRun(const std::vector<cv::Point2d>& pts, const Run& run, const Config& cfg,
                 Fit& out) {
    const std::size_t n = pts.size();
    if (run.walk < static_cast<std::size_t>(kSeedPoints)) return false;
    const double reject = std::max(cfg.edge_straightness_mm, cfg.edge_defect_reject_mm);

    std::vector<std::size_t> idx;
    idx.reserve(run.walk);
    for (std::size_t k = 0; k < run.walk; ++k) idx.push_back((run.start + k) % n);

    Moments m;
    for (std::size_t i : idx) m.add(pts[i]);
    if (!m.direction(out.dir)) return false;
    out.centre = m.centroid();
    out.nrm = cv::Point2d(-out.dir.y, out.dir.x);

    std::vector<std::size_t> keep = idx;
    std::vector<double> ts, ys;
    for (int round = 0; round < 3; ++round) {
        double h = 1e-9;
        for (std::size_t i : keep) h = std::max(h, std::abs(out.x(pts[i])));
        out.h = h;

        ts.clear();
        ys.clear();
        for (std::size_t i : keep) {
            ts.push_back(out.x(pts[i]) / h);
            ys.push_back(out.y(pts[i]));
        }
        if (!fitQuadratic(ts, ys, out.A, out.B, out.C)) return false;
        if (round == 2) break;

        std::vector<std::size_t> next;
        next.reserve(keep.size());
        for (std::size_t i : keep) {
            if (std::abs(out.off(pts[i])) <= reject) next.push_back(i);
        }
        if (next.size() < static_cast<std::size_t>(kSeedPoints) || next.size() == keep.size()) {
            break;
        }
        keep.swap(next);
    }

    double lo = 0.0, hi = 0.0, sumSq = 0.0;
    bool first = true;
    for (std::size_t i : keep) {
        const double xx = out.x(pts[i]);
        const double off = out.off(pts[i]);
        sumSq += off * off;
        if (first) {
            lo = hi = xx;
            first = false;
        } else {
            lo = std::min(lo, xx);
            hi = std::max(hi, xx);
        }
    }
    if (first) return false;
    out.roughness = std::sqrt(sumSq / static_cast<double>(keep.size()));
    // What the edge measured before anything was reached for, so how much the
    // corner correction is worth can be read off rather than argued about.
    const double rawLo = lo, rawHi = hi;

    // Reach on past each end for the corner.
    //
    // The run stops where the boundary turns, which is a little short of the
    // seam: fabric does not turn a square corner, it rounds one, and the tape
    // is laid to where the two edges would have met. So the extent is carried
    // outward while the boundary is still recognisably this edge, and the
    // corner is taken from the arc rather than from the rounding, which is
    // where the seam is. Bounded, because past a certain point this stops being
    // the edge and starts being the next one.
    // How far the cloth itself reaches along this edge.
    //
    // Nothing the corner search does may put an end of the measurement past
    // this. Whatever the geometry says about where two edges would have met,
    // the garment stops where it stops, and a line drawn past the cloth is a
    // line measuring the mat. This is a hard bound, not a tolerance.
    //
    // It is also, for a seam meeting the edge square, very nearly the right
    // answer on its own: the rounding carries the boundary out to the same
    // along-position the two edges would have crossed at, whatever radius it
    // rounds through. So the crossing refines inside this bound rather than
    // reaching past it.
    double clothLo = lo, clothHi = hi;
    for (std::size_t k = 0; k < n; ++k) {
        if (std::abs(out.off(pts[k])) > cfg.edge_band_mm) continue;
        const double xx = out.x(pts[k]);
        clothLo = std::min(clothLo, xx);
        clothHi = std::max(clothHi, xx);
    }

    const double cap = std::max(0.0, cfg.edge_corner_reach_mm);
    if (cap > 0.0 && !idx.empty()) {
        const double tol = std::max(0.05, cfg.edge_straightness_mm);
        const double minAngle =
            std::max(5.0, cfg.edge_corner_min_angle_deg) * 3.14159265358979323846 / 180.0;
        const double searchMm = std::max(40.0, cfg.edge_corner_search_mm);
        const double fitMm = 0.5 * searchMm;

        const double sense = (out.x(pts[idx.back()]) >= out.x(pts[idx.front()])) ? 1.0 : -1.0;
        const std::size_t ends[2] = {idx.back(), idx.front()};
        const int steps[2] = {1, -1};
        const double senses[2] = {sense, -sense};

        for (int side = 0; side < 2; ++side) {
            cv::Point2d linePt, lineDir;
            if (!adjacentEdge(pts, ends[side], steps[side], out, tol, searchMm, fitMm, minAngle,
                              linePt, lineDir)) {
                continue;
            }
            // Carry this edge on from where the cloth stopped being flat, and
            // cross it with the one leaving. That crossing is the seam, and it
            // is what the tape is laid to.
            const double xEnd = (senses[side] > 0.0) ? rawHi : rawLo;
            const cv::Point2d from = out.at(xEnd);
            const cv::Point2d along = out.tangent(xEnd);
            const double denom = along.x * lineDir.y - along.y * lineDir.x;
            if (std::abs(denom) < 1e-9) continue;
            const cv::Point2d w(linePt.x - from.x, linePt.y - from.y);
            const double t = (w.x * lineDir.y - w.y * lineDir.x) / denom;
            const cv::Point2d seam(from.x + along.x * t, from.y + along.y * t);

            // Outward only, and only so far. A crossing further out than this
            // is a crossing with something that is not this corner.
            const double gained = senses[side] * (out.x(seam) - xEnd);
            if (!(gained > 0.0) || gained > cap) continue;
            lo = std::min(lo, out.x(seam));
            hi = std::max(hi, out.x(seam));
        }
    }

    // ...and never past the cloth, whatever the crossing said.
    lo = std::max(lo, clothLo);
    hi = std::min(hi, clothHi);

    out.a = out.at(lo);
    out.b = out.at(hi);
    out.length = std::hypot(out.b.x - out.a.x, out.b.y - out.a.y);
    out.reach = std::max(0.0, (hi - lo) - (rawHi - rawLo));

    // The edge itself, sampled along the arc between the two corners. Fine
    // enough that the drawn trace sits on the cloth at this resolution, and
    // fine enough that summing it is the arc length to well under the noise.
    constexpr int kPathSamples = 48;
    out.path.clear();
    out.path.reserve(kPathSamples + 1);
    out.traced = 0.0;
    for (int k = 0; k <= kPathSamples; ++k) {
        const cv::Point2d p = out.at(lo + (hi - lo) * k / kPathSamples);
        if (k) out.traced += std::hypot(p.x - out.path.back().x, p.y - out.path.back().y);
        out.path.push_back(p);
    }
    // Sagitta of the fitted arc over the span actually measured.
    const double span = (hi - lo) / out.h;
    out.bow = std::abs(out.C) * span * span / 4.0;
    return out.length > 0.0;
}

double distanceToSegment(const cv::Point2d& p, const cv::Point2d& a, const cv::Point2d& b) {
    const cv::Point2d v = b - a;
    const double vv = v.x * v.x + v.y * v.y;
    double t = 0.0;
    if (vv > 1e-12) t = std::min(1.0, std::max(0.0, ((p - a).x * v.x + (p - a).y * v.y) / vv));
    return std::hypot(p.x - a.x - v.x * t, p.y - a.y - v.y * t);
}

// How close a point sits to the border of the working area, not counting the
// side the fold is presented to.
//
// A fold lying hard against the rail is a fold laid the way the step asked for
// it, and every reading would be refused if that counted as running off the
// picture. It is the two ends of the edge that have to be inside the frame, and
// they leave it through one of the other three sides.
double distanceToBorder(const cv::Point2d& p, const std::vector<cv::Point2d>& border,
                        const cv::Point2d& scan) {
    const std::size_t n = border.size();

    // The rail is the side the machine looks in from, which is the one lying
    // nearest along the direction it looks.
    std::size_t rail = 0;
    double shallowest = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const cv::Point2d& a = border[k];
        const cv::Point2d& b = border[(k + 1) % n];
        const double d = 0.5 * ((a.x + b.x) * scan.x + (a.y + b.y) * scan.y);
        if (k == 0 || d < shallowest) {
            shallowest = d;
            rail = k;
        }
    }

    double best = 1e18;
    for (std::size_t k = 0; k < n; ++k) {
        if (k == rail) continue;
        best = std::min(best, distanceToSegment(p, border[k], border[(k + 1) % n]));
    }
    return best;
}

}  // namespace

vjson::Value EdgeResult::toJson() const {
    vjson::Value o = vjson::Value::object();
    o.set("ok", vjson::Value::boolean(ok));
    if (ok) {
        o.set("length_mm", vjson::Value::real(length_mm));
        o.set("straightness_mm", vjson::Value::real(straightness_mm));
        o.set("bow_mm", vjson::Value::real(bow_mm));
        o.set("reach_mm", vjson::Value::real(reach_mm));
        o.set("traced_mm", vjson::Value::real(traced_mm));
    } else {
        if (clipped) o.set("clipped", vjson::Value::boolean(true));
        if (!error.empty()) o.set("error", vjson::Value::str(error));
    }
    return o;
}

bool scanDirection(const RoiBox& roi, const PlaneModel& model, const std::string& from,
                   cv::Point2d& out) {
    if (!roi.valid()) return false;

    const double cx = 0.5 * (roi.x1 + roi.x2), cy = 0.5 * (roi.y1 + roi.y2);
    cv::Point2d startPx, endPx;
    if (from == "right") {
        startPx = cv::Point2d(roi.x2, cy);
        endPx = cv::Point2d(roi.x1, cy);
    } else if (from == "top") {
        startPx = cv::Point2d(cx, roi.y1);
        endPx = cv::Point2d(cx, roi.y2);
    } else if (from == "bottom") {
        startPx = cv::Point2d(cx, roi.y2);
        endPx = cv::Point2d(cx, roi.y1);
    } else {  // "left"
        startPx = cv::Point2d(roi.x1, cy);
        endPx = cv::Point2d(roi.x2, cy);
    }

    const std::vector<cv::Point2d> mm = model.pxToMm({startPx, endPx});
    if (mm.size() != 2) return false;
    const cv::Point2d v = mm[1] - mm[0];
    const double len = std::hypot(v.x, v.y);
    if (len < 1e-6) return false;
    out = cv::Point2d(v.x / len, v.y / len);
    return true;
}

double railLengthMm(const RoiBox& roi, const PlaneModel& model, const std::string& from) {
    if (!roi.valid()) return 0.0;
    const bool vertical = (from == "left" || from == "right");
    cv::Point2d a, b;
    if (vertical) {
        const double x = (from == "left") ? roi.x1 : roi.x2;
        a = cv::Point2d(x, roi.y1);
        b = cv::Point2d(x, roi.y2);
    } else {
        const double y = (from == "top") ? roi.y1 : roi.y2;
        a = cv::Point2d(roi.x1, y);
        b = cv::Point2d(roi.x2, y);
    }
    const std::vector<cv::Point2d> mm = model.pxToMm({a, b});
    if (mm.size() != 2) return 0.0;
    return std::hypot(mm[1].x - mm[0].x, mm[1].y - mm[0].y);
}

std::vector<cv::Point2d> workingAreaMm(const RoiBox& roi, const PlaneModel& model) {
    if (!roi.valid()) return {};
    return model.pxToMm({cv::Point2d(roi.x1, roi.y1), cv::Point2d(roi.x2, roi.y1),
                         cv::Point2d(roi.x2, roi.y2), cv::Point2d(roi.x1, roi.y2)});
}

bool measureEdge(const std::vector<cv::Point2d>& outlineMm, const cv::Point2d& scan,
                 const Config& cfg, EdgeResult& out, const std::vector<cv::Point2d>* borderMm) {
    out = EdgeResult();

    const std::size_t n = outlineMm.size();
    if (n < 32) {
        out.error = "nothing usable was segmented";
        return false;
    }

    // How deep into the picture each boundary point sits, measured along the
    // direction the machine looks in. The fold is presented to that side, so
    // the edge that is the measurement is the shallowest one there is.
    std::vector<double> depth(n);
    for (std::size_t k = 0; k < n; ++k) {
        depth[k] = outlineMm[k].x * scan.x + outlineMm[k].y * scan.y;
    }

    const double reject = std::max(cfg.edge_straightness_mm, cfg.edge_defect_reject_mm);
    const double bridge = std::max(0.0, cfg.edge_defect_bridge_mm);
    const double windowMm = std::max(kSeedMm, cfg.edge_tangent_window_mm);
    const double turnLimit =
        std::max(1.0, cfg.edge_turn_limit_deg) * 3.14159265358979323846 / 180.0;

    // Every edge of the boundary long enough to be a fold, wherever it lies.
    // Which of them is the near one is a question about the edges, not about
    // individual points: a single point can be a thread hanging past the fold,
    // and taking the near side of the picture from it either hides the fold
    // behind that thread or leaves only the part of it the thread's own depth
    // happens to admit.
    Scratch scratch;
    std::vector<Run> candidates;
    for (std::size_t s = 0; s < n; ++s) {
        const Run r = growRun(outlineMm, depth, s, reject, bridge, windowMm, turnLimit, scratch);
        if (r.length >= cfg.edge_min_length_mm) candidates.push_back(r);
    }

    const char* const kNoEdge =
        "no clean straight edge was found facing the marked side. Fold the garment so the "
        "measurement is one straight edge, and turn that edge to face the mark.";

    if (candidates.empty()) {
        out.error = kNoEdge;
        return false;
    }

    // The nearest of them sets where the near side of the picture is; anything
    // within the band of it is competing to be the fold, and the longest of
    // those wins. That is what makes this the FIRST edge from the marked side
    // rather than whichever edge of the garment happens to be straightest.
    double nearest = candidates.front().depth;
    for (const Run& r : candidates) nearest = std::min(nearest, r.depth);
    const double band = nearest + std::max(0.0, cfg.edge_band_mm);

    const Run* winner = nullptr;
    for (const Run& r : candidates) {
        if (r.depth > band) continue;
        if (!winner || r.length > winner->length) winner = &r;
    }
    if (!winner) {
        out.error = kNoEdge;
        return false;
    }

    Fit fit;
    if (!describeRun(outlineMm, *winner, cfg, fit)) {
        out.error = "the edge facing the mark could not be fitted";
        return false;
    }

    out.a_mm = fit.a;
    out.b_mm = fit.b;
    out.path_mm = fit.path;
    out.traced_mm = fit.traced;
    out.length_mm = cfg.edge_measure_along_curve ? fit.traced : fit.length;
    out.straightness_mm = fit.roughness;
    out.bow_mm = fit.bow;
    out.reach_mm = fit.reach;

    if (out.length_mm < cfg.edge_min_length_mm) {
        out.error = kNoEdge;
        return false;
    }

    // How bowed the edge is, as a share of its own length.
    //
    // A garment edge is not a straight line and never was: a hem sweeps, a
    // shoulder slopes, a chest line runs across a body that is wider at the
    // bottom. That curvature belongs to the garment, so the arc follows it and
    // the chord across it is still the measurement. What it cannot be allowed
    // to hide is a fold gone wrong, where the cloth between the corners is
    // slack and the corners themselves are no longer where the seams are. The
    // line between the two is drawn here, and it is a share rather than a
    // millimetre count because a 14 mm sweep means one thing across half a
    // metre of chest and another across a 230 mm sleeve.
    if (out.bow_mm > cfg.edge_max_bow_fraction * out.length_mm) {
        out.error = "the edge facing the mark bows by " + vjson::formatFixed(out.bow_mm, 0) +
                    " mm across " + vjson::formatFixed(out.length_mm, 0) +
                    " mm, too much to trust where it ends. Smooth the fold out and lay it "
                    "again.";
        return false;
    }

    // Does the run cover the whole edge, or only part of one?
    //
    // The arc handles a curved edge, and the bow limit above handles one bent
    // past use. This is the backstop for everything else: a run that stopped
    // early on a defect, a corner in the middle of what should have been one
    // edge, a lay where the cloth facing the mark is simply longer than the
    // stretch of it that reads as a single edge. The band of boundary lying
    // along the arc is compared against the chord, and on a lay that was
    // presented properly they are the same length.
    {
        double lo = 0.0, hi = 0.0;
        bool first = true;
        for (std::size_t k = 0; k < n; ++k) {
            if (std::abs(fit.off(outlineMm[k])) > cfg.edge_band_mm) continue;
            const double t = fit.x(outlineMm[k]);
            if (first) {
                lo = hi = t;
                first = false;
            } else {
                lo = std::min(lo, t);
                hi = std::max(hi, t);
            }
        }
        const double cover = std::max(0.05, std::min(1.0, cfg.edge_coverage_fraction));
        if (hi - lo > out.length_mm / cover) {
            out.error = "the edge facing the mark is not one edge: only " +
                        vjson::formatFixed(out.length_mm, 0) + " mm of it reads as one out of " +
                        vjson::formatFixed(hi - lo, 0) +
                        " mm. Smooth the fold out and lay it again.";
            return false;
        }
    }

    // An edge that ends on the border of the picture did not end there because
    // the garment did. What comes back is the width of the view, and it looks
    // exactly like a measurement: straight, repeatable, and short by however
    // much of the fold was outside the frame. Say so instead.
    if (borderMm && borderMm->size() >= 3 && cfg.edge_clip_margin_mm > 0.0) {
        const double margin = cfg.edge_clip_margin_mm;
        if (distanceToBorder(fit.a, *borderMm, scan) < margin ||
            distanceToBorder(fit.b, *borderMm, scan) < margin) {
            out.clipped = true;
            out.error = "the fold runs off the edge of the picture, so only the " +
                        vjson::formatFixed(out.length_mm, 0) +
                        " mm of it that is in view was measured. Slide the garment until both "
                        "ends of the fold are inside the frame.";
            return false;
        }
    }

    out.ok = true;
    return true;
}

bool measureEdge(const MeasureResult& result, const PlaneModel& model, const cv::Point2d& scan,
                 const Config& cfg, EdgeResult& out, const RoiBox* working) {
    out = EdgeResult();
    if (!result.calibrated) {
        out.error = "not calibrated, so there are no millimetres to measure in";
        return false;
    }
    if (result.outline_px.size() < 32) {
        out.error = "the measurement carries no usable outline";
        return false;
    }
    std::vector<cv::Point2d> border;
    if (working) border = workingAreaMm(*working, model);
    return measureEdge(model.pxToMm(result.outline_px), scan, cfg, out,
                       border.size() >= 3 ? &border : nullptr);
}

}  // namespace verimeasure
