#include "verimeasure/perimeter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "verimeasure/cv_compat.hpp"

namespace verimeasure {
namespace {

constexpr double kPi = 3.14159265358979323846;
inline double toRad(double d) { return d * kPi / 180.0; }
inline double toDeg(double r) { return r * 180.0 / kPi; }
inline double dist(const cv::Point2d& a, const cv::Point2d& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// The working samples the fitted length is taken on, in millimetres apart.
constexpr double kSampleMm = 1.0;

// Length of the polyline from index a to index b.
double polyLength(const std::vector<cv::Point2d>& p, std::size_t a, std::size_t b) {
    double L = 0.0;
    for (std::size_t i = a; i < b && i + 1 < p.size(); ++i) L += dist(p[i], p[i + 1]);
    return L;
}

// Samples every `step` of arc along a polyline, first and last points kept.
std::vector<cv::Point2d> resampleArc(const std::vector<cv::Point2d>& in, double step) {
    std::vector<cv::Point2d> out;
    if (in.empty()) return out;
    out.push_back(in.front());
    if (in.size() < 2 || !(step > 0.0)) {
        if (in.size() >= 2) out.push_back(in.back());
        return out;
    }
    double need = step;  // arc still to travel before the next sample
    for (std::size_t i = 0; i + 1 < in.size(); ++i) {
        const cv::Point2d a = in[i];
        const cv::Point2d b = in[i + 1];
        const double L = dist(a, b);
        if (!(L > 0.0)) continue;
        double pos = 0.0;
        while (L - pos >= need) {
            pos += need;
            out.push_back(a + (b - a) * (pos / L));
            need = step;
        }
        need -= (L - pos);
    }
    if (dist(out.back(), in.back()) > 1e-6 * step) out.push_back(in.back());
    return out;
}

// A Gaussian along an open path, reflected through its ends so that both ends
// stay exactly where they were and a straight end stays straight. Only the
// scatter the sub-pixel refinement leaves is meant to go; sigma is in samples.
std::vector<cv::Point2d> smoothOpen(const std::vector<cv::Point2d>& in, double sigma) {
    const int n = static_cast<int>(in.size());
    if (n < 3 || !(sigma > 0.25)) return in;
    const int half = static_cast<int>(std::ceil(3.0 * sigma));
    std::vector<double> w(static_cast<std::size_t>(2 * half + 1));
    double sum = 0.0;
    for (int k = -half; k <= half; ++k) {
        const double v = std::exp(-0.5 * (k / sigma) * (k / sigma));
        w[static_cast<std::size_t>(k + half)] = v;
        sum += v;
    }
    const auto at = [&](int i) -> cv::Point2d {
        if (i < 0) return in[0] * 2.0 - in[static_cast<std::size_t>(std::min(-i, n - 1))];
        if (i >= n) {
            return in[static_cast<std::size_t>(n - 1)] * 2.0 -
                   in[static_cast<std::size_t>(std::max(2 * (n - 1) - i, 0))];
        }
        return in[static_cast<std::size_t>(i)];
    };
    std::vector<cv::Point2d> out(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        cv::Point2d acc(0.0, 0.0);
        for (int k = -half; k <= half; ++k) acc += at(i + k) * w[static_cast<std::size_t>(k + half)];
        out[static_cast<std::size_t>(i)] = acc * (1.0 / sum);
    }
    return out;
}

// The direction the path runs at each sample, from the chord either side of
// it, unwrapped so a turn reads as a turn and not as a jump of a full circle.
std::vector<double> tangentAngles(const std::vector<cv::Point2d>& p, int k) {
    const int n = static_cast<int>(p.size());
    std::vector<double> th(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& a = p[static_cast<std::size_t>(std::max(0, i - k))];
        const cv::Point2d& b = p[static_cast<std::size_t>(std::min(n - 1, i + k))];
        th[static_cast<std::size_t>(i)] = std::atan2(b.y - a.y, b.x - a.x);
        if (i > 0) {
            double& t = th[static_cast<std::size_t>(i)];
            const double prev = th[static_cast<std::size_t>(i - 1)];
            while (t - prev > kPi) t -= 2.0 * kPi;
            while (t - prev < -kPi) t += 2.0 * kPi;
        }
    }
    return th;
}

// Least squares y = A + B s + C s^2 with s scaled into [-1, 1], as edge.cpp
// fits its arcs, so the normal equations stay well conditioned.
bool fitQuadratic(const std::vector<double>& s, const std::vector<double>& y, double& A, double& B,
                  double& C) {
    const std::size_t n = s.size();
    if (n < 5 || n != y.size()) return false;
    double s0 = static_cast<double>(n), s1 = 0, s2 = 0, s3 = 0, s4 = 0, u0 = 0, u1 = 0, u2 = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double a = s[k], a2 = a * a;
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

// A parabola fitted to samples a..b in the frame of their own chord.
struct Parabola {
    bool ok = false;
    cv::Point2d origin{0.0, 0.0};
    cv::Point2d u{1.0, 0.0};  // along the chord
    cv::Point2d v{0.0, 1.0};  // across it
    double tMid = 0.0, tHalf = 1.0;
    double A = 0.0, B = 0.0, C = 0.0;
    double tFirst = 0.0, tLast = 0.0;  // where the first and last samples project
    double rms = 0.0, worst = 0.0;
    std::size_t worstAt = 0;

    double y(double t) const {
        const double s = (t - tMid) / tHalf;
        return A + B * s + C * s * s;
    }
    double slope(double t) const {
        const double s = (t - tMid) / tHalf;
        return (B + 2.0 * C * s) / tHalf;
    }
    cv::Point2d at(double t) const { return origin + u * t + v * y(t); }
    // Length of the curve between two positions along the chord.
    double length(double t0, double t1) const {
        const int steps = 96;
        double L = 0.0;
        cv::Point2d prev = at(t0);
        for (int k = 1; k <= steps; ++k) {
            const cv::Point2d q = at(t0 + (t1 - t0) * k / steps);
            L += dist(prev, q);
            prev = q;
        }
        return L;
    }
};

Parabola fitParabola(const std::vector<cv::Point2d>& p, std::size_t a, std::size_t b) {
    Parabola f;
    if (b <= a + 4 || b >= p.size()) return f;
    const cv::Point2d chord = p[b] - p[a];
    const double len = std::hypot(chord.x, chord.y);
    if (!(len > 1e-9)) return f;
    f.origin = p[a];
    f.u = chord * (1.0 / len);
    f.v = cv::Point2d(-f.u.y, f.u.x);
    std::vector<double> t, y;
    t.reserve(b - a + 1);
    y.reserve(b - a + 1);
    double tMin = 1e18, tMax = -1e18;
    for (std::size_t i = a; i <= b; ++i) {
        const cv::Point2d d = p[i] - f.origin;
        const double ti = d.dot(f.u);
        t.push_back(ti);
        y.push_back(d.dot(f.v));
        tMin = std::min(tMin, ti);
        tMax = std::max(tMax, ti);
    }
    f.tMid = 0.5 * (tMin + tMax);
    f.tHalf = std::max(1e-9, 0.5 * (tMax - tMin));
    std::vector<double> s(t.size());
    for (std::size_t k = 0; k < t.size(); ++k) s[k] = (t[k] - f.tMid) / f.tHalf;
    if (!fitQuadratic(s, y, f.A, f.B, f.C)) return f;
    double ss = 0.0;
    for (std::size_t k = 0; k < t.size(); ++k) {
        const double r = std::abs(y[k] - f.y(t[k]));
        ss += r * r;
        if (r > f.worst) {
            f.worst = r;
            f.worstAt = a + k;
        }
    }
    f.rms = std::sqrt(ss / static_cast<double>(t.size()));
    f.tFirst = t.front();
    f.tLast = t.back();
    f.ok = true;
    return f;
}

// One edge's length: the samples a..b fitted as one smooth curve, or, where
// the edge has real shape in it, as a few, split where the fit is worst.
// Short stretches are simply followed.
double edgeLength(const std::vector<cv::Point2d>& p, std::size_t a, std::size_t b,
                  const Config& cfg, int depth = 0) {
    if (b <= a) return 0.0;
    const double followed = polyLength(p, a, b);
    const double minPiece = std::max(4.0, cfg.perimeter_edge_piece_min_mm);
    if (b - a < 8 || (b - a) * kSampleMm < minPiece) return followed;
    const Parabola f = fitParabola(p, a, b);
    const double tol = std::max(0.05, cfg.perimeter_edge_tolerance_mm);
    // A parabola stands in for a circle only while it turns a little: across
    // a quarter circle it fits to within a millimetre and still reads 1% short.
    // So an edge that turns more than this either side of its chord is fitted
    // in pieces, each of which turns less.
    const double maxSlope = std::tan(toRad(15.0));
    const bool gentle = f.ok && std::abs(f.slope(f.tFirst)) <= maxSlope &&
                        std::abs(f.slope(f.tLast)) <= maxSlope;
    if (f.ok && gentle && f.rms <= tol && f.worst <= 3.0 * tol) return f.length(f.tFirst, f.tLast);
    if (depth >= 10) return followed;
    const std::size_t minSide =
        static_cast<std::size_t>(std::max(4.0, std::round(0.5 * minPiece / kSampleMm)));
    if (b - a < 2 * minSide) return followed;
    // Where the fit is worst when it is the wrong shape; in the middle when it
    // is the right shape but turns too far.
    std::size_t k = (f.ok && !(f.rms <= tol && f.worst <= 3.0 * tol)) ? f.worstAt : (a + b) / 2;
    k = std::max(a + minSide, std::min(b - minSide, k));
    return edgeLength(p, a, k, cfg, depth + 1) + edgeLength(p, k, b, cfg, depth + 1);
}

// Total least squares line through samples a..b: centroid and direction.
bool fitLine(const std::vector<cv::Point2d>& p, std::size_t a, std::size_t b, cv::Point2d& c,
             cv::Point2d& d) {
    if (b <= a || b >= p.size()) return false;
    const double n = static_cast<double>(b - a + 1);
    double sx = 0, sy = 0;
    for (std::size_t i = a; i <= b; ++i) {
        sx += p[i].x;
        sy += p[i].y;
    }
    c = cv::Point2d(sx / n, sy / n);
    double cxx = 0, cxy = 0, cyy = 0;
    for (std::size_t i = a; i <= b; ++i) {
        const double dx = p[i].x - c.x, dy = p[i].y - c.y;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    const double half = 0.5 * (cxx + cyy);
    const double diff = 0.5 * (cxx - cyy);
    const double major = half + std::sqrt(diff * diff + cxy * cxy);
    cv::Point2d v(cxy, major - cxx);
    double len = std::hypot(v.x, v.y);
    const cv::Point2d w(major - cyy, cxy);
    const double lenW = std::hypot(w.x, w.y);
    if (lenW > len) {
        v = w;
        len = lenW;
    }
    if (len < 1e-12) return false;
    d = v * (1.0 / len);
    return true;
}

// The end of an edge as a point and a direction, from the samples a..b next
// to the corner, read at the end nearest it.
//
// A straight line, unless the edge is visibly curved there. A line's direction
// is several times steadier than the end slope of a curve fitted to the same
// samples, and it matters: at an underarm the two edges meet at 45 degrees
// about 50 mm past the cloth, where a degree of direction is two millimetres
// of perimeter. A back neck really does curve into its corner, and there the
// curve is carried on along itself - but the corner is shallow, so what is
// carried on is short.
bool edgeEnd(const std::vector<cv::Point2d>& p, std::size_t a, std::size_t b, bool atEnd,
             cv::Point2d& e, cv::Point2d& d) {
    const Parabola f = fitParabola(p, a, b);
    // The fitted curve's departure from its own chord at the middle, in mm:
    // with s scaled into [-1, 1] that is simply |C|.
    if (f.ok && std::abs(f.C) > 0.4) {
        const double t = atEnd ? f.tLast : f.tFirst;
        e = f.at(t);
        cv::Point2d dir = f.u + f.v * f.slope(t);
        const double len = std::hypot(dir.x, dir.y);
        if (len > 1e-12) {
            d = dir * (1.0 / len);
            return true;
        }
    }
    cv::Point2d c;
    if (!fitLine(p, a, b, c, d)) return false;
    if ((p[b] - p[a]).dot(d) < 0.0) d = -d;
    e = c + d * ((p[atEnd ? b : a] - c).dot(d));
    return true;
}

// Carry the edges either side of a corner on until they cross.
//
// The corner occupies samples zs..ze; the edge coming in may use samples back
// to `lo`, the edge going out forward to `hi`. Refused, and the cloth followed
// instead, whenever the answer is not believable: too little edge either side
// to fit, edges too nearly parallel for their crossing to mean anything, a
// crossing behind either edge, or one further out than the reach allows.
bool crossCorner(const std::vector<cv::Point2d>& p, std::size_t lo, std::size_t zs,
                 std::size_t ze, std::size_t hi, const Config& cfg, cv::Point2d& e1,
                 cv::Point2d& x, cv::Point2d& e2, double& turnDeg) {
    const std::size_t fit = static_cast<std::size_t>(std::max(4.0, std::round(cfg.perimeter_corner_fit_mm / kSampleMm)));
    const std::size_t minFit =
        static_cast<std::size_t>(std::max(4.0, std::round(cfg.perimeter_corner_fit_min_mm / kSampleMm)));
    const std::size_t a0 = (zs > lo + fit) ? zs - fit : lo;
    const std::size_t b1 = std::min(hi, ze + fit);
    if (zs < a0 + minFit || b1 < ze + minFit) return false;

    cv::Point2d d1, d2;
    if (!edgeEnd(p, a0, zs, true, e1, d1)) return false;
    if (!edgeEnd(p, ze, b1, false, e2, d2)) return false;
    // Both along the direction of travel.
    if ((p[zs] - p[a0]).dot(d1) < 0.0) d1 = -d1;
    if ((p[b1] - p[ze]).dot(d2) < 0.0) d2 = -d2;

    const double turn = std::acos(std::max(-1.0, std::min(1.0, d1.dot(d2))));
    turnDeg = toDeg(turn);
    if (turn < toRad(cfg.perimeter_corner_min_turn_deg)) return false;

    const double cr = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(cr) < 1e-6) return false;
    const cv::Point2d w = e2 - e1;
    const double s = (w.x * d2.y - w.y * d2.x) / cr;  // along the incoming edge from e1
    const double u = (w.x * d1.y - w.y * d1.x) / cr;  // along the outgoing edge from e2
    x = e1 + d1 * s;
    // Ahead of the incoming edge and behind the outgoing one, within a
    // millimetre: a corner is where the two meet, not somewhere they pass.
    if (s < -1.0 || u > 1.0) return false;
    const double reach = std::max(0.0, cfg.perimeter_corner_reach_mm);
    if (dist(x, e1) > reach || dist(x, e2) > reach) return false;
    return true;
}

// The divider: steps of a fixed chord walked along the path, plus what is left.
double dividerLength(const std::vector<cv::Point2d>& p, double step) {
    const std::size_t n = p.size();
    if (n < 2) return 0.0;
    if (!(step > 0.0)) return polyLength(p, 0, n - 1);
    cv::Point2d cur = p[0];
    std::size_t j = 0;
    double t = 0.0;
    double total = 0.0;
    while (j + 1 < n) {
        const cv::Point2d A = p[j];
        const cv::Point2d D = p[j + 1] - p[j];
        const double a = D.dot(D);
        if (a <= 1e-18) {
            ++j;
            t = 0.0;
            continue;
        }
        const cv::Point2d AC = A - cur;
        const double b = 2.0 * D.dot(AC);
        const double c = AC.dot(AC) - step * step;
        const double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            const double s = (-b + std::sqrt(disc)) / (2.0 * a);
            if (s > t - 1e-12 && s <= 1.0) {
                cur = A + D * s;
                total += step;
                t = s;
                continue;
            }
        }
        ++j;
        t = 0.0;
    }
    return total + dist(p[n - 1], cur);
}

// Millimetres per pixel in the middle of the working area.
double meanMmPerPx(const PlaneModel& model, const RoiBox& roi) {
    const cv::Point2d mid(0.5 * (roi.x1 + roi.x2), 0.5 * (roi.y1 + roi.y2));
    const std::pair<double, double> s = model.localMmPerPx(mid);
    const double v = 0.5 * (s.first + s.second);
    return v > 1e-9 ? v : 0.137;
}

// How far a point of the picture is from each side of the working area, in
// millimetres on the mat, and where a line a given distance in from a side
// lies. The scale is read at nine places along each side and interpolated,
// because it varies across the view: the mat is a few degrees off square to
// this camera and the lens bends the corners.
class SideScale {
public:
    SideScale(const PlaneModel& model, const RoiBox& roi) : roi_(roi) {
        for (int s = 0; s < 4; ++s) {
            for (int k = 0; k < kSamples; ++k) {
                const double f = static_cast<double>(k) / (kSamples - 1);
                cv::Point2d at;
                const double x = roi.x1 + f * (roi.x2 - 1 - roi.x1);
                const double y = roi.y1 + f * (roi.y2 - 1 - roi.y1);
                switch (static_cast<PerimeterSide>(s)) {
                    case PerimeterSide::Left: at = cv::Point2d(roi.x1, y); break;
                    case PerimeterSide::Right: at = cv::Point2d(roi.x2 - 1, y); break;
                    case PerimeterSide::Top: at = cv::Point2d(x, roi.y1); break;
                    case PerimeterSide::Bottom: at = cv::Point2d(x, roi.y2 - 1); break;
                }
                const std::pair<double, double> m = model.localMmPerPx(at);
                const bool across = (s == 0 || s == 2);  // left and right are measured across x
                double v = across ? m.first : m.second;
                if (!(v > 1e-9)) v = 0.137;
                scale_[s][k] = v;
            }
        }
    }

    // Position along side s of a point: x for the top and bottom, y for the sides.
    double along(const cv::Point2d& p, PerimeterSide s) const {
        return (s == PerimeterSide::Top || s == PerimeterSide::Bottom) ? p.x : p.y;
    }

    double scaleAt(PerimeterSide s, double pos) const {
        const bool horiz = (s == PerimeterSide::Top || s == PerimeterSide::Bottom);
        const double lo = horiz ? roi_.x1 : roi_.y1;
        const double hi = horiz ? roi_.x2 - 1 : roi_.y2 - 1;
        double f = (hi > lo) ? (pos - lo) / (hi - lo) : 0.0;
        f = std::max(0.0, std::min(1.0, f)) * (kSamples - 1);
        const int k = std::min(kSamples - 2, static_cast<int>(f));
        const double r = f - k;
        const int si = static_cast<int>(s);
        return scale_[si][k] * (1.0 - r) + scale_[si][k + 1] * r;
    }

    double distancePx(const cv::Point2d& p, PerimeterSide s) const {
        switch (s) {
            case PerimeterSide::Left: return p.x - roi_.x1;
            case PerimeterSide::Right: return (roi_.x2 - 1) - p.x;
            case PerimeterSide::Top: return p.y - roi_.y1;
            case PerimeterSide::Bottom: return (roi_.y2 - 1) - p.y;
        }
        return 0.0;
    }

    double distanceMm(const cv::Point2d& p, PerimeterSide s) const {
        return distancePx(p, s) * scaleAt(s, along(p, s));
    }

    // The point `mm` in from side s at position `pos` along it.
    cv::Point2d inset(PerimeterSide s, double pos, double mm) const {
        const double px = mm / scaleAt(s, pos);
        switch (s) {
            case PerimeterSide::Left: return cv::Point2d(roi_.x1 + px, pos);
            case PerimeterSide::Right: return cv::Point2d(roi_.x2 - 1 - px, pos);
            case PerimeterSide::Top: return cv::Point2d(pos, roi_.y1 + px);
            case PerimeterSide::Bottom: return cv::Point2d(pos, roi_.y2 - 1 - px);
        }
        return cv::Point2d(pos, pos);
    }

    // A line `mm` in from side s, from end to end of it.
    std::vector<cv::Point2d> line(PerimeterSide s, double mm, int pieces = 32) const {
        const bool horiz = (s == PerimeterSide::Top || s == PerimeterSide::Bottom);
        const double lo = horiz ? roi_.x1 : roi_.y1;
        const double hi = horiz ? roi_.x2 - 1 : roi_.y2 - 1;
        std::vector<cv::Point2d> out;
        for (int k = 0; k <= pieces; ++k) out.push_back(inset(s, lo + (hi - lo) * k / pieces, mm));
        return out;
    }

private:
    static constexpr int kSamples = 9;
    RoiBox roi_;
    double scale_[4][kSamples];
};

// The clip colour as a mask. OpenCV keeps hue in half degrees.
cv::Mat clipMask(const cv::Mat& bgr, const ClipColour& c) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    const auto wrap = [](double d) {
        d = std::fmod(d, 360.0);
        return d < 0.0 ? d + 360.0 : d;
    };
    const double lo = wrap(c.hue_lo_deg) / 2.0;
    const double hi = wrap(c.hue_hi_deg) / 2.0;
    const cv::Scalar least(0, std::max(0, c.sat_min), std::max(0, c.val_min));
    cv::Mat mask;
    if (lo <= hi) {
        cv::inRange(hsv, cv::Scalar(lo, least[1], least[2]), cv::Scalar(hi, 255, 255), mask);
    } else {
        cv::Mat a, b;
        cv::inRange(hsv, cv::Scalar(lo, least[1], least[2]), cv::Scalar(180, 255, 255), a);
        cv::inRange(hsv, cv::Scalar(0, least[1], least[2]), cv::Scalar(hi, 255, 255), b);
        cv::bitwise_or(a, b, mask);
    }
    return mask;
}

const char* awayFrom(PerimeterSide s) {
    switch (s) {
        case PerimeterSide::Left: return "right";
        case PerimeterSide::Right: return "left";
        case PerimeterSide::Top: return "down";
        case PerimeterSide::Bottom: return "up";
    }
    return "";
}

const char* toward(PerimeterSide s) {
    switch (s) {
        case PerimeterSide::Left: return "left";
        case PerimeterSide::Right: return "right";
        case PerimeterSide::Top: return "up";
        case PerimeterSide::Bottom: return "down";
    }
    return "";
}

std::string sidesText(unsigned mask) {
    std::vector<std::string> names;
    for (int s = 0; s < 4; ++s) {
        if (mask & (1u << s)) names.push_back(perimeterSideName(static_cast<PerimeterSide>(s)));
    }
    std::string out;
    for (std::size_t k = 0; k < names.size(); ++k) {
        if (k) out += (k + 1 == names.size()) ? " and " : ", ";
        out += names[k];
    }
    return out;
}

// The smallest stretch of a closed outline, as a first and last index, that
// covers every index in `members` (ascending). The largest gap between members
// is what it leaves out.
bool cyclicCover(const std::vector<int>& members, int n, int& first, int& last) {
    if (members.empty() || n <= 0) return false;
    int bestGap = -1;
    std::size_t after = 0;  // index into members of the one after the biggest gap
    for (std::size_t k = 0; k < members.size(); ++k) {
        const int a = members[k];
        const int b = members[(k + 1) % members.size()];
        const int gap = (k + 1 == members.size()) ? (b + n - a) : (b - a);
        if (gap > bestGap) {
            bestGap = gap;
            after = (k + 1) % members.size();
        }
    }
    first = members[after];
    last = members[(after + members.size() - 1) % members.size()];
    return true;
}

inline int cyclicCount(int first, int last, int n) { return ((last - first) % n + n) % n + 1; }

bool cyclicContains(int first, int last, int n, int i) {
    return ((i - first) % n + n) % n <= ((last - first) % n + n) % n;
}

}  // namespace

bool parsePerimeterSide(const std::string& name, PerimeterSide& side) {
    if (name == "left") side = PerimeterSide::Left;
    else if (name == "top") side = PerimeterSide::Top;
    else if (name == "right") side = PerimeterSide::Right;
    else if (name == "bottom") side = PerimeterSide::Bottom;
    else return false;
    return true;
}

const char* perimeterSideName(PerimeterSide side) {
    switch (side) {
        case PerimeterSide::Left: return "left";
        case PerimeterSide::Top: return "top";
        case PerimeterSide::Right: return "right";
        case PerimeterSide::Bottom: return "bottom";
    }
    return "bottom";
}

ClipColour clipColourFrom(const Config& cfg) {
    ClipColour c;
    c.hue_lo_deg = cfg.perimeter_clip_hue_lo_deg;
    c.hue_hi_deg = cfg.perimeter_clip_hue_hi_deg;
    c.sat_min = cfg.perimeter_clip_sat_min;
    c.val_min = cfg.perimeter_clip_val_min;
    return c;
}

std::vector<PerimeterClip> findClips(const cv::Mat& frameBgr, const RoiBox& roi,
                                     const PlaneModel& model, const ClipColour& colour,
                                     const Config& cfg, bool refine) {
    std::vector<PerimeterClip> out;
    if (frameBgr.empty() || frameBgr.type() != CV_8UC3 || !roi.valid()) return out;
    const cv::Rect area = roi.toRect() & cv::Rect(0, 0, frameBgr.cols, frameBgr.rows);
    if (area.width < 8 || area.height < 8) return out;

    const int step = std::max(1, cfg.detect_step);
    cv::Mat small;
    cv::resize(frameBgr(area), small,
               cv::Size(std::max(1, area.width / step), std::max(1, area.height / step)), 0, 0,
               cv::INTER_AREA);
    const double sx = static_cast<double>(area.width) / small.cols;
    const double sy = static_cast<double>(area.height) / small.rows;

    cv::Mat mask = clipMask(small, colour);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    const double mmpp = meanMmPerPx(model, roi);
    const double cellMm2 = sx * sy * mmpp * mmpp;

    for (int k = 1; k < n; ++k) {
        const double areaMm2 = stats.at<int>(k, cv::CC_STAT_AREA) * cellMm2;
        if (areaMm2 < cfg.perimeter_clip_area_min_mm2 || areaMm2 > cfg.perimeter_clip_area_max_mm2) {
            continue;
        }
        const int bx = stats.at<int>(k, cv::CC_STAT_LEFT), by = stats.at<int>(k, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(k, cv::CC_STAT_WIDTH), bh = stats.at<int>(k, cv::CC_STAT_HEIGHT);
        PerimeterClip clip;
        clip.area_mm2 = areaMm2;
        clip.px = cv::Point2d((centroids.at<double>(k, 0) + 0.5) * sx - 0.5 + area.x,
                              (centroids.at<double>(k, 1) + 0.5) * sy - 0.5 + area.y);
        clip.box_px = cv::Rect(area.x + static_cast<int>(std::floor(bx * sx)),
                               area.y + static_cast<int>(std::floor(by * sy)),
                               static_cast<int>(std::ceil(bw * sx)),
                               static_cast<int>(std::ceil(bh * sy)));

        if (refine) {
            // Outlined again at full resolution, in a window round it.
            const int pad = 2 * step + 4;
            const cv::Rect win = cv::Rect(clip.box_px.x - pad, clip.box_px.y - pad,
                                          clip.box_px.width + 2 * pad, clip.box_px.height + 2 * pad) &
                                 area;
            if (win.width >= 3 && win.height >= 3) {
                cv::Mat full = clipMask(frameBgr(win), colour);
                cv::morphologyEx(full, full, cv::MORPH_OPEN, kernel);
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(full, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
                double best = 0.0;
                int which = -1;
                for (std::size_t c = 0; c < contours.size(); ++c) {
                    const double a = std::abs(cv::contourArea(contours[c]));
                    if (a > best) {
                        best = a;
                        which = static_cast<int>(c);
                    }
                }
                if (which >= 0 && best > 4.0) {
                    std::vector<cv::Point>& c = contours[static_cast<std::size_t>(which)];
                    const cv::Moments m = cv::moments(c);
                    if (std::abs(m.m00) > 1e-9) {
                        clip.px = cv::Point2d(m.m10 / m.m00 + win.x, m.m01 / m.m00 + win.y);
                    }
                    for (cv::Point& q : c) q += win.tl();
                    clip.contour_px = c;
                    clip.box_px = cv::boundingRect(c);
                    clip.area_mm2 = best * mmpp * mmpp;
                }
            }
        }
        if (clip.contour_px.empty()) {
            // The coarse outline, carried to full resolution.
            cv::Mat one = (labels == k);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(one, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (!contours.empty()) {
                for (const cv::Point& q : contours.front()) {
                    clip.contour_px.emplace_back(
                        static_cast<int>(std::lround((q.x + 0.5) * sx - 0.5 + area.x)),
                        static_cast<int>(std::lround((q.y + 0.5) * sy - 0.5 + area.y)));
                }
            }
        }
        out.push_back(clip);
    }
    return out;
}

void measurePerimeterPath(const std::vector<cv::Point2d>& pathMm, const Config& cfg,
                          PerimeterHalf& out) {
    out.fitted_mm = 0.0;
    out.traced_mm = 0.0;
    out.reach_mm = 0.0;
    out.corners = 0;
    out.crossed = 0;
    out.defects = 0;
    out.corner_list.clear();
    out.path_mm.clear();
    if (pathMm.size() < 2) return;

    out.traced_mm = dividerLength(pathMm, cfg.perimeter_trace_step_mm);

    // Working samples: the sub-pixel scatter smoothed away, then every
    // millimetre of what is left.
    const double fineMm = 0.25;
    const std::vector<cv::Point2d> fine = resampleArc(pathMm, fineMm);
    const std::vector<cv::Point2d> p = resampleArc(smoothOpen(fine, 1.0 / fineMm), kSampleMm);
    const std::size_t n = p.size();
    if (n < 24) {
        out.fitted_mm = polyLength(p, 0, n - 1);
        out.path_mm = p;
        return;
    }

    const std::vector<double> th = tangentAngles(p, 3);
    // How sharply the path is turning at each sample, over five millimetres
    // either side, in radians per millimetre.
    std::vector<double> kappa(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = i >= 5 ? i - 5 : 0;
        const std::size_t hi = std::min(n - 1, i + 5);
        if (hi > lo) kappa[i] = std::abs(th[hi] - th[lo]) / ((hi - lo) * kSampleMm);
    }
    // How far it turns across the corner window, centred on each sample.
    const std::size_t m = static_cast<std::size_t>(
        std::max(2.0, std::round(0.5 * cfg.perimeter_corner_window_mm / kSampleMm)));
    std::vector<double> turn(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t lo = i >= m ? i - m : 0;
        const std::size_t hi = std::min(n - 1, i + m);
        turn[i] = std::abs(th[hi] - th[lo]);
    }

    const double minTurn = toRad(cfg.perimeter_corner_min_turn_deg);
    const double edgeCurv = toRad(cfg.perimeter_corner_curvature_deg_per_mm);
    const std::size_t maxHalfZone = static_cast<std::size_t>(std::round(80.0 / kSampleMm));

    struct Zone {
        std::size_t s, e, peak;
    };
    std::vector<Zone> zones;
    // Where the window turns far enough there is a corner somewhere inside it.
    // The window turns by the same amount wherever it sits across one corner,
    // so it says THAT there is a corner and not where: the corner is where the
    // path curves hardest, and its extent is where it still curves hard
    // compared with that. A stretch can hold more than one corner, so each is
    // taken in turn, hardest first.
    std::vector<unsigned char> taken(n, 0);
    for (std::size_t r0 = 0; r0 < n;) {
        if (turn[r0] < minTurn) {
            ++r0;
            continue;
        }
        std::size_t r1 = r0;
        while (r1 + 1 < n && turn[r1 + 1] >= minTurn) ++r1;
        for (;;) {
            std::size_t c = n;
            for (std::size_t i = r0; i <= r1; ++i) {
                if (taken[i]) continue;
                if (c == n || kappa[i] > kappa[c]) c = i;
            }
            if (c == n || kappa[c] < edgeCurv) break;
            const double thr = std::max(0.5 * edgeCurv, 0.25 * kappa[c]);
            std::size_t s = c, e = c;
            while (s > 0 && !taken[s - 1] && kappa[s - 1] >= thr && c - s < maxHalfZone) --s;
            while (e + 1 < n && !taken[e + 1] && kappa[e + 1] >= thr && e - c < maxHalfZone) ++e;
            s = s >= 3 ? s - 3 : 0;
            e = std::min(n - 1, e + 3);
            for (std::size_t i = s; i <= e; ++i) taken[i] = 1;
            zones.push_back({s, e, c});
        }
        r0 = r1 + 1;
    }
    // In order along the path, and one where two have grown into each other.
    std::sort(zones.begin(), zones.end(), [](const Zone& a, const Zone& b) { return a.s < b.s; });
    {
        std::vector<Zone> merged;
        for (const Zone& z : zones) {
            if (!merged.empty() && z.s <= merged.back().e + 1) {
                merged.back().e = std::max(merged.back().e, z.e);
            } else {
                merged.push_back(z);
            }
        }
        zones.swap(merged);
    }

    // Each place the outline turns hard is either a corner, which the edges
    // either side of it are carried on to meet, or a defect: a notch or a spike
    // where the boundary goes somewhere and comes back - a thread, a label, a
    // printed line of the mat that the segmentation followed a few millimetres
    // into the cloth. The folded-edge steps step over those, and so does this.
    // What gives a defect away is that across it the outline ends up heading
    // the way it was going: it turns hard, and turns back, within a short run.
    struct Decision {
        bool crossed = false;
        bool defect = false;
        cv::Point2d e1, x, e2;
        double turnDeg = 0.0;
    };
    std::vector<Decision> dec(zones.size());
    for (std::size_t z = 0; z < zones.size(); ++z) {
        const std::size_t lo = z ? zones[z - 1].e : 0;
        const std::size_t hi = (z + 1 < zones.size()) ? zones[z + 1].s : n - 1;
        Decision& d = dec[z];
        if (crossCorner(p, lo, zones[z].s, zones[z].e, hi, cfg, d.e1, d.x, d.e2, d.turnDeg)) {
            // Crossing can only lengthen a corner; one that would shorten it is
            // not a corner the edges describe.
            d.crossed = dist(d.e1, d.x) + dist(d.x, d.e2) >= polyLength(p, zones[z].s, zones[z].e) - 0.5;
        }
    }
    {
        const double span = std::max(0.0, cfg.perimeter_defect_span_mm);
        std::vector<Zone> grouped;
        std::vector<Decision> groupedDec;
        for (std::size_t z = 0; z < zones.size();) {
            std::size_t g = z;
            if (!dec[z].crossed) {
                while (g + 1 < zones.size() && !dec[g + 1].crossed &&
                       (zones[g + 1].e - zones[z].s) * kSampleMm <= span) {
                    ++g;
                }
            }
            const bool shortRun = (zones[g].e - zones[z].s) * kSampleMm <= span;
            const double net = std::abs(th[zones[g].e] - th[zones[z].s]);
            if (!dec[z].crossed && shortRun && net < minTurn) {
                grouped.push_back({zones[z].s, zones[g].e, zones[z].peak});
                Decision d;
                d.defect = true;
                d.turnDeg = toDeg(net);
                groupedDec.push_back(d);
                z = g + 1;
            } else {
                grouped.push_back(zones[z]);
                groupedDec.push_back(dec[z]);
                ++z;
            }
        }
        zones.swap(grouped);
        dec.swap(groupedDec);
    }

    std::vector<cv::Point2d> path;
    path.reserve(n + 3 * zones.size());
    const auto append = [&](std::size_t a, std::size_t b) {
        for (std::size_t i = a; i <= b && i < n; ++i) {
            if (!path.empty() && dist(path.back(), p[i]) < 1e-9) continue;
            path.push_back(p[i]);
        }
    };

    double fitted = 0.0;
    std::size_t edgeStart = 0;
    int defects = 0;
    for (std::size_t z = 0; z < zones.size(); ++z) {
        const Zone& zone = zones[z];
        const Decision& d = dec[z];
        fitted += edgeLength(p, edgeStart, zone.s, cfg);
        append(edgeStart, zone.s);
        if (d.defect) {
            // Straight across, and not a corner of the garment.
            fitted += dist(p[zone.s], p[zone.e]);
            path.push_back(p[zone.e]);
            ++defects;
            edgeStart = zone.e;
            continue;
        }
        PerimeterCorner corner;
        corner.at_mm = p[zone.peak];
        corner.turn_deg = toDeg(std::abs(th[zone.e] - th[zone.s]));
        const double arc = polyLength(p, zone.s, zone.e);
        if (d.crossed) {
            const double crossedLength = dist(d.e1, d.x) + dist(d.x, d.e2);
            corner.crossed = true;
            corner.at_mm = d.x;
            corner.turn_deg = d.turnDeg;
            corner.reach_mm = crossedLength - arc;
            fitted += crossedLength;
            path.push_back(d.e1);
            path.push_back(d.x);
            path.push_back(d.e2);
            ++out.crossed;
        } else {
            fitted += arc;
            append(zone.s, zone.e);
        }
        out.reach_mm += corner.reach_mm;
        out.corner_list.push_back(corner);
        edgeStart = zone.e;
    }
    fitted += edgeLength(p, edgeStart, n - 1, cfg);
    append(edgeStart, n - 1);

    out.fitted_mm = fitted;
    out.corners = static_cast<int>(out.corner_list.size());
    out.defects = defects;
    out.path_mm = path;
}

bool measurePerimeterCapture(const std::vector<cv::Point2d>& outlinePx, double correction,
                             const cv::Point2d& centre, const cv::Mat& frameBgr,
                             const PlaneModel& model, const RoiBox& roi,
                             const ClipColour& colour, const Config& cfg, PerimeterCapture& out) {
    out = PerimeterCapture();
    const int n = static_cast<int>(outlinePx.size());
    if (n < 64) {
        out.error = "the reading carries no usable outline";
        return false;
    }
    if (!roi.valid()) {
        out.error = "there is no working area to measure in";
        return false;
    }
    PerimeterSide join = PerimeterSide::Bottom;
    parsePerimeterSide(cfg.perimeter_join_side, join);
    const std::string joinName = perimeterSideName(join);

    // Back onto the picture as it is. The outline carries the fold-height
    // correction as a scaling about the centre of the lens; the clips and the
    // border are where the camera sees them.
    const double c = correction > 1e-6 ? correction : 1.0;
    std::vector<cv::Point2d> raw(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& q = outlinePx[static_cast<std::size_t>(i)];
        raw[static_cast<std::size_t>(i)] = centre + (q - centre) * (1.0 / c);
    }

    // Which outline points lie on the border of the working area: where the
    // garment runs out of the picture, the outline runs along the border.
    const double tol = std::max(1.0, cfg.perimeter_border_touch_px);
    std::vector<unsigned> side(static_cast<std::size_t>(n), 0u);
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& q = raw[static_cast<std::size_t>(i)];
        unsigned bits = 0u;
        if (q.x <= roi.x1 + tol) bits |= 1u << static_cast<int>(PerimeterSide::Left);
        if (q.y <= roi.y1 + tol) bits |= 1u << static_cast<int>(PerimeterSide::Top);
        if (q.x >= roi.x2 - 1 - tol) bits |= 1u << static_cast<int>(PerimeterSide::Right);
        if (q.y >= roi.y2 - 1 - tol) bits |= 1u << static_cast<int>(PerimeterSide::Bottom);
        side[static_cast<std::size_t>(i)] = bits;
    }

    // The clips on the edge of the shirt, and the stretch of outline each one
    // covers: the outline bulges round a clip, because the segmentation sees
    // the clip as part of the garment.
    const std::vector<PerimeterClip> found = findClips(frameBgr, roi, model, colour, cfg, true);
    const double mmpp = meanMmPerPx(model, roi);
    const double clearPx = std::max(1.0, cfg.perimeter_clip_clear_mm / mmpp);
    const double onEdgePx = std::max(1.0, cfg.perimeter_clip_on_edge_mm / mmpp);
    std::vector<PerimeterClip> onEdge;
    std::vector<std::vector<int>> members;
    for (const PerimeterClip& clip : found) {
        if (clip.contour_px.size() < 3) continue;
        const int grow = static_cast<int>(std::ceil(clearPx)) + 2;
        const cv::Rect around(clip.box_px.x - grow, clip.box_px.y - grow, clip.box_px.width + 2 * grow,
                              clip.box_px.height + 2 * grow);
        std::vector<int> inside;
        double closest = -1e18;
        for (int i = 0; i < n; ++i) {
            const cv::Point2d& q = raw[static_cast<std::size_t>(i)];
            if (q.x < around.x || q.y < around.y || q.x > around.x + around.width ||
                q.y > around.y + around.height) {
                continue;
            }
            const double d = cv::pointPolygonTest(clip.contour_px,
                                                  cv::Point2f(static_cast<float>(q.x), static_cast<float>(q.y)),
                                                  true);
            closest = std::max(closest, d);
            if (d >= -clearPx) inside.push_back(i);
        }
        if (closest >= -onEdgePx && !inside.empty()) {
            onEdge.push_back(clip);
            members.push_back(inside);
        }
    }
    out.clips = onEdge;
    if (onEdge.empty()) {
        out.error = "no clips found on the edge of the shirt: clip the middle of the back neck and "
                    "the middle of the hem";
        return false;
    }
    if (onEdge.size() == 1) {
        out.error = "only one clip found on the edge of the shirt: the back neck and the hem each "
                    "need one";
        return false;
    }
    if (onEdge.size() > 2) {
        out.error = std::to_string(onEdge.size()) +
                    " clips found on the edge of the shirt, and a half is cut at exactly two";
        return false;
    }

    int first[2] = {0, 0}, last[2] = {0, 0};
    for (int k = 0; k < 2; ++k) {
        if (!cyclicCover(members[static_cast<std::size_t>(k)], n, first[k], last[k]) ||
            cyclicCount(first[k], last[k], n) > n / 4) {
            out.error = "a clip could not be cut cleanly out of the outline";
            return false;
        }
    }
    // Apart, with outline between them both ways round.
    for (int k = 0; k < 2; ++k) {
        const int o = 1 - k;
        if (cyclicContains(first[k], last[k], n, first[o]) ||
            cyclicContains(first[k], last[k], n, last[o])) {
            out.error = "the two clips are too close together";
            return false;
        }
    }
    const int between01 = cyclicCount(last[0], first[1], n) - 2;
    const int between10 = cyclicCount(last[1], first[0], n) - 2;
    if (between01 < 16 || between10 < 16) {
        out.error = "the two clips are too close together";
        return false;
    }

    // Where each clip cuts the outline: its centre, on the straight bridge
    // across the stretch it covered.
    cv::Point2d cut[2];
    for (int k = 0; k < 2; ++k) {
        const cv::Point2d& before = raw[static_cast<std::size_t>((first[k] - 1 + n) % n)];
        const cv::Point2d& after = raw[static_cast<std::size_t>((last[k] + 1) % n)];
        if (side[static_cast<std::size_t>((first[k] - 1 + n) % n)] ||
            side[static_cast<std::size_t>((last[k] + 1) % n)]) {
            out.error = "a clip is too close to the edge of the picture: slide the shirt until "
                        "both clips sit in the band";
            return false;
        }
        const cv::Point2d ab = after - before;
        const double len2 = ab.dot(ab);
        double t = len2 > 1e-12 ? (onEdge[static_cast<std::size_t>(k)].px - before).dot(ab) / len2 : 0.5;
        t = std::max(0.0, std::min(1.0, t));
        cut[k] = before + ab * t;
    }

    // The two ways round from one clip to the other.
    std::vector<cv::Point2d> path[2];
    unsigned touches[2] = {0u, 0u};
    for (int w = 0; w < 2; ++w) {
        const int from = (last[w] + 1) % n;
        const int to = (first[1 - w] - 1 + n) % n;
        path[w].push_back(cut[w]);
        for (int i = from;; i = (i + 1) % n) {
            path[w].push_back(raw[static_cast<std::size_t>(i)]);
            touches[w] |= side[static_cast<std::size_t>(i)];
            if (i == to) break;
        }
        path[w].push_back(cut[1 - w]);
    }

    std::vector<int> halves;
    if (!touches[0] && !touches[1]) {
        out.whole = true;
        halves = {0, 1};
    } else if (touches[0] && touches[1]) {
        out.error = "both sides of the clips run off the picture (" + sidesText(touches[0] | touches[1]) +
                    "): lay one half fully inside the view with the clips in the band, and the other "
                    "half flat past the " + joinName + " edge";
        return false;
    } else {
        halves = {touches[0] ? 1 : 0};
    }

    out.halves = static_cast<int>(halves.size());
    for (std::size_t h = 0; h < halves.size(); ++h) {
        const std::vector<cv::Point2d>& rawPath = path[static_cast<std::size_t>(halves[h])];
        std::vector<cv::Point2d> corrected(rawPath.size());
        for (std::size_t i = 0; i < rawPath.size(); ++i) {
            corrected[i] = centre + (rawPath[i] - centre) * c;
        }
        PerimeterHalf& half = out.half[h];
        measurePerimeterPath(model.pxToMm(corrected), cfg, half);

        half.path_px = model.mmToPx(half.path_mm);
        for (cv::Point2d& q : half.path_px) q = centre + (q - centre) * (1.0 / c);
        std::vector<cv::Point2d> crossedMm;
        for (const PerimeterCorner& k : half.corner_list) {
            if (k.crossed) crossedMm.push_back(k.at_mm);
        }
        half.corner_px = crossedMm.empty() ? std::vector<cv::Point2d>() : model.mmToPx(crossedMm);
        for (cv::Point2d& q : half.corner_px) q = centre + (q - centre) * (1.0 / c);

        // The two cuts in order along the join side, and where the far point
        // lies between them.
        cv::Point2d c0 = rawPath.front(), c1 = rawPath.back();
        const bool alongX = (join == PerimeterSide::Top || join == PerimeterSide::Bottom);
        if ((alongX ? c1.x < c0.x : c1.y < c0.y)) std::swap(c0, c1);
        half.cut_px[0] = c0;
        half.cut_px[1] = c1;
        const cv::Point2d axis = c1 - c0;
        const double axisLen2 = axis.dot(axis);
        double farthest = -1.0;
        half.far_u = 0.5;
        if (axisLen2 > 1e-9) {
            const double axisLen = std::sqrt(axisLen2);
            for (const cv::Point2d& q : rawPath) {
                const cv::Point2d d = q - c0;
                const double off = std::abs(d.x * axis.y - d.y * axis.x) / axisLen;
                if (off > farthest) {
                    farthest = off;
                    half.far_u = d.dot(axis) / axisLen2;
                }
            }
        }
    }

    // Two halves from one picture have to look like two halves of one shirt.
    // The other half folded back inside the view instead of laid past the
    // edge also leaves both ways round clear of the border, and what it shows
    // is not the other half's edge.
    if (out.whole) {
        const double a = out.half[0].fitted_mm, b = out.half[1].fitted_mm;
        if (!(std::min(a, b) > 0.85 * std::max(a, b))) {
            out.error = "only part of the other half is in view: lay the other half flat past the " +
                        joinName + " edge, or the whole shirt flat inside the view";
            out.whole = false;
            out.halves = 0;
            return false;
        }
    }

    out.ok = true;
    return true;
}

bool measurePerimeterCapture(const MeasureResult& result, const cv::Mat& frameBgr,
                             const PlaneModel& model, const RoiBox& roi,
                             const ClipColour& colour, const Config& cfg, PerimeterCapture& out) {
    out = PerimeterCapture();
    if (!result.calibrated) {
        out.error = "not calibrated, so there are no millimetres to measure in";
        return false;
    }
    return measurePerimeterCapture(result.outline_px, result.parallax_correction, model.centre(),
                                   frameBgr, model, roi, colour, cfg, out);
}

bool turnedEndForEnd(const PerimeterHalf& first, const PerimeterHalf& second, std::string& why) {
    const double a = first.far_u, b = second.far_u;
    const double clear = 0.15;
    if (std::abs(a - 0.5) <= clear || std::abs(b - 0.5) <= clear) return true;
    if ((a < 0.5) != (b < 0.5)) return true;
    why = "the second half lies the same way round as the first, so it may be the same half again: "
          "lift the shirt, turn it end for end so the collar is at the other end of the band, and lay "
          "the other half";
    return false;
}

PerimeterResult joinPerimeter(const PerimeterHalf& first, const PerimeterHalf& second, bool traced,
                              bool whole) {
    PerimeterResult r;
    r.ok = true;
    r.traced = traced;
    r.whole = whole;
    r.half[0] = first;
    r.half[1] = second;
    r.fitted_mm = first.fitted_mm + second.fitted_mm;
    r.traced_mm = first.traced_mm + second.traced_mm;
    r.reach_mm = first.reach_mm + second.reach_mm;
    r.corners = first.corners + second.corners;
    r.crossed = first.crossed + second.crossed;
    r.difference_mm = r.halfMm(0) - r.halfMm(1);
    return r;
}

vjson::Value PerimeterHalf::toJson() const {
    vjson::Value o = vjson::Value::object();
    o.set("fitted_mm", vjson::Value::real(fitted_mm));
    o.set("traced_mm", vjson::Value::real(traced_mm));
    o.set("reach_mm", vjson::Value::real(reach_mm));
    o.set("corners", vjson::Value::integer(corners));
    o.set("crossed", vjson::Value::integer(crossed));
    o.set("defects", vjson::Value::integer(defects));
    o.set("far_u", vjson::Value::real(far_u));
    vjson::Value list = vjson::Value::array();
    for (const PerimeterCorner& k : corner_list) {
        vjson::Value e = vjson::Value::object();
        e.set("turn_deg", vjson::Value::real(k.turn_deg));
        e.set("crossed", vjson::Value::boolean(k.crossed));
        e.set("reach_mm", vjson::Value::real(k.reach_mm));
        list.push(e);
    }
    o.set("corner_list", list);
    return o;
}

vjson::Value PerimeterCapture::toJson() const {
    vjson::Value o = vjson::Value::object();
    o.set("ok", vjson::Value::boolean(ok));
    if (!error.empty()) o.set("error", vjson::Value::str(error));
    o.set("whole", vjson::Value::boolean(whole));
    vjson::Value h = vjson::Value::array();
    for (int k = 0; k < halves && k < 2; ++k) h.push(half[k].toJson());
    o.set("halves", h);
    vjson::Value clipList = vjson::Value::array();
    for (const PerimeterClip& c : clips) {
        vjson::Value e = vjson::Value::object();
        e.set("x_px", vjson::Value::real(c.px.x));
        e.set("y_px", vjson::Value::real(c.px.y));
        e.set("area_mm2", vjson::Value::real(c.area_mm2));
        clipList.push(e);
    }
    o.set("clips", clipList);
    return o;
}

vjson::Value PerimeterResult::toJson() const {
    vjson::Value o = vjson::Value::object();
    o.set("ok", vjson::Value::boolean(ok));
    if (!error.empty()) o.set("error", vjson::Value::str(error));
    o.set("measure", vjson::Value::str(traced ? "traced" : "fitted"));
    o.set("whole", vjson::Value::boolean(whole));
    o.set("fitted_mm", vjson::Value::real(fitted_mm));
    o.set("traced_mm", vjson::Value::real(traced_mm));
    o.set("reach_mm", vjson::Value::real(reach_mm));
    o.set("difference_mm", vjson::Value::real(difference_mm));
    o.set("corners", vjson::Value::integer(corners));
    o.set("crossed", vjson::Value::integer(crossed));
    vjson::Value h = vjson::Value::array();
    h.push(half[0].toJson());
    h.push(half[1].toJson());
    o.set("halves", h);
    return o;
}

PerimeterGuideLines perimeterGuideLines(const PlaneModel& model, const RoiBox& roi,
                                        const Config& cfg) {
    PerimeterGuideLines g;
    if (!roi.valid()) return g;
    parsePerimeterSide(cfg.perimeter_join_side, g.join);
    const SideScale scale(model, roi);
    g.band_near = scale.line(g.join, cfg.perimeter_band_near_mm);
    g.band_far = scale.line(g.join, cfg.perimeter_band_far_mm);
    for (int s = 0; s < 4; ++s) {
        if (static_cast<PerimeterSide>(s) == g.join) continue;
        g.margin[s] = scale.line(static_cast<PerimeterSide>(s), cfg.perimeter_margin_mm);
    }
    return g;
}

void evaluatePerimeterGuide(const std::vector<cv::Point2d>& liveOutlinePx,
                            const cv::Mat& frameBgr, const PlaneModel& model, const RoiBox& roi,
                            const ClipColour& colour, const Config& cfg, PerimeterGuide& out) {
    out = PerimeterGuide();
    PerimeterSide join = PerimeterSide::Bottom;
    parsePerimeterSide(cfg.perimeter_join_side, join);
    const std::string joinName = perimeterSideName(join);
    if (!roi.valid()) return;
    if (liveOutlinePx.size() < 8) {
        out.message = "Lay one half of the shirt flat on the mat, face up, with a clip on the middle "
                      "of the back neck and one on the middle of the hem in the band";
        return;
    }
    out.garment = true;
    const SideScale scale(model, roi);

    double lo[4], hi[4];
    for (int s = 0; s < 4; ++s) {
        lo[s] = 1e18;
        hi[s] = -1e18;
    }
    for (const cv::Point2d& q : liveOutlinePx) {
        for (int s = 0; s < 4; ++s) {
            const PerimeterSide side = static_cast<PerimeterSide>(s);
            const double d = scale.distanceMm(q, side);
            if (side == join) {
                if (d <= 2.0) out.reaches_join = true;
                continue;
            }
            if (d < cfg.perimeter_margin_mm) {
                out.margin[s] = true;
                const double a = scale.along(q, side);
                lo[s] = std::min(lo[s], a);
                hi[s] = std::max(hi[s], a);
            }
        }
    }
    unsigned touched = 0u;
    for (int s = 0; s < 4; ++s) {
        if (!out.margin[s]) continue;
        touched |= 1u << s;
        const PerimeterSide side = static_cast<PerimeterSide>(s);
        out.touch[s] = {scale.inset(side, lo[s] - 12.0, cfg.perimeter_margin_mm),
                        scale.inset(side, hi[s] + 12.0, cfg.perimeter_margin_mm)};
    }

    // Clips near the edge of the cloth. The live outline is coarse, so this
    // is generous; the reading itself checks each clip against the refined one.
    const double mmpp = meanMmPerPx(model, roi);
    for (const PerimeterClip& clip : findClips(frameBgr, roi, model, colour, cfg, false)) {
        double best = 1e18;
        for (const cv::Point2d& q : liveOutlinePx) best = std::min(best, dist(q, clip.px));
        if (best * mmpp > 15.0) continue;
        out.clip_list.push_back(clip);
        const double d = scale.distanceMm(clip.px, join);
        const bool in = d >= cfg.perimeter_band_near_mm && d <= cfg.perimeter_band_far_mm;
        out.in_band.push_back(in);
        if (in) ++out.clips_in_band;
    }
    const std::size_t clips = out.clip_list.size();

    // What to put right first.
    const bool acrossX = out.margin[static_cast<int>(PerimeterSide::Left)] &&
                         out.margin[static_cast<int>(PerimeterSide::Right)];
    const bool acrossY = out.margin[static_cast<int>(PerimeterSide::Top)] &&
                         out.margin[static_cast<int>(PerimeterSide::Bottom)];
    if (acrossX || acrossY) {
        out.message = "The shirt is too big for the view this way: lay just one half, straight along "
                      "the " + joinName + " edge, with the clips in the band";
        return;
    }
    if (touched) {
        std::string moves;
        for (int s = 0; s < 4; ++s) {
            if (!(touched & (1u << s))) continue;
            if (!moves.empty()) moves += " and ";
            moves += awayFrom(static_cast<PerimeterSide>(s));
        }
        out.message = "Cloth is inside the " + sidesText(touched) +
                      (touched & (touched - 1u) ? " margins" : " margin") + ": slide the shirt " + moves;
        return;
    }
    if (clips == 0) {
        out.message = "No clips found on the edge of the shirt: clip the middle of the back neck and "
                      "the middle of the hem";
        return;
    }
    if (clips == 1) {
        out.message = "Only one clip found on the edge of the shirt: the back neck and the hem each "
                      "need one";
        return;
    }
    if (clips > 2) {
        out.message = std::to_string(clips) +
                      " clips found on the edge of the shirt: take off all but the two at the back "
                      "neck and the hem";
        return;
    }
    if (!out.reaches_join) {
        out.ok = true;
        out.message = "The shirt does not reach the " + joinName +
                      " edge. If the whole shirt is flat inside the view it is read in one picture; "
                      "if not, lay the other half past the " + joinName + " edge";
        return;
    }
    if (out.clips_in_band < 2) {
        bool tooFar = false;
        for (std::size_t k = 0; k < clips; ++k) {
            if (!out.in_band[k] && scale.distanceMm(out.clip_list[k].px, join) > cfg.perimeter_band_far_mm) {
                tooFar = true;
            }
        }
        out.message = std::string("Slide the shirt ") + (tooFar ? toward(join) : awayFrom(join)) +
                      " until both clips sit in the band";
        return;
    }
    out.ok = true;
    out.message = "Both clips in the band and every edge clear: hold still";
}

}  // namespace verimeasure
