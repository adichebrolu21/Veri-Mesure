#include "verimeasure/curve.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/geometry/2d.hpp>

#include "verimeasure/numeric.hpp"

namespace verimeasure {
namespace {

// How far across the corridor the seam may step between one sample and the
// next. Three quarters of a millimetre per quarter of one, which is steeper
// than any garment seam crosses its own corridor; the cost of a step, not this
// cap, is what actually shapes the path. The cap is here so the search stays
// linear in the length of the corridor.
const int kMaxStep = 3;

double dist(const cv::Point2d& a, const cv::Point2d& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Uniformly spaced points along a polyline, including both ends.
std::vector<cv::Point2d> resample(const std::vector<cv::Point2d>& in, double stepMm) {
    std::vector<cv::Point2d> out;
    if (in.size() < 2 || stepMm <= 0.0) return out;

    double total = 0.0;
    for (std::size_t k = 1; k < in.size(); ++k) total += dist(in[k - 1], in[k]);
    if (!(total > 0.0)) return out;

    const int n = std::max(1, static_cast<int>(std::lround(total / stepMm)));
    out.reserve(static_cast<std::size_t>(n) + 1);

    std::size_t seg = 0;
    double consumed = 0.0;  // arc length up to the start of `seg`
    double segLen = dist(in[0], in[1]);
    for (int k = 0; k <= n; ++k) {
        const double want = total * static_cast<double>(k) / n;
        while (seg + 2 < in.size() && want > consumed + segLen) {
            consumed += segLen;
            ++seg;
            segLen = dist(in[seg], in[seg + 1]);
        }
        const double t = segLen > 0.0 ? std::min(1.0, (want - consumed) / segLen) : 0.0;
        out.emplace_back(in[seg].x + (in[seg + 1].x - in[seg].x) * t,
                         in[seg].y + (in[seg + 1].y - in[seg].y) * t);
    }
    return out;
}

// The circle through three points. False when they are too nearly in a line for
// one to mean anything.
bool circleThrough(const cv::Point2d& a, const cv::Point2d& b, const cv::Point2d& c,
                   cv::Point2d& centre, double& radius) {
    const double d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    const double chord = dist(a, c);
    if (std::fabs(d) < 1e-9 || chord <= 0.0) return false;

    const double aa = a.x * a.x + a.y * a.y;
    const double bb = b.x * b.x + b.y * b.y;
    const double cc = c.x * c.x + c.y * c.y;
    centre.x = (aa * (b.y - c.y) + bb * (c.y - a.y) + cc * (a.y - b.y)) / d;
    centre.y = (aa * (c.x - b.x) + bb * (a.x - c.x) + cc * (b.x - a.x)) / d;
    radius = dist(centre, a);

    // A circle a hundred chords across is a straight line wearing a radius, and
    // the arc through it is numerically worse than the line it approximates.
    return radius < 100.0 * chord;
}

// Signed difference between two angles, in (-pi, pi].
double signedAngle(double from, double to) {
    double d = to - from;
    while (d > CV_PI) d -= 2.0 * CV_PI;
    while (d <= -CV_PI) d += 2.0 * CV_PI;
    return d;
}

// Running line fit over a window, evaluated at the centre of the window.
//
// A regression rather than an average: an average of a curved run is pulled
// toward the inside of the curve by the sagitta of the window, which at a 4 mm
// window on a neckline is small but is a bias, and biases here only ever point
// the same way. A line fit has no such term.
std::vector<double> runningLineFit(const std::vector<double>& y, int window) {
    const int n = static_cast<int>(y.size());
    std::vector<double> out(y.size());
    const int half = std::max(1, window / 2);
    for (int i = 0; i < n; ++i) {
        const int lo = std::max(0, i - half);
        const int hi = std::min(n - 1, i + half);
        const int m = hi - lo + 1;
        if (m < 3) {
            out[static_cast<std::size_t>(i)] = y[static_cast<std::size_t>(i)];
            continue;
        }
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (int k = lo; k <= hi; ++k) {
            const double x = k - i;  // centred, so the fit is evaluated at zero
            const double v = y[static_cast<std::size_t>(k)];
            sx += x;
            sy += v;
            sxx += x * x;
            sxy += x * v;
        }
        const double mm = static_cast<double>(m);
        const double den = mm * sxx - sx * sx;
        if (std::fabs(den) < 1e-12) {
            out[static_cast<std::size_t>(i)] = sy / mm;
            continue;
        }
        const double slope = (mm * sxy - sx * sy) / den;
        out[static_cast<std::size_t>(i)] = (sy - slope * sx) / mm;  // the line at x = 0
    }
    return out;
}

// Running QUADRATIC fit over a window, evaluated at the centre of the window.
//
// The line fit above cannot be used to smooth a traced path, and the reason is
// worth stating because it looks like it could. A hand puts down a point every
// fifteen millimetres or so, each several millimetres off the seam, so
// averaging out the hand means a window spanning several of those points -
// forty or fifty millimetres. A straight line fitted across fifty millimetres
// of a seventy-seven millimetre neckline cuts the corner by four, which is the
// curve being flattened to remove the hand that drew it.
//
// A quadratic follows curvature exactly to second order, so the same window
// costs nothing: what it removes is the shake, and what it leaves is the arc.
std::vector<double> runningQuadFit(const std::vector<double>& y, int window) {
    const int n = static_cast<int>(y.size());
    std::vector<double> out(y.size());
    const int half = std::max(2, window / 2);
    for (int i = 0; i < n; ++i) {
        const int lo = std::max(0, i - half);
        const int hi = std::min(n - 1, i + half);
        const int m = hi - lo + 1;
        if (m < 5) {
            out[static_cast<std::size_t>(i)] = y[static_cast<std::size_t>(i)];
            continue;
        }
        double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, t0 = 0, t1 = 0, t2 = 0;
        for (int k = lo; k <= hi; ++k) {
            const double x = k - i;  // centred, so the answer is the constant term
            const double v = y[static_cast<std::size_t>(k)];
            const double x2 = x * x;
            s0 += 1.0;
            s1 += x;
            s2 += x2;
            s3 += x2 * x;
            s4 += x2 * x2;
            t0 += v;
            t1 += x * v;
            t2 += x2 * v;
        }
        // Cramer on the 3x3 normal equations for a + b x + c x^2.
        const double d = s0 * (s2 * s4 - s3 * s3) - s1 * (s1 * s4 - s3 * s2) +
                         s2 * (s1 * s3 - s2 * s2);
        if (std::fabs(d) < 1e-12) {
            out[static_cast<std::size_t>(i)] = t0 / s0;
            continue;
        }
        const double da = t0 * (s2 * s4 - s3 * s3) - s1 * (t1 * s4 - s3 * t2) +
                          s2 * (t1 * s3 - s2 * t2);
        out[static_cast<std::size_t>(i)] = da / d;
    }
    return out;
}

// Bilinear read of an 8-bit gray image. Samples off the picture are reported
// rather than clamped: a corridor running past the border has not been read.
inline float sampleGray(const cv::Mat& gray, double x, double y, bool& inside) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= gray.cols || y0 + 1 >= gray.rows) {
        inside = false;
        return 0.0f;
    }
    inside = true;
    const double fx = x - x0;
    const double fy = y - y0;
    const unsigned char* r0 = gray.ptr<unsigned char>(y0);
    const unsigned char* r1 = gray.ptr<unsigned char>(y0 + 1);
    const double top = r0[x0] * (1.0 - fx) + r0[x0 + 1] * fx;
    const double bot = r1[x0] * (1.0 - fx) + r1[x0 + 1] * fx;
    return static_cast<float>(top * (1.0 - fy) + bot * fy);
}

}  // namespace

bool fitCircleRadius(const std::vector<cv::Point2d>& pts, double& radiusMm,
                     cv::Point2d& centreMm) {
    const std::size_t n = pts.size();
    if (n < 8) return false;

    double mx = 0.0, my = 0.0;
    for (const cv::Point2d& p : pts) {
        mx += p.x;
        my += p.y;
    }
    mx /= static_cast<double>(n);
    my /= static_cast<double>(n);

    // Taubin's fit. An ordinary algebraic least squares is biased inward on a
    // short arc, and a neckline is a short arc: over the 120 degrees a front
    // neck subtends it reads several percent small, consistently, which is
    // exactly the kind of error that survives averaging.
    double Mxx = 0, Myy = 0, Mxy = 0, Mxz = 0, Myz = 0, Mzz = 0, Mz = 0;
    for (const cv::Point2d& p : pts) {
        const double x = p.x - mx;
        const double y = p.y - my;
        const double z = x * x + y * y;
        Mxx += x * x;
        Myy += y * y;
        Mxy += x * y;
        Mxz += x * z;
        Myz += y * z;
        Mzz += z * z;
        Mz += z;
    }
    const double N = static_cast<double>(n);
    Mxx /= N;
    Myy /= N;
    Mxy /= N;
    Mxz /= N;
    Myz /= N;
    Mzz /= N;
    Mz /= N;

    const double covXY = Mxx * Myy - Mxy * Mxy;
    const double varZ = Mzz - Mz * Mz;
    const double A3 = 4.0 * Mz;
    const double A2 = -3.0 * Mz * Mz - Mzz;
    const double A1 = varZ * Mz + 4.0 * covXY * Mz - Mxz * Mxz - Myz * Myz;
    const double A0 =
        Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - varZ * covXY;
    const double A22 = A2 + A2;
    const double A33 = A3 + A3 + A3;

    double x = 0.0, yv = A0;
    for (int it = 0; it < 40; ++it) {
        const double dy = A1 + x * (A22 + A33 * x);
        if (std::fabs(dy) < 1e-18) break;
        const double xn = x - yv / dy;
        if (!std::isfinite(xn) || xn == x) break;
        const double yn = A0 + xn * (A1 + xn * (A2 + xn * A3));
        if (std::fabs(yn) >= std::fabs(yv)) break;
        x = xn;
        yv = yn;
    }

    const double det = x * x - x * Mz + covXY;
    if (std::fabs(det) < 1e-12) return false;
    const double cx = (Mxz * (Myy - x) - Myz * Mxy) / det / 2.0;
    const double cy = (Myz * (Mxx - x) - Mxz * Mxy) / det / 2.0;
    const double r = std::sqrt(cx * cx + cy * cy + Mz);
    if (!std::isfinite(r) || r <= 0.0) return false;

    const double chord = dist(pts.front(), pts.back());
    if (chord > 0.0 && r > 100.0 * chord) return false;  // straight, in effect

    centreMm = cv::Point2d(cx + mx, cy + my);
    radiusMm = r;
    return true;
}

bool seedPath(const std::vector<cv::Point2d>& seedsMm, double sampleMm,
              std::vector<cv::Point2d>& pathMm, double extendMm, double traceSmoothMm) {
    pathMm.clear();
    if (seedsMm.size() < 2 || sampleMm <= 0.0) return false;
    extendMm = std::max(0.0, extendMm);

    // Three taps: the arc through them, not the polyline between them.
    //
    // This is the case the interaction is built around, and it is worth being
    // explicit about why. A neckline dips seventy to ninety millimetres below
    // the chord joining its two shoulder points. A corridor built around that
    // chord, or around the two straight segments through a middle tap, contains
    // no neckline along most of its length: the snap would find nothing, or
    // worse, find the shoulder seam and measure that instead. Three points are
    // the fewest that describe a curve rather than its ends, and a garment
    // neckline is close enough to circular that the arc through three of them
    // lands within a few millimetres of the seam the whole way along, which is
    // what the corridor is sized for.
    if (seedsMm.size() == 3) {
        cv::Point2d centre;
        double radius = 0.0;
        if (circleThrough(seedsMm[0], seedsMm[1], seedsMm[2], centre, radius)) {
            const double a0 = std::atan2(seedsMm[0].y - centre.y, seedsMm[0].x - centre.x);
            const double a1 = std::atan2(seedsMm[1].y - centre.y, seedsMm[1].x - centre.x);
            const double a2 = std::atan2(seedsMm[2].y - centre.y, seedsMm[2].x - centre.x);

            // The arc that actually passes through the middle tap. Two arcs
            // join the ends and only one of them is the neckline; the other
            // goes the long way round.
            double sweep = signedAngle(a0, a2);
            const double toMid = signedAngle(a0, a1);
            if ((toMid < 0.0) != (sweep < 0.0) || std::fabs(toMid) > std::fabs(sweep)) {
                sweep += (sweep < 0.0 ? 2.0 * CV_PI : -2.0 * CV_PI);
            }

            // Carried on past both taps, around the same circle.
            const double bite = radius > 1e-9 ? extendMm / radius : 0.0;
            const double way = sweep < 0.0 ? -1.0 : 1.0;
            const double from = a0 - way * bite;
            const double span = sweep + way * 2.0 * bite;

            const int n =
                std::max(2, static_cast<int>(std::lround(std::fabs(span) * radius / sampleMm)));
            pathMm.reserve(static_cast<std::size_t>(n) + 1);
            for (int k = 0; k <= n; ++k) {
                const double a = from + span * static_cast<double>(k) / n;
                pathMm.emplace_back(centre.x + radius * std::cos(a),
                                    centre.y + radius * std::sin(a));
            }
            return pathMm.size() >= 2;
        }
        // Too straight for a circle. The polyline through them is then a
        // perfectly good description of the same thing.
    }

    pathMm = resample(seedsMm, sampleMm);
    if (pathMm.size() < 2) return false;

    // A traced path is smoothed before it becomes a corridor.
    //
    // Four or more points means a hand drew this, and a hand drew it at
    // whatever rate the camera was running: a dozen points across two hundred
    // millimetres, each carrying its own shake. Resampling that gives a
    // corridor made of straight segments meeting at kinks, and the corridor's
    // NORMAL - which is the direction every sample across the ribbon is taken
    // along - jumps at every one of them. The ribbon would be built out of
    // little fans, and a seam running smoothly through it would arrive looking
    // like a saw.
    //
    // The kinks are the cheap half. The expensive half is that the corridor is
    // built ALONG this path, so any wiggle left in it is wiggle in the ribbon's
    // own geometry - and the search that follows can move across the corridor
    // but never along it. A seam found perfectly still comes back measured on a
    // crooked ruler, longer than it is, every time and only ever longer.
    //
    // So the window is long enough to actually remove a hand rather than merely
    // to round off the corners: at 30 mm a 254 mm neckline traced with three
    // millimetres of shake read 262, and at 45 it read 254.1. It costs nothing
    // real because the fit is a quadratic - see curve_trace_smooth_mm for what
    // that buys and where the ceiling is.
    if (seedsMm.size() >= 4) {
        const int win = std::max(
            3, static_cast<int>(std::lround(std::max(0.0, traceSmoothMm) / sampleMm)) | 1);
        std::vector<double> xs(pathMm.size()), ys(pathMm.size());
        for (std::size_t k = 0; k < pathMm.size(); ++k) {
            xs[k] = pathMm[k].x;
            ys[k] = pathMm[k].y;
        }
        const std::vector<double> sx = runningQuadFit(xs, win);
        const std::vector<double> sy = runningQuadFit(ys, win);
        for (std::size_t k = 0; k < pathMm.size(); ++k) pathMm[k] = cv::Point2d(sx[k], sy[k]);
        pathMm = resample(pathMm, sampleMm);
        if (pathMm.size() < 2) return false;
    }

    // A polyline carries on straight, along the direction each end is already
    // running in.
    const int grow = static_cast<int>(std::lround(extendMm / sampleMm));
    if (grow > 0) {
        const auto unit = [](cv::Point2d d) {
            const double n = std::hypot(d.x, d.y);
            return n > 1e-9 ? cv::Point2d(d.x / n, d.y / n) : cv::Point2d(1.0, 0.0);
        };
        const cv::Point2d head =
            unit(cv::Point2d(pathMm.front().x - pathMm[1].x, pathMm.front().y - pathMm[1].y));
        const cv::Point2d tail = unit(cv::Point2d(
            pathMm.back().x - pathMm[pathMm.size() - 2].x,
            pathMm.back().y - pathMm[pathMm.size() - 2].y));
        const cv::Point2d a = pathMm.front();
        const cv::Point2d b = pathMm.back();

        std::vector<cv::Point2d> lead;
        lead.reserve(static_cast<std::size_t>(grow));
        for (int k = grow; k >= 1; --k) {
            lead.emplace_back(a.x + head.x * k * sampleMm, a.y + head.y * k * sampleMm);
        }
        lead.insert(lead.end(), pathMm.begin(), pathMm.end());
        for (int k = 1; k <= grow; ++k) {
            lead.emplace_back(b.x + tail.x * k * sampleMm, b.y + tail.y * k * sampleMm);
        }
        pathMm.swap(lead);
    }
    return pathMm.size() >= 2;
}

namespace {

// The half-angle of the circular arc of length `arc` on chord `chord`, from
// arc / chord = t / sin t, which rises from 1 at a straight line through pi/2
// at a semicircle and on without limit as the arc closes toward a circle. A
// collar laid flat is often past the semicircle - a horseshoe - so the major
// arc is allowed. False when the numbers cannot be an arc at all.
bool arcHalfAngle(double arc, double chord, double& t) {
    t = 0.0;
    if (!(chord > 1e-9) || !(arc > 0.0)) return false;
    const double ratio = arc / chord;
    if (ratio <= 1.0) return true;  // straight, or the target is shorter than the chord
    if (ratio >= 20.0) return false;  // a chord that is a tiny fraction of the arc: not a neck
    t = ratio < CV_PI / 2.0 ? 1.0 : 2.4;
    for (int it = 0; it < 80; ++it) {
        const double s = std::sin(t);
        if (std::fabs(s) < 1e-12) break;
        const double f = t / s - ratio;
        const double df = (s - t * std::cos(t)) / (s * s);
        if (std::fabs(df) < 1e-12) break;
        const double tn = t - f / df;
        if (!(tn > 1e-6) || tn >= CV_PI - 1e-6) break;
        const bool done = std::fabs(tn - t) < 1e-10;
        t = tn;
        if (done) break;
    }
    return t > 0.0;
}

}  // namespace

bool findNeckline(const std::vector<cv::Point2d>& outlineMm, const cv::Point2d& towardMark,
                  double targetArcMm, NecklineFind& out) {
    out = NecklineFind();
    const int n = static_cast<int>(outlineMm.size());
    if (n < 20) {
        out.error = "no outline to find a neckline in";
        return false;
    }
    const double tn = std::hypot(towardMark.x, towardMark.y);
    if (tn < 1e-9) {
        out.error = "which side the neck faces is not known";
        return false;
    }
    const cv::Point2d toward(towardMark.x / tn, towardMark.y / tn);

    // The hull and its defects, on integer tenths of a millimetre.
    std::vector<cv::Point> pts(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        pts[static_cast<std::size_t>(i)] =
            cv::Point(static_cast<int>(std::lround(outlineMm[static_cast<std::size_t>(i)].x * 10.0)),
                      static_cast<int>(std::lround(outlineMm[static_cast<std::size_t>(i)].y * 10.0)));
    }
    std::vector<int> hull;
    cv::convexHull(pts, hull, false, false);
    if (hull.size() < 3) {
        out.error = "the outline has no shape to it";
        return false;
    }
    std::vector<cv::Vec4i> defects;
    try {
        cv::convexityDefects(pts, hull, defects);
    } catch (const cv::Exception&) {
        out.error = "the outline crosses itself";
        return false;
    }

    // The neck is the notch nearest the mark that opens toward it.
    //
    // A flat shirt's outline has several notches: the neck, an armpit either
    // side, sometimes a scooped hem. Which is the neck cannot be told from the
    // shape alone - an armpit between a spread sleeve and the body is a notch
    // of much the same size - so the step's convention decides: the shirt is
    // laid with its neck toward the mark, the way a fold faces the mark on an
    // edge step. Among notches wide enough to be a neck, the one whose mouth
    // sits nearest that side, dipping away from it into the garment, is it.
    int best = -1;
    double bestScore = -1e30;
    for (std::size_t k = 0; k < defects.size(); ++k) {
        const cv::Vec4i& d = defects[k];
        const cv::Point2d& a = outlineMm[static_cast<std::size_t>(d[0])];
        const cv::Point2d& b = outlineMm[static_cast<std::size_t>(d[1])];
        const cv::Point2d& far = outlineMm[static_cast<std::size_t>(d[2])];
        const double depth = d[3] / 256.0 / 10.0;
        const double chord = std::hypot(b.x - a.x, b.y - a.y);
        // Wide enough to be a neck; the upper bound is loose because when the
        // shoulder line runs level into a sleeve the hull spans the whole
        // shirt and the corners are found afterwards, by walking in.
        if (chord < 60.0 || chord > 900.0 || depth < 4.0) continue;
        const cv::Point2d mid(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
        const double into = (mid.x - far.x) * toward.x + (mid.y - far.y) * toward.y;
        if (into <= 0.0) continue;  // opens away from the mark: not the neck
        const double score = mid.x * toward.x + mid.y * toward.y;
        if (score > bestScore) {
            bestScore = score;
            best = static_cast<int>(k);
        }
    }
    if (best < 0) {
        out.error = "no notch in the outline on the side the neck should face";
        return false;
    }
    const cv::Vec4i& d = defects[static_cast<std::size_t>(best)];

    // The notch: the outline from one end of the defect to the other, the way
    // round that passes through its deepest point.
    std::vector<cv::Point2d> notch;
    for (int i = d[0];; i = (i + 1) % n) {
        notch.push_back(outlineMm[static_cast<std::size_t>(i)]);
        if (i == d[1]) break;
        if (static_cast<int>(notch.size()) > n) break;
    }
    if (notch.size() < 3) {
        out.error = "the notch is too short to be a neckline";
        return false;
    }

    // The corners are where the outline LEAVES the line across the notch's
    // mouth, which is not always where the hull says the defect starts. When
    // the shoulder line runs level into the edge of a sleeve, the hull's
    // vertices are the sleeve tips, the defect spans the whole shirt, and the
    // shoulder corners are somewhere along its edge. Walking in from each end
    // while the outline is still on the line finds them either way.
    {
        const cv::Point2d a0 = notch.front();
        const cv::Point2d b0 = notch.back();
        const double c0 = std::hypot(b0.x - a0.x, b0.y - a0.y);
        if (c0 > 1e-9) {
            const cv::Point2d u((b0.x - a0.x) / c0, (b0.y - a0.y) / c0);
            const auto off = [&](const cv::Point2d& p) {
                return std::fabs((p.x - a0.x) * u.y - (p.y - a0.y) * u.x);
            };
            std::size_t lo = 0;
            std::size_t hi = notch.size() - 1;
            while (lo + 1 < hi && off(notch[lo + 1]) < 1.5) ++lo;
            while (hi > lo + 1 && off(notch[hi - 1]) < 1.5) --hi;

            // Where the outline is seen to depart is known only to that
            // tolerance, which on a gently sloping notch is several points
            // past the corner. The corner itself is where the departing edge,
            // carried back, meets the line across the mouth: the two edges'
            // intersection, which is what a corner is and what a shoulder
            // point is. Fitted over the first eight millimetres of the
            // departing edge, so one noisy point cannot tilt it.
            const int L = static_cast<int>(lo);
            const int H = static_cast<int>(hi);
            const auto cornerAt = [&](int from, int dir, const cv::Point2d& fallback) {
                int to = from;
                while (to + dir > L && to + dir < H &&
                       std::hypot(notch[static_cast<std::size_t>(to + dir)].x -
                                      notch[static_cast<std::size_t>(from)].x,
                                  notch[static_cast<std::size_t>(to + dir)].y -
                                      notch[static_cast<std::size_t>(from)].y) < 8.0) {
                    to += dir;
                }
                if (to == from) return fallback;
                const cv::Point2d p = notch[static_cast<std::size_t>(from)];
                const cv::Point2d q = notch[static_cast<std::size_t>(to)];
                const cv::Point2d v(q.x - p.x, q.y - p.y);
                const double den = v.x * u.y - v.y * u.x;
                if (std::fabs(den) < 1e-9) return fallback;
                // p + t v lies on a0 + s u where cross(p - a0 + t v, u) = 0.
                const double t = -((p.x - a0.x) * u.y - (p.y - a0.y) * u.x) / den;
                const cv::Point2d c(p.x + v.x * t, p.y + v.y * t);
                return std::hypot(c.x - fallback.x, c.y - fallback.y) < 20.0 ? c : fallback;
            };
            const cv::Point2d left = H > L + 1 ? cornerAt(L + 1, +1, notch[lo]) : notch[lo];
            const cv::Point2d right = H > L + 1 ? cornerAt(H - 1, -1, notch[hi]) : notch[hi];
            notch.assign(notch.begin() + static_cast<std::ptrdiff_t>(lo),
                         notch.begin() + static_cast<std::ptrdiff_t>(hi) + 1);
            notch.front() = left;
            notch.back() = right;
        }
    }
    out.shoulder_mm[0] = notch.front();
    out.shoulder_mm[1] = notch.back();
    out.chord_mm = std::hypot(out.shoulder_mm[1].x - out.shoulder_mm[0].x,
                              out.shoulder_mm[1].y - out.shoulder_mm[0].y);
    if (out.chord_mm < 60.0 || out.chord_mm > 400.0) {
        out.error = "the neck's corners are " + vjson::formatFixed(out.chord_mm, 0) +
                    " mm apart, which is not a neck";
        return false;
    }
    {
        const cv::Point2d& a = out.shoulder_mm[0];
        const cv::Point2d u((out.shoulder_mm[1].x - a.x) / out.chord_mm,
                            (out.shoulder_mm[1].y - a.y) / out.chord_mm);
        for (const cv::Point2d& p : notch) {
            out.silhouette_depth_mm = std::max(
                out.silhouette_depth_mm, std::fabs((p.x - a.x) * u.y - (p.y - a.y) * u.x));
        }
    }

    // How deep the seam ought to dip by the tech pack's reckoning: the
    // circular arc of the target length on this chord. Reported, and used to
    // order the corridors; never relied on, and never a reason to stop - a
    // target the arithmetic cannot place on this chord is a target that is
    // wrong for this garment, which the reading will say in its own time.
    if (targetArcMm > 0.0) {
        double t = 0.0;
        if (arcHalfAngle(targetArcMm, out.chord_mm, t) && t > 0.0) {
            const double radius = out.chord_mm / (2.0 * std::sin(t));
            out.expected_depth_mm = radius * (1.0 - std::cos(t));
        }
    }

    // The corridors, most likely first.
    //
    // Two kinds. The arc through a depth: on a shirt laid flat the back panel
    // fills the neck opening, so the outline's own dip at the neck is the
    // shallow back neck and the front neckline is an arc INSIDE it, from one
    // corner to the other, dipping some way into the garment. And the notch
    // itself, where the outline dips deep enough to be the seam - a collar
    // flattened so that its edge is the outline.
    //
    // How deep, the picture has to say. The target cannot: it is the garment
    // as designed, and it was wrong by a third on the first real shirt this
    // met, while the collar it was describing sat a hundred millimetres deep
    // between corners a hundred and fifty apart - a horseshoe, past a
    // semicircle. So the depths tried cover everything a neck does, from a
    // boat neck at a sixth of its width to a horseshoe at four fifths, twenty
    // millimetres apart, which is what the search's rigid model can bow to
    // from either side. Every line found is kept; choosing comes after.
    const cv::Point2d& a = out.shoulder_mm[0];
    const cv::Point2d& b = out.shoulder_mm[1];
    cv::Point2d nrm(-(b.y - a.y) / out.chord_mm, (b.x - a.x) / out.chord_mm);
    // Into the garment: away from the mark.
    if (nrm.x * toward.x + nrm.y * toward.y > 0.0) nrm = cv::Point2d(-nrm.x, -nrm.y);
    const auto arcAt = [&](double depth) {
        const cv::Point2d mid(0.5 * (a.x + b.x) + nrm.x * depth, 0.5 * (a.y + b.y) + nrm.y * depth);
        return std::vector<cv::Point2d>{a, mid, b};
    };
    for (double f = 0.15; f <= 0.80; f += 0.13) {
        const double depth = out.chord_mm * f;
        if (depth < 3.0) continue;
        out.candidates.push_back(arcAt(depth));
    }
    if (out.silhouette_depth_mm >= 15.0) {
        std::vector<cv::Point2d> along = resample(notch, 12.0);
        if (along.size() < 4) along = notch;
        if (along.size() >= 2) {
            // First when the target's reckoning says the notch is the seam;
            // otherwise after the arcs.
            const bool first = out.expected_depth_mm > 0.0 &&
                               std::fabs(out.silhouette_depth_mm - out.expected_depth_mm) <= 25.0;
            if (first) {
                out.candidates.insert(out.candidates.begin(), along);
            } else {
                out.candidates.push_back(along);
            }
        }
    }
    if (out.candidates.empty()) {
        out.error = "the notch is too short to be a neckline";
        return false;
    }
    out.seeds_mm = out.candidates.front();
    out.how = out.seeds_mm.size() == 3 ? "arc" : "outline";
    out.ok = true;
    return true;
}

std::vector<SeamCandidate> findSeams(const cv::Mat& frame, const PlaneModel& model,
                                     const NecklineFind& find, const Config& cfg) {
    std::vector<SeamCandidate> seams;
    if (!find.ok || find.chord_mm <= 1e-9) return seams;
    const cv::Point2d& a = find.shoulder_mm[0];
    const cv::Point2d& b = find.shoulder_mm[1];
    const cv::Point2d u((b.x - a.x) / find.chord_mm, (b.y - a.y) / find.chord_mm);

    for (const std::vector<cv::Point2d>& seeds : find.candidates) {
        CurveResult c;
        if (!snapCurve(frame, model, seeds, cfg, c)) continue;

        // How far it dips from the corners' chord, at most: the same number
        // for the same line whichever corridor found it, which is what lets
        // two corridors that landed on one line be counted once.
        double depth = 0.0;
        for (const cv::Point2d& p : c.path_mm) {
            depth = std::max(depth, std::fabs((p.x - a.x) * u.y - (p.y - a.y) * u.x));
        }
        bool same = false;
        for (SeamCandidate& s : seams) {
            if (std::fabs(s.depth_mm - depth) < 6.0) {
                same = true;
                if (c.confidence > s.curve.confidence) {
                    s.curve = c;
                    s.depth_mm = depth;
                    s.seeds_mm = seeds;
                }
                break;
            }
        }
        if (!same) seams.push_back({c, depth, seeds});
    }
    std::sort(seams.begin(), seams.end(),
              [](const SeamCandidate& l, const SeamCandidate& r) { return l.depth_mm < r.depth_mm; });
    return seams;
}

bool chooseSeam(const std::vector<SeamCandidate>& seams, double targetMm, bool alongArc,
                const std::string& prefer, std::size_t& chosen, std::string& why) {
    why.clear();
    if (seams.empty()) {
        why = "no seam any corridor could read";
        return false;
    }
    const auto lengthOf = [&](const SeamCandidate& s) {
        return alongArc ? s.curve.arc_mm : s.curve.chord_mm;
    };
    const auto list = [&](const std::vector<std::size_t>& idx) {
        std::string text;
        for (std::size_t k : idx) {
            if (!text.empty()) text += ", ";
            text += vjson::formatFixed(lengthOf(seams[k]), 0) + " mm at " +
                    vjson::formatFixed(seams[k].depth_mm, 0) + " deep";
        }
        return text;
    };

    std::vector<std::size_t> ok;
    for (std::size_t i = 0; i < seams.size(); ++i) {
        if (targetMm > 1e-9 && std::fabs(lengthOf(seams[i]) - targetMm) > 0.25 * targetMm) continue;
        ok.push_back(i);
    }
    if (ok.empty()) {
        std::vector<std::size_t> all;
        for (std::size_t i = 0; i < seams.size(); ++i) all.push_back(i);
        why = "found " + list(all) + ", none within a quarter of the " +
              vjson::formatFixed(targetMm, 0) + " mm target";
        return false;
    }
    if (ok.size() == 1) {
        chosen = ok[0];
        return true;
    }
    if (ok.size() == 2) {
        const double gap = std::fabs(seams[ok[1]].depth_mm - seams[ok[0]].depth_mm);
        if (gap >= 6.0 && gap <= 30.0) {
            // A band round an opening: its outer edge is the larger horseshoe,
            // the one that dips DEEPER from the chord across the mouth, and
            // its inner edge - the opening itself - the shallower. Seen on the
            // first real collar and got backwards once.
            const bool inner = prefer == "inner";
            chosen = inner ? ok[0] : ok[1];  // shallowest first: inner, then outer
            why = "the two edges of a band " + vjson::formatFixed(gap, 0) + " mm apart, taking the " +
                  (inner ? "inner" : "outer");
            return true;
        }
    }
    why = "found " + list(ok) + " and cannot tell which is meant";
    return false;
}

bool measurePath(const std::vector<cv::Point2d>& drawnMm, const Config& cfg, PathResult& out) {
    out = PathResult();
    out.drawn_mm = drawnMm;
    if (drawnMm.size() < 2) {
        out.error = "draw from one end to the other first";
        return false;
    }
    double drawn = 0.0;
    for (std::size_t k = 1; k < drawnMm.size(); ++k) drawn += dist(drawnMm[k - 1], drawnMm[k]);
    // Two taps on one place, or a drag that never went anywhere, is a slip and
    // not a dimension. A millimetre is well under anything a tech pack asks for.
    if (!(drawn > 1.0)) {
        out.error = "the points are in the same place";
        return false;
    }

    // The same fit the corridor centre gets from a hand-drawn trace, for the
    // same reason: two points give the straight line, three the arc through
    // them, more the drawn path with the hand taken out of it.
    const double sampleMm = std::max(0.05, cfg.curve_sample_mm);
    if (!seedPath(drawnMm, sampleMm, out.path_mm, 0.0, cfg.curve_trace_smooth_mm) ||
        out.path_mm.size() < 2) {
        out.error = "those points do not describe a path";
        return false;
    }

    out.chord_mm = dist(out.path_mm.front(), out.path_mm.back());
    for (std::size_t k = 1; k < out.path_mm.size(); ++k) {
        out.arc_mm += dist(out.path_mm[k - 1], out.path_mm[k]);
    }
    if (out.chord_mm > 1e-6) {
        const cv::Point2d a = out.path_mm.front();
        const cv::Point2d b = out.path_mm.back();
        const cv::Point2d u((b.x - a.x) / out.chord_mm, (b.y - a.y) / out.chord_mm);
        for (const cv::Point2d& p : out.path_mm) {
            out.bow_mm = std::max(out.bow_mm, std::fabs((p.x - a.x) * u.y - (p.y - a.y) * u.x));
        }
    }
    out.ok = true;
    return true;
}

vjson::Value CurveResult::toJson() const {
    vjson::Value v = vjson::Value::object();
    v.set("ok", vjson::Value::boolean(ok));
    if (!error.empty()) v.set("error", vjson::Value::str(error));
    v.set("measured_mm", vjson::Value::real(measured_mm()));
    v.set("along_arc", vjson::Value::boolean(along_arc));
    v.set("chord_mm", vjson::Value::real(chord_mm));
    v.set("arc_mm", vjson::Value::real(arc_mm));
    v.set("bow_mm", vjson::Value::real(bow_mm));
    v.set("radius_mm", vjson::Value::real(radius_mm));
    v.set("roughness_mm", vjson::Value::real(roughness_mm));
    v.set("confidence", vjson::Value::real(confidence));
    v.set("drift_mm", vjson::Value::real(drift_mm));
    v.set("margin", vjson::Value::real(margin));
    v.set("clipped_fraction", vjson::Value::real(clipped_fraction));
    vjson::Value seeds = vjson::Value::array();
    for (const cv::Point2d& p : seeds_mm) {
        vjson::Value pair = vjson::Value::array();
        pair.push(vjson::Value::real(p.x));
        pair.push(vjson::Value::real(p.y));
        seeds.push(pair);
    }
    v.set("seeds_mm", seeds);
    return v;
}

bool snapCurve(const cv::Mat& frame, const PlaneModel& model,
               const std::vector<cv::Point2d>& seedsMm, const Config& cfg, CurveResult& out) {
    out = CurveResult();
    out.seeds_mm = seedsMm;
    out.along_arc = cfg.curve_measure_along_arc;

    if (seedsMm.size() < 2) {
        out.error = "point at both ends of the curve first";
        return false;
    }
    if (frame.empty()) {
        out.error = "no picture to read the seam from";
        return false;
    }

    const double sampleMm = std::max(0.05, cfg.curve_sample_mm);

    // The corridor is built longer than the reading will be, and trimmed back
    // afterwards.
    //
    // Without this the two ends of the measurement sit wherever the arc through
    // the taps happens to stop, and that arc is fitted to taps that are off the
    // seam - so a tap displaced ACROSS the seam tilts the whole corridor and
    // slides its ends ALONG it. On a scoop neck pointed at ten millimetres off,
    // that alone moved the reading by nearly two millimetres between one
    // pointing and the next, which is most of what a half-inch band allows a
    // gauge to spend on itself, spent on nothing.
    //
    // Snapping over a longer run and then cutting back to the feet of the two
    // end taps on the seam that was found breaks the coupling: a tap that is
    // only off across the seam lands back on exactly the point it was aimed at.
    // What is left is the part no picture can fix - a tap that slid along the
    // seam - and that stays, visibly, as the operator's own.
    std::vector<cv::Point2d> seed;
    if (!seedPath(seedsMm, sampleMm, seed, cfg.curve_corridor_mm, cfg.curve_trace_smooth_mm)) {
        out.error = "those points do not describe a curve";
        return false;
    }

    // The length test is on what was pointed at, not on the scaffolding either
    // side of it.
    double seedLen = 0.0;
    {
        std::vector<cv::Point2d> plain;
        seedPath(seedsMm, sampleMm, plain, 0.0, cfg.curve_trace_smooth_mm);
        for (std::size_t k = 1; k < plain.size(); ++k) seedLen += dist(plain[k - 1], plain[k]);
    }
    if (seedLen < cfg.curve_min_length_mm) {
        out.error = "that is shorter than a curve, at " +
                    std::to_string(static_cast<int>(std::lround(seedLen))) + " mm";
        return false;
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        if (frame.depth() == CV_8U) {
            gray = frame;
        } else {
            frame.convertTo(gray, CV_8U);
        }
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    // -- the corridor, rectified --
    //
    // Sampled into a straight ribbon: one row per sample along the seed path,
    // one column per sample across it, both in millimetres. Everything after
    // this point works on a small rectangular image in which the seam runs top
    // to bottom, which is what makes the search a choice of one column per row.
    // Such a path cannot loop, cannot double back on itself and cannot leave
    // the corridor, none of which a general shortest path through the picture
    // could promise.
    const int halfW =
        std::max(2, static_cast<int>(std::lround(cfg.curve_corridor_mm / sampleMm)));
    const int W = 2 * halfW + 1;
    const int N = static_cast<int>(seed.size());

    std::vector<cv::Point2d> normal(seed.size());
    for (int i = 0; i < N; ++i) {
        const cv::Point2d& a = seed[static_cast<std::size_t>(std::max(0, i - 1))];
        const cv::Point2d& b = seed[static_cast<std::size_t>(std::min(N - 1, i + 1))];
        cv::Point2d t(b.x - a.x, b.y - a.y);
        const double len = std::hypot(t.x, t.y);
        if (len > 1e-9) {
            t.x /= len;
            t.y /= len;
        } else {
            t = cv::Point2d(1.0, 0.0);
        }
        normal[static_cast<std::size_t>(i)] = cv::Point2d(-t.y, t.x);
    }

    std::vector<cv::Point2d> wantMm;
    wantMm.reserve(static_cast<std::size_t>(N) * static_cast<std::size_t>(W));
    for (int i = 0; i < N; ++i) {
        const cv::Point2d& p = seed[static_cast<std::size_t>(i)];
        const cv::Point2d& n = normal[static_cast<std::size_t>(i)];
        for (int j = 0; j < W; ++j) {
            const double off = (j - halfW) * sampleMm;
            wantMm.emplace_back(p.x + n.x * off, p.y + n.y * off);
        }
    }
    const std::vector<cv::Point2d> wantPx = model.mmToPx(wantMm);

    cv::Mat ribbon(N, W, CV_32F);
    std::vector<int> rowOutside(static_cast<std::size_t>(N), 0);
    for (int i = 0; i < N; ++i) {
        float* row = ribbon.ptr<float>(i);
        for (int j = 0; j < W; ++j) {
            bool inside = false;
            const cv::Point2d& q = wantPx[static_cast<std::size_t>(i) * W + j];
            row[j] = sampleGray(gray, q.x, q.y, inside);
            if (!inside) ++rowOutside[static_cast<std::size_t>(i)];
        }
    }

    // Smoothed along the seam far harder than across it.
    //
    // This is the single thing that separates a seam from cloth, and it works
    // because a seam is the only thing in the corridor that RUNS. Knit texture
    // is a millimetre across and goes nowhere. A crease wanders. A speck is a
    // speck. Averaging several millimetres of the seam's own length lifts it
    // clear of all of them, while the smoothing across - where the accuracy
    // actually lives - stays at the width of the feature.
    //
    // It is only possible because the corridor was rectified first: "along"
    // here means along the seam, wherever the seam happens to be going, and not
    // along a row of the picture.
    cv::GaussianBlur(ribbon, ribbon, cv::Size(0, 0),
                     std::max(0.4, cfg.curve_blur_mm / sampleMm),
                     std::max(0.4, cfg.curve_along_mm / sampleMm));

    // Two kinds of line, because a seam is one or the other and never both.
    //
    // Where a rib meets a body the shade changes and the seam is a STEP, which
    // the first derivative across the corridor finds. Where a seam is
    // topstitched into cloth of its own colour there is no step at all, only
    // the stitching: a RIDGE, and the first derivative is exactly zero at the
    // middle of one, with a peak on either flank. A step detector pointed at a
    // stitched seam does not find it badly, it finds the two sides of it and
    // reports whichever flank happened to score - about a millimetre out, in a
    // direction that depends on nothing anyone can see.
    //
    // So the second derivative is computed as well, and it is the mirror image:
    // it peaks in the middle of a ridge and passes through zero at a step.
    //
    // Both are scaled into the picture's own units - gray levels per sample,
    // and per sample squared - because one of the two floors below is an
    // absolute one, and an absolute floor needs a unit that means something.
    cv::Mat d1, d2;
    cv::Scharr(ribbon, d1, CV_32F, 1, 0, 1.0 / 32.0);
    cv::Sobel(ribbon, d2, CV_32F, 2, 0, 3, 1.0 / 4.0);
    d1 = cv::abs(d1);
    d2 = cv::abs(d2);

    // Measured against the corridor's own noise, and saturating.
    //
    // Robust, because a topstitch or a hard fold sets the maximum of any
    // corridor it appears in, and a scale taken from that maximum makes the
    // seam beside it look like nothing. The median and the median absolute
    // deviation do not move when a few percent of the corridor is a bright
    // line.
    //
    // Saturating, because once a line is clearly a line, being brighter must
    // not make it more correct. This is the heart of the fix. A neckline has
    // four lines within a centimetre of each other, the strongest of them is
    // almost never the one the tech pack means, and nothing in the picture
    // knows which is wanted. Cap every one of them at "yes, that is a line",
    // and the choice between them falls to the only thing that does know.
    // ...and against an absolute floor as well, which is not optional.
    //
    // A scale taken only from the corridor's own noise divides by whatever
    // little noise it finds, so on smooth cloth the TAILS of a feature clear
    // four sigma just as its middle does. The evidence is then flat for a
    // couple of millimetres either side of the seam, the search has nothing to
    // choose between within that band, and the pointing places the path - which
    // is precisely the thing this whole file exists to prevent. On a rendered
    // pair of steps it did exactly that: it put the reading in the empty gap
    // between them, at full confidence, because the gap was where the operator
    // had pointed.
    //
    // So a line must also be worth at least curve_min_contrast gray levels
    // across itself, whatever the noise happens to be. A seam is five to
    // fifteen; smoothed sensor noise is well under one.
    const double featureMm = std::max(0.3, 2.0 * cfg.curve_blur_mm);
    const double floor1 = cfg.curve_min_contrast * sampleMm / featureMm;
    const double floor2 = cfg.curve_min_contrast * sampleMm * sampleMm / (featureMm * featureMm);

    const auto scaleOf = [&](cv::Mat& m, cv::Mat& unit, double floorLevel) {
        const double mid = medianOfMat(m);
        const double sigma = 1.4826 * medianAbsDevFromMat(m, mid);
        const double scale =
            std::max(std::max(0.5, cfg.curve_evidence_sigmas) * sigma, floorLevel);
        if (!(scale > 1e-12)) return false;
        unit = (m - mid) / scale;
        return true;
    };
    cv::Mat n1, n2;
    if (!scaleOf(d1, n1, floor1) || !scaleOf(d2, n2, floor2)) {
        out.error = "nothing to read in that corridor";
        return false;
    }

    cv::Mat c1 = n1.clone();
    cv::threshold(c1, c1, 0.0, 0.0, cv::THRESH_TOZERO);
    cv::threshold(c1, c1, 1.0, 1.0, cv::THRESH_TRUNC);

    // A ridge counts only where there is no step to explain it. Without that
    // gate the flanks of an ordinary step - where the second derivative peaks -
    // would outscore the step itself and pull every reading a millimetre off.
    cv::Mat gated = n2.mul(cv::Scalar::all(1.0) - c1);
    cv::Mat sharp = cv::max(n1, gated);  // kept unclamped: localisation needs the peak
    cv::threshold(sharp, sharp, 0.0, 0.0, cv::THRESH_TOZERO);

    // Compressed rather than clipped.
    //
    // Clipping at one was the obvious way to stop a brighter line being a more
    // correct line, and it went wrong in a way worth recording. A strong seam
    // clears the cap for two millimetres either side of itself, so the evidence
    // is FLAT across that whole band - and inside a flat band the only thing
    // left with an opinion is the pointing, which duly placed the reading a
    // millimetre and a half off the seam it had just found. The same flatness
    // made two positions on one line look like two different lines to the
    // ambiguity test below.
    //
    // x/(1+x) has neither problem. It flattens the difference between a strong
    // line and a merely clear one - 0.88 against 0.67, against a pointing that
    // is worth 0.69 - so the choice of line still falls to the operator. But it
    // is strictly increasing everywhere, so a line still has a peak, and the
    // peak is still exactly where the picture puts it.
    cv::Mat evid = sharp.mul(1.0 / (1.0 + sharp));

    // What the operator pointed at, as a cost.
    //
    // The other half of the fix, and what actually chooses between four
    // parallel lines. Sitting away from the middle of the corridor costs the
    // square of how far, reaching the worth of a perfect line at
    // curve_prior_mm.
    //
    // Square rather than straight because a square is flat at the bottom: near
    // the middle it has no opinion at all, so the picture alone places the seam,
    // and only further out does it begin to bite. It chooses WHICH line. It
    // does not, and must not, say where that line is - the refinement below runs
    // on the picture and never sees this.
    //
    // And capped, well under the most a line can ever score. That ceiling
    // is what keeps the prior an opinion rather than a veto: past it, every
    // position is equally discouraged, so a lone seam is still found however
    // badly it was pointed at - it is the only thing in the corridor carrying
    // any evidence at all, and evidence still outweighs the ceiling. Without
    // the cap the cost grows without limit and a tap a centimetre out puts the
    // seam beyond reach, which is a worse failure than the one the prior is
    // here to prevent.
    //
    // The working range is 0.50 to 0.60, measured: below it a lone seam ten
    // millimetres off the pointing stops being found, above it two lines four
    // millimetres apart stop being told apart. It sits in the middle of that.
    const double kPriorCap = 0.55;
    std::vector<double> prior(static_cast<std::size_t>(W));
    {
        const double pm = std::max(0.25, cfg.curve_prior_mm);
        for (int j = 0; j < W; ++j) {
            const double off = (j - halfW) * sampleMm / pm;
            prior[static_cast<std::size_t>(j)] = std::min(kPriorCap, off * off);
        }
    }

    // -- stage one: WHICH line --
    //
    // A free path that is merely penalised for turning quickly is not enough,
    // and the way it fails is worth stating because it looks like success. Such
    // a path pays for the RATE at which it moves across the corridor and never
    // for how far it has gone, so drifting one sample sideways every twentieth
    // row costs almost nothing and arrives, four hundred rows later, on an
    // entirely different line. Measured on a rendered neckline it did exactly
    // that: one reading spent 57 percent of its length on the attachment seam
    // and 36 percent on the neck opening six millimetres away, another spent 74
    // percent on the topstitch, and each came back smooth, confident and
    // wrong by a centimetre and a half of arc.
    //
    // A seam is not an arbitrary smooth path. It is ONE line. So the choice of
    // which line is made once, for the whole length, over a rigid family:
    //
    //     offset(u) = a0 + a1 u + a2 (2u^2 - 1),   u from -1 to +1 along it
    //
    // - a shift, a tilt and a bow, relative to the arc through the operator's
    // taps. That is exactly the freedom a circle fitted to three slightly wrong
    // points needs in order to become the right circle, and no more. A model
    // that spends part of its length on a second line now pays for the part
    // that does not fit, over the whole run, instead of drifting between them
    // for free.
    const double half = 0.5 * std::max(1, N - 1);
    const int stride = std::max(1, N / 400);
    // The grid is a fixed COUNT of steps across the corridor rather than a fixed
    // step in millimetres, so a wider corridor does not cube the cost of
    // searching it. That is affordable because this stage only decides WHICH
    // line: a three-quarter-millimetre grid is far finer than the distance
    // between two lines that could be confused, and the refinement below plus
    // the whole of stage two are what actually locate the seam.
    const int coarse = std::max(1, halfW / 30);

    const auto modelScore = [&](double a0, double a1, double a2) {
        double total = 0.0;
        for (int i = 0; i < N; i += stride) {
            const double u = (i - half) / half;
            const int j = halfW + static_cast<int>(std::lround(a0 + a1 * u +
                                                              a2 * (2.0 * u * u - 1.0)));
            // Off the corridor is not a candidate. Charged rather than skipped,
            // so a model that leaves cannot win by not being scored.
            if (j < 0 || j >= W) {
                total -= 4.0;
                continue;
            }
            total += evid.ptr<float>(i)[j] - prior[static_cast<std::size_t>(j)];
        }
        return total;
    };

    // Every candidate is kept, not just the winner, because the runner-up on a
    // different line has to be asked about afterwards and scoring the grid
    // twice would double the only expensive part of this file.
    struct Candidate {
        float score;
        short a0, a1, a2;
    };
    std::vector<Candidate> cands;
    cands.reserve(4096);

    double bestScore = -1e30;
    double m0 = 0.0, m1 = 0.0, m2 = 0.0;
    for (int a0 = -halfW; a0 <= halfW; a0 += coarse) {
        for (int a1 = -halfW; a1 <= halfW; a1 += coarse) {
            for (int a2 = -halfW / 2; a2 <= halfW / 2; a2 += coarse) {
                const double sc = modelScore(a0, a1, a2);
                cands.push_back({static_cast<float>(sc), static_cast<short>(a0),
                                 static_cast<short>(a1), static_cast<short>(a2)});
                if (sc > bestScore) {
                    bestScore = sc;
                    m0 = a0;
                    m1 = a1;
                    m2 = a2;
                }
            }
        }
    }
    // And again at one sample, around the winner. The coarse pass decides which
    // line; this only stops the handover to stage two starting half a
    // millimetre off it.
    for (int a0 = static_cast<int>(m0) - coarse; a0 <= static_cast<int>(m0) + coarse; ++a0) {
        for (int a1 = static_cast<int>(m1) - coarse; a1 <= static_cast<int>(m1) + coarse; ++a1) {
            for (int a2 = static_cast<int>(m2) - coarse; a2 <= static_cast<int>(m2) + coarse;
                 ++a2) {
                const double sc = modelScore(a0, a1, a2);
                if (sc > bestScore) {
                    bestScore = sc;
                    m0 = a0;
                    m1 = a1;
                    m2 = a2;
                }
            }
        }
    }

    // -- was it actually a choice? --
    //
    // Everything above assumes the pointing can say which line is wanted, and
    // sometimes it cannot. A coverstitch two millimetres outside the seam it
    // holds down is a real, strong, perfectly straight line, and a tap worth
    // three millimetres does not distinguish them - not because the search is
    // poor but because the information is not there. Measured on rendered
    // cloth, the winner in those cases was decided by which line the arc
    // through the taps happened to land nearer, which is to say by nothing.
    //
    // So the runner-up on a DIFFERENT line is scored too, and how much the
    // winner beat it by is reported and can refuse the reading. A bench that
    // says "point again, more carefully" is worth having. A bench that silently
    // measures the topstitch on one shirt and the seam on the next is not: the
    // two differ by five millimetres of arc, which is half a tolerance band
    // arriving as random noise between garments.
    {
        // Far enough apart to be two lines rather than one line seen twice.
        // That is a question about the smoothing across the corridor, which is
        // what sets how close two features can be and still be told apart, and
        // not about how far the path is allowed to wander afterwards.
        const double apart = std::max(2.5 * cfg.curve_blur_mm, 1.0) / sampleMm;

        // A rival is a model that keeps its distance from the winner over the
        // whole run, and it has to be asked that way rather than by where it
        // sits on average. A model free to tilt and bow can ride one line at
        // the ends and the other through the middle; its average position is
        // then somewhere between the two, and a rival test that only compared
        // averages would call that a different line when it is really both of
        // them, and call the genuine second line the same one.
        // The separation is between two smooth curves, so nine points along it
        // describe it as well as four hundred would.
        double rival = -1e30;
        const int kProbes = 9;
        for (const Candidate& c : cands) {
            double sep = 0.0;
            for (int k = 0; k < kProbes; ++k) {
                const double u = -1.0 + 2.0 * k / (kProbes - 1);
                sep += std::fabs((c.a0 - m0) + (c.a1 - m1) * u +
                                 (c.a2 - m2) * (2.0 * u * u - 1.0));
            }
            if (sep / kProbes < apart) continue;
            rival = std::max(rival, static_cast<double>(c.score));
        }
        const int sampled = std::max(1, (N + stride - 1) / stride);
        out.margin = rival > -1e29 ? (bestScore - rival) / sampled : 1.0;
    }

    // -- stage two: follow it --
    //
    // A real seam is not a quadratic either, so the line chosen above is a
    // starting point and not the answer. The path is now free again - but only
    // within curve_follow_mm of the line it was given, which is wide enough for
    // every wobble a garment seam actually has and far too narrow to reach the
    // next line along. Choosing and following are separated precisely so that
    // following can never quietly re-choose.
    //
    // And the pointing takes no part in this stage. It has already done its
    // work, which was to say which line; where that line lies is the picture's
    // to answer, and leaving the prior switched on here would let a tap drag
    // the reading a fraction of a millimetre off the seam it had correctly
    // chosen. Two operators pointing differently at the same seam must get the
    // same number, and this is where that is won.
    const int followCols =
        std::max(2, static_cast<int>(std::lround(cfg.curve_follow_mm / sampleMm)));
    std::vector<int> centreCol(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
        const double u = (i - half) / half;
        const int j = halfW + static_cast<int>(std::lround(m0 + m1 * u +
                                                           m2 * (2.0 * u * u - 1.0)));
        centreCol[static_cast<std::size_t>(i)] = std::max(0, std::min(W - 1, j));
    }

    const double kBarred = 1e12;
    const auto barred = [&](int i, int j) {
        return std::abs(j - centreCol[static_cast<std::size_t>(i)]) > followCols;
    };

    const double lambda = std::max(0.0, cfg.curve_smoothness);
    std::vector<double> prev(static_cast<std::size_t>(W)), cur(static_cast<std::size_t>(W));
    cv::Mat back(N, W, CV_16S);

    {
        const float* e0 = evid.ptr<float>(0);
        for (int j = 0; j < W; ++j) {
            prev[static_cast<std::size_t>(j)] = barred(0, j) ? kBarred : -e0[j];
        }
    }
    for (int i = 1; i < N; ++i) {
        const float* e = evid.ptr<float>(i);
        short* bp = back.ptr<short>(i);
        for (int j = 0; j < W; ++j) {
            double best = 1e18;
            int from = j;
            for (int d = -kMaxStep; d <= kMaxStep; ++d) {
                const int k = j + d;
                if (k < 0 || k >= W) continue;
                const double c = prev[static_cast<std::size_t>(k)] + lambda * std::abs(d);
                if (c < best) {
                    best = c;
                    from = k;
                }
            }
            cur[static_cast<std::size_t>(j)] = barred(i, j) ? kBarred : best - e[j];
            bp[j] = static_cast<short>(from);
        }
        prev.swap(cur);
    }

    int end = 0;
    for (int j = 1; j < W; ++j) {
        if (prev[static_cast<std::size_t>(j)] < prev[static_cast<std::size_t>(end)]) end = j;
    }
    std::vector<int> col(static_cast<std::size_t>(N));
    col[static_cast<std::size_t>(N - 1)] = end;
    for (int i = N - 1; i > 0; --i) {
        col[static_cast<std::size_t>(i - 1)] =
            back.ptr<short>(i)[col[static_cast<std::size_t>(i)]];
    }

    // -- where the seam sits, to better than a sample --
    //
    // The search works in whole columns; the seam does not. A parabola through
    // the response either side of the chosen column puts it where that response
    // actually peaks, which is what refineContourSubpixel does along an outline
    // normal and for the same reason: a quarter-millimetre grid is a
    // quarter-millimetre bias otherwise.
    //
    // Two things this deliberately does NOT use. Not the saturated evidence the
    // search ran on - the top of a saturated peak is flat, and a parabola
    // through a flat top is a division by nothing. And not the prior: the
    // operator chose which line, and that is the whole of their contribution.
    // Where that line lies is the picture's to say, to a fraction of a sample,
    // exactly as it would be if they had pointed perfectly.
    std::vector<double> sub(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
        const int j = col[static_cast<std::size_t>(i)];
        double refined = j;
        if (j > 0 && j + 1 < W) {
            // Whichever of the two kinds of line won here reads its own
            // response, so a step is located on the step and a ridge on the
            // ridge, rather than on the seam where the two combine.
            const float* n1r = n1.ptr<float>(i);
            const float* gr = gated.ptr<float>(i);
            const float* e = (gr[j] > n1r[j]) ? gr : n1r;
            const double den = e[j - 1] - 2.0 * e[j] + e[j + 1];
            if (den < -1e-9) {  // a peak, not a trough or a straight run
                const double shift = 0.5 * (e[j - 1] - e[j + 1]) / den;
                if (std::fabs(shift) <= 1.0) refined = j + shift;
            }
        }
        sub[static_cast<std::size_t>(i)] = refined;
    }

    const int window =
        std::max(3, static_cast<int>(std::lround(cfg.curve_smooth_mm / sampleMm)) | 1);
    const std::vector<double> smooth = runningLineFit(sub, window);

    // The seam over the whole snapped run, scaffolding included.
    std::vector<cv::Point2d> whole(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
        const double off = (smooth[static_cast<std::size_t>(i)] - halfW) * sampleMm;
        const cv::Point2d& p = seed[static_cast<std::size_t>(i)];
        const cv::Point2d& n = normal[static_cast<std::size_t>(i)];
        whole[static_cast<std::size_t>(i)] = cv::Point2d(p.x + n.x * off, p.y + n.y * off);
    }

    // -- cutting the reading back to what was pointed at --
    //
    // The end of the measurement is the foot of the operator's tap ON THE SEAM
    // THAT WAS FOUND, which is the nearest point of it to where they pointed.
    // Not the end of the corridor, which is scaffolding, and not the tap
    // itself, which is off the cloth.
    //
    // This is what makes a tap that missed across the seam cost nothing at all:
    // its foot is the same point whether it landed on the seam, ten millimetres
    // inside it or ten outside. What survives is a tap that slid along the
    // seam, and nothing in the picture can know it did.
    const auto footOf = [&whole](const cv::Point2d& q) {
        int at = 0;
        double best = dist(whole[0], q);
        for (std::size_t k = 1; k < whole.size(); ++k) {
            const double d = dist(whole[k], q);
            if (d < best) {
                best = d;
                at = static_cast<int>(k);
            }
        }
        return at;
    };
    int i0 = footOf(seedsMm.front());
    int i1 = footOf(seedsMm.back());
    if (i0 > i1) std::swap(i0, i1);
    if (i1 - i0 < 8) {
        out.error = "those points do not span enough of a seam to measure";
        return false;
    }

    const int rows = i1 - i0 + 1;
    out.path_mm.assign(whole.begin() + i0, whole.begin() + i1 + 1);

    long long outside = 0;
    double sumSq = 0.0;
    double drift = 0.0;
    std::vector<double> onPath;
    onPath.reserve(static_cast<std::size_t>(rows));
    for (int i = i0; i <= i1; ++i) {
        const double d = sub[static_cast<std::size_t>(i)] - smooth[static_cast<std::size_t>(i)];
        sumSq += d * d;
        drift = std::max(drift,
                         std::fabs((smooth[static_cast<std::size_t>(i)] - halfW) * sampleMm));
        outside += rowOutside[static_cast<std::size_t>(i)];
        onPath.push_back(sharp.ptr<float>(i)[col[static_cast<std::size_t>(i)]]);
    }
    out.roughness_mm = std::sqrt(sumSq / rows) * sampleMm;
    out.drift_mm = drift;
    out.clipped_fraction =
        static_cast<double>(outside) / (static_cast<double>(rows) * static_cast<double>(W));

    // How much of the corridor's gradient the chosen path actually carries. The
    // median rather than the mean, so a care label lying across a tenth of the
    // seam cannot make the other nine tenths look unreadable.
    // Measured on the unsaturated response on purpose.
    //
    // The search runs on evidence capped at one, because past a point a
    // brighter line must not be a more correct line. But a cap of one is also
    // what an empty corridor's worst noise reaches, so a confidence read off
    // the capped evidence says a corridor with nothing in it is a certainty.
    // Read off the response itself, in units of the corridor's own noise, an
    // empty corridor scores near zero and a seam scores many times one.
    out.confidence = std::min(1.0, std::max(0.0, median(onPath)));

    out.chord_mm = dist(out.path_mm.front(), out.path_mm.back());
    double arc = 0.0;
    for (std::size_t k = 1; k < out.path_mm.size(); ++k) {
        arc += dist(out.path_mm[k - 1], out.path_mm[k]);
    }
    out.arc_mm = arc;

    if (out.chord_mm > 1e-6) {
        const cv::Point2d a = out.path_mm.front();
        const cv::Point2d b = out.path_mm.back();
        const cv::Point2d u((b.x - a.x) / out.chord_mm, (b.y - a.y) / out.chord_mm);
        for (const cv::Point2d& p : out.path_mm) {
            out.bow_mm =
                std::max(out.bow_mm, std::fabs((p.x - a.x) * u.y - (p.y - a.y) * u.x));
        }
    }

    cv::Point2d centre;
    double radius = 0.0;
    if (fitCircleRadius(out.path_mm, radius, centre)) out.radius_mm = radius;

    // -- the guards --
    //
    // Each one, let through, gives a confidently wrong number rather than a
    // slightly worse one, and there is nothing downstream that can catch it.
    //
    // A corridor hanging off the edge of the picture returns a seam that is
    // smooth, repeatable and exactly as long as the view. That is the one
    // failure which looks precisely like a measurement, so it is refused rather
    // than reported - the same rule the folded edge works to.
    if (out.clipped_fraction > 0.02) {
        out.error = "the curve runs off the picture";
        return false;
    }
    if (out.confidence < cfg.curve_min_confidence) {
        out.error = "no seam the corridor could find, at " +
                    std::to_string(static_cast<int>(std::lround(out.confidence * 100.0))) +
                    " percent confidence";
        return false;
    }
    if (out.margin < cfg.curve_min_margin) {
        out.error = "two lines here and the pointing cannot say which - point again, "
                    "closer to the one you want";
        return false;
    }
    if (out.drift_mm > cfg.curve_max_drift_mm) {
        out.error = "the seam found is " +
                    std::to_string(static_cast<int>(std::lround(out.drift_mm))) +
                    " mm off the one that was pointed at";
        return false;
    }

    out.ok = true;
    return true;
}

}  // namespace verimeasure
