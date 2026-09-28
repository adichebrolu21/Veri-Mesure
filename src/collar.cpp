#include "verimeasure/collar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

#include "verimeasure/cv_compat.hpp"

namespace verimeasure {
namespace {

// Every walk along a line goes in steps this long. Finer than a pixel of this
// bench is wasted; much coarser starts to show in where an end lands.
constexpr double kWalkMm = 0.25;

// How far a line from the first, coarse pass may be carried on to meet the
// edge of the cloth. Generous, because the first pass is fitted to guesses
// made from the box and the ends of what it returns sit wherever those guesses
// put them. It only has to give the read that follows a guide from edge to
// edge; the read itself is held to collar_edge_reach_mm.
constexpr double kGuideReachMm = 45.0;

// Profiles across a line: how finely, and how much of each end of the line
// is left out when judging what kind of line it is. A collar line is only
// itself away from its ends, where it meets the shoulder, the back of the
// collar and the edge of the cloth.
constexpr double kProfileStepMm = 0.1;
constexpr double kProfileSkipMm = 15.0;

// The band is looked for in stretches this long along the collar, from this
// far in from either end, and this far either side of the frame line. A
// stretch is long enough for the texture of the cloth to average out of it
// and short enough that the frame line cannot turn far within it.
constexpr double kStretchMm = 20.0;
constexpr double kStretchSkipMm = 3.0;
constexpr double kBandSearchMm = 40.0;

// How wide a change of shade may be and still be an edge. The edge of a
// piece of cloth or a seam changes over about a millimetre on this bench;
// shading changes over several.
constexpr double kCrispMm = 2.0;

// A tech pack's number gates at a quarter either side of it, as chooseSeam
// does, and for the same reason: a gate on plausibility, never a preference.
constexpr double kGate = 0.25;

// A neckline dips. A line from edge to edge whose depth is under this share of
// its chord is a fold across the shirt or the back of the collar.
constexpr double kMinDepthShare = 0.12;

// Two readings of one line agree to far better than this. Two different lines
// of a collar are further apart than this everywhere.
constexpr double kSameLineMm = 1.0;

constexpr double kPiRad = 3.14159265358979323846;

// ---- small geometry ----

double dist(const cv::Point2d& a, const cv::Point2d& b) { return std::hypot(a.x - b.x, a.y - b.y); }

cv::Point2d unit(const cv::Point2d& d) {
    const double n = std::hypot(d.x, d.y);
    return n > 1e-12 ? cv::Point2d(d.x / n, d.y / n) : cv::Point2d(0.0, 0.0);
}

double lengthOf(const std::vector<cv::Point2d>& p) {
    double s = 0.0;
    for (std::size_t k = 1; k < p.size(); ++k) s += dist(p[k - 1], p[k]);
    return s;
}

// Uniformly spaced points along a polyline, both ends included.
std::vector<cv::Point2d> resampleEvery(const std::vector<cv::Point2d>& in, double stepMm) {
    std::vector<cv::Point2d> out;
    if (in.size() < 2 || stepMm <= 0.0) return out;
    const double total = lengthOf(in);
    if (!(total > 0.0)) return out;

    const int n = std::max(1, static_cast<int>(std::lround(total / stepMm)));
    out.reserve(static_cast<std::size_t>(n) + 1);
    std::size_t seg = 0;
    double consumed = 0.0;
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

// The way a polyline is running at one end, over its last `spanMm`, pointing
// off that end.
cv::Point2d endDirection(const std::vector<cv::Point2d>& p, bool atBack, double spanMm) {
    if (p.size() < 2) return cv::Point2d(0.0, 0.0);
    double run = 0.0;
    if (atBack) {
        std::size_t k = p.size() - 1;
        while (k > 0 && run < spanMm) {
            run += dist(p[k], p[k - 1]);
            --k;
        }
        return unit(p.back() - p[k]);
    }
    std::size_t k = 0;
    while (k + 1 < p.size() && run < spanMm) {
        run += dist(p[k], p[k + 1]);
        ++k;
    }
    return unit(p.front() - p[k]);
}

// Nearest distance from a point to a polyline.
double distanceToPath(const cv::Point2d& q, const std::vector<cv::Point2d>& p) {
    double best = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k + 1 < p.size(); ++k) {
        const cv::Point2d d = p[k + 1] - p[k];
        const double len2 = d.dot(d);
        double t = len2 > 1e-18 ? (q - p[k]).dot(d) / len2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        best = std::min(best, dist(q, p[k] + d * t));
    }
    if (p.size() == 1) best = dist(q, p[0]);
    return best;
}

// Left-hand normals of a uniformly sampled polyline, each from the chord
// `half` samples either side, so a sample's jitter does not swing it.
std::vector<cv::Point2d> normalsOf(const std::vector<cv::Point2d>& p, int half) {
    const int n = static_cast<int>(p.size());
    std::vector<cv::Point2d> out(p.size());
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& a = p[static_cast<std::size_t>(std::max(0, i - half))];
        const cv::Point2d& b = p[static_cast<std::size_t>(std::min(n - 1, i + half))];
        const cv::Point2d t = unit(b - a);
        out[static_cast<std::size_t>(i)] = cv::Point2d(-t.y, t.x);
    }
    return out;
}

// Bilinear read of an 8-bit gray picture. False off the picture.
inline bool grayAt(const cv::Mat& gray, double x, double y, float& v) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    if (x0 < 0 || y0 < 0 || x0 + 1 >= gray.cols || y0 + 1 >= gray.rows) return false;
    const double fx = x - x0;
    const double fy = y - y0;
    const unsigned char* r0 = gray.ptr<unsigned char>(y0);
    const unsigned char* r1 = gray.ptr<unsigned char>(y0 + 1);
    const double top = r0[x0] * (1.0 - fx) + r0[x0 + 1] * fx;
    const double bot = r1[x0] * (1.0 - fx) + r1[x0 + 1] * fx;
    v = static_cast<float>(top * (1.0 - fy) + bot * fy);
    return true;
}

double middleValue(std::vector<double> v) {
    if (v.empty()) return 0.0;
    const std::size_t m = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(m), v.end());
    double hi = v[m];
    if (v.size() % 2 == 1) return hi;
    const double lo = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(m));
    return 0.5 * (lo + hi);
}

// ---- the cloth and the mat ----
//
// Which side of the edge of the shirt a point on the mat is on, near the box.
// From the outline the reading was taken from when there is one, and from the
// picture's own brightness when there is not. Either way it is a guide - the
// ends are put on the picture afterwards - so it only has to be right to a few
// millimetres, which is what an Otsu split of pale cloth against a dark mat
// is.
class Cloth {
public:
    bool build(const cv::Mat& gray, const PlaneModel& model, const std::vector<cv::Point2d>& outlineMm,
               const cv::Rect& regionPx, std::string& why) {
        model_ = &model;
        region_ = regionPx & cv::Rect(0, 0, gray.cols, gray.rows);
        if (region_.width < 32 || region_.height < 32) {
            why = "the box is off the picture";
            return false;
        }
        mask_ = cv::Mat::zeros(region_.size(), CV_8U);

        if (outlineMm.size() >= 20) {
            const std::vector<cv::Point2d> px = model.mmToPx(outlineMm);
            std::vector<cv::Point> poly(px.size());
            for (std::size_t k = 0; k < px.size(); ++k) {
                poly[k] = cv::Point(static_cast<int>(std::lround((px[k].x - region_.x) * 16.0)),
                                    static_cast<int>(std::lround((px[k].y - region_.y) * 16.0)));
            }
            const std::vector<std::vector<cv::Point>> polys{poly};
            cv::fillPoly(mask_, polys, cv::Scalar(255), cv::LINE_8, 4);
        } else {
            const std::pair<double, double> s = model.localMmPerPx(
                cv::Point2d(region_.x + region_.width / 2.0, region_.y + region_.height / 2.0));
            const double mmPerPx = std::max(1e-3, 0.5 * (s.first + s.second));
            cv::Mat crop;
            cv::GaussianBlur(gray(region_), crop, cv::Size(0, 0), 1.5);
            cv::Mat bw;
            cv::threshold(crop, bw, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            // The mat's printed grid is bright and thin: an opening a few
            // millimetres across takes it out, and a closing puts back the
            // stitching and shadowed folds a threshold takes for mat.
            const int k = std::max(3, static_cast<int>(std::lround(3.5 / mmPerPx)) | 1);
            const cv::Mat el = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
            cv::morphologyEx(bw, bw, cv::MORPH_OPEN, el);
            cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, el);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (contours.empty()) {
                why = "no shirt in the box";
                return false;
            }
            std::size_t big = 0;
            for (std::size_t i = 1; i < contours.size(); ++i) {
                if (cv::contourArea(contours[i]) > cv::contourArea(contours[big])) big = i;
            }
            // Filled whole: the neck opening shows the back of the shirt, which
            // is cloth, and a fold that fell below the threshold is cloth too.
            cv::drawContours(mask_, contours, static_cast<int>(big), cv::Scalar(255), cv::FILLED);
        }
        return true;
    }

    // 1 on the cloth, 0 on the mat, -1 outside what was looked at.
    signed char atPx(const cv::Point2d& p) const {
        const int x = static_cast<int>(std::floor(p.x)) - region_.x;
        const int y = static_cast<int>(std::floor(p.y)) - region_.y;
        if (x < 0 || y < 0 || x >= mask_.cols || y >= mask_.rows) return -1;
        return mask_.at<unsigned char>(y, x) ? 1 : 0;
    }

    std::vector<signed char> at(const std::vector<cv::Point2d>& mm) const {
        const std::vector<cv::Point2d> px = model_->mmToPx(mm);
        std::vector<signed char> out(px.size());
        for (std::size_t k = 0; k < px.size(); ++k) out[k] = atPx(px[k]);
        return out;
    }

    // The edge of the cloth from `a` to `b`, both within `nearPx` of it, in
    // picture pixels: the way round the cloth's boundary that never runs along
    // the edge of the region looked at, and the shorter if both do not.
    // Between two shoulder points that is the back of the neck; the other way
    // is the rest of the shirt, and it leaves the region. Empty when there is
    // no such way.
    std::vector<cv::Point2d> edgeBetween(const cv::Point2d& a, const cv::Point2d& b, double nearPx) const {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask_.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        const cv::Point2d o(region_.x, region_.y);
        int which = -1, ia = -1, ib = -1;
        double best = std::numeric_limits<double>::max();
        for (std::size_t c = 0; c < contours.size(); ++c) {
            int na = -1, nb = -1;
            double da = nearPx, db = nearPx;
            for (std::size_t k = 0; k < contours[c].size(); ++k) {
                const cv::Point2d q = cv::Point2d(contours[c][k]) + o;
                const double qa = dist(q, a), qb = dist(q, b);
                if (qa <= da) {
                    da = qa;
                    na = static_cast<int>(k);
                }
                if (qb <= db) {
                    db = qb;
                    nb = static_cast<int>(k);
                }
            }
            if (na >= 0 && nb >= 0 && na != nb && da + db < best) {
                best = da + db;
                which = static_cast<int>(c);
                ia = na;
                ib = nb;
            }
        }
        if (which < 0) return {};
        const std::vector<cv::Point>& c = contours[static_cast<std::size_t>(which)];
        const int n = static_cast<int>(c.size());
        const auto onBorder = [&](const cv::Point& p) {
            return p.x <= 0 || p.y <= 0 || p.x >= mask_.cols - 1 || p.y >= mask_.rows - 1;
        };
        std::vector<cv::Point2d> way[2];
        bool clean[2] = {true, true};
        for (int w = 0; w < 2; ++w) {
            const int from = w == 0 ? ia : ib;
            const int to = w == 0 ? ib : ia;
            for (int k = from;; k = (k + 1) % n) {
                if (onBorder(c[static_cast<std::size_t>(k)])) clean[w] = false;
                way[w].push_back(cv::Point2d(c[static_cast<std::size_t>(k)]) + o);
                if (k == to) break;
            }
        }
        std::reverse(way[1].begin(), way[1].end());  // both run from a to b
        const auto length = [](const std::vector<cv::Point2d>& p) { return lengthOf(p); };
        if (clean[0] && (!clean[1] || length(way[0]) <= length(way[1]))) return way[0];
        if (clean[1]) return way[1];
        return {};
    }

private:
    const PlaneModel* model_ = nullptr;
    cv::Rect region_;
    cv::Mat mask_;
};

// ---- a dark garment ----
//
// The floors every line is held to - two gray levels a millimetre for an edge
// (marksOf), the snap's curve_min_contrast, two and a half for a seam's groove
// - are in gray levels, and on a black shirt the collar's edges are one or two
// levels deep. So a cloth that dark is read from the picture scaled until the
// cloth sits at collar_dark_level. Scaled from the frame's own colour before
// it was rounded to whole levels, so what comes up is the shades the rounding
// took out rather than steps of one level stretched apart. A scale changes
// nothing that is judged against the picture's own noise; only the floors
// move, to where they are on a cloth that bright. Over the region looked at
// only: nothing is read outside it.
//
// `gray` is replaced only when the cloth in the box is darker than
// collar_dark_below, and is otherwise left exactly as it was.
void liftDarkCloth(const cv::Mat& frame, const Cloth& cloth, const cv::Rect2d& box, const cv::Rect& region,
                   const Config& cfg, cv::Mat& gray, std::vector<std::string>& notes) {
    const double stride = std::max(1.0, std::min(box.width, box.height) / 200.0);
    std::vector<cv::Point> at;
    std::vector<double> level8;
    for (double y = box.y + 0.5; y < box.y + box.height; y += stride) {
        for (double x = box.x + 0.5; x < box.x + box.width; x += stride) {
            const cv::Point q(static_cast<int>(x), static_cast<int>(y));
            if (q.x >= gray.cols || q.y >= gray.rows || cloth.atPx(cv::Point2d(x, y)) != 1) continue;
            at.push_back(q);
            level8.push_back(gray.at<unsigned char>(q.y, q.x));
        }
    }
    if (at.empty() || middleValue(level8) >= cfg.collar_dark_below) return;

    const cv::Rect r = region & cv::Rect(0, 0, gray.cols, gray.rows);
    cv::Mat f;
    frame(r).convertTo(f, CV_32F);
    if (f.channels() == 3) cv::cvtColor(f, f, cv::COLOR_BGR2GRAY);
    if (f.channels() == 4) cv::cvtColor(f, f, cv::COLOR_BGRA2GRAY);
    std::vector<double> fine;
    fine.reserve(at.size());
    for (const cv::Point& q : at) {
        if (r.contains(q)) fine.push_back(f.at<float>(q.y - r.y, q.x - r.x));
    }
    if (fine.empty()) return;
    const double level = middleValue(fine);
    const double k = cfg.collar_dark_level / std::max(0.5, level);
    cv::Mat lifted = gray.clone();
    cv::Mat inside = lifted(r);
    f.convertTo(inside, CV_8U, k);
    gray = lifted;
    notes.push_back("a dark cloth, " + vjson::formatFixed(level, 1) + " gray levels in the box: read scaled by " +
                    vjson::formatFixed(k, 1));
}

// ---- carrying a line to the edge of the cloth ----

struct Reached {
    std::vector<cv::Point2d> path;  // from where it leaves the cloth to where it leaves it again
    bool meets[2] = {false, false};  // front, back
};

// Walk a line out from its middle to where it leaves the cloth at each end, or
// carry it on, the way it is going, for up to `reachMm` to get there.
//
// From the middle, because the middle of a collar line is the one part of it
// that is certainly on the shirt: the front of the neck, deep in the cloth. The
// first place the walk leaves the cloth is the edge of the shirt, which is
// where the collar's seam ends.
Reached reachEdge(const std::vector<cv::Point2d>& in, const Cloth& cloth, double reachMm) {
    Reached r;
    const std::vector<cv::Point2d> p = resampleEvery(in, kWalkMm);
    const int n = static_cast<int>(p.size());
    if (n < 5) return r;
    const std::vector<signed char> side = cloth.at(p);
    const int mid = n / 2;
    if (side[static_cast<std::size_t>(mid)] != 1) {
        r.path = p;
        return r;
    }
    int lo = mid, hi = mid;
    while (hi + 1 < n && side[static_cast<std::size_t>(hi + 1)] == 1) ++hi;
    while (lo - 1 >= 0 && side[static_cast<std::size_t>(lo - 1)] == 1) --lo;
    std::vector<cv::Point2d> out(p.begin() + lo, p.begin() + hi + 1);

    const auto finish = [&](bool back) {
        const int next = back ? hi + 1 : lo - 1;
        if (next >= 0 && next < n) {
            // It left the region looked at before it left the cloth: nothing
            // can be said about where it ends.
            if (side[static_cast<std::size_t>(next)] != 0) return false;
            const cv::Point2d a = back ? out.back() : out.front();
            const cv::Point2d c = (a + p[static_cast<std::size_t>(next)]) * 0.5;
            if (back) {
                out.push_back(c);
            } else {
                out.insert(out.begin(), c);
            }
            return true;
        }
        // It ended on the cloth. Carry it on the way it was going.
        const cv::Point2d d = endDirection(out, back, 5.0);
        if (d.x == 0.0 && d.y == 0.0) return false;
        const cv::Point2d e = back ? out.back() : out.front();
        std::vector<cv::Point2d> ray;
        for (double s = kWalkMm; s <= reachMm + 1e-9; s += kWalkMm) ray.push_back(e + d * s);
        const std::vector<signed char> rs = cloth.at(ray);
        std::vector<cv::Point2d> ext;
        for (std::size_t k = 0; k < ray.size(); ++k) {
            if (rs[k] == 1) {
                ext.push_back(ray[k]);
                continue;
            }
            if (rs[k] != 0) return false;
            ext.push_back(((k ? ray[k - 1] : e) + ray[k]) * 0.5);
            if (back) {
                out.insert(out.end(), ext.begin(), ext.end());
            } else {
                out.insert(out.begin(), ext.rbegin(), ext.rend());
            }
            return true;
        }
        return false;
    };
    r.meets[1] = finish(true);
    r.meets[0] = finish(false);
    r.path.swap(out);
    return r;
}

// Least squares a + b t + c t^2 through (t, v). False when the points cannot
// hold a quadratic.
bool fitQuadratic(const std::vector<double>& t, const std::vector<double>& v, double& a, double& b, double& c) {
    const std::size_t n = t.size();
    if (n < 5 || v.size() != n) return false;
    double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, r0 = 0, r1 = 0, r2 = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double x = t[k], x2 = x * x;
        s0 += 1.0;
        s1 += x;
        s2 += x2;
        s3 += x2 * x;
        s4 += x2 * x2;
        r0 += v[k];
        r1 += x * v[k];
        r2 += x2 * v[k];
    }
    const cv::Matx33d A(s0, s1, s2, s1, s2, s3, s2, s3, s4);
    const cv::Vec3d rhs(r0, r1, r2);
    cv::Vec3d sol;
    if (!cv::solve(A, rhs, sol, cv::DECOMP_CHOLESKY)) return false;
    a = sol[0];
    b = sol[1];
    c = sol[2];
    return true;
}

// Carry a line on past both ends along the curve it is already following - a
// quadratic fitted to its last `fitMm` - until it leaves the cloth, or for
// `maxMm` at most. The part of the line it is given is kept as it is.
//
// Along its own curve, and not in the direction of anything else, because
// what lies past the end of a guide is exactly the part nothing has been
// found on yet: carrying a collar's edge on the way a frame line happened to
// run there put the shoulder point wherever that line went.
std::vector<cv::Point2d> extendByCurve(const std::vector<cv::Point2d>& in, const Cloth& cloth, double fitMm,
                                       double maxMm, bool meets[2], std::size_t added[2]) {
    meets[0] = meets[1] = false;
    added[0] = added[1] = 0;
    std::vector<cv::Point2d> p = resampleEvery(in, kWalkMm);
    if (p.size() < 8) return p;
    std::vector<cv::Point2d> ext[2];
    for (int end = 0; end < 2; ++end) {
        // Arc length measured from this end, into the line.
        std::vector<double> t, xs, ys;
        double s = 0.0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const std::size_t k = end == 0 ? i : p.size() - 1 - i;
            if (i > 0) s += dist(p[k], p[end == 0 ? k - 1 : k + 1]);
            if (s > fitMm) break;
            t.push_back(s);
            xs.push_back(p[k].x);
            ys.push_back(p[k].y);
        }
        double ax, bx, cx, ay, by, cy;
        if (!fitQuadratic(t, xs, ax, bx, cx) || !fitQuadratic(t, ys, ay, by, cy)) continue;
        std::vector<cv::Point2d> ray;
        for (double u = -kWalkMm; u >= -maxMm - 1e-9; u -= kWalkMm) {
            ray.emplace_back(ax + bx * u + cx * u * u, ay + by * u + cy * u * u);
        }
        const std::vector<signed char> side = cloth.at(ray);
        for (std::size_t k = 0; k < ray.size(); ++k) {
            if (side[k] == 1) {
                ext[end].push_back(ray[k]);
                continue;
            }
            if (side[k] == 0) {
                ext[end].push_back(ray[k]);
                meets[end] = true;
            }
            break;
        }
    }
    std::vector<cv::Point2d> out(ext[0].rbegin(), ext[0].rend());
    out.insert(out.end(), p.begin(), p.end());
    out.insert(out.end(), ext[1].begin(), ext[1].end());
    added[0] = ext[0].size();
    added[1] = ext[1].size();
    return out;
}

// Where a line really leaves the cloth, on the picture.
//
// Sampled along the line's own direction across the outline's crossing and
// averaged across it over a couple of millimetres, so the shade along the line
// is read and not the texture beside it. The fall from cloth to mat is found
// to a fraction of a sample. It is by far the strongest change of shade near
// the end of a collar line on a pale shirt on a dark mat - which is also why
// a glow pushing the outline out onto the mat does not move it.
//
// `out` is the line's direction, pointing off the cloth. False, leaving the
// crossing where it was, when the cloth and the mat cannot be told apart here
// - a dark shirt on this mat - and then the outline is the best there is.
bool edgeOnPicture(const cv::Mat& gray, const PlaneModel& model, const cv::Point2d& at,
                   const cv::Point2d& out, double searchMm, cv::Point2d& edge) {
    const double lo = -(searchMm + 4.0);
    const double hi = searchMm + 4.0;
    const int n = static_cast<int>(std::lround((hi - lo) / kProfileStepMm)) + 1;
    const cv::Point2d across(-out.y, out.x);
    const double spread[] = {-1.0, -0.5, 0.0, 0.5, 1.0};
    std::vector<cv::Point2d> mm;
    mm.reserve(static_cast<std::size_t>(n) * 5);
    for (int i = 0; i < n; ++i) {
        const double s = lo + i * kProfileStepMm;
        for (double t : spread) mm.push_back(at + out * s + across * t);
    }
    const std::vector<cv::Point2d> px = model.mmToPx(mm);
    std::vector<double> prof(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        double sum = 0.0;
        int count = 0;
        for (int t = 0; t < 5; ++t) {
            float v = 0.0f;
            const cv::Point2d& q = px[static_cast<std::size_t>(i) * 5 + t];
            if (grayAt(gray, q.x, q.y, v)) {
                sum += v;
                ++count;
            }
        }
        if (count == 0) return false;
        prof[static_cast<std::size_t>(i)] = sum / count;
    }
    // A third of a millimetre of smoothing along the line: a stitch across the
    // collar at its very end must not outscore the edge of the cloth.
    {
        cv::Mat m(1, n, CV_64F, prof.data());
        cv::GaussianBlur(m, m, cv::Size(0, 0), 0.3 / kProfileStepMm, 0.0, cv::BORDER_REPLICATE);
    }
    const int four = static_cast<int>(std::lround(4.0 / kProfileStepMm));
    double cloth = 0.0, mat = 0.0;
    for (int i = 0; i < four; ++i) {
        cloth += prof[static_cast<std::size_t>(i)];
        mat += prof[static_cast<std::size_t>(n - 1 - i)];
    }
    cloth /= four;
    mat /= four;
    // Under ten gray levels between the shirt and the mat there is no edge on
    // this picture worth trusting over the outline.
    if (std::fabs(cloth - mat) < 10.0) return false;
    const double sign = cloth > mat ? 1.0 : -1.0;

    int best = -1;
    double bestFall = 0.0;
    for (int i = four; i < n - four; ++i) {
        if (i < 1 || i + 1 >= n) continue;
        const double fall = -sign * (prof[static_cast<std::size_t>(i + 1)] -
                                     prof[static_cast<std::size_t>(i - 1)]);
        if (fall > bestFall) {
            bestFall = fall;
            best = i;
        }
    }
    if (best < 1 || best + 1 >= n) return false;
    const auto fallAt = [&](int i) {
        return -sign * (prof[static_cast<std::size_t>(i + 1)] - prof[static_cast<std::size_t>(i - 1)]);
    };
    double shift = 0.0;
    if (best >= 2 && best + 2 < n) {
        const double a = fallAt(best - 1), b = fallAt(best), c = fallAt(best + 1);
        const double den = a - 2.0 * b + c;
        if (den < -1e-12) shift = std::max(-1.0, std::min(1.0, 0.5 * (a - c) / den));
    }
    edge = at + out * (lo + (best + shift) * kProfileStepMm);
    return true;
}

// Where one end of a line meets the edge of the cloth, and how much of the
// line to keep before it.
//
// The last centimetre of the line, less its final two millimetres, fitted
// straight and carried on to where the picture shows the cloth ending. Less
// the final two millimetres because that is where a seam's groove runs out
// into the edge of the cloth and stops being a line: followed to the end, the
// read curled a couple of millimetres sideways there on one box and not on
// another, and the shoulder point moved along the edge by as much. A seam is
// straight over a centimetre; the curl was the picture running out of seam.
//
// `p` is the line walked to the edge, sampled every kWalkMm. False when the
// straightened end does not reach the edge of the cloth.
bool straightEnd(const std::vector<cv::Point2d>& p, bool atBack, const cv::Mat& gray,
                 const PlaneModel& model, const Cloth& cloth, const Config& cfg, cv::Point2d& end,
                 std::size_t& keep) {
    const int n = static_cast<int>(p.size());
    const int dropN = static_cast<int>(std::lround(2.0 / kWalkMm));
    const int fitN = static_cast<int>(std::lround(10.0 / kWalkMm));
    if (n < fitN + 8) return false;
    // Index `i` counted from the end in question.
    const auto at = [&](int i) { return atBack ? p[static_cast<std::size_t>(n - 1 - i)] : p[static_cast<std::size_t>(i)]; };

    cv::Point2d c(0.0, 0.0);
    for (int i = dropN; i <= fitN; ++i) c += at(i);
    c *= 1.0 / (fitN - dropN + 1);
    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (int i = dropN; i <= fitN; ++i) {
        const cv::Point2d q = at(i) - c;
        sxx += q.x * q.x;
        sxy += q.x * q.y;
        syy += q.y * q.y;
    }
    const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    cv::Point2d d(std::cos(angle), std::sin(angle));
    if ((at(dropN) - at(fitN)).dot(d) < 0.0) d = -d;  // pointing off this end

    // From the fitted line opposite where the kept part stops, walk out to
    // the first mat.
    const cv::Point2d s = c + d * (at(dropN) - c).dot(d);
    std::vector<cv::Point2d> ray;
    for (double t = 0.0; t <= cfg.collar_edge_reach_mm + 2.0 + 1e-9; t += kWalkMm) ray.push_back(s + d * t);
    const std::vector<signed char> side = cloth.at(ray);
    if (side.empty() || side[0] != 1) return false;
    std::size_t k = 1;
    while (k < side.size() && side[k] == 1) ++k;
    if (k >= side.size() || side[k] != 0) return false;
    const cv::Point2d coarse = (ray[k - 1] + ray[k]) * 0.5;

    end = coarse;
    cv::Point2d onPicture;
    if (edgeOnPicture(gray, model, coarse, d, cfg.collar_edge_search_mm, onPicture) &&
        (onPicture - s).dot(d) > 0.0) {
        end = onPicture;
    }
    keep = atBack ? static_cast<std::size_t>(n - 1 - dropN) : static_cast<std::size_t>(dropN);
    return true;
}

// ---- the back of the neck ----
//
// A round neck laid flat face up shows its front as the horseshoe and its back
// as the edge of the shirt between the two shoulder points: the back of the
// collar is the top of the garment, lying along the silhouette. So the whole
// collar is the band's outer edge over the front and that edge of the shirt
// across the back, and the two meet at the shoulder points.
//
// The edge is taken from the cloth's own boundary between the shoulder points
// - the short way, which is the back of the neck - and then put on the picture
// every half millimetre, where the cloth falls to the mat, exactly as the
// shoulder points themselves are put there. The boundary is a guide and not
// the answer: glow off pale cloth pushes an outline out onto the mat, and
// every pixel step of a mask is length that is not there. Across the edge the
// offsets from the guide are smoothed over a few millimetres, since a fold of
// cloth does not jump, and a stretch the picture cannot show is carried
// across from either side of it.
//
// Fills L.back_path_mm, from the line's last point round to its first, and
// L.back_mm. False, with `why`, when the edge between the two cannot be
// followed or does not run like the back of a neck.
bool readBack(const cv::Mat& gray, const PlaneModel& model, const Cloth& cloth, const Config& cfg,
              CollarLine& L, std::string& why) {
    L.back_path_mm.clear();
    L.back_mm = 0.0;
    if (L.path_mm.size() < 2) {
        why = "there is no line to close";
        return false;
    }
    const cv::Point2d from = L.path_mm.back();
    const cv::Point2d to = L.path_mm.front();
    const std::vector<cv::Point2d> endsPx = model.mmToPx({from, to});
    const std::pair<double, double> s = model.localMmPerPx(0.5 * (endsPx[0] + endsPx[1]));
    const double pxPerMm = 2.0 / std::max(1e-6, s.first + s.second);
    const std::vector<cv::Point2d> edgePx = cloth.edgeBetween(endsPx[0], endsPx[1], 10.0 * pxPerMm);
    if (edgePx.size() < 8) {
        why = "the edge of the shirt across the back of the neck could not be followed from one shoulder point to "
              "the other";
        return false;
    }
    std::vector<cv::Point2d> guide = model.pxToMm(edgePx);
    guide.front() = from;
    guide.back() = to;

    // Every half millimetre, the mask's steps smoothed out over two either
    // side, the two ends held where they are.
    std::vector<cv::Point2d> p = resampleEvery(guide, 0.5);
    const int n = static_cast<int>(p.size());
    if (n < 16) {
        why = "the shoulder points are too close together to have a neck between them";
        return false;
    }
    {
        std::vector<cv::Point2d> smooth = p;
        const int h = 4;
        for (int i = 1; i + 1 < n; ++i) {
            const int reach = std::min(h, std::min(i, n - 1 - i));
            cv::Point2d sum(0.0, 0.0);
            for (int k = -reach; k <= reach; ++k) sum += p[static_cast<std::size_t>(i + k)];
            smooth[static_cast<std::size_t>(i)] = sum * (1.0 / (2 * reach + 1));
        }
        p.swap(smooth);
    }

    // Off the cloth, the same way all along: the side the cloth is not on,
    // by a vote of the whole edge.
    std::vector<cv::Point2d> nrm = normalsOf(p, 6);
    {
        std::vector<cv::Point2d> probe;
        probe.reserve(static_cast<std::size_t>(n) * 2);
        for (int i = 0; i < n; ++i) {
            probe.push_back(p[static_cast<std::size_t>(i)] + nrm[static_cast<std::size_t>(i)] * 2.0);
            probe.push_back(p[static_cast<std::size_t>(i)] - nrm[static_cast<std::size_t>(i)] * 2.0);
        }
        const std::vector<signed char> side = cloth.at(probe);
        int vote = 0;
        for (int i = 0; i < n; ++i) {
            const signed char plus = side[static_cast<std::size_t>(i) * 2];
            const signed char minus = side[static_cast<std::size_t>(i) * 2 + 1];
            if (plus == 0 && minus == 1) ++vote;
            if (plus == 1 && minus == 0) --vote;
        }
        if (vote == 0) {
            why = "the edge of the shirt across the back of the neck could not be told from the mat";
            return false;
        }
        if (vote < 0) {
            for (cv::Point2d& v : nrm) v = -v;
        }
    }

    // Where the cloth falls to the mat, across the guide at every point.
    std::vector<double> off(static_cast<std::size_t>(n), std::numeric_limits<double>::quiet_NaN());
    int seen = 0;
    for (int i = 1; i + 1 < n; ++i) {
        cv::Point2d e;
        if (edgeOnPicture(gray, model, p[static_cast<std::size_t>(i)], nrm[static_cast<std::size_t>(i)],
                          cfg.collar_edge_search_mm, e)) {
            off[static_cast<std::size_t>(i)] = (e - p[static_cast<std::size_t>(i)]).dot(nrm[static_cast<std::size_t>(i)]);
            ++seen;
        }
    }
    // The shoulder points are on the picture already: no correction there.
    off.front() = 0.0;
    off.back() = 0.0;
    if (seen * 3 >= n) {
        // Gaps carried across from either side.
        int prev = 0;
        for (int i = 1; i < n; ++i) {
            if (std::isnan(off[static_cast<std::size_t>(i)])) continue;
            for (int k = prev + 1; k < i; ++k) {
                const double t = static_cast<double>(k - prev) / (i - prev);
                off[static_cast<std::size_t>(k)] =
                    off[static_cast<std::size_t>(prev)] + (off[static_cast<std::size_t>(i)] - off[static_cast<std::size_t>(prev)]) * t;
            }
            prev = i;
        }
        // A running median then a running mean, three millimetres each way:
        // a stitch or a speck at the edge does not move it, and the edge of a
        // piece of cloth does not jump.
        const int h = 6;
        std::vector<double> med(off.size()), win;
        for (int i = 0; i < n; ++i) {
            win.clear();
            for (int k = std::max(0, i - h); k <= std::min(n - 1, i + h); ++k) win.push_back(off[static_cast<std::size_t>(k)]);
            med[static_cast<std::size_t>(i)] = middleValue(win);
        }
        for (int i = 1; i + 1 < n; ++i) {
            double sum = 0.0;
            int c = 0;
            for (int k = std::max(0, i - h); k <= std::min(n - 1, i + h); ++k, ++c) sum += med[static_cast<std::size_t>(k)];
            // Brought in to nothing over the last two millimetres before each
            // shoulder point, where the line is already on the picture.
            const double ramp = std::min(1.0, std::min(i, n - 1 - i) / 4.0);
            off[static_cast<std::size_t>(i)] = ramp * sum / c;
        }
        off.front() = 0.0;
        off.back() = 0.0;
        for (int i = 0; i < n; ++i) p[static_cast<std::size_t>(i)] += nrm[static_cast<std::size_t>(i)] * off[static_cast<std::size_t>(i)];
    }
    p.front() = from;
    p.back() = to;

    // The back of a neck laid flat is nearly straight. An edge that wanders far
    // from the straight line between the shoulder points has left the neck -
    // round a shoulder, into a fold.
    const double back = lengthOf(p);
    const double chord = dist(from, to);
    if (back > 1.5 * chord + 10.0) {
        why = "the edge of the shirt between the shoulder points runs " + vjson::formatFixed(back, 0) +
              " mm where they are " + vjson::formatFixed(chord, 0) + " mm apart: that is not the back of a neck";
        return false;
    }
    L.back_path_mm.swap(p);
    L.back_mm = back;
    return true;
}

void measureShape(CollarLine& L) {
    L.arc_mm = lengthOf(L.path_mm);
    L.chord_mm = L.path_mm.size() >= 2 ? dist(L.path_mm.front(), L.path_mm.back()) : 0.0;
    L.depth_mm = 0.0;
    if (L.chord_mm > 1e-6) {
        const cv::Point2d a = L.path_mm.front();
        const cv::Point2d u = unit(L.path_mm.back() - a);
        for (const cv::Point2d& p : L.path_mm) {
            L.depth_mm = std::max(L.depth_mm, std::fabs((p - a).x * u.y - (p - a).y * u.x));
        }
    }
    cv::Point2d centre;
    double radius = 0.0;
    L.radius_mm = fitCircleRadius(L.path_mm, radius, centre) ? radius : 0.0;
}

// ---- the picture across a line ----
//
// Sampled along each point's normal at a run of offsets and averaged along the
// line, leaving out its ends. Averaging along the WHOLE line is what makes this
// work: a line parallel to this one keeps its offset the whole way round and
// adds up, while a fold, a print or a shadow crosses at an angle and averages
// away to nothing. What survives is the band - both its edges and the
// stitching beside them - at the offsets it actually has.
struct Across {
    double lo = 0.0;
    std::vector<double> level;  // mean gray at each offset, NaN where too little was seen
};

Across acrossLine(const cv::Mat& gray, const PlaneModel& model, const Cloth* cloth,
                  const std::vector<cv::Point2d>& pts, const std::vector<cv::Point2d>& nrm,
                  double lo, double hi, double skipMm) {
    Across a;
    a.lo = lo;
    const int nOff = static_cast<int>(std::lround((hi - lo) / kProfileStepMm)) + 1;
    a.level.assign(static_cast<std::size_t>(nOff), std::numeric_limits<double>::quiet_NaN());
    if (pts.size() < 3) return a;

    // The points far enough from both ends, by arc length.
    std::vector<double> along(pts.size(), 0.0);
    for (std::size_t k = 1; k < pts.size(); ++k) along[k] = along[k - 1] + dist(pts[k - 1], pts[k]);
    const double total = along.back();
    std::vector<std::size_t> use;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if (along[k] >= skipMm && along[k] <= total - skipMm) use.push_back(k);
    }
    if (use.size() < 8) return a;

    std::vector<cv::Point2d> mm;
    mm.reserve(use.size() * static_cast<std::size_t>(nOff));
    for (std::size_t k : use) {
        for (int j = 0; j < nOff; ++j) mm.push_back(pts[k] + nrm[k] * (lo + j * kProfileStepMm));
    }
    const std::vector<cv::Point2d> px = model.mmToPx(mm);
    std::vector<double> sum(static_cast<std::size_t>(nOff), 0.0);
    std::vector<int> count(static_cast<std::size_t>(nOff), 0);
    for (std::size_t i = 0; i < use.size(); ++i) {
        for (int j = 0; j < nOff; ++j) {
            const cv::Point2d& q = px[i * static_cast<std::size_t>(nOff) + static_cast<std::size_t>(j)];
            if (cloth && cloth->atPx(q) != 1) continue;
            float v = 0.0f;
            if (!grayAt(gray, q.x, q.y, v)) continue;
            sum[static_cast<std::size_t>(j)] += v;
            ++count[static_cast<std::size_t>(j)];
        }
    }
    const int most = *std::max_element(count.begin(), count.end());
    for (int j = 0; j < nOff; ++j) {
        if (count[static_cast<std::size_t>(j)] * 2 >= most && count[static_cast<std::size_t>(j)] > 0) {
            a.level[static_cast<std::size_t>(j)] = sum[static_cast<std::size_t>(j)] / count[static_cast<std::size_t>(j)];
        }
    }
    return a;
}

// A line across the band, in one stretch of the collar: where it sits, how
// strong it is, what kind it is, and whether it is crisp.
struct Mark {
    double offset = 0.0;
    double strength = 0.0;  // in units of the stretch's own threshold
    bool step = true;       // a change of shade; otherwise a stitch or a seam's groove
    bool crisp = true;      // an edge of cloth or a stitch, not shading
};

// The lines in a profile across the band.
//
// Both kinds, as snapCurve looks for both: a STEP, where the shade changes -
// a band meeting the body, the opening's edge - found as a peak of the first
// derivative; and a RIDGE, a stitch or a seam's groove in cloth of its own
// colour, found as a peak of the second derivative where no step explains
// it. Strength is in units of the profile's own noise, with a floor in gray
// levels, so a smooth profile does not make its own ripples look like lines.
//
// And each step is asked how wide it is: the change of shade across it
// divided by how steeply it changes. The edge of a piece of cloth, or a seam,
// changes over a millimetre or so. Shading - the body bunching up beside a
// collar, a soft fold - changes over several. On the first real shirt this
// met, shading ran parallel to the collar all the way round, fifteen
// millimetres outside the seam, reaching the edge of the cloth at both ends
// exactly as the seam does. It is a line in every sense but that one.
std::vector<Mark> marksOf(const Across& a) {
    std::vector<Mark> out;
    const int n = static_cast<int>(a.level.size());
    if (n < 40) return out;

    // The valid run: the offsets seen on enough of the stretch.
    int first = 0, last = n - 1;
    while (first < n && std::isnan(a.level[static_cast<std::size_t>(first)])) ++first;
    while (last > first && std::isnan(a.level[static_cast<std::size_t>(last)])) --last;
    if (last - first < 40) return out;
    std::vector<double> v(a.level.begin() + first, a.level.begin() + last + 1);
    for (std::size_t k = 1; k < v.size(); ++k) {
        if (std::isnan(v[k])) v[k] = v[k - 1];
    }
    const int m = static_cast<int>(v.size());
    {
        cv::Mat mat(1, m, CV_64F, v.data());
        cv::GaussianBlur(mat, mat, cv::Size(0, 0), 0.25 / kProfileStepMm, 0.0, cv::BORDER_REPLICATE);
    }

    // First and second derivative, in gray levels per millimetre and per
    // millimetre squared; the second over three samples a side, which is the
    // width of a stitch.
    const int h = 3;
    std::vector<double> d1(static_cast<std::size_t>(m), 0.0), d2(static_cast<std::size_t>(m), 0.0);
    for (int i = h; i + h < m; ++i) {
        d1[static_cast<std::size_t>(i)] =
            (v[static_cast<std::size_t>(i + 1)] - v[static_cast<std::size_t>(i - 1)]) / (2.0 * kProfileStepMm);
        const double w = h * kProfileStepMm;
        d2[static_cast<std::size_t>(i)] = (v[static_cast<std::size_t>(i + h)] - 2.0 * v[static_cast<std::size_t>(i)] +
                                           v[static_cast<std::size_t>(i - h)]) / (w * w);
    }
    std::vector<double> a1, a2;
    for (int i = h; i + h < m; ++i) {
        a1.push_back(std::fabs(d1[static_cast<std::size_t>(i)]));
        a2.push_back(std::fabs(d2[static_cast<std::size_t>(i)]));
    }
    const double s1 = std::max(1.4826 * middleValue(a1), 1e-6);
    const double s2 = std::max(1.4826 * middleValue(a2), 1e-6);
    // A line has to change the picture by a couple of gray levels across a
    // millimetre whatever the noise is - the same floor curve_min_contrast
    // puts under the snap.
    const double thr1 = std::max(5.0 * s1, 2.0);
    const double thr2 = std::max(5.0 * s2, 8.0);

    const int win = static_cast<int>(std::lround(1.0 / kProfileStepMm));
    const auto isPeak = [&](const std::vector<double>& d, int i) {
        const double here = std::fabs(d[static_cast<std::size_t>(i)]);
        for (int k = std::max(h, i - win); k <= std::min(m - h - 1, i + win); ++k) {
            if (std::fabs(d[static_cast<std::size_t>(k)]) > here) return false;
        }
        return true;
    };
    const int one = static_cast<int>(std::lround(1.0 / kProfileStepMm));
    const int three = static_cast<int>(std::lround(3.0 / kProfileStepMm));
    const auto meanOver = [&](int from, int to) {
        from = std::max(0, from);
        to = std::min(m - 1, to);
        double s = 0.0;
        int c = 0;
        for (int k = from; k <= to; ++k, ++c) s += v[static_cast<std::size_t>(k)];
        return c ? s / c : 0.0;
    };
    std::vector<double> stepsAt;
    for (int i = h; i + h < m; ++i) {
        const double s = std::fabs(d1[static_cast<std::size_t>(i)]);
        if (s < thr1 || !isPeak(d1, i)) continue;
        const double off = a.lo + (first + i) * kProfileStepMm;
        // How wide the change is: all of it, over how steeply it happens.
        const double change = std::fabs(meanOver(i + one, i + three) - meanOver(i - three, i - one));
        const double width = change / s;
        out.push_back({off, s / thr1, true, width <= kCrispMm});
        stepsAt.push_back(off);
    }
    for (int i = h; i + h < m; ++i) {
        const double s = std::fabs(d2[static_cast<std::size_t>(i)]);
        if (s < thr2 || !isPeak(d2, i)) continue;
        const double off = a.lo + (first + i) * kProfileStepMm;
        // A step's own shoulders are peaks of the second derivative, a
        // fraction of a millimetre either side of it. Those are the step, not
        // a stitch beside it.
        bool explained = false;
        for (double at : stepsAt) {
            if (std::fabs(at - off) < 1.2) explained = true;
        }
        if (!explained) out.push_back({off, s / thr2, false, true});
    }
    std::sort(out.begin(), out.end(), [](const Mark& l, const Mark& r) { return l.strength > r.strength; });
    if (out.size() > 24) out.resize(24);
    std::sort(out.begin(), out.end(), [](const Mark& l, const Mark& r) { return l.offset < r.offset; });
    return out;
}

// How much of a line's contrast is a change of shade rather than a stitch.
//
// Across the line itself, a few millimetres either side, averaged along it:
// the step is the difference between the two sides; the ridge is how far the
// middle stands off the average of the two. A band's edge is mostly the
// former, a topstitch is mostly the latter, and an attachment seam that is
// both - a change of cloth with its own stitch in it - reads as a step.
double stepShare(const cv::Mat& gray, const PlaneModel& model, const std::vector<cv::Point2d>& path) {
    const std::vector<cv::Point2d> pts = resampleEvery(path, 0.5);
    if (pts.size() < 20) return 0.0;
    const std::vector<cv::Point2d> nrm = normalsOf(pts, 4);
    const Across a = acrossLine(gray, model, nullptr, pts, nrm, -4.0, 4.0, kProfileSkipMm);
    const auto mean = [&](double from, double to) {
        double s = 0.0;
        int c = 0;
        for (std::size_t j = 0; j < a.level.size(); ++j) {
            const double off = a.lo + static_cast<double>(j) * kProfileStepMm;
            if (off < from - 1e-9 || off > to + 1e-9 || std::isnan(a.level[j])) continue;
            s += a.level[j];
            ++c;
        }
        return c ? s / c : std::numeric_limits<double>::quiet_NaN();
    };
    const double left = mean(-2.5, -0.8);
    const double right = mean(0.8, 2.5);
    const double mid = mean(-0.3, 0.3);
    if (std::isnan(left) || std::isnan(right) || std::isnan(mid)) return 0.0;
    const double step = std::fabs(right - left);
    const double ridge = std::fabs(mid - 0.5 * (left + right));
    return step / (step + ridge + 1e-9);
}

// Mean distance from the points of one line to another. How two readings are
// told to be the same line or two lines.
double meanDistance(const std::vector<cv::Point2d>& a, const std::vector<cv::Point2d>& b) {
    const std::vector<cv::Point2d> pa = resampleEvery(a, 2.0);
    if (pa.empty() || b.size() < 2) return std::numeric_limits<double>::max();
    double s = 0.0;
    for (const cv::Point2d& q : pa) s += distanceToPath(q, b);
    return s / static_cast<double>(pa.size());
}

// Where the bottom of a seam's groove lies, all along it: for each point of
// the line, how far along its normal to move it to put it there.
//
// An attachment seam is a groove: the band's edge turns down into it on one
// side, the body comes up out of it on the other, and the stitching is at the
// bottom. The snap reads a step where it finds one, and the band side of a
// groove is a step - the band is lighter than the groove, and usually than the
// body too - so it reads the band's edge, half a millimetre short of the
// stitching. Half a millimetre across a seam is a millimetre and a half along
// a neckline, every time and always short.
//
// Point by point rather than once for the whole line, because of where a
// collar meets the shoulder. There the seam's groove and the edge of a crease
// where the collar folds under run side by side a couple of millimetres apart,
// and which of the two a reading followed over its last centimetre depended
// on nothing more than which box had been drawn - on a real shirt, a shoulder
// point two and a half millimetres along the edge of the cloth from where
// another box put it. Located on the groove itself at every point, the line
// has no choice to make there.
//
// The groove is found as a NARROW dark line: darker than the cloth a third to
// two thirds of a millimetre either side of it. A crease is dark over several
// millimetres and a stitch's own groove over one, so this finds the groove
// and passes over the crease. Averaged five millimetres along the line and
// smoothed as much again, so a speck or a missed stitch does not move it.
// Only for the seam: the opening's edge is the edge of the cloth itself, and
// the shadow it throws on what is under it is a dark line that is not the
// edge. Empty when the seam shows no groove at all, and the line is left as
// the snap read it.
std::vector<double> grooveOffsets(const cv::Mat& gray, const PlaneModel& model, const std::vector<cv::Point2d>& pts,
                                  const std::vector<cv::Point2d>& nrm) {
    const int n = static_cast<int>(pts.size());
    const double reach = 1.5;
    const int nOff = static_cast<int>(std::lround(2.0 * reach / kProfileStepMm)) + 1;
    if (n < 20) return {};
    std::vector<cv::Point2d> mm;
    mm.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(nOff));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < nOff; ++j) {
            mm.push_back(pts[static_cast<std::size_t>(i)] + nrm[static_cast<std::size_t>(i)] * (-reach + j * kProfileStepMm));
        }
    }
    const std::vector<cv::Point2d> px = model.mmToPx(mm);
    // Running sums along the line, one per offset, for the window along it.
    std::vector<double> csum(static_cast<std::size_t>(n + 1) * static_cast<std::size_t>(nOff), 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < nOff; ++j) {
            float v = 0.0f;
            const cv::Point2d& q = px[static_cast<std::size_t>(i) * nOff + j];
            if (!grayAt(gray, q.x, q.y, v)) return {};
            csum[static_cast<std::size_t>(i + 1) * nOff + j] = csum[static_cast<std::size_t>(i) * nOff + j] + v;
        }
    }
    const int half = static_cast<int>(std::lround(2.5 / 0.5));  // five millimetres of line, at half a millimetre
    const int in = static_cast<int>(std::lround(0.3 / kProfileStepMm));
    const int outer = static_cast<int>(std::lround(0.6 / kProfileStepMm));
    std::vector<double> off(static_cast<std::size_t>(n), std::numeric_limits<double>::quiet_NaN());
    std::vector<double> prof(static_cast<std::size_t>(nOff));
    std::vector<double> resp(static_cast<std::size_t>(nOff), -1e9);
    int found = 0;
    for (int i = 0; i < n; ++i) {
        const int lo = std::max(0, i - half), hi = std::min(n - 1, i + half);
        for (int j = 0; j < nOff; ++j) {
            prof[static_cast<std::size_t>(j)] = (csum[static_cast<std::size_t>(hi + 1) * nOff + j] -
                                                 csum[static_cast<std::size_t>(lo) * nOff + j]) /
                                                (hi - lo + 1);
        }
        int best = -1;
        for (int j = outer; j < nOff - outer; ++j) {
            double side = 0.0;
            for (int k = in; k <= outer; ++k) {
                side += prof[static_cast<std::size_t>(j - k)] + prof[static_cast<std::size_t>(j + k)];
            }
            side /= 2.0 * (outer - in + 1);
            resp[static_cast<std::size_t>(j)] = side - prof[static_cast<std::size_t>(j)];
            if (best < 0 || resp[static_cast<std::size_t>(j)] > resp[static_cast<std::size_t>(best)]) best = j;
        }
        // A groove two and a half gray levels deeper than the cloth beside it.
        if (best <= outer || best >= nOff - outer - 1 || resp[static_cast<std::size_t>(best)] < 2.5) continue;
        const double l = resp[static_cast<std::size_t>(best - 1)], c = resp[static_cast<std::size_t>(best)],
                     r = resp[static_cast<std::size_t>(best + 1)];
        const double den = l - 2.0 * c + r;
        const double s = den < -1e-9 ? std::max(-1.0, std::min(1.0, 0.5 * (l - r) / den)) : 0.0;
        off[static_cast<std::size_t>(i)] = -reach + (best + s) * kProfileStepMm;
        ++found;
    }
    // Most of the seam has to show a groove for any of it to be moved.
    if (found < n / 2) return {};
    // Gaps carried across from either side; the ends held at the nearest.
    int prev = -1;
    for (int i = 0; i < n; ++i) {
        if (std::isnan(off[static_cast<std::size_t>(i)])) continue;
        if (prev < 0) {
            for (int k = 0; k < i; ++k) off[static_cast<std::size_t>(k)] = off[static_cast<std::size_t>(i)];
        } else {
            for (int k = prev + 1; k < i; ++k) {
                const double t = static_cast<double>(k - prev) / (i - prev);
                off[static_cast<std::size_t>(k)] =
                    off[static_cast<std::size_t>(prev)] + (off[static_cast<std::size_t>(i)] - off[static_cast<std::size_t>(prev)]) * t;
            }
        }
        prev = i;
    }
    for (int k = prev + 1; k < n; ++k) off[static_cast<std::size_t>(k)] = off[static_cast<std::size_t>(prev)];
    // A running median then a running mean, five millimetres each.
    std::vector<double> med(off.size());
    std::vector<double> win;
    for (int i = 0; i < n; ++i) {
        win.clear();
        for (int k = std::max(0, i - half); k <= std::min(n - 1, i + half); ++k) win.push_back(off[static_cast<std::size_t>(k)]);
        med[static_cast<std::size_t>(i)] = middleValue(win);
    }
    for (int i = 0; i < n; ++i) {
        double s = 0.0;
        int c = 0;
        for (int k = std::max(0, i - half); k <= std::min(n - 1, i + half); ++k, ++c) s += med[static_cast<std::size_t>(k)];
        off[static_cast<std::size_t>(i)] = s / c;
    }
    return off;
}

// Read one line of the collar from a guide that runs from edge to edge: the
// snap does the reading, the attachment seam is put in the bottom of its
// groove, and the ends are then put where the cloth ends.
bool readLine(const cv::Mat& gray, const PlaneModel& model, const Cloth& cloth,
              const std::vector<cv::Point2d>& guide, const Config& cfg, CollarLine& L, bool seam = false) {
    const Reached g = reachEdge(guide, cloth, kGuideReachMm);
    if (!g.meets[0] || !g.meets[1] || g.path.size() < 8) return false;

    // Every three millimetres: a trace, as far as the snap is concerned,
    // followed as it lies. Its first and last points are the guide's own
    // crossings of the edge, and the snap trims the reading back to the feet
    // of exactly those two.
    std::vector<cv::Point2d> seeds = resampleEvery(g.path, 3.0);
    if (seeds.size() < 4) return false;

    Config read = cfg;
    read.curve_corridor_mm = std::max(1.0, cfg.collar_corridor_mm);
    // Nothing here was pointed at, so there is no pointing for the found line
    // to have drifted from, and no pointing that failed to choose.
    read.curve_max_drift_mm = std::numeric_limits<double>::max();
    read.curve_min_margin = -std::numeric_limits<double>::max();
    CurveResult c;
    if (!snapCurve(gray, model, seeds, read, c)) return false;
    if (seam) {
        // Cut at the edge of the cloth first, so the groove is looked for
        // over the seam and only the seam: how far past the edge the snap's
        // corridor happened to run moved the smoothing at the very end, and
        // with it a shoulder point, by up to a millimetre from box to box.
        const Reached cut = reachEdge(c.path_mm, cloth, std::max(0.5, cfg.collar_edge_reach_mm));
        std::vector<cv::Point2d> pts = resampleEvery(cut.path.size() >= 8 ? cut.path : c.path_mm, 0.5);
        if (pts.size() >= 20) {
            const std::vector<cv::Point2d> nrm = normalsOf(pts, 4);
            const std::vector<double> offs = grooveOffsets(gray, model, pts, nrm);
            if (offs.size() == pts.size()) {
                for (std::size_t k = 0; k < pts.size(); ++k) pts[k] += nrm[k] * offs[k];
                c.path_mm = pts;
            }
        }
    }

    Reached f = reachEdge(c.path_mm, cloth, std::max(0.5, cfg.collar_edge_reach_mm));
    L.meets_edge = f.meets[0] && f.meets[1];
    if (f.path.size() < 8) return false;
    if (L.meets_edge) {
        cv::Point2d e0, e1;
        std::size_t keep0 = 0, keep1 = 0;
        if (!straightEnd(f.path, false, gray, model, cloth, cfg, e0, keep0) ||
            !straightEnd(f.path, true, gray, model, cloth, cfg, e1, keep1) || keep1 <= keep0 + 8) {
            L.meets_edge = false;
        } else {
            std::vector<cv::Point2d> p;
            p.reserve(keep1 - keep0 + 3);
            p.push_back(e0);
            p.insert(p.end(), f.path.begin() + static_cast<std::ptrdiff_t>(keep0),
                     f.path.begin() + static_cast<std::ptrdiff_t>(keep1) + 1);
            p.push_back(e1);
            f.path.swap(p);
        }
    }
    L.path_mm = f.path;
    measureShape(L);
    L.confidence = c.confidence;
    L.margin = c.margin;
    L.roughness_mm = c.roughness_mm;
    L.step_share = stepShare(gray, model, L.path_mm);
    return true;
}

void takeLine(CollarResult& r, int k) {
    const CollarLine& L = r.lines[static_cast<std::size_t>(k)];
    r.chosen = k;
    r.arc_mm = L.arc_mm;
    r.chord_mm = L.chord_mm;
    r.depth_mm = L.depth_mm;
    r.end_mm[0] = L.path_mm.front();
    r.end_mm[1] = L.path_mm.back();
    r.back_path_mm = L.back_path_mm;
    r.back_mm = L.back_mm;
}

// ---- the band, stretch by stretch ----
//
// A collar band is two crisp lines a band's width apart with nothing crisp
// between them: the opening's edge, plain rib, and the attachment seam. That
// is looked for across the collar in short stretches along a frame line, each
// stretch on its own, and then the band is followed from one stretch to the
// next by how little its width and place change.
//
// Each part of that is there because something else went wrong without it.
// One profile averaged round the whole collar found the lines, but which
// lines depended on which line the box's guesses had held to, and that
// depended on the box: one collar read 264 to 306 mm from eight boxes. Taking
// the band as "two groups of lines a band's width apart" took a seam and a
// line of shading fifteen millimetres outside it for the band whenever the
// opening's edge was not among what had been found. And a frame line that had
// strayed from one edge of the band to the other part of the way round
// carried every line found along it with it.
//
// Stretch by stretch, none of that reaches the answer. Each stretch finds the
// band where it is, relative to the frame there, so a frame that strays only
// moves where each stretch finds it; shading is not crisp and cannot be one
// side of a band; a seam and the topstitch beside it are not a band because
// the seam lies between them; and the box has nothing to do with any of it.
struct BandFind {
    bool found = false;
    std::string why;         // when it was not
    double wMin = 0.0, wMax = 0.0;
    int stretches = 0;
    int with = 0;            // stretches the band was found in
    bool readOk[2] = {false, false};
    CollarLine edges[2];     // inner, outer
    std::vector<std::string> notes;
    // Where the band's outer edge was found, however little of it: what says
    // a box cut the collar when not enough of the band was inside it to read.
    std::vector<cv::Point2d> partial;
    // How far along the frame the band was followed, from its first stretch
    // to its last, and how long the frame is.
    double runFrom = 0.0, runTo = 0.0, total = 0.0;
    // How far the band's outer edge lay from the frame, at worst, over the
    // stretches it was found in. Along a frame that IS the outer edge just
    // read, this is how far that reading strayed from the band anywhere.
    double outerStray = 0.0;
    // Whether the band was followed to within reach of both ends of the
    // frame. Along a frame that is the band's own edge, read to the edge of
    // the cloth, that is the band seen right down to both shoulder points.
    bool reachesEnds() const { return found && runFrom <= kEndsMm && total - runTo <= kEndsMm; }
    static constexpr double kEndsMm = 35.0;
};

BandFind findBand(const cv::Mat& gray, const PlaneModel& model, const Cloth& cloth,
                  const std::vector<cv::Point2d>& frameLine, const Config& cfg) {
    BandFind bf;
    const std::vector<cv::Point2d> rpts = resampleEvery(frameLine, 0.5);
    std::vector<cv::Point2d> rnrm = normalsOf(rpts, 6);
    {
        // Outward: away from the neck opening, the side the collar curves round.
        cv::Point2d centre(0.0, 0.0);
        for (const cv::Point2d& p : rpts) centre += p;
        centre *= 1.0 / std::max<std::size_t>(1, rpts.size());
        double sense = 0.0;
        for (std::size_t k = 0; k < rpts.size(); ++k) sense += (rpts[k] - centre).dot(rnrm[k]);
        if (sense < 0.0) {
            for (cv::Point2d& n : rnrm) n = -n;
        }
    }
    std::vector<double> along(rpts.size(), 0.0);
    for (std::size_t k = 1; k < rpts.size(); ++k) along[k] = along[k - 1] + dist(rpts[k - 1], rpts[k]);
    const double total = along.empty() ? 0.0 : along.back();
    // Stretches from end to end. A shorter one at each end, to find the band
    // right down to the edge of the cloth, was tried: ten millimetres of
    // collar is too little for the texture of the cloth to average out of,
    // and both shoulder points got noisier rather than steadier.
    std::vector<std::pair<double, double>> spans;
    {
        const int n = static_cast<int>(std::floor((total - 2.0 * kStretchSkipMm) / kStretchMm));
        for (int i = 0; i < n; ++i) {
            spans.emplace_back(kStretchSkipMm + i * kStretchMm, kStretchSkipMm + (i + 1) * kStretchMm);
        }
    }
    const int S = static_cast<int>(spans.size());
    bf.stretches = S;
    if (S < 5) {
        bf.why = "the collar in that box is too short to read: " + vjson::formatFixed(total, 0) + " mm round";
        return bf;
    }
    std::vector<double> centreAt(static_cast<std::size_t>(S));
    std::vector<std::vector<Mark>> marks(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
        centreAt[static_cast<std::size_t>(s)] =
            0.5 * (spans[static_cast<std::size_t>(s)].first + spans[static_cast<std::size_t>(s)].second);
    }
    cv::parallel_for_(cv::Range(0, S), [&](const cv::Range& range) {
        for (int s = range.start; s < range.end; ++s) {
            const double from = spans[static_cast<std::size_t>(s)].first;
            const double to = spans[static_cast<std::size_t>(s)].second;
            std::vector<cv::Point2d> sp, sn;
            for (std::size_t k = 0; k < rpts.size(); ++k) {
                if (along[k] < from || along[k] >= to) continue;
                sp.push_back(rpts[k]);
                sn.push_back(rnrm[k]);
            }
            const Across a = acrossLine(gray, model, &cloth, sp, sn, -kBandSearchMm, kBandSearchMm, 0.0);
            marks[static_cast<std::size_t>(s)] = marksOf(a);
        }
    });

    // Every band each stretch offers: two crisp lines within a band's width
    // of each other and nothing crisp between them that is anything like as
    // strong as they are.
    struct Band {
        double in = 0.0, out = 0.0, score = 0.0;
    };
    std::vector<std::vector<Band>> bands(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
        const std::vector<Mark>& m = marks[static_cast<std::size_t>(s)];
        for (std::size_t i = 0; i < m.size(); ++i) {
            for (std::size_t j = i + 1; j < m.size(); ++j) {
                if (!m[i].crisp || !m[j].crisp) continue;
                const double gap = m[j].offset - m[i].offset;
                if (gap < cfg.collar_band_min_mm || gap > cfg.collar_band_max_mm) continue;
                const double weaker = std::min(m[i].strength, m[j].strength);
                bool plain = true;
                for (std::size_t k = i + 1; k < j && plain; ++k) {
                    if (m[k].crisp && m[k].strength >= 0.6 * weaker) plain = false;
                }
                if (!plain) continue;
                bands[static_cast<std::size_t>(s)].push_back(
                    {m[i].offset, m[j].offset, std::min(4.0, m[i].strength) + std::min(4.0, m[j].strength)});
            }
        }
    }

    // The band all the way round: one band per stretch, chosen together, so
    // that from one stretch to the next its width barely changes and its
    // place changes no faster than the frame can stray. A stretch may go
    // without - a care label across the band, a fold - at a cost, and one
    // stretch at a time.
    const double kSkip = 2.0;
    const auto link = [&](int s0, const Band& a, int s1, const Band& b, double& cost) {
        const double d = centreAt[static_cast<std::size_t>(s1)] - centreAt[static_cast<std::size_t>(s0)];
        const double dw = std::fabs((b.out - b.in) - (a.out - a.in));
        const double dm = std::fabs(0.5 * (b.out + b.in) - 0.5 * (a.out + a.in));
        if (dw > 3.0 + 0.03 * d || dm > 4.0 + 0.25 * d) return false;
        cost = dw + 0.15 * dm;
        return true;
    };
    std::vector<std::vector<double>> best(static_cast<std::size_t>(S));
    std::vector<std::vector<std::pair<int, int>>> came(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
        const std::vector<Band>& here = bands[static_cast<std::size_t>(s)];
        best[static_cast<std::size_t>(s)].assign(here.size(), 0.0);
        came[static_cast<std::size_t>(s)].assign(here.size(), std::make_pair(-1, -1));
        for (std::size_t k = 0; k < here.size(); ++k) {
            // Starting here: every stretch before it went without.
            double b = here[k].score - kSkip * s;
            std::pair<int, int> f(-1, -1);
            for (int back = 1; back <= 2; ++back) {
                const int p = s - back;
                if (p < 0) continue;
                const std::vector<Band>& there = bands[static_cast<std::size_t>(p)];
                for (std::size_t q = 0; q < there.size(); ++q) {
                    double c = 0.0;
                    if (!link(p, there[q], s, here[k], c)) continue;
                    const double v = best[static_cast<std::size_t>(p)][q] - c - (back - 1) * kSkip + here[k].score;
                    if (v > b) {
                        b = v;
                        f = std::make_pair(p, static_cast<int>(q));
                    }
                }
            }
            best[static_cast<std::size_t>(s)][k] = b;
            came[static_cast<std::size_t>(s)][k] = f;
        }
    }
    int endS = -1, endK = -1;
    double endScore = -1e30;
    for (int s = 0; s < S; ++s) {
        for (std::size_t k = 0; k < bands[static_cast<std::size_t>(s)].size(); ++k) {
            const double v = best[static_cast<std::size_t>(s)][k] - kSkip * (S - 1 - s);
            if (v > endScore) {
                endScore = v;
                endS = s;
                endK = static_cast<int>(k);
            }
        }
    }
    std::vector<int> chosenS;
    std::vector<Band> chosenB;
    for (int s = endS, k = endK; s >= 0 && k >= 0;) {
        chosenS.push_back(s);
        chosenB.push_back(bands[static_cast<std::size_t>(s)][static_cast<std::size_t>(k)]);
        const std::pair<int, int> f = came[static_cast<std::size_t>(s)][static_cast<std::size_t>(k)];
        s = f.first;
        k = f.second;
    }
    std::reverse(chosenS.begin(), chosenS.end());
    std::reverse(chosenB.begin(), chosenB.end());

    // Past either end of the run, by the seam alone.
    //
    // Down by the shoulders the opening's edge runs beside the back of the
    // collar, where it is faint - the band against the back band rather than
    // against a shadow - and the last stretch or two can show no band at
    // all. The seam is as strong there as anywhere, and it is the edge that
    // runs to the shoulder point. So the run is carried on, a stretch at a
    // time, wherever a crisp line lies within two millimetres of where the
    // seam was - near enough to be it and not the topstitch beside it - with
    // the opening's edge kept the band's width inside it.
    const auto carryOn = [&](int dir) {
        while (!chosenS.empty()) {
            const int last = dir > 0 ? chosenS.back() : chosenS.front();
            const Band lb = dir > 0 ? chosenB.back() : chosenB.front();
            const int s = last + dir;
            if (s < 0 || s >= S) break;
            const std::vector<Mark>& m = marks[static_cast<std::size_t>(s)];
            int pick = -1;
            double nearest = 2.0;
            for (std::size_t i = 0; i < m.size(); ++i) {
                if (!m[i].crisp) continue;
                const double d = std::fabs(m[i].offset - lb.out);
                if (d <= nearest) {
                    nearest = d;
                    pick = static_cast<int>(i);
                }
            }
            if (pick < 0) break;
            const double out = m[static_cast<std::size_t>(pick)].offset;
            const Band nb{out - (lb.out - lb.in), out, 0.0};
            if (dir > 0) {
                chosenS.push_back(s);
                chosenB.push_back(nb);
            } else {
                chosenS.insert(chosenS.begin(), s);
                chosenB.insert(chosenB.begin(), nb);
            }
            bf.notes.push_back("  stretch " + std::to_string(s + 1) + ": the seam alone, at " +
                               vjson::formatFixed(out, 1) + " mm");
        }
    };
    carryOn(+1);
    carryOn(-1);

    for (int s = 0; s < S; ++s) {
        const auto it = std::find(chosenS.begin(), chosenS.end(), s);
        const std::string counts = std::to_string(marks[static_cast<std::size_t>(s)].size()) + " lines, " +
                                   std::to_string(bands[static_cast<std::size_t>(s)].size()) + " possible bands";
        if (it == chosenS.end()) {
            bf.notes.push_back("  stretch " + std::to_string(s + 1) + ": no band (" + counts + ")");
        } else {
            const Band& b = chosenB[static_cast<std::size_t>(it - chosenS.begin())];
            bf.notes.push_back("  stretch " + std::to_string(s + 1) + ": band " + vjson::formatFixed(b.in, 1) +
                               " to " + vjson::formatFixed(b.out, 1) + " mm (" + counts + ")");
        }
    }
    bf.with = static_cast<int>(chosenS.size());

    // Enough of the band to say where the collar is. Measured as a run along
    // the frame rather than as a share of it, because the frame is only
    // something to look along: one that carried on round a shoulder past the
    // collar is longer than the collar, and the band is rightly missing from
    // its ends. A neckline is longer than this anywhere on it.
    for (std::size_t i = 0; i < chosenS.size(); ++i) {
        const double c = centreAt[static_cast<std::size_t>(chosenS[i])];
        std::size_t k = 0;
        while (k + 1 < along.size() && along[k] < c) ++k;
        bf.partial.push_back(rpts[k] + rnrm[k] * chosenB[i].out);
    }
    bf.total = total;
    if (!chosenS.empty()) {
        bf.runFrom = centreAt[static_cast<std::size_t>(chosenS.front())];
        bf.runTo = centreAt[static_cast<std::size_t>(chosenS.back())];
    }
    for (const Band& b : chosenB) bf.outerStray = std::max(bf.outerStray, std::fabs(b.out));
    const double kMinRunMm = 130.0;
    const bool enough = !chosenS.empty() && static_cast<int>(chosenS.size()) >= 5 &&
                        centreAt[static_cast<std::size_t>(chosenS.back())] -
                                centreAt[static_cast<std::size_t>(chosenS.front())] >=
                            kMinRunMm;
    if (!enough) {
        bf.why = "could not find the collar band - two edges a band's width apart - along enough of the collar";
        return bf;
    }
    bf.wMin = 1e9;
    for (const Band& b : chosenB) {
        bf.wMin = std::min(bf.wMin, b.out - b.in);
        bf.wMax = std::max(bf.wMax, b.out - b.in);
    }

    // Each edge of the band as a guide: where the band was found in each
    // stretch, joined, over the run it was found along - and past the ends of
    // that run, carried on along the edge's own curve to the edge of the
    // cloth. Not along the frame: past the last stretch nothing has been
    // found, and a frame that turned there took the shoulder point with it.
    // Between the middles of the first and last stretches, where every point
    // is between two places the band was found; past them, the curve.
    const double runFrom = centreAt[static_cast<std::size_t>(chosenS.front())];
    const double runTo = centreAt[static_cast<std::size_t>(chosenS.back())];
    const auto offsetAt = [&](bool inner, double s) {
        const auto of = [&](std::size_t i) { return inner ? chosenB[i].in : chosenB[i].out; };
        const auto at = [&](std::size_t i) { return centreAt[static_cast<std::size_t>(chosenS[i])]; };
        if (s <= at(0)) return of(0);
        for (std::size_t i = 0; i + 1 < chosenS.size(); ++i) {
            if (s <= at(i + 1)) {
                const double t = (s - at(i)) / std::max(1e-9, at(i + 1) - at(i));
                return of(i) + (of(i + 1) - of(i)) * t;
            }
        }
        return of(chosenS.size() - 1);
    };
    std::vector<cv::Point2d> guides[2];
    for (int e = 0; e < 2; ++e) {
        std::vector<cv::Point2d> run, before, after;
        for (std::size_t k = 0; k < rpts.size(); ++k) {
            const cv::Point2d g = rpts[k] + rnrm[k] * offsetAt(e == 0, along[k]);
            if (along[k] < runFrom) {
                before.push_back(g);
            } else if (along[k] > runTo) {
                after.push_back(g);
            } else {
                run.push_back(g);
            }
        }
        bool meets[2] = {false, false};
        std::size_t added[2] = {0, 0};
        std::vector<cv::Point2d> curved = extendByCurve(run, cloth, 30.0, kGuideReachMm, meets, added);
        // An end the curve does not carry to the edge of the cloth within
        // reach - the frame stopped short of the collar there, and the curve
        // fitted to the last of the run turned away - is carried on along the
        // frame instead, as far as the frame goes. The look along the band's
        // own edge that follows takes the frame out of it again.
        if (!meets[0] || !meets[1]) {
            std::vector<cv::Point2d> g;
            if (meets[0]) {
                g.insert(g.end(), curved.begin(), curved.begin() + static_cast<std::ptrdiff_t>(added[0]));
            } else {
                g.insert(g.end(), before.begin(), before.end());
            }
            g.insert(g.end(), curved.begin() + static_cast<std::ptrdiff_t>(added[0]),
                     curved.end() - static_cast<std::ptrdiff_t>(added[1]));
            if (meets[1]) {
                g.insert(g.end(), curved.end() - static_cast<std::ptrdiff_t>(added[1]), curved.end());
            } else {
                g.insert(g.end(), after.begin(), after.end());
            }
            curved.swap(g);
        }
        guides[e] = curved;
    }
    bf.found = true;
    char readOk[2] = {0, 0};
    cv::parallel_for_(cv::Range(0, 2), [&](const cv::Range& range) {
        for (int e = range.start; e < range.end; ++e) {
            readOk[e] = readLine(gray, model, cloth, guides[e], cfg, bf.edges[e], e == 1) ? 1 : 0;
        }
    });
    bf.edges[0].role = "inner";
    bf.edges[1].role = "outer";
    for (int e = 0; e < 2; ++e) {
        bf.readOk[e] = readOk[e] != 0;
        if (!bf.readOk[e]) continue;
        CollarLine& L = bf.edges[e];
        // Where it sits from the frame line, outward from the opening.
        const std::vector<cv::Point2d> q = resampleEvery(L.path_mm, 2.0);
        double sum = 0.0;
        int count = 0;
        for (const cv::Point2d& p : q) {
            std::size_t near = 0;
            double nd = std::numeric_limits<double>::max();
            for (std::size_t k = 0; k < rpts.size(); ++k) {
                const double d = dist(p, rpts[k]);
                if (d < nd) {
                    nd = d;
                    near = k;
                }
            }
            if (near == 0 || near + 1 == rpts.size()) continue;
            sum += (p - rpts[near]).dot(rnrm[near]);
            ++count;
        }
        L.offset_mm = count ? sum / count : 0.0;
    }
    return bf;
}

}  // namespace

bool detectCollar(const cv::Mat& frame, const PlaneModel& model,
                  const std::vector<cv::Point2d>& outlineMm, const cv::Rect2d& boxPx,
                  const CollarRequest& req, const Config& cfg, CollarResult& out) {
    out = CollarResult();
    out.box_px = boxPx;
    out.edge = req.edge == "inner" ? "inner" : "outer";
    out.along_arc = req.along_arc;
    out.around = req.around;
    const auto refuse = [&](const std::string& kind, const std::string& why) {
        out.ok = false;
        out.refusal = kind;
        out.error = why;
        return false;
    };

    if (frame.empty()) return refuse("collar", "no held picture to find the collar in");
    // The back of the neck is the edge of the shirt, which is where the outer
    // edge ends and not where the opening's does.
    if (out.around && out.edge != "outer") {
        return refuse("collar", "a collar measured all the way round is measured along the band's outer edge");
    }
    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    if (gray.depth() != CV_8U) gray.convertTo(gray, CV_8U);

    const cv::Rect2d whole(0.0, 0.0, frame.cols, frame.rows);
    const cv::Rect2d box = boxPx & whole;
    out.box_px = box;
    out.box_mm = model.pxToMm({box.tl(), cv::Point2d(box.x + box.width, box.y), box.br(),
                               cv::Point2d(box.x, box.y + box.height)});
    const double boxW = std::max(dist(out.box_mm[0], out.box_mm[1]), dist(out.box_mm[3], out.box_mm[2]));
    const double boxH = std::max(dist(out.box_mm[0], out.box_mm[3]), dist(out.box_mm[1], out.box_mm[2]));
    if (boxW < 50.0 || boxH < 40.0) {
        return refuse("box", "that box is " + vjson::formatFixed(boxW, 0) + " by " +
                                 vjson::formatFixed(boxH, 0) +
                                 " mm, which is smaller than a collar. Draw it round the whole collar");
    }

    // The cloth and the mat, over the box and a margin round it: the lines are
    // read out past the box's edges before the box is asked whether it held
    // them.
    const std::pair<double, double> s = model.localMmPerPx(
        cv::Point2d(box.x + box.width / 2.0, box.y + box.height / 2.0));
    const double pxPerMm = 2.0 / std::max(1e-3, s.first + s.second);
    const int margin = static_cast<int>(std::lround((kGuideReachMm + 10.0) * pxPerMm));
    const cv::Rect region(static_cast<int>(std::floor(box.x)) - margin,
                          static_cast<int>(std::floor(box.y)) - margin,
                          static_cast<int>(std::ceil(box.width)) + 2 * margin,
                          static_cast<int>(std::ceil(box.height)) + 2 * margin);
    Cloth cloth;
    std::string why;
    if (!cloth.build(gray, model, outlineMm, region, why)) return refuse("box", why);
    // A dark garment's collar is read from a brightened picture (liftDarkCloth).
    if (cfg.collar_dark_below > 0.0 && frame.depth() == CV_8U) {
        liftDarkCloth(frame, cloth, box, region, cfg, gray, out.notes);
    }

    // -- which way the collar opens --
    //
    // A collar laid flat hangs from the edge of the shirt: its seam leaves the
    // edge at one shoulder, dips into the cloth and comes back out at the
    // other. So a box drawn round it has the mat along one side - the side the
    // collar opens toward - and the collar filling the rest. Which side is
    // read off the box's own four sides, because the operator draws the box
    // round the collar as it lies and its sides are the collar's own frame.
    // Taking the way from the mat to the cloth on average over the box was
    // tried first, and on a real shirt the mat under one sloping shoulder
    // tipped it by twenty-five degrees and put every guess half out of the box.
    //
    // Everything to the end of the first pass is in the box's own pixels:
    // along the open side, and into the box from it.
    struct Side {
        cv::Point2d origin, along, into;
        double lenA, lenB;
    };
    const double W = box.width, H = box.height;
    const Side sides[4] = {
        {cv::Point2d(box.x, box.y + H), cv::Point2d(1.0, 0.0), cv::Point2d(0.0, -1.0), W, H},  // bottom
        {cv::Point2d(box.x, box.y), cv::Point2d(1.0, 0.0), cv::Point2d(0.0, 1.0), W, H},       // top
        {cv::Point2d(box.x, box.y), cv::Point2d(0.0, 1.0), cv::Point2d(1.0, 0.0), H, W},       // left
        {cv::Point2d(box.x + W, box.y), cv::Point2d(0.0, 1.0), cv::Point2d(-1.0, 0.0), H, W},  // right
    };
    int open = -1;
    double openShare = 0.0;
    for (int s = 0; s < 4; ++s) {
        int mat = 0, seen = 0;
        for (int i = 0; i < 40; ++i) {
            for (int j = 0; j < 6; ++j) {
                const double a = (i + 0.5) / 40.0 * sides[s].lenA;
                const double b = (j + 0.5) / 6.0 * sides[s].lenB / 6.0;
                const signed char c = cloth.atPx(sides[s].origin + sides[s].along * a + sides[s].into * b);
                if (c < 0) continue;
                ++seen;
                if (c == 0) ++mat;
            }
        }
        const double share = seen ? static_cast<double>(mat) / seen : 0.0;
        if (share > openShare) {
            openShare = share;
            open = s;
        }
    }
    if (open < 0 || openShare < 0.15) {
        return refuse("box", "the box has to reach past the edge of the shirt at the neck, onto the mat: "
                             "the collar's seam ends there, at the shoulder points");
    }
    {
        int onCloth = 0, seen = 0;
        for (int j = 0; j < 30; ++j) {
            for (int i = 0; i < 40; ++i) {
                const signed char c =
                    cloth.atPx(cv::Point2d(box.x + (i + 0.5) * W / 40.0, box.y + (j + 0.5) * H / 30.0));
                if (c < 0) continue;
                ++seen;
                if (c == 1) ++onCloth;
            }
        }
        if (seen == 0 || onCloth < 0.3 * seen) {
            return refuse("box", "there is hardly any shirt in that box. Draw it round the collar");
        }
    }
    const Side& side = sides[open];
    const auto pxAt = [&](double a, double b) { return side.origin + side.along * a + side.into * b; };

    // -- the edge of the shirt across the box --
    //
    // For each place along the open side, how far into the box the cloth
    // begins: the first cloth met coming in off the mat.
    const double stepA = std::max(1.0, 2.0 * pxPerMm);
    const double stepB = std::max(0.5, 0.5 * pxPerMm);
    const int nA = std::max(2, static_cast<int>(side.lenA / stepA));
    const int nB = std::max(2, static_cast<int>(side.lenB / stepB));
    std::vector<double> edgeB(static_cast<std::size_t>(nA), std::numeric_limits<double>::quiet_NaN());
    for (int i = 0; i < nA; ++i) {
        bool sawMat = false;
        for (int j = 0; j < nB; ++j) {
            const signed char c = cloth.atPx(pxAt((i + 0.5) * stepA, j * stepB));
            if (c == 0) sawMat = true;
            if (c == 1 && sawMat) {
                edgeB[static_cast<std::size_t>(i)] = j * stepB;
                break;
            }
        }
    }
    const auto edgeAt = [&](double a) {
        const int i = std::max(0, std::min(nA - 1, static_cast<int>(std::lround(a / stepA - 0.5))));
        for (int r = 0; r < nA; ++r) {
            for (int sgn : {-1, 1}) {
                const int k = i + sgn * r;
                if (k >= 0 && k < nA && !std::isnan(edgeB[static_cast<std::size_t>(k)])) {
                    return edgeB[static_cast<std::size_t>(k)];
                }
            }
        }
        return std::numeric_limits<double>::quiet_NaN();
    };
    const double aMid = 0.5 * side.lenA;
    if (std::isnan(edgeAt(aMid))) {
        return refuse("box", "the edge of the shirt does not cross the box. Draw it round the collar and "
                             "a little of the mat past the shoulders");
    }

    // -- first pass: the box's guesses --
    //
    // Half-ellipses standing on the edge of the cloth near either side of the
    // box, rising toward the far side, at a few sizes. This is the only place
    // the box's size and position reach, and they reach it only as somewhere
    // to start: each guess is snapped to the line nearest it in the picture,
    // and the reading is then rebuilt round what was found rather than round
    // the guess. A box drawn a little differently guesses a little differently
    // and arrives at the same line.
    //
    // Half-ellipses and not arcs of a circle, and that was learned on a real
    // collar. A collar laid flat is a horseshoe: its legs come down to the
    // edge of the shirt nearly square to it, and its top is a flatter curve
    // than a circle through the same three points. The circle's legs lean
    // inward, so down one leg the guess sat across the band, the snap held to
    // whichever edge was nearer there, and the line it returned was the
    // attachment seam over the top and the opening's edge down one side.
    // An ellipse's legs stand upright, as the collar's do, and each leg stands
    // on its own foot, so a shirt laid a little askew - one shoulder lower on
    // the mat than the other - is guessed at as it lies.
    std::vector<std::vector<cv::Point2d>> guesses;
    const double base = edgeAt(aMid);
    const double footIn = 2.0 * pxPerMm;
    for (double k : {1.00, 0.88, 0.76, 0.64}) {
        const double ha = k * 0.46 * side.lenA;
        const double bL = edgeAt(aMid - ha) + footIn;
        const double bR = edgeAt(aMid + ha) + footIn;
        const double apex = base + k * 0.95 * (side.lenB - base);
        const double hb = apex - 0.5 * (bL + bR);
        if (std::isnan(bL) || std::isnan(bR) || hb < 15.0 * pxPerMm) continue;
        std::vector<cv::Point2d> g;
        const int n = 24;
        for (int i = 0; i <= n; ++i) {
            // From the foot at the start of the open side, over the top, to
            // the foot at its end.
            const double t = kPiRad * (1.0 - static_cast<double>(i) / n);
            const double feet = bR + (bL - bR) * (t / kPiRad);
            g.push_back(pxAt(aMid + ha * std::cos(t), feet + hb * std::sin(t)));
        }
        guesses.push_back(model.pxToMm(g));
    }
    if (guesses.empty()) {
        return refuse("box", "there is not enough of the shirt above its edge in that box to hold a collar");
    }
    out.guesses_mm = guesses;

    Config guess = cfg;
    guess.curve_max_drift_mm = std::numeric_limits<double>::max();
    guess.curve_min_margin = -std::numeric_limits<double>::max();
    std::vector<CurveResult> first(guesses.size());
    cv::parallel_for_(cv::Range(0, static_cast<int>(guesses.size())), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            snapCurve(gray, model, guesses[static_cast<std::size_t>(i)], guess,
                      first[static_cast<std::size_t>(i)]);
        }
    });

    // The frames to look at the collar in: every line the guesses held to,
    // strongest first and those that run from the edge of the cloth to the
    // edge of the cloth before those that do not, then the largest guess
    // itself. None has to be an edge of the band, nor even one line the whole
    // way round, because nothing is measured on a frame: the band is found in
    // stretches along it, each stretch on its own. But a frame has to run
    // alongside the collar, and one that followed a line of shading off round
    // a shoulder does not, so each is tried in turn until one shows the band.
    struct Frame {
        std::vector<cv::Point2d> path;
        double score = 0.0;
        std::string what;
    };
    std::vector<Frame> frames;
    for (std::size_t i = 0; i < first.size(); ++i) {
        out.first_mm.push_back(first[i].path_mm);
        if (!first[i].ok) {
            out.notes.push_back("guess " + std::to_string(i + 1) + ": nothing held, " + first[i].error);
            continue;
        }
        const Reached r = reachEdge(first[i].path_mm, cloth, kGuideReachMm);
        out.notes.push_back("guess " + std::to_string(i + 1) + ": a line " +
                            vjson::formatFixed(lengthOf(r.path), 0) + " mm edge to edge, confidence " +
                            vjson::formatFixed(first[i].confidence, 2) +
                            (r.meets[0] && r.meets[1] ? "" : ", not reaching the edge at both ends"));
        if (r.path.size() < 40) continue;
        frames.push_back({r.path, first[i].confidence + (r.meets[0] && r.meets[1] ? 10.0 : 0.0),
                          "the line guess " + std::to_string(i + 1) + " held to"});
    }
    std::stable_sort(frames.begin(), frames.end(), [](const Frame& a, const Frame& b) { return a.score > b.score; });
    // Then the guesses themselves, largest first. A frame only has to run
    // within reach of the band all the way round, not on any line, and when
    // a box is loose and off to one side of the collar no guess lies close
    // enough to one line for the snap to hold to it - but the one nearest the
    // collar's size still runs alongside it.
    for (std::size_t i = 0; i < guesses.size(); ++i) {
        const Reached r = reachEdge(guesses[i], cloth, kGuideReachMm);
        if (r.path.size() >= 40) frames.push_back({r.path, 0.0, "guess " + std::to_string(i + 1) + " itself"});
    }
    if (frames.empty()) {
        return refuse("collar", "nothing in the box runs round like a collar. Is the collar in the box, "
                                "laid flat and face up?");
    }

    // -- the band --
    //
    // Along each frame in turn: find the band and read its edges, then look
    // again along the outer edge just read, and again, until looking again
    // changes nothing - until the band's outer edge, looked for along the
    // edge that was read, is that edge all the way round and right down to
    // both shoulder points. Only then is a reading taken.
    //
    // The first look is along whatever line the box's guesses held to, and
    // past the stretches the band was found in, each edge is carried on the
    // way that line ran. Where it turned away from the collar the edge went
    // with it: on a real shirt, three millimetres into the band down the end
    // of one leg, meeting the edge of the cloth five millimetres from where a
    // different box put it. Looked at again along the band itself, nothing of
    // the box is left in the frame.
    //
    // And "until nothing changes" rather than "once more", because once more
    // was not enough. A frame that stopped short on one leg had the edge
    // carried on down the frame instead of down the collar; the second look,
    // along that edge, found the band everywhere except where it had gone
    // wrong, and read a rendered collar thirteen millimetres short of one
    // shoulder at full confidence. A reading that has settled has been
    // checked against the band along every stretch of its own length.
    const double kSettledMm = 1.5;
    const int kMaxLooks = 3;
    BandFind bf;
    bool partlyOutside = false;
    bool haveBand = false;
    std::string lastWhy;
    for (const Frame& f : frames) {
        BandFind cur = findBand(gray, model, cloth, f.path, cfg);
        out.notes.push_back("looked along " + f.what + ":");
        for (const std::string& n : cur.notes) out.notes.push_back(n);
        if (!cur.found) {
            lastWhy = cur.why;
            // Some of the band found, and some of that outside the box: the
            // box cut the collar, whatever else is wrong.
            if (cur.partial.size() >= 3) {
                for (const cv::Point2d& p : model.mmToPx(cur.partial)) {
                    if (!box.contains(p)) partlyOutside = true;
                }
            }
            continue;
        }
        for (int look = 0; look < kMaxLooks && !haveBand; ++look) {
            if (!cur.readOk[1] || !cur.edges[1].meets_edge) {
                lastWhy = "the band's outer edge could not be followed to the edge of the shirt";
                break;
            }
            BandFind next = findBand(gray, model, cloth, cur.edges[1].path_mm, cfg);
            out.notes.push_back("looked again, along the band's outer edge:");
            for (const std::string& n : next.notes) out.notes.push_back(n);
            if (!next.found) {
                lastWhy = "the band could not be found again along its own edge";
                break;
            }
            if (next.reachesEnds() && next.outerStray <= kSettledMm) {
                bf = next;
                haveBand = true;
                break;
            }
            lastWhy = next.reachesEnds() ? "the band's edge did not settle: it moved " +
                                               vjson::formatFixed(next.outerStray, 1) + " mm when looked at again"
                                         : "the band could not be followed down to both shoulder points";
            out.notes.push_back("  not settled: " + lastWhy);
            cur = next;
        }
        if (haveBand) break;
    }
    if (!haveBand && partlyOutside) {
        return refuse("box", "the collar runs out of the box. Draw it round the whole collar, with the mat past "
                             "both shoulders inside it");
    }
    if (!haveBand) {
        bf = BandFind();
        bf.why = lastWhy.empty() ? "could not find the collar band" : lastWhy;
    }
    if (!bf.found) {
        // Not a band the picture can show. What it did show is offered, each
        // line read properly, so an operator who can see which one is meant
        // can say so with a tap - and every number is still the line's own.
        for (std::size_t i = 0; i < first.size(); ++i) {
            if (!first[i].ok) continue;
            const Reached r = reachEdge(first[i].path_mm, cloth, kGuideReachMm);
            if (!r.meets[0] || !r.meets[1]) continue;
            CollarLine L;
            if (!readLine(gray, model, cloth, r.path, cfg, L) || !L.meets_edge) continue;
            bool same = false;
            for (const CollarLine& o : out.lines) {
                if (meanDistance(L.path_mm, o.path_mm) < kSameLineMm) same = true;
            }
            if (same) continue;
            const std::vector<cv::Point2d> px = model.mmToPx(L.path_mm);
            L.inside_box = true;
            for (const cv::Point2d& p : px) {
                if (!box.contains(p)) L.inside_box = false;
            }
            // A line offered for a step that measures the whole collar is
            // offered whole, back and all, or not at all: a tap has no picture
            // to read the back from.
            std::string noBack;
            if (out.around) {
                if (!readBack(gray, model, cloth, cfg, L, noBack)) continue;
                for (const cv::Point2d& p : model.mmToPx(L.back_path_mm)) {
                    if (!box.contains(p)) L.inside_box = false;
                }
            }
            out.lines.push_back(L);
        }
        if (!out.lines.empty()) {
            out.how = bf.why + ". Tap the line you mean";
            return refuse("choose", out.how);
        }
        return refuse("collar", bf.why + ". Is the collar laid flat, face up, with its band showing?");
    }

    int index[2] = {-1, -1};
    for (int e = 0; e < 2; ++e) {
        if (!bf.readOk[e]) continue;
        CollarLine& L = bf.edges[e];
        const std::vector<cv::Point2d> px = model.mmToPx(L.path_mm);
        L.inside_box = true;
        for (const cv::Point2d& p : px) {
            if (p.x < box.x - 0.5 || p.y < box.y - 0.5 || p.x > box.x + box.width + 0.5 ||
                p.y > box.y + box.height + 0.5) {
                L.inside_box = false;
                break;
            }
        }
        index[e] = static_cast<int>(out.lines.size());
        out.lines.push_back(L);
    }

    const double wMin = bf.wMin, wMax = bf.wMax;
    const int want = out.edge == "inner" ? 0 : 1;
    const std::string which = "the band's " + out.edge + " edge";
    out.how = "the collar band, " + vjson::formatFixed(wMin, 0) + " to " + vjson::formatFixed(wMax, 0) +
              " mm wide, found in " + std::to_string(bf.with) + " of " + std::to_string(bf.stretches) +
              " stretches; taking its " + out.edge + " edge";
    if (index[want] < 0) return refuse("collar", which + " was found but could not be followed along its length");
    CollarLine& L = out.lines[static_cast<std::size_t>(index[want])];
    // The two edges read where the band was found, or one of them strayed
    // onto a line beside it on the way.
    if (index[0] >= 0 && index[1] >= 0) {
        const double apart = out.lines[static_cast<std::size_t>(index[1])].offset_mm -
                             out.lines[static_cast<std::size_t>(index[0])].offset_mm;
        if (apart < wMin - 3.0 || apart > wMax + 3.0) {
            return refuse("collar", "the band's two edges read " + vjson::formatFixed(apart, 1) +
                                        " mm apart where the band is " + vjson::formatFixed(wMin, 0) + " to " +
                                        vjson::formatFixed(wMax, 0) + " mm wide: one strayed");
        }
    }
    if (!L.meets_edge) {
        return refuse("collar", which + " does not reach the edge of the shirt at both ends");
    }
    if (L.depth_mm < kMinDepthShare * L.chord_mm) {
        return refuse("collar", which + " does not dip like a neckline");
    }
    // The whole collar: on from the second shoulder point across the back of
    // the neck to the first. Inside the box too, which was drawn past the
    // edge of the shirt and so round the back of the neck.
    if (out.around) {
        std::string noBack;
        if (!readBack(gray, model, cloth, cfg, L, noBack)) return refuse("collar", noBack);
        for (const cv::Point2d& p : model.mmToPx(L.back_path_mm)) {
            if (p.x < box.x - 0.5 || p.y < box.y - 0.5 || p.x > box.x + box.width + 0.5 ||
                p.y > box.y + box.height + 0.5) {
                L.inside_box = false;
                break;
            }
        }
    }
    const double reading = out.around ? L.arc_mm + L.back_mm : out.along_arc ? L.arc_mm : L.chord_mm;
    if (req.target_mm > 1e-9 && std::fabs(reading - req.target_mm) > kGate * req.target_mm) {
        return refuse("collar", (out.around ? "the whole collar, front and back," : which) + " reads " +
                                    vjson::formatFixed(reading, 0) + " mm, not within a quarter of the " +
                                    vjson::formatFixed(req.target_mm, 0) +
                                    " mm target: this is not the collar the style describes");
    }
    // Nothing above asked the box anything: where the band is and which of
    // its edges the step wants are questions about the picture. The box is
    // asked one thing now, and it can only refuse - whether the line about to
    // be measured lies inside it. One that runs out of it means the box cut
    // the collar, and a reading of a cut collar would depend on where the cut
    // was.
    if (!L.inside_box) {
        return refuse("box", "the collar runs out of the box. Draw it round the whole collar, with the mat past "
                             "both shoulders inside it");
    }
    takeLine(out, index[want]);
    out.ok = true;
    return true;
}

bool pickCollarLine(CollarResult& r, const cv::Point2d& tapMm, double reachMm) {
    int best = -1;
    double bestD = reachMm;
    for (std::size_t k = 0; k < r.lines.size(); ++k) {
        const CollarLine& L = r.lines[k];
        if (!L.meets_edge || !L.inside_box || L.path_mm.size() < 2) continue;
        const double d = distanceToPath(tapMm, L.path_mm);
        if (d < bestD) {
            bestD = d;
            best = static_cast<int>(k);
        }
    }
    if (best < 0) return false;
    takeLine(r, best);
    r.ok = true;
    r.error.clear();
    r.refusal.clear();
    r.how = "tapped, " + vjson::formatFixed(bestD, 1) + " mm from the line";
    return true;
}

CurveResult CollarResult::asCurve() const {
    CurveResult c;
    c.ok = ok;
    c.error = error;
    c.along_arc = along_arc;
    if (ok && chosen >= 0 && chosen < static_cast<int>(lines.size())) {
        const CollarLine& L = lines[static_cast<std::size_t>(chosen)];
        c.chord_mm = L.chord_mm;
        c.arc_mm = L.arc_mm;
        c.bow_mm = L.depth_mm;
        c.radius_mm = L.radius_mm;
        c.roughness_mm = L.roughness_mm;
        c.confidence = L.confidence;
        c.margin = L.margin;
        c.path_mm = L.path_mm;
        c.seeds_mm = {end_mm[0], end_mm[1]};
    }
    return c;
}

vjson::Value CollarResult::toJson() const {
    const auto point = [](const cv::Point2d& p) {
        vjson::Value a = vjson::Value::array();
        a.push(vjson::Value::real(p.x));
        a.push(vjson::Value::real(p.y));
        return a;
    };
    vjson::Value v = vjson::Value::object();
    v.set("ok", vjson::Value::boolean(ok));
    if (!error.empty()) v.set("error", vjson::Value::str(error));
    if (!refusal.empty()) v.set("refusal", vjson::Value::str(refusal));
    v.set("edge", vjson::Value::str(edge));
    v.set("along_arc", vjson::Value::boolean(along_arc));
    if (around) v.set("around", vjson::Value::boolean(true));
    if (ok) {
        v.set("measured_mm", vjson::Value::real(measured_mm()));
        v.set("arc_mm", vjson::Value::real(arc_mm));
        v.set("chord_mm", vjson::Value::real(chord_mm));
        v.set("depth_mm", vjson::Value::real(depth_mm));
        if (around) v.set("back_mm", vjson::Value::real(back_mm));
        vjson::Value ends = vjson::Value::array();
        ends.push(point(end_mm[0]));
        ends.push(point(end_mm[1]));
        v.set("ends_mm", ends);
    }
    v.set("how", vjson::Value::str(how));
    v.set("chosen", vjson::Value::integer(chosen));
    vjson::Value boxPx = vjson::Value::array();
    boxPx.push(vjson::Value::real(box_px.x));
    boxPx.push(vjson::Value::real(box_px.y));
    boxPx.push(vjson::Value::real(box_px.width));
    boxPx.push(vjson::Value::real(box_px.height));
    v.set("box_px", boxPx);
    vjson::Value boxMm = vjson::Value::array();
    for (const cv::Point2d& p : box_mm) boxMm.push(point(p));
    v.set("box_mm", boxMm);
    vjson::Value all = vjson::Value::array();
    for (const CollarLine& L : lines) {
        vjson::Value l = vjson::Value::object();
        l.set("role", vjson::Value::str(L.role));
        l.set("arc_mm", vjson::Value::real(L.arc_mm));
        l.set("chord_mm", vjson::Value::real(L.chord_mm));
        l.set("depth_mm", vjson::Value::real(L.depth_mm));
        l.set("radius_mm", vjson::Value::real(L.radius_mm));
        l.set("offset_mm", vjson::Value::real(L.offset_mm));
        l.set("confidence", vjson::Value::real(L.confidence));
        l.set("margin", vjson::Value::real(L.margin));
        l.set("step_share", vjson::Value::real(L.step_share));
        l.set("meets_edge", vjson::Value::boolean(L.meets_edge));
        l.set("inside_box", vjson::Value::boolean(L.inside_box));
        if (around && !L.back_path_mm.empty()) l.set("back_mm", vjson::Value::real(L.back_mm));
        if (!L.path_mm.empty()) {
            l.set("from_mm", point(L.path_mm.front()));
            l.set("to_mm", point(L.path_mm.back()));
        }
        all.push(l);
    }
    v.set("lines", all);
    return v;
}

}  // namespace verimeasure
