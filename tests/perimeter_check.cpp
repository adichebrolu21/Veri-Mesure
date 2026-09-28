// perimeter_check: the full-perimeter step, with no camera and no garment.
//
//   perimeter_check --selftest
//   perimeter_check --case perimeter-fail.json [--draw out.png]
//
// The selftest renders a T-shirt of known perimeter onto a mat, lays it in two
// halves the way the station asks - one half in view with the clips in the
// band, the other past the bottom edge, then turned end for end - and puts
// each picture through the same code the station runs. What it checks is what
// the step claims:
//
//   * the two halves add up to the shirt: the fitted length to the pattern's
//     sharp-cornered perimeter, the traced length to the cloth's own;
//   * the clips are the join: moved along the edge, the halves change and the
//     total does not;
//   * where the shirt lies is not supposed to matter: shifted and turned a
//     little, the total does not move;
//   * a lay it cannot read is refused rather than measured: a half off the
//     edge of the picture, a clip missing or one too many, a clip at the
//     border, the same half laid twice.
//
// --case replays what a station saw when it refused a half, from the pair of
// files it leaves behind, and says why.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "verimeasure/calibration.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/cv_compat.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/perimeter.hpp"
#include "verimeasure/plane_model.hpp"

using namespace verimeasure;

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;

void check(bool ok, const std::string& what, const std::string& detail = std::string()) {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s%s%s\n", ok ? " ok " : "FAIL", what.c_str(), detail.empty() ? "" : ": ",
                detail.c_str());
}

std::string mm(double v) { return vjson::formatFixed(v, 2) + " mm"; }

double polyLength(const std::vector<cv::Point2d>& p, bool closed) {
    double L = 0.0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i) L += std::hypot(p[i + 1].x - p[i].x, p[i + 1].y - p[i].y);
    if (closed && p.size() > 2) L += std::hypot(p.front().x - p.back().x, p.front().y - p.back().y);
    return L;
}

// -- the shirt --
//
// In the garment's own millimetres: s runs down the body from the high
// shoulder point (0) to the hem (710), w across it from the middle. One side,
// w >= 0, from the middle of the back neck round to the middle of the hem;
// the other side is its mirror. Numbers are a men's medium tee.
struct Shirt {
    // The sharp outline of one side, as the pattern draws it: the back neck as
    // a curve, the rest straight edges meeting at corners.
    std::vector<cv::Point2d> side;
    std::vector<int> corner;    // indices into `side` that are corners
    std::vector<double> radius;  // how far the cloth rounds each one
    double scale = 1.0;
};

Shirt makeShirt(double scale) {
    Shirt t;
    t.scale = scale;
    // The back neck: s = 20 - 20 (w / 95)^2, from the middle out to the neck point.
    for (int k = 0; k <= 40; ++k) {
        const double w = 95.0 * k / 40.0;
        t.side.emplace_back(20.0 - 20.0 * (w / 95.0) * (w / 95.0), w);
    }
    const auto add = [&](double s, double w, double r) {
        t.corner.push_back(static_cast<int>(t.side.size()) - 1);
        t.radius.push_back(r);
        t.side.emplace_back(s, w);
    };
    // Each call closes the corner at the point before it with radius r.
    add(45.0, 230.0, 4.0);    // neck point -> shoulder point
    add(157.0, 425.0, 0.0);   // shoulder point (barely a corner) -> cuff, top
    add(308.0, 348.0, 4.0);   // cuff top corner -> cuff, underarm side
    add(230.0, 270.0, 4.0);   // cuff bottom corner -> underarm
    add(710.0, 267.0, 20.0);  // underarm -> hem corner
    add(710.0, 0.0, 6.0);     // hem corner -> middle of the hem
    for (cv::Point2d& p : t.side) p *= scale;
    for (double& r : t.radius) r *= scale;
    return t;
}

// Both sides, as one closed loop, starting at the middle of the back neck.
std::vector<cv::Point2d> wholeSharp(const Shirt& t, std::vector<int>* corners = nullptr,
                                    std::vector<double>* radii = nullptr) {
    std::vector<cv::Point2d> loop = t.side;  // middle of the neck -> middle of the hem
    std::vector<int> c = t.corner;
    std::vector<double> r = t.radius;
    // Back up the other side, mirrored: middle of the hem -> middle of the neck.
    const int n = static_cast<int>(t.side.size());
    for (int i = n - 2; i >= 1; --i) {
        loop.emplace_back(t.side[static_cast<std::size_t>(i)].x, -t.side[static_cast<std::size_t>(i)].y);
        for (std::size_t k = 0; k < t.corner.size(); ++k) {
            if (t.corner[k] == i) {
                c.push_back(static_cast<int>(loop.size()) - 1);
                r.push_back(t.radius[k]);
            }
        }
    }
    if (corners) *corners = c;
    if (radii) *radii = r;
    return loop;
}

// The loop with every corner rounded to its radius, sampled finely.
std::vector<cv::Point2d> roundedLoop(const Shirt& t) {
    std::vector<int> corners;
    std::vector<double> radii;
    const std::vector<cv::Point2d> sharp = wholeSharp(t, &corners, &radii);
    const int n = static_cast<int>(sharp.size());
    std::vector<double> r(static_cast<std::size_t>(n), 0.0);
    for (std::size_t k = 0; k < corners.size(); ++k) r[static_cast<std::size_t>(corners[k])] = radii[k];

    std::vector<cv::Point2d> out;
    const auto segment = [&](const cv::Point2d& a, const cv::Point2d& b) {
        const double L = std::hypot(b.x - a.x, b.y - a.y);
        const int steps = std::max(1, static_cast<int>(std::ceil(L / 0.2)));
        for (int k = 0; k < steps; ++k) out.push_back(a + (b - a) * (static_cast<double>(k) / steps));
    };
    // Where each vertex's fillet starts and ends.
    std::vector<cv::Point2d> t1(static_cast<std::size_t>(n)), t2(static_cast<std::size_t>(n));
    std::vector<bool> filleted(static_cast<std::size_t>(n), false);
    std::vector<cv::Point2d> centre(static_cast<std::size_t>(n));
    std::vector<double> a0(static_cast<std::size_t>(n)), sweep(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        t1[static_cast<std::size_t>(i)] = t2[static_cast<std::size_t>(i)] = sharp[static_cast<std::size_t>(i)];
        const double rad = r[static_cast<std::size_t>(i)];
        if (!(rad > 0.0)) continue;
        const cv::Point2d& V = sharp[static_cast<std::size_t>(i)];
        const cv::Point2d& A = sharp[static_cast<std::size_t>((i - 1 + n) % n)];
        const cv::Point2d& B = sharp[static_cast<std::size_t>((i + 1) % n)];
        cv::Point2d u1 = V - A, u2 = B - V;
        u1 *= 1.0 / std::hypot(u1.x, u1.y);
        u2 *= 1.0 / std::hypot(u2.x, u2.y);
        const double turn = std::acos(std::max(-1.0, std::min(1.0, u1.dot(u2))));
        if (turn < 1e-3) continue;
        const double tlen = rad * std::tan(turn / 2.0);
        const cv::Point2d p1 = V - u1 * tlen, p2 = V + u2 * tlen;
        const double cr = u1.x * u2.y - u1.y * u2.x;  // which way it turns
        const cv::Point2d n1 = cr > 0 ? cv::Point2d(-u1.y, u1.x) : cv::Point2d(u1.y, -u1.x);
        const cv::Point2d c = p1 + n1 * rad;
        t1[static_cast<std::size_t>(i)] = p1;
        t2[static_cast<std::size_t>(i)] = p2;
        centre[static_cast<std::size_t>(i)] = c;
        a0[static_cast<std::size_t>(i)] = std::atan2(p1.y - c.y, p1.x - c.x);
        sweep[static_cast<std::size_t>(i)] = cr > 0 ? turn : -turn;
        filleted[static_cast<std::size_t>(i)] = true;
    }
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        if (filleted[static_cast<std::size_t>(i)]) {
            const double rad = r[static_cast<std::size_t>(i)];
            const int steps = std::max(8, static_cast<int>(std::ceil(std::abs(sweep[static_cast<std::size_t>(i)]) * rad / 0.2)));
            for (int k = 0; k < steps; ++k) {
                const double a = a0[static_cast<std::size_t>(i)] + sweep[static_cast<std::size_t>(i)] * k / steps;
                out.push_back(centre[static_cast<std::size_t>(i)] + cv::Point2d(std::cos(a), std::sin(a)) * rad);
            }
        }
        segment(t2[static_cast<std::size_t>(i)], t1[static_cast<std::size_t>(j)]);
    }
    return out;
}

// -- the bench --
//
// A camera looking straight down at 0.33 mm a pixel: 2400 x 1600 pixels is
// 792 x 528 mm, a little bigger than the rig's view, with no lens distortion.
struct Bench {
    cv::Size size{2400, 1600};
    double mmpp = 0.33;
    RoiBox roi{8, 8, 2392, 1592};
    PlaneModelPtr model;
};

Bench makeBench() {
    Bench b;
    const cv::Matx33d H(1.0 / b.mmpp, 0.0, 0.0, 0.0, 1.0 / b.mmpp, 0.0, 0.0, 0.0, 1.0);
    b.model = std::make_shared<PlaneModel>(H, 0.0, 0.0, cv::Point2d(b.size.width / 2.0, b.size.height / 2.0),
                                           b.size.width / 2.0, 1.0);
    return b;
}

// Where the shirt lies: garment millimetres to mat millimetres. `turned` lays
// it end for end, which is how the second half comes into view.
struct Lay {
    double collarX = 22.0;  // where the high shoulder point is, along the view
    double bandY = 0.0;     // the middle of the shirt, down the view
    double angleDeg = 0.0;  // a little skew, about the middle of the band
    bool turned = false;
    cv::Point2d offset{0.0, 0.0};

    cv::Point2d place(const cv::Point2d& g, double length) const {
        // Collar at the left, the w > 0 half up the view; turned end for end,
        // the collar goes to the right and the w < 0 half comes up instead.
        cv::Point2d p = turned ? cv::Point2d(collarX + length - g.x, bandY + g.y)
                               : cv::Point2d(collarX + g.x, bandY - g.y);
        const cv::Point2d pivot(collarX + length / 2.0, bandY);
        const double a = angleDeg * kPi / 180.0;
        const cv::Point2d d = p - pivot;
        p = pivot + cv::Point2d(d.x * std::cos(a) - d.y * std::sin(a), d.x * std::sin(a) + d.y * std::cos(a));
        return p + offset;
    }
};

// A clip on the edge, at garment position `at`, lying along the edge's normal.
struct ClipAt {
    cv::Point2d at;      // on the edge, garment millimetres
    cv::Point2d normal;  // out of the cloth, garment millimetres
};

// A clip on the back neck, `alongNeck` millimetres across from the middle,
// and one on the hem, `alongHem` from the middle.
std::vector<ClipAt> clipsOn(const Shirt& t, double alongNeck, double alongHem) {
    std::vector<ClipAt> out;
    const double k = t.scale;
    // The back neck is s = k (20 - 20 (w / 95k)^2); out of the cloth is
    // towards smaller s, up past the collar.
    const double w = alongNeck;
    const double s = k * (20.0 - 20.0 * (w / (95.0 * k)) * (w / (95.0 * k)));
    const double dsdw = -40.0 * k * w / ((95.0 * k) * (95.0 * k));
    cv::Point2d normal(-1.0, dsdw);
    normal *= 1.0 / std::hypot(normal.x, normal.y);
    out.push_back({cv::Point2d(s, w), normal});
    out.push_back({cv::Point2d(710.0 * k, alongHem), cv::Point2d(1.0, 0.0)});
    return out;
}

struct Picture {
    cv::Mat frame;
    std::vector<cv::Point2d> outline;  // "the engine's": full-frame pixels
    std::vector<cv::Point2d> live;     // the decimated live outline, for the guide
};

cv::Point2d toPx(const Bench& b, const cv::Point2d& mm) {
    return b.model->mmToPx({mm}).front();
}

// Render the mat, the shirt and its clips, and outline them the way the
// engine would: everything that is not mat, the largest blob, every pixel of
// its boundary, resampled every three pixels, with the scatter the sub-pixel
// refinement leaves on it.
Picture photograph(const Bench& b, const Shirt& t, const Lay& lay, const std::vector<ClipAt>& clips,
                   unsigned seed) {
    Picture pic;
    const cv::Scalar mat(52, 58, 30), line(170, 182, 176), cloth(222, 230, 236);
    const cv::Scalar clipColour(24, 206, 236);  // a yellow sewing clip
    pic.frame = cv::Mat(b.size, CV_8UC3, mat);
    // The mat's printed grid, every ten millimetres.
    for (double x = 0.0; x < b.size.width * b.mmpp; x += 10.0) {
        const int px = static_cast<int>(std::lround(x / b.mmpp));
        cv::line(pic.frame, cv::Point(px, 0), cv::Point(px, b.size.height - 1), line, 1);
    }
    for (double y = 0.0; y < b.size.height * b.mmpp; y += 10.0) {
        const int py = static_cast<int>(std::lround(y / b.mmpp));
        cv::line(pic.frame, cv::Point(0, py), cv::Point(b.size.width - 1, py), line, 1);
    }

    const double length = 710.0 * t.scale;
    std::vector<cv::Point> poly;
    for (const cv::Point2d& g : roundedLoop(t)) {
        const cv::Point2d q = toPx(b, lay.place(g, length));
        poly.emplace_back(static_cast<int>(std::lround(q.x * 8.0)), static_cast<int>(std::lround(q.y * 8.0)));
    }
    cv::fillPoly(pic.frame, std::vector<std::vector<cv::Point>>{poly}, cloth, cv::LINE_8, 3);
    // A red print across the chest, which is not a clip.
    {
        const cv::Point2d a = toPx(b, lay.place(cv::Point2d(250.0 * t.scale, 20.0), length));
        cv::circle(pic.frame, cv::Point(static_cast<int>(a.x), static_cast<int>(a.y)), 60, cv::Scalar(40, 40, 200), -1);
    }
    for (const ClipAt& c : clips) {
        // 27 x 11 mm, 16 mm of it over the cloth and 11 beyond the edge.
        const cv::Point2d n = c.normal * (1.0 / std::hypot(c.normal.x, c.normal.y));
        const cv::Point2d tan(-n.y, n.x);
        std::vector<cv::Point> quad;
        for (const cv::Point2d& corner : {cv::Point2d(-16.0, -5.5), cv::Point2d(11.0, -5.5),
                                          cv::Point2d(11.0, 5.5), cv::Point2d(-16.0, 5.5)}) {
            const cv::Point2d g = c.at + n * corner.x + tan * corner.y;
            const cv::Point2d q = toPx(b, lay.place(g, length));
            quad.emplace_back(static_cast<int>(std::lround(q.x * 8.0)), static_cast<int>(std::lround(q.y * 8.0)));
        }
        cv::fillPoly(pic.frame, std::vector<std::vector<cv::Point>>{quad}, clipColour, cv::LINE_8, 3);
    }

    // The segmentation, reduced to what matters here: not mat. Drawn again at
    // three times the resolution for the outline, because the engine's is
    // refined to a fraction of a pixel and a pixel staircase at this camera's
    // third of a millimetre would be a coarser boundary than any the station
    // hands over.
    const cv::Rect area = b.roi.toRect();
    const int up = 3;
    cv::Mat fine(area.height * up, area.width * up, CV_8UC1, cv::Scalar(0));
    {
        const auto fineShape = [&](const std::vector<cv::Point2d>& gs) {
            std::vector<cv::Point> q;
            for (const cv::Point2d& g : gs) {
                const cv::Point2d px = toPx(b, lay.place(g, length));
                q.emplace_back(static_cast<int>(std::lround(((px.x - area.x) * up + (up - 1) / 2.0) * 8.0)),
                               static_cast<int>(std::lround(((px.y - area.y) * up + (up - 1) / 2.0) * 8.0)));
            }
            cv::fillPoly(fine, std::vector<std::vector<cv::Point>>{q}, cv::Scalar(255), cv::LINE_8, 3);
        };
        fineShape(roundedLoop(t));
        for (const ClipAt& c : clips) {
            const cv::Point2d n = c.normal * (1.0 / std::hypot(c.normal.x, c.normal.y));
            const cv::Point2d tan(-n.y, n.x);
            std::vector<cv::Point2d> quad;
            for (const cv::Point2d& corner : {cv::Point2d(-16.0, -5.5), cv::Point2d(11.0, -5.5),
                                              cv::Point2d(11.0, 5.5), cv::Point2d(-16.0, 5.5)}) {
                quad.push_back(c.at + n * corner.x + tan * corner.y);
            }
            fineShape(quad);
        }
    }
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fine, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::size_t best = 0;
    double bestArea = 0.0;
    for (std::size_t k = 0; k < contours.size(); ++k) {
        const double a = cv::contourArea(contours[k]);
        if (a > bestArea) {
            bestArea = a;
            best = k;
        }
    }
    if (contours.empty()) return pic;

    std::vector<cv::Point2d> chain;
    for (const cv::Point& q : contours[best]) {
        chain.emplace_back((q.x - (up - 1) / 2.0) / up + area.x, (q.y - (up - 1) / 2.0) / up + area.y);
    }
    // Every three pixels of arc, as the engine resamples its refined boundary.
    std::vector<cv::Point2d> resampled;
    double need = 0.0;
    for (std::size_t i = 0; i < chain.size(); ++i) {
        const cv::Point2d a = chain[i], c = chain[(i + 1) % chain.size()];
        const double L = std::hypot(c.x - a.x, c.y - a.y);
        double pos = 0.0;
        while (L - pos >= need) {
            pos += need;
            resampled.push_back(a + (c - a) * (L > 0 ? pos / L : 0.0));
            need = 3.0;
        }
        need -= (L - pos);
    }
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.3);
    for (cv::Point2d& q : resampled) q += cv::Point2d(noise(rng), noise(rng));
    pic.outline = resampled;

    // The live pass works on the picture as the camera took it.
    cv::Mat crop = pic.frame(area);
    cv::Mat diff;
    cv::absdiff(crop, cv::Scalar(52, 58, 30), diff);
    cv::Mat gray;
    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
    cv::Mat mask = gray > 60;
    cv::Mat hsv;
    cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch;
    cv::split(hsv, ch);
    cv::Mat gridLine = (ch[1] < 60) & (ch[2] < 200) & (ch[2] > 150);
    mask.setTo(0, gridLine);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    // And the live pass's coarse outline, six pixels to a cell.
    cv::Mat small;
    cv::resize(mask, small, cv::Size(mask.cols / 6, mask.rows / 6), 0, 0, cv::INTER_AREA);
    small = small > 127;
    std::vector<std::vector<cv::Point>> coarse;
    cv::findContours(small, coarse, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::size_t big = 0;
    double bigArea = 0.0;
    for (std::size_t k = 0; k < coarse.size(); ++k) {
        const double a = cv::contourArea(coarse[k]);
        if (a > bigArea) {
            bigArea = a;
            big = k;
        }
    }
    if (!coarse.empty()) {
        for (const cv::Point& q : coarse[big]) pic.live.emplace_back(q.x * 6 + 2.5 + area.x, q.y * 6 + 2.5 + area.y);
    }
    return pic;
}

struct Truth {
    double sharp = 0.0;    // the pattern: every corner sharp
    double rounded = 0.0;  // the cloth: every corner rounded
};

Truth truthOf(const Shirt& t) {
    Truth tr;
    tr.sharp = polyLength(wholeSharp(t), true);
    tr.rounded = polyLength(roundedLoop(t), true);
    return tr;
}

Config benchConfig() {
    Config cfg;
    cfg.detect_step = 6;
    return cfg;
}

bool capture(const Bench& b, const Picture& pic, const Config& cfg, PerimeterCapture& out,
             double correction = 1.0) {
    std::vector<cv::Point2d> outline = pic.outline;
    const cv::Point2d centre = b.model->centre();
    for (cv::Point2d& q : outline) q = centre + (q - centre) * correction;
    return measurePerimeterCapture(outline, correction, centre, pic.frame, *b.model, b.roi,
                                   clipColourFrom(cfg), cfg, out);
}

// Both halves of one lay pair: the first with the collar left, the second
// turned end for end. Returns false if either was refused.
bool twoHalves(const Bench& b, const Shirt& t, Lay first, Lay second, const std::vector<ClipAt>& clips,
               const Config& cfg, PerimeterResult& result, std::string& why, unsigned seed,
               double correction = 1.0) {
    second.turned = true;
    const Picture p1 = photograph(b, t, first, clips, seed);
    const Picture p2 = photograph(b, t, second, clips, seed + 1);
    PerimeterCapture c1, c2;
    if (!capture(b, p1, cfg, c1, correction)) {
        why = "first half: " + c1.error;
        return false;
    }
    if (!capture(b, p2, cfg, c2, correction)) {
        why = "second half: " + c2.error;
        return false;
    }
    if (!turnedEndForEnd(c1.half[0], c2.half[0], why)) return false;
    result = joinPerimeter(c1.half[0], c2.half[0], false, false);
    return true;
}

// -- geometry of one path, on its own --

void geometryChecks(const Config& cfg) {
    std::printf("\nOne path, measured on its own\n");
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.15);

    // A straight edge with the scatter a real outline carries.
    {
        std::vector<cv::Point2d> p;
        for (int i = 0; i <= 1250; ++i) p.emplace_back(0.4 * i + noise(rng), noise(rng));
        PerimeterHalf h;
        measurePerimeterPath(p, cfg, h);
        const double raw = polyLength(p, false);
        check(std::abs(h.fitted_mm - 500.0) < 0.3, "500 mm straight edge, fitted", mm(h.fitted_mm));
        check(std::abs(h.traced_mm - 500.0) < 0.3, "500 mm straight edge, traced", mm(h.traced_mm));
        check(raw > 505.0, "the same edge followed point to point reads long", mm(raw));
        check(h.corners == 0, "no corner on a straight edge", std::to_string(h.corners));
    }
    // Three sides of a rectangle, corners rounded to 10 mm.
    {
        std::vector<cv::Point2d> sharp{{0, 200}, {0, 0}, {400, 0}, {400, 200}};
        const double r = 10.0;
        std::vector<cv::Point2d> p;
        const auto run = [&](cv::Point2d a, cv::Point2d b2) {
            const double L = std::hypot(b2.x - a.x, b2.y - a.y);
            for (double s = 0.0; s < L; s += 0.4) p.push_back(a + (b2 - a) * (s / L));
        };
        const auto arc = [&](cv::Point2d c, double a0, double a1) {
            const int k = 60;
            for (int i = 0; i < k; ++i) {
                const double a = a0 + (a1 - a0) * i / k;
                p.push_back(c + cv::Point2d(std::cos(a), std::sin(a)) * r);
            }
        };
        run({0, 200}, {0, r});
        arc({r, r}, kPi, 1.5 * kPi);
        run({r, 0}, {400 - r, 0});
        arc({400 - r, r}, 1.5 * kPi, 2.0 * kPi);
        run({400, r}, {400, 200});
        p.emplace_back(400, 200);
        for (cv::Point2d& q : p) q += cv::Point2d(noise(rng), noise(rng));
        PerimeterHalf h;
        measurePerimeterPath(p, cfg, h);
        const double sharpLen = 800.0;
        const double roundLen = 800.0 - 2.0 * (2.0 - kPi / 2.0) * r;
        check(std::abs(h.fitted_mm - sharpLen) < 0.8, "U of 800 mm, corners at 10 mm: fitted to the sharp corners",
              mm(h.fitted_mm));
        check(std::abs(h.traced_mm - roundLen) < 0.8, "and traced along the rounded cloth, " + mm(roundLen),
              mm(h.traced_mm));
        check(h.corners == 2 && h.crossed == 2, "two corners found and crossed",
              std::to_string(h.corners) + " found, " + std::to_string(h.crossed) + " crossed");
    }
    // A straight edge with a notch in it, and one with a spike: the outline
    // went somewhere and came back, and the edge ran straight on underneath.
    for (const double depth : {5.5, -8.0}) {
        std::vector<cv::Point2d> p;
        for (int i = 0; i <= 1250; ++i) {
            const double x = 0.4 * i;
            const double d = std::abs(x - 250.0);
            const double y = d < 4.0 ? depth * (1.0 - d / 4.0) : 0.0;  // a V, 8 mm across
            p.emplace_back(x + noise(rng), y + noise(rng));
        }
        PerimeterHalf h;
        measurePerimeterPath(p, cfg, h);
        check(std::abs(h.fitted_mm - 500.0) < 0.5 && h.defects >= 1,
              std::string("500 mm edge with a ") + (depth > 0 ? "notch" : "spike") + " " +
                  vjson::formatFixed(std::abs(depth), 1) + " mm deep, bridged",
              mm(h.fitted_mm) + ", " + std::to_string(h.defects) + " bridged, " + std::to_string(h.corners) +
                  " corners");
    }
    // A quarter circle: curved all the way, with no corner in it.
    {
        std::vector<cv::Point2d> p;
        for (double a = 0.0; a <= kPi / 2.0; a += 0.4 / 200.0) {
            p.emplace_back(200.0 * std::cos(a) + noise(rng), 200.0 * std::sin(a) + noise(rng));
        }
        PerimeterHalf h;
        measurePerimeterPath(p, cfg, h);
        check(std::abs(h.fitted_mm - kPi * 100.0) < 0.6, "quarter circle of 200 mm radius, fitted, 314.16 mm",
              mm(h.fitted_mm));
        check(h.crossed == 0, "a curve is not a corner", std::to_string(h.crossed) + " crossed");
    }
}

// -- the step, end to end --

void stepChecks(const Config& cfg) {
    const Bench b = makeBench();
    const Shirt shirt = makeShirt(1.0);
    const Truth truth = truthOf(shirt);
    std::printf("\nA shirt of %s as the pattern draws it, %s along the cloth\n", mm(truth.sharp).c_str(),
                mm(truth.rounded).c_str());

    // The view's bottom edge in mat millimetres, and the band 40 mm inside it.
    const double bottomMm = (b.roi.y2 - 1) * b.mmpp;
    Lay base;
    base.bandY = bottomMm - 40.0;
    base.collarX = 30.0;
    const std::vector<ClipAt> clips = clipsOn(shirt, 0.0, 0.0);

    PerimeterResult res;
    std::string why;
    const bool ok = twoHalves(b, shirt, base, base, clips, cfg, res, why, 11);
    check(ok, "two halves read", why);
    if (!ok) return;
    std::printf("    halves %s + %s (fitted), %s + %s (traced), %d of %d corners crossed, reach %s\n",
                mm(res.half[0].fitted_mm).c_str(), mm(res.half[1].fitted_mm).c_str(),
                mm(res.half[0].traced_mm).c_str(), mm(res.half[1].traced_mm).c_str(), res.crossed,
                res.corners, mm(res.reach_mm).c_str());
    if (g_verbose) {
        for (int h = 0; h < 2; ++h) {
            std::printf("    half %d corners:", h + 1);
            for (const PerimeterCorner& k : res.half[h].corner_list) {
                std::printf(" [%.0f deg %s %.2f]", k.turn_deg, k.crossed ? "crossed" : "followed", k.reach_mm);
            }
            std::printf("\n");
        }
    }
    const double fitErr = res.fitted_mm - truth.sharp;
    const double traceErr = res.traced_mm - truth.rounded;
    check(std::abs(fitErr) < 3.0, "fitted total against the pattern's " + mm(truth.sharp),
          mm(res.fitted_mm) + ", " + (fitErr >= 0 ? "+" : "") + mm(fitErr));
    check(std::abs(traceErr) < 6.0, "traced total against the cloth's " + mm(truth.rounded),
          mm(res.traced_mm) + ", " + (traceErr >= 0 ? "+" : "") + mm(traceErr));
    check(std::abs(res.difference_mm) < 3.0, "the two halves of a symmetric shirt agree",
          (res.difference_mm >= 0 ? "+" : "") + mm(res.difference_mm));

    // The same lay photographed again: what the scatter alone does. A reading
    // of this shirt scatters by about 0.7 mm, one standard deviation, from the
    // boundary's own scatter; the thresholds below are set from that.
    {
        double rlo = res.fitted_mm, rhi = res.fitted_mm;
        for (unsigned seed = 101; seed < 109; seed += 2) {
            PerimeterResult r;
            std::string w;
            if (twoHalves(b, shirt, base, base, clips, cfg, r, w, seed)) {
                rlo = std::min(rlo, r.fitted_mm);
                rhi = std::max(rhi, r.fitted_mm);
            }
        }
        check(rhi - rlo < 3.0, "the same lay read five times", "spread " + mm(rhi - rlo));
    }

    // The clips are the join: wherever they are put, the total is the same.
    // Folding a shirt to find its middle puts a clip within a centimetre or so
    // of it; the last pair is far out, and the shirt is laid skewed the way
    // the guide asks, so both clips still sit in the band. Each is read six
    // times and the averages compared, so the scatter cannot hide a total that
    // follows the clips.
    std::printf("\nClips moved along the edge\n");
    struct Moved {
        double neck, hem, skew;
    };
    double lo = 1e18, hi = -1e18;
    for (const Moved& at : {Moved{0.0, 0.0, 0.0}, Moved{12.0, -10.0, 0.0}, Moved{-12.0, 10.0, 0.0},
                            Moved{8.0, 15.0, 0.0}, Moved{-15.0, -8.0, 0.0}, Moved{30.0, -30.0, -5.0}}) {
        Lay a = base, c = base;
        a.angleDeg = at.skew;
        c.angleDeg = at.skew;
        double sum = 0.0;
        int count = 0;
        std::string w, halves;
        for (unsigned seed = 201; seed < 213; seed += 2) {
            PerimeterResult r;
            if (!twoHalves(b, shirt, a, c, clipsOn(shirt, at.neck, at.hem), cfg, r, w, seed)) break;
            sum += r.fitted_mm;
            ++count;
            if (halves.empty()) halves = "halves " + mm(r.half[0].fitted_mm) + " + " + mm(r.half[1].fitted_mm);
        }
        const bool fine = count == 6;
        const double mean = fine ? sum / count : 0.0;
        check(fine, "neck clip " + vjson::formatFixed(at.neck, 0) + " mm, hem clip " + vjson::formatFixed(at.hem, 0) +
                        " mm off the middle" + (at.skew != 0.0 ? ", laid " + vjson::formatFixed(at.skew, 0) + " degrees" : ""),
              fine ? "average " + mm(mean) + " (" + halves + ")" : w);
        if (fine) {
            lo = std::min(lo, mean);
            hi = std::max(hi, mean);
        }
    }
    check(hi - lo < 1.0, "the total does not follow the clips", "averages spread " + mm(hi - lo));

    // Where the shirt lies does not matter, within the band.
    std::printf("\nThe shirt laid differently\n");
    lo = res.fitted_mm;
    hi = res.fitted_mm;
    struct Variant {
        double dx, dy, angle1, angle2;
    };
    for (const Variant& v : {Variant{12.0, -8.0, 1.5, -1.0}, Variant{-10.0, 6.0, -2.0, 2.0},
                             Variant{4.0, -12.0, 0.8, 1.6}}) {
        Lay a = base, c = base;
        a.offset = cv::Point2d(v.dx, v.dy);
        a.angleDeg = v.angle1;
        c.offset = cv::Point2d(-v.dx * 0.5, v.dy);
        c.angleDeg = v.angle2;
        PerimeterResult r;
        std::string w;
        const bool fine = twoHalves(b, shirt, a, c, clips, cfg, r, w, 31);
        if (fine && g_verbose) {
            for (int h = 0; h < 2; ++h) {
                std::printf("    half %d: %.2f fitted, corners:", h + 1, r.half[h].fitted_mm);
                for (const PerimeterCorner& k : r.half[h].corner_list) {
                    std::printf(" [%.0f deg %s %.2f]", k.turn_deg, k.crossed ? "crossed" : "followed", k.reach_mm);
                }
                std::printf("%c", 10);
            }
        }
        check(fine, "shifted " + vjson::formatFixed(v.dx, 0) + ", " + vjson::formatFixed(v.dy, 0) +
                        " mm and turned " + vjson::formatFixed(v.angle1, 1) + " / " +
                        vjson::formatFixed(v.angle2, 1) + " degrees",
              fine ? mm(r.fitted_mm) : w);
        if (fine) {
            lo = std::min(lo, r.fitted_mm);
            hi = std::max(hi, r.fitted_mm);
        }
    }
    check(hi - lo < 3.0, "the total does not follow the lay", "spread " + mm(hi - lo));

    // The fold height reaches the length exactly as it reaches an edge step.
    {
        PerimeterResult r;
        std::string w;
        const double c = 0.99;
        const bool fine = twoHalves(b, shirt, base, base, clips, cfg, r, w, 11, c);
        check(fine && std::abs(r.fitted_mm - c * res.fitted_mm) < 0.8,
              "a 1% height correction takes 1% off the total",
              fine ? mm(r.fitted_mm) + " against " + mm(c * res.fitted_mm) : w);
    }

    // Refusals: a lay the station cannot stand behind is not measured.
    std::printf("\nLays that are refused\n");
    {
        Lay up = base;
        up.bandY = bottomMm - 150.0;  // clips 150 mm in: the sleeve leaves by the top
        PerimeterCapture c;
        const bool read = capture(b, photograph(b, shirt, up, clips, 41), cfg, c);
        check(!read && c.error.find("top") != std::string::npos, "a half running off the top is refused",
              read ? "it was read" : c.error);
    }
    {
        PerimeterCapture c;
        const std::vector<ClipAt> one{clips.front()};
        const bool read = capture(b, photograph(b, shirt, base, one, 42), cfg, c);
        check(!read && c.error.find("only one clip") != std::string::npos, "one clip is refused",
              read ? "it was read" : c.error);
    }
    {
        PerimeterCapture c;
        const bool read = capture(b, photograph(b, shirt, base, {}, 43), cfg, c);
        check(!read && c.error.find("no clips") != std::string::npos, "no clips is refused",
              read ? "it was read" : c.error);
    }
    {
        std::vector<ClipAt> three = clips;
        three.push_back({cv::Point2d(600.0, 267.0), cv::Point2d(0.0, 1.0)});  // on the side seam
        PerimeterCapture c;
        const bool read = capture(b, photograph(b, shirt, base, three, 44), cfg, c);
        check(!read && c.error.find("3 clips") != std::string::npos, "three clips are refused",
              read ? "it was read" : c.error);
    }
    {
        Lay low = base;
        low.bandY = bottomMm - 4.0;  // clips on the border itself
        PerimeterCapture c;
        const bool read = capture(b, photograph(b, shirt, low, clips, 45), cfg, c);
        check(!read, "clips at the very edge of the picture are refused", read ? "it was read" : c.error);
    }
    {
        // The same half twice: lifted and laid again without being turned.
        const Picture p1 = photograph(b, shirt, base, clips, 46);
        Lay again = base;
        again.offset = cv::Point2d(6.0, -3.0);
        const Picture p2 = photograph(b, shirt, again, clips, 47);
        PerimeterCapture c1, c2;
        std::string w;
        const bool both = capture(b, p1, cfg, c1) && capture(b, p2, cfg, c2);
        const bool caught = both && !turnedEndForEnd(c1.half[0], c2.half[0], w);
        check(caught, "the same half laid twice is caught",
              both ? (caught ? w : "it was not") : c1.error + c2.error);
    }

    // A shirt small enough to lie wholly in view is read in one picture.
    std::printf("\nA shirt that fits the view\n");
    {
        const Shirt small = makeShirt(0.58);
        const Truth st = truthOf(small);
        Lay whole;
        whole.collarX = 120.0;
        whole.bandY = 260.0;
        PerimeterCapture c;
        const bool read = capture(b, photograph(b, small, whole, clipsOn(small, 0.0, 0.0), 51), cfg, c);
        check(read && c.whole && c.halves == 2, "read whole, both halves from one picture",
              read ? (c.whole ? "whole" : "one half only") : c.error);
        if (read && c.whole) {
            const PerimeterResult r = joinPerimeter(c.half[0], c.half[1], false, true);
            check(std::abs(r.fitted_mm - st.sharp) < 4.0, "against the pattern's " + mm(st.sharp),
                  mm(r.fitted_mm));
        }
    }

    // The guide says what is wrong in words, and nothing when nothing is.
    std::printf("\nThe guide\n");
    {
        const Picture p = photograph(b, shirt, base, clips, 61);
        PerimeterGuide g;
        evaluatePerimeterGuide(p.live, p.frame, *b.model, b.roi, clipColourFrom(cfg), cfg, g);
        check(g.ok && g.clip_list.size() == 2 && g.clips_in_band == 2, "a good lay passes", g.message);
    }
    {
        Lay up = base;
        up.bandY = bottomMm - 150.0;
        const Picture p = photograph(b, shirt, up, clips, 62);
        PerimeterGuide g;
        evaluatePerimeterGuide(p.live, p.frame, *b.model, b.roi, clipColourFrom(cfg), cfg, g);
        check(!g.ok && g.margin[static_cast<int>(PerimeterSide::Top)] && g.message.find("down") != std::string::npos,
              "a half over the top is told to slide down", g.message);
    }
    {
        Lay high = base;
        high.bandY = bottomMm - 85.0;  // on the view, clips just out of the band
        const Picture p = photograph(b, shirt, high, clips, 63);
        PerimeterGuide g;
        evaluatePerimeterGuide(p.live, p.frame, *b.model, b.roi, clipColourFrom(cfg), cfg, g);
        check(!g.ok && g.clips_in_band == 0 && g.message.find("band") != std::string::npos,
              "clips above the band are pointed at it", g.message);
    }
    {
        const Picture p = photograph(b, shirt, base, {clips.front()}, 64);
        PerimeterGuide g;
        evaluatePerimeterGuide(p.live, p.frame, *b.model, b.roi, clipColourFrom(cfg), cfg, g);
        check(!g.ok && g.message.find("one clip") != std::string::npos, "a missing clip is named", g.message);
    }
}

// -- how much of a difference is the scatter --
//
// The same lays photographed many times with fresh scatter each time, so a
// difference between two lays can be told from the noise in either.
void repeatability(const Config& cfg, int count) {
    const Bench b = makeBench();
    const Shirt shirt = makeShirt(1.0);
    const Truth truth = truthOf(shirt);
    const double bottomMm = (b.roi.y2 - 1) * b.mmpp;
    Lay base;
    base.bandY = bottomMm - 40.0;
    base.collarX = 30.0;
    struct Case {
        const char* name;
        double neck, hem, skew;
    };
    std::printf("pattern %s, cloth %s, %d photographs of each lay\n", mm(truth.sharp).c_str(),
                mm(truth.rounded).c_str(), count);
    for (const Case& c : {Case{"clips at the middle", 0, 0, 0}, Case{"clips 12 / -10 off", 12, -10, 0},
                          Case{"clips 30 / -30 off, laid -5 deg", 30, -30, -5}}) {
        std::vector<double> v;
        for (int k = 0; k < count; ++k) {
            Lay a = base, d = base;
            a.angleDeg = c.skew;
            d.angleDeg = c.skew;
            PerimeterResult r;
            std::string w;
            if (twoHalves(b, shirt, a, d, clipsOn(shirt, c.neck, c.hem), cfg, r, w, 1000u + 2u * k)) v.push_back(r.fitted_mm);
        }
        double mean = 0, sd = 0;
        for (double x : v) mean += x;
        mean /= std::max<std::size_t>(1, v.size());
        for (double x : v) sd += (x - mean) * (x - mean);
        sd = v.size() > 1 ? std::sqrt(sd / (v.size() - 1)) : 0.0;
        std::printf("  %-34s mean %s, sd %s, over %zu\n", c.name, mm(mean).c_str(), mm(sd).c_str(),
                    v.size());
    }
}

// -- replaying a station's case --

int replay(const std::string& path, const std::string& drawPath) {
    vjson::Value v;
    std::string error;
    if (!vjson::parseFile(path, v, error)) {
        std::printf("could not read %s: %s\n", path.c_str(), error.c_str());
        return 2;
    }
    const std::filesystem::path dir = std::filesystem::path(path).parent_path();
    const auto nextTo = [&](const std::string& p) {
        const std::filesystem::path f(p);
        return (f.is_absolute() || dir.empty()) ? f.string() : (dir / f).string();
    };
    const cv::Mat frame = cv::imread(nextTo(v.getString("frame")), cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::printf("could not read the frame %s\n", v.getString("frame").c_str());
        return 2;
    }
    Config cfg;
    cfg.calibration_file = nextTo(v.getString("calibration", "calibration.json"));
    const PlaneModelPtr model = loadCalibration(cfg);
    if (!model) {
        std::printf("could not load the calibration %s\n", cfg.calibration_file.c_str());
        return 2;
    }
    RoiBox roi;
    if (const vjson::Value* r = v.find("roi"); r && r->isArray() && r->size() == 4) {
        roi = RoiBox{static_cast<int>(r->at(0).asNumber()), static_cast<int>(r->at(1).asNumber()),
                     static_cast<int>(r->at(2).asNumber()), static_cast<int>(r->at(3).asNumber())};
    }
    std::vector<cv::Point2d> outline;
    if (const vjson::Value* o = v.find("outline_px"); o && o->isArray()) {
        for (std::size_t k = 0; k < o->size(); ++k) {
            const vjson::Value& q = o->at(k);
            if (q.isArray() && q.size() == 2) outline.emplace_back(q.at(0).asNumber(), q.at(1).asNumber());
        }
    }
    cv::Point2d centre = model->centre();
    if (const vjson::Value* c = v.find("centre_px"); c && c->isArray() && c->size() == 2) {
        centre = cv::Point2d(c->at(0).asNumber(), c->at(1).asNumber());
    }
    ClipColour colour = clipColourFrom(cfg);
    if (const vjson::Value* h = v.find("clip_hue_deg"); h && h->isArray() && h->size() == 2) {
        colour.hue_lo_deg = h->at(0).asNumber();
        colour.hue_hi_deg = h->at(1).asNumber();
    }
    cfg.perimeter_join_side = v.getString("join_side", cfg.perimeter_join_side);

    PerimeterCapture cap;
    const bool ok = measurePerimeterCapture(outline, v.getNumber("correction", 1.0), centre, frame, *model, roi,
                                            colour, cfg, cap);
    std::printf("%s\n", cap.toJson().dump(2).c_str());
    if (g_verbose) {
        for (int h = 0; h < cap.halves; ++h) {
            const PerimeterHalf& half = cap.half[h];
            std::printf("half %d: fitted %.2f, traced %.2f, reach %.2f, defects %d, far_u %.3f\n", h + 1, half.fitted_mm,
                        half.traced_mm, half.reach_mm, half.defects, half.far_u);
            for (const PerimeterCorner& k : half.corner_list) {
                std::printf("   corner at (%.1f, %.1f) mm, turn %.1f deg, %s, reach %.2f\n", k.at_mm.x, k.at_mm.y,
                            k.turn_deg, k.crossed ? "crossed" : "followed", k.reach_mm);
            }
        }
    }
    std::printf("%s\n", ok ? "read" : ("refused: " + cap.error).c_str());
    if (!v.getString("refused").empty()) std::printf("the station said: %s\n", v.getString("refused").c_str());

    if (!drawPath.empty()) {
        const double s = 0.25;
        cv::Mat out;
        cv::resize(frame, out, cv::Size(), s, s, cv::INTER_AREA);
        const auto pts = [&](const std::vector<cv::Point2d>& in) {
            std::vector<cv::Point> p;
            for (const cv::Point2d& q : in) p.emplace_back(static_cast<int>(q.x * s), static_cast<int>(q.y * s));
            return p;
        };
        cv::rectangle(out, cv::Point(static_cast<int>(roi.x1 * s), static_cast<int>(roi.y1 * s)),
                      cv::Point(static_cast<int>(roi.x2 * s), static_cast<int>(roi.y2 * s)), cv::Scalar(90, 90, 90), 1);
        cv::polylines(out, std::vector<std::vector<cv::Point>>{pts(outline)}, true, cv::Scalar(230, 230, 230), 1,
                      cv::LINE_AA);
        for (const PerimeterClip& c : findClips(frame, roi, *model, colour, cfg, true)) {
            cv::circle(out, cv::Point(static_cast<int>(c.px.x * s), static_cast<int>(c.px.y * s)), 10,
                       cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
        }
        for (int h = 0; h < cap.halves; ++h) {
            cv::polylines(out, std::vector<std::vector<cv::Point>>{pts(cap.half[h].path_px)}, false,
                          cv::Scalar(90, 185, 138), 2, cv::LINE_AA);
            for (const cv::Point2d& q : cap.half[h].corner_px) {
                cv::circle(out, cv::Point(static_cast<int>(q.x * s), static_cast<int>(q.y * s)), 4,
                           cv::Scalar(90, 185, 138), -1, cv::LINE_AA);
            }
            for (const cv::Point2d& q : cap.half[h].cut_px) {
                cv::circle(out, cv::Point(static_cast<int>(q.x * s), static_cast<int>(q.y * s)), 5,
                           cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
            }
        }
        if (!cv::imwrite(drawPath, out)) std::printf("could not write %s\n", drawPath.c_str());
    }
    return ok ? 0 : 1;
}

void usage() {
    std::printf("usage: perimeter_check --selftest [--verbose]\n"
                "       perimeter_check --case perimeter-fail.json [--draw out.png]\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string casePath, drawPath;
    bool selftest = false;
    int repeatCount = 0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--selftest") {
            selftest = true;
        } else if (a == "--case" && i + 1 < argc) {
            casePath = argv[++i];
        } else if (a == "--draw" && i + 1 < argc) {
            drawPath = argv[++i];
        } else if (a == "--verbose") {
            g_verbose = true;
        } else if (a == "--repeat" && i + 1 < argc) {
            repeatCount = std::atoi(argv[++i]);
        } else {
            usage();
            return 2;
        }
    }
    if (!casePath.empty()) return replay(casePath, drawPath);
    if (repeatCount > 0) {
        repeatability(benchConfig(), repeatCount);
        return 0;
    }
    if (!selftest) {
        usage();
        return 2;
    }

    const Config cfg = benchConfig();
    geometryChecks(cfg);
    stepChecks(cfg);
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
