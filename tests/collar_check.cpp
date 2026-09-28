/*
Collar detection harness. Draws a box round a collar a dozen different ways and
checks the answer, with no camera attached.

    collar_check --selftest
    collar_check --frame lay.png --box X,Y,W,H [--calibration calibration.json]
                 [--edge outer|inner] [--chord] [--target MM] [--draw out.png]
    collar_check --case collar-fail.json [--draw out.png]
    collar_check --write-sample DIR

The step this tests takes the operator out of the reading. The box they draw is
a region and not a position: it says where the collar is and nothing else, and
everything the reading carries - where each line lies, which line is the
step's, where it starts and where it stops - comes from the picture. So the
claim to test is the one a measurement has to meet, stated as strongly as it
can be: draw the box differently and the number does not move.

The collar is rendered in millimetres through a plane model with this bench's
scale, the way a shirt laid face up presents one - and with every thing that
fooled an earlier version of this on a real shirt: a rib band between the
opening's edge and the attachment seam, a dark groove at the seam with a
dotted topstitch three millimetres outside it, the opening in shadow with the
back of the collar across the bottom of it, a line of shading fifteen
millimetres outside the seam where the body bunches up, folds, knit texture,
sensor noise and a mat with its printed grid. The attachment seam is a circle
of known radius running to the edge of the cloth at both shoulders, so its
length between the shoulder points is known exactly rather than measured by
something else.
*/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "verimeasure/collar.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/plane_model.hpp"
#include "verimeasure/qc.hpp"
#include "verimeasure/template.hpp"

using namespace verimeasure;

namespace {

const double kPi = 3.14159265358979;

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    std::printf(ok ? "  ok    %s\n" : "  FAIL  %s\n", what.c_str());
    if (!ok) ++g_failures;
}

void checkNear(double got, double want, double tol, const std::string& what) {
    char buf[260];
    std::snprintf(buf, sizeof(buf), "%s  (got %.3f, want %.3f, within %.3f)", what.c_str(), got, want, tol);
    check(std::fabs(got - want) <= tol, buf);
}

void checkBelow(double got, double limit, const std::string& what) {
    char buf[260];
    std::snprintf(buf, sizeof(buf), "%s  (got %.3f, limit %.3f)", what.c_str(), got, limit);
    check(got <= limit, buf);
}

bool parseBoxText(const std::string& text, cv::Rect2d& box) {
    double v[4] = {0, 0, 0, 0};
    std::size_t i = 0;
    for (int k = 0; k < 4; ++k) {
        const std::size_t comma = text.find(',', i);
        v[k] = std::atof(text.substr(i, comma == std::string::npos ? std::string::npos : comma - i).c_str());
        if (comma == std::string::npos) {
            if (k != 3) return false;
            break;
        }
        i = comma + 1;
    }
    box = cv::Rect2d(v[0], v[1], v[2], v[3]);
    return box.width > 0.0 && box.height > 0.0;
}

// A deterministic generator, so a failure can be reproduced from the line that
// reported it rather than from a lucky rerun.
struct Rng {
    unsigned long long s;
    explicit Rng(unsigned long long seed) : s(seed ? seed : 1) {}
    double uniform() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<double>(s >> 11) / 9007199254740992.0;
    }
    double signed1() { return uniform() * 2.0 - 1.0; }
};

// ---- printing and drawing a result ----

void printResult(const CollarResult& r) {
    for (const std::string& n : r.notes) std::printf("  . %s\n", n.c_str());
    std::printf("%d lines read in the box:\n", static_cast<int>(r.lines.size()));
    for (std::size_t k = 0; k < r.lines.size(); ++k) {
        const CollarLine& L = r.lines[k];
        std::printf("  %c %-6s offset %+6.1f  arc %6.1f  chord %6.1f  depth %5.1f  conf %.2f  margin %+.3f"
                    "  step %.2f  %s%s\n",
                    static_cast<int>(k) == r.chosen ? '*' : ' ', L.role.empty() ? "-" : L.role.c_str(),
                    L.offset_mm, L.arc_mm, L.chord_mm, L.depth_mm, L.confidence, L.margin, L.step_share,
                    L.meets_edge ? "edge-to-edge" : "stops short", L.inside_box ? "" : ", leaves the box");
    }
    if (r.ok && r.around) {
        std::printf("READ: the whole collar, %.2f mm = %.2f over the front (%s edge) + %.2f across the back "
                    "(chord %.2f, depth %.2f) - %s\n",
                    r.measured_mm(), r.arc_mm, r.edge.c_str(), r.back_mm, r.chord_mm, r.depth_mm, r.how.c_str());
        std::printf("      shoulder points (%.2f, %.2f) and (%.2f, %.2f) mm\n", r.end_mm[0].x, r.end_mm[0].y,
                    r.end_mm[1].x, r.end_mm[1].y);
    } else if (r.ok) {
        std::printf("READ: %s edge, %s %.2f mm (arc %.2f, chord %.2f, depth %.2f) - %s\n", r.edge.c_str(),
                    r.along_arc ? "along" : "across", r.measured_mm(), r.arc_mm, r.chord_mm, r.depth_mm,
                    r.how.c_str());
        std::printf("      shoulder points (%.2f, %.2f) and (%.2f, %.2f) mm\n", r.end_mm[0].x, r.end_mm[0].y,
                    r.end_mm[1].x, r.end_mm[1].y);
    } else {
        std::printf("NOT READ (%s): %s\n", r.refusal.c_str(), r.error.c_str());
    }
}

void drawResult(const cv::Mat& frame, const PlaneModel& model, const CollarResult& r, const std::string& path) {
    cv::Mat over;
    if (frame.channels() == 1) {
        cv::cvtColor(frame, over, cv::COLOR_GRAY2BGR);
    } else {
        over = frame.clone();
    }
    const auto poly = [&](const std::vector<cv::Point2d>& mm) {
        std::vector<cv::Point> out;
        for (const cv::Point2d& p : model.mmToPx(mm)) {
            out.emplace_back(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
        }
        return out;
    };
    cv::rectangle(over, r.box_px, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    for (const std::vector<cv::Point2d>& g : r.guesses_mm) {
        for (const cv::Point& p : poly(g)) cv::circle(over, p, 7, cv::Scalar(160, 160, 160), 2, cv::LINE_AA);
    }
    if (std::getenv("COLLAR_DRAW_FIRST")) {
        const cv::Scalar ink[] = {cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 128, 255),
                                  cv::Scalar(255, 128, 0)};
        for (std::size_t k = 0; k < r.first_mm.size(); ++k) {
            const std::vector<cv::Point> line = poly(r.first_mm[k]);
            if (line.size() >= 2) cv::polylines(over, line, false, ink[k % 4], 3, cv::LINE_AA);
        }
    }
    for (std::size_t k = 0; k < r.lines.size(); ++k) {
        const std::vector<cv::Point> line = poly(r.lines[k].path_mm);
        if (line.size() < 2) continue;
        const bool chosen = static_cast<int>(k) == r.chosen;
        cv::polylines(over, line, false, cv::Scalar(0, 0, 0), chosen ? 9 : 5, cv::LINE_AA);
        cv::polylines(over, line, false, chosen ? cv::Scalar(80, 255, 80) : cv::Scalar(60, 170, 255),
                      chosen ? 4 : 2, cv::LINE_AA);
        cv::putText(over, vjson::formatFixed(r.lines[k].arc_mm, 1), line[line.size() / 2] + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    if (r.ok && r.back_path_mm.size() >= 2) {
        const std::vector<cv::Point> back = poly(r.back_path_mm);
        cv::polylines(over, back, false, cv::Scalar(0, 0, 0), 9, cv::LINE_AA);
        cv::polylines(over, back, false, cv::Scalar(80, 255, 80), 4, cv::LINE_AA);
        cv::putText(over, vjson::formatFixed(r.back_mm, 1), back[back.size() / 2] + cv::Point(8, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    if (r.ok) {
        for (const cv::Point& p : poly({r.end_mm[0], r.end_mm[1]})) {
            cv::circle(over, p, 16, cv::Scalar(0, 0, 0), 6, cv::LINE_AA);
            cv::circle(over, p, 16, cv::Scalar(80, 255, 80), 3, cv::LINE_AA);
        }
    }
    // A close look at one place, the lines a pixel wide over the raw picture,
    // magnified: which stitch a line is on cannot be seen under a thick one.
    if (const char* z = std::getenv("COLLAR_ZOOM")) {
        cv::Rect2d zr;
        if (parseBoxText(z, zr)) {
            cv::Mat thin;
            if (frame.channels() == 1) {
                cv::cvtColor(frame, thin, cv::COLOR_GRAY2BGR);
            } else {
                thin = frame.clone();
            }
            const cv::Scalar ink[] = {cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0),
                                      cv::Scalar(0, 0, 255), cv::Scalar(255, 128, 0), cv::Scalar(0, 255, 255)};
            for (std::size_t k = 0; k < r.lines.size(); ++k) {
                cv::polylines(thin, poly(r.lines[k].path_mm), false, ink[k % 6], 1, cv::LINE_8);
            }
            cv::Mat big;
            cv::resize(thin(cv::Rect(zr) & cv::Rect(0, 0, thin.cols, thin.rows)), big, cv::Size(), 3.0, 3.0,
                       cv::INTER_NEAREST);
            const std::string zp = path + ".zoom.png";
            if (cv::imwrite(zp, big)) {
                std::printf("drew %s (lines in order: magenta, cyan, green, red, blue, yellow)\n", zp.c_str());
            }
        }
    }
    // The box and a margin round it: the whole frame is too big to look at.
    const int m = 250;
    cv::Rect crop(static_cast<int>(r.box_px.x) - m, static_cast<int>(r.box_px.y) - m,
                  static_cast<int>(r.box_px.width) + 2 * m, static_cast<int>(r.box_px.height) + 2 * m);
    crop &= cv::Rect(0, 0, over.cols, over.rows);
    if (crop.area() > 0 && cv::imwrite(path, over(crop))) std::printf("drew %s\n", path.c_str());
}

// ---- the bench ----
//
// A plane model with this bench's scale, 0.1378 mm per pixel, no distortion and
// no tilt. The point of the test is the detection, not the model, and a model
// with terms in it that the truth generator does not share would be testing
// the wrong thing.
const double kPxPerMm = 1.0 / 0.13787740569262713;
const int kFrameW = 2600;
const int kFrameH = 1800;

PlaneModelPtr benchModel() {
    cv::Matx33d H(kPxPerMm, 0.0, 40.0, 0.0, kPxPerMm, 40.0, 0.0, 0.0, 1.0);
    return std::make_shared<PlaneModel>(H);
}

// ---- a collar, laid flat ----
//
// Built in the collar's own frame, the way the bench sees a shirt laid with
// its neck to the rail: the garment above, the edge of the cloth along the
// bottom, the collar a horseshoe standing on that edge. Rotated and placed
// onto the mat by `angle` and `at`, so the same collar can be laid any way
// round.
struct Collar {
    cv::Point2d at{180.0, 190.0};  // where the middle of the shoulder line sits on the mat
    double angle = 0.0;             // radians; zero is neck toward +y on the mat
    double radius = 88.0;           // the attachment seam, a circle of this radius...
    double drop = 5.0;              // ...centred this far above the edge of the cloth
    double band = 14.0;             // rib width: the opening's edge is this far inside the seam
    double backBand = 12.0;         // the back of the collar across the bottom of the opening
    double topstitch = 3.0;         // a dotted stitch this far outside the seam
    double shading = 15.0;          // a soft step in the body this far outside the seam
    bool drawCollar = true;
    bool drawGarment = true;        // false: the empty mat, for a station to settle on first

    // Collar frame <-> mat. In the collar frame the edge of the cloth is the
    // line y = 0, the garment is y < 0, and the seam's centre is (0, -drop).
    cv::Point2d toMat(const cv::Point2d& c) const {
        const double cs = std::cos(angle), sn = std::sin(angle);
        return cv::Point2d(at.x + c.x * cs - c.y * sn, at.y + c.x * sn + c.y * cs);
    }
    cv::Point2d toCollar(const cv::Point2d& m) const {
        const double cs = std::cos(angle), sn = std::sin(angle);
        const double dx = m.x - at.x, dy = m.y - at.y;
        return cv::Point2d(dx * cs + dy * sn, -dx * sn + dy * cs);
    }
    // The edge of the cloth: level across the collar, the shoulders falling
    // away either side of it.
    double edgeY(double x) const {
        const double flat = radius + 6.0;
        return std::fabs(x) <= flat ? 0.0 : -0.35 * (std::fabs(x) - flat);
    }
    // One of the collar's circles - the seam at `r` = radius, the opening at
    // radius - band - where it reaches the edge of the cloth, and its length
    // over the top between those two points.
    double footX(double r) const { return std::sqrt(r * r - drop * drop); }
    double arcOf(double r) const {
        // The circle below its centre is cut off by the edge; the rest, over
        // the top, is the collar.
        const double half = std::asin(drop / r);  // angle below the centre's level to the edge
        return r * (kPi + 2.0 * half);
    }
    double chordOf(double r) const { return 2.0 * footX(r); }
    cv::Point2d footMat(double r, int side) const { return toMat(cv::Point2d(side * footX(r), 0.0)); }
    double trueArc() const { return arcOf(radius); }
    double trueChord() const { return chordOf(radius); }
    double innerArc() const { return arcOf(radius - band); }

    // The garment's outline on the mat, a polygon round the part in view.
    std::vector<cv::Point2d> outline() const {
        std::vector<cv::Point2d> c;
        for (double x = -400.0; x <= 400.0; x += 1.0) c.emplace_back(x, edgeY(x));
        c.emplace_back(400.0, -700.0);
        c.emplace_back(-400.0, -700.0);
        std::vector<cv::Point2d> m;
        for (const cv::Point2d& p : c) m.push_back(toMat(p));
        return m;
    }
};

// Render the lay: the mat and its grid, the garment, the collar.
cv::Mat renderCollar(const PlaneModel& model, const Collar& col, Rng& rng) {
    std::vector<cv::Point2d> px;
    px.reserve(static_cast<std::size_t>(kFrameW) * kFrameH);
    for (int y = 0; y < kFrameH; ++y) {
        for (int x = 0; x < kFrameW; ++x) px.emplace_back(x, y);
    }
    const std::vector<cv::Point2d> mm = model.pxToMm(px);

    // Knit texture, a millimetre across.
    cv::Mat fine(kFrameH / 6, kFrameW / 6, CV_32F);
    for (int y = 0; y < fine.rows; ++y) {
        float* r = fine.ptr<float>(y);
        for (int x = 0; x < fine.cols; ++x) r[x] = static_cast<float>(rng.signed1() * 5.0);
    }
    cv::Mat texture;
    cv::resize(fine, texture, cv::Size(kFrameW, kFrameH), 0, 0, cv::INTER_CUBIC);

    const double R = col.radius;
    const double Ri = col.radius - col.band;
    cv::Mat img(kFrameH, kFrameW, CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        float* out = img.ptr<float>(y);
        const float* tex = texture.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) {
            const cv::Point2d& p = mm[static_cast<std::size_t>(y) * kFrameW + x];
            const cv::Point2d c = col.toCollar(p);
            // Soft edge of the cloth, a quarter of a millimetre.
            const double outside = c.y - col.edgeY(c.x);
            const double onCloth = col.drawGarment ? 0.5 * (1.0 - std::tanh(outside / 0.25)) : 0.0;

            // The mat: dark, with a bright printed line every ten millimetres.
            double matV = 36.0;
            const double gx = std::fabs(std::remainder(p.x, 10.0));
            const double gy = std::fabs(std::remainder(p.y, 10.0));
            if (gx < 0.3 || gy < 0.3) matV = 95.0;

            double v = 150.0;
            if (col.drawCollar) {
                const double dx = c.x, dy = c.y + col.drop;
                const double r = std::hypot(dx, dy);
                const double toSeam = r - R;  // signed, outward
                // The body round the collar, and a line of shading well outside
                // the seam where it bunches up: soft, over several millimetres.
                v += 9.0 * 0.5 * (1.0 + std::tanh((toSeam - col.shading) / 2.5));
                if (r < R) {
                    // Inside the seam: the rib band, a shade lighter, its wales
                    // running across it.
                    const double wales = std::sin(std::atan2(dy, dx) * r / 0.28);
                    v = 162.0 + 3.0 * wales;
                    if (r < Ri) {
                        // Inside the band: the opening, the inside of the back of
                        // the shirt in shadow - or, across the bottom of it, the
                        // back of the collar.
                        v = (c.y > -col.backBand) ? 148.0 : 112.0;
                        if (c.y <= -col.backBand) {
                            // Folds in the back panel.
                            v += 10.0 * std::exp(-std::pow((c.x - 0.35 * c.y - 12.0) / 3.0, 2.0));
                            v -= 12.0 * std::exp(-std::pow((c.x + 0.5 * c.y + 30.0) / 4.0, 2.0));
                        }
                    }
                }
                // The opening's edge: the band's own edge throws a thin shadow
                // onto what is under it.
                v -= 18.0 * std::exp(-std::pow((r - (Ri - 0.7)) / 0.45, 2.0)) * (c.y <= -col.backBand ? 1.0 : 0.4);
                // The back of the collar's own stitched edge.
                if (r < Ri) v -= 12.0 * std::exp(-std::pow((c.y + col.backBand) / 0.4, 2.0));
                // The attachment seam: a narrow dark groove.
                v -= 30.0 * std::exp(-std::pow(toSeam / 0.4, 2.0));
                // The topstitch outside it, dotted: three millimetres a stitch.
                const double along = std::atan2(dy, dx) * (R + col.topstitch);
                const double dot = std::fabs(std::remainder(along, 3.0)) < 1.0 ? 1.0 : 0.25;
                v -= 16.0 * dot * std::exp(-std::pow((toSeam - col.topstitch) / 0.35, 2.0));
            }
            v += tex[x];
            // Lighting across the bench, gentle and unavoidable.
            v += 0.03 * (p.x - 180.0);
            out[x] = static_cast<float>(onCloth * v + (1.0 - onCloth) * matV);
        }
    }
    cv::GaussianBlur(img, img, cv::Size(0, 0), 0.9);
    cv::Mat noise(img.size(), CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        float* r = noise.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) r[x] = static_cast<float>((rng.signed1() + rng.signed1() + rng.signed1()) * 1.4);
    }
    img += noise;
    cv::Mat gray;
    img.convertTo(gray, CV_8U);
    // In colour, as the camera delivers it; the salmon of the first real shirt.
    std::vector<cv::Mat> ch = {gray * 0.62, gray * 0.78, gray * 1.05};
    for (cv::Mat& c : ch) c.convertTo(c, CV_8U);
    cv::Mat bgr;
    cv::merge(ch, bgr);
    return bgr;
}

// The box round a collar as an operator might draw it: the collar's own
// extent on the picture, loosened by `grow` millimetres all round, shifted by
// `shift`, and stretched by `tall` along the collar's height.
cv::Rect2d boxAround(const PlaneModel& model, const Collar& col, double grow, cv::Point2d shift = {0.0, 0.0},
                     double tall = 0.0) {
    std::vector<cv::Point2d> c;
    const double R = col.radius;
    for (double t = 0.0; t <= 2.0 * kPi; t += 0.05) {
        const cv::Point2d q(R * std::cos(t), -col.drop + R * std::sin(t));
        if (q.y <= 0.0) c.push_back(col.toMat(q));
    }
    // The edge of the cloth beneath the feet, and a little mat past it.
    c.push_back(col.toMat(cv::Point2d(-col.footX(R), 4.0)));
    c.push_back(col.toMat(cv::Point2d(col.footX(R), 4.0)));
    c.push_back(col.toMat(cv::Point2d(0.0, -R - col.drop - tall)));
    const std::vector<cv::Point2d> px = model.mmToPx(c);
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const cv::Point2d& p : px) {
        x0 = std::min(x0, p.x);
        y0 = std::min(y0, p.y);
        x1 = std::max(x1, p.x);
        y1 = std::max(y1, p.y);
    }
    const double g = grow * kPxPerMm;
    const double sx = shift.x * kPxPerMm, sy = shift.y * kPxPerMm;
    return cv::Rect2d(x0 - g + sx, y0 - g + sy, (x1 - x0) + 2 * g, (y1 - y0) + 2 * g);
}

// ---- the checks ----

struct Case {
    const PlaneModel& model;
    const Config& cfg;
};

CollarResult read(const cv::Mat& frame, const Case& k, const Collar& col, const cv::Rect2d& box,
                  const std::string& edge = "outer", bool alongArc = true, double target = 0.0,
                  bool withOutline = true) {
    CollarRequest req;
    req.edge = edge;
    req.along_arc = alongArc;
    req.target_mm = target;
    CollarResult r;
    detectCollar(frame, k.model, withOutline ? col.outline() : std::vector<cv::Point2d>(), box, req, k.cfg, r);
    return r;
}

void testReadsTheSeam(const Case& k) {
    std::printf("\nThe attachment seam, from the edge of the cloth to the edge of the cloth\n");
    Collar col;
    Rng rng(1101);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    const CollarResult r = read(frame, k, col, boxAround(k.model, col, 12.0), "outer", true, 280.0);
    check(r.ok, "the collar is read: " + r.error);
    if (!r.ok) {
        printResult(r);
        return;
    }
    checkNear(r.arc_mm, col.trueArc(), 1.0, "along the seam, mm");
    checkNear(r.chord_mm, col.trueChord(), 1.0, "and straight between its ends, mm");
    const cv::Point2d feet[2] = {col.footMat(col.radius, -1), col.footMat(col.radius, 1)};
    double worst = 0.0;
    for (const cv::Point2d& e : {r.end_mm[0], r.end_mm[1]}) {
        worst = std::max(worst, std::min(std::hypot(e.x - feet[0].x, e.y - feet[0].y),
                                         std::hypot(e.x - feet[1].x, e.y - feet[1].y)));
    }
    checkBelow(worst, 1.0, "each end is the shoulder point, where the seam meets the edge, mm");
    // On the seam, not beside it.
    double sum = 0.0, far = 0.0;
    int n = 0;
    const CollarLine& L = r.lines[static_cast<std::size_t>(r.chosen)];
    for (std::size_t i = 4; i + 4 < L.path_mm.size(); ++i) {
        const cv::Point2d c = col.toCollar(L.path_mm[i]);
        const double d = std::fabs(std::hypot(c.x, c.y + col.drop) - col.radius);
        sum += d * d;
        far = std::max(far, d);
        ++n;
    }
    checkBelow(std::sqrt(sum / std::max(1, n)), 0.25, "the line read sits on the seam, mm RMS");
    check(r.how.find("band") != std::string::npos, "and it was chosen as the band's outer edge: " + r.how);
}

void testTheBoxDoesNotMatter(const Case& k) {
    std::printf("\nThe box is a region, not a position: twelve boxes, one number\n");
    Collar col;
    Rng rng(2202);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    struct B {
        double grow;
        cv::Point2d shift;
        double tall;
        const char* what;
    };
    const B boxes[] = {
        {8.0, {0.0, 0.0}, 0.0, "tight"},
        {25.0, {0.0, 0.0}, 0.0, "loose"},
        {45.0, {0.0, 0.0}, 0.0, "very loose"},
        {20.0, {12.0, 0.0}, 0.0, "shifted right"},
        {20.0, {-12.0, 0.0}, 0.0, "shifted left"},
        {12.0, {0.0, -12.0}, 0.0, "shifted up"},
        {18.0, {0.0, 6.0}, 0.0, "shifted down, still holding the edge"},
        {10.0, {0.0, 0.0}, 30.0, "tall"},
        {20.0, {10.0, -8.0}, 0.0, "loose and off to one side"},
        {6.0, {3.0, 2.0}, 0.0, "tight and a little off"},
        {30.0, {-12.0, -10.0}, 20.0, "big and off the other way"},
        {15.0, {0.0, 0.0}, 12.0, "a little tall"},
    };
    std::vector<double> arcs;
    std::vector<cv::Point2d> lefts, rights;
    for (const B& b : boxes) {
        const CollarResult r = read(frame, k, col, boxAround(k.model, col, b.grow, b.shift, b.tall), "outer", true,
                                    280.0);
        char got[160];
        std::snprintf(got, sizeof(got), ": %.2f mm, ends (%.2f, %.2f) and (%.2f, %.2f)", r.arc_mm, r.end_mm[0].x,
                      r.end_mm[0].y, r.end_mm[1].x, r.end_mm[1].y);
        check(r.ok, std::string("read from a ") + b.what + " box" + (r.ok ? got : ": " + r.error));
        if (!r.ok) continue;
        arcs.push_back(r.arc_mm);
        const bool swap = r.end_mm[0].x > r.end_mm[1].x;
        lefts.push_back(swap ? r.end_mm[1] : r.end_mm[0]);
        rights.push_back(swap ? r.end_mm[0] : r.end_mm[1]);
    }
    if (arcs.size() < 2) return;
    const auto spreadOf = [](const std::vector<cv::Point2d>& p) {
        double worst = 0.0;
        for (const cv::Point2d& a : p) {
            for (const cv::Point2d& b : p) worst = std::max(worst, std::hypot(a.x - b.x, a.y - b.y));
        }
        return worst;
    };
    const double spread = *std::max_element(arcs.begin(), arcs.end()) - *std::min_element(arcs.begin(), arcs.end());
    checkBelow(spread, 0.3, "the reading's spread over every box, mm");
    // Half a millimetre, where the reading itself holds to a tenth: a shoulder
    // point can slide a little way along the edge of the cloth as the last
    // few millimetres of the seam tilt, and a slide along that edge hardly
    // changes the length of a leg that meets it square.
    checkBelow(spreadOf(lefts), 0.5, "one shoulder point wherever the box was, mm");
    checkBelow(spreadOf(rights), 0.5, "and the other, mm");
    double mean = 0.0;
    for (double a : arcs) mean += a;
    mean /= static_cast<double>(arcs.size());
    checkNear(mean, col.trueArc(), 1.0, "and it is the seam's length, mm");
}

void testEveryWayRound(const Case& k) {
    std::printf("\nThe collar laid any way round the mat\n");
    struct W {
        double angle;
        cv::Point2d at;
        const char* what;
    };
    const W ways[] = {
        {0.0, {180.0, 190.0}, "neck to the rail below"},
        {kPi, {180.0, 55.0}, "neck toward the top of the picture"},
        {0.5 * kPi, {40.0, 125.0}, "neck to the left"},
        {0.14, {185.0, 185.0}, "askew by eight degrees"},
    };
    for (const W& w : ways) {
        Collar col;
        col.angle = w.angle;
        col.at = w.at;
        Rng rng(3303);
        const cv::Mat frame = renderCollar(k.model, col, rng);
        const CollarResult r = read(frame, k, col, boxAround(k.model, col, 14.0), "outer", true, 280.0);
        check(r.ok, std::string(w.what) + ": read" + (r.ok ? "" : ": " + r.error));
        if (r.ok) checkNear(r.arc_mm, col.trueArc(), 1.0, std::string(w.what) + ": the seam, mm");
    }
}

void testTheOtherEdgeAndTheChord(const Case& k) {
    std::printf("\nThe step says which edge of the band, and which of its two lengths\n");
    Collar col;
    Rng rng(4404);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    const cv::Rect2d box = boxAround(k.model, col, 14.0);
    const CollarResult in = read(frame, k, col, box, "inner", true, 280.0);
    check(in.ok, "the band's inner edge is read: " + in.error);
    if (in.ok) checkNear(in.arc_mm, col.innerArc(), 1.2, "the opening's edge, edge of the cloth to edge, mm");
    const CollarResult chord = read(frame, k, col, box, "outer", false, 180.0);
    check(chord.ok, "a step that wants the chord reads: " + chord.error);
    if (chord.ok) {
        checkNear(chord.measured_mm(), col.trueChord(), 1.0, "the neck width, shoulder point to shoulder point, mm");
        check(!chord.along_arc && chord.arc_mm > chord.chord_mm, "with the arc recorded beside it");
    }
    // Without the station's outline the cloth is told from the mat by
    // brightness; the reading must not care.
    const CollarResult a = read(frame, k, col, box, "outer", true, 280.0, true);
    const CollarResult b = read(frame, k, col, box, "outer", true, 280.0, false);
    check(a.ok && b.ok, "read with the outline and without it");
    if (a.ok && b.ok) checkBelow(std::fabs(a.arc_mm - b.arc_mm), 0.3, "and they agree, mm");
}

void testRefusals(const Case& k) {
    std::printf("\nWhat it refuses, and why\n");
    Collar col;
    Rng rng(5505);
    const cv::Mat frame = renderCollar(k.model, col, rng);

    {
        // A box that stops short of the top of the collar.
        cv::Rect2d box = boxAround(k.model, col, 10.0);
        const double cut = 30.0 * kPxPerMm;
        box.y += cut;
        box.height -= cut;
        const CollarResult r = read(frame, k, col, box, "outer", true, 280.0);
        check(!r.ok, "a box that cuts the collar is not read");
        check(r.refusal == "box", "and it says so as a matter of the box: " + r.error);
    }
    {
        // A box on plain shirt, above the collar, with no edge of the cloth.
        const std::vector<cv::Point2d> px = k.model.mmToPx({col.toMat(cv::Point2d(-60.0, -170.0))});
        const cv::Rect2d box(px[0].x, px[0].y, 120.0 * kPxPerMm, 60.0 * kPxPerMm);
        const CollarResult r = read(frame, k, col, box, "outer", true, 280.0);
        check(!r.ok && r.refusal == "box", "a box with no edge of the cloth in it: " + r.error);
    }
    {
        const cv::Rect2d box(900.0, 900.0, 120.0, 90.0);
        const CollarResult r = read(frame, k, col, box, "outer", true, 280.0);
        check(!r.ok && r.refusal == "box", "a box smaller than any collar: " + r.error);
    }
    {
        Collar none = col;
        none.drawCollar = false;
        Rng blank(5506);
        const cv::Mat plain = renderCollar(k.model, none, blank);
        const CollarResult r = read(plain, k, none, boxAround(k.model, col, 12.0), "outer", true, 280.0);
        check(!r.ok, "a shirt with no collar showing is not read");
        check(r.refusal != "box", "and that is about the collar, not the box: " + r.error);
    }
    {
        const CollarResult r = read(frame, k, col, boxAround(k.model, col, 12.0), "outer", true, 420.0);
        check(!r.ok, "a collar nowhere near the style's number is not taken as it");
        check(r.error.find("quarter") != std::string::npos, "and it says what it read instead: " + r.error);
    }
}

void testTheVerdict(const Case& k) {
    std::printf("\nFrom the style file to the verdict\n");
    Collar col;
    Rng rng(6606);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    const CollarResult r = read(frame, k, col, boxAround(k.model, col, 12.0), "outer", true, 280.0);

    Template t;
    t.style = "collar test";
    Step s;
    s.name = "collar_detection";
    s.kind = "collar";
    s.measure = "arc";
    s.seam = "outer";
    s.target_mm = col.trueArc();
    s.tol_minus_mm = -5.0;
    s.tol_plus_mm = 5.0;
    t.steps.push_back(s);

    const StepResult v = judgeCollar(t, 0, r);
    check(v.measured && v.pass, "a collar on target passes");
    check(v.kind == "collar", "and the log line says it was a collar step");
    checkNear(v.measured_mm, r.arc_mm, 1e-9, "judged on the reading, mm");
    const vjson::Value rec = v.toJson();
    check(rec.find("arc_mm") && rec.find("chord_mm") && rec.find("confidence"),
          "both lengths and the confidence are on the log line");

    t.steps[0].target_mm = col.trueArc() - 20.0;
    const StepResult off = judgeCollar(t, 0, r);
    check(off.measured && !off.pass, "a collar off target reads, and fails");

    CollarResult refused;
    refused.ok = false;
    refused.error = "the collar runs out of the box";
    const StepResult no = judgeCollar(t, 0, refused);
    check(!no.measured && !no.pass, "a collar that could not be read is a FAIL, not a skipped step");
    check(no.reason == refused.error, "and the reason is the refusal's own");

    // The step survives a round trip through the file: targets are edited
    // from the bench and written back, and a save that dropped "kind" would
    // turn the collar step into a folded-edge step.
    Template again;
    std::string error;
    check(parseTemplate(templateToJson(t), again, error) && !again.steps.empty() &&
              again.steps[0].isCollar() && again.steps[0].isPointed() && again.steps[0].seam == "outer",
          "the style rewrites and reads back as a collar step: " + error);

    // A tap takes whichever line is under it, and nothing else about the
    // reading changes.
    if (r.ok && r.lines.size() >= 2) {
        CollarResult tapped = r;
        int inner = -1;
        for (std::size_t i = 0; i < r.lines.size(); ++i) {
            if (r.lines[i].role == "inner") inner = static_cast<int>(i);
        }
        if (inner >= 0) {
            const std::vector<cv::Point2d>& p = r.lines[static_cast<std::size_t>(inner)].path_mm;
            const cv::Point2d near = p[p.size() / 2] + cv::Point2d(1.5, 1.0);
            check(pickCollarLine(tapped, near) && tapped.chosen == inner, "a tap beside a line takes that line");
            checkNear(tapped.arc_mm, r.lines[static_cast<std::size_t>(inner)].arc_mm, 1e-9,
                      "and the number is the line's own, mm");
            check(!pickCollarLine(tapped, cv::Point2d(-500.0, -500.0)), "a tap on nothing takes nothing");
        }
    }
}

void testTheTesterStyle() {
    std::printf("\nThe tester style carries a collar step\n");
    Template t;
    std::string error;
    if (!loadTemplate("templates/innerverse-tester.json", t, error)) {
        std::printf("        (skipped: %s)\n", error.c_str());
        return;
    }
    int idx = -1;
    for (std::size_t k = 0; k < t.steps.size(); ++k) {
        if (t.steps[k].isCollar()) idx = static_cast<int>(k);
    }
    check(idx >= 0, "there is a collar step in it");
    if (idx < 0) return;
    const Step& s = t.steps[static_cast<std::size_t>(idx)];
    check(s.measure == "arc" || s.measure == "chord" || s.measure == "around", "it says which length it wants");
    check(s.seam.empty() || s.seam == "outer" || s.seam == "inner", "and which edge of the band");
    check(!s.isCurve() && !s.isDistance() && s.isPointed(), "and it is its own kind of step");
}

// ---- the whole collar ----
//
// "measure": "around": the seam over the front, from one shoulder point to the
// other, and the edge of the cloth across the back of the neck between the
// same two points. On the rendered collar the edge of the cloth between the
// feet is the straight line y = 0, so the back is exactly the chord and the
// whole collar is the seam's arc and its chord added up.

CollarResult readAround(const cv::Mat& frame, const Case& k, const Collar& col, const cv::Rect2d& box,
                        double target, bool withOutline = true) {
    CollarRequest req;
    req.edge = "outer";
    req.around = true;
    req.target_mm = target;
    CollarResult r;
    detectCollar(frame, k.model, withOutline ? col.outline() : std::vector<cv::Point2d>(), box, req, k.cfg, r);
    return r;
}

void testTheWholeCollar(const Case& k) {
    std::printf("\nThe whole collar: the seam over the front, the edge of the shirt across the back\n");
    Collar col;
    Rng rng(7707);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    const double whole = col.trueArc() + col.trueChord();
    const CollarResult r = readAround(frame, k, col, boxAround(k.model, col, 12.0), whole);
    check(r.ok, "the whole collar is read: " + r.error);
    if (!r.ok) {
        printResult(r);
        return;
    }
    checkNear(r.measured_mm(), whole, 1.5, "front and back together, mm");
    checkNear(r.back_mm, col.trueChord(), 1.0, "the back, shoulder point to shoulder point along the edge, mm");
    checkBelow(std::fabs(r.back_mm - r.chord_mm), 0.3, "along a straight edge the back is the chord, mm");
    double sum = 0.0, far = 0.0;
    int n = 0;
    for (const cv::Point2d& p : r.back_path_mm) {
        const double d = std::fabs(col.toCollar(p).y);
        sum += d * d;
        far = std::max(far, d);
        ++n;
    }
    checkBelow(std::sqrt(sum / std::max(1, n)), 0.35, "the back lies on the edge of the cloth, mm RMS");
    checkBelow(far, 0.6, "all the way along, mm");
    const auto same = [](const cv::Point2d& a, const cv::Point2d& b) { return std::hypot(a.x - b.x, a.y - b.y) < 1e-9; };
    check(r.back_path_mm.size() >= 2 && same(r.back_path_mm.front(), r.end_mm[1]) &&
              same(r.back_path_mm.back(), r.end_mm[0]),
          "and it closes the front at both shoulder points");

    struct B {
        double grow;
        cv::Point2d shift;
        double tall;
        const char* what;
    };
    const B boxes[] = {
        {8.0, {0.0, 0.0}, 0.0, "tight"},          {25.0, {0.0, 0.0}, 0.0, "loose"},
        {20.0, {12.0, 0.0}, 0.0, "shifted right"}, {20.0, {-12.0, 0.0}, 0.0, "shifted left"},
        {12.0, {0.0, -12.0}, 0.0, "shifted up"},   {10.0, {0.0, 0.0}, 30.0, "tall"},
    };
    std::vector<double> wholes;
    for (const B& b : boxes) {
        const CollarResult q = readAround(frame, k, col, boxAround(k.model, col, b.grow, b.shift, b.tall), whole);
        char got[64];
        std::snprintf(got, sizeof(got), ": %.2f mm", q.measured_mm());
        check(q.ok, std::string("the whole collar from a ") + b.what + " box" + (q.ok ? got : ": " + q.error));
        if (q.ok) wholes.push_back(q.measured_mm());
    }
    if (wholes.size() >= 2) {
        checkBelow(*std::max_element(wholes.begin(), wholes.end()) - *std::min_element(wholes.begin(), wholes.end()),
                   0.5, "its spread over every box, mm");
    }

    const CollarResult plain = readAround(frame, k, col, boxAround(k.model, col, 12.0), whole, false);
    check(plain.ok, "read without the station's outline too: " + plain.error);
    if (plain.ok) checkBelow(std::fabs(plain.measured_mm() - r.measured_mm()), 0.5, "and the two agree, mm");

    Collar turned;
    turned.angle = 0.5 * kPi;
    turned.at = {40.0, 125.0};
    Rng rngTurned(7708);
    const cv::Mat laid = renderCollar(k.model, turned, rngTurned);
    const CollarResult t = readAround(laid, k, turned, boxAround(k.model, turned, 14.0), whole);
    check(t.ok, "neck to the left: read: " + t.error);
    if (t.ok) checkNear(t.measured_mm(), whole, 1.5, "neck to the left: the whole collar, mm");
}

void testTheWholeCollarVerdict(const Case& k) {
    std::printf("\nThe whole collar, from the style file to the verdict\n");
    Collar col;
    Rng rng(7709);
    const cv::Mat frame = renderCollar(k.model, col, rng);
    const double whole = col.trueArc() + col.trueChord();
    const CollarResult r = readAround(frame, k, col, boxAround(k.model, col, 12.0), whole);

    Template t;
    t.style = "collar test";
    Step s;
    s.name = "collar_detection";
    s.kind = "collar";
    s.measure = "around";
    s.seam = "outer";
    s.target_mm = whole;
    s.tol_minus_mm = -5.0;
    s.tol_plus_mm = 5.0;
    t.steps.push_back(s);

    const StepResult v = judgeCollar(t, 0, r);
    check(v.measured && v.pass, "a whole collar on target passes");
    checkNear(v.measured_mm, r.arc_mm + r.back_mm, 1e-9, "judged on the front and the back together, mm");
    check(v.toJson().find("back_mm") != nullptr, "and the back is on the log line beside the front");
    t.steps[0].target_mm = r.arc_mm;
    const StepResult front = judgeCollar(t, 0, r);
    check(front.measured && !front.pass, "the front alone is not the whole collar: a target of just the front fails");

    std::string error;
    Template again;
    t.steps[0].target_mm = whole;
    check(parseTemplate(templateToJson(t), again, error) && !again.steps.empty() && again.steps[0].measure == "around",
          "the style rewrites and reads back measuring around: " + error);
    t.steps[0].seam = "inner";
    Template bad;
    check(!parseTemplate(templateToJson(t), bad, error), "a style asking for the opening all the way round is refused: " + error);
}

int selftest() {
    std::printf("Collar detection selftest\n");
    Config cfg;
    const PlaneModelPtr model = benchModel();
    const Case k{*model, cfg};
    testReadsTheSeam(k);
    testTheBoxDoesNotMatter(k);
    testEveryWayRound(k);
    testTheOtherEdgeAndTheChord(k);
    testRefusals(k);
    testTheVerdict(k);
    testTheTesterStyle();
    testTheWholeCollar(k);
    testTheWholeCollarVerdict(k);
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

// ---- reading a capture ----

PlaneModelPtr loadModel(const std::string& calPath) {
    vjson::Value doc;
    std::string error;
    if (!vjson::parseFile(calPath, doc, error)) {
        std::printf("could not read %s: %s\n", calPath.c_str(), error.c_str());
        return nullptr;
    }
    PlaneModelPtr model = PlaneModel::fromJson(doc);
    if (!model) std::printf("%s is not a calibration this build understands\n", calPath.c_str());
    return model;
}

int readFrame(const std::string& framePath, const std::string& calPath, const cv::Rect2d& box,
              const std::vector<cv::Point2d>& outlineMm, const CollarRequest& req, const std::string& drawPath) {
    const cv::Mat frame = cv::imread(framePath, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::printf("could not read %s\n", framePath.c_str());
        return 1;
    }
    const PlaneModelPtr model = loadModel(calPath);
    if (!model) return 1;
    Config cfg;
    CollarResult r;
    const double t0 = static_cast<double>(cv::getTickCount());
    detectCollar(frame, *model, outlineMm, box, req, cfg, r);
    const double ms = (static_cast<double>(cv::getTickCount()) - t0) * 1000.0 / cv::getTickFrequency();
    printResult(r);
    std::printf("took %.0f ms\n", ms);
    if (const char* dump = std::getenv("COLLAR_DUMP")) {
        if (r.chosen >= 0) {
            vjson::Value pts = vjson::Value::array();
            for (const cv::Point2d& p : r.lines[static_cast<std::size_t>(r.chosen)].path_mm) {
                vjson::Value a = vjson::Value::array();
                a.push(vjson::Value::real(p.x));
                a.push(vjson::Value::real(p.y));
                pts.push(a);
            }
            vjson::writeFile(dump, pts);
        }
    }
    if (!drawPath.empty()) drawResult(frame, *model, r, drawPath);
    return r.ok ? 0 : 1;
}

// A case the station left behind: the held picture, the box, the outline its
// reading was taken from, and what the step asked for. Replayed exactly.
int readCase(const std::string& casePath, const std::string& calOverride, const std::string& drawPath,
             bool around) {
    vjson::Value doc;
    std::string error;
    if (!vjson::parseFile(casePath, doc, error)) {
        std::printf("could not read %s: %s\n", casePath.c_str(), error.c_str());
        return 1;
    }
    const std::filesystem::path dir = std::filesystem::path(casePath).parent_path();
    std::string frame = doc.getString("frame");
    if (!frame.empty() && std::filesystem::path(frame).is_relative()) frame = (dir / frame).string();
    const std::string cal = calOverride.empty() ? doc.getString("calibration", "calibration.json") : calOverride;
    cv::Rect2d box;
    if (const vjson::Value* b = doc.find("box_px")) {
        if (b->isArray() && b->size() == 4) {
            box = cv::Rect2d(b->at(0).asNumber(), b->at(1).asNumber(), b->at(2).asNumber(), b->at(3).asNumber());
        }
    }
    std::vector<cv::Point2d> outline;
    if (const vjson::Value* o = doc.find("outline_mm")) {
        for (std::size_t i = 0; o->isArray() && i < o->size(); ++i) {
            const vjson::Value& p = o->at(i);
            if (p.isArray() && p.size() == 2) outline.emplace_back(p.at(0).asNumber(), p.at(1).asNumber());
        }
    }
    CollarRequest req;
    req.edge = doc.getString("edge", "outer");
    req.along_arc = doc.getBool("along_arc", true);
    req.around = around || doc.getBool("around", false);
    req.target_mm = doc.getNumber("target_mm", 0.0);
    std::printf("replaying %s: box %.0f,%.0f %.0fx%.0f px, %s edge%s, %d outline points, target %.1f mm\n",
                casePath.c_str(), box.x, box.y, box.width, box.height, req.edge.c_str(),
                req.around ? ", all the way round" : "", static_cast<int>(outline.size()), req.target_mm);
    return readFrame(frame, cal, box, outline, req, drawPath);
}

// A synthetic lay and the calibration that goes with it, so the --frame path
// can be exercised without a camera and a bench being commissioned has a
// known-good pair to compare its own captures against.
int writeSample(const std::string& dir) {
    const PlaneModelPtr model = benchModel();
    Collar col;
    Rng rng(20260918);
    const cv::Mat frame = renderCollar(*model, col, rng);
    const std::string img = dir + "/sample-collar.png";
    const std::string cal = dir + "/sample-calibration.json";
    if (!cv::imwrite(img, frame) || !vjson::writeFile(cal, model->toJson(), 2)) {
        std::printf("could not write the sample into %s\n", dir.c_str());
        return 1;
    }
    // And the mat with nothing on it, the same way lit: what a station settles
    // on before the shirt is laid, so a replayed station has a background to
    // tell the shirt from.
    Collar none = col;
    none.drawGarment = false;
    Rng rngMat(20260919);
    const std::string mat = dir + "/sample-mat.png";
    if (!cv::imwrite(mat, renderCollar(*model, none, rngMat))) {
        std::printf("could not write %s\n", mat.c_str());
        return 1;
    }
    const cv::Rect2d box = boxAround(*model, col, 15.0);
    std::printf("wrote %s, %s and %s\n", img.c_str(), mat.c_str(), cal.c_str());
    std::printf("true seam %.2f mm along, %.2f mm across; the opening's edge %.2f mm\n", col.trueArc(),
                col.trueChord(), col.innerArc());
    std::printf("collar_check --frame %s --calibration %s --box %.0f,%.0f,%.0f,%.0f --target 280\n", img.c_str(),
                cal.c_str(), box.x, box.y, box.width, box.height);
    return 0;
}

void usage() {
    std::printf(
        "Collar detection harness.\n"
        "\n"
        "  collar_check --selftest\n"
        "  collar_check --frame lay.png --box X,Y,W,H [--calibration calibration.json]\n"
        "               [--edge outer|inner] [--chord | --around] [--target MM] [--draw out.png]\n"
        "  collar_check --case collar-fail.json [--calibration FILE] [--around] [--draw out.png]\n"
        "  collar_check --write-sample DIR\n"
        "\n"
        "--around reads the whole collar: the outer edge over the front and the edge of\n"
        "the shirt across the back of the neck, as a step with \"measure\": \"around\".\n"
        "--box is in pixels of the frame, as the station draws it. Without --case the\n"
        "cloth is told from the mat by brightness inside the box; the station uses the\n"
        "outline its reading was taken from, and --case replays exactly that. --draw\n"
        "puts every line found back on the picture, the one read in green: which\n"
        "stitch a line is on is not a number and cannot be read from one.\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string framePath, calPath, boxText, drawPath, casePath, samplePath;
    bool wantSelftest = false;
    CollarRequest req;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--selftest") {
            wantSelftest = true;
        } else if (a == "--frame" && i + 1 < argc) {
            framePath = argv[++i];
        } else if (a == "--calibration" && i + 1 < argc) {
            calPath = argv[++i];
        } else if (a == "--box" && i + 1 < argc) {
            boxText = argv[++i];
        } else if (a == "--edge" && i + 1 < argc) {
            req.edge = argv[++i];
        } else if (a == "--chord") {
            req.along_arc = false;
        } else if (a == "--around") {
            req.around = true;
        } else if (a == "--target" && i + 1 < argc) {
            req.target_mm = std::atof(argv[++i]);
        } else if (a == "--draw" && i + 1 < argc) {
            drawPath = argv[++i];
        } else if (a == "--case" && i + 1 < argc) {
            casePath = argv[++i];
        } else if (a == "--write-sample" && i + 1 < argc) {
            samplePath = argv[++i];
        } else {
            usage();
            return 2;
        }
    }
    if (wantSelftest) return selftest();
    if (!samplePath.empty()) return writeSample(samplePath);
    if (!casePath.empty()) return readCase(casePath, calPath, drawPath, req.around);
    if (framePath.empty() || boxText.empty()) {
        usage();
        return 2;
    }
    cv::Rect2d box;
    if (!parseBoxText(boxText, box)) {
        std::printf("could not read the box: X,Y,W,H in pixels\n");
        return 2;
    }
    return readFrame(framePath, calPath.empty() ? "calibration.json" : calPath, box, {}, req, drawPath);
}
