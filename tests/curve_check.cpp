/*
Traced-curve harness. Points at a seam of known length, as sloppily as a hand
does, and checks the answer, with no camera attached.

    curve_check --selftest
    curve_check --frame lay.png --seeds "120,40;170,95;220,40"

The whole feature rests on one claim: the operator's pointing may be bad and the
reading will not be. Everything below is that claim, taken apart.

The seam is a circular arc of known radius drawn into the picture at a known
place, so its chord and its arc length are known exactly rather than measured by
something else. It is drawn as a shade step, which is what a collar meeting a
body actually looks like from above, and there are wrinkles laid across it,
because a garment has those and a search that cannot ignore them is no use.

Then it is pointed at badly. Off the seam by up to a centimetre, which is more
than a fingertip's parallax and its tip ambiguity and a hand's shake put
together, and differently badly every time.

Three things have to hold.

The path has to land on the seam. Not near it - on it, to a fraction of a
millimetre, because that is what makes this a measurement rather than a trace.

The answer must not move when the pointing does. Two operators, or one operator
twice, must get the same number.

And where the pointing genuinely does reach the answer, it has to be bounded and
visible rather than silent. It reaches exactly one place: the two ends. A tap
five millimetres along the seam from where the last one landed moves the end of
the measurement five millimetres along the seam, because nothing in the picture
says where a neckline stops - only the operator does. The snap corrects across
the seam and cannot correct along it. That is a real limit of this design, it is
tested here rather than hidden, and it is why the corner crossing in edge.cpp is
the next thing to reach for.
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
#include <opencv2/geometry/2d.hpp>

#include "verimeasure/config.hpp"
#include "verimeasure/curve.hpp"
#include "verimeasure/finger.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/numeric.hpp"
#include "verimeasure/plane_model.hpp"
#include "verimeasure/segmentation.hpp"
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
    char buf[240];
    std::snprintf(buf, sizeof(buf), "%s  (got %.3f, want %.3f, within %.3f)", what.c_str(), got,
                  want, tol);
    check(std::fabs(got - want) <= tol, buf);
}

void checkBelow(double got, double limit, const std::string& what) {
    char buf[240];
    std::snprintf(buf, sizeof(buf), "%s  (got %.3f, limit %.3f)", what.c_str(), got, limit);
    check(got <= limit, buf);
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

// -- the bench --
//
// A plane model with the scale this bench actually has, 0.1378 mm per pixel, no
// distortion and no tilt. The point of the test is the search, not the model,
// and a model with terms in it that the truth generator does not share would be
// testing the wrong thing.
const double kPxPerMm = 1.0 / 0.13787740569262713;
const int kFrameW = 2600;
const int kFrameH = 1800;

PlaneModelPtr benchModel() {
    cv::Matx33d H(kPxPerMm, 0.0, 40.0, 0.0, kPxPerMm, 40.0, 0.0, 0.0, 1.0);
    return std::make_shared<PlaneModel>(H);
}

// Millimetre coordinates of every pixel, computed once through the real plane
// model and reused by every render.
struct MmGrid {
    cv::Mat x;  // CV_64F, one per pixel
    cv::Mat y;
};

MmGrid buildGrid(const PlaneModel& model) {
    std::vector<cv::Point2d> px;
    px.reserve(static_cast<std::size_t>(kFrameW) * kFrameH);
    for (int y = 0; y < kFrameH; ++y) {
        for (int x = 0; x < kFrameW; ++x) px.emplace_back(x, y);
    }
    const std::vector<cv::Point2d> mm = model.pxToMm(px);
    MmGrid g;
    g.x.create(kFrameH, kFrameW, CV_64F);
    g.y.create(kFrameH, kFrameW, CV_64F);
    for (int y = 0; y < kFrameH; ++y) {
        double* rx = g.x.ptr<double>(y);
        double* ry = g.y.ptr<double>(y);
        for (int x = 0; x < kFrameW; ++x) {
            const cv::Point2d& p = mm[static_cast<std::size_t>(y) * kFrameW + x];
            rx[x] = p.x;
            ry[x] = p.y;
        }
    }
    return g;
}

// A neckline, as a garment actually presents one.
//
// A scoop neck 180 mm across the shoulder points and 75 mm deep is a circular
// arc of radius 91.5 mm sweeping 2.78 radians, so its arc length is 254.4 mm
// against a 180 mm chord. Those two numbers differ by forty percent, which is
// why the style file has to say which of them the tech pack means.
struct Seam {
    cv::Point2d centre{0.0, 0.0};
    double radius = 0.0;
    double a0 = 0.0;     // where the operator starts pointing
    double sweep = 0.0;  // and how far round
    double trueChord() const { return 2.0 * radius * std::sin(std::fabs(sweep) / 2.0); }
    double trueArc() const { return radius * std::fabs(sweep); }
    cv::Point2d at(double t) const {  // t in [0, 1]
        const double a = a0 + sweep * t;
        return cv::Point2d(centre.x + radius * std::cos(a), centre.y + radius * std::sin(a));
    }
    // Distance from a point to the circle the seam lies on. Zero on the seam.
    double offBy(const cv::Point2d& p) const {
        return std::fabs(std::hypot(p.x - centre.x, p.y - centre.y) - radius);
    }
};

Seam scoopNeck(cv::Point2d centre = cv::Point2d(180.0, 120.0), double rotate = 0.0) {
    Seam s;
    s.centre = centre;
    s.radius = 91.5;
    // Symmetric about straight down, which is how a neckline sits when the
    // shirt is laid square to the camera.
    s.sweep = 2.7804;
    s.a0 = rotate - s.sweep / 2.0;
    return s;
}

// Render the shirt: a shade step at the seam, wrinkles across it, sensor noise.
cv::Mat renderSeam(const MmGrid& grid, const Seam& seam, bool drawSeam, Rng& rng,
                   double contrast = 30.0) {
    cv::Mat img(kFrameH, kFrameW, CV_8U);

    // The transition width. A collar seam is a band a millimetre or two across
    // rather than a step, and rendering it as an ideal step would let the
    // search look better here than it can be on cloth.
    const double softMm = 1.1;

    for (int y = 0; y < kFrameH; ++y) {
        const double* rx = grid.x.ptr<double>(y);
        const double* ry = grid.y.ptr<double>(y);
        unsigned char* out = img.ptr<unsigned char>(y);
        for (int x = 0; x < kFrameW; ++x) {
            double v = 120.0;
            if (drawSeam) {
                const double d =
                    std::hypot(rx[x] - seam.centre.x, ry[x] - seam.centre.y) - seam.radius;
                const double t = std::max(-1.0, std::min(1.0, d / softMm));
                v += contrast * 0.5 * (1.0 + std::sin(kPi * 0.5 * t));
            } else {
                v += contrast * 0.5;
            }
            out[x] = static_cast<unsigned char>(std::max(0.0, std::min(255.0, v)));
        }
    }

    // Wrinkles: soft lines lying across the seam rather than along it. This is
    // what the across-corridor derivative is for, and a test without them would
    // not exercise it.
    for (int k = 0; k < 6; ++k) {
        const cv::Point a(static_cast<int>(rng.uniform() * kFrameW),
                          static_cast<int>(rng.uniform() * kFrameH));
        const cv::Point b(a.x + static_cast<int>(rng.signed1() * 900),
                          a.y + static_cast<int>(rng.signed1() * 900));
        cv::line(img, a, b, cv::Scalar(120.0 + contrast * 0.75), 9, cv::LINE_AA);
    }
    cv::GaussianBlur(img, img, cv::Size(0, 0), 3.0);

    cv::Mat noise(img.size(), CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        float* r = noise.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) {
            r[x] = static_cast<float>((rng.signed1() + rng.signed1() + rng.signed1()) * 1.6);
        }
    }
    cv::Mat f;
    img.convertTo(f, CV_32F);
    f += noise;
    f.convertTo(img, CV_8U);
    return img;
}

// -- a neckline as cloth actually presents one --
//
// The renderer above draws one isolated step on flat ground, and that turned
// out to be the wrong test entirely: it cannot fail in the way a real garment
// makes this fail. A neckline is not one line in an empty corridor. It is FOUR
// lines within a centimetre of each other -
//
//     the neck opening, where the cloth ends and the back panel shows through
//     the inner edge of the rib binding
//     the attachment seam, which is the one the tech pack means
//     the topstitch, a hard thin line a couple of millimetres outside it
//
// - lying in knit texture, with folds running across the seam and, worse, along
// it. On real cloth the seam that matters is rarely the strongest of the four.
// The topstitch is a sharp dark line and always wins on gradient; the neck
// opening carries the biggest shade step because it is cloth against a shadowed
// second layer. So a search that takes the strongest line in the corridor takes
// the wrong one, confidently, and reports it to three decimal places.
//
// Everything below is drawn in millimetres through the plane model, so the
// distances between the four lines are the distances a real garment has.
struct Neckline {
    Seam seam;             // the attachment seam: the measurand
    double binding_mm = 15.0;  // rib width, so the neck opening is this far in
    double topstitch_mm = 4.0; // and the coverstitch this far out
};

cv::Mat renderGarment(const MmGrid& grid, const Neckline& neck, Rng& rng,
                      double seamContrast = 7.0, bool wrinkles = true) {
    const Seam& seam = neck.seam;
    cv::Mat img(kFrameH, kFrameW, CV_32F);

    // Folds. Two of them run ALONG the seam rather than across it, which is the
    // case that matters: a fold crossing the seam is suppressed by taking the
    // derivative across the corridor, and a fold parallel to it is not.
    struct Fold { double radius, width, amp; };
    const Fold folds[] = {
        {seam.radius - 12.0, 4.0, 10.0},
        {seam.radius + 11.0, 5.0, -9.0},
        {seam.radius + 26.0, 6.0, 8.0},
    };

    for (int y = 0; y < kFrameH; ++y) {
        const double* rx = grid.x.ptr<double>(y);
        const double* ry = grid.y.ptr<double>(y);
        float* out = img.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) {
            const double dx = rx[x] - seam.centre.x;
            const double dy = ry[x] - seam.centre.y;
            const double r = std::hypot(dx, dy);
            const double d = r - seam.radius;   // signed, across the seam

            double v = 120.0;

            // The neck opening: cloth ends, the back panel shows through and
            // sits in shadow. The biggest step of the four, and the wrong one.
            const double open = -neck.binding_mm;
            v += -13.0 * 0.5 * (1.0 + std::tanh((open - d) / 0.9));

            // The rib binding, a shade lighter than the body.
            if (d < 0.0 && d > open) v += 7.0;

            // The attachment seam itself: the measurand. A modest step, which
            // is what it is on cloth.
            v += seamContrast * 0.5 * (1.0 + std::tanh(-d / 0.9));

            // The topstitch: a hard, thin, dark line. The strongest gradient
            // anywhere near the seam, and two millimetres away from it.
            const double ts = (d - neck.topstitch_mm) / 0.45;
            v += -11.0 * std::exp(-ts * ts);

            if (wrinkles) {
                for (const Fold& f : folds) {
                    const double t = (r - f.radius) / f.width;
                    v += f.amp * std::exp(-t * t);
                }
            }

            // Lighting across the bench, gentle and unavoidable.
            v += 0.010 * (rx[x] - 180.0);
            out[x] = static_cast<float>(v);
        }
    }

    // Knit texture: band-limited, about a millimetre across, which is the scale
    // that defeats a search looking for the sharpest thing in the corridor.
    const int tw = kFrameW / 6, th = kFrameH / 6;
    cv::Mat fine(th, tw, CV_32F);
    for (int y = 0; y < th; ++y) {
        float* r = fine.ptr<float>(y);
        for (int x = 0; x < tw; ++x) r[x] = static_cast<float>(rng.signed1() * 9.0);
    }
    cv::Mat texture;
    cv::resize(fine, texture, img.size(), 0, 0, cv::INTER_CUBIC);
    img += texture;

    cv::Mat noise(img.size(), CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        float* r = noise.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) {
            r[x] = static_cast<float>((rng.signed1() + rng.signed1() + rng.signed1()) * 1.4);
        }
    }
    img += noise;

    cv::Mat out8;
    img.convertTo(out8, CV_8U);
    return out8;
}

// Three taps on the seam, pushed off it by up to `acrossMm` across and
// `alongMm` along. The two are separated because they do completely different
// things to the answer.
std::vector<cv::Point2d> pointAt(const Seam& seam, Rng& rng, double acrossMm, double alongMm) {
    std::vector<cv::Point2d> seeds;
    for (int k = 0; k < 3; ++k) {
        const double t = k * 0.5;
        // Along: the tap slides up or down the seam. Nothing in the picture can
        // correct this and nothing tries to.
        const double slide = alongMm > 0.0 ? rng.signed1() * alongMm / seam.radius : 0.0;
        const double a = seam.a0 + seam.sweep * t + slide;
        // Across: the tap sits off the seam. This is what the snap removes.
        const double r = seam.radius + rng.signed1() * acrossMm;
        seeds.emplace_back(seam.centre.x + r * std::cos(a), seam.centre.y + r * std::sin(a));
    }
    return seeds;
}

// How far the snapped path sits from the seam that was drawn.
void pathError(const CurveResult& r, const Seam& seam, double& rms, double& worst) {
    double sum = 0.0;
    worst = 0.0;
    for (const cv::Point2d& p : r.path_mm) {
        const double d = seam.offBy(p);
        sum += d * d;
        worst = std::max(worst, d);
    }
    rms = r.path_mm.empty() ? 0.0 : std::sqrt(sum / static_cast<double>(r.path_mm.size()));
}

// -- the checks --

void testSeedPath() {
    std::printf("\nThe corridor follows the curve, not the chord across it\n");
    const Seam seam = scoopNeck();

    std::vector<cv::Point2d> seeds{seam.at(0.0), seam.at(0.5), seam.at(1.0)};
    std::vector<cv::Point2d> path;
    check(seedPath(seeds, 0.25, path), "three taps describe a path");

    double worst = 0.0;
    for (const cv::Point2d& p : path) worst = std::max(worst, seam.offBy(p));
    checkBelow(worst, 0.05, "the arc through three exact taps lies on the seam");

    // The reason three taps and not two. A neckline dips 75 mm below the chord
    // joining its ends, so a corridor built around that chord holds no neckline
    // at all along most of its length: everything downstream would be searching
    // bare cloth.
    std::vector<cv::Point2d> two{seam.at(0.0), seam.at(1.0)};
    std::vector<cv::Point2d> straight;
    seedPath(two, 0.25, straight);
    double deepest = 0.0;
    for (const cv::Point2d& p : straight) deepest = std::max(deepest, seam.offBy(p));
    check(deepest > 60.0,
          "two taps miss the seam by far more than any corridor is wide, which is why "
          "three are asked for");
}

void testSnapAccuracy(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nThe path lands on the seam, however badly it was pointed at\n");
    const Seam seam = scoopNeck();
    Rng rng(20260909);
    const cv::Mat frame = renderSeam(grid, seam, true, rng);

    // Pointed at across-the-seam only, which isolates what the snap can fix
    // from what it cannot.
    const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 10.0, 0.0);
    double seedWorst = 0.0;
    for (const cv::Point2d& p : seeds) seedWorst = std::max(seedWorst, seam.offBy(p));
    check(seedWorst > 3.0, "the taps really are off the seam to begin with");

    CurveResult r;
    check(snapCurve(frame, model, seeds, cfg, r), "the seam is found");
    if (!r.ok) {
        std::printf("        refused: %s\n", r.error.c_str());
        return;
    }

    double rms = 0.0, worst = 0.0;
    pathError(r, seam, rms, worst);
    checkBelow(rms, 0.20, "RMS distance from the found path to the seam that was drawn");
    checkBelow(worst, 0.60, "and the worst single sample");

    checkNear(r.arc_mm, seam.trueArc(), 1.5, "arc length");
    checkNear(r.chord_mm, seam.trueChord(), 1.0, "chord");
    checkNear(r.radius_mm, seam.radius, 1.5, "the radius the seam was drawn at");
    check(r.arc_mm > r.chord_mm + 60.0,
          "the arc runs far longer than the chord, so the style file must say which");
    check(r.confidence > 0.5, "confident, because there really is a seam there");
    check(r.drift_mm > 2.0,
          "and the snap moved a long way to get there, which is the pointing being corrected");
}

void testRepeatability(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nThe answer does not move when the pointing does\n");
    const Seam seam = scoopNeck();
    Rng rng(777);
    const cv::Mat frame = renderSeam(grid, seam, true, rng);

    std::vector<double> arcs, chords;
    for (int trial = 0; trial < 6; ++trial) {
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 10.0, 0.0);
        CurveResult r;
        if (!snapCurve(frame, model, seeds, cfg, r)) {
            check(false, "trial " + std::to_string(trial) + " was refused: " + r.error);
            return;
        }
        arcs.push_back(r.arc_mm);
        chords.push_back(r.chord_mm);
    }
    const double arcSpread = *std::max_element(arcs.begin(), arcs.end()) -
                             *std::min_element(arcs.begin(), arcs.end());
    const double chordSpread = *std::max_element(chords.begin(), chords.end()) -
                               *std::min_element(chords.begin(), chords.end());

    // The bar. A half-inch tolerance band is 25.4 mm wide, and a gauge is
    // usually asked to spend no more than a tenth of the band on itself, so 2.5
    // mm is the number this has to beat. It beats it by an order of magnitude,
    // and it has to, because the operator's own pointing is still to come.
    checkBelow(arcSpread, 0.8, "spread of six arc readings, six different sloppy points");
    checkBelow(chordSpread, 0.8, "spread of six chord readings");
}

void testMatMoves(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nThe mat may move under the camera\n");
    Rng rng(31415);
    double first = 0.0;
    for (int k = 0; k < 3; ++k) {
        const cv::Point2d where(150.0 + k * 20.0, 110.0 + k * 8.0);
        const Seam seam = scoopNeck(where, k * 0.35 - 0.35);
        const cv::Mat frame = renderSeam(grid, seam, true, rng);
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 8.0, 0.0);
        CurveResult r;
        if (!snapCurve(frame, model, seeds, cfg, r)) {
            check(false, "lay " + std::to_string(k) + " was refused: " + r.error);
            continue;
        }
        if (k == 0) first = r.arc_mm;
        checkNear(r.arc_mm, seam.trueArc(), 1.5,
                  "arc at position " + std::to_string(k) + ", turned and shifted");
        checkNear(r.arc_mm, first, 1.5, "and it agrees with the first lay");
    }
}

void testEndsAreTheOperators(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nWhat the snap cannot fix: where the operator stopped pointing\n");
    const Seam seam = scoopNeck();
    Rng rng(2024);
    const cv::Mat frame = renderSeam(grid, seam, true, rng);

    std::vector<double> arcs;
    double worstPath = 0.0;
    for (int trial = 0; trial < 5; ++trial) {
        // Now the taps also slide along the seam, by up to 6 mm, which is about
        // what a hand does.
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 8.0, 6.0);
        CurveResult r;
        if (!snapCurve(frame, model, seeds, cfg, r)) {
            check(false, "trial " + std::to_string(trial) + " was refused: " + r.error);
            return;
        }
        arcs.push_back(r.arc_mm);
        double rms = 0.0, worst = 0.0;
        pathError(r, seam, rms, worst);
        worstPath = std::max(worstPath, rms);
    }

    // The path is still exactly on the seam. Only its two ends moved.
    checkBelow(worstPath, 0.20,
               "the path is still on the seam to the same accuracy, every trial");

    const double spread = *std::max_element(arcs.begin(), arcs.end()) -
                          *std::min_element(arcs.begin(), arcs.end());
    check(spread > 1.0,
          "the reading now does move, because the ends of it are where the taps were");
    checkBelow(spread, 22.0,
               "and it moves by about what the taps moved, not more - the error is bounded "
               "by the pointing rather than multiplied by it");
}

// The test that matters, and the one the first version of this file did not
// have: several lines in the corridor, and the operator points at one of them.
//
// The distances are a real tee's. A rib binding is fifteen millimetres, so the
// neck opening is fifteen millimetres inside the attachment seam, and a
// coverstitch sits about four outside it. The pointing is what a deliberate tap
// on a line you can see is worth, about two and a half millimetres.
void testRealCloth(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nSeveral lines in the corridor, and the pointed-at one has to win\n");

    Neckline neck;
    neck.seam = scoopNeck();
    Rng rng(4242);
    const cv::Mat frame = renderGarment(grid, neck, rng);

    std::vector<double> arcs;
    double worstOff = 0.0;
    int found = 0;
    for (int trial = 0; trial < 5; ++trial) {
        const std::vector<cv::Point2d> seeds = pointAt(neck.seam, rng, 2.5, 0.0);
        CurveResult r;
        if (!snapCurve(frame, model, seeds, cfg, r)) {
            std::printf("        trial %d refused: %s\n", trial, r.error.c_str());
            continue;
        }
        ++found;
        double rms = 0.0, worst = 0.0;
        pathError(r, neck.seam, rms, worst);
        worstOff = std::max(worstOff, rms);
        arcs.push_back(r.arc_mm);
        std::printf("        trial %d: arc %.2f  off %.2f  drift %.2f  conf %.2f  margin %.2f\n",
                    trial, r.arc_mm, rms, r.drift_mm, r.confidence, r.margin);
    }
    check(found == 5, "every lay is read");
    if (arcs.size() < 2) return;

    checkBelow(worstOff, 0.6,
               "the path sits on the attachment seam, not the coverstitch four "
               "millimetres out or the neck opening fifteen in, mm RMS");

    const double spread = *std::max_element(arcs.begin(), arcs.end()) -
                          *std::min_element(arcs.begin(), arcs.end());
    checkBelow(spread, 1.5, "and five pointings agree");

    double mean = 0.0;
    for (double a : arcs) mean += a;
    mean /= static_cast<double>(arcs.size());
    checkNear(mean, neck.seam.trueArc(), 2.0, "the arc that comes back");
}

// Point at a different line and a different line is measured.
//
// This is the design claim stated the other way round, and it is worth its own
// check. A tech pack that asks for the neck OPENING wants the hole, not the
// seam fifteen millimetres outside it, and the machine has no way to know which
// was meant except that somebody pointed at it. A search that always took the
// strongest line, or always the innermost, would pass the test above and be
// useless here.
void testPointingChooses(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nPointing at the other line measures the other line\n");

    Neckline neck;
    neck.seam = scoopNeck();
    Rng rng(1717);
    const cv::Mat frame = renderGarment(grid, neck, rng);

    // The neck opening: the same circle, fifteen millimetres smaller.
    Seam opening = neck.seam;
    opening.radius -= neck.binding_mm;

    const std::vector<cv::Point2d> seeds = pointAt(opening, rng, 2.5, 0.0);
    CurveResult r;
    if (!snapCurve(frame, model, seeds, cfg, r)) {
        check(false, std::string("pointing at the neck opening is refused: ") + r.error);
        return;
    }
    double rms = 0.0, worst = 0.0;
    pathError(r, opening, rms, worst);
    checkBelow(rms, 0.6, "the path sits on the neck opening that was pointed at, mm RMS");

    double toSeam = 0.0;
    pathError(r, neck.seam, toSeam, worst);
    check(toSeam > 10.0,
          "and nowhere near the attachment seam, which nobody pointed at");
}

// A coverstitch close enough to the seam that a single tap could not choose.
//
// It is still resolved, and the reason is worth stating: the choice is not made
// tap by tap. The line is chosen once for the whole length of the corridor, so
// the pointing that decides it is the AVERAGE of three taps over two hundred
// millimetres of seam, not any one of them. Three taps worth three millimetres
// each choose between lines a millimetre and a half apart, which no single one
// of them could.
void testCloseCoverstitch(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nA coverstitch close beside the seam it holds down\n");

    Neckline neck;
    neck.seam = scoopNeck();
    neck.topstitch_mm = 1.6;
    Rng rng(8080);
    const cv::Mat frame = renderGarment(grid, neck, rng);

    double worstOff = 0.0;
    int read = 0;
    for (int trial = 0; trial < 5; ++trial) {
        const std::vector<cv::Point2d> seeds = pointAt(neck.seam, rng, 3.0, 0.0);
        CurveResult r;
        if (!snapCurve(frame, model, seeds, cfg, r)) continue;
        ++read;
        double rms = 0.0, worst = 0.0;
        pathError(r, neck.seam, rms, worst);
        worstOff = std::max(worstOff, rms);
    }
    check(read >= 4, "the lay reads");
    checkBelow(worstOff, 1.5,
               "a stitch line 1.6 mm out does not capture the reading, mm RMS off the seam");
}

// Two identical lines, one either side of where the operator pointed.
//
// The one failure this design can have that looks exactly like success, built
// on purpose so the guard against it can be shown to work. A double-needle
// coverstitch is two parallel lines a few millimetres apart, and an operator
// pointing between them has said nothing about which. Both lines are real, both
// are equally strong, both are the same distance from the taps, so which one
// wins is decided by nothing at all - and the reading is then repeatable within
// a pointing and jumps by the spacing between garments.
//
// Refusing is the only honest answer. `margin` is what refuses it, and it is
// the only thing in this file that can - the path is smooth, the confidence is
// full, and the drift is small, whichever of the two it picked.
cv::Mat renderTwinLines(const MmGrid& grid, const Seam& seam, double apartMm, Rng& rng) {
    cv::Mat img(kFrameH, kFrameW, CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        const double* rx = grid.x.ptr<double>(y);
        const double* ry = grid.y.ptr<double>(y);
        float* out = img.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) {
            const double d =
                std::hypot(rx[x] - seam.centre.x, ry[x] - seam.centre.y) - seam.radius;
            // Two steps the SAME way up, not two dark lines: a staircase.
            //
            // Two dark lines would leave a bright band between them, and a
            // bright band is a real feature that the search would rightly find
            // and the operator would rightly have pointed at. A staircase
            // leaves nothing at all in the middle, so there is genuinely
            // nothing to prefer about either tread.
            double v = 118.0;
            for (const double side : {-1.0, 1.0}) {
                v += 13.0 * 0.5 * (1.0 + std::tanh((d - side * apartMm) / 0.5));
            }
            out[x] = static_cast<float>(v);
        }
    }
    cv::Mat noise(img.size(), CV_32F);
    for (int y = 0; y < kFrameH; ++y) {
        float* r = noise.ptr<float>(y);
        for (int x = 0; x < kFrameW; ++x) r[x] = static_cast<float>(rng.signed1() * 2.0);
    }
    img += noise;
    cv::Mat out8;
    img.convertTo(out8, CV_8U);
    return out8;
}

void testTwinLines(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nTwo identical lines, one either side of the pointing\n");

    const Seam seam = scoopNeck();
    Rng rng(2727);
    const cv::Mat frame = renderTwinLines(grid, seam, 2.0, rng);

    // The guard is off by default, because the only numbers it has ever been
    // set against are the ones in this file, and on the first real garment it
    // met it refused a lay whose seam had been found at full confidence. What
    // is tested here is that the mechanism works and would catch this, so that
    // a bench which has collected its own margins can turn it on and rely on
    // it.
    Config armed = cfg;
    armed.curve_min_margin = 0.05;

    int refused = 0;
    for (int trial = 0; trial < 5; ++trial) {
        // Pointed exactly at the gap between them, which is the only way to
        // make this genuinely undecidable: point a little nearer one and the
        // machine prefers that one, correctly and repeatably. Here it cannot
        // prefer either, and the honest answer is to say so.
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 0.0, 0.0);
        CurveResult r;
        const bool ok = snapCurve(frame, model, seeds, armed, r);
        std::printf("        trial %d: %s  margin %.3f  conf %.2f\n", trial,
                    ok ? "ANSWERED" : "refused", r.margin, r.confidence);
        if (!ok) ++refused;
    }
    check(refused == 5,
          "a lay whose two lines cannot be told apart is refused rather than guessed at");
}

// A path drawn along the seam, the way a hand draws one.
//
// This is what three taps became. Two of the three were always easy - an end of
// a seam is a corner and there is something to aim at - and the middle one
// never was, because at the deepest part of an arc there is no landmark and the
// operator is estimating where the middle of a curve ought to be. On the first
// real garment that middle point landed sixteen millimetres into plain cloth,
// and since the arc goes exactly through all three it bent the whole corridor
// rather than shifting it.
//
// A trace has no such point. It is drawn while looking at the seam the whole
// way along, so it is near the seam everywhere, and what it carries instead is
// the hand: a point every fifteen millimetres or so at the rate the camera
// runs, each one shaking by several. That is what the smoothing in seedPath is
// for, and this is the test of it.
void testTracedSeam(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nA path drawn along the seam, shake and all\n");

    Neckline neck;
    neck.seam = scoopNeck();
    Rng rng(3131);
    const cv::Mat frame = renderGarment(grid, neck, rng);

    std::vector<double> arcs;
    double worstOff = 0.0;
    for (int trial = 0; trial < 4; ++trial) {
        // A hand's worth of trace: a point roughly every fifteen millimetres,
        // carrying the two errors a hovering finger actually has, which are not
        // the same kind of error and must not be modelled as one.
        //
        // Parallax is a BIAS. The fingertip is above the plane the model is
        // fitted to, so it is displaced outward from the centre of the picture
        // by an amount that grows smoothly across the frame - several
        // millimetres at the edges, none in the middle, and never jumping. It
        // is smooth by construction, which is exactly what the rigid model in
        // stage one exists to absorb.
        //
        // Shake is NOISE, and it is the smaller of the two: a couple of
        // millimetres, independent from one point to the next, which is what
        // the smoothing removes.
        //
        // Modelling both as one large random number, as the first version of
        // this test did, asks the search to do something no amount of it can:
        // average out a bias that is not random and follow noise that is not
        // real.
        std::vector<cv::Point2d> drawn;
        const int n = static_cast<int>(neck.seam.trueArc() / 15.0);
        // What parallax leaves behind AFTER the working distance has been used
        // to take it out - about a millimetre and a half at the edge of the
        // frame, from the four and a half it starts as. A bench run without a
        // lens focal length keeps the whole four, and where a second line lies
        // four millimetres from the seam that is enough to decide which one
        // gets measured; that is the reason to pass --focal-length-mm and not a
        // small one.
        const double lean = rng.signed1() * 1.5;
        for (int k = 0; k <= n; ++k) {
            const double t = static_cast<double>(k) / n;
            const double a = neck.seam.a0 + neck.seam.sweep * t;
            const double bias = lean * std::sin(kPi * t);   // smooth across the frame
            const double shake = rng.signed1() * 2.0;
            const double r = neck.seam.radius + bias + shake;
            drawn.emplace_back(neck.seam.centre.x + r * std::cos(a),
                               neck.seam.centre.y + r * std::sin(a));
        }
        check(drawn.size() >= 4, "the trace really is a trace and not three taps");

        CurveResult res;
        if (!snapCurve(frame, model, drawn, cfg, res)) {
            check(false, std::string("trial ") + std::to_string(trial) + " refused: " + res.error);
            continue;
        }
        double rms = 0.0, worst = 0.0;
        pathError(res, neck.seam, rms, worst);
        worstOff = std::max(worstOff, rms);
        arcs.push_back(res.arc_mm);
        std::printf("        trial %d: arc %.2f  off %.2f  conf %.2f  margin %.2f\n", trial,
                    res.arc_mm, rms, res.confidence, res.margin);
    }
    if (arcs.size() < 2) return;

    // A hand-drawn trace is not three taps and is not held to the same bar. A
    // millimetre on the path and two and a half across four traces is a
    // standard deviation near one, against a gauge budget of two and a half for
    // a half-inch band - so the trace spends well under half of what it is
    // allowed, and buys the middle of the seam being on the seam.
    checkBelow(worstOff, 1.0, "the path sits on the seam, mm RMS");
    const double spread = *std::max_element(arcs.begin(), arcs.end()) -
                          *std::min_element(arcs.begin(), arcs.end());
    checkBelow(spread, 2.5, "and four traces of the same seam agree");

    double mean = 0.0;
    for (double a : arcs) mean += a;
    mean /= static_cast<double>(arcs.size());
    checkNear(mean, neck.seam.trueArc(), 2.5, "the arc that comes back");
}

// A seam with no shade step at all, only stitching: a ridge rather than an
// edge.
//
// This is the case a first-derivative search cannot do in principle, not merely
// badly. The gradient magnitude of a ridge has a ZERO at the ridge and a peak
// on either flank, so a search for the strongest gradient locks onto a flank
// and reads a millimetre off - consistently, in a direction that depends on
// nothing the operator can see.
void testRidgeSeam(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nA seam that is a ridge rather than a step\n");

    Neckline neck;
    neck.seam = scoopNeck();
    Rng rng(616);
    // No step across the seam at all, and the topstitch sitting right on it, so
    // the only thing marking the seam is the stitching itself.
    neck.topstitch_mm = 0.0;
    const cv::Mat frame = renderGarment(grid, neck, rng, 0.0);

    const std::vector<cv::Point2d> seeds = pointAt(neck.seam, rng, 3.0, 0.0);
    CurveResult r;
    if (!snapCurve(frame, model, seeds, cfg, r)) {
        check(false, std::string("a stitched seam is refused: ") + r.error);
        return;
    }
    double rms = 0.0, worst = 0.0;
    pathError(r, neck.seam, rms, worst);
    checkBelow(rms, 0.5, "the path sits on the stitch line, not on one of its flanks, mm RMS");
    checkNear(r.arc_mm, neck.seam.trueArc(), 2.0, "the arc that comes back");
}

void testRefusals(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nRefusing a corridor it cannot read\n");
    const Seam seam = scoopNeck();
    Rng rng(99);

    // Nothing there at all. The search still returns a best path - smooth,
    // plausible, and invented - so this is the case that has to be caught.
    {
        const cv::Mat blank = renderSeam(grid, seam, false, rng);
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 2.0, 0.0);
        CurveResult r;
        const bool ok = snapCurve(blank, model, seeds, cfg, r);
        std::printf("        blank corridor: conf %.3f  drift %.2f  err %s\n", r.confidence,
                    r.drift_mm, r.error.c_str());
        check(!ok, "a corridor with no seam in it is refused rather than measured");
        check(r.confidence < cfg.curve_min_confidence,
              "and it is the confidence that refuses it");
    }

    // Pointed a long way off the seam, past the corridor. There is nothing to
    // snap to inside the corridor, so the same guard catches it.
    {
        const cv::Mat frame = renderSeam(grid, seam, true, rng);
        Seam wrong = seam;
        wrong.radius += 45.0;  // a corridor 45 mm inside the seam
        const std::vector<cv::Point2d> seeds = pointAt(wrong, rng, 0.0, 0.0);
        CurveResult r;
        const bool ok = snapCurve(frame, model, seeds, cfg, r);
        check(!ok, "pointing far off the seam is refused, not quietly snapped to something else");
    }

    // Off the picture.
    {
        const cv::Mat frame = renderSeam(grid, seam, true, rng);
        const std::vector<cv::Point2d> seeds{cv::Point2d(-60.0, 100.0), cv::Point2d(-40.0, 150.0),
                                             cv::Point2d(-60.0, 200.0)};
        CurveResult r;
        check(!snapCurve(frame, model, seeds, cfg, r), "a corridor off the picture is refused");
    }

    // Two taps in the same place.
    {
        const cv::Mat frame = renderSeam(grid, seam, true, rng);
        const std::vector<cv::Point2d> seeds{cv::Point2d(150.0, 150.0),
                                             cv::Point2d(151.0, 151.0)};
        CurveResult r;
        check(!snapCurve(frame, model, seeds, cfg, r), "two taps in the same place are refused");
    }
}

// -- the pointer --

// Pale cloth, near enough neutral, which is what a shirt on this bench is.
cv::Mat blankScene(int w, int h) {
    cv::Mat m(h, w, CV_8UC3, cv::Scalar(196, 200, 205));
    return m;
}

// An arm reaching in from the left, with its fingertip at `tip`.
//
// `shadowBeyondPx` casts a shadow that reaches PAST the fingertip, which is what
// a hand over pale cloth under a bench light actually does, and which is the
// thing that broke this on real garments: a shadow is darker than the cloth by
// a long way, it lies in the same direction as the arm, and its far corner is
// further from the border than any part of the hand. Anything that finds the
// foreground by how much the brightness changed takes that corner for a
// fingertip and puts the point a couple of centimetres past where anybody
// pointed.
cv::Mat sceneWithArm(int w, int h, cv::Point tip, int shadowBeyondPx = 0) {
    cv::Mat m = blankScene(w, h);

    if (shadowBeyondPx > 0) {
        // Cast along the arm and past the tip. Every channel scaled by the same
        // factor, because that is what a shadow is: less light, same light.
        cv::Mat shade = m.clone();
        const cv::Point from(-20, tip.y + 60);
        const double len = std::hypot(tip.x - from.x, tip.y - from.y);
        const cv::Point beyond(
            static_cast<int>(tip.x + (tip.x - from.x) / len * shadowBeyondPx),
            static_cast<int>(tip.y + (tip.y - from.y) / len * shadowBeyondPx));
        cv::line(shade, from, beyond, cv::Scalar(0, 0, 0), 150, cv::LINE_AA);
        cv::circle(shade, beyond, 60, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);

        cv::Mat where;
        cv::cvtColor(shade, where, cv::COLOR_BGR2GRAY);
        cv::threshold(where, where, 40, 255, cv::THRESH_BINARY_INV);
        const cv::Mat darkened = m * 0.62;
        darkened.copyTo(m, where);
    }

    // Skin: darker than the cloth AND a different colour, which is what lets it
    // be told from its own shadow.
    //
    // Drawn as an arm actually is: a THICK forearm, a hand, and a THIN finger
    // out in front of it. The first version of this fixture was one uniform
    // stroke from border to fingertip, and that single simplification hid a
    // real bug for three rounds - the setback that puts the tip under the pad
    // measures how wide the fingertip is, and with a finger the same width as
    // the forearm it cannot matter what it measures it over. On a real hand it
    // measured the fist, set the tip back thirty-five millimetres, and tracked
    // the middle of the hand.
    const cv::Scalar skin(55, 75, 120);
    const cv::Point from(-20, tip.y + 160);
    const double len = std::hypot(tip.x - from.x, tip.y - from.y);
    const cv::Point knuckle(static_cast<int>(tip.x - (tip.x - from.x) / len * 90),
                            static_cast<int>(tip.y - (tip.y - from.y) / len * 90));
    cv::line(m, from, knuckle, skin, 150, cv::LINE_AA);   // forearm, 150 px across
    cv::circle(m, knuckle, 78, skin, -1, cv::LINE_AA);    // the hand
    cv::line(m, knuckle, tip, skin, 34, cv::LINE_AA);     // the finger, 34 px
    cv::circle(m, tip, 17, skin, -1, cv::LINE_AA);
    return m;
}

void testFinger() {
    std::printf("\nFinding the finger\n");
    const int w = 1200, h = 800;

    Config cfg;
    cfg.detect_step = 4;
    cfg.finger_dwell_seconds = 0.5;
    cfg.finger_rearm_seconds = 0.8;

    // Half a millimetre per pixel, so this frame is 600 by 400 mm of mat and a
    // hand drawn on it is the size of a hand.
    //
    // The bench's own 0.138 mm per pixel would make this frame 165 mm across -
    // smaller than the hand it is meant to contain - and a fixture at that
    // scale cannot be a fair test of anything that reasons about how wide a
    // fingertip is, which is most of what is below. The forearm here is 75 mm
    // across, the hand 39 mm, the finger 17 mm wide and 45 mm long.
    const double mmPerPx = 0.5;
    cv::Matx33d H(1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0);
    PlaneModel model(H);
    RoiBox roi{0, 0, w, h};

    const cv::Point tip(760, 430);

    // A blob that touches no border is not an arm. This is the guard that stops
    // the garment settling further, or a shadow crossing the mat, from putting
    // a seed somewhere nobody pointed.
    {
        cv::Mat ref = blankScene(w, h);
        cv::Mat cur = ref.clone();
        cv::circle(cur, cv::Point(600, 400), 60, cv::Scalar(64, 70, 96), -1);
        FingerTracker t(cfg);
        t.setReference(ref, roi, true);
        const FingerState s = t.update(cur, &model, 0.0);
        check(!s.present, "a moving blob that touches no border is not a hand");
    }

    // An arm is.
    {
        cv::Mat ref = blankScene(w, h);
        cv::Mat cur = sceneWithArm(w, h, tip);
        FingerTracker t(cfg);
        t.setReference(ref, roi, true);
        const FingerState s = t.update(cur, &model, 0.0);
        check(s.present, "an arm reaching in from the border is");
        if (s.present) {
            const double off = std::hypot(s.tip_px.x - tip.x, s.tip_px.y - tip.y);
            checkBelow(off * mmPerPx, 4.0,
                       "and the tip is found where the pad touches, not half a finger past "
                       "it, mm");
        }
    }

    // The dwell. Holding still commits a point; moving does not.
    {
        cv::Mat ref = blankScene(w, h);
        FingerTracker t(cfg);
        t.setReference(ref, roi, true);

        bool committed = false;
        cv::Point2d point;
        for (int k = 0; k < 12; ++k) {
            const double now = k * 0.1;
            const cv::Mat cur = sceneWithArm(w, h, tip);
            const FingerState s = t.update(cur, &model, now);
            if (s.committed) {
                committed = true;
                point = s.point_mm;
                check(now >= cfg.finger_dwell_seconds - 1e-9,
                      "the point lands only after the dwell has been held");
                break;
            }
        }
        check(committed, "holding still commits a point");
        if (committed) {
            const std::vector<cv::Point2d> want = model.pxToMm({cv::Point2d(tip.x, tip.y)});
            checkBelow(std::hypot(point.x - want[0].x, point.y - want[0].y), 4.0,
                       "committed near where the finger actually was, mm");
        }
    }

    // Parallax, put in on purpose and taken out again.
    //
    // A fingertip ten millimetres above the mat images displaced OUTWARD from
    // the middle of the picture, by the radius times the height over the
    // working distance. It is the largest single error in pointing and the only
    // one that is not random: smoothing a trace will average away shake all day
    // and never touch this, because it is the same push in the same direction
    // every time. Here the tip is drawn where the camera would really see it,
    // and what comes back has to be where the finger really was.
    {
        // Its own field, and a wide one, because parallax is a function of how
        // far out from the middle of the picture the finger is. The small frame
        // the other checks use spans 165 mm and displaces a tip by less than a
        // millimetre, which would let this pass whatever the code did. This one
        // spans 720 by 480 mm, which is the bench.
        const int bw = 2400, bh = 1600;
        const double mmPerPx = 0.3;
        cv::Matx33d bigH(1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0);
        PlaneModel big(bigH);

        Config lens = cfg;
        lens.camera_distance_mm = 935.0;  // a 16 mm lens at this magnification
        lens.finger_height_mm = 10.0;

        const cv::Point2d axis(0.5 * bw, 0.5 * bh);
        const cv::Point trueAt(2100, 300);  // well out toward a corner
        const double push = 935.0 / (935.0 - 10.0);
        const cv::Point seenAt(static_cast<int>(axis.x + (trueAt.x - axis.x) * push),
                               static_cast<int>(axis.y + (trueAt.y - axis.y) * push));

        const double shifted = std::hypot(seenAt.x - trueAt.x, seenAt.y - trueAt.y) * mmPerPx;
        check(shifted > 2.5, "the fixture really does displace the tip, by millimetres");

        cv::Mat ref = blankScene(bw, bh);
        cv::Mat cur = sceneWithArm(bw, bh, seenAt);
        const RoiBox bigRoi{0, 0, bw, bh};

        const std::vector<cv::Point2d> want = big.pxToMm({cv::Point2d(trueAt.x, trueAt.y)});

        FingerTracker off(cfg);  // no working distance: no correction
        off.setReference(ref, bigRoi, true);
        const FingerState a = off.update(cur, &big, 0.0);

        FingerTracker on(lens);
        on.setReference(ref, bigRoi, true);
        const FingerState b = on.update(cur, &big, 0.0);

        if (a.has_mm && b.has_mm) {
            // Measured as distance FROM THE MIDDLE OF THE PICTURE, because that
            // is the only thing parallax changes. The tip also carries the
            // fixture's own geometry - a synthetic finger drawn twenty-seven
            // millimetres wide at this scale - and that error points along the
            // arm rather than outward, so a straight distance between two
            // points would mix the two and report neither.
            const std::vector<cv::Point2d> axisMm = big.pxToMm({axis});
            const auto radius = [&](const cv::Point2d& p) {
                return std::hypot(p.x - axisMm[0].x, p.y - axisMm[0].y);
            };
            // Compared with each other, not with the truth.
            //
            // Both trackers set the tip back under the pad by the same amount,
            // and that setback runs along the arm - which on this lay has a
            // radial component of its own, pointing inward, that happens to
            // cancel most of the parallax pushing outward. Measured against the
            // truth, the uncorrected tip therefore looks nearly right and the
            // corrected one looks worse, and neither number is about parallax
            // at all. The difference BETWEEN the two carries the setback on
            // both sides, so it cancels, and what is left is exactly what the
            // correction does.
            const double injected =
                radius(big.pxToMm({cv::Point2d(seenAt.x, seenAt.y)})[0]) - radius(want[0]);
            const double removed = radius(a.tip_mm) - radius(b.tip_mm);
            std::printf("        parallax: %.2f mm put in, %.2f mm taken out\n",
                        injected, removed);
            check(injected > 2.5, "the fixture really does push the tip outward, radially");
            checkNear(removed, injected, 0.4,
                      "and the working distance takes out just about exactly that much");
        } else {
            check(false, "the displaced tip is found at all");
        }
    }

    // The shadow the hand casts is not the hand.
    //
    // This is what broke it on real garments. The shadow reaches further than
    // the fingertip, lies in the same direction, and is a large, solid,
    // convincing blob touching the same border. A foreground test that only
    // asks how much the brightness changed cannot tell them apart.
    {
        cv::Mat ref = blankScene(w, h);
        cv::Mat cur = sceneWithArm(w, h, tip, 190);

        // The fixture has to actually contain the trap, or this check passes
        // for the wrong reason. Two hundred pixels past the tip, along the arm,
        // the cloth must be substantially darker than it was and must still be
        // the same colour it was - a shadow, not an object.
        {
            // On the shadow's own axis, well past the fingertip: the arm runs
            // in from the left and only slightly upward, so the shadow beyond
            // the tip is almost straight out to the right of it.
            const cv::Point far(tip.x + 150, tip.y - 12);
            const cv::Vec3b was = ref.at<cv::Vec3b>(far);
            const cv::Vec3b now = cur.at<cv::Vec3b>(far);
            const double lumWas = was[0] + was[1] + was[2];
            const double lumNow = now[0] + now[1] + now[2];
            check(lumNow < 0.75 * lumWas, "the fixture really does cast a shadow past the tip");
            const double spread =
                std::fabs(static_cast<double>(now[0]) / (was[0] + 1.0) -
                          static_cast<double>(now[2]) / (was[2] + 1.0));
            check(spread < 0.05, "and it is a shadow rather than an object, by its colour");
        }

        FingerTracker t(cfg);
        t.setReference(ref, roi, true);
        const FingerState s = t.update(cur, &model, 0.0);
        check(s.present, "an arm casting a shadow past its own fingertip is still found");
        if (s.present) {
            const double off = std::hypot(s.tip_px.x - tip.x, s.tip_px.y - tip.y) * mmPerPx;
            checkBelow(off, 6.0,
                       "and the tip is the finger, not the far corner of its shadow, mm");
        }
    }

    // The whole gesture: hold, draw, and stop.
    //
    // The two holds are timed differently on purpose and this is where that is
    // checked. Starting is easy - the hand arrives and settles. Ending must not
    // be, because a hand drawing a seam slows almost to a stop at every corner
    // and at the deepest part of every arc; at the six tenths of a second that
    // starts a trace, each of those pauses would end it halfway along the
    // collar for no reason the operator could see.
    {
        Config g = cfg;
        g.finger_dwell_seconds = 0.5;
        g.finger_trace_end_seconds = 3.0;

        FingerTracker t(g);
        t.setReference(blankScene(w, h), roi, true);

        double now = 0.0;
        const auto step = [&](cv::Point at) {
            now += 0.1;
            return t.update(sceneWithArm(w, h, at), &model, now);
        };

        // Hold at the start.
        cv::Point p(400, 300);
        bool started = false;
        for (int k = 0; k < 10 && !started; ++k) started = step(p).tracing;
        check(started, "a hold starts the trace");

        // Draw. Pausing on the way - a second and a half of it, which is what a
        // hand does at a corner - must not end anything.
        bool endedEarly = false;
        for (int k = 0; k < 14; ++k) {
            p.x += 22;
            if (step(p).finished) endedEarly = true;
        }
        for (int k = 0; k < 15; ++k) {  // a pause, 1.5 s of it
            if (step(p).finished) endedEarly = true;
        }
        check(!endedEarly, "and pausing for a second and a half part way along does not end it");
        for (int k = 0; k < 6; ++k) {
            p.x += 22;
            if (step(p).finished) endedEarly = true;
        }
        check(!endedEarly, "nor does carrying on afterwards");

        // Now stand still for three.
        FingerState done;
        for (int k = 0; k < 40; ++k) {
            done = step(p);
            if (done.finished) break;
        }
        check(done.finished, "standing still for three seconds ends it");
        check(done.path_mm.size() >= 8,
              "and the path it hands over is the whole line, not the last of it");
    }

    // Taking the hand away ends it too, which is what most operators will
    // actually do.
    {
        Config g = cfg;
        g.finger_dwell_seconds = 0.5;
        FingerTracker t(g);
        t.setReference(blankScene(w, h), roi, true);
        double now = 0.0;
        cv::Point p(400, 300);
        for (int k = 0; k < 10; ++k) { now += 0.1; t.update(sceneWithArm(w, h, p), &model, now); }
        for (int k = 0; k < 16; ++k) {
            p.x += 22;
            now += 0.1;
            t.update(sceneWithArm(w, h, p), &model, now);
        }
        FingerState gone;
        for (int k = 0; k < 6; ++k) {
            now += 0.1;
            gone = t.update(blankScene(w, h), &model, now);
            if (gone.finished) break;
        }
        check(gone.finished, "taking the hand away ends the trace");
    }

    // A hand sweeping across the mat commits nothing, which is what lets the
    // operator reach over the shirt without scattering seeds behind them.
    {
        cv::Mat ref = blankScene(w, h);
        FingerTracker t(cfg);
        t.setReference(ref, roi, true);
        bool any = false;
        for (int k = 0; k < 14; ++k) {
            const cv::Mat cur = sceneWithArm(w, h, cv::Point(300 + k * 45, 430));
            if (t.update(cur, &model, k * 0.1).committed) any = true;
        }
        check(!any, "a hand moving across the mat commits nothing");
    }
}

// -- the gesture, at the frame rate this bench actually has --
//
// Hold, draw, lift. Every wait in it is counted in frames because at 1.9
// frames a second every frame is half a second of the operator looking at a
// ring, and the two questions that matter are how many frames each wait takes
// and whether anything in the middle can end a trace by accident.
void testTraceGesture() {
    std::printf("\nHold, draw, lift - at two frames a second\n");
    const int w = 1200, h = 800;
    const double mmPerPx = 0.5;
    cv::Matx33d H(1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0 / mmPerPx, 0.0, 0.0, 0.0, 1.0);
    PlaneModel model(H);
    RoiBox roi{0, 0, w, h};

    Config cfg;  // the defaults, because the defaults are what the bench runs
    cfg.detect_step = 4;

    {
        FingerTracker t(cfg);
        t.setReference(blankScene(w, h), roi, true);
        double now = 0.0;
        const double dt = 1.0 / 1.9;
        cv::Point p(400, 300);

        int framesToStart = 0;
        bool started = false;
        for (int k = 0; k < 6 && !started; ++k) {
            now += dt;
            started = t.update(sceneWithArm(w, h, p), &model, now).tracing;
            ++framesToStart;
        }
        check(started, "a hold starts the trace at 1.9 fps");
        checkNear(framesToStart, 2.0, 0.0, "and it takes two frames - one second - not three");

        // Draw, pause for four frames - two seconds, which is what a hand does
        // at a corner - and draw on. Nothing in that may end the trace.
        bool endedEarly = false;
        FingerState s;
        for (int k = 0; k < 5; ++k) {
            p.x += 40;
            now += dt;
            s = t.update(sceneWithArm(w, h, p), &model, now);
            endedEarly = endedEarly || s.finished;
        }
        for (int k = 0; k < 4; ++k) {
            now += dt;
            s = t.update(sceneWithArm(w, h, p), &model, now);
            endedEarly = endedEarly || s.finished;
        }
        for (int k = 0; k < 5; ++k) {
            p.x += 40;
            now += dt;
            s = t.update(sceneWithArm(w, h, p), &model, now);
            endedEarly = endedEarly || s.finished;
        }
        check(!endedEarly, "a two-second pause part way along does not end it");
        check(!s.rejected, "and a hand drawing at 38 mm/s is never taken for a lost finger");

        int framesToEnd = 0;
        FingerState gone;
        for (int k = 0; k < 5 && !gone.finished; ++k) {
            now += dt;
            gone = t.update(blankScene(w, h), &model, now);
            ++framesToEnd;
        }
        check(gone.finished, "taking the hand away ends it");
        checkNear(framesToEnd, 2.0, 0.0, "after two frames without a hand - one second");
        check(gone.path_mm.size() >= 10, "and the path is every frame the hand was seen moving");
    }

    // The motion gate: a tip that could not have got there is not a point.
    {
        FingerTracker t(cfg);
        t.setReference(blankScene(w, h), roi, true);
        double now = 0.0;
        const double dt = 0.1;
        cv::Point p(400, 300);
        bool started = false;
        for (int k = 0; k < 8 && !started; ++k) {
            now += dt;
            started = t.update(sceneWithArm(w, h, p), &model, now).tracing;
        }
        check(started, "a trace running at 10 fps");

        FingerState s;
        for (int k = 0; k < 5; ++k) {
            p.x += 10;
            now += dt;
            s = t.update(sceneWithArm(w, h, p), &model, now);
        }
        const std::size_t before = s.trace_mm.size();

        // One frame with the hand 200 mm away: a fold that lifted, a shadow's
        // corner, the elbow. Not believed, not recorded.
        now += dt;
        s = t.update(sceneWithArm(w, h, cv::Point(p.x, p.y + 400)), &model, now);
        check(s.rejected, "a tip that jumped 200 mm in a tenth of a second is not believed");
        check(s.trace_mm.size() == before, "and is not recorded");

        // Back on the line the next frame: recorded as if nothing happened.
        p.x += 10;
        now += dt;
        s = t.update(sceneWithArm(w, h, p), &model, now);
        check(!s.rejected && s.trace_mm.size() == before + 1,
              "the next frame back on the line is recorded");

        // Two frames agreeing on a new place: the hand really is there.
        const cv::Point far(p.x + 20, p.y + 400);
        now += dt;
        s = t.update(sceneWithArm(w, h, far), &model, now);
        check(s.rejected, "the first frame at a new place is doubted");
        now += dt;
        s = t.update(sceneWithArm(w, h, cv::Point(far.x + 10, far.y)), &model, now);
        check(!s.rejected, "the second, agreeing with it, is believed");
    }
}

// -- which end of the arm is the finger --
//
// Straight at the mask, with no segmentation in front of it, so that what is
// being asked is exactly the one question findArmTip answers.
//
// A hand at the scale a hand is: a 65 mm forearm, a palm 95 mm long and 85 mm
// across, and an index finger 75 mm of 17 mm out in front of that. The palm is
// the reason this test exists. It is five times the width of the finger and
// nearly as long, so as the wrist turns it is the palm, not the forearm, that
// comes to look like the far end of the arm - and the far end of the arm was
// what the tip used to be.
void drawHand(cv::Mat& mask, cv::Point2d fromPt, cv::Point2d tipPt, double bend, double perMm) {
    const double vx = tipPt.x - fromPt.x, vy = tipPt.y - fromPt.y;
    const double n = std::hypot(vx, vy);
    if (n < 1e-6) return;
    // The hand's own axis: the whole reach, turned at the wrist.
    const double a = std::atan2(vy, vx) + bend;
    const cv::Point2d u(std::cos(a), std::sin(a));
    const cv::Point2d base(tipPt.x - 83.5 * perMm * u.x, tipPt.y - 83.5 * perMm * u.y);
    const cv::Point2d wrist(base.x - 95.0 * perMm * u.x, base.y - 95.0 * perMm * u.y);
    const auto at = [](const cv::Point2d& p) {
        return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    };
    const cv::Scalar on(255, 255, 255);
    cv::line(mask, at(fromPt), at(wrist), on, static_cast<int>(std::lround(65.0 * perMm)),
             cv::LINE_AA);
    cv::line(mask, at(wrist), at(base), on, static_cast<int>(std::lround(85.0 * perMm)),
             cv::LINE_AA);
    cv::line(mask, at(base), at(tipPt), on, static_cast<int>(std::lround(17.0 * perMm)),
             cv::LINE_AA);
    cv::circle(mask, at(tipPt), static_cast<int>(std::lround(8.5 * perMm)), on, -1, cv::LINE_AA);
}

void testArmTipOnBentWrist() {
    std::printf("\nWhich end of a bent arm is the finger\n");
    const int w = 900, h = 700;
    const double perMm = 2.0;  // half a millimetre a pixel, so the hand is a hand

    // Nought is the arm reaching straight at the seam, which is the only pose
    // the straight-line rule ever had to survive. A hundred is the wrist turned
    // right over, which is what the far end of a neckline asks for.
    for (double deg : {0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0}) {
        cv::Mat mask(h, w, CV_8U, cv::Scalar(0));
        const cv::Point2d tip(400.0, 300.0);
        drawHand(mask, cv::Point2d(w + 40.0, 380.0), tip, deg * kPi / 180.0, perMm);
        cv::threshold(mask, mask, 128, 255, cv::THRESH_BINARY);

        cv::Point2d got;
        const bool ok = findArmTip(mask, 3, 200.0, got, nullptr, 15.0 * perMm, 10.0 * perMm);
        check(ok, "the arm is found at all");
        const double off = ok ? std::hypot(got.x - tip.x, got.y - tip.y) / perMm : 1e9;
        std::printf("        wrist %3.0f deg: found (%.0f,%.0f) against (%.0f,%.0f), off %.1f mm\n",
                    deg, got.x, got.y, tip.x, tip.y, off);
        // A fingertip is ambiguous by most of a centimetre under an occluding
        // nail, so a centimetre is the bar. Landing on the palm is 80 mm out and
        // could never be mistaken for passing this.
        checkBelow(off, 10.0, "the tip is the finger, not the palm behind it, mm");
    }

    // And it has to be affordable, because it runs on every frame of every
    // trace. At this bench's own decimation a mask is about 900 by 600.
    {
        cv::Mat mask(600, 900, CV_8U, cv::Scalar(0));
        drawHand(mask, cv::Point2d(940.0, 380.0), cv::Point2d(400.0, 300.0), 1.0, 2.0);
        cv::threshold(mask, mask, 128, 255, cv::THRESH_BINARY);
        cv::Point2d got;
        const double t0 = static_cast<double>(cv::getTickCount());
        const int runs = 50;
        for (int k = 0; k < runs; ++k) findArmTip(mask, 3, 200.0, got, nullptr, 30.0, 20.0);
        const double ms = 1000.0 * (static_cast<double>(cv::getTickCount()) - t0) /
                          cv::getTickFrequency() / runs;
        std::printf("        %.1f ms a frame on a %d x %d mask\n", ms, mask.cols, mask.rows);
        checkBelow(ms, 25.0, "finding the tip costs less than a frame, ms");
    }
}

// -- a hand drawing a curve, through the tracker that watches it --
//
// Everything above this point tests the trace by handing snapCurve a path a
// hand is IMAGINED to have drawn. That is a fair test of the snap and no test
// at all of the tracker, and two of the failures that matter most live in the
// tracker: it can lose the fingertip, and it can keep the fingertip and still
// hand back the wrong shape.
//
// The straight-line fixture in testFinger cannot see either one. A tip dragged
// along a straight line is recovered by almost any rule - the furthest point of
// the blob, the middle of the hand, the far corner of the bounding box - so the
// trace comes back straight and the check passes whatever the tracker actually
// followed. A seam is not straight. A neckline turns through a hundred and
// sixty degrees, and the whole reason a curve step exists is that the shape
// between the ends is the measurement.
//
// So the fixture below draws the arm on the garment, moves the fingertip along
// the seam itself, and asks what came back: how far it sits from the arc, how
// much of the arc's own bow it kept, and what the snap then measures. The bow is
// the check a straight line cannot pass - seventy-five millimetres of it here.
struct ArmPose {
    cv::Point2d fromMm{430.0, 120.0};  // where the arm crosses the border
    double wrist = 0.0;   // how far the hand is turned off the reach, radians
    double fps = 10.0;
    double shakeMm = 0.0;
    // The wrist does not hold one angle for the length of a seam. It starts
    // turned one way, comes through straight, and ends turned the other, which
    // is what an arm does when the elbow stays on the bench and the hand goes
    // round a neckline. A fixed angle is the easy half of this test.
    double wristSwingTo = 1e9;
    double at(double t) const {
        return wristSwingTo > 1e8 ? wrist : wrist + (wristSwingTo - wrist) * t;
    }
};

cv::Mat garmentWithArm(const cv::Mat& garmentBgr, const PlaneModel& model, const ArmPose& pose,
                       cv::Point2d tipMm, double wristNow) {
    cv::Mat m = garmentBgr.clone();
    const double vx = tipMm.x - pose.fromMm.x, vy = tipMm.y - pose.fromMm.y;
    const double n = std::hypot(vx, vy);
    if (n < 1e-6) return m;
    const double a = std::atan2(vy, vx) + wristNow;
    const cv::Point2d u(std::cos(a), std::sin(a));
    const cv::Point2d baseMm(tipMm.x - 83.5 * u.x, tipMm.y - 83.5 * u.y);
    const cv::Point2d wristMm(baseMm.x - 95.0 * u.x, baseMm.y - 95.0 * u.y);

    const std::vector<cv::Point2d> px = model.mmToPx({pose.fromMm, wristMm, baseMm, tipMm});
    const auto at = [](const cv::Point2d& p) {
        return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    };
    const cv::Scalar skin(55, 75, 120);
    cv::line(m, at(px[0]), at(px[1]), skin, static_cast<int>(std::lround(65.0 * kPxPerMm)),
             cv::LINE_AA);
    cv::line(m, at(px[1]), at(px[2]), skin, static_cast<int>(std::lround(85.0 * kPxPerMm)),
             cv::LINE_AA);
    cv::line(m, at(px[2]), at(px[3]), skin, static_cast<int>(std::lround(17.0 * kPxPerMm)),
             cv::LINE_AA);
    cv::circle(m, at(px[3]), static_cast<int>(std::lround(8.5 * kPxPerMm)), skin, -1, cv::LINE_AA);
    return m;
}

// How far a path departs from the straight line between its own ends, at the
// worst point. The shape of a curve, in one number.
double bowOf(const std::vector<cv::Point2d>& p) {
    if (p.size() < 3) return 0.0;
    const cv::Point2d a = p.front(), b = p.back();
    const double c = std::hypot(b.x - a.x, b.y - a.y);
    if (c < 1e-9) return 0.0;
    const cv::Point2d u((b.x - a.x) / c, (b.y - a.y) / c);
    double bow = 0.0;
    for (const cv::Point2d& q : p) {
        bow = std::max(bow, std::fabs((q.x - a.x) * u.y - (q.y - a.y) * u.x));
    }
    return bow;
}

struct TraceStats {
    bool started = false;
    bool finished = false;
    int points = 0;
    double rms = 0.0;
    double worst = 0.0;
    double drawn = 0.0;
    double bow = 0.0;
    std::vector<cv::Point2d> path;
};

TraceStats runTrace(const cv::Mat& garment, const PlaneModel& model, const Config& cfg,
                    const Seam& seam, const ArmPose& pose) {
    TraceStats st;
    Config g = cfg;
    g.finger_dwell_seconds = 0.5;
    g.finger_trace_end_seconds = 3.0;

    FingerTracker t(g);
    RoiBox roi{0, 0, kFrameW, kFrameH};
    t.setReference(garment, roi, true);

    Rng rng(4242);
    const double dt = 1.0 / pose.fps;
    double now = 0.0;
    const auto step = [&](const cv::Point2d& tipMm, double u) {
        now += dt;
        return t.update(garmentWithArm(garment, model, pose, tipMm, pose.at(u)), &model, now);
    };

    // Hold at one end, draw the whole seam, take the hand away.
    for (int k = 0; k < 12 && !st.started; ++k) st.started = step(seam.at(0.0), 0.0).tracing;
    if (!st.started) return st;

    FingerState s;
    const int frames = std::max(6, static_cast<int>(std::lround(5.0 * pose.fps)));
    for (int k = 1; k <= frames; ++k) {
        const double u = static_cast<double>(k) / frames;
        cv::Point2d p = seam.at(u);
        if (pose.shakeMm > 0.0) {
            p.x += rng.signed1() * pose.shakeMm;
            p.y += rng.signed1() * pose.shakeMm;
        }
        s = step(p, u);
        if (s.finished) break;
    }
    for (int k = 0; k < 8 && !s.finished; ++k) {
        now += dt;
        s = t.update(garment, &model, now);
    }
    st.finished = s.finished;
    if (!s.finished) return st;

    st.path = s.path_mm;
    st.points = static_cast<int>(s.path_mm.size());
    for (const cv::Point2d& p : s.path_mm) {
        const double d = seam.offBy(p);
        st.rms += d * d;
        st.worst = std::max(st.worst, d);
    }
    st.rms = std::sqrt(st.rms / std::max<std::size_t>(1, s.path_mm.size()));
    for (std::size_t k = 1; k < s.path_mm.size(); ++k) {
        st.drawn += std::hypot(s.path_mm[k].x - s.path_mm[k - 1].x,
                               s.path_mm[k].y - s.path_mm[k - 1].y);
    }
    st.bow = bowOf(s.path_mm);
    return st;
}

void testFingerFollowsCurve(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nA hand drawing a curve, watched by the tracker\n");

    Neckline neck;
    neck.seam = scoopNeck();
    Rng rng(9091);
    const cv::Mat gray = renderGarment(grid, neck, rng);
    cv::Mat garment;
    cv::cvtColor(gray, garment, cv::COLOR_GRAY2BGR);
    const double wantBow = neck.seam.radius * (1.0 - std::cos(neck.seam.sweep / 2.0));
    std::printf("        the seam: arc %.1f mm, bow %.1f mm\n", neck.seam.trueArc(), wantBow);

    struct Case { const char* name; ArmPose pose; };
    const Case cases[] = {
        {"arm swings, 10 fps, no shake", ArmPose{{430.0, 120.0}, 0.0, 10.0, 0.0}},
        {"wrist bent 30 deg", ArmPose{{430.0, 120.0}, 0.52, 10.0, 0.0}},
        {"wrist bent 60 deg", ArmPose{{430.0, 120.0}, 1.05, 10.0, 0.0}},
        {"wrist bent -60 deg", ArmPose{{430.0, 120.0}, -1.05, 10.0, 0.0}},
        {"2 fps", ArmPose{{430.0, 120.0}, 0.0, 2.0, 0.0}},
        {"2 fps, wrist 60 deg", ArmPose{{430.0, 120.0}, 1.05, 2.0, 0.0}},
        {"3 mm of shake", ArmPose{{430.0, 120.0}, 0.0, 10.0, 3.0}},
        {"arm in from below", ArmPose{{180.0, 330.0}, 0.0, 10.0, 0.0}},
        {"arm in from below, wrist 60", ArmPose{{180.0, 330.0}, 1.05, 10.0, 0.0}},
        {"wrist swings -60 to +60", ArmPose{{430.0, 120.0}, -1.05, 10.0, 0.0, 1.05}},
        {"wrist swings, 2 fps, 3 mm shake", ArmPose{{430.0, 120.0}, -1.05, 2.0, 3.0, 1.05}},
        {"wrist swings, from below", ArmPose{{180.0, 330.0}, -1.05, 10.0, 0.0, 1.05}},
    };

    for (const Case& c : cases) {
        const TraceStats st = runTrace(garment, model, cfg, neck.seam, c.pose);
        check(st.started && st.finished, std::string(c.name) + ": the gesture is a trace");
        if (!st.finished) continue;

        CurveResult res;
        const bool ok = snapCurve(gray, model, st.path, cfg, res);
        std::printf("        %-32s %3d pts  off %5.2f/%5.2f  len %6.1f  bow %5.1f  ->  ",
                    c.name, st.points, st.rms, st.worst, st.drawn, st.bow);
        if (ok) {
            double srms = 0.0, sworst = 0.0;
            pathError(res, neck.seam, srms, sworst);
            std::printf("arc %6.2f  off %.2f  conf %.2f  margin %.2f  drift %.1f\n", res.arc_mm,
                        srms, res.confidence, res.margin, res.drift_mm);
            checkBelow(srms, 1.0, "the snapped seam sits on the seam, mm RMS");
            checkNear(res.arc_mm, neck.seam.trueArc(), 5.0, "and the arc it measures");
        } else {
            std::printf("REFUSED: %s\n", res.error.c_str());
            check(false, std::string(c.name) + ": refused - " + res.error);
            continue;
        }

        // The trace itself, before the snap touches it. A tip that has wandered
        // onto the hand still yields a smooth, plausible, confident reading, so
        // the numbers above cannot be trusted to notice it - these can.
        checkBelow(st.rms, 2.5, "the traced path sits on the seam it was drawn along, mm RMS");
        checkNear(st.bow, wantBow, 6.0, "and it kept the curve's bow rather than cutting across");
    }
}

// -- two points, measured straight between them --
//
// The plainest step there is, and the one with the least in it to go wrong:
// the reading is the two points. What has to hold is that the number is the
// distance between them and nothing cleverer, that the verdict is taken by the
// same rule as every other step, and that a style file can say "distance" and
// be read back saying it.
void testDistanceStep() {
    std::printf("\nTwo points, measured straight between them\n");

    Template t;
    t.style = "ruler";
    Step s;
    s.name = "neck_width";
    s.kind = "distance";
    s.target_mm = 180.0;
    s.tol_minus_mm = -10.0;
    s.tol_plus_mm = 10.0;
    t.steps.push_back(s);
    check(t.steps[0].isDistance() && t.steps[0].isPointed() && !t.steps[0].isCurve(),
          "a distance step is pointed at, and is not a curve");

    StepResult r = judgeDistance(t, 0, cv::Point2d(100.0, 50.0), cv::Point2d(280.0, 50.0));
    check(r.measured, "two points give a reading");
    checkNear(r.measured_mm, 180.0, 1e-9, "and it is the straight distance between them, mm");
    check(r.pass, "on target, it passes");
    check(r.kind == "distance", "and the log line says it was a distance");
    checkNear(r.chord_mm, 180.0, 1e-9, "recorded as the chord, which it is");

    // Off at an angle, so nothing is being read off one axis.
    r = judgeDistance(t, 0, cv::Point2d(10.0, 20.0), cv::Point2d(10.0 + 108.0, 20.0 + 144.0));
    checkNear(r.measured_mm, 180.0, 1e-9, "a diagonal reads the same as a horizontal");

    r = judgeDistance(t, 0, cv::Point2d(100.0, 50.0), cv::Point2d(100.0, 244.0));
    check(r.measured && !r.pass, "14 mm over on a 10 mm band fails");
    check(r.reason.find("over by") != std::string::npos, "and says by how much");

    r = judgeDistance(t, 0, cv::Point2d(100.0, 50.0), cv::Point2d(100.3, 50.2));
    check(!r.measured && !r.pass, "two taps on the same place are not a distance");

    // Through the style file and back.
    std::string error;
    const std::string path = "curve_check_distance_tmp.json";
    check(saveTemplate(t, path, error), "the style writes: " + error);
    Template back;
    check(loadTemplate(path, back, error), "and reads back: " + error);
    check(back.steps.size() == 1 && back.steps[0].isDistance(),
          "and the step is still a distance step afterwards");
    std::remove(path.c_str());

    // A kind this build does not know is still refused, so a typo in a style
    // file is a message and not a silently folded edge.
    vjson::Value bad;
    check(vjson::parse("{\"style\":\"x\",\"steps\":[{\"name\":\"a\",\"target_mm\":10,"
                       "\"tol_minus_mm\":-1,\"tol_plus_mm\":1,\"kind\":\"ruler\"}]}",
                       bad, error),
          "a style file with an unknown kind parses as JSON");
    Template no;
    check(!parseTemplate(bad, no, error), "and is refused as a style");
}

// -- a path drawn with the mouse, measured along itself --
//
// The tape to the ruler above. What has to hold: two points give the straight
// line and nothing else; a curve drawn with a hand's worth of jitter reads the
// curve's length and not the jitter's - which is the whole reason it is
// smoothed, because jitter only ever makes an arc longer; and a step that
// wants the span across a drawn curve gets that instead.
void testDrawnPath() {
    std::printf("\nA path drawn with the mouse, measured along itself\n");
    Config cfg;

    // Two points: the straight line, exactly.
    {
        PathResult p;
        check(measurePath({cv::Point2d(20.0, 30.0), cv::Point2d(120.0, 30.0)}, cfg, p),
              "two points measure");
        checkNear(p.arc_mm, 100.0, 1e-6, "as the straight line between them");
        checkNear(p.chord_mm, 100.0, 1e-6, "which is also the chord");
        checkNear(p.bow_mm, 0.0, 1e-6, "with no bow");
    }

    // A neckline's worth of arc, drawn the way a mouse draws one: a point every
    // couple of millimetres, each a screen pixel or so off the curve.
    const Seam seam = scoopNeck();
    Rng rng(2718);
    std::vector<cv::Point2d> drawn;
    const int n = static_cast<int>(seam.trueArc() / 2.0);
    for (int k = 0; k <= n; ++k) {
        cv::Point2d q = seam.at(static_cast<double>(k) / n);
        q.x += rng.signed1() * 0.6;
        q.y += rng.signed1() * 0.6;
        drawn.push_back(q);
    }
    double raw = 0.0;
    for (std::size_t k = 1; k < drawn.size(); ++k) {
        raw += std::hypot(drawn[k].x - drawn[k - 1].x, drawn[k].y - drawn[k - 1].y);
    }
    PathResult p;
    check(measurePath(drawn, cfg, p), "a drawn curve measures");
    std::printf("        drawn %d points: %.1f mm as drawn, %.1f mm smoothed, %.1f mm true\n",
                static_cast<int>(drawn.size()), raw, p.arc_mm, seam.trueArc());
    check(raw > seam.trueArc() + 3.0, "the jitter really does make the raw path read long");
    checkNear(p.arc_mm, seam.trueArc(), 2.0, "and the smoothed path reads the curve, mm");
    checkNear(p.chord_mm, seam.trueChord(), 2.0, "with the chord between its ends");
    checkNear(p.bow_mm, seam.radius * (1.0 - std::cos(seam.sweep / 2.0)), 2.0,
              "and the curve's own bow");

    // Judged: along the curve by default, across it when the step says so.
    Template t;
    t.style = "tape";
    Step s;
    s.name = "neck_line";
    s.kind = "distance";
    s.target_mm = seam.trueArc();
    s.tol_minus_mm = -5.0;
    s.tol_plus_mm = 5.0;
    t.steps.push_back(s);
    StepResult r = judgePath(t, 0, p);
    check(r.measured && r.pass && r.kind == "distance", "a drawn neckline on target passes");
    checkNear(r.measured_mm, p.arc_mm, 1e-9, "judged on the length along the path");
    t.steps[0].measure = "chord";
    t.steps[0].target_mm = seam.trueChord();
    r = judgePath(t, 0, p);
    checkNear(r.measured_mm, p.chord_mm, 1e-9, "or across it, when the step asks for the chord");

    PathResult none;
    measurePath({cv::Point2d(5.0, 5.0), cv::Point2d(5.2, 5.1)}, cfg, none);
    r = judgePath(t, 0, none);
    check(!r.measured && !r.pass, "a drag that went nowhere is refused, not measured");
}

// -- finding the neck in an outline, and choosing among what it holds --
//
// The geometry half of the automatic neckline, on a shirt-shaped outline
// drawn in millimetres with the neck toward the mark: the two corners have to
// be found exactly, the armpits must not be mistaken for the neck, the
// corridors must span every depth a neck can have, and the choosing rule must
// take the band's outer edge, refuse three lines it cannot tell apart, and
// let the target gate but never prefer.
void testNeckFinder() {
    std::printf("\nFinding the neck in an outline\n");

    // A T-shirt, hem at the top, neck toward the mark at the bottom (+y). The
    // neck is a 20 mm dip between corners 180 mm apart; each armpit is a much
    // deeper notch that opens toward the hem, which is the trap.
    std::vector<cv::Point2d> shirt = {
        {60.0, 0.0},    {340.0, 0.0},   {340.0, 470.0}, {440.0, 470.0}, {500.0, 560.0},
        {480.0, 600.0}, {290.0, 600.0}, {250.0, 585.0}, {200.0, 580.0}, {150.0, 585.0},
        {110.0, 600.0}, {-80.0, 600.0}, {-100.0, 560.0}, {-40.0, 470.0}, {60.0, 470.0}};
    // Densified, the way a real outline is.
    std::vector<cv::Point2d> outline;
    for (std::size_t k = 0; k < shirt.size(); ++k) {
        const cv::Point2d& a = shirt[k];
        const cv::Point2d& b = shirt[(k + 1) % shirt.size()];
        const int n = std::max(1, static_cast<int>(std::hypot(b.x - a.x, b.y - a.y) / 2.0));
        for (int i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / n;
            outline.emplace_back(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
        }
    }

    NecklineFind find;
    const bool foundNeck = findNeckline(outline, cv::Point2d(0.0, 1.0), 254.4, find);
    check(foundNeck, "the neck is found: " + find.error);
    if (find.ok) {
        const double left = std::min(find.shoulder_mm[0].x, find.shoulder_mm[1].x);
        const double right = std::max(find.shoulder_mm[0].x, find.shoulder_mm[1].x);
        checkNear(left, 110.0, 1.0, "the left corner is the left shoulder point, mm");
        checkNear(right, 290.0, 1.0, "and the right corner the right, mm");
        checkNear(find.chord_mm, 180.0, 1.0, "180 mm apart");
        checkNear(find.silhouette_depth_mm, 20.0, 1.0, "the outline dips 20 mm between them");
        checkNear(find.expected_depth_mm, 75.1, 0.5,
                  "a 254 mm seam on that chord dips 75, by the arithmetic");
        check(find.candidates.size() >= 5, "and several depths are offered to the picture");
        double shallowest = 1e9, deepest = 0.0;
        for (const std::vector<cv::Point2d>& c : find.candidates) {
            if (c.size() != 3) continue;
            const double d = c[1].y - 600.0;  // into the garment is -y here
            shallowest = std::min(shallowest, -d);
            deepest = std::max(deepest, -d);
        }
        check(shallowest < 30.0 && deepest > 130.0,
              "from a boat neck to a horseshoe, past a semicircle");
        for (const std::vector<cv::Point2d>& c : find.candidates) {
            if (c.size() == 3) check(c[1].y < 600.0, "every arc dips into the garment, away from the mark");
        }
    }

    // The wrong side: with the mark on the hem side there is no neck to find.
    NecklineFind wrong;
    const bool hemSide = findNeckline(outline, cv::Point2d(0.0, -1.0), 254.4, wrong);
    check(!hemSide, "on the hem side there is no neck, and it says so: " + wrong.error);

    // Choosing. Three seams as a collar offers them: the back band's edge, and
    // the two edges of the collar, 19 mm apart.
    const auto seam = [](double depth, double arc) {
        SeamCandidate s;
        s.depth_mm = depth;
        s.curve.ok = true;
        s.curve.arc_mm = arc;
        s.curve.chord_mm = 148.0;
        s.curve.confidence = 0.9;
        return s;
    };
    const std::vector<SeamCandidate> three = {seam(23.6, 158.0), seam(81.0, 247.0), seam(100.3, 289.0)};
    std::size_t chosen = 0;
    std::string why;
    const bool threeChosen = chooseSeam(three, 0.0, true, "outer", chosen, why);
    check(!threeChosen, "three seams and no target is not a choice the machine makes: " + why);
    check(why.find("cannot tell") != std::string::npos, "and it says it cannot tell");

    const std::vector<SeamCandidate> band = {seam(81.0, 247.0), seam(100.3, 289.0)};
    check(chooseSeam(band, 0.0, true, "outer", chosen, why) && chosen == 1,
          "a band's two edges: the outer is the deeper, the larger horseshoe");
    check(chooseSeam(band, 0.0, true, "inner", chosen, why) && chosen == 0,
          "and the inner is the shallower, the opening");

    check(chooseSeam(three, 290.0, true, "outer", chosen, why) && chosen == 2,
          "a target gates: only the seam within a quarter of 290 is left, and it is taken");
    const bool farChosen = chooseSeam(three, 500.0, true, "outer", chosen, why);
    check(!farChosen, "a target nothing is near refuses everything: " + why);
    check(why.find("none within a quarter") != std::string::npos, "and says what it found instead");

    const std::vector<SeamCandidate> one = {seam(81.0, 247.0)};
    check(chooseSeam(one, 0.0, true, "outer", chosen, why) && chosen == 0, "one seam is the answer");
    check(!chooseSeam({}, 0.0, true, "outer", chosen, why), "no seam is not");
}

// -- the style file, the snap and the verdict, end to end --

void testStyleFile(const MmGrid& grid, const Config& cfg, const PlaneModel& model) {
    std::printf("\nA curve step, from the style file to the verdict\n");

    Template t;
    std::string error;
    if (!loadTemplate("templates/innerverse-tester.json", t, error)) {
        // Run from somewhere without the templates folder. Say so and move on
        // rather than failing a check about geometry over a working directory.
        std::printf("        (skipped: %s)\n", error.c_str());
        return;
    }

    int idx = -1;
    for (std::size_t k = 0; k < t.steps.size(); ++k) {
        if (t.steps[k].isCurve()) idx = static_cast<int>(k);
    }
    check(idx >= 0, "the tester style carries a curve step");
    if (idx < 0) return;
    check(t.steps[static_cast<std::size_t>(idx)].measure == "arc",
          "and it says which of the two lengths it wants");

    const Seam seam = scoopNeck();
    Rng rng(5150);
    const cv::Mat frame = renderSeam(grid, seam, true, rng);

    // A garment on target, pointed at badly.
    {
        Template on = t;
        on.steps[static_cast<std::size_t>(idx)].target_mm = seam.trueArc();
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 9.0, 0.0);
        CurveResult r;
        snapCurve(frame, model, seeds, cfg, r);
        const StepResult v = judgeCurve(on, idx, r);
        check(v.measured, "a garment on target reads");
        check(v.pass, "and passes");
        check(v.kind == "curve", "and the log line says it was a curve step");
        checkNear(v.measured_mm, seam.trueArc(), 1.5, "judged on the arc, as the step asked");
        check(v.arc_mm > v.chord_mm, "with both lengths recorded either way");
    }

    // A garment well outside the band still reads, and fails for being the
    // wrong size rather than for being unreadable. The two are different
    // things and an audit trail has to keep them apart.
    {
        Template off = t;
        off.steps[static_cast<std::size_t>(idx)].target_mm = seam.trueArc() - 40.0;
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 9.0, 0.0);
        CurveResult r;
        snapCurve(frame, model, seeds, cfg, r);
        const StepResult v = judgeCurve(off, idx, r);
        check(v.measured, "a garment off target still reads");
        check(!v.pass, "and fails");
    }

    // A seam that could not be read is a FAIL, not a skipped step.
    {
        const cv::Mat blank = renderSeam(grid, seam, false, rng);
        const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 2.0, 0.0);
        CurveResult r;
        snapCurve(blank, model, seeds, cfg, r);
        const StepResult v = judgeCurve(t, idx, r);
        check(!v.measured, "a seam that cannot be read is marked unmeasured");
        check(!v.pass, "and it is a FAIL, not a step the operator may walk past");
    }

    // The step survives a round trip through the file. Targets are edited from
    // the bench and written straight back, so a save that dropped "kind" would
    // turn the neckline step into a folded-edge step at the end of a shift.
    {
        Template again;
        vjson::Value doc = templateToJson(t);
        check(parseTemplate(doc, again, error),
              "the style rewrites and reads back");
        if (!again.steps.empty() && idx < static_cast<int>(again.steps.size())) {
            check(again.steps[static_cast<std::size_t>(idx)].isCurve(),
                  "and the step is still a curve step afterwards");
            check(again.steps[static_cast<std::size_t>(idx)].measure == "arc",
                  "and still measures along the arc");
        }
    }
}

int selftest() {
    std::printf("Traced-curve selftest\n");

    Config cfg;
    const PlaneModelPtr model = benchModel();
    std::printf("  building the millimetre grid for a %d x %d frame\n", kFrameW, kFrameH);
    const MmGrid grid = buildGrid(*model);

    testSeedPath();
    testSnapAccuracy(grid, cfg, *model);
    testRepeatability(grid, cfg, *model);
    testMatMoves(grid, cfg, *model);
    testEndsAreTheOperators(grid, cfg, *model);
    testRealCloth(grid, cfg, *model);
    testPointingChooses(grid, cfg, *model);
    testCloseCoverstitch(grid, cfg, *model);
    testTwinLines(grid, cfg, *model);
    testTracedSeam(grid, cfg, *model);
    testRidgeSeam(grid, cfg, *model);
    testRefusals(grid, cfg, *model);
    testStyleFile(grid, cfg, *model);
    testDistanceStep();
    testDrawnPath();
    testNeckFinder();
    testFinger();
    testTraceGesture();
    testArmTipOnBentWrist();
    testFingerFollowsCurve(grid, cfg, *model);

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

// -- a hand on a real frame --
//
// What the tracker actually sees on THIS bench, with THIS garment, is a
// question about colour and it cannot be answered on gray cloth. A brown hand
// over salmon jersey differs from the cloth by a few degrees of colour angle
// and a third of its brightness, which is inside every threshold the garment
// segmentation uses to ignore a shadow - so the segmentation ignores the hand.
// Composite a hand onto a real held frame, run the real mask, and look.
struct HandProbeResult {
    cv::Mat mask;        // decimated arm mask, as the tracker sees it
    cv::Point2d tipMm;   // where the tracker put the tip
    bool found = false;
    double offMm = 0.0;  // from where the finger actually was
};

// A hand drawn with the shading a real one has: a skin colour, darker at the
// edges, and a soft shadow thrown ahead of it onto the cloth.
void paintHand(cv::Mat& bgr, cv::Point2d fromPx, cv::Point2d tipPx, double pxPerMm,
               cv::Scalar skin, double wristBend) {
    const double vx = tipPx.x - fromPx.x, vy = tipPx.y - fromPx.y;
    const double n = std::hypot(vx, vy);
    if (n < 1e-6) return;
    const double a = std::atan2(vy, vx) + wristBend;
    const cv::Point2d u(std::cos(a), std::sin(a));
    const cv::Point2d base(tipPx.x - 83.5 * pxPerMm * u.x, tipPx.y - 83.5 * pxPerMm * u.y);
    const cv::Point2d wrist(base.x - 95.0 * pxPerMm * u.x, base.y - 95.0 * pxPerMm * u.y);
    const auto at = [](const cv::Point2d& p) {
        return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    };

    // The shadow first: the cloth darkened by 30 percent, uniformly across the
    // channels, in a blurred copy of the hand's own silhouette shifted a little
    // toward the tip. Nothing about it is skin-coloured, which is the point.
    {
        cv::Mat sil(bgr.size(), CV_8U, cv::Scalar(0));
        const cv::Point2d sh(18.0 * pxPerMm * u.x, 18.0 * pxPerMm * u.y);
        const auto shift = [&](const cv::Point2d& p) { return at(cv::Point2d(p.x + sh.x, p.y + sh.y)); };
        cv::line(sil, shift(fromPx), shift(wrist), cv::Scalar(255), static_cast<int>(70.0 * pxPerMm));
        cv::line(sil, shift(wrist), shift(base), cv::Scalar(255), static_cast<int>(90.0 * pxPerMm));
        cv::line(sil, shift(base), shift(tipPx), cv::Scalar(255), static_cast<int>(22.0 * pxPerMm));
        cv::circle(sil, shift(tipPx), static_cast<int>(11.0 * pxPerMm), cv::Scalar(255), -1);
        cv::Mat soft;
        cv::GaussianBlur(sil, soft, cv::Size(0, 0), 12.0 * pxPerMm);
        cv::Mat f;
        soft.convertTo(f, CV_32F, 1.0 / 255.0);
        std::vector<cv::Mat> ch(3);
        cv::split(bgr, ch);
        const float dim[3] = {0.27f, 0.30f, 0.33f};  // B, G, R: shade goes a little blue
        for (int i = 0; i < 3; ++i) {
            cv::Mat cf;
            ch[i].convertTo(cf, CV_32F);
            cf = cf.mul(1.0f - dim[i] * f);
            cf.convertTo(ch[i], CV_8U);
        }
        cv::merge(ch, bgr);
    }

    // Then the skin, in a colour that shades darker toward its edges the way a
    // rounded limb does under one light: drawn as a wider dark stroke with a
    // narrower bright one on top.
    const cv::Scalar dark(skin[0] * 0.65, skin[1] * 0.65, skin[2] * 0.65);
    const auto limb = [&](const cv::Point2d& p, const cv::Point2d& q, double wMm) {
        cv::line(bgr, at(p), at(q), dark, static_cast<int>(std::lround(wMm * pxPerMm)), cv::LINE_AA);
        cv::line(bgr, at(p), at(q), skin, static_cast<int>(std::lround(0.6 * wMm * pxPerMm)),
                 cv::LINE_AA);
    };
    limb(fromPx, wrist, 65.0);
    limb(wrist, base, 85.0);
    limb(base, tipPx, 17.0);
    cv::circle(bgr, at(tipPx), static_cast<int>(std::lround(8.5 * pxPerMm)), dark, -1, cv::LINE_AA);
    cv::circle(bgr, at(tipPx), static_cast<int>(std::lround(5.0 * pxPerMm)), skin, -1, cv::LINE_AA);
}

int handProbe(const std::string& framePath, const std::string& calPath, const std::string& outDir,
              const std::vector<cv::Point2d>& tapsPx) {
    const cv::Mat frame = cv::imread(framePath, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::printf("could not read %s\n", framePath.c_str());
        return 1;
    }
    vjson::Value doc;
    std::string error;
    if (!vjson::parseFile(calPath, doc, error)) {
        std::printf("could not read %s: %s\n", calPath.c_str(), error.c_str());
        return 1;
    }
    const PlaneModelPtr model = PlaneModel::fromJson(doc);
    if (!model) return 1;

    Config cfg;
    const int step = std::max(1, cfg.detect_step);
    RoiBox roi{0, 0, frame.cols, frame.rows};
    const std::pair<double, double> sc =
        model->localMmPerPx(cv::Point2d(0.5 * frame.cols, 0.5 * frame.rows));
    const double pxPerMm = 2.0 / (sc.first + sc.second);

    // Skin as it photographs under this bench's light, BGR.
    const cv::Scalar skins[] = {cv::Scalar(70, 95, 150), cv::Scalar(45, 65, 110),
                                cv::Scalar(120, 150, 205)};
    const char* skinNames[] = {"brown", "dark", "light"};

    // The neckline in this frame, as three places on the seam the finger might
    // be: the left shoulder, the deepest point, the right shoulder. Hand from
    // the bottom-left corner, wrist turned as it would be.
    // Three places on a seam the finger might be. The defaults were the left
    // shoulder point, the deepest point and the right shoulder point of the
    // neckline in the first curve-fail.png this was written against; a
    // different frame wants --seeds-px --seeds "x,y;x,y;x,y".
    cv::Point2d tips[] = {cv::Point2d(2075, 2300), cv::Point2d(2650, 1931),
                          cv::Point2d(3300, 2500)};
    if (tapsPx.size() >= 3) {
        for (int k = 0; k < 3; ++k) tips[k] = tapsPx[static_cast<std::size_t>(k)];
    }
    const cv::Point2d from(-200.0, frame.rows + 300.0);

    for (int s = 0; s < 3; ++s) {
        for (int k = 0; k < 3; ++k) {
            cv::Mat scene = frame.clone();
            paintHand(scene, from, tips[k], pxPerMm, skins[s], 0.6);

            FingerTracker t(cfg);
            t.setReference(frame, roi, true);
            FingerState st;
            for (int f = 0; f < 3; ++f) st = t.update(scene, model.get(), 0.5 * (f + 1));

            const std::vector<cv::Point2d> want = model->pxToMm({tips[k]});
            const double off = st.present ? std::hypot(st.tip_mm.x - want[0].x,
                                                       st.tip_mm.y - want[0].y)
                                          : -1.0;
            std::printf("  skin %-6s tip %d: %s  off %.1f mm  (tip px %.0f,%.0f  want %.0f,%.0f)\n",
                        skinNames[s], k, st.present ? "found" : "NOT FOUND", off, st.tip_px.x,
                        st.tip_px.y, tips[k].x, tips[k].y);

            if (!outDir.empty()) {
                // The mask, the way the tracker computes it.
                const cv::Mat ref = strideSample(frame, step).clone();
                const cv::Mat cur = strideSample(scene, step).clone();
                const double gain = cfg.illumination_normalise ? robustGain(cur, ref) : 1.0;
                cv::Mat mask, score;
                double changed = 0.0;
                Config seg = cfg;
                seg.chroma_angle_threshold = cfg.finger_chroma_angle_threshold;
                seg.intensity_ratio_low = cfg.finger_intensity_ratio_low;
                seg.intensity_ratio_high = cfg.finger_intensity_ratio_high;
                const double mmPerCell = 0.5 * (sc.first + sc.second) * step;
                const int bridge = std::max(3, static_cast<int>(std::lround(cfg.finger_bridge_mm / mmPerCell)));
                segmentObject(cur, ref, seg, gain, bridge,
                              cfg.finger_min_area_fraction * cur.cols * cur.rows * 0.2, mask,
                              changed, score);
                cv::Mat vis;
                cv::cvtColor(mask, vis, cv::COLOR_GRAY2BGR);
                cv::addWeighted(cur, 0.5, vis, 0.5, 0.0, vis);
                if (st.present) {
                    cv::circle(vis, cv::Point(static_cast<int>(st.tip_px.x / step),
                                              static_cast<int>(st.tip_px.y / step)),
                               12, cv::Scalar(0, 255, 0), 2);
                }
                cv::circle(vis, cv::Point(static_cast<int>(tips[k].x / step),
                                          static_cast<int>(tips[k].y / step)),
                           12, cv::Scalar(0, 0, 255), 2);
                cv::imwrite(outDir + "/hand_" + skinNames[s] + "_" + std::to_string(k) + ".png", vis);
                if (s == 0 && k == 1) cv::imwrite(outDir + "/scene_brown_1.png", scene);
            }
        }
    }
    // -- the whole gesture, on the real frame, at the real frame rate --
    //
    // A hand drawn along the seam this frame actually has, at 1.9 frames a
    // second: hold at one shoulder point, draw round to the other, lift. The
    // seam's own line comes from snapping three taps first, so the fingertip
    // follows the real neckline and not a circle somebody imagined.
    {
        std::vector<cv::Point2d> tapsMm = model->pxToMm({tips[0], tips[1], tips[2]});
        CurveResult truth;
        if (!snapCurve(frame, *model, tapsMm, cfg, truth)) {
            std::printf("\n  no seam under the three taps to trace along (%s) - give the probe\n"
                        "  three taps on a seam with --seeds-px --seeds \"x,y;x,y;x,y\"\n",
                        truth.error.c_str());
            return 0;
        }
        std::printf("\n  the seam, from three taps: arc %.1f mm, chord %.1f mm\n", truth.arc_mm,
                    truth.chord_mm);

        const double fps = 1.9;
        const double dt = 1.0 / fps;
        const double drawSeconds = 6.0;
        const int frames = static_cast<int>(std::lround(drawSeconds * fps));
        Rng rng(77);
        for (int s = 0; s < 3; ++s) {
            FingerTracker t(cfg);
            t.setReference(frame, roi, true);
            double now = 0.0;
            const auto along = [&](double u) {
                const std::size_t n = truth.path_mm.size();
                const double f = u * static_cast<double>(n - 1);
                const std::size_t k = std::min(n - 2, static_cast<std::size_t>(f));
                const double w = f - static_cast<double>(k);
                return cv::Point2d(truth.path_mm[k].x * (1.0 - w) + truth.path_mm[k + 1].x * w,
                                   truth.path_mm[k].y * (1.0 - w) + truth.path_mm[k + 1].y * w);
            };
            const auto show = [&](const cv::Point2d& tipMm, double wrist) {
                cv::Mat scene = frame.clone();
                const std::vector<cv::Point2d> px = model->mmToPx({tipMm});
                paintHand(scene, from, px[0], pxPerMm, skins[s], wrist);
                now += dt;
                return t.update(scene, model.get(), now);
            };

            FingerState st;
            bool started = false;
            for (int k = 0; k < 4 && !started; ++k) started = show(along(0.0), 0.3).tracing;
            for (int k = 1; k <= frames && !st.finished; ++k) {
                const double u = static_cast<double>(k) / frames;
                cv::Point2d p = along(u);
                p.x += rng.signed1() * 2.0;  // a hovering hand, two millimetres of it
                p.y += rng.signed1() * 2.0;
                st = show(p, 0.3 + 0.6 * u);  // the wrist turning as it goes round
            }
            int lifted = 0;
            for (int k = 0; k < 4 && !st.finished; ++k) {
                now += dt;
                st = t.update(frame, model.get(), now);
                ++lifted;
            }
            if (!started || !st.finished) {
                std::printf("  skin %-6s: NO TRACE (started %d, finished %d)\n", skinNames[s],
                            started ? 1 : 0, st.finished ? 1 : 0);
                continue;
            }

            double rms = 0.0, worst = 0.0;
            for (const cv::Point2d& p : st.path_mm) {
                double best = 1e9;
                for (const cv::Point2d& q : truth.path_mm) best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
                rms += best * best;
                worst = std::max(worst, best);
            }
            rms = std::sqrt(rms / std::max<std::size_t>(1, st.path_mm.size()));

            CurveResult res;
            const bool ok = snapCurve(frame, *model, st.path_mm, cfg, res);
            std::printf("  skin %-6s: %d points, %.1f mm rms / %.1f worst off the seam, lifted after %d frames -> ",
                        skinNames[s], static_cast<int>(st.path_mm.size()), rms, worst, lifted);
            if (ok) {
                std::printf("arc %.1f mm (three taps said %.1f), conf %.2f, drift %.1f\n",
                            res.arc_mm, truth.arc_mm, res.confidence, res.drift_mm);
            } else {
                std::printf("REFUSED: %s\n", res.error.c_str());
            }

            if (!outDir.empty() && s == 0) {
                // The last frame's eye, in the station's own bundle format, so
                // the replay path is exercised on something real.
                const FingerTracker::Seen& seen = t.lastSeen();
                if (seen.hasTip) {
                    const std::string dir = outDir + "/pointing-debug";
                    std::error_code ec;
                    std::filesystem::create_directories(dir, ec);
                    cv::imwrite(dir + "/reference.png", seen.ref);
                    cv::imwrite(dir + "/hand.png", seen.frame);
                    cv::imwrite(dir + "/mask.png", seen.mask);
                    vjson::Value v = vjson::Value::object();
                    v.set("mm_per_cell", vjson::Value::real(seen.mmPerCell));
                    v.set("step", vjson::Value::integer(seen.step));
                    vjson::Value tp = vjson::Value::array();
                    tp.push(vjson::Value::real(seen.tipSmall.x));
                    tp.push(vjson::Value::real(seen.tipSmall.y));
                    v.set("tip_cells", tp);
                    vjson::writeFile(dir + "/trace.json", v);
                }
                // And the trace as the operator would see it: dots where the
                // tip was found each frame, the fitted curve through them, and
                // the snapped seam.
                cv::Mat over = frame.clone();
                std::vector<cv::Point2d> curve;
                if (seedPath(st.path_mm, 2.0, curve, 0.0, cfg.curve_trace_smooth_mm)) {
                    std::vector<cv::Point> poly;
                    for (const cv::Point2d& p : model->mmToPx(curve)) {
                        poly.emplace_back(static_cast<int>(std::lround(p.x)),
                                          static_cast<int>(std::lround(p.y)));
                    }
                    cv::polylines(over, poly, false, cv::Scalar(0, 0, 0), 10, cv::LINE_AA);
                    cv::polylines(over, poly, false, cv::Scalar(60, 200, 255), 4, cv::LINE_AA);
                }
                if (ok) {
                    std::vector<cv::Point> poly;
                    for (const cv::Point2d& p : model->mmToPx(res.path_mm)) {
                        poly.emplace_back(static_cast<int>(std::lround(p.x)),
                                          static_cast<int>(std::lround(p.y)));
                    }
                    cv::polylines(over, poly, false, cv::Scalar(0, 0, 0), 8, cv::LINE_AA);
                    cv::polylines(over, poly, false, cv::Scalar(80, 255, 80), 3, cv::LINE_AA);
                }
                for (const cv::Point2d& p : model->mmToPx(st.path_mm)) {
                    const cv::Point at(static_cast<int>(std::lround(p.x)),
                                       static_cast<int>(std::lround(p.y)));
                    cv::circle(over, at, 14, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                    cv::circle(over, at, 10, cv::Scalar(60, 200, 255), -1, cv::LINE_AA);
                }
                cv::imwrite(outDir + "/trace_on_frame.png", over);
            }
        }
    }
    return 0;
}


// -- replaying what the tracker saw --
//
// The station writes pointing-debug/ at the end of every trace: the settled
// scene and the last frame with the hand in it, both at the decimation the
// tracker works at, the mask it made of them, and where in that mask it put the
// tip. This runs that frame back through the same locate() the bench ran, with
// this build's settings, and draws the result - so a trace that went wrong on
// a bench can be looked at, and a change to the tracker can be tried against
// it, without a garment or a camera in the room.
int replayPointing(const std::string& dir) {
    const cv::Mat ref = cv::imread(dir + "/reference.png", cv::IMREAD_COLOR);
    const cv::Mat cur = cv::imread(dir + "/hand.png", cv::IMREAD_COLOR);
    vjson::Value doc;
    std::string error;
    if (ref.empty() || cur.empty()) {
        std::printf("could not read %s/reference.png and %s/hand.png\n", dir.c_str(), dir.c_str());
        return 1;
    }
    if (!vjson::parseFile(dir + "/trace.json", doc, error)) {
        std::printf("could not read %s/trace.json: %s\n", dir.c_str(), error.c_str());
        return 1;
    }
    const double mmPerCell = doc.getNumber("mm_per_cell", 0.0);
    const int step = static_cast<int>(doc.getInt("step", 1));

    Config cfg;
    cfg.detect_step = step;
    double lookPx = 0.0, setbackPx = 1e9;
    int bridgePx = 3;
    if (mmPerCell > 1e-9) {
        lookPx = cfg.finger_tip_look_mm / mmPerCell;
        setbackPx = cfg.finger_tip_setback_max_mm / mmPerCell;
        bridgePx = std::max(3, static_cast<int>(std::lround(cfg.finger_bridge_mm / mmPerCell)));
    }
    const double minArea =
        cfg.finger_min_area_fraction * static_cast<double>(cur.cols) * cur.rows;

    cv::Mat mask;
    cv::Point2d tip;
    const bool found = FingerTracker::locate(cur, ref, cfg, lookPx, setbackPx, bridgePx, minArea,
                                             nullptr, mask, tip);

    cv::Point2d then(0.0, 0.0);
    bool haveThen = false;
    if (const vjson::Value* t = doc.find("tip_cells")) {
        if (t->isArray() && t->size() >= 2) {
            then = cv::Point2d(t->at(0).asNumber(), t->at(1).asNumber());
            haveThen = true;
        }
    }

    std::printf("%d x %d cells of %.3f mm, step %d\n", cur.cols, cur.rows, mmPerCell, step);
    if (found) {
        std::printf("this build finds the tip at (%.1f, %.1f) cells", tip.x, tip.y);
        if (haveThen) {
            std::printf("; the bench found (%.1f, %.1f), %.1f mm away", then.x, then.y,
                        std::hypot(tip.x - then.x, tip.y - then.y) * mmPerCell);
        }
        std::printf("\n");
    } else {
        std::printf("this build finds no arm in the frame at all\n");
    }
    if (const vjson::Value* tr = doc.find("trace_mm")) {
        std::printf("the trace had %d points\n", static_cast<int>(tr->size()));
    }

    cv::Mat vis;
    if (!mask.empty()) {
        cv::cvtColor(mask, vis, cv::COLOR_GRAY2BGR);
        cv::addWeighted(cur, 0.55, vis, 0.45, 0.0, vis);
    } else {
        vis = cur.clone();
    }
    const auto at = [](const cv::Point2d& p) {
        return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    };
    if (haveThen) cv::circle(vis, at(then), 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    if (found) cv::circle(vis, at(tip), 12, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    const std::string out = dir + "/replay.png";
    if (cv::imwrite(out, vis)) {
        std::printf("drew %s: the mask over the frame, green where this build puts the tip, "
                    "red where the bench did\n",
                    out.c_str());
    }
    return found ? 0 : 1;
}

// -- finding the neckline on a real frame --
//
// The station finds the neckline from the refined outline of a held reading.
// Offline there is no reading and no empty-mat reference to difference
// against, so this takes the garment's outline the crude way - pale cloth
// against a dark mat by brightness - which is good enough to put the finder's
// notch and shoulder points on a real shirt and see where they land, and to
// hand snapCurve what the station would.
int neckProbe(const std::string& framePath, const std::string& calPath, const std::string& towardName,
              double targetArc, const std::string& drawPath) {
    const cv::Mat frame = cv::imread(framePath, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::printf("could not read %s\n", framePath.c_str());
        return 1;
    }
    vjson::Value doc;
    std::string error;
    if (!vjson::parseFile(calPath, doc, error)) {
        std::printf("could not read %s: %s\n", calPath.c_str(), error.c_str());
        return 1;
    }
    const PlaneModelPtr model = PlaneModel::fromJson(doc);
    if (!model) return 1;

    // The garment against the mat, by brightness. The grid's lines are bright
    // too but thin, and an opening at the width of a fingertip takes them out.
    cv::Mat gray, mask;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(25, 25));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) {
        std::printf("no garment in the frame\n");
        return 1;
    }
    std::size_t big = 0;
    for (std::size_t i = 1; i < contours.size(); ++i) {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[big])) big = i;
    }
    std::vector<cv::Point2d> outlinePx;
    for (std::size_t i = 0; i < contours[big].size(); i += 6) {
        outlinePx.emplace_back(contours[big][i].x, contours[big][i].y);
    }
    const std::vector<cv::Point2d> outlineMm = model->pxToMm(outlinePx);
    std::printf("outline: %d points\n", static_cast<int>(outlineMm.size()));

    cv::Point2d toward(0.0, 1.0);
    if (towardName == "top") toward = cv::Point2d(0.0, -1.0);
    else if (towardName == "left") toward = cv::Point2d(-1.0, 0.0);
    else if (towardName == "right") toward = cv::Point2d(1.0, 0.0);

    NecklineFind find;
    const bool found = findNeckline(outlineMm, toward, targetArc, find);
    if (!found) {
        std::printf("not found: %s\n", find.error.c_str());
    } else {
        std::printf("shoulder points (%.1f, %.1f) and (%.1f, %.1f) mm, %.1f mm apart\n",
                    find.shoulder_mm[0].x, find.shoulder_mm[0].y, find.shoulder_mm[1].x,
                    find.shoulder_mm[1].y, find.chord_mm);
        std::printf("the outline dips %.1f mm between them; the target says %.1f; corridor along the %s (%d seeds)\n",
                    find.silhouette_depth_mm, find.expected_depth_mm, find.how.c_str(),
                    static_cast<int>(find.seeds_mm.size()));
    }

    CurveResult curve;
    bool snapped = false;
    std::vector<cv::Point2d> used;
    std::vector<SeamCandidate> seams;
    if (found) {
        Config cfg;
        seams = findSeams(frame, *model, find, cfg);
        std::printf("%d distinct seams between the corners, from %d corridors:\n",
                    static_cast<int>(seams.size()), static_cast<int>(find.candidates.size()));
        for (const SeamCandidate& s : seams) {
            std::printf("  %5.1f mm deep: arc %.1f  chord %.1f  radius %.1f  conf %.2f  margin %.3f  drift %.1f\n",
                        s.depth_mm, s.curve.arc_mm, s.curve.chord_mm, s.curve.radius_mm,
                        s.curve.confidence, s.curve.margin, s.curve.drift_mm);
        }
        std::size_t chosen = 0;
        std::string why;
        if (chooseSeam(seams, targetArc, true, "outer", chosen, why)) {
            snapped = true;
            curve = seams[chosen].curve;
            used = seams[chosen].seeds_mm;
            std::printf("chosen: %.1f mm deep, arc %.2f mm, chord %.2f mm%s%s\n",
                        seams[chosen].depth_mm, curve.arc_mm, curve.chord_mm,
                        why.empty() ? "" : " - ", why.c_str());
        } else {
            std::printf("not chosen: %s\n", why.c_str());
        }
    }

    if (!drawPath.empty()) {
        cv::Mat over = frame.clone();
        const auto at = [](const cv::Point2d& p) {
            return cv::Point(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
        };
        std::vector<cv::Point> poly;
        for (const cv::Point2d& p : outlinePx) poly.push_back(at(p));
        cv::polylines(over, poly, true, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        if (found) {
            for (const cv::Point2d& p : model->mmToPx(snapped ? used : find.seeds_mm)) {
                cv::circle(over, at(p), 8, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                cv::circle(over, at(p), 6, cv::Scalar(60, 160, 255), -1, cv::LINE_AA);
            }
            for (const cv::Point2d& s : model->mmToPx({find.shoulder_mm[0], find.shoulder_mm[1]})) {
                cv::circle(over, at(s), 22, cv::Scalar(0, 0, 0), 6, cv::LINE_AA);
                cv::circle(over, at(s), 22, cv::Scalar(60, 160, 255), 3, cv::LINE_AA);
            }
        }
        // Every seam found, thin; the chosen one thick and green.
        for (const SeamCandidate& s : seams) {
            std::vector<cv::Point> line;
            for (const cv::Point2d& p : model->mmToPx(s.curve.path_mm)) line.push_back(at(p));
            cv::polylines(over, line, false, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
            cv::polylines(over, line, false, cv::Scalar(255, 200, 80), 2, cv::LINE_AA);
        }
        if (snapped) {
            std::vector<cv::Point> seam;
            for (const cv::Point2d& p : model->mmToPx(curve.path_mm)) seam.push_back(at(p));
            cv::polylines(over, seam, false, cv::Scalar(0, 0, 0), 9, cv::LINE_AA);
            cv::polylines(over, seam, false, cv::Scalar(80, 255, 80), 4, cv::LINE_AA);
        }
        if (cv::imwrite(drawPath, over)) std::printf("drew %s\n", drawPath.c_str());
    }
    return found && snapped ? 0 : 1;
}

// -- reading a real capture --

bool parseSeeds(const std::string& text, std::vector<cv::Point2d>& out) {
    std::size_t i = 0;
    while (i < text.size()) {
        const std::size_t semi = text.find(';', i);
        const std::string one = text.substr(i, semi == std::string::npos ? std::string::npos
                                                                        : semi - i);
        const std::size_t comma = one.find(',');
        if (comma == std::string::npos) return false;
        out.emplace_back(std::atof(one.substr(0, comma).c_str()),
                         std::atof(one.substr(comma + 1).c_str()));
        if (semi == std::string::npos) break;
        i = semi + 1;
    }
    return out.size() >= 2;
}

// Write out a synthetic capture and the calibration that goes with it, so the
// --frame path can be exercised without a camera and so a bench being
// commissioned has a known-good pair to compare its own captures against.
int writeSample(const std::string& dir) {
    const PlaneModelPtr model = benchModel();
    const MmGrid grid = buildGrid(*model);
    const Seam seam = scoopNeck();
    Rng rng(20260909);
    const cv::Mat frame = renderSeam(grid, seam, true, rng);

    const std::string img = dir + "/sample-neckline.png";
    const std::string cal = dir + "/sample-calibration.json";
    if (!cv::imwrite(img, frame)) {
        std::printf("could not write %s\n", img.c_str());
        return 1;
    }
    if (!vjson::writeFile(cal, model->toJson(), 2)) {
        std::printf("could not write %s\n", cal.c_str());
        return 1;
    }

    // Points as sloppy as a hand makes them, so the sample exercises the snap
    // rather than handing it the answer.
    const std::vector<cv::Point2d> seeds = pointAt(seam, rng, 9.0, 0.0);
    std::printf("wrote %s and %s\n", img.c_str(), cal.c_str());
    std::printf("true chord %.2f mm, true arc %.2f mm\n", seam.trueChord(), seam.trueArc());
    std::printf("curve_check --frame %s --calibration %s --seeds \"", img.c_str(), cal.c_str());
    for (std::size_t k = 0; k < seeds.size(); ++k) {
        std::printf("%s%.1f,%.1f", k ? ";" : "", seeds[k].x, seeds[k].y);
    }
    std::printf("\"\n");
    return 0;
}

void usage() {
    std::printf(
        "Traced-curve harness.\n"
        "\n"
        "  curve_check --selftest\n"
        "  curve_check --frame lay.png --seeds \"x,y;x,y;x,y\" [--calibration calibration.json]\n"
        "                                        [--seeds-px] [--draw out.png]\n"
        "  curve_check --ruler --frame lay.png --seeds-px --seeds \"x,y;x,y\"\n"
        "  curve_check --pointing pointing-debug\n"
        "  curve_check --hand-probe --frame lay.png [--out DIR]\n"
        "  curve_check --write-sample DIR\n"
        "\n"
        "--seeds are millimetres on the mat, in the order they were pointed at, or\n"
        "pixels of the frame with --seeds-px. Two describe the ends of a straight\n"
        "seam; three describe a curve, which is what a neckline needs. --draw puts\n"
        "the snapped seam back on the picture, which is the only way to know it\n"
        "took the seam and not the topstitch beside it.\n"
        "\n"
        "--ruler is the station's tap-and-drag ruler, offline: the same points to\n"
        "millimetres, nothing else in the way. Two crossings of the mat's grid\n"
        "should read what the grid says; when they do not, recalibrate.\n"
        "--pointing replays what the finger tracker saw at the end of a trace.\n"
        "--hand-probe paints a hand onto a real frame and asks where the tip is.\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string framePath, seedText, calPath = "calibration.json", samplePath;
    bool wantSelftest = false;
    // Seeds read off the saved frame with a picture viewer are in pixels, and
    // turning them into millimetres by hand needs the distortion model as well
    // as the scale. The calibration is already open here; let it do it.
    bool seedsArePixels = false;
    // Which line it took is not a number, and the numbers cannot say it. A
    // neckline carries four lines within a centimetre and the arc lengths they
    // give differ by millimetres, all of them plausible; the only way to know
    // the seam was measured rather than the topstitch beside it is to look.
    std::string drawPath;
    std::string handProbeDir;
    bool wantHandProbe = false;
    std::string pointingDir;
    // The ruler, offline: the same points-to-millimetres the station does when
    // the operator taps or drags, with nothing else in the way. Two points on
    // the mat's own grid should read what the grid says, which is the check to
    // run when a reading disagrees with a tape.
    bool wantRuler = false;
    bool wantNeck = false;
    std::string towardName = "bottom";
    double targetArc = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--selftest") {
            wantSelftest = true;
        } else if (a == "--frame" && i + 1 < argc) {
            framePath = argv[++i];
        } else if (a == "--seeds" && i + 1 < argc) {
            seedText = argv[++i];
        } else if (a == "--calibration" && i + 1 < argc) {
            calPath = argv[++i];
        } else if (a == "--seeds-px") {
            seedsArePixels = true;
        } else if (a == "--draw" && i + 1 < argc) {
            drawPath = argv[++i];
        } else if (a == "--hand-probe") {
            wantHandProbe = true;
        } else if (a == "--ruler") {
            wantRuler = true;
        } else if (a == "--neck") {
            wantNeck = true;
        } else if (a == "--toward" && i + 1 < argc) {
            towardName = argv[++i];
        } else if (a == "--target" && i + 1 < argc) {
            targetArc = std::atof(argv[++i]);
        } else if (a == "--pointing" && i + 1 < argc) {
            pointingDir = argv[++i];
        } else if (a == "--out" && i + 1 < argc) {
            handProbeDir = argv[++i];
        } else if (a == "--write-sample" && i + 1 < argc) {
            samplePath = argv[++i];
        } else {
            usage();
            return 2;
        }
    }

    if (!samplePath.empty()) return writeSample(samplePath);
    if (wantSelftest) return selftest();
    if (wantHandProbe && !framePath.empty()) {
        std::vector<cv::Point2d> taps;
        if (!seedText.empty() && seedsArePixels) parseSeeds(seedText, taps);
        return handProbe(framePath, calPath, handProbeDir, taps);
    }
    if (!pointingDir.empty()) return replayPointing(pointingDir);
    if (wantNeck && !framePath.empty()) {
        return neckProbe(framePath, calPath, towardName, targetArc, drawPath);
    }
    if (framePath.empty() || seedText.empty()) {
        usage();
        return 2;
    }

    const cv::Mat frame = cv::imread(framePath, cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::printf("could not read %s\n", framePath.c_str());
        return 1;
    }

    vjson::Value doc;
    std::string error;
    if (!vjson::parseFile(calPath, doc, error)) {
        std::printf("could not read %s: %s\n", calPath.c_str(), error.c_str());
        return 1;
    }
    const PlaneModelPtr model = PlaneModel::fromJson(doc);
    if (!model) {
        std::printf("%s is not a calibration this build understands\n", calPath.c_str());
        return 1;
    }

    std::vector<cv::Point2d> seeds;
    if (!parseSeeds(seedText, seeds)) {
        std::printf("could not read the seeds\n");
        return 1;
    }

    if (seedsArePixels) seeds = model->pxToMm(seeds);

    if (wantRuler) {
        std::printf("points (mm):");
        for (const cv::Point2d& p : seeds) std::printf(" %.2f,%.2f", p.x, p.y);
        std::printf("\n");
        double along = 0.0;
        for (std::size_t k = 1; k < seeds.size(); ++k) {
            along += std::hypot(seeds[k].x - seeds[k - 1].x, seeds[k].y - seeds[k - 1].y);
        }
        const double chord =
            std::hypot(seeds.back().x - seeds.front().x, seeds.back().y - seeds.front().y);
        std::printf("straight between the ends: %.2f mm\n", chord);
        if (seeds.size() > 2) std::printf("along the %d points as given: %.2f mm\n",
                                          static_cast<int>(seeds.size()), along);
        Config cfg;
        PathResult path;
        if (measurePath(seeds, cfg, path)) {
            std::printf("as the station measures a drag: %.2f mm along, %.2f mm across, bow %.2f mm\n",
                        path.arc_mm, path.chord_mm, path.bow_mm);
        } else {
            std::printf("%s\n", path.error.c_str());
        }
        return 0;
    }

    Config cfg;
    CurveResult r;
    snapCurve(frame, *model, seeds, cfg, r);
    std::printf("seeds (mm):");
    for (const cv::Point2d& p : seeds) std::printf(" %.1f,%.1f", p.x, p.y);
    std::printf("\n");
    std::printf("%s\n", r.toJson().dump(2).c_str());

    if (!drawPath.empty()) {
        cv::Mat over = frame.clone();
        const std::vector<cv::Point2d> seedPx = model->mmToPx(seeds);
        std::vector<cv::Point> poly;
        for (const cv::Point2d& p : model->mmToPx(r.path_mm)) {
            poly.emplace_back(static_cast<int>(std::lround(p.x)),
                              static_cast<int>(std::lround(p.y)));
        }
        if (poly.size() >= 2) {
            cv::polylines(over, poly, false, cv::Scalar(0, 0, 0), 9, cv::LINE_AA);
            cv::polylines(over, poly, false, cv::Scalar(80, 255, 80), 4, cv::LINE_AA);
        }
        for (const cv::Point2d& p : seedPx) {
            const cv::Point at(static_cast<int>(std::lround(p.x)),
                               static_cast<int>(std::lround(p.y)));
            cv::circle(over, at, 16, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
            cv::circle(over, at, 16, cv::Scalar(60, 160, 255), 3, cv::LINE_AA);
        }
        if (cv::imwrite(drawPath, over)) {
            std::printf("drew the reading on %s\n", drawPath.c_str());
        } else {
            std::printf("could not write %s\n", drawPath.c_str());
        }
    }
    return r.ok ? 0 : 1;
}
