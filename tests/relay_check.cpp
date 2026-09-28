// relay_check: laying the garment again for the next step without clearing the
// mat, with no camera and no garment.
//
//   relay_check --selftest [--verbose]
//
// The engine holds a reading until the mat is cleared. A step still waiting for
// its reading asks it for the next lay instead (MeasurementEngine::awaitNewLay),
// so a shirt refolded where it lies is measured without being lifted clear
// first. The selftest renders folds of a shirt onto a mat, puts every frame
// through the real engine, and checks what that has to guarantee:
//
//   * a different lay put down where the last one lay is measured, with no
//     clear mat in between: refolded, or a fold opened out where it lies;
//   * the lay the held reading came from is never measured a second time -
//     left untouched, a hand resting on it, an arm swept across it, a hand
//     lying apart from it, a fold only straightened where it lies - because
//     measured again it would be judged against a step it was not laid for;
//   * except on a retake, where straightening the fold where it lies is the
//     retake and is read;
//   * a reading let go for a new lay is held again when the station stops
//     asking, and a clear mat still starts over as it always did;
//   * not asked, the engine holds exactly as it always has.

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/cv_compat.hpp"
#include "verimeasure/engine.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/plane_model.hpp"

using namespace verimeasure;

namespace {

int g_checks = 0;
int g_failures = 0;
bool g_verbose = false;

void check(bool ok, const std::string& what, const std::string& detail = std::string()) {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s%s%s\n", ok ? " ok " : "FAIL", what.c_str(), detail.empty() ? "" : ": ",
                detail.c_str());
}

// -- the bench --
//
// Looking straight down at 0.274 mm a pixel, twice the rig's, over a 900 x 600
// mm view. Detection runs every third pixel here, so a decimated cell is the
// rig's 0.82 mm and the lays are compared on the same grid they are on the
// bench, from frames a quarter of the size.
struct Bench {
    double mmpp = 0.274;
    cv::Size size{3285, 2190};
    RoiBox roi{8, 8, 3277, 2182};
    PlaneModelPtr model;
    cv::Mat mat;
    std::vector<cv::Mat> noise;
};

using Poly = std::vector<cv::Point2d>;  // millimetres on the mat

Bench makeBench() {
    Bench b;
    const cv::Matx33d H(1.0 / b.mmpp, 0.0, 0.0, 0.0, 1.0 / b.mmpp, 0.0, 0.0, 0.0, 1.0);
    b.model = std::make_shared<PlaneModel>(H, 0.0, 0.0, cv::Point2d(b.size.width / 2.0, b.size.height / 2.0),
                                           b.size.width / 2.0, 1.0);
    // A dark mat with its grid printed bright every ten millimetres, which is
    // this mat's polarity.
    b.mat = cv::Mat(b.size, CV_8UC3, cv::Scalar(52, 58, 30));
    const cv::Scalar line(170, 182, 176);
    for (double x = 5.0; x < b.size.width * b.mmpp; x += 10.0) {
        const int px = static_cast<int>(std::lround(x / b.mmpp));
        cv::line(b.mat, cv::Point(px, 0), cv::Point(px, b.size.height - 1), line, 1);
    }
    for (double y = 5.0; y < b.size.height * b.mmpp; y += 10.0) {
        const int py = static_cast<int>(std::lround(y / b.mmpp));
        cv::line(b.mat, cv::Point(0, py), cv::Point(b.size.width - 1, py), line, 1);
    }
    // Sensor noise, a different field each frame, so an untouched lay flickers
    // at its boundary the way a real one does.
    cv::RNG rng(20260923);
    for (int k = 0; k < 5; ++k) {
        cv::Mat n(b.size, CV_16SC3);
        rng.fill(n, cv::RNG::NORMAL, 0.0, 2.0);
        b.noise.push_back(n);
    }
    return b;
}

double polyArea(const Poly& p) {
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const cv::Point2d& u = p[i];
        const cv::Point2d& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return std::abs(a) / 2.0;
}

Poly shifted(const Poly& p, double dx, double dy) {
    Poly out = p;
    for (cv::Point2d& q : out) q += cv::Point2d(dx, dy);
    return out;
}

struct Layer {
    Poly poly;
    cv::Scalar colour;
};

const cv::Scalar kCloth(214, 222, 228);
const cv::Scalar kSkin(120, 150, 200);

cv::Mat photograph(const Bench& b, const std::vector<Layer>& layers, long long seq) {
    cv::Mat f = b.mat.clone();
    for (const Layer& l : layers) {
        std::vector<cv::Point> px;
        for (const cv::Point2d& q : l.poly) {
            px.emplace_back(static_cast<int>(std::lround(q.x / b.mmpp * 8.0)),
                            static_cast<int>(std::lround(q.y / b.mmpp * 8.0)));
        }
        cv::fillPoly(f, std::vector<std::vector<cv::Point>>{px}, l.colour, cv::LINE_AA, 3);
    }
    cv::Mat out;
    cv::add(f, b.noise[static_cast<std::size_t>(seq % static_cast<long long>(b.noise.size()))], out,
            cv::noArray(), CV_8UC3);
    return out;
}

// -- the folds --
//
// With the mark along the bottom of the view, as on the rig. Each is a shirt
// folded for one step, the way the operator refolds it where it lies.

// A long fold along the rail, as for a body length.
const Poly kBodyFold = {{120, 570}, {700, 570}, {700, 330}, {640, 300}, {200, 300}, {120, 330}};
// The shirt refolded for the chest, a little to the right and taller.
const Poly kChestFold = {{180, 570}, {680, 570}, {690, 290}, {170, 290}};
// The same fold with its right-hand side straightened 12 mm in, where it lies.
const Poly kChestStraightened = {{180, 570}, {668, 570}, {678, 290}, {170, 290}};
// The chest fold opened out where it lies: nothing uncovered, 85000 mm2 more.
const Poly kOpenedOut = {{180, 570}, {680, 570}, {690, 290}, {680, 120}, {180, 120}, {170, 290}};

// A hand resting on the body fold, reaching in from the bottom of the view.
const Poly kHandOnFold = {{650, 600}, {740, 600}, {720, 450}, {680, 420}, {640, 450}};
// A hand on the mat well away from any cloth.
const Poly kHandApart = {{790, 600}, {880, 600}, {870, 470}, {830, 450}, {790, 470}};
// An arm reaching across the empty mat above the fold to its top edge: more
// area than the addition test lets an arm be.
Poly armAcross(double dx) {
    return {{0, 150 + dx * 0.2}, {480 + dx, 170}, {520 + dx, 312}, {490 + dx, 330}, {0, 320 + dx * 0.2}};
}

// -- driving the engine --

struct Drive {
    const Bench& b;
    MeasurementEngine& e;
    long long seq = 0;

    // Frames of one picture. Returns the readings they produced, and whether
    // the engine held its reading through every one of them.
    struct Out {
        std::vector<MeasureResult> results;
        bool heldThroughout = true;
        bool leftHold = false;
    };
    Out frames(const std::vector<Layer>& layers, int n) {
        Out o;
        for (int k = 0; k < n; ++k) {
            ++seq;
            const std::optional<MeasureResult> r = e.process(photograph(b, layers, seq), seq);
            if (r.has_value()) o.results.push_back(*r);
            const bool held = e.state() == STATE_MEASURED;
            if (!held) {
                o.heldThroughout = false;
                o.leftHold = true;
            }
            if (g_verbose) {
                std::printf("      frame %lld: %s%s\n", seq, e.state().c_str(), r.has_value() ? "  READING" : "");
            }
        }
        return o;
    }
    // Frames of a sequence of pictures, one each.
    Out sequence(const std::vector<std::vector<Layer>>& pictures) {
        Out o;
        for (const std::vector<Layer>& p : pictures) {
            Out one = frames(p, 1);
            o.results.insert(o.results.end(), one.results.begin(), one.results.end());
            o.heldThroughout = o.heldThroughout && one.heldThroughout;
            o.leftHold = o.leftHold || one.leftHold;
        }
        return o;
    }
};

std::string areaText(double a) { return vjson::formatFixed(a, 0) + " mm2"; }

// The reading is of the lay it should be of: the areas agree within 2%.
bool isOf(const MeasureResult& r, const Poly& p) {
    return std::abs(r.area_mm2 - polyArea(p)) < 0.02 * polyArea(p);
}

std::string readingText(const Drive::Out& o) {
    if (o.results.empty()) return "no reading";
    return std::to_string(o.results.size()) + " reading(s), last " + areaText(o.results.back().area_mm2);
}

void selftest() {
    const Bench b = makeBench();
    Config cfg;
    cfg.detect_step = 3;
    MeasurementEngine e(cfg, b.model, b.roi, true);
    e.setBackground(b.mat);
    Drive d{b, e};

    const std::vector<Layer> clear;
    const std::vector<Layer> body{{kBodyFold, kCloth}};
    const std::vector<Layer> chest{{kChestFold, kCloth}};
    const std::vector<Layer> chestStraight{{kChestStraightened, kCloth}};
    const std::vector<Layer> opened{{kOpenedOut, kCloth}};

    std::printf("\nlaid on a clear mat\n");
    d.frames(clear, 3);
    Drive::Out o = d.frames(body, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kBodyFold), "the body fold is measured",
          readingText(o) + ", drawn " + areaText(polyArea(kBodyFold)));
    check(e.state() == STATE_MEASURED, "and held", e.state());

    std::printf("\nnot asked for a new lay\n");
    o = d.frames(chest, 16);
    check(o.results.empty() && o.heldThroughout, "refolded without the mat cleared: held, as it always was",
          readingText(o));
    check(!e.awaitingNewLay(), "and nothing is waited for");
    o = d.frames(clear, 4);
    check(e.state() == STATE_WAITING, "a clear mat lets go", e.state());
    o = d.frames(body, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kBodyFold), "the body fold is measured again",
          readingText(o));

    std::printf("\nasked for the next lay: the body fold must not be read again\n");
    e.awaitNewLay(false);
    check(e.awaitingNewLay(), "waiting for the next lay");
    o = d.frames(body, 16);
    check(o.results.empty() && o.heldThroughout, "left untouched: held throughout", readingText(o));
    o = d.frames({{kBodyFold, kCloth}, {kHandOnFold, kSkin}}, 12);
    check(o.results.empty() && o.heldThroughout, "a hand resting on the fold: held throughout", readingText(o));
    o = d.frames({{kBodyFold, kCloth}, {kHandApart, kSkin}}, 12);
    check(o.results.empty() && o.heldThroughout, "a hand on the mat apart from it: held throughout",
          readingText(o));
    {
        std::vector<std::vector<Layer>> sweep;
        for (int k = 0; k < 6; ++k) sweep.push_back({{kBodyFold, kCloth}, {armAcross(25.0 * k), kSkin}});
        o = d.sequence(sweep);
        const bool dropped = o.leftHold;
        Drive::Out after = d.frames(body, 14);
        check(o.results.empty() && after.results.empty(), "an arm swept across it and gone: nothing read",
              readingText(o) + ", then " + readingText(after));
        check(dropped, "(the arm was large enough to be looked at)", dropped ? "let go" : "never let go");
        check(e.state() == STATE_MEASURED && e.awaitingNewLay(),
              "the body fold's reading held again, still waiting for the next lay", e.state());
    }

    std::printf("\nrefolded where it lies\n");
    o = d.frames(chest, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kChestFold), "refolded for the chest: measured",
          readingText(o) + ", drawn " + areaText(polyArea(kChestFold)));
    check(e.state() == STATE_MEASURED && !e.awaitingNewLay(), "held, and no longer waiting", e.state());

    std::printf("\nstraightened where it lies\n");
    e.awaitNewLay(false);
    o = d.frames(chestStraight, 16);
    check(o.results.empty() && o.heldThroughout, "for a new step: not a new lay, held", readingText(o));
    e.keepHeldLay();
    o = d.frames(chest, 6);
    e.awaitNewLay(true);
    o = d.frames(chest, 16);
    check(o.results.empty() && o.heldThroughout, "retaken and left untouched: held throughout", readingText(o));
    o = d.frames(chestStraight, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kChestStraightened),
          "retaken and straightened: measured", readingText(o));

    std::printf("\nopened out where it lies\n");
    e.awaitNewLay(false);
    o = d.frames(opened, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kOpenedOut), "nothing uncovered, a fold opened out: measured",
          readingText(o) + ", drawn " + areaText(polyArea(kOpenedOut)));

    std::printf("\nlet go, then no longer asked\n");
    e.awaitNewLay(false);
    o = d.frames(chest, 3);
    check(o.leftHold, "refolded: the reading is let go", e.state());
    e.keepHeldLay();
    check(e.state() == STATE_MEASURED && !e.awaitingNewLay(), "not asked any more: held again at once", e.state());
    o = d.frames(chest, 14);
    check(o.results.empty() && o.heldThroughout, "and nothing read however long it lies", readingText(o));

    std::printf("\ncleared while waiting\n");
    e.awaitNewLay(false);
    d.frames(clear, 4);
    check(e.state() == STATE_WAITING && !e.awaitingNewLay(), "a clear mat starts over as it always did", e.state());
    o = d.frames(body, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), kBodyFold), "and the next lay is measured",
          readingText(o));

    std::printf("\nmoved along the mat\n");
    e.awaitNewLay(false);
    o = d.frames({{shifted(kBodyFold, 0.0, -40.0), kCloth}}, 14);
    check(o.results.size() == 1 && isOf(o.results.back(), shifted(kBodyFold, 0.0, -40.0)),
          "the same fold slid 40 mm off the rail: measured where it lies now", readingText(o));
}

}  // namespace

int main(int argc, char** argv) {
    bool selftestWanted = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--selftest") {
            selftestWanted = true;
        } else if (a == "--verbose") {
            g_verbose = true;
        } else {
            selftestWanted = false;
            break;
        }
    }
    if (!selftestWanted) {
        std::printf("usage: relay_check --selftest [--verbose]\n");
        return 2;
    }
    selftest();
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
