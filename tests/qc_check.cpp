/*
Step QC harness. Measures the folded edge and scores it against a style, with
no camera attached.

    qc_check --selftest
    qc_check --outline lay.json --template templates/style-42-polo-m.json --step 1

--selftest builds folded lays whose edge length is known by construction and
checks the answer against it. Three things are worth checking and all three are
here.

The length has to be right: a fold 492 mm long reads 492 mm, corner to corner.

It has to be the edge facing the machine, not whichever edge of the garment
happens to be straightest. A lay with a longer straight edge further in must
still return the near one, because that is the whole premise: the operator
presents the fold to a fixed side and nothing else needs interpreting.

It has to survive the mat moving. Every lay is measured, then turned and shifted
somewhere else entirely along with the scan direction, and the answer must not
change.

An outline file is JSON:  { "outline_mm": [[x, y], [x, y], ...] }
*/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/edge.hpp"
#include "verimeasure/json.hpp"
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

// A folded lay presented to the machine looking in along +x.
//
// The fold is the straight edge at x = 0, running edgeMm from corner to corner.
// Behind it the cloth falls away in a curve, which is what a garment actually
// presents: a hem and an armhole curve, they do not run in straight lines.
//
// bowMm bends the fold; the reading should not be trusted past a point and the
// straightness number is what says so.
std::vector<cv::Point2d> buildLay(double edgeMm, double depthMm, double bowMm = 0.0) {
    std::vector<cv::Point2d> pts;
    const int edgeSamples = static_cast<int>(edgeMm / 0.4);
    const int arcSamples = static_cast<int>(2.2 * depthMm / 0.4);

    for (int k = 0; k <= edgeSamples; ++k) {
        const double t = static_cast<double>(k) / edgeSamples;
        pts.push_back(cv::Point2d(bowMm * std::sin(kPi * t), t * edgeMm));
    }
    for (int k = 1; k < arcSamples; ++k) {
        const double u = static_cast<double>(k) / arcSamples;
        pts.push_back(cv::Point2d(depthMm * std::sin(kPi * u), edgeMm * (1.0 - u)));
    }
    return pts;
}

// A lay whose presented edge is an arc rather than a line, which is what a
// garment actually is. A hem sweeps, a shoulder slopes, a chest line crosses a
// body wider at the bottom than the top; on real shirts that is 13 to 15 mm of
// sweep across half a metre of edge.
//
// The measurement is the chord - what a tape laid flat between the two seams
// reads - and it is chordMm by construction here however hard the edge sweeps,
// which is the whole point of the fixture. A positive sagitta bulges toward the
// machine, a negative one away from it; neither is allowed to change the
// answer.
std::vector<cv::Point2d> buildCurvedLay(double chordMm, double sagittaMm, double depthMm) {
    std::vector<cv::Point2d> pts;
    const int edgeSamples = static_cast<int>(chordMm / 0.4);
    const int arcSamples = static_cast<int>(2.2 * depthMm / 0.4);

    for (int k = 0; k <= edgeSamples; ++k) {
        const double t = static_cast<double>(k) / edgeSamples;
        pts.push_back(cv::Point2d(-sagittaMm * 4.0 * t * (1.0 - t), t * chordMm));
    }
    for (int k = 1; k < arcSamples; ++k) {
        const double u = static_cast<double>(k) / arcSamples;
        pts.push_back(cv::Point2d(depthMm * std::sin(kPi * u), chordMm * (1.0 - u)));
    }
    return pts;
}

// The whole shape a folded shirt presents to the rail: an edge that sweeps,
// corners rounded rather than square, and a boundary carrying the noise the
// sub-pixel refinement leaves behind.
//
// The measurement is the distance between the two corners the seams would have
// met at had the cloth turned a square corner. That is what a tape laid flat
// between two seams reads, and it is chordMm here by construction however round
// the corners are - which is the point: nothing about the rounding is allowed
// to shorten the answer.
std::vector<cv::Point2d> buildGarmentLay(double chordMm, double sagittaMm, double radiusMm,
                                         double noiseMm, unsigned seed = 20260907u) {
    const double kStep = 0.4;
    const double depthMm = 400.0;
    std::vector<cv::Point2d> pts;

    unsigned state = seed;
    const auto jitter = [&]() {
        if (noiseMm <= 0.0) return 0.0;
        // Two uniforms, so the boundary carries something shaped a little like
        // sensor noise rather than a square block of it.
        double sum = 0.0;
        for (int i = 0; i < 2; ++i) {
            state = state * 1664525u + 1013904223u;
            sum += static_cast<double>(state >> 8) / 16777216.0 - 0.5;
        }
        return sum * noiseMm * 2.45;
    };
    const auto put = [&](double x, double y) {
        pts.push_back(cv::Point2d(x + jitter(), y + jitter()));
    };
    // The hem: a parabola through (0, 0) and (0, chord), swept toward -x.
    const auto hemX = [&](double t) { return -sagittaMm * 4.0 * t * (1.0 - t); };
    const auto fillet = [&](double cx, double cy, double a0, double a1) {
        const int n = std::max(4, static_cast<int>(std::fabs(a1 - a0) * radiusMm / kStep));
        for (int k = 0; k <= n; ++k) {
            const double a = a0 + (a1 - a0) * k / n;
            put(cx + radiusMm * std::cos(a), cy + radiusMm * std::sin(a));
        }
    };

    const double t0 = radiusMm / chordMm;
    const double x0 = hemX(t0);
    const double x1 = hemX(1.0 - t0);
    const int nSide = static_cast<int>(depthMm / kStep);
    const int nEdge = static_cast<int>(chordMm / kStep);

    for (int k = nSide; k >= 0; --k) put(x0 + radiusMm + k * depthMm / nSide, 0.0);
    fillet(x0 + radiusMm, radiusMm, -kPi / 2.0, -kPi);
    for (int k = static_cast<int>(t0 * nEdge); k <= static_cast<int>((1.0 - t0) * nEdge); ++k) {
        const double t = static_cast<double>(k) / nEdge;
        put(hemX(t), t * chordMm);
    }
    fillet(x1 + radiusMm, chordMm - radiusMm, kPi, kPi / 2.0);
    for (int k = 0; k <= nSide; ++k) put(x1 + radiusMm + k * depthMm / nSide, chordMm);
    for (int k = 1; k < nEdge; ++k) {
        put(x1 + radiusMm + depthMm, chordMm * (1.0 - static_cast<double>(k) / nEdge));
    }
    return pts;
}

// Something thin hanging off the lay and reaching out past the fold toward the
// machine: a loose thread, a care label, a corner of the binding. It is one
// spike on the outline and it is not the edge, but it is the nearest thing in
// the picture, so anything that decides where to look from the single nearest
// point loses the whole fold behind it.
//
// Inserted into the fold buildLay lays along x = 0, pointing to -x.
std::vector<cv::Point2d> withThread(const std::vector<cv::Point2d>& lay, double atMm,
                                    double reachMm, double widthMm) {
    std::vector<cv::Point2d> out;
    out.reserve(lay.size() + 512);
    bool placed = false;
    for (std::size_t k = 0; k < lay.size(); ++k) {
        out.push_back(lay[k]);
        if (placed || k + 1 >= lay.size()) continue;
        const bool inFold = std::fabs(lay[k].x) < 1e-9 && std::fabs(lay[k + 1].x) < 1e-9;
        if (!inFold || !(lay[k].y <= atMm && lay[k + 1].y > atMm)) continue;
        placed = true;
        const int steps = std::max(4, static_cast<int>(reachMm / 0.4));
        for (int i = 1; i <= steps; ++i) {
            const double u = static_cast<double>(i) / steps;
            out.push_back(cv::Point2d(-reachMm * u, atMm - 0.5 * widthMm * (1.0 - u)));
        }
        for (int i = steps - 1; i >= 1; --i) {
            const double u = static_cast<double>(i) / steps;
            out.push_back(cv::Point2d(-reachMm * u, atMm + 0.5 * widthMm * (1.0 - u)));
        }
    }
    return out;
}

// Turn the lay without turning the machine with it. The operator has laid the
// fold a few degrees off the rail, so one end of it sits deeper into the
// picture than the other. This is the ordinary case, not the awkward one: a
// fold square to the rail to better than a degree is not something a pair of
// hands does at a bench.
void turnLay(std::vector<cv::Point2d>& lay, double deg) {
    const double a = deg * kPi / 180.0, c = std::cos(a), s = std::sin(a);
    for (cv::Point2d& p : lay) p = cv::Point2d(p.x * c - p.y * s, p.x * s + p.y * c);
}

// Turn and shift the lay and the scan direction together, which is what a mat
// sliding under the camera amounts to once the station is recalibrated on it.
void moveBoth(std::vector<cv::Point2d>& lay, cv::Point2d& scan, double deg, double dx,
              double dy) {
    const double a = deg * kPi / 180.0, c = std::cos(a), s = std::sin(a);
    for (cv::Point2d& p : lay) {
        p = cv::Point2d(p.x * c - p.y * s + dx, p.x * s + p.y * c + dy);
    }
    scan = cv::Point2d(scan.x * c - scan.y * s, scan.x * s + scan.y * c);
}

Template demoStyle(double target = 492.0) {
    Template t;
    t.style = "selftest";
    Step s;
    s.name = "half_chest";
    s.target_mm = target;
    s.tol_minus_mm = -12.7;
    s.tol_plus_mm = 12.7;
    t.steps.push_back(s);
    return t;
}

int selftest() {
    Config cfg;
    const cv::Point2d east(1.0, 0.0);
    std::printf("Step QC selftest\n\n");

    std::printf("the edge reads its own length\n");
    for (const double want : {492.0, 220.0, 655.5}) {
        EdgeResult r;
        const bool ok = measureEdge(buildLay(want, 400.0), east, cfg, r);
        check(ok, "a clean fold of " + vjson::formatFixed(want, 1) + " mm is readable");
        if (ok) {
            checkNear(r.length_mm, want, 0.5, "length");
            checkNear(r.straightness_mm, 0.0, 0.2, "straight");
        } else {
            std::printf("        %s\n", r.error.c_str());
        }
    }
    std::printf("\n");

    std::printf("it takes the edge facing the machine, not the longest one\n");
    {
        // A 300 mm fold in front, with a 900 mm straight edge sitting well
        // behind it. The near one must win.
        std::vector<cv::Point2d> lay = buildLay(300.0, 400.0);
        for (int k = 0; k <= 2250; ++k) {
            lay.push_back(cv::Point2d(600.0, -450.0 + k * 0.4));
        }
        EdgeResult r;
        const bool ok = measureEdge(lay, east, cfg, r);
        check(ok, "readable with a longer edge behind it");
        if (ok) checkNear(r.length_mm, 300.0, 0.5, "the near edge is the one measured");
    }
    std::printf("\n");

    std::printf("a fold laid askew is still measured end to end\n");
    {
        // Five degrees off the rail puts the far corner of a 492 mm fold 43 mm
        // deeper into the picture than the near one. That is a lay, not a
        // fault, and all of the fold is still the fold.
        std::vector<cv::Point2d> lay = buildLay(492.0, 400.0);
        turnLay(lay, -5.0);
        EdgeResult r;
        const bool ok = measureEdge(lay, east, cfg, r);
        check(ok, "a fold five degrees off the rail is readable");
        if (ok) {
            checkNear(r.length_mm, 492.0, 1.0, "length");
        } else {
            std::printf("        %s\n", r.error.c_str());
        }
    }
    std::printf("\n");

    std::printf("a thread hanging over the fold does not hide it\n");
    {
        // 45 mm of thread reaching past the fold, nearer the machine than any
        // part of the edge being measured.
        std::vector<cv::Point2d> lay = withThread(buildLay(492.0, 400.0), 246.0, 45.0, 3.0);
        EdgeResult r;
        const bool ok = measureEdge(lay, east, cfg, r);
        check(ok, "the fold is found behind a thread hanging over it");
        if (ok) {
            checkNear(r.length_mm, 492.0, 1.0, "length");
        } else {
            std::printf("        %s\n", r.error.c_str());
        }
    }
    std::printf("\n");

    std::printf("and the two together are not an excuse to measure half of it\n");
    {
        // Both at once, which is what a bench actually presents. Taking the
        // near band from the nearest single point and then refusing to leave it
        // measures from the thread to wherever the tilt carries the fold out of
        // the band, and reports that as the garment.
        std::vector<cv::Point2d> lay = withThread(buildLay(492.0, 400.0), 246.0, 30.0, 3.0);
        turnLay(lay, -3.0);
        EdgeResult r;
        const bool ok = measureEdge(lay, east, cfg, r);
        check(ok, "an askew fold with a thread over it is readable");
        if (ok) {
            checkNear(r.length_mm, 492.0, 1.0, "length");
        } else {
            std::printf("        %s\n", r.error.c_str());
        }
    }
    std::printf("\n");

    std::printf("the mat may slide or turn\n");
    {
        EdgeResult base;
        measureEdge(buildLay(492.0, 400.0), east, cfg, base);
        const double places[][3] = {{0, 0, 0}, {2, 5, -3}, {37, -220, 480}, {-91.5, 1000, 1000},
                                    {180, -50, 25}};
        for (const auto& q : places) {
            std::vector<cv::Point2d> lay = buildLay(492.0, 400.0);
            cv::Point2d scan = east;
            moveBoth(lay, scan, q[0], q[1], q[2]);
            EdgeResult r;
            char what[160];
            std::snprintf(what, sizeof(what), "turned %.1f deg, shifted %.0f by %.0f", q[0], q[1],
                          q[2]);
            check(measureEdge(lay, scan, cfg, r), std::string("readable ") + what);
            if (r.ok) checkNear(r.length_mm, base.length_mm, 0.05, std::string("same ") + what);
        }
    }
    std::printf("\n");

    std::printf("a curved edge reads the chord across it, not the flat part of it\n");
    {
        // The measurement a factory floor takes is a tape laid flat between two
        // seams. It does not follow the sweep of the hem, so neither does this:
        // the answer is the chord, and the curvature is recorded beside it
        // rather than added to it or subtracted from it.
        //
        // 14 mm across 540 mm is what the shirts on this bench actually do.
        struct Case { double chord, sagitta; const char* what; };
        const Case cases[] = {
            {540.0,   0.0, "dead flat"},
            {540.0,   7.0, "a slight sweep toward the machine"},
            {540.0,  14.0, "the sweep a real hem has"},
            {540.0, -14.0, "the same sweep the other way"},
            {540.0,  20.0, "more sweep than a hem should have"},
            {430.0,  12.0, "a shoulder line"},
            {700.0,  16.0, "a body length"},
            {230.0,   5.0, "a sleeve"},
        };
        for (const Case& c : cases) {
            EdgeResult r;
            const bool ok = measureEdge(buildCurvedLay(c.chord, c.sagitta, 400.0), east, cfg, r);
            if (!ok) {
                check(false, std::string(c.what) + " is readable");
                std::printf("        %s\n", r.error.c_str());
                continue;
            }
            char what[200];
            std::snprintf(what, sizeof(what), "%-38s chord %6.1f, bow %5.2f mm", c.what,
                          r.length_mm, r.bow_mm);
            checkNear(r.length_mm, c.chord, 1.0, what);
        }
    }
    std::printf("\n");

    std::printf("the ends are the seams, found the way a floor finds them\n");
    {
        // Nobody on a factory floor measures to where the hem stops being flat.
        // They find the seam at one end, the seam at the other, and pull a tape
        // straight between them. The seam is where the hem and the side seam
        // meet, and cloth rounds that meeting into a curve so that neither edge
        // reaches it - the eye carries both on until they cross, and that is
        // what is automated here.
        //
        // What matters is that the crossing is refused when there is nothing
        // there to cross. That guard is the whole difference between this and
        // the version before it, which walked a fixed distance outward, found
        // no corner on a fold because a fold does not turn one, and added a
        // centimetre the garment did not have at both ends.
        for (const double radius : {2.0, 6.0, 10.0, 15.0, 20.0, 30.0}) {
            EdgeResult e;
            const bool ok =
                measureEdge(buildGarmentLay(540.0, 14.0, radius, 0.25), east, cfg, e);
            if (!ok) {
                check(false, "a corner rounded through " + vjson::formatFixed(radius, 0) +
                                 " mm is readable");
                std::printf("        %s\n", e.error.c_str());
                continue;
            }
            char what[180];
            std::snprintf(what, sizeof(what),
                          "rounded through %4.1f mm reads %6.1f (seam found at %4.1f mm out)",
                          radius, e.length_mm, e.reach_mm / 2.0);
            checkNear(e.length_mm, 540.0, 5.5, what);
            check(e.length_mm < 541.0, "  and never over the 540.0 it actually is");
        }

        // What the crossing is worth: turn it off and the reading falls short
        // by about twice the radius the corner is rounded through, because it
        // stops where the cloth stops being flat.
        Config asItLies = cfg;
        asItLies.edge_corner_reach_mm = 0.0;
        for (const double radius : {10.0, 20.0}) {
            EdgeResult e;
            if (!measureEdge(buildGarmentLay(540.0, 14.0, radius, 0.25), east, asItLies, e)) {
                continue;
            }
            check(e.reach_mm == 0.0 && e.length_mm < 540.0,
                  "measured to where the cloth stops being flat instead, " +
                      vjson::formatFixed(radius, 0) + " mm of rounding costs " +
                      vjson::formatFixed(540.0 - e.length_mm, 1) + " mm");
        }
    }
    std::printf("\n");

    std::printf("the edge is traced, and the trace is what was measured\n");
    {
        // The number and the line on the feed come from the same place, so an
        // operator watching the trace lie along the hem is watching the thing
        // that produced the reading rather than an illustration of it.
        EdgeResult e;
        const bool ok = measureEdge(buildCurvedLay(540.0, 14.0, 400.0), east, cfg, e);
        check(ok, "a swept edge comes back traced");
        if (ok) {
            check(e.path_mm.size() > 8, "the trace has a shape: " +
                                            std::to_string(e.path_mm.size()) + " points");
            checkNear(std::hypot(e.path_mm.front().x - e.a_mm.x,
                                 e.path_mm.front().y - e.a_mm.y),
                      0.0, 0.01, "and it starts at one corner");
            checkNear(std::hypot(e.path_mm.back().x - e.b_mm.x, e.path_mm.back().y - e.b_mm.y),
                      0.0, 0.01, "and ends at the other");
            // Following the edge is longer than crossing it, but only just:
            // this is a question about which definition the floor works to, not
            // about accuracy.
            check(e.traced_mm > e.length_mm,
                  "following the edge is longer than crossing it, by " +
                      vjson::formatFixed(e.traced_mm - e.length_mm, 2) + " mm in " +
                      vjson::formatFixed(e.length_mm, 0));
            check(e.traced_mm - e.length_mm < 3.0,
                  "  which is small enough that it is a definition, not an error");
        }
    }
    std::printf("\n");

    std::printf("the measurement never runs past the cloth\n");
    {
        // The invariant behind "it goes over the object". Whatever the geometry
        // says about where two edges would have met, the garment stops where it
        // stops: a line drawn past the cloth is a line measuring the mat, and
        // an operator watching the end of the span sitting on bare grid has no
        // reason to believe any of the rest of it.
        //
        // Checked on every shape here rather than on one, because the corner
        // search is the part that can reach, and it reaches differently
        // depending on what the boundary does past the end of the edge.
        struct Lay {
            std::vector<cv::Point2d> pts;
            std::string what;
        };
        std::vector<Lay> lays;
        lays.push_back({buildLay(492.0, 400.0), "a flat fold"});
        lays.push_back({buildLay(492.0, 400.0, 4.0), "a gently bowed fold"});
        lays.push_back({buildCurvedLay(540.0, 14.0, 400.0), "a swept hem"});
        lays.push_back({buildCurvedLay(540.0, -14.0, 400.0), "a hem swept the other way"});
        lays.push_back({buildCurvedLay(700.0, 16.0, 400.0), "a body length"});
        for (const double radius : {2.0, 10.0, 20.0, 30.0}) {
            lays.push_back({buildGarmentLay(540.0, 14.0, radius, 0.25),
                            "a hem cornered through " + vjson::formatFixed(radius, 0) + " mm"});
        }
        lays.push_back({withThread(buildLay(492.0, 400.0), 246.0, 45.0, 3.0),
                        "a fold with a thread over it"});

        for (const Lay& lay : lays) {
            EdgeResult e;
            if (!measureEdge(lay.pts, east, cfg, e)) continue;

            // How far the cloth itself reaches, along the line that was
            // measured. Nothing about the reading may lie outside it.
            const double dx = e.b_mm.x - e.a_mm.x, dy = e.b_mm.y - e.a_mm.y;
            const double len = std::hypot(dx, dy);
            if (!(len > 1.0)) continue;
            const cv::Point2d dir(dx / len, dy / len);
            double lo = 0.0, hi = 0.0;
            for (std::size_t k = 0; k < lay.pts.size(); ++k) {
                const double t = lay.pts[k].x * dir.x + lay.pts[k].y * dir.y;
                if (k == 0) {
                    lo = hi = t;
                } else {
                    lo = std::min(lo, t);
                    hi = std::max(hi, t);
                }
            }
            const double ta = e.a_mm.x * dir.x + e.a_mm.y * dir.y;
            const double tb = e.b_mm.x * dir.x + e.b_mm.y * dir.y;
            const double over = std::max(std::max(lo - ta, ta - hi), std::max(lo - tb, tb - hi));
            check(over <= 0.05, lay.what + ": both ends are on the cloth, " +
                                    (over <= 0.0 ? std::string("inside it")
                                                 : vjson::formatFixed(over, 2) + " mm out"));
        }
    }
    std::printf("\n");

    std::printf("a lay it cannot read is refused, not guessed at\n");
    {
        EdgeResult r;
        std::vector<cv::Point2d> few(8, cv::Point2d(0.0, 0.0));
        check(!measureEdge(few, east, cfg, r), "an outline with almost no points is refused");

        check(!measureEdge(buildLay(40.0, 400.0), east, cfg, r),
              "an edge too short to be a fold is refused");
        std::printf("        %s\n", r.error.c_str());

        check(!measureEdge(buildLay(492.0, 400.0, 30.0), east, cfg, r),
              "a badly bowed fold is refused");
        std::printf("        %s\n", r.error.c_str());
    }
    std::printf("\n");

    std::printf("an edge that runs off the picture is not a measurement\n");
    {
        // The working area, in millimetres. The machine looks in along +x, so
        // x = 0 is the rail: a fold lying hard against it is a fold laid the
        // way the step asked for it, and the other three sides are the border
        // of the picture.
        const std::vector<cv::Point2d> area{{0.0, -60.0},
                                            {520.0, -60.0},
                                            {520.0, 560.0},
                                            {0.0, 560.0}};
        EdgeResult r;
        check(measureEdge(buildLay(492.0, 400.0), east, cfg, r, &area),
              "a fold laid against the rail is not mistaken for one running off it");

        // The same fold with the border cutting through where it ends. It reads
        // straight, it reads 492 mm, and 492 mm is the width of the view rather
        // than the width of the garment. That is the one failure that looks
        // exactly like a measurement.
        const std::vector<cv::Point2d> cut{{0.0, -60.0},
                                           {520.0, -60.0},
                                           {520.0, 492.0},
                                           {0.0, 492.0}};
        EdgeResult c;
        check(!measureEdge(buildLay(492.0, 400.0), east, cfg, c, &cut),
              "a fold reaching the border of the picture is refused");
        check(c.clipped, "  and is recorded as having run off it");
        std::printf("        %s\n", c.error.c_str());
    }
    std::printf("\n");

    std::printf("a bow and a rough edge are two different faults\n");
    {
        // Training is what keeps folds flat; the numbers are what catch the bad
        // day. A slight bow still reads, and it is recorded as a bow - not as
        // roughness, which is the cloth, and not as a shorter garment, which is
        // what it used to become.
        EdgeResult r;
        const bool ok = measureEdge(buildLay(492.0, 400.0, 4.0), east, cfg, r);
        check(ok, "a slightly bowed fold still reads");
        if (ok) {
            checkNear(r.length_mm, 492.0, 1.0, "and reads its chord");
            check(r.bow_mm > 3.0 && r.bow_mm < 5.0,
                  "its sweep is recorded as a bow: " + vjson::formatFixed(r.bow_mm, 2) + " mm");
            check(r.straightness_mm < 0.5,
                  "and not as roughness, which is the cloth: " +
                      vjson::formatFixed(r.straightness_mm, 2) + " mm");
        }
    }
    std::printf("\n");

    std::printf("the verdict follows the tolerance band\n");
    {
        const Template t = demoStyle();
        const Step& s = t.steps[0];
        struct Case { double len; bool pass; const char* what; };
        const Case cases[] = {
            {492.0, true, "dead on target passes"},
            {s.target_mm + s.tol_plus_mm - 0.1, true, "just inside the upper limit passes"},
            {s.target_mm + s.tol_minus_mm + 0.1, true, "just inside the lower limit passes"},
            {s.target_mm + s.tol_plus_mm + 0.1, false, "just over the upper limit fails"},
            {s.target_mm + s.tol_minus_mm - 0.1, false, "just under the lower limit fails"},
        };
        for (const Case& c : cases) {
            EdgeResult e;
            e.ok = true;
            e.length_mm = c.len;
            const StepResult r = judgeStep(t, 0, e);
            check(r.pass == c.pass, c.what);
            if (!c.pass) check(!r.reason.empty(), "  and the failure says why");
        }

        EdgeResult bad;
        bad.error = "no clean straight edge was found";
        const StepResult r = judgeStep(t, 0, bad);
        check(!r.pass, "an edge that could not be read is a FAIL, not a skip");
        check(!r.measured, "and is recorded as unmeasured");
    }
    std::printf("\n");

    std::printf("a lay goes all the way to a verdict\n");
    {
        const Template t = demoStyle();
        for (const double len : {492.0, 470.0}) {
            std::vector<cv::Point2d> lay = buildLay(len, 400.0);
            cv::Point2d scan = east;
            moveBoth(lay, scan, 23.0, 800.0, -140.0);
            EdgeResult e;
            measureEdge(lay, scan, cfg, e);
            const StepResult r = judgeStep(t, 0, e);
            const bool expect = len > 479.3 && len < 504.7;
            check(r.pass == expect,
                  vjson::formatFixed(len, 1) + " mm fold -> " + (expect ? "PASS" : "FAIL"));
            std::printf("        %s\n", r.headline().c_str());
        }
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

bool loadOutline(const std::string& path, std::vector<cv::Point2d>& out, std::string& error) {
    vjson::Value doc;
    if (!vjson::parseFile(path, doc, error)) return false;
    const vjson::Value* pts = doc.find("outline_mm");
    if (!pts || !pts->isArray()) {
        error = "expected an \"outline_mm\" array of [x, y] pairs";
        return false;
    }
    for (std::size_t k = 0; k < pts->size(); ++k) {
        const vjson::Value& p = pts->at(k);
        if (!p.isArray() || p.size() < 2) {
            error = "point " + std::to_string(k) + " is not an [x, y] pair";
            return false;
        }
        out.push_back(cv::Point2d(p.at(0).asNumber(), p.at(1).asNumber()));
    }
    return true;
}

void printUsage() {
    std::printf(
        "usage: qc_check --selftest\n"
        "       qc_check --outline PATH [--template PATH] [--step N]\n\n"
        "  --selftest        build folds of known length and check the answers\n"
        "  --outline PATH    a JSON file holding an \"outline_mm\" array. The\n"
        "                    machine looks in along +x, as it would from the\n"
        "                    left of the picture.\n"
        "  --template PATH   the style to judge it against\n"
        "  --step N          which step of that style, 1 based, default 1\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string outlinePath, templatePath;
    int step = 1;
    bool wantSelftest = argc <= 1;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--selftest") {
            wantSelftest = true;
        } else if (a == "--outline" && i + 1 < argc) {
            outlinePath = argv[++i];
        } else if (a == "--template" && i + 1 < argc) {
            templatePath = argv[++i];
        } else if (a == "--step" && i + 1 < argc) {
            step = std::atoi(argv[++i]);
        } else if (a == "-h" || a == "--help") {
            printUsage();
            return 0;
        } else {
            std::printf("unrecognised argument: %s\n", a.c_str());
            printUsage();
            return 2;
        }
    }

    if (outlinePath.empty()) {
        if (!wantSelftest) {
            printUsage();
            return 2;
        }
        return selftest();
    }

    std::string error;
    std::vector<cv::Point2d> outline;
    if (!loadOutline(outlinePath, outline, error)) {
        std::printf("%s: %s\n", outlinePath.c_str(), error.c_str());
        return 1;
    }

    Config cfg;
    EdgeResult edge;
    const bool read = measureEdge(outline, cv::Point2d(1.0, 0.0), cfg, edge);
    if (read) {
        std::printf("edge      : %.2f mm, straight to %.2f mm\n", edge.length_mm,
                    edge.straightness_mm);
    } else {
        std::printf("edge      : could not be read, %s\n", edge.error.c_str());
    }

    if (templatePath.empty()) return read ? 0 : 1;

    Template t;
    if (!loadTemplate(templatePath, t, error)) {
        std::printf("%s: %s\n", templatePath.c_str(), error.c_str());
        return 1;
    }
    const StepResult r = judgeStep(t, step - 1, edge);
    std::printf("%s\nverdict   : %s\n", r.headline().c_str(), r.pass ? "PASS" : "FAIL");
    std::printf("%s\n", r.toJson().dump(2).c_str());
    return r.pass ? 0 : 1;
}
