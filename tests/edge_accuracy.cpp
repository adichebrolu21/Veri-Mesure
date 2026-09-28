// Edge accuracy bench.
//
// The one question this answers: how far is the boundary this pipeline reports
// from the boundary that is actually there? Not "does it look right on the
// feed", not "does the number seem plausible" - how far, in pixels, with a
// sign, against a shape whose true position is known exactly because we drew
// it.
//
// A rectangle is rendered at a known sub-pixel position by area coverage, then
// put through the same lens and sensor a real part goes through: a Gaussian
// point spread, sensor noise, 8-bit quantisation, and a printed grid mat
// underneath that has to cancel. The rendered pair then goes through the real
// segmentObject / largestContour / refineContourSubpixel, and every refined
// point is compared against the analytic edge it came from.
//
// Two numbers come out and they mean different things:
//
//   bias  the mean signed distance. A systematic error. This is the one that
//         matters, because it does not average away over frames and it scales
//         straight into the measurement: a boundary biased 0.2 px outward all
//         the way round makes every span read 0.4 px too long, every time.
//   rms   the spread. Averaging frames and fitting sides both reduce this, so
//         it is the less dangerous of the two, but it sets how much averaging
//         is needed to reach a given repeatability.
//
// The sweep over sub-pixel phase is the point of the whole harness. An edge
// estimator that is quietly rounding to whole pixels still scores well at
// phase 0 and phase 0.5; it gives itself away as a swing across the phases,
// which is what "pixel locking" is. A flat curve across phase is what a real
// sub-pixel estimator looks like.
//
//     edge_accuracy            the standard sweep
//     edge_accuracy --quick    one phase, for a fast check
//
// Exits non-zero if the pipeline fails to produce a boundary at all, not on
// the accuracy figures: this is a measurement, not a pass/fail gate, and the
// numbers are for comparing a change against the run before it.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/config.hpp"
#include "verimeasure/measure.hpp"
#include "verimeasure/segmentation.hpp"

using namespace verimeasure;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ---- the truth ----

struct TrueRect {
    cv::Point2d centre;
    double width = 0.0;   // along the rotated x axis
    double height = 0.0;  // along the rotated y axis
    double angleDeg = 0.0;
};

std::vector<cv::Point2d> cornersOf(const TrueRect& r) {
    const double t = r.angleDeg * kPi / 180.0;
    const double ct = std::cos(t), st = std::sin(t);
    const double hw = r.width / 2.0, hh = r.height / 2.0;
    const double lx[4] = {-hw, hw, hw, -hw};
    const double ly[4] = {-hh, -hh, hh, hh};
    std::vector<cv::Point2d> out(4);
    for (int k = 0; k < 4; ++k) {
        out[k] = cv::Point2d(r.centre.x + ct * lx[k] - st * ly[k],
                             r.centre.y + st * lx[k] + ct * ly[k]);
    }
    return out;
}

// Signed distance to the boundary of a convex polygon, positive outside.
//
// The maximum over the outward half-planes. Outside the shape this is the exact
// distance except in the wedge beyond a corner; inside, it is minus the
// distance to the nearest side. Corner neighbourhoods are excluded from the
// statistics for exactly that reason.
double signedDistance(const std::vector<cv::Point2d>& poly, const cv::Point2d& p) {
    double best = -1e18;
    const std::size_t n = poly.size();
    for (std::size_t k = 0; k < n; ++k) {
        const cv::Point2d& a = poly[k];
        const cv::Point2d& b = poly[(k + 1) % n];
        const double ex = b.x - a.x, ey = b.y - a.y;
        const double len = std::hypot(ex, ey);
        if (len < 1e-12) continue;
        // Outward normal for a clockwise polygon in image coordinates.
        const double nx = ey / len, ny = -ex / len;
        best = std::max(best, (p.x - a.x) * nx + (p.y - a.y) * ny);
    }
    return best;
}

double distanceToNearestCorner(const std::vector<cv::Point2d>& poly, const cv::Point2d& p) {
    double best = 1e18;
    for (const cv::Point2d& c : poly) best = std::min(best, std::hypot(p.x - c.x, p.y - c.y));
    return best;
}

bool insideConvex(const std::vector<cv::Point2d>& poly, const cv::Point2d& p) {
    return signedDistance(poly, p) <= 0.0;
}

// ---- the rendering ----

// Fraction of pixel (x, y) covered by the polygon, by supersampling.
//
// The pixel is centred ON its integer coordinate and spans [x-0.5, x+0.5).
// That is OpenCV's convention and it is the one bilinearSample, cv::Sobel and
// findContours all use, so it is the one the truth has to be stated in. Get it
// wrong by half a pixel and the harness reports the mismatch as estimator
// error, which is exactly the kind of self-deception this file exists to
// prevent.
double coverage(const std::vector<cv::Point2d>& poly, int x, int y, int sub) {
    int hits = 0;
    for (int j = 0; j < sub; ++j) {
        for (int i = 0; i < sub; ++i) {
            const cv::Point2d s(x - 0.5 + (i + 0.5) / sub, y - 0.5 + (j + 0.5) / sub);
            if (insideConvex(poly, s)) ++hits;
        }
    }
    return static_cast<double>(hits) / (sub * sub);
}

struct Scene {
    int width = 900;
    int height = 700;
    double gridPitchPx = 80.0;   // the printed mat, which must cancel
    double gridLinePx = 2.4;
    bool grid = true;
    double psfSigma = 0.9;       // lens and sensor point spread
    double noiseSigma = 1.5;     // grey levels, per channel
    cv::Scalar mat{58, 62, 66};
    cv::Scalar line{168, 172, 176};
    cv::Scalar fabric{196, 198, 190};
};

// The mat, before anything is laid on it. Rendered as float so the part image
// can be composited against exactly the same surface.
cv::Mat renderMat(const Scene& sc) {
    cv::Mat img(sc.height, sc.width, CV_32FC3);
    for (int y = 0; y < sc.height; ++y) {
        float* row = img.ptr<float>(y);
        const double py = y;
        for (int x = 0; x < sc.width; ++x) {
            const double px = x;
            double onLine = 0.0;
            if (sc.grid) {
                // Distance to the nearest grid line in each axis, softened over
                // the line width so the printed lines are not themselves a
                // staircase competing with the part.
                const double dx = std::abs(std::fmod(px, sc.gridPitchPx) - 0.0);
                const double dy = std::abs(std::fmod(py, sc.gridPitchPx) - 0.0);
                const double ex = std::min(dx, sc.gridPitchPx - dx);
                const double ey = std::min(dy, sc.gridPitchPx - dy);
                const double half = sc.gridLinePx / 2.0;
                const double cx = std::max(0.0, std::min(1.0, half + 0.5 - ex));
                const double cy = std::max(0.0, std::min(1.0, half + 0.5 - ey));
                onLine = std::max(cx, cy);
            }
            for (int c = 0; c < 3; ++c) {
                row[x * 3 + c] =
                    static_cast<float>(sc.mat[c] * (1.0 - onLine) + sc.line[c] * onLine);
            }
        }
    }
    return img;
}

// Optics, sensor and quantisation, applied identically to both frames so that
// nothing in them is a difference the segmentation could mistake for fabric.
cv::Mat expose(const cv::Mat& scene32, const Scene& sc, cv::RNG& rng) {
    cv::Mat blurred;
    if (sc.psfSigma > 0.0) {
        cv::GaussianBlur(scene32, blurred, cv::Size(0, 0), sc.psfSigma);
    } else {
        blurred = scene32.clone();
    }
    if (sc.noiseSigma > 0.0) {
        cv::Mat noise(blurred.size(), CV_32FC3);
        rng.fill(noise, cv::RNG::NORMAL, 0.0, sc.noiseSigma);
        blurred += noise;
    }
    cv::Mat out;
    blurred.convertTo(out, CV_8UC3);  // saturating, rounding
    return out;
}

cv::Mat renderPart(const cv::Mat& matScene32, const Scene& sc,
                   const std::vector<cv::Point2d>& poly) {
    cv::Mat img = matScene32.clone();

    // Only the band around the boundary needs supersampling; the interior is
    // covered outright and the outside is untouched.
    const cv::Rect box = cv::boundingRect(std::vector<cv::Point2f>{
        cv::Point2f(static_cast<float>(poly[0].x), static_cast<float>(poly[0].y)),
        cv::Point2f(static_cast<float>(poly[1].x), static_cast<float>(poly[1].y)),
        cv::Point2f(static_cast<float>(poly[2].x), static_cast<float>(poly[2].y)),
        cv::Point2f(static_cast<float>(poly[3].x), static_cast<float>(poly[3].y))});
    const cv::Rect clipped =
        (box + cv::Size(4, 4)) - cv::Point(2, 2);
    const cv::Rect area = clipped & cv::Rect(0, 0, img.cols, img.rows);

    for (int y = area.y; y < area.y + area.height; ++y) {
        float* row = img.ptr<float>(y);
        for (int x = area.x; x < area.x + area.width; ++x) {
            const cv::Point2d centre(x, y);
            const double d = signedDistance(poly, centre);
            double cov;
            if (d < -1.5) {
                cov = 1.0;
            } else if (d > 1.5) {
                cov = 0.0;
            } else {
                cov = coverage(poly, x, y, 16);
            }
            if (cov <= 0.0) continue;
            for (int c = 0; c < 3; ++c) {
                row[x * 3 + c] = static_cast<float>(row[x * 3 + c] * (1.0 - cov) +
                                                    sc.fabric[c] * cov);
            }
        }
    }
    return img;
}

// ---- a picture of the difference ----
//
// The bench answers "how far off is it" in numbers. This answers "what does
// that look like", which is the question an operator standing at the machine is
// actually asking. It draws, over a magnified crop of the rendered cloth, the
// truth and the two outlines the station can put on the feed: the decimated one
// the live pass produces, and the refined one a finished measurement produces.
void writeComparison(const std::string& path, const TrueRect& truth, const Scene& sc,
                     const Config& cfg, int detectStep) {
    const std::vector<cv::Point2d> poly = cornersOf(truth);
    cv::RNG rngBg(4242), rngPart(9119);
    const cv::Mat matScene = renderMat(sc);
    const cv::Mat background = expose(matScene, sc, rngBg);
    const cv::Mat part = expose(renderPart(matScene, sc, poly), sc, rngPart);

    // The refined outline, exactly as a finished measurement produces it.
    cv::Mat mask, score;
    double changed = 0.0;
    segmentObject(part, background, cfg, robustGain(part, background), 0, 0.0, mask, changed,
                  score);
    std::vector<cv::Point> contour;
    if (!largestContour(mask, 500.0, contour)) return;
    const std::vector<cv::Point2d> refined = refineContourSubpixel(score, contour, cfg);

    // The live outline, exactly as the decimated pass produces it.
    cv::Mat partSmall, bgSmall;
    cv::resize(part, partSmall, cv::Size(part.cols / detectStep, part.rows / detectStep), 0, 0,
               cv::INTER_AREA);
    cv::resize(background, bgSmall, cv::Size(part.cols / detectStep, part.rows / detectStep), 0,
               0, cv::INTER_AREA);
    cv::Mat maskSmall, scoreSmall;
    segmentObject(partSmall, bgSmall, cfg, robustGain(partSmall, bgSmall), 0, 0.0, maskSmall,
                  changed, scoreSmall);
    std::vector<cv::Point> contourSmall;
    const bool haveSmall = largestContour(maskSmall, 500.0 / (detectStep * detectStep),
                                          contourSmall);

    // A corner, magnified, because that is where an outline is easiest to judge.
    const int zoom = 6;
    const cv::Point2d anchor = poly[0];
    cv::Rect crop(static_cast<int>(anchor.x) - 30, static_cast<int>(anchor.y) - 24, 150, 96);
    crop &= cv::Rect(0, 0, part.cols, part.rows);
    cv::Mat view;
    cv::resize(part(crop), view, cv::Size(crop.width * zoom, crop.height * zoom), 0, 0,
               cv::INTER_NEAREST);

    const int bits = 4;
    const double sub = 16.0;
    auto place = [&](double x, double y) {
        return cv::Point(static_cast<int>(std::lround((x - crop.x) * zoom * sub)),
                         static_cast<int>(std::lround((y - crop.y) * zoom * sub)));
    };

    // Truth, in white, so the two outlines can be judged against it and not
    // against each other.
    std::vector<cv::Point> truthPts;
    for (const cv::Point2d& p : poly) truthPts.push_back(place(p.x, p.y));
    cv::polylines(view, truthPts, true, cv::Scalar(255, 255, 255), 1, cv::LINE_AA, bits);

    if (haveSmall) {
        std::vector<cv::Point> old;
        std::vector<cv::Point> centred;
        const double halfCell = 0.5 * (detectStep - 1);
        for (const cv::Point& p : contourSmall) {
            old.push_back(place(p.x * detectStep, p.y * detectStep));
            centred.push_back(
                place(p.x * detectStep + halfCell, p.y * detectStep + halfCell));
        }
        cv::polylines(view, old, true, cv::Scalar(70, 70, 235), 2, cv::LINE_AA, bits);
        cv::polylines(view, centred, true, cv::Scalar(60, 190, 240), 1, cv::LINE_AA, bits);
    }

    std::vector<cv::Point> fine;
    for (const cv::Point2d& p : refined) fine.push_back(place(p.x, p.y));
    cv::polylines(view, fine, true, cv::Scalar(100, 235, 140), 1, cv::LINE_AA, bits);

    // A solid backing for the key. The crop behind it is cloth and printed
    // lines, and a legend that has to be deciphered is worse than none.
    const int line = 22;
    cv::rectangle(view, cv::Rect(0, 0, 430, 4 * line + 12), cv::Scalar(16, 18, 20), cv::FILLED);
    int y = line;
    auto caption = [&](const char* text, const cv::Scalar& colour) {
        cv::putText(view, text, cv::Point(12, y), cv::FONT_HERSHEY_SIMPLEX, 0.42, colour, 1,
                    cv::LINE_AA);
        y += line;
    };
    caption("white  the cloth edge as drawn", cv::Scalar(255, 255, 255));
    caption("red    live outline, was: index x step, 2 px", cv::Scalar(70, 70, 235));
    caption("amber  live outline, now: block centre, 1 px", cv::Scalar(60, 190, 240));
    caption("green  refined outline, now drawn when held", cv::Scalar(100, 235, 140));

    if (cv::imwrite(path, view)) {
        std::printf("  wrote %s (%dx%d, %dx magnification)\n", path.c_str(), view.cols,
                    view.rows, zoom);
    } else {
        std::printf("  could not write %s\n", path.c_str());
    }
}

// ---- the measurement ----

struct Accuracy {
    bool ok = false;
    int points = 0;
    double biasPx = 0.0;   // mean signed distance, positive = outside the truth
    double rmsPx = 0.0;    // spread about zero
    double maxPx = 0.0;
    double rawBiasPx = 0.0;  // the same, for the unrefined integer contour
    double rawRmsPx = 0.0;
    bool haveFit = false;
    double lengthErrPx = 0.0;  // fitted long side minus the true long side
    double breadthErrPx = 0.0;

    // Per-side bias, in the order cornersOf() lays the sides out. A whole-
    // outline mean hides the case that matters most here: one side sitting on
    // a printed line while the other three are perfect averages out to a small
    // number and looks like nothing is wrong.
    double sideBiasPx[4] = {0.0, 0.0, 0.0, 0.0};
    int sideCount[4] = {0, 0, 0, 0};
};

// Which side of the polygon a point belongs to: the one whose outward
// half-plane distance is largest, which is the side it is nearest.
int nearestSide(const std::vector<cv::Point2d>& poly, const cv::Point2d& p) {
    int best = 0;
    double bestD = -1e18;
    const std::size_t n = poly.size();
    for (std::size_t k = 0; k < n; ++k) {
        const cv::Point2d& a = poly[k];
        const cv::Point2d& b = poly[(k + 1) % n];
        const double ex = b.x - a.x, ey = b.y - a.y;
        const double len = std::hypot(ex, ey);
        if (len < 1e-12) continue;
        const double d = (p.x - a.x) * (ey / len) + (p.y - a.y) * (-ex / len);
        if (d > bestD) {
            bestD = d;
            best = static_cast<int>(k);
        }
    }
    return best;
}

void accumulate(const std::vector<cv::Point2d>& pts, const std::vector<cv::Point2d>& truth,
                double cornerGuardPx, double& bias, double& rms, double& worst, int& n) {
    double sum = 0.0, sumSq = 0.0;
    worst = 0.0;
    n = 0;
    for (const cv::Point2d& p : pts) {
        // Corners are excluded: the half-plane distance is not the true
        // distance in the wedge beyond one, and a rounded corner is a property
        // of the renderer as much as of the estimator.
        if (distanceToNearestCorner(truth, p) < cornerGuardPx) continue;
        const double d = signedDistance(truth, p);
        if (std::abs(d) > 6.0) continue;  // a point that lost the edge entirely
        sum += d;
        sumSq += d * d;
        worst = std::max(worst, std::abs(d));
        ++n;
    }
    bias = n ? sum / n : 0.0;
    rms = n ? std::sqrt(sumSq / n) : 0.0;
}

Accuracy measureOnce(const TrueRect& truth, const Scene& sc, const Config& cfg,
                     unsigned long long seed) {
    Accuracy out;
    const std::vector<cv::Point2d> poly = cornersOf(truth);

    cv::RNG rngBg(seed);
    cv::RNG rngPart(seed + 977);
    const cv::Mat matScene = renderMat(sc);
    const cv::Mat background = expose(matScene, sc, rngBg);
    const cv::Mat part = expose(renderPart(matScene, sc, poly), sc, rngPart);

    const double gain = robustGain(part, background);
    cv::Mat mask, score;
    double changed = 0.0;
    segmentObject(part, background, cfg, gain, 0, 0.0, mask, changed, score);

    std::vector<cv::Point> contour;
    if (!largestContour(mask, 500.0, contour)) return out;

    std::vector<cv::Point2d> raw;
    raw.reserve(contour.size());
    for (const cv::Point& p : contour) raw.emplace_back(p.x, p.y);

    const std::vector<cv::Point2d> refined = refineContourSubpixel(score, contour, cfg);

    const double guard = 0.10 * std::min(truth.width, truth.height);
    int n = 0;
    double worst = 0.0;
    accumulate(raw, poly, guard, out.rawBiasPx, out.rawRmsPx, worst, n);
    accumulate(refined, poly, guard, out.biasPx, out.rmsPx, out.maxPx, n);
    out.points = n;
    out.ok = n > 20;

    for (const cv::Point2d& p : refined) {
        if (distanceToNearestCorner(poly, p) < guard) continue;
        const double d = signedDistance(poly, p);
        if (std::abs(d) > 6.0) continue;
        const int side = nearestSide(poly, p);
        out.sideBiasPx[side] += d;
        ++out.sideCount[side];
    }
    for (int k = 0; k < 4; ++k) {
        if (out.sideCount[k]) out.sideBiasPx[k] /= out.sideCount[k];
    }

    // What the boundary error is worth as a measurement. fitRectangleMm works
    // in whatever units it is handed, so handing it pixels answers the question
    // in pixels.
    RectFit fit;
    if (fitRectangleMm(refined, cfg, fit)) {
        out.haveFit = true;
        const double trueLong = std::max(truth.width, truth.height);
        const double trueShort = std::min(truth.width, truth.height);
        out.lengthErrPx = fit.length_mm - trueLong;
        out.breadthErrPx = fit.breadth_mm - trueShort;
    }
    return out;
}

}  // namespace

// Everything the sweep accumulates, so the two surfaces can be summarised side
// by side and the cost of the printed mat read straight off.
struct Tally {
    int runs = 0;
    int fits = 0;
    int failures = 0;
    double sumBias = 0.0, sumSqBias = 0.0, sumRms = 0.0;
    double sumLen = 0.0, sumSqLen = 0.0;
    double minBias = 1e18, maxBias = -1e18;

    void add(const Accuracy& r) {
        ++runs;
        sumBias += r.biasPx;
        sumSqBias += r.biasPx * r.biasPx;
        sumRms += r.rmsPx;
        minBias = std::min(minBias, r.biasPx);
        maxBias = std::max(maxBias, r.biasPx);
        if (r.haveFit) {
            ++fits;
            sumLen += r.lengthErrPx;
            sumSqLen += r.lengthErrPx * r.lengthErrPx;
        }
    }

    void report(const char* what) const {
        if (runs == 0) {
            std::printf("  %s: no runs completed\n", what);
            return;
        }
        std::printf("  %s, over %d runs\n", what, runs);
        std::printf("    mean bias           %+.4f px   systematic, doubles into every span\n",
                    sumBias / runs);
        std::printf("    bias RMS            %7.4f px\n", std::sqrt(sumSqBias / runs));
        std::printf("    bias swing by phase %7.4f px   pixel locking; flat is good\n",
                    maxBias - minBias);
        std::printf("    mean point RMS      %7.4f px\n", sumRms / runs);
        if (fits > 0) {
            std::printf("    length error        %+.4f px mean, %.4f px RMS, over %d fits\n",
                        sumLen / fits, std::sqrt(sumSqLen / fits), fits);
        }
    }
};

void printUsage() {
    std::printf(
        "usage: edge_accuracy [options]\n"
        "  --quick              two phases at one angle\n"
        "  --sobel              Sobel instead of Scharr for the gradient\n"
        "  --no-shift-filter    keep every correction as found, outliers included\n"
        "  --psf S              the optical point spread, in pixels (the aperture)\n"
        "  --blur S             the blur applied before the gradient\n"
        "  --search R           how far along the normal to look, in pixels\n"
        "  --resample P         boundary sample spacing, in pixels\n"
        "  --shift-y D          move the part D px down, to sit it off the printed lines\n"
        "  --image PATH         write a magnified picture of the outlines against the truth\n"
        "\n"
        "Every option changes one thing, so a run can be attributed. The numbers are\n"
        "for comparing against the run before them, not for passing a threshold.\n");
}

int main(int argc, char** argv) {
    bool quick = false;
    double shiftY = 0.0;
    std::string imagePath;

    Config cfg;
    Scene base;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const bool more = (i + 1 < argc);
        if (std::strcmp(a, "--quick") == 0) {
            quick = true;
        } else if (std::strcmp(a, "--noise") == 0 && more) {
            base.noiseSigma = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--sobel") == 0) {
            cfg.subpixel_scharr = false;
        } else if (std::strcmp(a, "--no-shift-filter") == 0) {
            cfg.subpixel_shift_outlier_px = 0.0;
        } else if (std::strcmp(a, "--psf") == 0 && more) {
            // The optics, not the algorithm: how wide a point of light lands on
            // the sensor. That is what the aperture sets, so sweeping it is how
            // this bench answers what to stop the lens down to.
            base.psfSigma = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--blur") == 0 && more) {
            cfg.subpixel_blur_sigma = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--search") == 0 && more) {
            cfg.subpixel_search_px = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--resample") == 0 && more) {
            cfg.contour_resample_px = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--shift-y") == 0 && more) {
            shiftY = std::atof(argv[++i]);
        } else if (std::strcmp(a, "--image") == 0 && more) {
            imagePath = argv[++i];
        } else {
            printUsage();
            return 2;
        }
    }

    const double phases[] = {0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875};
    const int phaseCount = quick ? 2 : static_cast<int>(sizeof(phases) / sizeof(phases[0]));
    const double angles[] = {0.0, 11.3, 31.0};
    const int angleCount = quick ? 1 : static_cast<int>(sizeof(angles) / sizeof(angles[0]));

    std::printf("Edge accuracy bench\n");
    std::printf("  scene      %dx%d, grid %.0f px pitch, PSF sigma %.2f, noise sigma %.2f\n",
                base.width, base.height, base.gridPitchPx, base.psfSigma, base.noiseSigma);
    std::printf("  refinement search +/-%.1f px, step %.2f px, blur %.2f, resample %.1f px,"
                " smooth %d\n",
                cfg.subpixel_search_px, cfg.subpixel_step_px, cfg.subpixel_blur_sigma,
                cfg.contour_resample_px, cfg.contour_smooth_window);
    std::printf("  gradient %s, shift outlier limit %.2f px, part shifted %+.1f px\n",
                cfg.subpixel_scharr ? "Scharr" : "Sobel", cfg.subpixel_shift_outlier_px,
                shiftY);
    std::printf("  Every case runs twice: on a plain mat, and on the printed grid the bench\n"
                "  actually uses. The grid cancels in the difference image in theory. The gap\n"
                "  between the two summaries is how well it cancels in practice.\n\n");

    Tally plain, printed;

    for (int g = 0; g < 2; ++g) {
        Scene sc = base;
        sc.grid = (g == 1);
        Tally& tally = sc.grid ? printed : plain;
        std::printf("== %s ==\n\n", sc.grid ? "printed grid mat" : "plain mat");

        for (int a = 0; a < angleCount; ++a) {
            std::printf("  angle %5.1f deg\n", angles[a]);
            std::printf("    phase   refined bias    refined rms    raw bias    raw rms"
                        "     length err   breadth err     per-side bias\n");
            for (int p = 0; p < phaseCount; ++p) {
                TrueRect truth;
                truth.width = 401.3;
                truth.height = 262.7;
                truth.angleDeg = angles[a];
                truth.centre = cv::Point2d(base.width / 2.0 + phases[p],
                                           base.height / 2.0 + phases[p] + shiftY);

                const Accuracy r = measureOnce(truth, sc, cfg, 20260904ull + 31ull * a + p);
                if (!r.ok) {
                    std::printf("    %5.3f   no boundary was produced\n", phases[p]);
                    ++tally.failures;
                    continue;
                }
                tally.add(r);

                char lenText[32] = "        -";
                char brdText[32] = "        -";
                if (r.haveFit) {
                    std::snprintf(lenText, sizeof(lenText), "%+9.4f", r.lengthErrPx);
                    std::snprintf(brdText, sizeof(brdText), "%+9.4f", r.breadthErrPx);
                }
                std::printf("    %5.3f     %+8.4f       %7.4f    %+8.4f    %7.4f    %s    %s"
                            "   %+6.3f %+6.3f %+6.3f %+6.3f\n",
                            phases[p], r.biasPx, r.rmsPx, r.rawBiasPx, r.rawRmsPx, lenText,
                            brdText, r.sideBiasPx[0], r.sideBiasPx[1], r.sideBiasPx[2],
                            r.sideBiasPx[3]);
            }
            std::printf("\n");
        }
    }

    if (!imagePath.empty()) {
        TrueRect shown;
        shown.width = 401.3;
        shown.height = 262.7;
        shown.angleDeg = 11.3;
        shown.centre = cv::Point2d(base.width / 2.0 + 0.375, base.height / 2.0 + 0.375);
        std::printf("\n");
        writeComparison(imagePath, shown, base, cfg, 6);
    }

    plain.report("plain mat");
    std::printf("\n");
    printed.report("printed grid mat");

    const int failures = plain.failures + printed.failures;
    if (failures > 0) {
        std::printf("\n%d configuration(s) produced no boundary at all\n", failures);
        return 1;
    }
    return 0;
}
