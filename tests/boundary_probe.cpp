// Why is the boundary where it is?
//
// Takes the two pictures the station dumps with the "dump" command - the frame
// it measured and the background it differenced against - and runs the same
// segmentation on them offline, reporting what the mask did and writing the
// intermediates out to look at.
//
// It exists because an outline drawn in the wrong place cannot be diagnosed
// from the overlay. The overlay is a quarter-resolution JPEG with the answer
// already drawn on it, and the questions worth asking - is this the cloth, the
// shadow beside it, or the printed grid underneath - are all about pixels that
// the overlay has thrown away.
//
//   boundary_probe measure-debug
//   boundary_probe measure-debug --pitch 72.5

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "verimeasure/config.hpp"
#include "verimeasure/cv_compat.hpp"
#include "verimeasure/measure.hpp"
#include "verimeasure/segmentation.hpp"

using namespace verimeasure;

namespace {

// How strongly a run of numbers repeats, and at what spacing. A boundary that
// is following the printed grid says so here and nowhere else.
void reportPeriodicity(const std::vector<double>& v, double pitchPx, const char* what) {
    if (v.size() < 64) {
        std::printf("  %s: too short to test\n", what);
        return;
    }
    double mean = 0.0;
    for (double x : v) mean += x;
    mean /= static_cast<double>(v.size());
    std::vector<double> c(v.size());
    double energy = 0.0;
    for (std::size_t i = 0; i < v.size(); ++i) {
        c[i] = v[i] - mean;
        energy += c[i] * c[i];
    }
    if (energy <= 0.0) {
        std::printf("  %s: perfectly flat\n", what);
        return;
    }
    const int maxLag = std::min<int>(static_cast<int>(v.size()) / 2, 400);
    int bestLag = 0;
    double best = 0.0;
    for (int lag = 4; lag <= maxLag; ++lag) {
        double s = 0.0;
        const int n = static_cast<int>(v.size()) - lag;
        for (int i = 0; i < n; ++i) s += c[i] * c[i + lag];
        const double norm = (s / n) / (energy / v.size());
        if (norm > best) {
            best = norm;
            bestLag = lag;
        }
    }
    std::printf("  %s: strongest repeat at %d px (score %.2f)", what, bestLag, best);
    if (pitchPx > 0.0 && best > 0.2) {
        const double ratio = bestLag / pitchPx;
        // A whole multiple of the pitch, and at least one of them: a repeat at
        // a twentieth of a pitch is the contour staircasing, not the grid.
        const bool onGrid = ratio >= 0.75 && std::fabs(ratio - std::lround(ratio)) < 0.12;
        std::printf("  = %.2f x the grid pitch%s", ratio,
                    onGrid ? "   <-- the boundary is following the printed grid" : "");
    }
    std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: boundary_probe <dir with frame.png and background.png> [--pitch N]\n");
        return 2;
    }
    const std::string dir = argv[1];
    double pitchPx = 0.0;
    for (int i = 2; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--pitch") == 0) pitchPx = std::atof(argv[i + 1]);
    }

    const cv::Mat frame = cv::imread(dir + "/frame.png", cv::IMREAD_COLOR);
    const cv::Mat bg = cv::imread(dir + "/background.png", cv::IMREAD_COLOR);
    if (frame.empty() || bg.empty()) {
        std::printf("could not read %s/frame.png and %s/background.png\n", dir.c_str(),
                    dir.c_str());
        return 2;
    }
    if (frame.size() != bg.size()) {
        std::printf("frame is %dx%d but background is %dx%d\n", frame.cols, frame.rows, bg.cols,
                    bg.rows);
        return 2;
    }
    std::printf("frame %dx%d\n\n", frame.cols, frame.rows);

    Config cfg;
    cv::Mat mask, score;
    double changed = 0.0;
    segmentObject(frame, bg, cfg, 1.0, 0, 0.0, mask, changed, score);

    std::vector<cv::Point> contour;
    if (!largestContour(mask, 500.0, contour)) {
        std::printf("no object found in the mask at all\n");
        cv::imwrite(dir + "/mask.png", mask);
        return 1;
    }
    const cv::Rect box = cv::boundingRect(contour);
    std::printf("object spans (%d,%d) to (%d,%d), %d x %d px\n", box.x, box.y, box.br().x,
                box.br().y, box.width, box.height);
    std::printf("changed fraction of the working area: %.2f%%\n\n", 100.0 * changed);

    // The lower boundary of the mask, column by column. This is the line that
    // gets drawn, and if it is chasing something printed rather than something
    // made of cloth, its shape says so.
    std::vector<double> lower;
    lower.reserve(static_cast<std::size_t>(box.width));
    for (int x = box.x; x < box.br().x; ++x) {
        int y = box.br().y - 1;
        while (y > box.y && mask.at<unsigned char>(y, x) == 0) --y;
        lower.push_back(static_cast<double>(y));
    }
    double lo = 1e9, hi = -1e9, mean = 0.0;
    for (double v : lower) {
        lo = std::min(lo, v);
        hi = std::max(hi, v);
        mean += v;
    }
    mean /= static_cast<double>(lower.size());
    std::printf("lower boundary: spans %.0f px of height, %.0f px from its own mean at the worst\n",
                hi - lo, std::max(hi - mean, mean - lo));
    reportPeriodicity(lower, pitchPx, "lower boundary");

    // The same question asked of the refined boundary, which is what is drawn.
    const std::vector<cv::Point2d> refined = refineContourSubpixel(score, contour, cfg);
    std::printf("\nrefinement moved %zu boundary points\n", refined.size());

    cv::Mat shown;
    frame.copyTo(shown);
    std::vector<std::vector<cv::Point>> polys{contour};
    cv::drawContours(shown, polys, -1, cv::Scalar(255, 255, 255), 2);
    cv::imwrite(dir + "/mask.png", mask);
    cv::Mat scoreU8;
    cv::normalize(score, scoreU8, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::imwrite(dir + "/score.png", scoreU8);
    cv::imwrite(dir + "/outline.png", shown);
    std::printf("\nwrote %s/mask.png, score.png and outline.png\n", dir.c_str());
    return 0;
}
