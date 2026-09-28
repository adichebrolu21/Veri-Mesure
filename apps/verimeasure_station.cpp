/*
VeriMeasure garment station - operator interface.

One shirt is a short list of steps. For each one the operator folds the garment
the way that step's picture shows, lays the fold against the rail, and the
station reads a single straight-line span across it and says PASS or FAIL. Next
takes them to the following step; Next shirt starts the list again.

Nothing about the measurement changes here. This file drives the same engine the
command line runner drives, and reads its state. The only difference is that the
overlay burned into the video is just the outline of the part; every number is
rendered as text in the page, which is both far more legible and far easier to
restyle than putText.

Every frame comes off the camera. There is no synthetic or replay source: if
the camera is not there, the station says so and stops rather than showing you
a number that came from anywhere else.

Run it:
    verimeasure_station --template templates/style-42-polo-m.json
    verimeasure_station --port 8080 --host 0.0.0.0

Then open http://localhost:8000 . Serving on 0.0.0.0 puts the readout on a
tablet or a second screen at the bench, which is usually what you want.
*/

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/calibration.hpp"
#include "verimeasure/camera.hpp"
#include "verimeasure/capture_store.hpp"
#include "verimeasure/collar.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/curve.hpp"
#include "verimeasure/engine.hpp"
#include "verimeasure/finger.hpp"
#include "verimeasure/http_server.hpp"
#include "verimeasure/perimeter.hpp"
#include "verimeasure/qc.hpp"
#include "verimeasure/edge.hpp"
#include "verimeasure/station_page.hpp"
#include "verimeasure/template.hpp"

using namespace verimeasure;

namespace {

// Only what the interface shows. Full records still go to the JSON Lines file
// if one was configured.
vjson::Value trimResult(const vjson::Value& full) {
    static const char* keep[] = {"object_number", "length_mm",          "breadth_mm",
                                 "area_mm2",      "angle_deg",          "timestamp",
                                 "fit_method",    "saturated_fraction", "frames_averaged",
                                 "settled"};
    vjson::Value out = vjson::Value::object();
    for (const char* key : keep) {
        if (const vjson::Value* v = full.find(key)) out.set(key, *v);
    }
    return out;
}

// A reference image path comes out of a template file, which is authored on the
// same machine, but it still becomes a file read triggered by an HTTP request.
// Anything that could climb out of the templates directory is refused rather
// than reasoned about.
bool safeRelativePath(const std::string& p) {
    if (p.empty()) return false;
    if (p.find("..") != std::string::npos) return false;
    if (p.front() == '/' || p.front() == '\\') return false;
    if (p.size() > 1 && p[1] == ':') return false;
    return true;
}

std::string contentTypeFor(const std::string& path) {
    const std::size_t dot = path.find_last_of('.');
    std::string ext = dot == std::string::npos ? std::string() : path.substr(dot + 1);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (ext == "svg") return "image/svg+xml";
    if (ext == "jpg" || ext == "jpeg") return "image/jpeg";
    if (ext == "png") return "image/png";
    if (ext == "webp") return "image/webp";
    if (ext == "gif") return "image/gif";
    return "application/octet-stream";
}

bool readWholeFile(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return true;
}

// -- drawing the overlay --
//
// OpenCV's drawing takes integer points, which would round every outline point
// to a whole display pixel and throw away the sub-pixel work that produced it.
// Its shift parameter takes the coordinates in fractions of a pixel instead, so
// the line is rasterised where the boundary actually is. Sixteenths are far
// finer than the display can resolve, which is the point: nothing is lost to
// rounding on the way to the screen.
constexpr int kSubBits = 4;
constexpr double kSubScale = 16.0;

cv::Point atSub(double x, double y) {
    return cv::Point(static_cast<int>(std::lround(x * kSubScale)),
                     static_cast<int>(std::lround(y * kSubScale)));
}

// A line drawn as dots rather than a stroke, for one that is shown and not
// measured, so the eye can tell it at once from the one that is. `shown` is in
// display pixels: a dot every `every` of them along it, starting at its first
// point, each over a dark backing so it reads on pale cloth and dark alike.
void drawDotted(cv::Mat& disp, const std::vector<cv::Point2d>& shown, double every, double radius,
                const cv::Scalar& ink, const cv::Scalar& shadow) {
    if (shown.size() < 2 || !(every > 0.0)) return;
    const int back = static_cast<int>(std::lround((radius + 1.0) * kSubScale));
    const int front = static_cast<int>(std::lround(radius * kSubScale));
    const auto dot = [&](const cv::Point2d& q) {
        const cv::Point at = atSub(q.x, q.y);
        cv::circle(disp, at, back, shadow, -1, cv::LINE_AA, kSubBits);
        cv::circle(disp, at, front, ink, -1, cv::LINE_AA, kSubBits);
    };
    dot(shown.front());
    double toNext = every;
    for (std::size_t k = 1; k < shown.size(); ++k) {
        const cv::Point2d a = shown[k - 1];
        const cv::Point2d b = shown[k];
        const double seg = std::hypot(b.x - a.x, b.y - a.y);
        double t = 0.0;
        while (seg - t >= toNext) {
            t += toNext;
            dot(a + (b - a) * (t / seg));
            toNext = every;
        }
        toNext -= seg - t;
    }
}

// Tidy a traced boundary for the screen, and for the screen only.
//
// The outline a segmentation produces is honest and it is ugly. A thread, a
// speck of the mat that came through, a crease catching the light: each puts a
// spike of two or three points on a boundary of several thousand. The live pass
// runs decimated so its outline arrives as a staircase as well.
//
// None of that reaches a number. The measurement is taken from the refined
// boundary in millimetres, by code that has never seen this function, and every
// figure the station reports would be identical with it deleted. But the trace
// is the thing anyone standing at the bench actually looks at, and a jagged one
// reads as a machine that is not sure what it is looking at - which is the
// opposite of what it is.
//
// A short median pass takes the spikes out without moving the run they sit in,
// because a median is not pulled by the outlier the way a mean is. A shorter
// mean pass after it takes the stair edges off what is left. Both wrap, because
// a garment outline is a closed curve and a seam in the smoothing would be
// exactly the artefact this exists to remove.
// How many samples of this boundary span `arcPx` of it.
//
// The held outline is resampled every few full-resolution pixels and the live
// one every decimated cell, so the same window in samples is a different window
// on the cloth for each of them. Sizing it by arc length instead means the two
// are tidied to the same degree and the operator sees one outline behave one
// way, rather than a smooth one that goes ragged the moment a reading clears.
int windowForArc(const std::vector<cv::Point2d>& in, double arcPx) {
    const int n = static_cast<int>(in.size());
    if (n < 8) return 3;

    double perimeter = 0.0;
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& a = in[static_cast<std::size_t>(i)];
        const cv::Point2d& b = in[static_cast<std::size_t>((i + 1) % n)];
        perimeter += std::hypot(b.x - a.x, b.y - a.y);
    }
    const double spacing = perimeter / n;
    if (!(spacing > 1e-9)) return 3;

    int win = static_cast<int>(std::lround(arcPx / spacing)) | 1;
    win = std::max(3, std::min(win, 41));
    return std::min(win, std::max(3, (n / 4) | 1));
}

// The median of a circular window, per axis.
//
// First, because an isolated spike is not a shape and averaging one merely
// spreads it over its neighbours. A median removes anything narrower than half
// the window outright and leaves a corner where it finds one, which is the
// whole reason it comes before anything that averages.
std::vector<cv::Point2d> medianAlong(const std::vector<cv::Point2d>& in, int win) {
    const int n = static_cast<int>(in.size());
    const int half = win / 2;
    std::vector<cv::Point2d> out(static_cast<std::size_t>(n));
    std::vector<double> bx(static_cast<std::size_t>(win));
    std::vector<double> by(static_cast<std::size_t>(win));
    for (int i = 0; i < n; ++i) {
        for (int k = 0; k < win; ++k) {
            const cv::Point2d& p = in[static_cast<std::size_t>(((i + k - half) % n + n) % n)];
            bx[static_cast<std::size_t>(k)] = p.x;
            by[static_cast<std::size_t>(k)] = p.y;
        }
        std::nth_element(bx.begin(), bx.begin() + half, bx.end());
        std::nth_element(by.begin(), by.begin() + half, by.end());
        out[static_cast<std::size_t>(i)] = cv::Point2d(bx[static_cast<std::size_t>(half)],
                                                       by[static_cast<std::size_t>(half)]);
    }
    return out;
}

// A straight line fitted to the boundary either side of each point, and the
// point slid onto it.
//
// Not an average of positions, which is the important part. An average pulls a
// corner inward because it averages across the turn, and it pulls a straight
// edge nowhere but leaves it as wavy as it found it. This fits a line to the
// window and projects the point onto that line, so a run of boundary that is
// straight comes back straight and in the same place, a curve comes back as a
// curve, and a corner keeps far more of itself than a mean leaves it.
//
// This is what makes a straight edge look straight. Nothing is simplified and
// nothing is cut: every sample is still drawn, still on the cloth, just no
// longer carrying the part of its position that was noise.
std::vector<cv::Point2d> lineFitAlong(const std::vector<cv::Point2d>& in, int win) {
    const int n = static_cast<int>(in.size());
    const int half = win / 2;
    const double inv = 1.0 / win;
    std::vector<cv::Point2d> out(static_cast<std::size_t>(n));

    for (int i = 0; i < n; ++i) {
        double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0, syy = 0.0;
        for (int k = 0; k < win; ++k) {
            const cv::Point2d& p = in[static_cast<std::size_t>(((i + k - half) % n + n) % n)];
            sx += p.x;
            sy += p.y;
            sxx += p.x * p.x;
            sxy += p.x * p.y;
            syy += p.y * p.y;
        }
        const double mx = sx * inv, my = sy * inv;
        const double cxx = sxx * inv - mx * mx;
        const double cxy = sxy * inv - mx * my;
        const double cyy = syy * inv - my * my;
        const double mean = 0.5 * (cxx + cyy);
        const double diff = 0.5 * (cxx - cyy);
        const double major = mean + std::sqrt(diff * diff + cxy * cxy);

        // Either column of the adjugate spans the eigenspace; take whichever is
        // better conditioned, so an edge lying square to the frame is no worse
        // a case than one lying across it.
        cv::Point2d v(cxy, major - cxx);
        double len = std::hypot(v.x, v.y);
        const cv::Point2d w(major - cyy, cxy);
        const double lenW = std::hypot(w.x, w.y);
        if (lenW > len) {
            v = w;
            len = lenW;
        }
        const cv::Point2d& p = in[static_cast<std::size_t>(i)];
        if (len < 1e-12) {
            out[static_cast<std::size_t>(i)] = p;
            continue;
        }
        v = cv::Point2d(v.x / len, v.y / len);
        const double t = (p.x - mx) * v.x + (p.y - my) * v.y;
        out[static_cast<std::size_t>(i)] = cv::Point2d(mx + v.x * t, my + v.y * t);
    }
    return out;
}

// Tidying the drawn outline, for the picture and for nothing else.
//
// Display only, and worth being blunt about: the boundary a measurement is
// taken from never comes through here. That one goes to measureEdge exactly as
// the refinement produced it, artefacts and all. What is tidied is a copy on
// its way to the screen.
//
// A median to take the spikes out, then a local line fit to take the noise out
// of what is left. Every sample survives both and is still drawn, so the line
// stays on the cloth the whole way round: closely fitted is the point, and a
// simplified outline is not fitted, it is approximated.
//
// Everything here is in whatever units it is handed. The caller works in
// display pixels, so the windows mean what they look like on the screen.
std::vector<cv::Point2d> tidyForDisplay(const std::vector<cv::Point2d>& in, double smoothPx,
                                        double simplifyPx) {
    const int n = static_cast<int>(in.size());
    if (n < 24 || !(smoothPx > 0.0)) return in;

    const int win = windowForArc(in, smoothPx);
    if (n < 4 * win) return in;

    const std::vector<cv::Point2d> smooth = lineFitAlong(medianAlong(in, win), win);
    if (!(simplifyPx > 0.0)) return smooth;

    std::vector<cv::Point2f> narrow(smooth.size());
    for (std::size_t k = 0; k < smooth.size(); ++k) {
        narrow[k] = cv::Point2f(static_cast<float>(smooth[k].x),
                                static_cast<float>(smooth[k].y));
    }
    std::vector<cv::Point2f> simple;
    cv::approxPolyDP(narrow, simple, simplifyPx, true);
    // A closed outline needs three corners to be one. Below that the tolerance
    // has eaten the shape, so the smoothed boundary is what gets drawn.
    if (simple.size() < 3) return smooth;

    std::vector<cv::Point2d> out(simple.size());
    for (std::size_t k = 0; k < simple.size(); ++k) {
        out[k] = cv::Point2d(simple[k].x, simple[k].y);
    }
    return out;
}

// A closed boundary resampled to an even spacing along its length, for the
// picture only. The refined outline arrives far denser than the live one and
// unevenly spaced, so a window counted in samples would smooth the two by
// different amounts; resampled first, a window means the same stretch of line
// on the screen for both.
std::vector<cv::Point2d> resampleEvenly(const std::vector<cv::Point2d>& in, double stepPx) {
    const std::size_t n = in.size();
    if (n < 3 || !(stepPx > 0.0)) return in;
    std::vector<double> at(n + 1, 0.0);
    for (std::size_t k = 0; k < n; ++k) {
        const cv::Point2d& a = in[k];
        const cv::Point2d& b = in[(k + 1) % n];
        at[k + 1] = at[k] + std::hypot(b.x - a.x, b.y - a.y);
    }
    const double total = at[n];
    if (!(total > 2.0 * stepPx)) return in;
    std::vector<cv::Point2d> out;
    out.reserve(static_cast<std::size_t>(total / stepPx) + 1);
    std::size_t seg = 0;
    for (double s = 0.0; s < total; s += stepPx) {
        while (seg + 1 < n && at[seg + 1] < s) ++seg;
        const double len = at[seg + 1] - at[seg];
        const double t = len > 1e-12 ? (s - at[seg]) / len : 0.0;
        const cv::Point2d& a = in[seg];
        const cv::Point2d& b = in[(seg + 1) % n];
        out.emplace_back(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    }
    return out;
}

// How much of the screen the held outline is tidied over, in display pixels.
// Wider than the live outline's (display_outline_smooth_px) because the refined
// boundary carries the cloth's fine edge and, on a pale shirt, the glow it
// throws into the base between the grid lines; at this width the white line
// reads as evenly as the live one. Measured on the real white shirt on the blue
// cloth: a fifth less roughness, still 1.3 px from the shirt's edge on average
// (the same as at 20); wider starts cutting corners.
const double kHeldOutlineSmoothPx = 30.0;

// Every stroke on the feed, in one place.
//
// These are deliberately thin. The overlay is there to say where the machine
// thinks the cloth ends, and a four pixel line cannot say that to better than
// four pixels: it covers the very edge it is reporting, and an operator
// checking whether the outline is on the fabric ends up judging the middle of a
// stripe. One pixel, anti-aliased, sits on the boundary instead of hiding it.
struct Strokes {
    int outline = 1;   // the detected edge of the cloth
    int roi = 1;       // the outer boundary of the region being watched
    int span = 2;      // the edge a measurement was taken across
    int spanHalo = 4;  // its dark backing, so it reads over pale cloth
    int rail = 2;      // the mark showing which side the fold faces
    int railHalo = 5;
    int arrow = 1;
    int arrowHalo = 3;
    int corner = 3;    // radius of the dot at each end of the measured span
    int seed = 5;      // radius of the dot marking a point the operator put down
    int pointer = 9;   // and the ring following the finger before it lands
};

// The marks a step's reading put on the feed, written down so the page can put
// them back when the operator returns to that step.
//
// The feed itself still forgets them on every move - serviceStepChange sees to
// that. The page keeps a copy against the verdict they belong to, in the
// browser's own storage, and draws it over the picture only when that step is
// looked at again. So every point is a fraction of the picture, as a tap is,
// and each line carries the colour and weights it was drawn in here, in the
// display pixels of `size`, so it comes back looking the same.
struct Marks {
    explicit Marks(const cv::Size& picture) : size(picture) {}

    cv::Size size;
    vjson::Value items = vjson::Value::array();

    // A line as drawn: its width and dark backing, and the dot at each end with
    // the backing under that. `dotted`, when set, is the spacing of a line
    // drawn as dots (drawDotted), `width` and `halo` their diameters.
    void line(const std::vector<cv::Point2d>& shown, const cv::Scalar& bgr, int width,
              int halo = 0, int dot = 0, int dotHalo = 0, bool closed = false, double dotted = 0.0) {
        if (shown.size() < 2 || size.width <= 0 || size.height <= 0) return;
        vjson::Value m = vjson::Value::object();
        m.set("p", points(thin(shown, closed)));
        m.set("c", vjson::Value::str(css(bgr)));
        m.set("w", vjson::Value::integer(width));
        if (halo > 0) m.set("h", vjson::Value::integer(halo));
        if (dot > 0) m.set("d", vjson::Value::integer(dot));
        if (dotHalo > 0) m.set("dh", vjson::Value::integer(dotHalo));
        if (closed) m.set("closed", vjson::Value::boolean(true));
        if (dotted > 0.0) m.set("dots", vjson::Value::real(dotted));
        items.push(m);
    }

    // A number written on the picture, from its left end or centred on `at`.
    void label(const std::string& text, const cv::Point2d& at, bool centred,
               const cv::Scalar& bgr) {
        if (size.width <= 0 || size.height <= 0) return;
        vjson::Value m = vjson::Value::object();
        m.set("t", vjson::Value::str(text));
        m.set("p", points({at}));
        m.set("c", vjson::Value::str(css(bgr)));
        if (centred) m.set("a", vjson::Value::str("center"));
        items.push(m);
    }

    vjson::Value json() const {
        vjson::Value v = vjson::Value::object();
        v.set("w", vjson::Value::integer(size.width));
        v.set("h", vjson::Value::integer(size.height));
        v.set("items", items);
        return v;
    }

private:
    // Four places is a tenth of a display pixel, and keeps the status small.
    vjson::Value points(const std::vector<cv::Point2d>& shown) const {
        vjson::Value p = vjson::Value::array();
        for (const cv::Point2d& q : shown) {
            p.push(vjson::Value::real(std::round(q.x / size.width * 1e4) / 1e4));
            p.push(vjson::Value::real(std::round(q.y / size.height * 1e4) / 1e4));
        }
        return p;
    }

    // Never more than half a display pixel off the line as drawn, in a fraction
    // of the points: a held outline is thousands of them.
    static std::vector<cv::Point2d> thin(const std::vector<cv::Point2d>& in, bool closed) {
        if (in.size() <= 16) return in;
        std::vector<cv::Point2f> narrow(in.size());
        for (std::size_t k = 0; k < in.size(); ++k) {
            narrow[k] = cv::Point2f(static_cast<float>(in[k].x), static_cast<float>(in[k].y));
        }
        std::vector<cv::Point2f> simple;
        cv::approxPolyDP(narrow, simple, 0.5, closed);
        if (simple.size() < (closed ? 3u : 2u)) return in;
        std::vector<cv::Point2d> out(simple.size());
        for (std::size_t k = 0; k < simple.size(); ++k) out[k] = cv::Point2d(simple[k].x, simple[k].y);
        return out;
    }

    static std::string css(const cv::Scalar& bgr) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", static_cast<int>(bgr[2]) & 0xff,
                      static_cast<int>(bgr[1]) & 0xff, static_cast<int>(bgr[0]) & 0xff);
        return buf;
    }
};

// The same calibration read at a magnification s about the point the lens
// looks straight down at, which is the one MeasurementEngine::measure takes
// off an outline for the fold's height: pxToMm of this model at p is pxToMm of
// the original at centre + s (p - centre). The scale goes onto the far side of
// the homography, and the radial terms keep their shape by working at a
// radius 1/s as long. mmToPx inverts it exactly, so a seam found on the
// picture through this model lies on the same pixels as through the original.
PlaneModelPtr magnifiedModel(const PlaneModel& m, double s) {
    const cv::Point2d c = m.centre();
    const cv::Matx33d toTop(1.0 / s, 0.0, c.x * (1.0 - 1.0 / s),
                            0.0, 1.0 / s, c.y * (1.0 - 1.0 / s),
                            0.0, 0.0, 1.0);
    return std::make_shared<PlaneModel>(toTop * m.H(), m.k1(), m.k2(), c, m.norm() / s,
                                        m.scaleCorrection(), m.meta());
}

// The steps the operator works through, out of all the steps in the style file.
//
// The box round the collar replaced two steps that had the operator draw the
// collar by hand: pointing along the neckline (a curve step) and tapping the
// two shoulder points (a distance step). A style that has a collar step keeps
// both off the bench, unless the station is told to show them - one switch
// away, as a fallback, while the box reading proves itself. Nothing is taken
// out of the file: the steps stay in it, and a target edited from the bench is
// written back into the whole file, the steps not shown included.
//
// fileIndex[k] is where step k of what is returned sits in the file.
Template operatorSteps(const Template& full, bool showManual, std::vector<int>& fileIndex) {
    fileIndex.clear();
    bool hasCollar = false;
    for (const Step& s : full.steps) hasCollar = hasCollar || s.isCollar();
    Template view = full;
    view.steps.clear();
    for (std::size_t k = 0; k < full.steps.size(); ++k) {
        const Step& s = full.steps[k];
        if (hasCollar && !showManual && (s.isCurve() || s.isDistance())) continue;
        view.steps.push_back(s);
        fileIndex.push_back(static_cast<int>(k));
    }
    return view;
}

// The reason on a step the operator could not get a reading for and handed
// on to be measured by hand. Not something a reading ever says, so the page
// and the shift's figures can tell a flagged step from a failed one.
const char* const MANUAL_QC_REASON = "flagged for manual QC";

class Station {
public:
    Station(const Config& cfg, int jpegQuality = 72, bool showManual = false)
        : cfg_(cfg), jpegQuality_(jpegQuality), distanceMeasured_(cfg.camera_distance_mm.has_value()),
          showManual_(showManual) {
        status_ = vjson::Value::object();
        status_.set("state", vjson::Value::str("STARTING"));
        status_.set("ready", vjson::Value::boolean(false));
        exposureLevel_ = meteringTarget(cfg_);
        exposureDefaultLevel_ = cfg_.target_highlight_level * cfg_.clear_mat_headroom;

        if (cfg_.capture_enabled) {
            CaptureStore::Settings s;
            s.root = cfg_.capture_dir;
            s.jpeg_quality = cfg_.capture_jpeg_quality;
            s.queue = cfg_.capture_queue;
            s.keep_days = cfg_.capture_keep_days;
            s.max_gb = cfg_.capture_max_gb;
            s.prune_every = cfg_.capture_prune_every;
            captures_.reset(new CaptureStore(s, [this](const std::string& text, bool operatorToo) {
                if (operatorToo) {
                    log(text);
                } else {
                    std::printf("[%s] %s\n", timestampClock().c_str(), text.c_str());
                    std::fflush(stdout);
                }
            }));
        }

        templates_ = offeredTemplates();
        if (!cfg_.template_file.empty()) {
            std::string error;
            if (loadTemplateFile(cfg_.template_file, error)) {
                log("Checking " + template_.style + ", " +
                    std::to_string(template_.steps.size()) + " steps per shirt");
            } else {
                // Not fatal. The station still measures, and the operator can
                // pick a working style from the page. Refusing to start would
                // leave a bench idle over a typo in a JSON file.
                log("Could not load " + cfg_.template_file + ": " + error);
                log("Measuring without a verdict until a style is chosen");
            }
        }
    }

    ~Station() { stop(); }

    void log(const std::string& text) {
        const std::string stamp = timestampClock();
        {
            std::lock_guard<std::mutex> guard(lock_);
            boot_.push_back({stamp, text});
            while (boot_.size() > 40) boot_.pop_front();
        }
        std::printf("[%s] %s\n", stamp.c_str(), text.c_str());
        std::fflush(stdout);
    }

    void start() {
        if (captures_) {
            log("Saving a picture of every verdict under " + cfg_.capture_dir + "/");
            captures_->start();
        }
        thread_ = std::thread(&Station::run, this);
    }

    void stop() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
        // Nothing more can be handed over once the capture thread has ended,
        // so what is waiting is written out before the station goes.
        if (captures_) captures_->stop();
        if (source_) source_->stop();
        // After the stream, not before: a recalibration still waiting for the
        // mat to settle gives up as soon as frames stop arriving, so this join
        // is short rather than sitting out the rest of its wait.
        if (calibThread_.joinable()) calibThread_.join();
        if (camera_) camera_->destroy();
    }

    void command(const std::string& name, const std::string& arg) {
        std::lock_guard<std::mutex> guard(lock_);
        command_ = name;
        commandArg_ = arg;
    }

    // -- recalibrating from the page --
    //
    // Taken here, on the HTTP thread, rather than through the command queue.
    // The page is told straight away whether it was accepted and the status
    // shows the run from that moment, so nobody presses the button and then
    // watches nothing happen. And it cannot be lost: the queue holds a single
    // command, and whatever the operator pressed next used to replace it. The
    // capture thread starts the work on its next pass.
    bool requestCalibration(std::string& why, bool& busy) {
        std::lock_guard<std::mutex> guard(lock_);
        busy = calib_.active;
        if (!live_) {
            why = "the station has not finished starting";
            return false;
        }
        if (calib_.active) {
            why = "a recalibration is already running";
            return false;
        }
        calib_.active = true;
        calib_.started = false;
        calib_.phase = "queued";
        calib_.done = 0;
        calib_.needed = 0;
        calib_.hidden = 0.0;
        calib_.startedAt = nowSeconds();
        ++calib_.seq;
        calibCancel_ = false;
        return true;
    }

    // Straight from the HTTP thread, like the request: only a run still
    // waiting for the base to be cleared can be called off, because nothing
    // has been touched yet. Past that it is seconds from done.
    bool cancelCalibration(std::string& why) {
        std::lock_guard<std::mutex> guard(lock_);
        if (!calib_.active) {
            why = "no recalibration is running";
            return false;
        }
        calibCancel_ = true;
        return true;
    }

    // The camera's exposure, from the page's Settings: the level the empty base
    // is metered to, and a recalibration to meter it and retake the base at it -
    // the path --exposure-target and Recalibrate already take between them.
    // Asked here, on the HTTP thread, as Recalibrate is, so the page is told at
    // once whether it was taken. The level itself is put into service on the
    // capture thread just before the recalibration starts (applyExposureLevel).
    bool requestExposureLevel(double level, std::string& why, bool& busy) {
        busy = false;
        if (!(level >= 40.0 && level <= 240.0)) {
            why = "the level must be between 40 and 240";
            return false;
        }
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (calib_.active) {
                busy = true;
                why = "a recalibration is already running";
                return false;
            }
            pendingExposureLevel_ = level;
        }
        if (requestCalibration(why, busy)) return true;
        std::lock_guard<std::mutex> guard(lock_);
        pendingExposureLevel_.reset();
        return false;
    }

    // A new shift, from the admin panel. Straight from the HTTP thread: it only
    // clears the shift's figures (see observeShift). The shirt on the bench
    // carries on and counts in the new shift when it is finished.
    void newShift() {
        std::lock_guard<std::mutex> guard(lock_);
        shift_.clear();
        shirtRecord_ = shirtDone_ ? kCountedBefore : -1;
        shiftStartedAt_ = timestampClock();
        shiftJson_ = shiftJsonLocked();
    }

    // The newest picture, if it is newer than `after`. What the MJPEG stream
    // needs and nothing else: it polls fifty times a second per viewer, and
    // snapshot() copies the whole status and results each time, under the lock
    // the capture thread publishes through, to have them thrown away.
    bool frameAfter(long long after, std::vector<unsigned char>& jpeg, long long& jpegSeq) {
        std::lock_guard<std::mutex> guard(lock_);
        if (jpeg_.empty() || jpegSeq_ == after) return false;
        jpeg = jpeg_;
        jpegSeq = jpegSeq_;
        return true;
    }

    void snapshot(std::vector<unsigned char>& jpeg, long long& jpegSeq, vjson::Value& status,
                  vjson::Value& results) {
        std::lock_guard<std::mutex> guard(lock_);
        jpeg = jpeg_;
        jpegSeq = jpegSeq_;
        status = status_;
        // Added here rather than in publish(), which only runs when a frame
        // arrives. At two frames a second that would leave the page up to half
        // a second behind the button it has just pressed.
        status.set("calibration", calibrationJsonLocked());
        status.set("scale_check", scaleCheckJsonLocked());
        // The shift's figures, kept here and not in any one browser, so every
        // screen on the station agrees. See observeShift.
        status.set("stats", shiftJson_);
        status.set("exposure", exposureJsonLocked());
        results = vjson::Value::array();
        for (const vjson::Value& r : results_) results.push(r);
    }

    // -- editing targets from the bench --
    //
    // Runs on the HTTP thread rather than through the command queue, because
    // the operator needs to be told whether it saved. It touches only the
    // template, which the capture thread reads under the same lock.
    bool editStep(int index, double target, double minus, double plus, std::string& error) {
        std::string path;
        std::string name;
        Template edited;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_) {
                error = "no style is loaded";
                return false;
            }
            if (index < 0 || index >= static_cast<int>(template_.steps.size())) {
                error = "there is no such step";
                return false;
            }
            if (!(target > 0.0)) {
                error = "the target must be a positive length in millimetres";
                return false;
            }
            Step& s = template_.steps[static_cast<std::size_t>(index)];
            s.target_mm = target;
            s.tol_minus_mm = -std::fabs(minus);
            s.tol_plus_mm = std::fabs(plus);
            // Into the whole file, and saved as the whole file: a step kept off
            // the bench is still in it, and must still be in it afterwards.
            const int f = index < static_cast<int>(fileIndex_.size())
                              ? fileIndex_[static_cast<std::size_t>(index)] : -1;
            if (f < 0 || f >= static_cast<int>(fileTemplate_.steps.size())) {
                error = "the step is not in the style file";
                return false;
            }
            Step& fs = fileTemplate_.steps[static_cast<std::size_t>(f)];
            fs.target_mm = s.target_mm;
            fs.tol_minus_mm = s.tol_minus_mm;
            fs.tol_plus_mm = s.tol_plus_mm;
            name = s.name;
            path = fileTemplate_.path;
            edited = fileTemplate_;
        }

        if (!saveTemplate(edited, path, error)) return false;
        log("Step " + std::to_string(index + 1) + " (" + name + ") set to " +
            vjson::formatFixed(target, 1) + " mm, saved to " + path);
        return true;
    }

    // The last picture of a step of the shirt on the bench, for the page:
    // 200 with the JPEG, 202 while it is still being written, 404 for none.
    // `step` counts from one, as the page does. On the HTTP thread.
    int captureImage(int garment, int step, std::string& body) const {
        if (!captures_ || garment < 1 || step < 1) return 404;
        unsigned long long id = 0;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (garment != garment_) return 404;
            const auto it = captureOf_.find({garment, step - 1});
            if (it == captureOf_.end()) return 404;
            id = it->second.id;
        }
        std::string rel;
        const CaptureStore::State st = captures_->state(id, &rel);
        if (st == CaptureStore::State::Saving) return 202;
        if (st != CaptureStore::State::Saved) return 404;
        return readWholeFile((std::filesystem::path(captures_->root()) / rel).string(), body) ? 200 : 404;
    }

    // The file backing the current step's reference picture, if there is one.
    bool referenceImagePath(int index, std::string& path) const {
        std::lock_guard<std::mutex> guard(lock_);
        if (!haveTemplate_) return false;
        if (index < 0 || index >= static_cast<int>(template_.steps.size())) return false;
        const std::string& rel = template_.steps[static_cast<std::size_t>(index)].reference_image;
        if (!safeRelativePath(rel)) return false;
        path = (std::filesystem::path(cfg_.templates_dir) / rel).string();
        return true;
    }

    // -- adding a style from the bench --
    //
    // A new style is made from one already on the bench. Its steps - how each
    // is folded and laid, how it is found, its picture and its words - are
    // copied as they are, and only what the new garment changes is typed in:
    // which of those steps it has, and what each should come to. It is written
    // as a new file beside the others and never over one; an existing style
    // only ever changes a step at a time, through Save to the style file.
    //
    // Both run on the HTTP thread, like editStep, so the page is told at once.
    // A style is asked for by name and looked up among the files the station
    // listed itself, and a new file's name is made from letters, digits and
    // dashes, so nothing a browser sends decides what is read or written.

    // The steps of one style, for the page to make a new style from. on_bench
    // is whether the operator is taken through the step (see operatorSteps).
    bool styleStepsJson(const std::string& style, vjson::Value& out, std::string& error) {
        const std::string path = stylePath(style);
        if (path.empty()) {
            error = "there is no style called " + style;
            return false;
        }
        Template t;
        if (!loadTemplate(path, t, error)) return false;
        bool show = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            show = showManual_;
        }
        std::vector<int> index;
        operatorSteps(t, show, index);
        std::vector<char> onBench(t.steps.size(), 0);
        for (int f : index) {
            if (f >= 0 && f < static_cast<int>(t.steps.size())) onBench[static_cast<std::size_t>(f)] = 1;
        }
        out = vjson::Value::object();
        out.set("style", vjson::Value::str(t.style));
        vjson::Value steps = vjson::Value::array();
        for (std::size_t k = 0; k < t.steps.size(); ++k) {
            const Step& st = t.steps[k];
            vjson::Value e = vjson::Value::object();
            e.set("name", vjson::Value::str(st.name));
            e.set("kind", vjson::Value::str(st.kind));
            e.set("target_mm", vjson::Value::real(st.target_mm));
            e.set("tol_minus_mm", vjson::Value::real(st.tol_minus_mm));
            e.set("tol_plus_mm", vjson::Value::real(st.tol_plus_mm));
            e.set("on_bench", vjson::Value::boolean(onBench[k] != 0));
            steps.push(e);
        }
        out.set("steps", steps);
        return true;
    }

    // A new style called `name`, with the steps of `base` that `steps` lists,
    // each as "step_name:target:minus:plus" in millimetres, separated by
    // commas. They are kept in the base style's order, whatever order they
    // come in. The name the style was saved under comes back in `saved`.
    bool addStyle(const std::string& name, const std::string& base, const std::string& steps,
                  std::string& saved, std::string& error) {
        std::string style = name;
        while (!style.empty() && std::isspace(static_cast<unsigned char>(style.front()))) style.erase(style.begin());
        while (!style.empty() && std::isspace(static_cast<unsigned char>(style.back()))) style.pop_back();
        if (style.empty()) {
            error = "the new style needs a name";
            return false;
        }
        if (style.size() > 60) {
            error = "keep the name to 60 characters";
            return false;
        }
        const auto lower = [](std::string t) {
            for (char& c : t) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return t;
        };
        rescanTemplates();
        {
            std::lock_guard<std::mutex> guard(lock_);
            for (const TemplateEntry& t : templates_) {
                if (lower(t.style) == lower(style)) {
                    error = "there is already a style called " + t.style;
                    return false;
                }
            }
        }
        const std::string basePath = stylePath(base);
        if (basePath.empty()) {
            error = "there is no style called " + base + " to copy the steps from";
            return false;
        }
        Template from;
        if (!loadTemplate(basePath, from, error)) return false;

        struct Wanted {
            double target = 0.0;
            double minus = 0.0;
            double plus = 0.0;
        };
        std::map<std::string, Wanted> wanted;
        std::size_t at = 0;
        while (at <= steps.size()) {
            const std::size_t end = std::min(steps.find(',', at), steps.size());
            const std::string item = steps.substr(at, end - at);
            at = end + 1;
            if (item.empty()) continue;
            std::vector<std::string> part;
            std::size_t p = 0;
            while (p <= item.size()) {
                const std::size_t q = std::min(item.find(':', p), item.size());
                part.push_back(item.substr(p, q - p));
                p = q + 1;
            }
            if (part.size() != 4) {
                error = "could not read the step \"" + item + "\"";
                return false;
            }
            double v[3] = {0.0, 0.0, 0.0};
            for (int k = 0; k < 3; ++k) {
                const std::string& text = part[static_cast<std::size_t>(k) + 1];
                char* stop = nullptr;
                v[k] = std::strtod(text.c_str(), &stop);
                if (text.empty() || stop == text.c_str() || *stop != '\0' || !std::isfinite(v[k])) {
                    error = "\"" + text + "\" is not a length for " + part[0];
                    return false;
                }
            }
            if (!(v[0] > 0.0 && v[0] < 10000.0)) {
                error = "the target for " + part[0] + " must be a length in millimetres above 0";
                return false;
            }
            if (std::fabs(v[1]) >= 1000.0 || std::fabs(v[2]) >= 1000.0) {
                error = "the tolerance for " + part[0] + " is out of range";
                return false;
            }
            if (wanted.count(part[0])) {
                error = part[0] + " is listed twice";
                return false;
            }
            bool known = false;
            for (const Step& st : from.steps) known = known || st.name == part[0];
            if (!known) {
                error = from.style + " has no step called " + part[0];
                return false;
            }
            wanted[part[0]] = Wanted{v[0], std::fabs(v[1]), std::fabs(v[2])};
        }

        Template made;
        made.style = style;
        made.source_units = "mm";
        for (const Step& st : from.steps) {
            const auto it = wanted.find(st.name);
            if (it == wanted.end()) continue;
            Step copy = st;
            copy.target_mm = it->second.target;
            copy.tol_minus_mm = -it->second.minus;
            copy.tol_plus_mm = it->second.plus;
            made.steps.push_back(copy);
        }
        if (made.steps.empty()) {
            error = "choose at least one step for the new style";
            return false;
        }

        std::string slug;
        for (char c : lower(style)) {
            if (std::isalnum(static_cast<unsigned char>(c)) && static_cast<unsigned char>(c) < 128) {
                slug += c;
            } else if (!slug.empty() && slug.back() != '-') {
                slug += '-';
            }
        }
        while (!slug.empty() && slug.back() == '-') slug.pop_back();
        if (slug.empty()) slug = "style";
        const std::filesystem::path dir(cfg_.templates_dir);
        std::filesystem::path file = dir / (slug + ".json");
        std::error_code ec;
        for (int n = 2; std::filesystem::exists(file, ec); ++n) {
            file = dir / (slug + "-" + std::to_string(n) + ".json");
        }
        made.path = file.string();
        if (!saveTemplate(made, made.path, error)) return false;
        rescanTemplates();
        log("Added the style " + style + ": " + std::to_string(made.steps.size()) +
            " steps, copied from " + from.style + ", saved to " + made.path);
        saved = style;
        return true;
    }

private:
    // -- startup --

    std::unique_ptr<FrameSource> openCamera() {
        if (!ARENA_AVAILABLE) {
            throw std::runtime_error(
                "This build has no camera backend. Rebuild with -DVERIMEASURE_WITH_ARENA=ON and "
                "-DARENA_SDK_ROOT=<path to the Arena SDK>. There is no fallback source: every "
                "measurement comes off the camera.");
        }

        camera_.reset(new Camera());
        camera_->connect();
        camera_->configureStream();
        camera_->disableAutoAdjustments();
        camera_->tuneStream(cfg_);
        camera_->startStream(cfg_.stream_buffers);

        std::unique_ptr<CameraStream> stream(new CameraStream(*camera_, cfg_));
        stream->start();

        cv::Mat frame;
        long long seq = 0;
        double age = 0.0;
        if (!stream->read(frame, seq, age, 15.0)) {
            throw std::runtime_error("no frames arrived after starting the stream");
        }
        reportBandwidth(frame.size(), frame.channels(), cfg_);
        setUpExposure(*stream);
        return std::unique_ptr<FrameSource>(stream.release());
    }

    void bootSequence() {
        log("Connecting to the camera");
        source_ = openCamera();

        log("Waiting for the surface to be still");
        cv::Mat background = captureStaticBackground(*source_, cfg_);
        log("Background reference captured");

        PlaneModelPtr model = loadCalibration(cfg_);
        std::string calibError;
        if (!model) {
            log("Calibrating from the printed grid");
            model = autoCalibrate(background, cfg_, true, &calibError);
            if (model) saveCalibration(*model, cfg_);
        }
        if (!model) {
            log("Calibration failed" + (calibError.empty() ? std::string() : ": " + calibError) +
                ". Clear the surface and press Recalibrate.");
        } else {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Calibrated, residual %.3f mm",
                          model->metaNumber("rmse_mm", 0.0));
            log(buf);
        }

        RoiBox roi;
        bool haveRoi = false;
        bool roiFollowedMat = false;
        if (model) {
            haveRoi = roiFromModel(*model, background.size(), cfg_, roi, &background,
                                   &roiFollowedMat);
        }
        if (!haveRoi) haveRoi = loadRoi(cfg_, roi);
        if (!haveRoi) {
            roi = RoiBox{0, 0, background.cols, background.rows};
            haveRoi = true;
        }
        saveRoi(roi, cfg_);
        if (model && haveRoi) logRailSource(roiFollowedMat);

        // The background is whatever was on the base at the moment of starting,
        // and a garment left there becomes part of it: nothing laid in its place
        // is seen until the base is retaken. Not refused - a start that waited
        // would look like one that hung - but said first thing.
        if (model) {
            const double hidden = gridHiddenFraction(
                background, model->detectionNumber("pitch_px", 0.0), cfg_, &roi);
            if (hidden > cfg_.base_hidden_max_fraction) {
                log("WARNING: something was lying on the base when the station started (" +
                    percentText(hidden) +
                    " of the printed grid hidden), so it is part of the background now. Lift it "
                    "off and press Recalibrate before measuring.");
            }
        }

        if (!cfg_.camera_distance_mm.has_value() && model) {
            double d = 0.0;
            bool derived = false;
            if (estimateCameraDistance(*model, background.size(), cfg_, d, &derived) && d != 0.0) {
                cfg_.camera_distance_mm = d;
                reportWorkingDistance(d, derived, *model, background.size());
            } else {
                log("The working distance could not be worked out, so nothing is corrected for "
                    "the thickness of the cloth. Pass --camera-distance-mm with a tape measure "
                    "reading from the lens to the mat.");
            }
        } else if (model && distanceMeasured_) {
            double d = 0.0;
            bool derived = false;
            const bool fromLens =
                estimateCameraDistance(*model, background.size(), cfg_, d, &derived) && !derived;
            compareMeasuredDistance(fromLens ? d : 0.0);
        }
        reportFoldHeight();

        engine_.reset(new MeasurementEngine(cfg_, model, roi, haveRoi));
        engine_->setBackground(background);
        rebuildDatum();
        {
            std::lock_guard<std::mutex> guard(lock_);
            live_ = true;
        }
        log("Ready");
    }

    // The exposure, the same on every start.
    //
    // It used to be metered afresh on every start, from whatever was under the
    // camera at the time and from wherever the last run had left it, by a
    // search that judged each step on the very next frame - which at two frames
    // a second is usually one taken before the step. So it stopped anywhere
    // from a third of the right exposure to three times it, and each start
    // began from where the last one stopped: a bright picture on one start and
    // a dark one on the next, ten times apart on the same bench under the same
    // light. Now it is set once from a clear mat by a search that waits for the
    // change to show (meterExposure), remembered in bench.json, and used as it
    // is until Recalibrate sets it again.
    void setUpExposure(FrameSource& stream) {
        if (!cfg_.auto_expose_on_start) return;
        BenchExposure saved;
        if (loadBenchExposure(cfg_, saved)) {
            if (camera_->setExposure(saved.us)) {
                log("Exposure " + exposureText(saved.us) + (saved.byHand ? ", set by hand" : "") +
                    ", as saved for this bench");
                noteExposure(saved.us, saved.byHand);
                checkExposureLight(stream, saved);
                return;
            }
            log("The saved exposure of " + exposureText(saved.us) +
                " could not be set on this camera, so it is being set again from the mat");
        } else {
            log("Setting the exposure from the mat");
        }
        double locked = 0.0;
        double level = 0.0;
        if (!autoSetExposure(*camera_, stream, cfg_, locked, &level)) return;
        noteExposure(locked, false);
        BenchExposure e;
        e.us = locked;
        e.level = level;
        if (saveBenchExposure(cfg_, e)) {
            log("Exposure set to " + exposureText(locked) +
                " and saved for this bench: every start uses it, and Recalibrate sets it again");
        } else {
            log("Exposure set to " + exposureText(locked) + ", but " + cfg_.bench_file +
                " could not be written, so it will be set again on the next start");
        }
    }

    // A saved exposure is right only under the light it was saved in. A
    // picture far brighter or darker than it gave then is a changed light, or
    // one not on yet - said once, and left to the operator, because a shirt
    // left on the mat changes the picture too.
    void checkExposureLight(FrameSource& stream, const BenchExposure& saved) {
        if (saved.level <= 0.0) return;
        double level = 0.0;
        if (!steadyHighlightLevel(stream, cfg_, level) || level <= 0.0) return;
        const double ratio = level / saved.level;
        if (ratio > 1.0 / 2.5 && ratio < 2.5) return;
        log("WARNING: the picture is " + vjson::formatFixed(ratio > 1.0 ? ratio : 1.0 / ratio, 1) +
            " times " + (ratio > 1.0 ? "brighter" : "darker") +
            " than when this exposure was set. If the light has changed, clear the mat and press "
            "Recalibrate to set the exposure for it; if it is not on yet, turn it on.");
    }

    // Recalibrating: the mat is clear for it, which is what the exposure is
    // set on, so it is set again first - unless it was set by hand. Runs on the
    // calibration thread, as the grab thread runs, which is how it has always
    // been set at startup too.
    struct ExposureChange {
        bool changed = false;
        double before = 0.0;
        double after = 0.0;
        double level = 0.0;
        // The search ran out of exposure with the picture still far short of
        // where it should be: the light is off, or nearly. See calibrationWorker.
        bool tooDark = false;
    };
    ExposureChange remeterExposure(const Config& cfg) {
        ExposureChange out;
        if (!camera_ || !cfg.auto_expose_on_start) return out;
        BenchExposure saved;
        if (loadBenchExposure(cfg, saved) && saved.byHand) return out;
        double before = 0.0;
        double lo = 0.0;
        double hi = 0.0;
        if (!camera_->exposureRange(before, lo, hi)) return out;
        {
            std::lock_guard<std::mutex> guard(lock_);
            calib_.phase = "exposure";
        }
        double locked = 0.0;
        double level = 0.0;
        ExposureSearch search;
        const bool ok = autoSetExposure(*camera_, *source_, cfg, locked, &level, &search);
        {
            std::lock_guard<std::mutex> guard(lock_);
            calib_.phase = "settle";
        }
        if (!ok) return out;
        out.changed = true;
        out.before = before;
        out.after = locked;
        out.level = level;
        // Under the bench's lamp the exposure sits far inside its range; run
        // out at the long end and still short of the target, the lamp is off.
        // Measured with it off: 537 ms, the longest there is, reached 119
        // against 140 in the room's light.
        const double target = meteringTarget(cfg);
        out.tooDark = search.clamped && locked >= 0.999 * hi && level < 0.9 * target;
        return out;
    }

    // A recalibration that failed leaves everything as it was, the exposure
    // included: the background still in use was taken at the old one.
    void restoreExposure(const ExposureChange& ex) {
        if (!ex.changed || !camera_ || ex.after == ex.before) return;
        if (camera_->setExposure(ex.before)) {
            log("The exposure is back to " + exposureText(ex.before) + ", as it was");
            noteExposure(ex.before, false);
        }
    }

    // The exposure the camera is running at, as the page's Settings show it.
    void noteExposure(double us, bool byHand) {
        std::lock_guard<std::mutex> guard(lock_);
        exposureUs_ = us;
        exposureByHand_ = byHand;
    }

    // A metering level asked for from the page, put into service here on the
    // capture thread just before the recalibration it came with copies the
    // config (serviceCalibration). Saved exactly as --exposure-target saves it,
    // so every start and every Recalibrate meter the empty base to it from now
    // on. An exposure set by hand is handed back to the station, keeping its
    // value, so that this recalibration meters afresh instead of keeping it.
    void applyExposureLevel(double level) {
        const std::string line = settleExposureTarget(cfg_, true, level);
        BenchExposure saved;
        bool freed = false;
        if (loadBenchExposure(cfg_, saved) && saved.byHand) {
            saved.byHand = false;
            freed = saveBenchExposure(cfg_, saved);
        }
        {
            std::lock_guard<std::mutex> guard(lock_);
            exposureLevel_ = meteringTarget(cfg_);
            if (freed) exposureByHand_ = false;
        }
        log("Exposure: the empty base is metered to " + std::to_string(std::lround(level)) +
            " now; recalibrating to set the camera for it");
        if (!line.empty() && line.find("could not") != std::string::npos) log(line);
        if (freed) log("The exposure set by hand is handed back to the station and metered from the base");
    }

    // What Settings shows of the exposure. Call with lock_ held.
    vjson::Value exposureJsonLocked() const {
        vjson::Value v = vjson::Value::object();
        v.set("us", exposureUs_ > 0.0 ? vjson::Value::real(exposureUs_) : vjson::Value::null());
        v.set("text", vjson::Value::str(exposureUs_ > 0.0 ? exposureText(exposureUs_) : std::string()));
        v.set("by_hand", vjson::Value::boolean(exposureByHand_));
        v.set("level", vjson::Value::real(exposureLevel_));
        v.set("default_level", vjson::Value::real(exposureDefaultLevel_));
        v.set("pending", vjson::Value::boolean(pendingExposureLevel_.has_value()));
        return v;
    }

    // A recalibration that took: the exposure it was set to is this bench's
    // from now on.
    void keepExposure(const ExposureChange& ex) {
        if (!ex.changed) return;
        BenchExposure e;
        e.us = ex.after;
        e.level = ex.level;
        noteExposure(ex.after, false);
        const std::string was = std::abs(ex.after - ex.before) > 0.02 * ex.before
                                    ? " (it was " + exposureText(ex.before) + ")"
                                    : "";
        if (saveBenchExposure(cfg_, e)) {
            log("Exposure " + exposureText(ex.after) + " for the light on the mat now" + was +
                ", saved for this bench");
        } else {
            log("Exposure " + exposureText(ex.after) + was + ", but " + cfg_.bench_file +
                " could not be written, so the next start sets it again");
        }
    }

    // The rail the operator lays the fold against, and how much room it leaves.
    //
    // Reported out loud because it is the hard limit on what this bench can
    // measure, and it is not obvious: a view is usually much wider than it is
    // tall, so a rail along the bottom leaves far less reach than one down the
    // left. A target longer than the reach is not a tolerance problem, it is a
    // measurement the rig cannot take.
    // Where the reach the operator is about to be quoted actually came from.
    //
    // The number on its own is not enough to act on. A rail that stops short
    // because the mat's border is printed in a colour of its own is fixed by
    // sliding the mat; one that stops short because the camera cannot see the
    // border is fixed by moving the camera, and nothing but knowing which is
    // which tells the two apart.
    void logRailSource(bool followedMat) {
        if (followedMat) {
            log("Working area reaches the edge of the mat");
        } else if (cfg_.roi_follow_mat) {
            log("The edge of the mat was not found, so the working area stops just past the "
                "printed grid");
        }
    }

    // Where the working distance came from, and what it is worth.
    //
    // Said out loud, with the lens it implies, because a derived number is one
    // the operator can check without instruments: a bench that reports a 13 mm
    // lens at three quarters of a metre is describing something recognisable,
    // and one that reports a 90 mm lens at four metres is not and should be
    // overridden with a tape measure.
    void reportWorkingDistance(double d, bool derived, const PlaneModel& model, cv::Size frame) {
        // Inverting estimateCameraDistance: it builds d as f x (1 + 1/m), so
        // the lens it implies is d divided by that same factor. Taken from this
        // model rather than from a constant, so it stays true when the camera is
        // moved. Dividing by the magnification alone - which is what this did
        // until the distance itself was corrected - reports the lens one focal
        // length too long, and the number an operator is meant to sanity-check
        // against is the one number that must not be quietly off.
        const std::pair<double, double> scale =
            model.localMmPerPx(cv::Point2d(frame.width / 2.0, frame.height / 2.0));
        const double mmPerPx = (scale.first + scale.second) / 2.0;
        const double lensMm =
            (mmPerPx > 0.0) ? d * cfg_.pixel_size_mm / (cfg_.pixel_size_mm + mmPerPx) : 0.0;
        std::string line = "Working distance " + vjson::formatFixed(d, 0) + " mm";
        if (derived) {
            // Loud, because this is the unreliable path and it is unreliable in
            // the direction that reads short. See estimateCameraDistance.
            line += ", guessed from the calibration (about a " + vjson::formatFixed(lensMm, 0) +
                    " mm lens) - CHECK IT. A squarely mounted camera makes this guess worse, not "
                    "better, and it can be badly wrong. Pass --focal-length-mm once with the "
                    "figure on the lens and it is remembered for this bench, or "
                    "--camera-distance-mm with a tape measure.";
        } else {
            line += ", from the " + vjson::formatFixed(cfg_.lens_focal_length_mm.value_or(lensMm), 1) +
                    " mm lens.";
        }
        log(line);
    }

    // A working distance measured with a tape, with what the lens makes of the
    // calibration said beside it.
    //
    // The two are independent, which is what makes the pair worth reading. The
    // lens figure is the focal length times one over the magnification, and
    // the magnification comes from the printed grid being 10 mm a square: a
    // grid printed a per cent small reads the lens distance a per cent long,
    // and every reading a per cent long with it, while the grid still agrees
    // perfectly with itself. On the rigid mat the tape and the lens agreed to
    // about a per cent. `fromLens` is zero when there is no lens to say.
    void compareMeasuredDistance(double fromLens) {
        const double measured = cfg_.camera_distance_mm.value_or(0.0);
        if (measured <= 0.0) return;
        std::string line = "Working distance " + vjson::formatFixed(measured, 0) + " mm, as measured";
        if (fromLens > 0.0) {
            const double off = 100.0 * (fromLens / measured - 1.0);
            line += "; the lens and the calibration put it at " + vjson::formatFixed(fromLens, 0) + " mm";
            if (std::abs(off) > 1.5) {
                log("WARNING: " + line + " (" + (off >= 0.0 ? "+" : "") + vjson::formatFixed(off, 1) +
                    "%). If the tape is right, the printed grid is not true to size and readings come "
                    "out about that much " + (off >= 0.0 ? "long" : "short") +
                    ": check the grid with a steel rule and correct it with Verify scale.");
                return;
            }
        }
        log(line);
    }

    // What the cloth's thickness is costing, while it is still zero.
    //
    // This is the largest correctable error left on this bench and it is
    // completely invisible: every reading looks reasonable, passes or fails
    // plausibly, and is simply a fixed percentage long. Saying nothing about it
    // is how it survives, so it is said once at startup with the size of it
    // worked out for the steps actually loaded.
    void reportFoldHeight() {
        double d = cfg_.camera_distance_mm.value_or(0.0);
        if (d <= 0.0) return;

        bool anyHeight = cfg_.object_height_mm > 0.0;
        double worstTarget = 0.0;
        {
            std::lock_guard<std::mutex> guard(lock_);
            for (const Step& s : template_.steps) {
                if (s.height_mm > 0.0) anyHeight = true;
                worstTarget = std::max(worstTarget, s.target_mm);
            }
        }
        if (anyHeight) return;

        // A folded jersey hem is a few millimetres; ten is a fair middle for a
        // fold of any kind, and it is only being used to show the scale of the
        // thing, not to correct anything.
        const double example = 10.0;
        const double pct = 100.0 * example / d;
        std::string line = "No fold thickness is set, so every reading is about " +
                           vjson::formatFixed(pct, 1) +
                           "% long for a 10 mm fold - the cloth's top face is nearer the lens "
                           "than the mat and images larger";
        if (worstTarget > 0.0) {
            line += ", which is " + vjson::formatFixed(worstTarget * example / d, 1) +
                    " mm on the " + vjson::formatFixed(worstTarget, 0) + " mm step";
        }
        line += ". Put \"height_mm\" on each step, or pass --object-height-mm.";
        log(line);
    }

    // The fold's thickness belongs to the step, not to the bench, so it follows
    // the step. Cheap enough to settle every pass rather than hooking every
    // place the step can change.
    void applyStepHeight() {
        if (!engine_) return;
        double h = cfg_.object_height_mm;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (haveTemplate_ && stepIndex_ >= 0 &&
                stepIndex_ < static_cast<int>(template_.steps.size())) {
                const double stepH = template_.steps[static_cast<std::size_t>(stepIndex_)].height_mm;
                if (stepH != 0.0) h = stepH;
            }
            // A reference being read for Check scale is read on the base's
            // plane; its own thickness is allowed for when it is applied.
            if (scaleCheck_.active) h = 0.0;
        }
        if (engine_->config().object_height_mm != h) engine_->config().object_height_mm = h;
    }

    // The frame-edge check, per step. A whole shirt laid flat - for the collar,
    // the perimeter or the neckline - runs off the picture on three sides at
    // this camera distance, and the check (refuse_frame_outline) turned every
    // such lay away, so the collar could never be boxed. On those steps a lay
    // is refused only when its outline runs along all four edges: the base
    // read as the garment, which is what the check is there to catch. Fold
    // steps keep the rule as configured. Called every pass, like the height.
    void applyStepEdgeRule() {
        if (!engine_) return;
        int sides = cfg_.outline_refuse_sides;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (haveTemplate_ && stepIndex_ >= 0 &&
                stepIndex_ < static_cast<int>(template_.steps.size())) {
                const Step& s = template_.steps[static_cast<std::size_t>(stepIndex_)];
                if (s.isPointed() || s.isPerimeter()) sides = std::max(sides, 4);
            }
        }
        if (engine_->config().outline_refuse_sides != sides) engine_->config().outline_refuse_sides = sides;
    }

    // The plane model a pointed-at reading - a curve, a distance - is taken
    // through. A folded edge has its fold height taken off the outline inside
    // the engine; a seam or two points on the picture went straight through
    // the calibration and never saw it, so a curve read at the scale of the
    // mat under the cloth rather than of the cloth's top face. With the same
    // height the engine is using and a working distance to scale by, this is
    // the calibration at that magnification (magnifiedModel); otherwise, and
    // at height zero, it is the calibration itself. Only curve and distance
    // steps: a collar is read from its box as it always has been. Takes
    // lock_, so never call it holding it.
    PlaneModelPtr pointedModel() const {
        const PlaneModelPtr model = engine_->model();
        if (!model) return model;
        bool curveOrDistance = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            curveOrDistance = pointedStepLocked() && !collarStepLocked();
        }
        if (!curveOrDistance) return model;
        const double h = engine_->config().object_height_mm;
        const std::optional<double>& d = engine_->config().camera_distance_mm;
        if (h == 0.0 || !d.has_value() || !(*d > h)) return model;
        return magnifiedModel(*model, (*d - h) / *d);
    }

    void rebuildDatum() {
        cv::Point2d dir;
        double reach = 0.0;
        PlaneModelPtr model = engine_ ? engine_->model() : nullptr;
        const bool ok = model && engine_->hasRoi() &&
                        scanDirection(engine_->roi(), *model, cfg_.scan_from, dir);
        if (ok) reach = railLengthMm(engine_->roi(), *model, cfg_.scan_from);
        {
            std::lock_guard<std::mutex> guard(lock_);
            scan_ = ok ? dir : cv::Point2d(0.0, 0.0);
            haveScan_ = ok;
            reachMm_ = reach;
        }
        if (!ok) return;
        log("Measuring the first straight edge from the " + cfg_.scan_from);
        log("The rail reaches " + vjson::formatFixed(reach, 0) +
            " mm. Nothing longer than that fits in the picture.");
        reportReach();
    }

    // Say so when the style asks for a fold longer than the rail.
    //
    // A fold that runs off the picture still reads. It comes back straight,
    // repeatable, and exactly as long as the view, and it fails every time for
    // what looks like a garment fault. That is worth hearing at the start of a
    // shift rather than arguing about at the end of one.
    void reportReach() {
        double reach = 0.0;
        std::vector<std::string> over;
        {
            std::lock_guard<std::mutex> guard(lock_);
            reach = reachMm_;
            if (!haveTemplate_ || reach <= 0.0) return;
            for (const Step& s : template_.steps) {
                // The whole outline is not laid along the rail, and nor is a
                // collar measured all the way round: that length is a loop.
                if (s.isPerimeter()) continue;
                if (s.isCollar() && s.measure == "around") continue;
                if (s.target_mm + s.tol_plus_mm <= reach) continue;
                over.push_back(s.name + " at " + vjson::formatFixed(s.target_mm, 0) + " mm");
            }
        }
        // log() takes lock_, so it cannot be called while holding it.
        for (const std::string& what : over) {
            log("WARNING: " + what + " is longer than the " + vjson::formatFixed(reach, 0) +
                " mm the rail reaches. Move the camera back, or this step cannot be checked.");
        }
    }

    // -- worker --

    void run() {
        try {
            bootSequence();
        } catch (const std::exception& e) {
            log(std::string("Startup failed: ") + e.what());
            std::lock_guard<std::mutex> guard(lock_);
            status_ = vjson::Value::object();
            status_.set("state", vjson::Value::str("FAULT"));
            status_.set("ready", vjson::Value::boolean(false));
            status_.set("fault", vjson::Value::str(e.what()));
            return;
        }

        long long lastSeq = -1;
        while (!stop_.load()) {
            // Before the frame, and not conditional on one arriving.
            //
            // These three used to sit after the read, below a `continue` taken
            // whenever it timed out, which made every one of them wait on the
            // camera. That is the wrong way round. A command is the operator
            // asking for something and a recalibration is the operator asking
            // for the camera itself to be dealt with, and neither should be
            // deliverable only while the camera is already well. This bench
            // runs at 1.9 fps against a two second read: one slow frame on a
            // link-local gigabit hop and the read returns false, the loop takes
            // the `continue`, and the button that was pressed is still sitting
            // in the queue unlooked-at. Press it on a camera that has properly
            // stalled and nothing happens at all, for as long as the stall
            // lasts - and a stalled camera is exactly when somebody reaches for
            // Recalibrate.
            //
            // A command that throws is a command that failed, not a reason to
            // take the station down with it. Left to escape, it would end this
            // thread, and an exception leaving a thread ends the process.
            try {
                handleCommand();
            } catch (const std::exception& e) {
                log(std::string("That did not work: ") + e.what());
            }
            // The picture of a verdict a command filed (a tap, a box, a flag),
            // now that the command has drawn it.
            flushCaptures();
            serviceCalibration();
            // A clock, not a measurement, so it should not stop because the
            // frames did.
            serviceAutoAdvance();
            applyStepHeight();
            applyStepEdgeRule();
            // After everything that can change the step and before anything is
            // measured or drawn, so nothing from the step just left reaches a
            // frame of the step just arrived at.
            serviceStepChange();
            // The shift's figures, watched and not fed back. Here for a verdict
            // a command filed (a box, a flag), below for one a frame did.
            observeShift();

            cv::Mat frame;
            long long seq = 0;
            double age = 0.0;
            if (!source_->read(frame, seq, age, 2.0)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Only ever measure a frame once. Without this the worker spins on
            // whatever the camera last delivered, and four passes over one
            // motionless copy reads as a part that has stopped moving.
            if (seq == lastSeq) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            lastSeq = seq;
            // The newest picture, for a verdict filed with none of its own
            // (a step flagged for manual QC). The same buffer, not a copy:
            // the camera hands out a new one with every frame.
            lastFrame_ = frame;

            if (dumpWholeViewNext_) {
                dumpWholeViewNext_ = false;
                dumpWholeView(frame);
            }

            // While the grid is being read the feed stays live and nothing on
            // it is measured. The model and the background are about to be
            // replaced, and what the engine would see meanwhile is the
            // operator clearing the mat, measured through a calibration on its
            // way out.
            if (calibrationRunning()) {
                publish(frame, age);
                continue;
            }

            const double t0 = nowSeconds();
            const std::optional<MeasureResult> result = engine_->process(frame, seq);
            procMs_ = 0.8 * procMs_ + 0.2 * (nowSeconds() - t0) * 1000.0;
            reportRefusal();

            if (result.has_value()) {
                if (takeScaleReading(*result)) {
                    // A reference being read for Check scale, not a garment.
                    printResult(*result);
                } else {
                    {
                        // Taken on the step showing now: serviceStepChange has
                        // already settled any change of step this pass.
                        std::lock_guard<std::mutex> guard(lock_);
                        heldForTake_ = true;
                        heldIndex_ = stepIndex_;
                        heldGarment_ = garment_;
                    }
                    printResult(*result);
                    judge(*result, frame);
                }
            }
            observeShift();

            servicePointing(frame);
            servicePerimeterGuide(frame);
            // And of one this frame filed, as it is about to be published.
            flushCaptures();
            publish(frame, age);
        }
    }

    // A lay the engine settled on and would not measure, said on the page. The
    // engine tries again each time the lay settles, so the same reason comes
    // round every few seconds while nothing changes; it is shown again only
    // when it says something new or half a minute has gone by.
    void reportRefusal() {
        const MeasurementEngine::Refusal& r = engine_->lastRefusal();
        if (r.count == refusalsShown_) return;
        refusalsShown_ = r.count;
        const double now = nowSeconds();
        if (r.reason == refusalText_ && now - refusalAt_ < 30.0) return;
        refusalText_ = r.reason;
        refusalAt_ = now;
        log(r.reason);
    }

    // The verdict is taken here, the moment the measurement finalises, from the
    // same refined outline that measurement was made on. It cannot disagree
    // with the reading on the screen, because it is not a second measurement.
    void judge(const MeasureResult& result, const cv::Mat& frame) {
        Template active;
        bool haveActive = false;
        int index = 0;
        int garment = 0;
        {
            std::lock_guard<std::mutex> guard(lock_);
            active = template_;
            haveActive = haveTemplate_;
            index = stepIndex_;
            garment = garment_;
        }

        vjson::Value record = result.toJson();

        // A pointed-at step is not finished by the garment settling. Settling
        // is only what makes the picture worth pointing at: the seam is
        // measured when the operator has said where it is, in measureCurve or
        // measureDistance below. Taking a folded-edge reading here as well
        // would put a verdict on the step before anybody had pointed at
        // anything, and then quietly replace it - and on a distance step,
        // where no edge was ever presented, that verdict is a FAIL about a fold
        // that does not exist, on the largest words on the screen, before the
        // operator is allowed to click.
        if (haveActive && index >= 0 && index < static_cast<int>(active.steps.size()) &&
            active.steps[static_cast<std::size_t>(index)].isPointed()) {
            const Step& s = active.steps[static_cast<std::size_t>(index)];
            {
                std::lock_guard<std::mutex> guard(lock_);
                haveEdge_ = false;
                edgePath_.clear();
            }
            if (s.isDistance()) {
                log("Ready. Tap the two ends of " + s.name + ", or drag from one to the other.");
            } else if (s.isCollar()) {
                log("Ready. Drag a box round the whole collar for " + s.name +
                    " - past both shoulders, with a little of the mat inside it.");
            } else if (s.autoFind() && autoNeckline(result, frame, s)) {
                // Found from the outline and read: nothing to ask for.
            } else {
                log("Ready. Point at " + s.name + ": " + std::to_string(cfg_.curve_seed_count) +
                    " points along the seam.");
            }
            return;
        }

        // A perimeter step is read from the garment settling, as a fold is, but
        // what is read is one half of the whole outline, cut at the clips. A
        // verdict lands only once both halves are in. See takePerimeterHalf.
        if (haveActive && index >= 0 && index < static_cast<int>(active.steps.size()) &&
            active.steps[static_cast<std::size_t>(index)].isPerimeter()) {
            {
                std::lock_guard<std::mutex> guard(lock_);
                haveEdge_ = false;
                edgePath_.clear();
            }
            takePerimeterHalf(result, frame, active, index, garment);
            return;
        }

        if (haveActive) {
            const PlaneModelPtr model = engine_->model();
            cv::Point2d scan;
            {
                std::lock_guard<std::mutex> guard(lock_);
                scan = scan_;
            }
            EdgeResult edge;
            if (model) {
                const RoiBox area = engine_->roi();
                measureEdge(result, *model, scan, cfg_, edge,
                            engine_->hasRoi() ? &area : nullptr);
            }
            StepResult step = judgeStep(active, index, edge);
            if (!model) {
                step.reason = "not calibrated, so there are no millimetres to judge";
            }
            {
                std::lock_guard<std::mutex> guard(lock_);
                haveEdge_ = edge.ok;
                if (edge.ok && model) {
                    edgePath_ = model->mmToPx(edge.path_mm);
                    if (edgePath_.size() < 2) haveEdge_ = false;
                }
            }
            step.garment = garment;
            record = step.toJson();
            record.set("measurement", trimResult(result.toJson()));
            fileStep(step, index, garment, record, frame);
        }

        writeRecord(record);
    }

    // Landing a verdict on the step it belongs to, and arming the move off it.
    //
    // Shared by the folded edge and the traced curve, because the hazard is the
    // same for both and is easy to get wrong once, let alone twice: the
    // operator can press Next, retake, start a new shirt or change style while
    // a reading is being scored, and a verdict that landed on whatever step
    // happens to be showing afterwards would be attributed to the wrong fold.
    // So it is re-checked against the step and the garment it was taken for,
    // and dropped rather than misfiled.
    //
    // A verdict that is filed has its picture taken (captureVerdict), from
    // `picture`, the frame the reading was taken from; `record`, the line
    // about to be written for it, is given the picture's name.
    void fileStep(const StepResult& step, int index, int garment, vjson::Value& record,
                  const cv::Mat& picture) {
        log(std::string(step.pass ? "PASS  " : "FAIL  ") + step.headline());

        {
            std::lock_guard<std::mutex> guard(lock_);
            if (stepIndex_ != index || garment_ != garment ||
                index >= static_cast<int>(stepResults_.size())) {
                return;
            }
            stepResults_[static_cast<std::size_t>(index)] = step;

            // A pass finishes this step, so arm the move to the next one. The last
            // step is left alone: what follows it is a new shirt, and starting one
            // is a decision about the garment in hand, not a consequence of the
            // reading.
            if (cfg_.auto_advance_on_pass && step.pass &&
                index + 1 < static_cast<int>(template_.steps.size())) {
                autoAdvancePending_ = true;
                autoAdvanceAt_ = nowSeconds() + cfg_.auto_advance_delay_s;
                autoAdvanceIndex_ = index;
                autoAdvanceGarment_ = garment;
            }
        }
        captureVerdict(step, index, garment, record, picture);
    }

    struct FeedOverlay;  // what the feed draws; see drawOverlay

    // -- a picture of every verdict --
    //
    // Taken the moment a verdict is filed, of what the feed shows for it: the
    // outline and the span, the seam, the distance or the collar, drawn by
    // drawOverlay exactly as the feed draws them, with a band above saying what
    // was judged. It is for looking back at - beside the feed when the step is
    // revisited, and on disk for QC - and never for measuring or for the feed.
    //
    // This thread only copies what is to be drawn and hands the picture it
    // already holds to the store's writer: the frame is shared, not copied,
    // because nothing writes into a frame once the camera has handed it out
    // (every grab is a new buffer, and heldFrame_ is replaced, never drawn on).
    // Drawing, encoding and writing are the writer's.
    //
    // In two parts. Filing the verdict names the picture, so the record written
    // straight after it can say which file it is. What it shows is copied once
    // whatever filed the verdict has finished (flushCaptures, from the loop):
    // a distance draws its line only after it has filed its reading.
    //
    // What the band across the top of a saved picture says:
    struct CaptureCaption {
        std::string kind;     // PASS, FAIL or MANUAL QC
        int tone = 0;         // 0 pass, 1 fail, 2 manual
        std::string what;     // step k of n, and its name
        std::string reading;  // the number and how far off
        std::string target;
        std::string reason;
        std::string who;      // style, shirt, date and time
    };

    struct PendingCapture {
        CaptureCaption caption;
        std::string rel;
        std::string kind;
        std::string when;
        int index = 0;
        int garment = 0;
        cv::Mat picture;
        double planned = 0.0;  // seconds this thread spent naming it
    };

    void captureVerdict(const StepResult& step, int index, int garment, vjson::Value& record,
                        const cv::Mat& picture) {
        if (!captures_ || picture.empty()) return;
        const double t0 = CaptureStore::steadySeconds();
        const std::string when = timestampIso();  // YYYY-MM-DDTHH:MM:SS, local
        const std::string day = when.substr(0, 10);
        const std::string clock = when.substr(11, 2) + when.substr(14, 2) + when.substr(17, 2);
        const bool manual = step.reason == MANUAL_QC_REASON;
        const std::string kind = manual ? "MANUAL" : (step.pass ? "PASS" : "FAIL");

        CaptureCaption caption;
        caption.kind = manual ? "MANUAL QC" : kind;
        caption.tone = manual ? 2 : (step.pass ? 0 : 1);
        caption.what = "Step " + std::to_string(index + 1) + " of " + std::to_string(step.total) + "  " +
                       step.step;
        if (step.measured) {
            std::string dev = (step.deviation_mm >= 0.0 ? "+" : "") + vjson::formatFixed(step.deviation_mm, 1) + " mm";
            if (step.has_percent()) {
                dev += " (" + std::string(step.deviation_pct() >= 0.0 ? "+" : "") +
                       vjson::formatFixed(step.deviation_pct(), 1) + "%)";
            }
            caption.reading = vjson::formatFixed(step.measured_mm, 1) + " mm   " + dev;
        } else {
            caption.reading = "not measured";
        }
        caption.target = "target " + vjson::formatFixed(step.target_mm, 1) + " mm, allowed " +
                         vjson::formatFixed(step.low_mm(), 1) + " to " + vjson::formatFixed(step.high_mm(), 1) +
                         " mm";
        caption.reason = step.reason;
        caption.who = step.style + "   Shirt " + std::to_string(garment) + "   " + day + " " + when.substr(11);

        std::string slug;
        for (const char c : step.style) {
            const unsigned char u = static_cast<unsigned char>(c);
            if (std::isalnum(u)) slug += static_cast<char>(std::tolower(u));
            else if (!slug.empty() && slug.back() != '-') slug += '-';
        }
        while (!slug.empty() && slug.back() == '-') slug.pop_back();
        if (slug.size() > 40) slug.resize(40);
        if (slug.empty()) slug = "style";
        PendingCapture c;
        c.rel = captures_->plan(
            kind, day, clock + "_g" + std::to_string(garment) + "_s" + std::to_string(index + 1) + "_" + slug);
        c.kind = kind;
        c.when = when;
        c.index = index;
        c.garment = garment;
        c.picture = picture;
        record.set("image", vjson::Value::str(cfg_.capture_dir + "/" + c.rel));
        c.caption = caption;
        c.planned = CaptureStore::steadySeconds() - t0;
        pendingCaptures_.push_back(std::move(c));
    }

    // The pictures of the verdicts just filed, as the feed now draws them,
    // handed to the writer. Called from the loop once whatever filed them has
    // returned: after a command, and before the frame is published.
    void flushCaptures() {
        if (pendingCaptures_.empty()) return;
        for (PendingCapture& c : pendingCaptures_) {
            const double t0 = CaptureStore::steadySeconds();
            FeedOverlay ov = overlayNow(calibrationRunning());
            // What the operator's hand was doing is not part of the reading.
            ov.pointerPresent = false;
            ov.tracing = false;
            ov.tracePx.clear();
            ov.traceCurvePx.clear();
            const double scale = cfg_.capture_scale > 0.0 ? cfg_.capture_scale : cfg_.display_scale;
            const cv::Mat frame = c.picture;
            const CaptureCaption caption = c.caption;
            const unsigned long long id = captures_->submit(
                c.rel,
                [frame, scale, ov = std::move(ov), caption]() { return renderCapture(frame, scale, ov, caption); },
                t0 - c.planned);
            std::lock_guard<std::mutex> guard(lock_);
            // Unless the shirt has changed since, in which case it is only on disk.
            if (c.garment == garment_) {
                CaptureRef& ref = captureOf_[{c.garment, c.index}];
                ref.id = id;
                ref.kind = c.kind;
                ref.at = c.when;
            }
        }
        pendingCaptures_.clear();
    }

    // Hershey fonts draw ASCII only.
    static std::string asciiOnly(const std::string& in) {
        std::string out;
        for (const char c : in) {
            const unsigned char u = static_cast<unsigned char>(c);
            out += (u >= 32 && u < 127) ? c : '?';
        }
        return out;
    }

    // A saved picture: the frame at `scale`, what the feed draws over it, and
    // the band above. On the store's writer thread.
    static cv::Mat renderCapture(const cv::Mat& frame, double scale, const FeedOverlay& ov,
                                 const CaptureCaption& c) {
        cv::Mat disp;
        cv::resize(frame, disp, cv::Size(), scale, scale, cv::INTER_AREA);
        Marks unused(disp.size());
        drawOverlay(disp, scale, ov, unused);

        // Sized to the picture, so a larger capture_scale reads the same.
        const double k = std::max(0.6, disp.cols / 1368.0);
        const int pad = static_cast<int>(std::lround(12 * k));
        const double big = 1.0 * k, mid = 0.7 * k, small = 0.55 * k;
        const int face = cv::FONT_HERSHEY_SIMPLEX;
        int base = 0;
        const cv::Size bigSize = cv::getTextSize(c.kind, face, big, static_cast<int>(2 * k + 0.5), &base);
        const int line = static_cast<int>(std::lround(26 * k));
        const int bandH = pad * 2 + line * 3;
        cv::Mat out(disp.rows + bandH, disp.cols, disp.type(), cv::Scalar(24, 22, 20));
        disp.copyTo(out(cv::Rect(0, bandH, disp.cols, disp.rows)));

        const cv::Scalar tones[3] = {cv::Scalar(94, 197, 34), cv::Scalar(70, 70, 240), cv::Scalar(36, 165, 245)};
        const cv::Scalar ink(235, 235, 235), muted(170, 170, 170);
        const int badgeW = bigSize.width + pad * 2;
        cv::rectangle(out, cv::Rect(pad, pad, badgeW, bandH - pad * 2), tones[c.tone], cv::FILLED);
        cv::putText(out, c.kind, cv::Point(pad * 2, bandH / 2 + bigSize.height / 2), face, big,
                    c.tone == 1 ? cv::Scalar(255, 255, 255) : cv::Scalar(10, 22, 2),
                    static_cast<int>(2 * k + 0.5), cv::LINE_AA);
        const int x = pad * 3 + badgeW;
        const int thin = std::max(1, static_cast<int>(k + 0.5));
        cv::putText(out, asciiOnly(c.what + "    " + c.reading), cv::Point(x, pad + line - 6), face, mid, ink,
                    thin + 1, cv::LINE_AA);
        cv::putText(out, asciiOnly(c.target + (c.reason.empty() ? std::string() : "    " + c.reason)),
                    cv::Point(x, pad + line * 2 - 4), face, small, c.tone == 0 ? muted : tones[c.tone], thin,
                    cv::LINE_AA);
        cv::putText(out, asciiOnly(c.who), cv::Point(x, pad + line * 3 - 4), face, small, muted, thin,
                    cv::LINE_AA);
        return out;
    }

    // The pictures of the shirt on the bench, one per step, for the strip.
    // Call with lock_ held.
    vjson::Value captureJsonLocked(int index) const {
        if (!captures_) return vjson::Value::null();
        const auto it = captureOf_.find({garment_, index});
        if (it == captureOf_.end()) return vjson::Value::null();
        const CaptureStore::State st = captures_->state(it->second.id);
        const char* state = st == CaptureStore::State::Saving  ? "saving"
                            : st == CaptureStore::State::Saved ? "saved"
                            : st == CaptureStore::State::Dropped ? "dropped"
                                                                 : "failed";
        vjson::Value c = vjson::Value::object();
        c.set("id", vjson::Value::integer(static_cast<long long>(it->second.id)));
        c.set("state", vjson::Value::str(state));
        c.set("kind", vjson::Value::str(it->second.kind));
        c.set("at", vjson::Value::str(it->second.at));
        return c;
    }

    // -- pointing at a seam --

    // Whether the step showing now is one the operator has to point at, and how
    // many points it wants.
    // Call with lock_ already held.
    bool curveStepLocked() const {
        if (!haveTemplate_) return false;
        if (stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return false;
        return template_.steps[static_cast<std::size_t>(stepIndex_)].isCurve();
    }

    bool curveStep(int* seedsWanted = nullptr) const {
        std::lock_guard<std::mutex> guard(lock_);
        if (seedsWanted) *seedsWanted = std::max(2, cfg_.curve_seed_count);
        return curveStepLocked();
    }

    // A step taken by pointing at the picture rather than laying an edge
    // against the rail: a curve, or a plain distance between two points.
    bool pointedStepLocked() const {
        if (!haveTemplate_) return false;
        if (stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return false;
        return template_.steps[static_cast<std::size_t>(stepIndex_)].isPointed();
    }
    bool pointedStep() const {
        std::lock_guard<std::mutex> guard(lock_);
        return pointedStepLocked();
    }
    bool distanceStepLocked() const {
        if (!haveTemplate_) return false;
        if (stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return false;
        return template_.steps[static_cast<std::size_t>(stepIndex_)].isDistance();
    }
    bool distanceStep() const {
        std::lock_guard<std::mutex> guard(lock_);
        return distanceStepLocked();
    }
    // A step read from a box drawn round the collar.
    bool collarStepLocked() const {
        if (!haveTemplate_) return false;
        if (stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return false;
        return template_.steps[static_cast<std::size_t>(stepIndex_)].isCollar();
    }
    bool collarStep() const {
        std::lock_guard<std::mutex> guard(lock_);
        return collarStepLocked();
    }
    // A step that reads the whole outer edge of the garment, in two halves.
    bool perimeterStepLocked() const {
        if (!haveTemplate_) return false;
        if (stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return false;
        return template_.steps[static_cast<std::size_t>(stepIndex_)].isPerimeter();
    }
    bool perimeterStep() const {
        std::lock_guard<std::mutex> guard(lock_);
        return perimeterStepLocked();
    }
    // How many points the step on now wants: exactly two for a distance, the
    // configured count for a curve.
    int seedsWantedLocked() const {
        return distanceStepLocked() ? 2 : std::max(2, cfg_.curve_seed_count);
    }

    void clearSeeds() {
        std::lock_guard<std::mutex> guard(lock_);
        seeds_.clear();
        curvePath_.clear();
        haveCurve_ = false;
        seamOptionsPx_.clear();
        pointerPresent_ = false;
        pointerDwell_ = 0.0;
        tracing_ = false;
        tracePx_.clear();
        traceCurvePx_.clear();
        if (finger_) {
            finger_->resetDwell();
            finger_->abandonTrace();
        }
        // Drop the scene the tracker is differencing against as well. Whatever
        // caused the points to be cleared - a move between steps, a new shirt,
        // a recalibration - changed the picture the reference was taken from,
        // and a stale reference finds a hand-shaped hole where the garment used
        // to be.
        fingerArmed_ = false;
        // And a collar's box and what was found in it, for the same reason:
        // they describe a picture that is no longer the one on the mat.
        clearCollarLocked();
    }

    // Call with lock_ held.
    void clearCollarLocked() {
        haveCollarBox_ = false;
        haveCollar_ = false;
        collar_ = CollarResult();
        collarLinesPx_.clear();
        collarRoles_.clear();
        collarBackPx_.clear();
        collarChosen_ = -1;
        collarMm_ = 0.0;
        collarState_.clear();
        collarWhy_.clear();
    }

    // The halves of a perimeter read so far, and everything drawn for them.
    // Call with lock_ held. The guide is not cleared: it is about the lay on
    // the mat now, and it is worked out again every frame.
    void clearPerimeterLocked() {
        perimHave_ = 0;
        perimHalf_[0] = PerimeterHalf();
        perimHalf_[1] = PerimeterHalf();
        perimPathsPx_.clear();
        perimCutsPx_.clear();
        perimCornersPx_.clear();
        perimMessage_.clear();
        perimBad_ = false;
        perimTotalMm_ = 0.0;
        perimWhole_ = false;
    }

    // One frame of watching for a finger.
    //
    // Only while a reading is held, and only on a step that asked to be pointed
    // at. Both matter. Before the garment has settled the picture is still
    // moving, so a difference against it finds the garment rather than the
    // hand; and on a folded-edge step there is nothing to point at, so watching
    // for one would be a way to go wrong for no gain.
    void servicePointing(const cv::Mat& frame) {
        const bool held = (engine_->state() == STATE_MEASURED);
        if (!held || !pointedStep()) {
            std::lock_guard<std::mutex> guard(lock_);
            fingerArmed_ = false;
            pointerPresent_ = false;
            pointerDwell_ = 0.0;
            // A distance line is a line on a garment that is no longer there,
            // and so are the seams the machine offered on it, and a collar's
            // box and the lines found in it.
            haveDistance_ = false;
            distancePx_.clear();
            seamOptionsPx_.clear();
            clearCollarLocked();
            // The points describe a seam on the garment that was on the mat.
            // Once the reading is not held there is no such garment - it has
            // been taken away, or moved, or has not settled yet - and a point
            // kept across that is a point on a shirt that is no longer there.
            // It draws on the feed over bare mat, which is exactly as
            // convincing and exactly as wrong as it sounds.
            tracing_ = false;
            tracePx_.clear();
            traceCurvePx_.clear();
            if (!seeds_.empty() || finger_) {
                seeds_.clear();
                curvePath_.clear();
                haveCurve_ = false;
                if (finger_) {
                    finger_->resetDwell();
                    finger_->abandonTrace();
                }
            }
            return;
        }

        // A distance step is taken with the mouse and nothing else. No hand is
        // watched for, so no reference is armed and nothing on the feed follows
        // a finger; the points stay where they were put.
        if (!curveStep()) {
            // ...but it still needs the picture the points will be put on: the
            // garment settled, taken once when the reading holds. The flag that
            // says the tracker has its reference is the flag that says this
            // picture has been taken; on a distance step that is all it says.
            // Without it the click had nothing to land on, and the message the
            // operator got was about calibration, which was fine.
            bool take = false;
            {
                std::lock_guard<std::mutex> guard(lock_);
                take = !fingerArmed_;
                fingerArmed_ = true;
                pointerPresent_ = false;
                pointerDwell_ = 0.0;
                tracing_ = false;
            }
            if (take) heldFrame_ = frame.clone();
            return;
        }

        if (!finger_) finger_.reset(new FingerTracker(cfg_));

        bool arm = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            arm = !fingerArmed_;
        }
        if (arm) {
            // The picture the seam will be read from, and the picture the hand
            // is measured against, are the same frame and it is this one: the
            // garment settled, with nobody's hand over it. Reading the seam off
            // a later frame would read it through whatever is lying across it
            // at the time, which on a curve step is by definition the
            // operator's own finger.
            heldFrame_ = frame.clone();
            finger_->setReference(frame, engine_->roi(), engine_->hasRoi());
            std::lock_guard<std::mutex> guard(lock_);
            fingerArmed_ = true;
        }

        // Through the model the reading will be taken through, so the points
        // land where the seam will be looked for.
        const PlaneModelPtr pointed = pointedModel();
        const FingerState s = finger_->update(frame, pointed.get(), nowSeconds());
        {
            std::lock_guard<std::mutex> guard(lock_);
            pointerPresent_ = s.present;
            pointerPx_ = s.tip_px;
            // While a trace is running the ring shows it ENDING rather than a
            // hold that can no longer start anything. Same ring, and it means
            // the same thing both times - something is about to happen, move if
            // you did not mean it.
            pointerDwell_ = s.tracing ? s.hold_fraction : s.dwell_fraction;
        }
        {
            std::lock_guard<std::mutex> guard(lock_);
            tracing_ = s.tracing;
            pointerRejected_ = s.rejected;
            tracePx_.clear();
            traceCurvePx_.clear();
            if (s.tracing && !s.trace_mm.empty()) {
                const PlaneModelPtr& model = pointed;
                if (model) {
                    tracePx_ = model->mmToPx(s.trace_mm);
                    // The curve through the points, as the corridor will be
                    // built along it: the same fit snapCurve uses, so what the
                    // operator watches grow is what will be measured. Sampled
                    // at two millimetres because it is for drawing; the window
                    // is in millimetres and does not care.
                    std::vector<cv::Point2d> curve;
                    if (s.trace_mm.size() >= 4 &&
                        seedPath(s.trace_mm, 2.0, curve, 0.0, cfg_.curve_trace_smooth_mm)) {
                        traceCurvePx_ = model->mmToPx(curve);
                    }
                }
            }
        }

        // A finished trace IS the pointing: the whole path replaces whatever
        // points were down, because it describes the seam everywhere rather
        // than at three places.
        if (s.finished && s.path_mm.size() >= 3) {
            {
                std::lock_guard<std::mutex> guard(lock_);
                seeds_ = s.path_mm;
            }
            log("Traced " + std::to_string(s.path_mm.size()) + " points along the seam");
            savePointingBundle(s.path_mm);
            measureCurve();
            return;
        }
        if (s.committed && !s.tracing) addSeed(s.point_mm);
    }

    // A tap on the picture, as the tablet sends it: a fraction of the way
    // across the feed and a fraction of the way down it.
    //
    // Fractions rather than pixels because the page is drawn at whatever size
    // the tablet has and scaled by cfg.display_scale before that, and a
    // coordinate that had to agree with both would be wrong on the first bench
    // that used a different screen. A fraction is the same number everywhere,
    // and this is the only place that has to know what it is a fraction of.
    void markFromPage(const std::string& arg) {
        if (engine_->state() != STATE_MEASURED) {
            log("Wait for the reading to hold before pointing");
            return;
        }
        if (collarStep()) {
            log("This step is read from a box: drag one round the collar");
            return;
        }
        if (!pointedStep()) {
            log("This step is measured from a folded edge, not by pointing");
            return;
        }
        const PlaneModelPtr model = pointedModel();
        if (!model) {
            log("Not calibrated, so a point on the picture is not a point on the mat");
            return;
        }
        if (heldFrame_.empty()) {
            log("No held picture yet to put a point on - wait a frame and tap again");
            return;
        }

        const std::size_t comma = arg.find(',');
        if (comma == std::string::npos) return;
        const double fx = std::atof(arg.substr(0, comma).c_str());
        const double fy = std::atof(arg.substr(comma + 1).c_str());
        if (!(fx >= 0.0 && fx <= 1.0 && fy >= 0.0 && fy <= 1.0)) return;

        const cv::Point2d px(fx * heldFrame_.cols, fy * heldFrame_.rows);
        const std::vector<cv::Point2d> mm = model->pxToMm({px});
        if (mm.empty()) return;
        addSeed(mm.front());
    }

    // A whole path drawn on the picture, as the tablet sends it: the same
    // fractions as a tap, several dozen of them.
    //
    // A drag on a screen is the one form of this that is exactly defined -
    // finger down, finger moves, finger up - where a camera looking straight
    // down can never see a finger lift and has to infer the end of a trace from
    // a hold or from the hand leaving. So this path is the reference the other
    // one is trying to be.
    void traceFromPage(const std::string& arg) {
        if (engine_->state() != STATE_MEASURED || !pointedStep() || collarStep()) return;
        const PlaneModelPtr model = pointedModel();
        if (!model) {
            log("Not calibrated, so a line on the picture is not a line on the mat");
            return;
        }
        if (heldFrame_.empty()) {
            log("No held picture yet to draw on - wait a frame and try again");
            return;
        }

        std::vector<cv::Point2d> px;
        std::size_t i = 0;
        while (i < arg.size()) {
            const std::size_t semi = arg.find(';', i);
            const std::string one =
                arg.substr(i, semi == std::string::npos ? std::string::npos : semi - i);
            const std::size_t comma = one.find(',');
            if (comma != std::string::npos) {
                const double fx = std::atof(one.substr(0, comma).c_str());
                const double fy = std::atof(one.substr(comma + 1).c_str());
                if (fx >= 0.0 && fx <= 1.0 && fy >= 0.0 && fy <= 1.0) {
                    px.emplace_back(fx * heldFrame_.cols, fy * heldFrame_.rows);
                }
            }
            if (semi == std::string::npos) break;
            i = semi + 1;
        }
        // On a distance step a drag is a tape laid along whatever was drawn:
        // press at one end, follow the curve, release at the other, and the
        // reading is the length along it. No seam is looked for. Two taps on
        // the same step are the straight line; this is the other half of the
        // same tool.
        if (distanceStep() && px.size() >= 2) {
            const std::vector<cv::Point2d> drawn = model->pxToMm(px);
            {
                std::lock_guard<std::mutex> guard(lock_);
                seeds_.clear();
                haveDistance_ = false;
                distancePx_.clear();
            }
            measureTrace(drawn);
            return;
        }
        if (px.size() < 3) {
            log("That was a tap rather than a trace");
            return;
        }

        {
            std::lock_guard<std::mutex> guard(lock_);
            seeds_ = model->pxToMm(px);
        }
        log("Traced " + std::to_string(px.size()) + " points along the seam");
        measureCurve();
    }

    // A point, from a finger in the air or a tap on the tablet. The two are the
    // same thing by the time they arrive here, which is the point of taking
    // them both as millimetres on the mat.
    void addSeed(const cv::Point2d& mm) {
        int wanted = 0;
        std::size_t have = 0;
        bool full = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            wanted = seedsWantedLocked();
            // The first point of a new distance takes the last reading's line
            // off the picture; two lines on the feed would be two readings.
            if (seeds_.empty() && distanceStepLocked()) {
                haveDistance_ = false;
                distancePx_.clear();
            }
            // A full set of points is a full set.
            //
            // A read that succeeds clears the points, so the only way to be
            // here with a full set is that the last one was refused - and the
            // right answer to a refusal is to point again, not to add a fourth
            // point to three that already failed. Left unguarded this ran away
            // badly: the hand stays near the seam while the operator reads the
            // verdict, a point lands every second or so, and the station ends
            // up with ten or thirteen points strung across the garment. Past
            // three, seedPath stops fitting an arc and follows the polyline
            // through all of them instead, so the corridor zigzags across the
            // shirt and there is genuinely no seam anywhere along it. The
            // refusal that follows is correct and completely unhelpful.
            if (static_cast<int>(seeds_.size()) >= wanted) {
                full = true;
            } else {
                seeds_.push_back(mm);
                have = seeds_.size();
            }
        }
        if (full) {
            log("Already have " + std::to_string(wanted) +
                " points. Undo one or Clear to point again.");
            return;
        }
        log("Point " + std::to_string(have) + " of " + std::to_string(wanted));
        if (static_cast<int>(have) >= wanted) {
            if (distanceStep()) {
                measureDistance();
            } else {
                measureCurve();
            }
        }
    }

    // Find the neckline from the garment's own outline and read it, without
    // being pointed at.
    //
    // The outline is the refined boundary the reading was just taken from; its
    // notch on the mark side is the neck, the notch's two corners are the
    // shoulder points, and the tech pack's length says how deep the seam dips
    // between them. Handed to snapCurve as if they had been pointed at, with
    // every guard it has. Returns true only when a reading was filed; anything
    // the machine is not sure of is logged and left for the operator, and is
    // never a verdict on the garment.
    bool autoNeckline(const MeasureResult& result, const cv::Mat& frame, const Step& s) {
        const PlaneModelPtr model = engine_->model();
        // The outline has had the fold height taken off it already, in the
        // engine, so it goes through the calibration; the seams are found on
        // the picture, so they go through the model that allows for it.
        const PlaneModelPtr pointed = pointedModel();
        if (!model || result.outline_px.size() < 20 || frame.empty()) return false;

        cv::Point2d toward;
        bool haveScan = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            toward = cv::Point2d(-scan_.x, -scan_.y);  // scan_ points in from the mark
            haveScan = haveScan_;
        }
        if (!haveScan) {
            log("Cannot find the neckline by itself: which side the neck faces is not known");
            return false;
        }

        // The length along the seam is what sizes the corridor. A step judged
        // on the chord has no arc to offer, and the notch itself is used.
        const double targetArc = (s.measure == "chord") ? 0.0 : s.target_mm;
        NecklineFind find;
        if (!findNeckline(model->pxToMm(result.outline_px), toward, targetArc, find)) {
            log("Could not find the neckline by itself: " + find.error);
            return false;
        }
        const std::string how = "Found the neck from the outline, " +
                                vjson::formatFixed(find.chord_mm, 0) + " mm between its corners";

        // The picture the seam is read from is this one: the garment settled,
        // no hand over it. The pointing loop would take the same frame a moment
        // later; taking it here means the reading does not wait for it.
        heldFrame_ = frame.clone();

        // Every seam the picture can find between the corners, then the rule
        // that chooses among them: the step's convention for a band, and a
        // loose gate on the target. When the rule cannot choose, what was
        // found goes on the feed, thin, and the operator points at the one
        // they mean - one tap near it is enough, because the ends are known.
        const std::vector<SeamCandidate> seams = findSeams(heldFrame_, *pointed, find, cfg_);
        const bool alongArc =
            s.measure.empty() ? cfg_.curve_measure_along_arc : (s.measure == "arc");
        std::size_t chosen = 0;
        std::string why;
        const bool decided = chooseSeam(seams, s.target_mm, alongArc,
                                        s.seam.empty() ? "outer" : s.seam, chosen, why);
        {
            std::lock_guard<std::mutex> guard(lock_);
            seamOptionsPx_.clear();
            if (!decided) {
                for (const SeamCandidate& c : seams) {
                    seamOptionsPx_.push_back(pointed->mmToPx(c.curve.path_mm));
                }
            }
        }
        if (!decided) {
            log(how + ": " + why + ". Point at the one you mean.");
            return false;
        }
        log(how + (why.empty() ? "" : ": " + why) + ". Reading it.");
        {
            std::lock_guard<std::mutex> guard(lock_);
            seeds_ = seams[chosen].seeds_mm;
        }
        return measureCurve(true);
    }

    // -- a collar, from a box --

    // The box the operator dragged round the collar, as the page sends it: two
    // corners, each a fraction of the way across the feed and down it, the
    // same fractions a tap is sent in.
    void collarFromPage(const std::string& arg) {
        if (engine_->state() != STATE_MEASURED) {
            log("Wait for the reading to hold before drawing the box");
            return;
        }
        if (!collarStep()) {
            log("This step is not read from a box");
            return;
        }
        const PlaneModelPtr model = engine_->model();
        if (!model) {
            log("Not calibrated, so there are no millimetres to find a collar in");
            return;
        }
        if (heldFrame_.empty()) {
            log("No held picture yet to find the collar in - wait a frame and draw the box again");
            return;
        }
        double f[4] = {0.0, 0.0, 0.0, 0.0};
        {
            std::size_t i = 0;
            for (int k = 0; k < 4; ++k) {
                const std::size_t comma = arg.find(',', i);
                const std::string one = arg.substr(i, comma == std::string::npos ? std::string::npos : comma - i);
                f[k] = std::atof(one.c_str());
                if (!(f[k] >= 0.0 && f[k] <= 1.0)) return;
                if (comma == std::string::npos) {
                    if (k != 3) return;
                    break;
                }
                i = comma + 1;
            }
        }
        const double x0 = std::min(f[0], f[2]) * heldFrame_.cols, x1 = std::max(f[0], f[2]) * heldFrame_.cols;
        const double y0 = std::min(f[1], f[3]) * heldFrame_.rows, y1 = std::max(f[1], f[3]) * heldFrame_.rows;
        const cv::Rect2d box(x0, y0, x1 - x0, y1 - y0);

        Template active;
        int index = 0;
        int garment = 0;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_) return;
            active = template_;
            index = stepIndex_;
            garment = garment_;
            clearCollarLocked();
            haveCollarBox_ = true;
            collarBoxPx_ = box;
            collarState_ = "reading";
            ++collarSeq_;
        }
        if (index < 0 || index >= static_cast<int>(active.steps.size())) return;
        const Step& s = active.steps[static_cast<std::size_t>(index)];
        CollarRequest req;
        req.edge = s.seam.empty() ? "outer" : s.seam;
        req.along_arc = s.measure.empty() ? cfg_.curve_measure_along_arc : (s.measure != "chord");
        req.around = s.measure == "around";
        req.target_mm = s.target_mm;

        // The outline the held reading was taken from: where the cloth ends,
        // near enough to start from. Where each line meets it is then found
        // on the picture itself.
        std::vector<cv::Point2d> outlineMm;
        const std::optional<MeasureResult>& last = engine_->lastResult();
        if (last.has_value() && last->outline_px.size() >= 20) outlineMm = model->pxToMm(last->outline_px);

        const double t0 = nowSeconds();
        CollarResult r;
        detectCollar(heldFrame_, *model, outlineMm, box, req, cfg_, r);
        log("Looked for the collar in the box: " +
            vjson::formatFixed((nowSeconds() - t0) * 1000.0, 0) + " ms");
        showCollar(r);
        fileCollar(r, active, index, garment, outlineMm, req);
    }

    // A tap on the feed on a collar step: when the machine could not find
    // the band and has offered the lines it did find, the one nearest the tap
    // is taken. Nothing else is: after a reading, a tap changes nothing, so
    // the step's rule and not the operator decides which edge is measured.
    void collarPickFromPage(const std::string& arg) {
        if (engine_->state() != STATE_MEASURED || !collarStep()) return;
        const PlaneModelPtr model = engine_->model();
        if (!model || heldFrame_.empty()) return;
        const std::size_t comma = arg.find(',');
        if (comma == std::string::npos) return;
        const double fx = std::atof(arg.substr(0, comma).c_str());
        const double fy = std::atof(arg.substr(comma + 1).c_str());
        if (!(fx >= 0.0 && fx <= 1.0 && fy >= 0.0 && fy <= 1.0)) return;

        CollarResult r;
        Template active;
        int index = 0;
        int garment = 0;
        bool allowed = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_ || collarState_ != "choose") return;
            allowed = showManual_;
            r = collar_;
            active = template_;
            index = stepIndex_;
            garment = garment_;
        }
        // Choosing a line the machine could not vouch for is a hand reading,
        // which is the fallback's to allow. Without it the answer is a new box
        // or a flag for manual QC.
        if (!allowed) {
            log("No collar band to read. Redraw the box, or flag the shirt for manual QC");
            return;
        }
        const std::vector<cv::Point2d> mm = model->pxToMm({cv::Point2d(fx * heldFrame_.cols, fy * heldFrame_.rows)});
        if (mm.empty()) return;
        if (!pickCollarLine(r, mm.front(), 8.0)) {
            log("No line of the collar there - tap on one of the lines drawn");
            return;
        }
        log("Collar line chosen with a tap");
        showCollar(r);
        CollarRequest req;
        req.edge = r.edge;
        req.along_arc = r.along_arc;
        req.around = r.around;
        fileCollar(r, active, index, garment, {}, req);
    }

    // What a collar reading draws on the feed and says on the panel.
    void showCollar(const CollarResult& r) {
        const PlaneModelPtr model = engine_->model();
        std::lock_guard<std::mutex> guard(lock_);
        collar_ = r;
        haveCollar_ = true;
        collarLinesPx_.clear();
        collarRoles_.clear();
        collarBackPx_.clear();
        if (model) {
            for (const CollarLine& L : r.lines) {
                collarLinesPx_.push_back(model->mmToPx(L.path_mm));
                collarRoles_.push_back(L.role);
            }
            if (r.ok && r.around && r.back_path_mm.size() >= 2) collarBackPx_ = model->mmToPx(r.back_path_mm);
        }
        collarChosen_ = r.ok ? r.chosen : -1;
        collarMm_ = r.ok ? r.measured_mm() : 0.0;
        collarState_ = r.ok ? "read" : (r.refusal == "choose" ? "choose" : "refused");
        collarWhy_ = r.ok ? r.how : r.error;
    }

    // A collar reading onto the step and into the log.
    //
    // A reading, and a collar that could not be read, are verdicts, as they
    // are for a seam: a garment whose collar cannot be seen has not been
    // measured. Two things are not. A box that holds no collar, or cuts the
    // one it holds, is about the box and not the garment - the operator draws
    // it again and nothing is filed. And lines offered for a tap are not a
    // verdict until one is tapped.
    void fileCollar(const CollarResult& r, const Template& active, int index, int garment,
                    const std::vector<cv::Point2d>& outlineMm, const CollarRequest& req) {
        if (!r.ok && !heldFrame_.empty()) saveCollarFailure(r, outlineMm, req);
        if (!r.ok && (r.refusal == "box" || r.refusal == "choose")) {
            log(r.refusal == "box" ? "Draw the box again: " + r.error : r.how);
            return;
        }
        StepResult step = judgeCollar(active, index, r);
        step.garment = garment;
        vjson::Value record = step.toJson();
        record.set("collar", r.toJson());
        fileStep(step, index, garment, record, heldFrame_);
        writeRecord(record);
    }

    // The operator's answer to a collar the box could not read: handed on to
    // be measured by hand. Filed as a FAIL with no number on it - no number was
    // taken, and a shirt that passed on the steps that happened to read has not
    // passed - under a reason no reading gives, so it is plain in the log and
    // on the page that this is a hand-off and not a measurement.
    void flagForManualQc() {
        Template active;
        int index = 0;
        int garment = 0;
        bool collar = false;
        bool armed = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_ || stepIndex_ < 0 || stepIndex_ >= static_cast<int>(template_.steps.size())) return;
            collar = template_.steps[static_cast<std::size_t>(stepIndex_)].isCollar();
            active = template_;
            index = stepIndex_;
            garment = garment_;
            armed = fingerArmed_;
        }
        // Its picture: the held one the box was drawn on, when it was taken
        // on this take (fingerArmed_ says so); otherwise the newest frame.
        const cv::Mat& picture = armed && !heldFrame_.empty() ? heldFrame_ : lastFrame_;
        if (!collar) {
            log("Only a collar step can be flagged for manual QC");
            return;
        }
        CollarResult none;
        none.error = MANUAL_QC_REASON;
        StepResult step = judgeCollar(active, index, none);
        step.measured = false;
        step.pass = false;
        step.measured_mm = 0.0;
        step.deviation_mm = 0.0;
        step.reason = MANUAL_QC_REASON;
        step.garment = garment;
        vjson::Value record = step.toJson();
        record.set("manual_qc", vjson::Value::boolean(true));
        fileStep(step, index, garment, record, picture);
        writeRecord(record);
    }

    // The held picture, the box and everything the box was read with, left
    // behind when a collar could not be read - in the form collar_check
    // replays, so the case can be taken away and run again with no garment
    // and no camera in the room. One pair of files, overwritten each time.
    void saveCollarFailure(const CollarResult& r, const std::vector<cv::Point2d>& outlineMm,
                           const CollarRequest& req) {
        const std::string png = "collar-fail.png";
        const std::string json = "collar-fail.json";
        const std::vector<int> fast{cv::IMWRITE_PNG_COMPRESSION, 1};
        if (!cv::imwrite(png, heldFrame_, fast)) {
            log("Could not write " + png);
            return;
        }
        vjson::Value v = vjson::Value::object();
        v.set("frame", vjson::Value::str(png));
        v.set("calibration", vjson::Value::str(cfg_.calibration_file));
        vjson::Value box = vjson::Value::array();
        box.push(vjson::Value::real(r.box_px.x));
        box.push(vjson::Value::real(r.box_px.y));
        box.push(vjson::Value::real(r.box_px.width));
        box.push(vjson::Value::real(r.box_px.height));
        v.set("box_px", box);
        v.set("edge", vjson::Value::str(req.edge));
        v.set("along_arc", vjson::Value::boolean(req.along_arc));
        if (req.around) v.set("around", vjson::Value::boolean(true));
        v.set("target_mm", vjson::Value::real(req.target_mm));
        vjson::Value outline = vjson::Value::array();
        for (const cv::Point2d& p : outlineMm) {
            vjson::Value a = vjson::Value::array();
            a.push(vjson::Value::real(p.x));
            a.push(vjson::Value::real(p.y));
            outline.push(a);
        }
        v.set("outline_mm", outline);
        v.set("refused", vjson::Value::str(r.error));
        if (!vjson::writeFile(json, v)) {
            log("Could not write " + json);
            return;
        }
        log("Saved " + png + " and " + json + ". To see why, offline and without a camera:");
        log("  collar_check --case " + json + " --draw collar-out.png");
    }

    // Everything the page needs to draw the collar strip. Call with lock_ held.
    vjson::Value collarJsonLocked() const {
        vjson::Value c = vjson::Value::object();
        // One per box drawn, so the page can tell the answer to the box it
        // just sent from the answer to the one before.
        c.set("seq", vjson::Value::integer(static_cast<long long>(collarSeq_)));
        c.set("box", vjson::Value::boolean(haveCollarBox_));
        c.set("state", vjson::Value::str(collarState_));
        c.set("why", vjson::Value::str(collarWhy_));
        if (collarState_ == "read") c.set("mm", vjson::Value::real(collarMm_));
        // All the way round: the front and the back the reading adds up.
        if (collar_.around) {
            c.set("around", vjson::Value::boolean(true));
            if (collarState_ == "read") {
                c.set("front_mm", vjson::Value::real(collar_.arc_mm));
                c.set("back_mm", vjson::Value::real(collar_.back_mm));
            }
        }
        vjson::Value lines = vjson::Value::array();
        for (std::size_t k = 0; k < collar_.lines.size(); ++k) {
            const CollarLine& L = collar_.lines[k];
            vjson::Value l = vjson::Value::object();
            l.set("role", vjson::Value::str(L.role));
            l.set("arc_mm", vjson::Value::real(L.arc_mm));
            l.set("chord_mm", vjson::Value::real(L.chord_mm));
            l.set("chosen", vjson::Value::boolean(static_cast<int>(k) == collarChosen_));
            lines.push(l);
        }
        c.set("lines", lines);
        return c;
    }

    // -- the full perimeter, in two halves --

    // The clips' colour for a step: its own, when it names one, or the bench's.
    ClipColour perimeterColour(const Step& s) const {
        ClipColour c = clipColourFrom(cfg_);
        if (s.hasClipHue()) {
            c.hue_lo_deg = s.clip_hue_lo_deg;
            c.hue_hi_deg = s.clip_hue_hi_deg;
        }
        return c;
    }

    static std::string capitalise(std::string text) {
        if (!text.empty()) text[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(text[0])));
        return text;
    }

    static std::string mmText(double v) { return vjson::formatFixed(v, 1) + " mm"; }

    // The length a step judges a half on.
    static double judgedMm(const Step& s, const PerimeterHalf& h) {
        return s.measure == "traced" ? h.traced_mm : h.fitted_mm;
    }

    static std::string halfText(const PerimeterHalf& h) {
        return mmText(h.fitted_mm) + " fitted, " + mmText(h.traced_mm) + " traced, " +
               std::to_string(h.crossed) + " of " + std::to_string(h.corners) + " corners crossed, reach " +
               mmText(h.reach_mm);
    }

    // What the feed draws for the halves just read. Call with lock_ held.
    void perimeterShowLocked(const PerimeterHalf* halves, int count) {
        perimPathsPx_.clear();
        perimCutsPx_.clear();
        perimCornersPx_.clear();
        for (int k = 0; k < count; ++k) {
            const PerimeterHalf& h = halves[k];
            if (h.path_px.size() >= 2) perimPathsPx_.push_back(h.path_px);
            perimCutsPx_.push_back(h.cut_px[0]);
            perimCutsPx_.push_back(h.cut_px[1]);
            for (const cv::Point2d& q : h.corner_px) perimCornersPx_.push_back(q);
        }
    }

    // One reading on a perimeter step: a half of the outline, cut at the clips.
    //
    // Nothing is filed until both halves are in. A half that cannot be read is
    // about the lay - a clip missing, the half off the edge of the picture - and
    // the operator lays it again: it is said on the panel and in the log, and
    // the picture is kept for perimeter_check, but it is not a verdict on the
    // garment. The second half has to be the other half. Laid the same way
    // round as the first it is refused, because the same half twice adds up to
    // a confident wrong number.
    void takePerimeterHalf(const MeasureResult& result, const cv::Mat& frame, const Template& active,
                           int index, int garment) {
        const Step& s = active.steps[static_cast<std::size_t>(index)];
        const PlaneModelPtr model = engine_->model();
        if (!model || !engine_->hasRoi()) {
            {
                std::lock_guard<std::mutex> guard(lock_);
                perimMessage_ = "Not calibrated, so there are no millimetres to measure the perimeter in";
                perimBad_ = true;
            }
            log("Perimeter: not calibrated, so there are no millimetres to measure it in");
            return;
        }
        const RoiBox roi = engine_->roi();
        const ClipColour colour = perimeterColour(s);
        const double t0 = nowSeconds();
        PerimeterCapture cap;
        measurePerimeterCapture(result, frame, *model, roi, colour, cfg_, cap);
        const std::string took = vjson::formatFixed((nowSeconds() - t0) * 1000.0, 0) + " ms";

        int have = 0;
        PerimeterHalf first;
        {
            std::lock_guard<std::mutex> guard(lock_);
            // Filed against the step and the shirt it was measured for, or
            // dropped: the operator can move on while a half is being read.
            if (stepIndex_ != index || garment_ != garment) return;
            have = perimHave_;
            first = perimHalf_[0];
        }
        const std::string which = have == 1 ? "second" : "first";

        if (!cap.ok) {
            savePerimeterFailure(result, frame, roi, colour, cap.error);
            {
                std::lock_guard<std::mutex> guard(lock_);
                perimMessage_ = capitalise(cap.error) + ". Lift the shirt clear of the mat and lay the " + which +
                                " half again.";
                perimBad_ = true;
                perimPathsPx_.clear();
                perimCutsPx_.clear();
                perimCornersPx_.clear();
            }
            log("Perimeter, " + which + " half not read: " + cap.error);
            return;
        }

        if (cap.whole) {
            log("Perimeter: the whole shirt is in view, both halves read from one picture (" + took + ")");
            finishPerimeter(cap.half[0], cap.half[1], true, active, index, garment, frame);
            return;
        }

        if (have == 1) {
            std::string why;
            if (!turnedEndForEnd(first, cap.half[0], why)) {
                {
                    std::lock_guard<std::mutex> guard(lock_);
                    perimMessage_ = capitalise(why) + ".";
                    perimBad_ = true;
                    perimeterShowLocked(&cap.half[0], 1);
                }
                log("Perimeter, second half not taken: " + why);
                return;
            }
            log("Perimeter, second half: " + halfText(cap.half[0]) + " (" + took + ")");
            finishPerimeter(first, cap.half[0], false, active, index, garment, frame);
            return;
        }

        // A first half: of this step, or of a fresh attempt at one already
        // read, whose verdict goes until the new pair is in.
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (perimHave_ == 2 && index < static_cast<int>(stepResults_.size())) {
                stepResults_[static_cast<std::size_t>(index)] = std::nullopt;
                autoAdvancePending_ = false;
            }
            perimHave_ = 1;
            perimHalf_[0] = cap.half[0];
            perimHalf_[1] = PerimeterHalf();
            perimWhole_ = false;
            perimTotalMm_ = 0.0;
            perimeterShowLocked(&cap.half[0], 1);
            perimMessage_ = "First half " + mmText(judgedMm(s, cap.half[0])) +
                            ". Lift the shirt clear of the mat, turn it end for end, and lay the other half "
                            "the same way, with the clips in the band.";
            perimBad_ = false;
        }
        log("Perimeter, first half: " + halfText(cap.half[0]) + " (" + took +
            "). Lift it clear, turn it end for end and lay the other half.");
    }

    // Both halves in: joined, judged and filed.
    void finishPerimeter(const PerimeterHalf& first, const PerimeterHalf& second, bool whole,
                         const Template& active, int index, int garment, const cv::Mat& frame) {
        const Step& s = active.steps[static_cast<std::size_t>(index)];
        const PerimeterResult r = joinPerimeter(first, second, s.measure == "traced", whole);
        StepResult step = judgePerimeter(active, index, r);
        step.garment = garment;
        {
            std::lock_guard<std::mutex> guard(lock_);
            perimHave_ = 2;
            perimHalf_[0] = first;
            perimHalf_[1] = second;
            perimWhole_ = whole;
            perimTotalMm_ = r.measured_mm();
            if (whole) {
                const PerimeterHalf both[2] = {first, second};
                perimeterShowLocked(both, 2);
            } else {
                perimeterShowLocked(&second, 1);
            }
            perimMessage_ = "Perimeter " + mmText(r.measured_mm()) +
                            (whole ? ", both halves from one picture: " : ": ") + mmText(r.halfMm(0)) + " + " +
                            mmText(r.halfMm(1)) + ".";
            perimBad_ = false;
        }
        vjson::Value record = step.toJson();
        record.set("perimeter", r.toJson());
        fileStep(step, index, garment, record, frame);
        writeRecord(record);
    }

    // The picture, the outline and everything a half was read with, left
    // behind when it could not be read - in the form perimeter_check replays,
    // so the case can be looked into with no garment and no camera. One pair
    // of files, overwritten each time.
    void savePerimeterFailure(const MeasureResult& result, const cv::Mat& frame, const RoiBox& roi,
                              const ClipColour& colour, const std::string& why) {
        const std::string png = "perimeter-fail.png";
        const std::string json = "perimeter-fail.json";
        const PlaneModelPtr model = engine_->model();
        const std::vector<int> fast{cv::IMWRITE_PNG_COMPRESSION, 1};
        if (frame.empty() || !model || !cv::imwrite(png, frame, fast)) {
            log("Could not write " + png);
            return;
        }
        vjson::Value v = vjson::Value::object();
        v.set("frame", vjson::Value::str(png));
        v.set("calibration", vjson::Value::str(cfg_.calibration_file));
        vjson::Value area = vjson::Value::array();
        area.push(vjson::Value::integer(roi.x1));
        area.push(vjson::Value::integer(roi.y1));
        area.push(vjson::Value::integer(roi.x2));
        area.push(vjson::Value::integer(roi.y2));
        v.set("roi", area);
        v.set("correction", vjson::Value::real(result.parallax_correction));
        vjson::Value centre = vjson::Value::array();
        centre.push(vjson::Value::real(model->centre().x));
        centre.push(vjson::Value::real(model->centre().y));
        v.set("centre_px", centre);
        vjson::Value hue = vjson::Value::array();
        hue.push(vjson::Value::real(colour.hue_lo_deg));
        hue.push(vjson::Value::real(colour.hue_hi_deg));
        v.set("clip_hue_deg", hue);
        v.set("join_side", vjson::Value::str(cfg_.perimeter_join_side));
        vjson::Value outline = vjson::Value::array();
        for (const cv::Point2d& p : result.outline_px) {
            vjson::Value a = vjson::Value::array();
            a.push(vjson::Value::real(p.x));
            a.push(vjson::Value::real(p.y));
            outline.push(a);
        }
        v.set("outline_px", outline);
        v.set("refused", vjson::Value::str(why));
        if (!vjson::writeFile(json, v)) {
            log("Could not write " + json);
            return;
        }
        log("Saved " + png + " and " + json + ". To see why, offline and without a camera:");
        log("  perimeter_check --case " + json + " --draw perimeter-out.png");
    }

    // The guide for laying a half, once a frame, from the live outline: what
    // is inside a margin, where the clips are and whether they are in the band.
    void servicePerimeterGuide(const cv::Mat& frame) {
        ClipColour colour = clipColourFrom(cfg_);
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!perimeterStepLocked()) {
                perimGuide_ = PerimeterGuide();
                return;
            }
            colour = perimeterColour(template_.steps[static_cast<std::size_t>(stepIndex_)]);
        }
        const PlaneModelPtr model = engine_->model();
        if (!model || !engine_->hasRoi() || frame.empty()) {
            std::lock_guard<std::mutex> guard(lock_);
            perimGuide_ = PerimeterGuide();
            perimGuide_.message = "Not calibrated: clear the mat and press Recalibrate";
            return;
        }
        const RoiBox roi = engine_->roi();
        const std::string calibratedAt = model->meta().getString("calibrated_at");
        bool needLines = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            needLines = !havePerimLines_ || perimLinesModel_ != model.get() ||
                        perimLinesCalibratedAt_ != calibratedAt || perimLinesRoi_.x1 != roi.x1 ||
                        perimLinesRoi_.y1 != roi.y1 || perimLinesRoi_.x2 != roi.x2 || perimLinesRoi_.y2 != roi.y2;
        }
        if (needLines) {
            const PerimeterGuideLines lines = perimeterGuideLines(*model, roi, cfg_);
            std::lock_guard<std::mutex> guard(lock_);
            perimLines_ = lines;
            havePerimLines_ = true;
            perimLinesModel_ = model.get();
            perimLinesCalibratedAt_ = calibratedAt;
            perimLinesRoi_ = roi;
        }
        const DebugState& dbg = engine_->lastDebug();
        std::vector<cv::Point2d> live;
        if (dbg.has_contour) {
            const double halfCell = 0.5 * (dbg.step - 1);
            live.reserve(dbg.contour.size());
            for (const cv::Point& p : dbg.contour) {
                live.emplace_back(p.x * dbg.step + halfCell + roi.x1, p.y * dbg.step + halfCell + roi.y1);
            }
        }
        PerimeterGuide g;
        evaluatePerimeterGuide(live, frame, *model, roi, colour, cfg_, g);
        std::lock_guard<std::mutex> guard(lock_);
        perimGuide_ = g;
    }

    // Everything the page needs for the perimeter strip. Call with lock_ held.
    // `awaiting` is whether the engine will let go of a held lay by itself for
    // the next one laid (MeasurementEngine::awaitingNewLay).
    vjson::Value perimeterJsonLocked(bool held, bool awaiting) const {
        vjson::Value p = vjson::Value::object();
        const bool traced = perimeterStepLocked() &&
                            template_.steps[static_cast<std::size_t>(stepIndex_)].measure == "traced";
        const char* which = perimHave_ == 1 ? "second" : "first";
        p.set("phase", vjson::Value::str(perimHave_ == 0 ? "first" : perimHave_ == 1 ? "second" : "done"));
        std::string message;
        bool bad = false;
        if (held && heldForTake_ && !perimMessage_.empty()) {
            // What the reading on the mat said.
            message = perimMessage_;
            bad = perimBad_;
        } else if (held && !heldForTake_ && !awaiting) {
            // The engine is holding a lay from before this take, and lets go of
            // it only once the mat has been clear for a moment. When it is
            // waiting for the next lay instead, the guide below is what helps.
            message = std::string("Lift the shirt clear of the mat, then lay the ") + which + " half";
        } else {
            message = perimGuide_.message;
        }
        p.set("message", vjson::Value::str(message));
        p.set("bad", vjson::Value::boolean(bad));
        p.set("guide_ok", vjson::Value::boolean(perimGuide_.ok));
        vjson::Value halves = vjson::Value::array();
        for (int k = 0; k < std::min(perimHave_, 2); ++k) {
            const PerimeterHalf& h = perimHalf_[k];
            vjson::Value e = vjson::Value::object();
            e.set("mm", vjson::Value::real(traced ? h.traced_mm : h.fitted_mm));
            e.set("fitted_mm", vjson::Value::real(h.fitted_mm));
            e.set("traced_mm", vjson::Value::real(h.traced_mm));
            e.set("corners", vjson::Value::integer(h.corners));
            e.set("crossed", vjson::Value::integer(h.crossed));
            halves.push(e);
        }
        p.set("halves", halves);
        if (perimHave_ >= 2) {
            p.set("total_mm", vjson::Value::real(perimTotalMm_));
            p.set("whole", vjson::Value::boolean(perimWhole_));
        }
        return p;
    }

    // Measure straight between the two points the operator put down, and score
    // it.
    //
    // The reading is the two points and nothing else. No seam is looked for,
    // nothing is snapped, nothing is corrected - the number is what a ruler laid
    // on the picture between those two places would say, through the
    // calibration, distortion and all. Which means its accuracy is the
    // operator's aim on the feed, about half a millimetre a pixel at this
    // scale, and the log records both points in pixels and in millimetres so a
    // disputed reading can be put back on the picture.
    void measureDistance() {
        Template active;
        int index = 0;
        int garment = 0;
        std::vector<cv::Point2d> seeds;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_) return;
            active = template_;
            index = stepIndex_;
            garment = garment_;
            seeds = seeds_;
        }
        if (seeds.size() < 2) {
            log("Tap both ends first");
            return;
        }
        const PlaneModelPtr model = pointedModel();
        if (!model) {
            log("Not calibrated, so there are no millimetres to judge");
            return;
        }

        StepResult step = judgeDistance(active, index, seeds[0], seeds[1]);
        step.garment = garment;
        vjson::Value record = step.toJson();

        const std::vector<cv::Point2d> px = model->mmToPx({seeds[0], seeds[1]});
        vjson::Value ptsMm = vjson::Value::array();
        vjson::Value ptsPx = vjson::Value::array();
        for (std::size_t k = 0; k < 2; ++k) {
            vjson::Value a = vjson::Value::array();
            a.push(vjson::Value::real(seeds[k].x));
            a.push(vjson::Value::real(seeds[k].y));
            ptsMm.push(a);
            if (px.size() == 2) {
                vjson::Value b = vjson::Value::array();
                b.push(vjson::Value::real(px[k].x));
                b.push(vjson::Value::real(px[k].y));
                ptsPx.push(b);
            }
        }
        record.set("points_mm", ptsMm);
        record.set("points_px", ptsPx);

        fileStep(step, index, garment, record, heldFrame_);
        writeRecord(record);
        log(step.headline());

        // The line stays on the picture after the reading, because it IS the
        // reading; the next first tap takes it off. The points themselves are
        // cleared so the next two taps are a new measurement and not a third
        // point on this one.
        {
            std::lock_guard<std::mutex> guard(lock_);
            haveDistance_ = step.measured && px.size() == 2;
            distancePx_ = haveDistance_ ? px : std::vector<cv::Point2d>();
            distanceMm_ = step.measured_mm;
            seeds_.clear();
        }
    }

    // Measure along a path the operator drew with the mouse, and score it.
    //
    // The tape to measureDistance's ruler. The path is what was drawn, smoothed
    // of the hand that drew it and nothing more; the number is its length. The
    // drawn points go in the log as they came, so the reading can be put back
    // on the picture and the smoothing re-run against a changed setting.
    void measureTrace(const std::vector<cv::Point2d>& drawnMm) {
        Template active;
        int index = 0;
        int garment = 0;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_) return;
            active = template_;
            index = stepIndex_;
            garment = garment_;
        }
        const PlaneModelPtr model = pointedModel();
        if (!model) {
            log("Not calibrated, so there are no millimetres to judge");
            return;
        }

        PathResult path;
        measurePath(drawnMm, cfg_, path);
        StepResult step = judgePath(active, index, path);
        step.garment = garment;
        vjson::Value record = step.toJson();

        vjson::Value pts = vjson::Value::array();
        for (const cv::Point2d& p : drawnMm) {
            vjson::Value a = vjson::Value::array();
            a.push(vjson::Value::real(p.x));
            a.push(vjson::Value::real(p.y));
            pts.push(a);
        }
        record.set("points_mm", pts);

        fileStep(step, index, garment, record, heldFrame_);
        writeRecord(record);
        log(step.headline());

        {
            std::lock_guard<std::mutex> guard(lock_);
            haveDistance_ = step.measured && path.path_mm.size() >= 2;
            distancePx_ = haveDistance_ ? model->mmToPx(path.path_mm) : std::vector<cv::Point2d>();
            distanceMm_ = step.measured_mm;
            seeds_.clear();
        }
    }

    // Read the seam the operator pointed at, and score it.
    // `byMachine` when the seeds came from the outline rather than the
    // operator: a refusal is then not a verdict on the garment but the machine
    // saying it is not sure, and is logged and handed back to the operator
    // rather than filed as a FAIL nobody can act on. Returns whether a reading
    // was taken.
    bool measureCurve(bool byMachine = false) {
        Template active;
        int index = 0;
        int garment = 0;
        std::vector<cv::Point2d> seeds;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!haveTemplate_) return false;
            active = template_;
            index = stepIndex_;
            garment = garment_;
            seeds = seeds_;
        }
        if (seeds.size() < 2) {
            log("Point at both ends of the seam first");
            return false;
        }

        CurveResult curve;
        const PlaneModelPtr model = pointedModel();
        if (!model) {
            curve.error = "not calibrated, so there are no millimetres to judge";
        } else if (heldFrame_.empty()) {
            curve.error = "no held picture to read the seam from";
        } else {
            snapCurve(heldFrame_, *model, seeds, cfg_, curve);
        }

        {
            std::lock_guard<std::mutex> guard(lock_);
            haveCurve_ = curve.ok;
            curvePath_.clear();
            if (curve.ok) seamOptionsPx_.clear();
            if (curve.ok && model) {
                curvePath_ = model->mmToPx(curve.path_mm);
                if (curvePath_.size() < 2) haveCurve_ = false;
            }
        }

        // Found by the machine rather than pointed at: a refusal, or a seam
        // that is not plausibly the one the step is about, is the machine
        // saying it is not sure. The answer to that is to ask the operator -
        // not to file a FAIL on a garment nobody has pointed at yet.
        if (byMachine) {
            std::string why;
            if (!curve.ok) {
                why = curve.error;
            } else if (index >= 0 && index < static_cast<int>(active.steps.size())) {
                const Step& s = active.steps[static_cast<std::size_t>(index)];
                const bool alongArc = s.measure.empty() ? curve.along_arc : (s.measure == "arc");
                const double got = alongArc ? curve.arc_mm : curve.chord_mm;
                if (s.target_mm > 1e-9 && std::fabs(got - s.target_mm) > 0.3 * s.target_mm) {
                    why = "the seam it found is " + vjson::formatFixed(got, 0) +
                          " mm against a target of " + vjson::formatFixed(s.target_mm, 0) +
                          ", which is not this seam";
                }
            }
            if (!why.empty()) {
                log("The neckline the machine found could not be read: " + why + ". Point at it.");
                std::lock_guard<std::mutex> guard(lock_);
                haveCurve_ = false;
                curvePath_.clear();
                seeds_.clear();
                return false;
            }
        }

        // A refusal the operator cannot do anything about is worth being able
        // to take away and look at.
        //
        // The bench that finds this feature wanting is not the one it was
        // written on, and a screenshot of a red FAIL says nothing about why. So
        // a refused read leaves behind exactly what it was given - the held
        // picture, and the three points, in the form curve_check takes - and
        // the whole case can be re-run offline as many times as it takes,
        // against a changed setting or a changed build, without a garment or a
        // camera in the room.
        if (!curve.ok && !heldFrame_.empty()) saveCurveFailure(curve, seeds);

        StepResult step = judgeCurve(active, index, curve);
        step.garment = garment;
        vjson::Value record = step.toJson();
        record.set("curve", curve.toJson());
        fileStep(step, index, garment, record, heldFrame_);
        writeRecord(record);

        // A refusal leaves the points where they are rather than throwing them
        // away. The usual cause is one tap in the wrong place, and asking for
        // all three again to fix one of them is how an operator learns to stop
        // using a feature. Retake clears them.
        //
        // A read that worked clears the points and nothing else, the way a
        // distance does. This used to call clearSeeds, which also takes away
        // the seam just found: set a few dozen lines above and gone before the
        // feed was next drawn, so a curve that passed was never shown at all.
        // It also dropped the picture the pointing is measured against, which
        // was re-taken on the next frame - with a hand in it, whenever the last
        // point had been held down with a fingertip. The seam now stays until
        // the step is left or retaken, and serviceStepChange takes it away
        // then.
        if (curve.ok) {
            std::lock_guard<std::mutex> guard(lock_);
            seeds_.clear();
            // A fingertip still resting on the seam would otherwise lay the
            // first point of a new set the moment this one was read.
            if (finger_) {
                finger_->resetDwell();
                finger_->abandonTrace();
            }
        }
        return curve.ok;
    }

    // What the tracker saw on the last frame of a trace, written out so the
    // trace can be replayed offline.
    //
    // Every trace, not only a refused one, because the trace that matters is
    // the one that LOOKED wrong on the screen and was accepted anyway - the
    // corridor found a seam near enough to whatever the hand drew, and the
    // number came back plausible. Nothing downstream records that the points
    // were a saw-tooth. What the tracker gets wrong is what it sees, and the
    // only way to understand that is to see it too: the settled scene, the
    // frame with the hand in it, the mask it made of the two, and where in
    // that mask it put the tip. One directory, overwritten each time.
    void savePointingBundle(const std::vector<cv::Point2d>& pathMm) {
        if (!finger_) return;
        const FingerTracker::Seen& seen = finger_->lastSeen();
        if (!seen.hasTip || seen.frame.empty() || seen.ref.empty()) return;

        const std::string dir = "pointing-debug";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        const bool wrote = cv::imwrite(dir + "/reference.png", seen.ref) &&
                           cv::imwrite(dir + "/hand.png", seen.frame) &&
                           cv::imwrite(dir + "/mask.png", seen.mask);

        vjson::Value v = vjson::Value::object();
        v.set("mm_per_cell", vjson::Value::real(seen.mmPerCell));
        v.set("step", vjson::Value::integer(seen.step));
        vjson::Value tip = vjson::Value::array();
        tip.push(vjson::Value::real(seen.tipSmall.x));
        tip.push(vjson::Value::real(seen.tipSmall.y));
        v.set("tip_cells", tip);
        vjson::Value trace = vjson::Value::array();
        for (const cv::Point2d& p : pathMm) {
            vjson::Value pair = vjson::Value::array();
            pair.push(vjson::Value::real(p.x));
            pair.push(vjson::Value::real(p.y));
            trace.push(pair);
        }
        v.set("trace_mm", trace);

        if (wrote && vjson::writeFile(dir + "/trace.json", v)) {
            log("Saved " + dir + "/ - what the tracker saw on the last frame. To replay it:");
            log("  curve_check --pointing " + dir);
        } else {
            log("Could not write " + dir);
        }
    }

    // The two pictures a measurement was decided from, on request.
    //
    // A boundary drawn in the wrong place cannot be argued about from the
    // overlay alone. The overlay is a quarter-resolution JPEG with the answer
    // already drawn on it, and every question worth asking - is this the cloth,
    // its shadow, or the printed grid underneath it - needs the frame and the
    // reference it was differenced against, at the resolution the decision was
    // actually taken at. So the bench can be asked for them.
    void dumpMeasurementInput() {
        if (!engine_) {
            log("Nothing to dump: the bench has not started measuring yet.");
            return;
        }
        // What the last full-resolution measurement actually ran on, not the
        // held picture - a step measured from a folded edge never holds one.
        const cv::Mat frame = engine_->lastFineFrame();
        const cv::Mat bg = engine_->lastFineBackground();
        if (frame.empty() || bg.empty()) {
            log("Nothing to dump: nothing has been measured yet. Lay a garment, let it read, "
                "then ask again.");
            return;
        }

        const std::string dir = "measure-debug";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (!cv::imwrite(dir + "/frame.png", frame) ||
            !cv::imwrite(dir + "/background.png", bg)) {
            log("Could not write " + dir);
            return;
        }
        log("Wrote " + dir + "/frame.png and background.png. To see what the segmentation made "
            "of them, offline and without a camera:");
        log("  boundary_probe " + dir);
        // And the whole working area, from the next frame: whether the base has
        // moved under a lay is read on the base round it, which the crop above
        // has only a strip of.
        dumpWholeViewNext_ = true;
    }

    // The whole working area as the engine sees it: this frame, and the
    // background it is measured against (before any following of the base).
    // Written once, after a `dump`.
    void dumpWholeView(const cv::Mat& frame) {
        const cv::Mat bg = engine_ ? engine_->background() : cv::Mat();
        if (bg.empty()) return;
        cv::Rect area(0, 0, frame.cols, frame.rows);
        if (bg.size() != frame.size()) area = engine_->roi().toRect() & area;
        if (area.size() != bg.size()) {
            log("Could not write the whole view: the working area has changed since the background.");
            return;
        }
        const std::string dir = "measure-debug";
        if (!cv::imwrite(dir + "/frame_full.png", frame(area)) ||
            !cv::imwrite(dir + "/background_full.png", bg)) {
            log("Could not write " + dir + "/frame_full.png");
            return;
        }
        log("Wrote " + dir + "/frame_full.png and background_full.png: the whole working area, "
            "now, and the background it is measured against.");
    }

    // The held picture and the points that were pointed at it, as a command.
    // One file, overwritten each time, so a bench that refuses all morning does
    // not fill its disk with mornings.
    void saveCurveFailure(const CurveResult& curve, const std::vector<cv::Point2d>& seeds) {
        const std::string path = "curve-fail.png";
        if (!cv::imwrite(path, heldFrame_)) {
            log("Could not write " + path);
            return;
        }
        std::string args;
        for (std::size_t k = 0; k < seeds.size(); ++k) {
            args += (k ? ";" : "") + vjson::formatFixed(seeds[k].x, 1) + "," +
                    vjson::formatFixed(seeds[k].y, 1);
        }
        log("Refused: " + curve.error);
        log("Saved " + path + ". To see why, offline and without a camera:");
        log("  curve_check --frame " + path + " --calibration " + cfg_.calibration_file +
            " --seeds \"" + args + "\"");
    }

    void writeRecord(const vjson::Value& record) {
        if (!cfg_.results_file.empty() && !appendResultLine(cfg_.results_file, record)) {
            log("WARNING: could not write to " + cfg_.results_file);
        }
        std::lock_guard<std::mutex> guard(lock_);
        results_.insert(results_.begin(), record);
        while (results_.size() > 24) results_.pop_back();
    }

    void printResult(const MeasureResult& r) {
        if (!r.calibrated) {
            std::printf("[#%d] %.0f x %.0f px (uncalibrated)\n", r.object_number, r.length_px,
                        r.breadth_px);
            std::fflush(stdout);
            return;
        }
        std::printf("\n[Object #%d]  %.1f x %.1f mm, %d frames averaged\n", r.object_number,
                    r.length_mm, r.breadth_mm, r.frames_averaged);
        if (r.saturated_fraction > 0.005) {
            // To the operator log, not only to a console nobody at the bench is
            // watching. Clipped white blooms into the pixels beside it and
            // pushes the edge outward - a real millimetre or two - and it is
            // fixed by turning the light down, not by anything in here.
            log("WARNING: " + vjson::formatFixed(r.saturated_fraction * 100.0, 1) +
                "% of the garment is clipped white. That blooms into the mat and reads long. "
                "Turn the light down or shorten the exposure.");
        }
        std::fflush(stdout);
    }

    // -- the step list --

    void resetSequence(bool newGarment) {
        std::lock_guard<std::mutex> guard(lock_);
        stepIndex_ = 0;
        stepResults_.assign(haveTemplate_ ? template_.steps.size() : 0, std::nullopt);
        // The pictures stay on disk; the page stops showing them.
        captureOf_.clear();
        autoAdvancePending_ = false;
        if (newGarment) ++garment_;
        // A fresh start even when it lands on the step already showing.
        ++stepTake_;
        // And a reading held from before it belongs to no step of this one,
        // even with the same number on a new style or a cleared log.
        heldIndex_ = -1;
    }

    // The operator has taken the wheel, so the station lets go of it.
    void cancelAutoAdvance() {
        std::lock_guard<std::mutex> guard(lock_);
        autoAdvancePending_ = false;
    }

    // Clear everything drawn over the picture the moment the step on screen is
    // a different one, or the same one taken again.
    //
    // Everything on the feed is an answer about one step: the outline a reading
    // was taken from, the span measured along it, a seam found between the
    // points, a distance with its number written on it, the points themselves.
    // Carried onto the next step it answers a question nobody is asking any
    // more, in the brightest colours on the screen, over a garment that has
    // usually been refolded under it. That is how a neck width came to be
    // pointed at through the outline of the sleeve lay before it, and how Retake
    // looked as if it had done nothing.
    //
    // Checked once a pass rather than hooked into each control that can change
    // the step. There are seven of those - Next, Back, the strip, the timed move
    // after a pass, Next shirt, a change of style, clearing the log - and the
    // hooks this replaces had already missed some: the timed move cleared
    // nothing at all, and none of them cleared the measured span or a distance
    // line.
    //
    // The engine is not simply released. It holds a reading until the mat is
    // cleared, and let go on a change of step it would measure the fold still
    // lying there from the step just left, and judge it against the step just
    // arrived at - a FAIL on the screen before the operator has touched the
    // cloth. So the reading stays held, its outline is no longer drawn, and a
    // step still waiting for its reading asks the engine for the next lay
    // instead: the moment the garment on the mat is plainly not the lay that
    // reading came from, it is settled and measured for this step, with no
    // need to clear the mat first. Refolding where it lies was leaving the
    // next fold unmeasured, with nothing drawn round it, until the shirt was
    // lifted clear or the station recalibrated.
    //
    // A step that already has its verdict does not ask. Going Back to look at
    // one is not a request to measure it again; Retake is.
    void serviceStepChange() {
        bool waiting = false;
        bool fine = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (garment_ == feedGarment_ && stepIndex_ == feedIndex_ && stepTake_ == feedTake_) {
                return;
            }
            feedGarment_ = garment_;
            feedIndex_ = stepIndex_;
            feedTake_ = stepTake_;
            heldForTake_ = false;
            haveEdge_ = false;
            edgePath_.clear();
            haveDistance_ = false;
            distancePx_.clear();
            distanceMm_ = 0.0;
            // A perimeter's halves belong to the take they were read on.
            clearPerimeterLocked();

            waiting = haveTemplate_ && stepIndex_ >= 0 &&
                      stepIndex_ < static_cast<int>(template_.steps.size()) &&
                      stepIndex_ < static_cast<int>(stepResults_.size()) &&
                      !stepResults_[static_cast<std::size_t>(stepIndex_)].has_value();
            // Retaking a reading this step took itself: that lay was laid for
            // this step, so a fold straightened where it lies is worth reading
            // again. Not on a pointing step, where a hand over the lay is the
            // operator pointing at it rather than a new lay.
            fine = waiting && heldIndex_ == stepIndex_ && heldGarment_ == garment_ &&
                   !pointedStepLocked();
        }
        // The seam, the points, the trace and the finger. This also drops the
        // picture the points are put on, so a pointing step takes a new one of
        // the garment as it lies now - which is what makes Retake on a pointing
        // step an actual retake. Takes lock_ itself.
        clearSeeds();
        if (waiting) {
            engine_->awaitNewLay(fine);
        } else {
            engine_->keepHeldLay();
        }
    }

    // Move on from a step that has passed, once the operator has had long
    // enough to see why. Called every frame; does nothing almost every time.
    //
    // Everything the move depends on is re-checked here rather than trusted
    // from when it was armed, because the whole point of the delay is that
    // things can happen during it.
    void serviceAutoAdvance() {
        std::string moved;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!autoAdvancePending_) return;
            if (nowSeconds() < autoAdvanceAt_) return;
            autoAdvancePending_ = false;

            const int idx = autoAdvanceIndex_;
            const bool stillTheSameFold =
                haveTemplate_ && idx >= 0 && stepIndex_ == idx &&
                garment_ == autoAdvanceGarment_ &&
                stepResults_.size() == template_.steps.size() &&
                idx < static_cast<int>(stepResults_.size()) &&
                stepResults_[static_cast<std::size_t>(idx)].has_value() &&
                stepResults_[static_cast<std::size_t>(idx)]->pass &&
                idx + 1 < static_cast<int>(template_.steps.size());
            if (!stillTheSameFold) return;

            ++stepIndex_;
            moved = "Passed. Step " + std::to_string(stepIndex_ + 1) + " of " +
                    std::to_string(template_.steps.size()) + ", " +
                    template_.steps[static_cast<std::size_t>(stepIndex_)].name;
        }
        // log() takes lock_, so it cannot be called while holding it.
        log(moved);
    }

    void handleCommand() {
        std::string name, arg;
        {
            std::lock_guard<std::mutex> guard(lock_);
            name.swap(command_);
            arg.swap(commandArg_);
        }
        if (name.empty()) return;

        // A recalibration is replacing the model and the background under
        // these. A point put down now would be read through the model on its
        // way out, so they wait. Moving through the steps does not care either
        // way and is left alone.
        if (calibrationRunning() && (name == "verify" || name == "mark" || name == "trace" ||
                                     name == "mark_measure" || name == "box" || name == "collar_pick")) {
            log("Recalibrating. Try that again when it has finished.");
            return;
        }

        // Anything that decides where in the list we are settles it by hand,
        // so a move the station had queued up is no longer wanted.
        //
        // What was drawn for the step being left - the points among it, which
        // carried across a move would put a corridor over the wrong part of the
        // shirt - is cleared by serviceStepChange once the command has run, and
        // only if the step really did change. Next on the last step, which the
        // Enter key can still send, used to throw away every point put down on
        // it.
        if (name == "next" || name == "back" || name == "retake" || name == "next_shirt" ||
            name == "template" || name == "clear" || name == "manual_steps" || name == "flag_manual_qc") {
            cancelAutoAdvance();
        }

        if (name == "verify") {
            verifyScale(arg);

        } else if (name == "scale_check") {
            scaleCheckCommand(arg);

        } else if (name == "advance_now") {
            // The page's "Start now": the move a pass has already armed, taken
            // now instead of at the end of its wait. It goes through
            // serviceAutoAdvance, so it is re-checked exactly as the timed move
            // would be. And if the move has already happened it does nothing,
            // where a Next that arrived a moment after the wait ran out would
            // move a second step and skip one.
            std::lock_guard<std::mutex> guard(lock_);
            const int from = std::atoi(arg.c_str()) - 1;
            if (autoAdvancePending_ && (arg.empty() || from == autoAdvanceIndex_)) {
                autoAdvanceAt_ = nowSeconds();
            }

        } else if (name == "goto") {
            // Straight to a step, from the strip along the top of the panel.
            // Absolute rather than a run of Nexts, so it cannot overshoot: a
            // page a moment behind that asks for the step the station has just
            // moved to lands on that step, not the one after it. Asking for the
            // step already showing changes nothing - in particular it leaves a
            // move that a pass has armed exactly as it was.
            const int target = std::atoi(arg.c_str()) - 1;
            bool moving = false;
            {
                std::lock_guard<std::mutex> guard(lock_);
                moving = haveTemplate_ && target >= 0 &&
                         target < static_cast<int>(template_.steps.size()) &&
                         target != stepIndex_;
            }
            if (moving) {
                // What any other move by hand does: the operator has the wheel.
                // The feed is cleared by serviceStepChange, as for every move.
                cancelAutoAdvance();
                std::string moved;
                {
                    std::lock_guard<std::mutex> guard(lock_);
                    if (haveTemplate_ && target < static_cast<int>(template_.steps.size())) {
                        stepIndex_ = target;
                        moved = "Step " + std::to_string(stepIndex_ + 1) + " of " +
                                std::to_string(template_.steps.size()) + ", " +
                                template_.steps[static_cast<std::size_t>(stepIndex_)].name;
                    }
                }
                if (!moved.empty()) log(moved);
            }

        } else if (name == "dump") {
            dumpMeasurementInput();

        } else if (name == "mark") {
            markFromPage(arg);

        } else if (name == "trace") {
            traceFromPage(arg);

        } else if (name == "box") {
            collarFromPage(arg);

        } else if (name == "collar_pick") {
            collarPickFromPage(arg);

        } else if (name == "collar_clear") {
            {
                std::lock_guard<std::mutex> guard(lock_);
                clearCollarLocked();
            }
            log("Box cleared");

        } else if (name == "flag_manual_qc") {
            flagForManualQc();

        } else if (name == "advance_delay") {
            // From the page's Settings: how long a pass is counted down before
            // the station moves on, 1-30 s. A move already counting keeps its
            // own time; the next pass takes the new one.
            char* end = nullptr;
            const double v = std::strtod(arg.c_str(), &end);
            if (end != arg.c_str() && v == v) {
                const double s = std::min(30.0, std::max(1.0, v));
                cfg_.auto_advance_delay_s = s;
                log("Time before the next step: " + std::to_string(std::lround(s)) + " s");
            }

        } else if (name == "manual_steps") {
            // From the admin panel: the hand-drawn collar steps back on the
            // bench, or off it again. "on", "off", or nothing to switch.
            bool show = false;
            {
                std::lock_guard<std::mutex> guard(lock_);
                show = arg == "on" || (arg != "off" && !showManual_);
            }
            setManualSteps(show);
            std::string style;
            std::size_t count = 0;
            {
                std::lock_guard<std::mutex> guard(lock_);
                style = haveTemplate_ ? template_.style : std::string();
                count = template_.steps.size();
            }
            log(std::string(show ? "Hand-drawn collar steps shown" : "Hand-drawn collar steps hidden") +
                (style.empty() ? std::string() : ": " + style + ", " + std::to_string(count) +
                                                     " steps, back to step 1"));

        } else if (name == "mark_undo") {
            std::size_t left = 0;
            {
                std::lock_guard<std::mutex> guard(lock_);
                if (!seeds_.empty()) seeds_.pop_back();
                left = seeds_.size();
                curvePath_.clear();
                haveCurve_ = false;
                if (finger_) finger_->resetDwell();
            }
            log("Point taken back, " + std::to_string(left) + " left");

        } else if (name == "mark_clear") {
            clearSeeds();
            log("Points cleared");

        } else if (name == "mark_measure") {
            // For a seam that wants a different number of points than the step
            // count expects: a straight collar band needs two, a complicated
            // neckline is better traced with five.
            if (distanceStep()) {
                measureDistance();
            } else {
                measureCurve();
            }

        } else if (name == "next") {
            std::string moved;
            {
                std::lock_guard<std::mutex> guard(lock_);
                if (haveTemplate_ && stepIndex_ + 1 < static_cast<int>(template_.steps.size())) {
                    ++stepIndex_;
                    moved = "Step " + std::to_string(stepIndex_ + 1) + " of " +
                            std::to_string(template_.steps.size()) + ", " +
                            template_.steps[static_cast<std::size_t>(stepIndex_)].name;
                }
            }
            if (!moved.empty()) log(moved);

        } else if (name == "back") {
            std::string moved;
            {
                std::lock_guard<std::mutex> guard(lock_);
                if (haveTemplate_ && stepIndex_ > 0) {
                    --stepIndex_;
                    moved = "Back to step " + std::to_string(stepIndex_ + 1) + ", " +
                            template_.steps[static_cast<std::size_t>(stepIndex_)].name;
                }
            }
            if (!moved.empty()) log(moved);

        } else if (name == "next_shirt") {
            resetSequence(true);
            std::string which;
            {
                std::lock_guard<std::mutex> guard(lock_);
                which = std::to_string(garment_);
            }
            log("Shirt " + which + ", step 1");

        } else if (name == "retake") {
            std::string which;
            bool pointed = false;
            {
                std::lock_guard<std::mutex> guard(lock_);
                // A new take of the same step: everything drawn for the last one
                // comes off the feed, including the outline of the reading the
                // engine is still holding. See serviceStepChange, which also has
                // the engine read the fold as soon as it is laid again - on the
                // mat where it lies, with no need to lift it clear first.
                ++stepTake_;
                if (haveTemplate_ && stepIndex_ < static_cast<int>(stepResults_.size())) {
                    stepResults_[static_cast<std::size_t>(stepIndex_)] = std::nullopt;
                    which = template_.steps[static_cast<std::size_t>(stepIndex_)].name;
                    pointed = template_.steps[static_cast<std::size_t>(stepIndex_)].isPointed();
                }
            }
            if (!which.empty()) {
                std::string how = collarStep() ? ": draw the box again" : ": point again";
                if (perimeterStep()) {
                    // Both halves again: a half read on one take is never
                    // added to a half of another.
                    how = ": lay the first half again";
                } else if (!pointed) {
                    how = ": lay the fold again";
                }
                log("Retaking " + which + how);
            }

        } else if (name == "template") {
            rescanTemplates();
            if (arg.empty()) {
                {
                    std::lock_guard<std::mutex> guard(lock_);
                    haveTemplate_ = false;
                    template_ = Template();
                    fileTemplate_ = Template();
                    fileIndex_.clear();
                    ++shiftEpoch_;  // what is laid next is a new shirt
                }
                resetSequence(false);
                log("No style selected, measuring without a verdict");
            } else {
                std::string error;
                if (selectTemplate(arg, error)) {
                    resetSequence(false);
                    std::string style;
                    std::size_t count = 0;
                    {
                        std::lock_guard<std::mutex> guard(lock_);
                        style = template_.style;
                        count = template_.steps.size();
                        garment_ = 1;
                        ++shiftEpoch_;  // shirt 1 of a style chosen now is a new shirt
                    }
                    log("Now checking " + style + ", " + std::to_string(count) + " steps");
                    reportReach();
                } else {
                    log("Could not load that style: " + error);
                }
            }

        } else if (name == "clear") {
            {
                std::lock_guard<std::mutex> guard(lock_);
                results_.clear();
                garment_ = 1;
                ++shiftEpoch_;
            }
            resetSequence(false);
            engine_->setCount(0);
            log("Log cleared, back to shirt 1 step 1");
        }
    }

    // Check scale: a reference of known length read on the base, and the
    // calibration's scale corrected to agree with it.
    //
    // This is what makes a reading true on a base that is not a ruler. The
    // geometry says how the picture maps onto a plane; only something of known
    // length says the plane is where it was measured - a cloth lies a mat's
    // thickness lower than the mat it replaced, and a camera raised a few
    // millimetres reads everything short by the same fraction.
    //
    // `arg` is the true length in mm, optionally with the reference's
    // thickness after a comma ("300,1.0"). The camera sees the reference's top
    // face, so it is read at that height. It used to be read at the fold
    // height of whichever step was showing: four plies is 0.46% at 700 mm,
    // 3 mm on a 700 mm body, carried into every reading afterwards.
    void verifyScale(const std::string& arg) {
        const PlaneModelPtr model = engine_->model();
        const auto refuse = [this](const std::string& why) {
            log(why);
            std::lock_guard<std::mutex> guard(lock_);
            scaleCheck_.haveOutcome = true;
            scaleCheck_.ok = false;
            scaleCheck_.outcome = why;
        };
        if (!model) {
            refuse("Check scale needs a calibration to correct: press Recalibrate first.");
            return;
        }
        char* endp = nullptr;
        const double trueMm = std::strtod(arg.c_str(), &endp);
        if (arg.empty() || endp == arg.c_str() || !(trueMm > 0.0)) {
            refuse("Check scale needs the reference's true length in mm.");
            return;
        }
        double thickness = cfg_.scale_reference_thickness_mm;
        if (*endp == ',') {
            char* end2 = nullptr;
            const double t = std::strtod(endp + 1, &end2);
            if (end2 != endp + 1 && t >= 0.0 && t < 100.0) thickness = t;
        }

        // The reference as read, on the base's own plane.
        double plane = 0.0;
        bool fromCheck = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (scaleCheck_.active && scaleCheck_.readings > 0) {
                plane = scaleCheck_.readingMm;
                fromCheck = true;
            }
        }
        if (!fromCheck) {
            const std::optional<MeasureResult>& last = engine_->lastResult();
            if (!last.has_value() || !last->calibrated) {
                refuse("Lay the reference on the base and let it read first.");
                return;
            }
            plane = last->length_mm / (last->parallax_correction > 0.0 ? last->parallax_correction : 1.0);
        }
        const double distance = engine_->config().camera_distance_mm.value_or(0.0);
        const auto topFace = [&](double onPlane) {
            return distance > thickness && thickness > 0.0 ? onPlane * (distance - thickness) / distance
                                                           : onPlane;
        };
        double read = topFace(plane);
        if (!(read > 0.0)) {
            refuse("The reference has not been read yet.");
            return;
        }
        double factor = trueMm / read;

        // Read in more than one part of the base: one correction for all of
        // them, or none. A scale error is the same everywhere, so every place
        // asks for the same correction; places that ask for different ones
        // are the lens or the corners of the picture, and a factor averaged
        // over them would be right nowhere.
        std::vector<ScaleCheck::Place> places;
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (fromCheck) places = scaleCheck_.places;
        }
        vjson::Value usedPlaces = vjson::Value::array();
        if (places.size() >= 2) {
            double lo = 0.0, hi = 0.0, sum = 0.0;
            std::string each;
            for (std::size_t k = 0; k < places.size(); ++k) {
                const double f = trueMm / topFace(places[k].mm);
                lo = k == 0 ? f : std::min(lo, f);
                hi = k == 0 ? f : std::max(hi, f);
                sum += f;
                each += (k ? ", " : "") + places[k].where + " " + (f >= 1.0 ? "+" : "") +
                        vjson::formatFixed((f - 1.0) * 100.0, 2) + "%";
                vjson::Value p = vjson::Value::object();
                p.set("where", vjson::Value::str(places[k].where));
                p.set("read_mm", vjson::Value::real(topFace(places[k].mm)));
                p.set("correction", vjson::Value::real(f));
                usedPlaces.push(p);
            }
            if (!(hi / lo - 1.0 <= cfg_.scale_check_agree)) {
                refuse("Not applied: the reference asks for a different correction in each place - " +
                       each + ". A scale error is the same everywhere; this is the lens or the "
                       "corners of the picture, which no one factor mends. Fit the lens distortion "
                       "(verimeasure_cli --fit-distortion), recalibrate on the rigid mat and check "
                       "again. Nothing was changed.");
                return;
            }
            factor = sum / static_cast<double>(places.size());
            read = trueMm / factor;
        }

        // The rigid mat against the cloth was 1.25%, a camera 20 mm higher
        // would be 3%. Five per cent is a length typed wrong, a reference read
        // with something else, or a geometry that no longer fits the camera -
        // none of them mended by scaling every reading to agree with it.
        if (!(factor > 0.95 && factor < 1.05)) {
            refuse("The reference reads " + vjson::formatFixed(read, 1) + " mm against the " +
                   vjson::formatFixed(trueMm, 1) + " mm typed: a " +
                   vjson::formatFixed((factor - 1.0) * 100.0, 1) +
                   "% correction, more than any scale error. Check the length typed and that only "
                   "the reference was read. If both are right, the geometry is: lay the rigid mat "
                   "and press Recalibrate. Nothing was changed.");
            return;
        }

        model->setScaleCorrection(model->scaleCorrection() * factor);
        model->meta().set("scale_verified_against_mm", vjson::Value::real(trueMm));
        model->meta().set("scale_verified_at", vjson::Value::str(timestampIso()));
        model->meta().set("scale_verified_read_mm", vjson::Value::real(read));
        model->meta().set("scale_reference_thickness_mm", vjson::Value::real(thickness));
        // Which readings the correction came from, when it came from several.
        if (places.size() >= 2) model->meta().set("scale_verified_places", usedPlaces);
        saveCalibration(*model, cfg_);

        const std::string readText =
            places.size() >= 2
                ? " mm reference read the same, to within " +
                      vjson::formatFixed(cfg_.scale_check_agree * 100.0, 1) + "%, in " +
                      std::to_string(places.size()) + " places (" + vjson::formatFixed(read, 2) +
                      " mm on average)"
                : " mm reference read " + vjson::formatFixed(read, 2) + " mm";
        const std::string said =
            "Scale corrected by " + std::string(factor >= 1.0 ? "+" : "") +
            vjson::formatFixed((factor - 1.0) * 100.0, 2) + "%: the " + vjson::formatFixed(trueMm, 1) +
            readText + ", and now reads " +
            vjson::formatFixed(trueMm, 2) + " mm. Every reading from now on is corrected the same "
            "way, and the correction is saved with the calibration.";
        log(said);
        std::lock_guard<std::mutex> guard(lock_);
        if (fromCheck) {
            scaleCheck_.readingMm *= factor;
            scaleCheck_.breadthMm *= factor;
            for (ScaleCheck::Place& p : scaleCheck_.places) {
                p.mm *= factor;
                p.breadth *= factor;
            }
        }
        scaleCheck_.haveOutcome = true;
        scaleCheck_.ok = true;
        scaleCheck_.outcome = said;
    }

    // "start" opens a scale check, "stop" closes it.
    void scaleCheckCommand(const std::string& arg) {
        if (arg == "start") {
            if (!engine_->model()) {
                log("Check scale needs a calibration: press Recalibrate first.");
                return;
            }
            {
                std::lock_guard<std::mutex> guard(lock_);
                const long long seq = scaleCheck_.seq + 1;
                scaleCheck_ = ScaleCheck();
                scaleCheck_.active = true;
                scaleCheck_.seq = seq;
            }
            cancelAutoAdvance();
            // A reference laid where the last garment lay is read the way a
            // re-laid garment is, without the base having to be cleared first.
            engine_->awaitNewLay(false);
            log("Checking the scale. Lay a steel rule flat on the base, clear of everything else, "
                "and let it read.");
        } else if (arg == "stop") {
            bool was = false;
            {
                std::lock_guard<std::mutex> guard(lock_);
                was = scaleCheck_.active;
                scaleCheck_.active = false;
            }
            if (!was) return;
            // And a garment laid where the reference lay is read for the step.
            engine_->awaitNewLay(false);
            log("Scale check closed");
        }
    }

    bool scaleCheckActive() const {
        std::lock_guard<std::mutex> guard(lock_);
        return scaleCheck_.active;
    }

    // A reading taken while the scale is being checked is the reference's, and
    // goes here instead of to a step. False when no check is open.
    bool takeScaleReading(const MeasureResult& r) {
        if (!r.calibrated) return scaleCheckActive();
        const double pc = r.parallax_correction > 0.0 ? r.parallax_correction : 1.0;
        const std::string where = scalePlace(r);
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!scaleCheck_.active) return false;
            scaleCheck_.readingMm = r.length_mm / pc;
            scaleCheck_.breadthMm = r.breadth_mm / pc;
            ++scaleCheck_.readings;
            scaleCheck_.haveOutcome = false;
            // A second reading in the same place replaces the first, so a lay
            // that went wrong can be laid again.
            ScaleCheck::Place* same = nullptr;
            for (ScaleCheck::Place& p : scaleCheck_.places) {
                if (p.where == where) same = &p;
            }
            if (!same) {
                scaleCheck_.places.push_back(ScaleCheck::Place());
                same = &scaleCheck_.places.back();
                same->where = where;
            }
            same->mm = scaleCheck_.readingMm;
            same->breadth = scaleCheck_.breadthMm;
        }
        log("Reference read: " + vjson::formatFixed(r.length_mm / pc, 2) + " mm end to end, " +
            vjson::formatFixed(r.breadth_mm / pc, 1) + " mm across, on the base's plane, " +
            (where == "centre" ? "in the middle" : "toward the " + where));
        return true;
    }

    // Which part of the base a reference was read in: the middle, or toward
    // one side of the working area, by where the middle of its outline lies.
    std::string scalePlace(const MeasureResult& r) const {
        if (r.outline_px.empty() || !engine_->hasRoi()) return "centre";
        cv::Point2d c(0.0, 0.0);
        for (const cv::Point2d& p : r.outline_px) c += p;
        c *= 1.0 / static_cast<double>(r.outline_px.size());
        const RoiBox& roi = engine_->roi();
        const double u = (c.x - roi.x1) / std::max(1, roi.width()) - 0.5;
        const double v = (c.y - roi.y1) / std::max(1, roi.height()) - 0.5;
        if (std::max(std::abs(u), std::abs(v)) < 0.15) return "centre";
        if (std::abs(u) >= std::abs(v)) return u < 0.0 ? "left" : "right";
        return v < 0.0 ? "top" : "bottom";
    }

    // What the page draws the scale check from. Call with lock_ held.
    vjson::Value scaleCheckJsonLocked() const {
        vjson::Value c = vjson::Value::object();
        c.set("active", vjson::Value::boolean(scaleCheck_.active));
        c.set("seq", vjson::Value::integer(scaleCheck_.seq));
        c.set("readings", vjson::Value::integer(scaleCheck_.readings));
        c.set("reading_mm", scaleCheck_.readings > 0 ? vjson::Value::real(scaleCheck_.readingMm)
                                                     : vjson::Value::null());
        c.set("breadth_mm", scaleCheck_.readings > 0 ? vjson::Value::real(scaleCheck_.breadthMm)
                                                     : vjson::Value::null());
        c.set("thickness_mm", vjson::Value::real(cfg_.scale_reference_thickness_mm));
        vjson::Value places = vjson::Value::array();
        for (const ScaleCheck::Place& p : scaleCheck_.places) {
            vjson::Value e = vjson::Value::object();
            e.set("where", vjson::Value::str(p.where));
            e.set("reading_mm", vjson::Value::real(p.mm));
            e.set("breadth_mm", vjson::Value::real(p.breadth));
            places.push(e);
        }
        c.set("places", places);
        c.set("agree_pct", vjson::Value::real(cfg_.scale_check_agree * 100.0));
        if (scaleCheck_.haveOutcome) {
            vjson::Value o = vjson::Value::object();
            o.set("ok", vjson::Value::boolean(scaleCheck_.ok));
            o.set("text", vjson::Value::str(scaleCheck_.outcome));
            c.set("outcome", o);
        } else {
            c.set("outcome", vjson::Value::null());
        }
        return c;
    }

    // -- recalibrating --
    //
    // This used to run inline in handleCommand, on the capture thread, which
    // is also the thread that publishes. For the whole wait for a still mat
    // and the whole fit the feed froze and the status stopped moving - even the
    // line saying it had started only reached the page once it was over. From
    // the bench that looks exactly like a station that has hung, and the
    // natural response is to press the button again, which queued a second
    // run behind the first.
    //
    // So it is split three ways. The HTTP thread asks (requestCalibration).
    // The capture thread starts a worker and keeps the feed live while it
    // runs. The worker does exactly what the inline code did - the same
    // combined settle-and-bank capture, the same fast lattice detection and
    // fit, on the same frames - and leaves its answer for the capture thread
    // to put into service, because that is the only thread the engine is ever
    // touched from. Nothing about the calibration itself changed, including
    // how long it takes.

    bool calibrationRunning() const {
        std::lock_guard<std::mutex> guard(lock_);
        return calib_.active;
    }

    // Start a requested recalibration, or apply one that has finished. Called
    // on every pass of the capture loop, and does nothing almost every time.
    void serviceCalibration() {
        bool start = false;
        bool finished = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            start = calib_.active && !calib_.started;
            if (start) {
                calib_.started = true;
                calib_.phase = "settle";
            }
            finished = calibFinished_;
        }

        if (start) {
            // A metering level asked for from Settings comes with this run: in
            // service before the worker copies the config below.
            std::optional<double> level;
            {
                std::lock_guard<std::mutex> guard(lock_);
                level.swap(pendingExposureLevel_);
            }
            if (level.has_value()) applyExposureLevel(*level);
            // The points were put down on whatever was on the mat, and the mat
            // is being cleared for this.
            clearSeeds();
            log("Recalibrating. Keep the mat clear until the background is taken.");
            // What the worker judges the base by, taken here, where the engine
            // lives: the printed pitch to size its squares by, the working area
            // to count them in, and whether there is a geometry to keep.
            BaseCheck check;
            const PlaneModelPtr current = engine_->model();
            check.pitchPx = current ? current->detectionNumber("pitch_px", 0.0) : 0.0;
            check.haveArea = engine_->hasRoi();
            if (check.haveArea) check.area = engine_->roi();
            check.keepGeometry = current != nullptr && !cfg_.dark_grid_sets_geometry;
            if (calibThread_.joinable()) calibThread_.join();
            calibThread_ = std::thread(&Station::calibrationWorker, this, cfg_, check);
            return;
        }
        if (finished) applyCalibration();
    }

    struct BaseCheck {
        double pitchPx = 0.0;
        RoiBox area;
        bool haveArea = false;
        // A geometry is in service, and a grid printed as a working base rather
        // than a ruler does not replace it. See cfg.dark_grid_sets_geometry.
        bool keepGeometry = false;
    };

    // The slow part, on its own thread. It reads frames from the camera
    // stream, which is safe from any thread, works on its own copy of the
    // config, and never touches the engine.
    //
    // What the base is decides what it gives. The rigid mat is a ruler: its
    // grid is fitted, and the fit becomes the geometry. The blue cloth - a grid
    // printed dark - is a working base: with a geometry already in service it
    // gives the exposure, the background and the working area, and its grid is
    // only read for its pitch, to tell whether the camera has moved. Fitted, it
    // measured 1.25% to 2.8% long against the mat, and a creased one does not
    // fit at all: 88 of 272 points agreeing on the bench's own frame of it.
    void calibrationWorker(Config cfg, BaseCheck check) {
        cv::Mat background;
        PlaneModelPtr model;
        std::string error;
        ExposureChange exposure;
        LatticeInfo lattice;
        bool baseOnly = false;
        bool cancelled = false;
        const RoiBox* area = check.haveArea ? &check.area : nullptr;
        try {
            // Nothing is metered and no background taken while something lies
            // on the base. The exposure would be set for the garment, and the
            // garment would become part of the empty base, invisible to every
            // measurement after it. So this waits for it to be lifted off.
            if (waitForClearBase(cfg, check.pitchPx, area, error, cancelled)) {
                exposure = remeterExposure(cfg);
                // Too dark to set an exposure for is too dark to calibrate by.
                // The grid can often still be read in a dim room, and the
                // calibration would then be kept along with an exposure several
                // times too long for the lamp - every garment afterwards
                // clipped white. Refused, everything stays as it was, the
                // exposure included.
                if (exposure.tooDark) {
                    error = "the picture is too dark even at the camera's longest exposure - turn "
                            "the light on, then press Recalibrate again";
                } else {
                    background = captureStaticBackground(*source_, cfg, [this](int done, int needed) {
                        std::lock_guard<std::mutex> guard(lock_);
                        calib_.done = done;
                        calib_.needed = needed;
                    });
                    // Laid down while the mat was settling, and held still
                    // enough to settle with it.
                    const double hidden =
                        stop_.load() ? -1.0 : gridHiddenFraction(background, check.pitchPx, cfg, area);
                    if (stop_.load()) {
                        error = "the station is shutting down";
                    } else if (hidden > cfg.base_hidden_max_fraction) {
                        error = "something was lying on the base when the background was taken (" +
                                percentText(hidden) +
                                " of the printed grid hidden) - lift it off, then press Recalibrate again";
                        background.release();
                    } else {
                        {
                            std::lock_guard<std::mutex> guard(lock_);
                            calib_.phase = "fit";
                        }
                        log("Background taken, reading the printed grid");
                        if (check.keepGeometry && gridPrintedDark(background, cfg)) {
                            baseOnly = true;
                            readGridPitch(background, cfg, lattice);
                            lattice.dark_lines = true;
                        } else {
                            model = autoCalibrate(background, cfg, true, &error, &lattice);
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            model = nullptr;
            baseOnly = false;
            error = e.what();
        } catch (...) {
            model = nullptr;
            baseOnly = false;
            error = "an unexpected error while reading the grid";
        }

        std::lock_guard<std::mutex> guard(lock_);
        calibModel_ = model;
        calibBackground_ = background;
        calibError_ = error;
        calibExposure_ = exposure;
        calibLattice_ = lattice;
        calibBaseOnly_ = baseOnly;
        calibCancelled_ = cancelled;
        calibFinished_ = true;
    }

    // Before a recalibration touches anything: wait until nothing on the base
    // hides its printed grid. True at once on a clear base. While something
    // lies there the page is told how much of the grid it hides, and this
    // keeps looking until it is lifted off, the run is called off, or
    // cfg.recalibrate_clear_wait_s goes by - so Recalibrate pressed with a
    // shirt still down starts by itself once the shirt is lifted, rather than
    // failing and having to be pressed again.
    bool waitForClearBase(const Config& cfg, double pitchPx, const RoiBox* area, std::string& error,
                          bool& cancelled) {
        // Nothing to size the squares by - never calibrated - is left to the
        // grid, as it always was: covered, it cannot be read.
        if (!(pitchPx > 0.0)) return true;
        const double deadline = nowSeconds() + std::max(0.0, cfg.recalibrate_clear_wait_s);
        long long seq = -1;
        bool waiting = false;
        for (;;) {
            cv::Mat frame;
            long long newSeq = 0;
            if (!source_->readNew(seq, frame, newSeq, 8.0)) {
                error = "no picture arrived from the camera";
                return false;
            }
            seq = newSeq;
            // Negative when it cannot say, which is not a reason to wait.
            const double hidden = gridHiddenFraction(frame, pitchPx, cfg, area);
            if (hidden <= cfg.base_hidden_max_fraction) {
                if (waiting) {
                    log("The base is clear, recalibrating");
                    std::lock_guard<std::mutex> guard(lock_);
                    calib_.phase = "settle";
                    calib_.hidden = 0.0;
                }
                return true;
            }
            if (!waiting) {
                log("Something is lying on the base: " + percentText(hidden) +
                    " of the printed grid is hidden. Lift everything off - recalibrating starts as "
                    "soon as the base is clear.");
                waiting = true;
            }
            {
                std::lock_guard<std::mutex> guard(lock_);
                calib_.phase = "clear";
                calib_.hidden = hidden;
            }
            if (stop_.load()) {
                error = "the station is shutting down";
                return false;
            }
            if (calibCancel_.load()) {
                cancelled = true;
                error = "cancelled while waiting for the base to be cleared";
                return false;
            }
            if (nowSeconds() > deadline) {
                error = "the base was not cleared: " + percentText(hidden) +
                        " of its printed grid was still hidden after " +
                        vjson::formatFixed(cfg.recalibrate_clear_wait_s, 0) +
                        " s. Lift everything off it, then press Recalibrate again";
                return false;
            }
        }
    }

    // 0.004 as "0.4%", 0.42 as "42%".
    static std::string percentText(double fraction) {
        const double pct = 100.0 * fraction;
        return vjson::formatFixed(pct, pct < 10.0 ? 1 : 0) + "%";
    }

    // Put a finished recalibration into service, here on the capture thread.
    //
    // A failed one changes nothing. The model, the background and the working
    // area all stay as they were, so a mat that could not be read leaves the
    // bench measuring as it was rather than not measuring at all.
    void applyCalibration() {
        if (calibThread_.joinable()) calibThread_.join();

        PlaneModelPtr model;
        cv::Mat bg;
        std::string error;
        double startedAt = 0.0;
        ExposureChange exposure;
        LatticeInfo lattice;
        bool baseOnly = false;
        bool cancelled = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            model.swap(calibModel_);
            bg = calibBackground_;
            calibBackground_.release();
            error.swap(calibError_);
            exposure = calibExposure_;
            calibExposure_ = ExposureChange();
            lattice = calibLattice_;
            calibLattice_ = LatticeInfo();
            baseOnly = calibBaseOnly_;
            cancelled = calibCancelled_;
            calibBaseOnly_ = false;
            calibCancelled_ = false;
            calibFinished_ = false;
            startedAt = calib_.startedAt;
        }
        const double seconds = nowSeconds() - startedAt;

        if (cancelled) {
            log("Recalibration cancelled before anything was changed");
            restoreExposure(exposure);
            finishCalibration(false, "Recalibration cancelled", "Nothing was changed.", seconds, true,
                              false);
            return;
        }

        // The base only: the geometry in service stays, and everything the
        // base itself decides is taken from it.
        const PlaneModelPtr current = engine_->model();
        if (baseOnly && current && !bg.empty()) {
            applyBaseOnly(current, bg, exposure, lattice, seconds);
            return;
        }

        if (!model) {
            if (error.empty()) error = "the printed grid could not be read";
            // Nothing to keep and a cloth to fit: the one case the cloth has to
            // be the ruler, and the one it most often is not good enough for.
            if (lattice.dark_lines && !current) {
                error += ". A grid printed on cloth seldom fits well enough to calibrate from: lay "
                         "the rigid mat, press Recalibrate, then put the cloth back and press it "
                         "again";
            }
            log("Calibration failed: " + error +
                ". Clear the surface and try again. The previous calibration is still in use.");
            restoreExposure(exposure);
            finishCalibration(false, "Recalibration failed", error, seconds);
            return;
        }

        // Once the new background is in use, the exposure it was taken at has
        // to stay with it, whatever else goes wrong.
        bool newBackground = false;
        // What the calibration being replaced makes of the same view, and a
        // copy of it. A grid can be read cleanly and still be wrong - a printed
        // cloth stretched by its clamps reads its squares a per cent or two off
        // where the rigid mat read them true - and a residual never shows that,
        // because the grid agrees with itself. The comparison does, and the
        // copy means it can be undone.
        CalibrationChange change;
        std::string previousPath;
        try {
            if (const PlaneModelPtr previous = engine_->model()) {
                change = compareCalibrations(*previous, *model, bg.size());
                Config keep = cfg_;
                const std::string& file = cfg_.calibration_file;
                const std::size_t dot = file.rfind('.');
                keep.calibration_file =
                    (dot == std::string::npos ? file : file.substr(0, dot)) + ".prev.json";
                saveCalibration(*previous, keep);
                previousPath = keep.calibration_file;
            }
            saveCalibration(*model, cfg_);
            engine_->setModel(model);
            RoiBox roi;
            bool roiFollowedMat = false;
            if (roiFromModel(*model, bg.size(), cfg_, roi, &bg, &roiFollowedMat)) {
                engine_->setRoi(roi);
                saveRoi(roi, cfg_);
                logRailSource(roiFollowedMat);
            }
            engine_->setBackground(bg);
            newBackground = true;

            // The working distance was worked out from the model this
            // replaces, and a camera that has been moved - which is usually why
            // anybody recalibrates - has moved that too. Two copies of the
            // config carry it: the engine's, for the parallax on a thick part,
            // and the finger tracker's, which is rebuilt from this one the next
            // time a hand is looked for.
            //
            // A distance measured with a tape is not replaced: it does not
            // depend on the printed grid being true, and a new calibration can
            // only have been taken from that grid. What the lens makes of this
            // calibration is said beside it instead.
            double distance = 0.0;
            bool derived = false;
            const bool estimated =
                estimateCameraDistance(*model, bg.size(), cfg_, distance, &derived) && distance != 0.0;
            if (distanceMeasured_ && cfg_.camera_distance_mm.has_value()) {
                engine_->config().camera_distance_mm = *cfg_.camera_distance_mm;
                compareMeasuredDistance(estimated && !derived ? distance : 0.0);
            } else if (estimated) {
                cfg_.camera_distance_mm = distance;
                engine_->config().camera_distance_mm = distance;
                reportWorkingDistance(distance, derived, *model, bg.size());
            }
            finger_.reset();
            clearSeeds();
        } catch (const std::exception& e) {
            error = std::string("the new calibration could not be put into service: ") + e.what();
            log("Calibration failed: " + error);
            if (newBackground) {
                keepExposure(exposure);
            } else {
                restoreExposure(exposure);
            }
            finishCalibration(false, "Recalibration failed", error, seconds);
            return;
        }
        keepExposure(exposure);

        const double rmse = model->metaNumber("rmse_mm", 0.0);
        const double points = model->metaNumber("lattice_points", 0.0);
        char buf[160];
        std::snprintf(buf, sizeof(buf), "Calibrated in %.1f s, residual %.3f mm", seconds, rmse);
        log(buf);
        rebuildDatum();

        std::snprintf(buf, sizeof(buf), "Residual %.3f mm from %.0f grid points.", rmse, points);
        std::string detail = buf;
        // Anything here worth reading keeps the card up until it has been.
        bool warned = false;
        if (change.valid) {
            const auto pct = [](double v) { return (v >= 0.0 ? "+" : "") + vjson::formatFixed(v, 2) + "%"; };
            std::string said = "Against the calibration it replaced, the same spans now measure " +
                               pct(change.centre_pct) + " across the middle and " + pct(change.low_pct) +
                               " to " + pct(change.high_pct) + " toward the edges.";
            // How far apart two calibrations of the same rigid mat already are,
            // on this bench: 0.14% across the middle, up to 0.83% at the edges,
            // where the grid's outermost points are all that pin the lens. A
            // printed cloth stretched by its clamps read 1.25% and 2.8%.
            if (std::abs(change.centre_pct) > 0.5 ||
                std::max(std::abs(change.low_pct), std::abs(change.high_pct)) > 1.5) {
                said += " If the camera has not moved, that is the printed grid: check it with a steel "
                        "rule in the middle and near each edge before trusting it. The previous "
                        "calibration is kept in " + previousPath + ".";
                log("WARNING: " + said);
                warned = true;
            } else {
                log(said);
            }
            detail += " " + said;
        }
        // Not refused - nothing ever has been - but said where the operator
        // will see it, because the console that prints the same warning is
        // not on the tablet.
        if (rmse > cfg_.max_acceptable_rmse_mm) {
            std::snprintf(buf, sizeof(buf),
                          " That is above %.2f mm: check the mat is flat, in focus and free of "
                          "glare.",
                          cfg_.max_acceptable_rmse_mm);
            detail += buf;
            log("WARNING: residual above " + vjson::formatFixed(cfg_.max_acceptable_rmse_mm, 2) +
                " mm. Check the mat is flat, in focus and free of glare.");
            warned = true;
        }
        // Only when there was no geometry to keep, or the bench was set to take
        // one from a working base anyway.
        if (lattice.dark_lines) {
            const std::string said =
                "This geometry was fitted to a grid printed dark on cloth, which is not true enough "
                "to measure by: on this bench the cloth's squares read 1.25% to 2.8% long against "
                "the rigid mat's. Lay the rigid mat and press Recalibrate, then put the cloth back "
                "and press it again - from then on the cloth only gives the background.";
            log("WARNING: " + said);
            detail += " " + said;
            warned = true;
        }
        finishCalibration(true, "Recalibrated", detail, seconds, false, warned);
    }

    // A recalibration on a working base: the exposure, the background and the
    // working area are the base's and are taken now; the geometry in service
    // was measured on a ruler and is kept, its checked scale with it.
    //
    // It is the background the operator is after when the light has changed,
    // the base has been moved or straightened, or the picture has drifted, and
    // none of those moves the camera. What would is looked for in the base's
    // printed pitch (checkBasePitch) and warned about.
    void applyBaseOnly(const PlaneModelPtr& model, const cv::Mat& bg, const ExposureChange& exposure,
                       const LatticeInfo& lattice, double seconds) {
        std::string moved;
        bool newBackground = false;
        try {
            moved = checkBasePitch(*model, lattice);
            saveCalibration(*model, cfg_);
            RoiBox roi;
            bool roiFollowedMat = false;
            if (roiFromModel(*model, bg.size(), cfg_, roi, &bg, &roiFollowedMat)) {
                engine_->setRoi(roi);
                saveRoi(roi, cfg_);
                logRailSource(roiFollowedMat);
            }
            engine_->setBackground(bg);
            newBackground = true;
            finger_.reset();
            clearSeeds();
        } catch (const std::exception& e) {
            const std::string error =
                std::string("the new background could not be put into service: ") + e.what();
            log("Calibration failed: " + error);
            if (newBackground) {
                keepExposure(exposure);
            } else {
                restoreExposure(exposure);
            }
            finishCalibration(false, "Recalibration failed", error, seconds);
            return;
        }
        keepExposure(exposure);
        rebuildDatum();

        const std::string kept = keptGeometryText(*model);
        log("Recalibrated in " + vjson::formatFixed(seconds, 1) +
            " s. The background and the exposure were taken from the base. " + kept);
        std::string detail = "Background and exposure retaken from the base. " + kept;
        // Each of these is something to do, so the card stays until read.
        bool warned = !moved.empty();
        const vjson::Value* detection = model->meta().find("detection");
        if (detection && detection->getBool("dark_lines", false)) warned = true;
        if (!model->meta().has("scale_verified_against_mm")) {
            detail += " The scale has not been checked against a reference yet: lay a steel rule "
                      "and use Check scale.";
            warned = true;
        }
        if (!moved.empty()) detail += " " + moved;
        finishCalibration(true, "Recalibrated", detail, seconds, false, warned);
    }

    // "The measuring geometry of 2026-09-21 13:53 is kept (residual 0.228 mm,
    // scale checked against 300.0 mm).", then why, with a warning when that
    // geometry was itself fitted to a printed cloth.
    static std::string keptGeometryText(const PlaneModel& model) {
        std::string at = model.meta().getString("calibrated_at", "an earlier calibration");
        if (at.size() >= 16 && at[10] == 'T') at = at.substr(0, 10) + " " + at.substr(11, 5);
        std::string out = "The measuring geometry of " + at + " is kept (residual " +
                          vjson::formatFixed(model.metaNumber("rmse_mm", 0.0), 3) + " mm";
        if (model.meta().has("scale_verified_against_mm")) {
            out += ", scale checked against " +
                   vjson::formatFixed(model.metaNumber("scale_verified_against_mm", 0.0), 1) + " mm";
        }
        out += ").";
        const vjson::Value* detection = model.meta().find("detection");
        if (detection && detection->getBool("dark_lines", false)) {
            out += " That geometry was itself fitted to a grid printed on cloth, which is not true "
                   "enough to measure by: lay the rigid mat and press Recalibrate to replace it.";
        } else {
            out += " A grid printed on cloth is not true enough to measure by, so it only gives "
                   "the background.";
        }
        return out;
    }

    // Whether the camera has moved since the geometry being kept last saw this
    // base.
    //
    // A kept geometry is only as good as the camera's position. Raised or
    // lowered, the camera sees every printed square larger or smaller by the
    // same fraction, both ways alike, and every reading afterwards is off by
    // that fraction. A cloth moved or re-stretched on the table changes its
    // squares too, but seldom both ways alike. So the base's pitch is
    // remembered with the geometry (meta "base") and compared at every
    // recalibration. On this bench it repeats to 0.1-0.3% for an unmoved
    // camera, and 0.5% is 3.5 mm of height. Returns the warning, empty when
    // there is none.
    std::string checkBasePitch(PlaneModel& model, const LatticeInfo& lattice) {
        if (!(lattice.pitch_px_x > 0.0 && lattice.pitch_px_y > 0.0)) return std::string();
        std::string warning;
        const vjson::Value* base = model.meta().find("base");
        const vjson::Value* was = base ? base->find("pitch_px") : nullptr;
        if (was && was->isArray() && was->size() >= 2 && was->at(0).asNumber() > 0.0 &&
            was->at(1).asNumber() > 0.0) {
            const double dx = 100.0 * (lattice.pitch_px_x / was->at(0).asNumber() - 1.0);
            const double dy = 100.0 * (lattice.pitch_px_y / was->at(1).asNumber() - 1.0);
            const double limit = cfg_.base_pitch_change_warn_pct;
            if (std::max(std::abs(dx), std::abs(dy)) > limit) {
                const auto pct = [](double v) {
                    return (v >= 0.0 ? "+" : "") + vjson::formatFixed(v, 2) + "%";
                };
                std::string at = base->getString("at", "the last recalibration");
                if (at.size() >= 16 && at[10] == 'T') at = at.substr(0, 10) + " " + at.substr(11, 5);
                const bool alike = dx * dy > 0.0 && std::min(std::abs(dx), std::abs(dy)) > 0.5 * limit;
                const double both = 0.5 * (dx + dy);
                warning = "The base's printed squares look " + pct(dx) + " across and " + pct(dy) +
                          " down against " + at + ". ";
                if (alike) {
                    warning += "Both ways alike is what raising or lowering the camera does: if it "
                               "has moved, readings now come out about " +
                               vjson::formatFixed(std::abs(both), 1) + "% " +
                               (both > 0.0 ? "long" : "short") +
                               ". Check the scale with a steel rule (Check scale), or lay the rigid "
                               "mat and press Recalibrate.";
                } else {
                    warning += "More one way than the other is what moving or re-stretching the "
                               "cloth does, which changes nothing that is measured. If the camera "
                               "has been moved as well, check the scale with a steel rule.";
                }
                log("WARNING: " + warning);
            }
        }
        vjson::Value seen = vjson::Value::object();
        vjson::Value pitch = vjson::Value::array();
        pitch.push(vjson::Value::real(lattice.pitch_px_x));
        pitch.push(vjson::Value::real(lattice.pitch_px_y));
        seen.set("pitch_px", pitch);
        seen.set("at", vjson::Value::str(timestampIso()));
        model.meta().set("base", seen);
        return warning;
    }

    void finishCalibration(bool ok, const std::string& headline, const std::string& detail,
                           double seconds, bool cancelled = false, bool warned = false) {
        // The status the page reads was last published mid-run and still says
        // CALIBRATING, and the next frame to replace it is up to half a second
        // away. Put the engine's own state back now, so the lamp and the
        // result on the page agree from the moment it finishes.
        const std::string state = engine_->state();
        std::lock_guard<std::mutex> guard(lock_);
        status_.set("state", vjson::Value::str(state));
        calib_.active = false;
        calib_.started = false;
        calib_.phase.clear();
        calib_.haveOutcome = true;
        calib_.outcomeSeq = calib_.seq;
        calib_.ok = ok;
        calib_.cancelled = cancelled;
        calib_.warned = warned;
        calib_.headline = headline;
        calib_.detail = detail;
        calib_.seconds = seconds;
    }

    // What the page needs to draw a recalibration: whether one is running, how
    // far it has got, and how the last one ended. Call with lock_ held.
    vjson::Value calibrationJsonLocked() const {
        vjson::Value c = vjson::Value::object();
        c.set("active", vjson::Value::boolean(calib_.active));
        c.set("seq", vjson::Value::integer(calib_.seq));
        if (calib_.active) {
            c.set("phase", vjson::Value::str(calib_.phase));
            c.set("done", vjson::Value::integer(calib_.done));
            c.set("needed", vjson::Value::integer(calib_.needed));
            c.set("hidden", vjson::Value::real(calib_.hidden));
            c.set("elapsed_s", vjson::Value::real(nowSeconds() - calib_.startedAt));
        }
        if (calib_.haveOutcome) {
            vjson::Value last = vjson::Value::object();
            last.set("seq", vjson::Value::integer(calib_.outcomeSeq));
            last.set("ok", vjson::Value::boolean(calib_.ok));
            if (calib_.cancelled) last.set("cancelled", vjson::Value::boolean(true));
            if (calib_.warned) last.set("warn", vjson::Value::boolean(true));
            last.set("headline", vjson::Value::str(calib_.headline));
            last.set("detail", vjson::Value::str(calib_.detail));
            last.set("seconds", vjson::Value::real(calib_.seconds));
            c.set("last", last);
        } else {
            c.set("last", vjson::Value::null());
        }
        return c;
    }

    // -- publishing --

    // Everything drawn over the picture, and what it is drawn from.
    //
    // The feed and a verdict's saved picture (captureVerdict) are drawn by the
    // same code, drawOverlay, from a FeedOverlay: a copy of what the station
    // would draw at that moment, taken on the capture thread by overlayNow.
    // The copy is what lets the saved picture be drawn on another thread, and
    // on the picture the reading was taken from rather than the live one.
    struct FeedOverlay {
        bool calibrating = false;
        bool held = false;
        bool heldForTake = false;
        bool hasRoi = false;
        RoiBox roi;
        std::string scanFrom;
        double outlineSmoothPx = 0.0;
        double outlineSimplifyPx = 0.0;
        // A perimeter step: the band, margins and clips, and the halves read.
        bool onPerimeter = false;
        bool havePerimLines = false;
        PerimeterGuideLines perimLines;
        PerimeterGuide perimGuide;
        std::vector<std::vector<cv::Point2d>> perimPaths;
        std::vector<cv::Point2d> perimCuts, perimCorners;
        // A folded edge's measured span.
        bool pointedStep = false;
        bool haveEdge = false;
        std::vector<cv::Point2d> edgePath;
        // Pointing: a seam, the seams offered, a distance, a collar.
        bool haveCurve = false;
        std::vector<cv::Point2d> curvePath;
        std::vector<std::vector<cv::Point2d>> seamOptions;
        bool haveDistance = false;
        std::vector<cv::Point2d> distancePx;
        double distanceMm = 0.0;
        bool haveCollarBox = false;
        cv::Rect2d collarBox;
        bool haveCollar = false;
        bool collarOffering = false;
        std::vector<std::vector<cv::Point2d>> collarLines;
        std::vector<std::string> collarRoles;
        int collarChosen = -1;
        std::vector<cv::Point2d> collarBack;
        double collarMm = 0.0;
        bool tracing = false;
        std::vector<cv::Point2d> tracePx, traceCurvePx;
        std::vector<cv::Point2d> seedsPx;  // the points put down, on the picture
        bool pointerPresent = false;
        bool pointerRejected = false;
        cv::Point2d pointerPx;
        double pointerDwell = 0.0;
        // The held reading's refined outline, and what it is drawn back out by.
        bool haveRefined = false;
        std::vector<cv::Point2d> refined;
        double parallax = 1.0;
        bool haveModel = false;
        cv::Point2d axis;
        // The live pass's outline, decimated.
        bool haveLive = false;
        std::vector<cv::Point> liveContour;
        int liveStep = 1;
    };

    // What the feed would draw now. On the capture thread; takes lock_ itself.
    FeedOverlay overlayNow(bool calibrating) const {
        MeasurementEngine& e = *engine_;
        const DebugState& dbg = e.lastDebug();
        FeedOverlay ov;
        ov.calibrating = calibrating;
        ov.held = !calibrating && (e.state() == STATE_MEASURED);
        ov.hasRoi = e.hasRoi();
        if (ov.hasRoi) ov.roi = e.roi();
        ov.scanFrom = cfg_.scan_from;
        ov.outlineSmoothPx = cfg_.display_outline_smooth_px;
        ov.outlineSimplifyPx = cfg_.display_outline_simplify_px;
        // Through the model the points were put down through. Takes lock_.
        const PlaneModelPtr pointed = pointedModel();
        {
            std::lock_guard<std::mutex> guard(lock_);
            ov.heldForTake = heldForTake_;
            ov.onPerimeter = perimeterStepLocked();
            if (ov.onPerimeter) {
                ov.havePerimLines = havePerimLines_;
                ov.perimLines = perimLines_;
                ov.perimGuide = perimGuide_;
                ov.perimPaths = perimPathsPx_;
                ov.perimCuts = perimCutsPx_;
                ov.perimCorners = perimCornersPx_;
            }
            ov.pointedStep = pointedStepLocked();
            ov.haveEdge = haveEdge_;
            if (haveEdge_) ov.edgePath = edgePath_;
            ov.haveCurve = haveCurve_;
            ov.curvePath = curvePath_;
            ov.seamOptions = seamOptionsPx_;
            ov.haveDistance = haveDistance_;
            ov.distancePx = distancePx_;
            ov.distanceMm = distanceMm_;
            ov.haveCollarBox = haveCollarBox_;
            ov.collarBox = collarBoxPx_;
            ov.haveCollar = haveCollar_;
            ov.collarOffering = collarState_ == "choose" && showManual_;
            if (haveCollar_) {
                ov.collarLines = collarLinesPx_;
                ov.collarRoles = collarRoles_;
                ov.collarChosen = collarChosen_;
                ov.collarBack = collarBackPx_;
                ov.collarMm = collarMm_;
            }
            ov.tracing = tracing_;
            if (tracing_) {
                ov.tracePx = tracePx_;
                ov.traceCurvePx = traceCurvePx_;
            }
            if (ov.held && !seeds_.empty() && pointed) ov.seedsPx = pointed->mmToPx(seeds_);
            ov.pointerPresent = pointerPresent_;
            ov.pointerRejected = pointerRejected_;
            ov.pointerPx = pointerPx_;
            ov.pointerDwell = pointerDwell_;
        }
        // Only the outline that will be drawn is copied: the refined one of a
        // reading held for this take, and the live one otherwise.
        const std::optional<MeasureResult>& last = e.lastResult();
        if (ov.held && ov.heldForTake && last.has_value() && last->outline_px.size() >= 3) {
            ov.haveRefined = true;
            ov.refined = last->outline_px;
            ov.parallax = last->parallax_correction;
            const PlaneModelPtr m = e.model();
            ov.haveModel = m != nullptr;
            if (m) ov.axis = m->centre();
        }
        if (dbg.has_contour && !ov.haveRefined) {
            ov.haveLive = true;
            ov.liveContour = dbg.contour;
            ov.liveStep = dbg.step;
        }
        return ov;
    }

    // Draws `ov` onto `disp`, a picture at `scale` of the camera's, and notes
    // in `marks` what a step's reading put there. Static, and reads nothing
    // but its arguments, so a saved picture can be drawn off the capture
    // thread by exactly the code that draws the feed.
    static void drawOverlay(cv::Mat& disp, double scale, const FeedOverlay& ov, Marks& marks) {
        const bool calibrating = ov.calibrating;
        const bool held = ov.held;
        // Held, but for a step the operator has since left or retaken. The
        // engine keeps a reading until the mat is cleared or the next lay is
        // laid, and knows nothing of steps, so this is the outline of the lay
        // the last step was measured on - usually being refolded - and it is
        // not drawn. Once the garment is laid again the engine lets go of it,
        // and the new lay is outlined live as it settles. Pointing still works
        // over a lay left as it was: the reading is held, only its outline is
        // not shown.
        const bool heldForTake = ov.heldForTake;
        const bool staleHold = held && !heldForTake;
        // A perimeter step has no rail: it has the band the clips go in, the
        // margins, and the halves read.
        const bool onPerimeter = ov.onPerimeter;
        const bool havePerimLines = ov.havePerimLines;
        const PerimeterGuideLines& perimLines = ov.perimLines;
        const PerimeterGuide& perimGuide = ov.perimGuide;
        const std::vector<std::vector<cv::Point2d>>& perimPaths = ov.perimPaths;
        const std::vector<cv::Point2d>& perimCuts = ov.perimCuts;
        const std::vector<cv::Point2d>& perimCorners = ov.perimCorners;
        const Strokes stroke;
        // Amber while the part is still settling, bone white once the reading is
        // held. The outline and the number change together, so the operator can
        // tell from either one whether the value on screen is final.
        const cv::Scalar colour = held ? cv::Scalar(233, 237, 232) : cv::Scalar(61, 163, 232);
        const auto shown = [scale](const std::vector<cv::Point2d>& fullPx) {
            std::vector<cv::Point2d> out;
            out.reserve(fullPx.size());
            for (const cv::Point2d& p : fullPx) out.emplace_back(p.x * scale, p.y * scale);
            return out;
        };

        if (ov.hasRoi && !calibrating) {
            const RoiBox& roi = ov.roi;
            cv::rectangle(disp, atSub(roi.x1 * scale, roi.y1 * scale),
                          atSub(roi.x2 * scale, roi.y2 * scale), cv::Scalar(67, 61, 31),
                          stroke.roi, cv::LINE_AA, kSubBits);
        }

        // The mark: which side of the picture the fold has to face. This is
        // the only thing on the feed that tells the operator how to orient the
        // cloth, so it is the brightest thing on it.
        if (ov.hasRoi && !calibrating && !onPerimeter) {
            const RoiBox& r = ov.roi;
            const int x1 = static_cast<int>(r.x1 * scale), y1 = static_cast<int>(r.y1 * scale);
            const int x2 = static_cast<int>(r.x2 * scale), y2 = static_cast<int>(r.y2 * scale);
            const cv::Scalar mark(61, 163, 232);
            const cv::Scalar shadow(20, 20, 20);

            const bool vertical = (ov.scanFrom == "left" || ov.scanFrom == "right");
            const bool atStart = (ov.scanFrom == "left" || ov.scanFrom == "top");
            cv::Point a1, b1, inward;
            if (vertical) {
                const int x = atStart ? x1 : x2;
                a1 = cv::Point(x, y1);
                b1 = cv::Point(x, y2);
                inward = cv::Point(atStart ? 1 : -1, 0);
            } else {
                const int y = atStart ? y1 : y2;
                a1 = cv::Point(x1, y);
                b1 = cv::Point(x2, y);
                inward = cv::Point(0, atStart ? 1 : -1);
            }

            // The working area now runs out to the border of the picture, so
            // the mark can land on the last row of the display with half its
            // stroke off the screen, and a mark the operator cannot see is not
            // a mark. Hold it far enough in to be drawn whole. This moves the
            // line on the screen and nothing else: the rail is still where the
            // working area says it is, and that is what is measured from.
            const int inset = stroke.railHalo / 2 + 1;
            const auto holdIn = [&](cv::Point p) {
                p.x = std::max(inset, std::min(p.x, disp.cols - 1 - inset));
                p.y = std::max(inset, std::min(p.y, disp.rows - 1 - inset));
                return p;
            };
            a1 = holdIn(a1);
            b1 = holdIn(b1);

            cv::line(disp, a1, b1, shadow, stroke.railHalo, cv::LINE_AA);
            cv::line(disp, a1, b1, mark, stroke.rail, cv::LINE_AA);

            // The words are placed first, so the arrows can be laid out around
            // them instead of through them.
            const char* text = "FOLD FACES THIS SIDE";
            int baseline = 0;
            const cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            cv::Point label((a1.x + b1.x) / 2 + inward.x * 40 - ts.width / 2,
                            (a1.y + b1.y) / 2 + inward.y * 40 + ts.height / 2);
            label.x = std::max(4, std::min(label.x, disp.cols - ts.width - 4));
            label.y = std::max(ts.height + 6, std::min(label.y, disp.rows - 8));
            const cv::Rect words(label.x - 12, label.y - ts.height - 10, ts.width + 24,
                                 ts.height + baseline + 20);

            // Arrows along the mark, pointing the way the machine looks, so the
            // side is unmistakable rather than merely coloured. Spaced by the
            // length of the rail rather than by a fixed count, so they sit the
            // same distance apart whether the rail is a hand's width or the
            // whole picture, and stood off the words rather than run through
            // them.
            const double railLen = std::hypot(static_cast<double>(b1.x - a1.x),
                                              static_cast<double>(b1.y - a1.y));
            const int steps = std::max(3, std::min(10, static_cast<int>(railLen / 150.0)));
            for (int k = 1; k < steps; ++k) {
                const double t = static_cast<double>(k) / steps;
                const cv::Point base(static_cast<int>(a1.x + (b1.x - a1.x) * t),
                                     static_cast<int>(a1.y + (b1.y - a1.y) * t));
                const cv::Point tip(base.x + inward.x * 26, base.y + inward.y * 26);
                if (words.contains(base) || words.contains(tip)) continue;
                cv::arrowedLine(disp, base, tip, shadow, stroke.arrowHalo, cv::LINE_AA, 0, 0.4);
                cv::arrowedLine(disp, base, tip, mark, stroke.arrow, cv::LINE_AA, 0, 0.4);
            }

            cv::putText(disp, text, label, cv::FONT_HERSHEY_SIMPLEX, 0.6, shadow, 3, cv::LINE_AA);
            cv::putText(disp, text, label, cv::FONT_HERSHEY_SIMPLEX, 0.6, mark, 1, cv::LINE_AA);
        }

        // The perimeter's guide. The band the clips go in, along the edge the
        // other half runs out past; the margin to keep clear on the other three
        // sides, dashed, and red along any stretch the cloth is inside; a ring
        // on each clip found, green in the band and amber out of it. Then the
        // half just read, the way an edge is drawn once it is measured.
        if (onPerimeter && !calibrating && havePerimLines && perimLines.band_near.size() >= 2 &&
            perimLines.band_far.size() >= 2) {
            const cv::Scalar found(90, 185, 138);
            const cv::Scalar live(61, 163, 232);
            const cv::Scalar shadow(20, 20, 20);
            const cv::Scalar ink(233, 237, 232);
            const cv::Scalar margin(170, 170, 170);
            const cv::Scalar red(60, 60, 235);
            const auto toDisp = [scale](const std::vector<cv::Point2d>& in) {
                std::vector<cv::Point> out;
                out.reserve(in.size());
                for (const cv::Point2d& p : in) out.push_back(atSub(p.x * scale, p.y * scale));
                return out;
            };

            // The band, tinted, and its two edges.
            {
                cv::Mat tint = disp.clone();
                std::vector<cv::Point> band = toDisp(perimLines.band_near);
                const std::vector<cv::Point> farEdge = toDisp(perimLines.band_far);
                band.insert(band.end(), farEdge.rbegin(), farEdge.rend());
                cv::fillPoly(tint, std::vector<std::vector<cv::Point>>{band}, found, cv::LINE_AA, kSubBits);
                cv::addWeighted(tint, 0.22, disp, 0.78, 0.0, disp);
                cv::polylines(disp, toDisp(perimLines.band_near), false, found, 1, cv::LINE_AA, kSubBits);
                cv::polylines(disp, farEdge, false, found, 1, cv::LINE_AA, kSubBits);

                const char* text = "CLIPS IN THIS BAND";
                int baseline = 0;
                const cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
                const cv::Point2d nearMid = perimLines.band_near[perimLines.band_near.size() / 2] * scale;
                const cv::Point2d farMid = perimLines.band_far[perimLines.band_far.size() / 2] * scale;
                cv::Point2d inward = farMid - nearMid;
                const double len = std::hypot(inward.x, inward.y);
                if (len > 1e-6) inward *= 1.0 / len;
                const cv::Point2d at = farMid + inward * (8.0 + ts.height);
                cv::Point label(static_cast<int>(at.x) - ts.width / 2, static_cast<int>(at.y) + ts.height / 2);
                label.x = std::max(4, std::min(label.x, disp.cols - ts.width - 4));
                label.y = std::max(ts.height + 6, std::min(label.y, disp.rows - 8));
                cv::putText(disp, text, label, cv::FONT_HERSHEY_SIMPLEX, 0.55, shadow, 3, cv::LINE_AA);
                cv::putText(disp, text, label, cv::FONT_HERSHEY_SIMPLEX, 0.55, found, 1, cv::LINE_AA);
            }

            // The margins, dashed.
            const auto dashed = [&](const std::vector<cv::Point2d>& fullPx, const cv::Scalar& c) {
                std::vector<cv::Point2d> pts;
                pts.reserve(fullPx.size());
                for (const cv::Point2d& p : fullPx) pts.emplace_back(p.x * scale, p.y * scale);
                const double dash = 8.0, gap = 6.0;
                double pos = 0.0;
                for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                    const cv::Point2d a = pts[i], b = pts[i + 1];
                    const double L = std::hypot(b.x - a.x, b.y - a.y);
                    double t = 0.0;
                    while (t < L - 1e-9) {
                        const double cycle = std::fmod(pos, dash + gap);
                        const bool drawn = cycle < dash;
                        const double left = drawn ? dash - cycle : dash + gap - cycle;
                        const double stepLen = std::min(left, L - t);
                        if (drawn) {
                            const cv::Point2d p0 = a + (b - a) * (t / L);
                            const cv::Point2d p1 = a + (b - a) * ((t + stepLen) / L);
                            cv::line(disp, atSub(p0.x, p0.y), atSub(p1.x, p1.y), c, 1, cv::LINE_AA, kSubBits);
                        }
                        t += stepLen;
                        pos += stepLen;
                    }
                }
            };
            for (int s = 0; s < 4; ++s) {
                if (perimLines.margin[s].size() >= 2) dashed(perimLines.margin[s], margin);
            }
            for (int s = 0; s < 4; ++s) {
                if (!perimGuide.margin[s] || perimGuide.touch[s].size() != 2) continue;
                const cv::Point a = atSub(perimGuide.touch[s][0].x * scale, perimGuide.touch[s][0].y * scale);
                const cv::Point b = atSub(perimGuide.touch[s][1].x * scale, perimGuide.touch[s][1].y * scale);
                cv::line(disp, a, b, shadow, 6, cv::LINE_AA, kSubBits);
                cv::line(disp, a, b, red, 3, cv::LINE_AA, kSubBits);
            }

            // The clips.
            for (std::size_t k = 0; k < perimGuide.clip_list.size(); ++k) {
                const cv::Point2d& q = perimGuide.clip_list[k].px;
                const cv::Point at = atSub(q.x * scale, q.y * scale);
                const bool inBand = k < perimGuide.in_band.size() && perimGuide.in_band[k];
                cv::circle(disp, at, 14 << kSubBits, shadow, 4, cv::LINE_AA, kSubBits);
                cv::circle(disp, at, 14 << kSubBits, inBand ? found : live, 2, cv::LINE_AA, kSubBits);
            }

            // The half read on this take, as measured: corners crossed and all.
            if (held && heldForTake) {
                for (const std::vector<cv::Point2d>& path : perimPaths) {
                    if (path.size() < 2) continue;
                    const std::vector<cv::Point> line = toDisp(path);
                    cv::polylines(disp, line, false, shadow, stroke.spanHalo, cv::LINE_AA, kSubBits);
                    cv::polylines(disp, line, false, found, stroke.span, cv::LINE_AA, kSubBits);
                    marks.line(shown(path), found, stroke.span, stroke.spanHalo);
                }
                for (const cv::Point2d& q : perimCorners) {
                    cv::circle(disp, atSub(q.x * scale, q.y * scale), stroke.corner << kSubBits, found, -1,
                               cv::LINE_AA, kSubBits);
                }
                for (const cv::Point2d& q : perimCuts) {
                    const cv::Point at = atSub(q.x * scale, q.y * scale);
                    cv::circle(disp, at, (stroke.seed + 1) << kSubBits, shadow, -1, cv::LINE_AA, kSubBits);
                    cv::circle(disp, at, stroke.seed << kSubBits, ink, -1, cv::LINE_AA, kSubBits);
                }
            }
        }

        // The edge that was actually measured, drawn over the held reading, so
        // the operator can see what the number came from rather than trusting
        // that it found the right thing.
        {
            // Never on a curve step. Nothing on one is measured from a folded
            // edge, so the green span is either stale from the step before it
            // or an answer to a question nobody asked - and it is drawn across
            // the garment in the brightest colour on the feed, which makes it
            // look like the reading.
            if (held && !ov.pointedStep && ov.haveEdge && ov.edgePath.size() >= 2) {
                std::vector<cv::Point> trace;
                trace.reserve(ov.edgePath.size());
                for (const cv::Point2d& p : ov.edgePath) {
                    trace.push_back(atSub(p.x * scale, p.y * scale));
                }

                // The edge the machine followed, thin and dim. This is not the
                // measurement. It is how the two ends were found, and an
                // operator checking that the machine read the right piece of
                // cloth has to be able to see it lying on that cloth.
                cv::polylines(disp, trace, false, cv::Scalar(58, 112, 88), stroke.outline,
                              cv::LINE_AA, kSubBits);

                // The measurement: the straight line between those two ends.
                //
                // A hem sweeps and a shoulder slopes, and following that sweep
                // is not what anybody on a floor does. They find the seam at
                // one end, find the seam at the other, and pull a tape straight
                // between them. The curve is how the ends are found; the line
                // between them is the measurement, and it is the line drawn
                // bright, because it is the one the number came from.
                const cv::Point p1 = trace.front();
                const cv::Point p2 = trace.back();
                cv::line(disp, p1, p2, cv::Scalar(20, 20, 20), stroke.spanHalo, cv::LINE_AA,
                         kSubBits);
                cv::line(disp, p1, p2, cv::Scalar(90, 185, 138), stroke.span, cv::LINE_AA,
                         kSubBits);
                // The points are in sixteenths, so the radius has to be too.
                const int dot = stroke.corner << kSubBits;
                cv::circle(disp, p1, dot, cv::Scalar(90, 185, 138), -1, cv::LINE_AA, kSubBits);
                cv::circle(disp, p2, dot, cv::Scalar(90, 185, 138), -1, cv::LINE_AA, kSubBits);

                marks.line(shown(ov.edgePath), cv::Scalar(58, 112, 88), stroke.outline);
                marks.line(shown({ov.edgePath.front(), ov.edgePath.back()}), cv::Scalar(90, 185, 138),
                           stroke.span, stroke.spanHalo, stroke.corner);
            }
        }

        // Pointing at a seam: what has been put down, what was found from it,
        // and where the finger is now.
        //
        // Drawn in the same order it happens, and drawn at all because this is
        // the one measurement on the bench that the operator is inside. On a
        // folded edge they can check the machine found the right line. Here
        // they also have to be able to check that the line the machine found is
        // the one they meant, which they cannot do from a number.
        {
            const cv::Scalar ink(233, 237, 232);
            const cv::Scalar shadow(20, 20, 20);
            const cv::Scalar live(61, 163, 232);
            const cv::Scalar found(90, 185, 138);

            if (held && ov.haveCurve && ov.curvePath.size() >= 2) {
                std::vector<cv::Point> trace;
                trace.reserve(ov.curvePath.size());
                for (const cv::Point2d& p : ov.curvePath) {
                    trace.push_back(atSub(p.x * scale, p.y * scale));
                }
                // The seam itself is the measurement here, not a chord across
                // it, so it is the seam that is drawn bright. Which of its two
                // lengths the step reports is written on the panel; the shape
                // on the feed is the same either way.
                cv::polylines(disp, trace, false, shadow, stroke.spanHalo, cv::LINE_AA, kSubBits);
                cv::polylines(disp, trace, false, found, stroke.span, cv::LINE_AA, kSubBits);
                const int dot = stroke.corner << kSubBits;
                cv::circle(disp, trace.front(), dot, found, -1, cv::LINE_AA, kSubBits);
                cv::circle(disp, trace.back(), dot, found, -1, cv::LINE_AA, kSubBits);
                marks.line(shown(ov.curvePath), found, stroke.span, stroke.spanHalo, stroke.corner);
            }

            // The seams the machine found and could not choose between, thin,
            // so the operator can see what there is to point at.
            if (held && !ov.haveCurve && !ov.seamOptions.empty()) {
                for (const std::vector<cv::Point2d>& option : ov.seamOptions) {
                    if (option.size() < 2) continue;
                    std::vector<cv::Point> line;
                    line.reserve(option.size());
                    for (const cv::Point2d& p : option) {
                        line.push_back(atSub(p.x * scale, p.y * scale));
                    }
                    cv::polylines(disp, line, false, shadow, stroke.spanHalo, cv::LINE_AA,
                                  kSubBits);
                    cv::polylines(disp, line, false, live, 1, cv::LINE_AA, kSubBits);
                }
            }

            // A distance, as the two points and the line between them with the
            // number on it. Drawn in the colour of a found reading because it
            // is one: this is the measurement, not a hint of where one might be.
            if (held && ov.haveDistance && ov.distancePx.size() >= 2) {
                // Two points give a straight line; a drag gives the smoothed
                // path, which is what was measured. Either way the ends are
                // marked, because the ends are the operator's and the number
                // depends on nothing else.
                std::vector<cv::Point> line;
                line.reserve(ov.distancePx.size());
                for (const cv::Point2d& p : ov.distancePx) {
                    line.push_back(atSub(p.x * scale, p.y * scale));
                }
                cv::polylines(disp, line, false, shadow, stroke.spanHalo, cv::LINE_AA, kSubBits);
                cv::polylines(disp, line, false, found, stroke.span, cv::LINE_AA, kSubBits);
                for (const cv::Point2d& p : {ov.distancePx.front(), ov.distancePx.back()}) {
                    const cv::Point at = atSub(p.x * scale, p.y * scale);
                    cv::circle(disp, at, (stroke.seed + 1) << kSubBits, shadow, -1, cv::LINE_AA,
                               kSubBits);
                    cv::circle(disp, at, stroke.seed << kSubBits, found, -1, cv::LINE_AA,
                               kSubBits);
                }
                const std::string text = vjson::formatFixed(ov.distanceMm, 1) + " mm";
                const cv::Point2d& m = ov.distancePx[ov.distancePx.size() / 2];
                const cv::Point mid(static_cast<int>(std::lround(m.x * scale)),
                                    static_cast<int>(std::lround(m.y * scale)) - 12);
                cv::putText(disp, text, mid, cv::FONT_HERSHEY_SIMPLEX, 0.7, shadow, 4, cv::LINE_AA);
                cv::putText(disp, text, mid, cv::FONT_HERSHEY_SIMPLEX, 0.7, found, 1, cv::LINE_AA);
                marks.line(shown(ov.distancePx), found, stroke.span, stroke.spanHalo, stroke.seed,
                           stroke.seed + 1);
                marks.label(text, cv::Point2d(mid.x, mid.y), false, found);
            }

            // A collar read from a box: the box, thin, so the operator can see
            // what they drew; every line found in it, thin; and the edge that
            // was measured, bright, with its two shoulder points and its
            // number, because that is the reading. When no band could be found
            // and lines are offered for a tap instead, they are drawn in the
            // colour of something to point at.
            if (held && ov.haveCollarBox) {
                const cv::Rect2d& b = ov.collarBox;
                const cv::Point tl = atSub(b.x * scale, b.y * scale);
                const cv::Point br = atSub((b.x + b.width) * scale, (b.y + b.height) * scale);
                cv::rectangle(disp, tl, br, shadow, 3, cv::LINE_AA, kSubBits);
                cv::rectangle(disp, tl, br, live, 1, cv::LINE_AA, kSubBits);
                marks.line(shown({{b.x, b.y}, {b.x + b.width, b.y},
                                  {b.x + b.width, b.y + b.height}, {b.x, b.y + b.height}}),
                           live, 1, 3, 0, 0, true);
            }
            if (held && ov.haveCollar) {
                // Offered for a tap only with the hand-drawn collar steps on;
                // otherwise the lines are only what was found, and a tap takes
                // none of them.
                const bool offering = ov.collarOffering;
                for (std::size_t k = 0; k < ov.collarLines.size(); ++k) {
                    if (static_cast<int>(k) == ov.collarChosen || ov.collarLines[k].size() < 2) continue;
                    // The band's inner edge, the opening, is shown and not
                    // measured: dotted, so that it cannot be taken for the
                    // line that was.
                    if (!offering && k < ov.collarRoles.size() && ov.collarRoles[k] == "inner") {
                        drawDotted(disp, shown(ov.collarLines[k]), 6.0, 1.2, ink, shadow);
                        marks.line(shown(ov.collarLines[k]), ink, 2, 4, 0, 0, false, 6.0);
                        continue;
                    }
                    std::vector<cv::Point> line;
                    line.reserve(ov.collarLines[k].size());
                    for (const cv::Point2d& p : ov.collarLines[k]) line.push_back(atSub(p.x * scale, p.y * scale));
                    cv::polylines(disp, line, false, shadow, 3, cv::LINE_AA, kSubBits);
                    cv::polylines(disp, line, false, offering ? live : ink, 1, cv::LINE_AA, kSubBits);
                    marks.line(shown(ov.collarLines[k]), offering ? live : ink, 1, 3);
                }
                if (ov.collarChosen >= 0 && ov.collarChosen < static_cast<int>(ov.collarLines.size()) &&
                    ov.collarLines[static_cast<std::size_t>(ov.collarChosen)].size() >= 2) {
                    const std::vector<cv::Point2d>& seam = ov.collarLines[static_cast<std::size_t>(ov.collarChosen)];
                    std::vector<cv::Point> line;
                    line.reserve(seam.size());
                    for (const cv::Point2d& p : seam) line.push_back(atSub(p.x * scale, p.y * scale));
                    // Measured all the way round: the back of the neck as
                    // well, in the same ink, closing the front into the whole
                    // collar. Both backings first, so neither line is cut by
                    // the other's, and the shoulder points over both.
                    std::vector<cv::Point> back;
                    back.reserve(ov.collarBack.size());
                    for (const cv::Point2d& p : ov.collarBack) back.push_back(atSub(p.x * scale, p.y * scale));
                    cv::polylines(disp, line, false, shadow, stroke.spanHalo, cv::LINE_AA, kSubBits);
                    if (back.size() >= 2) cv::polylines(disp, back, false, shadow, stroke.spanHalo, cv::LINE_AA, kSubBits);
                    cv::polylines(disp, line, false, found, stroke.span, cv::LINE_AA, kSubBits);
                    if (back.size() >= 2) cv::polylines(disp, back, false, found, stroke.span, cv::LINE_AA, kSubBits);
                    if (back.size() >= 2) marks.line(shown(ov.collarBack), found, stroke.span, stroke.spanHalo);
                    for (const cv::Point& at : {line.front(), line.back()}) {
                        cv::circle(disp, at, (stroke.seed + 1) << kSubBits, shadow, -1, cv::LINE_AA, kSubBits);
                        cv::circle(disp, at, stroke.seed << kSubBits, found, -1, cv::LINE_AA, kSubBits);
                    }
                    // The number in the middle of the neck opening: the middle
                    // of a horseshoe's points is inside it whichever way round
                    // the shirt lies, clear of the seam and of the box.
                    const std::string text = vjson::formatFixed(ov.collarMm, 1) + " mm";
                    cv::Point2d m(0.0, 0.0);
                    for (const cv::Point2d& p : seam) m += p;
                    m *= 1.0 / static_cast<double>(seam.size());
                    int baseline = 0;
                    const cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 1, &baseline);
                    const cv::Point mid(static_cast<int>(std::lround(m.x * scale)) - ts.width / 2,
                                        static_cast<int>(std::lround(m.y * scale)) + ts.height / 2);
                    cv::putText(disp, text, mid, cv::FONT_HERSHEY_SIMPLEX, 0.7, shadow, 4, cv::LINE_AA);
                    cv::putText(disp, text, mid, cv::FONT_HERSHEY_SIMPLEX, 0.7, found, 1, cv::LINE_AA);
                    marks.line(shown(seam), found, stroke.span, stroke.spanHalo, stroke.seed,
                               stroke.seed + 1);
                    marks.label(text, m * scale, true, found);
                }
            }

            // The trace as it is being drawn, so the operator can see the line
            // they are describing rather than trust that a finger was followed.
            if (held && ov.tracing && !ov.tracePx.empty()) {
                // The fitted curve where there is one yet, the raw line where
                // there is not - and the points themselves on top of either,
                // because the operator is entitled to see exactly where the
                // machine put their fingertip, frame by frame, and to watch
                // each one land as they move. A smooth line alone would hide
                // a point that landed wrong; the points alone would look like
                // the saw-tooth they are at two frames a second.
                const std::vector<cv::Point2d>& line =
                    ov.traceCurvePx.size() >= 2 ? ov.traceCurvePx : ov.tracePx;
                if (line.size() >= 2) {
                    std::vector<cv::Point> drawn;
                    drawn.reserve(line.size());
                    for (const cv::Point2d& p : line) {
                        drawn.push_back(atSub(p.x * scale, p.y * scale));
                    }
                    cv::polylines(disp, drawn, false, shadow, stroke.spanHalo, cv::LINE_AA,
                                  kSubBits);
                    cv::polylines(disp, drawn, false, live, stroke.span, cv::LINE_AA, kSubBits);
                }
                for (const cv::Point2d& p : ov.tracePx) {
                    const cv::Point at = atSub(p.x * scale, p.y * scale);
                    cv::circle(disp, at, (stroke.corner + 1) << kSubBits, shadow, -1, cv::LINE_AA,
                               kSubBits);
                    cv::circle(disp, at, stroke.corner << kSubBits, live, -1, cv::LINE_AA,
                               kSubBits);
                }
            }

            // The taps, drawn where they actually landed rather than where the
            // seam turned out to be. An operator who can see both can see the
            // machine correcting them, which is the whole arrangement working
            // in front of them.
            if (held && !ov.seedsPx.empty()) {
                for (const cv::Point2d& q : ov.seedsPx) {
                    const cv::Point at = atSub(q.x * scale, q.y * scale);
                    cv::circle(disp, at, stroke.seed << kSubBits, shadow, 2, cv::LINE_AA,
                               kSubBits);
                    cv::circle(disp, at, stroke.seed << kSubBits, ink, -1, cv::LINE_AA,
                               kSubBits);
                }
            }

            // The finger, and how much of the dwell it has held. A point that
            // lands with no warning feels like a machine deciding things on its
            // own; a ring that closes is the operator deciding.
            if (ov.pointerPresent) {
                const cv::Point at = atSub(ov.pointerPx.x * scale, ov.pointerPx.y * scale);
                const int r = stroke.pointer << kSubBits;
                cv::circle(disp, at, r, shadow, 3, cv::LINE_AA, kSubBits);
                // Dark only, no live ring, on a frame whose tip was not
                // believed: the machine saw a hand somewhere it could not have
                // got to and did not record it. Seen once, it explains itself.
                if (!ov.pointerRejected) cv::circle(disp, at, r, live, 1, cv::LINE_AA, kSubBits);
                if (ov.pointerDwell > 0.02) {
                    cv::ellipse(disp, at, cv::Size(r, r), -90.0, 0.0, 360.0 * ov.pointerDwell,
                                live, 3, cv::LINE_AA, kSubBits);
                }
            }
        }

        // The outline of the cloth.
        //
        // Which outline depends on what the machine currently knows. Once a
        // reading is held there is a refined one: the boundary the measurement
        // was actually taken from, found to a fraction of a full-resolution
        // pixel along each point's own normal. That is the one to draw, because
        // an operator checking whether the machine found the right edge is
        // entitled to see the edge it found, not an approximation of it.
        //
        // Until then there is only the live pass, which runs decimated by
        // cfg.detect_step so that it can keep up with the camera. Its outline is
        // quantised to that grid and no redrawing can undo it; what can be
        // undone is the half-cell offset. A decimated pixel covers a block of
        // full-resolution ones, so it stands for the centre of that block, not
        // its top-left corner. Multiplying the index by the step alone puts the
        // whole live outline up and to the left of the cloth by half a cell,
        // which at a step of six is two and a half pixels of the real frame,
        // every frame, in the same direction.
        const auto strokeOutline = [&](const std::vector<cv::Point2d>& boundary, bool refined) {
            // Scaled to the screen first, then tidied, so the straightening
            // tolerance is in the pixels the operator is actually looking at
            // and not in sensor pixels that mean something different on every
            // lens. A copy throughout: `boundary` is the measurement's own and
            // is not touched.
            std::vector<cv::Point2d> shown;
            shown.reserve(boundary.size());
            for (const cv::Point2d& p : boundary) {
                shown.emplace_back(p.x * scale, p.y * scale);
            }
            // The refined outline, evened out and tidied over a wider stretch
            // so it reads as smoothly as the live one (kHeldOutlineSmoothPx).
            if (refined) shown = resampleEvenly(shown, 1.0);
            const double smoothPx = refined ? std::max(ov.outlineSmoothPx, kHeldOutlineSmoothPx)
                                            : ov.outlineSmoothPx;
            const std::vector<cv::Point2d> tidy = tidyForDisplay(
                shown, smoothPx, ov.outlineSimplifyPx);

            std::vector<cv::Point> pts;
            pts.reserve(tidy.size());
            for (const cv::Point2d& p : tidy) pts.push_back(atSub(p.x, p.y));
            cv::polylines(disp, pts, true, colour, stroke.outline, cv::LINE_AA, kSubBits);
            return tidy;
        };

        bool drewRefined = false;
        if (ov.haveRefined) {
            // Where the edge is in the picture, which is not quite where the
            // measurement holds it. The engine takes the fold's height off the
            // outline itself - a magnification about the lens axis - so that
            // every step downstream measures the cloth's top face at its true
            // size. Drawn as it is, that outline sat inside the cloth by the
            // correction: nothing at the middle of the picture, and on a half
            // chest at 3.2 mm about 11 px of the frame at its corners, three
            // pixels of the screen, all of them on the side toward the middle.
            // So the drawing puts the magnification back. The reading keeps
            // it; this copy is only ever drawn.
            const double c = ov.parallax;
            if (ov.haveModel && c > 0.0 && c != 1.0) {
                const cv::Point2d axis = ov.axis;
                std::vector<cv::Point2d> seen;
                seen.reserve(ov.refined.size());
                for (const cv::Point2d& p : ov.refined) seen.push_back(axis + (p - axis) * (1.0 / c));
                marks.line(strokeOutline(seen, true), colour, stroke.outline, 0, 0, 0, true);
            } else {
                marks.line(strokeOutline(ov.refined, true), colour, stroke.outline, 0, 0, 0, true);
            }
            drewRefined = true;
        }

        // Nor the live outline in its place. While the engine holds, that is
        // drawn in the colour of a held reading, and on a step left behind
        // there is no reading for this step to be the colour of.
        if (!drewRefined && !staleHold && ov.haveLive && !calibrating) {
            const double offX = ov.hasRoi ? ov.roi.x1 : 0;
            const double offY = ov.hasRoi ? ov.roi.y1 : 0;
            const double halfCell = 0.5 * (ov.liveStep - 1);
            std::vector<cv::Point2d> boundary(ov.liveContour.size());
            for (std::size_t k = 0; k < ov.liveContour.size(); ++k) {
                boundary[k] = cv::Point2d(ov.liveContour[k].x * ov.liveStep + halfCell + offX,
                                          ov.liveContour[k].y * ov.liveStep + halfCell + offY);
            }
            strokeOutline(boundary, false);
        }
    }

    void publish(const cv::Mat& frame, double age) {
        MeasurementEngine& e = *engine_;
        const DebugState& dbg = e.lastDebug();
        const double scale = cfg_.display_scale;

        cv::Mat disp;
        cv::resize(frame, disp, cv::Size(), scale, scale, cv::INTER_AREA);

        // While the grid is being read nothing is being measured, so nothing
        // the engine last said is drawn - it would be an outline of a garment
        // that has just been taken away, or a working area about to be
        // replaced. The operator sees the bare mat, which is what the
        // calibration is looking at, and can see anything left on it.
        const bool calibrating = calibrationRunning();
        const bool held = !calibrating && (e.state() == STATE_MEASURED);
        const std::optional<MeasureResult>& last = e.lastResult();
        // What this step's reading draws below, for the page to keep. See Marks.
        Marks marks(disp.size());
        drawOverlay(disp, scale, overlayNow(calibrating), marks);

        std::vector<unsigned char> jpeg;
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpegQuality_};
        const bool ok = cv::imencode(".jpg", disp, jpeg, params);

        const std::optional<LiveDims>& live = dbg.live;
        PlaneModelPtr model = e.model();

        // How far through settling we are, so the bar on screen means something.
        // Nothing is settling while the grid is read; the page shows that
        // wait separately.
        double progress = 0.0;
        if (calibrating) {
            progress = 0.0;
        } else if (e.state() == STATE_STABILISING) {
            progress =
                static_cast<double>(e.recentCount()) / std::max(cfg_.stable_frames_required, 1);
        } else if (e.state() == STATE_CAPTURING) {
            progress = static_cast<double>(e.accumulatedFrames()) /
                       std::max(cfg_.measurement_average_frames, 1);
        } else if (held) {
            progress = 1.0;
        }

        vjson::Value status = vjson::Value::object();
        status.set("ready", vjson::Value::boolean(true));
        status.set("state", vjson::Value::str(calibrating ? std::string("CALIBRATING") : e.state()));
        status.set("held", vjson::Value::boolean(held));
        status.set("progress", vjson::Value::real(std::min(1.0, progress)));
        status.set("count", vjson::Value::integer(e.count()));

        if (live.has_value()) {
            vjson::Value liveJson = vjson::Value::object();
            liveJson.set("length_mm", vjson::Value::real(live->length_mm));
            liveJson.set("breadth_mm", vjson::Value::real(live->breadth_mm));
            status.set("live", liveJson);
        } else {
            status.set("live", vjson::Value::null());
        }

        status.set("last", last.has_value() ? trimResult(last->toJson()) : vjson::Value::null());

        const bool awaiting = e.awaitingNewLay();
        {
            std::lock_guard<std::mutex> guard(lock_);
            status.set("seeds", vjson::Value::integer(static_cast<long long>(seeds_.size())));
            status.set("pointer", vjson::Value::boolean(pointerPresent_));
            status.set("pointer_dwell", vjson::Value::real(pointerDwell_));
            status.set("tracing", vjson::Value::boolean(tracing_));
            status.set("traced", vjson::Value::integer(
                                     static_cast<long long>(tracePx_.size())));
            status.set("collar", collarJsonLocked());
            status.set("perimeter",
                       perimeterStepLocked() ? perimeterJsonLocked(held, awaiting) : vjson::Value::null());
        }

        {
            std::lock_guard<std::mutex> guard(lock_);
            publishSequence(status);

            // Only beside a verdict: the page keeps them against it, and a step
            // with no verdict has nothing to keep.
            const vjson::Value* verdict = status.find("result");
            const bool keep = marks.items.size() > 0 && verdict && !verdict->isNull();
            status.set("marks", keep ? marks.json() : vjson::Value::null());

            vjson::Value offered = vjson::Value::array();
            for (const TemplateEntry& t : templates_) {
                vjson::Value o = vjson::Value::object();
                o.set("style", vjson::Value::str(t.style));
                o.set("steps", vjson::Value::integer(t.steps));
                offered.push(o);
            }
            status.set("styles", offered);
        }

        status.set("calibrated", vjson::Value::boolean(model != nullptr));
        if (model) {
            const vjson::Value* rmse = model->meta().find("rmse_mm");
            status.set("rmse_mm", rmse ? *rmse : vjson::Value::null());
            status.set("scale_correction", vjson::Value::real(model->scaleCorrection()));
            status.set("scale_verified",
                       vjson::Value::boolean(model->meta().has("scale_verified_against_mm")));
            status.set("scale_verified_at",
                       vjson::Value::str(model->meta().getString("scale_verified_at", "")));
            status.set("calibrated_at",
                       vjson::Value::str(model->meta().getString("calibrated_at", "")));
        } else {
            status.set("rmse_mm", vjson::Value::null());
            status.set("scale_correction", vjson::Value::null());
            status.set("scale_verified", vjson::Value::boolean(false));
            status.set("scale_verified_at", vjson::Value::str(""));
            status.set("calibrated_at", vjson::Value::str(""));
        }
        // What Check scale allows a reference's thickness with.
        const std::optional<double>& distance = e.config().camera_distance_mm;
        status.set("working_distance_mm",
                   distance.has_value() ? vjson::Value::real(*distance) : vjson::Value::null());

        status.set("fps", vjson::Value::real(std::round(source_->fps() * 10.0) / 10.0));
        status.set("lag_ms",
                   vjson::Value::integer(static_cast<long long>(std::llround(age * 1000.0))));
        status.set("proc_ms",
                   vjson::Value::integer(static_cast<long long>(std::llround(procMs_))));
        status.set("dropped", vjson::Value::integer(source_->incompleteCount()));
        status.set("restarts", vjson::Value::integer(source_->restartCount()));

        {
            std::lock_guard<std::mutex> guard(lock_);
            vjson::Value logJson = vjson::Value::array();
            const std::size_t start = boot_.size() > 8 ? boot_.size() - 8 : 0;
            for (std::size_t k = boot_.size(); k > start; --k) {
                vjson::Value entry = vjson::Value::object();
                entry.set("time", vjson::Value::str(boot_[k - 1].first));
                entry.set("text", vjson::Value::str(boot_[k - 1].second));
                logJson.push(entry);
            }
            status.set("log", logJson);

            if (ok) {
                jpeg_ = jpeg;
                ++jpegSeq_;
            }
            status_ = status;
        }
    }

    // Everything the page needs to draw the step it is on. Called with lock_
    // already held.
    void publishSequence(vjson::Value& status) {
        // How long until the station moves on by itself, if it is going to.
        // Null whenever nothing is queued, which is almost always.
        if (autoAdvancePending_) {
            status.set("advance_in_s",
                       vjson::Value::real(std::max(0.0, autoAdvanceAt_ - nowSeconds())));
            // And how long the whole wait is, so the page can count it down
            // from the start rather than from whenever it first looked.
            status.set("advance_total_s", vjson::Value::real(cfg_.auto_advance_delay_s));
        } else {
            status.set("advance_in_s", vjson::Value::null());
        }

        // Whether the hand-drawn collar steps are on the bench, and how many
        // of the style's steps are kept off it. For the admin panel.
        status.set("manual_steps", vjson::Value::boolean(showManual_));
        status.set("hidden_steps", vjson::Value::integer(hiddenStepsLocked()));
        // How long a pass is counted down before the station moves on.
        status.set("advance_delay_s", vjson::Value::real(cfg_.auto_advance_delay_s));

        if (!haveTemplate_) {
            status.set("style", vjson::Value::null());
            status.set("step", vjson::Value::null());
            status.set("result", vjson::Value::null());
            status.set("steps", vjson::Value::array());
            status.set("garment", vjson::Value::integer(0));
            return;
        }

        const int total = static_cast<int>(template_.steps.size());
        // Belt and braces. The two are kept in step everywhere they are set,
        // and an index off the end of either would take the whole page down.
        if (total == 0 || stepResults_.size() != template_.steps.size()) {
            status.set("style", vjson::Value::str(template_.style));
            status.set("step", vjson::Value::null());
            status.set("result", vjson::Value::null());
            status.set("steps", vjson::Value::array());
            status.set("garment", vjson::Value::integer(garment_));
            return;
        }
        const int idx = std::min(std::max(stepIndex_, 0), total - 1);
        const Step& s = template_.steps[static_cast<std::size_t>(idx)];

        status.set("style", vjson::Value::str(template_.style));
        status.set("garment", vjson::Value::integer(garment_));

        vjson::Value step = vjson::Value::object();
        step.set("index", vjson::Value::integer(idx + 1));
        step.set("total", vjson::Value::integer(total));
        step.set("name", vjson::Value::str(s.name));
        step.set("instruction", vjson::Value::str(s.instruction));
        step.set("has_reference", vjson::Value::boolean(!s.reference_image.empty()));
        step.set("kind", vjson::Value::str(s.kind));
        // Which of a curve's two lengths this step reports. On the panel
        // because the operator has to know whether "254 mm" is the seam
        // followed or the span across it, and on a neckline those differ by
        // forty percent.
        if (s.isCurve()) {
            const bool arc = s.measure.empty() ? cfg_.curve_measure_along_arc
                                               : (s.measure == "arc");
            step.set("measure", vjson::Value::str(arc ? "arc" : "chord"));
            step.set("seeds_wanted", vjson::Value::integer(std::max(2, cfg_.curve_seed_count)));
        }
        // A distance is two points, always.
        if (s.isDistance()) step.set("seeds_wanted", vjson::Value::integer(2));
        // A collar says which edge of its band it reads, and which length.
        if (s.isCollar()) {
            const bool arc = s.measure.empty() ? cfg_.curve_measure_along_arc : (s.measure == "arc");
            step.set("measure", vjson::Value::str(s.measure == "around" ? "around" : arc ? "arc" : "chord"));
            step.set("seam", vjson::Value::str(s.seam.empty() ? "outer" : s.seam));
        }
        step.set("target_mm", vjson::Value::real(s.target_mm));
        step.set("tol_minus_mm", vjson::Value::real(s.tol_minus_mm));
        step.set("tol_plus_mm", vjson::Value::real(s.tol_plus_mm));
        step.set("low_mm", vjson::Value::real(s.low_mm()));
        step.set("high_mm", vjson::Value::real(s.high_mm()));

        // The same band as a share of the target, which is how the page states
        // it. Computed here rather than in the browser so the page and the log
        // cannot drift apart over a rounding rule. A step with no target has no
        // percentage, and the page falls back to millimetres.
        const bool hasPercent = s.target_mm > 1e-9;
        step.set("has_percent", vjson::Value::boolean(hasPercent));
        if (hasPercent) {
            step.set("tol_minus_pct", vjson::Value::real(s.tol_minus_mm / s.target_mm * 100.0));
            step.set("tol_plus_pct", vjson::Value::real(s.tol_plus_mm / s.target_mm * 100.0));
        }
        step.set("last", vjson::Value::boolean(idx + 1 >= total));
        status.set("step", step);

        const std::optional<StepResult>& current =
            stepResults_[static_cast<std::size_t>(idx)];
        if (current.has_value()) {
            vjson::Value result = current->toJson();
            if (current->reason == MANUAL_QC_REASON) result.set("manual_qc", vjson::Value::boolean(true));
            status.set("result", result);
        } else {
            status.set("result", vjson::Value::null());
        }

        // The strip along the top of the panel: one pip per step, so the
        // operator can see at a glance what is left and what already failed.
        vjson::Value strip = vjson::Value::array();
        for (int k = 0; k < total; ++k) {
            const Step& st = template_.steps[static_cast<std::size_t>(k)];
            vjson::Value pip = vjson::Value::object();
            pip.set("name", vjson::Value::str(st.name));
            pip.set("current", vjson::Value::boolean(k == idx));
            const std::optional<StepResult>& r = stepResults_[static_cast<std::size_t>(k)];
            pip.set("state", vjson::Value::str(!r.has_value() ? "pending"
                                                              : (r->pass ? "pass" : "fail")));
            // What the tooltip over the pip says: what the step asks for and,
            // once it has been read, what it read. A shirt's readings at a
            // glance, without stepping back through them to find out.
            pip.set("target_mm", vjson::Value::real(st.target_mm));
            if (r.has_value() && r->measured) {
                pip.set("measured_mm", vjson::Value::real(r->measured_mm));
                if (r->has_percent()) {
                    pip.set("deviation_pct", vjson::Value::real(r->deviation_pct()));
                } else {
                    pip.set("deviation_mm", vjson::Value::real(r->deviation_mm));
                }
            }
            if (r.has_value() && !r->reason.empty()) {
                pip.set("reason", vjson::Value::str(r->reason));
                if (r->reason == MANUAL_QC_REASON) pip.set("manual_qc", vjson::Value::boolean(true));
            }
            pip.set("kind", vjson::Value::str(st.kind));
            // The step's last picture on this shirt, if it has one: kept
            // across a Retake, so the take being redone can still be seen.
            pip.set("capture", captureJsonLocked(k));
            strip.push(pip);
        }
        status.set("steps", strip);
    }

    // -- the style on the bench --
    //
    // Guarded by lock_, because the operator can change style from the page
    // between garments while the capture thread is measuring one. Nothing here
    // may call log() while holding that lock: log() takes it too.
    bool loadTemplateFile(const std::string& path, std::string& error) {
        Template loaded;
        if (!loadTemplate(path, loaded, error)) return false;
        std::lock_guard<std::mutex> guard(lock_);
        fileTemplate_ = loaded;
        template_ = operatorSteps(loaded, showManual_, fileIndex_);
        haveTemplate_ = true;
        stepIndex_ = 0;
        stepResults_.assign(template_.steps.size(), std::nullopt);
        captureOf_.clear();
        autoAdvancePending_ = false;
        return true;
    }

    // Show or hide the steps the box round the collar replaced (see
    // operatorSteps). The list of steps changes under the shirt on the mat,
    // so the shirt starts again from step one; nothing already filed is lost.
    void setManualSteps(bool show) {
        {
            std::lock_guard<std::mutex> guard(lock_);
            showManual_ = show;
            if (haveTemplate_) template_ = operatorSteps(fileTemplate_, showManual_, fileIndex_);
        }
        resetSequence(false);
        rescanTemplates();
    }

    // -- the shift's figures --
    //
    // How many shirts the bench has finished today, and how long each took.
    // Only watched, never fed back: the time a shirt starts is read off the
    // engine's own state as it goes from an empty or held mat to settling a
    // new lay, and the time it ends is the moment its last verdict is on the
    // steps. Nothing here decides anything or changes when anything happens,
    // and it is all in memory, the way the steps' readings are, so every
    // screen on the station sees the same figures and a restart starts them
    // again. Called on the capture thread only; takes lock_ itself.
    void observeShift() {
        const double now = nowSeconds();
        const std::string engineState = engine_ ? engine_->state() : std::string();
        // A new lay: the engine starts settling something, from an empty mat
        // or from a lay it was holding.
        const bool newLay = engineState == STATE_STABILISING &&
                            (shiftEnginePrev_ == STATE_WAITING || shiftEnginePrev_ == STATE_MEASURED);
        const bool layInHand = engineState == STATE_STABILISING || engineState == STATE_CAPTURING;
        shiftEnginePrev_ = engineState;
        const std::string today = timestampIso().substr(0, 10);

        std::lock_guard<std::mutex> guard(lock_);
        bool changed = false;
        // Finished: every step has its verdict. A shirt fails on any step
        // that failed; one whose only misses were handed on to be measured
        // by hand is a manual-QC shirt rather than a failed one.
        bool done = haveTemplate_ && !stepResults_.empty();
        bool pass = true;
        bool flagged = false;
        bool failed = false;
        for (const std::optional<StepResult>& r : stepResults_) {
            if (!r.has_value()) {
                done = false;
                break;
            }
            pass = pass && r->pass;
            const bool flag = r->reason == MANUAL_QC_REASON;
            flagged = flagged || flag;
            failed = failed || (!r->pass && !flag);
        }
        // Midnight: the figures start again. A shirt on the bench that was
        // already counted stays counted where it was, even if a step of it
        // is taken again; one still being measured counts in the new day.
        if (today != shiftDay_) {
            if (!shiftDay_.empty()) {
                shift_.clear();
                shirtRecord_ = shirtDone_ ? kCountedBefore : -1;
            }
            shiftDay_ = today;
            shiftStartedAt_ = timestampClock();
            changed = true;
        }
        // Which shirt is on the bench: a new one with every Next shirt, and
        // with every change of style or cleared log, which start at shirt 1.
        const std::string style = haveTemplate_ ? template_.style : std::string();
        if (garment_ != shirtGarment_ || shiftEpoch_ != shirtEpoch_ || style != shirtStyle_) {
            shirtGarment_ = garment_;
            shirtEpoch_ = shiftEpoch_;
            shirtStyle_ = style;
            shirtBegan_ = now;
            // Already being laid when the shirt began: it started then.
            shirtLaid_ = layInHand;
            shirtLayAt_ = now;
            shirtRecord_ = -1;
            // A new shirt has no verdicts yet. If these are all in already,
            // they are the last shirt's, carried over by something that did
            // not start the steps again, and are not counted twice.
            shirtDone_ = done;
        }
        if (newLay && !shirtLaid_) {
            shirtLaid_ = true;
            shirtLayAt_ = now;
        }
        // Counted once; a step retaken and judged again after that updates
        // the same shirt.
        if (done && !shirtDone_ && shirtRecord_ != kCountedBefore) {
            ShiftShirt s;
            s.garment = garment_;
            s.seconds = now - (shirtLaid_ ? shirtLayAt_ : shirtBegan_);
            s.endedAt = now;
            s.clock = timestampClock();
            s.pass = pass;
            s.flagged = flagged;
            s.failed = failed;
            if (shirtRecord_ >= 0 && shirtRecord_ < static_cast<int>(shift_.size())) {
                shift_[static_cast<std::size_t>(shirtRecord_)] = s;
            } else {
                shift_.push_back(s);
                shirtRecord_ = static_cast<int>(shift_.size()) - 1;
            }
            changed = true;
        }
        shirtDone_ = done;
        if (changed) shiftJson_ = shiftJsonLocked();
    }

    // The figures, as the page shows them. Call with lock_ held.
    //
    // A shirt that took more than five minutes was left, not measured, so it
    // is counted and marked idle but kept out of the average, the median and
    // the rate. The rate is the bench's pace from one result to the next,
    // which includes laying the next shirt; a gap of over five minutes
    // between two results is a break, not a pace, and is left out. So there
    // is no rate until two shirts have come one after the other.
    vjson::Value shiftJsonLocked() const {
        const double IDLE_S = 300.0;
        vjson::Value v = vjson::Value::object();
        v.set("day", vjson::Value::str(shiftDay_));
        v.set("since", vjson::Value::str(shiftStartedAt_));
        v.set("idle_after_s", vjson::Value::real(IDLE_S));
        int passed = 0, failed = 0, manualQc = 0, flagged = 0, idle = 0;
        std::vector<double> times;
        double active = 0.0;
        int paced = 0;
        for (std::size_t k = 0; k < shift_.size(); ++k) {
            const ShiftShirt& s = shift_[k];
            if (s.pass) ++passed;
            else if (s.failed) ++failed;
            else ++manualQc;
            if (s.flagged) ++flagged;
            const bool isIdle = s.seconds > IDLE_S;
            if (isIdle) {
                ++idle;
                continue;
            }
            times.push_back(s.seconds);
            if (k > 0) {
                const double cycle = s.endedAt - shift_[k - 1].endedAt;
                if (cycle <= IDLE_S) {
                    active += cycle;
                    ++paced;
                }
            }
        }
        v.set("shirts", vjson::Value::integer(static_cast<long long>(shift_.size())));
        v.set("passed", vjson::Value::integer(passed));
        // Not passed: failed on a reading, or passed everywhere but handed
        // on for manual QC somewhere. The three add up to the shirts.
        v.set("failed", vjson::Value::integer(failed));
        v.set("manual_qc", vjson::Value::integer(manualQc));
        v.set("flagged", vjson::Value::integer(flagged));
        v.set("idle", vjson::Value::integer(idle));
        if (!times.empty()) {
            double sum = 0.0;
            for (double t : times) sum += t;
            std::vector<double> sorted = times;
            std::sort(sorted.begin(), sorted.end());
            const std::size_t n = sorted.size();
            const double median = n % 2 ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
            v.set("avg_s", vjson::Value::real(sum / static_cast<double>(n)));
            v.set("median_s", vjson::Value::real(median));
        } else {
            v.set("avg_s", vjson::Value::null());
            v.set("median_s", vjson::Value::null());
        }
        v.set("per_hour", active > 0.0 && paced > 0
                              ? vjson::Value::real(3600.0 * paced / active) : vjson::Value::null());
        if (!shift_.empty()) {
            const ShiftShirt& last = shift_.back();
            vjson::Value l = vjson::Value::object();
            l.set("shirt", vjson::Value::integer(last.garment));
            l.set("seconds", vjson::Value::real(last.seconds));
            l.set("at", vjson::Value::str(last.clock));
            l.set("result", vjson::Value::str(last.result()));
            l.set("idle", vjson::Value::boolean(last.seconds > IDLE_S));
            v.set("last", l);
        } else {
            v.set("last", vjson::Value::null());
        }
        // The most recent few, newest first, for the admin panel.
        vjson::Value recent = vjson::Value::array();
        for (std::size_t k = shift_.size(); k > 0 && shift_.size() - k < 12; --k) {
            const ShiftShirt& s = shift_[k - 1];
            vjson::Value r = vjson::Value::object();
            r.set("shirt", vjson::Value::integer(s.garment));
            r.set("seconds", vjson::Value::real(s.seconds));
            r.set("at", vjson::Value::str(s.clock));
            r.set("result", vjson::Value::str(s.result()));
            r.set("idle", vjson::Value::boolean(s.seconds > IDLE_S));
            recent.push(r);
        }
        v.set("recent", recent);
        return v;
    }

    // How many of the style's steps are kept off the bench. Call with lock_ held.
    int hiddenStepsLocked() const {
        return haveTemplate_ ? static_cast<int>(fileTemplate_.steps.size() - template_.steps.size()) : 0;
    }

    // Accepts a style name, as the page sends it, or a path, as a barcode
    // scanner or a shift script might.
    bool selectTemplate(const std::string& styleOrPath, std::string& error) {
        std::string path;
        {
            std::lock_guard<std::mutex> guard(lock_);
            for (const TemplateEntry& t : templates_) {
                if (t.style == styleOrPath) {
                    path = t.path;
                    break;
                }
            }
        }
        return loadTemplateFile(path.empty() ? styleOrPath : path, error);
    }

    void rescanTemplates() {
        std::vector<TemplateEntry> found = offeredTemplates();
        std::lock_guard<std::mutex> guard(lock_);
        templates_.swap(found);
    }

    // The file of the style with this name, from the list the station made of
    // its own templates directory; empty if there is none.
    std::string stylePath(const std::string& style) const {
        std::lock_guard<std::mutex> guard(lock_);
        for (const TemplateEntry& t : templates_) {
            if (t.style == style) return t.path;
        }
        return std::string();
    }

    // The styles in the templates directory, each with the number of steps the
    // operator will work through rather than the number in its file.
    std::vector<TemplateEntry> offeredTemplates() const {
        std::vector<TemplateEntry> found = listTemplates(cfg_.templates_dir);
        bool show = false;
        {
            std::lock_guard<std::mutex> guard(lock_);
            show = showManual_;
        }
        for (TemplateEntry& t : found) {
            Template full;
            std::string error;
            std::vector<int> index;
            if (loadTemplate(t.path, full, error)) {
                t.steps = static_cast<int>(operatorSteps(full, show, index).steps.size());
            }
        }
        return found;
    }

    Config cfg_;
    int jpegQuality_;
    // The working distance came off a tape measure (--camera-distance-mm, or
    // saved in bench.json) rather than out of the lens and a calibration. A
    // recalibration then keeps it, and only compares it with the lens.
    bool distanceMeasured_ = false;

    // The steps the operator works through (see operatorSteps), and the style
    // file they came from, whole, which is what a target edited from the bench
    // is saved into. fileIndex_[k] is step k's place in the file.
    bool showManual_ = false;
    Template template_;
    Template fileTemplate_;
    std::vector<int> fileIndex_;
    bool haveTemplate_ = false;
    std::vector<TemplateEntry> templates_;

    // -- the camera's exposure, as Settings shows it (see exposureJsonLocked) --
    //
    // Under lock_. exposureLevel_ mirrors the metering target in cfg_, which
    // only the capture thread changes, so the HTTP thread never reads cfg_.
    double exposureUs_ = 0.0;
    bool exposureByHand_ = false;
    double exposureLevel_ = 0.0;
    double exposureDefaultLevel_ = 0.0;
    std::optional<double> pendingExposureLevel_;

    // -- the shift's figures (see observeShift) --
    //
    // Under lock_, except shiftEnginePrev_, which only the capture thread
    // touches. shiftEpoch_ moves with a change of style or a cleared log,
    // which start the shirts at 1 again, so shirt 1 then is a new shirt.
    struct ShiftShirt {
        int garment = 0;
        double seconds = 0.0;  // from its first lay to its last verdict
        double endedAt = 0.0;  // nowSeconds() of that last verdict
        std::string clock;     // and the time of day it came
        bool pass = false;
        bool flagged = false;  // a step handed on for manual QC
        bool failed = false;   // a step that failed on its reading
        // What the shirt came to: FAIL over MANUAL QC over PASS.
        const char* result() const { return pass ? "PASS" : failed ? "FAIL" : "MANUAL QC"; }
    };
    // shirtRecord_ for a shirt finished and counted in a shift since
    // cleared: a step of it taken again is not counted again.
    static constexpr int kCountedBefore = -2;
    std::vector<ShiftShirt> shift_;
    std::string shiftDay_;
    std::string shiftStartedAt_;
    vjson::Value shiftJson_;
    long long shiftEpoch_ = 0;
    int shirtGarment_ = -1;
    long long shirtEpoch_ = -1;
    std::string shirtStyle_;
    double shirtBegan_ = 0.0;
    double shirtLayAt_ = 0.0;
    bool shirtLaid_ = false;
    bool shirtDone_ = false;
    int shirtRecord_ = -1;
    std::string shiftEnginePrev_;

    // Where we are in the list, and what each step has said so far for the
    // shirt currently on the mat.
    cv::Point2d scan_{0.0, 0.0};
    bool haveScan_ = false;
    double reachMm_ = 0.0;  // how long the rail is, and so what the bench can reach
    // The measured edge as it lies, in full-resolution pixels. A polyline
    // rather than two points, because the edge is not a straight line and an
    // operator checking whether the machine found the right thing is entitled
    // to see what it actually measured lying on the cloth.
    std::vector<cv::Point2d> edgePath_;
    bool haveEdge_ = false;

    // -- pointing at a seam --
    //
    // The points the operator has put down for the step they are on, and the
    // seam that was found between them. Both in the units they are used in: the
    // seeds in millimetres, because that is what snapCurve takes and what the
    // audit log records, and the drawn path in full-resolution pixels, because
    // that is what the overlay needs.
    //
    // Cleared on every move between steps, garments and styles. A seed left
    // over from the last step would put a corridor across the wrong part of the
    // shirt, and the reading that came back would look perfectly ordinary.
    std::vector<cv::Point2d> seeds_;
    std::vector<cv::Point2d> curvePath_;
    bool haveCurve_ = false;
    // Where the finger is right now, for the feed. Not a measurement and never
    // logged; the seeds are what survive a point.
    bool pointerPresent_ = false;
    cv::Point2d pointerPx_{0.0, 0.0};
    double pointerDwell_ = 0.0;
    std::unique_ptr<FingerTracker> finger_;
    // The trace being drawn right now, in full-resolution pixels, for the feed.
    // Not a measurement and never logged: what survives a trace is the path it
    // hands over when it ends.
    std::vector<cv::Point2d> tracePx_;
    // The curve those points describe, fitted the way the corridor will be
    // built along it, so what is drawn while the hand moves is what will be
    // measured when it stops.
    std::vector<cv::Point2d> traceCurvePx_;
    // The tip on this frame was somewhere a hand could not have got to, and was
    // not recorded; the ring is drawn hollow for it.
    bool pointerRejected_ = false;
    // The last distance read on this step: its two ends in full-resolution
    // pixels, and the number, kept on the picture until the next first tap -
    // or until the step is left or retaken, which clears it with the rest of
    // the feed.
    std::vector<cv::Point2d> distancePx_;
    bool haveDistance_ = false;
    double distanceMm_ = 0.0;
    // The seams the machine found between the shoulder corners and could not
    // choose between, in full-resolution pixels, drawn thin on the feed until
    // the operator points or the reading is taken.
    std::vector<std::vector<cv::Point2d>> seamOptionsPx_;
    // -- a collar read from a box --
    //
    // The box the operator dragged round the collar, in full-resolution
    // pixels, and what was read inside it: the result itself, kept so a tap
    // can choose among its lines when it offered them; its lines and the one
    // measured, for the feed; and what the panel says about it. Cleared with
    // everything else drawn for a step (clearSeeds, from serviceStepChange)
    // and whenever the reading stops being held.
    bool haveCollarBox_ = false;
    cv::Rect2d collarBoxPx_;
    bool haveCollar_ = false;
    CollarResult collar_;
    std::vector<std::vector<cv::Point2d>> collarLinesPx_;
    // Each line's role ("inner", "outer", or empty for one only offered), and
    // for a collar measured all the way round the back of the neck that
    // closes the chosen line into the whole collar.
    std::vector<std::string> collarRoles_;
    std::vector<cv::Point2d> collarBackPx_;
    int collarChosen_ = -1;
    double collarMm_ = 0.0;
    std::string collarState_;  // "", "reading", "read", "choose", "refused"
    std::string collarWhy_;    // how the edge was chosen, or why nothing was read
    unsigned long long collarSeq_ = 0;  // boxes drawn; never reset
    // -- the full perimeter, in two halves --
    //
    // The halves read so far on the take showing, and what the feed and the
    // panel say about them. Cleared with everything else drawn for a step, in
    // serviceStepChange, so a half read for one shirt or one take can never be
    // added to a half of another.
    int perimHave_ = 0;  // 0 waiting for the first half, 1 for the second, 2 read
    PerimeterHalf perimHalf_[2];
    bool perimWhole_ = false;  // both halves came from one picture
    double perimTotalMm_ = 0.0;
    // The halves as measured, the cuts at the clips and the corners crossed,
    // on the raw picture, for the feed.
    std::vector<std::vector<cv::Point2d>> perimPathsPx_;
    std::vector<cv::Point2d> perimCutsPx_;
    std::vector<cv::Point2d> perimCornersPx_;
    std::string perimMessage_;  // what the last reading of this take said
    bool perimBad_ = false;     // and that it was refused
    // The live guide, worked out every frame on a perimeter step, and where
    // its band and margins lie, worked out once per calibration and area.
    PerimeterGuide perimGuide_;
    PerimeterGuideLines perimLines_;
    bool havePerimLines_ = false;
    const PlaneModel* perimLinesModel_ = nullptr;
    std::string perimLinesCalibratedAt_;  // with the address: a new model can reuse an old one's
    RoiBox perimLinesRoi_;
    bool tracing_ = false;
    // The frame the reading is held on: garment settled, no hand over it. The
    // seam is read from this and not from the live frame, because on a curve
    // step the live frame has the operator's finger lying across the very thing
    // being measured. Touched only by the capture thread.
    cv::Mat heldFrame_;
    // A `dump` asked for the whole working area too: written from the next
    // frame (dumpWholeView). Touched only by the capture thread.
    bool dumpWholeViewNext_ = false;
    // Whether the pointing has its picture: heldFrame_ taken, and on a curve
    // step the tracker's reference with it. Set when a reading is held, dropped
    // whenever the picture underneath changes.
    bool fingerArmed_ = false;

    int stepIndex_ = 0;
    int garment_ = 1;
    std::vector<std::optional<StepResult>> stepResults_;

    // The pictures of the verdicts (captureVerdict). The store writes them;
    // this says which is the last one of each step of the shirt on the bench,
    // (garment, step index) -> the store's id for it, and is emptied with
    // every new shirt, style or cleared log. Guarded by lock_. A Retake keeps
    // the step's picture: it is the take being redone.
    struct CaptureRef {
        unsigned long long id = 0;
        std::string kind;  // PASS, FAIL or MANUAL
        std::string at;    // when, local, YYYY-MM-DDTHH:MM:SS
    };
    std::map<std::pair<int, int>, CaptureRef> captureOf_;
    std::unique_ptr<CaptureStore> captures_;
    // Named by captureVerdict, handed over by flushCaptures. Capture thread only.
    std::vector<PendingCapture> pendingCaptures_;
    // The newest frame, shared rather than copied. Capture thread only.
    cv::Mat lastFrame_;

    // Which take of which step the feed is drawn for.
    //
    // A take is one attempt at one step. Arriving at a step starts one, and so
    // does anything that starts a step over where it stands: Retake, and a
    // change of style or a cleared log, which put the station back on step one
    // of shirt one - where it may already have been, so the step and the shirt
    // alone would not show that anything had changed. Those bump stepTake_.
    //
    // serviceStepChange compares the take on screen against the one the feed
    // was last drawn for, and clears the feed when they differ.
    unsigned long long stepTake_ = 0;
    int feedGarment_ = 0;
    int feedIndex_ = -1;
    unsigned long long feedTake_ = 0;
    // Whether the reading the engine is holding was taken during this take.
    // The engine keeps a reading until the mat has been cleared or the next
    // lay is measured, and knows nothing of steps, so without this its outline
    // outlives the step it was measured for.
    bool heldForTake_ = false;
    // Which step of which shirt the reading the engine last took was taken
    // for, whichever take it was. A retake of that step may read the lay
    // again on a smaller change than a step arrived at fresh; see
    // serviceStepChange. -1 once the sequence starts over.
    int heldIndex_ = -1;
    int heldGarment_ = 0;

    // A pass that has not yet turned into a move.
    //
    // The move is deferred rather than taken on the spot so the operator sees
    // the verdict that caused it. It is armed against a particular step of a
    // particular shirt, and re-checked against both before it fires, because
    // between arming and firing the operator may have pressed Back, retaken the
    // step, started a new shirt or changed style. In every one of those cases
    // the move would land on the wrong fold, so it is dropped instead.
    bool autoAdvancePending_ = false;
    double autoAdvanceAt_ = 0.0;
    int autoAdvanceIndex_ = -1;
    int autoAdvanceGarment_ = 0;

    std::unique_ptr<MeasurementEngine> engine_;
    std::unique_ptr<FrameSource> source_;
    std::unique_ptr<Camera> camera_;

    mutable std::mutex lock_;
    std::vector<unsigned char> jpeg_;
    long long jpegSeq_ = 0;
    vjson::Value status_;
    std::vector<vjson::Value> results_;
    std::atomic<bool> stop_{false};
    std::string command_;
    std::string commandArg_;
    std::deque<std::pair<std::string, std::string>> boot_;
    double procMs_ = 0.0;
    // The engine's refusals already shown, so a lay refused on every settle
    // is not said again every few seconds. See reportRefusal.
    long long refusalsShown_ = 0;
    std::string refusalText_;
    double refusalAt_ = 0.0;
    std::thread thread_;

    // Booted and measuring. Nothing can be recalibrated before this.
    bool live_ = false;

    // -- recalibrating --
    //
    // The request, its progress and its outcome, as the page sees them. All
    // under lock_: the HTTP thread asks for a run and reads its state, the
    // worker reports how far it has got, and the capture thread starts it and
    // puts the result into service.
    struct Calibration {
        bool active = false;   // asked for and not yet finished
        bool started = false;  // the worker has been started for it
        // "queued", then "settle", then "fit" - with "clear" first while
        // something lies on the base, and "exposure" when it is metered
        std::string phase;
        int done = 0;          // how far the wait for a still mat has got
        int needed = 0;
        double hidden = 0.0;   // while "clear": how much of the grid is hidden
        double startedAt = 0.0;
        long long seq = 0;     // one per request, so the page can tell runs apart

        // How the last run to finish ended.
        bool haveOutcome = false;
        long long outcomeSeq = 0;
        bool ok = false;
        bool cancelled = false;  // called off from the page: nothing to report
        bool warned = false;     // done, with something the operator should act on
        std::string headline;
        std::string detail;
        double seconds = 0.0;
    };
    Calibration calib_;
    // Set from the page to call off a run still waiting for the base to clear.
    std::atomic<bool> calibCancel_{false};

    // -- checking the scale --
    //
    // Open from the page while a reference of known length - a steel rule - is
    // laid on the base and read. Nothing is judged meanwhile: the rule is not a
    // garment, and a step scored on it would be a verdict about nothing. Under
    // lock_, like everything the page reads.
    struct ScaleCheck {
        bool active = false;
        long long seq = 0;         // one per opening, so the page can tell them apart
        int readings = 0;          // how many times the reference has been read
        double readingMm = 0.0;    // its length end to end, on the base's plane
        double breadthMm = 0.0;
        // The latest reading in each part of the base the reference has been
        // laid in, in the order they were first read. See scalePlace.
        struct Place {
            std::string where;
            double mm = 0.0;
            double breadth = 0.0;
        };
        std::vector<Place> places;
        bool haveOutcome = false;  // how the last Apply ended
        bool ok = false;
        std::string outcome;
    };
    ScaleCheck scaleCheck_;
    // What the worker came back with, waiting for the capture thread.
    bool calibFinished_ = false;
    PlaneModelPtr calibModel_;
    cv::Mat calibBackground_;
    std::string calibError_;
    ExposureChange calibExposure_;
    // What the grid was, and whether the run only retook the base (see
    // calibrationWorker).
    LatticeInfo calibLattice_;
    bool calibBaseOnly_ = false;
    bool calibCancelled_ = false;
    std::thread calibThread_;
};

Station* g_station = nullptr;
HttpServer* g_server = nullptr;

void onInterrupt(int) {
    std::printf("\nStopping.\n");
    std::fflush(stdout);
    if (g_server) g_server->shutdown();
}

struct Args {
    std::string host = "127.0.0.1";
    int port = 8000;
    std::string results;
    std::string template_path;
    std::string templates_dir;
    std::string scan_from;
    bool have_object_height = false;
    double object_height_mm = 0.0;
    bool have_focal_length = false;
    double focal_length_mm = 0.0;
    bool have_camera_distance = false;
    double camera_distance_mm = 0.0;
    bool have_corner_reach = false;
    double corner_reach_mm = 0.0;
    bool measure_along_curve = false;
    bool have_exposure = false;
    double exposure_us = 0.0;
    bool have_exposure_target = false;
    double exposure_target = 0.0;
    bool forget_distortion = false;
    bool manual_collar_steps = false;
    bool have_advance_delay = false;
    double advance_delay_s = 0.0;
    bool no_capture = false;
    std::string capture_dir;
    int capture_keep_days = -1;
    double capture_max_gb = -1.0;
    int capture_quality = -1;
};

// How long a pass is shown, counted down, before the station moves on by
// itself. Long enough to read the result and see what comes next.
const double DEFAULT_ADVANCE_DELAY_S = 5.0;
double clampAdvanceDelay(double s) { return std::min(30.0, std::max(1.0, s)); }

void printUsage() {
    std::printf(
        "usage: verimeasure_station [--host HOST] [--port PORT] [--results PATH]\n"
        "                           [--template PATH] [--templates-dir DIR]\n"
        "                           [--object-height-mm F] [--focal-length-mm F]\n"
        "                           [--camera-distance-mm F] [--exposure-us F]\n"
        "                           [--exposure-target N] [--forget-distortion]\n"
        "                           [--manual-collar-steps] [--advance-delay-s F]\n"
        "                           [--no-capture] [--capture-dir DIR] [--capture-quality N]\n"
        "                           [--capture-keep-days N] [--capture-max-gb F]\n\n"
        "  --host HOST           use 0.0.0.0 to reach it from a tablet on the bench\n"
        "  --port PORT           default 8000\n"
        "  --results PATH        append every step result to this JSON Lines file\n"
        "  --template PATH       the style to check: its steps are what the\n"
        "                        operator works through. The style can also be\n"
        "                        switched from the page, which offers everything\n"
        "                        in the templates directory.\n"
        "  --templates-dir DIR   where styles and their reference pictures live,\n"
        "                        default templates/\n"
        "  --focal-length-mm F   the lens, which the working distance for the fold\n"
        "                        thickness correction is worked out from. Saved for\n"
        "                        this bench in bench.json, so it only needs giving\n"
        "                        once; 0 forgets it. Without one the distance is\n"
        "                        guessed from the calibration, which can be badly off.\n"
        "  --camera-distance-mm F the distance from the camera to the surface,\n"
        "                        measured with a tape. Used for the fold thickness\n"
        "                        correction in place of the lens, kept through a\n"
        "                        recalibration, and saved in bench.json so it only\n"
        "                        needs giving once; 0 forgets it.\n"
        "  --exposure-us F       the camera exposure, in microseconds. Normally set\n"
        "                        by the station from the clear mat, saved in\n"
        "                        bench.json and used on every start, and set again\n"
        "                        by Recalibrate. Given here it is saved as set by\n"
        "                        hand, and Recalibrate leaves it alone; 0 forgets\n"
        "                        the saved one, so it is set from the mat again.\n"
        "  --exposure-target N   the level, 0-255, the clear base is metered to when\n"
        "                        the exposure is set from it. Default about 140, which\n"
        "                        leaves a white garment room before it clips; higher\n"
        "                        lifts dark cloth off the noise and takes that room\n"
        "                        away. Saved in bench.json; 0 forgets it. Takes effect\n"
        "                        at the next Recalibrate.\n"
        "  --forget-distortion   drop the lens distortion saved in bench.json by\n"
        "                        verimeasure_cli --fit-distortion, so recalibrations\n"
        "                        fit the radial terms afresh again.\n"
        "  --manual-collar-steps show the hand-drawn collar steps (tracing the\n"
        "                        neckline, tapping the shoulder points) in a style\n"
        "                        that also has a box collar step. Hidden by default;\n"
        "                        also switched from the page's Settings.\n"
        "  --advance-delay-s F   how long a pass is counted down on the page before\n"
        "                        the station moves to the next step, 1-30 s.\n"
        "                        Default 5; also set from the page's Settings.\n"
        "  --no-capture          do not save a picture of each verdict. By default\n"
        "                        every PASS and FAIL is saved, as the feed showed it,\n"
        "                        under DIR/PASS|FAIL|MANUAL/YYYY-MM-DD/, and shown\n"
        "                        again beside the feed when the step is revisited.\n"
        "  --capture-dir DIR     where those pictures go, default captures\n"
        "  --capture-quality N   their JPEG quality, 1-100, default 85\n"
        "  --capture-keep-days N keep the last N days of them, 0 (default) for all\n"
        "  --capture-max-gb F    keep at most F GB of them, oldest days removed\n"
        "                        first; 0 (default) for no limit. Only *.jpg in the\n"
        "                        date folders under DIR is ever removed.\n"
        "  --measure-along-curve follow the edge instead of crossing it. A tape laid\n"
        "                        flat between two seams reads the chord, which is what\n"
        "                        a tech pack states and what this reports by default;\n"
        "                        this reports the length of the edge itself. On a real\n"
        "                        garment the two differ by about a millimetre in five\n"
        "                        hundred. Both are in the log either way.\n"
        "  --corner-reach-mm F   how far past the end of an edge the measurement may\n"
        "                        reach for the corner the seams would have met at.\n"
        "                        Cloth rounds a corner rather than turning one, and\n"
        "                        a tape is laid to the meeting point; this is that\n"
        "                        correction. It only ever adds, so it is bounded and\n"
        "                        reported as reach_mm on every reading. Set it to 0\n"
        "                        to turn it off and read the cloth as it lies.\n"
        "  --scan-from EDGE      which side of the picture the fold is presented to:\n"
        "                        left, right, top or bottom. Default bottom, which\n"
        "                        lays the fold across the long axis of the view so a\n"
        "                        full chest or body width fits in frame. The machine\n"
        "                        measures the first straight edge it meets looking in\n"
        "                        from here, and the side is marked on the live feed.\n");
}

bool parseArgs(int argc, char** argv, Args& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--host" && i + 1 < argc) {
            args.host = argv[++i];
        } else if (a == "--port" && i + 1 < argc) {
            args.port = std::atoi(argv[++i]);
        } else if (a == "--results" && i + 1 < argc) {
            args.results = argv[++i];
        } else if (a == "--template" && i + 1 < argc) {
            args.template_path = argv[++i];
        } else if (a == "--templates-dir" && i + 1 < argc) {
            args.templates_dir = argv[++i];
        } else if (a == "--scan-from" && i + 1 < argc) {
            args.scan_from = argv[++i];
        } else if (a == "--object-height-mm" && i + 1 < argc) {
            args.object_height_mm = std::strtod(argv[++i], nullptr);
            args.have_object_height = true;
        } else if (a == "--measure-along-curve") {
            args.measure_along_curve = true;
        } else if (a == "--corner-reach-mm" && i + 1 < argc) {
            args.corner_reach_mm = std::strtod(argv[++i], nullptr);
            args.have_corner_reach = true;
        } else if (a == "--focal-length-mm" && i + 1 < argc) {
            args.focal_length_mm = std::strtod(argv[++i], nullptr);
            args.have_focal_length = true;
        } else if (a == "--camera-distance-mm" && i + 1 < argc) {
            args.camera_distance_mm = std::strtod(argv[++i], nullptr);
            args.have_camera_distance = true;
        } else if (a == "--exposure-us" && i + 1 < argc) {
            args.exposure_us = std::strtod(argv[++i], nullptr);
            args.have_exposure = true;
        } else if (a == "--exposure-target" && i + 1 < argc) {
            args.exposure_target = std::strtod(argv[++i], nullptr);
            args.have_exposure_target = true;
        } else if (a == "--forget-distortion") {
            args.forget_distortion = true;
        } else if (a == "--manual-collar-steps") {
            args.manual_collar_steps = true;
        } else if (a == "--advance-delay-s" && i + 1 < argc) {
            args.advance_delay_s = std::strtod(argv[++i], nullptr);
            args.have_advance_delay = true;
        } else if (a == "--no-capture") {
            args.no_capture = true;
        } else if (a == "--capture-dir" && i + 1 < argc) {
            args.capture_dir = argv[++i];
        } else if (a == "--capture-quality" && i + 1 < argc) {
            args.capture_quality = std::atoi(argv[++i]);
        } else if (a == "--capture-keep-days" && i + 1 < argc) {
            args.capture_keep_days = std::atoi(argv[++i]);
        } else if (a == "--capture-max-gb" && i + 1 < argc) {
            args.capture_max_gb = std::strtod(argv[++i], nullptr);
        } else if (a == "-h" || a == "--help") {
            printUsage();
            std::exit(0);
        } else {
            std::printf("unrecognised argument: %s\n", a.c_str());
            printUsage();
            return false;
        }
    }
    return true;
}

double queryNumber(const std::map<std::string, std::string>& query, const char* key,
                   double fallback) {
    const auto it = query.find(key);
    if (it == query.end()) return fallback;
    char* end = nullptr;
    const double v = std::strtod(it->second.c_str(), &end);
    if (end == it->second.c_str()) return fallback;
    return v;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 2;

    Config cfg;
    if (!args.results.empty()) cfg.results_file = args.results;
    if (!args.template_path.empty()) cfg.template_file = args.template_path;
    if (!args.templates_dir.empty()) cfg.templates_dir = args.templates_dir;
    if (!args.scan_from.empty()) cfg.scan_from = args.scan_from;
    if (args.have_object_height) cfg.object_height_mm = args.object_height_mm;
    // Given once and remembered, so a start without the flag still has it.
    const std::string lensLine =
        settleLens(cfg, args.have_focal_length, args.focal_length_mm);
    // The same for the exposure, which is otherwise set from the mat and
    // remembered; see setUpExposure.
    const std::string exposureLine = settleExposure(cfg, args.have_exposure, args.exposure_us);
    // And the level it is metered to, which is the bench's to choose.
    const std::string targetLine =
        settleExposureTarget(cfg, args.have_exposure_target, args.exposure_target);
    // Straight from a tape measure, and preferred over the focal length when
    // both are given. Deriving the distance from the lens needs the focal
    // length and the sensor's pixel pitch to both be right, and a figure off a
    // datasheet for a lens that has since been refocused is not obviously
    // wrong in the way a tape measure is obviously right. Given once and
    // remembered, like the lens.
    const std::string distanceLine =
        settleDistance(cfg, args.have_camera_distance, args.camera_distance_mm);
    // The lens's distortion, measured once across several pictures of the
    // rigid mat and kept like the lens, so a recalibration fits only where the
    // camera and the mat are.
    const std::string distortionLine = settleDistortion(cfg, args.forget_distortion);
    if (args.have_corner_reach) cfg.edge_corner_reach_mm = args.corner_reach_mm;
    if (args.measure_along_curve) cfg.edge_measure_along_curve = true;
    cfg.auto_advance_delay_s =
        clampAdvanceDelay(args.have_advance_delay ? args.advance_delay_s : DEFAULT_ADVANCE_DELAY_S);
    if (args.no_capture) cfg.capture_enabled = false;
    if (!args.capture_dir.empty()) cfg.capture_dir = args.capture_dir;
    if (args.capture_quality > 0) cfg.capture_jpeg_quality = std::min(100, args.capture_quality);
    if (args.capture_keep_days >= 0) cfg.capture_keep_days = args.capture_keep_days;
    if (args.capture_max_gb >= 0.0) cfg.capture_max_gb = args.capture_max_gb;

    Station station(cfg, 72, args.manual_collar_steps);
    g_station = &station;
    // First in the log, so what the working distance is worked out from is on
    // the page before anything is measured with it.
    if (!lensLine.empty()) station.log(lensLine);
    if (!distanceLine.empty()) station.log(distanceLine);
    if (!distortionLine.empty()) station.log(distortionLine);
    if (!exposureLine.empty()) station.log(exposureLine);
    if (!targetLine.empty()) station.log(targetLine);
    station.start();

    HttpServer server(
        args.host, args.port,
        [&station](const std::string& method, const std::string& path,
                   const std::map<std::string, std::string>& query, HttpConnection& conn) {
            if (method == "GET") {
                if (path == "/") {
                    conn.sendResponse(200, "text/html; charset=utf-8", STATION_PAGE);
                    return;
                }
                if (path == "/api/status") {
                    std::vector<unsigned char> jpeg;
                    long long jpegSeq = 0;
                    vjson::Value status, results;
                    station.snapshot(jpeg, jpegSeq, status, results);
                    conn.sendResponse(200, "application/json", status.dump());
                    return;
                }
                if (path == "/api/results") {
                    std::vector<unsigned char> jpeg;
                    long long jpegSeq = 0;
                    vjson::Value status, results;
                    station.snapshot(jpeg, jpegSeq, status, results);
                    conn.sendResponse(200, "application/json", results.dump());
                    return;
                }
                // One style's steps, for making a new style from it.
                if (path == "/api/style") {
                    const auto it = query.find("name");
                    vjson::Value out;
                    std::string error;
                    if (it == query.end() || !station.styleStepsJson(it->second, out, error)) {
                        vjson::Value no = vjson::Value::object();
                        no.set("ok", vjson::Value::boolean(false));
                        no.set("error", vjson::Value::str(it == query.end() ? "no style named" : error));
                        conn.sendResponse(404, "application/json", no.dump());
                        return;
                    }
                    out.set("ok", vjson::Value::boolean(true));
                    conn.sendResponse(200, "application/json", out.dump());
                    return;
                }
                // The picture for one step. The page asks by step number and
                // never names a file, so nothing a browser sends decides what
                // gets read off disk.
                if (path == "/api/reference") {
                    const auto it = query.find("step");
                    const int index = it == query.end() ? -1 : std::atoi(it->second.c_str()) - 1;
                    std::string file, body;
                    if (!station.referenceImagePath(index, file) || !readWholeFile(file, body)) {
                        conn.sendResponse(404, "text/plain", "no reference image for that step");
                        return;
                    }
                    conn.sendResponse(200, contentTypeFor(file), body);
                    return;
                }
                // A verdict's saved picture: the last one taken of a step of
                // the shirt on the bench. Asked for by shirt and step number,
                // never by file - the file is the station's own record of what
                // it wrote - so nothing a browser sends picks what is read.
                if (path == "/api/capture") {
                    const auto number = [&query](const char* key) {
                        const auto it = query.find(key);
                        if (it == query.end() || it->second.empty() || it->second.size() > 6) return -1;
                        for (const char c : it->second) {
                            if (c < '0' || c > '9') return -1;
                        }
                        return std::atoi(it->second.c_str());
                    };
                    std::string body;
                    const int code = station.captureImage(number("garment"), number("step"), body);
                    if (code == 200) {
                        conn.sendResponse(200, "image/jpeg", body);
                    } else if (code == 202) {
                        conn.sendResponse(202, "text/plain", "saving");
                    } else {
                        conn.sendResponse(404, "text/plain", "no picture for that step");
                    }
                    return;
                }
                if (path == "/stream.mjpg") {
                    conn.beginMjpeg();
                    long long sent = -1;
                    while (conn.alive()) {
                        std::vector<unsigned char> jpeg;
                        long long jpegSeq = 0;
                        if (!station.frameAfter(sent, jpeg, jpegSeq)) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(20));
                            continue;
                        }
                        sent = jpegSeq;
                        if (!conn.sendJpegPart(jpeg)) break;
                    }
                    return;
                }
                conn.sendResponse(404, "text/plain", "not found");
                return;
            }

            if (method == "POST") {
                if (path == "/api/step") {
                    const int index =
                        static_cast<int>(queryNumber(query, "index", 0.0)) - 1;
                    const double target = queryNumber(query, "target_mm", -1.0);
                    const double minus = queryNumber(query, "tol_minus_mm", 0.0);
                    const double plus = queryNumber(query, "tol_plus_mm", 0.0);
                    std::string error;
                    const bool ok = station.editStep(index, target, minus, plus, error);
                    vjson::Value out = vjson::Value::object();
                    out.set("ok", vjson::Value::boolean(ok));
                    if (!ok) out.set("error", vjson::Value::str(error));
                    conn.sendResponse(ok ? 200 : 400, "application/json", out.dump());
                    return;
                }

                // A new style, made from one already here. Answered at once, so
                // the page can say whether it saved and under what name.
                if (path == "/api/style_add") {
                    const auto get = [&query](const char* key) {
                        const auto it = query.find(key);
                        return it == query.end() ? std::string() : it->second;
                    };
                    std::string saved, error;
                    const bool ok = station.addStyle(get("name"), get("base"), get("steps"), saved, error);
                    vjson::Value out = vjson::Value::object();
                    out.set("ok", vjson::Value::boolean(ok));
                    if (ok) out.set("style", vjson::Value::str(saved));
                    else out.set("error", vjson::Value::str(error));
                    conn.sendResponse(ok ? 200 : 400, "application/json", out.dump());
                    return;
                }

                if (path != "/api/command") {
                    conn.sendResponse(404, "text/plain", "not found");
                    return;
                }
                std::string name, arg;
                const auto nameIt = query.find("name");
                if (nameIt != query.end()) name = nameIt->second;
                const auto argIt = query.find("arg");
                if (argIt != query.end()) arg = argIt->second;

                static const char* allowed[] = {
                    "calibrate",  "verify",     "clear",        "next",     "back",
                    "retake",     "next_shirt", "template",     "mark",     "mark_undo",
                    "mark_clear", "mark_measure", "trace",      "advance_now", "goto",
                    "dump",       "box",        "collar_pick",  "collar_clear",
                    "calibrate_cancel", "scale_check", "flag_manual_qc", "manual_steps",
                    "stats_new_shift", "advance_delay", "exposure_level"};
                bool known = false;
                for (const char* a : allowed) {
                    if (name == a) known = true;
                }
                if (!known) {
                    conn.sendResponse(400, "application/json", "{\"ok\":false}");
                    return;
                }
                // The one command that takes seconds rather than a frame, so
                // the page is told whether it was taken and follows the run in
                // the status from there.
                if (name == "calibrate") {
                    std::string why;
                    bool busy = false;
                    const bool ok = station.requestCalibration(why, busy);
                    vjson::Value out = vjson::Value::object();
                    out.set("ok", vjson::Value::boolean(ok));
                    if (!ok) {
                        out.set("error", vjson::Value::str(why));
                        out.set("busy", vjson::Value::boolean(busy));
                    }
                    conn.sendResponse(ok ? 200 : 400, "application/json", out.dump());
                    return;
                }
                // A new exposure from Settings: a metering level and the
                // recalibration that sets the camera for it, answered at once
                // like calibrate.
                if (name == "exposure_level") {
                    std::string why;
                    bool busy = false;
                    char* end = nullptr;
                    const double level = std::strtod(arg.c_str(), &end);
                    const bool ok = end != arg.c_str() && station.requestExposureLevel(level, why, busy);
                    if (end == arg.c_str()) why = "no level given";
                    vjson::Value out = vjson::Value::object();
                    out.set("ok", vjson::Value::boolean(ok));
                    if (!ok) {
                        out.set("error", vjson::Value::str(why));
                        out.set("busy", vjson::Value::boolean(busy));
                    }
                    conn.sendResponse(ok ? 200 : 400, "application/json", out.dump());
                    return;
                }
                // Not through the capture thread's one-slot mailbox, which a
                // tap arriving just before could take the place of: the worker
                // it is for only reads a flag.
                if (name == "calibrate_cancel") {
                    std::string why;
                    const bool ok = station.cancelCalibration(why);
                    vjson::Value out = vjson::Value::object();
                    out.set("ok", vjson::Value::boolean(ok));
                    if (!ok) out.set("error", vjson::Value::str(why));
                    conn.sendResponse(ok ? 200 : 400, "application/json", out.dump());
                    return;
                }
                // Only clears the figures, so it is done here and cannot be
                // lost in the one-slot mailbox behind a tap.
                if (name == "stats_new_shift") {
                    station.newShift();
                    station.log("New shift: the shift's figures start again");
                    conn.sendResponse(200, "application/json", "{\"ok\":true}");
                    return;
                }
                station.command(name, arg);
                conn.sendResponse(200, "application/json", "{\"ok\":true}");
                return;
            }

            conn.sendResponse(404, "text/plain", "not found");
        });

    if (!server.start()) {
        station.stop();
        return 1;
    }
    g_server = &server;
    std::signal(SIGINT, onInterrupt);

    const std::string shown =
        (args.host == "127.0.0.1" || args.host == "0.0.0.0") ? "localhost" : args.host;
    std::printf("\nStation running at http://%s:%d\n", shown.c_str(), args.port);
    if (args.host == "0.0.0.0") {
        std::printf("Reachable from other machines on the network.\n");
    }
    std::printf("Ctrl-C to stop.\n\n");
    std::fflush(stdout);

    server.serveForever();
    server.shutdown();
    station.stop();
    return 0;
}
