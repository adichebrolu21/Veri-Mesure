/*
Veri Measure - automated plane calibration, segmentation and metric measurement.

The operator no longer clicks grid intersections or types in a spacing. The
empty mat's printed grid is detected directly, the model is a plane homography
plus radial distortion rather than a scale factor, the distortion terms are only
kept if they earn their place against a holdout set, shadows are rejected by a
chromaticity test rather than a grayscale magnitude test, and every contour
point is refined to the sub-pixel peak of the intensity gradient along its own
local normal.

Accuracy is bounded by the printed mat, not by the software. A self-healing
cutting mat is a drawing aid, not a metrology artefact; print scale on these is
commonly off by half a percent to two percent and is not uniform. Everything
here gets you a model that is faithful to the mat. To be traceable to a real
millimetre you need one known artefact: a steel rule, a gauge block, a
calibrated coupon. Press 'v', measure it, and the engine stores a single global
scale correction. One number, one degree of freedom, no overfitting risk, and it
is the difference between repeatable and correct.

Controls
    'c'  recalibrate the plane model from the current empty surface
    'b'  recapture the background reference
    'v'  verify and correct global scale against a known artefact
    'r'  draw the ROI by hand instead of using the lattice footprint
    'm'  toggle the mask view
    'q'  quit
*/

#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/calibration.hpp"
#include "verimeasure/camera.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/engine.hpp"
#include "verimeasure/qc.hpp"
#include "verimeasure/edge.hpp"
#include "verimeasure/template.hpp"

using namespace verimeasure;

namespace {

struct Args {
    bool headless = false;
    bool recalibrate = false;
    bool manual_roi = false;
    std::string calibrate_image;
    std::string results;
    std::string template_path;
    int pin_step = 0;  // 1-based; zero means walk the list
    bool have_object_height = false;
    double object_height_mm = 0.0;
    bool have_focal_length = false;
    double focal_length_mm = 0.0;
    bool have_camera_distance = false;
    double camera_distance_mm = 0.0;
    bool have_exposure_target = false;
    double exposure_target = 0.0;
    bool fit_distortion = false;
    std::vector<std::string> distortion_frames;
    bool save = false;
    bool forget_distortion = false;
};

void printUsage() {
    std::printf(
        "usage: verimeasure_cli [--headless] [--recalibrate] [--manual-roi]\n"
        "                       [--calibrate-image PATH] [--results PATH]\n"
        "                       [--object-height-mm F] [--focal-length-mm F]\n"
        "                       [--template PATH] [--forget-distortion]\n"
        "       verimeasure_cli --fit-distortion PATH PATH... [--save]\n\n"
        "  --headless             no OpenCV windows, results to stdout as JSON Lines\n"
        "  --recalibrate          ignore any saved calibration and fit a new one\n"
        "  --manual-roi           draw the ROI by hand instead of using the lattice\n"
        "  --calibrate-image PATH calibrate from a still image and exit\n"
        "  --results PATH         append results to this JSON Lines file\n"
        "  --object-height-mm F   top-surface height above the plane, for parallax\n"
        "  --focal-length-mm F    the lens, for the fold thickness correction. Saved\n"
        "                         for this bench in bench.json, shared with the\n"
        "                         station, so it only needs giving once; 0 forgets it\n"
        "  --camera-distance-mm F lens-to-mat distance, measured; overrides the focal\n"
        "                         length. Saved in bench.json, shared with the\n"
        "                         station; 0 forgets it\n"
        "  --exposure-target N    the level the clear mat is metered to, default about\n"
        "                         140. Saved in bench.json, shared with the station;\n"
        "                         0 forgets it\n"
        "  --template PATH        the style to check. Each lay is taken as the next\n"
        "                         step in its list, wrapping to step 1 after the\n"
        "                         last, so a run of lays walks whole garments.\n"
        "  --step N               stay on step N instead of walking the list, for\n"
        "                         setting a rig up against one fold\n"
        "  --fit-distortion PATH PATH...\n"
        "                         measure the lens's distortion from several pictures\n"
        "                         of the rigid mat, the mat moved a few centimetres\n"
        "                         between them and the camera not moved at all, and\n"
        "                         say how far calibrations of them agree with and\n"
        "                         without it. With --save it is kept in bench.json and\n"
        "                         every recalibration fits the homography only\n"
        "  --forget-distortion    drop the saved distortion: recalibrations fit the\n"
        "                         radial terms afresh again\n");
}

bool parseArgs(int argc, char** argv, Args& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](double& out) {
            if (i + 1 >= argc) return false;
            out = std::strtod(argv[++i], nullptr);
            return true;
        };
        auto nextStr = [&](std::string& out) {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };

        if (a == "--headless") {
            args.headless = true;
        } else if (a == "--recalibrate") {
            args.recalibrate = true;
        } else if (a == "--manual-roi") {
            args.manual_roi = true;
        } else if (a == "--calibrate-image") {
            if (!nextStr(args.calibrate_image)) return false;
        } else if (a == "--results") {
            if (!nextStr(args.results)) return false;
        } else if (a == "--template") {
            if (!nextStr(args.template_path)) return false;
        } else if (a == "--step") {
            double v = 0.0;
            if (!next(v)) return false;
            args.pin_step = static_cast<int>(v);
        } else if (a == "--object-height-mm") {
            if (!next(args.object_height_mm)) return false;
            args.have_object_height = true;
        } else if (a == "--focal-length-mm") {
            if (!next(args.focal_length_mm)) return false;
            args.have_focal_length = true;
        } else if (a == "--camera-distance-mm") {
            if (!next(args.camera_distance_mm)) return false;
            args.have_camera_distance = true;
        } else if (a == "--exposure-target") {
            if (!next(args.exposure_target)) return false;
            args.have_exposure_target = true;
        } else if (a == "--fit-distortion") {
            args.fit_distortion = true;
            while (i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0) {
                args.distortion_frames.push_back(argv[++i]);
            }
        } else if (a == "--save") {
            args.save = true;
        } else if (a == "--forget-distortion") {
            args.forget_distortion = true;
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

void emitResult(const MeasureResult& result, const Config& cfg, const StepResult* step) {
    // One line per step, the verdict and the measurement it came from together.
    // Splitting them across two records would leave the audit trail needing a
    // join to answer "did this fold pass", which is the only question anyone
    // asks of it.
    vjson::Value record;
    if (step) {
        record = step->toJson();
        record.set("measurement", result.toJson());
    } else {
        record = result.toJson();
    }
    const std::string line = record.dump();
    if (!cfg.results_file.empty()) {
        std::ofstream out(cfg.results_file, std::ios::app);
        if (out) out << line << "\n";
    }
    std::printf("RESULT %s\n", line.c_str());
    std::fflush(stdout);
}

void printResult(const MeasureResult& r, const Config& cfg) {
    if (!r.calibrated) {
        std::printf("[#%d] %.0f x %.0f px (uncalibrated)\n", r.object_number, r.length_px,
                    r.breadth_px);
        std::fflush(stdout);
        return;
    }
    std::printf("\n[Object #%d]\n", r.object_number);
    std::printf("  Length      : %.3f mm\n", r.length_mm);
    std::printf("  Breadth     : %.3f mm\n", r.breadth_mm);
    std::printf("  Area        : %.1f mm^2\n", r.area_mm2);
    std::printf("  Angle       : %.2f deg\n", r.angle_deg);
    std::printf("  Settled     : area stable to %.2f%%, moved %.1f px over %d averaged frames\n",
                r.settled.area_spread_fraction * 100.0, r.settled.centroid_movement_px,
                r.frames_averaged);
    if (r.parallax_correction != 1.0) {
        std::printf("  Parallax    : x%.5f for %g mm object height\n", r.parallax_correction,
                    cfg.object_height_mm);
    }
    if (r.saturated_fraction > 0.005) {
        std::printf(
            "  WARNING: %.1f%% of the object is clipped white. Blooming inflates edges. Reduce "
            "exposure.\n",
            r.saturated_fraction * 100.0);
    }
    std::printf("\n");
    std::fflush(stdout);
}

void printStep(const StepResult& r) {
    std::printf("  Step        : %d of %d, %s\n", r.index + 1, r.total, r.step.c_str());
    if (r.measured) {
        // The reading is a length, so it stays in millimetres. How far off it
        // came out, and how far off it was allowed to be, are shares of the
        // target: the same words then mean the same thing on a shoulder as on
        // a chest, which a bare millimetre figure does not.
        std::printf("  Edge        : %.2f mm   target %.2f mm\n", r.measured_mm, r.target_mm);
        if (r.has_percent()) {
            std::printf("  Deviation   : %+.2f%%   allowed %+.2f%% / %+.2f%%\n",
                        r.deviation_pct(), r.tol_minus_pct(), r.tol_plus_pct());
        } else {
            std::printf("  Deviation   : %+.2f mm   allowed %+.2f / %+.2f mm\n", r.deviation_mm,
                        r.tol_minus_mm, r.tol_plus_mm);
        }
        std::printf("  Straight to : %.2f mm\n", r.straightness_mm);
    }
    std::printf("  Verdict     : %s", r.pass ? "PASS" : "FAIL");
    if (!r.reason.empty()) std::printf("   %s", r.reason.c_str());
    std::printf("\n\n");
    std::fflush(stdout);
}

// ---- Manual ROI selection ----

struct DragSelector {
    bool dragging = false;
    bool haveStart = false;
    cv::Point start;
    cv::Point end;

    static void callback(int event, int x, int y, int, void* param) {
        DragSelector* self = static_cast<DragSelector*>(param);
        if (event == cv::EVENT_LBUTTONDOWN) {
            self->dragging = true;
            self->haveStart = true;
            self->start = cv::Point(x, y);
            self->end = cv::Point(x, y);
        } else if (event == cv::EVENT_MOUSEMOVE && self->dragging) {
            self->end = cv::Point(x, y);
        } else if (event == cv::EVENT_LBUTTONUP) {
            self->dragging = false;
            self->end = cv::Point(x, y);
        }
    }

    bool box(double scale, RoiBox& out) const {
        if (!haveStart) return false;
        const int x1 = std::min(start.x, end.x);
        const int x2 = std::max(start.x, end.x);
        const int y1 = std::min(start.y, end.y);
        const int y2 = std::max(start.y, end.y);
        if (x2 - x1 < 5 || y2 - y1 < 5) return false;
        out = RoiBox{static_cast<int>(x1 / scale), static_cast<int>(y1 / scale),
                     static_cast<int>(x2 / scale), static_cast<int>(y2 / scale)};
        return true;
    }
};

bool selectRoiInteractive(FrameSource& stream, const Config& cfg, RoiBox& out) {
    const char* win = "Draw ROI: drag, then 's' to save, 'r' to redo";
    DragSelector sel;
    cv::namedWindow(win);
    cv::setMouseCallback(win, &DragSelector::callback, &sel);
    const double scale = cfg.display_scale;

    while (true) {
        cv::Mat frame;
        long long seq = 0;
        double age = 0.0;
        if (!stream.read(frame, seq, age, 2.0)) continue;

        cv::Mat disp;
        cv::resize(frame, disp, cv::Size(), scale, scale, cv::INTER_NEAREST);
        if (sel.haveStart) {
            cv::rectangle(disp, sel.start, sel.end, cv::Scalar(0, 255, 0), 2);
        }
        cv::putText(disp, "drag  |  's' save  'r' redo", cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        cv::imshow(win, disp);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'r') {
            sel.haveStart = false;
        } else if (key == 's') {
            if (sel.box(scale, out)) {
                cv::destroyWindow(win);
                return true;
            }
        }
    }
}

// Traceability check against a known artefact. The grid on a cutting mat is a
// drawing aid with a print tolerance, not a metrology standard, so this is the
// step that turns a repeatable number into a correct one. One global scale
// factor, one degree of freedom, applied to every axis equally.
void verifyScale(MeasurementEngine& engine, const Config& cfg) {
    const std::optional<MeasureResult>& live = engine.lastResult();
    if (!live.has_value() || !live->calibrated) {
        std::printf(
            "No completed measurement yet. Place the reference artefact on the surface, let it "
            "measure, then press 'v'. The live overlay figure is a coarse estimate and is not "
            "accurate enough to calibrate against.\n");
        std::fflush(stdout);
        return;
    }
    std::printf("\nLast completed measurement: %.3f x %.3f mm\n", live->length_mm,
                live->breadth_mm);
    std::printf("True length of the artefact's long side in mm (blank to cancel): ");
    std::fflush(stdout);

    std::string raw;
    if (!std::getline(std::cin, raw)) {
        std::printf("Cancelled.\n");
        std::fflush(stdout);
        return;
    }
    bool blank = true;
    for (char c : raw) {
        if (!std::isspace(static_cast<unsigned char>(c))) blank = false;
    }
    if (blank) return;

    char* endp = nullptr;
    const double trueMm = std::strtod(raw.c_str(), &endp);
    if (endp == raw.c_str() || trueMm <= 0.0) {
        std::printf("Cancelled.\n");
        std::fflush(stdout);
        return;
    }

    const double factor = trueMm / live->length_mm;
    if (!(factor > 0.9 && factor < 1.1)) {
        std::printf(
            "Correction of %.4f is more than 10 percent. That is a calibration or detection "
            "fault, not a scale error. Not applying it.\n",
            factor);
        std::fflush(stdout);
        return;
    }

    PlaneModelPtr model = engine.model();
    model->setScaleCorrection(model->scaleCorrection() * factor);
    model->meta().set("scale_verified_against_mm", vjson::Value::real(trueMm));
    model->meta().set("scale_verified_at", vjson::Value::str(timestampIso()));
    saveCalibration(*model, cfg);
    std::printf("Scale correction now %.6f (%+.3f%% applied).\n", model->scaleCorrection(),
                (factor - 1.0) * 100.0);
    std::fflush(stdout);
}

// Calibrate from a saved still. Useful for validating a setup without the
// camera attached, and for the frontend team to test against fixtures.
int runOffline(const std::string& imagePath, const Config& cfg) {
    const cv::Mat frame = cv::imread(imagePath);
    if (frame.empty()) {
        std::printf("Could not read %s\n", imagePath.c_str());
        return 1;
    }
    PlaneModelPtr model = autoCalibrate(frame, cfg);
    if (!model) return 1;
    saveCalibration(*model, cfg);
    return 0;
}

// Measure the lens's distortion across several pictures of the rigid mat and,
// with --save, keep it for the bench. See fitBenchDistortion.
int runFitDistortion(const std::vector<std::string>& paths, bool save, const Config& cfg) {
    if (paths.size() < 2) {
        std::printf("--fit-distortion needs two or more pictures of the rigid mat: the camera not "
                    "moved, the mat moved a few centimetres between them. Five to eight is right.\n");
        return 2;
    }
    std::vector<cv::Mat> frames;
    for (const std::string& p : paths) {
        frames.push_back(cv::imread(p, cv::IMREAD_COLOR));
        if (frames.back().empty()) {
            std::printf("Could not read %s\n", p.c_str());
            return 1;
        }
    }
    DistortionFit fit;
    const bool ok = fitBenchDistortion(frames, paths, cfg, fit);

    std::printf("\n=== LENS DISTORTION FROM %zu PICTURES ===\n", frames.size());
    int moved = 0;
    for (std::size_t k = 0; k < fit.frames.size(); ++k) {
        const DistortionFitFrame& f = fit.frames[k];
        if (!f.error.empty()) {
            std::printf("  %s\n    not used: %s\n", f.name.c_str(), f.error.c_str());
            continue;
        }
        if (k > 0 && f.shift_px > 20.0) ++moved;
        std::printf("  %s\n    %d of %d points; shared fit %.3f mm RMSE, worst %.3f mm; on its own "
                    "%.3f mm, k1 %+.5f k2 %+.5f (%s); mat %.0f px from the first picture\n",
                    f.name.c_str(), f.points_used, f.points_detected, f.rmse_mm, f.max_mm,
                    f.alone_rmse_mm, f.alone_k1, f.alone_k2,
                    f.alone_distortion ? "kept" : "rejected", f.shift_px);
    }
    if (!ok) {
        std::printf("  FAILED: %s\n", fit.error.c_str());
        return 1;
    }
    std::printf("  shared distortion   : k1 %+.6f, k2 %+.6f about (%.1f, %.1f), radius %.1f px\n",
                fit.lens.k1, fit.lens.k2, fit.lens.cx, fit.lens.cy, fit.lens.norm);
    std::printf("  shared fit          : %d points, %.4f mm RMSE\n", fit.points, fit.rmse_mm);
    std::printf("  how far calibrations of these pictures put the same pixels apart, worst pair,\n"
                "  mm on the mat, by distance from the centre as a share of half the width:\n"
                "                          < 0.5  0.5-0.7  0.7-0.85  edges  corners\n");
    const auto row = [](const char* what, const GeometryAgreement& g) {
        std::printf("    %-20s %6.2f  %7.2f  %8.2f  %5.2f  %7.2f\n", what, g.centre_mm, g.mid_mm,
                    g.outer_mm, g.edge_mm, g.corner_mm);
    };
    row("each on its own", fit.alone);
    row("distortion shared", fit.locked);
    if (moved == 0) {
        std::printf("  NOTE: the mat does not seem to have moved between these pictures. A misprinted\n"
                    "  square then sits in the same place in every picture and is fitted into the\n"
                    "  lens. Move the mat a few centimetres between pictures, camera untouched.\n");
    }
    if (!save) {
        std::printf("Not saved. Run again with --save to keep it for this bench.\n");
        return 0;
    }
    if (!saveBenchDistortion(cfg, fit.lens, fit.rmse_mm, fit.points)) {
        std::printf("Could not write %s\n", cfg.bench_file.c_str());
        return 1;
    }
    std::printf("Saved to %s. Every recalibration from now on fits the homography only, on the\n"
                "station too; --forget-distortion drops it. Recalibrate on the rigid mat now.\n",
                cfg.bench_file.c_str());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) return 2;

    Config cfg;
    if (!args.results.empty()) cfg.results_file = args.results;
    if (!args.template_path.empty()) cfg.template_file = args.template_path;
    if (args.have_object_height) cfg.object_height_mm = args.object_height_mm;
    // Given once and remembered; see settleLens.
    const std::string lensLine = settleLens(cfg, args.have_focal_length, args.focal_length_mm);
    if (!lensLine.empty()) {
        std::printf("%s\n", lensLine.c_str());
        std::fflush(stdout);
    }
    // Measured beats derived: see the note in the station on why. Given once
    // and remembered, like the lens; see settleDistance.
    const std::string distanceLine =
        settleDistance(cfg, args.have_camera_distance, args.camera_distance_mm);
    if (!distanceLine.empty()) {
        std::printf("%s\n", distanceLine.c_str());
        std::fflush(stdout);
    }
    const std::string targetLine =
        settleExposureTarget(cfg, args.have_exposure_target, args.exposure_target);
    if (!targetLine.empty()) {
        std::printf("%s\n", targetLine.c_str());
        std::fflush(stdout);
    }
    // The lens's distortion, measured once for the bench; see settleDistortion.
    const std::string distortionLine = settleDistortion(cfg, args.forget_distortion);
    if (!distortionLine.empty()) {
        std::printf("%s\n", distortionLine.c_str());
        std::fflush(stdout);
    }
    if (args.fit_distortion) return runFitDistortion(args.distortion_frames, args.save, cfg);

    Template activeTemplate;
    bool haveTemplate = false;
    int stepIndex = 0;
    int garment = 1;
    if (!cfg.template_file.empty()) {
        std::string error;
        if (!loadTemplate(cfg.template_file, activeTemplate, error)) {
            std::printf("%s\n", error.c_str());
            return 1;
        }
        haveTemplate = true;
        std::printf("Checking \"%s\": %zu steps per garment\n", activeTemplate.style.c_str(),
                    activeTemplate.steps.size());
        for (std::size_t k = 0; k < activeTemplate.steps.size(); ++k) {
            const Step& st = activeTemplate.steps[k];
            std::printf("  %zu. %-22s %8.1f mm  %+.1f / %+.1f\n", k + 1, st.name.c_str(),
                        st.target_mm, st.tol_minus_mm, st.tol_plus_mm);
        }
        std::fflush(stdout);
    }

    if (haveTemplate && args.pin_step > 0) {
        if (args.pin_step > static_cast<int>(activeTemplate.steps.size())) {
            std::printf("--step %d is past the end of this style\n", args.pin_step);
            return 1;
        }
        stepIndex = args.pin_step - 1;
        std::printf("Staying on step %d, %s\n", args.pin_step,
                    activeTemplate.steps[static_cast<std::size_t>(stepIndex)].name.c_str());
    }

    if (!args.calibrate_image.empty()) {
        return runOffline(args.calibrate_image, cfg);
    }

    if (!ARENA_AVAILABLE) {
        std::printf(
            "This build has no camera backend, so there is nothing to measure. Rebuild with\n"
            "  -DVERIMEASURE_WITH_ARENA=ON -DARENA_SDK_ROOT=<path to the Arena SDK>\n"
            "and run it on the machine wired to the camera. Every measurement comes off the\n"
            "camera; there is no substitute source. To work from a saved capture instead, use\n"
            "--calibrate-image.\n");
        return 1;
    }

    Camera device;
    CameraStream* stream = nullptr;
    bool showMask = false;
    int exitCode = 0;

    try {
        device.connect();
        device.configureStream();

        std::printf("\nLocking camera response:\n");
        disableCameraAutoAdjustments(device);

        std::printf("Tuning the stream for live viewing:\n");
        tuneStream(device, cfg);

        device.startStream(cfg.stream_buffers);
        static CameraStream cameraStream(device, cfg);
        stream = &cameraStream.start();

        cv::Mat first;
        long long seq = 0;
        double age = 0.0;
        if (!stream->read(first, seq, age, 15.0)) {
            throw std::runtime_error("no frames arrived after starting the stream");
        }
        reportBandwidth(first.size(), first.channels(), cfg);

        if (cfg.auto_expose_on_start) {
            std::printf("\nSetting exposure:\n");
            double locked = 0.0;
            autoSetExposure(device, *stream, cfg, locked);
        }

        cv::Mat backgroundFull = captureStaticBackground(*stream, cfg);

        PlaneModelPtr model = args.recalibrate ? nullptr : loadCalibration(cfg);
        if (!model) {
            model = autoCalibrate(backgroundFull, cfg);
            if (model) saveCalibration(*model, cfg);
        }
        if (!model) {
            std::printf(
                "Running uncalibrated. Output will be in pixels until the grid can be detected. "
                "Press 'c' to retry once the surface is clear.\n");
            std::fflush(stdout);
        }

        if (!cfg.camera_distance_mm.has_value() && model) {
            double d = 0.0;
            bool derived = false;
            if (estimateCameraDistance(*model, backgroundFull.size(), cfg, d, &derived) &&
                d != 0.0) {
                cfg.camera_distance_mm = d;
                // It used to say "from the lens focal length" either way, which
                // was the one thing it must not claim when there was no lens.
                if (derived) {
                    std::printf(
                        "Working distance %.0f mm, GUESSED from the calibration - check it. Pass "
                        "--focal-length-mm once with the figure on the lens and it is remembered.\n",
                        d);
                } else {
                    std::printf("Working distance %.0f mm, from the %.1f mm lens.\n", d,
                                cfg.lens_focal_length_mm.value_or(0.0));
                }
                std::fflush(stdout);
            }
        }

        RoiBox roiBox;
        bool haveRoi = false;
        if (args.manual_roi) {
            haveRoi = selectRoiInteractive(*stream, cfg, roiBox);
            if (haveRoi) saveRoi(roiBox, cfg);
        } else {
            bool roiFollowedMat = false;
            if (model) {
                haveRoi = roiFromModel(*model, backgroundFull.size(), cfg, roiBox, &backgroundFull,
                                       &roiFollowedMat);
            }
            if (!haveRoi) haveRoi = loadRoi(cfg, roiBox);
            if (!haveRoi) {
                roiBox = RoiBox{0, 0, backgroundFull.cols, backgroundFull.rows};
                haveRoi = true;
            }
            saveRoi(roiBox, cfg);
            std::printf("ROI taken from the %s: (%d, %d, %d, %d)\n",
                        roiFollowedMat ? "edge of the mat" : "calibrated footprint", roiBox.x1,
                        roiBox.y1, roiBox.x2, roiBox.y2);
            std::fflush(stdout);
        }

        MeasurementEngine engine(cfg, model, roiBox, haveRoi);
        engine.setBackground(backgroundFull);

        std::printf(
            "\nRunning. 'c' recalibrate  'b' background  'v' verify scale  'r' manual ROI  'm' "
            "mask  'q' quit\n\n");
        std::fflush(stdout);

        const double scale = cfg.display_scale;
        double procMs = 0.0;
        long long lastSeq = -1;
        double stalledSince = 0.0;
        bool haveStalled = false;

        while (true) {
            cv::Mat frame;
            long long frameSeq = 0;
            double frameAge = 0.0;
            const bool got = stream->read(frame, frameSeq, frameAge, 2.0);

            if (!got || frameSeq == lastSeq) {
                // No new frame yet. Keep the window responsive and keep watching
                // for a link that has gone quiet, rather than blocking on a grab.
                if (!got) {
                    const double now = nowSeconds();
                    if (!haveStalled) {
                        haveStalled = true;
                        stalledSince = now;
                    }
                    if (now - stalledSince > 15.0) {
                        const std::string err = stream->error();
                        std::printf("[%s] no frames for 15 s (%s)\n", timestampClock().c_str(),
                                    err.empty() ? "unknown cause" : err.c_str());
                        std::fflush(stdout);
                        stalledSince = now;
                    }
                }
                if (!args.headless) {
                    if ((cv::waitKey(5) & 0xFF) == 'q') break;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                continue;
            }

            haveStalled = false;
            lastSeq = frameSeq;

            const double t0 = nowSeconds();
            const std::optional<MeasureResult> result = engine.process(frame, frameSeq);
            procMs = 0.8 * procMs + 0.2 * (nowSeconds() - t0) * 1000.0;

            if (result.has_value()) {
                printResult(*result, cfg);
                StepResult step;
                if (haveTemplate) {
                    const PlaneModelPtr model = engine.model();
                    cv::Point2d scan(1.0, 0.0);
                    if (model) scanDirection(engine.roi(), *model, cfg.scan_from, scan);
                    step = runStepQc(*result, model.get(), scan, activeTemplate, stepIndex, cfg,
                                     engine.hasRoi() ? &engine.roi() : nullptr);
                    step.garment = garment;
                    printStep(step);

                    // Without buttons, the only sensible reading of a lay is
                    // that it is the next step, so a run of lays walks whole
                    // garments. --step pins one fold for setting a rig up.
                    if (args.pin_step == 0) {
                        if (++stepIndex >= static_cast<int>(activeTemplate.steps.size())) {
                            stepIndex = 0;
                            ++garment;
                        }
                    }
                }
                emitResult(*result, cfg, haveTemplate ? &step : nullptr);
            }

            if (args.headless) continue;

            cv::Mat disp;
            cv::resize(frame, disp, cv::Size(), scale, scale, cv::INTER_NEAREST);
            const RoiBox& roi = engine.roi();
            const DebugState& dbg = engine.lastDebug();
            const cv::Scalar colour =
                dbg.live.has_value() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            if (dbg.has_contour) {
                std::vector<cv::Point> pts(dbg.contour.size());
                for (std::size_t k = 0; k < dbg.contour.size(); ++k) {
                    pts[k] = cv::Point(
                        static_cast<int>((dbg.contour[k].x * dbg.step + roi.x1) * scale),
                        static_cast<int>((dbg.contour[k].y * dbg.step + roi.y1) * scale));
                }
                cv::polylines(disp, pts, true, cv::Scalar(0, 255, 0), 2);
            }

            char line[160];
            if (dbg.live.has_value() && dbg.live->calibrated) {
                std::snprintf(line, sizeof(line), "~%.1f x %.1f mm (live estimate)",
                              dbg.live->length_mm, dbg.live->breadth_mm);
            } else if (dbg.live.has_value()) {
                std::snprintf(line, sizeof(line), "~%.0f x %.0f px uncalibrated",
                              dbg.live->length_px, dbg.live->breadth_px);
            } else {
                std::snprintf(line, sizeof(line), "no object");
            }

            std::vector<std::string> lines;
            char buf[192];
            std::snprintf(buf, sizeof(buf), "%s   measured: %d", engine.state().c_str(),
                          engine.count());
            lines.emplace_back(buf);
            lines.emplace_back(line);

            const std::optional<MeasureResult>& last = engine.lastResult();
            if (last.has_value() && last->calibrated) {
                std::snprintf(buf, sizeof(buf), "LAST  %.3f x %.3f mm", last->length_mm,
                              last->breadth_mm);
            } else {
                std::snprintf(buf, sizeof(buf), "LAST  none yet");
            }
            lines.emplace_back(buf);

            std::snprintf(buf, sizeof(buf), "cam %4.1f fps   lag %4.0f ms   proc %4.0f ms",
                          stream->fps(), frameAge * 1000.0, procMs);
            lines.emplace_back(buf);

            if (engine.model()) {
                std::snprintf(buf, sizeof(buf), "model RMSE %.3f mm   scale x%.5f",
                              engine.model()->metaNumber("rmse_mm", 0.0),
                              engine.model()->scaleCorrection());
            } else {
                std::snprintf(buf, sizeof(buf), "uncalibrated");
            }
            lines.emplace_back(buf);

            std::snprintf(buf, sizeof(buf), "changed %.1f%%   gain x%.3f   dropped %lld",
                          dbg.changed_fraction * 100.0, dbg.gain, stream->incompleteCount());
            lines.emplace_back(buf);

            cv::rectangle(disp,
                          cv::Point(static_cast<int>(roi.x1 * scale),
                                    static_cast<int>(roi.y1 * scale)),
                          cv::Point(static_cast<int>(roi.x2 * scale),
                                    static_cast<int>(roi.y2 * scale)),
                          colour, 1);
            for (std::size_t i = 0; i < lines.size(); ++i) {
                cv::putText(disp, lines[i], cv::Point(10, 24 + static_cast<int>(i) * 21),
                            cv::FONT_HERSHEY_SIMPLEX, 0.48, colour, 2);
            }

            cv::imshow("Veri Measure", disp);

            if (showMask && !dbg.mask.empty()) {
                cv::Mat maskDisp;
                cv::resize(dbg.mask, maskDisp, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
                cv::imshow("Mask", maskDisp);
            }

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') {
                break;
            } else if (key == 'm') {
                showMask = !showMask;
                if (!showMask) {
                    try {
                        cv::destroyWindow("Mask");
                    } catch (const cv::Exception&) {
                    }
                }
            } else if (key == 'b') {
                backgroundFull = captureStaticBackground(*stream, cfg);
                engine.setBackground(backgroundFull);
                std::printf("Background recaptured.\n");
                std::fflush(stdout);
            } else if (key == 'c') {
                backgroundFull = captureStaticBackground(*stream, cfg);
                // A grid printed dark is the station's working base, the blue
                // cloth, not a ruler: as on the station, it gives the
                // background and the geometry in use is kept.
                const PlaneModelPtr current = engine.model();
                if (current && !cfg.dark_grid_sets_geometry &&
                    gridPrintedDark(backgroundFull, cfg)) {
                    RoiBox newRoi;
                    if (roiFromModel(*current, backgroundFull.size(), cfg, newRoi,
                                     &backgroundFull)) {
                        engine.setRoi(newRoi);
                        saveRoi(newRoi, cfg);
                    }
                    engine.setBackground(backgroundFull);
                    std::printf("The grid is printed dark - a working base, not a ruler - so the "
                                "geometry in use is kept and only the background was retaken.\n");
                    std::fflush(stdout);
                    continue;
                }
                PlaneModelPtr newModel = autoCalibrate(backgroundFull, cfg);
                if (newModel) {
                    saveCalibration(*newModel, cfg);
                    engine.setModel(newModel);
                    RoiBox newRoi;
                    if (roiFromModel(*newModel, backgroundFull.size(), cfg, newRoi,
                                     &backgroundFull)) {
                        engine.setRoi(newRoi);
                        saveRoi(newRoi, cfg);
                    }
                    engine.setBackground(backgroundFull);
                }
            } else if (key == 'v') {
                verifyScale(engine, cfg);
            } else if (key == 'r') {
                RoiBox newRoi;
                if (selectRoiInteractive(*stream, cfg, newRoi)) {
                    engine.setRoi(newRoi);
                    saveRoi(newRoi, cfg);
                    backgroundFull = captureBackground(*stream, cfg);
                    engine.setBackground(backgroundFull);
                }
            }
        }
    } catch (const std::exception& e) {
        std::printf("Camera error: %s\n", e.what());
        std::fflush(stdout);
        exitCode = 1;
    }

    if (stream != nullptr) {
        stream->stop();
        std::printf("Frames grabbed: %lld, incomplete: %lld, stream restarts: %lld\n",
                    stream->grabbedCount(), stream->incompleteCount(), stream->restartCount());
    }
    device.destroy();
    cv::destroyAllWindows();
    std::printf("Disconnected.\n");
    std::fflush(stdout);
    return exitCode;
}
