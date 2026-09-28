// Automatic plane calibration from the printed grid on the empty measurement
// surface, plus persistence of the fitted model and the ROI.

#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "verimeasure/config.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

struct RoiBox {
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;

    int width() const { return x2 - x1; }
    int height() const { return y2 - y1; }
    bool valid() const { return x2 > x1 && y2 > y1; }
    cv::Rect toRect() const { return cv::Rect(x1, y1, x2 - x1, y2 - y1); }
};

struct LatticeInfo {
    std::string error;  // empty when detection succeeded

    double occupied_fraction = 0.0;
    double phi_h = 0.0;
    double phi_v = 90.0;
    int h_lines = 0;
    int v_lines = 0;
    double pitch_px = 0.0;
    double pitch_px_x = 0.0;
    double pitch_px_y = 0.0;
    double step_mm_x = 10.0;  // x direction
    double step_mm_y = 10.0;  // y direction
    int pitch_multiple_x = 1;
    int pitch_multiple_y = 1;
    int lattice_points = 0;

    // Set when the lines could only be indexed by starting the walk from the
    // printed pitch rather than from the median gap; the first attempt's
    // reason for refusing the frame is kept alongside.
    bool pitch_seeded = false;
    std::string first_attempt_error;

    // The grid printed dark on a light ground rather than bright on a dark
    // one, and the evidence: how much deeper the grid's narrow dips run than
    // its narrow peaks rise, across the middle of the picture. See
    // printedDarkLines.
    bool dark_lines = false;
    double dip_to_peak = 0.0;
};

struct FitReport {
    std::string error;
    double rmse_mm = 0.0;
    double max_residual_mm = 0.0;
    double holdout_plain_mm = 0.0;
    double holdout_distortion_mm = 0.0;
    bool has_distortion_holdout = false;
    bool distortion_used = false;
    double gain = 0.0;
    int points_used = 0;
    int points_detected = 0;
};

// Detect the printed grid on the empty measurement surface and return matched
// pixel and millimetre correspondences. Returns false on failure, with the
// reason in info.error.
bool detectLattice(const cv::Mat& bgr, const Config& cfg, std::vector<cv::Point2d>& pointsPx,
                   std::vector<cv::Point2d>& pointsMm, LatticeInfo& info, bool verbose = true);

// Only the first pass of that: which way round the grid is printed and its
// pitch on each axis, without finding, indexing or fitting a single line. For
// a recalibration that keeps its geometry and only needs to know what base it
// is looking at, and whether it has moved. False, with info.error, when no
// pitch could be found.
bool readGridPitch(const cv::Mat& bgr, const Config& cfg, LatticeInfo& info);

// Just which way round: true for a grid printed dark on a light ground. The
// test the first pass starts with, a few tens of milliseconds on its own.
bool gridPrintedDark(const cv::Mat& bgr, const Config& cfg, double* dipToPeak = nullptr);

// Fit the homography, then test whether the two radial terms actually earn
// their place. The split is deterministic so a rerun on the same data gives the
// same verdict.
//
// With the bench's lens distortion in cfg.lens_distortion, and the picture the
// size it was fitted on, the radial terms are taken from it and only the
// homography is fitted; nothing is held out, and the model's meta says
// "distortion_source": "bench".
PlaneModelPtr fitPlaneModel(const std::vector<cv::Point2d>& pointsPx,
                            const std::vector<cv::Point2d>& pointsMm, cv::Size imageSize,
                            const Config& cfg, bool verbose, double pitchPx, double gridStepMm,
                            FitReport& report);

// Detection plus fitting, with the console reporting the Python runner printed.
// Returns null on failure, and when `error` is given, says why in it: the
// console is not where an operator at a tablet is looking. `lattice`, when
// given, gets what detection found - which way round the grid is printed and
// its pitch - whether or not the fit was accepted.
PlaneModelPtr autoCalibrate(const cv::Mat& frame, const Config& cfg, bool verbose = true,
                            std::string* error = nullptr, LatticeInfo* lattice = nullptr);

// How much of the working area hides the printed grid, from 0 to 1: the share
// of squares two pitches across in which the lines do not stand out from the
// local level by cfg.base_grid_min_contrast. A clear base hides none of it, a
// garment most of what it covers. Read either way round, so it needs no idea
// of how the base is printed, and relative to the local level, so it holds at
// any exposure. Counts only squares inside `area` when one is given. Returns a
// negative number when it cannot say: no frame, or no pitch to size the
// squares by.
double gridHiddenFraction(const cv::Mat& bgr, double pitchPx, const Config& cfg,
                          const RoiBox* area = nullptr);

// -- persistence --
PlaneModelPtr loadCalibration(const Config& cfg);
void saveCalibration(const PlaneModel& model, const Config& cfg);
bool loadRoi(const Config& cfg, RoiBox& roi);
void saveRoi(const RoiBox& roi, const Config& cfg);

// The extent of the mat in the frame, found by growing out from the printed
// grid through everything that shares the mat's colour.
//
// `seed` is the lattice footprint: pixel bounds known to be on the mat, which
// is both where the mat's own colour is sampled from and what the result has to
// contain to be believed. `pitchPx` sets how wide a printed marking may be and
// still be bridged over. Returns false when no region containing the seed comes
// out of it, which is the honest answer on a mat whose border the camera cannot
// see; the caller then keeps whatever the grid gave it.
bool matFootprint(const cv::Mat& bgr, const cv::Rect& seed, double pitchPx, const Config& cfg,
                  cv::Rect& out);

// Derive the working area from the calibrated footprint, reaching
// cfg.roi_extrapolation_pitches past it and stopping
// cfg.roi_frame_margin_px short of the border of the picture.
//
// Give `background` a clear frame of the surface and, with cfg.roi_follow_mat
// set, the box also reaches out to the edge of the mat wherever the mat reaches
// further than that - capped at cfg.roi_mat_max_pitches past the lattice, since
// past there the plane model is extrapolating further than it has evidence for.
// `fromMat` reports whether the mat actually moved anything, for a caller that
// tells an operator where the rail came from.
bool roiFromModel(const PlaneModel& model, cv::Size frameSize, const Config& cfg, RoiBox& roi,
                  const cv::Mat* background = nullptr, bool* fromMat = nullptr);

// Working distance: how far the lens is above the mat.
//
// From the lens focal length when one has been given, and otherwise recovered
// from the homography itself, which carries enough of the camera's geometry to
// say what focal length took the picture. `derived` reports which of the two
// happened, because an estimate the operator can check against a tape measure
// should say that it is one.
//
// Used for the parallax correction, which is the difference between measuring
// the top face of a folded garment and measuring the mat it is lying on.
bool estimateCameraDistance(const PlaneModel& model, cv::Size frameSize, const Config& cfg,
                            double& distanceMm, bool* derived = nullptr);

// Which lens this run uses, settled once at startup and remembered between runs
// in cfg.bench_file.
//
// Given on the command line (`given`), it is used and saved for next time, and
// zero forgets the saved one. Not given, the saved one is used. With neither,
// cfg is left with no lens and the working distance is guessed from the
// calibration, which is the last resort estimateCameraDistance explains.
//
// Returns one line saying which of those happened, for the log, or nothing when
// there is nothing to say.
std::string settleLens(Config& cfg, bool given, double givenMm);

// The lens's radial distortion, settled at startup the way the lens is: the
// one saved in cfg.bench_file goes into cfg.lens_distortion, and `forget`
// drops it from the file so recalibrations fit the radial terms afresh again.
// Returns a line for the log, or nothing.
std::string settleDistortion(Config& cfg, bool forget);
bool loadBenchDistortion(const Config& cfg, LensDistortion& out);
// Keeps everything else in the file. `rmseMm` and `points` are the joint fit's,
// kept beside it for whoever reads the file.
bool saveBenchDistortion(const Config& cfg, const LensDistortion& d, double rmseMm, int points);

// How far apart several calibrations of one camera put the same pixels, once
// the rigid move between their frames of reference is taken out - the mat may
// have been moved between the pictures, and the grid's origin is wherever the
// indexing started. The move is fitted over the middle of the picture, where
// calibrations agree best, so what is left shows where they do not. Worst over
// every pair, in millimetres on the mat, by distance from the centre of the
// picture as a share of half its width.
struct GeometryAgreement {
    int models = 0;
    double centre_mm = 0.0;  // under 0.5
    double mid_mm = 0.0;     // 0.5 to 0.7
    double outer_mm = 0.0;   // 0.7 to 0.85
    double edge_mm = 0.0;    // beyond 0.85, corners apart
    double corner_mm = 0.0;  // within 5% of two sides
};
GeometryAgreement compareGeometry(const std::vector<PlaneModelPtr>& models, cv::Size frame);

// One radial distortion shared by several pictures of the rigid mat and a
// homography for each: the lens measured once, over far more of the picture's
// edges than any one lattice reaches, with the mat moved between the pictures
// so no one misprinted square is always in the same place. The camera must not
// move. Each picture is also calibrated on its own, as a recalibration would
// calibrate it, and again with the shared distortion, so what locking it buys
// is there to read.
struct DistortionFitFrame {
    std::string name;
    std::string error;       // why the picture was not used; empty when it was
    int points_detected = 0;
    int points_used = 0;     // in the shared fit
    double rmse_mm = 0.0;    // under the shared distortion
    double max_mm = 0.0;
    double alone_rmse_mm = 0.0;  // calibrated on its own
    bool alone_distortion = false;
    double alone_k1 = 0.0;
    double alone_k2 = 0.0;
    double shift_px = 0.0;   // how far the mat moved from the first picture, if it could be told
};
struct DistortionFit {
    bool ok = false;
    std::string error;
    LensDistortion lens;
    double rmse_mm = 0.0;
    int points = 0;
    std::vector<DistortionFitFrame> frames;
    GeometryAgreement alone;   // each picture calibrated on its own, as today
    GeometryAgreement locked;  // each picture's homography under the shared distortion
};
bool fitBenchDistortion(const std::vector<cv::Mat>& frames, const std::vector<std::string>& names,
                        const Config& cfg, DistortionFit& out);

// How differently two calibrations of the same view measure the same pixel
// spans - across the middle both ways, and along each edge - as a share of what
// `before` reads. On one unmoved camera and one rigid mat two calibrations agree
// to a few tenths of a per cent; a different surface, or one whose print is not
// true to size, shows as more.
struct CalibrationChange {
    bool valid = false;
    double centre_pct = 0.0;  // the two spans through the middle, averaged
    double low_pct = 0.0;     // the most negative of all six spans
    double high_pct = 0.0;    // the most positive
};
CalibrationChange compareCalibrations(const PlaneModel& before, const PlaneModel& after, cv::Size frame);

// The measured distance from the camera to the surface, settled the same way
// as the lens and kept in the same file: --camera-distance-mm saves it, 0
// forgets it, and without the flag the saved one is used. A measured distance
// is preferred over one worked out from the lens, and a recalibration keeps it.
std::string settleDistance(Config& cfg, bool given, double givenMm);

// The exposure this bench runs at, remembered in cfg.bench_file.
//
// Set once from a clear mat and then used on every start as it is, because a
// picture metered afresh on each start comes out as bright as whatever was
// under the camera at that moment - a shirt, a hand, the light not yet on -
// and a background, a threshold and a clipped-white warning are only
// comparable from one run to the next at the same exposure. Recalibrate, which
// needs the mat clear anyway, sets it again; one given by hand is left alone.
struct BenchExposure {
    double us = 0.0;       // zero: none remembered
    double level = 0.0;    // the highlight level it gave when it was set, zero if not known
    bool byHand = false;   // given with --exposure-us rather than set from the mat
    std::string savedAt;
};
bool loadBenchExposure(const Config& cfg, BenchExposure& out);
// Keeps everything else in the file, the lens included.
bool saveBenchExposure(const Config& cfg, const BenchExposure& e);
bool forgetBenchExposure(const Config& cfg);
// --exposure-us, the same way settleLens takes --focal-length-mm: a positive
// figure is saved as set by hand, zero forgets the saved one so it is set from
// the mat on this start. Returns a line for the log, or nothing.
std::string settleExposure(const Config& cfg, bool given, double givenUs);
// --exposure-target, the level the clear base is metered to, taken the way
// settleLens takes the lens: given, it is used and saved; not given, the saved
// one is used; with neither, cfg is left as it is (about 140). Zero forgets
// the saved one. Returns a line for the log, or nothing.
std::string settleExposureTarget(Config& cfg, bool given, double level);
// An exposure the way a person reads one: "23.5 ms", "850 us".
std::string exposureText(double us);

// "%Y-%m-%dT%H:%M:%S" in local time.
std::string timestampIso();
// "%H:%M:%S" in local time.
std::string timestampClock();

}  // namespace verimeasure
