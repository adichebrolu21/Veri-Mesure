#include "verimeasure/engine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <utility>

#include "verimeasure/cv_compat.hpp"

#include "verimeasure/creep.hpp"
#include "verimeasure/numeric.hpp"
#include "verimeasure/segmentation.hpp"

namespace verimeasure {

const char* const STATE_WAITING = "WAITING";
const char* const STATE_STABILISING = "STABILISING";
const char* const STATE_CAPTURING = "CAPTURING";
const char* const STATE_MEASURED = "MEASURED_WAIT_FOR_CLEAR";

double nowSeconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

namespace {

std::vector<cv::Point2f> toPoint2f(const std::vector<cv::Point2d>& pts) {
    std::vector<cv::Point2f> out(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) {
        out[k] = cv::Point2f(static_cast<float>(pts[k].x), static_cast<float>(pts[k].y));
    }
    return out;
}

}  // namespace

MeasurementEngine::MeasurementEngine(const Config& cfg, PlaneModelPtr model, const RoiBox& roi,
                                     bool hasRoi)
    : cfg_(cfg), model_(std::move(model)), roi_(roi), hasRoi_(hasRoi), state_(STATE_WAITING) {
    step_ = std::max(1, cfg_.detect_step);
    stateSince_ = nowSeconds();
    updateBridge();
}

// Area-average, not every fourth pixel.
//
// Plain stride sampling throws away three quarters of the evidence and keeps
// whatever happens to land under the sample points. On a mat covered in thin
// high-contrast printed lines, half a pixel of camera shift changes which
// samples sit on a line, and the difference lights up along every line on the
// surface. Averaging each block instead is stable under sub-pixel movement, and
// at these frame rates the extra cost is nothing.
cv::Mat MeasurementEngine::decimate(const cv::Mat& img) const {
    if (step_ <= 1) return img;
    cv::Mat out;
    cv::resize(img, out, cv::Size(img.cols / step_, img.rows / step_), 0, 0, cv::INTER_AREA);
    return out;
}

cv::Mat MeasurementEngine::crop(const cv::Mat& frame) const {
    if (!hasRoi_) return frame;
    const cv::Rect r = roi_.toRect() & cv::Rect(0, 0, frame.cols, frame.rows);
    return frame(r);
}

void MeasurementEngine::setBackground(const cv::Mat& frame) {
    background_ = crop(frame).clone();
    bgSmall_ = decimate(background_).clone();
    trustedGain_ = 1.0;
    enter(STATE_WAITING);
    haveStaleSince_ = false;
    forgetHeldLay();
}

void MeasurementEngine::setRoi(const RoiBox& roi) {
    roi_ = roi;
    hasRoi_ = true;
    background_.release();
    bgSmall_.release();
    haveMinArea_ = false;
    forgetHeldLay();
}

void MeasurementEngine::awaitNewLay(bool fine) {
    if (state_ == STATE_MEASURED && !heldLay_.empty()) awaitNew_ = true;
    awaitFine_ = fine;
    movedRun_ = 0;
}

void MeasurementEngine::keepHeldLay() {
    const bool letGo = awaitNew_ && state_ != STATE_MEASURED;
    awaitNew_ = false;
    movedRun_ = 0;
    if (letGo && lastResult_.has_value()) enter(STATE_MEASURED);
}

void MeasurementEngine::forgetHeldLay() {
    heldLay_.release();
    heldLayGrown_.release();
    heldLayArea_ = 0.0;
    captureLay_.release();
    awaitNew_ = false;
    movedRun_ = 0;
}

void MeasurementEngine::setModel(PlaneModelPtr model) {
    model_ = std::move(model);
    haveMinArea_ = false;
    updateBridge();
}

void MeasurementEngine::updateBridge() {
    double pitch = 0.0;
    if (model_) pitch = model_->detectionNumber("pitch_px", 0.0);
    if (pitch > 0.0) {
        bridge_ = std::max(3, static_cast<int>(pitch * cfg_.mask_bridge_pitch_fraction));
    } else {
        bridge_ = cfg_.mask_bridge_px_fallback;
    }
}

double MeasurementEngine::minAreaPx() {
    if (haveMinArea_) return minArea_;
    if (!model_) {
        minArea_ = 5000.0;
        haveMinArea_ = true;
        return minArea_;
    }
    cv::Point2d centre(0.0, 0.0);
    if (hasRoi_) {
        centre = cv::Point2d((roi_.x1 + roi_.x2) / 2.0, (roi_.y1 + roi_.y2) / 2.0);
    }
    const std::pair<double, double> scale = model_->localMmPerPx(centre);
    minArea_ = cfg_.min_contour_area_mm2 / std::max(scale.first * scale.second, 1e-9);
    haveMinArea_ = true;
    return minArea_;
}

void MeasurementEngine::enter(const char* state) {
    state_ = state;
    stateSince_ = nowSeconds();
    stabiliseFrames_ = 0;
    recent_.clear();
    acc_.release();
    accN_ = 0;
    haveAccBox_ = false;
    movedRun_ = 0;
}

// Millimetres per decimated cell at the middle of the working area, or zero
// with no calibration to say.
double MeasurementEngine::mmPerCell() const {
    if (!model_) return 0.0;
    cv::Point2d centre(0.0, 0.0);
    if (hasRoi_) {
        centre = cv::Point2d((roi_.x1 + roi_.x2) / 2.0, (roi_.y1 + roi_.y2) / 2.0);
    }
    const std::pair<double, double> scale = model_->localMmPerPx(centre);
    return std::sqrt(std::max(scale.first * scale.second, 0.0)) * step_;
}

cv::Mat MeasurementEngine::grownLay(const cv::Mat& lay) const {
    const double cell = mmPerCell();
    const int r = cell > 0.0
                      ? std::max(1, static_cast<int>(std::lround(cfg_.relay_tolerance_mm / cell)))
                      : 2;
    cv::Mat out;
    cv::dilate(lay, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
    return out;
}

// Whether the lay on the mat now is a different one from the lay the held
// reading was taken from. See awaitNewLay.
//
// Compared as the decimated pass sees them: the largest outline, filled, which
// is the outline a measurement would be taken from, so a hand lying apart from
// the garment is no more a change here than it would be in the reading.
// Cloth that has gone from where the held lay was is the sure sign: nothing
// but the garment moving uncovers the mat under it. Cloth that has arrived is
// weaker evidence, because an arm reaching over the mat arrives too, so unless
// this is a retake it has to be more than an arm could be.
//
// `how`, when asked for, says how much went and how much arrived, so the
// console shows what the decision was made on.
bool MeasurementEngine::layMoved(const std::vector<cv::Point>& contourSmall, std::string* how) const {
    if (heldLay_.empty() || contourSmall.empty() || heldLayArea_ <= 0.0) return false;

    cv::Mat lay = cv::Mat::zeros(heldLay_.size(), CV_8UC1);
    cv::drawContours(lay, std::vector<std::vector<cv::Point>>{contourSmall}, -1, cv::Scalar(255),
                     cv::FILLED);
    const cv::Mat grown = grownLay(lay);
    const cv::Mat gone = heldLay_ & ~grown;
    const cv::Mat arrived = lay & ~heldLayGrown_;
    const double gonePx = static_cast<double>(cv::countNonZero(gone));
    const double arrivedPx = static_cast<double>(cv::countNonZero(arrived));

    // The larger of a share of the held lay and an area on the mat, in cells.
    // Uncalibrated there are no square millimetres, and the share alone decides.
    const double cell = mmPerCell();
    const auto floorPx = [&](double fraction, double minMm2) {
        return std::max(fraction * heldLayArea_, cell > 0.0 ? minMm2 / (cell * cell) : 0.0);
    };
    if (how) {
        char buf[160];
        if (cell > 0.0) {
            std::snprintf(buf, sizeof(buf), "%.0f mm2 of the held lay uncovered, %.0f mm2 laid beside it",
                          gonePx * cell * cell, arrivedPx * cell * cell);
        } else {
            std::snprintf(buf, sizeof(buf), "%.1f%% of the held lay uncovered, %.1f%% laid beside it",
                          100.0 * gonePx / heldLayArea_, 100.0 * arrivedPx / heldLayArea_);
        }
        *how = buf;
    }
    if (awaitFine_) {
        const double need = floorPx(cfg_.relay_retake_fraction, cfg_.relay_retake_min_mm2);
        return gonePx >= need || arrivedPx >= need;
    }
    return gonePx >= floorPx(cfg_.relay_removed_fraction, cfg_.relay_removed_min_mm2) ||
           arrivedPx >= floorPx(cfg_.relay_added_fraction, cfg_.relay_added_min_mm2);
}

void MeasurementEngine::notice(const std::string& text) {
    const std::string stamp = timestampClock();
    notices_.emplace_back(stamp, text);
    if (notices_.size() > 20) {
        notices_.erase(notices_.begin(), notices_.end() - 20);
    }
    std::printf("[%s] %s\n", stamp.c_str(), text.c_str());
    std::fflush(stdout);
}

MeasurementEngine::SettleMetrics MeasurementEngine::settleMetrics(
    const std::vector<cv::Point>& contour) {
    // Area and centroid of the decimated contour. Both are integrals over the
    // whole outline, so single-pixel boundary noise averages out instead of
    // setting the value the way an enclosing box does.
    const cv::Moments m = cv::moments(contour);
    SettleMetrics out;
    if (m.m00 <= 0.0) {
        const cv::Rect r = cv::boundingRect(contour);
        out.area = static_cast<double>(r.width) * r.height;
        out.cx = r.x + r.width / 2.0;
        out.cy = r.y + r.height / 2.0;
        return out;
    }
    out.area = m.m00;
    out.cx = m.m10 / m.m00;
    out.cy = m.m01 / m.m00;
    return out;
}

std::optional<MeasureResult> MeasurementEngine::process(const cv::Mat& frame, long long frameId) {
    if (frameId >= 0 && haveLastFrameId_ && frameId == lastFrameId_) return std::nullopt;
    if (frameId >= 0) {
        lastFrameId_ = frameId;
        haveLastFrameId_ = true;
    }

    if (background_.empty()) {
        lastDebug_ = DebugState();
        lastDebug_.has_note = true;
        lastDebug_.note = "no background reference";
        return std::nullopt;
    }

    const cv::Mat roi = crop(frame);
    if (roi.rows != background_.rows || roi.cols != background_.cols) {
        lastDebug_ = DebugState();
        lastDebug_.has_note = true;
        lastDebug_.note = "ROI changed, background stale";
        return std::nullopt;
    }

    const int step = step_;
    const cv::Mat small = decimate(roi);

    // Same segmentation the final measurement uses, on a decimated frame. The
    // live path previously ran a cheaper metric of its own with its own
    // thresholds, which is what let the coarse outline disagree with the
    // measurement and wander off the part.
    const double viewGain = cfg_.illumination_normalise ? robustGain(small, bgSmall_) : 1.0;
    double gain = viewGain;

    cv::Mat mask, score;
    double changed = 0.0;
    const int bridgeSmall = std::max(3, bridge_ / step);
    const double speckleSmall = minAreaPx() / static_cast<double>(step * step) * 0.2;
    segmentObject(small, bgSmall_, cfg_, gain, bridgeSmall, speckleSmall, mask, changed, score);
    const double firstPass = changed;

    // A gain no lamp would give: the median has been taken over by a dark
    // garment and is measuring it rather than the light. Read it again from
    // the mat alone and run this pass again with it. Where too little mat is
    // left in view to read, the last gain read with nothing on the mat stands
    // in - not the median that failed. A bright garment keeps the whole-view
    // gain unless gain_mat_only_brighter says otherwise; see config.hpp.
    const bool darker = viewGain < cfg_.gain_trust_band_low;
    const bool brighter = cfg_.gain_mat_only_brighter &&
                          (changed >= cfg_.gain_mat_only_coverage || viewGain > cfg_.gain_trust_band_high);
    const bool matOnly = cfg_.illumination_normalise && (darker || brighter);
    if (matOnly) {
        double fromMat = 0.0;
        const int margin = std::max(0, cfg_.gain_mat_margin_px / step);
        gain = matGain(small, bgSmall_, mask, margin, cfg_.gain_mat_min_fraction, fromMat)
                   ? fromMat
                   : trustedGain_;
        segmentObject(small, bgSmall_, cfg_, gain, bridgeSmall, speckleSmall, mask, changed, score);
    }
    gain_ = gain;

    // A garment darker than the base: this pass again, with the shadow vote
    // kept off whatever shows on its own evidence that it is not the base,
    // and the full-resolution pass told the same. A lay as bright as the base
    // or brighter - every white shirt - never comes here and is segmented
    // exactly as it always was. A garment near black is told from its shadow
    // by its own darkness instead (blackGarmentCeiling), on both passes.
    darkGarment_ = cfg_.dark_garment_keeps_evidence &&
                   darkerThanBase(small, bgSmall_, mask, gain, cfg_.dark_garment_ratio);
    blackCeiling_ = darkGarment_ ? blackGarmentCeiling(small, bgSmall_, mask, gain, cfg_) : 0.0;
    if (darkGarment_) {
        segmentObject(small, bgSmall_, cfg_, gain, bridgeSmall, speckleSmall, mask, changed, score,
                      true, blackCeiling_);
    }

    std::vector<cv::Point> contourSmall;
    const bool present =
        largestContour(mask, minAreaPx() / static_cast<double>(step * step), contourSmall);
    const double blobArea = present ? cv::contourArea(contourSmall) : 0.0;
    const double changedPx = static_cast<double>(cv::countNonZero(mask));
    if (!present && cfg_.illumination_normalise && changed < cfg_.gain_mat_only_coverage) {
        trustedGain_ = gain;
    }

    lastDebug_ = DebugState();
    lastDebug_.changed_fraction = changed;
    lastDebug_.mask = mask;
    lastDebug_.contour = contourSmall;
    lastDebug_.has_contour = present;
    lastDebug_.step = step;
    lastDebug_.gain = gain;
    lastDebug_.gain_mat_only = matOnly;
    lastDebug_.view_gain = viewGain;
    lastDebug_.first_pass_fraction = firstPass;
    lastDebug_.dark_garment = darkGarment_;
    lastDebug_.black_ceiling = blackCeiling_;

    watchBackground(roi, changed, blobArea, changedPx, frame);

    if (present) {
        ++presentRun_;
        absentRun_ = 0;
    } else {
        ++absentRun_;
        presentRun_ = 0;
    }

    std::optional<LiveDims> live;
    if (present) live = liveDims(contourSmall);
    lastDebug_.live = live;

    const double now = nowSeconds();

    // --- clearing, with hysteresis so one noisy frame cannot wedge us ---
    if (absentRun_ >= cfg_.clear_frames_required) {
        if (state_ != STATE_WAITING) enter(STATE_WAITING);
        // Whatever is laid on a clear mat is a new lay, with nothing to
        // compare it against.
        awaitNew_ = false;
        driftBackground(roi);
        return std::nullopt;
    }

    if (state_ == STATE_MEASURED) {
        // Laid again, for a step that asked for a new lay (awaitNewLay), and
        // seen to be a different lay on consecutive frames. From here it is
        // settled and measured exactly as a lay put down on a clear mat is,
        // starting with this frame.
        std::string how;
        movedRun_ = (awaitNew_ && present && layMoved(contourSmall, &how)) ? movedRun_ + 1 : 0;
        if (movedRun_ >= std::max(1, cfg_.relay_confirm_frames)) {
            notice("the garment has been laid again (" + how + "), measuring the new lay");
            enter(STATE_STABILISING);
        } else {
            // A part left sitting on the surface is legitimate, but a phantom
            // contour that never clears would otherwise stop the machine dead.
            if (now - stateSince_ > cfg_.measured_timeout_s) {
                notice("held in MEASURED too long, resetting and recapturing the background");
                setBackground(frame);
            }
            return std::nullopt;
        }
    }

    if (!present || !live.has_value()) return std::nullopt;

    if (state_ == STATE_WAITING) {
        if (presentRun_ < cfg_.present_frames_required) return std::nullopt;
        enter(STATE_STABILISING);
    }

    if (state_ == STATE_STABILISING) {
        ++stabiliseFrames_;
        // Time out on wall clock AND frames seen. A wall-clock-only test
        // misfires whenever the link is slow, aborting a perfectly good
        // measurement just because the frames arrived unhurriedly.
        if (now - stateSince_ > cfg_.stabilise_timeout_s &&
            stabiliseFrames_ > 4 * cfg_.stable_frames_required) {
            notice("object never settled, restarting the wait");
            enter(STATE_WAITING);
            return std::nullopt;
        }

        recent_.push_back(settleMetrics(contourSmall));
        if (recent_.size() > static_cast<std::size_t>(cfg_.stable_frames_required)) {
            recent_.erase(recent_.begin(),
                          recent_.end() - cfg_.stable_frames_required);
        }
        if (recent_.size() < static_cast<std::size_t>(cfg_.stable_frames_required)) {
            return std::nullopt;
        }

        double minArea = recent_.front().area;
        double maxArea = recent_.front().area;
        double sumArea = 0.0;
        double minCx = recent_.front().cx, maxCx = recent_.front().cx;
        double minCy = recent_.front().cy, maxCy = recent_.front().cy;
        for (const SettleMetrics& m : recent_) {
            minArea = std::min(minArea, m.area);
            maxArea = std::max(maxArea, m.area);
            sumArea += m.area;
            minCx = std::min(minCx, m.cx);
            maxCx = std::max(maxCx, m.cx);
            minCy = std::min(minCy, m.cy);
            maxCy = std::max(maxCy, m.cy);
        }
        const double meanArea = sumArea / static_cast<double>(recent_.size());
        const double areaSpread = (maxArea - minArea) / std::max(meanArea, 1.0);
        const double moved = std::max(maxCx - minCx, maxCy - minCy);

        if (areaSpread > cfg_.stability_area_fraction || moved > cfg_.stability_centroid_px) {
            return std::nullopt;
        }
        // Settled where the held reading's lay already was: touched, or passed
        // over, and left as it lay. That lay has been measured once, for the
        // step it was laid for, and measured again it would be judged against
        // a step it was not laid for. So the reading it gave is held again,
        // and the next lay is still waited for.
        std::string how;
        if (awaitNew_ && !layMoved(contourSmall, &how)) {
            notice("settled where the held reading was taken (" + how + "), holding that reading again");
            enter(STATE_MEASURED);
            return std::nullopt;
        }
        beginCapture(contourSmall, mask.size(), roi, areaSpread, moved);
        return std::nullopt;
    }

    if (state_ == STATE_CAPTURING) return accumulate(roi);

    return std::nullopt;
}

void MeasurementEngine::beginCapture(const std::vector<cv::Point>& contourSmall, cv::Size smallSize,
                                     const cv::Mat& roi, double areaSpread, double moved) {
    const int step = step_;
    const int pad = cfg_.crop_pad_px;
    const cv::Rect b = cv::boundingRect(contourSmall);
    const int rh = roi.rows;
    const int rw = roi.cols;
    const int x1 = std::max(0, b.x * step - pad);
    const int y1 = std::max(0, b.y * step - pad);
    const int x2 = std::min(rw, (b.x + b.width) * step + pad);
    const int y2 = std::min(rh, (b.y + b.height) * step + pad);
    if (x2 - x1 < 16 || y2 - y1 < 16) {
        enter(STATE_WAITING);
        return;
    }

    enter(STATE_CAPTURING);
    accBox_ = cv::Rect(x1, y1, x2 - x1, y2 - y1);
    haveAccBox_ = true;
    acc_ = cv::Mat::zeros(y2 - y1, x2 - x1, CV_32FC3);
    accN_ = 0;
    spreadArea_ = areaSpread;
    spreadMoved_ = moved;

    // The lay as it settled, for the next lay to be compared with once this
    // one's reading is held.
    captureLay_ = cv::Mat::zeros(smallSize, CV_8UC1);
    cv::drawContours(captureLay_, std::vector<std::vector<cv::Point>>{contourSmall}, -1,
                     cv::Scalar(255), cv::FILLED);
}

std::optional<MeasureResult> MeasurementEngine::accumulate(const cv::Mat& roi) {
    cv::Mat crop32;
    roi(accBox_).convertTo(crop32, CV_32FC3);
    acc_ += crop32;
    ++accN_;
    if (accN_ < cfg_.measurement_average_frames) return std::nullopt;
    return finalise(roi);
}

std::optional<MeasureResult> MeasurementEngine::finalise(const cv::Mat& roi) {
    cv::Mat scaled;
    acc_.convertTo(scaled, CV_32FC3, 1.0 / accN_);
    const cv::Mat averaged = toU8Truncate(scaled);
    cv::Mat bgCrop = background_(accBox_);

    // The base where it lies now. A cloth base is dragged by the garment laid
    // on it, and every printed line it moves is a difference the segmentation
    // below takes for garment - joined to the garment beside it, so the outline
    // runs out along the grid. The movement is read on the bare base round the
    // lay, on the latest frame, and the background followed; where the base has
    // not moved, the background is used exactly as it was. See creep.hpp.
    if (cfg_.base_creep_align) {
        cv::Mat aligned;
        const BaseCreep creep =
            alignCreptBase(roi, background_, captureLay_, step_, accBox_, cfg_, aligned);
        if (creep.aligned) {
            bgCrop = aligned;
            char buf[240];
            std::snprintf(buf, sizeof(buf),
                          "the base had moved since the background was taken (up to %.1f px, "
                          "%.1f px typically, read on %d patches of base); measured against it "
                          "where it lies now",
                          creep.max_px, creep.median_px, creep.patches);
            notice(buf);
        }
    }

    // Full resolution from here down. Cropping to the object keeps this
    // affordable without changing a single thing about how it is measured.
    // Kept so the bench can be asked afterwards what it was looking at. These
    // two are the entire input to the decision, at the resolution it was taken
    // at, which is the one thing the overlay cannot show.
    lastFineFrame_ = averaged.clone();
    lastFineBackground_ = bgCrop.clone();

    cv::Mat mask, score;
    double changed = 0.0;
    segmentObject(averaged, bgCrop, cfg_, gain_, bridge_, minAreaPx() * 0.2, mask, changed, score,
                  darkGarment_, blackCeiling_);

    std::vector<cv::Point> contour;
    if (!largestContour(mask, minAreaPx(), contour)) {
        notice("fine pass found no object, discarding this attempt");
        enter(STATE_WAITING);
        return std::nullopt;
    }

    // An outline along three or four sides of the working area is the edge of
    // the picture: the base has been taken for the object, and whatever would
    // be measured along it is the frame. Refused, never measured.
    if (cfg_.refuse_frame_outline) {
        const int sides = sidesTouched(cv::boundingRect(contour) + accBox_.tl());
        if (sides >= std::max(1, cfg_.outline_refuse_sides)) {
            refuse("Not measured: the outline found runs along " + std::to_string(sides) +
                   " sides of the working area, which is the edge of the picture and not a "
                   "garment. The base is being read as the object - usually a dark garment "
                   "covering most of the view, or a background taken with something on the "
                   "base. Check the base, and press Recalibrate with it clear if in doubt.");
            enter(STATE_WAITING);
            return std::nullopt;
        }
    }

    const int accN = accN_;
    const double spreadArea = spreadArea_;
    const double spreadMoved = spreadMoved_;

    std::optional<MeasureResult> result =
        measure(averaged, contour, score, cv::Point(accBox_.x, accBox_.y));
    if (!result.has_value()) {
        enter(STATE_WAITING);
        return std::nullopt;
    }

    ++count_;
    result->finalised = true;
    result->object_number = count_;
    result->frames_averaged = accN;
    result->has_settled = true;
    result->settled.area_spread_fraction = spreadArea;
    result->settled.centroid_movement_px = spreadMoved;
    result->timestamp = timestampIso();

    lastResult_ = result;
    heldLay_ = captureLay_;
    captureLay_.release();
    heldLayGrown_ = grownLay(heldLay_);
    heldLayArea_ = static_cast<double>(cv::countNonZero(heldLay_));
    awaitNew_ = false;
    enter(STATE_MEASURED);
    return result;
}

// How many sides of the working area a box in its coordinates runs along.
int MeasurementEngine::sidesTouched(const cv::Rect& box) const {
    const int t = std::max(0, cfg_.outline_edge_touch_px);
    const int w = background_.cols;
    const int h = background_.rows;
    int n = 0;
    if (box.x <= t) ++n;
    if (box.y <= t) ++n;
    if (box.x + box.width - 1 >= w - 1 - t) ++n;
    if (box.y + box.height - 1 >= h - 1 - t) ++n;
    return n;
}

void MeasurementEngine::refuse(const std::string& reason) {
    ++refusal_.count;
    refusal_.reason = reason;
    notice(reason);
}

std::optional<LiveDims> MeasurementEngine::liveDims(
    const std::vector<cv::Point>& contourSmall) const {
    if (contourSmall.empty()) return std::nullopt;

    // A decimated pixel stands for a whole block of full-resolution ones, so it
    // maps back to the centre of that block and not to its corner. Multiplying
    // the index by the step alone shifts the entire live outline up and to the
    // left by half a cell, which at a step of six is two and a half pixels of
    // the real frame. It never reached a measurement, which is taken at full
    // resolution, but it is what the live readout and the overlay are built
    // from, so both were quietly reporting a part slightly smaller and slightly
    // further up the mat than the one on the bench.
    const double halfCell = 0.5 * (step_ - 1);

    std::vector<cv::Point2d> pts;
    pts.reserve(contourSmall.size());
    for (const cv::Point& p : contourSmall) {
        double x = static_cast<double>(p.x) * step_ + halfCell;
        double y = static_cast<double>(p.y) * step_ + halfCell;
        if (hasRoi_) {
            x += roi_.x1;
            y += roi_.y1;
        }
        pts.emplace_back(x, y);
    }

    LiveDims out;
    if (!model_) {
        const cv::RotatedRect rect = cv::minAreaRect(toPoint2f(pts));
        out.calibrated = false;
        out.length_px = std::max(rect.size.width, rect.size.height);
        out.breadth_px = std::min(rect.size.width, rect.size.height);
        out.angle_deg = rect.angle;
        out.length_mm = out.length_px;
        out.breadth_mm = out.breadth_px;
        return out;
    }

    const cv::RotatedRect rect = cv::minAreaRect(toPoint2f(model_->pxToMm(pts)));
    out.calibrated = true;
    out.coarse = true;
    out.length_mm = std::max(rect.size.width, rect.size.height);
    out.breadth_mm = std::min(rect.size.width, rect.size.height);
    out.angle_deg = rect.angle;
    return out;
}

std::optional<MeasureResult> MeasurementEngine::measure(const cv::Mat& roiCrop,
                                                        const std::vector<cv::Point>& contour,
                                                        const cv::Mat& score,
                                                        cv::Point origin) const {
    if (contour.empty()) return std::nullopt;

    std::vector<cv::Point2d> pts;
    if (cfg_.subpixel_enabled) {
        pts = refineContourSubpixel(score, contour, cfg_);
    } else {
        pts.reserve(contour.size());
        for (const cv::Point& p : contour) pts.emplace_back(p.x, p.y);
    }

    cv::Point2d offset(origin.x, origin.y);
    if (hasRoi_) {
        offset.x += roi_.x1;
        offset.y += roi_.y1;
    }

    if (!model_) {
        const cv::RotatedRect rect = cv::minAreaRect(toPoint2f(pts));
        MeasureResult out;
        out.calibrated = false;
        out.length_px = std::max(rect.size.width, rect.size.height);
        out.breadth_px = std::min(rect.size.width, rect.size.height);
        out.angle_deg = rect.angle;
        return out;
    }

    std::vector<cv::Point2d> ptsFull(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) {
        ptsFull[k] = cv::Point2d(pts[k].x + offset.x, pts[k].y + offset.y);
    }

    // Take the parallax out of the outline, not out of a length afterwards.
    //
    // It used to come off the length inside measureInMm, and that meant it came
    // off nothing this bench actually reports. A step of kind "edge" is scored
    // by measureEdge, which is handed pxToMm(outline_px) and finds the fold in
    // it; a "curve" step traces a seam through the same outline; a "distance"
    // step is two points on it. None of them read length_mm, so none of them
    // ever saw the correction, and every step type was reporting the top face
    // of the cloth at the scale of the mat underneath it. Rendered through this
    // pipeline, a 225 mm edge at 11 mm of fold came back 228.63 mm - and the
    // bench it is modelled on reported 228.64 mm for the same nominal 225.
    //
    // Applied to the boundary instead, it is one operation in one place and
    // every measurement downstream inherits it, including ones not written yet.
    // It is a magnification about the point the lens looks straight down at, so
    // that is the fixed point it scales about - not the middle of the garment,
    // which would leave the garment's own size unchanged and move it sideways
    // instead, and not the corner of the frame.
    double correction = 1.0;
    const double heightMm = cfg_.object_height_mm;
    if (heightMm != 0.0 && cfg_.camera_distance_mm.has_value() &&
        *cfg_.camera_distance_mm > heightMm) {
        const double d = *cfg_.camera_distance_mm;
        correction = (d - heightMm) / d;
        const cv::Point2d axis = model_->centre();
        for (cv::Point2d& p : ptsFull) {
            p.x = axis.x + (p.x - axis.x) * correction;
            p.y = axis.y + (p.y - axis.y) * correction;
        }
    }

    // measureInMm carries a correction of its own for callers that have not
    // been through here. This outline has, so it is told there is nothing left
    // to take off rather than having it taken off twice.
    Config measured = cfg_;
    measured.object_height_mm = 0.0;
    MeasureResult result = measureInMm(ptsFull, *model_, measured);
    result.parallax_correction = correction;
    result.outline_px = ptsFull;
    result.calibrated = true;
    result.has_saturation = true;
    result.saturated_fraction = saturation(roiCrop, contour);
    return result;
}

// Blown-out highlights bloom into neighbouring pixels and push the apparent
// edge outward. If this climbs above about half a percent, drop exposure.
double MeasurementEngine::saturation(const cv::Mat& roi,
                                     const std::vector<cv::Point>& contour) const {
    cv::Mat m = cv::Mat::zeros(roi.rows, roi.cols, CV_8UC1);
    std::vector<std::vector<cv::Point>> polys{contour};
    cv::drawContours(m, polys, -1, cv::Scalar(255), cv::FILLED);

    long long inside = 0;
    long long clipped = 0;
    for (int y = 0; y < roi.rows; ++y) {
        const unsigned char* mp = m.ptr<unsigned char>(y);
        const unsigned char* rp = roi.ptr<unsigned char>(y);
        for (int x = 0; x < roi.cols; ++x) {
            if (!mp[x]) continue;
            ++inside;
            const int i = x * 3;
            const unsigned char mx = std::max(rp[i], std::max(rp[i + 1], rp[i + 2]));
            if (mx >= 254) ++clipped;
        }
    }
    if (inside == 0) return 0.0;
    return static_cast<double>(clipped) / static_cast<double>(inside);
}

// Recover from a stale background reference without operator intervention.
//
// This is what used to end the run. Lighting and sensor response drift over
// minutes; once the reference no longer matches, the difference mask fills with
// scattered noise, the largest blob is junk, and the machine sits in
// STABILISING forever looking like it has stopped seeing anything.
//
// A real part is one coherent blob. A stale reference is confetti. Testing the
// ratio between the largest blob and the total changed area tells the two
// apart, so a genuinely large workpiece is never mistaken for drift.
void MeasurementEngine::watchBackground(const cv::Mat& roi, double changed, double blobArea,
                                        double changedPx, const cv::Mat& frame) {
    (void)frame;
    const bool coherent = blobArea > 0.5 * std::max(changedPx, 1.0);
    if (changed < cfg_.background_stale_fraction || coherent) {
        haveStaleSince_ = false;
        return;
    }
    const double now = nowSeconds();
    if (!haveStaleSince_) {
        haveStaleSince_ = true;
        staleSince_ = now;
        return;
    }
    if (now - staleSince_ < cfg_.background_stale_seconds) return;

    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "background reference has gone stale (%.0f%% of the surface changed with no "
                  "coherent object), rebuilding it from the current view",
                  changed * 100.0);
    notice(buf);
    background_ = roi.clone();
    bgSmall_ = decimate(background_).clone();
    trustedGain_ = 1.0;
    enter(STATE_WAITING);
    haveStaleSince_ = false;
    forgetHeldLay();
}

// Track slow lighting drift while the surface is clear. Running whenever
// nothing is detected, rather than only below a fixed change threshold, means
// the reference cannot fall behind and strand the detector, which is exactly
// how it used to fail.
void MeasurementEngine::driftBackground(const cv::Mat& roi) {
    const double now = nowSeconds();
    if (!haveIdleSince_) {
        haveIdleSince_ = true;
        idleSince_ = now;
        return;
    }
    if (now - idleSince_ < cfg_.background_drift_idle_seconds) return;

    const double a = cfg_.background_drift_blend;
    cv::Mat blended;
    cv::addWeighted(background_, 1.0 - a, roi, a, 0.0, blended);
    background_ = blended;
    bgSmall_ = decimate(background_).clone();
    idleSince_ = now;
}

}  // namespace verimeasure
