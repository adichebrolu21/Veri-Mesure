// Frame in, result out. No windows, no keyboard, no camera dependency. This is
// the object to wire the frontend to.
//
//     MeasurementEngine engine(cfg, model, roi);
//     engine.setBackground(emptySurfaceFrame);
//     auto result = engine.process(frame, seq);   // set when a measurement completes
//
// Work is split across two scales, which is what makes it keep up with the
// camera. Every frame runs a cheap decimated test that answers only "is
// something there, and roughly where". The expensive part, sub-pixel edge
// refinement and the four-sided fit, runs once per object at full resolution on
// a crop around it. Measurement accuracy is therefore identical to a
// full-resolution pass; only the per-frame cost changed.

#pragma once

#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "verimeasure/calibration.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/measure.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

extern const char* const STATE_WAITING;
extern const char* const STATE_STABILISING;
extern const char* const STATE_CAPTURING;
extern const char* const STATE_MEASURED;

// Rough dimensions from the decimated contour, for the overlay and for deciding
// when the part has stopped moving. Never reported as a result.
struct LiveDims {
    bool calibrated = false;
    bool coarse = false;
    double length_mm = 0.0;
    double breadth_mm = 0.0;
    double length_px = 0.0;
    double breadth_px = 0.0;
    double angle_deg = 0.0;
};

struct DebugState {
    bool has_note = false;
    std::string note;

    double changed_fraction = 0.0;
    cv::Mat mask;                        // decimated
    std::vector<cv::Point> contour;      // decimated, empty when nothing found
    bool has_contour = false;
    int step = 1;
    double gain = 1.0;
    // Whether the gain above was read from the mat alone, and the whole-view
    // gain and coverage of the first pass that decided it.
    bool gain_mat_only = false;
    double view_gain = 1.0;
    double first_pass_fraction = 0.0;
    // Whether the lay was taken for a garment darker than the base, and, for
    // one near black, how bright a pixel may be and still be it (0 if not).
    bool dark_garment = false;
    double black_ceiling = 0.0;
    std::optional<LiveDims> live;
};

class MeasurementEngine {
public:
    MeasurementEngine(const Config& cfg, PlaneModelPtr model, const RoiBox& roi, bool hasRoi);

    void setBackground(const cv::Mat& frame);
    void setRoi(const RoiBox& roi);
    void setModel(PlaneModelPtr model);

    // frameId, when supplied, must be a value that changes with every new frame
    // from the camera. Feeding the same frame in twice is not harmless: settling
    // is judged by comparing consecutive frames, so a repeated frame looks like
    // a perfectly motionless part. Process one frame four times and the engine
    // will call a part settled while it is still being put down, then measure
    // that. Passing the sequence number makes the mistake impossible rather than
    // merely documented.
    std::optional<MeasureResult> process(const cv::Mat& frame, long long frameId = -1);

    // Let go of the held reading once the garment is laid again, without the
    // mat having to be cleared first.
    //
    // A reading is held until the mat has been clear for a moment, and on a
    // bench where the shirt is refolded where it lies between one fold and the
    // next, that moment never comes: the next fold sits on the mat unmeasured,
    // with nothing drawn round it. Releasing the hold outright on a change of
    // step is no better - it measures the fold still lying there from the step
    // just finished and judges it against the new one.
    //
    // So this releases it on evidence instead. While it is armed, the lay on
    // the mat is compared frame by frame with the lay the held reading was
    // taken from, and once it is plainly a different one - cloth gone from
    // where the old lay was, or a large new piece laid beside it - it is
    // settled and measured exactly as a lay put down on a clear mat would be.
    // A lay that is touched and left as it was, or a hand passing over it,
    // comes back to the reading already held and nothing new is measured.
    //
    // `fine` is for a retake of the reading being held: that lay was laid for
    // the step being retaken, so a fold only straightened where it lies is
    // worth reading again. Otherwise the change has to be larger than a hand
    // or an arm in the picture can make, because a hand is how a seam is
    // pointed at on the lay that is being held.
    //
    // Does nothing unless a reading is held. Cleared by the next reading, by a
    // clear mat and by a new background.
    void awaitNewLay(bool fine);
    // Hold whatever the mat does until it is cleared, as without awaitNewLay.
    // A reading already let go for a new lay that has not been measured yet is
    // held again, so nothing is read for a step that did not ask for one.
    void keepHeldLay();
    bool awaitingNewLay() const { return awaitNew_; }

    // Why the last lay that settled was refused rather than measured. The
    // count moves with every refusal, so a caller can tell a new one from one
    // it has already shown.
    struct Refusal {
        long long count = 0;
        std::string reason;
    };
    const Refusal& lastRefusal() const { return refusal_; }

    // -- state --
    const std::string& state() const { return state_; }
    int count() const { return count_; }
    void setCount(int value) { count_ = value; }
    const DebugState& lastDebug() const { return lastDebug_; }
    const std::optional<MeasureResult>& lastResult() const { return lastResult_; }
    PlaneModelPtr model() const { return model_; }
    bool hasRoi() const { return hasRoi_; }
    const RoiBox& roi() const { return roi_; }
    // The background reference as it is actually being differenced against,
    // already cropped to the working area. Exposed so a bench that is drawing a
    // boundary in the wrong place can be asked to hand over the two pictures
    // the decision was made from.
    const cv::Mat& background() const { return background_; }
    // The frame and the reference the last full-resolution measurement was
    // taken from, cropped the same way. Empty until one has been taken.
    const cv::Mat& lastFineFrame() const { return lastFineFrame_; }
    const cv::Mat& lastFineBackground() const { return lastFineBackground_; }
    std::size_t recentCount() const { return recent_.size(); }
    int accumulatedFrames() const { return accN_; }
    const Config& config() const { return cfg_; }
    Config& config() { return cfg_; }

private:
    struct SettleMetrics {
        double area = 0.0;
        double cx = 0.0;
        double cy = 0.0;
    };

    cv::Mat decimate(const cv::Mat& img) const;
    cv::Mat crop(const cv::Mat& frame) const;
    double minAreaPx();
    void updateBridge();
    void enter(const char* state);
    void notice(const std::string& text);

    static SettleMetrics settleMetrics(const std::vector<cv::Point>& contour);
    void beginCapture(const std::vector<cv::Point>& contourSmall, cv::Size smallSize,
                      const cv::Mat& roi, double areaSpread, double moved);
    double mmPerCell() const;
    cv::Mat grownLay(const cv::Mat& lay) const;
    bool layMoved(const std::vector<cv::Point>& contourSmall, std::string* how = nullptr) const;
    void forgetHeldLay();
    std::optional<MeasureResult> accumulate(const cv::Mat& roi);
    std::optional<MeasureResult> finalise(const cv::Mat& roi);
    int sidesTouched(const cv::Rect& boxInArea) const;
    void refuse(const std::string& reason);
    std::optional<LiveDims> liveDims(const std::vector<cv::Point>& contourSmall) const;
    std::optional<MeasureResult> measure(const cv::Mat& roiCrop,
                                         const std::vector<cv::Point>& contour,
                                         const cv::Mat& score, cv::Point origin) const;
    double saturation(const cv::Mat& roi, const std::vector<cv::Point>& contour) const;
    void watchBackground(const cv::Mat& roi, double changed, double blobArea, double changedPx,
                         const cv::Mat& frame);
    void driftBackground(const cv::Mat& roi);

    Config cfg_;
    PlaneModelPtr model_;
    cv::Mat lastFineFrame_;
    cv::Mat lastFineBackground_;
    RoiBox roi_;
    bool hasRoi_ = false;

    cv::Mat background_;
    cv::Mat bgSmall_;
    std::string state_;
    std::vector<SettleMetrics> recent_;
    int count_ = 0;
    DebugState lastDebug_;
    std::optional<MeasureResult> lastResult_;
    Refusal refusal_;
    std::vector<std::pair<std::string, std::string>> notices_;

    int step_ = 1;
    bool haveMinArea_ = false;
    double minArea_ = 0.0;
    int presentRun_ = 0;
    int absentRun_ = 0;
    double stateSince_ = 0.0;
    int stabiliseFrames_ = 0;
    bool haveStaleSince_ = false;
    double staleSince_ = 0.0;
    bool haveIdleSince_ = false;
    double idleSince_ = 0.0;
    double gain_ = 1.0;
    // The last gain read with nothing on the mat: what the gain falls back to
    // when a garment leaves too little mat in view to read it from.
    double trustedGain_ = 1.0;
    // The lay on the latest frame is darker than the base; the fine pass is
    // told so too. See darkerThanBase. And for a lay near black, the ceiling
    // it is told from its shadow by (blackGarmentCeiling), 0 otherwise.
    bool darkGarment_ = false;
    double blackCeiling_ = 0.0;
    long long lastFrameId_ = -1;
    bool haveLastFrameId_ = false;
    int bridge_ = 0;

    cv::Mat acc_;
    int accN_ = 0;
    cv::Rect accBox_;
    bool haveAccBox_ = false;
    double spreadArea_ = 0.0;
    double spreadMoved_ = 0.0;

    // Where the cloth lay for the reading being held, as the decimated pass
    // saw it when that lay settled: filled, and again grown by the tolerance.
    // captureLay_ is the lay being captured now, which becomes the held one
    // only if the capture ends in a reading.
    cv::Mat heldLay_;
    cv::Mat heldLayGrown_;
    double heldLayArea_ = 0.0;
    cv::Mat captureLay_;
    bool awaitNew_ = false;
    bool awaitFine_ = false;
    int movedRun_ = 0;
};

// Monotonic seconds, used for every elapsed-time test in the engine.
double nowSeconds();

}  // namespace verimeasure
