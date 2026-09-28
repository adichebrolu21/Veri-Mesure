// Where the operator is pointing.
//
// The seam a curve step measures is invisible to the machine, so somebody has
// to say where it is. This is the hands-free way of saying it: reach in, hold a
// fingertip on the seam for half a second, lift, repeat. Three of those describe
// a neckline. The other hand never leaves the fabric.
//
// What comes out of here is a seed for curve.hpp and nothing else. It is not a
// measurement and must never become one - a fingertip is ten millimetres above
// a plane model fitted to the mat, which at the edge of this frame is four
// millimetres of parallax that grows from the centre outward, and the tip
// itself is ambiguous by most of a centimetre under an occluding nail. Fed to
// snapCurve as a corridor centre those errors are irrelevant, because the seam
// is found from the picture afterwards. Reported as a position they would be
// the whole error budget.
//
// The hand is found by differencing against the SETTLED scene - the garment
// lying on the mat - rather than against the empty mat. That is the difference
// between a picture with an arm in it and a picture with a shirt and an arm
// merged into one blob, and there is no arm to find in the second. It also
// means the tracker has to be told when the scene it is differencing against
// has changed, which is what setReference is for.
//
// And it is differenced the way segmentation.cpp differences things, by the
// ANGLE between the two colour vectors rather than by how much the brightness
// changed. That is not a refinement, it is the difference between working and
// not: a hand held over a white shirt under a bench light throws a shadow that
// reaches further across the cloth than the hand does, and a brightness
// threshold cannot tell the two apart. It finds the shadow, takes its furthest
// corner for a fingertip, and puts the point somewhere nobody pointed. The
// shadow scales all three channels together and leaves the colour vector's
// direction alone; skin does not.

#pragma once

#include <optional>
#include <opencv2/core.hpp>
#include <vector>

#include "verimeasure/calibration.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/plane_model.hpp"

namespace verimeasure {

struct FingerState {
    bool present = false;      // an arm is in the picture
    bool dwelling = false;     // and it has stopped moving
    double dwell_s = 0.0;      // for how long, so the page can draw the wait
    double dwell_fraction = 0.0;  // that, as a share of what is needed

    cv::Point2d tip_px{0.0, 0.0};  // full-resolution pixels
    cv::Point2d tip_mm{0.0, 0.0};  // through the plane model, on the mat plane
    bool has_mm = false;

    // A point was committed on this frame, and here it is. The mean of the tip
    // over the whole dwell rather than its last position: half a second of
    // frames averages out most of the shake, and it costs nothing to keep them.
    bool committed = false;
    cv::Point2d point_mm{0.0, 0.0};

    // -- the trace --
    //
    // Hold still at one end of the seam until the ring closes, draw along it,
    // hold still again at the other end - or simply take the hand away. What
    // the finger did in between is the path.
    //
    // Three taps were the first way of doing this and the trace is the answer
    // to how they failed. Two of the three are easy: an end of a seam is a
    // corner, there is something to aim at. The middle one is not, because
    // there is no landmark at the deepest part of an arc - the operator is
    // estimating where the middle of a curve ought to be, and on the first real
    // jersey this met it landed sixteen millimetres off, into plain cloth. That
    // one point sets the whole shape, because the arc goes exactly through all
    // three, so being wrong there bends the corridor rather than shifting it. A
    // traced path has no such point: it is on the seam everywhere, because the
    // operator was looking at the seam the whole way along.
    //
    // And it costs nothing in occlusion, which is the part that makes it work
    // at all. The seam is read from the frame held BEFORE the hand came in, so
    // a finger lying across the seam it is tracing hides nothing that is going
    // to be measured. The operator can hover rather than touch, which is also
    // the end of the objection that a finger dragged along ribbed jersey
    // stretches the thing being measured.
    bool tracing = false;                // a trace is being drawn now
    std::vector<cv::Point2d> trace_mm;   // what it has recorded so far, for the feed
    // How long the tip has been standing still while tracing, as a share of
    // what it takes to end one. The feed draws it as the same closing ring that
    // starts a trace, so the operator can see the trace about to finish and
    // move again if they did not mean it.
    double hold_fraction = 0.0;
    // The tip found on this frame was further from the last one than a hand
    // can move in the time, so it was not recorded. The feed can show the ring
    // hollow for it, and an operator who sees that once learns what it means.
    bool rejected = false;
    bool finished = false;               // and it ended on this frame
    std::vector<cv::Point2d> path_mm;    // with this path
};

class FingerTracker {
public:
    explicit FingerTracker(const Config& cfg) : cfg_(cfg) {}

    // The scene to difference against: the garment as it lies, with no hand in
    // it. Call this whenever the reading is held and before pointing starts.
    void setReference(const cv::Mat& frameBgr, const RoiBox& roi, bool hasRoi);
    bool hasReference() const { return !ref_.empty(); }

    // Forget the dwell in progress, without forgetting the scene. Used when a
    // point lands, when the operator clears the seeds, and when the step
    // changes underneath.
    void resetDwell();

    // Give up the trace in progress without measuring it.
    void abandonTrace();

    // One frame. `now` is monotonic seconds; passing it in rather than reading
    // a clock is what lets the offline harness run this at any speed it likes.
    FingerState update(const cv::Mat& frameBgr, const PlaneModel* model, double now);

    // What the tracker saw on the last frame with a hand in it, at the
    // decimation it works at: the settled scene, the frame, the arm mask, and
    // the tip in that mask. Kept so a trace that went wrong on a bench can be
    // written out and replayed offline against a changed setting or a changed
    // build - which is the only way a failure of this layer has ever been
    // understood, because what it gets wrong is what it SEES, and the numbers
    // it reports afterwards cannot show that.
    struct Seen {
        cv::Mat ref, frame, mask;
        cv::Point2d tipSmall{0.0, 0.0};
        bool hasTip = false;
        double mmPerCell = 0.0;  // zero when there was no plane model
        int step = 1;
    };
    const Seen& lastSeen() const { return seen_; }

    // The tracker's eye, one frame of it: from the decimated frame and the
    // decimated settled scene to the arm mask and the tip in it, with every
    // length already in cells. update() is this plus the gesture; the offline
    // replay is this alone, on the pictures update() saved. One function, so
    // the two cannot drift apart.
    static bool locate(const cv::Mat& curSmall, const cv::Mat& refSmall, const Config& cfg,
                       double lookPx, double setbackPx, int bridgePx, double minAreaPx,
                       const cv::Point2d* previous, cv::Mat& mask, cv::Point2d& tipSmall);

private:
    Seen seen_;
    Config cfg_;

    cv::Mat ref_;  // decimated colour of the settled scene, ROI only
    RoiBox roi_;
    bool hasRoi_ = false;
    int step_ = 1;

    bool dwelling_ = false;
    double dwellSince_ = 0.0;
    double rearmAt_ = 0.0;
    cv::Point2d anchorMm_{0.0, 0.0};
    std::vector<cv::Point2d> dwellPoints_;

    void finishTrace(FingerState& state);

    bool tracing_ = false;
    std::vector<cv::Point2d> path_;
    int absent_ = 0;  // consecutive frames with no hand, while tracing
    bool still_ = false;
    double stillSince_ = 0.0;
    int stillFrames_ = 0;
    cv::Point2d stillAt_{0.0, 0.0};
    bool haveLast_ = false;
    cv::Point2d lastTipSmall_{0.0, 0.0};

    // The motion gate: where the tip was last believed, and when, and how many
    // frames in a row it has been somewhere it could not have got to.
    bool haveSeen_ = false;
    cv::Point2d seenMm_{0.0, 0.0};
    double seenAt_ = 0.0;
    int lost_ = 0;
};

// The tip of an arm reaching into the picture, in decimated pixels.
//
// `mask` is foreground against the settled scene, already shadow-invariant.
//
// Two questions, and the first one is what makes this work at all: does the
// moving blob touch the border? A hand is attached to an arm and an arm comes
// in from outside the picture. A moving thing that touches no border is a
// shadow, a draught in a hanging sleeve, or the garment settling further - and
// each of those, taken for a fingertip, puts a seed somewhere nobody pointed.
//
// The second is which end of it is the finger, and it is asked in two parts.
//
// Furthest from where the arm crosses the border, measured ALONG the arm. Not
// in a straight line, which is what this used to do: a straight line is exact
// only while the finger is in line with the arm, and tracing a curve is the one
// thing that guarantees it is not. Nobody re-aims a whole arm at every
// millimetre of a neckline; the forearm stays where it is and the wrist bends.
// Along the arm the fingertip is the last thing reached however the wrist is
// turned, because the finger is attached to the hand and the hand to the arm.
//
// And thin. A palm is eighty-five millimetres across and a finger seventeen, so
// with the wrist turned right over the far corner of the palm can be further
// along the arm than the fingertip - not by much, and not always, which is the
// worst way for a rule to be wrong. A fingertip is the far end of the THIN part
// of the arm, and the distance transform says which part that is: eight or nine
// millimetres down a finger, forty across a palm, and nothing in between. When
// no part of the blob is that thin - a closed fist - it falls back to the far
// end of the arm and the setback cap bounds what that can cost.
//
// Returns false when nothing in the picture is an arm.
// `previous`, when given, is where the tip was on the last frame, in the same
// decimated pixels. Among the components that could be an arm it prefers the
// one that still contains that point, and only falls back to the largest when
// none does. A hand does not teleport between frames, and without that the tip
// steps to whatever else has become the biggest thing touching a border - a
// sleeve settling, a sheet of paper nudged by the wrist - and the trace follows
// it there.
//
// `lookPx` is how much of the blob around the tip counts as "the tip" when
// measuring how wide the fingertip is, and how far back down the finger the
// setback is aimed - the middle of that window sits behind the leading edge,
// down the middle of the finger, because that is the only thing in it. Aiming
// the setback along the ARM instead, as this used to, turns it sideways as soon
// as the wrist bends, and a centimetre at right angles to the finger is a
// centimetre straight across the seam.
//
// `maxSetbackPx` is a fingertip's own half-width, and it does two jobs: it caps
// how far the tip may be moved back, and it is the width at or under which a
// part of the arm counts as thin enough to be a finger. Both in the decimated
// pixels this works in; the caller turns millimetres into them.
bool findArmTip(const cv::Mat& mask, int borderMargin, double minAreaPx, cv::Point2d& tip,
                const cv::Point2d* previous = nullptr, double lookPx = 6.0,
                double maxSetbackPx = 1e9);

}  // namespace verimeasure
