#include "verimeasure/finger.hpp"

#include <algorithm>
#include <cmath>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/numeric.hpp"
#include "verimeasure/segmentation.hpp"

namespace verimeasure {
namespace {

// Kept in colour, and that is the whole point: the shadow test below needs all
// three channels, and a gray copy has already thrown away the only thing that
// separates a hand from the shadow it casts.
cv::Mat decimatedColour(const cv::Mat& frameBgr, const RoiBox& roi, bool hasRoi, int step) {
    cv::Mat crop = frameBgr;
    if (hasRoi && roi.valid()) {
        const cv::Rect r = roi.toRect() & cv::Rect(0, 0, frameBgr.cols, frameBgr.rows);
        if (r.width > 0 && r.height > 0) crop = frameBgr(r);
    }
    return strideSample(crop, std::max(1, step)).clone();
}

// How far every pixel of the arm is from where the arm crossed the border,
// measured ALONG the arm.
//
// This is the whole of which end is the finger, and a straight line is the
// wrong ruler for it. The rule used to be the point of the blob furthest from
// the border in a STRAIGHT line, on the reasoning that a limb reaching in is
// straight enough - and it is, right up to the moment the operator does the one
// thing a curve step asks them to do. Tracing a neckline is not done by re-
// aiming the whole arm at every millimetre of seam; the forearm is held where
// it is and the wrist bends. Past about forty degrees of that bend the far side
// of the hand is further from the border than the fingertip is, the tip jumps
// back onto the knuckles, and it stays there: measured on a rendered neckline at
// sixty degrees, the reported tip sat 48 mm from the finger and 29 mm from the
// knuckle. What the operator then draws along the seam, the machine records as
// the hand wandering behind it - a shorter, blunter, nearly straight path that
// is not the curve and is not refused either, because it is perfectly smooth.
//
// Along the arm, none of that can happen. Walk out from the border and the
// fingertip is the last thing reached however the wrist is turned, because the
// finger is attached to the hand and the hand to the arm. It is the same
// question the old rule was asking, asked with the ruler that fits the shape.
//
// Computed by chamfer sweeps rather than a queue: an arm is a long thin thing
// and the wavefront runs down it almost monotonically, so a couple of passes
// over the bounding box settle it, and a pass costs no allocation. `dist` comes
// back over the box alone, negative off the component.
bool armDistance(const cv::Mat& labels, int best, int x0, int y0, int x1, int y1, int margin,
                 int w, int h, cv::Mat& dist) {
    const int bw = x1 - x0;
    const int bh = y1 - y0;
    if (bw <= 0 || bh <= 0) return false;

    const float kFar = 1e18f;
    dist.create(bh, bw, CV_32F);

    long long sources = 0;
    for (int y = 0; y < bh; ++y) {
        const int* row = labels.ptr<int>(y + y0);
        float* dr = dist.ptr<float>(y);
        for (int x = 0; x < bw; ++x) {
            if (row[x + x0] != best) {
                dr[x] = -1.0f;  // not the arm
                continue;
            }
            const int gx = x + x0;
            const int gy = y + y0;
            const bool onBorder =
                gx <= margin || gy <= margin || gx >= w - 1 - margin || gy >= h - 1 - margin;
            dr[x] = onBorder ? 0.0f : kFar;
            if (onBorder) ++sources;
        }
    }
    // The bounding box reaching the border is not the same as a pixel doing so,
    // and only a pixel can be where the arm came in.
    if (sources == 0) return false;

    const float kD = 1.0f;
    const float kQ = 1.41421356f;
    for (int pass = 0; pass < 16; ++pass) {
        bool moved = false;
        for (int y = 0; y < bh; ++y) {
            float* dr = dist.ptr<float>(y);
            const float* up = y > 0 ? dist.ptr<float>(y - 1) : nullptr;
            for (int x = 0; x < bw; ++x) {
                if (dr[x] < 0.0f) continue;
                float d = dr[x];
                if (x > 0 && dr[x - 1] >= 0.0f) d = std::min(d, dr[x - 1] + kD);
                if (up) {
                    if (up[x] >= 0.0f) d = std::min(d, up[x] + kD);
                    if (x > 0 && up[x - 1] >= 0.0f) d = std::min(d, up[x - 1] + kQ);
                    if (x + 1 < bw && up[x + 1] >= 0.0f) d = std::min(d, up[x + 1] + kQ);
                }
                if (d < dr[x]) {
                    dr[x] = d;
                    moved = true;
                }
            }
        }
        for (int y = bh - 1; y >= 0; --y) {
            float* dr = dist.ptr<float>(y);
            const float* down = y + 1 < bh ? dist.ptr<float>(y + 1) : nullptr;
            for (int x = bw - 1; x >= 0; --x) {
                if (dr[x] < 0.0f) continue;
                float d = dr[x];
                if (x + 1 < bw && dr[x + 1] >= 0.0f) d = std::min(d, dr[x + 1] + kD);
                if (down) {
                    if (down[x] >= 0.0f) d = std::min(d, down[x] + kD);
                    if (x + 1 < bw && down[x + 1] >= 0.0f) d = std::min(d, down[x + 1] + kQ);
                    if (x > 0 && down[x - 1] >= 0.0f) d = std::min(d, down[x - 1] + kQ);
                }
                if (d < dr[x]) {
                    dr[x] = d;
                    moved = true;
                }
            }
        }
        if (!moved) break;
    }
    return true;
}

}  // namespace

bool findArmTip(const cv::Mat& maskIn, int borderMargin, double minAreaPx, cv::Point2d& tip,
                const cv::Point2d* previous, double lookPx, double maxSetbackPx) {
    if (maskIn.empty()) return false;
    cv::Mat mask = maskIn.clone();

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (n <= 1) return false;

    const int margin = std::max(0, borderMargin);
    const int w = mask.cols;
    const int h = mask.rows;

    // The component the tip was in last frame, if it is still an arm. A hand
    // does not teleport, so continuity beats size: without this the tip steps
    // to whatever else has become the biggest thing touching a border, and the
    // trace follows it there.
    int carried = -1;
    if (previous) {
        const int px = static_cast<int>(std::lround(previous->x));
        const int py = static_cast<int>(std::lround(previous->y));
        if (px >= 0 && py >= 0 && px < mask.cols && py < mask.rows) {
            const int at = labels.at<int>(py, px);
            if (at > 0 && stats.at<int>(at, cv::CC_STAT_AREA) >= minAreaPx) carried = at;
        }
    }

    int best = -1;
    int bestArea = 0;
    for (int k = 1; k < n; ++k) {
        const int area = stats.at<int>(k, cv::CC_STAT_AREA);
        if (area < minAreaPx || area <= bestArea) continue;

        // Does it reach the border? An arm does; nothing else in this picture
        // should. The bounding box is enough to ask with, and it is the box the
        // component labeller has already computed.
        const int x = stats.at<int>(k, cv::CC_STAT_LEFT);
        const int y = stats.at<int>(k, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(k, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(k, cv::CC_STAT_HEIGHT);
        const bool touches =
            x <= margin || y <= margin || x + bw >= w - margin || y + bh >= h - margin;
        if (!touches) continue;

        best = k;
        bestArea = area;
    }

    // Only if it still reaches a border, which is the test that made any of
    // this work: continuity must not keep the tip on a blob that has stopped
    // being an arm.
    if (carried > 0) {
        const int x = stats.at<int>(carried, cv::CC_STAT_LEFT);
        const int y = stats.at<int>(carried, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(carried, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(carried, cv::CC_STAT_HEIGHT);
        if (x <= margin || y <= margin || x + bw >= w - margin || y + bh >= h - margin) {
            best = carried;
        }
    }
    if (best < 0) return false;

    // Where the arm crosses the border, and then the point of it furthest from
    // there ALONG the arm. Every pass below walks only the winning component's
    // own bounding box.
    const int x0 = stats.at<int>(best, cv::CC_STAT_LEFT);
    const int y0 = stats.at<int>(best, cv::CC_STAT_TOP);
    const int x1 = x0 + stats.at<int>(best, cv::CC_STAT_WIDTH);
    const int y1 = y0 + stats.at<int>(best, cv::CC_STAT_HEIGHT);

    cv::Mat along;
    if (!armDistance(labels, best, x0, y0, x1, y1, margin, w, h, along)) return false;

    // How thick the arm is at every point of it, which is the second half of
    // saying which end is the finger.
    //
    // Distance along the arm alone is not quite enough, and a palm is why. A
    // hand is eighty-five millimetres across and a finger seventeen, so when the
    // wrist is turned the finger points back over the hand and the far corner of
    // the PALM can be further along the arm than the fingertip is - not by much,
    // and not always, which is the worst way for a rule to be wrong. A fingertip
    // is not merely the far end of the arm. It is the far end of the THIN part
    // of it, and thin is a thing the picture can be asked directly: the distance
    // transform at a point is how far it is to the nearest edge, so it reads
    // eight or nine millimetres down a finger and forty across a palm. Nothing
    // in between is close.
    //
    // So the tip is the furthest point along the arm among those on a part of it
    // no wider than a fingertip - and if the operator points with a closed fist,
    // where no part of the blob is that thin, it is the furthest point along the
    // arm and the setback cap below is what bounds the damage.
    //
    // How wide the arm is AT a point is not the distance from that point to the
    // nearest edge, and using the one for the other would make this test very
    // nearly do nothing: every point within a fingertip's half-width of any
    // boundary reads thin, which is the whole rim of the palm as well as the
    // finger, and the far end of the arm is on the boundary in any case. What is
    // wanted is the widest the arm gets nearby, so a point on the rim of a palm
    // is judged by the palm behind it. Dilating the distance transform is
    // exactly that, everywhere at once.
    cv::Mat dt, thick;
    cv::distanceTransform(mask, dt, cv::DIST_L2, 3);
    const int span = 2 * std::max(1, static_cast<int>(std::lround(std::max(3.0, lookPx)))) + 1;
    cv::dilate(dt, thick, cv::Mat::ones(span, span, CV_8U));

    double bestDist = -1.0;
    double bestThin = -1.0;
    cv::Point2d far, thin;
    for (int y = 0; y < along.rows; ++y) {
        const float* dr = along.ptr<float>(y);
        const float* tr = thick.ptr<float>(y + y0);
        for (int x = 0; x < along.cols; ++x) {
            if (dr[x] < 0.0f) continue;
            if (dr[x] > bestDist) {
                bestDist = dr[x];
                far = cv::Point2d(x + x0, y + y0);
            }
            if (tr[x + x0] <= maxSetbackPx && dr[x] > bestThin) {
                bestThin = dr[x];
                thin = cv::Point2d(x + x0, y + y0);
            }
        }
    }
    if (bestDist < 0.0) return false;
    if (bestThin >= 0.0) far = thin;

    // Back off by half a finger.
    //
    // The furthest point of the silhouette is the leading edge of the fingertip.
    // The cloth is touched under the pad, which from directly above sits at the
    // centre of the rounding rather than at its front - half a finger further
    // back, and half a finger is most of a centimetre. Left in, that offset
    // points along whatever direction the operator's hand happened to lie, so
    // it lands partly across the seam, where the snap removes it, and partly
    // ALONG the seam, where nothing can. The along part would go straight into
    // the reading.
    //
    // How far back is not assumed, it is measured: the distance transform of
    // the blob is exactly the distance to the nearest edge of it, so its
    // largest value AT the tip is the half-width of that fingertip, whether the
    // operator is pointing with a finger, a thumb or a gloved hand.
    //
    // At the tip, and both of those words were got wrong once. The window used
    // to be a fraction of how far the arm reached in, which on a real arm is
    // seventy millimetres of it - so what got measured was not the fingertip's
    // half-width but the HAND's, thirty-five millimetres of fist, and the tip
    // was duly set back into the middle of the hand. It tracked the hand rather
    // than the finger, and the trace it drew wandered around the knuckles.
    //
    // So the window is a fingertip's own size, and the setback is capped at a
    // fingertip's own half-width. Both are lengths on the mat rather than
    // fractions of an arm, and the cap is what bounds the damage when somebody
    // points with a closed fist and there is no fingertip to find.
    //
    // And it is backed off along the FINGER, which is a different direction
    // from along the arm as soon as the wrist is bent - which on a curve step is
    // most of the time. The same window that measures how wide the fingertip is
    // gives the direction for free: its centre of mass sits behind the leading
    // edge, down the middle of the finger, because that is the only thing in the
    // window. Aimed along the arm instead, a bent wrist turns the setback
    // sideways, and a centimetre applied at right angles to the finger is a
    // centimetre straight across the seam.
    double half = 0.0;
    double sx = 0.0, sy = 0.0;
    long long inLook = 0;
    const double look = std::max(3.0, lookPx);
    for (int y = y0; y < y1; ++y) {
        const int* row = labels.ptr<int>(y);
        const float* dr = dt.ptr<float>(y);
        for (int x = x0; x < x1; ++x) {
            if (row[x] != best) continue;
            if (std::hypot(x - far.x, y - far.y) > look) continue;
            half = std::max(half, static_cast<double>(dr[x]));
            sx += x;
            sy += y;
            ++inLook;
        }
    }

    tip = far;
    half = std::min(half, maxSetbackPx);
    if (half > 0.0 && inLook > 0) {
        const double ax = far.x - sx / static_cast<double>(inLook);
        const double ay = far.y - sy / static_cast<double>(inLook);
        const double n = std::hypot(ax, ay);
        if (n > 1e-6) {
            tip.x -= half * ax / n;
            tip.y -= half * ay / n;
        }
    }
    return true;
}

bool FingerTracker::locate(const cv::Mat& curSmall, const cv::Mat& refSmall, const Config& cfg,
                           double lookPx, double setbackPx, int bridgePx, double minAreaPx,
                           const cv::Point2d* previous, cv::Mat& mask, cv::Point2d& tipSmall) {
    if (curSmall.empty() || curSmall.size() != refSmall.size() ||
        curSmall.type() != refSmall.type()) {
        return false;
    }

    // The same segmentation the rest of the pipeline uses, pointed at a
    // different pair of pictures: the settled garment as the background, the
    // garment with an arm over it as the frame. Reusing it rather than writing
    // a second foreground test is deliberate - the shadow rejection is the hard
    // part, it is already solved here, and two copies of it would drift.
    const double gain = cfg.illumination_normalise ? robustGain(curSmall, refSmall) : 1.0;

    // With the hand's thresholds, not the garment's. The garment's are tuned to
    // see cloth against a dark mat and to ignore the cloth's own shadow, and
    // against them a brown hand over a salmon shirt IS a shadow: a few degrees
    // of colour change, seventy percent of the brightness. The shirt's own
    // shadow test - does it darken every channel alike? - is what tells the two
    // apart, and it is kept; the two gates in front of it are opened. See
    // finger_chroma_angle_threshold for the numbers.
    Config seg = cfg;
    seg.chroma_angle_threshold = cfg.finger_chroma_angle_threshold;
    seg.intensity_ratio_low = cfg.finger_intensity_ratio_low;
    seg.intensity_ratio_high = cfg.finger_intensity_ratio_high;

    // Closed over a fold's width, so a hand whose wrist crosses the hem's
    // shadow stays attached to the arm that reaches it. See finger_bridge_mm.
    cv::Mat score;
    double changed = 0.0;
    segmentObject(curSmall, refSmall, seg, gain, bridgePx, minAreaPx * 0.2, mask, changed, score);

    return findArmTip(mask, cfg.finger_border_margin_px, minAreaPx, tipSmall, previous, lookPx,
                      setbackPx);
}


void FingerTracker::setReference(const cv::Mat& frameBgr, const RoiBox& roi, bool hasRoi) {
    step_ = std::max(1, cfg_.detect_step);
    roi_ = roi;
    hasRoi_ = hasRoi;
    ref_ = decimatedColour(frameBgr, roi, hasRoi, step_);
    resetDwell();
}

// Hand the path over, if there is enough of one to be a seam. A trace that
// ended after a centimetre was a hand passing through, not a measurement.
void FingerTracker::finishTrace(FingerState& state) {
    double len = 0.0;
    for (std::size_t k = 1; k < path_.size(); ++k) {
        len += std::hypot(path_[k].x - path_[k - 1].x, path_[k].y - path_[k - 1].y);
    }
    tracing_ = false;
    absent_ = 0;
    still_ = false;
    stillFrames_ = 0;
    haveSeen_ = false;
    lost_ = 0;
    state.tracing = false;
    state.hold_fraction = 0.0;
    if (path_.size() >= 3 && len >= cfg_.curve_min_length_mm) {
        state.finished = true;
        state.path_mm = path_;
    }
    path_.clear();
}

void FingerTracker::abandonTrace() {
    tracing_ = false;
    absent_ = 0;
    still_ = false;
    stillFrames_ = 0;
    haveLast_ = false;
    haveSeen_ = false;
    lost_ = 0;
    path_.clear();
}

void FingerTracker::resetDwell() {
    dwelling_ = false;
    dwellSince_ = 0.0;
    dwellPoints_.clear();
}

FingerState FingerTracker::update(const cv::Mat& frameBgr, const PlaneModel* model, double now) {
    FingerState state;
    if (ref_.empty() || frameBgr.empty()) return state;

    const cv::Mat cur = decimatedColour(frameBgr, roi_, hasRoi_, step_);
    if (cur.size() != ref_.size() || cur.type() != ref_.type()) return state;

    const double area = static_cast<double>(cur.cols) * static_cast<double>(cur.rows);
    const double minArea = cfg_.finger_min_area_fraction * area;

    // A fingertip's size and half-width, and a fold's width, in the decimated
    // pixels findArmTip works in. Taken from the plane model's own local scale
    // so they are lengths on the mat rather than numbers that mean something
    // different on the next lens.
    double lookPx = 0.0, setbackPx = 0.0, mmPerCell = 0.0;
    int bridgePx = 3;
    if (model) {
        const std::pair<double, double> scale =
            model->localMmPerPx(cv::Point2d(0.5 * frameBgr.cols, 0.5 * frameBgr.rows));
        mmPerCell = 0.5 * (scale.first + scale.second) * step_;
        if (mmPerCell > 1e-9) {
            lookPx = cfg_.finger_tip_look_mm / mmPerCell;
            setbackPx = cfg_.finger_tip_setback_max_mm / mmPerCell;
            bridgePx = std::max(3, static_cast<int>(std::lround(cfg_.finger_bridge_mm / mmPerCell)));
        }
    }
    if (!(setbackPx > 0.0)) setbackPx = 1e9;  // uncalibrated: leave it as found

    cv::Mat mask;
    cv::Point2d tipSmall;
    if (!locate(cur, ref_, cfg_, lookPx, setbackPx, bridgePx, minArea,
                haveLast_ ? &lastTipSmall_ : nullptr, mask, tipSmall)) {
        // Taking the hand away is the other way to end a trace, and the one an
        // operator reaches for first: draw along the seam, lift off at the end.
        // A camera looking straight down cannot see a finger lift - the finger
        // is still there, a few millimetres higher - so what counts is the hand
        // leaving the picture, and it has to be gone for more than one frame,
        // because a hand passing behind its own shadow drops out for a frame
        // and comes back.
        if (tracing_ && ++absent_ >= cfg_.finger_trace_lift_frames) {
            finishTrace(state);
        }
        if (absent_ >= cfg_.finger_trace_lift_frames) haveLast_ = false;
        resetDwell();
        return state;
    }
    absent_ = 0;
    lastTipSmall_ = tipSmall;
    haveLast_ = true;

    // Kept for the replay. Shared buffers, not copies: `cur` is this frame's
    // own decimated picture and nothing else writes to it.
    seen_.ref = ref_;
    seen_.frame = cur;
    seen_.mask = mask;
    seen_.tipSmall = tipSmall;
    seen_.hasTip = true;
    seen_.mmPerCell = mmPerCell;
    seen_.step = step_;

    state.present = true;

    // Back to full-resolution pixels. A decimated pixel stands for the centre of
    // the block it covers, not its top-left corner, so the half-cell goes back
    // on - the same correction the live outline needs on the operator's feed,
    // and for the same reason.
    const double halfCell = 0.5 * (step_ - 1);
    const double offX = hasRoi_ && roi_.valid() ? roi_.x1 : 0.0;
    const double offY = hasRoi_ && roi_.valid() ? roi_.y1 : 0.0;
    state.tip_px = cv::Point2d(tipSmall.x * step_ + halfCell + offX,
                               tipSmall.y * step_ + halfCell + offY);

    if (!model) {
        resetDwell();
        return state;
    }
    const std::vector<cv::Point2d> mm = model->pxToMm({state.tip_px});
    if (mm.empty()) {
        resetDwell();
        return state;
    }
    state.tip_mm = mm.front();
    state.has_mm = true;

    // Take the parallax out of it.
    //
    // The tip is a fingertip's height above the plane the model is fitted to,
    // so it images displaced outward from the middle of the picture in
    // proportion to how far out it already is. Scaling it back toward the
    // middle by (D - h) / D is the same correction measure.cpp applies to a
    // thick object's size, applied to a single point instead.
    //
    // Skipped when the working distance is not known, because a correction
    // applied with a guessed distance is a second error rather than the removal
    // of a first one.
    if (cfg_.camera_distance_mm.has_value() &&
        *cfg_.camera_distance_mm > cfg_.finger_height_mm && cfg_.finger_height_mm > 0.0) {
        const double d = *cfg_.camera_distance_mm;
        const std::vector<cv::Point2d> axis =
            model->pxToMm({cv::Point2d(0.5 * frameBgr.cols, 0.5 * frameBgr.rows)});
        if (!axis.empty()) {
            const double k = (d - cfg_.finger_height_mm) / d;
            state.tip_mm = cv::Point2d(axis[0].x + (state.tip_mm.x - axis[0].x) * k,
                                       axis[0].y + (state.tip_mm.y - axis[0].y) * k);
        }
    }

    if (tracing_) {
        state.tracing = true;

        // Could a hand have got here from where it was? The tip is found afresh
        // on every frame from a blob and a rule, and a blob can be wrong - a
        // fold that lifted when the hand brushed it, the far corner of a
        // shadow, the elbow on a frame where the hand went dark. Any of those
        // is a point a hundred millimetres off the seam, and one is enough:
        // the corridor is built along the path, and a spike in the path is a
        // spike in the corridor that no amount of snapping takes out.
        //
        // So a move faster than a drawing hand is not recorded. The distance
        // is judged against the actual time since the last believed tip,
        // which is what makes this mean the same thing at two frames a second
        // as at ten, and it is forgiven after finger_lost_frames in a row,
        // because two frames agreeing on a new place is the hand being there.
        if (haveSeen_ && cfg_.finger_max_speed_mm_s > 0.0) {
            const double dt = std::max(1e-3, now - seenAt_);
            const double moved =
                std::hypot(state.tip_mm.x - seenMm_.x, state.tip_mm.y - seenMm_.y);
            if (moved > cfg_.finger_max_speed_mm_s * dt && ++lost_ < cfg_.finger_lost_frames) {
                state.rejected = true;
                state.trace_mm = path_;
                return state;
            }
        }
        lost_ = 0;
        haveSeen_ = true;
        seenMm_ = state.tip_mm;
        seenAt_ = now;

        // Standing still ends a trace only where a bench asks for it. It is
        // timed separately from the hold that starts one, because while
        // drawing a seam a hand slows almost to a stop all the time - at a
        // corner, at the deepest part of an arc, whenever the operator looks
        // up - and a hold short enough to be worth having would end the trace
        // halfway along the collar. The default is off: taking the hand away
        // ends a trace, and that is one wait rather than two.
        if (cfg_.finger_trace_end_seconds > 0.0) {
            const double drift = std::hypot(state.tip_mm.x - stillAt_.x,
                                            state.tip_mm.y - stillAt_.y);
            if (!still_ || drift > cfg_.finger_dwell_radius_mm) {
                still_ = true;
                stillSince_ = now;
                stillFrames_ = 0;
                stillAt_ = state.tip_mm;
            }
            ++stillFrames_;
            const double held = now - stillSince_;
            state.hold_fraction = std::min(1.0, held / cfg_.finger_trace_end_seconds);

            // Frames as well as seconds, for the same reason the starting hold
            // counts them: on a camera running at two frames a second, three
            // seconds of stillness that nobody actually watched is not
            // stillness.
            if (held >= cfg_.finger_trace_end_seconds && stillFrames_ >= 3) {
                path_.push_back(state.tip_mm);
                finishTrace(state);
                return state;
            }
        }
        // Recorded when the tip has actually moved, so a hand resting still
        // does not fill the path with a thousand copies of one place. Two
        // millimetres is finer than any wobble the corridor cares about and
        // coarse enough that a slow, careful trace is not mostly jitter.
        if (path_.empty() ||
            std::hypot(state.tip_mm.x - path_.back().x, state.tip_mm.y - path_.back().y) >
                cfg_.finger_trace_step_mm) {
            path_.push_back(state.tip_mm);
        }
        state.trace_mm = path_;

        // Nothing below applies while a trace is running, and letting it run
        // was a real bug rather than untidiness: the hold that STARTS a trace
        // went on firing during one, and each time it fired it started the
        // trace again - clearing the path and resetting the stillness clock. A
        // trace could then never get past about a second and a third, so it
        // never ended by standing still and never handed over more than the
        // last scrap of itself.
        return state;
    }

    if (now < rearmAt_) {
        // Still lifting away from the last point. Holding the tracker off here
        // rather than in the caller means one steady finger cannot lay down
        // twenty seeds in the same place, however the caller is written.
        resetDwell();
        return state;
    }

    const double moved = std::hypot(state.tip_mm.x - anchorMm_.x, state.tip_mm.y - anchorMm_.y);
    if (!dwelling_ || moved > cfg_.finger_dwell_radius_mm) {
        dwelling_ = true;
        dwellSince_ = now;
        anchorMm_ = state.tip_mm;
        dwellPoints_.clear();
    }
    dwellPoints_.push_back(state.tip_mm);

    state.dwelling = true;
    state.dwell_s = now - dwellSince_;
    state.dwell_fraction =
        std::min(1.0, state.dwell_s / std::max(1e-6, cfg_.finger_dwell_seconds));

    // Long enough AND enough frames of it.
    //
    // A wall-clock hold alone is not a hold that anybody checked. This bench
    // runs at 1.9 frames per second, where four tenths of a second is barely
    // one frame: the tip was "held still" on the strength of having been seen
    // once, points landed about every second the hand stayed near the seam,
    // and a step that wanted three of them collected thirteen. Counting frames
    // as well makes the requirement self-adjusting - the four tenths on a
    // camera keeping up, a deliberate second on one that is not, and never a
    // commitment made on a single sighting.
    const std::size_t minFrames = static_cast<std::size_t>(std::max(1, cfg_.finger_dwell_frames));
    if (state.dwell_s >= cfg_.finger_dwell_seconds && dwellPoints_.size() >= minFrames) {
        // The mean over the whole dwell, not the last position. Half a second
        // of frames averages out most of the shake and costs nothing, and the
        // seed is worth having as good as it comes for free even though the
        // corridor would forgive far worse.
        double sx = 0.0, sy = 0.0;
        for (const cv::Point2d& p : dwellPoints_) {
            sx += p.x;
            sy += p.y;
        }
        const double n = static_cast<double>(dwellPoints_.size());
        state.committed = true;
        state.point_mm = cv::Point2d(sx / n, sy / n);
        rearmAt_ = now + cfg_.finger_rearm_seconds;
        resetDwell();

        // One hold starts the trace, the next one ends it. The dwell test
        // cannot fire while the hand is moving along the seam, so the gesture
        // needs nothing the tracker was not already doing: hold, draw, hold.
        // A hold only ever STARTS a trace. What ends one is standing still for
        // a good deal longer, or taking the hand away.
        tracing_ = true;
        absent_ = 0;
        still_ = false;
        stillFrames_ = 0;
        lost_ = 0;
        haveSeen_ = true;
        seenMm_ = state.point_mm;
        seenAt_ = now;
        path_.clear();
        path_.push_back(state.point_mm);
        state.tracing = true;
        state.trace_mm = path_;
    }

    return state;
}

}  // namespace verimeasure
