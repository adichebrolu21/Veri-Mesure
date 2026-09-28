// Shadow-invariant foreground segmentation.
//
// A shadow scales all three channels by roughly the same factor, so the colour
// vector's direction is preserved and only its length shrinks. A real object
// changes the direction. Testing the angle between the current and background
// BGR vectors therefore separates object from shadow, which a grayscale
// magnitude threshold cannot do.

#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "verimeasure/config.hpp"

namespace verimeasure {

// Drop components below a size floor before any gap bridging.
//
// Bridging exists to rejoin strips of one part. Run on a mask that still has
// scattered noise in it, it does the opposite: it glues the noise to the part
// and the outline balloons to enclose everything. Clearing the small stuff
// first means bridging can only ever join things large enough to be real.
cv::Mat removeSpeckle(const cv::Mat& mask, double minAreaPx);

// Full-resolution segmentation.
//
// bridgePx closes gaps in the mask. Where an object happens to match part of
// the printed mat in both brightness and colour, the difference legitimately
// falls to nothing along those lines and the mask is cut into strips. Bridging
// by a fraction of the grid pitch closes the printed lines without merging
// genuinely separate objects.
//
// score is the foreground evidence kept as a float image for the sub-pixel edge
// fit. The printed grid appears in both frames and cancels there, which is the
// whole point: refining against the raw image lets a grid line a few pixels
// outside the object win the gradient search and drag the boundary outward.
//
// darkerGarment says the garment on the base is darker than the base (see
// darkerThanBase). The shadow vote then leaves alone any pixel that shows, on
// its own evidence, that it is not the base; otherwise nothing changes.
//
// blackCeiling, with darkerGarment, says the garment is near black (see
// blackGarmentCeiling): a pixel at most that bright, lightly smoothed, is the
// garment whatever else is said about it, and a darker pixel brighter than
// that is left to the vote, witness or not. 0 leaves the rule above.
void segmentObject(const cv::Mat& roiBgr, const cv::Mat& backgroundBgr, const Config& cfg,
                   double gain, int bridgePx, double speckleMinPx, cv::Mat& mask,
                   double& changedFraction, cv::Mat& score, bool darkerGarment = false,
                   double blackCeiling = 0.0);

// Whether what the mask covers is darker than the base under it: the median
// luminance ratio over the covered samples, against the gain-scaled
// background, below `ratio`. False with nothing covered.
bool darkerThanBase(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& mask, double gain,
                    double ratio);

// How bright a pixel of the lay may be and still be the garment itself, when
// the garment is near black; 0 when it is not, or when the bench has turned
// this off (Config::dark_garment_black_share). From what the mask covers, over
// the samples darkerThanBase takes: the lay's middle gray level and the gain-
// scaled base's under it.
double blackGarmentCeiling(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& mask,
                           double gain, const Config& cfg);

// Global illumination gain between two frames, as the median of the per-pixel
// luminance ratio.
//
// Comparing medians of the two frames separately looked equivalent and was not.
// The mat is bimodal, dark cells against bright printed lines, so once a pale
// part covers a fifth of the view the median slides down the distribution and
// reports a large brightness change that never happened. The median of the
// ratios holds up far better, but it is still a median of everything in view,
// and a median breaks at half. On a real flat lay it read 1.04 with 45% of the
// view covered, 1.12 at 50%, 1.6 at 52% and its 2.0 clamp from 58%; a dark
// garment drives it to the 0.5 clamp just past half. From there on it measures
// the garment, not the light, which is why the engine reads it again from the
// mat alone (matGain) once the view is that full.
double robustGain(const cv::Mat& curBgr, const cv::Mat& bgBgr);

// The same median, over the mat alone: the samples outside every object in
// objectMask, filled and grown by marginPx. False, with gain untouched, when
// fewer than minFraction of the samples are left.
bool matGain(const cv::Mat& curBgr, const cv::Mat& bgBgr, const cv::Mat& objectMask,
             int marginPx, double minFraction, double& gain);

// Largest external contour above the area floor. Returns false when nothing
// qualifies.
bool largestContour(const cv::Mat& mask, double minAreaPx, std::vector<cv::Point>& contour);

}  // namespace verimeasure
