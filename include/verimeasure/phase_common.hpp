// Helpers shared between the Phase 2 and Phase 3+4 stand-alone tools, matching
// the functions phase3_4 imported from phase2 in the Python original.

#pragma once

#include <opencv2/core.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/calibration.hpp"

namespace verimeasure {

// Centred ROI rectangle covering `fraction` of the frame's width and height.
inline RoiBox computeRoiBox(int frameWidth, int frameHeight, double fraction) {
    const int roiW = static_cast<int>(frameWidth * fraction);
    const int roiH = static_cast<int>(frameHeight * fraction);
    RoiBox box;
    box.x1 = (frameWidth - roiW) / 2;
    box.y1 = (frameHeight - roiH) / 2;
    box.x2 = box.x1 + roiW;
    box.y2 = box.y1 + roiH;
    return box;
}

// Crop the frame to the ROI and convert to grayscale for comparison.
inline cv::Mat extractRoiGray(const cv::Mat& frame, const RoiBox& box) {
    const cv::Mat roi = frame(box.toRect());
    cv::Mat gray;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

}  // namespace verimeasure
