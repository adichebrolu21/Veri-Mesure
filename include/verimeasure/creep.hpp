// Where the base has moved to since the background was taken.
//
// The background is a photograph of the base taken while it was clear, and the
// fine pass finds the garment as whatever is different from it. That only
// works while the base is still where it was photographed. A cloth base is not:
// laying a garment on it drags it, a few pixels to a couple of millimetres, and
// not evenly - on 2026-09-25 a sleeve laid on the blue cloth moved the cloth
// 4 to 11 px sideways on one side of it and 12 to 19 px down on the other. Every
// printed line then stands a few pixels from where the background has it,
// which is a difference, and a difference is garment: the lines beside the
// garment were joined to it and the outline ran out along the grid, up to 80 mm
// past the garment's edge.
//
// This measures that movement on the bare base round the lay and resamples the
// background to where the base is now, so the fine pass compares the garment
// with the base it is actually lying on. Where the base has not moved it does
// nothing at all.

#pragma once

#include <opencv2/core.hpp>

#include "verimeasure/config.hpp"

namespace verimeasure {

struct BaseCreep {
    // Patches of bare base the movement was read from, after the ones that
    // disagreed with their neighbours were dropped.
    int patches = 0;
    // The background was resampled: at least base_creep_min_patches places
    // on the base had moved by base_creep_min_px or more.
    bool aligned = false;
    // The movement where the base was read, smoothed over the patches round
    // each place, in working-area pixels.
    double max_px = 0.0;
    double median_px = 0.0;
};

// frameBgr and backgroundBgr are the whole working area at full resolution.
// lay is the lay as the decimated pass saw it - filled, nonzero on the garment -
// in cells of `step` working-area pixels. crop is the region the fine pass
// measures, in working-area pixels.
//
// When the base is found to have moved by base_creep_min_px or more in at least
// base_creep_min_patches places, alignedCrop is set to the background over the
// crop resampled to where the base is now, and the result says aligned.
// Otherwise alignedCrop is left as it was and the caller measures against the
// background untouched.
BaseCreep alignCreptBase(const cv::Mat& frameBgr, const cv::Mat& backgroundBgr,
                         const cv::Mat& lay, int step, const cv::Rect& crop, const Config& cfg,
                         cv::Mat& alignedCrop);

}  // namespace verimeasure
