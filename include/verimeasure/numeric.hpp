// Small numeric helpers. These replace the handful of NumPy routines the
// pipeline depends on, with the same conventions: median averages the two
// middle values on an even count, and percentile interpolates linearly between
// the bracketing order statistics.

#pragma once

#include <cstdint>
#include <functional>
#include <opencv2/core.hpp>
#include <vector>

namespace verimeasure {

// Median of a copy of the input. Empty input returns 0.
double median(std::vector<double> values);
double medianOf(const float* data, std::size_t n);

// Linear-interpolation percentile, q in [0, 100].
double percentile(std::vector<double> values, double q);
double percentileOf(std::vector<float> values, double q);

// Compact Levenberg-Marquardt with a numerical Jacobian. Kept local so the
// module has no dependency beyond OpenCV, which matters when this ships to a
// machine that only has the Arena SDK environment installed.
//
// residual(x, r) must resize r and fill it. Returns the final cost and leaves
// the solution in x.
using ResidualFn = std::function<void(const std::vector<double>&, std::vector<double>&)>;
double levenbergMarquardt(const ResidualFn& residual, std::vector<double>& x,
                          int iterations = 80, double eps = 1e-6);

// Bilinear sample of a float32 single-channel image at float coordinates.
// valid[i] is false where the 2x2 footprint falls outside the image; the value
// is still computed from clamped coordinates, matching the NumPy original.
void bilinearSample(const cv::Mat& img, const std::vector<double>& xs,
                    const std::vector<double>& ys, std::vector<double>& out,
                    std::vector<unsigned char>& valid);

// One-dimensional linear interpolation, clamped at both ends (np.interp).
void interp(const std::vector<double>& targets, const std::vector<double>& xp,
            const std::vector<double>& fp, std::vector<double>& out);

// Truncating float-to-uint8 conversion. NumPy's astype(uint8) truncates toward
// zero where cv::Mat::convertTo rounds, and a one-level shift here moves the
// Otsu threshold, so the difference is worth preserving.
cv::Mat toU8Truncate(const cv::Mat& src32f);

// Every step-th row and column, as NumPy's img[::step, ::step] would give.
cv::Mat strideSample(const cv::Mat& src, int step);

// -- the same results, computed faster --
//
// Everything below returns bit-for-bit what the obvious call above it returns.
// None of it is an approximation, a decimation or a relaxed tolerance: the
// arithmetic performed on each value is identical, only the order in which
// independent pieces of work are scheduled changes. That is the whole point.
// Calibration decides what a millimetre is, so it may get quicker but it may
// not get different.

// cv::GaussianBlur, split across cores over column strips of the same image.
//
// OpenCV's own Gaussian is single-threaded here, and at the kernel sizes this
// pipeline uses (several hundred taps) that leaves nineteen of twenty cores
// idle for seconds at a time. Each strip is a view INTO src rather than a copy
// of it, so the filter reads the true neighbouring pixels across every strip
// boundary and reflects only at the real image border, exactly as the
// full-frame call does. Same kernel, same taps, same order of accumulation
// within each output pixel; only different pixels go to different cores.
//
// The strip width is chosen so the column pass's working set, one ring of
// ksize rows, stays inside L2. Widen it and the ring spills to L3, which costs
// more than the threading saves: at sigma 133 an over-wide strip measured six
// times slower than the single-threaded call it was meant to beat.
void gaussianBlurFast(const cv::Mat& src, cv::Mat& dst, double sigma);

// Rows [y0, y1) and columns [x0, x1) of what gaussianBlurFast would produce
// over the whole image, computed from only the pixels those rows or columns
// actually depend on.
//
// For a caller that reads a narrow band through the middle of the frame, this
// is most of the work skipped rather than shared out. The halo is taken from
// the parent image, so a band computed this way is not merely close to the
// corresponding band of the full blur, it is the same bytes.
cv::Mat gaussianBlurRowBand(const cv::Mat& src, int y0, int y1, double sigma);
cv::Mat gaussianBlurColBand(const cv::Mat& src, int x0, int x1, double sigma);

// Order statistics over a CV_32F image, without materialising the copy.
//
// medianOfMat(m) equals median() over every pixel widened to double.
// medianAbsDevFromMat(m, c) equals median() over |pixel - c|.
// percentileOfMat(m, q) equals percentileOf() over every pixel.
//
// "Equals" here means the same double, not a close one. A histogram over the
// leading sixteen bits of the IEEE pattern locates the bucket each rank falls
// in, and only that bucket's members are gathered and finished with
// nth_element. Values are compared and counted, never summed or scaled, so no
// rounding is introduced and ties cannot be resolved differently. What it
// avoids is the 160 MB vector<double> the direct route builds and sorts, three
// times over, on every calibration.
double medianOfMat(const cv::Mat& m32f);
double medianAbsDevFromMat(const cv::Mat& m32f, double centre);
double percentileOfMat(const cv::Mat& m32f, double q);

}  // namespace verimeasure
