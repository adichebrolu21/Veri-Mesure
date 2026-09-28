// Automatic plane calibration.
//
// The printed grid on the empty mat is detected directly: line families are
// separated morphologically, each line is fitted to sub-pixel precision over
// its full run, intersections are solved analytically, and integer lattice
// indices are assigned. Typically several hundred to a few thousand
// correspondences, with no clicking.
//
// The model is a plane homography plus radial distortion, not a scale factor.
// Ten parameters fitted to thousands of observations, which handles
// perspective, a non-perpendicular mount, and lens distortion in one step.

#include "verimeasure/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>
#include <numeric>
#include <random>
#include <cstring>

#include "verimeasure/cv_compat.hpp"

#include "verimeasure/numeric.hpp"

namespace verimeasure {
namespace {

constexpr double kPi = 3.14159265358979323846;

// How far, as a share of one printed square, the spacing the lines were
// indexed at may sit from the pitch and still count as that square.
constexpr double kStepTolerance = 0.28;

struct FittedLine {
    cv::Vec3d coef{0.0, 0.0, 0.0};  // v = c0 u^2 + c1 u + c2
    double u_min = 0.0;
    double u_max = 0.0;
    int support = 0;
    double rms = 0.0;
};

struct IndexAssignment {
    bool ok = false;
    std::vector<int> selected;  // indices into the original offset array
    std::vector<int> indices;   // integer lattice index per selected line
    double base = 0.0;
};

double lineValue(const cv::Vec3d& coef, double u) {
    return coef[0] * u * u + coef[1] * u + coef[2];
}

// Ridge image: a high-pass that keeps the printed lines and drops the shading.
cv::Mat ridgeImage(const cv::Mat& gray32f, double sigma) {
    cv::Mat bg;
    gaussianBlurFast(gray32f, bg, sigma);
    cv::Mat ridge;
    cv::subtract(gray32f, bg, ridge);
    cv::max(ridge, 0.0, ridge);
    return ridge;
}

// Find the FINEST printed pitch on each axis, not the strongest period.
//
// Autocorrelation is the obvious tool here and it is the wrong one. These mats
// print a heavy line every fifth square, so the 50 mm period dominates the
// correlogram and the estimate comes back five times too large. That then sizes
// the morphological kernel too long, wipes out the fine lines, and the whole
// chain resolves 50 mm spacing as 10 mm. Every measurement would read one fifth
// of its true size, with a clean-looking residual to go with it.
//
// Counting the gaps between ridge peaks in a narrow central band avoids that
// entirely: every printed line contributes a peak regardless of its weight, so
// the median gap is the fine pitch by construction.
// The only two parts of the coarse ridge anyone looks at: a horizontal strip
// and a vertical strip through the middle of the frame, one per axis. Stated
// once, so the code that builds the ridge and the code that reads it cannot
// drift apart and quietly start profiling uncomputed pixels.
void pitchProfileBands(int h, int w, int& y0, int& y1, int& x0, int& x1) {
    const int bandH = std::max(40, static_cast<int>(h * 0.06));
    y0 = std::max(0, h / 2 - bandH / 2);
    y1 = std::min(h, y0 + bandH);
    const int bandW = std::max(40, static_cast<int>(w * 0.06));
    x0 = std::max(0, w / 2 - bandW / 2);
    x1 = std::min(w, x0 + bandW);
}

// Which way round the grid is printed.
//
// Everything downstream finds the grid as ridges brighter than their
// surroundings - the positive part of the picture less its blur - because the
// cutting mat this was written for is near black with its lines printed pale.
// The printed cloth that replaced it on this bench in September 2026 is the
// other way round: dark lines on mid blue. Handed that, the ridge is the inside
// of every square and not a single line is found, so Recalibrate failed with
// "too few grid lines (h=0, v=0)" however the cloth was lit.
//
// Deciding which it is cannot be done pixel by pixel. The cloth's weave throws
// small bright glints, and the creases it was folded with are pale ridges, and
// between them they outnumber the extremes of the dark lines themselves -
// measured on the bench, a local high-pass called the cloth "bright lines" as
// firmly as it called the old mat that. So the same two strips through the
// middle that the pitch is read from are each averaged along their length
// first. Weave and glints average away, a crease is one feature among eighty
// lines, and what is left is the grid's own profile: a comb of narrow dips on a
// dark-lined print, a comb of narrow peaks on a bright-lined one. The deeper
// side wins, by a margin.
//
// Measured: the cloth's dips run 1.94 to 1.99 times deeper than its peaks at
// every exposure tried; the old mat's 0.70 to 0.75 clear, at most 1.19 with a
// shirt across the middle; synthetic mats 0.32 to 0.48. A bright-lined mat
// therefore never crosses the line, and on one the calibration is exactly what
// it was, to the bit.
constexpr double kDarkLinesDipToPeak = 1.4;

bool printedDarkLines(const cv::Mat& gray32f, const Config& cfg, double& dipToPeak) {
    const int h = gray32f.rows;
    const int w = gray32f.cols;
    int y0 = 0, y1 = 0, x0 = 0, x1 = 0;
    pitchProfileBands(h, w, y0, y1, x0, x1);

    cv::Mat along[2];
    cv::reduce(gray32f.rowRange(y0, y1), along[0], 0, cv::REDUCE_AVG, CV_64F);
    cv::reduce(gray32f.colRange(x0, x1), along[1], 1, cv::REDUCE_AVG, CV_64F);
    along[1] = along[1].t();

    // Narrow structure only: the profile less a blur a little wider than a
    // printed line, clear of the frame's border.
    std::vector<float> detail;
    const int m = cfg.border_margin_px;
    for (const cv::Mat& p : along) {
        cv::Mat smooth;
        cv::GaussianBlur(p, smooth, cv::Size(0, 0), 8.0);
        const double* a = p.ptr<double>(0);
        const double* s = smooth.ptr<double>(0);
        for (int k = m; k < p.cols - m; ++k) detail.push_back(static_cast<float>(a[k] - s[k]));
    }
    if (detail.empty()) {
        dipToPeak = 0.0;
        return false;
    }
    const double peaks = percentileOf(detail, 99.0);
    const double dips = -percentileOf(std::move(detail), 1.0);
    dipToPeak = dips / std::max(peaks, 1e-6);
    return dipToPeak > kDarkLinesDipToPeak;
}

bool profilePitch(const cv::Mat& ridge, bool alongX, const Config& cfg, double& pitch) {
    const int h = ridge.rows;
    const int w = ridge.cols;

    int y0 = 0, y1 = 0, x0 = 0, x1 = 0;
    pitchProfileBands(h, w, y0, y1, x0, x1);

    cv::Mat prof;
    if (alongX) {
        cv::Mat band = ridge.rowRange(y0, y1);
        cv::reduce(band, prof, 0, cv::REDUCE_AVG, CV_32F);  // 1 x w
        prof = prof.reshape(1, w);                          // w x 1
    } else {
        cv::Mat band = ridge.colRange(x0, x1);
        cv::reduce(band, prof, 1, cv::REDUCE_AVG, CV_32F);  // h x 1
    }

    cv::Mat smooth;
    cv::GaussianBlur(prof, smooth, cv::Size(0, 0), 1.2);

    const int n = smooth.rows;
    const float* p = smooth.ptr<float>(0);
    double maxVal = 0.0;
    for (int k = 0; k < n; ++k) maxVal = std::max(maxVal, static_cast<double>(p[k]));
    if (maxVal <= 0.0) return false;

    // The pitch is the strongest periodicity in the profile. It is not the
    // median gap between the profile's peaks.
    //
    // Taking the median of consecutive gaps assumes every peak is a grid line,
    // and on a real cutting mat most of them are not. Alongside the grid this
    // surface carries rulers down two edges, radius arcs, diagonals, circle
    // targets, hexagons and blocks of engraved text, and every one of those
    // puts peaks into the profile at spacings of its own. The gap list then
    // holds two populations - the grid's, and the printing's - and a median
    // reports whichever is more numerous, which is not the one that was asked
    // for. Measured on this bench the grid sits at 71 px and the median gap
    // came back as 14, so the pitch was rejected as out of range and every
    // recalibration failed in under three seconds, on a mat that was perfectly
    // readable.
    //
    // Correlating the profile against itself asks the question that was
    // actually meant: at what shift does this profile look most like itself?
    // Printing that is not periodic contributes nothing to that, however many
    // peaks it has, because it only lines up with itself at one shift. On the
    // same frame the correlation peaks at 71 px and again at 142, 213 and 284 -
    // its own harmonics, which is what a real grid looks like - while 14 px
    // scores 0.02, or nothing at all.
    double mean = 0.0;
    for (int k = 0; k < n; ++k) mean += p[k];
    mean /= n;
    std::vector<double> v(static_cast<std::size_t>(n));
    double energy = 0.0;
    for (int k = 0; k < n; ++k) {
        v[static_cast<std::size_t>(k)] = p[k] - mean;
        energy += v[static_cast<std::size_t>(k)] * v[static_cast<std::size_t>(k)];
    }
    if (energy <= 0.0) return false;
    const double variance = energy / n;

    const int minLag = std::max(2, cfg.min_pitch_px);
    const int maxLag = std::min(n / 2, cfg.max_pitch_px);
    if (maxLag <= minLag + 1) return false;

    std::vector<double> ac(static_cast<std::size_t>(maxLag) + 2, 0.0);
    for (int lag = minLag - 1; lag <= maxLag + 1 && lag < n; ++lag) {
        if (lag < 1) continue;
        double sum = 0.0;
        const int count = n - lag;
        for (int k = 0; k < count; ++k) {
            sum += v[static_cast<std::size_t>(k)] * v[static_cast<std::size_t>(k + lag)];
        }
        // Per-lag, so a long shift is not penalised for having fewer terms to
        // add up and the comparison between lags stays fair.
        ac[static_cast<std::size_t>(lag)] = (sum / count) / variance;
    }

    double best = 0.0;
    for (int lag = minLag; lag <= maxLag; ++lag) best = std::max(best, ac[static_cast<std::size_t>(lag)]);
    // Nothing in this picture repeats. That is a surface with no grid on it,
    // not a grid whose pitch needs reporting.
    if (best < 0.10) return false;

    // The smallest strong peak, not the tallest. A grid correlates with itself
    // at its pitch and at every multiple of it, and those multiples are often
    // the taller peaks; taking the tallest would report a pitch two or three
    // times the real one and put every lattice index out by that factor.
    int fundamental = -1;
    for (int lag = minLag; lag <= maxLag; ++lag) {
        const std::size_t i = static_cast<std::size_t>(lag);
        if (ac[i] >= 0.80 * best && ac[i] >= ac[i - 1] && ac[i] >= ac[i + 1]) {
            fundamental = lag;
            break;
        }
    }
    if (fundamental < 0) return false;

    // Sub-pixel, by fitting a parabola through the peak and its two
    // neighbours. A pitch is a scale factor for the whole model, so a
    // half-pixel of it is worth having.
    const std::size_t f = static_cast<std::size_t>(fundamental);
    // Not y0/y1/y2: <math.h> declares y0 and y1 as Bessel functions and the
    // shadowing is an error, not a warning.
    const double acLo = ac[f - 1];
    const double acMid = ac[f];
    const double acHi = ac[f + 1];
    const double denom = acLo - 2.0 * acMid + acHi;
    double delta = (denom != 0.0) ? 0.5 * (acLo - acHi) / denom : 0.0;
    if (!(delta > -1.0 && delta < 1.0)) delta = 0.0;

    pitch = static_cast<double>(fundamental) + delta;
    return true;
}

bool estimatePitch(const cv::Mat& ridge, const Config& cfg, double& pitchX, double& pitchY) {
    double px = 0.0;
    double py = 0.0;
    const bool okX = profilePitch(ridge, true, cfg, px);   // spacing between vertical lines
    const bool okY = profilePitch(ridge, false, cfg, py);  // spacing between horizontal lines
    if (!okX && !okY) return false;
    if (!okX) px = py;
    if (!okY) py = px;
    pitchX = px;
    pitchY = py;
    return true;
}

// Recover the two printed line directions from the gradient orientation
// histogram.
//
// This matters more than it looks. A directional opening with an axis-aligned
// kernel only works if the grid is square to the sensor. Put the mat down three
// degrees out and a 120 pixel vertical kernel drifts seven pixels sideways
// across a two pixel line, so the erosion removes the entire family and half
// the calibration data vanishes without any error being raised.
void estimateGridAngles(const cv::Mat& ridge, double& phiH, double& phiV) {
    phiH = 0.0;
    phiV = 90.0;

    cv::Mat gx, gy, mag;
    cv::Sobel(ridge, gx, CV_32F, 1, 0, 3);
    cv::Sobel(ridge, gy, CV_32F, 0, 1, 3);
    cv::magnitude(gx, gy, mag);

    if (cv::countNonZero(mag.reshape(1, 1)) == 0) return;

    const double thr = percentileOfMat(mag, 96.0);
    const double cut = std::max(thr, 1e-6);

    const int bins = 360;  // half-degree resolution
    std::vector<double> hist(bins, 0.0);
    long long selected = 0;
    for (int y = 0; y < mag.rows; ++y) {
        const float* m = mag.ptr<float>(y);
        const float* dx = gx.ptr<float>(y);
        const float* dy = gy.ptr<float>(y);
        for (int x = 0; x < mag.cols; ++x) {
            if (!(m[x] > cut)) continue;
            ++selected;
            double a = std::atan2(static_cast<double>(dy[x]), static_cast<double>(dx[x])) * 180.0 / kPi;
            a = std::fmod(a, 180.0);
            if (a < 0.0) a += 180.0;
            int idx = static_cast<int>(std::floor(a * (bins / 180.0)));
            if (idx < 0) idx = 0;
            if (idx >= bins) idx = bins - 1;
            hist[idx] += static_cast<double>(m[x]);
        }
    }
    if (selected < 500) return;

    // Circular five-tap box smoothing, as np.convolve over a tripled histogram.
    std::vector<double> tiled(3 * bins);
    for (int k = 0; k < 3 * bins; ++k) tiled[k] = hist[k % bins];
    std::vector<double> conv(3 * bins, 0.0);
    for (int k = 0; k < 3 * bins; ++k) {
        double acc = 0.0;
        for (int t = -2; t <= 2; ++t) {
            const int j = k + t;
            if (j >= 0 && j < 3 * bins) acc += tiled[j];
        }
        conv[k] = acc / 5.0;
    }
    for (int k = 0; k < bins; ++k) hist[k] = conv[bins + k];

    auto refine = [&](int i) {
        const double a = hist[(i - 1 + bins) % bins];
        const double b = hist[i];
        const double c = hist[(i + 1) % bins];
        const double denom = a - 2 * b + c;
        double off = (std::abs(denom) > 1e-9) ? 0.5 * (a - c) / denom : 0.0;
        off = std::min(1.0, std::max(-1.0, off));
        return (static_cast<double>(i) + off) * 0.5;
    };

    int g1 = static_cast<int>(std::max_element(hist.begin(), hist.end()) - hist.begin());
    int g2 = g1;
    double best = -std::numeric_limits<double>::infinity();
    for (int t = -60; t <= 60; ++t) {
        int idx = ((g1 + bins / 2 + t) % bins + bins) % bins;
        if (hist[idx] > best) {
            best = hist[idx];
            g2 = idx;
        }
    }

    double phis[2];
    const int picks[2] = {g1, g2};
    for (int k = 0; k < 2; ++k) {
        double phi = refine(picks[k]) - 90.0;  // line direction, gradient is normal
        phi = std::fmod(phi, 180.0);
        if (phi < 0.0) phi += 180.0;
        if (phi > 90.0) phi -= 180.0;
        phis[k] = phi;
    }

    // min/max by absolute value, first wins on a tie, as Python's min/max do.
    const bool firstSmaller = std::abs(phis[0]) <= std::abs(phis[1]);
    phiH = firstSmaller ? phis[0] : phis[1];
    phiV = firstSmaller ? phis[1] : phis[0];
    if (std::abs(phiV) < 45.0) phiV = phiH + 90.0;  // both peaks landed on one family
    if (phiV < 0.0) phiV += 180.0;
}

// Rotate so the target family lies along an axis, open with a fast rectangular
// kernel, then rotate the mask back. The rotation only ever touches the mask.
// Every sub-pixel line fit later runs against the original, unresampled ridge
// image, so no interpolation error enters the calibration.
cv::Mat directionalOpen(const cv::Mat& binary, double angleDeg, int klen, bool horizontal) {
    const int h = binary.rows;
    const int w = binary.cols;

    if (std::abs(angleDeg) < 0.05) {
        cv::Mat out;
        const cv::Size k = horizontal ? cv::Size(klen, 1) : cv::Size(1, klen);
        cv::morphologyEx(binary, out, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, k));
        const cv::Size bridge = horizontal ? cv::Size(klen * 3, 1) : cv::Size(1, klen * 3);
        cv::Mat closed;
        cv::morphologyEx(out, closed, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_RECT, bridge));
        return closed;
    }

    const cv::Point2f centre(static_cast<float>(w / 2.0), static_cast<float>(h / 2.0));
    cv::Mat M = cv::getRotationMatrix2D(centre, angleDeg, 1.0);
    const double cosv = std::abs(M.at<double>(0, 0));
    const double sinv = std::abs(M.at<double>(0, 1));
    const int nw = static_cast<int>(h * sinv + w * cosv) + 2;
    const int nh = static_cast<int>(h * cosv + w * sinv) + 2;
    M.at<double>(0, 2) += nw / 2.0 - centre.x;
    M.at<double>(1, 2) += nh / 2.0 - centre.y;

    cv::Mat rot;
    cv::warpAffine(binary, rot, M, cv::Size(nw, nh), cv::INTER_NEAREST);

    const cv::Size k = horizontal ? cv::Size(klen, 1) : cv::Size(1, klen);
    cv::Mat opened;
    cv::morphologyEx(rot, opened, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, k));

    // Bridge along the line direction only. Perspective bends a long line enough
    // that the opening clips it into pieces, and a set of short fragments looks
    // like noise to the length filter even though it is one printed line.
    const cv::Size bridge = horizontal ? cv::Size(klen * 3, 1) : cv::Size(1, klen * 3);
    cv::morphologyEx(opened, opened, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, bridge));

    cv::Mat Minv;
    cv::invertAffineTransform(M, Minv);
    cv::Mat back;
    cv::warpAffine(opened, back, Minv, cv::Size(w, h), cv::INTER_NEAREST);

    // Nearest-neighbour resampling punches occasional single-pixel holes in a two
    // pixel wide line, which splits one grid line into a dozen components and
    // then loses all of them to the minimum-length filter. Grow the surviving
    // mask and intersect it back with the original binary: connectivity is
    // restored, and the pixel support stays exactly what the camera saw.
    cv::Mat grown;
    cv::dilate(back, grown, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    cv::Mat result;
    cv::bitwise_and(grown, binary, result);
    return result;
}

// Each connected component of the family mask is one printed grid line. Fit a
// weighted quadratic along its run rather than a straight line: real lines bend
// slightly under lens distortion, and forcing a straight fit through curved
// evidence biases every intersection derived from it.
//
// axis 0: line runs left to right, v = f(u) with u = x.
// axis 1: line runs top to bottom, v = f(u) with u = y.
std::vector<FittedLine> fitLineFamily(const cv::Mat& mask, const cv::Mat& ridge, int axis,
                                      double minLength) {
    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

    // One component is one printed line, and no line's fit depends on any
    // other's, so they go out across the cores. Each writes to its own slot and
    // the slots are collected in component order afterwards, so the list handed
    // back is the same list, in the same order, that fitting them one after
    // another produced.
    const int slots = std::max(0, n - 1);
    std::vector<FittedLine> fitted(static_cast<std::size_t>(slots));
    std::vector<unsigned char> accepted(static_cast<std::size_t>(slots), 0);

    cv::parallel_for_(cv::Range(1, n), [&](const cv::Range& range) {
        std::vector<double> us, vs, ws;

        for (int i = range.start; i < range.end; ++i) {
            const int x = stats.at<int>(i, cv::CC_STAT_LEFT);
            const int y = stats.at<int>(i, cv::CC_STAT_TOP);
            const int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
            const int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            const int area = stats.at<int>(i, cv::CC_STAT_AREA);

            const int length = (axis == 0) ? w : h;
            const int thickness = (axis == 0) ? h : w;
            if (length < minLength || area < minLength * 0.4) continue;
            if (thickness > length * 0.25) continue;

            us.clear();
            vs.clear();
            ws.clear();
            for (int yy = y; yy < y + h; ++yy) {
                const int* lab = labels.ptr<int>(yy);
                const float* rid = ridge.ptr<float>(yy);
                for (int xx = x; xx < x + w; ++xx) {
                    if (lab[xx] != i) continue;
                    const double uu = (axis == 0) ? static_cast<double>(xx) : static_cast<double>(yy);
                    const double vv = (axis == 0) ? static_cast<double>(yy) : static_cast<double>(xx);
                    us.push_back(uu);
                    vs.push_back(vv);
                    ws.push_back(static_cast<double>(rid[xx]) + 1e-3);
                }
            }
            if (us.size() < 20) continue;

            const int m = static_cast<int>(us.size());
            cv::Mat A(m, 3, CV_64F);
            cv::Mat b(m, 1, CV_64F);
            for (int k = 0; k < m; ++k) {
                const double weight = ws[k];
                A.at<double>(k, 0) = us[k] * us[k] * weight;
                A.at<double>(k, 1) = us[k] * weight;
                A.at<double>(k, 2) = weight;
                b.at<double>(k, 0) = vs[k] * weight;
            }

            cv::Mat coefMat;
            if (!cv::solve(A, b, coefMat, cv::DECOMP_SVD)) continue;
            const cv::Vec3d coef(coefMat.at<double>(0, 0), coefMat.at<double>(1, 0),
                                 coefMat.at<double>(2, 0));

            double sq = 0.0;
            for (int k = 0; k < m; ++k) {
                const double pred = lineValue(coef, us[k]);
                sq += (pred - vs[k]) * (pred - vs[k]);
            }
            const double rms = std::sqrt(sq / m);
            if (rms > 2.5) continue;  // not a clean printed line

            // Reject anything that is not close to axis aligned. Cutting mats carry
            // printed diagonals, radius arcs and angle fans, and a long-radius arc
            // can survive a directional opening. Grid lines run near flat; those do
            // not.
            const auto minmax = std::minmax_element(us.begin(), us.end());
            const double uMin = *minmax.first;
            const double uMax = *minmax.second;
            const double uMid = 0.5 * (uMin + uMax);
            const double slope = std::abs(2.0 * coef[0] * uMid + coef[1]);
            if (slope > 0.30) continue;

            FittedLine line;
            line.coef = coef;
            line.u_min = uMin;
            line.u_max = uMax;
            line.support = m;
            line.rms = rms;
            fitted[static_cast<std::size_t>(i - 1)] = line;
            accepted[static_cast<std::size_t>(i - 1)] = 1;
        }
    });

    std::vector<FittedLine> lines;
    for (int i = 0; i < slots; ++i) {
        if (accepted[static_cast<std::size_t>(i)]) lines.push_back(fitted[static_cast<std::size_t>(i)]);
    }
    return lines;
}

// Turn sorted line offsets into integer lattice indices. Gaps are classified as
// integer multiples of a refined base spacing, which tolerates missing lines and
// the gentle spacing drift that perspective produces. Lines whose gap is not
// close to an integer multiple are dropped rather than guessed at.
//
// The refinement starts from the median gap, unless seedBase says where to
// start instead; see the retry in detectLattice for why it sometimes must.
IndexAssignment assignIndices(const std::vector<double>& offsets, const Config& cfg,
                              double seedBase = 0.0) {
    IndexAssignment out;
    if (offsets.size() < 4) return out;

    std::vector<int> order(offsets.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return offsets[a] < offsets[b]; });

    std::vector<double> o(order.size());
    for (std::size_t k = 0; k < order.size(); ++k) o[k] = offsets[order[k]];
    if (o.size() < 4) return out;

    // One printed line occasionally survives as two components and gets fitted
    // twice. Near-duplicate offsets contribute nothing and confuse the spacing
    // statistics, so collapse them first.
    std::vector<std::size_t> unique{0};
    for (std::size_t i = 1; i < o.size(); ++i) {
        if (o[i] - o[unique.back()] > 2.0) unique.push_back(i);
    }
    {
        std::vector<int> order2;
        std::vector<double> o2;
        order2.reserve(unique.size());
        o2.reserve(unique.size());
        for (std::size_t idx : unique) {
            order2.push_back(order[idx]);
            o2.push_back(o[idx]);
        }
        order.swap(order2);
        o.swap(o2);
    }
    if (o.size() < 4) return out;

    std::vector<double> gaps;
    for (std::size_t i = 1; i < o.size(); ++i) {
        const double g = o[i] - o[i - 1];
        if (g > 1.0) gaps.push_back(g);
    }
    if (gaps.size() < 3) return out;

    double base = seedBase > 0.0 ? seedBase : median(gaps);
    for (int iter = 0; iter < 6; ++iter) {
        std::vector<double> ratios;
        bool any = false;
        for (double g : gaps) {
            const double mult = std::max(1.0, std::nearbyint(g / base));
            if (std::abs(g / base - mult) < 0.3) {
                ratios.push_back(g / mult);
                any = true;
            }
        }
        if (!any) break;
        const double baseNew = median(ratios);
        if (std::abs(baseNew - base) < 1e-6) {
            base = baseNew;
            break;
        }
        base = baseNew;
    }
    // A seed may be refined, not replaced. When enough fragments sit at a
    // fraction of a square to drag the refinement out of the band the spacing
    // check accepts, they have captured it, and the seed is the better start.
    if (seedBase > 0.0 && std::abs(base / seedBase - 1.0) > kStepTolerance) base = seedBase;

    // Perspective makes the spacing drift steadily from one side of the frame to
    // the other. Against a single global spacing, the far edge of the mat rounds
    // to the wrong integer index and poisons the fit with correspondences that
    // look plausible and are wrong by a whole square. Carrying a local estimate
    // that adapts as the walk proceeds keeps the rounding honest.
    double local = base;
    std::vector<int> indices{0};
    std::vector<std::size_t> kept{0};
    int cursor = 0;
    double last = o[0];
    const double tol = cfg.max_index_residual;

    for (std::size_t i = 1; i < o.size(); ++i) {
        // Measure from the last ACCEPTED line, not the last line seen. Measuring
        // from a rejected neighbour silently drops that gap from the running
        // index, so every correspondence after the first rejection is out by a
        // whole square. The fit still converges and the residual still looks
        // plausible, which is what makes it dangerous.
        const double gap = o[i] - last;
        if (gap <= 1.0) continue;
        const double step = gap / local;
        const int nearest = static_cast<int>(std::nearbyint(step));
        if (nearest < 1 || std::abs(step - nearest) > tol * std::sqrt(static_cast<double>(nearest)))
            continue;
        cursor += nearest;
        indices.push_back(cursor);
        kept.push_back(i);
        last = o[i];
        local = 0.65 * local + 0.35 * (gap / nearest);
    }

    if (kept.size() < 4) return out;

    out.ok = true;
    out.base = base;
    out.indices = indices;
    out.selected.reserve(kept.size());
    for (std::size_t idx : kept) out.selected.push_back(order[idx]);
    return out;
}

// Horizontal line y = fh(x); vertical line x = fv(y). Fixed-point iteration,
// which converges in a handful of steps because both curves are nearly axis
// aligned by construction.
void intersect(const cv::Vec3d& coefH, const cv::Vec3d& coefV, double seedX, double seedY,
               double& outX, double& outY) {
    double x = seedX;
    double y = seedY;
    for (int k = 0; k < 12; ++k) {
        const double xn = lineValue(coefV, y);
        const double yn = lineValue(coefH, xn);
        if (std::abs(xn - x) < 1e-9 && std::abs(yn - y) < 1e-9) {
            x = xn;
            y = yn;
            break;
        }
        x = xn;
        y = yn;
    }
    outX = x;
    outY = y;
}

std::vector<cv::Point2f> toPoint2f(const std::vector<cv::Point2d>& pts) {
    std::vector<cv::Point2f> out(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) {
        out[k] = cv::Point2f(static_cast<float>(pts[k].x), static_cast<float>(pts[k].y));
    }
    return out;
}

PlaneModelPtr fitModel(const std::vector<cv::Point2d>& pointsMm,
                       const std::vector<cv::Point2d>& pointsPx, cv::Point2d centre, double norm,
                       bool useDistortion, const Config& cfg) {
    const cv::Mat H0mat = cv::findHomography(toPoint2f(pointsMm), toPoint2f(pointsPx), 0);
    if (H0mat.empty()) return nullptr;

    cv::Matx33d H0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) H0(r, c) = H0mat.at<double>(r, c);
    }
    H0 = H0 * (1.0 / H0(2, 2));

    if (!useDistortion) {
        return std::make_shared<PlaneModel>(H0, 0.0, 0.0, centre, norm);
    }

    const std::size_t n = pointsMm.size();
    const double cx = centre.x;
    const double cy = centre.y;

    auto residual = [&](const std::vector<double>& p, std::vector<double>& r) {
        r.assign(2 * n, 0.0);
        for (std::size_t k = 0; k < n; ++k) {
            const double X = pointsMm[k].x;
            const double Y = pointsMm[k].y;
            const double pu = p[0] * X + p[1] * Y + p[2];
            const double pv = p[3] * X + p[4] * Y + p[5];
            double pw = p[6] * X + p[7] * Y + 1.0;
            if (std::abs(pw) < 1e-12) pw = 1e-12;
            const double u = pu / pw;
            const double v = pv / pw;
            const double dx = (u - cx) / norm;
            const double dy = (v - cy) / norm;
            const double r2 = dx * dx + dy * dy;
            const double f = 1.0 + p[8] * r2 + p[9] * r2 * r2;
            r[k] = cx + dx * norm * f - pointsPx[k].x;
            r[n + k] = cy + dy * norm * f - pointsPx[k].y;
        }
    };

    std::vector<double> x{H0(0, 0), H0(0, 1), H0(0, 2), H0(1, 0), H0(1, 1),
                          H0(1, 2), H0(2, 0), H0(2, 1), 0.0,      0.0};
    levenbergMarquardt(residual, x, cfg.lm_iterations);

    const cv::Matx33d H(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], 1.0);
    return std::make_shared<PlaneModel>(H, x[8], x[9], centre, norm);
}

// The same model with the radial terms already known: only the homography is
// fitted, to the same residual in the picture, with k1 and k2 held where the
// bench measured them.
PlaneModelPtr fitModelLocked(const std::vector<cv::Point2d>& pointsMm,
                             const std::vector<cv::Point2d>& pointsPx, const LensDistortion& lens,
                             const Config& cfg) {
    const cv::Point2d centre(lens.cx, lens.cy);
    const double norm = lens.norm;
    const double k1 = lens.k1;
    const double k2 = lens.k2;

    // Started from where each point would have been imaged by a perfect lens.
    const PlaneModel lensOnly(cv::Matx33d::eye(), k1, k2, centre, norm);
    const std::vector<cv::Point2d> ideal = lensOnly.pxToMm(pointsPx);
    const cv::Mat H0mat = cv::findHomography(toPoint2f(pointsMm), toPoint2f(ideal), 0);
    if (H0mat.empty()) return nullptr;
    cv::Matx33d H0;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) H0(r, c) = H0mat.at<double>(r, c);
    }
    H0 = H0 * (1.0 / H0(2, 2));

    const std::size_t n = pointsMm.size();
    const double cx = centre.x;
    const double cy = centre.y;
    auto residual = [&](const std::vector<double>& p, std::vector<double>& r) {
        r.assign(2 * n, 0.0);
        for (std::size_t k = 0; k < n; ++k) {
            const double X = pointsMm[k].x;
            const double Y = pointsMm[k].y;
            const double pu = p[0] * X + p[1] * Y + p[2];
            const double pv = p[3] * X + p[4] * Y + p[5];
            double pw = p[6] * X + p[7] * Y + 1.0;
            if (std::abs(pw) < 1e-12) pw = 1e-12;
            const double dx = (pu / pw - cx) / norm;
            const double dy = (pv / pw - cy) / norm;
            const double r2 = dx * dx + dy * dy;
            const double f = 1.0 + k1 * r2 + k2 * r2 * r2;
            r[k] = cx + dx * norm * f - pointsPx[k].x;
            r[n + k] = cy + dy * norm * f - pointsPx[k].y;
        }
    };
    std::vector<double> x{H0(0, 0), H0(0, 1), H0(0, 2), H0(1, 0),
                          H0(1, 1), H0(1, 2), H0(2, 0), H0(2, 1)};
    levenbergMarquardt(residual, x, cfg.lm_iterations);

    const cv::Matx33d H(x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], 1.0);
    return std::make_shared<PlaneModel>(H, k1, k2, centre, norm);
}

void rmseMm(const PlaneModel& model, const std::vector<cv::Point2d>& pointsPx,
            const std::vector<cv::Point2d>& pointsMm, double& rmse, double& worst) {
    const std::vector<cv::Point2d> pred = model.pxToMm(pointsPx);
    double sum = 0.0;
    worst = 0.0;
    for (std::size_t k = 0; k < pred.size(); ++k) {
        const double ex = pred[k].x - pointsMm[k].x;
        const double ey = pred[k].y - pointsMm[k].y;
        sum += ex * ex + ey * ey;
        worst = std::max(worst, std::sqrt(ex * ex + ey * ey));
    }
    rmse = pred.empty() ? 0.0 : std::sqrt(sum / pred.size());
}

std::vector<double> residualNorms(const PlaneModel& model, const std::vector<cv::Point2d>& px,
                                  const std::vector<cv::Point2d>& mm) {
    const std::vector<cv::Point2d> pred = model.pxToMm(px);
    std::vector<double> out(pred.size());
    for (std::size_t k = 0; k < pred.size(); ++k) {
        out[k] = std::hypot(pred[k].x - mm[k].x, pred[k].y - mm[k].y);
    }
    return out;
}

using ModelFit = std::function<PlaneModelPtr(const std::vector<cv::Point2d>& pointsMm,
                                             const std::vector<cv::Point2d>& pointsPx)>;

// Which of a lattice's points a model is fitted to.
//
// Outlier handling in three steps, and the order is the whole point.
//
// A grid line given the wrong index is out by a full square, and such a
// correspondence looks entirely plausible: the fit still converges and the
// residual still reads sensibly. Those have to go. But a strict
// homography-only screen also rejects the peripheral points, because
// distortion is largest at the frame edges, and those are precisely the
// points that carry the distortion information.
//
// So: screen hard to get a trustworthy core, fit the real model on that
// core, then re-admit from the full set anything the real model explains.
// `fit` is the real model.
void screenLattice(const std::vector<cv::Point2d>& allPx, const std::vector<cv::Point2d>& allMm,
                   double pitchPx, double gridStepMm, const Config& cfg, const ModelFit& fit,
                   std::vector<cv::Point2d>& pointsPx, std::vector<cv::Point2d>& pointsMm) {
    pointsPx = allPx;
    pointsMm = allMm;
    const double gross = std::max(3.0, 0.12 * (pitchPx > 0.0 ? pitchPx : 40.0));
    {
        cv::Mat inlierMask;
        cv::findHomography(toPoint2f(pointsMm), toPoint2f(pointsPx), cv::RANSAC, gross, inlierMask);
        if (!inlierMask.empty()) {
            std::vector<cv::Point2d> keepPx, keepMm;
            for (int k = 0; k < inlierMask.rows; ++k) {
                if (inlierMask.at<unsigned char>(k, 0)) {
                    keepPx.push_back(pointsPx[k]);
                    keepMm.push_back(pointsMm[k]);
                }
            }
            const std::size_t floorCount =
                static_cast<std::size_t>(std::max(cfg.min_lattice_points / 2, 30));
            if (keepPx.size() >= floorCount) {
                pointsPx.swap(keepPx);
                pointsMm.swap(keepMm);
            }
        }
    }

    const double stepMm = (gridStepMm > 0.0) ? gridStepMm : cfg.grid_pitch_mm;
    PlaneModelPtr probe = fit(pointsMm, pointsPx);
    for (int iter = 0; iter < 3; ++iter) {
        if (!probe) break;
        std::vector<double> coreErr = residualNorms(*probe, pointsPx, pointsMm);
        const double mad = median(coreErr) + 1e-9;
        // Comfortably tighter than one grid square, so a whole-square index
        // error can never be re-admitted however loose the noise estimate gets.
        const double thresh = std::min(0.3 * stepMm, std::max(6.0 * mad, 0.05));
        const std::vector<double> errAll = residualNorms(*probe, allPx, allMm);

        std::vector<cv::Point2d> keepPx, keepMm;
        for (std::size_t k = 0; k < errAll.size(); ++k) {
            if (errAll[k] < thresh) {
                keepPx.push_back(allPx[k]);
                keepMm.push_back(allMm[k]);
            }
        }
        const std::size_t floorCount =
            static_cast<std::size_t>(std::max(cfg.min_lattice_points, 40));
        if (keepPx.size() < floorCount) break;

        if (keepPx.size() == pointsPx.size()) {
            bool same = true;
            for (std::size_t k = 0; k < keepPx.size() && same; ++k) {
                same = (keepPx[k] == pointsPx[k]);
            }
            if (same) break;
        }
        pointsPx.swap(keepPx);
        pointsMm.swap(keepMm);
        probe = fit(pointsMm, pointsPx);
    }
}

// The bench's distortion, when there is one for a picture this size.
const LensDistortion* benchDistortionFor(const Config& cfg, cv::Size imageSize) {
    if (!cfg.lens_distortion.has_value()) return nullptr;
    const LensDistortion& d = *cfg.lens_distortion;
    if (d.width != imageSize.width || d.height != imageSize.height || !(d.norm > 0.0)) return nullptr;
    return &d;
}

}  // namespace

std::string timestampIso() {
    const std::time_t t = std::time(nullptr);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tmv);
    return buf;
}

std::string timestampClock() {
    const std::time_t t = std::time(nullptr);
    std::tm tmv{};
#ifdef _WIN32
    localtime_s(&tmv, &t);
#else
    localtime_r(&t, &tmv);
#endif
    char buf[16];
    std::strftime(buf, sizeof(buf), "%H:%M:%S", &tmv);
    return buf;
}

namespace {

// Pass one of reading the grid, shared by detectLattice and readGridPitch:
// which way round it is printed, and its finest pitch on each axis. Leaves
// `gray` as detectLattice goes on to use it - the negative, for a grid printed
// dark.
bool firstPass(const cv::Mat& bgr, const Config& cfg, cv::Mat& gray, LatticeInfo& info,
               double& pitchX, double& pitchY) {
    cv::Mat gray8;
    cv::cvtColor(bgr, gray8, cv::COLOR_BGR2GRAY);
    gray8.convertTo(gray, CV_32F);
    const int h = gray.rows;
    const int w = gray.cols;

    // A grid printed dark on a light ground is read as its negative, which
    // turns its lines into the bright ridges everything after looks for. The
    // occupancy test in detectLattice is symmetric about the level and is the
    // same either way round. On a bright-lined mat nothing here changes a pixel.
    info.dark_lines = printedDarkLines(gray, cfg, info.dip_to_peak);
    if (info.dark_lines) cv::subtract(cv::Scalar::all(255.0), gray, gray);

    // Pass one: coarse high-pass, which keeps every printed line, then measure
    // the finest pitch on each axis independently.
    //
    // Only the two bands above are ever read out of this, about six per cent of
    // the frame, so only those two are built. The sigma here is the widest in
    // the whole pipeline and the kernel runs to a thousand taps, which made
    // blurring all 20 MP of it the single largest cost in a calibration: half
    // the total, to look at a twentieth of the picture. The bands are computed
    // with their halo taken from the frame around them, so each one holds the
    // same bytes the full-frame blur would have put there. What is skipped is
    // only ever the part that was thrown away.
    const double coarseSigma = cfg.max_pitch_px / 3.0;
    int bandY0 = 0, bandY1 = 0, bandX0 = 0, bandX1 = 0;
    pitchProfileBands(h, w, bandY0, bandY1, bandX0, bandX1);
    cv::Mat coarse = cv::Mat::zeros(h, w, CV_32F);
    {
        const cv::Mat bg = gaussianBlurRowBand(gray, bandY0, bandY1, coarseSigma);
        cv::Mat band = coarse.rowRange(bandY0, bandY1);
        cv::subtract(gray.rowRange(bandY0, bandY1), bg, band);
        cv::max(band, 0.0, band);
    }
    {
        const cv::Mat bg = gaussianBlurColBand(gray, bandX0, bandX1, coarseSigma);
        cv::Mat band = coarse.colRange(bandX0, bandX1);
        cv::subtract(gray.colRange(bandX0, bandX1), bg, band);
        cv::max(band, 0.0, band);
    }

    if (!estimatePitch(coarse, cfg, pitchX, pitchY)) {
        info.error = "could not find a repeating grid pitch";
        return false;
    }
    return true;
}

}  // namespace

bool gridPrintedDark(const cv::Mat& bgr, const Config& cfg, double* dipToPeak) {
    cv::Mat gray8, gray;
    cv::cvtColor(bgr, gray8, cv::COLOR_BGR2GRAY);
    gray8.convertTo(gray, CV_32F);
    double ratio = 0.0;
    const bool dark = printedDarkLines(gray, cfg, ratio);
    if (dipToPeak) *dipToPeak = ratio;
    return dark;
}

bool readGridPitch(const cv::Mat& bgr, const Config& cfg, LatticeInfo& info) {
    info = LatticeInfo();
    cv::Mat gray;
    double pitchX = 0.0;
    double pitchY = 0.0;
    if (!firstPass(bgr, cfg, gray, info, pitchX, pitchY)) return false;
    const double pitch = std::min(pitchX, pitchY);
    if (!(pitch >= cfg.min_pitch_px && pitch <= cfg.max_pitch_px)) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "grid pitch %.1f px is outside the expected range", pitch);
        info.error = buf;
        return false;
    }
    info.pitch_px = pitch;
    info.pitch_px_x = pitchX;
    info.pitch_px_y = pitchY;
    return true;
}

bool detectLattice(const cv::Mat& bgr, const Config& cfg, std::vector<cv::Point2d>& pointsPx,
                   std::vector<cv::Point2d>& pointsMm, LatticeInfo& info, bool verbose) {
    pointsPx.clear();
    pointsMm.clear();
    info = LatticeInfo();

    cv::Mat gray;
    double pitchX = 0.0;
    double pitchY = 0.0;
    if (!firstPass(bgr, cfg, gray, info, pitchX, pitchY)) return false;
    const int h = gray.rows;
    const int w = gray.cols;
    const double pitch = std::min(pitchX, pitchY);
    if (!(pitch >= cfg.min_pitch_px && pitch <= cfg.max_pitch_px)) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "grid pitch %.1f px is outside the expected range", pitch);
        info.error = buf;
        return false;
    }

    // Pass two: ridge tuned to the pitch we found.
    cv::Mat ridge = ridgeImage(gray, std::max(2.0, pitch / 2.5));

    // Suppress anything sitting on the surface. On a genuinely empty mat this
    // removes nothing; if the operator left something there it stops that object
    // from contributing false lines.
    cv::Mat coarseLevel;
    gaussianBlurFast(gray, coarseLevel, pitch);

    // The level and the spread of it, straight off the image. Copying 20 MP
    // into a vector<double> and sorting it, twice, cost more than the blur that
    // produced it; these read the same two order statistics out of the pixels
    // where they already are. Same numbers, to the bit.
    const double med = medianOfMat(coarseLevel);
    const double mad = medianAbsDevFromMat(coarseLevel, med) + 1e-3;
    const double occThresh = 6.0 * 1.4826 * mad;

    cv::Mat occupied(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y) {
        const float* p = coarseLevel.ptr<float>(y);
        unsigned char* d = occupied.ptr<unsigned char>(y);
        for (int x = 0; x < w; ++x) {
            d[x] = (std::abs(static_cast<double>(p[x]) - med) > occThresh) ? 1 : 0;
        }
    }
    const int kOcc = static_cast<int>(pitch) | 1;
    cv::dilate(occupied, occupied, cv::Mat::ones(kOcc, kOcc, CV_8U));

    double occupiedSum = 0.0;
    for (int y = 0; y < h; ++y) {
        const unsigned char* o = occupied.ptr<unsigned char>(y);
        float* r = ridge.ptr<float>(y);
        for (int x = 0; x < w; ++x) {
            if (o[x]) {
                r[x] = 0.0f;
                occupiedSum += 1.0;
            }
        }
    }

    const int m = cfg.border_margin_px;
    ridge.rowRange(0, m).setTo(0.0f);
    ridge.rowRange(h - m, h).setTo(0.0f);
    ridge.colRange(0, m).setTo(0.0f);
    ridge.colRange(w - m, w).setTo(0.0f);
    info.occupied_fraction = occupiedSum / static_cast<double>(h) / static_cast<double>(w);

    cv::Mat normalised;
    cv::normalize(ridge, normalised, 0, 255, cv::NORM_MINMAX, CV_32F);
    const cv::Mat r8 = toU8Truncate(normalised);
    cv::Mat binary;
    cv::threshold(r8, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    double phiH = 0.0;
    double phiV = 90.0;
    if (info.dark_lines) {
        // The angles are read off the strongest few per cent of gradients, and
        // on a woven cloth those are the weave's, a pixel or two across, not
        // the printed lines': measured on the bench's cloth the orientation
        // histogram came out flat, 0.6% in its best half-degree bin, and the
        // second family landed 8 degrees off square, which is enough for the
        // directional opening to find none of those lines at all. The lines
        // are several pixels wide and the weave is not, so a blur the size of
        // a line takes the weave away and leaves the grid: 4% and 2% peaks, at
        // right angles, on every frame tried. The printed mat never needed it.
        cv::Mat soft;
        gaussianBlurFast(ridge, soft, std::max(1.5, pitch / 24.0));
        estimateGridAngles(soft, phiH, phiV);
    } else {
        estimateGridAngles(ridge, phiH, phiV);
    }
    info.phi_h = phiH;
    info.phi_v = phiV;

    // A single kernel length does not suit every rig. Too long and a converging
    // line family gets clipped away; too short and printed diagonals survive.
    // Try progressively shorter kernels and keep whichever run indexes the most
    // lines, rather than assuming one setting fits every camera height.
    const double minLen = pitch * cfg.min_line_length_pitches;
    bool haveBest = false;
    int bestScore = -1;
    std::vector<FittedLine> hLines, vLines;
    const double factors[3] = {cfg.line_kernel_pitches, cfg.line_kernel_pitches * 0.6,
                               cfg.line_kernel_pitches * 0.35};
    for (double factor : factors) {
        const int klen = std::max(7, static_cast<int>(std::nearbyint(pitch * factor)));
        std::vector<FittedLine> hTry =
            fitLineFamily(directionalOpen(binary, phiH, klen, true), ridge, 0, minLen);
        std::vector<FittedLine> vTry =
            fitLineFamily(directionalOpen(binary, phiV - 90.0, klen, false), ridge, 1, minLen);
        const int score = static_cast<int>(std::min(hTry.size(), vTry.size()));
        if (!haveBest || score > bestScore) {
            haveBest = true;
            bestScore = score;
            hLines.swap(hTry);
            vLines.swap(vTry);
        }
        if (score >= 20) break;
    }

    info.h_lines = static_cast<int>(hLines.size());
    info.v_lines = static_cast<int>(vLines.size());
    info.pitch_px = pitch;
    info.pitch_px_x = pitchX;
    info.pitch_px_y = pitchY;

    if (hLines.size() < 4 || vLines.size() < 4) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "too few grid lines detected (h=%d, v=%d)", info.h_lines,
                      info.v_lines);
        info.error = buf;
        return false;
    }

    std::vector<double> hOff(hLines.size());
    std::vector<double> vOff(vLines.size());
    for (std::size_t k = 0; k < hLines.size(); ++k) hOff[k] = lineValue(hLines[k].coef, w / 2.0);
    for (std::size_t k = 0; k < vLines.size(); ++k) vOff[k] = lineValue(vLines[k].coef, h / 2.0);

    const IndexAssignment hAssign = assignIndices(hOff, cfg);
    const IndexAssignment vAssign = assignIndices(vOff, cfg);

    // Which printed spacing did we actually latch onto? If only the heavy 50 mm
    // lines survived detection, the base gap is five pitches and calling it 10 mm
    // would put every measurement out by a factor of five. Resolve it explicitly
    // rather than assuming.
    auto resolveStep = [&](double baseGap, double axisPitch, int& mult) {
        const double ratio = baseGap / axisPitch;
        int bestMult = cfg.allowed_pitch_multiples.front();
        double bestDelta = std::abs(ratio - bestMult);
        for (int candidate : cfg.allowed_pitch_multiples) {
            const double delta = std::abs(ratio - candidate);
            if (delta < bestDelta) {
                bestDelta = delta;
                bestMult = candidate;
            }
        }
        if (std::abs(ratio - bestMult) > kStepTolerance * bestMult) return false;
        mult = bestMult;
        return true;
    };

    // From one indexing of the two families to the lattice. Writes only to what
    // it is handed, so a second indexing can be tried without disturbing the
    // first one's outcome.
    auto buildLattice = [&](const IndexAssignment& hA, const IndexAssignment& vA, LatticeInfo& inf,
                            std::vector<cv::Point2d>& outPx, std::vector<cv::Point2d>& outMm,
                            int& hMult, int& vMult) {
        if (!hA.ok || !vA.ok) {
            inf.error = "grid line spacing was not regular enough to index";
            return false;
        }
        if (!resolveStep(hA.base, pitchY, hMult) || !resolveStep(vA.base, pitchX, vMult)) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "detected line spacing (%.1f, %.1f px) is not a clean multiple of the "
                          "printed pitch (%.1f, %.1f px)",
                          vA.base, hA.base, pitchX, pitchY);
            inf.error = buf;
            return false;
        }

        const double stepMmH = cfg.grid_pitch_mm * hMult;
        const double stepMmV = cfg.grid_pitch_mm * vMult;
        inf.step_mm_x = stepMmV;
        inf.step_mm_y = stepMmH;
        inf.pitch_multiple_x = vMult;
        inf.pitch_multiple_y = hMult;

        for (std::size_t a = 0; a < hA.selected.size(); ++a) {
            const FittedLine& lh = hLines[hA.selected[a]];
            const int li = hA.indices[a];
            for (std::size_t b = 0; b < vA.selected.size(); ++b) {
                const FittedLine& lv = vLines[vA.selected[b]];
                const int lj = vA.indices[b];
                double x = 0.0;
                double y = 0.0;
                intersect(lh.coef, lv.coef, vOff[vA.selected[b]], hOff[hA.selected[a]], x, y);
                if (!(x >= m && x < w - m && y >= m && y < h - m)) continue;
                // only keep intersections inside both lines' observed runs
                if (!(lh.u_min - pitch <= x && x <= lh.u_max + pitch)) continue;
                if (!(lv.u_min - pitch <= y && y <= lv.u_max + pitch)) continue;
                outPx.emplace_back(x, y);
                outMm.emplace_back(lj * stepMmV, li * stepMmH);
            }
        }

        inf.lattice_points = static_cast<int>(outPx.size());
        if (outPx.size() < static_cast<std::size_t>(cfg.min_lattice_points)) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "only %d lattice points found, need %d",
                          inf.lattice_points, cfg.min_lattice_points);
            inf.error = buf;
            return false;
        }
        return true;
    };

    int hMult = 1;
    int vMult = 1;
    IndexAssignment hSeeded;
    IndexAssignment vSeeded;
    const IndexAssignment* hUsed = &hAssign;
    const IndexAssignment* vUsed = &vAssign;
    // A grid printed dark is on cloth, and cloth has creases: every line that
    // crosses one is found as separate pieces a few pixels apart, three or four
    // to a line on the bench's cloth, and those near-duplicates halve the
    // median gap the first walk starts from - it refused every cloth frame
    // tried, as "not a clean multiple" at half a square. The walk from the
    // measured pitch is not fooled by them, and where both walks work they
    // index the same lines, so a cloth goes straight to it.
    const bool walkFromPitch = info.dark_lines;
    if (walkFromPitch || !buildLattice(hAssign, vAssign, info, pointsPx, pointsMm, hMult, vMult)) {
        // A frame the walk indexes never gets past here, so every calibration
        // that worked before is the same calibration, to the bit. What follows
        // only ever turns a refusal into a calibration.
        //
        // The walk starts from the median gap between neighbouring lines, and
        // that is its fragile part. The printed arcs and their labels, the
        // hexagon, and anything left on the mat cut grid lines into pieces, and
        // a piece seen near one edge reaches the centre of the frame only by
        // extrapolating its own fit a long way, landing anywhere between the
        // true lines. On the bench's clear mat more than half the gaps come
        // from such pieces: the median starts out below the tolerance and only
        // its refinement drags it back, so whether a frame passes is down to
        // chance, and when it does not, the spacing reads as about half a
        // square and the frame is refused as "not a clean multiple" of the
        // pitch. The pitch was measured before any of this, from a strip down
        // the middle, and pieces do not fool it: start the same walk from it.
        // Where the median is not fooled the two starts settle on the same
        // spacing, so the retry indexes the lines the walk would have indexed
        // on a better day.
        hSeeded = assignIndices(hOff, cfg, pitchY);
        vSeeded = assignIndices(vOff, cfg, pitchX);
        LatticeInfo alt = info;
        std::vector<cv::Point2d> altPx;
        std::vector<cv::Point2d> altMm;
        int hAlt = 1;
        int vAlt = 1;
        if (!buildLattice(hSeeded, vSeeded, alt, altPx, altMm, hAlt, vAlt)) {
            if (verbose) {
                std::printf("  walk from pitch     : failed%s, %s\n", walkFromPitch ? "" : " too",
                            alt.error.c_str());
                std::fflush(stdout);
            }
            // With the first attempt's reason, as before - or, on a cloth,
            // where there was no first attempt, with this one's.
            if (walkFromPitch) info.error = alt.error;
            return false;
        }
        alt.first_attempt_error = info.error;
        alt.pitch_seeded = true;
        alt.error.clear();
        info = alt;
        pointsPx.swap(altPx);
        pointsMm.swap(altMm);
        hMult = hAlt;
        vMult = vAlt;
        hUsed = &hSeeded;
        vUsed = &vSeeded;
    }

    if (verbose) {
        if (info.dark_lines) {
            std::printf("  printed lines       : dark on a light ground (dips %.2fx the peaks), "
                        "read as its negative\n",
                        info.dip_to_peak);
        }
        std::printf("  printed pitch       : %.2f px in x, %.2f px in y\n", pitchX, pitchY);
        std::printf("  grid orientation    : %+.2f deg and %+.2f deg\n", phiH, phiV);
        std::printf("  lines detected      : %d horizontal, %d vertical\n", info.h_lines,
                    info.v_lines);
        if (info.pitch_seeded && !info.first_attempt_error.empty()) {
            std::printf("  first attempt       : %s\n", info.first_attempt_error.c_str());
            std::printf("  walk restarted      : from the printed pitch\n");
        } else if (info.pitch_seeded) {
            std::printf("  walk                : from the printed pitch\n");
        }
        std::printf("  lines indexed       : %d vertical, %d horizontal\n",
                    static_cast<int>(vUsed->selected.size()),
                    static_cast<int>(hUsed->selected.size()));
        std::printf("  spacing used        : %.1f mm in x, %.1f mm in y\n", info.step_mm_x,
                    info.step_mm_y);
        if (vMult != 1 || hMult != 1) {
            std::printf(
                "  NOTE: only every %dth / %dth printed line survived detection, so the heavier "
                "grid is being used as the reference. This is fine, but confirm the printed "
                "spacing matches.\n",
                vMult, hMult);
        }
        std::printf("  lattice points      : %d\n", info.lattice_points);
        std::fflush(stdout);
    }

    return true;
}

PlaneModelPtr fitPlaneModel(const std::vector<cv::Point2d>& pointsPxIn,
                            const std::vector<cv::Point2d>& pointsMmIn, cv::Size imageSize,
                            const Config& cfg, bool verbose, double pitchPx, double gridStepMm,
                            FitReport& report) {
    report = FitReport();

    const int w = imageSize.width;
    const int h = imageSize.height;
    const cv::Point2d centre(w / 2.0, h / 2.0);
    const double norm = std::max(w, h) / 2.0;

    const int totalPoints = static_cast<int>(pointsPxIn.size());

    // With the lens already measured for the bench there is no decision to
    // take about its distortion, only a homography to fit under it.
    const LensDistortion* lens = benchDistortionFor(cfg, imageSize);

    std::vector<cv::Point2d> pointsPx;
    std::vector<cv::Point2d> pointsMm;
    if (lens) {
        screenLattice(pointsPxIn, pointsMmIn, pitchPx, gridStepMm, cfg,
                      [&](const std::vector<cv::Point2d>& mm, const std::vector<cv::Point2d>& px) {
                          return fitModelLocked(mm, px, *lens, cfg);
                      },
                      pointsPx, pointsMm);
    } else {
        screenLattice(pointsPxIn, pointsMmIn, pitchPx, gridStepMm, cfg,
                      [&](const std::vector<cv::Point2d>& mm, const std::vector<cv::Point2d>& px) {
                          return fitModel(mm, px, centre, norm, true, cfg);
                      },
                      pointsPx, pointsMm);
    }

    const int n = static_cast<int>(pointsPx.size());

    PlaneModelPtr final;
    double plainHold = 0.0;
    double distHold = std::numeric_limits<double>::infinity();
    bool haveDist = false;
    double gain = 0.0;
    bool useDistortion = false;
    if (lens) {
        final = fitModelLocked(pointsMm, pointsPx, *lens, cfg);
        useDistortion = true;
        distHold = 0.0;
    } else {
        // Deterministic split, so a rerun on the same data gives the same
        // verdict. The seed matches the Python original. The permutation
        // itself differs from NumPy's PCG64 stream, which changes only which
        // points land in the holdout set; the chosen structure is refitted on
        // every point afterwards, so the model that gets used is unaffected.
        std::vector<int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);
        std::mt19937_64 rng(20260831ull);
        for (int i = n - 1; i > 0; --i) {
            std::uniform_int_distribution<int> pick(0, i);
            std::swap(perm[i], perm[pick(rng)]);
        }

        const int nHold = std::max(20, static_cast<int>(n * cfg.holdout_fraction));
        std::vector<cv::Point2d> holdPx, holdMm, trainPx, trainMm;
        for (int k = 0; k < n; ++k) {
            const int idx = perm[k];
            if (k < nHold) {
                holdPx.push_back(pointsPx[idx]);
                holdMm.push_back(pointsMm[idx]);
            } else {
                trainPx.push_back(pointsPx[idx]);
                trainMm.push_back(pointsMm[idx]);
            }
        }

        PlaneModelPtr plain = fitModel(trainMm, trainPx, centre, norm, false, cfg);
        if (!plain) {
            report.error = "homography fit failed";
            return nullptr;
        }
        double ignored = 0.0;
        rmseMm(*plain, holdPx, holdMm, plainHold, ignored);

        PlaneModelPtr dist = fitModel(trainMm, trainPx, centre, norm, true, cfg);
        haveDist = dist != nullptr;
        if (dist) rmseMm(*dist, holdPx, holdMm, distHold, ignored);

        gain = (plainHold > 0.0) ? (plainHold - distHold) / plainHold : 0.0;
        useDistortion = haveDist && (gain > cfg.distortion_gain_required);

        // Refit the chosen structure on everything now the decision is made.
        final = fitModel(pointsMm, pointsPx, centre, norm, useDistortion, cfg);
    }
    if (!final) {
        report.error = "homography fit failed";
        return nullptr;
    }

    double rmse = 0.0;
    double worst = 0.0;
    rmseMm(*final, pointsPx, pointsMm, rmse, worst);

    const std::pair<double, double> localScale = final->localMmPerPx(centre);

    vjson::Value meta = vjson::Value::object();
    meta.set("lattice_points", vjson::Value::integer(n));
    meta.set("lattice_points_detected", vjson::Value::integer(totalPoints));
    meta.set("rmse_mm", vjson::Value::real(rmse));
    meta.set("max_residual_mm", vjson::Value::real(worst));
    meta.set("holdout_rmse_plain_mm", lens ? vjson::Value::null() : vjson::Value::real(plainHold));
    meta.set("holdout_rmse_distortion_mm",
             haveDist ? vjson::Value::real(distHold) : vjson::Value::null());
    meta.set("distortion_used", vjson::Value::boolean(useDistortion));
    if (lens) {
        meta.set("distortion_source", vjson::Value::str("bench"));
        meta.set("distortion_fitted_at", vjson::Value::str(lens->fitted_at));
    }
    vjson::Value legacy = vjson::Value::array();
    legacy.push(vjson::Value::real(localScale.first));
    legacy.push(vjson::Value::real(localScale.second));
    meta.set("legacy_mm_per_px", legacy);
    meta.set("calibrated_at", vjson::Value::str(timestampIso()));
    vjson::Value size = vjson::Value::array();
    size.push(vjson::Value::integer(w));
    size.push(vjson::Value::integer(h));
    meta.set("image_size", size);
    final->meta() = meta;

    report.rmse_mm = rmse;
    report.max_residual_mm = worst;
    report.holdout_plain_mm = plainHold;
    report.holdout_distortion_mm = distHold;
    report.has_distortion_holdout = haveDist;
    report.distortion_used = useDistortion;
    report.gain = gain;
    report.points_used = n;
    report.points_detected = totalPoints;

    if (verbose) {
        std::printf("  points used         : %d of %d detected\n", n, totalPoints);
        if (lens) {
            std::printf("  distortion terms    : the bench's, fitted %s on %d pictures "
                        "(k1 %+.6f, k2 %+.6f); homography only\n",
                        lens->fitted_at.c_str(), lens->frames, lens->k1, lens->k2);
        } else {
            std::printf("  holdout RMSE        : %.4f mm homography only\n", plainHold);
            if (haveDist) {
                std::printf("                        %.4f mm with radial terms (%+.1f%%)\n", distHold,
                            gain * 100.0);
            }
            std::printf("  distortion terms    : %s\n",
                        useDistortion ? "kept" : "rejected, no meaningful holdout gain");
        }
        std::printf("  full-fit RMSE       : %.4f mm, worst point %.4f mm\n", rmse, worst);
        std::printf("  local scale, centre : %.6f mm/px in x, %.6f mm/px in y\n", localScale.first,
                    localScale.second);
        if (rmse > cfg.max_acceptable_rmse_mm) {
            std::printf(
                "  WARNING: residual above %g mm. Check the surface is flat, in focus and free of "
                "glare.\n",
                cfg.max_acceptable_rmse_mm);
        }
        std::fflush(stdout);
    }

    return final;
}

PlaneModelPtr autoCalibrate(const cv::Mat& frame, const Config& cfg, bool verbose,
                            std::string* error, LatticeInfo* lattice) {
    std::printf("\n=== AUTOMATIC PLANE CALIBRATION ===\n");
    std::fflush(stdout);
    if (error) error->clear();

    std::vector<cv::Point2d> pointsPx, pointsMm;
    LatticeInfo local;
    LatticeInfo& info = lattice ? *lattice : local;
    if (!detectLattice(frame, cfg, pointsPx, pointsMm, info, verbose)) {
        std::printf("  FAILED: %s\n", info.error.c_str());
        std::printf("  The surface must be empty, evenly lit, in focus, and the printed\n");
        std::printf("  grid must be visible across most of the frame.\n");
        std::fflush(stdout);
        if (error) *error = info.error;
        return nullptr;
    }

    FitReport report;
    PlaneModelPtr model =
        fitPlaneModel(pointsPx, pointsMm, frame.size(), cfg, verbose, info.pitch_px,
                      std::min(info.step_mm_x, info.step_mm_y), report);
    if (!model) {
        std::printf("  FAILED: %s\n", report.error.c_str());
        std::fflush(stdout);
        if (error) *error = report.error;
        return nullptr;
    }

    // A frame the first attempt refused is only taken if the fit says the
    // lattice it got is sound: most of its points agreeing with the model, and
    // a residual the station would not warn about. Otherwise it is refused for
    // the reason it always was. The walk keeps a mis-indexed fragment now and
    // then whichever way it starts, and the fit drops those; a clear mat on
    // this bench loses up to a sixth of its points that way and a cluttered one
    // more, so the bar is set where a lattice has plainly come apart.
    if (info.pitch_seeded) {
        constexpr double kMinAgreeing = 0.6;
        const double agreeing =
            static_cast<double>(report.points_used) / std::max(1, report.points_detected);
        // A cloth's creases move its printed grid by a millimetre or so where
        // they cross it, and no plane explains that, so a sound cloth lattice
        // fits a little worse than a sound mat one: 0.22 to 0.38 mm on the
        // bench's cloth, with 98% of points agreeing. What this check exists to
        // catch - a lattice indexed a whole square wrong - costs agreement long
        // before it costs that. Above the usual limit it is still accepted with
        // the usual warning, which is the right thing to hear about a creased
        // cloth.
        const double rmseLimit = info.dark_lines
                                     ? std::max(cfg.max_acceptable_rmse_mm, cfg.dark_grid_max_rmse_mm)
                                     : cfg.max_acceptable_rmse_mm;
        if (agreeing < kMinAgreeing || report.rmse_mm > rmseLimit) {
            char buf[512];
            std::snprintf(buf, sizeof(buf),
                          "%s (started from the printed pitch the grid indexed, but the fit was "
                          "poor: residual %.3f mm, %d of %d points agree)",
                          info.first_attempt_error.empty() ? "the printed grid did not index cleanly"
                                                           : info.first_attempt_error.c_str(),
                          report.rmse_mm, report.points_used, report.points_detected);
            std::printf("  FAILED: %s\n", buf);
            std::fflush(stdout);
            if (error) *error = buf;
            return nullptr;
        }
    }

    vjson::Value detection = vjson::Value::object();
    detection.set("h_lines", vjson::Value::integer(info.h_lines));
    detection.set("v_lines", vjson::Value::integer(info.v_lines));
    detection.set("pitch_px", vjson::Value::real(info.pitch_px));
    vjson::Value stepMm = vjson::Value::array();
    stepMm.push(vjson::Value::real(info.step_mm_x));
    stepMm.push(vjson::Value::real(info.step_mm_y));
    detection.set("step_mm", stepMm);
    detection.set("lattice_points", vjson::Value::integer(info.lattice_points));
    if (info.pitch_seeded) detection.set("pitch_seeded", vjson::Value::boolean(true));
    if (info.dark_lines) detection.set("dark_lines", vjson::Value::boolean(true));
    model->meta().set("detection", detection);

    const cv::Rect footprint = cv::boundingRect(toPoint2f(pointsPx));
    vjson::Value fp = vjson::Value::array();
    fp.push(vjson::Value::integer(footprint.x));
    fp.push(vjson::Value::integer(footprint.y));
    fp.push(vjson::Value::integer(footprint.width));
    fp.push(vjson::Value::integer(footprint.height));
    model->meta().set("lattice_footprint", fp);

    std::printf("===================================\n\n");
    std::fflush(stdout);
    return model;
}

GeometryAgreement compareGeometry(const std::vector<PlaneModelPtr>& models, cv::Size frame) {
    GeometryAgreement out;
    std::vector<const PlaneModel*> ms;
    for (const PlaneModelPtr& m : models) {
        if (m) ms.push_back(m.get());
    }
    out.models = static_cast<int>(ms.size());
    const double w = frame.width;
    const double h = frame.height;
    if (ms.size() < 2 || !(w > 0.0 && h > 0.0)) return out;

    // A grid over the picture, kept off its border as the lattice is, and each
    // point's distance from the centre as a share of half the width - the
    // radius the radial terms are normalised by.
    const double half = std::max(w, h) / 2.0;
    std::vector<cv::Point2d> px;
    std::vector<int> region;
    for (double y = 24.0; y <= h - 24.0; y += 48.0) {
        for (double x = 24.0; x <= w - 24.0; x += 48.0) {
            px.emplace_back(x, y);
            const bool corner = (x < 0.05 * w || x > 0.95 * w) && (y < 0.05 * h || y > 0.95 * h);
            const double r = std::hypot(x - w / 2.0, y - h / 2.0) / half;
            region.push_back(corner ? 4 : r < 0.5 ? 0 : r < 0.7 ? 1 : r < 0.85 ? 2 : 3);
        }
    }
    std::vector<std::vector<cv::Point2d>> mm;
    for (const PlaneModel* m : ms) mm.push_back(m->pxToMm(px));

    double worst[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < mm.size(); ++i) {
        for (std::size_t j = i + 1; j < mm.size(); ++j) {
            // The rigid move from one's millimetres to the other's, least
            // squares over the middle of the picture.
            cv::Point2d ca(0.0, 0.0), cb(0.0, 0.0);
            int count = 0;
            for (std::size_t k = 0; k < px.size(); ++k) {
                if (region[k] != 0) continue;
                ca += mm[i][k];
                cb += mm[j][k];
                ++count;
            }
            if (count == 0) continue;
            ca *= 1.0 / count;
            cb *= 1.0 / count;
            double dot = 0.0, cross = 0.0;
            for (std::size_t k = 0; k < px.size(); ++k) {
                if (region[k] != 0) continue;
                const cv::Point2d a = mm[i][k] - ca;
                const cv::Point2d b = mm[j][k] - cb;
                dot += a.x * b.x + a.y * b.y;
                cross += a.x * b.y - a.y * b.x;
            }
            const double t = std::atan2(cross, dot);
            const double c = std::cos(t), s = std::sin(t);
            for (std::size_t k = 0; k < px.size(); ++k) {
                const cv::Point2d a = mm[i][k] - ca;
                const cv::Point2d moved(cb.x + c * a.x - s * a.y, cb.y + s * a.x + c * a.y);
                const double d = std::hypot(moved.x - mm[j][k].x, moved.y - mm[j][k].y);
                worst[region[k]] = std::max(worst[region[k]], d);
            }
        }
    }
    out.centre_mm = worst[0];
    out.mid_mm = worst[1];
    out.outer_mm = worst[2];
    out.edge_mm = worst[3];
    out.corner_mm = worst[4];
    return out;
}

bool fitBenchDistortion(const std::vector<cv::Mat>& frames, const std::vector<std::string>& names,
                        const Config& cfgIn, DistortionFit& out) {
    out = DistortionFit();
    // Each picture on its own is read and screened exactly as a recalibration
    // with no saved distortion would do it.
    Config cfg = cfgIn;
    cfg.lens_distortion.reset();
    if (frames.empty()) {
        out.error = "no pictures";
        return false;
    }
    const cv::Size size = frames.front().size();
    const cv::Point2d centre(size.width / 2.0, size.height / 2.0);
    const double norm = std::max(size.width, size.height) / 2.0;

    // How far the mat moved from the first picture: what makes pictures of one
    // mat worth more than one. Read at an eighth of the size, where a mat
    // moved by a centimetre is still several pixels.
    cv::Mat first;
    {
        cv::Mat g;
        cv::cvtColor(frames.front(), g, cv::COLOR_BGR2GRAY);
        cv::resize(g, first, cv::Size(size.width / 8, size.height / 8), 0.0, 0.0, cv::INTER_AREA);
        first.convertTo(first, CV_32F);
    }

    struct Used {
        std::size_t frame = 0;
        std::vector<cv::Point2d> allPx, allMm;  // as detected
        std::vector<cv::Point2d> px, mm;        // as kept
        double pitchPx = 0.0;
        double stepMm = 0.0;
        PlaneModelPtr alone;
    };
    std::vector<Used> used;
    for (std::size_t f = 0; f < frames.size(); ++f) {
        DistortionFitFrame rep;
        rep.name = f < names.size() ? names[f] : std::to_string(f + 1);
        const cv::Mat& frame = frames[f];
        if (frame.size() != size) {
            rep.error = "not the same size as the first picture";
            out.frames.push_back(rep);
            continue;
        }
        {
            cv::Mat g, small;
            cv::cvtColor(frame, g, cv::COLOR_BGR2GRAY);
            cv::resize(g, small, first.size(), 0.0, 0.0, cv::INTER_AREA);
            small.convertTo(small, CV_32F);
            const cv::Point2d shift = cv::phaseCorrelate(first, small);
            rep.shift_px = 8.0 * std::hypot(shift.x, shift.y);
        }
        Used u;
        u.frame = f;
        LatticeInfo info;
        if (!detectLattice(frame, cfg, u.allPx, u.allMm, info, false)) {
            rep.error = info.error;
            out.frames.push_back(rep);
            continue;
        }
        if (info.dark_lines) {
            rep.error = "a grid printed dark on a light ground: the cloth, which is not a ruler";
            out.frames.push_back(rep);
            continue;
        }
        rep.points_detected = static_cast<int>(u.allPx.size());
        u.pitchPx = info.pitch_px;
        u.stepMm = std::min(info.step_mm_x, info.step_mm_y);
        FitReport alone;
        u.alone = fitPlaneModel(u.allPx, u.allMm, size, cfg, false, u.pitchPx, u.stepMm, alone);
        if (!u.alone) {
            rep.error = alone.error;
            out.frames.push_back(rep);
            continue;
        }
        rep.alone_rmse_mm = alone.rmse_mm;
        rep.alone_distortion = alone.distortion_used;
        rep.alone_k1 = u.alone->k1();
        rep.alone_k2 = u.alone->k2();
        screenLattice(u.allPx, u.allMm, u.pitchPx, u.stepMm, cfg,
                      [&](const std::vector<cv::Point2d>& mm, const std::vector<cv::Point2d>& px) {
                          return fitModel(mm, px, centre, norm, true, cfg);
                      },
                      u.px, u.mm);
        used.push_back(std::move(u));
        out.frames.push_back(rep);
    }
    if (used.size() < 2) {
        out.error = "fewer than two pictures of the rigid mat could be read";
        return false;
    }

    // Two radial terms shared by every picture, eight homography terms each.
    // Started from each picture's own fit, radial terms from the middle of
    // theirs.
    std::vector<double> x(2 + 8 * used.size(), 0.0);
    {
        std::vector<double> k1s, k2s;
        for (const Used& u : used) {
            k1s.push_back(u.alone->k1());
            k2s.push_back(u.alone->k2());
        }
        x[0] = median(k1s);
        x[1] = median(k2s);
    }
    for (std::size_t f = 0; f < used.size(); ++f) {
        LensDistortion start;
        start.k1 = x[0];
        start.k2 = x[1];
        start.cx = centre.x;
        start.cy = centre.y;
        start.norm = norm;
        const PlaneModelPtr m = fitModelLocked(used[f].mm, used[f].px, start, cfg);
        const cv::Matx33d H = m ? m->H() : used[f].alone->H();
        for (int k = 0; k < 8; ++k) x[2 + 8 * f + k] = H(k / 3, k % 3);
    }

    const auto jointFit = [&]() {
        std::size_t total = 0;
        for (const Used& u : used) total += u.px.size();
        auto residual = [&](const std::vector<double>& p, std::vector<double>& r) {
            r.assign(2 * total, 0.0);
            std::size_t at = 0;
            for (std::size_t f = 0; f < used.size(); ++f) {
                const double* q = &p[2 + 8 * f];
                const std::vector<cv::Point2d>& mm = used[f].mm;
                const std::vector<cv::Point2d>& px = used[f].px;
                for (std::size_t k = 0; k < mm.size(); ++k, ++at) {
                    const double X = mm[k].x;
                    const double Y = mm[k].y;
                    double pw = q[6] * X + q[7] * Y + 1.0;
                    if (std::abs(pw) < 1e-12) pw = 1e-12;
                    const double dx = ((q[0] * X + q[1] * Y + q[2]) / pw - centre.x) / norm;
                    const double dy = ((q[3] * X + q[4] * Y + q[5]) / pw - centre.y) / norm;
                    const double r2 = dx * dx + dy * dy;
                    const double s = 1.0 + p[0] * r2 + p[1] * r2 * r2;
                    r[2 * at] = centre.x + dx * norm * s - px[k].x;
                    r[2 * at + 1] = centre.y + dy * norm * s - px[k].y;
                }
            }
        };
        levenbergMarquardt(residual, x, cfg.lm_iterations);
    };
    const auto frameModel = [&](std::size_t f) {
        const double* q = &x[2 + 8 * f];
        const cv::Matx33d H(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], 1.0);
        return std::make_shared<PlaneModel>(H, x[0], x[1], centre, norm);
    };

    // Fitted, then each picture's points screened again against the shared
    // model the way screenLattice screens them, and fitted again while that
    // changes which points are kept.
    jointFit();
    for (int iter = 0; iter < 3; ++iter) {
        bool changed = false;
        for (std::size_t f = 0; f < used.size(); ++f) {
            Used& u = used[f];
            const PlaneModelPtr m = frameModel(f);
            const double mad = median(residualNorms(*m, u.px, u.mm)) + 1e-9;
            const double thresh = std::min(0.3 * u.stepMm, std::max(6.0 * mad, 0.05));
            const std::vector<double> err = residualNorms(*m, u.allPx, u.allMm);
            std::vector<cv::Point2d> keepPx, keepMm;
            for (std::size_t k = 0; k < err.size(); ++k) {
                if (err[k] < thresh) {
                    keepPx.push_back(u.allPx[k]);
                    keepMm.push_back(u.allMm[k]);
                }
            }
            if (keepPx.size() < static_cast<std::size_t>(std::max(cfg.min_lattice_points, 40))) continue;
            if (keepPx != u.px) {
                u.px.swap(keepPx);
                u.mm.swap(keepMm);
                changed = true;
            }
        }
        if (!changed) break;
        jointFit();
    }

    out.lens.k1 = x[0];
    out.lens.k2 = x[1];
    out.lens.cx = centre.x;
    out.lens.cy = centre.y;
    out.lens.norm = norm;
    out.lens.width = size.width;
    out.lens.height = size.height;
    out.lens.frames = static_cast<int>(used.size());
    out.lens.fitted_at = timestampIso();

    // What each picture makes of it, and what a recalibration on each picture
    // would give with it saved: the homography alone, through fitPlaneModel.
    Config lockedCfg = cfg;
    lockedCfg.lens_distortion = out.lens;
    std::vector<PlaneModelPtr> alone, locked;
    double sum = 0.0;
    int points = 0;
    for (std::size_t f = 0; f < used.size(); ++f) {
        const Used& u = used[f];
        DistortionFitFrame& rep = out.frames[u.frame];
        double rmse = 0.0, worst = 0.0;
        rmseMm(*frameModel(f), u.px, u.mm, rmse, worst);
        rep.points_used = static_cast<int>(u.px.size());
        rep.rmse_mm = rmse;
        rep.max_mm = worst;
        sum += rmse * rmse * static_cast<double>(u.px.size());
        points += static_cast<int>(u.px.size());
        alone.push_back(u.alone);
        FitReport ignored;
        locked.push_back(
            fitPlaneModel(u.allPx, u.allMm, size, lockedCfg, false, u.pitchPx, u.stepMm, ignored));
    }
    out.points = points;
    out.rmse_mm = points > 0 ? std::sqrt(sum / points) : 0.0;
    out.alone = compareGeometry(alone, size);
    out.locked = compareGeometry(locked, size);
    out.ok = true;
    return true;
}

PlaneModelPtr loadCalibration(const Config& cfg) {
    vjson::Value data;
    std::string error;
    if (!vjson::parseFile(cfg.calibration_file, data, error)) {
        // Absent file is the normal first-run case and is not worth reporting;
        // a malformed one is.
        std::FILE* probe = std::fopen(cfg.calibration_file.c_str(), "rb");
        if (probe) {
            std::fclose(probe);
            std::printf("Could not read %s: %s\n", cfg.calibration_file.c_str(), error.c_str());
            std::fflush(stdout);
        }
        return nullptr;
    }

    PlaneModelPtr model = PlaneModel::fromJson(data);
    if (!model) {
        std::printf(
            "%s is in the old scalar mm/px format. That model cannot represent perspective, so it "
            "is being ignored. Recalibrating.\n",
            cfg.calibration_file.c_str());
        std::fflush(stdout);
        return nullptr;
    }

    std::printf("\n--- Calibration loaded ---\n");
    const vjson::Value* lp = model->meta().find("lattice_points");
    if (lp && lp->isNumber()) {
        std::printf("  Fitted from   : %lld lattice points\n",
                    static_cast<long long>(lp->asNumber()));
    } else {
        std::printf("  Fitted from   : ? lattice points\n");
    }
    std::printf("  Residual      : %.4f mm RMSE, %.4f mm worst\n",
                model->metaNumber("rmse_mm", 0.0), model->metaNumber("max_residual_mm", 0.0));
    std::printf("  Distortion    : %s\n",
                model->meta().getBool("distortion_used", false) ? "modelled" : "not needed");
    std::printf("  Scale factor  : %.6f%s\n", model->scaleCorrection(),
                model->scaleCorrection() != 1.0 ? "" : "  (no artefact check done yet)");
    std::printf("  Calibrated    : %s\n",
                model->meta().getString("calibrated_at", "unknown").c_str());
    std::printf("--------------------------\n\n");
    std::fflush(stdout);
    return model;
}

void saveCalibration(const PlaneModel& model, const Config& cfg) {
    vjson::writeFile(cfg.calibration_file, model.toJson(), 2);
    std::printf("Calibration saved to %s\n", cfg.calibration_file.c_str());
    std::fflush(stdout);
}

bool loadRoi(const Config& cfg, RoiBox& roi) {
    vjson::Value data;
    std::string error;
    if (!vjson::parseFile(cfg.roi_config_file, data, error)) return false;
    const vjson::Value* box = data.find("roi_box");
    if (!box || !box->isArray() || box->size() < 4) return false;
    roi.x1 = static_cast<int>(box->at(0).asNumber());
    roi.y1 = static_cast<int>(box->at(1).asNumber());
    roi.x2 = static_cast<int>(box->at(2).asNumber());
    roi.y2 = static_cast<int>(box->at(3).asNumber());
    return true;
}

void saveRoi(const RoiBox& roi, const Config& cfg) {
    vjson::Value root = vjson::Value::object();
    vjson::Value box = vjson::Value::array();
    box.push(vjson::Value::integer(roi.x1));
    box.push(vjson::Value::integer(roi.y1));
    box.push(vjson::Value::integer(roi.x2));
    box.push(vjson::Value::integer(roi.y2));
    root.set("roi_box", box);
    vjson::writeFile(cfg.roi_config_file, root, 2);
    std::printf("ROI saved: (%d, %d, %d, %d)\n", roi.x1, roi.y1, roi.x2, roi.y2);
    std::fflush(stdout);
}

namespace {

double medianOf(std::vector<unsigned char>& v) {
    if (v.empty()) return 0.0;
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(mid), v.end());
    return static_cast<double>(v[mid]);
}

// The colour of the mat's body, and how far a pixel may sit from it and still
// be called mat.
//
// Not the median, which is the obvious thing to reach for and is wrong here.
//
// A median is the mat's colour only while the mat's plain surface is a clear
// majority of what was sampled, and inside the lattice it frequently is not.
// The printed grid, the rulers, the cut guides and the target rings are all mat
// and none of them are its colour, and a garment left on the surface through a
// recalibration is half the picture by itself. Measured across this bench with
// a shirt on it the median comes out at L=139 - the mat is 25 and the shirt is
// 230, so the median has landed between the two modes, describing neither - and
// the deviation around a centre that fits nothing is enormous. The tolerance it
// implies was 347 on a channel that only goes to 255, which accepted every
// pixel in the frame, bench included, and returned the whole picture as mat.
//
// So the mat's colour is taken as the dominant mode instead. A coarse histogram
// finds it and a few mean-shift steps sharpen it. The part that actually fixes
// the failure above is the last step: the spread is measured over the pixels
// belonging to that mode and no others, so whatever else is lying on the mat
// cannot widen it, having never been counted.
bool matColour(const std::vector<cv::Vec3b>& samples, double sigmas, double centre[3],
               double tol[3]) {
    if (samples.size() < 64) return false;

    constexpr int kBins = 16;
    constexpr double kStep = 256.0 / kBins;
    std::vector<int> hist(static_cast<std::size_t>(kBins) * kBins * kBins, 0);
    for (const cv::Vec3b& s : samples) {
        const int i = std::min(kBins - 1, static_cast<int>(s[0] / kStep));
        const int j = std::min(kBins - 1, static_cast<int>(s[1] / kStep));
        const int k = std::min(kBins - 1, static_cast<int>(s[2] / kStep));
        ++hist[static_cast<std::size_t>((i * kBins + j) * kBins + k)];
    }
    const std::size_t peak =
        static_cast<std::size_t>(std::max_element(hist.begin(), hist.end()) - hist.begin());
    centre[2] = static_cast<double>(peak % kBins) * kStep + kStep / 2.0;
    centre[1] = static_cast<double>((peak / kBins) % kBins) * kStep + kStep / 2.0;
    centre[0] = static_cast<double>(peak / (static_cast<std::size_t>(kBins) * kBins)) * kStep +
                kStep / 2.0;

    // Shift onto the mode. Medians rather than means, so a neighbouring mode
    // reaching into the window cannot drag the centre across into it.
    std::vector<unsigned char> window[3];
    for (int pass = 0; pass < 3; ++pass) {
        for (int c = 0; c < 3; ++c) window[c].clear();
        for (const cv::Vec3b& s : samples) {
            if (std::fabs(s[0] - centre[0]) > kStep || std::fabs(s[1] - centre[1]) > kStep ||
                std::fabs(s[2] - centre[2]) > kStep) {
                continue;
            }
            for (int c = 0; c < 3; ++c) window[c].push_back(s[c]);
        }
        if (window[0].size() < 64) break;
        for (int c = 0; c < 3; ++c) centre[c] = medianOf(window[c]);
    }

    for (int c = 0; c < 3; ++c) window[c].clear();
    for (const cv::Vec3b& s : samples) {
        if (std::fabs(s[0] - centre[0]) > 1.5 * kStep ||
            std::fabs(s[1] - centre[1]) > 1.5 * kStep ||
            std::fabs(s[2] - centre[2]) > 1.5 * kStep) {
            continue;
        }
        for (int c = 0; c < 3; ++c) {
            window[c].push_back(static_cast<unsigned char>(std::fabs(s[c] - centre[c])));
        }
    }
    // A mode that thin is not a surface, it is whatever happened to be the most
    // common colour in a picture with no dominant one.
    if (window[0].size() < 64 || window[0].size() * 20 < samples.size()) return false;

    for (int c = 0; c < 3; ++c) {
        // 1.4826 turns a median absolute deviation into the standard deviation
        // it would imply for a normal distribution. The floor is there because
        // a mat photographed under even light can come back with a MAD of zero
        // on a channel, and a tolerance of zero accepts nothing at all.
        tol[c] = std::max(6.0, sigmas * 1.4826 * medianOf(window[c]));
    }
    return true;
}

}  // namespace

bool matFootprint(const cv::Mat& bgr, const cv::Rect& seed, double pitchPx, const Config& cfg,
                  cv::Rect& out) {
    if (bgr.empty() || bgr.type() != CV_8UC3) return false;
    const cv::Rect frame(0, 0, bgr.cols, bgr.rows);
    const cv::Rect seedFull = seed & frame;
    if (seedFull.width < 16 || seedFull.height < 16) return false;

    // Work small.
    //
    // What is being looked for is where a mat half a metre across stops, and
    // that boundary is not a subtle feature: at a twentieth of the resolution it
    // is still tens of pixels from the last mat pixel to the first bench one.
    // The frame off this camera is twenty megapixels and calibration is
    // something an operator stands at the bench and waits through, so the whole
    // of this runs on a picture small enough not to be felt.
    const int longest = std::max(bgr.cols, bgr.rows);
    const double scale = (longest > 720) ? 720.0 / static_cast<double>(longest) : 1.0;
    cv::Mat small;
    if (scale < 1.0) {
        cv::resize(bgr, small, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
        small = bgr;
    }

    // Lab, because the bench is told from the mat by its colour at least as much
    // as by its brightness, and a shadow falling across a corner of the mat is
    // not a reason to call that corner bench.
    cv::Mat lab;
    cv::cvtColor(small, lab, cv::COLOR_BGR2Lab);

    cv::Rect seedSmall(static_cast<int>(std::lround(seedFull.x * scale)),
                       static_cast<int>(std::lround(seedFull.y * scale)),
                       static_cast<int>(std::lround(seedFull.width * scale)),
                       static_cast<int>(std::lround(seedFull.height * scale)));
    seedSmall &= cv::Rect(0, 0, lab.cols, lab.rows);
    if (seedSmall.width < 8 || seedSmall.height < 8) return false;

    // Sample from inside the grid rather than from its edge. The lattice
    // footprint is the bounding box of the outermost lines that were fitted, so
    // its border sits on printed ink by construction, and a colour sampled there
    // is the colour of the ink.
    const int insetX = seedSmall.width / 10;
    const int insetY = seedSmall.height / 10;
    const cv::Rect sample(seedSmall.x + insetX, seedSmall.y + insetY,
                          seedSmall.width - 2 * insetX, seedSmall.height - 2 * insetY);
    if (sample.width < 4 || sample.height < 4) return false;

    const int stride = std::max(
        1, static_cast<int>(std::lround(std::sqrt(static_cast<double>(sample.area()) / 20000.0))));
    std::vector<cv::Vec3b> samples;
    samples.reserve(24000);
    for (int yy = sample.y; yy < sample.y + sample.height; yy += stride) {
        const cv::Vec3b* row = lab.ptr<cv::Vec3b>(yy);
        for (int xx = sample.x; xx < sample.x + sample.width; xx += stride) {
            samples.push_back(row[xx]);
        }
    }

    double centre[3] = {0.0, 0.0, 0.0};
    double tol[3] = {0.0, 0.0, 0.0};
    if (!matColour(samples, std::max(0.5, cfg.roi_mat_colour_sigmas), centre, tol)) return false;

    // Grow out from the grid, rather than thresholding the whole picture at
    // once.
    //
    // A single window of accepted colour cannot survive controlled lighting. A
    // lamp over one end of the bench puts a brightness gradient across the mat
    // that is wider than any window tight enough to exclude a pale bench, and
    // the two failures trade off against each other with nothing in between:
    // hold brightness tightly and the far end of the mat drops out of the
    // working area, open it up to reach and a white table is admitted, because
    // a white table and a mat differ in brightness and barely at all in hue.
    //
    // The way out is that a lighting gradient is smooth and a material boundary
    // is a step. So the mat is filled outward from the grid, each pixel judged
    // against the neighbour it is reached from rather than against one global
    // colour: a gradient is followed however far it runs, because no single
    // step along it is large, and the mat's edge stops the fill dead whichever
    // side of it is brighter. Chroma is still checked globally, as a fence the
    // fill cannot cross - a brighter patch of the same mat keeps its hue, so
    // this costs nothing under any lighting, and it is what stops a fill that
    // finds a soft edge from wandering off across the bench.
    // Only wide enough to swallow a grid line. At this working size a printed
    // line is under a pixel across, and a window far wider than the thing it is
    // removing pulls the mat's own border in after it - twenty pixels of real
    // reach, given away to erase something two pixels wide.
    int smooth = static_cast<int>(std::lround(0.5 * pitchPx * scale));
    smooth = std::min(7, std::max(3, smooth));
    if ((smooth % 2) == 0) ++smooth;
    cv::Mat plain;
    // A median erases the printed grid without softening the mat's own border,
    // which is the one edge here that has to stay sharp.
    cv::medianBlur(small, plain, smooth);
    cv::Mat labPlain;
    cv::cvtColor(plain, labPlain, cv::COLOR_BGR2Lab);

    // 0 is open, 1 is fenced off, 255 is filled. The border ring floodFill
    // requires is left open so a mat that runs past the frame still fills.
    cv::Mat filled(labPlain.rows + 2, labPlain.cols + 2, CV_8U, cv::Scalar(0));
    // Exactly the mode's own window, not a multiple of it. Hue is the one
    // measurement here that a lamp does not move, so it is the one that can
    // afford to be held tightly, and holding it loosely gives away the only
    // thing that separates this mat from a bench of a different colour but
    // similar brightness: the bench two shades darker than the mat differs from
    // it by 22 levels of b, which a doubled window waves straight through.
    const double chromaFence[2] = {tol[1], tol[2]};
    for (int yy = 0; yy < labPlain.rows; ++yy) {
        const cv::Vec3b* row = labPlain.ptr<cv::Vec3b>(yy);
        unsigned char* dst = filled.ptr<unsigned char>(yy + 1) + 1;
        for (int xx = 0; xx < labPlain.cols; ++xx) {
            const bool hue = std::fabs(row[xx][1] - centre[1]) <= chromaFence[0] &&
                             std::fabs(row[xx][2] - centre[2]) <= chromaFence[1];
            dst[xx] = hue ? 0 : 1;
        }
    }

    // How far one step along the gradient may go.
    //
    // This is the mat's pixel-to-pixel noise, not the width of the window its
    // colour sits in, and the difference between the two is the whole
    // behaviour. The window has to be wide enough to hold a whole mat; a step
    // has only to cover the difference between one median-filtered pixel and
    // the next, which even under a lamp is a fraction of a level. Set from the
    // window instead, the allowance reaches about twenty levels, and twenty
    // levels is larger than the drop from this mat to a bench darker than it -
    // so the fill walks over the edge of the mat without registering that
    // anything happened and returns the whole room.
    //
    // tol is 4.5 robust deviations wide by construction, so dividing it back
    // down recovers roughly the one deviation that a single step should get.
    const double step = std::max(3.0, tol[0] / 6.0);
    const cv::Scalar slack(step, step, step);
    int seeds = 0;
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            const int px = sample.x + sample.width * (2 * j + 1) / 14;
            const int py = sample.y + sample.height * (2 * i + 1) / 14;
            if (filled.at<unsigned char>(py + 1, px + 1) != 0) continue;
            // Seed only where the mat plainly is, so a seed that lands on a
            // printed shape or on something left lying on the surface does not
            // become the thing that gets measured.
            const cv::Vec3b v = labPlain.at<cv::Vec3b>(py, px);
            if (std::fabs(v[0] - centre[0]) > tol[0] || std::fabs(v[1] - centre[1]) > tol[1] ||
                std::fabs(v[2] - centre[2]) > tol[2]) {
                continue;
            }
            cv::floodFill(labPlain, filled, cv::Point(px, py), cv::Scalar(), nullptr, slack, slack,
                          4 | cv::FLOODFILL_MASK_ONLY | (255 << 8));
            ++seeds;
        }
    }
    if (seeds == 0) return false;

    cv::Mat mask = (filled(cv::Rect(1, 1, labPlain.cols, labPlain.rows)) == 255);

    // Close over the printed shapes the fill had to route around.
    int bridge = static_cast<int>(std::lround(1.5 * pitchPx * scale));
    bridge = std::min(31, std::max(3, bridge));
    if ((bridge % 2) == 0) ++bridge;
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(bridge, bridge)));

    cv::Mat labels, stats, centroids;
    const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    if (n < 2) return false;

    // The mat is whichever region holds most of the grid, not whichever is
    // biggest. On a bench that is itself pale and wide, the biggest region is
    // the bench.
    std::vector<int> overlap(static_cast<std::size_t>(n), 0);
    for (int yy = sample.y; yy < sample.y + sample.height; ++yy) {
        const int* row = labels.ptr<int>(yy);
        for (int xx = sample.x; xx < sample.x + sample.width; ++xx) {
            const int id = row[xx];
            if (id > 0) ++overlap[static_cast<std::size_t>(id)];
        }
    }
    int best = 0;
    for (int i = 1; i < n; ++i) {
        if (overlap[static_cast<std::size_t>(i)] > overlap[static_cast<std::size_t>(best)]) best = i;
    }
    // A region that covers less than a quarter of the grid it is supposed to be
    // underneath has not found the mat, whatever else it has found.
    if (best == 0 || overlap[static_cast<std::size_t>(best)] * 4 < sample.area()) return false;

    const int bx = stats.at<int>(best, cv::CC_STAT_LEFT);
    const int by = stats.at<int>(best, cv::CC_STAT_TOP);
    const int bw = stats.at<int>(best, cv::CC_STAT_WIDTH);
    const int bh = stats.at<int>(best, cv::CC_STAT_HEIGHT);

    const double inv = 1.0 / scale;
    cv::Rect full(static_cast<int>(std::floor(bx * inv)), static_cast<int>(std::floor(by * inv)),
                  static_cast<int>(std::ceil(bw * inv)), static_cast<int>(std::ceil(bh * inv)));
    full &= frame;
    if (full.width < 16 || full.height < 16) return false;

    // A mat that does not contain the grid printed on it is not a mat, it is a
    // segmentation that went wrong. The slack is the resampling: a box derived
    // at a twentieth of the resolution can miss the seed by a pixel or two of
    // the original without that meaning anything.
    const int slackX = std::max(4, seedFull.width / 50);
    const int slackY = std::max(4, seedFull.height / 50);
    if (full.x > seedFull.x + slackX || full.y > seedFull.y + slackY ||
        full.x + full.width < seedFull.x + seedFull.width - slackX ||
        full.y + full.height < seedFull.y + seedFull.height - slackY) {
        return false;
    }

    out = full;
    return true;
}

bool roiFromModel(const PlaneModel& model, cv::Size frameSize, const Config& cfg, RoiBox& roi,
                  const cv::Mat* background, bool* fromMat) {
    if (fromMat) *fromMat = false;
    int x = 0, y = 0, w = 0, h = 0;
    if (!model.latticeFootprint(x, y, w, h)) return false;
    const double pitch = model.detectionNumber("pitch_px", 40.0);

    // Out past the last grid line the camera saw, not in from it.
    //
    // The working area used to stop a pitch inside the detected lattice, which
    // made the reach of the bench a property of how far the printed grid
    // happened to reach rather than of what the camera can see. On a view that
    // covers more mat than the grid does, that is reach thrown away, and the
    // reach along the rail is the hard limit on what can be measured at all: a
    // fold longer than it does not fail a tolerance, it comes back as the width
    // of the picture.
    //
    // Extrapolating the model this far is safe. It is a global fit to hundreds
    // of lattice points, not an interpolation between neighbouring ones, and a
    // few pitches past the last of them is a fraction of the span it was fitted
    // over. Set roi_extrapolation_pitches negative to inset instead.
    const int reach = static_cast<int>(std::lround(pitch * cfg.roi_extrapolation_pitches));
    const int margin = std::max(0, cfg.roi_frame_margin_px);
    const int fw = frameSize.width;
    const int fh = frameSize.height;
    int x1 = std::max(margin, x - reach);
    int y1 = std::max(margin, y - reach);
    int x2 = std::min(fw - margin, x + w + reach);
    int y2 = std::min(fh - margin, y + h + reach);

    // ...and then to the edge of the mat, in whichever direction that lies.
    //
    // The count of pitches above is a guess at where the mat ends and it is
    // wrong in both directions, not just one. On this bench the grid is printed
    // almost to the mat's border, so four pitches of extrapolation puts the
    // working area 288 px past the last grid line and roughly 35 mm of it lands
    // on the table beyond the mat. A rail drawn out there is not reach, it is a
    // rail across the bench top: the surface under it was never calibrated, and
    // anything the camera sees there - a hand, a cuff, the edge of the table
    // itself - is inside the working area and eligible to be measured.
    //
    // So where the mat is found, the mat decides, inward as readily as outward.
    // Two things bound it. It can never cut inside the lattice, because
    // matFootprint rejects any region that does not contain the grid, so the
    // worst case is a working area exactly as big as the calibrated footprint
    // and no measurement loses ground that was ever supported by evidence. And
    // it can never reach further than the cap, which is what stops a
    // segmentation that has run off onto the bench from quoting millimetres for
    // a surface the model was never fitted over.
    if (cfg.roi_follow_mat && background && !background->empty()) {
        cv::Rect mat;
        if (matFootprint(*background, cv::Rect(x, y, w, h), pitch, cfg, mat)) {
            const int cap =
                static_cast<int>(std::lround(pitch * std::max(0.0, cfg.roi_mat_max_pitches)));
            const int capX1 = std::max(margin, x - cap);
            const int capY1 = std::max(margin, y - cap);
            const int capX2 = std::min(fw - margin, x + w + cap);
            const int capY2 = std::min(fh - margin, y + h + cap);
            // Between the cap and the lattice: the mat's edge where it falls
            // inside that band, the band's own limit where it does not.
            const int mx1 = std::min(x, std::max(capX1, std::max(margin, mat.x)));
            const int my1 = std::min(y, std::max(capY1, std::max(margin, mat.y)));
            const int mx2 =
                std::max(x + w, std::min(capX2, std::min(fw - margin, mat.x + mat.width)));
            const int my2 =
                std::max(y + h, std::min(capY2, std::min(fh - margin, mat.y + mat.height)));
            if (mx2 - mx1 >= 50 && my2 - my1 >= 50) {
                if (fromMat) *fromMat = true;
                x1 = mx1;
                y1 = my1;
                x2 = mx2;
                y2 = my2;
            }
        }
    }

    if (x2 - x1 < 50 || y2 - y1 < 50) return false;
    roi = RoiBox{x1, y1, x2, y2};
    return true;
}

// The lens focal length, in pixels, recovered from the homography alone.
//
// A plane homography is K[r1 r2 t] up to scale, and r1 and r2 are two columns
// of a rotation matrix, so they are the same length and at right angles. With
// the principal point taken as the centre of the picture and no skew, that is
// two equations in the single unknown f, and either of them solves it.
//
// Neither is reliable on its own. Both divide by the perspective terms of the
// homography, and those go to zero as the camera approaches square-on to the
// mat: on this bench H[2][0] is 6e-06 against H[2][1] at 1e-04, which is to say
// the mount is tilted about one axis and hardly at all about the other, and the
// orthogonality constraint - which needs both - returns no real solution. The
// equal-length constraint uses the difference of the two columns' norms and
// still has something to work with. So both are tried, the first plausible
// answer is taken, and "plausible" is checked against the lens it implies
// rather than against the number itself.
bool focalFromHomography(const PlaneModel& model, cv::Size frameSize, const Config& cfg,
                         double& focalPx) {
    const cv::Matx33d H = model.H();
    const double cx = frameSize.width / 2.0;
    const double cy = frameSize.height / 2.0;

    const double a1 = H(0, 0) - cx * H(2, 0);
    const double b1 = H(1, 0) - cy * H(2, 0);
    const double c1 = H(2, 0);
    const double a2 = H(0, 1) - cx * H(2, 1);
    const double b2 = H(1, 1) - cy * H(2, 1);
    const double c2 = H(2, 1);

    const double candidates[2] = {
        (c1 * c2 != 0.0) ? -(a1 * a2 + b1 * b2) / (c1 * c2) : -1.0,
        (c2 * c2 - c1 * c1 != 0.0)
            ? ((a1 * a1 + b1 * b1) - (a2 * a2 + b2 * b2)) / (c2 * c2 - c1 * c1)
            : -1.0};

    for (const double fSq : candidates) {
        if (!(fSq > 0.0)) continue;
        const double f = std::sqrt(fSq);
        // Sanity is easier to judge in millimetres of glass than in pixels. A
        // bench camera is not wearing a 2 mm lens or a 200 mm one, and a
        // homography noisy enough to claim otherwise is a homography to ignore.
        const double lensMm = f * cfg.pixel_size_mm;
        if (lensMm < 3.0 || lensMm > 100.0) continue;
        focalPx = f;
        return true;
    }
    return false;
}

bool estimateCameraDistance(const PlaneModel& model, cv::Size frameSize, const Config& cfg,
                            double& distanceMm, bool* derived) {
    if (derived) *derived = false;
    const std::pair<double, double> scale =
        model.localMmPerPx(cv::Point2d(frameSize.width / 2.0, frameSize.height / 2.0));
    const double mmPerPx = (scale.first + scale.second) / 2.0;

    // How much bigger the mat is than its picture: one over the magnification.
    // A millimetre of mat covers pixel_size_mm of sensor divided by this.
    const double invMagnification = mmPerPx / cfg.pixel_size_mm;

    // ...and the distance is f x (1 + 1/m), not f x (1/m).
    //
    // The missing term is exactly one focal length, because f/m is the distance
    // from the lens's FRONT FOCAL POINT to the mat and the thin-lens object
    // distance is measured from the principal plane, which sits one focal
    // length behind it. On this bench that is 12 mm out of 697, which is 1.7%
    // of the parallax correction: 0.18 mm on the 710 mm body length at a 10 mm
    // fold. Small, and free, and it was biasing every corrected reading the
    // same way - the distance came out short, so the correction came out large,
    // so the reading came out short. That is the direction this pipeline
    // refuses to fail in everywhere else.
    //
    // What the correction actually wants is the distance to the entrance pupil
    // rather than to the principal plane, and on a retrofocus wide-angle those
    // differ by a few millimetres of barrel that no formula here can know. That
    // residual is under a percent of a percent of a reading, and a bench that
    // needs better than this should measure the distance rather than derive it:
    // --camera-distance-mm exists for exactly that, and is preferred over this
    // wherever both are available.
    if (cfg.lens_focal_length_mm.has_value() && *cfg.lens_focal_length_mm != 0.0) {
        distanceMm = *cfg.lens_focal_length_mm * (1.0 + invMagnification);
        return true;
    }

    // Nobody said what lens this is, so ask the calibration. Last resort, and
    // the reason is the one thing this function cannot tell the caller.
    //
    // It is still worth doing. The parallax correction it feeds is (d - h) / d,
    // and refusing to guess leaves the whole of it: an uncorrected 2 mm lay
    // reads 2 mm long on a 710 mm body length, which is a fifth of the
    // tolerance spent on nothing. So an estimate beats silence.
    //
    // But it is a bad estimate, and it gets worse as the bench gets better.
    // focalFromHomography reads f out of the perspective terms of the
    // homography, and those terms are only non-zero because the camera is
    // slightly tilted off the mat. Square the mount up - which is what anyone
    // chasing accuracy does - and the signal it depends on goes to zero.
    // Measured on this bench in September 2026: H(2,0) is 1.0e-05 against
    // H(2,1) at 9.6e-05, the orthogonality constraint has no real solution at
    // all, and the equal-norms one that survives returned 565 mm where the lens
    // spec and a tape both say about 690. That is a 17% error, and it is not
    // noise that averages out - a ±0.05% difference between the mat's printed x
    // and y pitch swings the answer from 452 mm to 659 mm.
    //
    // Which direction it fails in is what settles this. Too small a d makes the
    // correction too large and the reading too SHORT, and a reading that is too
    // short is indistinguishable from a garment out of spec. So: pass
    // --focal-length-mm if the lens is known, or --camera-distance-mm if it has
    // been measured. Both are preferred over this, and both are one flag.
    double focalPx = 0.0;
    if (!focalFromHomography(model, frameSize, cfg, focalPx)) return false;
    // The same f x (1 + 1/m), with the focal length arriving in pixels.
    distanceMm = focalPx * cfg.pixel_size_mm * (1.0 + invMagnification);
    if (derived) *derived = true;
    return true;
}

// Remembered because a figure that has to be typed on every start is a figure
// that gets left off, and what fills the gap is the guess above - silently, in
// the direction that reads short. That is not hypothetical: in September 2026
// this bench was started without the flag the morning after a recalibration,
// the guess came back at 428 mm against a real 698, and with the fold heights
// set every long step read up to 1.6 mm short.
//
// Kept in a file of its own, not in the calibration, because the calibration
// is rewritten from scratch every time the grid is read again and the lens has
// not changed when that happens.
std::string settleLens(Config& cfg, bool given, double givenMm) {
    const std::string& path = cfg.bench_file;
    vjson::Value bench;
    std::string error;
    bool unreadable = false;
    if (!vjson::parseFile(path, bench, error) || !bench.isObject()) {
        // Absent is the normal first run. Present and unreadable is worth saying.
        std::FILE* probe = std::fopen(path.c_str(), "rb");
        if (probe) {
            std::fclose(probe);
            unreadable = true;
        }
        bench = vjson::Value::object();
    }
    const double saved = bench.getNumber("lens_focal_length_mm", 0.0);
    const auto mm = [](double v) { return vjson::formatFixed(v, 1) + " mm"; };

    if (!given) {
        if (saved > 0.0) {
            cfg.lens_focal_length_mm = saved;
            return "Lens " + mm(saved) + ", as saved for this bench in " + path;
        }
        if (unreadable) {
            return path + " could not be read (" + error +
                   "), so no saved lens is being used. Pass --focal-length-mm to save it again.";
        }
        return std::string();
    }

    // Zero or less means "forget it", and leaves this run with no lens at all.
    if (!(givenMm > 0.0)) {
        cfg.lens_focal_length_mm.reset();
        if (!(saved > 0.0)) return std::string();
        for (std::size_t k = 0; k < bench.keys.size();) {
            if (bench.keys[k] == "lens_focal_length_mm" || bench.keys[k] == "lens_saved_at") {
                bench.keys.erase(bench.keys.begin() + static_cast<std::ptrdiff_t>(k));
                bench.items.erase(bench.items.begin() + static_cast<std::ptrdiff_t>(k));
            } else {
                ++k;
            }
        }
        if (!vjson::writeFile(path, bench, 2)) {
            return "The saved lens (" + mm(saved) + ") is not being used on this run, but " + path +
                   " could not be written, so it will be back on the next start.";
        }
        return "The saved lens (" + mm(saved) + ") is forgotten. The working distance will be " +
               "guessed from the calibration until one is given again.";
    }

    cfg.lens_focal_length_mm = givenMm;
    if (saved == givenMm) return "Lens " + mm(givenMm) + ", already saved in " + path;

    // A whole number is written as one: this file is for a person to read as
    // much as for the station, and "1.2e+01" is not how anybody writes a lens.
    bench.set("lens_focal_length_mm", std::floor(givenMm) == givenMm
                                          ? vjson::Value::integer(static_cast<long long>(givenMm))
                                          : vjson::Value::real(givenMm));
    bench.set("lens_saved_at", vjson::Value::str(timestampIso()));
    if (!vjson::writeFile(path, bench, 2)) {
        return "Lens " + mm(givenMm) + " for this run, but it could not be saved to " + path +
               " - give --focal-length-mm again next time.";
    }
    if (saved > 0.0) {
        return "Lens " + mm(givenMm) + " saved for this bench in " + path + ", replacing " +
               mm(saved) + ".";
    }
    return "Lens " + mm(givenMm) + " saved for this bench in " + path +
           ". It is used on every start from now on, with or without --focal-length-mm.";
}

namespace {

// The bench file as it stands, or an empty object when there is none yet.
vjson::Value readBench(const Config& cfg) {
    vjson::Value bench;
    std::string error;
    if (!vjson::parseFile(cfg.bench_file, bench, error) || !bench.isObject()) {
        bench = vjson::Value::object();
    }
    return bench;
}

void dropKeys(vjson::Value& bench, std::initializer_list<const char*> names) {
    for (std::size_t k = 0; k < bench.keys.size();) {
        bool drop = false;
        for (const char* key : names) drop = drop || bench.keys[k] == key;
        if (drop) {
            bench.keys.erase(bench.keys.begin() + static_cast<std::ptrdiff_t>(k));
            bench.items.erase(bench.items.begin() + static_cast<std::ptrdiff_t>(k));
        } else {
            ++k;
        }
    }
}

}  // namespace

// What a garment does to the base that nothing else does: it hides the printed
// grid. Glare dims the lines and a crease bends them, a new exposure scales
// them, a cloth moved on the table shifts them, and they still stand out from
// the ground around them. Under cloth they are gone.
//
// Measured on this bench's frames (squares 2 pitches across, ~900 to a frame):
// the clear rigid mat and the clear blue cloth, lamp on and off, at three
// exposures - not one square below 0.05 of the local level, most near 0.08
// (cloth) and 0.25 (mat). With a garment down, 18% to 72% of squares fell
// below 0.03: a white shirt on the cloth, a dark one on the mat, half a shirt.
//
// Read on a copy decimated until the pitch is about 16 px. The lines are
// averaged down, not lost, and it is their average this reads, so a quarter of
// this bench's frame gives the full-size answer to the per cent in ~40 ms.
double gridHiddenFraction(const cv::Mat& bgr, double pitchPx, const Config& cfg,
                          const RoiBox* area) {
    if (bgr.empty() || !(pitchPx > 0.0)) return -1.0;
    const int step = std::max(1, std::min(4, static_cast<int>(pitchPx / 16.0)));
    cv::Mat small = bgr;
    if (step > 1) {
        cv::resize(bgr, small, cv::Size(bgr.cols / step, bgr.rows / step), 0.0, 0.0,
                   cv::INTER_AREA);
    }
    cv::Mat gray8, gray;
    if (small.channels() == 3) {
        cv::cvtColor(small, gray8, cv::COLOR_BGR2GRAY);
    } else {
        gray8 = small;
    }
    gray8.convertTo(gray, CV_32F);

    const double pitch = pitchPx / step;
    cv::Mat local, level, detail;
    cv::GaussianBlur(gray, local, cv::Size(0, 0), std::max(1.0, pitch / 2.5));
    cv::GaussianBlur(gray, level, cv::Size(0, 0), pitch);
    // Either way round: a line printed dark and one printed bright both stand
    // out from the blur, by the same amount above and below it.
    cv::absdiff(gray, local, detail);

    int x0 = 0, y0 = 0, x1 = gray.cols, y1 = gray.rows;
    if (area && area->valid()) {
        x0 = std::max(x0, area->x1 / step);
        y0 = std::max(y0, area->y1 / step);
        x1 = std::min(x1, area->x2 / step);
        y1 = std::min(y1, area->y2 / step);
    }
    const int margin = static_cast<int>(std::lround(cfg.border_margin_px / static_cast<double>(step)));
    x0 = std::max(x0, margin);
    y0 = std::max(y0, margin);
    x1 = std::min(x1, gray.cols - margin);
    y1 = std::min(y1, gray.rows - margin);

    const int cell = std::max(4, static_cast<int>(std::lround(2.0 * pitch)));
    int cells = 0;
    int hidden = 0;
    for (int y = y0; y + cell <= y1; y += cell) {
        for (int x = x0; x + cell <= x1; x += cell) {
            const cv::Rect r(x, y, cell, cell);
            const double contrast =
                0.5 * cv::mean(detail(r))[0] / std::max(4.0, cv::mean(level(r))[0]);
            ++cells;
            if (contrast < cfg.base_grid_min_contrast) ++hidden;
        }
    }
    return cells > 0 ? static_cast<double>(hidden) / cells : -1.0;
}

CalibrationChange compareCalibrations(const PlaneModel& before, const PlaneModel& after, cv::Size frame) {
    CalibrationChange out;
    const double w = frame.width;
    const double h = frame.height;
    if (w <= 0.0 || h <= 0.0) return out;
    // Inset from the border, where a garment is actually measured.
    const cv::Point2d spans[6][2] = {
        {{0.10 * w, 0.50 * h}, {0.90 * w, 0.50 * h}}, {{0.50 * w, 0.10 * h}, {0.50 * w, 0.90 * h}},
        {{0.10 * w, 0.15 * h}, {0.90 * w, 0.15 * h}}, {{0.10 * w, 0.85 * h}, {0.90 * w, 0.85 * h}},
        {{0.12 * w, 0.10 * h}, {0.12 * w, 0.90 * h}}, {{0.88 * w, 0.10 * h}, {0.88 * w, 0.90 * h}}};
    const auto length = [](const PlaneModel& m, const cv::Point2d& a, const cv::Point2d& b) {
        const std::vector<cv::Point2d> mm = m.pxToMm({a, b});
        return std::hypot(mm[1].x - mm[0].x, mm[1].y - mm[0].y);
    };
    double pct[6];
    for (int k = 0; k < 6; ++k) {
        const double a = length(before, spans[k][0], spans[k][1]);
        const double b = length(after, spans[k][0], spans[k][1]);
        if (!(a > 0.0) || !std::isfinite(b)) return out;
        pct[k] = 100.0 * (b / a - 1.0);
    }
    out.valid = true;
    out.centre_pct = 0.5 * (pct[0] + pct[1]);
    out.low_pct = *std::min_element(pct, pct + 6);
    out.high_pct = *std::max_element(pct, pct + 6);
    return out;
}

// Measured with a tape from the camera to the surface, and remembered for the
// reason the lens is: a figure that has to be typed on every start gets left
// off. Given, it is also what the parallax correction uses in preference to the
// lens, because it does not depend on the printed grid being true - and on a
// printed cloth it may not be.
std::string settleDistance(Config& cfg, bool given, double givenMm) {
    const std::string& path = cfg.bench_file;
    vjson::Value bench = readBench(cfg);
    const double saved = bench.getNumber("camera_distance_mm", 0.0);
    const auto mm = [](double v) { return vjson::formatFixed(v, 1) + " mm"; };

    if (!given) {
        if (saved > 0.0) {
            cfg.camera_distance_mm = saved;
            return "Working distance " + mm(saved) + ", measured and saved for this bench in " + path;
        }
        return std::string();
    }
    if (!(givenMm > 0.0)) {
        cfg.camera_distance_mm.reset();
        if (!(saved > 0.0)) return std::string();
        dropKeys(bench, {"camera_distance_mm", "camera_distance_saved_at"});
        if (!vjson::writeFile(path, bench, 2)) {
            return "The saved working distance (" + mm(saved) + ") is not being used on this run, but " +
                   path + " could not be written, so it will be back on the next start.";
        }
        return "The saved working distance (" + mm(saved) + ") is forgotten. It is worked out from "
               "the lens again.";
    }
    cfg.camera_distance_mm = givenMm;
    if (saved == givenMm) return "Working distance " + mm(givenMm) + ", already saved in " + path;
    bench.set("camera_distance_mm", std::floor(givenMm) == givenMm
                                        ? vjson::Value::integer(static_cast<long long>(givenMm))
                                        : vjson::Value::real(givenMm));
    bench.set("camera_distance_saved_at", vjson::Value::str(timestampIso()));
    if (!vjson::writeFile(path, bench, 2)) {
        return "Working distance " + mm(givenMm) + " for this run, but it could not be saved to " +
               path + " - give --camera-distance-mm again next time.";
    }
    return "Working distance " + mm(givenMm) + " measured, saved for this bench in " + path +
           (saved > 0.0 ? ", replacing " + mm(saved) + "." : ". Every start uses it from now on.");
}

bool loadBenchDistortion(const Config& cfg, LensDistortion& out) {
    out = LensDistortion();
    const vjson::Value bench = readBench(cfg);
    const vjson::Value* d = bench.find("distortion");
    if (!d || !d->isObject()) return false;
    const vjson::Value* centre = d->find("centre");
    const vjson::Value* size = d->find("image_size");
    if (!centre || !centre->isArray() || centre->size() < 2 || !size || !size->isArray() ||
        size->size() < 2) {
        return false;
    }
    out.k1 = d->getNumber("k1", 0.0);
    out.k2 = d->getNumber("k2", 0.0);
    out.cx = centre->at(0).asNumber();
    out.cy = centre->at(1).asNumber();
    out.norm = d->getNumber("norm", 0.0);
    out.width = static_cast<int>(size->at(0).asNumber());
    out.height = static_cast<int>(size->at(1).asNumber());
    out.frames = static_cast<int>(d->getInt("frames", 0));
    out.fitted_at = d->getString("fitted_at");
    return out.norm > 0.0 && out.width > 0 && out.height > 0;
}

bool saveBenchDistortion(const Config& cfg, const LensDistortion& d, double rmseMm, int points) {
    vjson::Value bench = readBench(cfg);
    vjson::Value o = vjson::Value::object();
    o.set("k1", vjson::Value::real(d.k1));
    o.set("k2", vjson::Value::real(d.k2));
    vjson::Value centre = vjson::Value::array();
    centre.push(vjson::Value::real(d.cx));
    centre.push(vjson::Value::real(d.cy));
    o.set("centre", centre);
    o.set("norm", vjson::Value::real(d.norm));
    vjson::Value size = vjson::Value::array();
    size.push(vjson::Value::integer(d.width));
    size.push(vjson::Value::integer(d.height));
    o.set("image_size", size);
    o.set("frames", vjson::Value::integer(d.frames));
    o.set("points", vjson::Value::integer(points));
    o.set("rmse_mm", vjson::Value::real(rmseMm));
    o.set("fitted_at", vjson::Value::str(d.fitted_at));
    bench.set("distortion", o);
    return vjson::writeFile(cfg.bench_file, bench, 2);
}

// The lens does not change between recalibrations, so its distortion is kept
// where the lens is. Only a recalibration's homography, which is where the
// camera and the mat are, is fitted each time.
std::string settleDistortion(Config& cfg, bool forget) {
    const std::string& path = cfg.bench_file;
    LensDistortion d;
    const bool have = loadBenchDistortion(cfg, d);
    const bool present = readBench(cfg).has("distortion");
    const auto terms = [](const LensDistortion& v) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "k1 %+.6f, k2 %+.6f, fitted %s on %d pictures", v.k1, v.k2,
                      v.fitted_at.c_str(), v.frames);
        return std::string(buf);
    };
    if (forget) {
        cfg.lens_distortion.reset();
        if (!present) return std::string();
        vjson::Value bench = readBench(cfg);
        dropKeys(bench, {"distortion"});
        if (!vjson::writeFile(path, bench, 2)) {
            return "The saved lens distortion is not being used on this run, but " + path +
                   " could not be written, so it will be back on the next start.";
        }
        return "The saved lens distortion" + (have ? " (" + terms(d) + ")" : std::string()) +
               " is forgotten: every recalibration fits the radial terms afresh again.";
    }
    if (!have) {
        cfg.lens_distortion.reset();
        return present ? "The lens distortion in " + path +
                             " could not be read, so recalibrations fit the radial terms afresh. "
                             "Fit it again with verimeasure_cli --fit-distortion."
                       : std::string();
    }
    cfg.lens_distortion = d;
    return "Lens distortion " + terms(d) + ", as saved for this bench in " + path +
           ": a recalibration fits the homography only.";
}

namespace {

const char* const kExposureKeys[] = {"exposure_us", "exposure_level", "exposure_set_by",
                                     "exposure_saved_at"};

}  // namespace

std::string exposureText(double us) {
    return us >= 1000.0 ? vjson::formatFixed(us / 1000.0, 1) + " ms" : vjson::formatFixed(us, 0) + " us";
}

bool loadBenchExposure(const Config& cfg, BenchExposure& out) {
    out = BenchExposure();
    const vjson::Value bench = readBench(cfg);
    const double us = bench.getNumber("exposure_us", 0.0);
    if (!(us > 0.0)) return false;
    out.us = us;
    out.level = bench.getNumber("exposure_level", 0.0);
    out.byHand = bench.getString("exposure_set_by") == "hand";
    out.savedAt = bench.getString("exposure_saved_at");
    return true;
}

bool saveBenchExposure(const Config& cfg, const BenchExposure& e) {
    vjson::Value bench = readBench(cfg);
    bench.set("exposure_us", vjson::Value::integer(static_cast<long long>(std::llround(e.us))));
    if (e.level > 0.0) {
        bench.set("exposure_level", vjson::Value::integer(static_cast<long long>(std::llround(e.level))));
    } else {
        bench.set("exposure_level", vjson::Value::null());
    }
    bench.set("exposure_set_by", vjson::Value::str(e.byHand ? "hand" : "mat"));
    bench.set("exposure_saved_at", vjson::Value::str(timestampIso()));
    return vjson::writeFile(cfg.bench_file, bench, 2);
}

bool forgetBenchExposure(const Config& cfg) {
    vjson::Value bench = readBench(cfg);
    bool had = false;
    for (std::size_t k = 0; k < bench.keys.size();) {
        bool drop = false;
        for (const char* key : kExposureKeys) drop = drop || bench.keys[k] == key;
        if (drop) {
            bench.keys.erase(bench.keys.begin() + static_cast<std::ptrdiff_t>(k));
            bench.items.erase(bench.items.begin() + static_cast<std::ptrdiff_t>(k));
            had = true;
        } else {
            ++k;
        }
    }
    return !had || vjson::writeFile(cfg.bench_file, bench, 2);
}

std::string settleExposure(const Config& cfg, bool given, double givenUs) {
    if (!given) return std::string();
    const std::string& path = cfg.bench_file;
    if (!(givenUs > 0.0)) {
        BenchExposure old;
        const bool had = loadBenchExposure(cfg, old);
        if (!forgetBenchExposure(cfg)) {
            return path + " could not be written, so the saved exposure is still there and will be "
                          "used.";
        }
        return had ? "The saved exposure (" + exposureText(old.us) +
                         ") is forgotten. It is set again from the mat on this start, so keep the "
                         "mat clear while the station starts."
                   : std::string();
    }
    BenchExposure e;
    e.us = givenUs;
    e.byHand = true;
    if (!saveBenchExposure(cfg, e)) {
        return "Exposure " + exposureText(givenUs) + " could not be saved to " + path + ".";
    }
    return "Exposure " + exposureText(givenUs) + " saved for this bench in " + path +
           ", set by hand: every start uses it and Recalibrate leaves it alone. --exposure-us 0 "
           "hands it back to the station.";
}

std::string settleExposureTarget(Config& cfg, bool given, double level) {
    const std::string& path = cfg.bench_file;
    vjson::Value bench = readBench(cfg);
    const double saved = bench.getNumber("exposure_target_level", 0.0);
    const auto text = [](double v) { return vjson::formatFixed(v, 0); };
    if (!given) {
        if (saved > 0.0) {
            cfg.exposure_target_level = saved;
            return "The clear base is metered to " + text(saved) + ", as saved for this bench in " + path;
        }
        return std::string();
    }
    if (!(level > 0.0)) {
        cfg.exposure_target_level.reset();
        if (!(saved > 0.0)) return std::string();
        dropKeys(bench, {"exposure_target_level", "exposure_target_saved_at"});
        if (!vjson::writeFile(path, bench, 2)) {
            return "The saved metering target (" + text(saved) + ") is not being used on this run, but " +
                   path + " could not be written, so it will be back on the next start.";
        }
        return "The saved metering target (" + text(saved) + ") is forgotten; the clear base is metered to " +
               text(cfg.target_highlight_level * cfg.clear_mat_headroom) + " again from the next Recalibrate.";
    }
    cfg.exposure_target_level = level;
    if (saved == level) return "The clear base is metered to " + text(level) + ", already saved in " + path;
    bench.set("exposure_target_level", std::floor(level) == level
                                           ? vjson::Value::integer(static_cast<long long>(level))
                                           : vjson::Value::real(level));
    bench.set("exposure_target_saved_at", vjson::Value::str(timestampIso()));
    if (!vjson::writeFile(path, bench, 2)) {
        return "The clear base is metered to " + text(level) + " on this run, but it could not be saved to " +
               path + " - give --exposure-target again next time.";
    }
    return "The clear base is metered to " + text(level) + " from now on, saved for this bench in " + path +
           ". The exposure in use was set for the old target: clear the base and press Recalibrate to set "
           "it for this one.";
}

}  // namespace verimeasure
