#include "verimeasure/numeric.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <opencv2/core/utility.hpp>
#include "verimeasure/cv_compat.hpp"

namespace verimeasure {

double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    const std::size_t n = values.size();
    const std::size_t mid = n / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    const double hi = values[mid];
    if (n % 2 == 1) return hi;
    const double lo = *std::max_element(values.begin(), values.begin() + mid);
    return 0.5 * (lo + hi);
}

double medianOf(const float* data, std::size_t n) {
    if (n == 0) return 0.0;
    std::vector<double> v(n);
    for (std::size_t k = 0; k < n; ++k) v[k] = static_cast<double>(data[k]);
    return median(std::move(v));
}

double percentile(std::vector<double> values, double q) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double pos = (q / 100.0) * static_cast<double>(values.size() - 1);
    const double lo = std::floor(pos);
    const double hi = std::ceil(pos);
    const double frac = pos - lo;
    const double a = values[static_cast<std::size_t>(lo)];
    const double b = values[static_cast<std::size_t>(hi)];
    return a + (b - a) * frac;
}

double percentileOf(std::vector<float> values, double q) {
    if (values.empty()) return 0.0;
    const double pos = (q / 100.0) * static_cast<double>(values.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double frac = pos - std::floor(pos);

    // nth_element twice rather than a full sort: this runs over every pixel of
    // a 20 MP frame during calibration.
    std::nth_element(values.begin(), values.begin() + lo, values.end());
    const double a = static_cast<double>(values[lo]);
    if (hi == lo) return a;
    std::nth_element(values.begin() + lo + 1, values.begin() + hi, values.end());
    const double b = static_cast<double>(values[hi]);
    return a + (b - a) * frac;
}

double levenbergMarquardt(const ResidualFn& residualFn, std::vector<double>& x,
                          int iterations, double eps) {
    std::vector<double> r;
    residualFn(x, r);
    double cost = 0.0;
    for (double v : r) cost += v * v;

    double lam = 1e-3;
    const int n = static_cast<int>(x.size());
    const int m = static_cast<int>(r.size());
    if (m == 0 || n == 0) return cost;

    cv::Mat J(m, n, CV_64F);
    std::vector<double> rp, xp, xNew, rNew;
    std::vector<double> dx(n, 0.0);

    for (int iter = 0; iter < iterations; ++iter) {
        for (int i = 0; i < n; ++i) {
            const double step = eps * std::max(1.0, std::abs(x[i]));
            xp = x;
            xp[i] += step;
            residualFn(xp, rp);
            for (int k = 0; k < m; ++k) {
                J.at<double>(k, i) = (rp[k] - r[k]) / step;
            }
        }

        cv::Mat rMat(m, 1, CV_64F, r.data());
        cv::Mat A = J.t() * J;
        cv::Mat g = J.t() * rMat;

        std::vector<double> diag(n);
        for (int i = 0; i < n; ++i) {
            double v = A.at<double>(i, i);
            diag[i] = (v <= 0.0) ? 1e-12 : v;
        }

        bool improved = false;
        for (int attempt = 0; attempt < 12; ++attempt) {
            cv::Mat damped = A.clone();
            for (int i = 0; i < n; ++i) damped.at<double>(i, i) += lam * diag[i];

            cv::Mat dxMat;
            if (!cv::solve(damped, -g, dxMat, cv::DECOMP_LU)) break;

            xNew.assign(n, 0.0);
            for (int i = 0; i < n; ++i) {
                dx[i] = dxMat.at<double>(i, 0);
                xNew[i] = x[i] + dx[i];
            }
            residualFn(xNew, rNew);
            double costNew = 0.0;
            for (double v : rNew) costNew += v * v;

            if (costNew < cost) {
                x = xNew;
                r = rNew;
                cost = costNew;
                lam = std::max(lam * 0.3, 1e-10);
                improved = true;
                break;
            }
            lam *= 4.0;
        }

        if (!improved) break;

        double norm = 0.0;
        for (double v : dx) norm += v * v;
        if (std::sqrt(norm) < 1e-10) break;
    }

    return cost;
}

void bilinearSample(const cv::Mat& img, const std::vector<double>& xs,
                    const std::vector<double>& ys, std::vector<double>& out,
                    std::vector<unsigned char>& valid) {
    const int h = img.rows;
    const int w = img.cols;
    const std::size_t n = xs.size();
    out.assign(n, 0.0);
    valid.assign(n, 0);

    for (std::size_t k = 0; k < n; ++k) {
        const double xv = xs[k];
        const double yv = ys[k];
        const int x0 = static_cast<int>(std::floor(xv));
        const int y0 = static_cast<int>(std::floor(yv));
        const int x1 = x0 + 1;
        const int y1 = y0 + 1;
        valid[k] = (x0 >= 0 && y0 >= 0 && x1 < w && y1 < h) ? 1 : 0;

        const int x0c = std::min(std::max(x0, 0), w - 1);
        const int x1c = std::min(std::max(x1, 0), w - 1);
        const int y0c = std::min(std::max(y0, 0), h - 1);
        const int y1c = std::min(std::max(y1, 0), h - 1);
        const double fx = xv - x0;
        const double fy = yv - y0;

        const double v00 = img.at<float>(y0c, x0c);
        const double v01 = img.at<float>(y0c, x1c);
        const double v10 = img.at<float>(y1c, x0c);
        const double v11 = img.at<float>(y1c, x1c);

        out[k] = v00 * (1 - fx) * (1 - fy) + v01 * fx * (1 - fy) +
                 v10 * (1 - fx) * fy + v11 * fx * fy;
    }
}

void interp(const std::vector<double>& targets, const std::vector<double>& xp,
            const std::vector<double>& fp, std::vector<double>& out) {
    out.assign(targets.size(), 0.0);
    if (xp.empty()) return;
    for (std::size_t k = 0; k < targets.size(); ++k) {
        const double t = targets[k];
        if (t <= xp.front()) { out[k] = fp.front(); continue; }
        if (t >= xp.back()) { out[k] = fp.back(); continue; }
        const auto it = std::upper_bound(xp.begin(), xp.end(), t);
        const std::size_t hi = static_cast<std::size_t>(it - xp.begin());
        const std::size_t lo = hi - 1;
        const double span = xp[hi] - xp[lo];
        const double frac = (span > 0.0) ? (t - xp[lo]) / span : 0.0;
        out[k] = fp[lo] + (fp[hi] - fp[lo]) * frac;
    }
}

cv::Mat toU8Truncate(const cv::Mat& src32f) {
    const int channels = src32f.channels();
    cv::Mat out(src32f.size(), CV_8UC(channels));
    const int width = src32f.cols * channels;
    for (int y = 0; y < src32f.rows; ++y) {
        const float* s = src32f.ptr<float>(y);
        unsigned char* d = out.ptr<unsigned char>(y);
        for (int x = 0; x < width; ++x) {
            double v = std::trunc(static_cast<double>(s[x]));
            if (v < 0.0) v = 0.0;
            if (v > 255.0) v = 255.0;
            d[x] = static_cast<unsigned char>(v);
        }
    }
    return out;
}

cv::Mat strideSample(const cv::Mat& src, int step) {
    if (step <= 1) return src;
    const int rows = (src.rows + step - 1) / step;
    const int cols = (src.cols + step - 1) / step;
    cv::Mat out(rows, cols, src.type());
    const std::size_t elem = src.elemSize();
    for (int y = 0; y < rows; ++y) {
        const unsigned char* s = src.ptr<unsigned char>(y * step);
        unsigned char* d = out.ptr<unsigned char>(y);
        for (int x = 0; x < cols; ++x) {
            std::memcpy(d + static_cast<std::size_t>(x) * elem,
                        s + static_cast<std::size_t>(x) * step * elem, elem);
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// The same results, computed faster
// ---------------------------------------------------------------------------
//
// Read the note in numeric.hpp before changing anything here. Every function
// below has a plainer equivalent it has to agree with to the last bit, and the
// calibration harness checks exactly that against a frozen copy of the old
// code. A change that is merely very close is a wrong change.

namespace {

// The kernel OpenCV builds for GaussianBlur(src, dst, Size(0,0), sigma) on a
// float image: four sigma each side, forced odd. Mirrored here so the banded
// forms know how much halo they have to reach for.
int gaussianKernelSize(double sigma) {
    return cvRound(sigma * 4 * 2 + 1) | 1;
}

// Column strip width for the parallel passes. Sized so one ring of ksize rows
// of a strip stays inside a core's L2, which is what makes the split a win
// rather than a loss. See the note in the header.
int blurStripWidth() { return 96; }

// One separable half. axis 0 filters along x, axis 1 along y. The other axis
// gets a 1x1 identity kernel, so this is exactly one of the two passes
// GaussianBlur performs, and the pair of them reproduces it.
//
// The row pass splits over row stripes and the column pass over column strips,
// because in each case that is the direction along which outputs are
// independent AND the working set stays small. Both take their halo from the
// parent of whatever view they are handed.
void separablePass(const cv::Mat& src, cv::Mat& dst, const cv::Mat& kernel, int axis,
                   int stripW) {
    const cv::Mat identity = cv::Mat::ones(1, 1, CV_32F);
    const cv::Mat& kx = (axis == 0) ? kernel : identity;
    const cv::Mat& ky = (axis == 0) ? identity : kernel;

    if (axis == 0) {
        const int stripes = std::max(1, std::min(src.rows, cv::getNumThreads() * 4));
        cv::parallel_for_(cv::Range(0, stripes), [&](const cv::Range& r) {
            for (int s = r.start; s < r.end; ++s) {
                const int y0 = static_cast<int>(static_cast<long long>(src.rows) * s / stripes);
                const int y1 =
                    static_cast<int>(static_cast<long long>(src.rows) * (s + 1) / stripes);
                if (y1 <= y0) continue;
                cv::Mat si = src.rowRange(y0, y1);
                cv::Mat di = dst.rowRange(y0, y1);
                cv::sepFilter2D(si, di, CV_32F, kx, ky, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
            }
        });
    } else {
        const int strips = (src.cols + stripW - 1) / stripW;
        cv::parallel_for_(cv::Range(0, strips), [&](const cv::Range& r) {
            for (int s = r.start; s < r.end; ++s) {
                const int x0 = s * stripW;
                const int x1 = std::min(src.cols, x0 + stripW);
                cv::Mat si = src.colRange(x0, x1);
                cv::Mat di = dst.colRange(x0, x1);
                cv::sepFilter2D(si, di, CV_32F, kx, ky, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
            }
        });
    }
}

// Exact order statistics under a key that orders the values the same way they
// order themselves. Counting and comparison only: no value is ever added,
// scaled or rounded, so each answer is an element of the input, the very one
// nth_element would have picked. The parallel parts accumulate integer counts
// and gather members, both order-independent, so the result does not depend on
// how the work happened to be divided between cores.
template <typename Key, typename KeyFn>
std::vector<Key> exactOrderStats(const cv::Mat& m, const std::vector<std::size_t>& ranks,
                                 KeyFn key, int shift) {
    const int kBuckets = 65536;

    // One chunk per core, no more. Each chunk builds a private 65536-entry
    // tally, which is half a megabyte to allocate and clear, so letting OpenCV
    // split by rows in its usual way would pay that toll hundreds of times and
    // spend ten times longer on bookkeeping than on the pixels.
    const int chunks = std::max(1, cv::getNumThreads());

    std::vector<std::size_t> hist(kBuckets, 0);
    std::mutex histLock;
    cv::parallel_for_(cv::Range(0, m.rows), [&](const cv::Range& r) {
        std::vector<std::size_t> local(kBuckets, 0);
        for (int y = r.start; y < r.end; ++y) {
            const float* p = m.ptr<float>(y);
            for (int x = 0; x < m.cols; ++x) {
                ++local[static_cast<std::size_t>(key(p[x]) >> shift)];
            }
        }
        std::lock_guard<std::mutex> guard(histLock);
        for (int i = 0; i < kBuckets; ++i) hist[i] += local[i];
    }, chunks);

    std::vector<Key> out;
    out.reserve(ranks.size());
    for (std::size_t k : ranks) {
        std::size_t below = 0;
        std::size_t bucket = 0;
        for (; bucket + 1 < static_cast<std::size_t>(kBuckets); ++bucket) {
            if (below + hist[bucket] > k) break;
            below += hist[bucket];
        }

        std::vector<Key> members;
        members.reserve(hist[bucket]);
        std::mutex memberLock;
        cv::parallel_for_(cv::Range(0, m.rows), [&](const cv::Range& r) {
            std::vector<Key> local;
            for (int y = r.start; y < r.end; ++y) {
                const float* p = m.ptr<float>(y);
                for (int x = 0; x < m.cols; ++x) {
                    const Key u = key(p[x]);
                    if (static_cast<std::size_t>(u >> shift) == bucket) local.push_back(u);
                }
            }
            std::lock_guard<std::mutex> guard(memberLock);
            members.insert(members.end(), local.begin(), local.end());
        }, chunks);

        const std::size_t offset = k - below;
        std::nth_element(members.begin(), members.begin() + offset, members.end());
        out.push_back(members[offset]);
    }
    return out;
}

// Bit pattern of a non-negative float, which orders identically to the float
// itself. Everything these helpers rank is a blur, a magnitude or an absolute
// deviation, so none of it is negative.
std::uint32_t floatOrderKey(float f) {
    std::uint32_t u = 0;
    std::memcpy(&u, &f, sizeof(u));
    return u;
}

float floatFromKey(std::uint32_t u) {
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

double doubleFromKey(std::uint64_t u) {
    double d = 0.0;
    std::memcpy(&d, &u, sizeof(d));
    return d;
}

}  // namespace

void gaussianBlurFast(const cv::Mat& src, cv::Mat& dst, double sigma) {
    const int stripW = blurStripWidth();

    // Too small to be worth dividing, or not the float type the fast path was
    // measured on. Hand it to OpenCV and let it do what it always did.
    if (src.type() != CV_32FC1 || src.cols <= stripW || src.rows < 8) {
        cv::GaussianBlur(src, dst, cv::Size(0, 0), sigma);
        return;
    }

    dst.create(src.size(), src.type());
    const int strips = (src.cols + stripW - 1) / stripW;
    cv::parallel_for_(cv::Range(0, strips), [&](const cv::Range& r) {
        for (int s = r.start; s < r.end; ++s) {
            const int x0 = s * stripW;
            const int x1 = std::min(src.cols, x0 + stripW);
            // Views, not copies. The horizontal halo either side of a strip
            // comes from the neighbouring strip's pixels in the parent, so the
            // seams are invisible in the output because they are not seams.
            cv::Mat si = src.colRange(x0, x1);
            cv::Mat di = dst.colRange(x0, x1);
            cv::GaussianBlur(si, di, cv::Size(0, 0), sigma);
        }
    });
}

cv::Mat gaussianBlurRowBand(const cv::Mat& src, int y0, int y1, double sigma) {
    const cv::Mat kernel = cv::getGaussianKernel(gaussianKernelSize(sigma), sigma, CV_32F);
    const int radius = kernel.rows / 2;
    const int stripW = blurStripWidth();

    // The horizontal pass is needed over the band plus a radius of rows either
    // side, because the vertical pass that follows reaches that far. Where the
    // band sits close enough to the top or bottom that the reach runs off the
    // image, it stops at the edge and the vertical pass reflects there, which
    // is what the full-frame blur does at that same edge.
    const int ya = std::max(0, y0 - radius);
    const int yb = std::min(src.rows, y1 + radius);

    cv::Mat rowFiltered(yb - ya, src.cols, CV_32F);
    separablePass(src.rowRange(ya, yb), rowFiltered, kernel, 0, stripW);

    cv::Mat out(y1 - y0, src.cols, CV_32F);
    separablePass(rowFiltered.rowRange(y0 - ya, y1 - ya), out, kernel, 1, stripW);
    return out;
}

cv::Mat gaussianBlurColBand(const cv::Mat& src, int x0, int x1, double sigma) {
    const cv::Mat kernel = cv::getGaussianKernel(gaussianKernelSize(sigma), sigma, CV_32F);
    const int stripW = blurStripWidth();

    // No vertical halo to arrange for: the band is full height already. The
    // horizontal pass takes its halo from the columns either side in the
    // parent, which is why it is handed a view rather than a copy.
    cv::Mat rowFiltered(src.rows, x1 - x0, CV_32F);
    separablePass(src.colRange(x0, x1), rowFiltered, kernel, 0, stripW);

    cv::Mat out(src.rows, x1 - x0, CV_32F);
    separablePass(rowFiltered, out, kernel, 1, stripW);
    return out;
}

double medianOfMat(const cv::Mat& m32f) {
    const std::size_t n = m32f.total();
    if (n == 0) return 0.0;
    const std::size_t mid = n / 2;

    // median() averages the two middle values on an even count and takes the
    // middle one on an odd count. Mirrored exactly, including the widening to
    // double that happens before the average.
    if (n % 2 == 1) {
        const std::vector<std::uint32_t> k =
            exactOrderStats<std::uint32_t>(m32f, {mid}, floatOrderKey, 16);
        return static_cast<double>(floatFromKey(k[0]));
    }
    const std::vector<std::uint32_t> k =
        exactOrderStats<std::uint32_t>(m32f, {mid - 1, mid}, floatOrderKey, 16);
    return 0.5 * (static_cast<double>(floatFromKey(k[0])) +
                  static_cast<double>(floatFromKey(k[1])));
}

double medianAbsDevFromMat(const cv::Mat& m32f, double centre) {
    const std::size_t n = m32f.total();
    if (n == 0) return 0.0;
    const std::size_t mid = n / 2;

    // The deviations are doubles, so they are ranked by the leading bits of the
    // double pattern rather than the float one. The subtraction is done the
    // same way the direct route does it, on each value in turn.
    const auto key = [centre](float f) {
        const double d = std::abs(static_cast<double>(f) - centre);
        std::uint64_t u = 0;
        std::memcpy(&u, &d, sizeof(u));
        return u;
    };

    if (n % 2 == 1) {
        const std::vector<std::uint64_t> k = exactOrderStats<std::uint64_t>(m32f, {mid}, key, 48);
        return doubleFromKey(k[0]);
    }
    const std::vector<std::uint64_t> k =
        exactOrderStats<std::uint64_t>(m32f, {mid - 1, mid}, key, 48);
    return 0.5 * (doubleFromKey(k[0]) + doubleFromKey(k[1]));
}

double percentileOfMat(const cv::Mat& m32f, double q) {
    const std::size_t n = m32f.total();
    if (n == 0) return 0.0;

    // percentileOf's interpolation, to the letter: the rank is a fractional
    // position between two order statistics and the answer is the linear blend
    // of them. Same two elements, same weights, same result.
    const double pos = (q / 100.0) * static_cast<double>(n - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(pos));
    const double frac = pos - std::floor(pos);

    if (hi == lo) {
        const std::vector<std::uint32_t> k =
            exactOrderStats<std::uint32_t>(m32f, {lo}, floatOrderKey, 16);
        return static_cast<double>(floatFromKey(k[0]));
    }
    const std::vector<std::uint32_t> k =
        exactOrderStats<std::uint32_t>(m32f, {lo, hi}, floatOrderKey, 16);
    const double a = static_cast<double>(floatFromKey(k[0]));
    const double b = static_cast<double>(floatFromKey(k[1]));
    return a + (b - a) * frac;
}

}  // namespace verimeasure
