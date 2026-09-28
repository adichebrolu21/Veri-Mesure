#include "verimeasure/creep.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "verimeasure/cv_compat.hpp"

#include "verimeasure/numeric.hpp"

namespace verimeasure {

namespace {

// One patch of bare base: where it is, how far the base moved there, and how
// sure the correlation was.
struct Patch {
    double x = 0.0;
    double y = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double w = 0.0;
};

// The patches, filed by square so a question about one place only looks at
// the patches near it. The squares are three sigma across, which is as far as
// a patch is ever given any weight, so the nine round a place hold every patch
// that can count there.
class PatchIndex {
public:
    PatchIndex(const std::vector<Patch>& pts, const cv::Rect& region, double size)
        : pts_(pts), x0_(region.x), y0_(region.y), size_(size) {
        cols_ = std::max(1, static_cast<int>(std::ceil(region.width / size)) + 1);
        rows_ = std::max(1, static_cast<int>(std::ceil(region.height / size)) + 1);
        cells_.resize(static_cast<std::size_t>(cols_) * rows_);
        for (std::size_t k = 0; k < pts.size(); ++k) {
            cells_[static_cast<std::size_t>(cellOf(pts[k].y, y0_, rows_)) * cols_ +
                   cellOf(pts[k].x, x0_, cols_)]
                .push_back(static_cast<int>(k));
        }
    }

    template <class Visit>
    void near(double qx, double qy, Visit&& visit) const {
        const int cx = cellOf(qx, x0_, cols_);
        const int cy = cellOf(qy, y0_, rows_);
        for (int j = std::max(0, cy - 1); j <= std::min(rows_ - 1, cy + 1); ++j) {
            for (int i = std::max(0, cx - 1); i <= std::min(cols_ - 1, cx + 1); ++i) {
                for (int k : cells_[static_cast<std::size_t>(j) * cols_ + i]) visit(k);
            }
        }
    }

    template <class Visit>
    void all(Visit&& visit) const {
        for (std::size_t k = 0; k < pts_.size(); ++k) visit(static_cast<int>(k));
    }

private:
    int cellOf(double v, double origin, int n) const {
        return std::clamp(static_cast<int>(std::floor((v - origin) / size_)), 0, n - 1);
    }

    const std::vector<Patch>& pts_;
    double x0_;
    double y0_;
    double size_;
    int cols_ = 1;
    int rows_ = 1;
    std::vector<std::vector<int>> cells_;
};

// The movement at (qx, qy): a plane through the patches' movements, each
// weighted by its response and a Gaussian of its distance. `skip` leaves one
// patch out, which is how a patch is asked whether it agrees with the rest.
// `wide` looks at every patch, for a place further than three sigma from any.
// False where no patch is near enough to say.
//
// A plane rather than a plain average, because the movement is not even and
// the patches that decide it at a garment's edge all lie on one side of it: an
// average there is the movement a patch-width out onto the base, where a plane
// carries the gradient the last few tens of pixels in. The slopes are held by
// a light ridge, so where the patches are too few or too lopsided to hold a
// slope it leans back to the average rather than extrapolating one.
bool fitAt(const std::vector<Patch>& pts, const std::vector<char>& keep, const PatchIndex& index,
           double qx, double qy, double sigma, int skip, bool wide, double& fx, double& fy) {
    double s0 = 0.0, su = 0.0, sv = 0.0, suu = 0.0, suv = 0.0, svv = 0.0;
    double x0 = 0.0, xu = 0.0, xv = 0.0, y0 = 0.0, yu = 0.0, yv = 0.0;
    const double reach2 = 9.0 * sigma * sigma;
    const double inv2s2 = 1.0 / (2.0 * sigma * sigma);
    const auto add = [&](int k) {
        const Patch& p = pts[static_cast<std::size_t>(k)];
        if (!keep[static_cast<std::size_t>(k)] || k == skip) return;
        const double ex = p.x - qx;
        const double ey = p.y - qy;
        const double d2 = ex * ex + ey * ey;
        if (d2 > reach2) return;
        const double w = p.w * std::exp(-d2 * inv2s2);
        const double u = ex / sigma;
        const double v = ey / sigma;
        s0 += w;
        su += w * u;
        sv += w * v;
        suu += w * u * u;
        suv += w * u * v;
        svv += w * v * v;
        x0 += w * p.dx;
        xu += w * u * p.dx;
        xv += w * v * p.dx;
        y0 += w * p.dy;
        yu += w * u * p.dy;
        yv += w * v * p.dy;
    };
    if (wide) {
        index.all(add);
    } else {
        index.near(qx, qy, add);
    }
    if (s0 < 1e-9) return false;
    const double ridge = 0.05 * s0;
    const cv::Matx33d m(s0, su, sv, su, suu + ridge, suv, sv, suv, svv + ridge);
    const cv::Matx31d ax = m.solve(cv::Matx31d(x0, xu, xv), cv::DECOMP_CHOLESKY);
    const cv::Matx31d ay = m.solve(cv::Matx31d(y0, yu, yv), cv::DECOMP_CHOLESKY);
    fx = ax(0);
    fy = ay(0);
    return std::isfinite(fx) && std::isfinite(fy);
}

}  // namespace

BaseCreep alignCreptBase(const cv::Mat& frameBgr, const cv::Mat& backgroundBgr,
                         const cv::Mat& lay, int step, const cv::Rect& crop, const Config& cfg,
                         cv::Mat& alignedCrop) {
    BaseCreep out;
    CV_Assert(frameBgr.type() == CV_8UC3 && backgroundBgr.type() == CV_8UC3);
    CV_Assert(frameBgr.size() == backgroundBgr.size());
    if (lay.empty() || step < 1 || crop.width < 1 || crop.height < 1) return out;

    // Read at half resolution, in patches of 128 working-area pixels: nearly
    // two of this base's printed squares, which is what a correlation needs to
    // lock onto a grid, and a quarter of the work of reading them at full
    // resolution. The movement still comes back to a tenth of a pixel.
    const int patch = 64;
    const int stride = 32;
    const int patchFull = 2 * patch;
    const cv::Rect area(0, 0, frameBgr.cols, frameBgr.rows);
    const int band = std::max(0, cfg.base_creep_band_px);
    const cv::Rect region =
        cv::Rect(crop.x - band, crop.y - band, crop.width + 2 * band, crop.height + 2 * band) & area;
    if (region.width < 2 * patchFull || region.height < 2 * patchFull) return out;

    cv::Mat f2, b2;
    {
        cv::Mat g;
        const cv::Size half(region.width / 2, region.height / 2);
        cv::cvtColor(frameBgr(region), g, cv::COLOR_BGR2GRAY);
        cv::resize(g, f2, half, 0, 0, cv::INTER_AREA);
        cv::cvtColor(backgroundBgr(region), g, cv::COLOR_BGR2GRAY);
        cv::resize(g, b2, half, 0, 0, cv::INTER_AREA);
        f2.convertTo(f2, CV_32F);
        b2.convertTo(b2, CV_32F);
    }

    // The garment is not base, and neither is the rim round it where the
    // decimated outline is a cell or so out and its shadow lies: a patch is
    // read only if the lay grown by base_creep_margin_px covers little of it.
    // And only near the garment, where the movement decides an edge; base
    // further out adds work and nothing else.
    const cv::Mat layBin = lay > 0;
    cv::Mat grown;
    {
        const int r = std::max(0, (cfg.base_creep_margin_px + step - 1) / step);
        cv::dilate(layBin, grown,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
    }
    cv::Mat grownSum;
    cv::integral(grown / 255, grownSum, CV_32S);
    cv::Mat toLay;
    cv::distanceTransform(~layBin, toLay, cv::DIST_L2, 3);
    const auto cellX = [&](int x) { return std::clamp(x / step, 0, lay.cols); };
    const auto cellY = [&](int y) { return std::clamp(y / step, 0, lay.rows); };

    cv::Mat hann;
    cv::createHanningWindow(hann, cv::Size(patch, patch), CV_32F);

    std::vector<Patch> pts;
    for (int y = 0; y + patch <= f2.rows; y += stride) {
        for (int x = 0; x + patch <= f2.cols; x += stride) {
            // The patch in working-area pixels, and the cells of the lay under it.
            const int X0 = region.x + 2 * x;
            const int Y0 = region.y + 2 * y;
            const int X1 = X0 + patchFull;
            const int Y1 = Y0 + patchFull;
            const int cx0 = cellX(X0);
            const int cy0 = cellY(Y0);
            const int cx1 = std::min(lay.cols, std::max(cx0 + 1, cellX(X1 + step - 1)));
            const int cy1 = std::min(lay.rows, std::max(cy0 + 1, cellY(Y1 + step - 1)));
            if (cx1 <= cx0 || cy1 <= cy0) continue;
            const double cells = static_cast<double>(cx1 - cx0) * (cy1 - cy0);
            const double covered = grownSum.at<int>(cy1, cx1) - grownSum.at<int>(cy0, cx1) -
                                   grownSum.at<int>(cy1, cx0) + grownSum.at<int>(cy0, cx0);
            if (covered > cfg.base_creep_max_cover * cells) continue;
            const int mx = std::min(lay.cols - 1, (X0 + X1) / 2 / step);
            const int my = std::min(lay.rows - 1, (Y0 + Y1) / 2 / step);
            if (toLay.at<float>(my, mx) * step > band + patchFull / 2) continue;

            const cv::Rect r(x, y, patch, patch);
            cv::Scalar mean, sd;
            cv::meanStdDev(b2(r), mean, sd);
            // Bare base with nothing printed on it has nothing to be moved by.
            if (sd[0] < 2.0) continue;
            // phaseCorrelate multiplies the window into its inputs in place
            // when they are already a DFT size, so it is handed copies.
            double response = 0.0;
            const cv::Point2d d = cv::phaseCorrelate(b2(r).clone(), f2(r).clone(), hann, &response);
            if (!(response >= cfg.base_creep_min_response)) continue;
            Patch p;
            p.x = 0.5 * (X0 + X1);
            p.y = 0.5 * (Y0 + Y1);
            p.dx = 2.0 * d.x;
            p.dy = 2.0 * d.y;
            p.w = response;
            pts.push_back(p);
        }
    }
    if (pts.empty()) return out;

    // A patch that disagrees with the base round it by more than
    // base_creep_outlier_px is reading something else - a stray thread, a
    // corner of garment the decimated outline left out - and is dropped. Each
    // pass judges every patch against the same set.
    const double sigma = std::max(16.0, cfg.base_creep_smooth_px);
    const PatchIndex index(pts, region, 3.0 * sigma);
    std::vector<char> keep(pts.size(), 1);
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<char> next(keep);
        for (std::size_t k = 0; k < pts.size(); ++k) {
            double fx = 0.0, fy = 0.0;
            if (!fitAt(pts, keep, index, pts[k].x, pts[k].y, sigma, static_cast<int>(k), false, fx,
                       fy)) {
                continue;
            }
            next[k] = std::hypot(fx - pts[k].dx, fy - pts[k].dy) <= cfg.base_creep_outlier_px;
        }
        keep = next;
    }
    double loX = 1e9, hiX = -1e9, loY = 1e9, hiY = -1e9, sw = 0.0, swx = 0.0, swy = 0.0;
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if (!keep[k]) continue;
        ++out.patches;
        loX = std::min(loX, pts[k].dx);
        hiX = std::max(hiX, pts[k].dx);
        loY = std::min(loY, pts[k].dy);
        hiY = std::max(hiY, pts[k].dy);
        sw += pts[k].w;
        swx += pts[k].w * pts[k].dx;
        swy += pts[k].w * pts[k].dy;
    }
    if (out.patches < std::max(1, cfg.base_creep_min_patches)) return out;

    // Whether the base has moved is decided where it was read, from the
    // movement smoothed over the patches round each one - never from the
    // lattice below, which also covers the garment, far from any base, where
    // the plane is extrapolated and a patch's noise is multiplied by the
    // distance. One patch reads its movement to about a tenth of a pixel in
    // sensor noise alone; smoothed over its neighbours, to a few hundredths.
    // It has moved if at least base_creep_min_patches places agree it moved
    // by base_creep_min_px or more.
    std::vector<double> moved;
    moved.reserve(static_cast<std::size_t>(out.patches));
    for (std::size_t k = 0; k < pts.size(); ++k) {
        if (!keep[k]) continue;
        double fx = 0.0, fy = 0.0;
        if (!fitAt(pts, keep, index, pts[k].x, pts[k].y, sigma, -1, false, fx, fy)) continue;
        moved.push_back(std::hypot(fx, fy));
    }
    if (moved.empty()) return out;
    out.max_px = *std::max_element(moved.begin(), moved.end());
    out.median_px = median(moved);
    const auto agreeing = std::count_if(moved.begin(), moved.end(),
                                        [&](double v) { return v >= cfg.base_creep_min_px; });
    if (agreeing < std::max(1, cfg.base_creep_min_patches)) return out;

    // The movement on a lattice over the crop, every `cell` pixels: it varies
    // over hundreds of pixels, so between the nodes it is followed exactly
    // enough, for a fraction of the work of every pixel. Clamped to what was
    // actually measured, so a plane extrapolated under a large garment cannot
    // run away.
    const int cell = 64;
    const int nx = (crop.width - 1) / cell + 2;
    const int ny = (crop.height - 1) / cell + 2;
    cv::Mat nodeX(ny, nx, CV_64F), nodeY(ny, nx, CV_64F);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const double qx = crop.x + static_cast<double>(i) * cell;
            const double qy = crop.y + static_cast<double>(j) * cell;
            double fx = 0.0, fy = 0.0;
            if (!fitAt(pts, keep, index, qx, qy, sigma, -1, false, fx, fy) &&
                !fitAt(pts, keep, index, qx, qy, 4.0 * sigma, -1, true, fx, fy)) {
                fx = swx / sw;
                fy = swy / sw;
            }
            fx = std::clamp(fx, loX, hiX);
            fy = std::clamp(fy, loY, hiY);
            nodeX.at<double>(j, i) = fx;
            nodeY.at<double>(j, i) = fy;
        }
    }

    // The background, sampled from where each pixel of base was when it was
    // taken. Linear: on the sleeve it put the outline on the cloth's edge as
    // closely as cubic did, and it rings less at a printed line.
    cv::Mat mapX(crop.height, crop.width, CV_32F), mapY(crop.height, crop.width, CV_32F);
    std::vector<int> ci(static_cast<std::size_t>(crop.width));
    std::vector<double> ca(static_cast<std::size_t>(crop.width));
    for (int x = 0; x < crop.width; ++x) {
        const double t = static_cast<double>(x) / cell;
        ci[static_cast<std::size_t>(x)] = std::min(nx - 2, static_cast<int>(t));
        ca[static_cast<std::size_t>(x)] = t - ci[static_cast<std::size_t>(x)];
    }
    for (int y = 0; y < crop.height; ++y) {
        const double t = static_cast<double>(y) / cell;
        const int j = std::min(ny - 2, static_cast<int>(t));
        const double b = t - j;
        const double* x0 = nodeX.ptr<double>(j);
        const double* x1 = nodeX.ptr<double>(j + 1);
        const double* y0 = nodeY.ptr<double>(j);
        const double* y1 = nodeY.ptr<double>(j + 1);
        float* mx = mapX.ptr<float>(y);
        float* my = mapY.ptr<float>(y);
        for (int x = 0; x < crop.width; ++x) {
            const int i = ci[static_cast<std::size_t>(x)];
            const double a = ca[static_cast<std::size_t>(x)];
            const double fx = (1.0 - b) * ((1.0 - a) * x0[i] + a * x0[i + 1]) +
                              b * ((1.0 - a) * x1[i] + a * x1[i + 1]);
            const double fy = (1.0 - b) * ((1.0 - a) * y0[i] + a * y0[i + 1]) +
                              b * ((1.0 - a) * y1[i] + a * y1[i + 1]);
            mx[x] = static_cast<float>(crop.x + x - fx);
            my[x] = static_cast<float>(crop.y + y - fy);
        }
    }
    cv::remap(backgroundBgr, alignedCrop, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    out.aligned = true;
    return out;
}

}  // namespace verimeasure
