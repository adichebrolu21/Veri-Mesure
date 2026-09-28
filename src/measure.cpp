#include "verimeasure/measure.hpp"

#include <algorithm>
#include <cmath>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/numeric.hpp"

namespace verimeasure {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<cv::Point2f> toPoint2f(const std::vector<cv::Point2d>& pts) {
    std::vector<cv::Point2f> out(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) {
        out[k] = cv::Point2f(static_cast<float>(pts[k].x), static_cast<float>(pts[k].y));
    }
    return out;
}

std::vector<cv::Point2d> resampleContour(const std::vector<cv::Point2d>& pts, double step) {
    const std::size_t n = pts.size();
    std::vector<double> seg(n);
    double total = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const cv::Point2d& a = pts[k];
        const cv::Point2d& b = pts[(k + 1) % n];
        seg[k] = std::hypot(b.x - a.x, b.y - a.y);
        total += seg[k];
    }
    if (total < step * 8.0) return pts;

    std::vector<double> cum(n + 1, 0.0);
    for (std::size_t k = 0; k < n; ++k) cum[k + 1] = cum[k] + seg[k];

    std::vector<double> closedX(n + 1), closedY(n + 1);
    for (std::size_t k = 0; k < n; ++k) {
        closedX[k] = pts[k].x;
        closedY[k] = pts[k].y;
    }
    closedX[n] = pts[0].x;
    closedY[n] = pts[0].y;

    std::vector<double> targets;
    for (double t = 0.0; t < total; t += step) targets.push_back(t);

    std::vector<double> xs, ys;
    interp(targets, cum, closedX, xs);
    interp(targets, cum, closedY, ys);

    std::vector<cv::Point2d> out(targets.size());
    for (std::size_t k = 0; k < targets.size(); ++k) out[k] = cv::Point2d(xs[k], ys[k]);
    return out;
}

// Total least squares line through a point set. Returns (normal, offset).
void tlsLine(const std::vector<cv::Point2d>& points, cv::Point2d& normal, double& offset) {
    cv::Point2d c(0.0, 0.0);
    for (const cv::Point2d& p : points) {
        c.x += p.x;
        c.y += p.y;
    }
    c.x /= static_cast<double>(points.size());
    c.y /= static_cast<double>(points.size());

    cv::Mat A(static_cast<int>(points.size()), 2, CV_64F);
    for (std::size_t k = 0; k < points.size(); ++k) {
        A.at<double>(static_cast<int>(k), 0) = points[k].x - c.x;
        A.at<double>(static_cast<int>(k), 1) = points[k].y - c.y;
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A);
    const double dx = vt.at<double>(0, 0);
    const double dy = vt.at<double>(0, 1);
    normal = cv::Point2d(-dy, dx);
    offset = normal.x * c.x + normal.y * c.y;
}

}  // namespace

std::vector<cv::Point2d> refineContourSubpixel(const cv::Mat& edgeEvidence,
                                               const std::vector<cv::Point>& contour,
                                               const Config& cfg) {
    std::vector<cv::Point2d> pts;
    pts.reserve(contour.size());
    for (const cv::Point& p : contour) pts.emplace_back(p.x, p.y);
    if (pts.size() < 12) return pts;

    pts = resampleContour(pts, cfg.contour_resample_px);
    const int n = static_cast<int>(pts.size());
    if (n < 12) return pts;

    const int win = std::max(3, cfg.contour_smooth_window | 1);
    const int pad = win / 2;

    // Circular moving average, matching the padded 'valid' convolution.
    std::vector<cv::Point2d> smooth(n);
    for (int i = 0; i < n; ++i) {
        double sx = 0.0;
        double sy = 0.0;
        for (int k = 0; k < win; ++k) {
            const int idx = ((i - pad + k) % n + n) % n;
            sx += pts[idx].x;
            sy += pts[idx].y;
        }
        smooth[i] = cv::Point2d(sx / win, sy / win);
    }

    std::vector<cv::Point2d> normal(n);
    for (int i = 0; i < n; ++i) {
        const cv::Point2d& next = smooth[(i + 1) % n];
        const cv::Point2d& prev = smooth[(i - 1 + n) % n];
        double tx = next.x - prev.x;
        double ty = next.y - prev.y;
        double len = std::hypot(tx, ty);
        if (len < 1e-9) len = 1e-9;
        tx /= len;
        ty /= len;
        normal[i] = cv::Point2d(-ty, tx);
    }

    cv::Mat blurred, gx, gy, grad;
    cv::GaussianBlur(edgeEvidence, blurred, cv::Size(0, 0), cfg.subpixel_blur_sigma);
    if (cfg.subpixel_scharr) {
        // Scharr rather than Sobel. Same 3x3 footprint and the same cost, but
        // its weights are the ones chosen to make the operator rotationally
        // symmetric, where Sobel's were chosen to be small integers. A fold
        // lying at an angle across the frame is then located as well as one
        // lying square to it, instead of slightly worse.
        cv::Scharr(blurred, gx, CV_32F, 1, 0);
        cv::Scharr(blurred, gy, CV_32F, 0, 1);
    } else {
        cv::Sobel(blurred, gx, CV_32F, 1, 0, 3);
        cv::Sobel(blurred, gy, CV_32F, 0, 1, 3);
    }
    cv::magnitude(gx, gy, grad);

    const double R = cfg.subpixel_search_px;
    const double step = cfg.subpixel_step_px;
    std::vector<double> offsets;
    for (double t = -R; t < R + step * 0.5; t += step) offsets.push_back(t);
    const int nOff = static_cast<int>(offsets.size());

    std::vector<double> profile(static_cast<std::size_t>(n) * nOff, 0.0);
    std::vector<double> xs(n), ys(n), vals;
    std::vector<unsigned char> ok;

    for (int i = 0; i < nOff; ++i) {
        const double t = offsets[i];
        for (int k = 0; k < n; ++k) {
            xs[k] = pts[k].x + normal[k].x * t;
            ys[k] = pts[k].y + normal[k].y * t;
        }
        bilinearSample(grad, xs, ys, vals, ok);
        for (int k = 0; k < n; ++k) {
            profile[static_cast<std::size_t>(k) * nOff + i] = ok[k] ? vals[k] : 0.0;
        }
    }

    std::vector<int> peakIdx(n, 0);
    std::vector<double> peakVal(n, 0.0);
    for (int k = 0; k < n; ++k) {
        const double* row = &profile[static_cast<std::size_t>(k) * nOff];
        int best = 0;
        double bestVal = row[0];
        for (int i = 1; i < nOff; ++i) {
            if (row[i] > bestVal) {
                bestVal = row[i];
                best = i;
            }
        }
        peakIdx[k] = best;
        peakVal[k] = bestVal;
    }

    std::vector<double> positive;
    for (double v : peakVal) {
        if (v > 0.0) positive.push_back(v);
    }
    const double strengthFloor = positive.empty() ? 0.0 : 0.25 * median(positive);

    // The correction each point wants, worked out before any of them is applied.
    std::vector<double> shift(n, 0.0);
    std::vector<unsigned char> moved(n, 0);
    for (int k = 0; k < n; ++k) {
        if (!(peakIdx[k] > 0 && peakIdx[k] < nOff - 1 && peakVal[k] > strengthFloor)) continue;
        const double* row = &profile[static_cast<std::size_t>(k) * nOff];
        const double a = row[peakIdx[k] - 1];
        const double b = row[peakIdx[k]];
        const double c = row[peakIdx[k] + 1];
        const double denom = a - 2 * b + c;
        double frac = (std::abs(denom) > 1e-9) ? 0.5 * (a - c) / denom : 0.0;
        frac = std::min(1.0, std::max(-1.0, frac));
        const double s = offsets[peakIdx[k]] + frac * step;
        shift[k] = std::min(R, std::max(-R, s));
        moved[k] = 1;
    }

    // Points along the boundary were each refined in ignorance of their
    // neighbours, and a fabric edge is continuous. A correction that disagrees
    // with the run it sits in has not found a better reading of the edge, it
    // has found something else: a printed line under the cloth, a crease, a
    // speck. Left alone it puts a spike in the outline, which is then visible
    // on the feed and has to be averaged away by the side fits.
    //
    // Only the outliers are pulled back, to the median of their neighbours.
    // Median-filtering every shift would have been less code and would have
    // quietly rounded off the corners, and the corners are the points a span is
    // measured between.
    const int medianWindow = std::max(3, cfg.subpixel_shift_median_window | 1);
    if (cfg.subpixel_shift_outlier_px > 0.0 && n >= medianWindow) {
        const int half = medianWindow / 2;
        std::vector<double> corrected = shift;
        std::vector<double> neighbours;
        neighbours.reserve(static_cast<std::size_t>(medianWindow));
        for (int k = 0; k < n; ++k) {
            if (!moved[k]) continue;
            neighbours.clear();
            for (int d = -half; d <= half; ++d) {
                if (d == 0) continue;
                const int idx = ((k + d) % n + n) % n;
                if (moved[idx]) neighbours.push_back(shift[idx]);
            }
            if (neighbours.size() < 3) continue;
            const double agreed = median(neighbours);
            if (std::abs(shift[k] - agreed) > cfg.subpixel_shift_outlier_px) {
                corrected[k] = agreed;
            }
        }
        shift.swap(corrected);
    }

    std::vector<cv::Point2d> refined = pts;
    for (int k = 0; k < n; ++k) {
        if (!moved[k]) continue;
        refined[k] =
            cv::Point2d(pts[k].x + normal[k].x * shift[k], pts[k].y + normal[k].y * shift[k]);
    }

    return refined;
}

bool fitRectangleMm(const std::vector<cv::Point2d>& pointsMm, const Config& cfg, RectFit& out) {
    (void)cfg;
    const std::size_t n = pointsMm.size();
    if (n < 40) return false;

    const cv::RotatedRect rect = cv::minAreaRect(toPoint2f(pointsMm));
    const double cx = rect.center.x;
    const double cy = rect.center.y;
    const double w = rect.size.width;
    const double h = rect.size.height;
    const double angle = rect.angle;
    if (w <= 0.0 || h <= 0.0) return false;

    const double theta = angle * kPi / 180.0;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double halfW = w / 2.0;
    const double halfH = h / 2.0;
    const double nearLimit = 0.06 * std::min(w, h);

    std::vector<std::vector<cv::Point2d>> sideSets(4);
    for (std::size_t k = 0; k < n; ++k) {
        const double dx = pointsMm[k].x - cx;
        const double dy = pointsMm[k].y - cy;
        const double lx = ct * dx + st * dy;
        const double ly = -st * dx + ct * dy;

        // side order: -x, -y, +x, +y in the local frame
        const double d[4] = {std::abs(lx + halfW), std::abs(ly + halfH), std::abs(lx - halfW),
                             std::abs(ly - halfH)};
        int owner = 0;
        for (int i = 1; i < 4; ++i) {
            if (d[i] < d[owner]) owner = i;
        }
        if (!(d[owner] < nearLimit)) continue;

        // Drop the corners. Rounded or slightly damaged corners are common and
        // they bend a side fit far more than anything in the middle of the run.
        const double along = (owner % 2 == 0) ? (ly / halfH) : (lx / halfW);
        if (!(std::abs(along) < 0.86)) continue;

        sideSets[owner].push_back(pointsMm[k]);
    }

    std::vector<cv::Point2d> normals(4);
    std::vector<double> offsets(4);
    std::vector<double> residuals(4);

    for (int side = 0; side < 4; ++side) {
        std::vector<cv::Point2d> sub = sideSets[side];
        if (sub.size() < 12) return false;

        for (int iter = 0; iter < 3; ++iter) {
            cv::Point2d nrm;
            double off = 0.0;
            tlsLine(sub, nrm, off);

            std::vector<double> r(sub.size());
            for (std::size_t k = 0; k < sub.size(); ++k) {
                r[k] = std::abs(sub[k].x * nrm.x + sub[k].y * nrm.y - off);
            }
            const double mad = median(r) + 1e-9;
            const double limit = std::max(4.0 * mad, 0.05);

            std::vector<cv::Point2d> keep;
            for (std::size_t k = 0; k < sub.size(); ++k) {
                if (r[k] < limit) keep.push_back(sub[k]);
            }
            if (keep.size() < 12 || keep.size() == sub.size()) break;
            sub.swap(keep);
        }

        cv::Point2d nrm;
        double off = 0.0;
        tlsLine(sub, nrm, off);
        normals[side] = nrm;
        offsets[side] = off;

        double sq = 0.0;
        for (const cv::Point2d& p : sub) {
            const double e = p.x * nrm.x + p.y * nrm.y - off;
            sq += e * e;
        }
        residuals[side] = std::sqrt(sq / static_cast<double>(sub.size()));
    }

    std::vector<cv::Point2d> corners(4);
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d& n1 = normals[i];
        const cv::Point2d& n2 = normals[(i + 1) % 4];
        const double det = n1.x * n2.y - n1.y * n2.x;
        if (std::abs(det) < 1e-9) return false;
        const double o1 = offsets[i];
        const double o2 = offsets[(i + 1) % 4];
        corners[i] = cv::Point2d((o1 * n2.y - n1.y * o2) / det, (n1.x * o2 - o1 * n2.x) / det);
    }

    double sides[4];
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d& a = corners[i];
        const cv::Point2d& b = corners[(i + 1) % 4];
        sides[i] = std::hypot(b.x - a.x, b.y - a.y);
    }
    const double pairA = (sides[0] + sides[2]) / 2.0;
    const double pairB = (sides[1] + sides[3]) / 2.0;

    // Sanity: opposite sides should agree and corners should be square. If not,
    // this outline is not a rectangle and the fit means nothing.
    if (std::min(pairA, pairB) <= 0.0) return false;
    if (std::abs(sides[0] - sides[2]) > 0.05 * pairA ||
        std::abs(sides[1] - sides[3]) > 0.05 * pairB) {
        return false;
    }

    double squareness = 0.0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d& a = normals[i];
        const cv::Point2d& b = normals[(i + 1) % 4];
        double dot = a.x * b.x + a.y * b.y;
        dot = std::min(1.0, std::max(-1.0, dot));
        const double deg = std::acos(dot) * 180.0 / kPi;
        squareness = std::max(squareness, std::abs(90.0 - std::abs(deg)));
    }
    if (squareness > 8.0) return false;

    const double fitRms = (residuals[0] + residuals[1] + residuals[2] + residuals[3]) / 4.0;
    if (fitRms > 0.04 * std::min(pairA, pairB)) return false;

    out.length_mm = std::max(pairA, pairB);
    out.breadth_mm = std::min(pairA, pairB);
    out.corners_mm = corners;
    out.side_fit_rms_mm = fitRms;
    out.opposite_side_delta_mm[0] = std::abs(sides[0] - sides[2]);
    out.opposite_side_delta_mm[1] = std::abs(sides[1] - sides[3]);
    out.corner_squareness_deg = squareness;
    return true;
}

MeasureResult measureInMm(const std::vector<cv::Point2d>& contourPxFull, const PlaneModel& model,
                          const Config& cfg) {
    const std::vector<cv::Point2d> ptsMm = model.pxToMm(contourPxFull);
    const std::vector<cv::Point2f> ptsMmF = toPoint2f(ptsMm);
    const cv::RotatedRect rect = cv::minAreaRect(ptsMmF);
    const double cx = rect.center.x;
    const double cy = rect.center.y;
    const double w = rect.size.width;
    const double h = rect.size.height;
    const double angle = rect.angle;

    RectFit edgeFit;
    const bool haveEdgeFit = fitRectangleMm(ptsMm, cfg, edgeFit);

    double length = 0.0;
    double breadth = 0.0;
    std::string method;
    if (haveEdgeFit) {
        length = edgeFit.length_mm;
        breadth = edgeFit.breadth_mm;
        method = "edge_fit";
    } else {
        length = std::max(w, h);
        breadth = std::min(w, h);
        method = "min_area_rect";
    }

    double correction = 1.0;
    const double height = cfg.object_height_mm;
    if (height != 0.0 && cfg.camera_distance_mm.has_value() && *cfg.camera_distance_mm != 0.0 &&
        *cfg.camera_distance_mm > height) {
        // The top face sits closer to the lens than the calibrated plane, so it
        // images larger. Scale it back onto the plane.
        const double distance = *cfg.camera_distance_mm;
        correction = (distance - height) / distance;
        length *= correction;
        breadth *= correction;
    }

    const double area = cv::contourArea(ptsMmF) * correction * correction;
    const double perimeter = cv::arcLength(ptsMmF, true) * correction;

    MeasureResult out;
    out.length_mm = length;
    out.breadth_mm = breadth;
    out.area_mm2 = area;
    out.perimeter_mm = perimeter;
    out.centre_mm = cv::Point2d(cx, cy);
    out.angle_deg = angle;
    out.parallax_correction = correction;
    out.fit_method = method;

    cv::Point2f boxPts[4];
    rect.points(boxPts);
    out.box_mm.resize(4);
    for (int k = 0; k < 4; ++k) out.box_mm[k] = cv::Point2d(boxPts[k].x, boxPts[k].y);

    if (haveEdgeFit) {
        out.has_edge_fit = true;
        out.side_fit_rms_mm = edgeFit.side_fit_rms_mm;
        out.opposite_side_delta_mm[0] = edgeFit.opposite_side_delta_mm[0];
        out.opposite_side_delta_mm[1] = edgeFit.opposite_side_delta_mm[1];
        out.corner_squareness_deg = edgeFit.corner_squareness_deg;
        out.corners_mm = edgeFit.corners_mm;
    }

    return out;
}

vjson::Value MeasureResult::toJson() const {
    vjson::Value out = vjson::Value::object();

    auto pointArray = [](const std::vector<cv::Point2d>& pts) {
        vjson::Value arr = vjson::Value::array();
        for (const cv::Point2d& p : pts) {
            vjson::Value pair = vjson::Value::array();
            pair.push(vjson::Value::real(p.x));
            pair.push(vjson::Value::real(p.y));
            arr.push(pair);
        }
        return arr;
    };

    if (calibrated) {
        out.set("length_mm", vjson::Value::real(length_mm));
        out.set("breadth_mm", vjson::Value::real(breadth_mm));
        out.set("area_mm2", vjson::Value::real(area_mm2));
        out.set("perimeter_mm", vjson::Value::real(perimeter_mm));
        vjson::Value centre = vjson::Value::array();
        centre.push(vjson::Value::real(centre_mm.x));
        centre.push(vjson::Value::real(centre_mm.y));
        out.set("centre_mm", centre);
        out.set("angle_deg", vjson::Value::real(angle_deg));
        out.set("parallax_correction", vjson::Value::real(parallax_correction));
        out.set("fit_method", vjson::Value::str(fit_method));
        out.set("box_mm", pointArray(box_mm));
        if (has_edge_fit) {
            out.set("side_fit_rms_mm", vjson::Value::real(side_fit_rms_mm));
            vjson::Value delta = vjson::Value::array();
            delta.push(vjson::Value::real(opposite_side_delta_mm[0]));
            delta.push(vjson::Value::real(opposite_side_delta_mm[1]));
            out.set("opposite_side_delta_mm", delta);
            out.set("corner_squareness_deg", vjson::Value::real(corner_squareness_deg));
            out.set("corners_mm", pointArray(corners_mm));
        }
        out.set("calibrated", vjson::Value::boolean(true));
        if (has_saturation) {
            out.set("saturated_fraction", vjson::Value::real(saturated_fraction));
        }
    } else {
        out.set("length_px", vjson::Value::real(length_px));
        out.set("breadth_px", vjson::Value::real(breadth_px));
        out.set("angle_deg", vjson::Value::real(angle_deg));
        out.set("calibrated", vjson::Value::boolean(false));
    }

    if (finalised) {
        out.set("object_number", vjson::Value::integer(object_number));
        out.set("frames_averaged", vjson::Value::integer(frames_averaged));
        if (has_settled) {
            vjson::Value s = vjson::Value::object();
            s.set("area_spread_fraction", vjson::Value::real(settled.area_spread_fraction));
            s.set("centroid_movement_px", vjson::Value::real(settled.centroid_movement_px));
            out.set("settled", s);
        }
        out.set("timestamp", vjson::Value::str(timestamp));
    }

    return out;
}

}  // namespace verimeasure
