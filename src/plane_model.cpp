#include "verimeasure/plane_model.hpp"

#include <cmath>

namespace verimeasure {

PlaneModel::PlaneModel(const cv::Matx33d& H, double k1, double k2, cv::Point2d centre,
                       double norm, double scaleCorrection, const vjson::Value& meta)
    : k1_(k1),
      k2_(k2),
      cx_(centre.x),
      cy_(centre.y),
      norm_(norm),
      scaleCorrection_(scaleCorrection),
      meta_(meta) {
    const double w = H(2, 2);
    H_ = H * (1.0 / w);
    Hinv_ = H_.inv();
    if (meta_.type != vjson::Value::Type::Object) meta_ = vjson::Value::object();
}

void PlaneModel::distort(double u, double v, double& ud, double& vd) const {
    const double dx = (u - cx_) / norm_;
    const double dy = (v - cy_) / norm_;
    const double r2 = dx * dx + dy * dy;
    const double f = 1.0 + k1_ * r2 + k2_ * r2 * r2;
    ud = cx_ + dx * norm_ * f;
    vd = cy_ + dy * norm_ * f;
}

void PlaneModel::undistort(std::vector<double>& u, std::vector<double>& v) const {
    if (k1_ == 0.0 && k2_ == 0.0) return;

    const std::size_t n = u.size();
    std::vector<double> dx(n), dy(n), x(n), y(n);
    for (std::size_t k = 0; k < n; ++k) {
        dx[k] = (u[k] - cx_) / norm_;
        dy[k] = (v[k] - cy_) / norm_;
        x[k] = dx[k];
        y[k] = dy[k];
    }

    for (int iter = 0; iter < 20; ++iter) {
        double maxDx = 0.0;
        double maxDy = 0.0;
        for (std::size_t k = 0; k < n; ++k) {
            const double r2 = x[k] * x[k] + y[k] * y[k];
            double f = 1.0 + k1_ * r2 + k2_ * r2 * r2;
            if (std::abs(f) < 1e-6) f = 1e-6;
            const double xn = dx[k] / f;
            const double yn = dy[k] / f;
            maxDx = std::max(maxDx, std::abs(xn - x[k]));
            maxDy = std::max(maxDy, std::abs(yn - y[k]));
            x[k] = xn;
            y[k] = yn;
        }
        if (maxDx < 1e-12 && maxDy < 1e-12) break;
    }

    for (std::size_t k = 0; k < n; ++k) {
        u[k] = cx_ + x[k] * norm_;
        v[k] = cy_ + y[k] * norm_;
    }
}

std::vector<cv::Point2d> PlaneModel::mmToPx(const std::vector<cv::Point2d>& ptsMm) const {
    std::vector<cv::Point2d> out(ptsMm.size());
    for (std::size_t k = 0; k < ptsMm.size(); ++k) {
        const double X = ptsMm[k].x;
        const double Y = ptsMm[k].y;
        const double px = H_(0, 0) * X + H_(0, 1) * Y + H_(0, 2);
        const double py = H_(1, 0) * X + H_(1, 1) * Y + H_(1, 2);
        const double pw = H_(2, 0) * X + H_(2, 1) * Y + H_(2, 2);
        double ud = 0.0;
        double vd = 0.0;
        distort(px / pw, py / pw, ud, vd);
        out[k] = cv::Point2d(ud, vd);
    }
    return out;
}

std::vector<cv::Point2d> PlaneModel::pxToMm(const std::vector<cv::Point2d>& ptsPx) const {
    const std::size_t n = ptsPx.size();
    std::vector<double> u(n), v(n);
    for (std::size_t k = 0; k < n; ++k) {
        u[k] = ptsPx[k].x;
        v[k] = ptsPx[k].y;
    }
    undistort(u, v);

    std::vector<cv::Point2d> out(n);
    for (std::size_t k = 0; k < n; ++k) {
        const double px = Hinv_(0, 0) * u[k] + Hinv_(0, 1) * v[k] + Hinv_(0, 2);
        const double py = Hinv_(1, 0) * u[k] + Hinv_(1, 1) * v[k] + Hinv_(1, 2);
        const double pw = Hinv_(2, 0) * u[k] + Hinv_(2, 1) * v[k] + Hinv_(2, 2);
        out[k] = cv::Point2d(px / pw * scaleCorrection_, py / pw * scaleCorrection_);
    }
    return out;
}

std::pair<double, double> PlaneModel::localMmPerPx(cv::Point2d pointPx) const {
    const std::vector<cv::Point2d> probe{
        pointPx,
        cv::Point2d(pointPx.x + 1.0, pointPx.y),
        cv::Point2d(pointPx.x, pointPx.y + 1.0),
    };
    const std::vector<cv::Point2d> mm = pxToMm(probe);
    const double sx = std::hypot(mm[1].x - mm[0].x, mm[1].y - mm[0].y);
    const double sy = std::hypot(mm[2].x - mm[0].x, mm[2].y - mm[0].y);
    return {sx, sy};
}

double PlaneModel::metaNumber(const std::string& key, double fallback) const {
    return meta_.getNumber(key, fallback);
}

double PlaneModel::detectionNumber(const std::string& key, double fallback) const {
    const vjson::Value* det = meta_.find("detection");
    if (!det || !det->isObject()) return fallback;
    return det->getNumber(key, fallback);
}

bool PlaneModel::latticeFootprint(int& x, int& y, int& w, int& h) const {
    const vjson::Value* fp = meta_.find("lattice_footprint");
    if (!fp || !fp->isArray() || fp->size() < 4) return false;
    x = static_cast<int>(fp->at(0).asNumber());
    y = static_cast<int>(fp->at(1).asNumber());
    w = static_cast<int>(fp->at(2).asNumber());
    h = static_cast<int>(fp->at(3).asNumber());
    return true;
}

vjson::Value PlaneModel::toJson() const {
    double mmx = 0.0;
    double mmy = 0.0;
    if (const vjson::Value* legacy = meta_.find("legacy_mm_per_px")) {
        if (legacy->isArray() && legacy->size() >= 2) {
            mmx = legacy->at(0).asNumber();
            mmy = legacy->at(1).asNumber();
        }
    }

    vjson::Value out = vjson::Value::object();
    out.set("method", vjson::Value::str("auto_grid_homography"));
    out.set("version", vjson::Value::integer(2));

    vjson::Value rows = vjson::Value::array();
    for (int r = 0; r < 3; ++r) {
        vjson::Value row = vjson::Value::array();
        for (int c = 0; c < 3; ++c) row.push(vjson::Value::real(H_(r, c)));
        rows.push(row);
    }
    out.set("H_mm_to_px", rows);
    out.set("k1", vjson::Value::real(k1_));
    out.set("k2", vjson::Value::real(k2_));

    vjson::Value centre = vjson::Value::array();
    centre.push(vjson::Value::real(cx_));
    centre.push(vjson::Value::real(cy_));
    out.set("distortion_centre", centre);
    out.set("distortion_norm", vjson::Value::real(norm_));
    out.set("scale_correction", vjson::Value::real(scaleCorrection_));
    out.set("meta", meta_);

    // Legacy fields so anything still reading the old schema keeps working.
    // These are the LOCAL scale at the calibrated area's centre and are
    // reporting values only. Nothing in this project measures with them.
    out.set("mm_per_pixel_x", vjson::Value::real(mmx));
    out.set("mm_per_pixel_y", vjson::Value::real(mmy));
    out.set("mm_per_pixel",
            vjson::Value::real((mmx != 0.0 || mmy != 0.0) ? (mmx + mmy) / 2.0 : 0.0));
    return out;
}

PlaneModelPtr PlaneModel::fromJson(const vjson::Value& d) {
    if (!d.isObject()) return nullptr;
    if (d.getInt("version", -1) != 2) return nullptr;
    const vjson::Value* hv = d.find("H_mm_to_px");
    if (!hv || !hv->isArray() || hv->size() != 3) return nullptr;

    cv::Matx33d H;
    for (int r = 0; r < 3; ++r) {
        const vjson::Value& row = hv->at(static_cast<std::size_t>(r));
        if (!row.isArray() || row.size() != 3) return nullptr;
        for (int c = 0; c < 3; ++c) H(r, c) = row.at(static_cast<std::size_t>(c)).asNumber();
    }

    cv::Point2d centre(0.0, 0.0);
    if (const vjson::Value* cv_ = d.find("distortion_centre")) {
        if (cv_->isArray() && cv_->size() >= 2) {
            centre.x = cv_->at(0).asNumber();
            centre.y = cv_->at(1).asNumber();
        }
    }

    vjson::Value meta = vjson::Value::object();
    if (const vjson::Value* m = d.find("meta")) {
        if (m->isObject()) meta = *m;
    }

    return std::make_shared<PlaneModel>(H, d.getNumber("k1", 0.0), d.getNumber("k2", 0.0),
                                        centre, d.getNumber("distortion_norm", 1.0),
                                        d.getNumber("scale_correction", 1.0), meta);
}

}  // namespace verimeasure
