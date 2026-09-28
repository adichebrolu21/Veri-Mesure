// Plane model: image pixels <-> millimetres on the measurement surface.
//
//     (X, Y, 1) --H--> ideal pixel (u, v) --radial distortion--> observed pixel
//
// Inverse, pixels to millimetres, undistorts first then applies H inverse. Ten
// parameters at most: eight for the homography, two radial terms. Fitted
// against the full detected lattice, so the parameter-to-observation ratio is
// of order 1:200 or better.

#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "verimeasure/json.hpp"

namespace verimeasure {

class PlaneModel {
public:
    PlaneModel(const cv::Matx33d& H, double k1 = 0.0, double k2 = 0.0,
               cv::Point2d centre = cv::Point2d(0.0, 0.0), double norm = 1.0,
               double scaleCorrection = 1.0, const vjson::Value& meta = vjson::Value::object());

    // -- transforms --
    std::vector<cv::Point2d> mmToPx(const std::vector<cv::Point2d>& ptsMm) const;
    std::vector<cv::Point2d> pxToMm(const std::vector<cv::Point2d>& ptsPx) const;

    // Local scale at a point, for legacy reporting and sanity checks.
    std::pair<double, double> localMmPerPx(cv::Point2d pointPx) const;

    // -- persistence --
    vjson::Value toJson() const;
    // Returns nullptr for anything that is not a version-2 homography record.
    static std::shared_ptr<PlaneModel> fromJson(const vjson::Value& d);

    // -- accessors --
    const cv::Matx33d& H() const { return H_; }
    double k1() const { return k1_; }
    double k2() const { return k2_; }
    cv::Point2d centre() const { return cv::Point2d(cx_, cy_); }
    double norm() const { return norm_; }

    double scaleCorrection() const { return scaleCorrection_; }
    void setScaleCorrection(double v) { scaleCorrection_ = v; }

    vjson::Value& meta() { return meta_; }
    const vjson::Value& meta() const { return meta_; }

    // Convenience readers for meta fields the pipeline uses by name.
    double metaNumber(const std::string& key, double fallback = 0.0) const;
    double detectionNumber(const std::string& key, double fallback = 0.0) const;
    bool latticeFootprint(int& x, int& y, int& w, int& h) const;

private:
    void distort(double u, double v, double& ud, double& vd) const;
    void undistort(std::vector<double>& u, std::vector<double>& v) const;

    cv::Matx33d H_;
    cv::Matx33d Hinv_;
    double k1_ = 0.0;
    double k2_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double norm_ = 1.0;
    double scaleCorrection_ = 1.0;
    vjson::Value meta_;
};

using PlaneModelPtr = std::shared_ptr<PlaneModel>;

}  // namespace verimeasure
