/*
Veri Measure - Phase 3+4 (merged): Segmentation + Pixel Measurement

Builds directly on Phase 1 (camera) and Phase 2 (ROI + background diff). This
merged tool covers:

    PHASE 3 - Segmentation:
        background diff -> threshold -> morphological cleanup ->
        contour detection -> object boundary

    PHASE 4 - Pixel measurement:
        From the detected contour, compute BOTH:
            - boundingRect()  (axis-aligned width/height)
            - minAreaRect()   (rotated width/height + angle)
        Both are shown on screen so they can be compared against the actual
        object, which decides which one to standardise on going into
        calibration. This is still PIXEL measurement only - no mm conversion.

Controls (while the video window is focused):
    'b'  -> (re)capture the background reference frame (ROI must be empty)
    'm'  -> toggle showing the cleaned mask in a second window
    'q'  -> quit

ASSUMPTIONS:
    - Single object per ROI at a time (largest qualifying contour only).
    - MIN_CONTOUR_AREA_PX is a placeholder (5000 px^2, full-res units).
    - MORPH_KERNEL_SIZE is a starting guess for mask cleanup.
    - Both bounding measurements are in FULL-RESOLUTION PIXELS.
*/

#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/camera.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/phase_common.hpp"

using namespace verimeasure;

namespace {

// ---- Tunable parameters ----

constexpr double ROI_FRACTION = 0.6;
constexpr double DIFF_THRESHOLD = 25.0;

// Minimum contour area (full-res pixels^2) to count as a real object.
constexpr double MIN_CONTOUR_AREA_PX = 5000.0;

constexpr int MORPH_KERNEL_SIZE = 5;

// Morphological open (remove noise) then close (fill small holes).
cv::Mat cleanMask(const cv::Mat& rawMask) {
    const cv::Mat kernel = cv::Mat::ones(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE, CV_8U);
    cv::Mat opened, closed;
    cv::morphologyEx(rawMask, opened, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, kernel);
    return closed;
}

// Largest contour above the area threshold.
bool findLargestContour(const cv::Mat& mask, std::vector<cv::Point>& contour, double& area) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;

    std::size_t best = 0;
    double bestArea = cv::contourArea(contours[0]);
    for (std::size_t k = 1; k < contours.size(); ++k) {
        const double a = cv::contourArea(contours[k]);
        if (a > bestArea) {
            bestArea = a;
            best = k;
        }
    }
    if (bestArea < MIN_CONTOUR_AREA_PX) return false;

    contour = contours[best];
    area = bestArea;
    return true;
}

struct Measurements {
    cv::Rect axis_aligned;
    cv::RotatedRect rotated_rect;
    cv::Point2f rotated_box_points[4];
};

// PHASE 4: compute pixel dimensions two ways from the same contour.
Measurements measureContour(const std::vector<cv::Point>& contour) {
    Measurements m;
    m.axis_aligned = cv::boundingRect(contour);
    m.rotated_rect = cv::minAreaRect(contour);
    m.rotated_rect.points(m.rotated_box_points);
    return m;
}

}  // namespace

int main() {
    Config cfg;
    Camera device;
    cv::Mat backgroundRoiGray;
    bool showMaskWindow = false;
    bool maskWindowOpen = false;

    try {
        device.connect();
        device.configureStream();

        device.startStream(10);
        std::printf("Streaming started.\n");
        std::printf("Controls: 'b' = capture background, 'm' = toggle mask view, 'q' = quit\n");
        std::fflush(stdout);

        RoiBox roiBox;
        bool haveRoi = false;

        while (true) {
            cv::Mat frame;
            bool incomplete = false;
            std::string error;
            if (!device.grab(frame, cfg.grab_timeout_ms, incomplete, error, true)) {
                if (!error.empty()) {
                    std::printf("Camera error: %s\n", error.c_str());
                    std::fflush(stdout);
                    break;
                }
                continue;
            }

            if (!haveRoi) {
                roiBox = computeRoiBox(frame.cols, frame.rows, ROI_FRACTION);
                haveRoi = true;
                std::printf("ROI set to: (%d, %d, %d, %d) (frame size: %dx%d)\n", roiBox.x1,
                            roiBox.y1, roiBox.x2, roiBox.y2, frame.cols, frame.rows);
                std::fflush(stdout);
            }

            const cv::Mat currentRoiGray = extractRoiGray(frame, roiBox);

            std::vector<cv::Point> contour;
            double contourArea = 0.0;
            cv::Mat cleanedMask;
            Measurements measurements;
            bool haveMeasurements = false;

            if (!backgroundRoiGray.empty()) {
                cv::Mat diff, rawMask;
                cv::absdiff(currentRoiGray, backgroundRoiGray, diff);
                cv::threshold(diff, rawMask, DIFF_THRESHOLD, 255, cv::THRESH_BINARY);
                cleanedMask = cleanMask(rawMask);

                if (findLargestContour(cleanedMask, contour, contourArea)) {
                    measurements = measureContour(contour);
                    haveMeasurements = true;
                }
            }

            // ---- Display frame ----
            const double scale = 0.25;
            cv::Mat display;
            cv::resize(frame, display, cv::Size(), scale, scale);
            const cv::Point roiOffset(roiBox.x1, roiBox.y1);

            cv::Scalar boxColour;
            std::vector<std::string> statusLines;

            if (haveMeasurements) {
                boxColour = cv::Scalar(0, 255, 0);

                // Contour outline (offset from ROI-local to full-frame, then
                // scaled for display)
                std::vector<cv::Point> displayContour(contour.size());
                for (std::size_t k = 0; k < contour.size(); ++k) {
                    displayContour[k] =
                        cv::Point(static_cast<int>((contour[k].x + roiOffset.x) * scale),
                                  static_cast<int>((contour[k].y + roiOffset.y) * scale));
                }
                std::vector<std::vector<cv::Point>> polys{displayContour};
                cv::drawContours(display, polys, -1, cv::Scalar(0, 255, 0), 2);

                // Axis-aligned bounding box (yellow)
                const cv::Rect& b = measurements.axis_aligned;
                cv::rectangle(display,
                              cv::Point(static_cast<int>((b.x + roiOffset.x) * scale),
                                        static_cast<int>((b.y + roiOffset.y) * scale)),
                              cv::Point(static_cast<int>((b.x + b.width + roiOffset.x) * scale),
                                        static_cast<int>((b.y + b.height + roiOffset.y) * scale)),
                              cv::Scalar(0, 255, 255), 1);

                // Rotated bounding box (magenta)
                std::vector<cv::Point> rotPoints(4);
                for (int k = 0; k < 4; ++k) {
                    rotPoints[k] = cv::Point(
                        static_cast<int>((measurements.rotated_box_points[k].x + roiOffset.x) *
                                         scale),
                        static_cast<int>((measurements.rotated_box_points[k].y + roiOffset.y) *
                                         scale));
                }
                std::vector<std::vector<cv::Point>> rotPolys{rotPoints};
                cv::drawContours(display, rotPolys, -1, cv::Scalar(255, 0, 255), 1);

                char line[160];
                std::snprintf(line, sizeof(line), "Area: %.0f px^2", contourArea);
                statusLines.emplace_back(line);
                std::snprintf(line, sizeof(line), "Axis-aligned (yellow): %d x %d px", b.width,
                              b.height);
                statusLines.emplace_back(line);
                std::snprintf(line, sizeof(line),
                              "Rotated (magenta): %.0f x %.0f px, angle=%.1f deg",
                              measurements.rotated_rect.size.width,
                              measurements.rotated_rect.size.height,
                              measurements.rotated_rect.angle);
                statusLines.emplace_back(line);
            } else {
                boxColour = cv::Scalar(0, 0, 255);
                statusLines.emplace_back(backgroundRoiGray.empty() ? "No background captured"
                                                                   : "No object detected");
            }

            cv::rectangle(display, cv::Point(static_cast<int>(roiBox.x1 * scale),
                                             static_cast<int>(roiBox.y1 * scale)),
                          cv::Point(static_cast<int>(roiBox.x2 * scale),
                                    static_cast<int>(roiBox.y2 * scale)),
                          boxColour, 1);
            for (std::size_t i = 0; i < statusLines.size(); ++i) {
                cv::putText(display, statusLines[i],
                            cv::Point(10, 25 + static_cast<int>(i) * 22), cv::FONT_HERSHEY_SIMPLEX,
                            0.55, boxColour, 2);
            }

            cv::imshow("Veri Measure - Segmentation + Measurement", display);

            if (showMaskWindow && !cleanedMask.empty()) {
                cv::Mat maskDisplay;
                cv::resize(cleanedMask, maskDisplay, cv::Size(), 0.5, 0.5);
                cv::imshow("Cleaned Mask (ROI only)", maskDisplay);
                maskWindowOpen = true;
            } else if (!showMaskWindow && maskWindowOpen) {
                cv::destroyWindow("Cleaned Mask (ROI only)");
                maskWindowOpen = false;
            }

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') break;
            if (key == 'b') {
                backgroundRoiGray = currentRoiGray.clone();
                std::printf("Background reference captured.\n");
                std::fflush(stdout);
            } else if (key == 'm') {
                showMaskWindow = !showMaskWindow;
            }
        }
    } catch (const std::exception& e) {
        std::printf("Camera error: %s\n", e.what());
        std::fflush(stdout);
    }

    device.destroy();
    cv::destroyAllWindows();
    std::printf("Cleaned up and disconnected.\n");
    return 0;
}
