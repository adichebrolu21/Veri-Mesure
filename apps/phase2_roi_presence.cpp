/*
Veri Measure - Phase 2: ROI + object presence detection

Builds directly on Phase 1 (camera connection/acquisition). This phase does NOT
touch segmentation, contours, or measurement - just:

    1. Draw a Region of Interest (ROI) on the live feed.
    2. Capture a reference "empty" background frame within that ROI.
    3. For each new frame, compare against the background and decide:
       "Is an object currently inside the measurement area?"

Controls (while the video window is focused):
    'b'  -> (re)capture the background reference frame
             (do this once the ROI is empty/clear of any object)
    'q'  -> quit

ASSUMPTIONS:
    - ROI defaults to the centre 60% of the frame. Adjust ROI_FRACTION below
      once you can see where the table edges actually fall in the image.
    - Presence detection uses simple frame differencing against a captured
      background plus a pixel-count threshold. This assumes relatively stable,
      controlled lighting. If lighting is inconsistent, this threshold will need
      tuning, or a more robust method will be needed.
*/

#include <cstdio>
#include <exception>

#include <opencv2/highgui.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/camera.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/phase_common.hpp"

using namespace verimeasure;

namespace {

// ---- Tunable parameters (adjust these once you see the live feed) ----

// Fraction of the full frame the ROI covers, centred in the image.
constexpr double ROI_FRACTION = 0.6;

// How much a pixel's intensity must change (0-255) to count as "different"
// from the background. Lower = more sensitive to small changes, but more prone
// to false positives from lighting flicker.
constexpr double DIFF_THRESHOLD = 25.0;

// What fraction of ROI pixels must be "different" from the background before we
// declare "object present". Too low triggers on noise and shadows, too high
// misses small or low-contrast objects.
constexpr double PRESENCE_PIXEL_FRACTION = 0.02;  // i.e. 2% of ROI pixels changed

// Compare the current ROI against the background reference. Returns the raw
// changed fraction too, for tuning purposes.
bool isObjectPresent(const cv::Mat& currentRoiGray, const cv::Mat& backgroundRoiGray,
                     double& changedFraction) {
    if (backgroundRoiGray.empty()) {
        changedFraction = 0.0;
        return false;
    }
    cv::Mat diff, thresholded;
    cv::absdiff(currentRoiGray, backgroundRoiGray, diff);
    cv::threshold(diff, thresholded, DIFF_THRESHOLD, 255, cv::THRESH_BINARY);

    const double changed = static_cast<double>(cv::countNonZero(thresholded));
    changedFraction = changed / static_cast<double>(thresholded.total());
    return changedFraction >= PRESENCE_PIXEL_FRACTION;
}

}  // namespace

int main() {
    Config cfg;
    Camera device;
    cv::Mat backgroundRoiGray;

    try {
        device.connect();
        device.configureStream();

        device.startStream(10);
        std::printf("Streaming started.\n");
        std::printf("Controls: 'b' = capture background (ROI must be empty), 'q' = quit\n");
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
            double changedFraction = 0.0;
            const bool present = isObjectPresent(currentRoiGray, backgroundRoiGray, changedFraction);

            // ---- Build the display frame with overlays ----
            const double scale = 0.25;
            cv::Mat display;
            cv::resize(frame, display, cv::Size(), scale, scale);

            const cv::Scalar boxColour = present ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
            cv::rectangle(display, cv::Point(static_cast<int>(roiBox.x1 * scale),
                                             static_cast<int>(roiBox.y1 * scale)),
                          cv::Point(static_cast<int>(roiBox.x2 * scale),
                                    static_cast<int>(roiBox.y2 * scale)),
                          boxColour, 2);

            char status[128];
            if (backgroundRoiGray.empty()) {
                std::snprintf(status, sizeof(status),
                              "No background captured - press 'b' with ROI empty");
            } else if (present) {
                std::snprintf(status, sizeof(status), "OBJECT PRESENT (%.1f%% changed)",
                              changedFraction * 100.0);
            } else {
                std::snprintf(status, sizeof(status), "Empty (%.1f%% changed)",
                              changedFraction * 100.0);
            }
            cv::putText(display, status, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        boxColour, 2);

            cv::imshow("Veri Measure - Phase 2: ROI + Presence Detection", display);

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 'q') break;
            if (key == 'b') {
                backgroundRoiGray = currentRoiGray.clone();
                std::printf("Background reference captured.\n");
                std::fflush(stdout);
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
