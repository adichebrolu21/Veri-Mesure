/*
Veri Measure - Phase 1: Camera connection + acquisition + display

Goal (per project spec):
    - Connect to the Lucid TRI200S via Arena SDK
    - Detect the camera
    - Start acquisition
    - Retrieve frames
    - Convert to OpenCV and display

This tool does NOT do ROI, detection, segmentation, or measurement. Those come
in later phases. This is intentionally the smallest possible working camera loop
so the hardware/software path can be confirmed before building anything on top
of it.

Confirmed hardware: TRI200S-CC (colour variant). BGR8 output is requested
directly from the camera's PixelFormat node, which handles Bayer debayering on
camera or in the driver, so no Bayer conversion happens here.

Assumptions:
    - Free-running continuous acquisition, not hardware trigger.
    - Single camera on the network (first device found). With more than one
      Lucid camera connected this needs to select by serial number instead.
*/

#include <cstdio>
#include <exception>

#include <opencv2/highgui.hpp>
#include "verimeasure/cv_compat.hpp"

#include "verimeasure/camera.hpp"
#include "verimeasure/config.hpp"

using namespace verimeasure;

int main() {
    Config cfg;
    Camera device;

    try {
        device.connect();
        device.configureStream();

        device.startStream(10);
        std::printf("Streaming started. Press 'q' in the image window to quit.\n");
        std::fflush(stdout);

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

            // Native resolution is 5472x3648 - too large for a comfortable
            // preview window, so we resize ONLY for display. Measurement in
            // later phases uses the full-resolution frame.
            cv::Mat display;
            cv::resize(frame, display, cv::Size(), 0.25, 0.25);
            cv::imshow("Veri Measure - Phase 1: Raw Camera Feed", display);

            if ((cv::waitKey(1) & 0xFF) == 'q') break;
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
