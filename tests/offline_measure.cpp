// Offline measurement harness.
//
// Feeds saved stills through the same MeasurementEngine the live runner uses,
// so a rig can be validated, a fault reproduced, or this port checked against
// the Python original, all without a camera attached.
//
//     offline_measure <background.png> <part.png> [calibration.json]
//
// Prints the completed measurement as one JSON object on stdout.

#include <cstdio>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "verimeasure/calibration.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/engine.hpp"

using namespace verimeasure;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: offline_measure <background.png> <part.png> [calibration.json]\n");
        return 2;
    }

    Config cfg;
    if (argc >= 4) cfg.calibration_file = argv[3];

    const cv::Mat background = cv::imread(argv[1]);
    const cv::Mat part = cv::imread(argv[2]);
    if (background.empty() || part.empty()) {
        std::printf("could not read the input images\n");
        return 1;
    }

    PlaneModelPtr model = loadCalibration(cfg);
    if (!model) {
        model = autoCalibrate(background, cfg);
        if (model) saveCalibration(*model, cfg);
    }

    RoiBox roi{0, 0, background.cols, background.rows};
    bool haveRoi = true;
    if (model) {
        RoiBox derived;
        if (roiFromModel(*model, background.size(), cfg, derived, &background)) roi = derived;
    }

    MeasurementEngine engine(cfg, model, roi, haveRoi);
    engine.setBackground(background);

    for (long long seq = 1; seq <= 40; ++seq) {
        const auto result = engine.process(part, seq);
        if (result.has_value()) {
            std::printf("%s\n", result->toJson().dump().c_str());
            return 0;
        }
    }

    std::printf("no measurement completed after 40 frames (state %s)\n", engine.state().c_str());
    return 1;
}
