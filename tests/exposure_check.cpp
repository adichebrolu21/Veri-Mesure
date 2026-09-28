// exposure_check: the startup exposure search, against a simulated camera.
//
//   exposure_check --selftest
//
// The station used to set the exposure on every start with a search that
// judged each step on the very next frame. A camera at two frames a second
// delivers a frame or two taken before any change it is asked for, so the
// search corrected changes that had not shown yet and stopped wherever it was
// when its steps ran out - which is how one start came up ten times brighter
// than the next under the same lamp. This renders a mat whose brightness
// follows the exposure through a camera that is 0 to 4 frames behind, and
// checks what meterExposure has to guarantee:
//
//   * it lands on the target from anywhere, however far behind the camera is,
//     and lands on the same exposure from any start, so restarts agree;
//   * a picture clipped white comes down however far over it starts;
//   * a light too dim for the range stops at the end of the range and says so;
//   * a camera that never takes the setting does not send it running off;
//   * a flickering light still lands close.
//
// The remembered exposure in bench.json is checked too: it keeps the lens, a
// figure given by hand is marked as such, and 0 forgets it.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "verimeasure/calibration.hpp"
#include "verimeasure/camera.hpp"
#include "verimeasure/config.hpp"
#include "verimeasure/cv_compat.hpp"
#include "verimeasure/frame_source.hpp"
#include "verimeasure/json.hpp"
#include "verimeasure/numeric.hpp"

using namespace verimeasure;

namespace {

int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string& what, const std::string& detail = std::string()) {
    ++g_checks;
    if (!ok) ++g_failures;
    std::printf("  [%s] %s%s%s\n", ok ? " ok " : "FAIL", what.c_str(), detail.empty() ? "" : ": ",
                detail.c_str());
}

std::string ratio(double v) { return vjson::formatFixed(v, 2) + "x"; }

// -- the simulated camera --
//
// A dark mat with a sparse scatter of bright print, at a linear response: at
// the right exposure its highlights (99.5th percentile of the brightest
// channel) sit on the search's target. Exposure is in multiples of the right
// one. A frame shows the exposure that was set `latency` frames before it was
// delivered.
class SimCamera : public FrameSource {
public:
    SimCamera(double exposure, int latency, double flicker, double target, unsigned seed)
        : latency_(latency), flicker_(flicker), rng_(seed), current_(exposure) {
        cv::RNG r(seed);
        base_ = cv::Mat(160, 240, CV_32FC3);
        // The mat, a dark teal, its brightest channel about a tenth of the
        // highlights.
        for (int y = 0; y < base_.rows; ++y) {
            for (int x = 0; x < base_.cols; ++x) {
                const float n = static_cast<float>(r.uniform(0.9, 1.1));
                base_.at<cv::Vec3f>(y, x) = cv::Vec3f(0.10f * n, 0.11f * n, 0.05f * n);
            }
        }
        // Print on it, 1.2% of the picture.
        const int spots = static_cast<int>(base_.total() * 0.012);
        for (int k = 0; k < spots; ++k) {
            const int x = r.uniform(0, base_.cols), y = r.uniform(0, base_.rows);
            const float v = static_cast<float>(r.uniform(0.85, 1.05));
            base_.at<cv::Vec3f>(y, x) = cv::Vec3f(v, 0.97f * v, 0.6f * v);
        }
        // Scaled so that at an exposure of 1 the highlights are on the target.
        std::vector<float> values;
        for (int y = 0; y < base_.rows; y += 4) {
            for (int x = 0; x < base_.cols; x += 4) {
                const cv::Vec3f p = base_.at<cv::Vec3f>(y, x);
                values.push_back(std::max(p[0], std::max(p[1], p[2])));
            }
        }
        scale_ = static_cast<float>(target / percentileOf(std::move(values), 99.5));
    }

    // Taken by the sensor for frames delivered `latency` after this one.
    void set(double exposure) {
        if (ignoreSets) return;
        queue_.push_back({delivered_ + latency_ + 1, exposure});
    }

    bool ignoreSets = false;

    bool read(cv::Mat& frame, long long& seq, double& age, double timeout) override {
        age = 0.0;
        return readNew(delivered_, frame, seq, timeout);
    }

    bool readNew(long long afterSeq, cv::Mat& frame, long long& seq, double timeout) override {
        (void)afterSeq;
        (void)timeout;
        ++delivered_;
        while (!queue_.empty() && queue_.front().first <= delivered_) {
            current_ = queue_.front().second;
            queue_.erase(queue_.begin());
        }
        const double wobble = flicker_ > 0.0 ? 1.0 + flicker_ * uniform_(rng_) : 1.0;
        cv::Mat f32 = base_ * static_cast<float>(current_ * wobble * scale_);
        cv::Mat noise(f32.size(), CV_32FC3);
        cv::randn(noise, 0.0, 1.5);
        f32 += noise;
        f32.convertTo(frame, CV_8UC3);  // saturates at 255, as the sensor does
        seq = delivered_;
        return true;
    }

    double fps() override { return 1.9; }

private:
    int latency_ = 0;
    double flicker_ = 0.0;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_{-1.0, 1.0};
    cv::Mat base_;
    float scale_ = 1.0f;
    double current_ = 1.0;
    std::vector<std::pair<long long, double>> queue_;
    long long delivered_ = 0;
};

Config benchConfig() {
    Config cfg;
    return cfg;
}

double targetOf(const Config& cfg) { return cfg.target_highlight_level * cfg.clear_mat_headroom; }

// The search this replaces, as it was: one frame per step.
double oldSearch(SimCamera& cam, const Config& cfg, double current, double lo, double hi) {
    const double target = targetOf(cfg);
    long long seq = -1;
    for (int step = 0; step < cfg.auto_expose_max_steps; ++step) {
        cv::Mat frame;
        long long newSeq = 0;
        cam.readNew(seq, frame, newSeq, 8.0);
        seq = newSeq;
        std::vector<float> values;
        const cv::Mat s = strideSample(frame, 4);
        for (int y = 0; y < s.rows; ++y) {
            const unsigned char* p = s.ptr<unsigned char>(y);
            for (int x = 0; x < s.cols; ++x) {
                values.push_back(static_cast<float>(std::max(p[3 * x], std::max(p[3 * x + 1], p[3 * x + 2]))));
            }
        }
        const double level = percentileOf(std::move(values), 99.5);
        if (std::abs(level - target) <= cfg.auto_expose_tolerance) break;
        double factor = std::min(2.0, std::max(0.5, target / std::max(level, 1.0)));
        const double next = std::min(hi, std::max(lo, current * factor));
        if (next == current) break;
        current = next;
        cam.set(current);
    }
    return current;
}

ExposureSearch search(SimCamera& cam, const Config& cfg, double start, double lo, double hi) {
    ExposureSearch out;
    meterExposure(cam, [&cam](double e) { cam.set(e); return true; }, start, lo, hi, cfg, out);
    return out;
}

void searchChecks() {
    const Config cfg = benchConfig();
    const double lo = 0.001, hi = 50.0;
    const std::vector<double> starts = {0.05, 0.3, 1.0, 3.0, 10.0, 20.0};

    std::printf("\nthe search as it was, one frame per step (for comparison, not checked)\n");
    for (int latency = 0; latency <= 2; ++latency) {
        std::string line;
        for (double s : starts) {
            SimCamera cam(s, latency, 0.0, targetOf(cfg), 7u);
            line += " " + ratio(oldSearch(cam, cfg, s, lo, hi));
        }
        std::printf("    %d frame(s) behind, from 0.05x..20x ends at:%s\n", latency, line.c_str());
    }

    std::printf("\nthe search now, from anywhere, however far behind the camera is\n");
    for (int latency = 0; latency <= 4; ++latency) {
        double worst = 0.0;
        int most = 0;
        double lowest = 1e9, highest = 0.0;
        for (double s : starts) {
            SimCamera cam(s, latency, 0.0, targetOf(cfg), 11u + static_cast<unsigned>(latency));
            const ExposureSearch r = search(cam, cfg, s, lo, hi);
            worst = std::max(worst, std::abs(r.exposure_us - 1.0));
            most = std::max(most, r.frames);
            lowest = std::min(lowest, r.exposure_us);
            highest = std::max(highest, r.exposure_us);
        }
        check(worst < 0.08, std::to_string(latency) + " frame(s) behind: every start lands within 8%",
              "worst " + vjson::formatFixed(100.0 * worst, 1) + "%, " + ratio(lowest) + " to " +
                  ratio(highest) + ", at most " + std::to_string(most) + " frames");
        check(highest / lowest < 1.12, "  and the starts agree with each other",
              "spread " + vjson::formatFixed(100.0 * (highest / lowest - 1.0), 1) + "%");
        check(most <= cfg.auto_expose_max_frames + cfg.auto_expose_settle_frames,
              "  within the frames it is allowed", std::to_string(most));
    }

    std::printf("\nfurther behind than this camera has ever been\n");
    for (int latency = 5; latency <= 6; ++latency) {
        double worst = 0.0;
        for (double s : starts) {
            SimCamera cam(s, latency, 0.0, targetOf(cfg), 23u);
            const ExposureSearch r = search(cam, cfg, s, lo, hi);
            worst = std::max(worst, std::abs(r.exposure_us - 1.0));
        }
        check(worst < 0.15, std::to_string(latency) + " frames behind: every start still lands within 15%",
              "worst " + vjson::formatFixed(100.0 * worst, 1) + "%");
    }

    std::printf("\nclipped white, far over\n");
    {
        SimCamera cam(40.0, 2, 0.0, targetOf(cfg), 5u);
        const ExposureSearch r = search(cam, cfg, 40.0, lo, hi);
        check(std::abs(r.exposure_us - 1.0) < 0.08, "40x over, 2 frames behind: comes down to the target",
              ratio(r.exposure_us) + " after " + std::to_string(r.frames) + " frames");
    }

    std::printf("\na light too dim for the range\n");
    {
        SimCamera cam(0.5, 1, 0.0, targetOf(cfg), 9u);
        const ExposureSearch r = search(cam, cfg, 0.5, lo, 0.6);
        check(r.clamped && std::abs(r.exposure_us - 0.6) < 1e-9, "pinned at the end of the range, and says so",
              ratio(r.exposure_us) + (r.clamped ? ", clamped" : ", not clamped"));
    }

    std::printf("\na camera that never takes the setting\n");
    {
        SimCamera cam(0.3, 1, 0.0, targetOf(cfg), 13u);
        cam.ignoreSets = true;
        const ExposureSearch r = search(cam, cfg, 0.3, lo, hi);
        check(r.exposure_us < 1.5 && r.exposure_us > 0.5,
              "stops at its first estimate instead of running off",
              ratio(r.exposure_us) + " after " + std::to_string(r.frames) + " frames");
    }

    std::printf("\na flickering light, +-4%% frame to frame\n");
    {
        double worst = 0.0;
        for (double s : starts) {
            SimCamera cam(s, 2, 0.04, targetOf(cfg), 17u);
            const ExposureSearch r = search(cam, cfg, s, lo, hi);
            worst = std::max(worst, std::abs(r.exposure_us - 1.0));
        }
        check(worst < 0.15, "every start lands within 15%", "worst " + vjson::formatFixed(100.0 * worst, 1) + "%");
    }
}

void benchFileChecks() {
    std::printf("\nremembered in bench.json\n");
    Config cfg;
    cfg.bench_file = "exposure_check_bench.json";
    std::remove(cfg.bench_file.c_str());

    vjson::Value lens = vjson::Value::object();
    lens.set("lens_focal_length_mm", vjson::Value::integer(12));
    vjson::writeFile(cfg.bench_file, lens, 2);

    BenchExposure e;
    check(!loadBenchExposure(cfg, e), "none saved yet");

    BenchExposure set;
    set.us = 23456.4;
    set.level = 139.2;
    check(saveBenchExposure(cfg, set), "saved");
    const bool loaded = loadBenchExposure(cfg, e);
    check(loaded && e.us == 23456.0 && e.level == 139.0 && !e.byHand, "read back, set from the mat",
          vjson::formatFixed(e.us, 0) + " us at level " + vjson::formatFixed(e.level, 0));

    vjson::Value bench;
    std::string error;
    vjson::parseFile(cfg.bench_file, bench, error);
    check(bench.getNumber("lens_focal_length_mm", 0.0) == 12.0, "the lens is still there");

    const std::string byHand = settleExposure(cfg, true, 18000.0);
    check(loadBenchExposure(cfg, e) && e.us == 18000.0 && e.byHand, "--exposure-us 18000 is saved as set by hand",
          byHand);
    check(settleExposure(cfg, false, 0.0).empty(), "not given: nothing changes");
    const std::string forgot = settleExposure(cfg, true, 0.0);
    check(!loadBenchExposure(cfg, e), "--exposure-us 0 forgets it", forgot);
    vjson::parseFile(cfg.bench_file, bench, error);
    check(bench.getNumber("lens_focal_length_mm", 0.0) == 12.0 && !bench.has("exposure_set_by"),
          "and leaves the lens");
    std::remove(cfg.bench_file.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) != "--selftest") {
        std::printf("usage: exposure_check --selftest\n");
        return 2;
    }
    searchChecks();
    benchFileChecks();
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
