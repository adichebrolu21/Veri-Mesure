// Camera connection, acquisition and the background grab thread.
//
// Confirmed hardware: TRI200S-CC (colour variant). BGR8 is requested directly
// from the camera's PixelFormat node, which has Bayer debayering handled on
// camera or in the driver, so no conversion is needed on the PC.
//
// The whole Arena SDK dependency is behind VERIMEASURE_WITH_ARENA. Built
// without it, the module still compiles and links, ARENA_AVAILABLE is false,
// and the offline path (calibrate from a saved capture) keeps working.

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "verimeasure/config.hpp"
#include "verimeasure/frame_source.hpp"

namespace verimeasure {

extern const bool ARENA_AVAILABLE;

class Camera {
public:
    Camera();
    ~Camera();

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    // Discover and connect to the first available Lucid device. Throws
    // std::runtime_error with the same diagnostic list the Python version
    // printed when nothing is found.
    void connect();

    // Continuous acquisition and BGR8 output. Returns the pixel format the
    // camera reports afterwards.
    std::string configureStream();

    void disableAutoAdjustments();
    void tuneStream(const Config& cfg);

    void startStream(int buffers);
    void stopStream();
    void destroy();

    // Grab one buffer. Returns false on failure with the reason in error.
    // incomplete is set when the buffer arrived with dropped GigE packets; the
    // frame is then left empty and must be discarded rather than measured.
    // acceptIncomplete mirrors the bare Phase 1 loop, which displayed whatever
    // arrived; the measurement path leaves it false so partial buffers are
    // discarded rather than measured.
    bool grab(cv::Mat& frame, int timeoutMs, bool& incomplete, std::string& error,
              bool acceptIncomplete = false);

    bool exposureRange(double& current, double& lo, double& hi);
    bool setExposure(double value);

    bool connected() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Background acquisition thread holding exactly one frame: the newest.
//
// Grabbing on the main loop couples display latency to processing time, so a
// slow frame pushes the whole feed behind. Here the grab thread keeps draining
// the camera regardless of what the main loop is doing, and the main loop
// always picks up whatever arrived most recently.
//
// It also owns recovery. Incomplete buffers from dropped GigE packets are
// discarded rather than measured, and a run of failed grabs restarts the stream
// instead of leaving the application sitting on a dead socket.
class CameraStream : public FrameSource {
public:
    CameraStream(Camera& camera, const Config& cfg);
    ~CameraStream() override;

    CameraStream& start();

    bool read(cv::Mat& frame, long long& seq, double& age, double timeout = 10.0) override;
    bool readNew(long long afterSeq, cv::Mat& frame, long long& seq,
                 double timeout = 10.0) override;
    double fps() override;
    void stop() override;

    long long incompleteCount() const override { return incomplete_.load(); }
    long long restartCount() const override { return restarts_.load(); }
    long long grabbedCount() const override { return grabbed_.load(); }
    std::string error() const override;

private:
    void run();
    void restart();

    Camera& camera_;
    Config cfg_;

    mutable std::mutex lock_;
    cv::Mat frame_;
    long long seq_ = 0;
    double stamp_ = 0.0;
    std::deque<double> times_;
    std::string error_;

    std::atomic<bool> stop_{false};
    std::thread thread_;
    std::atomic<long long> grabbed_{0};
    std::atomic<long long> incomplete_{0};
    std::atomic<long long> restarts_{0};
};

// Lock the camera response: no auto exposure, gain or white balance, and a
// linear intensity ramp for the sub-pixel edge fit.
void disableCameraAutoAdjustments(Camera& camera);

// Stream settings that decide whether the feed is live or minutes behind.
void tuneStream(Camera& camera, const Config& cfg);

// Say plainly what the link can deliver, because no amount of code changes it.
void reportBandwidth(cv::Size frameSize, int channels, const Config& cfg);

// Drive exposure so the brightest content sits just below clipping, then lock
// it. Saturated white samples bloom, and blooming reads as extra millimetres.
// `level`, when given, gets the highlight level the exposure was locked at;
// `search`, when given, gets the whole of what the search ended on.
struct ExposureSearch;
bool autoSetExposure(Camera& camera, FrameSource& stream, const Config& cfg, double& locked,
                     double* level = nullptr, ExposureSearch* search = nullptr);

// What an exposure search ended on.
struct ExposureSearch {
    double exposure_us = 0.0;
    double level = 0.0;    // 99.5th percentile of the brightest channel there, 0-255
    int frames = 0;        // frames it looked at
    bool clamped = false;  // pinned at the end of the camera's range short of the target
    bool settled = false;  // the last level held still over consecutive frames
};

// The search itself, with the camera reduced to its range and a way to set it,
// so it can be run against a simulated camera (tests/exposure_check.cpp).
//
// A frame read straight after the exposure is changed is usually still one
// taken before the change: at two frames a second one frame is on the wire
// while the next is being exposed. Judged on those, a search keeps correcting
// a change that has not shown yet and stops wherever it happens to be when its
// steps run out - anywhere from a third of the right exposure to three times
// it, starting from wherever the last run left the camera. So after every
// change this waits for the picture to move and then hold still, and only then
// believes it. The sensor is linear (no gamma, no LUT), so one good reading
// gives the right exposure in a single step and the search is usually two.
bool meterExposure(FrameSource& stream, const std::function<bool(double)>& setExposure,
                   double current, double lo, double hi, const Config& cfg, ExposureSearch& out);

// The highlight level the clear base is metered to: the bench's own target
// when it has been given one (cfg.exposure_target_level), otherwise
// target_highlight_level x clear_mat_headroom.
double meteringTarget(const Config& cfg);

// The highlight level of the picture as it is now, once it holds still. For
// checking a saved exposure against the light it was saved under.
bool steadyHighlightLevel(FrameSource& stream, const Config& cfg, double& level);

// Average several frames. Single-frame read noise is the floor the sub-pixel
// edge fit works against, and averaging N frames divides it by root N. Bounded
// by a deadline so a slow or stalled link cannot hang startup. Throws
// std::runtime_error if no frames arrive at all.
cv::Mat captureBackground(FrameSource& stream, const Config& cfg);

// Wait for the surface to stop changing and come away with the background
// reference, from the very frames that proved it had stopped.
//
// These were two passes, one after the other: watch frames go by until enough
// consecutive ones matched, then start again and average the next several into
// the reference. Every frame of the first pass was thrown away, and at full
// resolution over a gigabit link a frame costs about half a second, so the
// second pass added five of them to every startup and every recalibration for
// no better a reference.
//
// The background is now averaged from the tail of the run being proved still.
// It is the same number of frames, consecutive, averaged the same way, of a
// surface confirmed to have held still across all of them; if anything moves
// part way through, the part-built average is discarded along with the count.
// Bounded by the same deadlines as before, and it falls back to taking frames
// as they come if the scene never settles, so a jittery bench still boots.
// Throws std::runtime_error if no frames arrive at all.
//
// `progress`, when given, is called after every frame with how many frames of
// the wait are done and how many it needs, so a caller can show the wait
// rather than sit silent through it. `done` drops back to zero whenever
// something moves. Nothing about the capture depends on it.
using SettleProgress = std::function<void(int done, int needed)>;
cv::Mat captureStaticBackground(FrameSource& stream, const Config& cfg,
                                const SettleProgress& progress = SettleProgress());

}  // namespace verimeasure
