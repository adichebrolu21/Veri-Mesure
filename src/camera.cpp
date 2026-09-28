#include "verimeasure/camera.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "verimeasure/cv_compat.hpp"

#include "verimeasure/calibration.hpp"
#include "verimeasure/engine.hpp"
#include "verimeasure/numeric.hpp"

#ifdef VERIMEASURE_WITH_ARENA
#include "ArenaApi.h"
#endif

namespace verimeasure {

#ifdef VERIMEASURE_WITH_ARENA
const bool ARENA_AVAILABLE = true;
#else
const bool ARENA_AVAILABLE = false;
#endif

namespace {

void sleepSeconds(double s) {
    std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

}  // namespace

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

#ifdef VERIMEASURE_WITH_ARENA

struct Camera::Impl {
    Arena::ISystem* system = nullptr;
    Arena::IDevice* device = nullptr;
    bool streaming = false;
};

Camera::Camera() : impl_(new Impl) {}

Camera::~Camera() {
    destroy();
}

bool Camera::connected() const {
    return impl_ && impl_->device != nullptr;
}

void Camera::connect() {
    std::printf("Searching for connected Lucid devices...\n");
    std::fflush(stdout);

    impl_->system = Arena::OpenSystem();
    impl_->system->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> infos = impl_->system->GetDevices();

    if (infos.empty()) {
        throw std::runtime_error(
            "No Lucid devices found on the network. Check:\n"
            "  - Camera is powered and connected via GigE\n"
            "  - Camera is on the same subnet as this PC\n"
            "  - Arena SDK / driver is installed correctly\n"
            "  - No firewall blocking GigE Vision discovery");
    }

    std::printf("Found %d device(s):\n", static_cast<int>(infos.size()));
    // Arena declares ModelName(), SerialNumber() and IpAddressStr() as non-const
    // member functions, so the loop variable cannot be const. `infos` is our own
    // local copy returned by value from GetDevices(), so binding a mutable
    // reference to its elements is safe and nothing here modifies them.
    for (Arena::DeviceInfo& info : infos) {
        std::printf("  - %s  %s  %s\n", info.ModelName().c_str(), info.SerialNumber().c_str(),
                    info.IpAddressStr().c_str());
    }

    // Create device from the first one found.
    // NOTE: if you have more than one Lucid camera on the network, select by
    // serial number here instead of index 0.
    impl_->device = impl_->system->CreateDevice(infos[0]);
    std::printf("Connected to device: %s %s\n", infos[0].ModelName().c_str(),
                infos[0].SerialNumber().c_str());
    std::fflush(stdout);
}

std::string Camera::configureStream() {
    GenApi::INodeMap* nodemap = impl_->device->GetNodeMap();

    // Continuous acquisition: camera streams frames until we stop it. This is
    // the simplest mode for prototyping. We are NOT using hardware triggering.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "AcquisitionMode", "Continuous");

    // Confirmed colour camera - request BGR8 directly so OpenCV can use the
    // frame with no further conversion.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat", "BGR8");
    const GenICam::gcstring fmt = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    std::printf("Camera PixelFormat set to: %s\n", fmt.c_str());
    std::fflush(stdout);
    return std::string(fmt.c_str());
}

namespace {

template <typename T>
bool setNode(GenApi::INodeMap* nodemap, const char* name, T value) {
    if (nodemap == nullptr) return false;
    try {
        Arena::SetNodeValue<T>(nodemap, name, value);
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

void Camera::disableAutoAdjustments() {
    GenApi::INodeMap* nodemap = impl_->device->GetNodeMap();
    const char* offNodes[3] = {"ExposureAuto", "GainAuto", "BalanceWhiteAuto"};
    for (const char* name : offNodes) {
        if (setNode<GenICam::gcstring>(nodemap, name, "Off")) {
            std::printf("  %s -> Off\n", name);
        } else {
            std::printf("  WARNING: could not set %s\n", name);
        }
    }
    // Gamma and LUT change the intensity ramp the sub-pixel edge fit runs on.
    // Linear response gives a cleaner, more symmetric gradient peak.
    for (const char* name : {"GammaEnable", "LUTEnable"}) {
        if (setNode<bool>(nodemap, name, false)) std::printf("  %s -> false\n", name);
    }
    std::fflush(stdout);
}

void Camera::tuneStream(const Config& cfg) {
    (void)cfg;
    GenApi::INodeMap* tl = impl_->device->GetTLStreamNodeMap();
    std::vector<std::string> applied;

    if (setNode<GenICam::gcstring>(tl, "StreamBufferHandlingMode", "NewestOnly")) {
        applied.push_back("StreamBufferHandlingMode=NewestOnly");
    }
    if (setNode<bool>(tl, "StreamAutoNegotiatePacketSize", true)) {
        applied.push_back("StreamAutoNegotiatePacketSize=True");
    }
    if (setNode<bool>(tl, "StreamPacketResendEnable", true)) {
        applied.push_back("StreamPacketResendEnable=True");
    }

    GenApi::INodeMap* nodemap = impl_->device->GetNodeMap();
    if (setNode<GenICam::gcstring>(nodemap, "DeviceLinkThroughputLimitMode", "Off")) {
        applied.push_back("DeviceLinkThroughputLimitMode=Off");
    }
    if (setNode<bool>(nodemap, "AcquisitionFrameRateEnable", false)) {
        applied.push_back("AcquisitionFrameRateEnable=False");
    }

    for (const std::string& item : applied) std::printf("  %s\n", item.c_str());

    bool haveHandling = false;
    for (const std::string& item : applied) {
        if (item.rfind("StreamBufferHandlingMode", 0) == 0) haveHandling = true;
    }
    if (!haveHandling) {
        std::printf(
            "  WARNING: could not set StreamBufferHandlingMode. If the feed lags behind reality, "
            "this node is why.\n");
    }
    std::fflush(stdout);
}

void Camera::startStream(int buffers) {
    impl_->device->StartStream(static_cast<size_t>(buffers));
    impl_->streaming = true;
}

void Camera::stopStream() {
    if (impl_ && impl_->device && impl_->streaming) {
        try {
            impl_->device->StopStream();
        } catch (...) {
        }
        impl_->streaming = false;
    }
}

void Camera::destroy() {
    if (!impl_) return;
    stopStream();
    if (impl_->device && impl_->system) {
        try {
            impl_->system->DestroyDevice(impl_->device);
        } catch (...) {
        }
        impl_->device = nullptr;
    }
    if (impl_->system) {
        try {
            Arena::CloseSystem(impl_->system);
        } catch (...) {
        }
        impl_->system = nullptr;
    }
}

bool Camera::grab(cv::Mat& frame, int timeoutMs, bool& incomplete, std::string& error,
                  bool acceptIncomplete) {
    incomplete = false;
    error.clear();
    Arena::IImage* image = nullptr;
    try {
        image = impl_->device->GetImage(static_cast<uint64_t>(timeoutMs));
    } catch (const GenICam::GenericException& e) {
        error = e.what();
        return false;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }

    bool ok = false;
    try {
        incomplete = image->IsIncomplete();
        if (incomplete && !acceptIncomplete) {
            // discarded
        } else {
            // Direct memory view over the buffer, then a copy, since the
            // buffer's memory becomes invalid once requeued back to the camera.
            const int h = static_cast<int>(image->GetHeight());
            const int w = static_cast<int>(image->GetWidth());
            const cv::Mat view(h, w, CV_8UC3,
                               const_cast<uint8_t*>(image->GetData()));
            frame = view.clone();
            ok = true;
        }
    } catch (const std::exception& e) {
        error = e.what();
    }

    try {
        impl_->device->RequeueBuffer(image);
    } catch (...) {
    }
    return ok;
}

bool Camera::exposureRange(double& current, double& lo, double& hi) {
    try {
        GenApi::CFloatPtr node = impl_->device->GetNodeMap()->GetNode("ExposureTime");
        if (!node.IsValid()) return false;
        current = node->GetValue();
        lo = node->GetMin();
        hi = node->GetMax();
        return true;
    } catch (...) {
        return false;
    }
}

bool Camera::setExposure(double value) {
    try {
        GenApi::CFloatPtr node = impl_->device->GetNodeMap()->GetNode("ExposureTime");
        if (!node.IsValid()) return false;
        node->SetValue(value);
        return true;
    } catch (...) {
        return false;
    }
}

#else  // no Arena SDK

struct Camera::Impl {
    int unused = 0;
};

Camera::Camera() : impl_(new Impl) {}
Camera::~Camera() = default;

bool Camera::connected() const { return false; }

void Camera::connect() {
    throw std::runtime_error(
        "Arena SDK not compiled in. Rebuild with -DVERIMEASURE_WITH_ARENA=ON on the machine that "
        "has the camera, or work from saved captures with --calibrate-image.");
}

std::string Camera::configureStream() { return std::string(); }
void Camera::disableAutoAdjustments() {}
void Camera::tuneStream(const Config&) {}
void Camera::startStream(int) {}
void Camera::stopStream() {}
void Camera::destroy() {}

bool Camera::grab(cv::Mat&, int, bool& incomplete, std::string& error, bool) {
    incomplete = false;
    error = "Arena SDK not compiled in";
    return false;
}

bool Camera::exposureRange(double&, double&, double&) { return false; }
bool Camera::setExposure(double) { return false; }

#endif

// ---------------------------------------------------------------------------
// CameraStream
// ---------------------------------------------------------------------------

CameraStream::CameraStream(Camera& camera, const Config& cfg) : camera_(camera), cfg_(cfg) {}

CameraStream::~CameraStream() {
    stop();
}

CameraStream& CameraStream::start() {
    thread_ = std::thread(&CameraStream::run, this);
    return *this;
}

void CameraStream::run() {
    int failures = 0;
    while (!stop_.load()) {
        cv::Mat frame;
        bool incomplete = false;
        std::string error;
        const bool ok = camera_.grab(frame, cfg_.grab_timeout_ms, incomplete, error);

        if (!ok && !incomplete && !error.empty()) {
            ++failures;
            {
                std::lock_guard<std::mutex> guard(lock_);
                error_ = error;
            }
            if (failures >= cfg_.max_consecutive_grab_failures) {
                restart();
                failures = 0;
            }
            sleepSeconds(0.05);
            continue;
        }

        if (incomplete) {
            ++incomplete_;
            continue;
        }
        if (!ok) continue;

        failures = 0;
        const double now = nowSeconds();
        {
            std::lock_guard<std::mutex> guard(lock_);
            error_.clear();
            frame_ = frame;
            ++seq_;
            stamp_ = now;
            times_.push_back(now);
            while (times_.size() > 30) times_.pop_front();
        }
        ++grabbed_;
    }
}

void CameraStream::restart() {
    {
        std::lock_guard<std::mutex> guard(lock_);
        error_ = "grab timed out, restarting the stream";
    }
    std::printf("[%s] grab timed out, restarting the stream\n", timestampClock().c_str());
    std::fflush(stdout);

    camera_.stopStream();
    sleepSeconds(0.5);
    try {
        camera_.startStream(cfg_.stream_buffers);
        ++restarts_;
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> guard(lock_);
        error_ = std::string("stream restart failed: ") + e.what();
        sleepSeconds(1.0);
    }
}

double CameraStream::fps() {
    std::vector<double> t;
    {
        std::lock_guard<std::mutex> guard(lock_);
        t.assign(times_.begin(), times_.end());
    }
    if (t.size() < 2) return 0.0;
    const double span = t.back() - t.front();
    return span > 0.0 ? (static_cast<double>(t.size()) - 1.0) / span : 0.0;
}

bool CameraStream::read(cv::Mat& frame, long long& seq, double& age, double timeout) {
    const double deadline = nowSeconds() + timeout;
    while (true) {
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!frame_.empty()) {
                frame = frame_;
                seq = seq_;
                age = nowSeconds() - stamp_;
                return true;
            }
            seq = seq_;
        }
        if (stop_.load() || nowSeconds() > deadline) {
            age = 0.0;
            return false;
        }
        sleepSeconds(0.003);
    }
}

bool CameraStream::readNew(long long afterSeq, cv::Mat& frame, long long& seq, double timeout) {
    const double deadline = nowSeconds() + timeout;
    while (true) {
        {
            std::lock_guard<std::mutex> guard(lock_);
            if (!frame_.empty() && seq_ > afterSeq) {
                frame = frame_;
                seq = seq_;
                return true;
            }
            seq = seq_;
        }
        if (stop_.load() || nowSeconds() > deadline) return false;
        sleepSeconds(0.003);
    }
}

void CameraStream::stop() {
    stop_.store(true);
    if (thread_.joinable()) thread_.join();
}

std::string CameraStream::error() const {
    std::lock_guard<std::mutex> guard(lock_);
    return error_;
}

// ---------------------------------------------------------------------------
// Startup helpers
// ---------------------------------------------------------------------------

void disableCameraAutoAdjustments(Camera& camera) {
    camera.disableAutoAdjustments();
}

void tuneStream(Camera& camera, const Config& cfg) {
    camera.tuneStream(cfg);
}

void reportBandwidth(cv::Size frameSize, int channels, const Config& cfg) {
    (void)cfg;
    const double perFrame = static_cast<double>(frameSize.width) * frameSize.height * channels;
    const double fps1g = 118e6 / perFrame;
    std::printf("\nFrame: %d x %d x %d = %.1f MB\n", frameSize.width, frameSize.height, channels,
                perFrame / 1e6);
    std::printf(
        "Ceiling on a 1 GigE link: about %.1f fps. That is a wire limit, not a software one.\n",
        fps1g);
    if (channels == 3) {
        std::printf(
            "Switching the camera to an 8-bit Bayer format and debayering on the PC would cut this "
            "to a third of the traffic and roughly triple the frame rate. Worth doing if you need "
            "a smoother feed, though check the colours look right afterwards, since a wrong Bayer "
            "phase leaves artefacts at edges that would hurt the sub-pixel fit.\n");
    }
    std::fflush(stdout);
}

namespace {

// The 99.5th percentile of the brightest channel, over every fourth pixel.
double highlightLevel(const cv::Mat& frame) {
    const cv::Mat sampled = strideSample(frame, 4);

    // The brightest CHANNEL, not the brightness.
    //
    // A sensor clips one channel at a time, and grey is not one of them: it is
    // 0.114 blue plus 0.587 green plus 0.299 red, so on anything that is not
    // neutral the strongest channel runs well above the grey the exposure is
    // being set by. Measured on this bench, the mat reads 175 in grey and 226
    // in red - so an exposure aimed at 225 of grey has already put red against
    // the ceiling, and the salmon garment that followed came back with 48% of
    // its red pixels at 255. What clips is what has to be measured.
    std::vector<float> values;
    values.reserve(sampled.total());
    for (int y = 0; y < sampled.rows; ++y) {
        const unsigned char* p = sampled.ptr<unsigned char>(y);
        for (int x = 0; x < sampled.cols; ++x) {
            const int i = x * 3;
            const unsigned char mx = (std::max)(p[i], (std::max)(p[i + 1], p[i + 2]));
            values.push_back(static_cast<float>(mx));
        }
    }
    return percentileOf(std::move(values), 99.5);
}

// Two readings of the same picture: within two levels, or a few percent.
bool sameLevel(double a, double b, const Config& cfg) {
    return std::abs(a - b) <= (std::max)(2.0, cfg.auto_expose_agree_fraction * (std::max)(a, b));
}

// Frames until the level holds still over two in a row.
//
// `changed` says the exposure has just been changed by enough to show, and
// `before` is the level the old exposure gave. A frame still reading that was
// taken before the change, however many of them arrive, and is not counted;
// which is the whole difference between this and reading the next frame.
// Gives up after `budget` frames with the middle of what it did see. `moved` is
// set once a frame has shown the change.
bool waitForLevel(FrameSource& stream, long long& seq, const Config& cfg, bool changed, double before,
                  int budget, double& level, int& frames, bool& moved) {
    std::vector<double> seen;
    double prev = -1.0;
    moved = !changed;
    for (int k = 0; k < budget; ++k) {
        cv::Mat frame;
        long long newSeq = 0;
        if (!stream.readNew(seq, frame, newSeq, 8.0)) break;
        seq = newSeq;
        ++frames;
        const double now = highlightLevel(frame);
        if (!moved && sameLevel(now, before, cfg)) continue;
        moved = true;
        seen.push_back(now);
        if (prev >= 0.0 && sameLevel(now, prev, cfg)) {
            level = 0.5 * (now + prev);
            return true;
        }
        prev = now;
    }
    if (seen.empty()) {
        level = before;
        return false;
    }
    std::sort(seen.begin(), seen.end());
    level = seen[seen.size() / 2];
    return false;
}

}  // namespace

double meteringTarget(const Config& cfg) {
    return cfg.exposure_target_level.value_or(cfg.target_highlight_level * cfg.clear_mat_headroom);
}

bool meterExposure(FrameSource& stream, const std::function<bool(double)>& setExposure,
                   double current, double lo, double hi, const Config& cfg, ExposureSearch& out) {
    out = ExposureSearch();
    out.exposure_us = current;

    // Room for something brighter than the bench to be put on it.
    //
    // This runs on a clear mat, and the mat is the darkest thing that will ever
    // be in the picture. Exposing it to the top of the range is exposing for
    // the one scene that does not matter: every frame afterwards has a garment
    // in it, and a garment is pale cloth under a lamp, several times brighter
    // than the dark mat it lies on. The headroom is what that garment is given
    // to land in.
    const double target = meteringTarget(cfg);
    const int settle = (std::max)(3, cfg.auto_expose_settle_frames);
    const int allowed = (std::max)(settle, cfg.auto_expose_max_frames);

    long long seq = -1;
    double level = 0.0;
    bool moved = false;
    bool steady = waitForLevel(stream, seq, cfg, false, 0.0, settle, level, out.frames, moved);
    if (out.frames == 0) return false;

    for (int step = 0; step < (std::max)(1, cfg.auto_expose_max_steps); ++step) {
        if (std::abs(level - target) <= cfg.auto_expose_tolerance) break;
        if (out.frames >= allowed) break;

        // Parenthesised deliberately. On Windows, ArenaApi.h pulls in
        // windows.h, which defines min and max as function-like macros; an
        // unparenthesised std::min( is then rewritten into nonsense before the
        // compiler sees it. Wrapping the name stops the macro from firing
        // regardless of include order or of whether NOMINMAX reached this
        // translation unit.
        //
        // The sensor is linear, so a level that is not clipped says exactly
        // how far to go. A clipped one only says "too far", by an unknown
        // amount, so it comes down as far as a step may go: under-exposed is
        // measurable and one more step puts it right, where a cut too shallow
        // to unclip it cannot be told from a camera still showing old frames.
        const bool clipped = level >= 250.0;
        double factor = clipped ? 1.0 / 16.0 : target / (std::max)(level, 1.0);
        factor = (std::min)(16.0, (std::max)(1.0 / 16.0, factor));
        const double next = (std::min)(hi, (std::max)(lo, current * factor));

        // Pinned against the end of the range, with nothing left to try.
        // Every further step would command this same exposure and measure the
        // same level again, so stop at the first one rather than the last.
        if (next == current) {
            out.clamped = true;
            break;
        }
        if (!setExposure(next)) break;

        // Whether the change is big enough for a frame taken after it to be
        // told from one taken before it. It nearly always is: a change small
        // enough not to show would have been inside the tolerance above.
        const double expected = clipped ? 0.0 : level * (next / current);
        const bool visible = clipped || !sameLevel(expected, level, cfg);
        const double before = level;
        current = next;
        out.exposure_us = current;
        steady = waitForLevel(stream, seq, cfg, visible, before, settle, level, out.frames, moved);

        // Nothing but frames reading what the old exposure read, for as long
        // as it would wait - several times longer than the camera is ever
        // behind. From a clipped picture that means it is still clipped, and
        // it comes down again. Otherwise the camera has not taken the setting,
        // and the level in hand is the old exposure's: another step taken on
        // it would correct the same error twice, so the exposure is left where
        // the last good reading put it.
        if (!moved && !clipped) {
            level = expected;
            break;
        }
    }

    out.level = level;
    out.settled = steady;
    return true;
}

bool steadyHighlightLevel(FrameSource& stream, const Config& cfg, double& level) {
    long long seq = -1;
    int frames = 0;
    bool moved = false;
    // A couple of frames first: anything in flight when the exposure was
    // last set was taken before it.
    for (int k = 0; k < 2; ++k) {
        cv::Mat frame;
        long long newSeq = 0;
        if (!stream.readNew(seq, frame, newSeq, 8.0)) return false;
        seq = newSeq;
    }
    waitForLevel(stream, seq, cfg, false, 0.0, (std::max)(3, cfg.auto_expose_settle_frames), level,
                 frames, moved);
    return frames > 0;
}

bool autoSetExposure(Camera& camera, FrameSource& stream, const Config& cfg, double& locked,
                     double* levelOut, ExposureSearch* searchOut) {
    double current = 0.0;
    double lo = 0.0;
    double hi = 0.0;
    if (!camera.exposureRange(current, lo, hi)) {
        std::printf("  exposure node unavailable, leaving exposure as configured\n");
        std::fflush(stdout);
        return false;
    }

    ExposureSearch search;
    const bool ok = meterExposure(
        stream, [&camera](double us) { return camera.setExposure(us); }, current, lo, hi, cfg, search);
    if (!ok) {
        std::printf("  no frames to set the exposure from, leaving it at %.0f us\n", current);
        std::fflush(stdout);
        return false;
    }

    const double target = meteringTarget(cfg);
    std::printf("  ExposureTime locked at %.0f us (99.5th percentile ~%.0f/255, %d frames%s)\n",
                search.exposure_us, search.level, search.frames,
                search.settled ? "" : ", the picture never held still");
    if (search.clamped && std::abs(search.level - target) > cfg.auto_expose_tolerance) {
        std::printf(
            "  NOTE: the exposure is at the end of its range and the picture is still %.0f levels "
            "short of the %.0f wanted. That is the lighting, not the camera. More light on the "
            "surface would also shorten the exposure, and the exposure is what currently sets the "
            "frame rate, so every wait measured in frames would shorten with it.\n",
            target - search.level, target);
    }
    std::fflush(stdout);
    locked = search.exposure_us;
    if (levelOut) *levelOut = search.level;
    if (searchOut) *searchOut = search;
    return true;
}

cv::Mat captureStaticBackground(FrameSource& stream, const Config& cfg,
                                const SettleProgress& progress) {
    const int wanted = (std::max)(1, cfg.background_average_frames);
    const int settleFrames = (std::max)(0, cfg.scene_settle_frames);
    const int block = (std::max)(1, cfg.scene_settle_block_px);

    // Where in the settling run the background starts being banked, so that the
    // frames averaged are the last ones before the surface is declared still.
    const int bankAfter = (std::max)(0, settleFrames - wanted);

    // How long the run of still frames has to get before the wait is over:
    // long enough to prove the surface still, and long enough to bank a full
    // reference from its tail. Only for reporting; the test below is the rule.
    const int needed = (std::max)(settleFrames, wanted);

    std::printf("Waiting for the surface to be still, banking the background as it settles...\n");
    std::fflush(stdout);

    cv::Mat prev;
    cv::Mat acc;
    int stable = 0;
    int banked = 0;
    long long seq = -1;
    double delta = 0.0;
    bool settled = false;
    const double settleDeadline = nowSeconds() + 30.0;

    while (nowSeconds() < settleDeadline) {
        cv::Mat frame;
        long long newSeq = 0;
        if (!stream.readNew(seq, frame, newSeq, 8.0)) {
            std::printf("  no frames arriving, continuing anyway\n");
            std::fflush(stdout);
            break;
        }
        seq = newSeq;

        // Block averages rather than single pixels, so that what is measured is
        // the surface rather than the sensor. See scene_settle_block_px.
        //
        // Not called "small": windows.h, which arrives here through ArenaApi.h,
        // defines that as a macro for char, and a cv::Mat declared with it stops
        // being a cv::Mat before the compiler ever sees it. Same trap as min and
        // max, which is why NOMINMAX is set for the whole target.
        cv::Mat blocks, gray8, gray;
        cv::resize(frame, blocks, cv::Size(), 1.0 / block, 1.0 / block, cv::INTER_AREA);
        cv::cvtColor(blocks, gray8, cv::COLOR_BGR2GRAY);
        gray8.convertTo(gray, CV_32F);

        bool steady = false;
        if (!prev.empty()) {
            delta = cv::mean(cv::abs(gray - prev))[0];
            steady = delta < cfg.scene_settle_tolerance;
        }
        prev = gray;

        if (!steady) {
            // Something moved. Anything banked so far was of a different scene,
            // so it goes with the count that vouched for it.
            stable = 0;
            banked = 0;
            acc.release();
            if (progress) progress(0, needed);
            continue;
        }

        ++stable;
        if (stable > bankAfter && banked < wanted) {
            cv::Mat f;
            frame.convertTo(f, CV_32FC3);
            if (acc.empty()) {
                acc = f;
            } else {
                acc += f;
            }
            ++banked;
        }
        if (progress) progress((std::min)(stable, needed), needed);

        if (stable >= settleFrames && banked >= wanted) {
            settled = true;
            break;
        }
    }

    if (settled) {
        std::printf("Surface is static: %d frames within %.2f, background taken from the last %d.\n",
                    stable, cfg.scene_settle_tolerance, banked);
    } else {
        std::printf("  surface never fully settled (last delta %.2f, allowed %.2f)\n", delta,
                    cfg.scene_settle_tolerance);
    }
    std::fflush(stdout);

    // However the wait ended, the reference still has to be a full average. If
    // the surface never held still long enough to bank one, take the frames as
    // they come, which is what this did before there was anything to bank.
    if (banked < wanted) {
        std::printf("Capturing background reference over up to %d more frames...\n",
                    wanted - banked);
        std::fflush(stdout);
        const double captureDeadline = nowSeconds() + 25.0;
        while (banked < wanted && nowSeconds() < captureDeadline) {
            cv::Mat frame;
            long long newSeq = 0;
            if (!stream.readNew(seq, frame, newSeq, 6.0)) break;
            seq = newSeq;
            cv::Mat f;
            frame.convertTo(f, CV_32FC3);
            if (acc.empty()) {
                acc = f;
            } else {
                acc += f;
            }
            ++banked;
            if (progress) progress(banked, wanted);
        }
    }

    if (acc.empty()) {
        throw std::runtime_error("no frames received while capturing the background");
    }
    if (banked < wanted) {
        std::printf("  averaged %d frames (wanted %d)\n", banked, wanted);
        std::fflush(stdout);
    }

    cv::Mat scaled;
    acc.convertTo(scaled, CV_32FC3, 1.0 / banked);
    return toU8Truncate(scaled);
}

cv::Mat captureBackground(FrameSource& stream, const Config& cfg) {
    const int n = cfg.background_average_frames;
    std::printf("Capturing background reference over up to %d frames...\n", n);
    std::fflush(stdout);

    cv::Mat acc;
    int got = 0;
    long long seq = -1;
    const double deadline = nowSeconds() + 25.0;

    while (got < n && nowSeconds() < deadline) {
        cv::Mat frame;
        long long newSeq = 0;
        if (!stream.readNew(seq, frame, newSeq, 6.0)) break;
        seq = newSeq;

        cv::Mat f;
        frame.convertTo(f, CV_32FC3);
        if (acc.empty()) {
            acc = f;
        } else {
            acc += f;
        }
        ++got;
    }

    if (acc.empty()) {
        throw std::runtime_error("no frames received while capturing the background");
    }
    if (got < n) {
        std::printf("  averaged %d frames (wanted %d)\n", got, n);
        std::fflush(stdout);
    }

    cv::Mat scaled;
    acc.convertTo(scaled, CV_32FC3, 1.0 / got);
    return toU8Truncate(scaled);
}

}  // namespace verimeasure
