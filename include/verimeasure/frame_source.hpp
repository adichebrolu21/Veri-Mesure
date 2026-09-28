// Common interface for anything that hands out frames: the live camera stream
// and the synthetic bench both satisfy it, which is what lets the station and
// the runner treat them identically.

#pragma once

#include <opencv2/core.hpp>
#include <string>

namespace verimeasure {

class FrameSource {
public:
    virtual ~FrameSource() = default;

    // Newest frame available. Returns false on timeout. age is seconds since the
    // frame was captured.
    virtual bool read(cv::Mat& frame, long long& seq, double& age, double timeout = 10.0) = 0;

    // Block until a frame newer than afterSeq arrives. Returns false on timeout.
    virtual bool readNew(long long afterSeq, cv::Mat& frame, long long& seq,
                         double timeout = 10.0) = 0;

    virtual double fps() = 0;
    virtual void stop() {}

    virtual long long incompleteCount() const { return 0; }
    virtual long long restartCount() const { return 0; }
    virtual long long grabbedCount() const { return 0; }
    virtual std::string error() const { return std::string(); }
};

}  // namespace verimeasure
