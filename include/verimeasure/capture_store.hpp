// Pictures of the verdicts, written to disk off the measuring thread.
//
// The station hands over a picture it already holds and a description of what
// to draw on it, and gets back at once. One thread of this store's own draws,
// encodes and writes, in the order they came. Nothing here reaches a
// measurement: it is only ever given copies, and it only ever writes files.
//
// The queue is bounded. A writer that has fallen behind - a disk that has
// stopped answering, a burst of verdicts on a slow host - costs pictures, the
// oldest first, and says so. It never costs the measuring thread a wait.
//
// Header only, so a check can build it on its own with no camera and no
// station.
#pragma once

#include <algorithm>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace verimeasure {

// The three folders a picture is filed under, by what its step was judged.
inline const std::vector<std::string>& captureKinds() {
    static const std::vector<std::string> kinds{"PASS", "FAIL", "MANUAL"};
    return kinds;
}

// "YYYY-MM-DD" and nothing else: the only folder name retention looks inside.
inline bool isCaptureDay(const std::string& name) {
    if (name.size() != 10 || name[4] != '-' || name[7] != '-') return false;
    for (std::size_t k = 0; k < name.size(); ++k) {
        if (k == 4 || k == 7) continue;
        if (name[k] < '0' || name[k] > '9') return false;
    }
    return true;
}

// A calendar day `days` before `day` (both "YYYY-MM-DD"), or "" if `day` is
// not one. Worked at noon, so a change of clocks never moves it a day.
inline std::string captureDayBefore(const std::string& day, int days) {
    if (!isCaptureDay(day)) return std::string();
    std::tm t{};
    t.tm_year = std::stoi(day.substr(0, 4)) - 1900;
    t.tm_mon = std::stoi(day.substr(5, 2)) - 1;
    t.tm_mday = std::stoi(day.substr(8, 2)) - days;
    t.tm_hour = 12;
    t.tm_isdst = -1;
    if (std::mktime(&t) == static_cast<std::time_t>(-1)) return std::string();
    char buf[16];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &t);
    return buf;
}

// What a pass of retention took away.
struct CapturePrune {
    int files = 0;
    unsigned long long bytes = 0;
    unsigned long long kept_bytes = 0;
    std::vector<std::string> days;  // "PASS/2026-09-01", each folder emptied
    std::vector<std::string> notes;  // anything that could not be removed
};

// Keep what the limits allow, oldest day first.
//
// Only ever *.jpg files that sit directly in <root>/PASS|FAIL|MANUAL/YYYY-MM-DD,
// and then that folder if it is left empty. Nothing is followed through a
// link, nothing is removed recursively, and anything else in those folders,
// or anywhere else, is not touched.
//
// keepDays: the most recent keepDays calendar days are kept, `today`
// included; 0 is no limit. maxGb: the pictures together, in gigabytes of
// 1e9 bytes; 0 is no limit. Whole days go, oldest first, until both hold.
// When only the newest day is left and it alone is over the size, its oldest
// pictures go one by one.
inline CapturePrune pruneCaptures(const std::string& rootDir, int keepDays, double maxGb,
                                  const std::string& today) {
    namespace fs = std::filesystem;
    CapturePrune out;
    std::error_code ec;
    const fs::path root(rootDir);

    struct Picture {
        std::string name;
        fs::path path;
        unsigned long long bytes = 0;
    };
    struct Folder {
        std::string kind;
        fs::path path;
        std::vector<Picture> pictures;
    };
    // By day, oldest first: ISO dates sort as they fall.
    std::map<std::string, std::vector<Folder>> byDay;

    for (const std::string& kind : captureKinds()) {
        const fs::path dir = root / kind;
        const fs::file_status st = fs::symlink_status(dir, ec);
        if (ec || !fs::is_directory(st)) continue;
        for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec)) {
            const fs::path day = it->path();
            const std::string name = day.filename().string();
            if (!isCaptureDay(name)) continue;
            std::error_code e2;
            if (!fs::is_directory(fs::symlink_status(day, e2)) || e2) continue;
            Folder f;
            f.kind = kind;
            f.path = day;
            for (fs::directory_iterator jt(day, e2), jend; !e2 && jt != jend; jt.increment(e2)) {
                std::error_code e3;
                const fs::path p = jt->path();
                if (!fs::is_regular_file(fs::symlink_status(p, e3)) || e3) continue;
                std::string ext = p.extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (ext != ".jpg") continue;
                Picture pic;
                pic.name = p.filename().string();
                pic.path = p;
                pic.bytes = static_cast<unsigned long long>(fs::file_size(p, e3));
                if (e3) pic.bytes = 0;
                f.pictures.push_back(pic);
            }
            byDay[name].push_back(f);
        }
    }

    const auto removePicture = [&out](const Picture& pic) {
        std::error_code e;
        if (fs::remove(pic.path, e) && !e) {
            ++out.files;
            out.bytes += pic.bytes;
            return true;
        }
        out.notes.push_back("could not remove " + pic.path.string() + (e ? ": " + e.message() : ""));
        return false;
    };
    // Its pictures, then the folder itself if that left it empty. fs::remove
    // of a directory refuses one with anything still in it.
    const auto removeDay = [&](const std::string& day) {
        for (const Folder& f : byDay[day]) {
            for (const Picture& pic : f.pictures) removePicture(pic);
            std::error_code e;
            fs::remove(f.path, e);
            out.days.push_back(f.kind + "/" + day);
        }
        byDay.erase(day);
    };

    // By age.
    if (keepDays > 0) {
        const std::string oldestKept = captureDayBefore(today, keepDays - 1);
        if (!oldestKept.empty()) {
            std::vector<std::string> old;
            for (const auto& d : byDay) {
                if (d.first < oldestKept) old.push_back(d.first);
            }
            for (const std::string& d : old) removeDay(d);
        }
    }

    // By size.
    unsigned long long total = 0;
    for (const auto& d : byDay) {
        for (const Folder& f : d.second) {
            for (const Picture& p : f.pictures) total += p.bytes;
        }
    }
    if (maxGb > 0.0) {
        const unsigned long long limit = static_cast<unsigned long long>(maxGb * 1e9);
        while (total > limit && byDay.size() > 1) {
            const std::string day = byDay.begin()->first;
            unsigned long long dayBytes = 0;
            for (const Folder& f : byDay.begin()->second) {
                for (const Picture& p : f.pictures) dayBytes += p.bytes;
            }
            removeDay(day);
            total -= std::min(total, dayBytes);
        }
        if (total > limit && byDay.size() == 1) {
            // The newest day alone is over: its oldest pictures, by the time
            // at the front of the name, across PASS, FAIL and MANUAL.
            std::vector<Picture> all;
            for (const Folder& f : byDay.begin()->second) {
                all.insert(all.end(), f.pictures.begin(), f.pictures.end());
            }
            std::sort(all.begin(), all.end(),
                      [](const Picture& a, const Picture& b) { return a.name < b.name; });
            for (const Picture& p : all) {
                if (total <= limit) break;
                if (removePicture(p)) total -= std::min(total, p.bytes);
            }
        }
    }
    out.kept_bytes = total;
    return out;
}

class CaptureStore {
public:
    struct Settings {
        std::string root = "captures";
        int jpeg_quality = 85;
        int queue = 8;
        int keep_days = 0;
        double max_gb = 0.0;
        int prune_every = 50;
    };
    enum class State { None, Saving, Saved, Failed, Dropped };

    // The picture as it is to be written, drawn on the writer's thread.
    using Render = std::function<cv::Mat()>;
    // `operatorToo` is for what the bench should see (a picture dropped, one
    // that could not be written, what retention removed); the rest is timing,
    // for the console.
    using Log = std::function<void(const std::string& text, bool operatorToo)>;

    CaptureStore(const Settings& s, Log log) : s_(s), log_(std::move(log)) {
        s_.queue = std::max(1, s_.queue);
        s_.jpeg_quality = std::min(100, std::max(1, s_.jpeg_quality));
        s_.prune_every = std::max(1, s_.prune_every);
    }
    ~CaptureStore() { stop(); }
    CaptureStore(const CaptureStore&) = delete;
    CaptureStore& operator=(const CaptureStore&) = delete;

    void start() {
        std::lock_guard<std::mutex> g(m_);
        if (thread_.joinable() || stopping_) return;
        thread_ = std::thread(&CaptureStore::run, this);
    }

    // Everything already handed over is written first.
    void stop() {
        {
            std::lock_guard<std::mutex> g(m_);
            stopping_ = true;
        }
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    const std::string& root() const { return s_.root; }

    // A name for a new picture, relative to the root:
    // KIND/YYYY-MM-DD/stem.jpg, with _2, _3 ... when this run has already
    // given the name out - a retake inside the same second. Never one that
    // would overwrite; the writer checks the disk again before it writes.
    std::string plan(const std::string& kind, const std::string& day, const std::string& stem) {
        std::lock_guard<std::mutex> g(m_);
        if (day != issuedDay_) {
            issued_.clear();
            issuedDay_ = day;
        }
        std::string rel = kind + "/" + day + "/" + stem + ".jpg";
        for (int n = 2; issued_.count(rel); ++n) {
            rel = kind + "/" + day + "/" + stem + "_" + std::to_string(n) + ".jpg";
        }
        issued_.insert(rel);
        return rel;
    }

    // Handed over from the measuring thread. `startedAt` is when that thread
    // began on it (steadySeconds), so the time it spent is on the log line.
    // Returns the picture's id.
    unsigned long long submit(const std::string& rel, Render render, double startedAt) {
        std::string dropped;
        unsigned long long id = 0;
        {
            std::lock_guard<std::mutex> g(m_);
            id = ++lastId_;
            if (queue_.size() >= static_cast<std::size_t>(s_.queue)) {
                Job& old = queue_.front();
                setStateLocked(old.id, State::Dropped, old.rel);
                dropped = old.rel;
                queue_.pop_front();
                ++dropped_;
            }
            Job j;
            j.id = id;
            j.rel = rel;
            j.render = std::move(render);
            setStateLocked(id, State::Saving, rel);
            j.enqueueMs = (steadySeconds() - startedAt) * 1000.0;
            queue_.push_back(std::move(j));
        }
        cv_.notify_one();
        if (!dropped.empty()) {
            say("Picture not saved, the writer is " + std::to_string(s_.queue) +
                    " behind: " + dropped + " dropped to keep up",
                true);
        }
        return id;
    }

    State state(unsigned long long id, std::string* rel = nullptr) const {
        std::lock_guard<std::mutex> g(m_);
        const auto it = states_.find(id);
        if (it == states_.end()) return State::None;
        if (rel) *rel = it->second.second;
        return it->second.first;
    }

    std::size_t pending() const {
        std::lock_guard<std::mutex> g(m_);
        return queue_.size() + (busy_ ? 1 : 0);
    }

    unsigned long long droppedCount() const {
        std::lock_guard<std::mutex> g(m_);
        return dropped_;
    }

    // One pass of retention now, on the calling thread. The writer runs it
    // itself; this is for a check.
    CapturePrune pruneNow() { return prune(); }

    static double steadySeconds() {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }

    static std::string today() {
        const std::time_t now = std::time(nullptr);
        std::tm t{};
#ifdef _WIN32
        localtime_s(&t, &now);
#else
        localtime_r(&now, &t);
#endif
        char buf[16];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d", &t);
        return buf;
    }

private:
    struct Job {
        unsigned long long id = 0;
        std::string rel;
        Render render;
        double enqueueMs = 0.0;
    };

    static std::string ms(double v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f", v);
        return buf;
    }

    void say(const std::string& text, bool operatorToo) {
        if (log_) log_(text, operatorToo);
    }

    // Call with m_ held. Only the last few thousand are remembered; the
    // station asks only about the shirt on the bench.
    void setStateLocked(unsigned long long id, State st, const std::string& rel) {
        states_[id] = {st, rel};
        while (states_.size() > 4096) states_.erase(states_.begin());
    }

    bool limited() const { return s_.keep_days > 0 || s_.max_gb > 0.0; }

    CapturePrune prune() {
        const CapturePrune p = pruneCaptures(s_.root, s_.keep_days, s_.max_gb, today());
        lastPrune_ = steadySeconds();
        sincePrune_ = 0;
        if (p.files > 0 || !p.days.empty()) {
            // The days, not every folder: "from 12 days, 2026-09-06 to 2026-09-17".
            std::set<std::string> dates;
            for (const std::string& d : p.days) dates.insert(d.substr(d.find('/') + 1));
            std::string days;
            if (dates.size() == 1) {
                days = " from " + *dates.begin();
            } else if (!dates.empty()) {
                days = " from " + std::to_string(dates.size()) + " days, " + *dates.begin() + " to " +
                       *dates.rbegin();
            }
            say("Pictures: removed " + std::to_string(p.files) + " (" + ms(p.bytes / 1e6) + " MB)" + days +
                    " to keep " +
                    (s_.keep_days > 0 ? std::to_string(s_.keep_days) + " days" : std::string()) +
                    (s_.keep_days > 0 && s_.max_gb > 0.0 ? " and " : "") +
                    (s_.max_gb > 0.0 ? ms(s_.max_gb) + " GB" : std::string()) + "; " +
                    ms(p.kept_bytes / 1e6) + " MB kept",
                true);
        }
        for (const std::string& n : p.notes) say("Pictures: " + n, true);
        return p;
    }

    // Whether a file of this name is already on disk, so a picture from an
    // earlier run inside the same second is not written over.
    static std::string freeName(const std::filesystem::path& root, const std::string& rel) {
        std::error_code ec;
        if (!std::filesystem::exists(root / rel, ec)) return rel;
        const std::string stem = rel.substr(0, rel.size() - 4);
        for (int n = 2; n < 1000; ++n) {
            const std::string alt = stem + "_" + std::to_string(n) + ".jpg";
            if (!std::filesystem::exists(root / alt, ec)) return alt;
        }
        return std::string();
    }

    void writeOne(Job& job) {
        namespace fs = std::filesystem;
        const double t0 = steadySeconds();
        cv::Mat img;
        std::string error;
        try {
            img = job.render();
        } catch (const std::exception& e) {
            error = e.what();
        }
        const double t1 = steadySeconds();
        std::vector<unsigned char> jpeg;
        if (error.empty()) {
            try {
                if (img.empty() ||
                    !cv::imencode(".jpg", img, jpeg, {cv::IMWRITE_JPEG_QUALITY, s_.jpeg_quality})) {
                    error = "could not encode the picture";
                }
            } catch (const std::exception& e) {
                error = e.what();
            }
        }
        const double t2 = steadySeconds();
        const fs::path root(s_.root);
        std::string rel = job.rel;
        if (error.empty()) {
            std::error_code ec;
            fs::create_directories((root / rel).parent_path(), ec);
            const std::string free = freeName(root, rel);
            if (free.empty()) {
                error = "no free name beside " + rel;
            } else {
                if (free != rel) {
                    say("Picture " + rel + " was already on disk; written as " + free, true);
                    rel = free;
                }
                std::FILE* f = nullptr;
#ifdef _WIN32
                if (_wfopen_s(&f, (root / rel).wstring().c_str(), L"wb") != 0) f = nullptr;
#else
                f = std::fopen((root / rel).string().c_str(), "wb");
#endif
                if (!f) {
                    error = "could not open " + (root / rel).string();
                } else {
                    const bool ok = std::fwrite(jpeg.data(), 1, jpeg.size(), f) == jpeg.size();
                    const bool closed = std::fclose(f) == 0;
                    if (!ok || !closed) {
                        error = "could not write " + (root / rel).string();
                        fs::remove(root / rel, ec);
                    }
                }
            }
        }
        const double t3 = steadySeconds();
        {
            std::lock_guard<std::mutex> g(m_);
            setStateLocked(job.id, error.empty() ? State::Saved : State::Failed, rel);
        }
        if (!error.empty()) {
            say("Picture not saved (" + rel + "): " + error, true);
            return;
        }
        say("Picture saved " + rel + ": measuring thread " + ms(job.enqueueMs) + " ms; writer " +
                ms((t3 - t0) * 1000.0) + " ms (draw " + ms((t1 - t0) * 1000.0) + ", encode " +
                ms((t2 - t1) * 1000.0) + ", write " + ms((t3 - t2) * 1000.0) + "), " +
                std::to_string(jpeg.size() / 1024) + " KB",
            false);
    }

    void run() {
        if (limited()) prune();
        lastPrune_ = steadySeconds();
        while (true) {
            Job job;
            {
                std::unique_lock<std::mutex> g(m_);
                cv_.wait_for(g, std::chrono::seconds(60), [this] { return stopping_ || !queue_.empty(); });
                if (queue_.empty()) {
                    if (stopping_) break;
                    g.unlock();
                    if (limited() && steadySeconds() - lastPrune_ >= 3600.0) prune();
                    continue;
                }
                job = std::move(queue_.front());
                queue_.pop_front();
                busy_ = true;
            }
            writeOne(job);
            {
                std::lock_guard<std::mutex> g(m_);
                busy_ = false;
            }
            ++sincePrune_;
            if (limited() && (sincePrune_ >= s_.prune_every || steadySeconds() - lastPrune_ >= 3600.0)) {
                prune();
            }
        }
    }

    Settings s_;
    Log log_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<Job> queue_;
    std::map<unsigned long long, std::pair<State, std::string>> states_;
    std::set<std::string> issued_;
    std::string issuedDay_;
    unsigned long long lastId_ = 0;
    unsigned long long dropped_ = 0;
    bool busy_ = false;
    bool stopping_ = false;
    std::thread thread_;
    // The writer's own.
    double lastPrune_ = 0.0;
    int sincePrune_ = 0;
};

}  // namespace verimeasure
