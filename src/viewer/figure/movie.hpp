#pragma once

// Movies of the frames the timeline shows: a camera path through keyframes,
// the video's timing, and H.264 encoding through an ffmpeg executable (as
// figure export uses a LaTeX one; nothing is built in).

#include "interaction/camera_controller.hpp"
#include "figure/figure_writers.hpp"  // ShellQuote
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

namespace RayFEM::UI::Movie {

using Figure::ReadText;
using Figure::ShellQuote;

// A camera pose at a point in the movie, measured in shown frames: 0 is the
// first frame of the selection and 2.5 is halfway between the third and
// fourth.
struct Keyframe {
    double position = 0;
    Vector3 target{};
    float distance = 1;
    Quaternion orientation{0, 0, 0, 1};  // the camera basis (CameraBasisQuaternion)
    float fovy = 40;
};

inline Keyframe KeyframeFrom(const Camera3D& camera, double position) {
    return {position, camera.target, Vector3Distance(camera.position, camera.target),
            CameraBasisQuaternion(CameraBasis(camera)), camera.fovy};
}

inline Camera3D CameraFrom(const Keyframe& key, int projection = CAMERA_PERSPECTIVE) {
    const Vector3 back = Vector3RotateByQuaternion({0, 0, 1}, key.orientation);
    const Vector3 up = Vector3RotateByQuaternion({0, 1, 0}, key.orientation);
    return {Vector3Add(key.target, Vector3Scale(back, key.distance)), key.target, up, key.fovy, projection};
}

enum class Easing { EaseInOut, Linear };

// The camera at position along the keyframes (sorted by position). Before the
// first and after the last keyframe the camera holds still. Orientation takes
// the shortest rotation; distance changes geometrically, so zooming reads as
// steady; EaseInOut slows to a stop at every keyframe.
inline std::optional<Camera3D> CameraAt(const std::vector<Keyframe>& keys, double position, Easing easing,
                                        int projection = CAMERA_PERSPECTIVE) {
    if (keys.empty()) return std::nullopt;
    if (position <= keys.front().position) return CameraFrom(keys.front(), projection);
    if (position >= keys.back().position) return CameraFrom(keys.back(), projection);
    const auto next = std::upper_bound(keys.begin(), keys.end(), position,
                                       [](double p, const Keyframe& k) { return p < k.position; });
    const Keyframe& a = *std::prev(next);
    const Keyframe& b = *next;
    const double span = b.position - a.position;
    double s = span > 0 ? (position - a.position) / span : 1;
    if (easing == Easing::EaseInOut) s = CameraViewEase(s);
    const float t = static_cast<float>(s);
    Quaternion to = b.orientation;
    if (a.orientation.x * to.x + a.orientation.y * to.y + a.orientation.z * to.z + a.orientation.w * to.w < 0)
        to = {-to.x, -to.y, -to.z, -to.w};
    Keyframe mixed;
    mixed.orientation = QuaternionNormalize(QuaternionSlerp(a.orientation, to, t));
    mixed.target = Vector3Lerp(a.target, b.target, t);
    mixed.distance = std::exp(std::lerp(std::log(std::max(a.distance, 1e-12f)), std::log(std::max(b.distance, 1e-12f)), t));
    mixed.fovy = std::lerp(a.fovy, b.fovy, t);
    return CameraFrom(mixed, projection);
}

inline void SortKeyframes(std::vector<Keyframe>& keys) {
    std::stable_sort(keys.begin(), keys.end(), [](const Keyframe& a, const Keyframe& b) { return a.position < b.position; });
}

// Video timing: each shown frame is held for seconds_per_frame.
struct Timing {
    int frames = 1;              // shown frames in the movie
    double fps = 30;             // video frames per second
    double seconds_per_frame = .2;
    int VideoFrames() const {
        return std::max(1, static_cast<int>(std::lround(std::max(1, frames) * seconds_per_frame * fps)));
    }
    double Seconds() const { return VideoFrames() / fps; }
    // Where video frame n falls, in shown frames (for the camera), and the
    // shown frame it displays.
    double Position(int n) const {
        const double perFrame = std::max(1e-9, seconds_per_frame * fps);
        return std::min(static_cast<double>(frames - 1), n / perFrame);
    }
    int Frame(int n) const {
        const double perFrame = std::max(1e-9, seconds_per_frame * fps);
        return std::clamp(static_cast<int>(std::floor(n / perFrame + 1e-9)), 0, std::max(0, frames - 1));
    }
};

// H.264 through ffmpeg: libx264 when the build has it, else the platform's
// hardware encoder, else whatever ffmpeg calls h264.
struct Encoder {
    std::string ffmpeg;   // empty: not found
    std::string codec;    // libx264, h264_videotoolbox, ...
    bool Available() const { return !ffmpeg.empty() && !codec.empty(); }

    static Encoder Discover() {
        Encoder found;
        std::error_code ignored;
        auto executable = [&](const std::filesystem::path& candidate) {
            const auto permissions = std::filesystem::status(candidate, ignored).permissions();
            return std::filesystem::is_regular_file(candidate, ignored) &&
                   (permissions & std::filesystem::perms::owner_exec) != std::filesystem::perms::none;
        };
        // RAYFEM_FFMPEG names one; otherwise PATH, then the usual places,
        // since a viewer started from the Finder or a desktop launcher often
        // lacks them on its PATH.
        std::vector<std::filesystem::path> candidates;
        if (const char* chosen = std::getenv("RAYFEM_FFMPEG"); chosen && *chosen) candidates.push_back(chosen);
        if (const char* path = std::getenv("PATH")) {
            std::stringstream list(path);
            for (std::string entry; std::getline(list, entry, ':');) if (!entry.empty()) candidates.push_back(std::filesystem::path(entry) / "ffmpeg");
        }
        for (const char* d : {"/opt/homebrew/bin", "/usr/local/bin", "/opt/local/bin", "/usr/bin", "/snap/bin"})
            candidates.push_back(std::filesystem::path(d) / "ffmpeg");
        for (const auto& candidate : candidates)
            if (executable(candidate)) { found.ffmpeg = candidate.string(); break; }
        if (found.ffmpeg.empty()) return found;
        std::string encoders;
        if (FILE* list = popen((ShellQuote(found.ffmpeg) + " -hide_banner -encoders 2>/dev/null").c_str(), "r")) {
            char buffer[4096];
            for (size_t n; (n = std::fread(buffer, 1, sizeof(buffer), list)) > 0;) encoders.append(buffer, n);
            pclose(list);
        }
        for (const char* codec : {"libx264", "h264_videotoolbox", "h264_nvenc", "h264_vaapi", "libopenh264"})
            if (encoders.find(std::string(" ") + codec + " ") != std::string::npos) { found.codec = codec; break; }
        return found;
    }

    // The command that reads raw RGBA frames on standard input. quality runs
    // from 0 (largest) to 51 (smallest) as libx264's CRF; 18 looks lossless.
    std::string Command(const std::string& output, int width, int height, double fps, int quality,
                        const std::string& log) const {
        std::ostringstream command;
        command << ShellQuote(ffmpeg) << " -hide_banner -loglevel error -y -f rawvideo -pixel_format rgba"
                << " -video_size " << width << "x" << height << " -framerate " << fps << " -i -"
                << " -c:v " << codec;
        if (codec == "libx264") command << " -preset medium -crf " << quality;
        else if (codec == "h264_videotoolbox") command << " -q:v " << std::clamp(100 - quality * 2, 1, 100);
        else command << " -b:v " << std::max(1, (51 - quality) / 4) << "M";
        // yuv420p plays everywhere; it needs even dimensions.
        command << " -pix_fmt yuv420p -movflags +faststart " << ShellQuote(output) << " 2>" << ShellQuote(log);
        return command.str();
    }
};

// Even sizes, as yuv420p requires.
inline int EvenSize(int value) { return std::max(2, value - value % 2); }

// An ffmpeg process fed one RGBA frame at a time.
class Writer {
public:
    Writer() = default;
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;
    ~Writer() { Abort(); }

    bool Open(const Encoder& encoder, const std::string& output, int width, int height, double fps, int quality,
              std::string& error) {
        Abort();
        width_ = width; height_ = height;
        std::error_code ignored;
        if (const auto parent = std::filesystem::path(output).parent_path(); !parent.empty())
            std::filesystem::create_directories(parent, ignored);
        log_ = (std::filesystem::temp_directory_path(ignored) / ("rayfem-movie-" + std::to_string(::getpid()) + ".log")).string();
        // A closed pipe must fail the write, not end the viewer.
        previous_ = std::signal(SIGPIPE, SIG_IGN);
        pipe_ = popen(encoder.Command(output, width, height, fps, quality, log_).c_str(), "w");
        if (!pipe_) { error = "Could not start ffmpeg"; Restore(); return false; }
        return true;
    }
    // rgba holds width*height*4 bytes, top row first.
    bool Write(const std::vector<unsigned char>& rgba, std::string& error) {
        if (!pipe_) { error = "The encoder is not running"; return false; }
        if (rgba.size() != static_cast<size_t>(width_) * height_ * 4) { error = "Frame size mismatch"; return false; }
        if (std::fwrite(rgba.data(), 1, rgba.size(), pipe_) != rgba.size()) {
            error = "ffmpeg stopped: " + Log();
            Abort();
            return false;
        }
        return true;
    }
    // Finish the file; false (with ffmpeg's message) if it failed.
    bool Close(std::string& error) {
        if (!pipe_) return false;
        const int status = pclose(pipe_);
        pipe_ = nullptr;
        Restore();
        if (status != 0) { error = "ffmpeg failed: " + Log(); return false; }
        std::error_code ignored;
        std::filesystem::remove(log_, ignored);
        return true;
    }
    void Abort() {
        if (pipe_) { pclose(pipe_); pipe_ = nullptr; }
        Restore();
    }
    bool Running() const { return pipe_ != nullptr; }

private:
    std::string Log() const {
        std::string text = ReadText(log_);
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
        return text.empty() ? "no message" : text;
    }
    void Restore() {
        if (previous_ != SIG_ERR) { std::signal(SIGPIPE, previous_); previous_ = SIG_ERR; }
    }
    FILE* pipe_ = nullptr;
    int width_ = 0, height_ = 0;
    std::string log_;
    void (*previous_)(int) = SIG_ERR;
};

} // namespace RayFEM::UI::Movie
