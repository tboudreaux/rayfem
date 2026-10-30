#include "figure/movie.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace RayFEM::UI::Movie;

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void Near(Vector3 a, Vector3 b, const std::string& message, float tolerance = 1e-4f) {
    Check(Vector3Distance(a, b) < tolerance, message + " (" + std::to_string(a.x) + ", " + std::to_string(a.y) + ", " +
          std::to_string(a.z) + ")");
}

void CameraPath() {
    Check(!CameraAt({}, 3, Easing::Linear), "No keyframes: no path");
    // Front view at frame 0, from +X at frame 10 twice as far, target moved.
    Camera3D front{{0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    Camera3D side{{18, 1, 0}, {2, 1, 0}, {0, 1, 0}, 30, CAMERA_PERSPECTIVE};
    std::vector<Keyframe> keys{KeyframeFrom(side, 10), KeyframeFrom(front, 0)};
    SortKeyframes(keys);
    Check(keys.front().time == 0 && keys.back().time == 10, "Keyframes sort by time");
    Near(CameraAt(keys, -5, Easing::Linear)->position, front.position, "The first keyframe holds before it");
    Near(CameraAt(keys, 20, Easing::Linear)->position, side.position, "The last keyframe holds after it");
    const Camera3D middle = *CameraAt(keys, 5, Easing::Linear);
    Near(middle.target, {1, .5f, 0}, "The target moves linearly");
    const float distance = Vector3Distance(middle.position, middle.target);
    Check(std::abs(distance - 8) < 1e-3f, "Distance changes geometrically: sqrt(4*16) = 8, got " + std::to_string(distance));
    const Vector3 back = Vector3Normalize(Vector3Subtract(middle.position, middle.target));
    Near(back, Vector3Normalize({1, 0, 1}), "Halfway between front and side views looks from 45°");
    Check(std::abs(middle.fovy - 35) < 1e-4f, "Field of view interpolates");
    // Easing lags at the start and leads at the end, matching at the middle.
    const Vector3 early = CameraAt(keys, 1, Easing::EaseInOut)->target, earlyLinear = CameraAt(keys, 1, Easing::Linear)->target;
    Check(early.x < earlyLinear.x, "Easing starts slowly");
    Near(CameraAt(keys, 5, Easing::EaseInOut)->target, middle.target, "Easing is symmetric");
    // Opposite-signed quaternions describe the same view: no detour.
    std::vector<Keyframe> flipped = keys;
    flipped.back().orientation = {-flipped.back().orientation.x, -flipped.back().orientation.y,
                                  -flipped.back().orientation.z, -flipped.back().orientation.w};
    Near(Vector3Normalize(Vector3Subtract(CameraAt(flipped, 5, Easing::Linear)->position, middle.target)), back,
         "Interpolation takes the shorter rotation");
}

void SceneKeyframes() {
    // A cut that moves and turns, then switches off; annotations appear.
    SceneState a;
    a.camera = {{0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    a.clip = true; a.clip_point = {0, 0, 0}; a.clip_normal = {1, 0, 0}; a.annotations = {1};
    SceneState b = a;
    b.clip_point = {2, 0, 0}; b.clip_normal = {0, 1, 0}; b.annotations = {1, 2};
    SceneState c = b;
    c.clip = false;
    std::vector<Keyframe> keys{KeyframeFrom(a, 0), KeyframeFrom(b, 2), KeyframeFrom(c, 4)};
    const auto middle = *StateAt(keys, 1, Easing::Linear);
    Check(middle.clip, "The cut stays on between keyframes that both cut");
    Near(middle.clip_point, {1, 0, 0}, "The cut plane moves");
    Near(middle.clip_normal, Vector3Normalize({1, 1, 0}), "The cut plane turns");
    Check(middle.annotations == std::vector<uint32_t>{1}, "Annotations switch at the next keyframe");
    const auto later = *StateAt(keys, 3, Easing::Linear);
    Check(later.clip && later.annotations.size() == 2, "Switches hold until the next keyframe");
    Near(later.clip_point, {2, 0, 0}, "A cut switching off does not move");
    Check(!StateAt(keys, 4, Easing::Linear)->clip && !StateAt(keys, 9, Easing::Linear)->clip, "The cut is off from its keyframe on");
    Check(StateAt(keys, -1, Easing::Linear)->annotations == std::vector<uint32_t>{1}, "Before the first keyframe its scene holds");
}

void MovieTiming() {
    const Timing timing{5, 30, .2};
    Check(timing.VideoFrames() == 30, "5 frames held 0.2 s at 30 fps: 30 video frames");
    Check(timing.Frame(0) == 0 && timing.Frame(5) == 0 && timing.Frame(6) == 1 && timing.Frame(29) == 4,
          "Each frame is held for 6 video frames");
    Check(std::abs(timing.Position(3) - .5) < 1e-12 && timing.Position(29) <= 4, "Camera positions are continuous");
    Check(Timing{1, 24, .01}.VideoFrames() == 1, "At least one video frame");
    // A total length: frames share it evenly; one frame fills it.
    const Timing total{4, 30, .2, 2};
    Check(total.VideoFrames() == 60 && total.Frame(14) == 0 && total.Frame(15) == 1 && total.Frame(59) == 3,
          "Four frames share two seconds");
    const Timing orbit{1, 30, .2, 10};
    Check(orbit.VideoFrames() == 300 && orbit.Frame(299) == 0 && orbit.Position(150) == 0, "One frame fills a ten-second orbit");
    Check(EvenSize(1081) == 1080 && EvenSize(1) == 2, "Sizes round down to even");
}

void Encode() {
    const auto encoder = Encoder::Discover();
    if (!encoder.Available()) {
        std::cout << "ffmpeg with H.264 not found; skipping the encoding check\n";
        return;
    }
    const auto command = encoder.Command("out dir/movie.mp4", 64, 48, 24, 20, "/tmp/log");
    Check(command.find("-video_size 64x48") != std::string::npos && command.find("'out dir/movie.mp4'") != std::string::npos &&
          command.find("-c:v " + encoder.codec) != std::string::npos && command.find("yuv420p") != std::string::npos,
          "Command: " + command);
    const auto directory = std::filesystem::temp_directory_path() / ("rayfem movie test " + std::to_string(::getpid()));
    std::filesystem::remove_all(directory);
    const auto path = (directory / "nested" / "clip.mp4").string();
    Writer writer;
    std::string error;
    Check(writer.Open(encoder, path, 64, 48, 24, 20, error), error);
    std::vector<unsigned char> rgba(64 * 48 * 4);
    for (int frame = 0; frame < 12; ++frame) {
        for (int y = 0; y < 48; ++y)
            for (int x = 0; x < 64; ++x) {
                unsigned char* p = &rgba[(y * 64 + x) * 4];
                p[0] = static_cast<unsigned char>(x * 4 + frame * 10); p[1] = static_cast<unsigned char>(y * 5); p[2] = 128; p[3] = 255;
            }
        Check(writer.Write(rgba, error), error);
    }
    Check(!writer.Write(std::vector<unsigned char>(10), error) && error == "Frame size mismatch", "Wrong sizes are refused");
    Check(writer.Close(error), error);
    Check(std::filesystem::file_size(path) > 500, "The movie has content");
    const std::string probe = encoder.ffmpeg.substr(0, encoder.ffmpeg.size() - 6) + "ffprobe";
    if (std::filesystem::exists(probe)) {
        std::string out;
        if (FILE* p = popen((ShellQuote(probe) + " -v error -select_streams v:0 -count_frames -show_entries "
                             "stream=codec_name,width,height,nb_read_frames -of csv=p=0 " + ShellQuote(path)).c_str(), "r")) {
            char buffer[256];
            for (size_t n; (n = std::fread(buffer, 1, sizeof(buffer), p)) > 0;) out.append(buffer, n);
            pclose(p);
        }
        Check(out.starts_with("h264,64,48,12"), "ffprobe: " + out);
    }
    // A file that cannot be written reports ffmpeg's message.
    Writer bad;
    Check(bad.Open(encoder, "/nonexistent-root-dir/x/clip.mp4", 64, 48, 24, 20, error) || !error.empty(), "Open");
    bool wrote = true;
    for (int i = 0; i < 3 && wrote; ++i) wrote = bad.Write(rgba, error);
    Check(!wrote || !bad.Close(error), "An unwritable file fails");
    Check(!error.empty(), "The failure has a message");
    std::filesystem::remove_all(directory);
    std::cout << "encoded with " << encoder.codec << '\n';
}
} // namespace

int main() {
    try {
        CameraPath();
        SceneKeyframes();
        MovieTiming();
        Encode();
        std::cout << "movie passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "movie test failed: " << e.what() << '\n';
        return 1;
    }
}
