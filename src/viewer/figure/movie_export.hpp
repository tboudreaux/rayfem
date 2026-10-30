#pragma once

// The Movie window: the frames the timeline shows, camera keyframes placed
// from the live view, a preview, and H.264 export through ffmpeg. It is a
// floating window, not a modal dialog, so the view stays usable for placing
// keyframes.

#include "figure/movie.hpp"
#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "extras/IconsFontAwesome6.h"
#include <functional>
#include <memory>
#include <optional>
#include <tuple>

namespace RayFEM::UI::Movie {

// What the viewer provides. Positions count the frames the timeline shows,
// from 0; during an export they count the frames captured when it started.
struct Host {
    std::function<int()> frames;                      // frames the timeline shows
    std::function<int()> current;                     // the shown frame on screen
    std::function<std::string(int)> label;            // a shown frame's caption
    std::function<Camera3D()> camera;                 // the live camera
    std::function<Vector2()> viewport;                // the view's size in pixels
    std::function<void(int, const Camera3D&)> show;   // put a shown frame and camera on screen
    std::function<int(int, int)> capture;             // keep shown frames first..last for an export; returns how many
    std::function<bool(int)> ready;                   // select a captured frame; true once its surface is built
    std::function<bool(const Camera3D&, int, int, int, std::vector<unsigned char>&)> render; // camera, width, height, supersample, RGBA
    std::function<void(Camera3D&, double)> spin;      // turn a camera by the view's spin over some seconds
    std::function<void()> release;                    // the export ended: drop the capture
};

class Exporter {
public:
    void Open(const Host& host) {
        open = true;
        if (!discovered) { encoder = Encoder::Discover(); discovered = true; }
        const int count = host.frames();
        if (last < 0 || last >= count) last = std::max(0, count - 1);
        first = std::clamp(first, 0, std::max(0, last));
        if (width == 0) {
            const Vector2 view = host.viewport();
            width = EvenSize(static_cast<int>(view.x));
            height = EvenSize(static_cast<int>(view.y));
        }
    }
    bool IsOpen() const { return open; }
    bool Exporting() const { return job != nullptr; }
    const std::string& Status() const { return status; }
    bool Failed() const { return failed; }
    std::vector<Keyframe>& Keyframes() { return keys; }

    // Scripted use: export the current settings now.
    void Export(const Host& host, const std::string& path) {
        std::snprintf(output, sizeof(output), "%s", path.c_str());
        Start(host);
    }
    void Settings(int first_frame, int last_frame, double fps_value, double seconds, int w, int h, int super) {
        first = first_frame; last = last_frame; fps = fps_value; secondsPerFrame = seconds;
        width = EvenSize(w); height = EvenSize(h); supersample = super;
    }
    void SetSpin(bool on) { includeSpin = on; }
    // Advance an export; call every frame, even while the window is closed.
    void Step(const Host& host) {
        if (!job) return;
        Job& j = *job;
        const double budget = GetTime() + 1.0 / 20;
        while (j.n < j.total) {
            const int frame = j.timing.Frame(j.n);
            if (!host.ready(frame)) {
                if (GetTime() - j.waiting > 120) Finish(host, "Timed out building frame " + std::to_string(frame + 1));
                return;
            }
            const Camera3D view = CameraFor(j, j.n);
            host.show(-1, view);
            std::string error;
            if (!host.render(view, j.width, j.height, j.supersample, j.rgba)) { Finish(host, "Could not render the frame"); return; }
            if (!j.writer.Write(j.rgba, error)) { Finish(host, error); return; }
            ++j.n;
            j.waiting = GetTime();
            if (GetTime() > budget) return;
        }
        std::string error;
        if (!j.writer.Close(error)) { Finish(host, error); return; }
        Finish(host, {});
    }

    void Draw(const Host& host) {
        Step(host);
        if (!open) return;
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        ImGui::SetNextWindowSize({430, std::min(720.f, display.y - 120)}, ImGuiCond_Appearing);
        ImGui::SetNextWindowPos({display.x - 470, 100}, ImGuiCond_Appearing);
        if (!ImGui::Begin(ICON_FA_FILM "  Movie###movie", &open, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse)) {
            ImGui::End();
            return;
        }
        const int count = host.frames();
        // The settings scroll; Export and progress stay at the bottom.
        const float footer = ImGui::GetFrameHeightWithSpacing() * (status.empty() ? 1.6f : 3.2f);
        ImGui::BeginChild("movie settings", {0, -footer});
        ImGui::BeginDisabled(job != nullptr);
        Frames(host, count);
        Camera(host, count);
        Preview(host, count);
        Output();
        ImGui::EndDisabled();
        ImGui::EndChild();
        Footer(host, count);
        ImGui::End();
    }

private:
    struct Job {
        Timing timing;
        int n = 0, total = 0, width = 0, height = 0, supersample = 1;
        double waiting = 0;
        Writer writer;
        std::vector<unsigned char> rgba;
        std::vector<Keyframe> keys;
        Easing easing = Easing::EaseInOut;
        Camera3D start{};
        int first = 0;
        bool spin = false;
        std::string path;
        int restoreFrame = 0;
    };

    static void Tip(const char* text) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
    }

    Timing MovieTiming() const { return {last - first + 1, fps, secondsPerFrame}; }

    // The camera for video frame n: the keyframe path (positions count shown
    // frames, so the path is offset by the first frame), or the view when the
    // export started, then the view's spin if included.
    Camera3D CameraFor(const Job& j, int n) const {
        const double position = j.first + j.timing.Position(n);
        Camera3D view = CameraAt(j.keys, position, j.easing, j.start.projection).value_or(j.start);
        if (j.spin && spinner) spinner(view, n / j.timing.fps);
        return view;
    }

    void Frames(const Host& host, int count) {
        if (!BeginSection("Frames")) return;
        if (count == 0) { ImGui::TextDisabled("No frames yet."); EndSection(); return; }
        ImGui::TextWrapped("%d frame%s shown on the timeline; the funnel filter chooses them.", count, count == 1 ? "" : "s");
        last = std::clamp(last, 0, count - 1);
        first = std::clamp(first, 0, last);
        int from = first + 1, to = last + 1;
        FieldLabel("From");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##movie_from", &from, 1, count, "%d", ImGuiSliderFlags_AlwaysClamp)) first = std::min(from, to) - 1;
        Tip(host.label(first).c_str());
        FieldLabel("To");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::SliderInt("##movie_to", &to, 1, count, "%d", ImGuiSliderFlags_AlwaysClamp)) last = std::max(to, from) - 1;
        Tip(host.label(last).c_str());
        FieldLabel("Video rate");
        int rate = fps == 24 ? 0 : fps == 60 ? 2 : 1;
        if (Segmented("##movie_fps", &rate, {"24 fps", "30 fps", "60 fps"})) fps = rate == 0 ? 24 : rate == 2 ? 60 : 30;
        FieldLabel("Seconds per frame");
        float seconds = static_cast<float>(secondsPerFrame);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat("##movie_hold", &seconds, .005f, 1.0f / 60, 10, "%.3f s", ImGuiSliderFlags_AlwaysClamp))
            secondsPerFrame = seconds;
        Tip("How long each frame stays on screen. The camera moves continuously between keyframes regardless.");
        const auto timing = MovieTiming();
        ImGui::TextDisabled("%.1f s, %d video frames", timing.Seconds(), timing.VideoFrames());
        EndSection();
    }

    void Camera(const Host& host, int count) {
        if (!BeginSection("Camera", keys.empty() ? nullptr : std::to_string(keys.size()).c_str())) return;
        FieldLabel("Between keyframes");
        int ease = easing == Easing::Linear ? 1 : 0;
        if (Segmented("##movie_easing", &ease, {"Ease in and out", "Constant speed"}))
            easing = ease == 1 ? Easing::Linear : Easing::EaseInOut;
        if (keys.empty())
            ImGui::TextWrapped("No keyframes: the camera keeps the view it has when the export starts. "
                               "Step to a frame, arrange the view, then add a keyframe.");
        int remove = -1;
        for (size_t i = 0; i < keys.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            const int at = std::clamp(static_cast<int>(std::lround(keys[i].position)), 0, std::max(0, count - 1));
            ImGui::AlignTextToFramePadding();
            ImGui::Text("%zu", i + 1);
            ImGui::SameLine(28);
            ImGui::TextUnformatted(count ? host.label(at).c_str() : "");
            ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 3*ImGui::GetFrameHeight() - 16);
            if (ImGui::Button(ICON_FA_LOCATION_ARROW)) host.show(at, CameraFrom(keys[i]));
            Tip("Go to this keyframe's frame and view");
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_CAMERA)) {
                keys[i] = KeyframeFrom(host.camera(), host.current());
                SortKeyframes(keys);
            }
            Tip("Replace with the frame and view on screen now");
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_TRASH_CAN)) remove = static_cast<int>(i);
            Tip("Remove this keyframe");
            ImGui::PopID();
        }
        if (remove >= 0) keys.erase(keys.begin() + remove);
        ImGui::BeginDisabled(count == 0);
        if (ImGui::Button(ICON_FA_PLUS "  Add keyframe here")) {
            const double at = host.current();
            std::erase_if(keys, [&](const Keyframe& k) { return std::abs(k.position - at) < .5; });
            keys.push_back(KeyframeFrom(host.camera(), at));
            SortKeyframes(keys);
        }
        ImGui::EndDisabled();
        Tip("The camera on screen now, at the frame on screen now. A keyframe at the same frame is replaced.");
        if (!keys.empty()) { ImGui::SameLine(); if (ImGui::Button("Clear")) keys.clear(); }
        Switch("Spin the view", &includeSpin);
        Tip("Add the spin set in the toolbar's Spin menu (axis and rate) on top of the keyframed camera.");
        EndSection();
    }

    void Preview(const Host& host, int count) {
        if (!BeginSection("Preview", nullptr, false)) return;
        if (count == 0) { EndSection(); return; }
        const auto timing = MovieTiming();
        int n = std::clamp(previewFrame, 0, timing.VideoFrames() - 1);
        ImGui::SetNextItemWidth(-1);
        const bool moved = ImGui::SliderInt("##movie_preview", &n, 0, timing.VideoFrames() - 1, "", ImGuiSliderFlags_AlwaysClamp);
        // The view when scrubbing starts: the movie's camera without
        // keyframes, put back when scrubbing ends.
        if (ImGui::IsItemActivated()) previewBase = host.camera();
        const bool released = ImGui::IsItemDeactivated();
        Tip("Scrub through the movie: the view shows each moment's frame and camera.");
        ImGui::TextDisabled("%.2f s · %s", n / timing.fps, host.label(first + timing.Frame(n)).c_str());
        if (moved && previewBase) {
            previewFrame = n;
            Job probe;
            probe.timing = timing; probe.keys = keys; probe.easing = easing; probe.start = *previewBase;
            probe.first = first; probe.spin = includeSpin;
            spinner = host.spin;
            host.show(first + timing.Frame(n), CameraFor(probe, n));
        }
        if (released && previewBase) {
            if (keys.empty()) host.show(-1, *previewBase);
            previewBase.reset();
        }
        EndSection();
    }

    void Output() {
        if (!BeginSection("Output")) return;
        FieldLabel("Size");
        ImGui::SetNextItemWidth(90);
        if (ImGui::InputInt("##movie_width", &width, 0)) width = EvenSize(std::clamp(width, 16, 7680));
        ImGui::SameLine(); ImGui::TextUnformatted("×"); ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        if (ImGui::InputInt("##movie_height", &height, 0)) height = EvenSize(std::clamp(height, 16, 4320));
        bool firstPreset = true;
        for (auto [label, w, h] : {std::tuple{"720p", 1280, 720}, {"1080p", 1920, 1080}, {"4K", 3840, 2160}}) {
            if (!firstPreset) ImGui::SameLine();
            firstPreset = false;
            if (ImGui::SmallButton(label)) { width = w; height = h; }
        }
        Tip("Sizes are rounded down to even numbers, as H.264 needs.");
        FieldLabel("Antialiasing");
        int ss = supersample == 2 ? 1 : 0;
        if (Segmented("##movie_ss", &ss, {"Off", "2× supersampling"})) supersample = ss == 1 ? 2 : 1;
        FieldLabel("Quality");
        ImGui::SetNextItemWidth(-1);
        ImGui::SliderInt("##movie_quality", &quality, 12, 34, quality <= 18 ? "%d (visually lossless)" : "%d", ImGuiSliderFlags_AlwaysClamp);
        Tip("x264's constant rate factor: lower is larger and sharper. 18 to 23 suits most figures.");
        FieldLabel("File");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##movie_path", output, sizeof(output));
        if (encoder.Available()) ImGui::TextDisabled("Encoding with %s (%s)", encoder.codec.c_str(), encoder.ffmpeg.c_str());
        else if (!encoder.ffmpeg.empty()) ImGui::TextColored(ThemeText(Palette().warning), "%s has no H.264 encoder.", encoder.ffmpeg.c_str());
        else ImGui::TextColored(ThemeText(Palette().warning), "ffmpeg was not found. Install it (for example\n"
                                "brew install ffmpeg) or set RAYFEM_FFMPEG.");
        EndSection();
    }

    void Footer(const Host& host, int count) {
        ImGui::Separator();
        if (job) {
            const float progress = job->total ? static_cast<float>(job->n) / job->total : 0;
            char text[64];
            std::snprintf(text, sizeof(text), "%d / %d", job->n, job->total);
            ImGui::ProgressBar(progress, {-90, 0}, text);
            ImGui::SameLine();
            if (ImGui::Button("Cancel", {-1, 0})) {
                job->writer.Abort();
                std::error_code ignored;
                std::filesystem::remove(job->path, ignored);
                Finish(host, "Canceled");
            }
            return;
        }
        ImGui::BeginDisabled(count == 0 || !encoder.Available() || !output[0]);
        if (ImGui::Button(ICON_FA_FILM "  Export movie", {-1, 0})) Start(host);
        ImGui::EndDisabled();
        if (!status.empty()) {
            ImGui::PushTextWrapPos(0);
            ImGui::TextColored(ThemeText(failed ? Palette().warning : Palette().success), "%s", status.c_str());
            ImGui::PopTextWrapPos();
        }
    }

    void Start(const Host& host) {
        if (job) return;
        failed = false;
        if (!discovered) { encoder = Encoder::Discover(); discovered = true; }
        if (!encoder.Available()) { status = "No H.264 encoder: install ffmpeg"; failed = true; return; }
        auto next = std::make_unique<Job>();
        const int captured = host.capture(first, last);
        if (captured <= 0) { status = "No frames to export"; failed = true; return; }
        next->timing = {captured, fps, secondsPerFrame};
        next->total = next->timing.VideoFrames();
        next->width = EvenSize(width); next->height = EvenSize(height); next->supersample = supersample;
        next->keys = keys; next->easing = easing; next->start = host.camera();
        next->first = first; next->spin = includeSpin; next->path = output;
        next->restoreFrame = host.current();
        next->waiting = GetTime();
        next->rgba.resize(static_cast<size_t>(next->width) * next->height * 4);
        spinner = host.spin;
        std::string error;
        if (!next->writer.Open(encoder, output, next->width, next->height, fps, quality, error)) {
            host.release();
            status = error; failed = true; return;
        }
        status.clear();
        job = std::move(next);
    }

    void Finish(const Host& host, const std::string& error) {
        if (!job) return;
        const Camera3D start = job->start;
        const int frame = job->restoreFrame;
        const std::string path = job->path;
        const int frames = job->n;
        const double seconds = job->n / job->timing.fps;
        job.reset();
        host.release();
        host.show(frame, start);
        failed = !error.empty();
        if (failed) status = error;
        else {
            char text[160];
            std::snprintf(text, sizeof(text), "Saved %d frames (%.1f s) to ", frames, seconds);
            status = text + path;
        }
    }

    bool open = false, discovered = false;
    Encoder encoder;
    int first = 0, last = -1;
    double fps = 30, secondsPerFrame = .2;
    int width = 0, height = 0, supersample = 2, quality = 20;
    char output[512] = "rayfem-exports/movie.mp4";
    std::vector<Keyframe> keys;
    Easing easing = Easing::EaseInOut;
    bool includeSpin = false;
    int previewFrame = 0;
    std::optional<Camera3D> previewBase;
    std::function<void(Camera3D&, double)> spinner;
    std::unique_ptr<Job> job;
    std::string status;
    bool failed = false;
};

} // namespace RayFEM::UI::Movie
