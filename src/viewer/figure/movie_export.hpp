#pragma once

// The Movie window: the frames the timeline shows, scene keyframes placed
// from the live view on the movie's own timeline, a preview, and H.264
// export through ffmpeg. It is a floating window, not a modal dialog, so the
// view stays usable for arranging each keyframe; it can also be pinned to the
// viewer's side column.
//
// Keyframes sit at times in the movie, not at data frames, so a movie of a
// single frame (one converged model, say) can still move the camera, cut and
// show annotations freely; with several frames, the frames play underneath.

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
// from 0; during an export, ready() counts the frames captured when it started.
struct Host {
    std::function<int()> frames;                       // frames the timeline shows
    std::function<std::string(int)> label;             // a shown frame's caption
    std::function<SceneState()> state;                 // the live camera, cut and annotations
    std::function<void(int, const SceneState&)> apply; // show a scene, at a shown frame unless -1
    std::function<Vector2()> viewport;                 // the view's size in pixels
    std::function<int(int, int)> capture;              // keep shown frames first..last for an export; returns how many
    std::function<bool(int)> ready;                    // select a captured frame; true once its surface is built
    std::function<bool(const Camera3D&, int, int, int, bool, std::vector<unsigned char>&)> render; // camera, width, height, supersample, axes, RGBA
    std::function<bool()> axesShown;                   // whether the view shows its axes
    std::function<void(Camera3D&, double)> spin;       // turn a camera by radians about the Spin menu's axis
    std::function<void()> release;                     // the export ended: drop the capture
};

// Whether a movie draws the view's axes: when the view shows them, always or never.
enum class AxesMode { AsInView, On, Off };

class Exporter {
public:
    void Open(const Host& host) {
        open = true;
        focus = true;
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
    bool Pinned() const { return open && pinned; }
    void Pin(bool on) { pinned = on; }
    bool Exporting() const { return job != nullptr; }
    const std::string& Status() const { return status; }
    bool Failed() const { return failed; }
    std::vector<Keyframe>& Keyframes() { return keys; }

    // Scripted use: set up, then export.
    void Settings(int first_frame, int last_frame, double fps_value, double seconds, int w, int h, int super) {
        first = first_frame; last = last_frame; fps = fps_value; secondsPerFrame = seconds;
        width = EvenSize(w); height = EvenSize(h); supersample = super;
    }
    // A total length in seconds instead of seconds per frame; 0 restores per frame.
    void SetLength(double seconds) { totalLength = seconds > 0; if (seconds > 0) totalSeconds = seconds; }
    void SetSpin(bool on, double turns = 1) { includeSpin = on; spinTurns = turns; }
    void SetAxes(AxesMode mode) { axes = mode; }
    void Export(const Host& host, const std::string& path) {
        std::snprintf(output, sizeof(output), "%s", path.c_str());
        Start(host);
    }

    // Advance an export; call every frame, even while the window is closed.
    void Step(const Host& host) {
        if (!job) return;
        Job& j = *job;
        const double budget = GetTime() + 1.0 / 20;
        while (j.n < j.total) {
            const SceneState scene = StateFor(j, j.n);
            // The scene first: a cut that moves needs its surface rebuilt.
            host.apply(-1, scene);
            if (!host.ready(j.timing.Frame(j.n))) {
                if (GetTime() - j.waiting > 120) Finish(host, "Timed out building frame " + std::to_string(j.timing.Frame(j.n) + 1));
                return;
            }
            std::string error;
            if (!host.render(scene.camera, j.width, j.height, j.supersample, j.axes, j.rgba)) { Finish(host, "Could not render the frame"); return; }
            if (!j.writer.Write(j.rgba, error)) { Finish(host, error); return; }
            ++j.n;
            j.waiting = GetTime();
            if (GetTime() > budget) return;
        }
        std::string error;
        if (!j.writer.Close(error)) { Finish(host, error); return; }
        Finish(host, {});
    }

    // Call every frame. While pinned, the viewer calls DrawPinned in its side
    // column instead of the floating window.
    void Draw(const Host& host) {
        Step(host);
        if (!open || pinned) return;
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        ImGui::SetNextWindowSize({440, std::min(760.f, display.y - 120)}, ImGuiCond_Appearing);
        ImGui::SetNextWindowPos({display.x - 480, 100}, ImGuiCond_Appearing);
        if (focus) { ImGui::SetNextWindowFocus(); focus = false; }
        if (ImGui::Begin(ICON_FA_FILM "  Movie###movie", &open, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse)) {
            TitleBarPin(pinned);
            Contents(host);
        }
        ImGui::End();
    }
    void DrawPinned(const Host& host) {
        if (!open || !pinned) return;
        PinnedHeader(ICON_FA_FILM "  Movie", pinned, open);
        Contents(host);
    }

private:
    void Contents(const Host& host) {
        const int count = host.frames();
        // The settings scroll; Export and progress stay at the bottom.
        const float footer = ImGui::GetFrameHeightWithSpacing() * (status.empty() ? 1.6f : 3.2f);
        ImGui::BeginChild("movie settings", {0, -footer});
        ImGui::BeginDisabled(job != nullptr);
        Frames(host, count);
        Timeline(host, count);
        Output(host);
        ImGui::EndDisabled();
        ImGui::EndChild();
        Footer(host, count);
    }

    // What decides the scene at each moment.
    struct Plan {
        Timing timing;
        std::vector<Keyframe> keys;
        SceneState start;
        bool spin = false;
        double turns = 1;
    };
    struct Job : Plan {
        int n = 0, total = 0, width = 0, height = 0, supersample = 1;
        bool axes = false;
        double waiting = 0;
        Writer writer;
        std::vector<unsigned char> rgba;
        std::string path;
        int restoreFrame = 0;
    };

    static void Tip(const char* text) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", text);
    }

    Timing MovieTiming() const { return {last - first + 1, fps, secondsPerFrame, totalLength ? totalSeconds : 0}; }
    double PreviewTime() const { return previewFrame / fps; }

    // The scene for video frame n: the keyframes at its time, or the scene
    // when the export started, then the spin: turns spread over the whole
    // movie, so a whole number of turns loops seamlessly.
    SceneState StateFor(const Plan& j, int n) const {
        SceneState scene = StateAt(j.keys, n / j.timing.fps, j.start.camera.projection).value_or(j.start);
        if (j.spin && spinner) spinner(scene.camera, 2 * PI * j.turns * n / j.timing.VideoFrames());
        return scene;
    }
    Plan Probe(const SceneState& start) const {
        Plan probe;
        probe.timing = MovieTiming(); probe.keys = keys; probe.start = start;
        probe.spin = includeSpin; probe.turns = spinTurns;
        return probe;
    }
    // Show the movie at video frame n in the view.
    void Preview(const Host& host, int n, const SceneState& start) {
        const auto timing = MovieTiming();
        previewFrame = std::clamp(n, 0, timing.VideoFrames() - 1);
        spinner = host.spin;
        host.apply(first + timing.Frame(previewFrame), StateFor(Probe(start), previewFrame));
    }

    void Frames(const Host& host, int count) {
        if (!BeginSection("Frames")) return;
        if (count == 0) { ImGui::TextDisabled("No frames yet."); EndSection(); return; }
        ImGui::TextWrapped("%d frame%s shown on the timeline; the funnel filter chooses them. A single frame works too.",
                           count, count == 1 ? "" : "s");
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
        FieldLabel("Length");
        int mode = totalLength ? 1 : 0;
        if (Segmented("##movie_length_mode", &mode, {"Per frame", "Total"})) totalLength = mode == 1;
        if (totalLength) {
            float seconds = static_cast<float>(totalSeconds);
            ImGui::SetNextItemWidth(-1);
            if (ImGui::DragFloat("##movie_total", &seconds, .1f, .5f, 3600, "%.1f s", ImGuiSliderFlags_AlwaysClamp))
                totalSeconds = seconds;
            Tip("The movie's length; its frames share it evenly. A single frame then fills the whole movie.");
        } else {
            float seconds = static_cast<float>(secondsPerFrame);
            ImGui::SetNextItemWidth(-1);
            if (ImGui::DragFloat("##movie_hold", &seconds, .005f, 1.0f / 60, 10, "%.3f s each", ImGuiSliderFlags_AlwaysClamp))
                secondsPerFrame = seconds;
            Tip("How long each frame stays on screen. The scene moves continuously between keyframes regardless.");
        }
        const auto timing = MovieTiming();
        ImGui::TextDisabled("%.1f s, %d video frames", timing.Seconds(), timing.VideoFrames());
        EndSection();
    }

    void Timeline(const Host& host, int count) {
        if (!BeginSection("Keyframes", keys.empty() ? nullptr : std::to_string(keys.size()).c_str())) return;
        if (count == 0) { EndSection(); return; }
        const auto timing = MovieTiming();
        // The preview scrubber picks the moment new keyframes go at.
        int n = std::clamp(previewFrame, 0, timing.VideoFrames() - 1);
        ImGui::SetNextItemWidth(-1);
        const bool moved = ImGui::SliderInt("##movie_preview", &n, 0, timing.VideoFrames() - 1, "", ImGuiSliderFlags_AlwaysClamp);
        // Without keyframes the movie keeps the view it starts with; the view
        // when scrubbing starts stands in for it and returns afterwards.
        if (ImGui::IsItemActivated()) scrubBase = host.state();
        const bool released = ImGui::IsItemDeactivated();
        Tip("Scrub through the movie: the view shows each moment's frame and scene. Keyframes are added at this moment.");
        if (moved && scrubBase) Preview(host, n, *scrubBase);
        if (released && scrubBase) {
            if (keys.empty()) host.apply(-1, *scrubBase);
            scrubBase.reset();
        }
        ImGui::TextDisabled("%.2f s of %.2f s · %s", PreviewTime(), timing.Seconds(),
                            host.label(first + timing.Frame(previewFrame)).c_str());

        if (ImGui::Button(ICON_FA_PLUS "  Add keyframe here")) {
            const double at = PreviewTime();
            std::erase_if(keys, [&](const Keyframe& k) { return std::abs(k.time - at) < .5 / fps; });
            Keyframe key = KeyframeFrom(host.state(), at);
            // A new keyframe moves on as the one before it does.
            const auto before = std::find_if(keys.rbegin(), keys.rend(), [&](const Keyframe& k) { return k.time < at; });
            if (before != keys.rend()) key.easing = before->easing;
            keys.push_back(std::move(key));
            SortKeyframes(keys);
        }
        Tip("Record the view on screen now (camera, cut and visible annotations) at the preview's moment. "
            "A keyframe at the same moment is replaced.");
        if (!keys.empty()) { ImGui::SameLine(); if (ImGui::Button("Clear")) keys.clear(); }
        if (keys.empty())
            ImGui::TextWrapped("No keyframes: the movie keeps the view it has when the export starts. Scrub to a moment, "
                               "arrange the camera, cut and annotations, then add a keyframe; add another at a later moment "
                               "and the scene moves between them.");
        int remove = -1;
        bool retimed = false;
        for (size_t i = 0; i < keys.size(); ++i) {
            auto& key = keys[i];
            ImGui::PushID(static_cast<int>(i));
            float seconds = static_cast<float>(key.time);
            ImGui::SetNextItemWidth(72);
            if (ImGui::DragFloat("##time", &seconds, .02f, 0, static_cast<float>(timing.Seconds()), "%.2f s", ImGuiSliderFlags_AlwaysClamp)) {
                key.time = seconds;
                retimed = true;
            }
            Tip("When this keyframe happens; drag to move it");
            ImGui::SameLine();
            std::string what;
            if (timing.frames > 1) what = host.label(first + timing.Frame(static_cast<int>(std::lround(key.time * fps))));
            if (key.clip) what += what.empty() ? ICON_FA_SCISSORS " cut" : "  " ICON_FA_SCISSORS;
            if (!key.annotations.empty()) what += (what.empty() ? "" : "  ") + std::string(ICON_FA_PEN " ") + std::to_string(key.annotations.size());
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("%s", what.c_str());
            ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - 3*ImGui::GetFrameHeight() - 16);
            if (ImGui::Button(ICON_FA_LOCATION_ARROW)) {
                previewFrame = std::clamp(static_cast<int>(std::lround(key.time * fps)), 0, timing.VideoFrames() - 1);
                host.apply(first + timing.Frame(previewFrame), StateFrom(key, host.state().camera.projection));
            }
            Tip("Go to this keyframe: its moment, frame and scene");
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_CAMERA)) {
                const Easing kept = key.easing;
                key = KeyframeFrom(host.state(), key.time);
                key.easing = kept;
            }
            Tip("Replace with the scene on screen now");
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_TRASH_CAN)) remove = static_cast<int>(i);
            Tip("Remove this keyframe");
            if (i + 1 < keys.size()) Move(key, keys[i + 1].time - key.time);
            ImGui::PopID();
        }
        if (remove >= 0) keys.erase(keys.begin() + remove);
        // Reorder once a time drag ends, so rows do not jump under the pointer.
        if (retimed) pendingSort = true;
        if (pendingSort && !ImGui::IsAnyItemActive()) { SortKeyframes(keys); pendingSort = false; }
        if (keys.size() >= 2) {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##movie_presets", "Set every move...")) {
                const struct { EasingPreset preset; const char* name; const char* tip; } presets[] = {
                    {EasingPreset::Smooth, "Ease in and out", "Every move starts and stops gently, pausing at each keyframe."},
                    {EasingPreset::Constant, "Constant speed", "Every move at one speed, with no pauses."},
                    {EasingPreset::Continuous, "One continuous move",
                     "Ease in from the first keyframe, constant speed through the middle ones, ease out into the last: "
                     "a single gliding move that starts and stops gently."},
                };
                for (const auto& p : presets) {
                    if (ImGui::Selectable(p.name)) ApplyEasingPreset(keys, p.preset);
                    Tip(p.tip);
                }
                ImGui::EndCombo();
            }
            Tip("Choose how every move between keyframes eases at once; each move's own menu changes just that one.");
        }
        Switch("Spin the view", &includeSpin);
        Tip("Turn the camera about the axis chosen in the toolbar's Spin menu, on top of any keyframes.");
        if (includeSpin) {
            float turns = static_cast<float>(spinTurns);
            ImGui::SetNextItemWidth(-1);
            if (ImGui::DragFloat("##movie_turns", &turns, .05f, -20, 20, "%.2f turns over the movie", ImGuiSliderFlags_AlwaysClamp))
                spinTurns = turns;
            Tip("Whole turns loop seamlessly. Negative turns go the other way.");
        }
        EndSection();
    }

    // The move from a keyframe to the next: its speed profile, easing and length.
    void Move(Keyframe& key, double seconds) {
        ImGui::Indent(18);
        const float size = ImGui::GetFrameHeight();
        EasingCurve(key.easing, size);
        Tip(EasingDescription(key.easing));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        if (ImGui::BeginCombo("##easing", EasingName(key.easing))) {
            for (const Easing easing : Easings) {
                ImGui::PushID(static_cast<int>(easing));
                const ImVec2 at = ImGui::GetCursorPos();
                if (ImGui::Selectable("##choice", easing == key.easing, 0, {0, size})) key.easing = easing;
                Tip(EasingDescription(easing));
                ImGui::SetCursorPos(at);
                EasingCurve(easing, size);
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(EasingName(easing));
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        Tip("How the scene moves from this keyframe to the next");
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextDisabled(ICON_FA_ARROW_DOWN " %.2f s", seconds);
        ImGui::Unindent(18);
    }
    // Progress against time for an easing, in a small square.
    static void EasingCurve(Easing easing, float size) {
        const ImVec2 corner = ImGui::GetCursorScreenPos();
        ImGui::Dummy({size, size});
        auto* draw = ImGui::GetWindowDrawList();
        const float pad = 4, side = size - 2 * pad;
        const ImVec2 lo{corner.x + pad, corner.y + pad}, hi{corner.x + size - pad, corner.y + size - pad};
        draw->AddRect(lo, hi, WithAlpha(Palette().muted, 70), 2);
        ImVec2 points[33];
        int count = 0;
        for (int i = 0; i <= 30; ++i) {
            const double u = i / 30.0;
            // A hold stays flat to the end, then cuts straight up.
            const double progress = easing == Easing::Hold ? 0 : Ease(easing, u);
            points[count++] = {lo.x + side * static_cast<float>(u), hi.y - side * static_cast<float>(progress)};
        }
        if (easing == Easing::Hold) points[count++] = {hi.x, lo.y};
        draw->AddPolyline(points, count, Palette().accent, 0, 1.6f);
    }

    void Output(const Host& host) {
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
        FieldLabel("Axes");
        int mode = static_cast<int>(axes);
        if (Segmented("##movie_axes", &mode, {"As in view", "On", "Off"})) axes = static_cast<AxesMode>(mode);
        Tip(host.axesShown && host.axesShown() ? "Axes over the mesh, styled under View > Axes. The view shows them now."
                                               : "Axes over the mesh, styled under View > Axes. The view hides them now.");
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
        const int restore = std::clamp(first + MovieTiming().Frame(previewFrame), 0, std::max(0, host.frames() - 1));
        const int captured = host.capture(first, last);
        if (captured <= 0) { status = "No frames to export"; failed = true; return; }
        next->timing = {captured, fps, secondsPerFrame, totalLength ? totalSeconds : 0};
        next->total = next->timing.VideoFrames();
        next->width = EvenSize(width); next->height = EvenSize(height); next->supersample = supersample;
        next->keys = keys; next->start = host.state();
        next->spin = includeSpin; next->turns = spinTurns; next->path = output;
        next->restoreFrame = restore;
        next->axes = axes == AxesMode::On || (axes == AxesMode::AsInView && host.axesShown && host.axesShown());
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
        const SceneState start = job->start;
        const int frame = job->restoreFrame;
        const std::string path = job->path;
        const int frames = job->n;
        const double seconds = job->n / job->timing.fps;
        job.reset();
        host.release();
        host.apply(frame, start);
        failed = !error.empty();
        if (failed) status = error;
        else {
            char text[160];
            std::snprintf(text, sizeof(text), "Saved %d frames (%.1f s) to ", frames, seconds);
            status = text + path;
        }
    }

    bool open = false, focus = false, pinned = false, discovered = false;
    AxesMode axes = AxesMode::AsInView;
    Encoder encoder;
    int first = 0, last = -1;
    double fps = 30, secondsPerFrame = .2;
    bool totalLength = false;
    double totalSeconds = 10;
    int width = 0, height = 0, supersample = 2, quality = 20;
    char output[512] = "rayfem-exports/movie.mp4";
    std::vector<Keyframe> keys;
    bool includeSpin = false;
    double spinTurns = 1;
    int previewFrame = 0;
    bool pendingSort = false;
    std::optional<SceneState> scrubBase;
    std::function<void(Camera3D&, double)> spinner;
    std::unique_ptr<Job> job;
    std::string status;
    bool failed = false;
};

} // namespace RayFEM::UI::Movie
