#pragma once

#include "figure/figure_writers.hpp"
#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "extras/IconsFontAwesome6.h"
#include "imgui.h"
#include "rlgl.h"

#include <chrono>
#include <future>
#include <memory>

namespace RayFEM::UI::Figure {

// What the viewer supplies to draw and annotate its current view.
struct Scene {
    // Draw the scene into the bound target, cleared to background. lineScale
    // widens edges and lines so they keep their on-screen weight.
    std::function<void(RenderTexture2D&, Color background, float lineScale, bool guides)> render;
    Camera3D camera{};
    Vector2 viewport{};
    BoundingBox bounds{};
    bool hasMesh = false;
    Visibility visible;
    bool hasColorBar = false;
    ColorScaleMapping mapping{};
    ColorMap map{};
    bool reverse = false;
    std::string field;
    Color theme{}, themeText{};
    unsigned frame = 0;
    AxesSettings axes;  // the view's axes; style None while it shows none
};

struct Options {
    int format = 0;      // PNG, PDF
    int background = 1;  // Theme, White, Transparent
    int width = 0, height = 0;
    bool lockAspect = true;
    float dpi = 300;
    int supersample = 1; // index into {1, 2, 4}
    float fontPt = 9, linePt = .7f;
    bool colorbar = true, guides = false, latex = false, keepSource = false, crop = true;
    // Tick labels: size is fontPt.
    bool showLabels = true;
    float labelGapPt = 2.5f, tickPt = 3.5f;
    int ticks = 5;
    // Axis titles.
    bool showTitles = true;
    float titlePt = 10, titleGapPt = 5, titleAlong = .5f;
    // A title over the whole figure (LaTeX), aligned left, center or right.
    char figureTitle[256] = "";
    float figureTitlePt = 11, figureTitleGapPt = 6;
    int figureTitleAlign = 1;
    // Layout: margin around everything; colorbar right or left of the mesh.
    float marginPt = 5;
    int colorbarSide = 0;
    float colorbarGapPt = 16, colorbarWidthPt = 7, colorbarLength = .75f;
    int engine = 0;
    AxesSettings axes;
    char path[512] = "";
};

// The overlay laid out around a scene of a given size: the canvas grows so
// labels outside the scene are kept.
struct Layout {
    Overlay overlay;
    Vector2 canvas{};
    Rectangle scene{};
};

inline void Tip(const char* text) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort | ImGuiHoveredFlags_AllowWhenDisabled)) {
        ImGui::BeginTooltip(); ImGui::PushTextWrapPos(ImGui::GetFontSize()*24); ImGui::TextUnformatted(text);
        ImGui::PopTextWrapPos(); ImGui::EndTooltip();
    }
}

// The axes' style and placement, as the figure and the view offer them;
// true when something changed. allowNone offers "None" as a style.
inline bool AxesControls(AxesSettings& axes, BoundingBox bounds, bool allowNone) {
    bool changed = false;
    const char* names[] = {"None", "Cartesian", "Cylindrical", "Spherical", "Polar"};
    const int skip = allowNone ? 0 : 1;
    int style = std::max(0, int(axes.style) - skip);
    if (ImGui::Combo("##axes_style", &style, names + skip, 5 - skip)) { axes.style = AxesStyle(style + skip); changed = true; }
    if (axes.style != AxesStyle::None && axes.style != AxesStyle::Cartesian) {
        FieldLabel("Origin");
        ImGui::NewLine();
        const float extent = std::max(1e-6f, Vector3Distance(bounds.min, bounds.max));
        changed |= ImGui::DragFloat3("##axes_origin", &axes.origin.x, extent*.002f, 0, 0, "%.4g");
        if (ImGui::Button("Mesh center", {-1, 0})) {
            axes.origin = Vector3Scale(Vector3Add(bounds.min, bounds.max), .5f); changed = true;
        }
        FieldLabel("Axis");
        const float third = (ImGui::GetContentRegionAvail().x - 2*ImGui::GetStyle().ItemSpacing.x)/3;
        const char* letters[] = {"X", "Y", "Z"};
        for (int a = 0; a < 3; ++a) {
            if (a) ImGui::SameLine();
            Vector3 unit{}; (&unit.x)[a] = 1;
            if (ToggleButton(letters[a], Vector3Distance(Vector3Normalize(axes.axis), unit) < 1e-4f, {third, 0})) {
                axes.axis = unit; changed = true;
            }
        }
        Tip("The polar or symmetry axis: z for cylinders, the pole for spheres, the normal of the polar plane.");
        // Ring radius: fitted to the mesh plus padding, or set.
        int custom = axes.radius > 0 ? 1 : 0;
        FieldLabel("Radius");
        if (Segmented("##ring_radius_mode", &custom, {"Fit mesh", "Custom"})) {
            axes.radius = custom ? std::max(1e-6f, extent*.5f) : 0; changed = true;
        }
        if (custom) {
            FieldLabel("Ring radius");
            changed |= ImGui::DragFloat("##ring_radius", &axes.radius, extent*.002f, 1e-6f, extent*100, "%.4g", ImGuiSliderFlags_AlwaysClamp);
            Tip("Radius of the rings and the length of the radial ruler, in mesh units.");
        } else {
            float percent = axes.padding*100;
            FieldLabel("Padding");
            if (ImGui::SliderFloat("##ring_padding", &percent, -50, 100, "%.0f%%")) { axes.padding = percent/100; changed = true; }
            Tip("Rings sit this far outside the mesh radius. Negative values draw them inside.");
        }
        FieldLabel("Angle step");
        int step = axes.angleStep;
        const int steps[] = {10, 15, 30, 45, 60, 90};
        if (ImGui::BeginCombo("##angle_step", (std::to_string(step) + "°").c_str())) {
            for (int value : steps) if (ImGui::Selectable((std::to_string(value) + "°").c_str(), value == step)) { axes.angleStep = value; changed = true; }
            ImGui::EndCombo();
        }
    }
    if (axes.style != AxesStyle::None) {
        int hidden = axes.showHidden ? 0 : 1;
        FieldLabel("Behind mesh");
        if (Segmented("##hidden_axes", &hidden, {"Dashed", "Hidden"})) { axes.showHidden = hidden == 0; changed = true; }
    }
    return changed;
}

class Exporter {
public:
    void Open(const Scene& scene) {
        open = true;
        focus = true;
        if (!initialized) {
            initialized = true;
            tools = LatexTools::Discover();
            options.latex = tools.Any();
            // pdflatex is the common default; xelatex and lualatex remain choices.
            int index = 0;
            for (const auto& [name, path] : tools.engines) { if (name == "pdflatex") options.engine = index; ++index; }
            SetSizeFromViewport(scene, 2);
            options.axes.origin = Vector3Scale(Vector3Add(scene.bounds.min, scene.bounds.max), .5f);
            if (scene.axes.style != AxesStyle::None) options.axes = scene.axes;
        }
        if (options.lockAspect) SetSizeFromViewport(scene, float(options.width)/std::max(1.f, scene.viewport.x));
        if (!options.path[0] || std::string(options.path).starts_with("rayfem-exports/figure-frame-"))
            std::snprintf(options.path, sizeof(options.path), "rayfem-exports/figure-frame-%u.%s", scene.frame, options.format ? "pdf" : "png");
        previewDirty = true;
    }

    void Shutdown() {
        if (preview.id) UnloadTexture(preview);
        preview = {};
        if (job.valid()) job.wait();
    }

    // Call every frame. The window floats over the viewer, which stays usable
    // so the view can be adjusted while the preview follows it. While pinned,
    // the viewer calls DrawPinned in its side column instead.
    void Draw(const Scene& scene) {
        PollJob();
        if (!open || pinned) return;
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        ImGui::SetNextWindowSize({std::min(1040.f, display.x - 80), std::min(680.f, display.y - 80)}, ImGuiCond_Appearing);
        ImGui::SetNextWindowPos({display.x*.5f, display.y*.5f}, ImGuiCond_Appearing, {.5f, .5f});
        if (focus) { ImGui::SetNextWindowFocus(); focus = false; }
        if (ImGui::Begin(ICON_FA_IMAGE "  Export figure###figure", &open, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoCollapse)) {
            TitleBarPin(pinned);
            Contents(scene, false);
        }
        ImGui::End();
    }
    void DrawPinned(const Scene& scene) {
        if (!open || !pinned) return;
        PinnedHeader(ICON_FA_IMAGE "  Export figure", pinned, open);
        Contents(scene, true);
    }
    bool IsOpen() const { return open; }
    bool Pinned() const { return open && pinned; }
    void Pin(bool on) { pinned = on; }
    // Whether it was just opened, once: the side column brings its tab forward.
    bool TakeFocus() { const bool was = focus; focus = false; return was; }

    // Scripted use: adjust the options, export, then poll for the result.
    Options& Settings() { return options; }
    const LatexTools& Tools() const { return tools; }
    void Export(const Scene& scene) { if (!job.valid()) StartExport(scene); }
    bool Busy() { PollJob(); return job.valid(); }
    const std::string& Status() const { return status; }
    bool Failed() const { return statusError; }

    // Lay out the overlay for a scene rendered at sceneSize.
    Layout BuildLayout(const Scene& scene, Vector2 sceneSize, float pxPerPt) const {
        Layout layout;
        Ink ink;
        ink.color = options.background == 0 ? scene.themeText : Color{25, 25, 30, 255};
        ink.hidden = ColorAlpha(ink.color, .45f);
        ink.font = options.fontPt*pxPerPt;
        ink.line = options.linePt*pxPerPt;
        ink.tick = options.tickPt*pxPerPt;
        ink.labelGap = options.labelGapPt*pxPerPt;
        ink.titleFont = options.titlePt*pxPerPt;
        ink.titleGap = options.titleGapPt*pxPerPt;
        ink.titleAlong = options.titleAlong;
        ink.labels = options.showLabels;
        ink.titles = options.showTitles;
        ink.ticks = options.ticks;
        const auto measure = metrics.Measurer();
        if (options.axes.style != AxesStyle::None)
            AddAxes(layout.overlay, options.axes, scene.bounds, View(scene.camera, sceneSize), scene.visible, ink, measure);
        // The mesh's place in the scene: its projected bounds, kept inside.
        const View view(scene.camera, sceneSize);
        float x0 = INFINITY, y0 = INFINITY, x1 = -INFINITY, y1 = -INFINITY;
        for (int corner = 0; corner < 8; ++corner) {
            const Vector3 p{corner & 1 ? scene.bounds.max.x : scene.bounds.min.x, corner & 2 ? scene.bounds.max.y : scene.bounds.min.y,
                            corner & 4 ? scene.bounds.max.z : scene.bounds.min.z};
            if (const auto s = view.Project(p)) { x0 = std::min(x0, s->x); y0 = std::min(y0, s->y); x1 = std::max(x1, s->x); y1 = std::max(y1, s->y); }
        }
        Rectangle mesh{0, 0, sceneSize.x, sceneSize.y};
        if (x1 > x0 && y1 > y0) {
            x0 = std::clamp(x0, 0.f, sceneSize.x); x1 = std::clamp(x1, 0.f, sceneSize.x);
            y0 = std::clamp(y0, 0.f, sceneSize.y); y1 = std::clamp(y1, 0.f, sceneSize.y);
            if (x1 > x0 && y1 > y0) mesh = {x0, y0, x1 - x0, y1 - y0};
        }
        // The colorbar sits beside the mesh rather than at the viewport edge.
        if (options.colorbar && scene.hasColorBar) {
            const float height = std::max(ink.font*6, mesh.height*options.colorbarLength);
            const float width = options.colorbarWidthPt*pxPerPt, gap = options.colorbarGapPt*pxPerPt;
            const bool left = options.colorbarSide == 1;
            // Beyond everything the axes occupy on that side, so rings and
            // rulers never run under the bar.
            const Rectangle axes = Extent(layout.overlay, measure, mesh);
            const float x = left ? axes.x - gap - width : axes.x + axes.width + gap;
            AddColorBar(layout.overlay, {x, mesh.y + (mesh.height - height)*.5f, width, height},
                        scene.mapping, scene.map, scene.reverse, scene.field, ink, left);
        }
        RemoveOverlaps(layout.overlay, measure);
        // Cropping keeps the mesh and annotations; otherwise the whole viewport.
        Rectangle extent = Extent(layout.overlay, measure, options.crop ? mesh : Rectangle{0, 0, sceneSize.x, sceneSize.y});
        if (options.figureTitle[0]) {
            const float align = options.figureTitleAlign*.5f;
            const std::string latex = options.figureTitle;
            layout.overlay.labels.push_back({{extent.x + align*extent.width, extent.y - options.figureTitleGapPt*pxPerPt},
                                             PlainFromLatex(latex), latex, options.figureTitlePt*pxPerPt, {align, 1}, ink.color});
            extent = Extent(layout.overlay, measure, extent);
        }
        const float pad = options.marginPt*pxPerPt;
        const float left = options.crop ? extent.x - pad : std::min(0.f, extent.x - pad);
        const float top = options.crop ? extent.y - pad : std::min(0.f, extent.y - pad);
        const float right = options.crop ? extent.x + extent.width + pad : std::max(sceneSize.x, extent.x + extent.width + pad);
        const float bottom = options.crop ? extent.y + extent.height + pad : std::max(sceneSize.y, extent.y + extent.height + pad);
        layout.overlay.Offset({-left, -top});
        layout.canvas = {std::ceil(right - left), std::ceil(bottom - top)};
        layout.scene = {-left, -top, sceneSize.x, sceneSize.y};
        return layout;
    }

private:
    Options options;
    LatexTools tools;
    TextMetrics metrics;
    bool initialized = false, open = false, focus = false, pinned = true, previewDirty = true;
    Texture2D preview{};
    Vector2 previewCanvas{};
    double previewAt = -1;
    Camera3D previewCamera{};
    std::future<std::string> job;
    std::string status;
    bool statusError = false;

    static int Supersample(int index) { return index == 2 ? 4 : index == 1 ? 2 : 1; }
    std::optional<Color> BackgroundColor(const Scene& scene) const {
        if (options.background == 2) return std::nullopt;
        return options.background == 0 ? scene.theme : WHITE;
    }
    // Clear color: transparent figures fade to white, not black, at edges.
    Color ClearColor(const Scene& scene) const { return BackgroundColor(scene).value_or(Color{255, 255, 255, 0}); }
    void SetSizeFromViewport(const Scene& scene, float factor) {
        const Vector2 v{std::max(1.f, scene.viewport.x), std::max(1.f, scene.viewport.y)};
        options.width = std::max(16, int(std::lround(v.x*factor)));
        options.height = std::max(16, int(std::lround(v.x*factor*v.y/v.x)));
    }
    bool UsingLatex() const {
        if (!options.latex || !tools.Any()) return false;
        return options.format == 1 || tools.PngLabels();
    }
    std::string Engine() const {
        auto it = tools.engines.begin();
        std::advance(it, std::clamp(options.engine, 0, int(tools.engines.size()) - 1));
        return it->first;
    }

    bool Controls(const Scene& scene) {
        bool changed = false;
        ImGui::PushItemWidth(-1);
        if (BeginSection("File")) {
            int format = options.format;
            if (Segmented("##format", &format, {"PNG", "PDF"})) {
                options.format = format; changed = true;
                std::string path = options.path;
                const auto dot = path.rfind('.');
                path = (dot == std::string::npos ? path : path.substr(0, dot)) + (format ? ".pdf" : ".png");
                std::snprintf(options.path, sizeof(options.path), "%s", path.c_str());
            }
            Tip("PNG is a bitmap. PDF keeps axes, colorbar and labels as vector graphics around a bitmap of the mesh.");
            ImGui::InputText("##figure_path", options.path, sizeof(options.path));
            EndSection();
        }
        if (BeginSection("Size")) {
            FieldLabel("Width (px)");
            if (ImGui::InputInt("##figure_width", &options.width, 0, 0)) {
                options.width = std::clamp(options.width, 16, 16384); changed = true;
                if (options.lockAspect) options.height = std::max(16, int(std::lround(options.width*scene.viewport.y/std::max(1.f, scene.viewport.x))));
            }
            FieldLabel("Height (px)");
            ImGui::BeginDisabled(options.lockAspect);
            if (ImGui::InputInt("##figure_height", &options.height, 0, 0)) { options.height = std::clamp(options.height, 16, 16384); changed = true; }
            ImGui::EndDisabled();
            if (Switch("Match viewport shape", &options.lockAspect)) {
                changed = true;
                if (options.lockAspect) SetSizeFromViewport(scene, float(options.width)/std::max(1.f, scene.viewport.x));
            }
            Tip("Keep the viewport's aspect ratio so the figure frames exactly what you see.");
            changed |= Switch("Crop to content", &options.crop);
            Tip("Trim empty background around the mesh, axes and colorbar. Off keeps the whole viewport.");
            const float third = (ImGui::GetContentRegionAvail().x - 2*ImGui::GetStyle().ItemSpacing.x)/3;
            for (int factor : {1, 2, 4}) {
                if (factor > 1) ImGui::SameLine();
                const std::string label = std::to_string(factor) + "× viewport";
                if (ImGui::Button(label.c_str(), {third, 0})) { options.lockAspect = true; SetSizeFromViewport(scene, float(factor)); changed = true; }
            }
            FieldLabel("Resolution (dpi)");
            if (ImGui::InputFloat("##figure_dpi", &options.dpi, 0, 0, "%.0f")) { options.dpi = std::clamp(options.dpi, 36.f, 2400.f); changed = true; }
            Tip("Sets the printed size and the size of text and lines: points convert to pixels at this resolution.");
            ImGui::TextDisabled("%.2f × %.2f in at %.0f dpi", options.width/options.dpi, options.height/options.dpi, options.dpi);
            FieldLabel("Smoothing");
            changed |= Segmented("##supersample", &options.supersample, {"1×", "2×", "4×"});
            Tip("Render at a multiple of the size and scale down, smoothing edges. Higher is slower.");
            EndSection();
        }
        if (BeginSection("Appearance")) {
            changed |= Segmented("##figure_background", &options.background, {"Theme", "White", "Transparent"});
            FieldLabel("Lines (pt)");
            changed |= ImGui::DragFloat("##line_pt", &options.linePt, .01f, .1f, 4, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            ImGui::BeginDisabled(!scene.hasColorBar);
            changed |= Switch("Colorbar", &options.colorbar);
            ImGui::EndDisabled();
            Tip(scene.hasColorBar ? "A colorbar for the field, beside the mesh." : "Color by Field to include a colorbar.");
            if (options.colorbar && scene.hasColorBar) {
                FieldLabel("Side");
                changed |= Segmented("##colorbar_side", &options.colorbarSide, {"Right", "Left"});
                FieldLabel("Gap (pt)");
                changed |= ImGui::DragFloat("##colorbar_gap", &options.colorbarGapPt, .2f, 0, 144, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                Tip("Space between the mesh (or the axis labels beside it) and the colorbar.");
                FieldLabel("Width (pt)");
                changed |= ImGui::DragFloat("##colorbar_width", &options.colorbarWidthPt, .1f, 2, 72, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                FieldLabel("Length");
                float length = options.colorbarLength*100;
                if (ImGui::SliderFloat("##colorbar_length", &length, 20, 150, "%.0f%% of mesh", ImGuiSliderFlags_AlwaysClamp)) {
                    options.colorbarLength = length/100; changed = true;
                }
            }
            changed |= Switch("Analysis guides", &options.guides);
            Tip("Include profile rays and planes and surface traces drawn in the viewport.");
            EndSection();
        }
        if (BeginSection("Axes", options.axes.style == AxesStyle::None ? nullptr : "On")) {
            changed |= AxesControls(options.axes, scene.bounds, true);
            EndSection();
        }
        if (BeginSection("Ticks and labels")) {
            changed |= Switch("Tick labels", &options.showLabels);
            FieldLabel("Label size (pt)");
            changed |= ImGui::DragFloat("##font_pt", &options.fontPt, .1f, 4, 36, "%.1f", ImGuiSliderFlags_AlwaysClamp);
            FieldLabel("Label distance (pt)");
            changed |= ImGui::DragFloat("##label_gap", &options.labelGapPt, .1f, -12, 48, "%.1f", ImGuiSliderFlags_AlwaysClamp);
            Tip("Space from the end of a tick mark to its label.");
            FieldLabel("Tick length (pt)");
            changed |= ImGui::DragFloat("##tick_pt", &options.tickPt, .1f, 0, 24, "%.1f", ImGuiSliderFlags_AlwaysClamp);
            FieldLabel("Ticks per axis");
            changed |= ImGui::SliderInt("##tick_count", &options.ticks, 2, 12, "about %d", ImGuiSliderFlags_AlwaysClamp);
            Tip("Ticks fall on round numbers, so the count is approximate. The colorbar keeps five.");
            EndSection();
        }
        if (BeginSection("Titles")) {
            changed |= Switch("Axis titles", &options.showTitles);
            if (options.showTitles && options.axes.style != AxesStyle::None) {
                const auto defaults = DefaultTitles(options.axes.style);
                for (int i = 0; i < 3; ++i) {
                    if (defaults[i].empty()) continue;
                    FieldLabel(PlainFromLatex(defaults[i]).c_str());
                    char buffer[128];
                    std::snprintf(buffer, sizeof(buffer), "%s", options.axes.titles[i].empty() ? defaults[i].c_str() : options.axes.titles[i].c_str());
                    ImGui::PushID(i);
                    if (ImGui::InputText("##title", buffer, sizeof(buffer))) { options.axes.titles[i] = buffer; changed = true; }
                    ImGui::PopID();
                }
                Tip("Write titles as LaTeX, for example $r/R_\\odot$. Without LaTeX, common symbols are shown as text.");
                FieldLabel("Size (pt)");
                changed |= ImGui::DragFloat("##title_pt", &options.titlePt, .1f, 4, 48, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                FieldLabel("Distance (pt)");
                changed |= ImGui::DragFloat("##title_gap", &options.titleGapPt, .1f, -24, 72, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                Tip("Space beyond the tick labels, outward from the axis.");
                FieldLabel("Along axis");
                changed |= ImGui::SliderFloat("##title_along", &options.titleAlong, 0, 1, options.titleAlong < .02f ? "start" :
                                              options.titleAlong > .98f ? "end" : "%.2f", ImGuiSliderFlags_AlwaysClamp);
                Tip("Where a straight axis's title sits, from its start (0) to its end (1). Ring titles sit below their ring.");
            }
            ImGui::Spacing();
            FieldLabel("Figure title");
            changed |= ImGui::InputText("##figure_title", options.figureTitle, sizeof(options.figureTitle));
            Tip("Shown above everything. Written as LaTeX when typesetting with LaTeX; leave empty for none.");
            if (options.figureTitle[0]) {
                FieldLabel("Size (pt)");
                changed |= ImGui::DragFloat("##figure_title_pt", &options.figureTitlePt, .1f, 4, 72, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                FieldLabel("Align");
                changed |= Segmented("##figure_title_align", &options.figureTitleAlign, {"Left", "Center", "Right"});
                FieldLabel("Distance (pt)");
                changed |= ImGui::DragFloat("##figure_title_gap", &options.figureTitleGapPt, .1f, -24, 144, "%.1f", ImGuiSliderFlags_AlwaysClamp);
                Tip("Space between the title and the figure below it.");
            }
            EndSection();
        }
        if (BeginSection("Layout")) {
            FieldLabel("Margin (pt)");
            changed |= ImGui::DragFloat("##margin_pt", &options.marginPt, .2f, 0, 144, "%.1f", ImGuiSliderFlags_AlwaysClamp);
            Tip("Empty space around the whole figure.");
            EndSection();
        }
        if (BeginSection("LaTeX", UsingLatex() ? "On" : nullptr)) {
            if (!tools.Any()) {
                ImGui::TextWrapped("No LaTeX found. Install a TeX distribution with pdflatex, xelatex or lualatex to typeset labels.");
            } else {
                changed |= Switch("Typeset text with LaTeX", &options.latex);
                std::vector<const char*> engines;
                for (const auto& [name, path] : tools.engines) engines.push_back(name.c_str());
                FieldLabel("Engine");
                changed |= ImGui::Combo("##latex_engine", &options.engine, engines.data(), int(engines.size()));
                Tip(std::next(tools.engines.begin(), std::clamp(options.engine, 0, int(engines.size()) - 1))->second.c_str());
                if (options.latex && options.format == 0 && !tools.PngLabels())
                    ImGui::TextWrapped("PNG labels need latex and dvipng; they will use the plain font.");
                else if (options.latex && options.format == 0)
                    ImGui::TextDisabled("Labels are rendered by dvipng.");
                if (options.format == 1) {
                    changed |= Switch("Keep LaTeX source", &options.keepSource);
                    Tip("Also save the .tex file and the mesh image beside the PDF, to edit labels or reuse the figure in a document.");
                }
            }
            EndSection();
        }
        ImGui::PopItemWidth();
        if (changed) previewDirty = true;
        return changed;
    }

    // Settings beside the preview in the floating window; in the narrow side
    // column, the preview above the settings.
    void Contents(const Scene& scene, bool docked) {
        if (!scene.hasMesh) {
            ImGui::TextDisabled("Nothing to export yet: no surface is shown.");
            return;
        }
        // A locked aspect follows the view, which pinning and resizing change.
        if (options.lockAspect) {
            const int height = std::max(16, int(std::lround(options.width*scene.viewport.y/std::max(1.f, scene.viewport.x))));
            if (height != options.height) { options.height = height; previewDirty = true; }
        }
        const float footer = ImGui::GetFrameHeightWithSpacing()*2.2f;
        if (docked) {
            const float available = ImGui::GetContentRegionAvail().y - footer;
            ImGui::BeginChild("figure preview", {0, std::clamp(available*.4f, 140.f, 460.f)}, ImGuiChildFlags_Borders);
            DrawPreview(scene, false);
            ImGui::EndChild();
            ImGui::BeginChild("figure options", {0, -footer}, ImGuiChildFlags_None);
            Controls(scene);
            ImGui::EndChild();
        } else {
            const float controls = 360;
            ImGui::BeginChild("figure options", {controls, -footer}, ImGuiChildFlags_None);
            const bool changed = Controls(scene);
            ImGui::EndChild();
            ImGui::SameLine();
            ImGui::BeginChild("figure preview", {0, -footer}, ImGuiChildFlags_Borders);
            DrawPreview(scene, changed);
            ImGui::EndChild();
        }
        Footer(scene);
    }

    // Render the scene at size*supersample and lay out the overlay.
    struct Rendered { RenderTexture2D target{}; Layout layout; int scale = 1; Vector2 size{}; };
    Rendered RenderAt(const Scene& scene, Vector2 size, float pxPerPt, int supersample) const {
        Rendered out;
        out.size = size;
        out.layout = BuildLayout(scene, size, pxPerPt);
        out.scale = supersample;
        // Stay within common GPU texture limits.
        while (out.scale > 1 && std::max(out.layout.canvas.x, out.layout.canvas.y)*out.scale > 8192) out.scale /= 2;
        out.target = LoadRenderTexture(int(size.x*out.scale), int(size.y*out.scale));
        scene.render(out.target, ClearColor(scene), size.x*out.scale/std::max(1.f, scene.viewport.x), options.guides);
        return out;
    }

    // The scene image at final size (for PDF and LaTeX), straight from the target.
    static Image SceneImage(const Rendered& r) {
        Image image = LoadImageFromTexture(r.target.texture);
        ImageFlipVertical(&image);
        ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        if (r.scale > 1) ImageResize(&image, int(r.size.x), int(r.size.y));
        return image;
    }

    // Composite the scene and overlay on the GPU, then scale down.
    Image Rasterize(const Scene& scene, const Rendered& r, bool text) const {
        const float s = float(r.scale);
        const Vector2 canvas = r.layout.canvas;
        RenderTexture2D target = LoadRenderTexture(int(canvas.x*s), int(canvas.y*s));
        Font font{};
        bool loaded = false;
        float largest = 0;
        for (const auto& l : r.layout.overlay.labels) largest = std::max(largest, l.size);
        if (text && largest > 0) {
            if (const auto path = FigureFontPath()) {
                auto codepoints = FigureCodepoints();
                font = LoadFontEx(path->c_str(), std::max(8, int(std::ceil(largest*s))), codepoints.data(), int(codepoints.size()));
                loaded = font.texture.id != 0;
                if (loaded) SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
            }
            if (!loaded) font = GetFontDefault();
        }
        BeginTextureMode(target);
        ClearBackground(ClearColor(scene));
        // Copy the scene as is, then blend the overlay with correct alpha.
        rlSetBlendFactors(RL_ONE, RL_ZERO, RL_FUNC_ADD);
        BeginBlendMode(BLEND_CUSTOM);
        DrawTexturePro(r.target.texture, {0, 0, float(r.target.texture.width), -float(r.target.texture.height)},
                       {r.layout.scene.x*s, r.layout.scene.y*s, r.size.x*s, r.size.y*s}, {0, 0}, 0, WHITE);
        EndBlendMode();
        rlSetBlendFactorsSeparate(RL_SRC_ALPHA, RL_ONE_MINUS_SRC_ALPHA, RL_ONE, RL_ONE_MINUS_SRC_ALPHA, RL_FUNC_ADD, RL_FUNC_ADD);
        BeginBlendMode(BLEND_CUSTOM_SEPARATE);
        DrawOverlay(r.layout.overlay, s, text ? &font : nullptr);
        EndBlendMode();
        EndTextureMode();
        if (loaded) UnloadFont(font);
        Image image = LoadImageFromTexture(target.texture);
        UnloadRenderTexture(target);
        ImageFlipVertical(&image);
        ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        if (r.scale > 1) ImageResize(&image, int(canvas.x), int(canvas.y));
        return image;
    }
    void DrawPreview(const Scene& scene, bool changed) {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const double now = GetTime();
        const Camera3D& c = scene.camera;
        const bool moved = !Vector3Equals(c.position, previewCamera.position) || !Vector3Equals(c.target, previewCamera.target) ||
                           !Vector3Equals(c.up, previewCamera.up) || c.fovy != previewCamera.fovy || c.projection != previewCamera.projection;
        // While the view moves, the preview keeps up at about 20 frames a second.
        if (changed || previewDirty || now - previewAt > .5 || (moved && now - previewAt > .05)) {
            previewAt = now;
            previewDirty = false;
            previewCamera = c;
            // Fit the whole canvas, labels included, in the panel.
            const Vector2 full{float(options.width), float(options.height)};
            const float pxPerPt = options.dpi/72;
            const Layout estimate = BuildLayout(scene, full, pxPerPt);
            const float k = std::min({1.f, (available.x - 16)/std::max(1.f, estimate.canvas.x), (available.y - 40)/std::max(1.f, estimate.canvas.y)});
            if (k > 0) {
                const Vector2 size{std::max(16.f, std::floor(full.x*k)), std::max(16.f, std::floor(full.y*k))};
                Rendered r = RenderAt(scene, size, pxPerPt*k, 2);
                Image image = Rasterize(scene, r, true);
                UnloadRenderTexture(r.target);
                if (preview.id) UnloadTexture(preview);
                preview = LoadTextureFromImage(image);
                SetTextureFilter(preview, TEXTURE_FILTER_BILINEAR);
                previewCanvas = {float(image.width), float(image.height)};
                UnloadImage(image);
            }
        }
        if (!preview.id) return;
        const ImVec2 at{ImGui::GetCursorScreenPos().x + (available.x - previewCanvas.x)*.5f, ImGui::GetCursorScreenPos().y + 8};
        auto* draw = ImGui::GetWindowDrawList();
        if (options.background == 2) {
            // A checkerboard shows which parts are transparent.
            const float cell = 8;
            for (float y = 0; y < previewCanvas.y; y += cell) for (float x = 0; x < previewCanvas.x; x += cell)
                draw->AddRectFilled({at.x + x, at.y + y}, {at.x + std::min(x + cell, previewCanvas.x), at.y + std::min(y + cell, previewCanvas.y)},
                                    (int(x/cell) + int(y/cell))%2 ? IM_COL32(205, 205, 210, 255) : IM_COL32(245, 245, 248, 255));
        }
        draw->AddImage(ImTextureID(preview.id), at, {at.x + previewCanvas.x, at.y + previewCanvas.y});
        draw->AddRect(at, {at.x + previewCanvas.x, at.y + previewCanvas.y}, ImGui::GetColorU32(ImGuiCol_Border));
        ImGui::SetCursorScreenPos({ImGui::GetCursorScreenPos().x, at.y + previewCanvas.y + 8});
        ImGui::TextDisabled(UsingLatex() ? "Preview with the plain font; the file is typeset with %s." : "Preview", UsingLatex() ? Engine().c_str() : "");
    }

    void Footer(const Scene& scene) {
        ImGui::Separator();
        const bool running = job.valid();
        if (running) {
            DrawSpinner(ImGui::GetWindowDrawList(), {ImGui::GetCursorScreenPos().x + 9, ImGui::GetCursorScreenPos().y + ImGui::GetFrameHeight()*.5f},
                        6, Palette().muted, GetTime());
            ImGui::Dummy({20, ImGui::GetFrameHeight()});
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled(UsingLatex() ? "Typesetting with %s…" : "Writing…", Engine().c_str());
        } else if (!status.empty()) {
            // A short line beside the buttons; the full message on hover.
            std::string line = status;
            if (!statusError && line.starts_with("Saved ")) line = "Saved " + std::filesystem::path(status.substr(6)).filename().string() +
                                                                   (status.ends_with("LaTeX source") ? " with its LaTeX source" : "");
            const float room = ImGui::GetContentRegionAvail().x - 240;
            while (line.size() > 4 && ImGui::CalcTextSize(line.c_str()).x > room) line = line.substr(0, line.size() - 5) + "…";
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(ThemeText(statusError ? Palette().warning : Palette().success), "%s", line.c_str());
            if (ImGui::IsItemHovered()) { ImGui::BeginTooltip(); ImGui::PushTextWrapPos(560); ImGui::TextUnformatted(status.c_str()); ImGui::PopTextWrapPos(); ImGui::EndTooltip(); }
        }
        const float buttons = 220;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - buttons));
        ImGui::BeginDisabled(running);
        if (ToggleButton(ICON_FA_FILE_EXPORT "  Export", true, {120, 0})) StartExport(scene);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Close", {-1, 0})) open = false;
    }

    void StartExport(const Scene& scene) {
        status.clear();
        statusError = false;
        const Vector2 size{float(options.width), float(options.height)};
        const float pxPerPt = options.dpi/72;
        Rendered r = RenderAt(scene, size, pxPerPt, Supersample(options.supersample));
        const bool latex = UsingLatex();
        const bool pdf = options.format == 1;
        Image raster{}, sceneImage{};
        if (pdf) {
            // Keep only the part of the scene inside the (possibly cropped) canvas.
            sceneImage = SceneImage(r);
            const float x0 = std::max(0.f, -r.layout.scene.x), y0 = std::max(0.f, -r.layout.scene.y);
            const float x1 = std::min(r.size.x, r.layout.canvas.x - r.layout.scene.x), y1 = std::min(r.size.y, r.layout.canvas.y - r.layout.scene.y);
            if (x1 > x0 && y1 > y0) {
                const Rectangle visible{std::floor(x0), std::floor(y0), std::ceil(x1) - std::floor(x0), std::ceil(y1) - std::floor(y0)};
                ImageCrop(&sceneImage, visible);
                r.layout.scene = {r.layout.scene.x + visible.x, r.layout.scene.y + visible.y, visible.width, visible.height};
            }
        }
        else raster = Rasterize(scene, r, !latex);
        UnloadRenderTexture(r.target);
        std::filesystem::path output = options.path;
        const auto background = BackgroundColor(scene);
        const float dpi = options.dpi;
        const bool keep = options.keepSource;
        const std::string engine = latex ? tools.engines.at(Engine()) : std::string();
        const std::string engineName = latex ? Engine() : std::string();
        auto layout = std::make_shared<Layout>(std::move(r.layout));
        const LatexTools latexTools = tools;
        const TextMetrics* textMetrics = &metrics;
        job = std::async(std::launch::async, [=]() mutable -> std::string {
            try {
                if (output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
                if (!pdf) {
                    if (latex) {
                        const auto directory = TemporaryDirectory();
                        std::string error;
                        auto images = RenderLatexLabels(latexTools, layout->overlay.labels, dpi, directory, error);
                        CompositeLabels(raster, layout->overlay.labels, images);
                        for (auto& image : images) UnloadImage(image);
                        std::filesystem::remove_all(directory);
                        if (!error.empty()) { UnloadImage(raster); return "!" + error; }
                    }
                    const bool saved = ExportImage(raster, output.string().c_str());
                    UnloadImage(raster);
                    return saved ? "Saved " + std::filesystem::absolute(output).string() : "!Could not write " + output.string();
                }
                if (!latex) {
                    const std::string data = BuildPdf(layout->canvas, dpi, background, sceneImage, {layout->scene.x, layout->scene.y},
                                                      layout->overlay, *textMetrics);
                    UnloadImage(sceneImage);
                    WriteText(output, data);
                    return "Saved " + std::filesystem::absolute(output).string();
                }
                const auto directory = TemporaryDirectory();
                const std::string stem = output.stem().string();
                const std::string imageName = stem + "-mesh.png";
                ExportImage(sceneImage, (directory/imageName).string().c_str());
                UnloadImage(sceneImage);
                const std::string tex = BuildTikz(layout->canvas, dpi, background, imageName, {layout->scene.x, layout->scene.y},
                                                  {layout->scene.width, layout->scene.height}, layout->overlay, engineName);
                WriteText(directory/(stem + ".tex"), tex);
                if (const auto error = CompileLatex(engine, directory, stem)) {
                    return "!" + *error + " (kept in " + directory.string() + ")";
                }
                std::filesystem::copy_file(directory/(stem + ".pdf"), output, std::filesystem::copy_options::overwrite_existing);
                if (keep) {
                    const auto folder = output.has_parent_path() ? output.parent_path() : std::filesystem::path(".");
                    std::filesystem::copy_file(directory/(stem + ".tex"), folder/(stem + ".tex"), std::filesystem::copy_options::overwrite_existing);
                    std::filesystem::copy_file(directory/imageName, folder/imageName, std::filesystem::copy_options::overwrite_existing);
                }
                std::filesystem::remove_all(directory);
                return "Saved " + std::filesystem::absolute(output).string() + (keep ? " with its LaTeX source" : "");
            } catch (const std::exception& e) { return std::string("!Export failed: ") + e.what(); }
        });
    }

    void PollJob() {
        if (!job.valid() || job.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
        std::string result = job.get();
        statusError = !result.empty() && result[0] == '!';
        status = statusError ? result.substr(1) : result;
    }
};

} // namespace RayFEM::UI::Figure
