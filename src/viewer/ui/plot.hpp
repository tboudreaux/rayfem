#pragma once

#include "imgui.h"
#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "figure/figure_writers.hpp"
#include "extras/IconsFontAwesome6.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace RayFEM::UI {

struct PlotPoint { double x = 0, y = 0; bool valid = true; };
struct PlotSeriesView {
    const char* label = "Series";
    std::span<const PlotPoint> points;
    ImU32 color = ThemeSeriesColor(0);
};
struct PlotOptions {
    const char* x_label = "";
    const char* y_label = "";
    bool legend = true;
    double cursor_x = std::numeric_limits<double>::quiet_NaN();
    // Optional output. A click inside the axes writes the domain coordinate.
    double* clicked_x = nullptr;
    // Optional output. Hovering inside the axes writes the domain coordinate.
    double* hovered_x = nullptr;
    // Optional vertical markers (for example path control points), numbered from 1.
    std::span<const double> markers{};
};
struct PlotRange {
    double x_min = 0, x_max = 1, y_min = 0, y_max = 1;
    size_t finite_points = 0;
};

inline ImU32 PlotColor(size_t index) { return ThemeSeriesColor(index); }
inline bool IsPlotPointValid(const PlotPoint& point) {
    return point.valid && std::isfinite(point.x) && std::isfinite(point.y);
}
// Scale before subtraction to avoid overflow for large, opposite-sign values.
inline double PlotFraction(double value, double low, double high) {
    if (!(high > low) || !std::isfinite(value)) return .5;
    const double scale = std::max(std::abs(low), std::abs(high));
    if (!(scale > 0)) return .5;
    return std::clamp((value/scale-low/scale)/(high/scale-low/scale), 0.0, 1.0);
}
inline double PlotCoordinate(double fraction, double low, double high) {
    fraction = std::clamp(fraction, 0.0, 1.0);
    // lerp preserves endpoints and stays between finite bounds, including
    // adjacent values near the largest representable number.
    return std::lerp(low,high,fraction);
}
inline std::string PlotTickLabel(double value,double low,double high) {
    // Suppress cancellation noise in tick labels only. Hover details and data
    // retain full precision, and genuinely small domains keep their scale.
    const double scale=std::max(std::abs(low),std::abs(high));
    const double zero_tolerance=scale>0?((high/scale-low/scale)*1e-9)*scale:0;
    if(std::abs(value)<=zero_tolerance)value=0;
    char label[64];std::snprintf(label,sizeof(label),"%.4g",value);
    return label;
}
inline void ExpandPlotRange(double& low, double& high, bool padding) {
    if (high == low) {
        const double margin = std::max(1.0, std::abs(low)*.05);
        const double a = low-margin, b = high+margin;
        if (std::isfinite(a)) low = a;
        if (std::isfinite(b)) high = b;
        return;
    }
    if (!padding) return;
    const double scale = std::max(std::abs(low), std::abs(high));
    const double margin = ((high/scale-low/scale)*.06)*scale;
    const double a = low-margin, b = high+margin;
    if (std::isfinite(a)) low = a;
    if (std::isfinite(b)) high = b;
}
inline PlotRange FindPlotRange(std::span<const PlotSeriesView> series) {
    PlotRange range;
    bool x_found = false, y_found = false;
    for (const auto& line : series) for (const auto& point : line.points) {
        // Missing values still retain their position on the horizontal axis.
        if (std::isfinite(point.x)) {
            if (!x_found) { range.x_min=range.x_max=point.x; x_found=true; }
            range.x_min=std::min(range.x_min,point.x); range.x_max=std::max(range.x_max,point.x);
        }
        if (!IsPlotPointValid(point)) continue;
        if (!y_found) { range.y_min=range.y_max=point.y; y_found=true; }
        range.y_min=std::min(range.y_min,point.y); range.y_max=std::max(range.y_max,point.y);
        ++range.finite_points;
    }
    ExpandPlotRange(range.x_min,range.x_max,false);
    ExpandPlotRange(range.y_min,range.y_max,true);
    return range;
}

// Each plot's own settings, kept for the session by its ImGui ID: linear
// or log axes, automatic or set ranges, labels, and the export.
enum class PlotScale { Linear, Log };
struct PlotAxisSettings {
    PlotScale scale = PlotScale::Linear;
    bool automatic = true;
    double min = 0, max = 1;   // in data units, while not automatic
    char label[128] = "";      // empty keeps the plot's own label
};
struct PlotSettings {
    PlotAxisSettings x, y;
    bool grid = true;
    float line = 2;
    // Export: PNG or PDF, size in inches at dpi, text in points.
    int format = 0;
    float width_in = 6.5f, height_in = 4, font_pt = 9, dpi = 300;
    char title[256] = "";
    char path[512] = "";
    std::string status;
    bool failed = false;
};
inline PlotSettings& PlotSettingsFor(ImGuiID id) {
    static std::unordered_map<ImGuiID, PlotSettings> settings;
    return settings[id];
}

// Axis coordinates: log10 on a log axis, where values at or below zero have none.
inline double PlotAxisValue(double value, PlotScale scale) {
    if (scale == PlotScale::Linear) return value;
    return value > 0 ? std::log10(value) : std::numeric_limits<double>::quiet_NaN();
}
inline double PlotDataValue(double coordinate, PlotScale scale) {
    return scale == PlotScale::Log ? std::pow(10.0, coordinate) : coordinate;
}
// Tick positions in axis coordinates: round numbers on a linear axis, whole
// decades on a log axis spanning at least one, else evenly spaced.
inline std::vector<double> PlotTicks(double low, double high, PlotScale scale, int count) {
    std::vector<double> ticks;
    if (scale == PlotScale::Log) {
        const double first = std::ceil(low - 1e-9), last = std::floor(high + 1e-9);
        if (last - first >= 1) {
            const double step = std::max(1.0, std::ceil((last - first)/std::max(1, count)));
            for (double e = first; e <= last + 1e-9; e += step) ticks.push_back(e);
            return ticks;
        }
    } else {
        // A finer step when only one round number falls in a short range.
        for (int target : {count, 2*count}) {
            ticks = Figure::NiceTicks(low, high, target);
            if (ticks.size() >= 2) return ticks;
        }
        ticks.clear();
    }
    for (int i = 0; i <= count; ++i) ticks.push_back(PlotCoordinate(double(i)/count, low, high));
    return ticks;
}
inline std::string PlotAxisLabel(double coordinate, double low, double high, PlotScale scale) {
    if (scale == PlotScale::Linear) return PlotTickLabel(coordinate, low, high);
    char label[64];
    std::snprintf(label, sizeof(label), "%.4g", std::pow(10.0, coordinate));
    return label;
}
// The series in axis coordinates, and the range the settings ask for.
struct PlotTransformed {
    std::vector<std::vector<PlotPoint>> points;
    std::vector<PlotSeriesView> series;
    PlotRange range;
    std::size_t hidden = 0;  // finite values a log axis cannot show
};
inline PlotTransformed TransformPlot(std::span<const PlotSeriesView> series, const PlotSettings& settings) {
    PlotTransformed t;
    const bool log = settings.x.scale == PlotScale::Log || settings.y.scale == PlotScale::Log;
    if (log) {
        t.points.reserve(series.size());
        for (const auto& line : series) {
            auto& out = t.points.emplace_back();
            out.reserve(line.points.size());
            for (const auto& p : line.points) {
                PlotPoint q{PlotAxisValue(p.x, settings.x.scale), PlotAxisValue(p.y, settings.y.scale), p.valid};
                if (IsPlotPointValid(p) && !IsPlotPointValid(q)) ++t.hidden;
                out.push_back(q);
            }
            t.series.push_back({line.label, out, line.color});
        }
    } else t.series.assign(series.begin(), series.end());
    t.range = FindPlotRange(t.series);
    auto apply = [](const PlotAxisSettings& axis, double& low, double& high) {
        if (axis.automatic) return;
        const double a = PlotAxisValue(axis.min, axis.scale), b = PlotAxisValue(axis.max, axis.scale);
        if (std::isfinite(a) && std::isfinite(b) && b > a) { low = a; high = b; }
    };
    apply(settings.x, t.range.x_min, t.range.x_max);
    apply(settings.y, t.range.y_min, t.range.y_max);
    return t;
}

// Lines on the plot's white page: the light theme's colors for series that
// use the current theme's.
inline Color PlotPrintColor(ImU32 color) {
    const auto& current = Palette().series;
    for (std::size_t i = 0; i < current.size(); ++i)
        if ((current[i] & ~IM_COL32_A_MASK) == (color & ~IM_COL32_A_MASK)) {
            color = (light_palette.series[i] & ~IM_COL32_A_MASK) | (color & IM_COL32_A_MASK);
            break;
        }
    return {static_cast<unsigned char>(color & 0xff), static_cast<unsigned char>((color >> 8) & 0xff),
            static_cast<unsigned char>((color >> 16) & 0xff), static_cast<unsigned char>((color >> 24) & 0xff)};
}

// The part of segment a-b inside box, if any (Liang-Barsky).
inline bool ClipSegment(Vector2& a, Vector2& b, Rectangle box) {
    float t0 = 0, t1 = 1;
    const float dx = b.x - a.x, dy = b.y - a.y;
    const float p[] = {-dx, dx, -dy, dy};
    const float q[] = {a.x - box.x, box.x + box.width - a.x, a.y - box.y, box.y + box.height - a.y};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0) { if (q[i] < 0) return false; continue; }
        const float r = q[i]/p[i];
        if (p[i] < 0) t0 = std::max(t0, r); else t1 = std::min(t1, r);
        if (t0 > t1) return false;
    }
    const Vector2 start{a.x + t0*dx, a.y + t0*dy}, end{a.x + t1*dx, a.y + t1*dy};
    a = start; b = end;
    return true;
}

// The plot as figure strokes and labels on a canvas of canvas pixels, at
// pxPerPt pixels per point: for PNG and PDF export.
inline Figure::Overlay PlotFigure(const PlotTransformed& t, const PlotSettings& settings, const char* xLabel, const char* yLabel,
                                  bool legend, Vector2 canvas, float pxPerPt, const Figure::Measure& measure) {
    using namespace Figure;
    Overlay overlay;
    const Color ink{25, 25, 30, 255}, muted{90, 95, 105, 255}, grid{222, 224, 230, 255};
    const float font = settings.font_pt*pxPerPt, margin = 6*pxPerPt, tick = 3*pxPerPt, gap = 2.5f*pxPerPt;
    const auto& r = t.range;
    float top = margin;
    if (settings.title[0]) {
        overlay.labels.push_back({{canvas.x*.5f, top}, settings.title, LatexText(settings.title), font*1.2f, {.5f, 0}, ink});
        top += font*1.2f + gap*2;
    }
    if (legend && !t.series.empty()) {
        float x = margin;
        for (const auto& line : t.series) {
            const std::string label = line.label ? line.label : "Series";
            const float width = 16*pxPerPt + measure(label, font);
            if (x > margin && x + width > canvas.x - margin) { x = margin; top += font*1.4f; }
            const Color color = PlotPrintColor(line.color);
            overlay.strokes.push_back({{{x, top + font*.55f}, {x + 12*pxPerPt, top + font*.55f}}, color, 1.2f*pxPerPt});
            overlay.labels.push_back({{x + 15*pxPerPt, top}, label, LatexText(label), font, {0, 0}, ink});
            x += width + 10*pxPerPt;
        }
        top += font*1.4f + gap;
    }
    if (yLabel && *yLabel) top += font + gap*2;
    const int yCount = std::clamp(int((canvas.y - top)/(font*4)), 2, 6), xCount = std::clamp(int(canvas.x/(font*9)), 2, 7);
    const auto yTicks = PlotTicks(r.y_min, r.y_max, settings.y.scale, yCount);
    const auto xTicks = PlotTicks(r.x_min, r.x_max, settings.x.scale, xCount);
    float labelWidth = 0;
    for (double v : yTicks) labelWidth = std::max(labelWidth, measure(PlotAxisLabel(v, r.y_min, r.y_max, settings.y.scale), font));
    const float lastX = xTicks.empty() ? 0 : measure(PlotAxisLabel(xTicks.back(), r.x_min, r.x_max, settings.x.scale), font)*.5f;
    const float left = margin + labelWidth + gap + tick, right = canvas.x - margin - lastX;
    const float bottom = canvas.y - margin - (xLabel && *xLabel ? font + gap*2 : 0) - font - gap - tick;
    if (right - left < 10 || bottom - top < 10) return overlay;
    const Rectangle box{left, top, right - left, bottom - top};
    auto sx = [&](double v) { return box.x + float(PlotFraction(v, r.x_min, r.x_max))*box.width; };
    auto sy = [&](double v) { return box.y + box.height - float(PlotFraction(v, r.y_min, r.y_max))*box.height; };
    const float hair = .4f*pxPerPt, frame = .8f*pxPerPt;
    for (double v : xTicks) {
        if (v < r.x_min - 1e-12 || v > r.x_max + 1e-12) continue;
        const float x = sx(v);
        if (settings.grid) overlay.strokes.push_back({{{x, box.y}, {x, box.y + box.height}}, grid, hair});
        overlay.strokes.push_back({{{x, box.y + box.height}, {x, box.y + box.height + tick}}, ink, frame});
        const std::string text = PlotAxisLabel(v, r.x_min, r.x_max, settings.x.scale);
        overlay.labels.push_back({{x, box.y + box.height + tick + gap}, text, LatexNumber(text), font, {.5f, 0}, ink});
    }
    for (double v : yTicks) {
        if (v < r.y_min - 1e-12 || v > r.y_max + 1e-12) continue;
        const float y = sy(v);
        if (settings.grid) overlay.strokes.push_back({{{box.x, y}, {box.x + box.width, y}}, grid, hair});
        overlay.strokes.push_back({{{box.x - tick, y}, {box.x, y}}, ink, frame});
        const std::string text = PlotAxisLabel(v, r.y_min, r.y_max, settings.y.scale);
        overlay.labels.push_back({{box.x - tick - gap, y}, text, LatexNumber(text), font, {1, .5f}, ink});
    }
    // Series, clipped to the axes and broken at gaps.
    for (const auto& line : t.series) {
        const Color color = PlotPrintColor(line.color);
        const float width = std::max(.3f, settings.line*.5f)*pxPerPt;
        Stroke run{{}, color, width};
        auto flush = [&] { if (run.points.size() > 1) overlay.strokes.push_back(run); run.points.clear(); };
        bool previousValid = false;
        Vector2 previous{};
        for (const auto& p : line.points) {
            if (!IsPlotPointValid(p)) { flush(); previousValid = false; continue; }
            // Unclamped, so points outside a set range lie beyond the box.
            const Vector2 raw{box.x + float((p.x - r.x_min)/(r.x_max - r.x_min))*box.width,
                              box.y + box.height - float((p.y - r.y_min)/(r.y_max - r.y_min))*box.height};
            if (previousValid) {
                Vector2 a = previous, b = raw;
                if (ClipSegment(a, b, box)) {
                    if (run.points.empty() || Vector2Distance(run.points.back(), a) > 1e-3f) { flush(); run.points.push_back(a); }
                    run.points.push_back(b);
                } else flush();
            }
            previous = raw; previousValid = true;
        }
        flush();
    }
    overlay.strokes.push_back({{{box.x, box.y}, {box.x + box.width, box.y}, {box.x + box.width, box.y + box.height},
                                {box.x, box.y + box.height}, {box.x, box.y}}, ink, frame});
    if (xLabel && *xLabel)
        overlay.labels.push_back({{box.x + box.width*.5f, canvas.y - margin}, xLabel, LatexText(xLabel), font, {.5f, 1}, ink});
    if (yLabel && *yLabel)
        overlay.labels.push_back({{box.x, box.y - gap*2}, yLabel, LatexText(yLabel), font, {0, 1}, muted});
    return overlay;
}

// Write the plot as PNG or PDF; needs the window's graphics context for PNG.
inline void ExportPlot(const PlotTransformed& t, PlotSettings& settings, const char* xLabel, const char* yLabel, bool legend) {
    static const Figure::TextMetrics metrics;
    settings.failed = false;
    try {
        const float dpi = std::clamp(settings.dpi, 36.f, 1200.f);
        const Vector2 canvas{std::round(std::clamp(settings.width_in, .5f, 40.f)*dpi), std::round(std::clamp(settings.height_in, .5f, 40.f)*dpi)};
        const auto overlay = PlotFigure(t, settings, xLabel, yLabel, legend, canvas, dpi/72, metrics.Measurer());
        const auto destination = std::filesystem::absolute(settings.path);
        if (destination.has_parent_path()) std::filesystem::create_directories(destination.parent_path());
        if (settings.format == 1) {
            const std::string pdf = Figure::BuildPdf(canvas, dpi, WHITE, Image{}, {0, 0}, overlay, metrics);
            std::ofstream out(destination, std::ios::binary);
            out.exceptions(std::ios::badbit | std::ios::failbit);
            out << pdf;
        } else {
            if (canvas.x*canvas.y > 16384.f*16384.f) throw std::runtime_error("The image is too large");
            RenderTexture2D target = LoadRenderTexture(int(canvas.x), int(canvas.y));
            if (!target.id) throw std::runtime_error("Could not create the image");
            float largest = 0;
            for (const auto& l : overlay.labels) largest = std::max(largest, l.size);
            Font font = GetFontDefault();
            bool loaded = false;
            if (const auto path = Figure::FigureFontPath()) {
                auto codepoints = Figure::FigureCodepoints();
                font = LoadFontEx(path->c_str(), std::max(8, int(std::ceil(largest))), codepoints.data(), int(codepoints.size()));
                loaded = font.texture.id != 0;
                if (loaded) SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR); else font = GetFontDefault();
            }
            BeginTextureMode(target);
            ClearBackground(WHITE);
            Figure::DrawOverlay(overlay, 1, &font);
            EndTextureMode();
            if (loaded) UnloadFont(font);
            Image image = LoadImageFromTexture(target.texture);
            UnloadRenderTexture(target);
            ImageFlipVertical(&image);
            const bool saved = ExportImage(image, destination.string().c_str());
            UnloadImage(image);
            if (!saved) throw std::runtime_error("Could not write " + destination.string());
        }
        settings.status = "Saved " + destination.string();
    } catch (const std::exception& error) {
        settings.status = error.what();
        settings.failed = true;
    }
}

// The settings popup: axes, appearance and export.
inline void PlotSettingsPopup(PlotSettings& settings, std::span<const PlotSeriesView> input, const PlotTransformed& t,
                              const PlotOptions& options, const char* id) {
    ImGui::SetNextWindowSizeConstraints({300, 0}, {360, FLT_MAX});
    if (!ImGui::BeginPopup("plot settings")) return;
    ImGui::PushItemWidth(-1);
    auto axis = [&](const char* name, PlotAxisSettings& a, double low, double high, const char* label) {
        ImGui::PushID(name);
        ImGui::SeparatorText(name);
        int scale = static_cast<int>(a.scale);
        FieldLabel("Scale");
        if (Segmented("##scale", &scale, {"Linear", "Log"})) a.scale = static_cast<PlotScale>(scale);
        bool automatic = a.automatic;
        if (Switch("Automatic range", &automatic)) {
            // Start a set range from what is shown.
            if (!automatic) { a.min = PlotDataValue(low, a.scale); a.max = PlotDataValue(high, a.scale); }
            a.automatic = automatic;
        }
        if (!a.automatic) {
            FieldLabel("From");
            ImGui::InputDouble("##min", &a.min, 0, 0, "%.6g");
            FieldLabel("To");
            ImGui::InputDouble("##max", &a.max, 0, 0, "%.6g");
            if (!(a.max > a.min) || (a.scale == PlotScale::Log && a.min <= 0))
                ImGui::TextColored(ThemeText(Palette().warning), a.scale == PlotScale::Log ? "A log range needs 0 < From < To." : "From must be below To.");
        }
        FieldLabel("Label");
        ImGui::InputTextWithHint("##label", label && *label ? label : "None", a.label, sizeof(a.label));
        ImGui::PopID();
    };
    axis("Horizontal axis", settings.x, t.range.x_min, t.range.x_max, options.x_label);
    axis("Vertical axis", settings.y, t.range.y_min, t.range.y_max, options.y_label);
    if (t.hidden) ImGui::TextColored(ThemeText(Palette().warning), "%zu values at or below zero are hidden on the log axis.", t.hidden);
    ImGui::SeparatorText("Appearance");
    Switch("Grid", &settings.grid);
    FieldLabel("Line width");
    ImGui::SliderFloat("##line", &settings.line, .5f, 5, "%.1f px", ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::Button("Reset plot settings", {-1, 0})) {
        const auto keep = std::make_tuple(settings.format, std::string(settings.path));
        settings = PlotSettings{};
        settings.format = std::get<0>(keep);
        std::snprintf(settings.path, sizeof(settings.path), "%s", std::get<1>(keep).c_str());
    }

    ImGui::SeparatorText("Export");
    int format = settings.format;
    if (Segmented("##format", &format, {"PNG", "PDF"})) {
        settings.format = format;
        std::string path = settings.path;
        const auto dot = path.rfind('.');
        path = (dot == std::string::npos ? path : path.substr(0, dot)) + (format ? ".pdf" : ".png");
        std::snprintf(settings.path, sizeof(settings.path), "%s", path.c_str());
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("PDF keeps lines and text as vector graphics.");
    FieldLabel("Title");
    ImGui::InputTextWithHint("##title", "None", settings.title, sizeof(settings.title));
    FieldLabel("Width (in)");
    ImGui::DragFloat("##width", &settings.width_in, .05f, .5f, 40, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    FieldLabel("Height (in)");
    ImGui::DragFloat("##height", &settings.height_in, .05f, .5f, 40, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    FieldLabel("Text (pt)");
    ImGui::DragFloat("##font", &settings.font_pt, .1f, 4, 36, "%.1f", ImGuiSliderFlags_AlwaysClamp);
    if (settings.format == 0) {
        FieldLabel("Resolution (dpi)");
        ImGui::DragFloat("##dpi", &settings.dpi, 1, 36, 1200, "%.0f", ImGuiSliderFlags_AlwaysClamp);
    }
    for (auto [label, w, h] : {std::tuple{"3.5 in", 3.5f, 2.4f}, {"6.5 in", 6.5f, 4.f}, {"7 in", 7.f, 3.5f}}) {
        if (ImGui::SmallButton(label)) { settings.width_in = w; settings.height_in = h; }
        ImGui::SameLine();
    }
    ImGui::TextDisabled("column widths");
    if (!settings.path[0]) {
        // A name from the plot's label or ID.
        std::string name = options.y_label && *options.y_label ? options.y_label : id;
        std::string clean;
        for (char c : name) if (std::isalnum(static_cast<unsigned char>(c))) clean += char(std::tolower(static_cast<unsigned char>(c)));
            else if (!clean.empty() && clean.back() != '-') clean += '-';
        while (!clean.empty() && clean.back() == '-') clean.pop_back();
        std::snprintf(settings.path, sizeof(settings.path), "rayfem-exports/plot-%s.%s", clean.empty() ? "plot" : clean.c_str(), settings.format ? "pdf" : "png");
    }
    ImGui::InputText("##path", settings.path, sizeof(settings.path));
    const char* xLabel = settings.x.label[0] ? settings.x.label : options.x_label;
    const char* yLabel = settings.y.label[0] ? settings.y.label : options.y_label;
    // With the settings as they are now, which this frame may have changed.
    if (ToggleButton(ICON_FA_FILE_EXPORT "  Export plot", true, {-1, 0}))
        ExportPlot(TransformPlot(input, settings), settings, xLabel, yLabel, options.legend && t.series.size() > 1);
    if (!settings.status.empty()) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 330);
        ImGui::TextColored(ThemeText(settings.failed ? Palette().warning : Palette().success), "%s", settings.status.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::PopItemWidth();
    ImGui::EndPopup();
}

// An in-window chart: callers keep ownership of point arrays. Invalid/non-finite
// samples create gaps; each series is independently scaled into shared axes.
// The gear at the top right sets log axes, ranges and labels, and exports.
inline void PlotLines(const char* id, std::span<const PlotSeriesView> input,
                      ImVec2 requested_size, const PlotOptions& given = {}) {
    const ImVec2 available=ImGui::GetContentRegionAvail();
    const ImVec2 size={std::max(1.0f,requested_size.x>0?requested_size.x:available.x),
                       std::max(1.0f,requested_size.y>0?requested_size.y:available.y)};
    const ImVec2 origin=ImGui::GetCursorScreenPos();
    PlotSettings& settings=PlotSettingsFor(ImGui::GetID(id));
    PlotOptions options=given;
    if(settings.x.label[0])options.x_label=settings.x.label;
    if(settings.y.label[0])options.y_label=settings.y.label;
    const PlotScale xScale=settings.x.scale,yScale=settings.y.scale;
    const PlotTransformed transformed=TransformPlot(input,settings);
    const std::span<const PlotSeriesView> series=transformed.series;
    ImGui::InvisibleButton(id,size);
    const bool hovered=ImGui::IsItemHovered();
    auto* draw=ImGui::GetWindowDrawList();
    draw->PushClipRect(origin,{origin.x+size.x,origin.y+size.y},true);
    const ImVec2 card_max={origin.x+size.x,origin.y+size.y};
    draw->AddRectFilled(origin,card_max,Palette().plot_background,CardRounding);
    draw->AddRect({origin.x+.5f,origin.y+.5f},{card_max.x-.5f,card_max.y-.5f},
                  Palette().grid,CardRounding);
    // The settings gear, at the top right.
    const ImVec2 gear_min{origin.x+size.x-30,origin.y+6},gear_max{origin.x+size.x-8,origin.y+28};
    const ImVec2 mouse=ImGui::GetMousePos();
    const bool on_gear=hovered && mouse.x>=gear_min.x && mouse.x<=gear_max.x && mouse.y>=gear_min.y && mouse.y<=gear_max.y;
    const bool customized=xScale!=PlotScale::Linear || yScale!=PlotScale::Linear || !settings.x.automatic || !settings.y.automatic;
    auto drawGear=[&] {
        if(on_gear)draw->AddRectFilled(gear_min,gear_max,WithAlpha(Palette().text,24),ControlRounding);
        const ImVec2 icon=ImGui::CalcTextSize(ICON_FA_GEAR);
        draw->AddText({(gear_min.x+gear_max.x-icon.x)*.5f,(gear_min.y+gear_max.y-icon.y)*.5f},
                      customized?Palette().accent:(on_gear?Palette().text:Palette().muted),ICON_FA_GEAR);
    };
    ImGui::PushID(id);
    if(on_gear && ImGui::IsMouseClicked(ImGuiMouseButton_Left))ImGui::OpenPopup("plot settings");
    if(size.x<170 || size.y<100) {
        draw->AddText({origin.x+12,origin.y+10},Palette().muted,"Enlarge this panel to view the plot");
        draw->PopClipRect();ImGui::PopID();return;
    }
    const PlotRange range=transformed.range;
    const float text_height=ImGui::GetTextLineHeight();
    float legend_height=0;
    if(options.legend && !series.empty()) {
        float x=origin.x+12,y=origin.y+10;
        const int max_rows=size.y>=190?2:1;
        int row=0;
        size_t shown=0;
        for(const auto& line:series) {
            const char* label=line.label?line.label:"Series";
            std::string short_label=label;
            const float max_label=std::max(45.0f,std::min(160.0f,size.x-150));
            if(ImGui::CalcTextSize(short_label.c_str()).x>max_label) {
                while(!short_label.empty() && ImGui::CalcTextSize(short_label.c_str()).x>max_label-16) {
                    size_t cut=short_label.size()-1;
                    while(cut && (static_cast<unsigned char>(short_label[cut])&0xC0)==0x80)--cut;
                    short_label.resize(cut);
                }
                short_label+="...";
            }
            float item_width=ImGui::CalcTextSize(short_label.c_str()).x+34;
            if(x>origin.x+12 && x+item_width>origin.x+size.x-100) {
                if(++row>=max_rows)break;
                x=origin.x+12;y+=text_height+7;
            }
            draw->AddLine({x,y+text_height*.5f},{x+14,y+text_height*.5f},line.color,2);
            draw->AddText({x+21,y},Palette().text,short_label.c_str());
            x+=item_width;
            ++shown;
        }
        if(shown<series.size()) {
            char remaining[32];std::snprintf(remaining,sizeof(remaining),"+%zu more",series.size()-shown);
            draw->AddText({origin.x+size.x-95,y},Palette().muted,remaining);
        }
        legend_height=y-origin.y+text_height+9;
    }
    const bool y_label=options.y_label && *options.y_label;
    const float axis_header=y_label?text_height+4:0;
    // Below the gear, so it never covers the axes.
    ImVec2 low={origin.x+70,std::max(origin.y+std::max(legend_height,8.0f)+4+axis_header,gear_max.y+4)};
    const ImVec2 high={origin.x+size.x-18,origin.y+size.y-
        (options.x_label&&*options.x_label?text_height*2+10:text_height+11)};
    const float height=high.y-low.y;
    const auto y_ticks=PlotTicks(range.y_min,range.y_max,yScale,std::clamp(static_cast<int>(height/45),2,5));
    std::vector<std::string> y_labels;
    y_labels.reserve(y_ticks.size());
    for(double v:y_ticks) {
        y_labels.push_back(PlotAxisLabel(v,range.y_min,range.y_max,yScale));
        low.x=std::max(low.x,origin.x+ImGui::CalcTextSize(y_labels.back().c_str()).x+20);
    }
    const float width=high.x-low.x;
    if(width<30 || height<25) {
        draw->AddText({origin.x+12,origin.y+std::max(legend_height,10.0f)},Palette().muted,"Enlarge this panel to view the plot");
        drawGear();draw->PopClipRect();PlotSettingsPopup(settings,input,transformed,given,id);ImGui::PopID();return;
    }
    // Points outside a set range land beyond the axes and are clipped.
    auto screen=[&](const PlotPoint& point) {
        return ImVec2{low.x+static_cast<float>((point.x-range.x_min)/(range.x_max-range.x_min))*width,
                      high.y-static_cast<float>((point.y-range.y_min)/(range.y_max-range.y_min))*height};
    };
    const auto x_ticks=PlotTicks(range.x_min,range.x_max,xScale,std::clamp(static_cast<int>(width/100),2,6));
    for(double v:x_ticks) {
        const float x=low.x+static_cast<float>(PlotFraction(v,range.x_min,range.x_max))*width;
        if(settings.grid)draw->AddLine({x,low.y},{x,high.y},Palette().grid);
        const auto text=PlotAxisLabel(v,range.x_min,range.x_max,xScale);
        const float label_width=ImGui::CalcTextSize(text.c_str()).x;
        const float label_x=std::clamp(x-label_width*.5f,origin.x+3,origin.x+size.x-label_width-3);
        draw->AddText({label_x,high.y+7},Palette().muted,text.c_str());
    }
    for(std::size_t i=0;i<y_ticks.size();++i) {
        const float y=high.y-static_cast<float>(PlotFraction(y_ticks[i],range.y_min,range.y_max))*height;
        if(settings.grid)draw->AddLine({low.x,y},{high.x,y},Palette().grid);
        const auto& text=y_labels[i];
        draw->AddText({low.x-ImGui::CalcTextSize(text.c_str()).x-10,y-text_height*.5f},Palette().muted,text.c_str());
    }
    draw->AddRect(low,high,Palette().border);
    if(options.x_label && *options.x_label) {
        const float label_width=ImGui::CalcTextSize(options.x_label).x;
        draw->AddText({low.x+(width-label_width)*.5f,origin.y+size.y-text_height-6},Palette().text,options.x_label);
    }
    if(y_label)
        draw->AddText({low.x,low.y-text_height-5},Palette().text,options.y_label);
    draw->PushClipRect({low.x-2,low.y-2},{high.x+2,high.y+2},true);
    for(const auto& line:series) {
        bool previous_valid=false;
        ImVec2 previous{};
        for(size_t i=0;i<line.points.size();++i) {
            const auto& point=line.points[i];
            if(!IsPlotPointValid(point)) {previous_valid=false;continue;}
            const ImVec2 current=screen(point);
            if(previous_valid) draw->AddLine(previous,current,line.color,settings.line);
            const bool isolated=!previous_valid && (i+1==line.points.size() || !IsPlotPointValid(line.points[i+1]));
            if(line.points.size()<=32 || isolated) draw->AddCircleFilled(current,settings.line+.5f,line.color);
            previous=current;previous_valid=true;
        }
    }
    for(std::size_t m=0;m<options.markers.size();++m) {
        // The end of a path lands on x_max up to rounding; keep it.
        const double slack=1e-9*std::max(1.0,range.x_max-range.x_min);
        const double value=PlotAxisValue(options.markers[m],xScale);
        if(!std::isfinite(value) || value<range.x_min-slack || value>range.x_max+slack)continue;
        const float x=low.x+static_cast<float>(PlotFraction(std::clamp(value,range.x_min,range.x_max),range.x_min,range.x_max))*width;
        draw->AddLine({x,low.y},{x,high.y},WithAlpha(Palette().guide,90),1);
        const std::string label=std::to_string(m+1);
        const float label_width=ImGui::CalcTextSize(label.c_str()).x;
        draw->AddText({x+3+label_width>high.x?x-3-label_width:x+3,low.y+2},Palette().guide_text,label.c_str());
    }
    const double cursor=PlotAxisValue(options.cursor_x,xScale);
    if(std::isfinite(cursor) && cursor>=range.x_min && cursor<=range.x_max) {
        const float x=low.x+static_cast<float>(PlotFraction(cursor,range.x_min,range.x_max))*width;
        for(float y=low.y;y<high.y;y+=8)draw->AddLine({x,y},{x,std::min(y+4,high.y)},Palette().cursor);
    }
    const bool inside=hovered && !on_gear && mouse.x>=low.x && mouse.x<=high.x && mouse.y>=low.y && mouse.y<=high.y;
    if(inside) {
        const double x=PlotCoordinate((mouse.x-low.x)/width,range.x_min,range.x_max);
        const double y=PlotCoordinate((high.y-mouse.y)/height,range.y_min,range.y_max);
        draw->AddLine({mouse.x,low.y},{mouse.x,high.y},Palette().crosshair);
        draw->AddLine({low.x,mouse.y},{high.x,mouse.y},Palette().crosshair);
        if(options.clicked_x && ImGui::IsMouseClicked(ImGuiMouseButton_Left))*options.clicked_x=PlotDataValue(x,xScale);
        if(options.hovered_x)*options.hovered_x=PlotDataValue(x,xScale);
        ImGui::BeginTooltip();
        ImGui::Text("%s %.7g",options.x_label&&*options.x_label?options.x_label:"x",PlotDataValue(x,xScale));
        ImGui::TextDisabled("Cursor value %.7g",PlotDataValue(y,yScale));
        bool any=false;
        for(const auto& line:series) {
            const PlotPoint* nearest=nullptr;double distance=std::numeric_limits<double>::infinity();
            for(const auto& point:line.points)if(IsPlotPointValid(point)) {
                const double dx=std::abs(PlotFraction(point.x,range.x_min,range.x_max)-PlotFraction(x,range.x_min,range.x_max));
                if(dx<distance){distance=dx;nearest=&point;}
            }
            if(nearest) {
                if(!any){ImGui::Separator();ImGui::TextDisabled("Nearest samples");any=true;}
                const ImVec4 color=ImGui::ColorConvertU32ToFloat4(line.color);
                ImGui::TextColored(color,"%s: %.9g",line.label?line.label:"Series",PlotDataValue(nearest->y,yScale));
                ImGui::SameLine();ImGui::TextDisabled("at %.7g",PlotDataValue(nearest->x,xScale));
                const ImVec2 p=screen(*nearest);
                draw->AddCircleFilled(p,4,line.color);
                draw->AddCircle(p,6,Palette().selection,0,1.2f);
            }
        }
        if(!any)ImGui::TextDisabled("No finite samples");
        ImGui::EndTooltip();
    }
    draw->PopClipRect();
    if(hovered && !on_gear && legend_height>0 && mouse.y<origin.y+legend_height) {
        ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*30);
        for(const auto& line:series) {
            ImGui::PushStyleColor(ImGuiCol_Text,ImGui::ColorConvertU32ToFloat4(line.color));
            ImGui::TextUnformatted(line.label?line.label:"Series");ImGui::PopStyleColor();
        }
        ImGui::PopTextWrapPos();ImGui::EndTooltip();
    }
    if(!range.finite_points) {
        const char* label="No valid samples";
        draw->AddText({low.x+(width-ImGui::CalcTextSize(label).x)*.5f,low.y+height*.5f},Palette().muted,label);
    }
    drawGear();
    if(on_gear)ImGui::SetTooltip("Axes, ranges and export");
    draw->PopClipRect();
    PlotSettingsPopup(settings,input,transformed,given,id);
    ImGui::PopID();
}

} // namespace RayFEM::UI
