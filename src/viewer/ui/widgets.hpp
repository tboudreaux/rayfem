#pragma once

#include "ui/theme.hpp"
#include "extras/IconsFontAwesome6.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "libRayfem/mesh/colormap.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>
#include <initializer_list>
#include <string>

// Inspector widgets: section cards, switches, segmented choices and a color
// map picker. They use the theme palette, so both light and dark modes follow.
namespace RayFEM::UI {

inline ImFont*& HeadingFont() { static ImFont* font = nullptr; return font; }

// A titled card. Click the header to collapse it; the badge stays visible.
// Call EndSection only when BeginSection returns true.
inline bool BeginSection(const char* title, const char* badge = nullptr, bool default_open = true) {
    ImGui::PushID(title);
    auto* storage = ImGui::GetStateStorage();
    const ImGuiID key = ImGui::GetID("open");
    bool open = storage->GetBool(key, default_open);
    const float width = ImGui::GetContentRegionAvail().x, height = ImGui::GetFrameHeight();
    const ImVec2 min = ImGui::GetCursorScreenPos(), max{min.x + width, min.y + height};
    if (ImGui::InvisibleButton("header", {width, height})) { open = !open; storage->SetBool(key, open); }
    const bool hovered = ImGui::IsItemHovered();
    auto* draw = ImGui::GetWindowDrawList();
    const auto& palette = Palette();
    if (hovered) draw->AddRectFilled(min, max, WithAlpha(palette.text, 14), ControlRounding);
    const float center = (min.y + max.y)*.5f;
    const char* chevron = open ? ICON_FA_CHEVRON_DOWN : ICON_FA_CHEVRON_RIGHT;
    ImGui::PushFont(nullptr, ImGui::GetFontSize()*.7f);
    const ImVec2 chevronSize = ImGui::CalcTextSize(chevron);
    draw->AddText({min.x + 6, center - chevronSize.y*.5f}, palette.muted, chevron);
    ImGui::PopFont();
    ImGui::PushFont(HeadingFont(), ImGui::GetFontSize()*.94f);
    const ImVec2 titleSize = ImGui::CalcTextSize(title);
    draw->AddText({min.x + 24, center - titleSize.y*.5f}, open || hovered ? palette.text : palette.muted, title);
    ImGui::PopFont();
    if (badge && *badge) {
        const ImVec2 textSize = ImGui::CalcTextSize(badge);
        const float padding = 7, pill = textSize.y + 4;
        const ImVec2 a{max.x - 10 - textSize.x - 2*padding, center - pill*.5f}, b{max.x - 10, center + pill*.5f};
        draw->AddRectFilled(a, b, WithAlpha(palette.accent, 48), pill*.5f);
        draw->AddText({a.x + padding, a.y + 2}, palette.accent, badge);
    }
    if (!open) { ImGui::PopID(); ImGui::Dummy({0, 2}); return false; }
    // The card: a lightly tinted panel sized to its contents.
    const bool light = CurrentTheme() == ThemeMode::Light;
    ImGui::PushStyleColor(ImGuiCol_ChildBg, light ? IM_COL32(20, 45, 70, 10) : IM_COL32(255, 255, 255, 9));
    ImGui::PushStyleColor(ImGuiCol_Border, WithAlpha(palette.border, light ? 120 : 90));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, CardRounding - 2);
    // Generous inner padding keeps labels and fields clear of the card edge.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {14, 12});
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {8, 8});
    ImGui::BeginChild("card", {0, 0}, ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding | ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    return true;
}

inline void EndSection() {
    ImGui::EndChild();
    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(2);
    ImGui::PopID();
    ImGui::Dummy({0, 4});
}

// A small busy indicator: a faint ring with a rotating arc.
inline void DrawSpinner(ImDrawList* draw, ImVec2 center, float radius, ImU32 color, double time) {
    constexpr float pi = 3.14159265f;
    const float start = static_cast<float>(std::fmod(time*2*pi, 2*pi));
    draw->AddCircle(center, radius, WithAlpha(color, 50), 24, 2);
    draw->PathArcTo(center, radius, start, start + 1.4f*pi, 20);
    draw->PathStroke(color, 0, 2);
}

// An on/off switch with its label. Same contract as ImGui::Checkbox.
inline bool Switch(const char* label, bool* value) {
    const float frame = ImGui::GetFrameHeight();
    const float height = std::round(frame*.6f), width = std::round(height*1.75f);
    const char* end = ImGui::FindRenderedTextEnd(label);
    const ImVec2 textSize = ImGui::CalcTextSize(label, end);
    const float spacing = ImGui::GetStyle().ItemInnerSpacing.x + 2;
    const ImVec2 min = ImGui::GetCursorScreenPos();
    const bool pressed = ImGui::InvisibleButton(label, {width + (textSize.x > 0 ? spacing + textSize.x : 0), frame});
    if (pressed) *value = !*value;
    const bool hovered = ImGui::IsItemHovered(), disabled = ImGui::GetItemFlags() & ImGuiItemFlags_Disabled;
    auto* draw = ImGui::GetWindowDrawList();
    const auto& palette = Palette();
    const ImVec2 a{min.x, min.y + (frame - height)*.5f}, b{a.x + width, a.y + height};
    const ImU32 track = *value ? (hovered ? WithAlpha(palette.accent, 230) : palette.accent)
                               : ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg);
    const float alpha = disabled ? .45f : 1.f;
    draw->AddRectFilled(a, b, ImGui::GetColorU32(track, alpha), height*.5f);
    if (!*value) draw->AddRect(a, b, WithAlpha(palette.border, 160), height*.5f);
    const float knob = height*.5f - 2.5f;
    const ImVec2 center{*value ? b.x - height*.5f : a.x + height*.5f, (a.y + b.y)*.5f};
    draw->AddCircleFilled(center, knob, *value ? IM_COL32(255, 255, 255, 255) : WithAlpha(palette.text, 200));
    if (textSize.x > 0)
        draw->AddText({b.x + spacing, min.y + (frame - textSize.y)*.5f}, disabled ? palette.muted : palette.text, label, end);
    return pressed;
}

// A row of mutually exclusive choices spanning the available width.
inline bool Segmented(const char* id, int* value, std::initializer_list<const char*> labels) {
    ImGui::PushID(id);
    const int count = static_cast<int>(labels.size());
    // The available width, or what the labels need where there is none yet
    // (the first frame of an auto-sizing popup): ImGui rejects empty buttons.
    float natural = 0;
    for (const char* label : labels) natural += ImGui::CalcTextSize(label, nullptr, true).x + 20;
    const float available = ImGui::GetContentRegionAvail().x;
    const float width = available >= natural * .5f && available > count ? available : natural;
    const float height = ImGui::GetFrameHeight();
    const ImVec2 min = ImGui::GetCursorScreenPos();
    auto* draw = ImGui::GetWindowDrawList();
    const auto& palette = Palette();
    draw->AddRectFilled(min, {min.x + width, min.y + height}, ImGui::GetColorU32(ImGuiCol_FrameBg), ControlRounding);
    bool changed = false;
    const float segment = width/count;
    int index = 0;
    for (const char* label : labels) {
        ImGui::SetCursorScreenPos({min.x + index*segment, min.y});
        ImGui::PushID(index);
        if (ImGui::InvisibleButton("segment", {segment, height}) && *value != index) { *value = index; changed = true; }
        const bool hovered = ImGui::IsItemHovered(), selected = *value == index;
        const ImVec2 a{min.x + index*segment + 2, min.y + 2}, b{min.x + (index + 1)*segment - 2, min.y + height - 2};
        if (selected) draw->AddRectFilled(a, b, WithAlpha(palette.accent, CurrentTheme() == ThemeMode::Light ? 70 : 90), ControlRounding - 1);
        else if (hovered) draw->AddRectFilled(a, b, WithAlpha(palette.text, 18), ControlRounding - 1);
        const char* end = ImGui::FindRenderedTextEnd(label);
        const ImVec2 text = ImGui::CalcTextSize(label, end);
        const float x = std::max(a.x + 8, (a.x + b.x - text.x)*.5f);
        draw->PushClipRect(a, b, true);
        draw->AddText({x, min.y + (height - text.y)*.5f}, selected ? palette.text : palette.muted, label, end);
        draw->PopClipRect();
        if (hovered && text.x > segment - 8) ImGui::SetTooltip("%.*s", int(end - label), label);
        ImGui::PopID();
        ++index;
    }
    ImGui::SetCursorScreenPos({min.x, min.y + height});
    ImGui::Dummy({width, 0});
    ImGui::PopID();
    return changed;
}

inline void DrawColorMapBar(ImDrawList* draw, ImVec2 a, ImVec2 b, RayFEM::ColorMap map, bool reverse) {
    constexpr int steps = 24;
    for (int i = 0; i < steps; ++i) {
        const float u0 = float(i)/steps, u1 = float(i + 1)/steps;
        const auto c0 = SampleColorMap(map, u0, reverse), c1 = SampleColorMap(map, u1, reverse);
        const ImU32 left = IM_COL32(c0.r, c0.g, c0.b, 255), right = IM_COL32(c1.r, c1.g, c1.b, 255);
        draw->AddRectFilledMultiColor({a.x + (b.x - a.x)*u0, a.y}, {a.x + (b.x - a.x)*u1 + .5f, b.y}, left, right, right, left);
    }
    draw->AddRect(a, b, WithAlpha(Palette().border, 140), 2);
}

// Color map choice showing each palette's gradient.
inline bool ColorMapPicker(const char* id, int* map, bool reverse) {
    bool changed = false;
    const float barWidth = 64;
    const bool open = ImGui::BeginCombo(id, "", ImGuiComboFlags_CustomPreview);
    if (ImGui::BeginComboPreview()) {
        const ImVec2 min = ImGui::GetItemRectMin();
        const float height = ImGui::GetFrameHeight();
        auto* draw = ImGui::GetWindowDrawList();
        const ImVec2 a{min.x + ImGui::GetStyle().FramePadding.x, min.y + height*.3f};
        DrawColorMapBar(draw, a, {a.x + barWidth, min.y + height*.7f}, static_cast<ColorMap>(*map), reverse);
        ImGui::SetCursorScreenPos({a.x + barWidth + 8, ImGui::GetCursorScreenPos().y});
        ImGui::TextUnformatted(ColorMapName(static_cast<ColorMap>(*map)));
        ImGui::EndComboPreview();
    }
    if (open) {
        for (int i = 0; i < ColorMapCount; ++i) {
            ImGui::PushID(i);
            const float height = ImGui::GetFrameHeight();
            if (ImGui::Selectable("##map", i == *map, 0, {0, height})) { *map = i; changed = true; }
            const ImVec2 min = ImGui::GetItemRectMin();
            auto* draw = ImGui::GetWindowDrawList();
            DrawColorMapBar(draw, {min.x + 4, min.y + height*.3f}, {min.x + 4 + barWidth, min.y + height*.7f}, static_cast<ColorMap>(i), reverse);
            const char* name = ColorMapName(static_cast<ColorMap>(i));
            draw->AddText({min.x + barWidth + 14, min.y + (height - ImGui::GetTextLineHeight())*.5f}, Palette().text, name);
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    return changed;
}

// A timeline scrubber: thin track, accent fill to the current position and a
// round handle. Hovering describes the frame under the cursor.
// marks: indices drawn as small pins above the track (pinned frames).
inline bool Scrubber(const char* id, int* value, int count, float width, const std::function<std::string(int)>& describe,
                     const std::vector<int>* marks = nullptr) {
    const float height = ImGui::GetFrameHeight();
    const ImVec2 min = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id, {width, height});
    const bool hovered = ImGui::IsItemHovered(), active = ImGui::IsItemActive();
    const bool disabled = ImGui::GetItemFlags() & ImGuiItemFlags_Disabled;
    auto* draw = ImGui::GetWindowDrawList();
    const auto& palette = Palette();
    const float knob = 7, left = min.x + knob, right = min.x + width - knob, center = min.y + height*.5f;
    auto fraction = [&](int index) { return count > 1 ? float(index)/(count - 1) : 0.f; };
    auto indexAt = [&](float x) { return count > 1 ? int(std::lround(std::clamp((x - left)/(right - left), 0.f, 1.f)*(count - 1))) : 0; };
    bool changed = false;
    if (active && count > 0 && !disabled) {
        const int index = indexAt(ImGui::GetMousePos().x);
        if (index != *value) { *value = index; changed = true; }
    }
    draw->AddRectFilled({left, center - 2}, {right, center + 2}, ImGui::GetColorU32(ImGuiCol_FrameBg), 2);
    if (count > 1 && count <= 160)
        for (int i = 0; i < count; ++i) {
            const float x = left + (right - left)*fraction(i);
            draw->AddLine({x, center + 5}, {x, center + 8}, WithAlpha(palette.muted, 90));
        }
    if (marks)
        for (int index : *marks) {
            const float mx = left + (right - left)*fraction(std::clamp(index, 0, std::max(0, count - 1)));
            draw->AddTriangleFilled({mx - 4, center - 10}, {mx + 4, center - 10}, {mx, center - 5}, palette.guide);
        }
    const float x = left + (right - left)*fraction(std::clamp(*value, 0, std::max(0, count - 1)));
    if (count > 0) {
        draw->AddRectFilled({left, center - 2}, {x, center + 2}, disabled ? palette.muted : palette.accent, 2);
        draw->AddCircleFilled({x, center}, hovered || active ? knob + 1 : knob, disabled ? palette.muted : palette.accent);
        draw->AddCircleFilled({x, center}, (hovered || active ? knob + 1 : knob) - 3, IM_COL32(255, 255, 255, 235));
    }
    if (hovered && count > 0 && describe) {
        const int index = indexAt(ImGui::GetMousePos().x);
        const float hx = left + (right - left)*fraction(index);
        draw->AddLine({hx, center - 8}, {hx, center + 8}, WithAlpha(palette.text, 120), 1.5f);
        ImGui::SetTooltip("%s", describe(index).c_str());
    }
    return changed;
}

} // namespace RayFEM::UI
