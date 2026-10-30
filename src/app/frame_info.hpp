#pragma once

#include "libRayfem/frame/render.hpp"
#include "theme.hpp"
#include "widgets.hpp"
#include "extras/IconsFontAwesome6.h"
#include "imgui.h"

#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

// What a client says about its frames: captions from its schema, a filter
// over tags and coordinates, and the panel that shows a frame's values.
namespace RayFEM::UI {

inline std::string FormatValue(const Value& value, const ValueField* field = nullptr) {
    char text[64];
    std::string out;
    if (const auto* i = std::get_if<int64_t>(&value)) out = std::to_string(*i);
    else if (const auto* d = std::get_if<double>(&value)) { std::snprintf(text, sizeof(text), "%.6g", *d); out = text; }
    else out = std::get<std::string>(value);
    if (field && !field->unit.empty()) out += " " + field->unit;
    return out;
}

// The coordinate a frame is labelled by: the schema's primary, if the frame
// carries it.
inline const ValueField* PrimaryCoordinate(const RenderFrame& frame) {
    if (!frame.schema || frame.schema->primary.empty()) return nullptr;
    const auto* field = frame.schema->find(frame.schema->primary);
    return field && frame.metadata.find(field->name) ? field : nullptr;
}
inline bool HasTime(const RenderFrame& frame) { return !frame.schema || frame.schema->has_time; }

// "iteration 12 · t 0.4" or "Frame 8 · t 0.4", plus tags.
inline std::string FrameCaption(const RenderFrame& frame, bool withTags = true) {
    std::string caption;
    if (const auto* primary = PrimaryCoordinate(frame)) caption = primary->name + " " + FormatValue(*frame.metadata.find(primary->name), primary);
    else caption = "Frame " + std::to_string(frame.metadata.frame_index);
    if (HasTime(frame)) {
        char time[48];
        std::snprintf(time, sizeof(time), "  ·  t %.6g", frame.metadata.sim_time);
        caption += time;
    }
    if (withTags) for (const auto& tag : frame.metadata.tags) caption += "  ·  " + tag;
    return caption;
}

// Frames shown on the timeline: any of the chosen tags, and every chosen
// coordinate inside its range. With nothing chosen every frame passes.
struct FrameFilter {
    std::set<std::string> tags;
    std::map<std::string, std::pair<double, double>> ranges;
    bool discard = false;  // drop non-matching frames as they arrive

    bool Active() const { return !tags.empty() || !ranges.empty(); }
    bool Matches(const RenderFrame& frame) const {
        if (!tags.empty() && std::none_of(tags.begin(), tags.end(), [&](const auto& t) { return frame.metadata.hasTag(t); })) return false;
        for (const auto& [name, range] : ranges) {
            const auto value = frame.metadata.number(name);
            if (!value || *value < range.first || *value > range.second) return false;
        }
        return true;
    }
};

// Tags and coordinates seen across retained frames, with value ranges.
struct FrameVocabulary {
    std::vector<TagField> tags;
    std::vector<ValueField> coordinates;
    std::map<std::string, std::pair<double, double>> extents;
    template<class Frames> static FrameVocabulary From(const Frames& frames) {
        FrameVocabulary vocabulary;
        std::set<std::string> tags, coordinates;
        for (const auto& frame : frames) {
            if (!frame || !frame->schema) continue;
            for (const auto& t : frame->schema->tags) if (tags.insert(t.name).second) vocabulary.tags.push_back(t);
            for (const auto& v : frame->schema->values) {
                if (!v.coordinate) continue;
                if (coordinates.insert(v.name).second) vocabulary.coordinates.push_back(v);
                if (const auto value = frame->metadata.number(v.name)) {
                    auto [it, added] = vocabulary.extents.try_emplace(v.name, *value, *value);
                    it->second.first = std::min(it->second.first, *value);
                    it->second.second = std::max(it->second.second, *value);
                }
            }
        }
        return vocabulary;
    }
};

// The timeline's filter popup. Returns true when the filter changed.
template<class Frames>
bool DrawFrameFilter(FrameFilter& filter, const Frames& frames, std::size_t shown) {
    bool changed = false;
    const auto vocabulary = FrameVocabulary::From(frames);
    ImGui::TextDisabled("Showing %zu of %zu retained frames", shown, frames.size());
    if (vocabulary.tags.empty() && vocabulary.coordinates.empty()) {
        ImGui::Spacing();
        ImGui::PushTextWrapPos(ImGui::GetFontSize()*22);
        ImGui::TextUnformatted("These frames carry no tags or coordinates. Clients declare them with Client::registerSchema.");
        ImGui::PopTextWrapPos();
        return false;
    }
    if (!vocabulary.tags.empty()) {
        ImGui::SeparatorText("Tags");
        ImGui::TextDisabled("Show frames with any checked tag");
        for (const auto& tag : vocabulary.tags) {
            bool on = filter.tags.contains(tag.name);
            if (ImGui::Checkbox(tag.name.c_str(), &on)) {
                if (on) filter.tags.insert(tag.name); else filter.tags.erase(tag.name);
                changed = true;
            }
            if (!tag.description.empty() && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", tag.description.c_str());
        }
    }
    if (!vocabulary.coordinates.empty()) {
        ImGui::SeparatorText("Ranges");
        for (const auto& coordinate : vocabulary.coordinates) {
            const auto extent = vocabulary.extents.contains(coordinate.name) ? vocabulary.extents.at(coordinate.name) : std::pair{0.0, 1.0};
            const auto found = filter.ranges.find(coordinate.name);
            bool on = found != filter.ranges.end();
            ImGui::PushID(coordinate.name.c_str());
            if (ImGui::Checkbox(coordinate.name.c_str(), &on)) {
                if (on) filter.ranges[coordinate.name] = extent; else filter.ranges.erase(coordinate.name);
                changed = true;
            }
            if (on) {
                auto& range = filter.ranges[coordinate.name];
                float low = float(range.first), high = float(range.second);
                const float speed = float(std::max(1e-9, (extent.second - extent.first)/200));
                ImGui::SetNextItemWidth(ImGui::GetFontSize()*16);
                const char* format = coordinate.type == ValueType::Integer ? "%.0f" : "%.4g";
                if (ImGui::DragFloatRange2("##range", &low, &high, speed, 0, 0, format, format)) {
                    range = {low, high};
                    changed = true;
                }
            }
            ImGui::PopID();
        }
    }
    ImGui::Separator();
    changed |= Switch("Discard others as they arrive", &filter.discard);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
        ImGui::SetTooltip("Keep only matching frames in the history, so frames you do not want never use memory.");
    if (filter.Active() && ImGui::Button("Clear filter", {-1, 0})) { filter.tags.clear(); filter.ranges.clear(); changed = true; }
    return changed;
}

// Values and tags of the shown frame. track(name) adds a numeric value to
// the tracks plot.
inline void DrawFrameDetails(const std::shared_ptr<RenderFrame>& frame, const std::function<void(const std::string&)>& track) {
    const bool described = frame && (!frame->metadata.values.empty() || !frame->metadata.tags.empty());
    if (!BeginSection("Frame", described ? frame->metadata.tags.empty() ? nullptr : frame->metadata.tags.front().c_str() : nullptr)) return;
    if (!frame) { ImGui::TextDisabled("No frame"); EndSection(); return; }
    ImGui::TextUnformatted(FrameCaption(*frame, false).c_str());
    if (!frame->metadata.tags.empty()) {
        // Tags as small chips.
        auto* draw = ImGui::GetWindowDrawList();
        for (const auto& tag : frame->metadata.tags) {
            const ImVec2 size = ImGui::CalcTextSize(tag.c_str());
            NextInlineItem(size.x + 14);
            const ImVec2 at = ImGui::GetCursorScreenPos();
            draw->AddRectFilled(at, {at.x + size.x + 14, at.y + size.y + 6}, WithAlpha(Palette().accent, 45), (size.y + 6)*.5f);
            draw->AddText({at.x + 7, at.y + 3}, Palette().text, tag.c_str());
            ImGui::Dummy({size.x + 14, size.y + 6});
            if (frame->schema && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                for (const auto& t : frame->schema->tags) if (t.name == tag && !t.description.empty()) ImGui::SetTooltip("%s", t.description.c_str());
        }
    }
    if (frame->metadata.values.empty()) {
        if (frame->metadata.tags.empty())
            ImGui::TextDisabled("No values. Clients send them after Client::registerSchema.");
        EndSection();
        return;
    }
    ImGui::Spacing();
    if (ImGui::BeginTable("frame values", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthStretch, 1.1f);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 1.2f);
        ImGui::TableSetupColumn("track", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
        for (const auto& value : frame->metadata.values) {
            const ValueField* field = frame->schema ? frame->schema->find(value.name) : nullptr;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("%s", value.name.c_str());
            if (field && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                ImGui::BeginTooltip();
                ImGui::Text("%s%s", field->coordinate ? "Coordinate" : "Value", field->type == ValueType::Integer ? " (integer)" :
                                                                                field->type == ValueType::Real ? " (real)" : " (text)");
                if (!field->description.empty()) { ImGui::PushTextWrapPos(ImGui::GetFontSize()*22); ImGui::TextUnformatted(field->description.c_str()); ImGui::PopTextWrapPos(); }
                ImGui::EndTooltip();
            }
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextWrapped("%s", FormatValue(value.value, field).c_str());
            ImGui::TableNextColumn();
            if (!std::holds_alternative<std::string>(value.value)) {
                ImGui::PushID(value.name.c_str());
                if (ImGui::SmallButton(ICON_FA_CHART_LINE)) track(value.name);
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("Track %s across frames", value.name.c_str());
                ImGui::PopID();
            }
        }
        ImGui::EndTable();
    }
    EndSection();
}

} // namespace RayFEM::UI
