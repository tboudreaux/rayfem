#pragma once

#include "libRayfem/frame/render.hpp"
#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "extras/IconsFontAwesome6.h"
#include "imgui.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
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

// Text with {name} filled in from a frame, for annotations and movie HUDs.
// A name is one of the frame's schema values or frame, time, label, tags or
// caption; {name:.3f} formats a number printf-style (e, f, g; d for whole
// numbers). Values carry no unit. A value the frame lacks shows as "–";
// unknown names and malformed fields stay as typed. {{ and }} are braces.
inline std::vector<std::string> FrameTextBuiltins() { return {"frame", "time", "label", "tags", "caption"}; }

namespace Detail {
inline bool NumberSpec(std::string_view spec) {
    size_t i = 0;
    while (i < spec.size() && std::string_view("-+ #0").find(spec[i]) != std::string_view::npos) ++i;
    while (i < spec.size() && std::isdigit(static_cast<unsigned char>(spec[i]))) ++i;
    if (i < spec.size() && spec[i] == '.') {
        ++i;
        while (i < spec.size() && std::isdigit(static_cast<unsigned char>(spec[i]))) ++i;
    }
    return spec.size() == i + 1 && std::string_view("eEfFgGd").find(spec[i]) != std::string_view::npos && spec.size() < 16;
}
inline std::string FormatNumber(double value, std::string_view spec) {
    char text[64];
    if (spec.back() == 'd') {
        const std::string format = "%" + std::string(spec.substr(0, spec.size() - 1)) + "lld";
        std::snprintf(text, sizeof(text), format.c_str(), static_cast<long long>(std::llround(value)));
    } else {
        const std::string format = "%" + std::string(spec);
        std::snprintf(text, sizeof(text), format.c_str(), value);
    }
    return text;
}
// One field's text; nullopt keeps the field as typed.
inline std::optional<std::string> FrameField(const RenderFrame* frame, std::string_view name, std::string_view spec) {
    std::optional<double> number;
    std::string text;
    const auto builtins = FrameTextBuiltins();
    const bool builtin = std::find(builtins.begin(), builtins.end(), name) != builtins.end();
    const bool known = builtin || (frame && frame->schema && frame->schema->find(name));
    if (!known) return std::nullopt;
    if (!frame) return "–";
    const auto& m = frame->metadata;
    if (name == "frame") number = m.frame_index;
    else if (name == "time") number = m.sim_time;
    else if (name == "label") text = m.label;
    else if (name == "tags") { for (const auto& t : m.tags) text += (text.empty() ? "" : ", ") + t; }
    else if (name == "caption") text = FrameCaption(*frame, false);
    else if (const Value* value = m.find(name)) {
        if (const auto* i = std::get_if<int64_t>(value)) number = static_cast<double>(*i);
        else if (const auto* d = std::get_if<double>(value)) number = *d;
        else text = std::get<std::string>(*value);
        if (spec.empty()) return FormatValue(*value);
    } else return "–";
    if (!number) return text;
    if (!spec.empty()) return FormatNumber(*number, spec);
    char plain[64];
    std::snprintf(plain, sizeof(plain), "%.6g", *number);
    return plain;
}
} // namespace Detail

inline std::string ExpandFrameText(std::string_view text, const RenderFrame* frame) {
    std::string out;
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if ((c == '{' || c == '}') && i + 1 < text.size() && text[i + 1] == c) { out += c; ++i; continue; }
        const size_t close = c == '{' ? text.find('}', i + 1) : std::string_view::npos;
        if (close == std::string_view::npos) { out += c; continue; }
        std::string_view field = text.substr(i + 1, close - i - 1), spec;
        if (const size_t colon = field.rfind(':'); colon != std::string_view::npos && Detail::NumberSpec(field.substr(colon + 1))) {
            spec = field.substr(colon + 1);
            field = field.substr(0, colon);
        }
        if (const auto value = field.empty() ? std::nullopt : Detail::FrameField(frame, field, spec)) {
            out += *value;
            i = close;
        } else out += c;
    }
    return out;
}

// The names ExpandFrameText knows for a frame: its schema's values, then the
// built-ins.
inline std::vector<std::string> FrameTextFields(const RenderFrame* frame) {
    std::vector<std::string> names;
    if (frame && frame->schema) for (const auto& v : frame->schema->values) names.push_back(v.name);
    for (auto& name : FrameTextBuiltins()) names.push_back(std::move(name));
    return names;
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
