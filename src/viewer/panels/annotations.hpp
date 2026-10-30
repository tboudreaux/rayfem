#pragma once

// Lines and labels placed on the model, for figures of the view and movies.
// A line joins two points picked on the surface; a label is text at one.
// They live in world coordinates, so they stay put across frames; movie
// keyframes record which are visible.

#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "extras/IconsFontAwesome6.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace RayFEM::UI {

struct Annotation {
    enum class Kind { Line, Label };
    uint32_t id = 0;
    Kind kind = Kind::Line;
    Vector3 a{}, b{};      // a line's ends; a label sits at a
    std::string text;      // a label's text; optional for a line (shown at its middle)
    Color color{255, 205, 95, 255};
    bool visible = true;
};

class Annotations {
public:
    enum class Tool { None, Line, Label };

    Tool Placing() const { return tool_; }
    void Place(Tool tool) { tool_ = tool; pending_.reset(); }
    void Cancel() { tool_ = Tool::None; pending_.reset(); }
    const std::vector<Annotation>& Items() const { return items_; }
    std::vector<Annotation>& Items() { return items_; }
    bool Empty() const { return items_.empty(); }

    // A click on the surface while placing; true when it was used.
    bool Click(Vector3 point) {
        if (tool_ == Tool::Label) {
            Annotation label;
            label.id = next_++; label.kind = Annotation::Kind::Label; label.a = label.b = point;
            label.text = "Label " + std::to_string(label.id);
            label.color = ColorFromImU32(Palette().text);
            items_.push_back(label);
            tool_ = Tool::None;
            return true;
        }
        if (tool_ == Tool::Line) {
            if (!pending_) { pending_ = point; return true; }
            Annotation line;
            line.id = next_++; line.kind = Annotation::Kind::Line; line.a = *pending_; line.b = point;
            items_.push_back(line);
            pending_.reset();
            tool_ = Tool::None;
            return true;
        }
        return false;
    }

    // Visible annotations by id, for movie keyframes.
    std::vector<uint32_t> Visible() const {
        std::vector<uint32_t> ids;
        for (const auto& item : items_) if (item.visible) ids.push_back(item.id);
        return ids;
    }
    void SetVisible(const std::vector<uint32_t>& ids) {
        for (auto& item : items_) item.visible = std::find(ids.begin(), ids.end(), item.id) != ids.end();
    }

    // Lines as rods and label points as dots, in a 3D pass; extent sets their
    // thickness relative to the model.
    void Draw3D(float extent) const {
        const float r = std::max(1e-6f, extent * .004f);
        rlDrawRenderBatchActive();
        for (const auto& item : items_) {
            if (!item.visible) continue;
            if (item.kind == Annotation::Kind::Line) {
                DrawCylinderEx(item.a, item.b, r, r, 8, item.color);
                DrawSphereEx(item.a, r * 1.8f, 8, 8, item.color);
                DrawSphereEx(item.b, r * 1.8f, 8, 8, item.color);
            } else {
                DrawSphereEx(item.a, r * 2.2f, 8, 8, item.color);
            }
        }
        if (pending_) DrawSphereEx(*pending_, r * 2.5f, 8, 8, ColorFromImU32(Palette().guide));
        rlDrawRenderBatchActive();
    }

    // Where each visible annotation's text goes on screen: labels beside
    // their point, line text beside the middle.
    struct Text { std::string text; Vector2 at; Color color; };
    std::vector<Text> Texts(const Camera3D& camera, int width, int height) const {
        std::vector<Text> texts;
        const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
        for (const auto& item : items_) {
            if (!item.visible || item.text.empty()) continue;
            const Vector3 at = item.kind == Annotation::Kind::Label ? item.a : Vector3Lerp(item.a, item.b, .5f);
            if (Vector3DotProduct(Vector3Subtract(at, camera.position), forward) <= 0) continue;
            const Vector2 p = GetWorldToScreenEx(at, camera, width, height);
            if (p.x < 0 || p.y < 0 || p.x > width || p.y > height) continue;
            texts.push_back({item.text, p, item.color});
        }
        return texts;
    }
    void DrawTexts(ImDrawList* draw, ImVec2 origin, ImVec2 size, const Camera3D& camera) const {
        for (const auto& t : Texts(camera, int(size.x), int(size.y))) {
            const ImU32 color = IM_COL32(t.color.r, t.color.g, t.color.b, 255);
            const ImVec2 at{origin.x + t.at.x + 8, origin.y + t.at.y - ImGui::GetTextLineHeight() * .5f};
            draw->AddText({at.x + 1, at.y + 1}, IM_COL32(0, 0, 0, 160), t.text.c_str());
            draw->AddText(at, color, t.text.c_str());
        }
    }

    // The list: add, edit, show and remove.
    void Controls() {
        if (tool_ != Tool::None) {
            ImGui::TextColored(ThemeText(Palette().guide), tool_ == Tool::Line
                ? (pending_ ? "Click the line's second point on the surface" : "Click the line's first point on the surface")
                : "Click the label's point on the surface");
            if (ImGui::SmallButton("Cancel (Esc)")) Cancel();
        } else {
            if (ImGui::Button(ICON_FA_PEN "  Line")) Place(Tool::Line);
            Tip("Draw a line between two points you click on the surface.");
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_FONT "  Label")) Place(Tool::Label);
            Tip("Place text at a point you click on the surface.");
        }
        int remove = -1;
        for (size_t i = 0; i < items_.size(); ++i) {
            auto& item = items_[i];
            ImGui::PushID(static_cast<int>(item.id));
            ImGui::Checkbox("##visible", &item.visible);
            Tip("Show or hide; movie keyframes record this");
            ImGui::SameLine();
            float color[3] = {item.color.r / 255.f, item.color.g / 255.f, item.color.b / 255.f};
            if (ImGui::ColorEdit3("##color", color, ImGuiColorEditFlags_NoInputs))
                item.color = {static_cast<unsigned char>(color[0] * 255), static_cast<unsigned char>(color[1] * 255),
                              static_cast<unsigned char>(color[2] * 255), 255};
            ImGui::SameLine();
            char text[256];
            std::snprintf(text, sizeof(text), "%s", item.text.c_str());
            ImGui::SetNextItemWidth(std::max(60.f, ImGui::GetContentRegionAvail().x - ImGui::GetFrameHeight() * 2 - 8));
            if (ImGui::InputTextWithHint("##text", item.kind == Annotation::Kind::Line ? "Line (optional text)" : "Label text",
                                         text, sizeof(text))) item.text = text;
            ImGui::SameLine();
            if (ImGui::Button(ICON_FA_TRASH_CAN)) remove = static_cast<int>(i);
            Tip("Remove");
            if (ImGui::TreeNode(item.kind == Annotation::Kind::Line ? "Ends" : "Point")) {
                ImGui::DragFloat3("##a", &item.a.x, .001f, 0, 0, "%.4g");
                if (item.kind == Annotation::Kind::Line) ImGui::DragFloat3("##b", &item.b.x, .001f, 0, 0, "%.4g");
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        if (remove >= 0) items_.erase(items_.begin() + remove);
        if (items_.empty() && tool_ == Tool::None) ImGui::TextDisabled("Lines and labels show in the view and in movies.");
    }

private:
    static void Tip(const char* text) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", text);
    }
    static Color ColorFromImU32(ImU32 c) {
        return {static_cast<unsigned char>(c & 0xff), static_cast<unsigned char>((c >> 8) & 0xff),
                static_cast<unsigned char>((c >> 16) & 0xff), 255};
    }
    std::vector<Annotation> items_;
    uint32_t next_ = 1;
    Tool tool_ = Tool::None;
    std::optional<Vector3> pending_;
};

} // namespace RayFEM::UI
