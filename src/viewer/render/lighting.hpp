#pragma once

// How the surface is lit: a shading model and one directional light, either
// carried with the camera or fixed in the scene. The surface shader reads
// these as uniforms; lines, markers and labels stay unlit.

#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>

namespace RayFEM::UI {

enum class Shading { Unlit, Matte, Faceted, Glossy };
inline constexpr Shading Shadings[] = {Shading::Unlit, Shading::Matte, Shading::Faceted, Shading::Glossy};
inline const char* ShadingName(Shading s) {
    switch (s) {
        case Shading::Unlit: return "Flat color";
        case Shading::Matte: return "Matte";
        case Shading::Faceted: return "Faceted";
        case Shading::Glossy: return "Glossy";
    }
    return "";
}
inline const char* ShadingDescription(Shading s) {
    switch (s) {
        case Shading::Unlit: return "No shading: every point shows exactly its colormap color, so colors read as values.";
        case Shading::Matte: return "Diffuse light on smoothly varying normals: curved surfaces look round.";
        case Shading::Faceted: return "Diffuse light with one normal per drawn triangle, showing the tessellation.";
        case Shading::Glossy: return "Matte plus a specular highlight that reflects the light toward the viewer.";
    }
    return "";
}

// The camera's right, up and backward (toward the viewer) directions.
struct ViewFrame { Vector3 right, up, back; };
inline ViewFrame FrameOf(const Camera3D& camera) {
    const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    Vector3 right = Vector3CrossProduct(forward, camera.up);
    right = Vector3Length(right) > 1e-6f ? Vector3Normalize(right) : Vector3{1, 0, 0};
    return {right, Vector3CrossProduct(right, forward), Vector3Negate(forward)};
}

struct Lighting {
    Shading shading = Shading::Matte;
    bool followView = false;
    // Toward the light: in the scene while fixed, or as right and up of the
    // view (the rest points at the viewer) while following it.
    Vector3 world = Vector3Normalize({.4f, .6f, .7f});
    Vector2 view{-.35f, .45f};
    float ambient = .45f, specular = .35f, shininess = 40;

    static Vector3 FromView(const ViewFrame& f, Vector2 v) {
        const float z = std::sqrt(std::max(0.f, 1 - v.x*v.x - v.y*v.y));
        return Vector3Normalize(Vector3Add(Vector3Add(Vector3Scale(f.right, v.x), Vector3Scale(f.up, v.y)), Vector3Scale(f.back, z)));
    }
    // Toward the light in the scene, for a camera.
    Vector3 Direction(const Camera3D& camera) const { return followView ? FromView(FrameOf(camera), view) : world; }

    // The inspector's controls; camera places the pad's view of a fixed light.
    void Controls(const Camera3D& camera) {
        FieldLabel("Shading");
        if (ImGui::BeginCombo("##shading", ShadingName(shading))) {
            for (const Shading s : Shadings) {
                if (ImGui::Selectable(ShadingName(s), s == shading)) shading = s;
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", ShadingDescription(s));
            }
            ImGui::EndCombo();
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", ShadingDescription(shading));
        if (shading == Shading::Unlit) return;

        const ViewFrame frame = FrameOf(camera);
        int attached = followView ? 0 : 1;
        FieldLabel("Light");
        if (Segmented("##light_frame", &attached, {"With view", "In scene"})) {
            // Keep the light where it is on screen.
            if (attached == 1) world = FromView(frame, view);
            else view = {Vector3DotProduct(world, frame.right), Vector3DotProduct(world, frame.up)};
            followView = attached == 0;
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
            ImGui::SetTooltip("With the view, the light keeps its place relative to the screen as you orbit. "
                              "In the scene, it stays put and the lit side turns with the mesh.");
        // The pad shows the light as seen from the camera.
        Vector2 shown = view;
        bool behind = false;
        if (!followView) {
            shown = {Vector3DotProduct(world, frame.right), Vector3DotProduct(world, frame.up)};
            behind = Vector3DotProduct(world, frame.back) < 0;
        }
        const float size = std::min(150.f, ImGui::GetContentRegionAvail().x*.5f);
        ImGui::BeginGroup();
        if (DirectionPad("##light_pad", &shown.x, &shown.y, size, behind)) {
            if (followView) view = shown;
            else world = FromView(frame, shown);
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
            ImGui::SetTooltip("Drag the light. The center shines from the camera, the rim from the side; a hollow marker is behind the mesh.");
        ImGui::EndGroup();
        ImGui::SameLine();
        ImGui::BeginGroup();
        const float width = ImGui::GetContentRegionAvail().x;
        auto place = [&](const char* label, Vector2 at) {
            if (ImGui::Button(label, {width, 0})) { if (followView) view = at; else world = FromView(frame, at); }
        };
        place("From camera", {0, 0});
        place("Upper left", {-.45f, .5f});
        place("Upper right", {.45f, .5f});
        place("Above", {0, .8f});
        if (ImGui::Button("Reset", {width, 0})) *this = Lighting{.shading = shading};
        ImGui::EndGroup();

        FieldLabel("Ambient");
        ImGui::SliderFloat("##ambient", &ambient, 0, 1, "%.2f", ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
            ImGui::SetTooltip("Light reaching every surface evenly. 1 is the same as Flat color; 0 leaves the far side black.");
        if (shading == Shading::Glossy) {
            FieldLabel("Highlight");
            ImGui::SliderFloat("##specular", &specular, 0, 1, "%.2f", ImGuiSliderFlags_AlwaysClamp);
            FieldLabel("Sharpness");
            ImGui::SliderFloat("##shininess", &shininess, 2, 256, "%.0f", ImGuiSliderFlags_Logarithmic | ImGuiSliderFlags_AlwaysClamp);
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                ImGui::SetTooltip("Higher values give a smaller, sharper highlight, as on polished metal.");
        }
    }
};

// Uniform locations for the lighting in the surface shader.
struct LightingUniforms {
    int direction = -1, eye = -1, shading = -1, ambient = -1, specular = -1, shininess = -1;
    void Locate(const Shader& shader) {
        direction = GetShaderLocation(shader, "lightDirection");
        eye = GetShaderLocation(shader, "eyePosition");
        shading = GetShaderLocation(shader, "shading");
        ambient = GetShaderLocation(shader, "ambient");
        specular = GetShaderLocation(shader, "specular");
        shininess = GetShaderLocation(shader, "shininess");
    }
    void Apply(const Shader& shader, const Lighting& lighting, const Camera3D& camera) const {
        const Vector3 direction = lighting.Direction(camera);
        const int mode = static_cast<int>(lighting.shading);
        const float specularStrength = lighting.shading == Shading::Glossy ? lighting.specular : 0;
        SetShaderValue(shader, this->direction, &direction, SHADER_UNIFORM_VEC3);
        SetShaderValue(shader, eye, &camera.position, SHADER_UNIFORM_VEC3);
        SetShaderValue(shader, shading, &mode, SHADER_UNIFORM_INT);
        SetShaderValue(shader, ambient, &lighting.ambient, SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader, specular, &specularStrength, SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader, shininess, &lighting.shininess, SHADER_UNIFORM_FLOAT);
    }
};

} // namespace RayFEM::UI
