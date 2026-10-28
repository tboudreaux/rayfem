#pragma once

#include "camera_controller.hpp"
#include "theme.hpp"
#include "extras/IconsFontAwesome6.h"
#include "imgui.h"
#include "libRayfem/frame/render.hpp"
#include "libRayfem/mesh/converter.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace RayFEM::UI {

// A ray endpoint fixed to the material: an element and a point in its
// reference cell. Its position is re-evaluated on every frame, so the ray
// follows displacement and deformation instead of floating in space.
struct RayAnchor {
    int element = -1;
    mfem::IntegrationPoint point{};
    bool Valid() const { return element >= 0; }
};

// Displayed position (displacement included, element explosion excluded).
inline std::optional<Vector3> AnchorPosition(mfem::Mesh& mesh, const RayAnchor& anchor, const RenderOptions& options) {
    if (!anchor.Valid() || anchor.element >= mesh.GetNE()) return std::nullopt;
    auto& transform = *mesh.GetElementTransformation(anchor.element);
    mfem::Vector x;
    transform.Transform(anchor.point, x);
    if (options.displacement && options.displacement_scale != 0) {
        mfem::Vector d;
        transform.SetIntPoint(&anchor.point);
        options.displacement->GetVectorValue(transform, anchor.point, d);
        x.Add(options.displacement_scale, d);
    }
    for (int i = 0; i < x.Size(); ++i) if (!std::isfinite(x[i])) return std::nullopt;
    return Vector3{float(x[0]), x.Size() > 1 ? float(x[1]) : 0, x.Size() > 2 ? float(x[2]) : 0};
}

// Every face of one element (the element itself in 2D), refined, with the
// reference point behind each vertex so picks map back to exact coordinates.
struct ElementSurface {
    std::vector<Vector3> positions;
    std::vector<mfem::IntegrationPoint> points;
    std::vector<double> values;
    std::vector<std::array<int, 3>> triangles;
    std::vector<std::vector<Vector3>> outline;
    BoundingBox bounds{};
};

inline ElementSurface BuildElementSurface(mfem::Mesh& mesh, int element, mfem::GridFunction* field,
                                          const RenderOptions& options, int refinement) {
    ElementSurface result;
    if (element < 0 || element >= mesh.GetNE()) throw std::out_of_range("Element is not in the displayed mesh");
    refinement = std::clamp(refinement, 2, 24);
    mfem::GeometryRefiner refiner;
    std::vector<std::vector<mfem::IntegrationPoint>> outlinePoints;
    // Each patch is a face geometry plus how its reference points map into the
    // element. The map is used immediately: MFEM reuses face transformations.
    auto addPatch = [&](mfem::Geometry::Type geometry, const std::function<mfem::IntegrationPoint(const mfem::IntegrationPoint&)>& toElement) {
        const auto* refined = refiner.Refine(geometry, refinement, 1);
        const int base = static_cast<int>(result.points.size());
        for (int i = 0; i < refined->RefPts.GetNPoints(); ++i) result.points.push_back(toElement(refined->RefPts.IntPoint(i)));
        const int stride = mfem::Geometry::NumVerts[geometry];
        for (int j = 0; j < refined->RefGeoms.Size(); j += stride) {
            result.triangles.push_back({base + refined->RefGeoms[j], base + refined->RefGeoms[j+1], base + refined->RefGeoms[j+2]});
            if (stride == 4) result.triangles.push_back({base + refined->RefGeoms[j], base + refined->RefGeoms[j+2], base + refined->RefGeoms[j+3]});
        }
        // Face edges, sampled finely enough to follow curved geometry.
        const auto* vertices = mfem::Geometries.GetVertices(geometry);
        const int corners = vertices->GetNPoints();
        for (int c = 0; c < corners; ++c) {
            const auto& a = vertices->IntPoint(c);
            const auto& b = vertices->IntPoint((c + 1) % corners);
            std::vector<mfem::IntegrationPoint> edge;
            for (int k = 0; k <= refinement; ++k) {
                const double t = double(k)/refinement;
                mfem::IntegrationPoint p;
                p.Set3(a.x + (b.x - a.x)*t, a.y + (b.y - a.y)*t, a.z + (b.z - a.z)*t);
                edge.push_back(toElement(p));
            }
            result.outline.emplace_back(edge.size());
            outlinePoints.push_back(std::move(edge));
        }
    };
    if (mesh.Dimension() == 2) {
        addPatch(mesh.GetElementBaseGeometry(element), [](const mfem::IntegrationPoint& p) { return p; });
    } else {
        mfem::Array<int> faces, orientations;
        mesh.GetElementFaces(element, faces, orientations);
        for (int face : faces) {
            auto* tr = mesh.GetFaceElementTransformations(face, mfem::FaceElementTransformations::HAVE_LOC1 |
                                                                mfem::FaceElementTransformations::HAVE_LOC2);
            if (!tr) continue;
            mfem::IntegrationPointTransformation* loc = tr->Elem1No == element ? &tr->Loc1 :
                (tr->Elem2No == element ? &tr->Loc2 : nullptr);
            if (!loc) continue;
            addPatch(mesh.GetFaceGeometry(face), [&](const mfem::IntegrationPoint& p) {
                mfem::IntegrationPoint out; loc->Transform(p, out); return out;
            });
        }
    }
    // Positions and values after all face queries; they reuse the element map.
    for (const auto& p : result.points) {
        RayAnchor anchor{element, p};
        const auto position = AnchorPosition(mesh, anchor, options);
        if (!position) throw std::runtime_error("Element geometry is not finite");
        result.positions.push_back(*position);
        result.values.push_back(field ? EvaluateField(*field, element, p, options.view, options.component) : 0);
    }
    for (std::size_t e = 0; e < outlinePoints.size(); ++e)
        for (std::size_t k = 0; k < outlinePoints[e].size(); ++k)
            result.outline[e][k] = AnchorPosition(mesh, {element, outlinePoints[e][k]}, options).value_or(Vector3{});
    if (!result.positions.empty()) {
        result.bounds = {result.positions[0], result.positions[0]};
        for (const auto& p : result.positions) {
            result.bounds.min = Vector3Min(result.bounds.min, p);
            result.bounds.max = Vector3Max(result.bounds.max, p);
        }
    }
    return result;
}

// Ray and triangle (Möller–Trumbore); barycentric weights of the hit.
inline std::optional<std::pair<float, Vector2>> IntersectTriangle(const Ray& ray, Vector3 a, Vector3 b, Vector3 c) {
    const Vector3 e1 = Vector3Subtract(b, a), e2 = Vector3Subtract(c, a);
    const Vector3 p = Vector3CrossProduct(ray.direction, e2);
    const float det = Vector3DotProduct(e1, p);
    if (std::abs(det) < 1e-12f) return std::nullopt;
    const Vector3 t = Vector3Subtract(ray.position, a);
    const float u = Vector3DotProduct(t, p)/det;
    if (u < 0 || u > 1) return std::nullopt;
    const Vector3 q = Vector3CrossProduct(t, e1);
    const float v = Vector3DotProduct(ray.direction, q)/det;
    if (v < 0 || u + v > 1) return std::nullopt;
    const float distance = Vector3DotProduct(e2, q)/det;
    if (distance <= 0) return std::nullopt;
    return std::pair{distance, Vector2{u, v}};
}

// The nearest surface point under a ray, as an anchor on this element.
inline std::optional<RayAnchor> PickElementSurface(const ElementSurface& surface, int element, const Ray& ray) {
    std::optional<RayAnchor> best;
    float nearest = std::numeric_limits<float>::infinity();
    for (const auto& t : surface.triangles) {
        const auto hit = IntersectTriangle(ray, surface.positions[t[0]], surface.positions[t[1]], surface.positions[t[2]]);
        if (!hit || hit->first >= nearest) continue;
        nearest = hit->first;
        const auto [u, v] = hit->second;
        const auto& a = surface.points[t[0]]; const auto& b = surface.points[t[1]]; const auto& c = surface.points[t[2]];
        RayAnchor anchor{element, {}};
        // Reference coordinates are linear on each refined sub-triangle.
        anchor.point.Set3(a.x + (b.x - a.x)*u + (c.x - a.x)*v, a.y + (b.y - a.y)*u + (c.y - a.y)*v,
                          a.z + (b.z - a.z)*u + (c.z - a.z)*v);
        best = anchor;
    }
    return best;
}

// Two-step placement: choose an element in the main view, then place the
// endpoint on that element's surface in an inset viewer. Start, then end.
class RayPlacement {
public:
    enum class Step { Idle, ChooseElement, PlacePoint };
    using ColorFor = std::function<Color(double)>;
    // Draws the rest of the mesh faintly with the inset camera, for orientation.
    using DrawContext = std::function<void(const Camera3D&, Vector2 viewport)>;

    Step Current() const { return step; }
    bool Active() const { return step != Step::Idle; }
    // The main view's element clicks go here while placing.
    bool WantsElement() const { return Active(); }
    int Endpoint() const { return endpoint; }
    const RayAnchor& Anchor(int which) const { return anchors[which]; }

    void Begin() { Close(); anchors = {}; endpoint = 0; single = false; step = Step::ChooseElement; }
    // Re-place one endpoint, opening its element directly.
    void Edit(int which, const std::array<RayAnchor, 2>& current) {
        Close(); anchors = current; endpoint = which; single = true;
        step = Step::ChooseElement;
        if (anchors[which].Valid()) { pending = anchors[which]; requestedElement = anchors[which].element; }
    }
    void Cancel() { Close(); step = Step::Idle; }
    // An element is opened on the next Draw, which has the frame and colors.
    void ChooseElement(int element) { requestedElement = element; pending.reset(); }

    // Main view overlay rectangle this window covers, for input routing.
    bool Covers(Vector2 mouse) const {
        return step == Step::PlacePoint && mouse.x >= windowMin.x && mouse.y >= windowMin.y && mouse.x < windowMax.x && mouse.y < windowMax.y;
    }

    // Draws the inset window when an element is open. Returns both anchors
    // once placement is complete. Main thread, between ImGui frames' windows.
    std::optional<std::array<RayAnchor, 2>> Draw(const std::shared_ptr<RenderFrame>& frame, mfem::GridFunction* field,
                                                 const RenderOptions& options, int refinement, ImVec2 viewportMin, ImVec2 viewportSize,
                                                 const Shader& shader, const ColorFor& color, const Camera3D& mainCamera,
                                                 const DrawContext& context = {}) {
        if (!Active()) return std::nullopt;
        viewDirection = Vector3Subtract(mainCamera.target, mainCamera.position);
        viewUp = mainCamera.up;
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) { Cancel(); return std::nullopt; }
        if (requestedElement >= 0 && frame && frame->mesh) {
            Open(*frame, requestedElement, field, options, refinement, shader, color);
            requestedElement = -1;
        }
        if (step != Step::PlacePoint) { windowMin = windowMax = {}; return std::nullopt; }
        // Keep the geometry current while frames stream or the mesh deforms.
        if (frame && frame.get() != openedFrameRaw && frame->mesh == openedMesh) Rebuild(*frame, field, options, refinement, shader, color);

        const auto& palette = Palette();
        // The window keeps its size and place between the start and end steps.
        if (windowSize.x <= 0) {
            windowSize = {std::min(440.0f, viewportSize.x*.45f), std::min(500.0f, viewportSize.y - 24)};
            windowPosition = {viewportMin.x + viewportSize.x - windowSize.x - 12, viewportMin.y + 12};
        }
        if (resize) {
            const bool expand = !expanded;
            if (expand) { restoreSize = windowSize; restorePosition = windowPosition;
                          windowSize = {viewportSize.x*.7f, viewportSize.y - 24};
                          windowPosition = {viewportMin.x + viewportSize.x*.3f - 12, viewportMin.y + 12}; }
            else { windowSize = restoreSize; windowPosition = restorePosition; }
            expanded = expand;
            ImGui::SetNextWindowPos(windowPosition, ImGuiCond_Always);
            ImGui::SetNextWindowSize(windowSize, ImGuiCond_Always);
            resize = false;
        } else {
            ImGui::SetNextWindowPos(windowPosition, ImGuiCond_Appearing);
            ImGui::SetNextWindowSize(windowSize, ImGuiCond_Appearing);
        }
        ImGui::SetNextWindowSizeConstraints({300, 320}, {std::max(300.0f, viewportSize.x), std::max(320.0f, viewportSize.y)});
        const std::string title = std::string(endpoint == 0 ? "Ray start" : "Ray end") + "  ·  element " + std::to_string(element) + "###ray placement";
        std::optional<std::array<RayAnchor, 2>> result;
        bool open = true;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, OverlayRounding);
        // A visible grip: the theme's default blends into the window border.
        ImGui::PushStyleColor(ImGuiCol_ResizeGrip, WithAlpha(Palette().accent, 110));
        const bool visible = ImGui::Begin(title.c_str(), &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                                            ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
        if (visible) {
            windowPosition = ImGui::GetWindowPos();
            windowSize = ImGui::GetWindowSize();
            windowMin = {windowPosition.x, windowPosition.y};
            windowMax = {windowMin.x + windowSize.x, windowMin.y + windowSize.y};
            const char* expandLabel = expanded ? ICON_FA_DOWN_LEFT_AND_UP_RIGHT_TO_CENTER "###expand" : ICON_FA_UP_RIGHT_AND_DOWN_LEFT_FROM_CENTER "###expand";
            const float expandWidth = ImGui::GetFrameHeight();
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - expandWidth - ImGui::GetStyle().ItemSpacing.x);
            ImGui::TextDisabled("Click the surface to place the %s. Drag to orbit, scroll to zoom. The faint lines are the rest of the mesh.",
                                endpoint == 0 ? "start" : "end");
            ImGui::PopTextWrapPos();
            ImGui::SameLine(ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - expandWidth);
            if (ImGui::Button(expandLabel, {expandWidth, 0})) resize = true;
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip(expanded ? "Restore size" : "Enlarge. Drag any edge or the corner to resize.");
            const float footer = ImGui::GetFrameHeightWithSpacing() + ImGui::GetTextLineHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
            const ImVec2 view{ImGui::GetContentRegionAvail().x, std::max(120.0f, ImGui::GetContentRegionAvail().y - footer)};
            DrawView(view, palette, context);
            if (pending) {
                ImGui::Text("Position %.5g, %.5g, %.5g", pendingPosition.x, pendingPosition.y, pendingPosition.z);
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                    ImGui::SetTooltip("Reference coordinates %.4f, %.4f, %.4f in element %d.\nThe endpoint stays at this material point as the mesh moves.",
                                      pending->point.x, pending->point.y, pending->point.z, element);
            } else ImGui::TextDisabled("No point placed yet");
            const float third = (ImGui::GetContentRegionAvail().x - 2*ImGui::GetStyle().ItemSpacing.x)/3;
            ImGui::BeginDisabled(!pending);
            const bool confirm = ToggleButton(ICON_FA_CHECK "  Use point", true, {third, 0}) || (pending && ImGui::IsKeyPressed(ImGuiKey_Enter, false));
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Other element", {third, 0})) { Close(); step = Step::ChooseElement; }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("Choose a different element in the main view. You can also click one there directly.");
            ImGui::SameLine();
            if (ImGui::Button("Cancel", {third, 0})) open = false;
            if (confirm && pending) {
                anchors[endpoint] = *pending;
                Close();
                if (!single && endpoint == 0) { endpoint = 1; step = Step::ChooseElement; }
                else { step = Step::Idle; result = anchors; }
            }
        }
        ImGui::End();
        if (!open) Cancel();
        return result;
    }

    // Marks already-placed endpoints in the main view.
    template<class Project> void DrawMainMarkers(ImDrawList* draw, mfem::Mesh& mesh, const RenderOptions& options, Project&& project) const {
        if (!Active()) return;
        for (int i = 0; i < 2; ++i) {
            if (!anchors[i].Valid() || i == endpoint) continue;
            const auto position = AnchorPosition(mesh, anchors[i], options);
            ImVec2 screen;
            if (!position || !project(*position, screen)) continue;
            draw->AddCircleFilled(screen, 6, Palette().guide);
            draw->AddCircle(screen, 9, Palette().handle_outline, 0, 2);
            draw->AddText({screen.x + 12, screen.y - 8}, Palette().guide_text, i == 0 ? "Start" : "End");
        }
    }

    void Shutdown() { Close(); }

private:
    void Close() {
        if (model.meshCount) UnloadModel(model);
        model = {};
        if (target.id) UnloadRenderTexture(target);
        target = {};
        surface = {};
        drag = {};
        pressing = false;
        windowMin = windowMax = {};
        openedFrameRaw = nullptr;
        openedMesh.reset();
    }

    void Open(const RenderFrame& frame, int chosen, mfem::GridFunction* field, const RenderOptions& options, int refinement,
              const Shader& shader, const ColorFor& color) {
        element = chosen;
        if (pending && pending->element != element) pending.reset();
        try {
            Build(frame, field, options, refinement, shader, color);
        } catch (const std::exception&) { Close(); step = Step::ChooseElement; return; }
        const Vector3 center = Vector3Scale(Vector3Add(surface.bounds.min, surface.bounds.max), .5f);
        radius = std::max(1e-6f, Vector3Length(Vector3Subtract(surface.bounds.max, surface.bounds.min))*.5f);
        // Start from the main view's direction so the element appears as it does there.
        const Vector3 back = Vector3Length(viewDirection) > 0 ? Vector3Negate(Vector3Normalize(viewDirection)) : Vector3Normalize({1, -1.4f, 1.1f});
        camera = {Vector3Add(center, Vector3Scale(back, radius*2.3f)), center, Vector3Length(viewUp) > 0 ? viewUp : Vector3{0, 0, 1}, 40, CAMERA_PERSPECTIVE};
        step = Step::PlacePoint;
    }

    void Rebuild(const RenderFrame& frame, mfem::GridFunction* field, const RenderOptions& options, int refinement,
                 const Shader& shader, const ColorFor& color) {
        try { Build(frame, field, options, refinement, shader, color); } catch (const std::exception&) {}
    }

    void Build(const RenderFrame& frame, mfem::GridFunction* field, const RenderOptions& options, int refinement,
               const Shader& shader, const ColorFor& color) {
        auto settings = options;
        settings.element_explosion = 0;
        surface = BuildElementSurface(*frame.mesh, element, field, settings, refinement);
        SurfaceData data;
        for (const auto& t : surface.triangles) {
            const Vector3 a = surface.positions[t[0]], b = surface.positions[t[1]], c = surface.positions[t[2]];
            Vector3 normal = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(b, a), Vector3Subtract(c, a)));
            for (int k = 0; k < 3; ++k) {
                const auto& p = surface.positions[t[k]];
                data.positions.insert(data.positions.end(), {p.x, p.y, p.z});
                data.normals.insert(data.normals.end(), {normal.x, normal.y, normal.z});
                const Color c = field ? color(surface.values[t[k]]) : Color{155, 195, 220, 255};
                data.colors.insert(data.colors.end(), {c.r, c.g, c.b, c.a});
            }
        }
        if (model.meshCount) UnloadModel(model);
        model = UploadSurface(data);
        for (int i = 0; i < model.materialCount; ++i) model.materials[i].shader = shader;
        if (pending) pendingPosition = AnchorPosition(*frame.mesh, *pending, settings).value_or(pendingPosition);
        settingsUsed = settings;
        meshForPositions = frame.mesh.get();
        openedMesh = frame.mesh;
        openedFrameRaw = &frame;
    }

    void DrawView(ImVec2 size, const ThemePalette& palette, const DrawContext& context) {
        const int width = std::max(1, int(size.x)), height = std::max(1, int(size.y));
        if (!target.id || target.texture.width != width || target.texture.height != height) {
            if (target.id) UnloadRenderTexture(target);
            target = LoadRenderTexture(width, height);
        }
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##element_view", size);
        const bool hovered = ImGui::IsItemHovered();
        const Vector2 mouse = GetMousePosition();
        // Orbit with a drag; a click without movement places the point.
        CameraInputSnapshot input{mouse, hovered, IsMouseButtonPressed(MOUSE_BUTTON_LEFT), IsMouseButtonDown(MOUSE_BUTTON_LEFT),
                                  IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE), IsMouseButtonDown(MOUSE_BUTTON_MIDDLE),
                                  hovered ? GetMouseWheelMove() : 0.0f, false};
        if (hovered && input.left_pressed) { pressing = true; pressMoved = false; pressPosition = mouse; }
        if (pressing) PrepareCameraOrbitGesture(input, drag, pressPosition, pressMoved);
        UpdateCameraController(camera, drag, input, radius);
        const Ray ray = GetScreenToWorldRayEx({mouse.x - origin.x, mouse.y - origin.y}, camera, width, height);
        const auto hover = hovered ? PickElementSurface(surface, element, ray) : std::nullopt;
        if (hover) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        if (pressing && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (!pressMoved && hover && meshForPositions) {
                pending = hover;
                pendingPosition = AnchorPosition(*meshForPositions, *hover, settingsUsed).value_or(pendingPosition);
            }
            pressing = false;
        }
        BeginTextureMode(target);
        ClearBackground(palette.viewport_background);
        BeginMode3D(camera);
        rlDisableBackfaceCulling();
        DrawModel(model, {0, 0, 0}, 1, WHITE);
        // Neighbours and the rest of the mesh, faint and behind the element.
        if (context) { rlDrawRenderBatchActive(); context(camera, {float(width), float(height)}); rlDrawRenderBatchActive(); }
        for (const auto& edge : surface.outline)
            for (std::size_t k = 1; k < edge.size(); ++k) DrawLine3D(edge[k-1], edge[k], palette.profile_guide);
        const float marker = radius*.035f;
        if (hover && meshForPositions)
            if (const auto p = AnchorPosition(*meshForPositions, *hover, settingsUsed)) DrawSphere(*p, marker*.8f, ThemeAlpha(palette.profile_guide, 150));
        if (pending) DrawSphere(pendingPosition, marker, palette.profile_guide);
        rlEnableBackfaceCulling();
        EndMode3D();
        EndTextureMode();
        auto* draw = ImGui::GetWindowDrawList();
        draw->AddImageRounded(ImTextureID(target.texture.id), origin, {origin.x + size.x, origin.y + size.y}, {0, 0}, {1, -1},
                              IM_COL32_WHITE, CardRounding);
        draw->AddRect(origin, {origin.x + size.x, origin.y + size.y}, ImGui::GetColorU32(ImGuiCol_Border), CardRounding);
    }

    Step step = Step::Idle;
    int endpoint = 0, element = -1, requestedElement = -1;
    bool single = false, pressing = false, pressMoved = false;
    std::array<RayAnchor, 2> anchors{};
    std::optional<RayAnchor> pending;
    Vector3 pendingPosition{};
    Vector2 pressPosition{}, windowMin{}, windowMax{};
    ImVec2 windowSize{}, windowPosition{}, restoreSize{}, restorePosition{};
    bool expanded = false, resize = false;
    Vector3 viewDirection{}, viewUp{0, 0, 1};
    ElementSurface surface;
    Model model{};
    RenderTexture2D target{};
    Camera3D camera{};
    CameraDragState drag;
    float radius = 1;
    RenderOptions settingsUsed;
    mfem::Mesh* meshForPositions = nullptr;
    std::shared_ptr<mfem::Mesh> openedMesh;
    const RenderFrame* openedFrameRaw = nullptr;
};

} // namespace RayFEM::UI
