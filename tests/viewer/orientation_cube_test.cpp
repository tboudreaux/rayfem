#include "render/orientation_cube.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>

namespace {
using namespace RayFEM;
using namespace RayFEM::UI;

void Check(bool okay, const std::string& message) {
    if (!okay) throw std::runtime_error(message);
}
void Near(float actual, float expected, const std::string& message, float tolerance = 2e-5f) {
    Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
void Near(Vector2 actual, Vector2 expected, const std::string& message) {
    Near(actual.x, expected.x, message + " x");
    Near(actual.y, expected.y, message + " y");
}
void Near(Vector3 actual, Vector3 expected, const std::string& message) {
    Near(actual.x, expected.x, message + " x");
    Near(actual.y, expected.y, message + " y");
    Near(actual.z, expected.z, message + " z");
}
Camera3D Camera(Vector3 direction = {0, 0, 1}, Vector3 up = {0, 1, 0}) {
    return {Vector3Scale(Vector3Normalize(direction), 5), {0, 0, 0}, up, 40, CAMERA_PERSPECTIVE};
}
std::array<int, 3> Key(Vector3 direction) {
    return {int(direction.x), int(direction.y), int(direction.z)};
}
const Rectangle box{100, 200, 128, 128};

void AllDestinations() {
    std::set<std::array<int, 3>> destinations;
    int faceCount = 0, edgeCount = 0, cornerCount = 0;
    for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
        float normal[3]{};
        normal[axis] = float(sign);
        // Deliberately use ordinary world-up even at the two poles, where
        // CameraBasis must recover a usable orthographic screen basis.
        const auto camera = Camera({normal[0], normal[1], normal[2]});
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        for (int row = -1; row <= 1; ++row) for (int col = -1; col <= 1; ++col) {
            float point[3]{}, expected[3]{};
            point[axis] = expected[axis] = float(sign);
            point[u] = float(col) * .8f;
            point[v] = float(row) * .8f;
            expected[u] = float(col);
            expected[v] = float(row);
            const Vector3 world{point[0], point[1], point[2]};
            const auto hit = PickOrientationCube(ProjectOrientationCubePoint(world, camera, box), camera, box);
            Check(hit.has_value(), "Every visible face tile is selectable");
            Near(hit->direction, {expected[0], expected[1], expected[2]}, "Face/edge/corner destination");
            Near(hit->point, world, "Pick ray reaches the visible face");
            Check(hit->face_axis == axis, "Face-aligned pick resolves the facing axis");
            destinations.insert(Key(hit->direction));
        }
    }
    Check(destinations.size() == 26, "All 26 distinct cube destinations are reachable");
    for (const auto& direction : destinations) {
        const int dimensions = (direction[0] != 0) + (direction[1] != 0) + (direction[2] != 0);
        faceCount += dimensions == 1;
        edgeCount += dimensions == 2;
        cornerCount += dimensions == 3;
    }
    Check(faceCount == 6 && edgeCount == 12 && cornerCount == 8, "Six faces, twelve edges and eight corners");
}

void ProjectionAndDepth() {
    const auto front = Camera();
    Near(ProjectOrientationCubePoint({0, 0, 0}, front, box), {164, 264}, "Cube origin is card center");
    Near(ProjectOrientationCubePoint({1, 0, 0}, front, box), {196, 264}, "Front view positive X points right");
    Near(ProjectOrientationCubePoint({0, 1, 0}, front, box), {164, 232}, "Front view positive Y points up");
    const auto rolled = Camera({0, 0, 1}, {1, 0, 0});
    Near(ProjectOrientationCubePoint({1, 0, 0}, rolled, box), {164, 232}, "Actual camera roll rotates X up");
    Near(ProjectOrientationCubePoint({0, 1, 0}, rolled, box), {132, 264}, "Actual camera roll rotates Y left");
    const auto rolledHit = PickOrientationCube({138.4f, 238.4f}, rolled, box);
    Check(rolledHit.has_value(), "Rolled corner can be selected");
    Near(rolledHit->direction, {1, 1, 1}, "Rolled screen pick retains world-axis identity");
    auto moved = rolled;
    moved.target = {11, -17, 35};
    moved.position = {11, -17, 85};
    moved.fovy = 70;
    moved.projection = CAMERA_ORTHOGRAPHIC;
    Near(ProjectOrientationCubePoint({.4f, -.7f, .3f}, moved, box),
         ProjectOrientationCubePoint({.4f, -.7f, .3f}, rolled, box),
         "Orientation display is independent of pan, zoom and projection");
    const auto depthHit = PickOrientationCube(ProjectOrientationCubePoint({0, 0, -1}, front, box), front, box);
    Check(depthHit.has_value(), "Ray intersects the front surface");
    Near(depthHit->direction, {0, 0, 1}, "Hidden back face cannot win over front face");
    Near(depthHit->point, {0, 0, 1}, "Pick selects nearest intersection");

    for (int sign : {-1, 1}) {
        const auto diagonal = Camera({float(sign), float(sign), float(sign)});
        for (int axis = 0; axis < 3; ++axis) {
            float face[3]{};
            face[axis] = float(sign);
            const Vector3 center{face[0], face[1], face[2]};
            const auto hit = PickOrientationCube(ProjectOrientationCubePoint(center, diagonal, box), diagonal, box);
            Check(hit.has_value(), "Each visible face center is selectable in an isometric view");
            Near(hit->direction, center, "Isometric face center selects one axis");
            Check(hit->face_axis == axis, "Isometric face axis");
        }
    }
}

void TileBoundariesAndMisses() {
    const auto camera = Camera();
    for (float x : {-.6001f, -.5999f, .5999f, .6001f}) {
        const auto hit = PickOrientationCube(ProjectOrientationCubePoint({x, 0, 1}, camera, box), camera, box);
        Check(hit.has_value(), "Threshold-adjacent point remains on the cube");
        const float expectedX = std::abs(x) > .6f ? (x < 0 ? -1.f : 1.f) : 0;
        Near(hit->direction, {expectedX, 0, 1}, "Face-to-edge boundary follows the drawn tile boundary");
    }
    for (float y : {-.6001f, -.5999f, .5999f, .6001f}) {
        const auto hit = PickOrientationCube(ProjectOrientationCubePoint({.8f, y, 1}, camera, box), camera, box);
        Check(hit.has_value(), "Corner-boundary point remains on the cube");
        const float expectedY = std::abs(y) > .6f ? (y < 0 ? -1.f : 1.f) : 0;
        Near(hit->direction, {1, expectedY, 1}, "Edge-to-corner boundary follows the drawn tile boundary");
    }
    for (Vector3 point : {Vector3{1.01f, 0, 1}, {-1.01f, 0, 1}, {0, 1.01f, 1}, {0, -1.01f, 1}}) {
        Check(!PickOrientationCube(ProjectOrientationCubePoint(point, camera, box), camera, box),
              "Parallel pick ray outside the cube silhouette must miss");
    }
    for (Vector2 mouse : {Vector2{101, 201}, {227, 201}, {101, 327}, {227, 327}, {99, 264}, {228, 264}}) {
        Check(!PickOrientationCube(mouse, camera, box), "Blank card corners and outside positions are not destinations");
    }
    Check(!PickOrientationCube({164, 264}, camera, {}), "Hidden cube cannot be picked");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    Check(!PickOrientationCube({nan, 264}, camera, box), "Nonfinite pointer cannot select a view");
    auto degenerate = camera;
    degenerate.position = degenerate.target;
    degenerate.up = {};
    const auto recovered = PickOrientationCube({164, 264}, degenerate, box);
    Check(recovered.has_value(), "Degenerate camera has a finite orientation fallback");
    Near(recovered->direction, {0, 0, 1}, "Fallback orientation is front");
}

void Layout() {
    for (Rectangle viewport : {Rectangle{10, 20, 960, 640}, {10, 20, 300, 120}, {10, 20, 72, 72}}) {
        const auto bounds = OrientationCubeBounds(viewport);
        Check(bounds.width >= 48 && bounds.width <= 132 && bounds.height == bounds.width,
              "Cube card remains square and within its useful size range");
        Near(bounds.x + bounds.width, viewport.x + viewport.width - 12, "Card keeps its right inset");
        Near(bounds.y, viewport.y + 12, "Card keeps its upper inset");
        Check(bounds.x >= viewport.x + 12 && bounds.y + bounds.height <= viewport.y + viewport.height - 12,
              "Card stays inside even the 120px minimum-height viewport");
    }
    Near(OrientationCubeBounds({10, 20, 300, 120}).width, 96, "Short viewport scales the cube");
    for (Rectangle viewport : {Rectangle{0, 0, 71, 100}, {0, 0, 100, 71}, {0, 0, 0, 0}, {0, 0, -10, 100}}) {
        Check(OrientationCubeBounds(viewport).width == 0, "Too-small viewport hides the card");
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (Rectangle viewport : {Rectangle{nan, 0, 200, 200}, {0, nan, 200, 200},
                               {0, 0, nan, 200}, {0, 0, 200, nan}, {0, 0, inf, 200}}) {
        Check(OrientationCubeBounds(viewport).width == 0, "Nonfinite layout hides the card");
    }
}

void InputOwnership() {
    const auto camera = Camera();
    OrientationCubeInputState state;
    CameraInputSnapshot input;
    input.mouse = {164, 264};
    input.viewport_hovered = true;
    input.left_pressed = input.left_down = true;
    auto result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && !result.selected && state.captured, "Cube press captures the pointer and waits for release");
    input.left_pressed = input.left_down = false;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && result.selected && !state.captured, "Cube click selects on release");
    Near(result.selected->direction, {0, 0, 1}, "Cube center click selects front");
    input.mouse = {90, 264};
    input.viewport_hovered = false;
    Check(!UpdateOrientationCubeInput(state, input, camera, box, false).owns_pointer,
          "Later pointer motion outside the cube is unclaimed");

    // A press that moves becomes a drag: it orbits by the pointer's motion,
    // also outside the cube, and never selects.
    input.mouse = {164, 264};
    input.left_pressed = input.left_down = true;
    input.viewport_hovered = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    input.left_pressed = false;
    input.mouse = {166, 265};
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(!result.dragging && result.orbit.x == 0 && result.orbit.y == 0, "A small wobble stays a click");
    input.mouse = {174, 260};
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.dragging && result.owns_pointer, "Moving past the threshold starts a drag");
    Near(Vector3{result.orbit.x, result.orbit.y, 0}, {10, -4, 0}, "The first drag step keeps the motion since the press");
    input.mouse = {90, 264};
    input.viewport_hovered = false;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.dragging && result.owns_pointer, "The drag continues outside the cube");
    Near(Vector3{result.orbit.x, result.orbit.y, 0}, {-84, 4, 0}, "Each step orbits by the pointer's motion");
    input.left_down = false;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && !result.selected && !result.dragging && !state.captured,
          "Releasing a drag selects nothing and ends capture");

    input = {};
    input.mouse = {101, 201};
    input.viewport_hovered = true;
    input.left_pressed = input.left_down = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && !result.selected && state.captured, "Blank card space captures without selecting");
    input.left_pressed = input.left_down = false;
    UpdateOrientationCubeInput(state, input, camera, box, false);
    input.mouse = {164, 264};
    input.middle_pressed = input.middle_down = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && !result.selected && state.captured, "Middle click never snaps or pans through the card");
    input.middle_pressed = input.middle_down = false;
    input.wheel = 1;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && !result.selected, "Wheel over the card cannot zoom the mesh");

    state = {};
    input = {};
    input.mouse = {164, 264};
    input.viewport_hovered = true;
    input.left_down = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, true);
    Check(!result.owns_pointer && !result.selected && !state.captured,
          "An established mesh or profile drag retains ownership across the cube");
    input.left_pressed = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(result.owns_pointer && state.cube_press, "A fresh cube press can acquire pointer after previous drag ends");
    input.cancel_drag = true;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(!result.owns_pointer && !result.selected && !state.captured, "Lost focus cancels cube input immediately");
    input.cancel_drag = false;
    input.viewport_hovered = false;
    result = UpdateOrientationCubeInput(state, input, camera, box, false);
    Check(!result.owns_pointer && !result.selected && !state.captured,
          "Popup-blocked viewport cannot start a cube selection");
}

void CubeOrbit() {
    Camera3D camera{{0, 0, 10}, {1, 2, 3}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    camera.position = {1, 2, 13};
    OrbitCamera(camera, {100, 0}, .01f);
    Check(std::abs(Vector3Distance(camera.position, camera.target) - 10) < 1e-4f, "Orbiting keeps the distance");
    Near(camera.up, {0, 1, 0}, "A horizontal drag keeps the up direction");
    Check(camera.position.x < 1, "Dragging right turns the scene with the pointer");
    const Camera3D before = camera;
    OrbitCamera(camera, {0, 0}, .01f);
    Near(camera.position, before.position, "No motion leaves the camera alone");
}

Vector2 Center(Rectangle bounds) {
    return {bounds.x + bounds.width / 2, bounds.y + bounds.height / 2};
}

void RotationButtonSelection() {
    const auto layout = OrientationControlsBounds({10, 20, 960, 640});
    const auto camera = Camera();
    const std::array<float, 4> expectedDegrees{45, 5, -5, -45};
    Check(CameraRotationButtons.size() == 4, "One row offers coarse/fine rotation in both image directions");
    for (int i = 0; i < 4; ++i) {
        const auto mouse = Center(layout.arrows[i]);
        Check(PickCameraRotationArrow(mouse, layout) == i, "Every visible arrow has its own hit target");
        Check(CameraRotationButtons[i].axis == CameraRotationAxis::Back,
              "Every arrow rotates the image about the viewing direction without orbiting the camera");
        Near(CameraRotationButtons[i].degrees, expectedDegrees[i], "Single/double arrows request signed 5/45 degree steps");
        OrientationCubeInputState state;
        CameraInputSnapshot input;
        input.mouse = mouse;
        input.viewport_hovered = input.left_pressed = input.left_down = true;
        const auto result = UpdateOrientationControlsInput(state, input, camera, layout, false);
        Check(result.owns_pointer && state.captured && result.rotation_button == i && !result.selected,
              "Arrow press captures one rotation action without selecting a cube face");
        input.left_pressed = false;
        const auto held = UpdateOrientationControlsInput(state, input, camera, layout, false);
        Check(held.owns_pointer && held.rotation_button == -1 && !held.selected,
              "Holding an arrow does not produce an increment every display frame");
    }
    const auto cubeCenter = Center(layout.cube);
    Check(PickCameraRotationArrow(cubeCenter, layout) == -1, "Cube face cannot also select an arrow");
    OrientationCubeInputState state;
    CameraInputSnapshot input;
    input.mouse = cubeCenter;
    input.viewport_hovered = input.left_pressed = input.left_down = true;
    auto result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(!result.selected && result.rotation_button == -1 && state.cube_press, "A cube press above the arrows is a cube press");
    input.left_pressed = input.left_down = false;
    result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(result.selected && result.rotation_button == -1, "Cube picking remains available above rotation controls");
    Near(result.selected->direction, {0, 0, 1}, "Combined controls retain front-face destination");
    input.mouse=Center(layout.home);
    input.left_pressed=input.left_down=true;
    state={};
    const auto home=UpdateOrientationControlsInput(state,input,camera,layout,false);
    Check(home.home && home.owns_pointer && state.captured && home.rotation_button==-1 && !home.selected,
          "Home owns the pointer and cannot trigger rotation or cube selection");
    input.left_pressed=false;
    Check(!UpdateOrientationControlsInput(state,input,camera,layout,false).home,"Holding Home only triggers once");
    Check(layout.home.x>layout.arrows[1].x && layout.home.x<layout.arrows[2].x,
          "Home sits between counterclockwise and clockwise arrows");
}

void RotationControlLayout() {
    // Exercise narrow windows and short viewports with an open plot, including
    // both transitions between complete controls, a cube alone, and no card.
    for (float width : {0.f, 71.f, 72.f, 100.f, 135.f, 136.f, 156.f, 300.f, 960.f}) {
        for (float height : {0.f, 71.f, 72.f, 95.f, 96.f, 119.f, 120.f, 187.f, 188.f, 220.f, 640.f}) {
            const Rectangle viewport{23, 47, width, height};
            const auto layout = OrientationControlsBounds(viewport);
            int arrowCount = 0;
            for (const auto arrow : layout.arrows) arrowCount += arrow.width > 0;
            Check(arrowCount == 0 || arrowCount == 4, "Responsive layout never exposes a partial set of directions");
            if (layout.bounds.width == 0) {
                Check(layout.cube.width == 0 && arrowCount == 0, "Hidden card has no hidden interactive child");
                Check(PickCameraRotationArrow({23, 47}, layout) == -1, "Hidden controls cannot be clicked");
                continue;
            }
            Check(layout.bounds.x >= viewport.x + 12 && layout.bounds.y >= viewport.y + 12 &&
                  layout.bounds.x + layout.bounds.width <= viewport.x + viewport.width - 12 &&
                  layout.bounds.y + layout.bounds.height <= viewport.y + viewport.height - 12,
                  "Entire orientation card stays inside the viewport with its insets");
            Near(layout.cube.width, layout.cube.height, "Resized cube remains square");
            Check(layout.cube.width >= 48, "Resized cube remains large enough to pick faces");
            for (int i = 0; i < 4; ++i) if (layout.arrows[i].width > 0) {
                const auto arrow = layout.arrows[i];
                Check(arrow.width >= 25 && arrow.height >= 20, "Compact arrow buttons retain usable hit targets");
                Near(arrow.y, layout.arrows[0].y, "All four rotation arrows share one row");
                Check(arrow.x >= layout.bounds.x && arrow.y >= layout.cube.y + layout.cube.height &&
                      arrow.x + arrow.width <= layout.bounds.x + layout.bounds.width &&
                      arrow.y + arrow.height <= layout.bounds.y + layout.bounds.height,
                      "Visible arrows are beneath the cube and inside the card");
                Check(PickCameraRotationArrow(Center(arrow), layout) == i, "Resizing keeps visible buttons selectable");
            }
        }
    }
    const auto compact = OrientationControlsBounds({10, 20, 300, 120});
    Check(compact.arrows[3].width > 0, "Minimum application viewport height retains all rotation controls");
    const auto narrow = OrientationControlsBounds({10, 20, 100, 640});
    Check(narrow.cube.width > 0 && narrow.arrows[0].width == 0, "Very narrow viewport falls back to cube alone");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (Rectangle viewport : {Rectangle{nan, 0, 300, 200}, {0, inf, 300, 200},
                               {0, 0, nan, 200}, {0, 0, 300, inf}}) {
        const auto invalid = OrientationControlsBounds(viewport);
        Check(invalid.bounds.width == 0 && PickCameraRotationArrow({0, 0}, invalid) == -1,
              "Nonfinite viewport never creates clickable rotation controls");
    }
}

void RotationControlOwnership() {
    const auto layout = OrientationControlsBounds({10, 20, 960, 640});
    const auto camera = Camera();
    const auto first = layout.arrows[0];
    const std::array<Vector2, 2> gaps{{
        {first.x + first.width + 2, first.y + first.height / 2},
        {first.x + first.width / 2, first.y - 2}}};
    for (auto point : gaps) {
        OrientationCubeInputState state;
        CameraInputSnapshot input;
        input.mouse = point;
        input.viewport_hovered = input.left_pressed = input.left_down = true;
        Check(PickCameraRotationArrow(point, layout) == -1, "Gaps below the cube and between buttons are not rotation actions");
        const auto result = UpdateOrientationControlsInput(state, input, camera, layout, false);
        Check(result.owns_pointer && state.captured && !result.selected && result.rotation_button == -1,
              "Blank control spacing consumes a press without starting an orbit behind the overlay");
    }
    OrientationCubeInputState state;
    CameraInputSnapshot input;
    input.mouse = Center(layout.arrows[3]);
    input.viewport_hovered = input.left_pressed = input.left_down = true;
    UpdateOrientationControlsInput(state, input, camera, layout, false);
    input.left_pressed = false;
    input.viewport_hovered = false;
    input.mouse = {0, 0};
    auto result = UpdateOrientationControlsInput(state, input, camera, {}, false);
    Check(result.owns_pointer && state.captured && result.rotation_button == -1,
          "Arrow capture survives leaving the overlay or resizing it away");
    input.left_down = false;
    result = UpdateOrientationControlsInput(state, input, camera, {}, false);
    Check(result.owns_pointer && !state.captured && result.rotation_button == -1,
          "Release outside a hidden overlay is consumed once");
    Check(!UpdateOrientationControlsInput(state, input, camera, {}, false).owns_pointer,
          "Pointer returns to mesh after the release frame");

    input = {};
    input.mouse = Center(first);
    input.viewport_hovered = input.middle_pressed = input.middle_down = true;
    result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(result.owns_pointer && state.captured && result.rotation_button == -1 && !result.selected,
          "Middle click on an arrow cannot pan the scene or request a rotation");
    input.middle_pressed = input.middle_down = false;
    input.wheel = 1;
    result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(result.owns_pointer && result.rotation_button == -1 && !result.selected,
          "Wheel over rotation controls cannot zoom the mesh");
    input.left_pressed = input.left_down = true;
    result = UpdateOrientationControlsInput(state, input, camera, layout, true);
    Check(!result.owns_pointer && !state.captured && result.rotation_button == -1,
          "An existing mesh or profile drag keeps control when crossing arrows");
    input.cancel_drag = true;
    result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(!result.owns_pointer && !state.captured && result.rotation_button == -1,
          "Lost focus cancels arrow presses immediately");
    input.cancel_drag = false;
    input.viewport_hovered = false;
    result = UpdateOrientationControlsInput(state, input, camera, layout, false);
    Check(!result.owns_pointer && !state.captured && result.rotation_button == -1,
          "Popup-blocked viewport cannot activate an arrow");
}
} // namespace

int main() {
    try {
        AllDestinations();
        ProjectionAndDepth();
        TileBoundariesAndMisses();
        Layout();
        InputOwnership();
        CubeOrbit();
        RotationButtonSelection();
        RotationControlLayout();
        RotationControlOwnership();
        std::cout << "Orientation controls passed: 26 cube views, four image-rotation buttons, rolled/pole projection, visible-surface picking, responsive layout and pointer ownership.\n";
    } catch (const std::exception& error) {
        std::cerr << "Orientation cube test failed: " << error.what() << '\n';
        return 1;
    }
}
