#include "../src/app/reference_plane.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) { throw std::runtime_error(message); }
}
void Near(float actual, float expected, float tolerance, const std::string& message) {
    Check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
void Near(Vector3 actual, Vector3 expected, float tolerance, const std::string& message) {
    Near(Vector3Distance(actual, expected), 0, tolerance, message);
}
void Frame(const RayFEM::CameraViewBasis& frame) {
    Near(Vector3Length(frame.right), 1, 2e-6f, "Plane right is a unit vector");
    Near(Vector3Length(frame.up), 1, 2e-6f, "Plane up is a unit vector");
    Near(Vector3Length(frame.back), 1, 2e-6f, "Plane normal is a unit vector");
    Near(Vector3DotProduct(frame.right, frame.up), 0, 2e-6f, "Plane axes are perpendicular");
    Near(Vector3DotProduct(frame.right, frame.back), 0, 2e-6f, "Right is perpendicular to the normal");
    Near(Vector3DotProduct(frame.up, frame.back), 0, 2e-6f, "Up is perpendicular to the normal");
    Near(Vector3CrossProduct(frame.right, frame.up), frame.back, 2e-6f, "Plane frame is right-handed");
}
void Basis() {
    auto basis = RayFEM::ReferencePlaneBasis({});
    Near(basis.right, {1, 0, 0}, 0, "Default plane uses world X horizontally");
    Near(basis.up, {0, 1, 0}, 0, "Default plane uses world Y vertically");
    Near(basis.back, {0, 0, 1}, 0, "Default plane is XY facing +Z");

    basis = RayFEM::ReferencePlaneBasis({90, 0, 0});
    Near(basis.up, {0, 0, 1}, 2e-6f, "Positive X rotation turns Y toward Z");
    Near(basis.back, {0, -1, 0}, 2e-6f, "Positive X rotation turns normal toward -Y");
    basis = RayFEM::ReferencePlaneBasis({-90, 0, 0});
    Near(basis.right, {1, 0, 0}, 2e-6f, "XZ preset keeps X horizontal");
    Near(basis.up, {0, 0, -1}, 2e-6f, "XZ preset has -Z screen-up");
    Near(basis.back, {0, 1, 0}, 2e-6f, "XZ preset faces +Y");
    basis = RayFEM::ReferencePlaneBasis({0, 90, 0});
    Near(basis.right, {0, 0, -1}, 2e-6f, "YZ preset has -Z horizontal");
    Near(basis.up, {0, 1, 0}, 2e-6f, "YZ preset keeps Y vertical");
    Near(basis.back, {1, 0, 0}, 2e-6f, "YZ preset faces +X");
    basis = RayFEM::ReferencePlaneBasis({0, 0, 90});
    Near(basis.right, {0, 1, 0}, 2e-6f, "In-plane rotation turns horizontal axis");
    Near(basis.up, {-1, 0, 0}, 2e-6f, "In-plane rotation turns screen-up");
    Near(basis.back, {0, 0, 1}, 2e-6f, "In-plane rotation keeps XY normal");

    for (const Vector3 angles : {Vector3{27, -38, 73}, Vector3{90, 90, 90},
                                Vector3{-170, 89.9f, 156}, Vector3{720, -360, 1080}}) {
        Frame(RayFEM::ReferencePlaneBasis(angles));
    }
    // Independent sequential axis rotations establish the Euler convention.
    Vector3 expected{0, 1, 0};
    expected = Vector3RotateByAxisAngle(expected, {1, 0, 0}, 27 * DEG2RAD);
    expected = Vector3RotateByAxisAngle(expected, {0, 1, 0}, -38 * DEG2RAD);
    expected = Vector3RotateByAxisAngle(expected, {0, 0, 1}, 73 * DEG2RAD);
    Near(RayFEM::ReferencePlaneBasis({27, -38, 73}).up, expected, 2e-6f,
         "Euler rotations apply world X, then Y, then Z");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    for (const Vector3 invalid : {Vector3{nan, 0, 0}, Vector3{0, infinity, 0}, Vector3{0, 0, nan}}) {
        const auto fallback = RayFEM::ReferencePlaneBasis(invalid);
        Near(fallback.right, {1, 0, 0}, 0, "Invalid rotations fall back to XY");
        Near(fallback.up, {0, 1, 0}, 0, "Invalid rotations have finite screen-up");
        Near(fallback.back, {0, 0, 1}, 0, "Invalid rotations have finite normal");
    }
}
void Grid() {
    const BoundingBox mesh{{2, 3, 4}, {6, 9, 12}};
    auto grid = RayFEM::ReferencePlaneGrid(mesh, 4, false, {});
    Near(grid.center, {}, 0, "Empty scene grid is centered at world origin");
    Near(grid.half_extent, 5, 0, "Empty scene grid spans ten world units");
    Near(grid.bounds.min, {-5, -5, 0}, 0, "Default grid bounds lie in XY");
    Near(grid.bounds.max, {5, 5, 0}, 0, "Default grid bounds lie in XY");
    grid = RayFEM::ReferencePlaneGrid(mesh, 4, true, {});
    Near(grid.center, {4, 6, 0}, 0, "Mesh center projects onto the world XY plane");
    Near(grid.half_extent, 8, 1e-6f, "Grid extent scales with mesh radius");

    const Vector3 mesh_center{4, 6, 8};
    for (const Vector3 rotation : {Vector3{0, 0, 0}, Vector3{-90, 0, 0},
                                  Vector3{0, 90, 0}, Vector3{23, 58, -31}}) {
        const auto basis = RayFEM::ReferencePlaneBasis(rotation);
        grid = RayFEM::ReferencePlaneGrid(mesh, 4, true, rotation);
        Near(grid.u, basis.right, 0, "Grid uses the plane's horizontal direction");
        Near(grid.v, basis.up, 0, "Grid uses the plane's vertical direction");
        Near(Vector3DotProduct(grid.center, basis.back), 0, 4e-6f, "Grid center stays on the world-origin plane");
        Near(Vector3DotProduct(Vector3Subtract(mesh_center, grid.center), basis.right), 0, 4e-6f,
             "Projection preserves the horizontal coordinate");
        Near(Vector3DotProduct(Vector3Subtract(mesh_center, grid.center), basis.up), 0, 4e-6f,
             "Projection preserves the vertical coordinate");
        Vector3 minimum = grid.corners[0], maximum = grid.corners[0];
        for (size_t i = 0; i < grid.corners.size(); ++i) {
            const auto corner = grid.corners[i];
            Near(Vector3DotProduct(corner, basis.back), 0, 5e-6f, "Every grid corner lies in the plane");
            Near(Vector3Distance(corner, grid.corners[(i + 1) % 4]), 16, 5e-6f,
                 "Rotated patch remains square");
            minimum = Vector3Min(minimum, corner);
            maximum = Vector3Max(maximum, corner);
        }
        Near(grid.bounds.min, minimum, 0, "Depth bounds include every rotated corner");
        Near(grid.bounds.max, maximum, 0, "Depth bounds include every rotated corner");
    }
    grid = RayFEM::ReferencePlaneGrid(mesh, 0, true, {});
    Check(grid.half_extent > 0 && std::isfinite(grid.half_extent), "Degenerate mesh still has a finite visible grid");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    grid = RayFEM::ReferencePlaneGrid({{nan, 0, 0}, {1, 1, 1}}, nan, true, {});
    Near(grid.center, {}, 0, "Invalid mesh bounds use an origin-centered grid");
    Near(grid.half_extent, 5, 0, "Invalid radius uses the empty-scene extent");
}
void Home() {
    for (const Vector3 rotation : {Vector3{0, 0, 0}, Vector3{-90, 0, 0},
                                  Vector3{0, 90, 0}, Vector3{0, 0, 90}, Vector3{23, 58, -31}}) {
        Camera3D camera{{7, -2, 13}, {2, 3, 5}, {1, 1, 0}, 37, CAMERA_ORTHOGRAPHIC};
        const auto original = camera;
        const float distance = Vector3Distance(camera.position, camera.target);
        RayFEM::CameraViewTransition transition;
        Check(RayFEM::BeginCameraHomeTransition(transition, camera, rotation, 10, .5), "Home begins a smooth transition");
        Near(camera.position, original.position, 0, "Starting Home does not jump camera position");
        Near(camera.up, original.up, 0, "Starting Home does not jump camera roll");
        Check(!transition.rolling, "Home uses full orientation interpolation");
        for (int step = 1; step <= 10; ++step) {
            RayFEM::UpdateCameraViewTransition(camera, transition, 10 + .05 * step);
            Near(camera.target, original.target, 0, "Home preserves target during transition");
            Near(Vector3Distance(camera.position, camera.target), distance, 3e-6f,
                 "Home preserves zoom during transition");
            Frame(RayFEM::CameraBasis(camera));
        }
        const auto basis = RayFEM::ReferencePlaneHomeBasis(rotation);
        const auto plane = RayFEM::ReferencePlaneBasis(rotation);
        Near(Vector3DotProduct(basis.back,plane.back),.5f,2e-6f,"Home elevation is 30 degrees above the selected plane");
        Near(Vector3DotProduct(basis.back,plane.up),-std::sqrt(.75f),2e-6f,"Home looks along the plane's local Y direction");
        Check(!transition.active, "Home finishes at its deadline");
        Near(RayFEM::CameraBasis(camera).back, basis.back, 2e-6f, "Home faces the selected plane");
        Near(camera.up, basis.up, 0, "Home adopts the plane's complete screen-up orientation");
        Near(camera.fovy, original.fovy, 0, "Home preserves field of view");
        Check(camera.projection == original.projection, "Home preserves projection type");
    }
    Camera3D camera{{0, 0, 5}, {}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    RayFEM::CameraViewTransition transition;
    Check(RayFEM::BeginCameraRotationStep(transition, camera, RayFEM::CameraRotationAxis::Back, 45, 1),
          "Begin a prior roll transition");
    RayFEM::UpdateCameraViewTransition(camera, transition, 1.1);
    const auto interrupted = camera;
    Check(RayFEM::BeginCameraHomeTransition(transition, camera, {0, 0, -30}, 1.1, 0), "Home can interrupt a roll");
    Near(camera.up, interrupted.up, 0, "Interrupting roll begins at displayed orientation");
    RayFEM::UpdateCameraViewTransition(camera, transition, 1.1);
    Near(camera.up, RayFEM::ReferencePlaneHomeBasis({0, 0, -30}).up, 0, "Immediate Home uses the plane's in-plane rotation");
    camera.position = camera.target;
    Check(!RayFEM::BeginCameraHomeTransition(transition, camera, {}, 2), "Home rejects degenerate camera distance");
}
} // namespace

int main() {
    try {
        Basis();
        Grid();
        Home();
        std::cout << "Reference plane passed: XY default, presets and Euler rotations, projected grid bounds, and smooth Home orientation.\n";
    } catch (const std::exception& error) {
        std::cerr << "Reference plane test failed: " << error.what() << '\n';
        return 1;
    }
}
