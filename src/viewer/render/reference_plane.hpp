#pragma once

#include "interaction/camera_controller.hpp"
#include <array>

namespace RayFEM {

// Start with XY and apply X, then Y, then Z rotations about the world axes.
// Keeping the entire frame gives Home a stable screen-up direction even when
// the normal is vertical, or when only the plane's in-plane rotation changes.
inline CameraViewBasis ReferencePlaneBasis(Vector3 degrees) {
    if (!CameraVectorFinite(degrees)) { return {}; }
    const Quaternion rotation = QuaternionNormalize(QuaternionFromEuler(
        std::remainder(degrees.x, 360.0f) * DEG2RAD,
        std::remainder(degrees.y, 360.0f) * DEG2RAD,
        std::remainder(degrees.z, 360.0f) * DEG2RAD));
    return {
        Vector3RotateByQuaternion({1, 0, 0}, rotation),
        Vector3RotateByQuaternion({0, 1, 0}, rotation),
        Vector3RotateByQuaternion({0, 0, 1}, rotation)
    };
}

struct ReferencePlaneLayout {
    Vector3 center{};
    Vector3 u{1, 0, 0};
    Vector3 v{0, 1, 0};
    float half_extent = 5;
    std::array<Vector3, 4> corners{};
    BoundingBox bounds{};
};

// The infinite reference plane always passes through the world origin. Only
// the visible patch follows the mesh's projected center, so translated meshes
// remain near the grid without moving the reference plane itself.
inline ReferencePlaneLayout ReferencePlaneGrid(BoundingBox scene, float radius,
                                                bool has_mesh, Vector3 rotation_degrees) {
    ReferencePlaneLayout grid;
    const auto basis = ReferencePlaneBasis(rotation_degrees);
    grid.u = basis.right;
    grid.v = basis.up;
    if (has_mesh && CameraVectorFinite(scene.min) && CameraVectorFinite(scene.max) &&
        scene.min.x <= scene.max.x && scene.min.y <= scene.max.y && scene.min.z <= scene.max.z) {
        const Vector3 center = Vector3Add(Vector3Scale(scene.min, .5f), Vector3Scale(scene.max, .5f));
        grid.center = Vector3Subtract(center, Vector3Scale(basis.back, Vector3DotProduct(center, basis.back)));
        if (!CameraVectorFinite(grid.center)) { grid.center = {}; }
    }
    if (has_mesh && std::isfinite(radius) && radius >= 0) {
        grid.half_extent = std::max(radius * .4f, 1e-6f) * 5;
        if (!std::isfinite(grid.half_extent)) { grid.half_extent = 5; }
    }
    const Vector3 u = Vector3Scale(grid.u, grid.half_extent);
    const Vector3 v = Vector3Scale(grid.v, grid.half_extent);
    grid.corners = {
        Vector3Subtract(Vector3Subtract(grid.center, u), v),
        Vector3Subtract(Vector3Add(grid.center, u), v),
        Vector3Add(Vector3Add(grid.center, u), v),
        Vector3Add(Vector3Subtract(grid.center, u), v)
    };
    grid.bounds = {grid.corners[0], grid.corners[0]};
    for (const auto point : grid.corners) {
        grid.bounds.min = Vector3Min(grid.bounds.min, point);
        grid.bounds.max = Vector3Max(grid.bounds.max, point);
    }
    return grid;
}

// Home is elevated 30 degrees above the plane, looking along local +Y.
inline CameraViewBasis ReferencePlaneHomeBasis(Vector3 rotation_degrees) {
    const auto plane=ReferencePlaneBasis(rotation_degrees);
    constexpr float elevation=30*DEG2RAD;
    return {plane.right,
        Vector3Add(Vector3Scale(plane.up,std::sin(elevation)),Vector3Scale(plane.back,std::cos(elevation))),
        Vector3Add(Vector3Scale(plane.up,-std::cos(elevation)),Vector3Scale(plane.back,std::sin(elevation)))};
}

inline bool BeginCameraHomeTransition(CameraViewTransition& transition, const Camera3D& camera,
                                      Vector3 rotation_degrees, double now, double duration = .55) {
    const auto basis = ReferencePlaneHomeBasis(rotation_degrees);
    if (!BeginCameraViewTransition(transition, camera, basis.back, now, duration)) { return false; }
    transition.destination = basis;
    transition.to = CameraBasisQuaternion(basis);
    return true;
}

} // namespace RayFEM
