#pragma once

// Axes a client reports for its model (FrameSchema::axes), such as a star's
// rotation axis: the view can be oriented by them and they can be drawn.

#include "interaction/camera_controller.hpp"
#include "libRayfem/network/packet.hpp"
#include <optional>
#include <utility>
#include <vector>

namespace RayFEM {

inline Vector3 ModelAxisDirection(const ModelAxis& axis) {
    const Vector3 d{static_cast<float>(axis.direction[0]), static_cast<float>(axis.direction[1]),
                    static_cast<float>(axis.direction[2])};
    const float length = Vector3Length(d);
    return CameraVectorFinite(d) && length > 1e-30f ? Vector3Scale(d, 1 / length) : Vector3{0, 0, 1};
}
inline Vector3 ModelAxisOrigin(const ModelAxis& axis) {
    return {static_cast<float>(axis.origin[0]), static_cast<float>(axis.origin[1]), static_cast<float>(axis.origin[2])};
}

// Home for a model with an axis: the axis points up the screen and the view
// is 20° above the model's equator. The side faced is the world front (+Z)
// projected onto the equator, or +X/+Y when the axis is close to it.
inline CameraViewBasis ModelAxisHomeBasis(Vector3 axis) {
    const Vector3 up = Vector3Normalize(axis);
    Vector3 horizontal{};
    for (const Vector3 front : {Vector3{0, 0, 1}, Vector3{1, 0, 0}, Vector3{0, 1, 0}}) {
        horizontal = Vector3Subtract(front, Vector3Scale(up, Vector3DotProduct(front, up)));
        if (Vector3Length(horizontal) > .2f) break;
    }
    horizontal = Vector3Normalize(horizontal);
    constexpr float elevation = 20 * DEG2RAD;
    const Vector3 back = Vector3Add(Vector3Scale(horizontal, std::cos(elevation)), Vector3Scale(up, std::sin(elevation)));
    const Vector3 screenUp = Vector3Subtract(Vector3Scale(up, std::cos(elevation)), Vector3Scale(horizontal, std::sin(elevation)));
    return {Vector3Normalize(Vector3CrossProduct(screenUp, back)), Vector3Normalize(screenUp), Vector3Normalize(back)};
}

// Ease the camera to basis, keeping its target and distance.
inline bool BeginCameraBasisTransition(CameraViewTransition& transition, const Camera3D& camera,
                                       const CameraViewBasis& basis, double now, double duration = .55) {
    if (!BeginCameraViewTransition(transition, camera, basis.back, now, duration)) return false;
    transition.destination = basis;
    transition.to = CameraBasisQuaternion(basis);
    return true;
}

// The drawn part of an axis: through its origin, past the model's bounding
// sphere by a quarter of its radius at each end.
inline std::pair<Vector3, Vector3> ModelAxisSegment(const ModelAxis& axis, BoundingBox bounds) {
    const Vector3 d = ModelAxisDirection(axis), origin = ModelAxisOrigin(axis);
    const Vector3 center = Vector3Scale(Vector3Add(bounds.min, bounds.max), .5f);
    const float radius = std::max(1e-6f, Vector3Length(Vector3Subtract(bounds.max, bounds.min)) * .5f);
    const float middle = Vector3DotProduct(Vector3Subtract(center, origin), d);
    const float reach = radius * 1.25f;
    return {Vector3Add(origin, Vector3Scale(d, middle - reach)), Vector3Add(origin, Vector3Scale(d, middle + reach))};
}

} // namespace RayFEM
