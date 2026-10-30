#pragma once

#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>

namespace RayFEM {

struct CameraViewBasis {
    Vector3 right{1, 0, 0};
    Vector3 up{0, 1, 0};
    // Back points from the target toward the camera.
    Vector3 back{0, 0, 1};
};

inline bool CameraVectorFinite(Vector3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline Vector3 CameraCanonicalUp(Vector3 back) {
    // Top and bottom views need an explicit screen-up direction: world Y is
    // parallel to the viewing axis there.
    if (std::abs(back.y) > 0.99999f) {
        return back.y > 0 ? Vector3{0, 0, -1} : Vector3{0, 0, 1};
    }
    return {0, 1, 0};
}

inline CameraViewBasis CameraBasis(const Camera3D& camera) {
    CameraViewBasis basis;
    const Vector3 offset = Vector3Subtract(camera.position, camera.target);
    const float length = Vector3Length(offset);
    if (CameraVectorFinite(offset) && std::isfinite(length) && length > 1e-12f) {
        basis.back = Vector3Scale(offset, 1 / length);
    }
    Vector3 up = CameraVectorFinite(camera.up) ? camera.up : CameraCanonicalUp(basis.back);
    Vector3 right = Vector3CrossProduct(up, basis.back);
    if (!CameraVectorFinite(right) || Vector3LengthSqr(right) < 1e-12f) {
        right = Vector3CrossProduct(CameraCanonicalUp(basis.back), basis.back);
    }
    basis.right = Vector3Normalize(right);
    basis.up = Vector3Normalize(Vector3CrossProduct(basis.back, basis.right));
    return basis;
}

inline Quaternion CameraBasisQuaternion(const CameraViewBasis& basis) {
    Matrix rotation = MatrixIdentity();
    rotation.m0 = basis.right.x;
    rotation.m1 = basis.right.y;
    rotation.m2 = basis.right.z;
    rotation.m4 = basis.up.x;
    rotation.m5 = basis.up.y;
    rotation.m6 = basis.up.z;
    rotation.m8 = basis.back.x;
    rotation.m9 = basis.back.y;
    rotation.m10 = basis.back.z;
    return QuaternionNormalize(QuaternionFromMatrix(rotation));
}

struct CameraViewTransition {
    bool active = false;
    double started_at = 0;
    double duration = 0.55;
    Vector3 target{};
    float distance = 0;
    Quaternion from{0, 0, 0, 1};
    Quaternion to{0, 0, 0, 1};
    CameraViewBasis destination;
    // Keep roll travel unwrapped: a full revolution has the same final
    // quaternion as no movement, but must still animate through every angle.
    bool rolling = false;
    double roll_degrees = 0;
    double roll_applied_degrees = 0;
    CameraViewBasis roll_base_destination;
};

// Quintic easing has zero velocity and acceleration at both ends, so neither
// starting nor arriving produces an abrupt camera impulse.
inline double CameraViewEase(double progress) {
    const double t = std::clamp(progress, 0.0, 1.0);
    return t * t * t * (t * (t * 6 - 15) + 10);
}

inline bool BeginCameraViewTransition(CameraViewTransition& transition, const Camera3D& camera,
                                      Vector3 direction, double now, double duration = 0.55) {
    transition.active = false;
    transition.rolling = false;
    const float direction_length = Vector3Length(direction);
    const float distance = Vector3Distance(camera.position, camera.target);
    if (!CameraVectorFinite(direction) || !CameraVectorFinite(camera.position) ||
        !CameraVectorFinite(camera.target) || !std::isfinite(direction_length) ||
        !std::isfinite(distance) || direction_length < 1e-12f || distance < 1e-12f ||
        !std::isfinite(now) || !std::isfinite(duration) || duration < 0) {
        return false;
    }
    const Vector3 back = Vector3Scale(direction, 1 / direction_length);
    // Build the desired frame around the origin to avoid precision loss when
    // a small model has a large world-space translation.
    const Camera3D desired{back, {0, 0, 0}, CameraCanonicalUp(back), camera.fovy, camera.projection};
    transition.target = camera.target;
    transition.distance = distance;
    transition.from = CameraBasisQuaternion(CameraBasis(camera));
    transition.destination = CameraBasis(desired);
    transition.to = CameraBasisQuaternion(transition.destination);
    transition.started_at = now;
    transition.duration = duration;
    transition.active = true;
    return true;
}

enum class CameraRotationAxis { Up, Right, Back };

inline CameraViewBasis CameraRolledBasis(CameraViewBasis basis, double degrees) {
    const Quaternion rotation = QuaternionFromAxisAngle(basis.back,
        static_cast<float>(std::remainder(degrees, 360.0) * DEG2RAD));
    basis.up = Vector3RotateByQuaternion(basis.up, rotation);
    basis.right = Vector3RotateByQuaternion(basis.right, rotation);
    return basis;
}

// Steps are relative to the camera frame, including its roll. While an earlier
// transition is still moving, rotate its destination so quick repeated clicks
// retain every requested increment. The new animation starts at the displayed
// camera, without changing its position at the instant of the click.
inline bool BeginCameraRotationStep(CameraViewTransition& transition, const Camera3D& camera,
                                    CameraRotationAxis axis, double degrees, double now,
                                    double duration = 0.35) {
    const float distance = Vector3Distance(camera.position, camera.target);
    if (!CameraVectorFinite(camera.position) || !CameraVectorFinite(camera.target) ||
        !CameraVectorFinite(camera.up) || !std::isfinite(distance) || distance < 1e-12f ||
        !std::isfinite(degrees) || !std::isfinite(now) || !std::isfinite(duration) || duration < 0) {
        return false;
    }
    if (degrees == 0) { return false; }
    const CameraViewBasis source = transition.active ? transition.destination : CameraBasis(camera);
    CameraViewTransition next;
    next.target = transition.active ? transition.target : camera.target;
    next.distance = transition.active ? transition.distance : distance;
    next.from = CameraBasisQuaternion(CameraBasis(camera));
    next.started_at = now;
    next.duration = duration;
    next.active = true;
    if (axis == CameraRotationAxis::Back) {
        next.rolling = true;
        next.roll_degrees = degrees;
        next.roll_base_destination = source;
        if (transition.active && transition.rolling) {
            next.roll_degrees += transition.roll_degrees - transition.roll_applied_degrees;
            if (!std::isfinite(next.roll_degrees)) { return false; }
            // Retain the roll already displayed while finishing any base
            // orientation change, such as a cube snap interrupted by a click.
            next.roll_base_destination = CameraRolledBasis(
                transition.roll_base_destination, transition.roll_applied_degrees);
        }
        next.to = CameraBasisQuaternion(next.roll_base_destination);
        next.destination = CameraRolledBasis(next.roll_base_destination, next.roll_degrees);
        transition = next;
        return true;
    }
    const double angle = std::remainder(degrees, 360.0);
    if (angle == 0) { return false; }
    Vector3 rotation_axis;
    switch (axis) {
        case CameraRotationAxis::Up: rotation_axis = source.up; break;
        case CameraRotationAxis::Right: rotation_axis = source.right; break;
        case CameraRotationAxis::Back: rotation_axis = source.back; break;
        default: return false;
    }
    const Quaternion rotation = QuaternionFromAxisAngle(rotation_axis, static_cast<float>(angle * DEG2RAD));
    const Vector3 back = axis == CameraRotationAxis::Back ? source.back :
        Vector3RotateByQuaternion(source.back, rotation);
    const Vector3 up = axis == CameraRotationAxis::Up ? source.up :
        Vector3RotateByQuaternion(source.up, rotation);
    const Camera3D desired{back, {0, 0, 0}, up, camera.fovy, camera.projection};
    next.destination = CameraBasis(desired);
    next.to = CameraBasisQuaternion(next.destination);
    transition = next;
    return true;
}

// This remains independent of clocks and input. The caller cancels with
// transition.active = false before applying a manual drag, pan, zoom, or fit.
inline bool UpdateCameraViewTransition(Camera3D& camera, CameraViewTransition& transition, double now) {
    if (!transition.active || !std::isfinite(now)) { return false; }
    const double progress = transition.duration == 0 ? 1.0 :
        std::clamp((now - transition.started_at) / transition.duration, 0.0, 1.0);
    if (progress <= 0) { return false; }
    CameraViewBasis basis = transition.destination;
    if (progress < 1) {
        const double eased = CameraViewEase(progress);
        const Quaternion rotation = QuaternionNormalize(QuaternionSlerp(
            transition.from, transition.to, static_cast<float>(eased)));
        basis.right = Vector3RotateByQuaternion({1, 0, 0}, rotation);
        basis.back = Vector3RotateByQuaternion({0, 0, 1}, rotation);
        basis.up = Vector3RotateByQuaternion({0, 1, 0}, rotation);
        if (transition.rolling) {
            transition.roll_applied_degrees = transition.roll_degrees * eased;
            basis = CameraRolledBasis(basis, transition.roll_applied_degrees);
        }
    } else {
        transition.roll_applied_degrees = transition.roll_degrees;
        transition.active = false;
    }
    const Vector3 position = Vector3Add(transition.target, Vector3Scale(basis.back, transition.distance));
    const bool changed = position.x != camera.position.x || position.y != camera.position.y ||
        position.z != camera.position.z || basis.up.x != camera.up.x || basis.up.y != camera.up.y ||
        basis.up.z != camera.up.z;
    camera.position = position;
    camera.target = transition.target;
    camera.up = basis.up;
    return changed;
}

struct CameraDragState {
    bool orbiting = false;
    bool panning = false;
    Vector2 previous_mouse{};
};

// Capture these values once per UI tick from the same input source. A press
// inside the viewport owns the drag until release, even after leaving it.
// cancel_drag lets a profile handle or lost window focus relinquish ownership.
struct CameraInputSnapshot {
    Vector2 mouse{};
    bool viewport_hovered = false;
    bool left_pressed = false;
    bool left_down = false;
    bool middle_pressed = false;
    bool middle_down = false;
    float wheel = 0;
    bool cancel_drag = false;
};

// A captured selection press becomes an orbit only after deliberate motion.
// Keep the full initial movement when crossing the threshold, and latch it so
// dragging back to the press point can never turn the gesture into a click.
inline void PrepareCameraOrbitGesture(CameraInputSnapshot& input, CameraDragState& drag,
                                      Vector2 press, bool& moved) {
    const bool begin = !moved && Vector2DistanceSqr(input.mouse, press) > 16;
    moved = moved || begin;
    input.left_pressed = false;
    if (!moved) {
        input.left_down = false;
    } else if (begin && input.left_down && !input.cancel_drag) {
        drag.orbiting = true;
        drag.previous_mouse = press;
    }
}

// Rotate the camera about its target as dragging by delta pixels does: the
// scene follows the pointer. This rotates the current screen frame rather
// than rebuilding yaw/pitch against world Y, which keeps a snapped pole or a
// partly completed view transition continuous when the user takes over.
inline void OrbitCamera(Camera3D& camera, Vector2 delta, float radians_per_pixel) {
    const float distance = Vector3Distance(camera.position, camera.target);
    if (!std::isfinite(distance) || distance < 1e-12f) return;
    auto basis = CameraBasis(camera);
    const Quaternion yaw = QuaternionFromAxisAngle(basis.up, -delta.x * radians_per_pixel);
    basis.back = Vector3RotateByQuaternion(basis.back, yaw);
    basis.right = Vector3RotateByQuaternion(basis.right, yaw);
    const Quaternion pitch = QuaternionFromAxisAngle(basis.right, -delta.y * radians_per_pixel);
    basis.back = Vector3Normalize(Vector3RotateByQuaternion(basis.back, pitch));
    basis.up = Vector3Normalize(Vector3RotateByQuaternion(basis.up, pitch));
    camera.position = Vector3Add(camera.target, Vector3Scale(basis.back, distance));
    camera.up = basis.up;
}

inline bool CameraViewportContains(Vector2 mouse, Rectangle viewport) {
    return mouse.x >= viewport.x && mouse.x < viewport.x + viewport.width &&
           mouse.y >= viewport.y && mouse.y < viewport.y + viewport.height;
}

// Pure controller update: no polling, ImGui state, graphics context, or clock.
// Returns true only when camera state changes, for the viewer's cameraAdjusted
// flag. Fitting a new mesh remains an explicit decision in the caller.
inline bool UpdateCameraController(Camera3D& camera, CameraDragState& drag,
                                   const CameraInputSnapshot& input, float radius) {
    if (input.cancel_drag || !std::isfinite(input.mouse.x) || !std::isfinite(input.mouse.y)) {
        drag.orbiting = drag.panning = false;
        drag.previous_mouse = input.mouse;
        return false;
    }
    if (input.viewport_hovered && input.left_pressed) {
        drag.orbiting = true;
        drag.previous_mouse = input.mouse;
    }
    if (input.viewport_hovered && input.middle_pressed) {
        drag.panning = true;
        drag.previous_mouse = input.mouse;
    }
    if (!input.left_down) { drag.orbiting = false; }
    if (!input.middle_down) { drag.panning = false; }
    Vector2 delta = Vector2Subtract(input.mouse, drag.previous_mouse);
    drag.previous_mouse = input.mouse;

    const float extent = std::isfinite(radius) ? std::max(radius, 1e-6f) : 1.0f;
    const float min_distance = std::max(0.02f, extent * 0.001f);
    const float max_distance = std::max(min_distance, extent * 10000.0f);
    bool changed = false;
    Vector3 offset = Vector3Subtract(camera.position, camera.target);
    float distance = Vector3Length(offset);
    // A coincident camera and target has no view direction. Recover only when
    // an actual interaction requires it, using the ordinary forward axis.
    auto ensure_direction = [&] {
        if (!std::isfinite(distance) || distance < 1e-12f) {
            distance = min_distance;
            offset = {0, 0, distance};
            camera.position = Vector3Add(camera.target, offset);
            changed = true;
        }
    };
    if (input.viewport_hovered && std::isfinite(input.wheel) && input.wheel != 0) {
        ensure_direction();
        const double requested = double(distance) * std::exp(-double(input.wheel) * 0.12);
        const float new_distance = static_cast<float>(std::clamp(requested, double(min_distance), double(max_distance)));
        changed = changed || new_distance != distance;
        distance = new_distance;
        offset = Vector3Scale(Vector3Normalize(offset), distance);
        camera.position = Vector3Add(camera.target, offset);
    }
    if (drag.orbiting && (delta.x != 0 || delta.y != 0)) {
        ensure_direction();
        OrbitCamera(camera, delta, 0.005f);
        changed = true;
    }
    if (drag.panning && (delta.x != 0 || delta.y != 0)) {
        ensure_direction();
        const auto basis = CameraBasis(camera);
        const float scale = distance * 0.0015f;
        const Vector3 move = Vector3Add(Vector3Scale(basis.right, -delta.x * scale),
                                        Vector3Scale(basis.up, delta.y * scale));
        camera.position = Vector3Add(camera.position, move);
        camera.target = Vector3Add(camera.target, move);
        changed = true;
    }
    return changed;
}
} // namespace RayFEM
