#pragma once

// A purely visual spin of the view, as in GLVis: the scene keeps turning
// about an axis at a steady rate. It moves the camera only; the data, its
// time and the model's own rotation are untouched.

#include "interaction/camera_controller.hpp"
#include <vector>

namespace RayFEM {

// Turn the scene by radians about the line through point along direction,
// by moving the camera (position, target and up) the opposite way.
inline void SpinCamera(Camera3D& camera, Vector3 direction, Vector3 point, float radians) {
    const float length = Vector3Length(direction);
    if (!CameraVectorFinite(direction) || !CameraVectorFinite(point) || !(length > 1e-12f) || !std::isfinite(radians)) return;
    const Quaternion turn = QuaternionFromAxisAngle(Vector3Scale(direction, 1 / length), -radians);
    auto rotate = [&](Vector3 p) { return Vector3Add(point, Vector3RotateByQuaternion(Vector3Subtract(p, point), turn)); };
    camera.position = rotate(camera.position);
    camera.target = rotate(camera.target);
    camera.up = Vector3Normalize(Vector3RotateByQuaternion(camera.up, turn));
}

// The scene's angular velocity (axis times radians per second) that an orbit
// drag implies when released while the pointer moves at velocity pixels per
// second: OrbitCamera turns the scene by rate per pixel about screen up for
// horizontal motion and about screen right for vertical motion.
inline Vector3 ThrownSpin(const Camera3D& camera, Vector2 velocity, float radians_per_pixel) {
    const auto basis = CameraBasis(camera);
    return Vector3Scale(Vector3Add(Vector3Scale(basis.up, velocity.x), Vector3Scale(basis.right, velocity.y)),
                        radians_per_pixel);
}

// Pointer positions during a drag, for its velocity when it is released.
class DragVelocity {
public:
    void Reset() { samples_.clear(); }
    void Add(double time, Vector2 mouse) {
        samples_.push_back({time, mouse});
        while (samples_.size() > 2 && time - samples_.front().time > .25) samples_.erase(samples_.begin());
    }
    // Pixels per second over the last window seconds; zero when the pointer
    // had come to rest before release, so a careful drag never throws.
    Vector2 Release(double now, double window = .1, double rest = .06) const {
        if (samples_.size() < 2 || now - samples_.back().time > rest) return {};
        size_t first = samples_.size() - 1;
        while (first > 0 && now - samples_[first - 1].time <= window) --first;
        if (first == samples_.size() - 1) first = samples_.size() - 2;
        const auto& a = samples_[first];
        const auto& b = samples_.back();
        const double span = b.time - a.time;
        if (!(span > 1e-4)) return {};
        return {static_cast<float>((b.mouse.x - a.mouse.x) / span), static_cast<float>((b.mouse.y - a.mouse.y) / span)};
    }
private:
    struct Sample { double time; Vector2 mouse; };
    std::vector<Sample> samples_;
};

} // namespace RayFEM
