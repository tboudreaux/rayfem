#include "interaction/spin.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void Near(Vector3 a, Vector3 b, const std::string& message, float tolerance = 1e-4f) {
    Check(Vector3Distance(a, b) < tolerance, message + " (" + std::to_string(a.x) + ", " + std::to_string(a.y) + ", " +
          std::to_string(a.z) + ")");
}

void Turntable() {
    Camera3D camera{{0, 2, 10}, {0, 2, 0}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    const auto start = camera;
    for (int i = 0; i < 360; ++i) RayFEM::SpinCamera(camera, {0, 1, 0}, camera.target, DEG2RAD);
    Near(camera.position, start.position, "A full turn returns the camera", 1e-3f);
    Near(camera.up, start.up, "A turntable keeps screen up");
    RayFEM::SpinCamera(camera, {0, 1, 0}, camera.target, PI / 2);
    Check(std::abs(Vector3Distance(camera.position, camera.target) - 10) < 1e-3f, "Spinning keeps the distance");
    // The scene turns by +90° about +Y, so the camera moves to where the
    // scene's -X side faces it: (-10, 2, 0).
    Near(camera.position, {-10, 2, 0}, "The scene turns counterclockwise about the axis seen from its positive end", 1e-3f);
}

void ModelAxisLine() {
    // An axis through (5, 0, 0) along +Z: the camera target, also on the
    // axis, stays put; a target off the axis circles it.
    Camera3D camera{{5, -10, 0}, {5, 0, 0}, {0, 0, 1}, 40, CAMERA_PERSPECTIVE};
    RayFEM::SpinCamera(camera, {0, 0, 3}, {5, 0, 0}, PI);
    Near(camera.target, {5, 0, 0}, "A target on the axis stays");
    Near(camera.position, {5, 10, 0}, "Half a turn puts the camera on the other side");
    Camera3D off{{0, -10, 0}, {0, 0, 0}, {0, 0, 1}, 40, CAMERA_PERSPECTIVE};
    RayFEM::SpinCamera(off, {0, 0, 1}, {5, 0, 0}, PI);
    Near(off.target, {10, 0, 0}, "A target off the axis circles the axis line");
    const auto before = off;
    RayFEM::SpinCamera(off, {0, 0, 0}, {5, 0, 0}, 1);
    Near(off.position, before.position, "A zero axis leaves the camera alone");
}

void Throw() {
    // Throwing turns the scene the same way the drag did.
    Camera3D dragged{{0, 0, 10}, {0, 0, 0}, {0, 1, 0}, 40, CAMERA_PERSPECTIVE};
    Camera3D thrown = dragged;
    const Vector2 velocity{300, -120};
    const float rate = .005f, seconds = .02f;
    RayFEM::OrbitCamera(dragged, Vector2Scale(velocity, seconds), rate);
    const Vector3 turn = RayFEM::ThrownSpin(thrown, velocity, rate);
    RayFEM::SpinCamera(thrown, turn, thrown.target, Vector3Length(turn) * seconds);
    Near(thrown.position, dragged.position, "A thrown spin continues the drag's motion", 2e-3f);

    RayFEM::DragVelocity drag;
    for (int i = 0; i <= 10; ++i) drag.Add(i * .016, {100.0f + 10 * i, 50});
    const Vector2 moving = drag.Release(10 * .016 + .016);
    Check(std::abs(moving.x - 625) < 1 && moving.y == 0, "Release velocity over the last tenth of a second: " +
          std::to_string(moving.x));
    Check(Vector2Length(drag.Release(10 * .016 + .2)) == 0, "A pointer at rest before release does not throw");
    drag.Reset();
    drag.Add(0, {0, 0});
    Check(Vector2Length(drag.Release(.01)) == 0, "One sample has no velocity");
}
} // namespace

int main() {
    try {
        Turntable();
        ModelAxisLine();
        Throw();
        std::cout << "spin passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "spin test failed: " << e.what() << '\n';
        return 1;
    }
}
