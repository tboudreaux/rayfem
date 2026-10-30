#pragma once

#include "libRayfem/profile/surface_trace.hpp"
#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace RayFEM::UI {

// A path over the outer surface through control points. Each segment lies in
// the plane through its two points and the center; rays from the center sweep
// that plane, so their surface hits trace the segment. The plane cut is a
// closed loop, giving two ways between the points: the short way (at most
// half a turn) or the long way around.
struct SurfacePathRays {
    std::vector<SurfaceTraceRay> rays;
    std::vector<double> control_parameters;  // cumulative angle at each control point (radians)
    std::vector<int> ambiguous_segments;     // points opposite through the center
};

// The arc of one segment: unit start direction, in-plane perpendicular and
// the signed sweep angle (positive is the short way).
struct SegmentArc { Vector3 start{}, across{}; double sweep = 0; bool ambiguous = false; };

inline SegmentArc MakeSegmentArc(Vector3 center, Vector3 from, Vector3 to, bool long_way, Vector3 fallback_normal) {
    const Vector3 a = Vector3Subtract(from, center), b = Vector3Subtract(to, center);
    const float la = Vector3Length(a), lb = Vector3Length(b);
    if (!(la > 0) || !(lb > 0)) throw std::invalid_argument("A path point coincides with the path center");
    SegmentArc arc;
    arc.start = Vector3Scale(a, 1/la);
    const Vector3 end = Vector3Scale(b, 1/lb);
    Vector3 normal = Vector3CrossProduct(arc.start, end);
    const double sine = Vector3Length(normal), cosine = Vector3DotProduct(arc.start, end);
    if (sine < 1e-6) {
        // Same or opposite directions: the plane is not defined by the points.
        arc.ambiguous = cosine < 0;
        // Reuse the previous segment's plane where possible; any normal
        // perpendicular to the start direction gives a valid plane.
        normal = Vector3Subtract(fallback_normal, Vector3Scale(arc.start, Vector3DotProduct(fallback_normal, arc.start)));
        if (Vector3Length(normal) < 1e-6) {
            const Vector3 axis = std::abs(arc.start.x) < .9f ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
            normal = Vector3CrossProduct(arc.start, axis);
        }
    }
    normal = Vector3Normalize(normal);
    arc.across = Vector3Normalize(Vector3CrossProduct(normal, arc.start));
    const double angle = std::atan2(sine, cosine);  // [0, pi]
    arc.sweep = long_way ? angle - 2*std::numbers::pi : angle;
    return arc;
}

inline Vector3 ArcDirection(const SegmentArc& arc, double phi) {
    return Vector3Add(Vector3Scale(arc.start, float(std::cos(phi))), Vector3Scale(arc.across, float(std::sin(phi))));
}

// Rays for the whole path. long_way has one entry per segment; a closed path
// adds the segment from the last point back to the first.
inline SurfacePathRays MakeSurfacePathRays(Vector3 center, const std::vector<Vector3>& points,
                                          const std::vector<bool>& long_way, bool closed, int total) {
    if (points.size() < 2) throw std::invalid_argument("A surface path needs at least two points");
    const std::size_t segments = closed ? points.size() : points.size() - 1;
    std::vector<SegmentArc> arcs;
    Vector3 previous_normal{0, 0, 1};
    double sweep_total = 0;
    for (std::size_t i = 0; i < segments; ++i) {
        const bool way = i < long_way.size() && long_way[i];
        auto arc = MakeSegmentArc(center, points[i], points[(i + 1) % points.size()], way, previous_normal);
        previous_normal = Vector3CrossProduct(arc.start, arc.across);
        sweep_total += std::abs(arc.sweep);
        arcs.push_back(arc);
    }
    SurfacePathRays result;
    double parameter = 0;
    const ProfilePosition origin{center.x, center.y, center.z};
    for (std::size_t i = 0; i < arcs.size(); ++i) {
        const auto& arc = arcs[i];
        if (arc.ambiguous) result.ambiguous_segments.push_back(int(i));
        result.control_parameters.push_back(parameter);
        const double share = sweep_total > 0 ? std::abs(arc.sweep)/sweep_total : 1.0/arcs.size();
        const int count = std::max(4, int(std::lround(std::max(2, total)*share)));
        // Each segment owns its start; the last one also includes its end.
        const bool last = i + 1 == arcs.size();
        const int steps = last ? count + 1 : count;
        for (int k = 0; k < steps; ++k) {
            const double t = double(k)/count;
            const Vector3 d = ArcDirection(arc, arc.sweep*t);
            result.rays.push_back({origin, {d.x, d.y, d.z}, parameter + std::abs(arc.sweep)*t});
        }
        parameter += std::abs(arc.sweep);
    }
    result.control_parameters.push_back(parameter);
    return result;
}

} // namespace RayFEM::UI
