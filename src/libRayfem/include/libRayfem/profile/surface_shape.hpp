#pragma once
#include "libRayfem/profile/surface_trace.hpp"
namespace RayFEM {
// The outer surface's shape about an axis: its radius from a center along
// rays at each colatitude of several meridians, and around the equator. With
// a rotation axis, this is a rotating model's oblateness. Rays go out from
// the center and stop at the nearest exterior face, as surface traces do.
struct SurfaceShapeOptions {
    ProfilePosition center{}, axis{0, 0, 1};
    // Azimuth 0, projected onto the equatorial plane.
    ProfilePosition reference{1, 0, 0};
    int colatitudes = 181;  // per meridian, pole to pole
    int meridians = 8;      // evenly spaced in azimuth
    int equator = 180;      // rays around the equator
};
// One colatitude across the meridians.
struct SurfaceShapeSample {
    double colatitude = 0;  // radians
    double mean = std::numeric_limits<double>::quiet_NaN(), min = mean, max = mean;
    int hits = 0;
};
struct SurfaceShape {
    std::vector<SurfaceShapeSample> profile;
    std::vector<double> equator_azimuth, equator_radius;  // radians; NaN where a ray missed
    // Radii in mesh units; NaN when the rays they need missed.
    double north = std::numeric_limits<double>::quiet_NaN(), south = north;
    double polar = north;                       // mean of north and south
    double equatorial = north;                  // mean around the equator
    double equatorial_min = north, equatorial_max = north;
    double volume_radius = north;               // radius of the sphere of equal volume
    double flattening = north;                  // 1 - polar/equatorial
    double axis_ratio = north;                  // polar/equatorial
    double equatorial_ellipticity = north;      // 1 - min/max around the equator
    double max_residual = 0;
    std::size_t rays = 0, hits = 0;
};
// Meridians first (colatitudes pole to pole for each), then the equator.
std::vector<SurfaceTraceRay> MakeSurfaceShapeRays(const SurfaceShapeOptions& options);
// samples: the surface trace of MakeSurfaceShapeRays(options), in order.
SurfaceShape MeasureSurfaceShape(const SurfaceShapeOptions& options, const std::vector<SurfaceTraceSample>& samples);
// Trace and measure in one step. The rays go in batches small enough for the
// trace's per-call limits, so meshes with many surface faces work too.
// samples, if given, receives the trace in ray order.
SurfaceShape SampleSurfaceShape(const SurfaceTraceRegistry& registry, mfem::Mesh& mesh, mfem::GridFunction& field,
                                const SurfaceShapeOptions& options, const RenderOptions& render = {},
                                std::vector<SurfaceTraceSample>* samples = nullptr);
void ExportSurfaceShapeCSV(std::ostream& stream, const SurfaceShape& shape, const ProfileMetadata& metadata);
} // namespace RayFEM
