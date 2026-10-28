#pragma once

#include "libRayfem/profile/sampler.hpp"

namespace RayFEM {

// Each ray is independent; parameter is an application-defined path coordinate.
// Directions are normalized. The nearest nonnegative exterior hit is selected.
struct SurfaceTraceRay {
    ProfilePosition origin{}, direction{};
    double parameter = 0;
};

struct SurfaceTraceSample {
    ProfilePosition position{}, reference{};
    double parameter = 0;
    // Sum of chord lengths between consecutive valid samples, never across gaps.
    double distance = std::numeric_limits<double>::quiet_NaN();
    double ray_distance = std::numeric_limits<double>::quiet_NaN();
    double value = std::numeric_limits<double>::quiet_NaN();
    // Perpendicular distance from the requested ray to the actual curved face.
    double residual = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
    int element = -1, face = -1, boundary = -1, boundary_attribute = -1, material = -1;
};

// Axis and the projected reference direction define an orthonormal spherical
// frame. All angular arguments/parameters use radians; both endpoints included.
std::vector<SurfaceTraceRay> MakeMeridianRays(ProfilePosition center, ProfilePosition axis,
    ProfilePosition reference_direction, double azimuth, double theta_begin, double theta_end, int count);
std::vector<SurfaceTraceRay> MakeAzimuthRays(ProfilePosition center, ProfilePosition axis,
    ProfilePosition reference_direction, double colatitude, double phi_begin, double phi_end, int count);

class SurfaceTraceAlgorithm {
public:
    virtual ~SurfaceTraceAlgorithm() = default;
    virtual std::string id() const = 0;
    virtual std::string name() const = 0;
    virtual std::vector<SurfaceTraceSample> sample(mfem::Mesh& mesh, mfem::GridFunction& field,
        const std::vector<SurfaceTraceRay>& rays, const RenderOptions& options) const = 0;
};

class SurfaceTraceRegistry {
public:
    // curved: refined-triangle seeds followed by a curved-face Newton solve.
    // refined: triangle barycentrics mapped back to the actual face, so field
    // evaluation is exact there but ray alignment is approximate (see residual).
    // Only original exterior faces are traced, not newly exposed material
    // interfaces. Embedded 2D surfaces in 3D are supported as well as 3D volumes.
    // Coplanar rays and nonconvergent intersections remain explicit gaps.
    // Oversized synchronous requests throw: at most 250000 seed triangles,
    // 24 million ray/triangle checks and 100000 curved ray/face solves.
    SurfaceTraceRegistry();
    void registerAlgorithm(std::shared_ptr<const SurfaceTraceAlgorithm> algorithm);
    std::vector<ProfileAlgorithmInfo> algorithms() const;
    std::vector<SurfaceTraceSample> sample(const std::string& id, mfem::Mesh& mesh,
        mfem::GridFunction& field, const std::vector<SurfaceTraceRay>& rays,
        const RenderOptions& options = {}) const;
private:
    std::vector<std::shared_ptr<const SurfaceTraceAlgorithm>> algorithms_;
};

void ExportSurfaceTraceCSV(std::ostream& stream, const std::vector<SurfaceTraceSample>& samples,
                           const ProfileMetadata& metadata);

} // namespace RayFEM
