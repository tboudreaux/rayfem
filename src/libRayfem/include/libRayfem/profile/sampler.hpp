#pragma once

#include "libRayfem/mesh/converter.hpp"
#include <array>
#include <cstdint>
#include <iosfwd>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace RayFEM {

using ProfilePosition = std::array<double, 3>;

struct ProfileSample {
    ProfilePosition position{};
    double s = std::numeric_limits<double>::quiet_NaN();
    double u = std::numeric_limits<double>::quiet_NaN();
    double v = std::numeric_limits<double>::quiet_NaN();
    double value = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
    int element = -1;
};

// Ray distance s includes both endpoints; direction is normalized internally.
std::vector<ProfileSample> MakeRaySamples(ProfilePosition origin, ProfilePosition direction,
                                          double length, int count);
// The plane's in-plane u and v axes for a normal (normalized internally); the
// same basis MakePlaneSamples uses, so callers can place handles on the plane.
std::array<ProfilePosition, 2> PlaneAxes(ProfilePosition normal);
// Plane coordinates u/v span [-width/2,width/2] and [-height/2,height/2].
// Samples are row-major (u varies fastest), using a stable orthonormal basis.
std::vector<ProfileSample> MakePlaneSamples(ProfilePosition center, ProfilePosition normal,
                                            double width, double height, int columns, int rows);

// Exact copy of the displayed nodal geometry, shared by inverse profiles and
// physical integration. Returns nullptr when no displacement is active. A
// displacement in a different geometry basis is explicitly unavailable.
std::unique_ptr<mfem::Mesh> MakeDisplacedMesh(mfem::Mesh& mesh, const RenderOptions& options);

class ProfileAlgorithm {
public:
    virtual ~ProfileAlgorithm() = default;
    virtual std::string id() const = 0;
    virtual std::string name() const = 0;
    virtual std::vector<ProfileSample> sample(mfem::Mesh& mesh, mfem::GridFunction& field,
                                             const std::vector<ProfileSample>& locations,
                                             const RenderOptions& options) const = 0;
};

struct ProfileAlgorithmInfo { std::string id, name; };

class ProfileRegistry {
public:
    // Built-ins: exhaustive and coherent. Both evaluate actual FE values via
    // inverse element maps. Shared-boundary ties select the lowest element ID.
    ProfileRegistry();
    // An existing ID is replaced, allowing applications to update a plugin.
    void registerAlgorithm(std::shared_ptr<const ProfileAlgorithm> algorithm);
    std::vector<ProfileAlgorithmInfo> algorithms() const;
    std::vector<ProfileSample> sample(const std::string& id, mfem::Mesh& mesh,
                                     mfem::GridFunction& field,
                                     const std::vector<ProfileSample>& locations,
                                     const RenderOptions& options = {}) const;
private:
    std::vector<std::shared_ptr<const ProfileAlgorithm>> algorithms_;
};

struct ProfileMetadata {
    std::string field_name;
    std::uint64_t frame = 0;
    double time = 0.0;
    FieldView view = FieldView::Value;
    int component = 0;
    std::string algorithm;
    std::vector<int> excluded_materials;
    std::vector<int> excluded_elements;
};

// RFC 4180 escaping, classic-locale full precision. Metadata is repeated in
// each data row so ordinary CSV readers can consume the complete table.
void ExportProfileCSV(std::ostream& stream, const std::vector<ProfileSample>& samples,
                      const ProfileMetadata& metadata);

} // namespace RayFEM
