#pragma once
#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/sampler.hpp"
#include <deque>
#include <map>
#include <optional>
#include <tuple>

namespace RayFEM {

enum class MeasurementKind { DofCoefficient, Minimum, Maximum, Mean, Integral, L2Norm, PointProbe };
const char* MeasurementName(MeasurementKind kind);

struct ElementSelectionScope {
    std::uint64_t session_id = 0, mesh_id = 0, mesh_revision = 0;
    bool operator==(const ElementSelectionScope&) const = default;
};

struct TrackingRequest {
    std::string field_name;
    MeasurementKind kind = MeasurementKind::DofCoefficient;
    // Global scalar-space DOF and coefficient component (DofToVDof mapping).
    // ND/RT directional coefficients have one component, regardless of VectorDim.
    int dof = 0;
    int component = 0;
    FieldView view = FieldView::Value;
    bool use_displacement = true;
    double displacement_scale = 1.0;
    // 0 selects an order based on geometry and field orders. Min/max are over
    // quadrature points; integral/mean/L2 use physical Jacobian weights.
    int quadrature_order = 0;
    ProfilePosition probe{};
    std::string algorithm = "coherent";
    std::vector<int> excluded_materials;
    std::vector<int> excluded_elements;
    // A viewer's picked element indices belong to one mesh identity. An unset
    // scope applies the caller's explicit exclusions to every appended frame.
    std::optional<ElementSelectionScope> element_scope;
};

struct MeasurementSample {
    std::uint64_t session_id = 0, mesh_id = 0, mesh_revision = 0;
    std::uint32_t frame = 0;
    double time = 0.0;
    double value = std::numeric_limits<double>::quiet_NaN();
    double measure = std::numeric_limits<double>::quiet_NaN();
    bool valid = false;
    std::string error;
    // Effective selection for this sample: user exclusions plus any materials
    // outside the field's intrinsic support in this particular frame.
    std::vector<int> excluded_materials;
    std::vector<int> excluded_elements;
};

class FrameTracker {
public:
    explicit FrameTracker(TrackingRequest request, std::size_t capacity = 4096);
    // Explicitly bind a clicked DOF to the current frame before backfilling
    // older history. Otherwise the first valid DOF sample establishes binding.
    void bindDof(const RenderFrame& frame);
    // False for repeated or stale frame indices within the same stream/mesh
    // revision. Unsupported/missing data append an invalid sample with a reason.
    bool append(RenderFrame& frame);
    const std::deque<MeasurementSample>& samples() const { return samples_; }
    const TrackingRequest& request() const { return request_; }
    void clear();
    // Rebuild values under a new material selection without rebinding a clicked
    // DOF to another session/topology. Requeue retained frames after this call.
    void setExcludedMaterials(std::vector<int> excluded_materials);
    void setExcludedElements(std::vector<int> excluded_elements,
                             std::optional<ElementSelectionScope> scope = {});
    ProfileRegistry& registry() { return registry_; }
private:
    struct DofBinding {
        std::uint64_t session = 0, mesh = 0, revision = 0, layout = 0;
        std::string collection;
        int ndofs = 0, vdim = 0;
    };
    DofBinding bindingFor(const RenderFrame& frame, mfem::GridFunction& field) const;
    double dofValue(const RenderFrame& frame, mfem::GridFunction& field);
    TrackingRequest request_;
    std::size_t capacity_;
    std::deque<MeasurementSample> samples_;
    // Bounded independently from samples: up to 256 stream/revision identities.
    // A new identity beyond that limit requires clear(), preserving deduplication
    // without retaining an unbounded ID history or any RenderFrame objects.
    std::map<std::tuple<std::uint64_t,std::uint64_t,std::uint64_t>,std::uint32_t> latest_;
    std::optional<DofBinding> binding_;
    ProfileRegistry registry_;
};

void ExportTrackingCSV(std::ostream& stream, const FrameTracker& tracker);

} // namespace RayFEM
