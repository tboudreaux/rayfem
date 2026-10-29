#include "libRayfem/profile/sampler.hpp"
#include "libRayfem/threading.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {
using RayFEM::ProfilePosition;
using RayFEM::ProfileSample;

bool Finite(const ProfilePosition& position) {
    return std::all_of(position.begin(), position.end(), [](double x) { return std::isfinite(x); });
}
ProfilePosition Normalize(ProfilePosition direction) {
    if (!Finite(direction)) { throw std::invalid_argument("Profile directions must be finite"); }
    const double length = std::hypot(direction[0], direction[1], direction[2]);
    if (!std::isfinite(length) || length <= 0) { throw std::invalid_argument("Profile direction must be nonzero"); }
    for (double& x : direction) { x /= length; }
    return direction;
}
ProfilePosition Cross(const ProfilePosition& a, const ProfilePosition& b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}

// True when the displacement shares the geometry's basis and DOF numbering,
// so displaced nodes are the node coefficients plus scaled displacement ones.
bool SharesGeometrySpace(const mfem::Mesh& mesh, const mfem::FiniteElementSpace& geometry_space,
                         const mfem::FiniteElementSpace& displacement_space) {
    if (geometry_space.GetNDofs() != displacement_space.GetNDofs() ||
        geometry_space.GetVDim() != displacement_space.GetVDim() ||
        std::string(geometry_space.FEColl()->Name()) != displacement_space.FEColl()->Name()) {
        return false;
    }
    const auto* geometry_nurbs = geometry_space.GetNURBSext();
    const auto* displacement_nurbs = displacement_space.GetNURBSext();
    if (bool(geometry_nurbs) != bool(displacement_nurbs)) { return false; }
    if (geometry_nurbs) {
        if (geometry_nurbs->GetNKV() != displacement_nurbs->GetNKV() ||
            geometry_nurbs->GetWeights().Size() != displacement_nurbs->GetWeights().Size()) {
            return false;
        }
        for (int i = 0; i < geometry_nurbs->GetWeights().Size(); ++i) {
            if (geometry_nurbs->GetWeights()[i] != displacement_nurbs->GetWeights()[i]) { return false; }
        }
        for (int i = 0; i < geometry_nurbs->GetNKV(); ++i) {
            const auto& a = *geometry_nurbs->GetKnotVector(i);
            const auto& b = *displacement_nurbs->GetKnotVector(i);
            if (a.Size() != b.Size() || a.GetOrder() != b.GetOrder()) { return false; }
            for (int k = 0; k < a.Size(); ++k) { if (a[k] != b[k]) { return false; } }
        }
    }
    // DOF ordering may differ (byNODES/byVDIM), but scalar DOF numbering must
    // match before the two coefficient arrays can be composed.
    mfem::Array<int> geometry_dofs, displacement_dofs;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        geometry_space.GetElementDofs(e, geometry_dofs);
        displacement_space.GetElementDofs(e, displacement_dofs);
        if (geometry_dofs.Size() != displacement_dofs.Size()) { return false; }
        for (int j = 0; j < geometry_dofs.Size(); ++j) {
            if (geometry_dofs[j] != displacement_dofs[j]) { return false; }
        }
    }
    return true;
}

std::unique_ptr<mfem::Mesh> DisplacedGeometry(mfem::Mesh& mesh, const RayFEM::RenderOptions& options) {
    if (!std::isfinite(options.displacement_scale)) { throw std::invalid_argument("Displacement scale must be finite"); }
    if (!options.displacement || options.displacement_scale == 0) { return nullptr; }
    const auto* displacement_space = options.displacement->FESpace();
    if (!displacement_space || displacement_space->GetMesh() != &mesh ||
        options.displacement->VectorDim() != mesh.SpaceDimension()) {
        throw std::invalid_argument("Profile displacement must be a spatial vector field on the displayed mesh");
    }
    std::unique_lock construction(RayFEM::FiniteElementConstructionMutex());
    auto displayed = std::make_unique<mfem::Mesh>(mesh);
    // EnsureNodes converts NURBS to polynomial H1 even when nodes exist.
    if (!displayed->GetNodes()) { displayed->EnsureNodes(); }
    construction.unlock();
    const int space_dimension = mesh.SpaceDimension();
    if (SharesGeometrySpace(mesh, *displayed->GetNodes()->FESpace(), *displacement_space)) {
        auto& nodes = *displayed->GetNodes();
        const auto* geometry_space = nodes.FESpace();
        for (int i = 0; i < geometry_space->GetNDofs(); ++i) {
            for (int d = 0; d < space_dimension; ++d) {
                auto& coordinate = nodes[geometry_space->DofToVDof(i, d)];
                coordinate += options.displacement_scale*(*options.displacement)[displacement_space->DofToVDof(i, d)];
                if (!std::isfinite(coordinate)) { throw std::invalid_argument("Displaced geometry has non-finite coordinates"); }
            }
        }
        displayed->NodesUpdated();
        return displayed;
    }
    // Any other displacement space (another order, a separate DOF numbering
    // such as a field transferred to a submesh, or a discontinuous basis):
    // give the copy nodes rich enough for both the geometry and the
    // displacement, then set each node to its original position plus the
    // scaled displacement, both evaluated in the original element.
    const int geometry_order = mesh.GetNodes() ? mesh.GetNodalFESpace()->GetMaxElementOrder() : 1;
    const int order = std::max({1, geometry_order, displacement_space->GetMaxElementOrder()});
    const bool discontinuous = dynamic_cast<const mfem::L2_FECollection*>(displacement_space->FEColl()) != nullptr;
    construction.lock();
    if (displayed->NURBSext) { displayed->EnsureNodes(); }
    displayed->SetCurvature(order, discontinuous, space_dimension, mfem::Ordering::byVDIM);
    construction.unlock();
    auto& nodes = *displayed->GetNodes();
    const auto& node_space = *nodes.FESpace();
    mfem::Array<int> vdofs;
    mfem::Vector position, displacement;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        const auto& reference_nodes = node_space.GetFE(e)->GetNodes();
        node_space.GetElementVDofs(e, vdofs);
        auto& transform = *mesh.GetElementTransformation(e);
        for (int j = 0; j < reference_nodes.GetNPoints(); ++j) {
            const auto& ip = reference_nodes.IntPoint(j);
            transform.SetIntPoint(&ip);
            transform.Transform(ip, position);
            options.displacement->GetVectorValue(transform, ip, displacement);
            for (int d = 0; d < space_dimension; ++d) {
                const double coordinate = position[d] + options.displacement_scale*displacement[d];
                if (!std::isfinite(coordinate)) { throw std::invalid_argument("Displaced geometry has non-finite coordinates"); }
                const int vdof = vdofs[j + d*reference_nodes.GetNPoints()];
                nodes[vdof >= 0 ? vdof : -1 - vdof] = coordinate;
            }
        }
    }
    displayed->NodesUpdated();
    return displayed;
}

// A residual check is essential for embedded meshes and Newton iterations
// that stop because the reference increment is small rather than converged.
bool Invert(mfem::Mesh& mesh, int element, const ProfilePosition& position,
            mfem::IntegrationPoint& reference, const mfem::IntegrationPoint* guess = nullptr) {
    auto& transform = *mesh.GetElementTransformation(element);
    mfem::Vector point(mesh.SpaceDimension());
    for (int d = 0; d < point.Size(); ++d) { point[d] = position[d]; }
    mfem::InverseElementTransformation inverse(&transform);
    inverse.SetMaxIter(48);
    inverse.SetReferenceTol(1e-13);
    inverse.SetPhysicalRelTol(1e-12);
    inverse.SetElementTol(1e-10);
    if (guess) { inverse.SetInitialGuess(*guess); }
    auto converged = [&]() {
        if (inverse.Transform(point, reference) != mfem::InverseElementTransformation::Inside ||
            !mfem::Geometry::CheckPoint(mesh.GetElementBaseGeometry(element), reference, 1e-9)) {
            return false;
        }
        mfem::Vector physical;
        transform.Transform(reference, physical);
        double error = 0, scale = 1;
        for (int d = 0; d < 3; ++d) {
            const double x = d < physical.Size() ? physical[d] : 0;
            if (!std::isfinite(x)) { return false; }
            error = std::max(error, std::abs(x-position[d]));
            scale = std::max({scale, std::abs(x), std::abs(position[d])});
        }
        return std::isfinite(error) && error <= 2e-10*scale;
    };
    if (converged()) { return true; }
    // Retry distorted elements from a reference grid. This also recovers from
    // a poor coherent guess at curved interfaces; failed inversions stay gaps.
    inverse.SetInitialGuessType(mfem::InverseElementTransformation::ClosestRefNode);
    inverse.SetInitGuessRelOrder(0);
    return converged();
}

// Padded bounds of each included element, from a dense sample of its mapped
// geometry. Inverse maps are only attempted where a point is inside the box,
// so points outside the mesh no longer cost a Newton solve per element. The
// padding covers curved bulges between the sampled points.
struct ElementBox { ProfilePosition low{}, high{}; };
std::vector<ElementBox> ElementBoxes(mfem::Mesh& mesh, const std::vector<bool>& included) {
    std::vector<ElementBox> boxes(mesh.GetNE());
    const int order = mesh.GetNodes() ? std::max(1, mesh.GetNodalFESpace()->GetMaxElementOrder()) : 1;
    mfem::GeometryRefiner refiner;
    mfem::Vector physical;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        if (!included[e]) { continue; }
        auto& box = boxes[e];
        box.low.fill(std::numeric_limits<double>::infinity());
        box.high.fill(-std::numeric_limits<double>::infinity());
        auto& transform = *mesh.GetElementTransformation(e);
        const auto* refined = refiner.Refine(mesh.GetElementBaseGeometry(e), order + 2, 1);
        for (int j = 0; j < refined->RefPts.GetNPoints(); ++j) {
            transform.Transform(refined->RefPts.IntPoint(j), physical);
            for (int d = 0; d < 3; ++d) {
                const double x = d < physical.Size() ? physical[d] : 0;
                box.low[d] = std::min(box.low[d], x); box.high[d] = std::max(box.high[d], x);
            }
        }
        const double diagonal = std::hypot(box.high[0]-box.low[0], box.high[1]-box.low[1], box.high[2]-box.low[2]);
        double magnitude = 1;
        for (int d = 0; d < 3; ++d) { magnitude = std::max({magnitude, std::abs(box.low[d]), std::abs(box.high[d])}); }
        const double pad = .25*diagonal + 1e-12*magnitude;
        for (int d = 0; d < 3; ++d) { box.low[d] -= pad; box.high[d] += pad; }
    }
    return boxes;
}
bool Contains(const ElementBox& box, const ProfilePosition& p) {
    for (int d = 0; d < 3; ++d) { if (p[d] < box.low[d] || p[d] > box.high[d]) { return false; } }
    return true;
}

class InverseProfileAlgorithm final : public RayFEM::ProfileAlgorithm {
public:
    explicit InverseProfileAlgorithm(bool coherent) : coherent_(coherent) {}
    std::string id() const override { return coherent_ ? "coherent" : "exhaustive"; }
    std::string name() const override {
        return coherent_ ? "Coherent inverse mapping" : "Exhaustive inverse mapping";
    }
    std::vector<ProfileSample> sample(mfem::Mesh& mesh, mfem::GridFunction& field,
                                      const std::vector<ProfileSample>& locations,
                                      const RayFEM::RenderOptions& options) const override {
        if (!field.FESpace() || field.FESpace()->GetMesh() != &mesh) {
            throw std::invalid_argument("Profile field must use the sampled mesh");
        }
        if (mesh.Dimension() < 2 || mesh.Dimension() > 3 || mesh.SpaceDimension() > 3) {
            throw std::invalid_argument("Profiles require a 2D or 3D mesh in at most 3 spatial dimensions");
        }
        std::vector<bool> included(mesh.GetNE());
        const RayFEM::ElementSelection selection(mesh, options);
        int first_included = -1;
        for (int e = 0; e < mesh.GetNE(); ++e) {
            included[e] = selection.Includes(e);
            if (included[e] && first_included < 0) { first_included = e; }
        }
        if (first_included >= 0) {
            // Validate field/view compatibility even when every query misses.
            RayFEM::EvaluateField(field, first_included, mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(first_included)), options.view, options.component);
        } else {
            auto result = locations;
            for (auto& point : result) { point.valid=false; point.element=-1; point.value=std::numeric_limits<double>::quiet_NaN(); }
            return result;
        }
        auto displaced = DisplacedGeometry(mesh, options);
        mfem::Mesh& displayed = displaced ? *displaced : mesh;
        const auto* neighbors = coherent_ ? &displayed.ElementToElementTable() : nullptr;
        std::vector<ProfilePosition> centers;
        std::vector<ElementBox> boxes;
        if (coherent_) {
            boxes = ElementBoxes(displayed, included);
            centers.resize(displayed.GetNE());
            for (int e = 0; e < displayed.GetNE(); ++e) {
                if (!included[e]) { continue; }
                mfem::Vector center;
                displayed.GetElementTransformation(e)->Transform(
                    mfem::Geometries.GetCenter(displayed.GetElementBaseGeometry(e)), center);
                for (int d = 0; d < center.Size(); ++d) { centers[e][d] = center[d]; }
                if (!Finite(centers[e])) { throw std::invalid_argument("Profile geometry has non-finite element centers"); }
            }
        }
        std::vector<ProfileSample> result = locations;
        int previous = -1;
        mfem::IntegrationPoint previous_reference;
        for (auto& point : result) {
            point.valid = false;
            point.element = -1;
            point.value = std::numeric_limits<double>::quiet_NaN();
            if (!Finite(point.position)) { continue; }
            // For a 2D mesh in R2 the displayed surface lies in z=0.
            if (displayed.SpaceDimension() == 2 && std::abs(point.position[2]) > 2e-10) { continue; }
            int found = -1;
            mfem::IntegrationPoint reference;
            bool boundary_candidate = false;
            bool searched_all = false;
            if (coherent_ && previous >= 0 && Invert(displayed, previous, point.position, reference, &previous_reference)) {
                if (mfem::Geometry::CheckPoint(displayed.GetElementBaseGeometry(previous), reference, -1e-8)) {
                    found = previous;
                } else { boundary_candidate = true; }
                // Boundary points need deterministic discontinuous traces:
                // both strategies choose the lowest containing element ID.
            }
            if (found < 0 && neighbors && previous >= 0 && !boundary_candidate) {
                const int* adjacent = neighbors->GetRow(previous);
                for (int j = 0; j < neighbors->RowSize(previous); ++j) {
                    const int e = adjacent[j];
                    if (!included[e] || !Contains(boxes[e], point.position)) { continue; }
                    if (!Invert(displayed, e, point.position, reference)) { continue; }
                    if (mfem::Geometry::CheckPoint(displayed.GetElementBaseGeometry(e), reference, -1e-8)) { found = e; }
                    else { boundary_candidate = true; }
                    // Boundary ties always take the ordered exhaustive path.
                    break;
                }
            }
            if (found < 0 && coherent_ && !boundary_candidate) {
                // Center distance only orders candidate elements; it never
                // substitutes for inverse-map containment or FE evaluation.
                // No candidates are discarded, including for curved elements.
                std::vector<std::pair<double, int>> candidates;
                candidates.reserve(centers.size());
                for (int e = 0; e < displayed.GetNE(); ++e) {
                    if (!included[e] || !Contains(boxes[e], point.position)) { continue; }
                    candidates.emplace_back(std::hypot(centers[e][0]-point.position[0],
                                                       centers[e][1]-point.position[1],
                                                       centers[e][2]-point.position[2]), e);
                }
                std::sort(candidates.begin(), candidates.end());
                searched_all = true;
                for (const auto& candidate : candidates) {
                    const int e = candidate.second;
                    if (!Invert(displayed, e, point.position, reference)) { continue; }
                    if (mfem::Geometry::CheckPoint(displayed.GetElementBaseGeometry(e), reference, -1e-8)) { found = e; }
                    else { searched_all = false; }
                    break;
                }
            }
            if (found < 0 && !searched_all) {
                for (int e = 0; e < displayed.GetNE(); ++e) {
                    if (!included[e] || (coherent_ && !Contains(boxes[e], point.position))) { continue; }
                    if (Invert(displayed, e, point.position, reference)) { found = e; break; }
                }
            }
            if (found >= 0) {
                point.value = RayFEM::EvaluateField(field, found, reference, options.view, options.component);
                point.element = found;
                point.valid = true;
                previous = found;
                previous_reference = reference;
            }
        }
        return result;
    }
private:
    bool coherent_;
};

std::string CsvString(const std::string& text) {
    std::string result = "\"";
    for (char c : text) { result += c; if (c == '"') { result += '"'; } }
    return result+'"';
}
const char* ViewName(RayFEM::FieldView view) {
    switch (view) {
        case RayFEM::FieldView::Value: return "value";
        case RayFEM::FieldView::Magnitude: return "magnitude";
        case RayFEM::FieldView::Component: return "component";
        case RayFEM::FieldView::Divergence: return "divergence";
        case RayFEM::FieldView::CurlMagnitude: return "curl_magnitude";
        case RayFEM::FieldView::CurlComponent: return "curl_component";
    }
    return "unknown";
}
} // namespace

namespace RayFEM {

std::unique_ptr<mfem::Mesh> MakeDisplacedMesh(mfem::Mesh& mesh, const RenderOptions& options) {
    return DisplacedGeometry(mesh, options);
}

std::vector<ProfileSample> MakeRaySamples(ProfilePosition origin, ProfilePosition direction,
                                          double length, int count) {
    if (!Finite(origin) || !std::isfinite(length) || length <= 0 || count < 2 || count > 1000000) {
        throw std::invalid_argument("Ray requires a finite origin, positive finite length, and 2 to 1000000 samples");
    }
    direction = Normalize(direction);
    std::vector<ProfileSample> samples(count);
    for (int i = 0; i < count; ++i) {
        auto& sample = samples[i];
        sample.s = length*(double(i)/(count-1));
        for (int d = 0; d < 3; ++d) { sample.position[d] = origin[d]+sample.s*direction[d]; }
        if (!Finite(sample.position)) { throw std::invalid_argument("Ray coordinates overflow"); }
    }
    return samples;
}

std::array<ProfilePosition, 2> PlaneAxes(ProfilePosition normal) {
    normal = Normalize(normal);
    ProfilePosition helper{};
    // Prefer y on ties, giving u=x and v=y for the usual z-normal plane.
    if (std::abs(normal[1]) <= std::abs(normal[0]) && std::abs(normal[1]) <= std::abs(normal[2])) { helper[1] = 1; }
    else if (std::abs(normal[0]) <= std::abs(normal[2])) { helper[0] = 1; }
    else { helper[2] = 1; }
    const auto u_axis = Normalize(Cross(helper, normal));
    return {u_axis, Cross(normal, u_axis)};
}

std::vector<ProfileSample> MakePlaneSamples(ProfilePosition center, ProfilePosition normal,
                                            double width, double height, int columns, int rows) {
    if (!Finite(center) || !std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0 ||
        columns < 2 || rows < 2 || static_cast<std::int64_t>(columns)*rows > 1000000) {
        throw std::invalid_argument("Plane requires finite coordinates, positive finite extents, and a grid of 2 by 2 to 1000000 samples");
    }
    const auto [u_axis, v_axis] = PlaneAxes(normal);
    std::vector<ProfileSample> samples(static_cast<std::size_t>(columns)*rows);
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            auto& sample = samples[static_cast<std::size_t>(row)*columns+column];
            sample.u = width*(double(column)/(columns-1)-0.5);
            sample.v = height*(double(row)/(rows-1)-0.5);
            for (int d = 0; d < 3; ++d) { sample.position[d] = center[d]+sample.u*u_axis[d]+sample.v*v_axis[d]; }
            if (!Finite(sample.position)) { throw std::invalid_argument("Plane coordinates overflow"); }
        }
    }
    return samples;
}

ProfileRegistry::ProfileRegistry() {
    registerAlgorithm(std::make_shared<InverseProfileAlgorithm>(false));
    registerAlgorithm(std::make_shared<InverseProfileAlgorithm>(true));
}
void ProfileRegistry::registerAlgorithm(std::shared_ptr<const ProfileAlgorithm> algorithm) {
    if (!algorithm || algorithm->id().empty() || algorithm->name().empty()) {
        throw std::invalid_argument("Profile algorithms require an ID and display name");
    }
    for (auto& entry : algorithms_) {
        if (entry->id() == algorithm->id()) { entry = std::move(algorithm); return; }
    }
    algorithms_.push_back(std::move(algorithm));
}
std::vector<ProfileAlgorithmInfo> ProfileRegistry::algorithms() const {
    std::vector<ProfileAlgorithmInfo> result;
    for (const auto& algorithm : algorithms_) { result.push_back({algorithm->id(), algorithm->name()}); }
    return result;
}
std::vector<ProfileSample> ProfileRegistry::sample(const std::string& id, mfem::Mesh& mesh,
                                                  mfem::GridFunction& field,
                                                  const std::vector<ProfileSample>& locations,
                                                  const RenderOptions& options) const {
    ValidateElementSelection(mesh, options);
    const ElementSelection selection(mesh, options);
    for (const auto& algorithm : algorithms_) {
        if (algorithm->id() == id) {
            auto result = algorithm->sample(mesh, field, locations, options);
            // A replacement locator may use a different search strategy, but
            // excluded-element results never enter analysis or exported data.
            if (!options.excluded_materials.empty() || !options.excluded_elements.empty()) for (auto& point : result) {
                if (point.valid && !selection.Includes(point.element)) {
                    point.valid=false; point.element=-1; point.value=std::numeric_limits<double>::quiet_NaN();
                }
            }
            return result;
        }
    }
    throw std::invalid_argument("Unknown profile algorithm: "+id);
}

void ExportProfileCSV(std::ostream& stream, const std::vector<ProfileSample>& samples,
                      const ProfileMetadata& metadata) {
    std::ostringstream row;
    row.imbue(std::locale::classic());
    row << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::string excluded;
    for (int id : metadata.excluded_materials) { if (!excluded.empty()) excluded+=';'; excluded+=std::to_string(id); }
    std::string elements;
    for (int id : metadata.excluded_elements) { if (!elements.empty()) elements+=';'; elements+=std::to_string(id); }
    stream << "field,frame,time,view,component,algorithm,s,u,v,x,y,z,value,valid,element,excluded_materials,excluded_elements\r\n";
    for (const auto& sample : samples) {
        row.str(""); row.clear();
        row << CsvString(metadata.field_name) << ',' << metadata.frame << ',' << metadata.time << ','
            << ViewName(metadata.view) << ',' << metadata.component << ',' << CsvString(metadata.algorithm) << ','
            << sample.s << ',' << sample.u << ',' << sample.v << ','
            << sample.position[0] << ',' << sample.position[1] << ',' << sample.position[2] << ','
            << (sample.valid ? sample.value : std::numeric_limits<double>::quiet_NaN()) << ','
            << (sample.valid ? 1 : 0) << ',' << (sample.valid ? sample.element : -1) << ',' << CsvString(excluded) << ',' << CsvString(elements) << "\r\n";
        stream << row.str();
    }
    if (!stream) { throw std::runtime_error("Could not write profile CSV"); }
}

} // namespace RayFEM
