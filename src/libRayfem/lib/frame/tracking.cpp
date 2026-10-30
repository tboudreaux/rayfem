#include "libRayfem/frame/tracking.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {

const RayFEM::FrameField& FieldRecord(const RayFEM::RenderFrame& frame, const std::string& name) {
    if (!frame.mesh) { throw std::invalid_argument("Frame has no mesh"); }
    for (const auto& field : frame.fields) {
        if (field.name == name && field.gf) {
            if (!field.gf->FESpace() || field.gf->FESpace()->GetMesh() != frame.mesh.get()) {
                throw std::invalid_argument("Named field does not use the frame mesh");
            }
            return field;
        }
    }
    throw std::invalid_argument("Field '"+name+"' is missing in this frame");
}
mfem::GridFunction& Field(const RayFEM::RenderFrame& frame, const std::string& name) {
    return *FieldRecord(frame, name).gf;
}

void Hash(std::uint64_t& hash, std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) { hash ^= (value >> (8*byte)) & 255; hash *= 1099511628211ULL; }
}
void HashReal(std::uint64_t& hash, double value) {
    std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); Hash(hash, bits);
}
std::uint64_t ComputeLayout(const mfem::FiniteElementSpace& space) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto& mesh = *space.GetMesh();
    Hash(hash, mesh.Dimension()); Hash(hash, mesh.SpaceDimension()); Hash(hash, mesh.GetNV());
    Hash(hash, space.GetNE()); Hash(hash, space.GetVectorDim());
    mfem::Array<int> dofs, vertices;
    for (int e = 0; e < space.GetNE(); ++e) {
        // L2 element-local numbering does not encode topology or reference
        // orientation, so include the ordered mesh connectivity as well.
        Hash(hash, mesh.GetElementBaseGeometry(e));
        mesh.GetElementVertices(e, vertices); Hash(hash, vertices.Size());
        for (int vertex : vertices) { Hash(hash, vertex); }
        Hash(hash, space.GetOrder(e));
        space.GetElementDofs(e, dofs); Hash(hash, dofs.Size());
        for (int dof : dofs) { Hash(hash, static_cast<std::uint64_t>(dof)); }
    }
    if (const auto* nurbs = space.GetNURBSext()) {
        Hash(hash, nurbs->GetNKV());
        for (int i = 0; i < nurbs->GetNKV(); ++i) {
            const auto& knot = *nurbs->GetKnotVector(i);
            Hash(hash, knot.GetOrder()); Hash(hash, knot.Size());
            for (int j = 0; j < knot.Size(); ++j) { HashReal(hash, knot[j]); }
        }
        for (int i = 0; i < nurbs->GetWeights().Size(); ++i) { HashReal(hash, nurbs->GetWeights()[i]); }
    }
    return hash;
}

std::uint64_t Layout(const RayFEM::FrameField& field) {
    const auto& space = *field.gf->FESpace();
    const auto& mesh = *space.GetMesh();
    auto& cache = field.dof_layout_cache;
    if (!cache || cache->space != &space || cache->mesh != &mesh ||
        cache->space_sequence != space.GetSequence() || cache->mesh_sequence != mesh.GetSequence()) {
        cache = RayFEM::FrameField::DofLayoutCache{&space, &mesh, space.GetSequence(),
                                                mesh.GetSequence(), ComputeLayout(space)};
    }
    return cache->fingerprint;
}

RayFEM::RenderOptions Options(const RayFEM::RenderFrame& frame, const RayFEM::TrackingRequest& request) {
    RayFEM::RenderOptions result;
    result.view = request.view; result.component = request.component;
    result.displacement = request.use_displacement ? frame.displacement.get() : nullptr;
    result.displacement_scale = request.displacement_scale;
    result.excluded_materials = request.excluded_materials;
    if (!request.element_scope || *request.element_scope == RayFEM::ElementSelectionScope{frame.session_id,frame.mesh_id,frame.mesh_revision})
        result.excluded_elements = request.excluded_elements;
    RayFEM::ValidateElementSelection(*frame.mesh, result);
    RayFEM::ExcludeUnsupportedMaterials(FieldRecord(frame, request.field_name),
                                       *frame.mesh, result.excluded_materials);
    return result;
}

void Aggregate(RayFEM::MeasurementSample& output, RayFEM::RenderFrame& frame,
               mfem::GridFunction& field, const RayFEM::TrackingRequest& request) {
    auto options = Options(frame, request);
    const RayFEM::ElementSelection selection(*frame.mesh, options);
    std::vector<int> included;
    for (int e = 0; e < frame.mesh->GetNE(); ++e) {
        if (selection.Includes(e)) { included.push_back(e); }
    }
    if (included.empty()) { throw std::invalid_argument("No included elements remain in the selection"); }
    auto deformed = RayFEM::MakeDisplacedMesh(*frame.mesh, options);
    mfem::Mesh& geometry = deformed ? *deformed : *frame.mesh;
    long double total = 0, integral = 0, square = 0;
    double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
    std::size_t evaluated = 0;
    // Check the complete budget before expensive field evaluation. A rejected
    // request must not stall the viewer after computing two million samples.
    std::vector<const mfem::IntegrationRule*> rules;
    rules.reserve(included.size());
    for (int e : included) {
        auto* transform = geometry.GetElementTransformation(e);
        const int order = request.quadrature_order ? request.quadrature_order :
            std::max(8, 2*field.FESpace()->GetFE(e)->GetOrder()+2*geometry.Dimension()*transform->Order()+4);
        if (order < 1 || order > 64) { throw std::invalid_argument("Tracking quadrature order must be between 1 and 64"); }
        const auto& rule = mfem::IntRules.Get(geometry.GetElementBaseGeometry(e), order);
        evaluated += rule.GetNPoints();
        if (evaluated > 2000000) { throw std::runtime_error("Tracking quadrature budget exceeded; choose a lower quadrature order"); }
        rules.push_back(&rule);
    }
    for (std::size_t k = 0; k < included.size(); ++k) {
        const int e = included[k];
        auto* transform = geometry.GetElementTransformation(e);
        const auto& rule = *rules[k];
        for (int q = 0; q < rule.GetNPoints(); ++q) {
            const auto& ip = rule.IntPoint(q);
            transform->SetIntPoint(&ip);
            const double weight = ip.weight*transform->Weight();
            if (!std::isfinite(weight) || weight < 0) { throw std::runtime_error("Invalid physical integration weight"); }
            const double value = RayFEM::EvaluateField(field, e, ip, request.view, request.component);
            total += weight;
            integral += static_cast<long double>(weight)*value;
            square += static_cast<long double>(weight)*value*value;
            minimum = std::min(minimum, value); maximum = std::max(maximum, value);
        }
    }
    if (!(total > 0) || !std::isfinite(total)) { throw std::runtime_error("Mesh has no positive finite physical measure"); }
    output.measure = static_cast<double>(total);
    switch (request.kind) {
        case RayFEM::MeasurementKind::Minimum: output.value = minimum; break;
        case RayFEM::MeasurementKind::Maximum: output.value = maximum; break;
        case RayFEM::MeasurementKind::Mean: output.value = static_cast<double>(integral/total); break;
        case RayFEM::MeasurementKind::Integral: output.value = static_cast<double>(integral); break;
        case RayFEM::MeasurementKind::L2Norm: output.value = static_cast<double>(std::sqrt(square)); break;
        default: throw std::invalid_argument("Unknown aggregate measurement");
    }
    if (!std::isfinite(output.measure)) { throw std::runtime_error("Physical measure exceeds numeric range"); }
}

std::string Quoted(const std::string& text) {
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

const char* MeasurementName(MeasurementKind kind) {
    switch (kind) {
        case MeasurementKind::DofCoefficient: return "DOF coefficient";
        case MeasurementKind::Minimum: return "Minimum (quadrature samples)";
        case MeasurementKind::Maximum: return "Maximum (quadrature samples)";
        case MeasurementKind::Mean: return "Physical mean";
        case MeasurementKind::Integral: return "Physical integral";
        case MeasurementKind::L2Norm: return "L2 norm";
        case MeasurementKind::PointProbe: return "Point probe";
        case MeasurementKind::FrameValue: return "Frame value";
    }
    return "Unknown measurement";
}

FrameTracker::FrameTracker(TrackingRequest request, std::size_t capacity)
    : request_(std::move(request)), capacity_(capacity) {
    if (request_.field_name.empty()) { throw std::invalid_argument("Tracking requires a named field"); }
    if (!capacity || capacity > 1000000) { throw std::invalid_argument("Tracking capacity must be between 1 and 1000000"); }
    if (request_.dof < 0 || request_.component < 0) { throw std::invalid_argument("DOF and component indices cannot be negative"); }
    if (request_.quadrature_order < 0 || request_.quadrature_order > 64) { throw std::invalid_argument("Tracking quadrature order must be 0 (automatic) or 1 to 64"); }
    if (!std::isfinite(request_.displacement_scale)) { throw std::invalid_argument("Tracking displacement scale must be finite"); }
    std::sort(request_.excluded_materials.begin(), request_.excluded_materials.end());
    request_.excluded_materials.erase(std::unique(request_.excluded_materials.begin(),request_.excluded_materials.end()), request_.excluded_materials.end());
    if (std::any_of(request_.excluded_elements.begin(),request_.excluded_elements.end(),[](int e){return e<0;}))
        throw std::invalid_argument("Excluded element indices cannot be negative");
    std::sort(request_.excluded_elements.begin(),request_.excluded_elements.end());
    request_.excluded_elements.erase(std::unique(request_.excluded_elements.begin(),request_.excluded_elements.end()),request_.excluded_elements.end());
}

FrameTracker::DofBinding FrameTracker::bindingFor(const RenderFrame& frame, mfem::GridFunction& field) const {
    auto& space = *field.FESpace();
    if (request_.dof >= space.GetNDofs()) { throw std::invalid_argument("Selected scalar DOF is out of range in this field"); }
    if (request_.component >= space.GetVDim()) {
        throw std::invalid_argument("Selected coefficient component is out of range; ND/RT directional coefficients use component 0");
    }
    return {frame.session_id, frame.mesh_id, frame.mesh_revision, Layout(FieldRecord(frame, request_.field_name)),
            space.FEColl()->Name(), space.GetNDofs(), space.GetVDim()};
}
void FrameTracker::bindDof(const RenderFrame& frame) {
    if (request_.kind != MeasurementKind::DofCoefficient) { return; }
    binding_ = bindingFor(frame, Field(frame, request_.field_name));
}

double FrameTracker::dofValue(const RenderFrame& frame, mfem::GridFunction& field) {
    if (binding_ && (binding_->session != frame.session_id || binding_->mesh != frame.mesh_id || binding_->revision != frame.mesh_revision)) {
        throw std::invalid_argument("DOF association changed: session, mesh, or revision differs from the selected DOF");
    }
    const auto current = bindingFor(frame, field);
    if (binding_) {
        if (binding_->collection != current.collection || binding_->ndofs != current.ndofs ||
            binding_->vdim != current.vdim || binding_->layout != current.layout) {
            throw std::invalid_argument("DOF association changed: finite element basis, order, or DOF layout differs");
        }
    }
    const auto options = Options(frame, request_);
    const ElementSelection selection(*frame.mesh, options);
    if (!options.excluded_materials.empty() || !options.excluded_elements.empty()) {
        bool supported = false;
        mfem::Array<int> dofs;
        for (int e = 0; e < frame.mesh->GetNE() && !supported; ++e) {
            if (!selection.Includes(e)) { continue; }
            field.FESpace()->GetElementDofs(e, dofs);
            for (int dof : dofs) {
                if ((dof >= 0 ? dof : -1-dof) == request_.dof) { supported=true; break; }
            }
        }
        if (!supported) { throw std::invalid_argument("Selected DOF has no support in the included elements"); }
    }
    const double result = field[field.FESpace()->DofToVDof(request_.dof, request_.component)];
    if (!std::isfinite(result)) { throw std::runtime_error("Selected DOF coefficient is non-finite"); }
    if (!binding_) { binding_ = current; }
    return result;
}

bool FrameTracker::append(RenderFrame& frame) {
    const auto key = std::make_tuple(frame.session_id, frame.mesh_id, frame.mesh_revision);
    auto latest = latest_.find(key);
    if (latest != latest_.end() && frame.metadata.frame_index <= latest->second) { return false; }
    if (latest == latest_.end() && latest_.size() >= 256) {
        throw std::runtime_error("Tracking identity limit reached (256); clear the series before adding another stream or revision");
    }
    latest_[key] = frame.metadata.frame_index;
    MeasurementSample sample;
    sample.session_id = frame.session_id; sample.mesh_id = frame.mesh_id; sample.mesh_revision = frame.mesh_revision;
    sample.frame = frame.metadata.frame_index; sample.time = frame.metadata.sim_time;
    if (frame.schema)
        for (const auto& field : frame.schema->values)
            if (field.coordinate) if (const auto v = frame.metadata.number(field.name)) sample.coordinates.emplace_back(field.name, *v);
    sample.excluded_materials = request_.excluded_materials;
    if (!request_.element_scope || *request_.element_scope == ElementSelectionScope{frame.session_id,frame.mesh_id,frame.mesh_revision})
        sample.excluded_elements = request_.excluded_elements;
    try {
        if (!std::isfinite(sample.time)) { throw std::invalid_argument("Frame simulation time is non-finite"); }
        if (request_.kind == MeasurementKind::FrameValue) {
            const auto value = frame.metadata.number(request_.field_name);
            if (!value) { throw std::runtime_error("Frame has no numeric value named " + request_.field_name); }
            sample.value = *value;
        } else {
        const auto& record = FieldRecord(frame, request_.field_name);
        auto& field = *record.gf;
        ExcludeUnsupportedMaterials(record, *frame.mesh, sample.excluded_materials);
        if (request_.kind == MeasurementKind::DofCoefficient) { sample.value = dofValue(frame, field); }
        else if (request_.kind == MeasurementKind::PointProbe) {
            ProfileSample point; point.position = request_.probe;
            auto result = registry_.sample(request_.algorithm, *frame.mesh, field, {point}, Options(frame, request_));
            if (result.size() != 1 || !result[0].valid) {
                throw std::runtime_error("Point is outside the displayed mesh or inverse mapping did not converge");
            }
            sample.value = result[0].value;
        } else { Aggregate(sample, frame, field, request_); }
        }
        if (!std::isfinite(sample.value)) { throw std::runtime_error("Measurement is non-finite"); }
        sample.valid = true;
    } catch (const std::exception& error) {
        sample.value = sample.measure = std::numeric_limits<double>::quiet_NaN();
        sample.error = error.what();
    }
    samples_.push_back(std::move(sample));
    while (samples_.size() > capacity_) { samples_.pop_front(); }
    return true;
}

void FrameTracker::clear() {
    samples_.clear(); latest_.clear(); binding_.reset();
}
void FrameTracker::setExcludedMaterials(std::vector<int> excluded_materials) {
    std::sort(excluded_materials.begin(), excluded_materials.end());
    excluded_materials.erase(std::unique(excluded_materials.begin(),excluded_materials.end()), excluded_materials.end());
    request_.excluded_materials = std::move(excluded_materials);
    samples_.clear(); latest_.clear();
}
void FrameTracker::setExcludedElements(std::vector<int> excluded_elements,
                                     std::optional<ElementSelectionScope> scope) {
    if (std::any_of(excluded_elements.begin(),excluded_elements.end(),[](int e){return e<0;}))
        throw std::invalid_argument("Excluded element indices cannot be negative");
    std::sort(excluded_elements.begin(),excluded_elements.end());
    excluded_elements.erase(std::unique(excluded_elements.begin(),excluded_elements.end()),excluded_elements.end());
    request_.excluded_elements = std::move(excluded_elements);
    request_.element_scope = scope;
    samples_.clear(); latest_.clear();
}

void ExportTrackingCSV(std::ostream& stream, const FrameTracker& tracker) {
    const auto& request = tracker.request();
    stream << "field,kind,dof,component,view,displacement_enabled,displacement_scale,quadrature_order,algorithm,probe_x,probe_y,probe_z,session,mesh,revision,frame,time,valid,value,measure,error,excluded_materials,excluded_elements,coordinates\r\n";
    std::ostringstream row;
    row.imbue(std::locale::classic()); row << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& sample : tracker.samples()) {
        std::string excluded;
        for (int id : sample.excluded_materials) { if (!excluded.empty()) excluded+=';'; excluded+=std::to_string(id); }
        std::string elements;
        for (int id : sample.excluded_elements) { if (!elements.empty()) elements+=';'; elements+=std::to_string(id); }
        std::ostringstream coordinates;
        coordinates.imbue(std::locale::classic()); coordinates << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& [name, value] : sample.coordinates) coordinates << (coordinates.tellp() > 0 ? ";" : "") << name << '=' << value;
        row.str(""); row.clear();
        row << Quoted(request.field_name) << ',' << Quoted(MeasurementName(request.kind)) << ','
            << request.dof << ',' << request.component << ',' << ViewName(request.view) << ','
            << (request.use_displacement ? 1 : 0) << ',' << request.displacement_scale << ','
            << request.quadrature_order << ',' << Quoted(request.algorithm) << ','
            << request.probe[0] << ',' << request.probe[1] << ',' << request.probe[2] << ','
            << sample.session_id << ',' << sample.mesh_id << ',' << sample.mesh_revision << ','
            << sample.frame << ',' << sample.time << ',' << (sample.valid ? 1 : 0) << ','
            << (sample.valid ? sample.value : std::numeric_limits<double>::quiet_NaN()) << ','
            << sample.measure << ',' << Quoted(sample.error) << ',' << Quoted(excluded) << ',' << Quoted(elements) << ',' << Quoted(coordinates.str()) << "\r\n";
        stream << row.str();
    }
    if (!stream) { throw std::runtime_error("Could not write tracking CSV"); }
}

} // namespace RayFEM
