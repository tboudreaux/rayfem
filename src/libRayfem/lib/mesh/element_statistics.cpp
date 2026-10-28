#include "libRayfem/mesh/element_statistics.hpp"
#include "libRayfem/profile/sampler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace RayFEM {
namespace {
double FiniteDouble(long double value, const char* what) {
    const double result = static_cast<double>(value);
    if (!std::isfinite(result)) throw std::runtime_error(what);
    return result;
}
std::size_t LatticeCount(mfem::Geometry::Type geometry, int subdivisions) {
    const std::size_t n = subdivisions;
    switch (geometry) {
        case mfem::Geometry::TRIANGLE: return (n+1)*(n+2)/2;
        case mfem::Geometry::SQUARE: return (n+1)*(n+1);
        case mfem::Geometry::TETRAHEDRON: return (n+1)*(n+2)*(n+3)/6;
        case mfem::Geometry::CUBE: return (n+1)*(n+1)*(n+1);
        case mfem::Geometry::PRISM: return (n+1)*(n+1)*(n+2)/2;
        case mfem::Geometry::PYRAMID: return (n+1)*(n+2)*(2*n+3)/6;
        default: throw std::invalid_argument("Element statistics do not support this geometry");
    }
}
template<class Sample>
void Lattice(mfem::Geometry::Type geometry, int n, Sample&& sample) {
    const bool volume = mfem::Geometry::Dimension[geometry] == 3;
    for (int k = 0; k <= (volume ? n : 0); ++k) {
        const int jmax = geometry == mfem::Geometry::TETRAHEDRON || geometry == mfem::Geometry::PYRAMID ? n-k : n;
        for (int j = 0; j <= jmax; ++j) {
            const int imax = geometry == mfem::Geometry::TRIANGLE || geometry == mfem::Geometry::PRISM ? n-j :
                             geometry == mfem::Geometry::TETRAHEDRON ? n-j-k :
                             geometry == mfem::Geometry::PYRAMID ? n-k : n;
            for (int i = 0; i <= imax; ++i) {
                mfem::IntegrationPoint point;
                point.Set3(double(i)/n, double(j)/n, double(k)/n);
                sample(point);
            }
        }
    }
}
} // namespace

ElementStatistics ComputeElementStatistics(mfem::Mesh& mesh,
    const std::vector<int>& selected_elements, mfem::GridFunction* field,
    const RenderOptions& options, const ElementStatisticsOptions& sampling) {
    if (mesh.Dimension() < 2 || mesh.Dimension() > 3 || mesh.SpaceDimension() > 3)
        throw std::invalid_argument("Element statistics require a 2D or 3D mesh embedded in at most 3 dimensions");
    if (sampling.quadrature_order < 0 || sampling.quadrature_order > 64 ||
        sampling.subdivisions < 0 || sampling.subdivisions > 64 || !sampling.max_samples)
        throw std::invalid_argument("Statistics quadrature/subdivisions must be 0 (automatic) or 1..64, with a positive sample budget");
    if (!std::isfinite(options.displacement_scale))
        throw std::invalid_argument("Statistics displacement scale must be finite");
    if (field && (!field->FESpace() || field->FESpace()->GetMesh() != &mesh || field->Size() != field->FESpace()->GetVSize()))
        throw std::invalid_argument("Statistics field must use the selected mesh");

    ElementStatistics result;
    result.dimension = mesh.Dimension(); result.space_dimension = mesh.SpaceDimension();
    result.nurbs = mesh.NURBSext != nullptr;
    auto selected = selected_elements;
    std::sort(selected.begin(), selected.end());
    selected.erase(std::unique(selected.begin(), selected.end()), selected.end());
    result.requested_count = selected.size();
    const ElementSelection selection(mesh, options);
    for (int e : selected) {
        if (e < 0 || e >= mesh.GetNE()) throw std::out_of_range("Selected element ID is out of range");
        if (selection.Includes(e)) result.elements.push_back(e);
    }
    result.element_count = result.elements.size();
    result.excluded_count = result.requested_count - result.element_count;
    if (result.elements.empty()) return result;

    // Reuse the exact nodal/NURBS displacement composition used by profiles
    // and tracking; never modify the caller's coordinates or field space.
    auto displaced = MakeDisplacedMesh(mesh, options);
    mfem::Mesh& physical = displaced ? *displaced : mesh;
    struct Work { int element, subdivisions; const mfem::IntegrationRule* rule; };
    std::vector<Work> work;
    std::size_t expected = 0;
    auto count = [&](std::size_t n) {
        if (n > sampling.max_samples - expected)
            throw std::runtime_error("Element statistics sample budget exceeded; select fewer elements or reduce sampling orders");
        expected += n;
    };
    for (int e : result.elements) {
        const auto geometry = mesh.GetElementBaseGeometry(e);
        const int geometry_order = physical.GetElementTransformation(e)->Order();
        const int field_order = field ? field->FESpace()->GetFE(e)->GetOrder() : 0;
        const int quadrature = sampling.quadrature_order ? sampling.quadrature_order :
            std::max(10, 2*field_order + 2*physical.Dimension()*geometry_order + 4);
        if (quadrature > 64)
            throw std::invalid_argument("Automatic statistics quadrature exceeds 64; choose an explicit quadrature order");
        const int subdivisions = sampling.subdivisions ? sampling.subdivisions :
            std::clamp(2*std::max(geometry_order, field_order)+2, 4, 64);
        const auto& rule = mfem::IntRules.Get(geometry, quadrature);
        count(rule.GetNPoints()); count(LatticeCount(geometry, subdivisions)); count(1);
        if (auto* nodes = physical.GetNodes()) count(nodes->FESpace()->GetFE(e)->GetNodes().GetNPoints());
        if (field) count(field->FESpace()->GetFE(e)->GetNodes().GetNPoints());
        work.push_back({e, subdivisions, &rule});
        ++result.material_counts[mesh.GetAttribute(e)];
        ++result.geometry_counts[geometry];
        ++result.geometry_orders[geometry_order];
        if (field) ++result.field_orders[field_order];
    }

    const double infinity = std::numeric_limits<double>::infinity();
    result.bounds_min.fill(infinity); result.bounds_max.fill(-infinity);
    ElementFieldStatistics values{infinity, -infinity, 0, 0};
    long double measure = 0, integral = 0;
    std::array<long double, 3> moments{};
    for (const auto& item : work) {
        const int e = item.element;
        const auto geometry = mesh.GetElementBaseGeometry(e);
        auto* transform = physical.GetElementTransformation(e);
        auto position = [&](const mfem::IntegrationPoint& point) {
            mfem::Vector x;
            transform->Transform(point, x);
            std::array<double, 3> result{};
            for (int d = 0; d < x.Size(); ++d) {
                if (!std::isfinite(x[d])) throw std::runtime_error("Statistics geometry contains non-finite coordinates");
                result[d] = x[d];
            }
            return result;
        };
        auto sample = [&](const mfem::IntegrationPoint& point, bool sample_field) {
            const auto x = position(point);
            for (int d = 0; d < 3; ++d) {
                result.bounds_min[d] = std::min(result.bounds_min[d], x[d]);
                result.bounds_max[d] = std::max(result.bounds_max[d], x[d]);
            }
            ++result.bounds_samples;
            if (sample_field && field) {
                const double value = EvaluateField(*field, e, point, options.view, options.component);
                values.minimum = std::min(values.minimum, value); values.maximum = std::max(values.maximum, value);
                ++result.field_samples;
                return value;
            }
            return 0.0;
        };
        for (int q = 0; q < item.rule->GetNPoints(); ++q) {
            const auto& point = item.rule->IntPoint(q);
            const auto x = position(point);
            transform->SetIntPoint(&point);
            const double jacobian = transform->Weight();
            const double weight = point.weight * jacobian;
            if (!std::isfinite(weight) || !(jacobian > 0) ||
                (physical.Dimension() == physical.SpaceDimension() && !(transform->Jacobian().Det() > 0)))
                throw std::runtime_error("Selected element has degenerate, inverted or non-finite geometry");
            // Some simplex quadrature rules have negative point weights;
            // physical Jacobians must remain positive, but rule weights needn't.
            const double value = sample(point, true);
            measure += weight;
            for (int d = 0; d < 3; ++d) moments[d] += static_cast<long double>(weight) * x[d];
            if (field) integral += static_cast<long double>(weight) * value;
            ++result.quadrature_samples;
        }
        Lattice(geometry, item.subdivisions, [&](const auto& point) { sample(point, true); });
        sample(mfem::Geometries.GetCenter(geometry), true);
        if (auto* nodes = physical.GetNodes()) {
            const mfem::IntegrationRule points(nodes->FESpace()->GetFE(e)->GetNodes());
            for (int i = 0; i < points.GetNPoints(); ++i) sample(points.IntPoint(i), false);
        }
        if (field) {
            const mfem::IntegrationRule points(field->FESpace()->GetFE(e)->GetNodes());
            for (int i = 0; i < points.GetNPoints(); ++i) sample(points.IntPoint(i), true);
        }
    }
    if (!(measure > 0)) throw std::runtime_error("Selected elements have no positive physical measure");
    result.measure = FiniteDouble(measure, "Statistics physical measure exceeds numeric range");
    for (int d = 0; d < 3; ++d) result.centroid[d] = FiniteDouble(moments[d]/measure, "Statistics centroid exceeds numeric range");
    if (field) {
        values.integral = FiniteDouble(integral, "Selected field integral exceeds numeric range");
        values.mean = FiniteDouble(integral/measure, "Selected field mean exceeds numeric range");
        result.field = values;
    }
    return result;
}
} // namespace RayFEM
