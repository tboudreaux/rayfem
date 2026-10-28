#include "libRayfem/mesh/field_range.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
std::size_t LatticeCount(mfem::Geometry::Type geometry, int subdivisions) {
    const std::size_t n = subdivisions;
    switch (geometry) {
        case mfem::Geometry::TRIANGLE: return (n+1)*(n+2)/2;
        case mfem::Geometry::SQUARE: return (n+1)*(n+1);
        case mfem::Geometry::TETRAHEDRON: return (n+1)*(n+2)*(n+3)/6;
        case mfem::Geometry::CUBE: return (n+1)*(n+1)*(n+1);
        case mfem::Geometry::PRISM: return (n+1)*(n+1)*(n+2)/2;
        case mfem::Geometry::PYRAMID: return (n+1)*(n+2)*(2*n+3)/6;
        default: throw std::invalid_argument("Field range does not support this element geometry");
    }
}

template<class Sample>
void Lattice(mfem::Geometry::Type geometry, int n, Sample&& sample) {
    const bool volume = mfem::Geometry::Dimension[geometry] == 3;
    for (int k = 0; k <= (volume ? n : 0); ++k) {
        const int jmax = geometry == mfem::Geometry::TETRAHEDRON ||
                         geometry == mfem::Geometry::PYRAMID ? n-k : n;
        for (int j = 0; j <= jmax; ++j) {
            int imax = n;
            if (geometry == mfem::Geometry::TRIANGLE || geometry == mfem::Geometry::PRISM) imax = n-j;
            else if (geometry == mfem::Geometry::TETRAHEDRON) imax = n-j-k;
            else if (geometry == mfem::Geometry::PYRAMID) imax = n-k;
            for (int i = 0; i <= imax; ++i) {
                mfem::IntegrationPoint point;
                point.Set3(double(i)/n, double(j)/n, double(k)/n);
                sample(point);
            }
        }
    }
}
} // namespace

namespace RayFEM {
FieldRange EvaluateFieldRange(mfem::Mesh& mesh, mfem::GridFunction& field,
                             const RenderOptions& options,
                             const FieldRangeOptions& sampling) {
    ValidateElementSelection(mesh,options);
    const ElementSelection selection(mesh,options);
    if (mesh.Dimension()<2 || mesh.Dimension()>3 || mesh.SpaceDimension()>3)
        throw std::invalid_argument("Field range requires a 2D or 3D mesh embedded in at most 3 dimensions");
    auto* space=field.FESpace();
    if (!space || space->GetMesh()!=&mesh || field.Size()!=space->GetVSize())
        throw std::invalid_argument("Field range requires a valid grid function on the supplied mesh");
    if (sampling.subdivisions<0 || sampling.subdivisions>64 || !sampling.max_samples)
        throw std::invalid_argument("Field range subdivisions must be 0 (automatic) or 1..64, and the sample budget must be positive");
    switch (options.view) {
        case FieldView::Value: case FieldView::Magnitude: case FieldView::Component:
        case FieldView::Divergence: case FieldView::CurlMagnitude: case FieldView::CurlComponent: break;
        default: throw std::invalid_argument("Unknown field range operation");
    }
    struct Element {int index, subdivisions, minimum;std::size_t nodes;};
    std::vector<Element> elements;
    FieldRange result;
    // Preflight before evaluating any field value or allocating sampling grids.
    // In particular, never return extrema from only the first budgeted cells.
    for (int e=0;e<mesh.GetNE();++e) {
        if (!selection.Includes(e)) continue;
        const auto* fe=space->GetFE(e);
        const int field_order=fe->GetOrder();
        const auto nodes=static_cast<std::size_t>(fe->GetNodes().GetNPoints());
        const int order=std::max(field_order,mesh.GetElementTransformation(e)->Order());
        const int n=sampling.subdivisions ? sampling.subdivisions :
                    (order>=31 ? 64 : std::max(4,2*std::max(0,order)+2));
        const int minimum=order>=63 ? 64 : std::max(2,((std::max(0,order)+1)/2)*2);
        elements.push_back({e,n,minimum,nodes});
    }
    if (elements.empty()) throw std::invalid_argument("Field range has no included elements");
    std::size_t expected=0;
    auto fits=[&] {
        expected=0;
        for (const auto element:elements) {
            const auto count=LatticeCount(mesh.GetElementBaseGeometry(element.index),element.subdivisions)+element.nodes+1;
            if (count>sampling.max_samples-expected) return false;
            expected+=count;
        }
        return true;
    };
    while (!fits()) {
        bool reduced=false;
        if (!sampling.subdivisions) for (auto& element:elements) if (element.subdivisions>element.minimum) {
            element.subdivisions=std::max(element.minimum,element.subdivisions-2);reduced=true;
        }
        if (!reduced && sampling.reduce_to_fit) {
            if (!result.lattice_dropped) {
                // Vertices plus FE nodes and center still bound nodal fields well.
                for (auto& element:elements) element.subdivisions=1;
                result.lattice_dropped=reduced=true;
            } else {
                // Keep an evenly strided subset, sized from the current total.
                std::size_t total=0;
                for (const auto element:elements)
                    total+=LatticeCount(mesh.GetElementBaseGeometry(element.index),element.subdivisions)+element.nodes+1;
                const std::size_t stride=std::max<std::size_t>(2,(total+sampling.max_samples-1)/sampling.max_samples);
                std::vector<Element> kept;
                kept.reserve(elements.size()/stride+1);
                for (std::size_t i=0;i<elements.size();i+=stride) kept.push_back(elements[i]);
                if (kept.size()<elements.size()) {
                    result.skipped_elements+=elements.size()-kept.size();
                    elements=std::move(kept);
                    reduced=true;
                }
            }
        }
        if (!reduced)
            throw std::runtime_error("Field range sample budget exceeded ("+
                std::to_string(sampling.max_samples)+"); reduce explicit subdivisions or select fewer materials");
        result.budget_limited=true;
    }
    result.min_subdivisions=64;
    for (const auto element:elements) {
        result.max_subdivisions=std::max(result.max_subdivisions,element.subdivisions);
        result.min_subdivisions=std::min(result.min_subdivisions,element.subdivisions);
    }
    result.min_value=std::numeric_limits<double>::infinity();
    result.max_value=-result.min_value;
    const std::size_t distribution_count=std::min(expected,sampling.distribution_samples);
    result.samples.reserve(distribution_count);
    // Evenly spaced indices cover the entire preflighted sequence, including
    // both endpoints (or its midpoint for a single requested value). Advance
    // quotient/remainder arithmetic without a potentially overflowing product.
    std::size_t distribution_next=distribution_count==1 ? expected/2 : 0;
    const std::size_t denominator=distribution_count>1 ? distribution_count-1 : 1;
    const std::size_t distribution_step=(expected-1)/denominator;
    const std::size_t distribution_remainder=(expected-1)%denominator;
    std::size_t distribution_error=0;
    // Polynomial reference bases are shared by many cells. Keep this cache
    // local to the call, bounded to 32 MiB per rule / 64 MiB overall; NURBS
    // shape functions depend on the element's mutable knot/weight state.
    std::unordered_map<const mfem::FiniteElement*,std::map<int,mfem::DenseMatrix>> shape_cache;
    std::size_t shape_bytes=0,shape_rules=0;
    for (const auto element:elements) {
        const auto geometry=mesh.GetElementBaseGeometry(element.index);
        const auto count=LatticeCount(geometry,element.subdivisions)+element.nodes+1;
        if (count>static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::runtime_error("Field range element sample count exceeds MFEM's integration-rule limit");
        mfem::IntegrationRule points(static_cast<int>(count));
        int next=0;
        auto add_point=[&](const mfem::IntegrationPoint& point) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                !mfem::Geometry::CheckPoint(geometry,point,1e-12))
                throw std::runtime_error("Field range encountered an invalid reference sample");
            if (next>=points.GetNPoints()) throw std::runtime_error("Field range sampling count changed during evaluation");
            points.IntPoint(next++).Set3(point.x,point.y,point.z);
        };
        auto sample_value=[&](double value) {
            if (!std::isfinite(value)) throw std::runtime_error("Field range encountered a non-finite sampled value");
            result.min_value=std::min(result.min_value,value);
            result.max_value=std::max(result.max_value,value);
            if (result.samples.size()<distribution_count && result.sample_count==distribution_next) {
                result.samples.push_back(value);
                if (result.samples.size()<distribution_count) {
                    distribution_next+=distribution_step;
                    if (distribution_error>=denominator-distribution_remainder) {
                        ++distribution_next;
                        distribution_error-=denominator-distribution_remainder;
                    } else distribution_error+=distribution_remainder;
                }
            }
            ++result.sample_count;
        };
        Lattice(geometry,element.subdivisions,add_point);
        // GetFE may reuse a mutable NURBS finite element. Copy its node rule so
        // subsequent parent-element evaluations cannot invalidate this rule.
        const mfem::IntegrationRule nodes(space->GetFE(element.index)->GetNodes());
        for (int p=0;p<nodes.GetNPoints();++p) add_point(nodes.IntPoint(p));
        add_point(mfem::Geometries.GetCenter(geometry));
        if (next!=points.GetNPoints()) throw std::runtime_error("Field range sampling count changed during evaluation");

        // Validate through the common evaluator once per parent. Rebuilding
        // curved geometry and gathering local coefficients at every sample
        // makes dense volume ranges prohibitively slow. MFEM's bulk routines
        // retain the same orientation, Piola, and integral-map semantics.
        (void)EvaluateField(field,element.index,points.IntPoint(0),options.view,options.component);
        const bool derivative=options.view==FieldView::Divergence ||
                              options.view==FieldView::CurlMagnitude || options.view==FieldView::CurlComponent;
        const auto* fe=space->GetFE(element.index);
        const bool scalar_basis=fe->GetRangeType()==mfem::FiniteElement::SCALAR;
        if (!derivative && scalar_basis && (field.VectorDim()==1 || options.view==FieldView::Component)) {
            mfem::Vector values;
            const std::size_t bytes=std::size_t(fe->GetDof())*points.GetNPoints()*sizeof(double);
            const bool cacheable=fe->GetMapType()==mfem::FiniteElement::VALUE &&
                !dynamic_cast<const mfem::NURBSFiniteElement*>(fe) && bytes<=32ULL*1024*1024;
            mfem::DenseMatrix* shapes=nullptr;
            if (cacheable) {
                auto basis=shape_cache.find(fe);
                if (basis!=shape_cache.end()) {
                    auto found=basis->second.find(element.subdivisions);
                    if (found!=basis->second.end()) shapes=&found->second;
                }
                if (!shapes && shape_rules<128 && bytes<=64ULL*1024*1024-shape_bytes) {
                    shapes=&shape_cache[fe][element.subdivisions];
                    shapes->SetSize(fe->GetDof(),points.GetNPoints());
                    mfem::Vector shape;
                    for (int p=0;p<points.GetNPoints();++p) {
                        shapes->GetColumnReference(p,shape);
                        fe->CalcShape(points.IntPoint(p),shape);
                    }
                    shape_bytes+=bytes;++shape_rules;
                }
            }
            if (shapes) {
                mfem::Array<int> dofs;
                auto* orientation=space->GetElementDofs(element.index,dofs);
                space->DofsToVDofs(options.view==FieldView::Component?options.component:0,dofs);
                mfem::Vector local;
                field.GetSubVector(dofs,local);
                if (orientation) orientation->InvTransformPrimal(local);
                values.SetSize(points.GetNPoints());
                shapes->MultTranspose(local,values);
            } else field.GetValues(element.index,points,values,options.view==FieldView::Component?options.component+1:1);
            for (double value:values) sample_value(options.view==FieldView::Magnitude?std::abs(value):value);
        } else {
            auto& transform=*mesh.GetElementTransformation(element.index);
            if (!derivative) {
                mfem::DenseMatrix values;
                field.GetVectorValues(transform,points,values);
                mfem::Vector vector;
                for (int p=0;p<points.GetNPoints();++p) {
                    values.GetColumnReference(p,vector);
                    sample_value(options.view==FieldView::Component ? vector[options.component] :
                                 (options.view==FieldView::Value && vector.Size()==1 ? vector[0] : vector.Norml2()));
                }
            } else {
                mfem::Vector curl;
                for (int p=0;p<points.GetNPoints();++p) {
                    transform.SetIntPoint(&points.IntPoint(p));
                    if (options.view==FieldView::Divergence) sample_value(field.GetDivergence(transform));
                    else {
                        field.GetCurl(transform,curl);
                        sample_value(options.view==FieldView::CurlComponent?curl[options.component]:curl.Norml2());
                    }
                }
            }
        }
    }
    result.element_count=elements.size();
    if (result.sample_count!=expected) throw std::runtime_error("Field range sampling count changed during evaluation");
    if (result.samples.size()!=distribution_count) throw std::runtime_error("Field range distribution sampling count changed during evaluation");
    return result;
}

void ApplyFieldColorRange(SurfaceData& surface, const FieldRange& range, ColorMap map, bool reverse) {
    ApplyFieldColorRange(surface,ColorScaleMapping{.scale=ColorScale::Linear,
        .minimum=range.min_value,.maximum=range.max_value},map,reverse);
}

void ApplyFieldColorRange(SurfaceData& surface, const ColorScaleMapping& mapping, ColorMap map, bool reverse) {
    if (!std::isfinite(mapping.minimum) || !std::isfinite(mapping.maximum) ||
        mapping.minimum>mapping.maximum)
        throw std::invalid_argument("Color range limits must be finite and ordered");
    const auto resolved=ResolveColorScale(mapping.minimum,mapping.maximum,
        {.scale=mapping.scale,.log_threshold=mapping.log_threshold});
    if (surface.positions.size()%3 || surface.values.size()!=surface.positions.size()/3)
        throw std::invalid_argument("Surface values must match the number of vertices");
    for (double value:surface.values)
        if (!std::isfinite(value)) throw std::invalid_argument("Surface contains non-finite field values");
    std::vector<unsigned char> colors;
    colors.reserve(surface.values.size()*4);
    for (double value:surface.values) {
        const auto color=FieldColor(value,resolved,map,reverse);
        colors.insert(colors.end(),{color.r,color.g,color.b,color.a});
    }
    surface.colors=std::move(colors);
    surface.min_value=resolved.minimum;surface.max_value=resolved.maximum;
}
} // namespace RayFEM
