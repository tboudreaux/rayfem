#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/explosion.hpp"
#include "surface_normal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_set>

namespace {

Vector3 AsVector3(const mfem::Vector& v) {
    return {v.Size() > 0 ? static_cast<float>(v[0]) : 0.0f,
            v.Size() > 1 ? static_cast<float>(v[1]) : 0.0f,
            v.Size() > 2 ? static_cast<float>(v[2]) : 0.0f};
}

Vector3 Subtract(Vector3 a, Vector3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vector3 Cross(Vector3 a, Vector3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}

void Validate(mfem::Mesh& mesh, mfem::GridFunction* field,
              const RayFEM::RenderOptions& options,
              const RayFEM::ElementSelection* selection = nullptr) {
    RayFEM::ValidateElementExplosion(options);
    RayFEM::ValidateElementSelection(mesh, options);
    if (mesh.Dimension() < 2 || mesh.Dimension() > 3 || mesh.SpaceDimension() > 3) {
        throw std::invalid_argument("Rendering requires a 2D or 3D mesh embedded in at most 3 dimensions");
    }
    if (!std::isfinite(options.displacement_scale)) {
        throw std::invalid_argument("Displacement scale must be finite");
    }
    auto check_mesh = [&](mfem::GridFunction* f) {
        if (f && (!f->FESpace() || f->FESpace()->GetMesh() != &mesh)) {
            throw std::invalid_argument("The grid function must use the displayed mesh");
        }
    };
    check_mesh(field);
    check_mesh(options.displacement);
    if (options.displacement && options.displacement->VectorDim() != mesh.SpaceDimension()) {
        throw std::invalid_argument("Displacement must have one component per spatial dimension");
    }
    if (!field || !mesh.GetNE()) { return; }
    std::optional<RayFEM::ElementSelection> local_selection;
    if (!selection) {
        local_selection.emplace(mesh, options);
        selection = &*local_selection;
    }
    int first_included = -1;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        if (selection->Includes(e)) { first_included=e; break; }
    }
    if (first_included < 0) { return; }
    const mfem::FiniteElement* fe = field->FESpace()->GetFE(first_included);
    const bool vector_fe = fe->GetRangeType() == mfem::FiniteElement::VECTOR;
    const bool div = options.view == RayFEM::FieldView::Divergence;
    const bool curl = options.view == RayFEM::FieldView::CurlMagnitude ||
                      options.view == RayFEM::FieldView::CurlComponent;
    if (div || curl) {
        if (vector_fe) {
            if ((div && fe->GetDerivType() != mfem::FiniteElement::DIV) ||
                (curl && fe->GetDerivType() != mfem::FiniteElement::CURL)) {
                throw std::invalid_argument(div ? "Divergence requires an H(div) or nodal vector field" :
                                                 "Curl requires an H(curl) or nodal vector field");
            }
        } else if (field->VectorDim() != mesh.SpaceDimension() ||
                   fe->GetMapType() != mfem::FiniteElement::VALUE) {
            throw std::invalid_argument("Curl and divergence require a spatial vector field");
        }
    }
    if (options.view == RayFEM::FieldView::Component &&
        (options.component < 0 || options.component >= field->VectorDim())) {
        throw std::invalid_argument("Field component is out of range");
    }
    if (options.view == RayFEM::FieldView::CurlComponent) {
        const int dim = vector_fe ? fe->GetCurlDim() : (mesh.SpaceDimension() == 3 ? 3 : 1);
        if (options.component < 0 || options.component >= dim) {
            throw std::invalid_argument("Curl component is out of range");
        }
    }
}

double SampleValue(mfem::GridFunction* field, mfem::ElementTransformation& transform,
                   const mfem::IntegrationPoint& ip, const RayFEM::RenderOptions& options,
                   mfem::Vector* physical_vector = nullptr) {
    if (!field) { return 0.0; }
    transform.SetIntPoint(&ip);
    mfem::Vector value;
    if (physical_vector || (options.view != RayFEM::FieldView::Divergence &&
                           options.view != RayFEM::FieldView::CurlMagnitude &&
                           options.view != RayFEM::FieldView::CurlComponent)) {
        field->GetVectorValue(transform, ip, value);
        if (physical_vector) { *physical_vector = value; }
    }
    double result = 0;
    switch (options.view) {
        case RayFEM::FieldView::Divergence:
            result = field->GetDivergence(transform); break;
        case RayFEM::FieldView::CurlMagnitude:
        case RayFEM::FieldView::CurlComponent:
            field->GetCurl(transform, value);
            result = options.view == RayFEM::FieldView::CurlMagnitude ?
                     value.Norml2() : value[options.component];
            break;
        case RayFEM::FieldView::Component:
            result = value[options.component]; break;
        case RayFEM::FieldView::Magnitude:
            result = value.Norml2(); break;
        case RayFEM::FieldView::Value:
            result = value.Size() == 1 ? value[0] : value.Norml2(); break;
    }
    if (!std::isfinite(result)) { throw std::runtime_error("Field contains non-finite sampled values"); }
    return result;
}

Vector3 SamplePosition(mfem::ElementTransformation& transform, const mfem::IntegrationPoint& ip,
                       const RayFEM::RenderOptions& options) {
    mfem::Vector position;
    transform.Transform(ip, position);
    if (options.displacement) {
        mfem::Vector displacement;
        transform.SetIntPoint(&ip);
        options.displacement->GetVectorValue(transform, ip, displacement);
        position.Add(options.displacement_scale, displacement);
    }
    for (int i = 0; i < position.Size(); ++i) {
        if (!std::isfinite(position[i])) { throw std::runtime_error("Mesh contains non-finite sampled coordinates"); }
    }
    return AsVector3(position);
}

// Reference shapes are identical on every polynomial element of a given
// type. Keep them only for this build: neither mutable NURBS basis state nor
// mesh/field data can leak across frames or finite-element-space changes.
class PatchBasisCache {
public:
    struct Basis {
        const mfem::FiniteElement* element;
        mfem::IntegrationRule points;
        mfem::DenseMatrix shapes;
        std::vector<mfem::DenseMatrix> derivatives;
        Basis(const mfem::FiniteElement& fe, const mfem::IntegrationRule& rule)
            : element(&fe), points(rule), shapes(fe.GetDof(), rule.GetNPoints()) {
            mfem::Vector shape;
            derivatives.reserve(rule.GetNPoints());
            for (int i = 0; i < rule.GetNPoints(); ++i) {
                shapes.GetColumnReference(i, shape);
                fe.CalcShape(rule.IntPoint(i), shape);
                derivatives.emplace_back(fe.GetDof(), fe.GetDim());
                fe.CalcDShape(rule.IntPoint(i), derivatives.back());
            }
        }
    };

    const Basis* Get(const mfem::FiniteElement& fe, const mfem::IntegrationRule& points) {
        if (fe.GetRangeType() != mfem::FiniteElement::SCALAR ||
            fe.GetMapType() != mfem::FiniteElement::VALUE ||
            dynamic_cast<const mfem::NURBSFiniteElement*>(&fe)) { return nullptr; }
        for (const auto& entry : entries) {
            if (entry->element != &fe || entry->points.GetNPoints() != points.GetNPoints()) { continue; }
            bool same = true;
            for (int i = 0; i < points.GetNPoints(); ++i) {
                const auto& a = entry->points.IntPoint(i);
                const auto& b = points.IntPoint(i);
                if (a.x != b.x || a.y != b.y || a.z != b.z) { same = false; break; }
            }
            if (same) { return entry.get(); }
        }
        const std::size_t count = std::size_t(fe.GetDof())*points.GetNPoints()*(1+fe.GetDim());
        // Bound the temporary cache for very high-order/mixed meshes.
        if (count > 4*1024*1024 - scalars) { return nullptr; }
        entries.push_back(std::make_unique<Basis>(fe, points));
        scalars += count;
        return entries.back().get();
    }
private:
    std::vector<std::unique_ptr<Basis>> entries;
    std::size_t scalars = 0;
};

struct PatchField {
    mfem::DenseMatrix values;
    mfem::Vector local_values;
    const PatchBasisCache::Basis* basis = nullptr;
    int dimension = 0;

    void Sample(mfem::GridFunction& field, mfem::ElementTransformation& transform,
                const mfem::IntegrationRule& points, PatchBasisCache& cache) {
        const auto* fe = field.FESpace()->GetFE(transform.ElementNo);
        basis = cache.Get(*fe, points);
        dimension = field.FESpace()->GetVDim();
        if (!basis) {
            // MFEM batches coefficient gathering and preserves Piola maps,
            // orientation transformations and NURBS element state.
            field.GetVectorValues(transform, points, values);
            return;
        }
        field.GetElementDofValues(transform.ElementNo, local_values);
        mfem::DenseMatrix coefficients(local_values.GetData(), fe->GetDof(), dimension);
        values.SetSize(dimension, points.GetNPoints());
        mfem::MultAtB(coefficients, basis->shapes, values);
    }

    void Gradient(int point, mfem::DenseMatrix& gradient) {
        const auto& derivative = basis->derivatives[point];
        mfem::DenseMatrix coefficients(local_values.GetData(), derivative.Height(), dimension);
        gradient.SetSize(dimension, derivative.Width());
        mfem::MultAtB(coefficients, derivative, gradient);
    }
};

Vector3 PatchNormal(const mfem::DenseMatrix& derivative,
                    mfem::IntegrationPointTransformation* face_map,
                    const mfem::IntegrationPoint& face_point,
                    mfem::DenseMatrix& tangents) {
    if (face_map) {
        face_map->Transf.SetIntPoint(&face_point);
        tangents.SetSize(derivative.Height(), 2);
        mfem::Mult(derivative, face_map->Transf.Jacobian(), tangents);
    } else { tangents = derivative; }
    if (tangents.Width() != 2) { return {}; }
    const double ax=tangents(0,0), ay=tangents(1,0), az=tangents.Height()>2?tangents(2,0):0;
    const double bx=tangents(0,1), by=tangents(1,1), bz=tangents.Height()>2?tangents(2,1):0;
    const double x=ay*bz-az*by, y=az*bx-ax*bz, z=ax*by-ay*bx;
    const double length=std::hypot(x,y,z);
    if (!std::isfinite(length) || length<=std::numeric_limits<double>::min()) { return {}; }
    return {float(x/length),float(y/length),float(z/length)};
}

// NURBS coefficients are not point values. Mark their Greville abscissae,
// using the supporting element containing the point (never extrapolating).
bool NURBSDofPoint(const mfem::NURBSFiniteElement& fe, int local_dof,
                  mfem::IntegrationPoint& ip) {
    const auto& knots = fe.KnotVectors();
    const int* ijk = fe.GetIJK();
    double xyz[3]{};
    for (int d = 0; d < knots.Size(); ++d) {
        const int p = knots[d]->GetOrder();
        const int local = local_dof % (p+1);
        local_dof /= p+1;
        const int span = ijk[d]+p;
        const double a = (*knots[d])[span];
        const double b = (*knots[d])[span+1];
        if (b <= a) { return false; }
        double greville = 0;
        for (int k = 1; k <= p; ++k) { greville += (*knots[d])[ijk[d]+local+k]; }
        greville = p ? greville/p : (a+b)*0.5;
        xyz[d] = (greville-a)/(b-a);
        if (xyz[d] < -1e-10 || xyz[d] > 1+1e-10) { return false; }
        xyz[d] = std::clamp(xyz[d], 0.0, 1.0);
    }
    ip.Set3(xyz[0], xyz[1], xyz[2]);
    return true;
}

} // namespace

namespace RayFEM::detail {
Vector3 SurfaceNormal(mfem::ElementTransformation& transform,
                      const mfem::IntegrationPoint& parent,
                      const RenderOptions& options,
                      mfem::IntegrationPointTransformation* face_map,
                      const mfem::IntegrationPoint* face_point) {
    transform.SetIntPoint(&parent);
    mfem::DenseMatrix derivative(transform.Jacobian());
    if (options.displacement && options.displacement_scale != 0) {
        const auto* fe = options.displacement->FESpace()->GetFE(transform.ElementNo);
        mfem::DenseMatrix gradient;
        if (fe->GetRangeType() == mfem::FiniteElement::SCALAR &&
            fe->GetMapType() == mfem::FiniteElement::VALUE) {
            // Reference derivatives avoid an inverse Jacobian and remain
            // valid for embedded surfaces and rational geometry.
            options.displacement->GetVectorGradientHat(transform, gradient);
        } else {
            gradient.SetSize(derivative.Height(), derivative.Width());
            // Piola/integral mapped displacement needs the derivative of its
            // physical value. Keep numerical probes inside the parent cell,
            // including at boundary corners where an axis has no two-sided
            // neighborhood.
            constexpr double h = 1e-7, inset = 32*h;
            auto base = parent;
            const auto& center = mfem::Geometries.GetCenter(transform.GetGeometryType());
            base.x += inset*(center.x-base.x);
            base.y += inset*(center.y-base.y);
            base.z += inset*(center.z-base.z);
            for (int d = 0; d < derivative.Width(); ++d) {
                auto plus=base, minus=base;
                if(d==0){plus.x+=h;minus.x-=h;}
                if(d==1){plus.y+=h;minus.y-=h;}
                if(d==2){plus.z+=h;minus.z-=h;}
                mfem::Vector a,b;
                transform.SetIntPoint(&plus);
                options.displacement->GetVectorValue(transform,plus,a);
                transform.SetIntPoint(&minus);
                options.displacement->GetVectorValue(transform,minus,b);
                for(int row=0;row<derivative.Height();++row)gradient(row,d)=(a[row]-b[row])/(2*h);
            }
            transform.SetIntPoint(&parent);
        }
        derivative.Add(options.displacement_scale, gradient);
    }
    mfem::DenseMatrix tangents;
    if (face_map) {
        face_map->Transf.SetIntPoint(face_point);
        tangents.SetSize(derivative.Height(),2);
        mfem::Mult(derivative,face_map->Transf.Jacobian(),tangents);
    } else { tangents=derivative; }
    if(tangents.Width()!=2)return {};
    const double ax=tangents(0,0),ay=tangents(1,0),az=tangents.Height()>2?tangents(2,0):0;
    const double bx=tangents(0,1),by=tangents(1,1),bz=tangents.Height()>2?tangents(2,1):0;
    const double x=ay*bz-az*by,y=az*bx-ax*bz,z=ax*by-ay*bx;
    const double length=std::hypot(x,y,z);
    if(!std::isfinite(length)||length<=std::numeric_limits<double>::min())return {};
    return {float(x/length),float(y/length),float(z/length)};
}
} // namespace RayFEM::detail

namespace RayFEM {

int RecommendedSurfaceRefinement(const mfem::Mesh& mesh,const mfem::GridFunction* displacement) {
    int order=1;
    for(const auto* field:{mesh.GetNodes(),displacement}) {
        if(field&&field->FESpace()) {
            order=std::max(order,field->FESpace()->GetMaxElementOrder());
            // Anisotropic NURBS collections report VariableOrder instead of a
            // degree; use their individual knot-vector orders in that case.
            if(const auto* nurbs=field->FESpace()->GetNURBSext();nurbs&&nurbs->GetOrders().Size())
                order=std::max(order,nurbs->GetOrders().Max());
        }
    }
    return std::clamp(order>1?std::max(6,2*std::min(order,16)):3,3,32);
}

void ValidateElementSelection(const mfem::Mesh& mesh, const RenderOptions& options) {
    for (int element : options.excluded_elements) {
        if (element < 0 || element >= mesh.GetNE()) {
            throw std::invalid_argument("Excluded element index is out of range for the displayed mesh");
        }
    }
}

ElementSelection::ElementSelection(const mfem::Mesh& mesh, const RenderOptions& options)
    : element_count_(mesh.GetNE()) {
    ValidateElementSelection(mesh, options);
    if (options.excluded_elements.empty() && options.excluded_materials.empty()) { return; }
    included_.assign(element_count_, true);
    for (int element : options.excluded_elements) { included_[element] = false; }
    if (!options.excluded_materials.empty()) {
        const std::unordered_set<int> materials(options.excluded_materials.begin(),options.excluded_materials.end());
        for (int e = 0; e < element_count_; ++e) {
            if (materials.contains(mesh.GetAttribute(e))) { included_[e] = false; }
        }
    }
}

bool IsElementIncluded(const mfem::Mesh& mesh, int element, const RenderOptions& options) {
    return element >= 0 && element < mesh.GetNE() &&
        std::find(options.excluded_elements.begin(), options.excluded_elements.end(), element) == options.excluded_elements.end() &&
        std::find(options.excluded_materials.begin(), options.excluded_materials.end(),
                  mesh.GetAttribute(element)) == options.excluded_materials.end();
}

Color AttributeColor(int attribute) {
    if (!attribute) { return {150, 158, 170, 255}; }
    const std::uint32_t hash = static_cast<std::uint32_t>(attribute)*2654435761U;
    const double hue = double(hash)*(6.0/4294967296.0);
    const int sector = static_cast<int>(hue);
    const double f = hue-sector, v = 0.92, p = 0.28, q = v-(v-p)*f, t = p+(v-p)*f;
    const double rgb[6][3] = {{v,t,p},{q,v,p},{p,v,t},{p,q,v},{t,p,v},{v,p,q}};
    return {static_cast<unsigned char>(255*rgb[sector][0]), static_cast<unsigned char>(255*rgb[sector][1]),
            static_cast<unsigned char>(255*rgb[sector][2]), 255};
}

std::vector<int> FaceBoundaryAttributes(const mfem::Mesh& mesh) {
    std::vector<int> attributes(mesh.GetNumFaces(), 0);
    for (int b = 0; b < mesh.GetNBE(); ++b) {
        const int face = mesh.GetBdrElementFaceIndex(b);
        if (face >= 0 && face < static_cast<int>(attributes.size())) { attributes[face] = mesh.GetBdrAttribute(b); }
    }
    return attributes;
}

SurfaceData BuildSurface(mfem::Mesh& mesh, mfem::GridFunction* field, const RenderOptions& options) {
    // Attribute coloring depends only on geometry, so an unavailable derivative
    // view or a non-finite field cannot prevent inspection of mesh attributes.
    if (options.coloring != SurfaceColoring::Field) { field = nullptr; }
    const ElementSelection selection(mesh, options);
    Validate(mesh, field, options, &selection);
    SurfaceData result;
    mfem::GeometryRefiner refiner;
    const int refinement = std::clamp(options.refinement, 1, 64);
    PatchBasisCache basis_cache;
    struct Sample { Vector3 point; double value; int material, boundary; Vector3 normal; };
    std::vector<int> render_faces;
    std::size_t capacity = 0;
    auto patch_vertices = [refinement](mfem::Geometry::Type geometry) {
        return std::size_t(geometry == mfem::Geometry::SQUARE ? 6 : 3)*refinement*refinement;
    };
    if (mesh.Dimension() == 2) {
        for (int e = 0; e < mesh.GetNE(); ++e) {
            if (selection.Includes(e)) { capacity += patch_vertices(mesh.GetElementBaseGeometry(e)); }
        }
    } else {
        for (int face = 0; face < mesh.GetNumFaces(); ++face) {
            const auto info = mesh.GetFaceInformation(face);
            if (info.IsNonconformingCoarse()) { continue; }
            const bool a = info.element[0].location == mfem::Mesh::ElementLocation::Local &&
                selection.Includes(info.element[0].index);
            const bool b = info.element[1].location == mfem::Mesh::ElementLocation::Local &&
                selection.Includes(info.element[1].index);
            if ((!a && !b) || (a && b && options.element_explosion == 0)) { continue; }
            render_faces.push_back(face);
            capacity += (int(a)+int(b))*patch_vertices(mesh.GetFaceGeometry(face));
        }
    }
    result.positions.reserve(capacity*3); result.normals.reserve(capacity*3);
    result.values.reserve(capacity); result.material_ids.reserve(capacity); result.boundary_ids.reserve(capacity);
    result.element_ids.reserve(capacity/3);

    auto add_triangle = [&](const Sample& a, const Sample& b, const Sample& c, bool reverse, int element) {
        Vector3 normal = Cross(Subtract(b.point, a.point), Subtract(c.point, a.point));
        const double length = std::sqrt(double(normal.x)*normal.x + double(normal.y)*normal.y + double(normal.z)*normal.z);
        if (length <= std::numeric_limits<float>::min()) { return; }
        result.element_ids.push_back(element);
        normal.x /= length; normal.y /= length; normal.z /= length;
        if (reverse) { normal.x = -normal.x; normal.y = -normal.y; normal.z = -normal.z; }
        for (const Sample* sample : {&a, reverse ? &c : &b, reverse ? &b : &c}) {
            Vector3 vertex_normal=sample->normal;
            if(vertex_normal.x==0&&vertex_normal.y==0&&vertex_normal.z==0)vertex_normal=normal;
            else if(reverse){vertex_normal.x=-vertex_normal.x;vertex_normal.y=-vertex_normal.y;vertex_normal.z=-vertex_normal.z;}
            result.positions.insert(result.positions.end(), {sample->point.x, sample->point.y, sample->point.z});
            result.normals.insert(result.normals.end(), {vertex_normal.x, vertex_normal.y, vertex_normal.z});
            result.values.push_back(sample->value);
            result.material_ids.push_back(sample->material); result.boundary_ids.push_back(sample->boundary);
        }
    };

    auto add_patch = [&](mfem::Geometry::Type geometry, mfem::ElementTransformation& transform,
                         mfem::IntegrationPointTransformation* face_to_element, int element,
                         int boundary, bool reverse) {
        if (geometry != mfem::Geometry::TRIANGLE && geometry != mfem::Geometry::SQUARE) {
            throw std::invalid_argument("Unsupported surface geometry");
        }
        const auto* refined = refiner.Refine(geometry, refinement, 1);
        const auto& indices = refined->RefGeoms;
        const auto display = MakeElementDisplayTransform(transform, options);
        mfem::IntegrationRule parent_points(refined->RefPts.GetNPoints());
        if (face_to_element) { face_to_element->Transform(refined->RefPts, parent_points); }
        else { parent_points = refined->RefPts; }

        auto* iso = dynamic_cast<mfem::IsoparametricTransformation*>(&transform);
        const auto* geometry_basis = iso ? basis_cache.Get(*iso->GetFE(), parent_points) : nullptr;
        mfem::DenseMatrix positions, geometry_coefficients;
        if (geometry_basis) {
            // Keep each coordinate's coefficients contiguous. MFEM's dot
            // product batch avoids repeatedly updating tiny 2/3-row outputs.
            geometry_coefficients.Transpose(iso->GetPointMat());
            positions.SetSize(transform.GetSpaceDim(), parent_points.GetNPoints());
            mfem::MultAtB(geometry_coefficients, geometry_basis->shapes, positions);
        } else { transform.Transform(parent_points, positions); }

        PatchField field_samples, displacement_samples;
        const bool derivative_view = options.view == FieldView::Divergence ||
            options.view == FieldView::CurlMagnitude || options.view == FieldView::CurlComponent;
        if (field && !derivative_view) { field_samples.Sample(*field, transform, parent_points, basis_cache); }
        if (options.displacement) {
            displacement_samples.Sample(*options.displacement, transform, parent_points, basis_cache);
            positions.Add(options.displacement_scale, displacement_samples.values);
        }
        mfem::DenseMatrix derivative, gradient, tangents;
        mfem::Vector value;
        std::vector<Sample> samples;
        samples.reserve(refined->RefPts.GetNPoints());
        for (int j = 0; j < refined->RefPts.GetNPoints(); ++j) {
            const auto& ip = parent_points.IntPoint(j);
            Vector3 position{};
            for (int d = 0; d < positions.Height(); ++d) {
                if (!std::isfinite(positions(d,j))) { throw std::runtime_error("Mesh contains non-finite sampled coordinates"); }
            }
            position = {float(positions(0,j)), float(positions(1,j)), positions.Height()>2?float(positions(2,j)):0};
            double scalar = 0;
            if (field) {
                if (derivative_view) { scalar = SampleValue(field, transform, ip, options); }
                else {
                    field_samples.values.GetColumnReference(j, value);
                    scalar = options.view == FieldView::Component ? value[options.component] :
                        (options.view == FieldView::Value && value.Size() == 1 ? value[0] : value.Norml2());
                    if (!std::isfinite(scalar)) { throw std::runtime_error("Field contains non-finite sampled values"); }
                }
            }
            Vector3 normal;
            if (options.displacement && options.displacement_scale != 0 && !displacement_samples.basis) {
                normal = detail::SurfaceNormal(transform, ip, options, face_to_element, &refined->RefPts.IntPoint(j));
            } else {
                if (geometry_basis) {
                    derivative.SetSize(transform.GetSpaceDim(), transform.GetDimension());
                    mfem::MultAtB(geometry_coefficients, geometry_basis->derivatives[j], derivative);
                } else {
                    transform.SetIntPoint(&ip);
                    derivative = transform.Jacobian();
                }
                if (options.displacement && options.displacement_scale != 0) {
                    displacement_samples.Gradient(j, gradient);
                    derivative.Add(options.displacement_scale, gradient);
                }
                normal = PatchNormal(derivative, face_to_element, refined->RefPts.IntPoint(j), tangents);
            }
            samples.push_back({display.Apply(position), scalar, mesh.GetAttribute(element), boundary, normal});
        }
        // MFEM RefGeoms contains quads for SQUARE, not triangle triples.
        const int stride = geometry == mfem::Geometry::SQUARE ? 4 : 3;
        for (int j = 0; j < indices.Size(); j += stride) {
            add_triangle(samples[indices[j]], samples[indices[j+1]], samples[indices[j+2]], reverse, element);
            if (stride == 4) { add_triangle(samples[indices[j]], samples[indices[j+2]], samples[indices[j+3]], reverse, element); }
        }
    };

    if (mesh.Dimension() == 2) {
        for (int e = 0; e < mesh.GetNE(); ++e) {
            if (!selection.Includes(e)) { continue; }
            add_patch(mesh.GetElementBaseGeometry(e), *mesh.GetElementTransformation(e), nullptr, e, 0, false);
        }
    } else {
        const auto boundary_attributes = FaceBoundaryAttributes(mesh);
        for (int face : render_faces) {
            const auto info = mesh.GetFaceInformation(face);
            // Most faces in a volume mesh are hidden. Avoid building their
            // reference maps (and fetching parent geometry) at all.
            const bool a = info.element[0].location == mfem::Mesh::ElementLocation::Local &&
                selection.Includes(info.element[0].index);
            const bool b = info.element[1].location == mfem::Mesh::ElementLocation::Local &&
                selection.Includes(info.element[1].index);
            // NURBS elements share mutable basis state: loading both physical
            // parent transforms is unsupported. Reference maps remain valid
            // independently, including MFEM's nonconforming face composition.
            auto* tr = mesh.GetFaceElementTransformations(face,
                mfem::FaceElementTransformations::HAVE_LOC1 |
                mfem::FaceElementTransformations::HAVE_LOC2);
            if (!tr) { continue; }
            // MFEM face orientation is outward from element 1; exposed faces
            // owned by element 2 require both reversed winding and its trace.
            if (a) { add_patch(mesh.GetFaceGeometry(face), *mesh.GetElementTransformation(tr->Elem1No),
                               &tr->Loc1, tr->Elem1No, boundary_attributes[face], false); }
            if (b) { add_patch(mesh.GetFaceGeometry(face), *mesh.GetElementTransformation(tr->Elem2No),
                               &tr->Loc2, tr->Elem2No, boundary_attributes[face], true); }
        }
    }
    if (!result.values.empty()) {
        const auto [minimum, maximum] = std::minmax_element(result.values.begin(), result.values.end());
        result.min_value = *minimum;
        result.max_value = *maximum;
        const auto mapping=field && options.coloring==SurfaceColoring::Field ?
            ResolveColorScale(*minimum,*maximum,options.color_scale,result.values) : ColorScaleMapping{};
        if (field && options.coloring==SurfaceColoring::Field) {
            result.min_value=mapping.minimum;result.max_value=mapping.maximum;
        }
        result.bounds.min = result.bounds.max = {result.positions[0], result.positions[1], result.positions[2]};
        result.colors.reserve(result.values.size()*4);
        for (std::size_t i = 0; i < result.values.size(); ++i) {
            const Color color = options.coloring == SurfaceColoring::Material ? AttributeColor(result.material_ids[i]) :
                options.coloring == SurfaceColoring::Boundary ? AttributeColor(result.boundary_ids[i]) :
                field ? FieldColor(result.values[i], mapping, options.color_map, options.reverse_colors) : Color{155, 195, 220, 255};
            result.colors.insert(result.colors.end(), {color.r, color.g, color.b, color.a});
            const Vector3 p{result.positions[3*i], result.positions[3*i+1], result.positions[3*i+2]};
            result.bounds.min.x = std::min(result.bounds.min.x, p.x); result.bounds.max.x = std::max(result.bounds.max.x, p.x);
            result.bounds.min.y = std::min(result.bounds.min.y, p.y); result.bounds.max.y = std::max(result.bounds.max.y, p.y);
            result.bounds.min.z = std::min(result.bounds.min.z, p.z); result.bounds.max.z = std::max(result.bounds.max.z, p.z);
        }
    }
    return result;
}

std::vector<DofPoint> SampleDofs(mfem::Mesh& mesh, mfem::GridFunction& field,
                                const RenderOptions& options, int max_points,
                                const std::vector<bool>* retained_elements) {
    const ElementSelection selection(mesh, options);
    Validate(mesh, &field, options, &selection);
    if (retained_elements && retained_elements->size() != static_cast<std::size_t>(mesh.GetNE())) {
        throw std::invalid_argument("DOF selection must contain one flag per mesh element");
    }
    std::vector<DofPoint> points;
    if (max_points <= 0) { return points; }
    auto& space = *field.FESpace();
    std::vector<bool> visited(space.GetNDofs(), false);
    const bool exploded = options.element_explosion > 0;
    std::size_t candidates = space.GetNDofs();
    if (exploded) {
        candidates = 0;
        for (int e = 0; e < mesh.GetNE(); ++e) {
            if (selection.Includes(e) && (!retained_elements || (*retained_elements)[e])) {
                candidates += space.GetFE(e)->GetDof();
            }
        }
    }
    const std::size_t stride = std::max<std::size_t>(1, (candidates+max_points-1)/max_points);
    std::size_t local_candidate = 0;
    mfem::Array<int> dofs;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        if (!selection.Includes(e)) { continue; }
        if (retained_elements && !(*retained_elements)[e]) { continue; }
        auto* transform = mesh.GetElementTransformation(e);
        const auto display = MakeElementDisplayTransform(*transform, options);
        const auto* fe = space.GetFE(e);
        const auto* nurbs = dynamic_cast<const mfem::NURBSFiniteElement*>(fe);
        // Vector NURBS bases have separate directional knot orders and no
        // MFEM point-functional nodes; scalar Greville indexing is not valid.
        if (nurbs && fe->GetRangeType() == mfem::FiniteElement::VECTOR) { continue; }
        const auto& nodes = fe->GetNodes();
        space.GetElementDofs(e, dofs);
        for (int j = 0; j < dofs.Size(); ++j) {
            const int index = dofs[j] < 0 ? -1-dofs[j] : dofs[j];
            if (exploded ? local_candidate++%stride != 0 : (visited[index] || index%stride)) { continue; }
            mfem::IntegrationPoint ip;
            if (nurbs) {
                if (!NURBSDofPoint(*nurbs, j, ip)) { continue; }
            } else if (j < nodes.GetNPoints()) {
                ip = nodes.IntPoint(j);
            } else { continue; }
            DofPoint point;
            point.position = display.Apply(SamplePosition(*transform, ip, options));
            point.index = index;
            point.element = e;
            mfem::Vector value;
            point.value = SampleValue(&field, *transform, ip, options, &value);
            point.vector = AsVector3(value);
            point.directional = fe->GetRangeType() == mfem::FiniteElement::VECTOR;
            point.interpolatory = !point.directional && !nurbs && dynamic_cast<const mfem::NodalFiniteElement*>(fe);
            const int component = std::clamp(options.component, 0, space.GetVDim()-1);
            point.coefficient = field[space.DofToVDof(index, component)];
            points.push_back(point);
            visited[index] = true;
        }
    }
    return points;
}

std::vector<VectorSample> SampleVectors(mfem::Mesh& mesh, mfem::GridFunction& field,
                                       const RenderOptions& options, int max_samples,
                                       const std::vector<bool>* retained_elements) {
    const ElementSelection selection(mesh, options);
    Validate(mesh, &field, options, &selection);
    if (retained_elements && retained_elements->size() != static_cast<std::size_t>(mesh.GetNE())) {
        throw std::invalid_argument("Vector selection must contain one flag per mesh element");
    }
    std::vector<VectorSample> samples;
    if (field.VectorDim() < 2 || max_samples <= 0) { return samples; }
    int eligible = 0;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        if (selection.Includes(e) && (!retained_elements || (*retained_elements)[e])) { ++eligible; }
    }
    const int stride = std::max(1, (eligible+max_samples-1)/max_samples);
    int selected = 0;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        if (!selection.Includes(e)) { continue; }
        if (retained_elements && !(*retained_elements)[e]) { continue; }
        if (selected++%stride) { continue; }
        auto& transform = *mesh.GetElementTransformation(e);
        const auto display = MakeElementDisplayTransform(transform, options);
        const auto& ip = mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e));
        mfem::Vector value;
        transform.SetIntPoint(&ip);
        field.GetVectorValue(transform, ip, value);
        samples.push_back({display.Apply(SamplePosition(transform, ip, options)), AsVector3(value), e});
    }
    return samples;
}

double EvaluateField(mfem::GridFunction& field, int element, const mfem::IntegrationPoint& point,
                     FieldView view, int component) {
    auto& mesh = *field.FESpace()->GetMesh();
    RenderOptions options;
    options.view = view; options.component = component;
    Validate(mesh, &field, options);
    if (element < 0 || element >= mesh.GetNE()) { throw std::out_of_range("Element index is out of range"); }
    return SampleValue(&field, *mesh.GetElementTransformation(element), point, options);
}

Model UploadSurface(const SurfaceData& surface) {
    if (surface.positions.empty()) { return Model{}; }
    if (surface.positions.size()%9 || surface.normals.size() != surface.positions.size() ||
        surface.colors.size() != surface.positions.size()/3*4) {
        throw std::invalid_argument("Invalid surface buffers");
    }
    ::Mesh mesh{};
    mesh.vertexCount = static_cast<int>(surface.positions.size()/3);
    mesh.triangleCount = mesh.vertexCount/3;
    mesh.vertices = static_cast<float*>(MemAlloc(surface.positions.size()*sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(surface.normals.size()*sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(surface.colors.size()));
    if (!mesh.vertices || !mesh.normals || !mesh.colors) {
        MemFree(mesh.vertices); MemFree(mesh.normals); MemFree(mesh.colors);
        throw std::bad_alloc();
    }
    std::copy(surface.positions.begin(), surface.positions.end(), mesh.vertices);
    std::copy(surface.normals.begin(), surface.normals.end(), mesh.normals);
    std::copy(surface.colors.begin(), surface.colors.end(), mesh.colors);
    UploadMesh(&mesh, false);
    return LoadModelFromMesh(mesh);
}

Model LoadModelFromMFEM(mfem::Mesh& mesh, mfem::GridFunction& field, int refinement_level) {
    RenderOptions options;
    options.refinement = refinement_level;
    return UploadSurface(BuildSurface(mesh, &field, options));
}

} // namespace RayFEM
