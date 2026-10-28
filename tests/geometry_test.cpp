#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/clipping.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool test, const std::string& message) {
    if (!test) { throw std::runtime_error(message); }
}
void Near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance) {
        std::ostringstream out;
        out << message << ": got " << actual << ", expected " << expected;
        throw std::runtime_error(out.str());
    }
}
void Buffers(const RayFEM::SurfaceData& data) {
    Check(!data.positions.empty() && data.positions.size()%9 == 0, "Triangle position buffer");
    Check(data.normals.size() == data.positions.size(), "Normal buffer size");
    Check(data.values.size()*3 == data.positions.size(), "Value buffer size");
    Check(data.colors.size() == data.values.size()*4, "Color buffer size");
    for (std::size_t i = 0; i < data.normals.size(); i += 3) {
        Near(std::sqrt(data.normals[i]*data.normals[i]+data.normals[i+1]*data.normals[i+1]+
                       data.normals[i+2]*data.normals[i+2]), 1, 2e-6, "Unit normals");
    }
}
std::size_t ExpectedTriangles(mfem::Mesh& mesh, int refinement) {
    std::size_t cells = 0;
    if (mesh.Dimension() == 2) {
        for (int e = 0; e < mesh.GetNE(); ++e) {
            cells += mesh.GetElementBaseGeometry(e) == mfem::Geometry::SQUARE ? 2 : 1;
        }
    } else {
        for (int f = 0; f < mesh.GetNumFaces(); ++f) {
            if (mesh.GetFaceInformation(f).IsBoundary()) {
                cells += mesh.GetFaceGeometry(f) == mfem::Geometry::SQUARE ? 2 : 1;
            }
        }
    }
    return cells*refinement*refinement;
}
void VerifyOutward(const RayFEM::SurfaceData& data) {
    for (std::size_t i = 0; i < data.positions.size(); i += 9) {
        double dot = 0;
        for (int d = 0; d < 3; ++d) {
            const double center = (data.positions[i+d]+data.positions[i+d+3]+data.positions[i+d+6])/3;
            dot += (center-0.5)*data.normals[i+d];
        }
        Check(dot > 0, "Every volume surface triangle must face outward");
    }
}
void SurfaceCase(mfem::Element::Type type, int dim, bool curved) {
    auto mesh = dim == 2 ? mfem::Mesh::MakeCartesian2D(2, 2, type, true) :
                          mfem::Mesh::MakeCartesian3D(2, 2, 2, type);
    if (curved) {
        mesh.SetCurvature(3, false, 3);
        mfem::VectorFunctionCoefficient deformation(3, [dim](const mfem::Vector& x, mfem::Vector& y) {
            y = x;
            if (dim == 2) { y[2] = .25*x[0]*x[1]; }
            else {
                y[0] += .06*x[1]*(1-x[1]);
                y[1] += .04*x[2]*(1-x[2]);
                y[2] += .08*x[0]*(1-x[0])*x[1];
            }
        });
        mesh.Transform(deformation);
    }
    mfem::H1_FECollection collection(3, dim);
    mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& x) { return x[0]+2*x[1]; });
    field.ProjectCoefficient(coefficient);
    for (int refinement : {1, 3, 6}) {
        RayFEM::RenderOptions options;
        options.refinement = refinement;
        const auto surface = RayFEM::BuildSurface(mesh, &field, options);
        Buffers(surface);
        Check(surface.positions.size()/9 == ExpectedTriangles(mesh, refinement), "Correct exterior triangle count");
        for (std::size_t i = 0; i < surface.values.size(); ++i) {
            Near(surface.values[i], surface.positions[3*i]+2*surface.positions[3*i+1], 8e-7,
                 "Parent-element field values agree with physical coordinates");
        }
        if (dim == 3) { VerifyOutward(surface); }
        else {
            for (std::size_t i = 2; i < surface.normals.size(); i += 3) {
                Check(surface.normals[i] > 0, "2D triangle/quad winding");
            }
            if (curved) { Near(surface.bounds.max.z, .25, 1e-6, "Curved geometry height"); }
        }
    }
    const auto points = RayFEM::SampleDofs(mesh, field);
    Check(points.size() == static_cast<std::size_t>(space.GetNDofs()), "All unique H1 DOFs sampled");
    for (const auto& point : points) {
        Near(point.value, point.coefficient, 2e-10, "Nodal coefficient equals point evaluation");
        Check(point.interpolatory && !point.directional, "H1 markers are point DOFs");
    }
    Check(RayFEM::SampleDofs(mesh, field, {}, 7).size() <= 7, "DOF sample cap");
    std::cout << "PASS " << dim << "D type " << type << (curved ? " curved" : " straight") << '\n';
}
void DisplacementCase() {
    auto mesh = mfem::Mesh::MakeCartesian3D(2, 2, 2, mfem::Element::HEXAHEDRON);
    mesh.SetCurvature(3);
    const mfem::Vector original(*mesh.GetNodes());
    mfem::H1_FECollection collection(3, 3);
    mfem::FiniteElementSpace space(&mesh, &collection, 3);
    mfem::GridFunction displacement(&space);
    mfem::VectorFunctionCoefficient coefficient(3, [](const mfem::Vector& x, mfem::Vector& y) {
        y.SetSize(3); y[0] = .2*x[0]; y[1] = -.1*x[1]; y[2] = .05*x[0]*x[1];
    });
    displacement.ProjectCoefficient(coefficient);
    const auto base = RayFEM::BuildSurface(mesh);
    RayFEM::RenderOptions options;
    options.displacement = &displacement;
    options.displacement_scale = 2;
    const auto shifted = RayFEM::BuildSurface(mesh, nullptr, options);
    Check(base.positions.size() == shifted.positions.size(), "Displacement preserves topology");
    for (std::size_t i = 0; i < base.positions.size(); i += 3) {
        Near(shifted.positions[i], 1.4*base.positions[i], 2e-7, "Displaced x");
        Near(shifted.positions[i+1], .8*base.positions[i+1], 2e-7, "Displaced y");
        Near(shifted.positions[i+2], base.positions[i+2]+.1*base.positions[i]*base.positions[i+1], 2e-7, "Displaced z");
    }
    for (int i = 0; i < original.Size(); ++i) { Near((*mesh.GetNodes())[i], original[i], 0, "Base mesh unchanged"); }
    const auto again = RayFEM::BuildSurface(mesh);
    Check(base.positions == again.positions, "Displacement does not accumulate between frames");
    std::cout << "PASS displacement and immutable base geometry\n";
}
void VectorCase(int dim, mfem::Element::Type type, mfem::Ordering::Type ordering = mfem::Ordering::byNODES) {
    auto mesh = dim == 2 ? mfem::Mesh::MakeCartesian2D(2, 2, type, true) :
                          mfem::Mesh::MakeCartesian3D(2, 2, 2, type);
    mfem::ND_FECollection nd(2, dim);
    mfem::RT_FECollection rt(1, dim);
    mfem::H1_FECollection h1(2, dim);
    mfem::FiniteElementSpace nd_space(&mesh, &nd), rt_space(&mesh, &rt), h1_space(&mesh, &h1, dim, ordering);
    mfem::GridFunction circulation(&nd_space), flux(&rt_space), velocity(&h1_space);
    mfem::VectorFunctionCoefficient rotation(dim, [dim](const mfem::Vector& x, mfem::Vector& y) {
        y.SetSize(dim); y = 0.0; y[0] = -x[1]; y[1] = x[0];
    });
    mfem::VectorFunctionCoefficient radial(dim, [dim](const mfem::Vector& x, mfem::Vector& y) {
        y.SetSize(dim); for (int i = 0; i < dim; ++i) { y[i] = x[i]; }
    });
    circulation.ProjectCoefficient(rotation); flux.ProjectCoefficient(radial); velocity.ProjectCoefficient(rotation);
    RayFEM::RenderOptions options;
    options.view = RayFEM::FieldView::CurlMagnitude;
    for (mfem::GridFunction* field : {&circulation, &velocity}) {
        const auto surface = RayFEM::BuildSurface(mesh, field, options);
        Buffers(surface);
        for (double value : surface.values) { Near(value, 2, 3e-10, "ND/H1 physical curl"); }
    }
    options.view = RayFEM::FieldView::CurlComponent;
    options.component = dim == 3 ? 2 : 0;
    for (double value : RayFEM::BuildSurface(mesh, &circulation, options).values) { Near(value, 2, 3e-10, "Signed curl component"); }
    options.view = RayFEM::FieldView::Divergence;
    for (double value : RayFEM::BuildSurface(mesh, &flux, options).values) { Near(value, dim, 3e-10, "RT physical divergence"); }
    velocity.ProjectCoefficient(radial);
    for (double value : RayFEM::BuildSurface(mesh, &velocity, options).values) { Near(value, dim, 3e-10, "H1 physical divergence"); }
    bool rejected = false;
    try { RayFEM::BuildSurface(mesh, &circulation, options); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "ND divergence rejects safely");
    options.view = RayFEM::FieldView::CurlMagnitude;
    rejected = false;
    try { RayFEM::BuildSurface(mesh, &flux, options); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "RT curl rejects safely");
    options.view = RayFEM::FieldView::Magnitude;
    // Surface batches must preserve physical vector/Piola evaluation and
    // coefficient ordering, including the polynomial basis reuse path.
    for (mfem::GridFunction* field : {&circulation, &flux, &velocity}) {
        const auto surface = RayFEM::BuildSurface(mesh, field, options);
        for (std::size_t i = 0; i < surface.values.size(); ++i) {
            const float* p = &surface.positions[3*i];
            const double z = dim == 3 && field != &circulation ? p[2] : 0;
            Near(surface.values[i], std::hypot(p[0], p[1], z), 3e-7, "Batched physical vector magnitude");
        }
        options.view = RayFEM::FieldView::Component;
        for (int component = 0; component < dim; ++component) {
            options.component = component;
            const auto component_surface = RayFEM::BuildSurface(mesh, field, options);
            for (std::size_t i = 0; i < component_surface.values.size(); ++i) {
                const float* p = &component_surface.positions[3*i];
                const double expected = field == &circulation ?
                    (component == 0 ? -p[1] : component == 1 ? p[0] : 0) : p[component];
                Near(component_surface.values[i], expected, 3e-7, "Batched physical vector component");
            }
        }
        options.view = RayFEM::FieldView::Magnitude;
    }
    for (mfem::GridFunction* field : {&circulation, &flux}) {
        const auto points = RayFEM::SampleDofs(mesh, *field, options);
        Check(points.size() == static_cast<std::size_t>(field->FESpace()->GetNDofs()), "All ND/RT unique DOF markers");
        for (const auto& point : points) { Check(point.directional && !point.interpolatory, "Directional DOF semantics"); }
        const auto vectors = RayFEM::SampleVectors(mesh, *field, options, 4);
        Check(!vectors.empty() && vectors.size() <= 4, "Bounded physical vector samples");
    }
    std::cout << "PASS " << dim << "D vector ND/RT/H1 type " << type << '\n';
}
void NURBSCase(const std::string& path, bool refine = false) {
    mfem::Mesh mesh(path.c_str(), 1, 1);
    Check(mesh.NURBSext != nullptr, "NURBS fixture");
    if (refine) { mesh.UniformRefinement(); }
    auto* nodes = mesh.GetNodes();
    mfem::FiniteElementSpace space(&mesh, nodes->FESpace()->FEColl());
    mfem::GridFunction field(&space);
    Check(space.GetNDofs() == nodes->FESpace()->GetNDofs(), "Matching NURBS field space");
    for (int i = 0; i < space.GetNDofs(); ++i) {
        field[i] = (*nodes)[nodes->FESpace()->DofToVDof(i, 0)]+2*(*nodes)[nodes->FESpace()->DofToVDof(i, 1)];
    }
    RayFEM::RenderOptions options;
    options.refinement = 5;
    const auto surface = RayFEM::BuildSurface(mesh, &field, options);
    Buffers(surface);
    for (std::size_t i = 0; i < surface.values.size(); ++i) {
        Near(surface.values[i], surface.positions[3*i]+2*surface.positions[3*i+1], 3e-6, "Rational NURBS mapping");
    }
    const auto points = RayFEM::SampleDofs(mesh, field);
    Check(points.size() == static_cast<std::size_t>(space.GetNDofs()), "All NURBS Greville markers");
    for (const auto& point : points) { Check(!point.interpolatory, "NURBS coefficient semantics"); }
    std::cout << "PASS NURBS " << path << " (" << points.size() << " DOFs)\n";
}

void MixedGeometryCase(const std::string& path) {
    mfem::Mesh mesh(path.c_str(), 1, 1);
    mfem::H1_FECollection collection(1, mesh.Dimension());
    mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& x) { return x[0]+2*x[1]; });
    field.ProjectCoefficient(coefficient);
    const auto surface = RayFEM::BuildSurface(mesh, &field);
    Buffers(surface);
    Check(surface.positions.size()/9 == ExpectedTriangles(mesh, 3), "Mixed-face triangle count");
    for (std::size_t i = 0; i < surface.values.size(); ++i) {
        Near(surface.values[i], surface.positions[3*i]+2*surface.positions[3*i+1], 1e-6, "Mixed-face parent interpolation");
    }
    if (path.find("inline-") != std::string::npos) { VerifyOutward(surface); }
    std::cout << "PASS mixed geometry " << path << '\n';
}

void NonconformingCase() {
    auto mesh = mfem::Mesh::MakeCartesian3D(2, 2, 2, mfem::Element::HEXAHEDRON);
    mesh.EnsureNCMesh();
    mfem::Array<int> refine(1); refine[0] = 0;
    mesh.GeneralRefinement(refine);
    const auto surface = RayFEM::BuildSurface(mesh);
    Buffers(surface);
    VerifyOutward(surface);
    for (std::size_t i = 0; i < surface.positions.size(); i += 9) {
        bool exterior = false;
        for (int d = 0; d < 3; ++d) {
            const double center = (surface.positions[i+d]+surface.positions[i+d+3]+surface.positions[i+d+6])/3;
            exterior = exterior || std::abs(center) < 1e-7 || std::abs(center-1) < 1e-7;
        }
        Check(exterior, "Nonconforming interfaces must not be rendered");
    }
    std::cout << "PASS nonconforming volume exterior\n";
}

void DiscontinuousCase() {
    auto mesh = mfem::Mesh::MakeCartesian2D(2, 2, mfem::Element::QUADRILATERAL, true);
    mfem::L2_FECollection collection(0, 2);
    mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    for (int i = 0; i < field.Size(); ++i) { field[i] = i-2; }
    const auto surface = RayFEM::BuildSurface(mesh, &field);
    Buffers(surface);
    const int points_per_element = 3*3*2*3;
    for (std::size_t i = 0; i < surface.values.size(); ++i) {
        Near(surface.values[i], static_cast<int>(i)/points_per_element-2, 0, "Discontinuous traces stay with their parent element");
    }
    Near(surface.min_value, -2, 0, "Negative scalar range");
    Near(surface.max_value, 1, 0, "Positive scalar range");
    Check(RayFEM::SampleDofs(mesh, field).size() == static_cast<std::size_t>(space.GetNDofs()), "L2 DOF markers");
    std::cout << "PASS discontinuous L2 traces\n";
}

void WholeElementOverlayCase() {
    auto mesh = mfem::Mesh::MakeCartesian2D(2, 1, mfem::Element::QUADRILATERAL, true);
    mesh.SetCurvature(3);
    mfem::H1_FECollection collection(3, 2);
    mfem::FiniteElementSpace space(&mesh, &collection, 2);
    mfem::GridFunction field(&space);
    mfem::VectorFunctionCoefficient coefficient(2, [](const mfem::Vector& x, mfem::Vector& y) {
        y.SetSize(2); y[0] = x[0]; y[1] = 1+x[1];
    });
    field.ProjectCoefficient(coefficient);
    const auto none = RayFEM::BuildClippedSurface(mesh, &field, {}, {{.4f,0,0},{1,0,0}}, RayFEM::ClipMode::WholeElements);
    Check(none.retained_elements.size() == 2, "Whole-element mask matches original mesh");
    Check(RayFEM::SampleDofs(mesh, field, {}, 20000, &none.retained_elements).empty(), "Discarded straddling-cell DOFs must not leak behind the plane");
    Check(RayFEM::SampleVectors(mesh, field, {}, 2000, &none.retained_elements).empty(), "Discarded straddling-cell vectors must not leak behind the plane");
    const auto right = RayFEM::BuildClippedSurface(mesh, &field, {}, {{.5f,0,0},{-1,0,0}}, RayFEM::ClipMode::WholeElements);
    const auto points = RayFEM::SampleDofs(mesh, field, {}, 20000, &right.retained_elements);
    mfem::Array<int> expected_dofs;
    int retained = -1;
    for (int e = 0; e < mesh.GetNE(); ++e) if (right.retained_elements[e]) { retained = e; }
    Check(retained >= 0, "Right-side cell retained");
    space.GetElementDofs(retained, expected_dofs);
    Check(points.size() == static_cast<std::size_t>(expected_dofs.Size()), "Shared DOFs remain visible when their first owner is discarded");
    bool shared_boundary = false;
    for (const auto& point : points) {
        Check(right.retained_elements[point.element], "Shared DOF physical trace uses retained parent");
        shared_boundary |= std::abs(point.position.x-.5) < 1e-6;
    }
    Check(shared_boundary, "Retained interface DOFs included");
    const auto vectors = RayFEM::SampleVectors(mesh, field, {}, 1, &right.retained_elements);
    Check(vectors.size() == 1 && right.retained_elements[vectors[0].element], "Sparse selection vector cap samples retained elements");
    auto cut = RayFEM::BuildClippedSurface(mesh, &field, {}, {{.4f,0,0},{1,0,0}});
    Check(cut.retained_elements.empty(), "Cut-through uses geometric clipping rather than element rejection");

    mfem::GridFunction displacement(mesh.GetNodes()->FESpace());
    displacement = 0.0;
    for (int i = 0; i < displacement.FESpace()->GetNDofs(); ++i) {
        displacement[displacement.FESpace()->DofToVDof(i, 0)] = .25;
    }
    RayFEM::RenderOptions shifted; shifted.displacement = &displacement;
    const auto moved = RayFEM::BuildClippedSurface(mesh, &field, shifted, {{.75f,0,0},{1,0,0}}, RayFEM::ClipMode::WholeElements);
    const auto moved_vectors = RayFEM::SampleVectors(mesh, field, shifted, 2000, &moved.retained_elements);
    Check(moved_vectors.size() == 1, "Overlay selection uses displaced corners");
    Near(moved_vectors[0].position.x, .5, 1e-7, "Selected vector uses displaced position");

    auto curved = mfem::Mesh::MakeCartesian2D(1, 1, mfem::Element::QUADRILATERAL, true);
    curved.SetCurvature(3);
    mfem::VectorFunctionCoefficient bend(2, [](const mfem::Vector& x, mfem::Vector& y) {
        y = x; y[0] += .8*x[0]*x[1]*(1-x[1]);
    });
    curved.Transform(bend);
    mfem::FiniteElementSpace curved_space(&curved, &collection);
    mfem::GridFunction scalar(&curved_space); scalar = 1.0;
    const auto curved_keep = RayFEM::BuildClippedSurface(curved, &scalar, {}, {{1.1f,0,0},{1,0,0}}, RayFEM::ClipMode::WholeElements);
    const auto curved_points = RayFEM::SampleDofs(curved, scalar, {}, 20000, &curved_keep.retained_elements);
    Check(curved_points.size() == static_cast<std::size_t>(curved_space.GetNDofs()), "Kept curved cells preserve every DOF");
    Check(std::any_of(curved_points.begin(), curved_points.end(), [](const auto& p) { return p.position.x > 1.1; }),
          "Whole-element overlays may extend beyond plane when curved corners are retained");
    const std::vector<bool> invalid_mask(3, true);
    bool rejected = false;
    try { RayFEM::SampleDofs(mesh, field, {}, 20000, &invalid_mask); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "Wrong-sized overlay masks reject safely");
    std::cout << "PASS whole-element overlay membership, shared DOFs, displacement, curved bulges\n";
}

void DifferentialNormals(const std::string& data) {
    auto planar=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    const auto cube=RayFEM::BuildSurface(planar);
    Check(RayFEM::RecommendedSurfaceRefinement(planar)==3,"Linear geometry keeps modest automatic detail");
    for(std::size_t i=0;i<cube.normals.size();i+=3) {
        const double sum=std::abs(cube.normals[i])+std::abs(cube.normals[i+1])+std::abs(cube.normals[i+2]);
        Near(sum,1,1e-7,"Planar cube faces retain hard axis-aligned normals at shared corners");
    }
    auto graph=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    graph.SetCurvature(5,false,3);
    Check(RayFEM::RecommendedSurfaceRefinement(graph)==10,"P5 geometry gets ten subdivisions automatically");
    mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& p,mfem::Vector& q) {
        q=p;q[2]=.2*p[0]*p[0]+.3*p[1]*p[1];
    });
    graph.Transform(bend);
    auto check_graph=[](const RayFEM::SurfaceData& surface) {
        for(std::size_t i=0;i<surface.positions.size();i+=3) {
            const double x=-.4*surface.positions[i],y=-.6*surface.positions[i+1],length=std::hypot(x,y,1.);
            Near(surface.normals[i],x/length,2e-6,"Curved graph analytical x normal");
            Near(surface.normals[i+1],y/length,2e-6,"Curved graph analytical y normal");
            Near(surface.normals[i+2],1/length,2e-6,"Curved graph analytical z normal");
        }
    };
    check_graph(RayFEM::BuildSurface(graph));
    auto flat=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    flat.SetCurvature(1,false,3);
    mfem::H1_FECollection collection(6,2);
    mfem::FiniteElementSpace space(&flat,&collection,3,mfem::Ordering::byVDIM);
    mfem::GridFunction displacement(&space);
    mfem::VectorFunctionCoefficient offset(3,[](const mfem::Vector& p,mfem::Vector& q) {
        q.SetSize(3);q=0.;q[2]=.2*p[0]*p[0]+.3*p[1]*p[1];
    });
    displacement.ProjectCoefficient(offset);
    RayFEM::RenderOptions options;options.displacement=&displacement;
    Check(RayFEM::RecommendedSurfaceRefinement(flat,&displacement)==12,"Automatic detail includes higher-order displacement");
    check_graph(RayFEM::BuildSurface(flat,nullptr,options));

    mfem::Mesh sphere((data+"/ball-nurbs.mesh").c_str(),1,1);
    auto& nodes=*sphere.GetNodes();
    const mfem::Vector original(nodes);
    mfem::GridFunction stretch(nodes.FESpace());
    const double axes[]={1.8,.7,1.2};
    for(int i=0;i<nodes.FESpace()->GetNDofs();++i)for(int d=0;d<3;++d) {
        const int v=nodes.FESpace()->DofToVDof(i,d);
        stretch[v]=(axes[d]-1)*nodes[v];
    }
    options.refinement=8;options.displacement=nullptr;
    auto check_sphere=[&](const RayFEM::SurfaceData& surface,bool deformed) {
        Buffers(surface);
        for(std::size_t i=0;i<surface.positions.size();i+=3) {
            double expected[3];for(int d=0;d<3;++d)expected[d]=surface.positions[i+d]/(deformed?axes[d]*axes[d]:1.);
            const double length=std::hypot(expected[0],expected[1],expected[2]);
            for(int d=0;d<3;++d)Near(surface.normals[i+d],expected[d]/length,3e-6,"Rational sphere/ellipsoid analytical normal");
        }
    };
    check_sphere(RayFEM::BuildSurface(sphere,nullptr,options),false);
    options.displacement=&stretch;
    check_sphere(RayFEM::BuildSurface(sphere,nullptr,options),true);
    auto clipped=RayFEM::BuildClippedSurface(sphere,nullptr,options,{{.17f,0,0},{1,0,0}},RayFEM::ClipMode::CutThrough);
    std::size_t cap_points=0,curved_points=0;
    for(std::size_t i=0;i<clipped.surface.values.size();++i) {
        if(clipped.surface.boundary_ids[i]==0) {
            ++cap_points;
            Near(clipped.surface.normals[3*i],1,2e-6,"Section caps retain flat outward normals");
            Near(clipped.surface.normals[3*i+1],0,2e-6,"Section cap y normal");
            Near(clipped.surface.normals[3*i+2],0,2e-6,"Section cap z normal");
        } else if(std::abs(clipped.surface.positions[3*i]-.17)>1e-5) {
            ++curved_points;
            double expected[3];for(int d=0;d<3;++d)expected[d]=clipped.surface.positions[3*i+d]/(axes[d]*axes[d]);
            const double length=std::hypot(expected[0],expected[1],expected[2]);
            for(int d=0;d<3;++d)Near(clipped.surface.normals[3*i+d],expected[d]/length,3e-6,"Clipped exterior preserves curved normals");
        }
    }
    Check(cap_points&&curved_points,"Clipped normal test exercises cap and curved exterior");
    for(int i=0;i<nodes.Size();++i)Near(nodes[i],original[i],0,"Normals leave original NURBS geometry unchanged");
    std::cout<<"PASS differential normals: planar hard edges, P5 graph, P6 displacement, rational sphere/ellipsoid and clipped caps\n";
}

void StroidCurvatureCase(const std::string& path) {
    mfem::Mesh mesh(path.c_str(),1,1);
    Check(mesh.GetNodes()&&mesh.GetNodes()->FESpace()->GetMaxElementOrder()==5,"Actual Stroid P5 fixture");
    RayFEM::RenderOptions options;options.excluded_materials={23};
    auto error=[](const RayFEM::SurfaceData& surface) {
        double maximum=0;
        for(std::size_t i=0;i<surface.positions.size();i+=9) {
            double p[3]{};
            for(int v=0;v<3;++v)for(int d=0;d<3;++d)p[d]+=surface.positions[i+3*v+d]/3.;
            maximum=std::max(maximum,std::abs(std::hypot(p[0],p[1],p[2]/.75)-1));
        }
        return maximum;
    };
    options.refinement=3;const auto coarse=RayFEM::BuildSurface(mesh,nullptr,options);
    options.refinement=RayFEM::RecommendedSurfaceRefinement(mesh);
    const auto automatic=RayFEM::BuildSurface(mesh,nullptr,options);
    Check(error(automatic)<error(coarse)/5,"Automatic detail resolves Stroid P5 face curvature");
    Check(error(automatic)<.005,"Automatic Stroid facet error is below half a percent of radius");
    const auto& bounds=automatic.bounds;
    Near(bounds.min.x,-1,.002,"Stroid refined minimum x");Near(bounds.max.x,1,.002,"Stroid refined maximum x");
    Near(bounds.min.z,-.75,.002,"Stroid refined minimum z");Near(bounds.max.z,.75,.002,"Stroid refined maximum z");
    std::cout<<"PASS actual Stroid P5 automatic detail: "<<coarse.positions.size()/9<<" -> "<<automatic.positions.size()/9
             <<" triangles; maximum radial facet error "<<error(coarse)<<" -> "<<error(automatic)<<'\n';
}
}

int main(int argc, char** argv) {
    try {
        for (bool curved : {false, true}) {
            SurfaceCase(mfem::Element::TRIANGLE, 2, curved);
            SurfaceCase(mfem::Element::QUADRILATERAL, 2, curved);
            SurfaceCase(mfem::Element::TETRAHEDRON, 3, curved);
            SurfaceCase(mfem::Element::HEXAHEDRON, 3, curved);
        }
        DisplacementCase();
        VectorCase(2, mfem::Element::TRIANGLE);
        VectorCase(2, mfem::Element::QUADRILATERAL);
        VectorCase(3, mfem::Element::TETRAHEDRON);
        VectorCase(3, mfem::Element::HEXAHEDRON);
        VectorCase(3, mfem::Element::HEXAHEDRON, mfem::Ordering::byVDIM);
        NonconformingCase();
        DiscontinuousCase();
        WholeElementOverlayCase();
        const std::string data = argc > 1 ? argv[1] : "subprojects/mfem/data";
        DifferentialNormals(data);
        if(argc>2)StroidCurvatureCase(argv[2]);
        for (const auto* filename : {"beam-quad-nurbs.mesh", "disc-nurbs.mesh", "beam-hex-nurbs.mesh", "ball-nurbs.mesh"}) {
            NURBSCase(data+"/"+filename);
        }
        NURBSCase(data+"/beam-quad-nurbs.mesh", true);
        for (const auto* filename : {"inline-wedge.mesh", "inline-pyramid.mesh", "fichera-mixed.mesh"}) {
            MixedGeometryCase(data+"/"+filename);
        }
        std::cout << "All geometry tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
