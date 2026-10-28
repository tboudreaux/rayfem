#include "libRayfem/mesh/element_statistics.hpp"

#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace {
using namespace RayFEM;
void Check(bool condition, const std::string& message) { if (!condition) throw std::runtime_error(message); }
void Near(double actual, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > tolerance) {
        std::ostringstream out; out << message << ": " << actual << " != " << expected;
        throw std::runtime_error(out.str());
    }
}
template<class Function> void Reject(Function function, const std::string& fragment) {
    try { function(); } catch (const std::exception& e) {
        Check(std::string(e.what()).find(fragment) != std::string::npos, "Unexpected rejection: " + std::string(e.what()));
        return;
    }
    throw std::runtime_error("Expected rejection: " + fragment);
}
std::vector<int> All(mfem::Mesh& mesh) {
    std::vector<int> result(mesh.GetNE()); std::iota(result.begin(), result.end(), 0); return result;
}

void Selection() {
    auto mesh = mfem::Mesh::MakeCartesian3D(2, 1, 1, mfem::Element::HEXAHEDRON);
    mesh.GetElement(0)->SetAttribute(7); mesh.GetElement(1)->SetAttribute(9); mesh.SetAttributes();
    mfem::H1_FECollection collection(2, 3); mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient linear([](const mfem::Vector& x) { return x[0]+2*x[1]+3*x[2]; });
    field.ProjectCoefficient(linear);
    const auto left = ComputeElementStatistics(mesh, {0, 0}, &field);
    Check(left.requested_count == 1 && left.element_count == 1 && left.material_counts.at(7) == 1, "Unique included selection and materials");
    Check(left.geometry_counts.at(mfem::Geometry::CUBE) == 1 && left.geometry_orders.at(1) == 1 && left.field_orders.at(2) == 1, "Geometry and polynomial orders");
    Near(left.measure, .5, 1e-12, "Selected volume");
    Near(left.centroid[0], .25, 1e-12, "Selected centroid x"); Near(left.centroid[1], .5, 1e-12, "Selected centroid y");
    Near(left.bounds_max[0], .5, 1e-12, "Selected bound");
    Check(left.field.has_value(), "Field statistics present");
    Near(left.field->minimum, 0, 1e-12, "Endpoint minimum"); Near(left.field->maximum, 5.5, 1e-12, "Endpoint maximum");
    Near(left.field->mean, 2.75, 1e-12, "Physical selected mean"); Near(left.field->integral, 1.375, 1e-12, "Physical selected integral");
    RenderOptions options; options.excluded_materials = {9};
    auto selected = ComputeElementStatistics(mesh, {1, 0, 1}, &field, options);
    Check(selected.requested_count == 2 && selected.excluded_count == 1 && selected.elements == std::vector<int>{0}, "Material exclusion applied before integration");
    Near(selected.field->integral, left.field->integral, 1e-12, "Hidden material contributes nothing");
    options.excluded_materials.clear(); options.excluded_elements = {0};
    selected = ComputeElementStatistics(mesh, {1, 0}, &field, options);
    Check(selected.elements == std::vector<int>{1} && selected.material_counts.at(9) == 1, "Individual element exclusion");
    Near(selected.field->mean, 3.25, 1e-12, "Remaining element mean");
    options.excluded_materials = {9};
    selected = ComputeElementStatistics(mesh, {0, 1}, &field, options);
    Check(selected.element_count == 0 && selected.measure == 0 && !selected.field, "Fully excluded selection is empty");
    Check(ComputeElementStatistics(mesh, {}).element_count == 0, "Empty selection");
    const auto geometry_only = ComputeElementStatistics(mesh, {0});
    Check(!geometry_only.field && geometry_only.field_orders.empty(), "Geometry-only mode");
    Reject([&] { ComputeElementStatistics(mesh, {-1}); }, "out of range");
    Reject([&] { ComputeElementStatistics(mesh, {2}); }, "out of range");
    ElementStatisticsOptions tiny; tiny.max_samples = 1;
    Reject([&] { ComputeElementStatistics(mesh, {0}, &field, {}, tiny); }, "budget");
    std::cout << "PASS selection, exclusions, duplicate IDs, geometry metadata and weighted field values\n";
}

void CurvedAndDisplaced() {
    auto mesh = mfem::Mesh::MakeCartesian3D(1, 1, 1, mfem::Element::HEXAHEDRON);
    mesh.SetCurvature(3);
    mfem::VectorFunctionCoefficient bend(3, [](const mfem::Vector& x, mfem::Vector& y) { y=x; y[0]=x[0]*(1+.4*x[1]); });
    mesh.Transform(bend);
    mfem::H1_FECollection collection(3, 3); mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient x_coordinate([](const mfem::Vector& x) { return x[0]; }); field.ProjectCoefficient(x_coordinate);
    const double integral = .5*(1+.4+.16/3);
    auto original = ComputeElementStatistics(mesh, {0}, &field);
    Near(original.measure, 1.2, 1e-11, "Curved volume"); Near(original.field->integral, integral, 1e-11, "Curved integration Jacobian");
    Near(original.field->mean, integral/1.2, 1e-11, "Curved physical mean");
    Near(original.centroid[0], integral/1.2, 1e-11, "Curved weighted centroid");
    Near(original.bounds_max[0], 1.4, 1e-11, "Curved bound includes boundary extrema");
    const mfem::Vector before(*mesh.GetNodes());
    mfem::FiniteElementSpace displacement_space(&mesh, mesh.GetNodes()->FESpace()->FEColl(), 3, mfem::Ordering::byVDIM);
    mfem::GridFunction displacement(&displacement_space);
    mfem::VectorFunctionCoefficient move(3, [](const mfem::Vector& x, mfem::Vector& d) { d=x; d[1]*=-.5; d[2]*=.5; });
    displacement.ProjectCoefficient(move);
    RenderOptions options; options.displacement=&displacement; options.displacement_scale=.5; options.element_explosion=.8;
    const auto shifted = ComputeElementStatistics(mesh, {0}, &field, options);
    const double dilation = 1.5*.75*1.25;
    Near(shifted.measure, original.measure*dilation, 1e-11, "Displacement-scaled physical volume");
    Near(shifted.field->mean, original.field->mean, 1e-11, "Field remains attached to parent coordinates");
    Near(shifted.field->integral, original.field->integral*dilation, 1e-11, "Displaced integration measure");
    Near(shifted.centroid[0], original.centroid[0]*1.5, 1e-11, "Displaced physical centroid");
    Near(shifted.bounds_max[0], 2.1, 1e-11, "Explosion cannot shrink physical statistics bounds");
    options.element_explosion=0;
    const auto joined=ComputeElementStatistics(mesh, {0}, &field, options);
    Near(joined.measure, shifted.measure, 0, "Explosion leaves physical measure exactly unchanged");
    for (int i=0; i<before.Size(); ++i) Near((*mesh.GetNodes())[i], before[i], 0, "Source coordinates unchanged");
    std::cout << "PASS high-order curvilinear volume, centroid, displacement scale and display-only explosion\n";
}

void EmbeddedAndMixed() {
    auto embedded=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    embedded.SetCurvature(2,false,3);
    mfem::VectorFunctionCoefficient tilt(3, [](const mfem::Vector& x, mfem::Vector& y) { y=x; y[2]=std::sqrt(3.)*x[0]; });
    embedded.Transform(tilt);
    mfem::H1_FECollection collection(2,2); mfem::FiniteElementSpace space(&embedded,&collection);
    mfem::GridFunction constant(&space); constant=3.;
    const auto surface=ComputeElementStatistics(embedded,{0},&constant);
    Near(surface.measure,2,1e-11,"Embedded surface physical area"); Near(surface.field->integral,6,1e-11,"Embedded surface integral");
    Near(surface.centroid[2],std::sqrt(3.)/2,1e-11,"Embedded centroid");
    Check(surface.dimension==2 && surface.space_dimension==3,"Embedded dimension metadata");
    for (auto type : {mfem::Element::TRIANGLE,mfem::Element::TETRAHEDRON,mfem::Element::WEDGE}) {
        const int dimension=type==mfem::Element::TRIANGLE?2:3;
        auto mesh=dimension==2?mfem::Mesh::MakeCartesian2D(1,1,type,true):mfem::Mesh::MakeCartesian3D(1,1,1,type);
        const auto stats=ComputeElementStatistics(mesh,All(mesh));
        Near(stats.measure,1,1e-11,"Simplex/prism physical measure");
        Near(stats.centroid[0],.5,1e-11,"Simplex/prism physical centroid");
    }
    std::cout << "PASS embedded surface area and simplex/prism measures\n";
}

void VectorViews() {
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    mfem::VectorFunctionCoefficient spin(3, [](const mfem::Vector& x,mfem::Vector& v) { v.SetSize(3);v[0]=-x[1];v[1]=x[0];v[2]=0; });
    mfem::H1_FECollection h1(2,3);mfem::FiniteElementSpace h1_space(&mesh,&h1,3);mfem::GridFunction nodal(&h1_space);nodal.ProjectCoefficient(spin);
    mfem::ND_FECollection nd(2,3);mfem::FiniteElementSpace nd_space(&mesh,&nd);mfem::GridFunction circulation(&nd_space);circulation.ProjectCoefficient(spin);
    RenderOptions options;options.view=FieldView::CurlMagnitude;
    for (auto* field : {&nodal,&circulation}) {
        const auto stats=ComputeElementStatistics(mesh,{0},field,options);
        Near(stats.field->minimum,2,1e-10,"Nodal/ND curl minimum");Near(stats.field->maximum,2,1e-10,"Nodal/ND curl maximum");
        Near(stats.field->mean,2,1e-10,"Nodal/ND curl physical mean");Near(stats.field->integral,2,1e-10,"Nodal/ND curl integral");
    }
    options.view=FieldView::Component;options.component=0;
    const auto component=ComputeElementStatistics(mesh,{0},&circulation,options);
    Near(component.field->minimum,-1,1e-11,"ND physical component minimum");Near(component.field->mean,-.5,1e-11,"ND physical component mean");
    options.view=FieldView::Divergence;
    Reject([&] { ComputeElementStatistics(mesh,{0},&circulation,options); }, "Divergence");
    mfem::RT_FECollection rt(1,3);mfem::FiniteElementSpace rt_space(&mesh,&rt);mfem::GridFunction flux(&rt_space);
    mfem::VectorFunctionCoefficient radial(3, [](const mfem::Vector& x,mfem::Vector& v) { v=x; });flux.ProjectCoefficient(radial);
    const auto divergence=ComputeElementStatistics(mesh,{0},&flux,options);
    Near(divergence.field->mean,3,1e-10,"RT divergence physical mean");Near(divergence.field->integral,3,1e-10,"RT divergence integral");
    std::cout << "PASS physical nodal/ND/RT components, curl and divergence\n";
}

void Nurbs(const std::string& data) {
    mfem::Mesh mesh((data+"/ball-nurbs.mesh").c_str(),1,1);
    auto* nodes=mesh.GetNodes();
    mfem::FiniteElementSpace space(&mesh,nodes->FESpace()->FEColl());mfem::GridFunction field(&space);field=3.;
    ElementStatisticsOptions sampling;sampling.quadrature_order=40;
    const auto original=ComputeElementStatistics(mesh,All(mesh),&field,{},sampling);
    constexpr double pi=3.14159265358979323846;
    Check(original.nurbs,"Rational geometry metadata");Near(original.measure,4*pi/3,2e-8,"Exact NURBS sphere volume");
    Near(original.field->integral,4*pi,6e-8,"NURBS scalar integral");Near(original.centroid[0],0,1e-11,"NURBS sphere centroid");
    Near(original.bounds_min[0],-1,1e-12,"NURBS sphere bound");Near(original.bounds_max[2],1,1e-12,"NURBS sphere upper bound");
    mfem::FiniteElementSpace displacement_space(&mesh,nodes->FESpace()->FEColl(),3,mfem::Ordering::byVDIM);
    mfem::GridFunction displacement(&displacement_space);
    const auto* geometry=nodes->FESpace();
    for(int i=0;i<geometry->GetNDofs();++i)for(int d=0;d<3;++d)
        displacement[displacement_space.DofToVDof(i,d)]=(d==0?1.:d==1?-.5:.5)*(*nodes)[geometry->DofToVDof(i,d)];
    RenderOptions options;options.displacement=&displacement;
    const auto ellipsoid=ComputeElementStatistics(mesh,All(mesh),&field,options,sampling);
    Near(ellipsoid.measure,original.measure*2*.5*1.5,2e-10,"NURBS ellipsoid physical volume");
    Near(ellipsoid.bounds_max[0],2,1e-12,"NURBS ellipsoid x bound");Near(ellipsoid.bounds_max[1],.5,1e-12,"NURBS ellipsoid y bound");
    std::cout << "PASS exact NURBS sphere and displaced ellipsoid physical statistics\n";
}
} // namespace

int main(int argc,char** argv) {
    try {
        mfem::set_error_action(mfem::MFEM_ERROR_THROW);
        Selection();CurvedAndDisplaced();EmbeddedAndMixed();VectorViews();
        if(argc>1)Nurbs(argv[1]);
        return 0;
    } catch(const std::exception& error) { std::cerr<<"Element statistics: "<<error.what()<<'\n';return 1; }
}
