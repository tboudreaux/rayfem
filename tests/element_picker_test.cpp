#include "libRayfem/mesh/clipping.hpp"
#include "../src/app/element_picker.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) { throw std::runtime_error(message); }
}
void Near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual-expected)>tolerance) { throw std::runtime_error(message); }
}
RayFEM::UI::ElementPicker Index(const RayFEM::SurfaceData& surface) {
    RayFEM::UI::ElementPicker picker;
    picker.Build(surface.positions,surface.element_ids);
    return picker;
}
void Owners(const RayFEM::SurfaceData& surface, mfem::Mesh& mesh) {
    Check(surface.element_ids.size()*9==surface.positions.size(),"One element owner per emitted triangle");
    for (std::size_t t=0;t<surface.element_ids.size();++t) {
        const int element=surface.element_ids[t];
        Check(element>=0 && element<mesh.GetNE(),"Valid physical element identity");
        for (int v=0;v<3;++v) {
            Check(surface.material_ids[3*t+v]==mesh.GetAttribute(element),"Triangle owner matches its material");
        }
    }
}
void NearestAndInvalid() {
    // Far triangle precedes the near one, and the input direction is not unit.
    const std::vector<float> positions{0,0,0,1,0,0,0,1,0, 0,0,1,1,0,1,0,1,1};
    const std::vector<int> owners{17,3};
    RayFEM::UI::ElementPicker picker;
    picker.Build(positions,owners);
    auto hit=picker.Pick({{.2f,.2f,3},{0,0,-7}});
    Check(hit && hit->element==3,"Nearest visible owner wins independently of input order");
    Near(hit->distance,2,1e-12,"Distance uses physical units for nonunit ray");
    Near(hit->position.z,1,1e-12,"Nearest hit location");
    Check(picker.TrianglesForElement(17).size()==1 && picker.TrianglesForElement(17)[0]==0,"Grouped owner triangle lookup");
    Check(picker.TrianglesForElement(99).empty(),"Unknown owner lookup is empty");
    Check(!picker.Pick({{.2f,.2f,3},{0,0,1}}),"Triangles behind camera are not selectable");
    {
        // The surface worker frees its buffers after building the index.
        std::vector<float> source=positions;
        RayFEM::UI::ElementPicker owned;owned.Build(source,owners);
        std::fill(source.begin(),source.end(),99.f);std::vector<float>().swap(source);
        const auto kept=owned.Pick({{.2f,.2f,3},{0,0,-7}});
        Check(kept && kept->element==3,"Picking survives the source buffer being freed");
    }
    Check(!picker.Pick({{2,2,3},{0,0,-1}}),"Empty viewport ray misses");
    Check(!picker.Pick({{.2f,.2f,3},{0,0,0}}),"Zero direction misses safely");
    Check(!picker.Pick({{NAN,0,0},{0,0,-1}}),"Nonfinite origin misses safely");
    Check(!picker.Pick({{0,0,0},{INFINITY,0,0}}),"Nonfinite direction misses safely");
    Check(picker.Pick({{.5f,.5f,3},{0,0,-1}}).has_value(),"Triangle-edge ray is selectable");
    Check(picker.Pick({{0,0,3},{0,0,-1}}).has_value(),"Triangle-vertex ray is selectable");
    Check(picker.Pick({{.2f,.2f,-1},{0,0,1}})->element==17,"Two-sided picking agrees with viewport rendering");
    for (auto bad : {std::vector<int>{1},std::vector<int>{1,-1}}) {
        bool rejected=false;
        try { picker.Build(positions,bad); } catch (const std::invalid_argument&) { rejected=true; }
        Check(rejected,"Invalid triangle ownership rejected");
    }
    picker.Build({},{});
    Check(picker.Empty() && !picker.Pick({{0,0,1},{0,0,-1}}),"Empty surface has no stale picks");
}
void ExplodedAndClipped() {
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.GetElement(0)->SetAttribute(7); mesh.GetElement(1)->SetAttribute(12); mesh.SetAttributes();
    RayFEM::RenderOptions options; options.refinement=4; options.element_explosion=.2;
    auto surface=RayFEM::BuildSurface(mesh,nullptr,options); Owners(surface,mesh);
    auto picker=Index(surface);
    Check(picker.Pick({{.5f,.5f,3},{0,0,-1}})->element==0,"Exploded first element owner");
    Check(picker.Pick({{1.5f,.5f,3},{0,0,-1}})->element==1,"Exploded second element owner");
    Check(!picker.Pick({{1,.5f,3},{0,0,-1}}),"Exploded gaps do not select invisible original faces");
    auto interior=picker.Pick({{1,.5f,.5f},{1,0,0}});
    Check(interior && interior->element==1,"Exposed interior trace has correct parent owner");
    Near(interior->position.x,1.1,1e-6,"Interior trace uses exploded position");
    for (auto algorithm : {RayFEM::CutAlgorithm::Refined,RayFEM::CutAlgorithm::Planar}) {
        auto cut=RayFEM::BuildClippedSurface(mesh,nullptr,options,{{0,0,.5f},{0,0,1}},RayFEM::ClipMode::CutThrough,algorithm);
        Owners(cut.surface,mesh); auto cut_picker=Index(cut.surface);
        for (int e=0;e<2;++e) {
            auto cap=cut_picker.Pick({{float(e)+.5f,.5f,3},{0,0,-1}});
            Check(cap && cap->element==e,"Clip cap retains owning element");
            Near(cap->position.z,.5,1e-6,"Nearest visible hit is clipping cap");
        }
    }
    auto whole=RayFEM::BuildClippedSurface(mesh,nullptr,options,{{1,0,0},{1,0,0}},RayFEM::ClipMode::WholeElements);
    Owners(whole.surface,mesh); auto whole_picker=Index(whole.surface);
    Check(whole_picker.Pick({{.5f,.5f,3},{0,0,-1}})->element==0,"Retained whole cell selectable");
    Check(!whole_picker.Pick({{1.5f,.5f,3},{0,0,-1}}),"Whole-cell clipping removes picks");
    options.excluded_elements={0};
    auto hidden=RayFEM::BuildSurface(mesh,nullptr,options); Owners(hidden,mesh); auto hidden_picker=Index(hidden);
    Check(!hidden_picker.Pick({{.5f,.5f,3},{0,0,-1}}),"Individually hidden element cannot be picked");
    Check(hidden_picker.Pick({{1.5f,.5f,3},{0,0,-1}})->element==1,"Hiding keeps original IDs");
    options.excluded_elements.clear(); options.excluded_materials={12};
    auto material_hidden=RayFEM::BuildSurface(mesh,nullptr,options); auto material_picker=Index(material_hidden);
    Check(!material_picker.Pick({{1.5f,.5f,3},{0,0,-1}}),"Excluded material cannot be picked");
}
void CurvedDisplaced() {
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    mesh.SetCurvature(4);
    mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& x,mfem::Vector& y) {
        y=x; y[2]+=.4*x[0]*(1-x[0])*x[1];
    });
    mesh.Transform(bend);
    RayFEM::RenderOptions options; options.refinement=8;
    auto surface=RayFEM::BuildSurface(mesh,nullptr,options); Owners(surface,mesh); auto picker=Index(surface);
    auto curved=picker.Pick({{.5f,.5f,3},{0,0,-1}});
    Check(curved && curved->element==0,"Curved high-order surface selectable");
    Near(curved->position.z,1.05,2e-6,"Picking follows curved tessellation instead of flat corners");
    mfem::H1_FECollection fec(1,3); mfem::FiniteElementSpace space(&mesh,&fec,3); mfem::GridFunction displacement(&space);
    mfem::VectorFunctionCoefficient shift(3,[](const mfem::Vector&,mfem::Vector& y) {
        y.SetSize(3);y[0]=.2;y[1]=-.1;y[2]=.3;
    });
    displacement.ProjectCoefficient(shift); options.displacement=&displacement; options.element_explosion=.2;
    auto moved=RayFEM::BuildSurface(mesh,nullptr,options); auto moved_picker=Index(moved);
    auto hit=moved_picker.Pick({{.7f,.4f,3},{0,0,-1}});
    Check(hit && hit->element==0,"Curved displaced exploded surface selectable");
    Near(hit->position.z,1.25,3e-6,"Picking follows displaced and exploded geometry");
}
void PickTriangleOwners(const RayFEM::SurfaceData& surface, mfem::Mesh& mesh) {
    Owners(surface,mesh); auto picker=Index(surface);
    // Probe just outside actual displayed triangles to exercise ownership
    // through NC parent maps, cap interpolation and rational geometry.
    for (std::size_t t=0;t<surface.element_ids.size();t+=13) {
        const float* p=surface.positions.data()+9*t;
        double a[3],b[3],n[3]; Vector3 center{};
        for (int d=0;d<3;++d) { a[d]=double(p[3+d])-p[d]; b[d]=double(p[6+d])-p[d]; }
        n[0]=a[1]*b[2]-a[2]*b[1];n[1]=a[2]*b[0]-a[0]*b[2];n[2]=a[0]*b[1]-a[1]*b[0];
        const double length=std::hypot(n[0],n[1],n[2]);
        if (length<1e-15) { continue; }
        center={float((double(p[0])+p[3]+p[6])/3),float((double(p[1])+p[4]+p[7])/3),float((double(p[2])+p[5]+p[8])/3)};
        Vector3 normal{float(n[0]/length),float(n[1]/length),float(n[2]/length)};
        auto hit=picker.Pick({{center.x+1e-4f*normal.x,center.y+1e-4f*normal.y,center.z+1e-4f*normal.z},
                              {-normal.x,-normal.y,-normal.z}});
        Check(hit && hit->element==surface.element_ids[t],"Displayed triangle picks its NC/NURBS physical parent");
    }
}
void NonconformingAndNurbs(const std::string& data) {
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.EnsureNCMesh(); mfem::Array<int> refine(1);refine[0]=0;mesh.GeneralRefinement(refine);
    for (int e=0;e<mesh.GetNE();++e) { mesh.GetElement(e)->SetAttribute(e+10); } mesh.SetAttributes();
    RayFEM::RenderOptions options;options.element_explosion=.2;options.refinement=3;
    auto nc=RayFEM::BuildSurface(mesh,nullptr,options); PickTriangleOwners(nc,mesh);
    auto cut=RayFEM::BuildClippedSurface(mesh,nullptr,options,{{0,0,.37f},{0,0,1}});
    PickTriangleOwners(cut.surface,mesh);
    mfem::Mesh rational((data+"/ball-nurbs.mesh").c_str(),1,1);
    Check(rational.NURBSext,"Rational fixture loaded");
    for (int e=0;e<rational.GetNE();++e) { rational.GetElement(e)->SetAttribute(e+100); } rational.SetAttributes();
    auto nurbs=RayFEM::BuildSurface(rational,nullptr,options);PickTriangleOwners(nurbs,rational);
}
void Hierarchy() {
    auto mesh=mfem::Mesh::MakeCartesian2D(40,40,mfem::Element::QUADRILATERAL,true);
    RayFEM::RenderOptions options;options.refinement=6;
    auto surface=RayFEM::BuildSurface(mesh,nullptr,options);auto picker=Index(surface);
    RayFEM::UI::ElementPickStats stats;
    auto hit=picker.Pick({{.5123f,.6123f,3},{0,0,-1}},&stats);
    Check(hit.has_value(),"Dense surface hit");
    Check(stats.triangles_tested<surface.element_ids.size()/100,"Hierarchy avoids scanning dense surface triangles");
    std::cout<<"Dense pick: "<<stats.triangles_tested<<" of "<<surface.element_ids.size()<<" triangles tested\n";
}
}
int main(int argc,char** argv) {
    try {
        NearestAndInvalid();ExplodedAndClipped();CurvedDisplaced();
        NonconformingAndNurbs(argc>1?argv[1]:"subprojects/mfem/data");Hierarchy();
        std::cout<<"PASS element picking: nearest hits, curvature, displacement, explosion, clip caps, NC, NURBS and hierarchy\n";
    } catch (const std::exception& error) { std::cerr<<"Element picking test failed: "<<error.what()<<'\n';return 1; }
}
