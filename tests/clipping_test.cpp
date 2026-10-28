#include "libRayfem/mesh/clipping.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool okay,const std::string& what){if(!okay)throw std::runtime_error(what);}
void near(double a,double b,double eps,const std::string& what){if(!std::isfinite(a)||std::abs(a-b)>eps)throw std::runtime_error(what+": "+std::to_string(a)+" vs "+std::to_string(b));}
double area(const RayFEM::SurfaceData& s,bool cap=false,double x=0) {
    double out=0;
    for(std::size_t i=0;i<s.positions.size();i+=9){
        if(cap&&(std::abs(s.positions[i]-x)>2e-6||std::abs(s.positions[i+3]-x)>2e-6||std::abs(s.positions[i+6]-x)>2e-6))continue;
        double a[3],b[3];for(int k=0;k<3;++k){a[k]=s.positions[i+3+k]-s.positions[i+k];b[k]=s.positions[i+6+k]-s.positions[i+k];}
        double cx=a[1]*b[2]-a[2]*b[1],cy=a[2]*b[0]-a[0]*b[2],cz=a[0]*b[1]-a[1]*b[0];out+=.5*std::sqrt(cx*cx+cy*cy+cz*cz);
    }return out;
}
void basics(mfem::Element::Type type){
    auto mesh=mfem::Mesh::MakeCartesian3D(2,2,2,type);mesh.SetCurvature(3);
    mfem::H1_FECollection h1(3,3);mfem::FiniteElementSpace space(&mesh,&h1);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x){return x[0]+2*x[1]+3*x[2];});field.ProjectCoefficient(ramp);
    RayFEM::RenderOptions o;o.refinement=4;
    for(auto algorithm:{RayFEM::CutAlgorithm::Refined,RayFEM::CutAlgorithm::Planar})for(float position:{.4f,.5f}){
        auto result=RayFEM::BuildClippedSurface(mesh,&field,o,{{position,0,0},{1,0,0}},RayFEM::ClipMode::CutThrough,algorithm);
        near(area(result.surface),2+4*position,2e-5,"Clipped closed surface area");near(area(result.surface,true,position),1,2e-5,"True section cap area");
        check(!result.element_boundaries.empty(),"Clipped element outlines");
        for(std::size_t i=0;i<result.surface.values.size();++i){
            const auto* p=&result.surface.positions[3*i];check(p[0]<=position+2e-6,"Hidden-side triangle leaked");
            near(result.surface.values[i],p[0]+2*p[1]+3*p[2],3e-6,"Parent field evaluation");
        }
    }
    auto whole=RayFEM::BuildClippedSurface(mesh,&field,o,{{.5f,0,0},{1,0,0}},RayFEM::ClipMode::WholeElements);
    near(area(whole.surface),4,2e-5,"Whole-element boundary includes exposed interior faces");
    auto none=RayFEM::BuildClippedSurface(mesh,&field,o,{{-1,0,0},{1,0,0}});check(none.surface.positions.empty(),"Completely hidden geometry");
    auto all=RayFEM::BuildClippedSurface(mesh,&field,o,{{2,0,0},{1,0,0}});near(area(all.surface),6,2e-5,"Unclipped full geometry");
    mfem::GridFunction displacement(mesh.GetNodes()->FESpace());displacement=0.0;
    for(int i=0;i<displacement.FESpace()->GetNDofs();++i)displacement[displacement.FESpace()->DofToVDof(i,0)]=.25;
    const mfem::Vector before(*mesh.GetNodes());o.displacement=&displacement;
    auto moved=RayFEM::BuildClippedSurface(mesh,&field,o,{{.5f,0,0},{1,0,0}});
    near(area(moved.surface,true,.5),1,2e-5,"Displaced cap");
    for(std::size_t i=0;i<moved.surface.values.size();++i){const auto* p=&moved.surface.positions[3*i];near(moved.surface.values[i],p[0]-.25+2*p[1]+3*p[2],3e-6,"Displaced reference field");}
    for(int i=0;i<before.Size();++i)near((*mesh.GetNodes())[i],before[i],0,"Reference nodes mutated");
    std::cout<<"PASS closed cut surface, two algorithms, whole elements, displacement: "<<type<<'\n';
}
void boundaries(){
    auto quad=mfem::Mesh::MakeCartesian2D(2,2,mfem::Element::QUADRILATERAL,true);quad.SetCurvature(4);
    RayFEM::RenderOptions o;o.refinement=5;
    auto lines=RayFEM::BuildElementBoundaries(quad,o);check(lines.size()==12*5,"Quad boundaries must omit tessellation diagonals");
    auto clipped=RayFEM::BuildClippedSurface(quad,nullptr,o,{{.4f,0,0},{1,0,0}});near(area(clipped.surface),.4,2e-6,"2D clip area");
    auto hex=mfem::Mesh::MakeCartesian3D(2,2,2,mfem::Element::HEXAHEDRON);hex.SetCurvature(3);
    check(RayFEM::BuildElementBoundaries(hex,o).size()==48*5,"Only exterior volume element edges");
    bool threw=false;try{RayFEM::BuildClippedSurface(hex,nullptr,o,{{},{0,0,0}});}catch(const std::invalid_argument&){threw=true;}check(threw,"Invalid plane must be rejected");
    std::cout<<"PASS original-element boundary topology, 2D clipping and plane validation\n";
}
void curved(){
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);mesh.SetCurvature(3);
    mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& x,mfem::Vector& y){y=x;y[1]+=.8*x[0]*(1-x[0])*x[2];});mesh.Transform(bend);
    RayFEM::RenderOptions o;o.refinement=8;RayFEM::ClipPlane plane{{0,0,.5},{0,0,1}};
    auto accurate=RayFEM::BuildClippedSurface(mesh,nullptr,o,plane,RayFEM::ClipMode::CutThrough,RayFEM::CutAlgorithm::Refined);
    auto fast=RayFEM::BuildClippedSurface(mesh,nullptr,o,plane,RayFEM::ClipMode::CutThrough,RayFEM::CutAlgorithm::Planar);
    auto capMaxY=[](const RayFEM::SurfaceData& s){double hi=0;for(std::size_t i=0;i<s.positions.size();i+=9)if(std::abs(s.normals[i+2]-1)<1e-5&&std::abs(s.positions[i+2]-.5)<1e-6)for(int k=0;k<3;++k)hi=std::max(hi,double(s.positions[i+3*k+1]));return hi;};
    check(capMaxY(accurate.surface)>capMaxY(fast.surface)+.08,"Curved algorithm must resolve curved section boundary");
    std::cout<<"PASS refined curved section differs from planar corner approximation\n";
}
void vectors(){
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    mfem::ND_FECollection nd(2,3);mfem::RT_FECollection rt(1,3);mfem::FiniteElementSpace n(&mesh,&nd),r(&mesh,&rt);mfem::GridFunction curl(&n),div(&r);
    mfem::VectorFunctionCoefficient spin(3,[](const mfem::Vector& x,mfem::Vector& v){v.SetSize(3);v[0]=-x[1];v[1]=x[0];v[2]=0;});
    mfem::VectorFunctionCoefficient radial(3,[](const mfem::Vector& x,mfem::Vector& v){v=x;});curl.ProjectCoefficient(spin);div.ProjectCoefficient(radial);
    RayFEM::RenderOptions o;o.refinement=3;o.view=RayFEM::FieldView::CurlMagnitude;
    auto a=RayFEM::BuildClippedSurface(mesh,&curl,o,{{.4f,0,0},{1,0,0}});for(double v:a.surface.values)near(v,2,1e-8,"ND curl on cut");
    o.view=RayFEM::FieldView::Divergence;auto b=RayFEM::BuildClippedSurface(mesh,&div,o,{{.4f,0,0},{1,0,0}});for(double v:b.surface.values)near(v,3,1e-8,"RT divergence on cut");
    std::cout<<"PASS ND/RT field derivatives on interior cuts\n";
}
void nurbs(const std::string& data){
    mfem::Mesh mesh((data+"/ball-nurbs.mesh").c_str(),1,1);check(mesh.NURBSext,"Original NURBS fixture");
    const mfem::Vector before(*mesh.GetNodes());RayFEM::RenderOptions o;o.refinement=5;
    for(auto algorithm:{RayFEM::CutAlgorithm::Refined,RayFEM::CutAlgorithm::Planar}){
        auto s=RayFEM::BuildClippedSurface(mesh,nullptr,o,{{.17f,0,0},{1,0,0}},RayFEM::ClipMode::CutThrough,algorithm);
        check(!s.surface.positions.empty(),"NURBS cut surface");check(area(s.surface,true,.17)>1,"NURBS interior cap");
        for(std::size_t i=0;i<s.surface.positions.size();i+=3)check(s.surface.positions[i]<=.170002,"NURBS clipping side");
    }
    check(mesh.NURBSext,"NURBS was converted");for(int i=0;i<before.Size();++i)near((*mesh.GetNodes())[i],before[i],0,"NURBS nodes changed");
    std::cout<<"PASS original rational NURBS clipping, no node mutation\n";
}
}
int main(int argc,char** argv){try{boundaries();basics(mfem::Element::HEXAHEDRON);basics(mfem::Element::TETRAHEDRON);curved();vectors();nurbs(argc>1?argv[1]:"subprojects/mfem/data");}catch(const std::exception& e){std::cerr<<"Clipping test failed: "<<e.what()<<'\n';return 1;}std::cout<<"Clipping tests passed\n";}
