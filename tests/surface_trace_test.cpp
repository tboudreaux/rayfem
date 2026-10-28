#include "libRayfem/profile/surface_trace.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {
using namespace RayFEM;
constexpr double pi=3.14159265358979323846;
void Check(bool okay,const std::string& message){if(!okay)throw std::runtime_error(message);}
void Near(double a,double b,double tolerance,const std::string& message){if(!std::isfinite(a)||std::abs(a-b)>tolerance){std::ostringstream s;s<<message<<": "<<a<<" vs "<<b;throw std::runtime_error(s.str());}}
double Norm(ProfilePosition p){return std::hypot(p[0],p[1],p[2]);}
double Ramp(ProfilePosition p){return 1+p[0]+2*p[1]+3*p[2];}
void ScalarCoordinates(mfem::GridFunction& field,const mfem::GridFunction& nodes){
    for(int dof=0;dof<field.Size();++dof){field[dof]=1;for(int d=0;d<3;++d)field[dof]+=(d+1)*nodes[nodes.FESpace()->DofToVDof(dof,d)];}
}
void Valid(const SurfaceTraceSample& sample){Check(sample.valid,"Expected a valid exterior intersection");Check(std::isfinite(sample.value)&&std::isfinite(sample.distance)&&sample.ray_distance>=0,"Valid trace metadata must be finite");}

void Paths(){
    const auto meridian=MakeMeridianRays({2,3,4},{0,0,7},{2,0,3},.5,0,pi,17);
    Near(meridian.front().direction[2],1,1e-15,"North pole");Near(meridian.back().direction[2],-1,1e-15,"South pole");
    Near(meridian[8].direction[0],std::cos(.5),1e-15,"Meridian azimuth");
    const auto ring=MakeAzimuthRays({0,0,0},{1,0,0},{0,0,1},pi/3,0,2*pi,33);
    for(const auto& ray:ring){Near(Norm(ray.direction),1,1e-15,"Normalized spherical frame");Near(ray.direction[0],.5,1e-15,"Rotated colatitude");}
    for(int d=0;d<3;++d)Near(ring.front().direction[d],ring.back().direction[d],1e-15,"Closed azimuth path");
    bool threw=false;try{MakeMeridianRays({},{0,0,1},{0,0,2},0,0,pi,3);}catch(const std::invalid_argument&){threw=true;}Check(threw,"Parallel axis/reference rejected");
    const auto tiny=MakeMeridianRays({},{0,0,1e-320},{1e-320,0,0},0,0,pi,3);Near(tiny[1].direction[0],1,1e-15,"Finite subnormal axes normalize safely");
    std::cout<<"PASS parameterized meridian/azimuth directions and validation\n";
}

void SphereAndEllipsoid(const std::string& data){
    mfem::Mesh mesh((data+"/ball-nurbs.mesh").c_str(),1,1);auto* nodes=mesh.GetNodes();Check(mesh.NURBSext&&nodes,"Exact sphere fixture");
    const mfem::Vector original(*nodes);mfem::FiniteElementSpace scalar_space(&mesh,nodes->FESpace()->FEColl());
    mfem::GridFunction scalar(&scalar_space);ScalarCoordinates(scalar,*nodes);
    const auto meridian=MakeMeridianRays({},{0,0,1},{1,0,0},.37,0,pi,81);
    const auto ring=MakeAzimuthRays({},{0,0,1},{1,0,0},1.1,0,2*pi,65);
    SurfaceTraceRegistry registry;RenderOptions options;options.refinement=3;
    for(const auto& rays:{meridian,ring}){
        const auto samples=registry.sample("curved",mesh,scalar,rays,options);
        for(std::size_t i=0;i<samples.size();++i){const auto& s=samples[i];Valid(s);Near(Norm(s.position),1,2e-10,"Exact rational sphere radius");
            for(int d=0;d<3;++d)Near(s.position[d],rays[i].direction[d],2e-10,"Exact angular surface location");
            Near(s.value,Ramp(s.position),2e-11,"True FE value on rational boundary");Near(s.residual,0,8e-11,"Ray/curved-face residual");
            Check(s.face>=0&&s.boundary>=0&&s.boundary_attribute==mesh.GetBdrAttribute(s.boundary)&&s.material==mesh.GetAttribute(s.element),"Boundary and material identity");
        }
    }
    const auto path=registry.sample("curved",mesh,scalar,meridian,options);
    Near(path.back().distance,160*std::sin(pi/160),2e-9,"Physical cumulative meridian chords");
    options.refinement=8;const auto benchmark_rays=MakeMeridianRays({},{0,0,1},{1,0,0},.37,0,pi,128);
    const auto started=std::chrono::steady_clock::now();auto benchmark=registry.sample("curved",mesh,scalar,benchmark_rays,options);
    for(const auto& s:benchmark)Valid(s);
    std::cout<<"NURBS sphere: 128 curved trace samples, refinement 8: "<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()<<" ms\n";
    double coarse=0,fine=0;
    for(int refinement:{1,12}){
        options.refinement=refinement;const auto samples=registry.sample("refined",mesh,scalar,meridian,options);double error=0;
        for(const auto& s:samples){Valid(s);Near(Norm(s.position),1,2e-12,"Approximation still evaluates actual curved face");Near(s.value,Ramp(s.position),2e-12,"Approximate trace uses actual parent FE point");error=std::max(error,s.residual);}
        if(refinement==1)coarse=error;else fine=error;
    }
    Check(coarse>1e-3&&fine<coarse/20,"Refined radial alignment must converge");
    options.refinement=3;
    for(auto ordering:{mfem::Ordering::byNODES,mfem::Ordering::byVDIM}){
        mfem::FiniteElementSpace displacement_space(&mesh,nodes->FESpace()->FEColl(),3,ordering);mfem::GridFunction displacement(&displacement_space);options.displacement=&displacement;
        for(double time:{0.0,pi/2,pi,3*pi/2,2*pi}){
            const double stretch=.4*(1-std::cos(time));const double axes[]={1+stretch,1-.5*stretch,1+.25*stretch};
            for(int i=0;i<displacement_space.GetNDofs();++i)for(int d=0;d<3;++d)displacement[displacement_space.DofToVDof(i,d)]=(axes[d]-1)*original[nodes->FESpace()->DofToVDof(i,d)];
            const auto samples=registry.sample("curved",mesh,scalar,meridian,options);
            for(std::size_t i=0;i<samples.size();++i){const auto& s=samples[i];Valid(s);double denominator=0;for(int d=0;d<3;++d)denominator+=std::pow(meridian[i].direction[d]/axes[d],2);
                const double radius=1/std::sqrt(denominator);ProfilePosition reference{};
                for(int d=0;d<3;++d){Near(s.position[d],radius*meridian[i].direction[d],3e-10,"Ellipsoid radial intersection");reference[d]=s.position[d]/axes[d];}
                Near(s.value,Ramp(reference),3e-11,"Displaced trace retains original material field value");
            }
            for(int i=0;i<original.Size();++i)Near((*nodes)[i],original[i],0,"Trace never mutates NURBS nodes");
        }
    }
    options.displacement=nullptr;
    // From outside, choose the near exterior intersection rather than the far
    // side. The ray does not point through a tessellation vertex.
    const std::vector<SurfaceTraceRay> outside{{{2,.13,.21},{-1,0,0},7}};
    auto hit=registry.sample("curved",mesh,scalar,outside,options);Valid(hit[0]);Near(hit[0].position[0],std::sqrt(1-.13*.13-.21*.21),2e-10,"Nearest positive exterior hit");
    mfem::Mesh polynomial_mesh((data+"/ball-nurbs.mesh").c_str(),1,1);polynomial_mesh.SetCurvature(5);auto* polynomial_nodes=polynomial_mesh.GetNodes();mfem::FiniteElementSpace polynomial_space(&polynomial_mesh,polynomial_nodes->FESpace()->FEColl());mfem::GridFunction polynomial(&polynomial_space);ScalarCoordinates(polynomial,*polynomial_nodes);
    auto curved=registry.sample("curved",polynomial_mesh,polynomial,meridian,options);for(const auto& s:curved){Valid(s);Near(Norm(s.position),1,.006,"High-order polynomial sphere approximation");Near(s.value,Ramp(s.position),2e-10,"High-order parent field");}
    std::cout<<"PASS rational/high-order meridians and rings, nearest hits, convergent approximation, ten ellipsoid states, both vector orderings\n";
}

void SelectionAndGaps(){
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON);mesh.SetAttribute(0,4);mesh.SetAttribute(1,9);mesh.SetCurvature(2);
    mfem::H1_FECollection h1(2,3);mfem::FiniteElementSpace space(&mesh,&h1);mfem::GridFunction field(&space);field=2.;SurfaceTraceRegistry registry;RenderOptions options;
    const std::vector<SurfaceTraceRay> rays{{{.25,.5,.5},{-1,0,0},0},{{.25,.5,.5},{1,0,0},1},{{.25,.5,.5},{0,-1,0},2},{{2,2,2},{1,0,0},3},{{},{},4}};
    auto all=registry.sample("curved",mesh,field,rays,options);Valid(all[0]);Valid(all[1]);Check(all[0].material==4&&all[1].material==9,"Parent material labels");Check(!all[3].valid&&!all[4].valid,"Outside and zero-direction queries are gaps");
    options.excluded_materials={9};auto selected=registry.sample("curved",mesh,field,rays,options);Valid(selected[0]);Valid(selected[2]);
    Check(!selected[1].valid&&std::isnan(selected[1].value)&&selected[1].element==-1,"Hidden material creates a gap, not an exposed interface trace");
    Near(selected[2].distance,selected[0].distance,0,"Distance does not bridge excluded samples");
    options.excluded_materials={4,9};options.view=FieldView::CurlMagnitude;auto empty=registry.sample("curved",mesh,field,rays,options);Check(std::none_of(empty.begin(),empty.end(),[](const auto& s){return s.valid;}),"All materials hidden, even when unused scalar curl would be incompatible");
    std::cout<<"PASS selected materials, original-boundary scope, metadata and invalid gaps\n";
}

void CurvedFacesAndVectorViews(){
    SurfaceTraceRegistry registry;
    for(auto geometry:{mfem::Element::QUADRILATERAL,mfem::Element::TRIANGLE}){
        auto mesh=mfem::Mesh::MakeCartesian2D(2,2,geometry,true);mesh.SetCurvature(3,false,3);
        mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& p,mfem::Vector& x){x=p;x[2]=1+.2*p[0]*p[0]+.1*p[1]*p[1];});mesh.Transform(bend);
        mfem::FiniteElementSpace space(&mesh,mesh.GetNodes()->FESpace()->FEColl());mfem::GridFunction field(&space);ScalarCoordinates(field,*mesh.GetNodes());
        std::vector<SurfaceTraceRay> rays{{{0,0,0},{.2,.4,1},0}};auto samples=registry.sample("curved",mesh,field,rays);Valid(samples[0]);
        const double z=2/(1+std::sqrt(1-4*.024));Near(samples[0].position[2],z,3e-11,"Curved embedded-face Newton intersection");Near(samples[0].value,Ramp(samples[0].position),3e-11,"Embedded parent field");Check(samples[0].face==-1&&samples[0].boundary==-1,"Embedded surface identity");
    }
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::TETRAHEDRON);mesh.SetCurvature(3);
    mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& p,mfem::Vector& x){x=p;x[2]*=1+.2*p[0]*p[0]+.1*p[1]*p[1];});mesh.Transform(bend);
    mfem::FiniteElementSpace space(&mesh,mesh.GetNodes()->FESpace()->FEColl());mfem::GridFunction scalar(&space);ScalarCoordinates(scalar,*mesh.GetNodes());
    const std::vector<SurfaceTraceRay> rays{{{.27,.43,.5},{0,0,1},0}};auto samples=registry.sample("curved",mesh,scalar,rays);Valid(samples[0]);Near(samples[0].position[2],1+.2*.27*.27+.1*.43*.43,2e-11,"Curved triangular volume face");
    auto cube=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);cube.SetCurvature(3);
    mfem::ND_FECollection nd(2,3);mfem::RT_FECollection rt(1,3);mfem::FiniteElementSpace nd_space(&cube,&nd),rt_space(&cube,&rt);
    mfem::GridFunction curl(&nd_space),div(&rt_space);mfem::VectorFunctionCoefficient spin(3,[](const mfem::Vector& p,mfem::Vector& v){v.SetSize(3);v[0]=-p[1];v[1]=p[0];v[2]=0;});
    mfem::VectorFunctionCoefficient flux(3,[](const mfem::Vector& p,mfem::Vector& v){v=p;});curl.ProjectCoefficient(spin);div.ProjectCoefficient(flux);
    mfem::H1_FECollection displacement_collection(2,3);mfem::FiniteElementSpace displacement_space(&cube,&displacement_collection,3);mfem::GridFunction displacement(&displacement_space);
    mfem::VectorFunctionCoefficient warp(3,[](const mfem::Vector& p,mfem::Vector& d){d.SetSize(3);d=0.;d[2]=.2*p[0]*p[0];});displacement.ProjectCoefficient(warp);
    RenderOptions options;options.displacement=&displacement;options.view=FieldView::CurlMagnitude;auto circulation=registry.sample("curved",cube,curl,rays,options);Valid(circulation[0]);Near(circulation[0].value,2,2e-11,"Original-geometry ND curl");Near(circulation[0].position[2],1+.2*.27*.27,2e-11,"Different displacement/geometry bases");
    options.view=FieldView::Divergence;auto divergence=registry.sample("curved",cube,div,rays,options);Near(divergence[0].value,3,2e-11,"Original-geometry RT divergence");
    std::cout<<"PASS curved triangular/quad exterior and embedded faces, ND curl/RT divergence, arbitrary nodal displacement basis\n";
}

void FoldedFaceAndIntegralDisplacement(){
    SurfaceTraceRegistry registry;
    auto flat=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);flat.SetCurvature(2,false,3);
    mfem::FiniteElementSpace flat_space(&flat,flat.GetNodes()->FESpace()->FEColl());mfem::GridFunction flat_field(&flat_space);flat_field=1.;RenderOptions flat_options;flat_options.refinement=1;
    const auto coplanar=registry.sample("curved",flat,flat_field,{{{-1,.5,0},{1,0,0},0}},flat_options);Check(!coplanar[0].valid,"Coplanar ray must be an explicit gap, not an arbitrary center hit");
    bool bounded=false;try{registry.sample("curved",flat,flat_field,std::vector<SurfaceTraceRay>(50001,{{.5,.5,-1},{0,0,1},0}),flat_options);}catch(const std::invalid_argument& e){bounded=std::string(e.what()).find("100000 curved solves")!=std::string::npos;}Check(bounded,"Actual Newton solves are bounded even when triangle seeds duplicate a ray/face pair");
    RenderOptions options;options.refinement=1;
    for(double offset:{.5,.25}){
        auto folded=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);folded.SetCurvature(2,false,3);
        mfem::VectorFunctionCoefficient bend(3,[offset](const mfem::Vector& p,mfem::Vector& x){x=p;x[2]=(p[0]-offset)*(p[0]-offset);});folded.Transform(bend);
        mfem::FiniteElementSpace space(&folded,folded.GetNodes()->FESpace()->FEColl());mfem::GridFunction field(&space);ScalarCoordinates(field,*folded.GetNodes());
        const double z=offset==.5?.1:.04;const auto hits=registry.sample("curved",folded,field,{{{-1,.5,z},{1,0,0},0}},options);
        Valid(hits[0]);Near(hits[0].position[0],offset-std::sqrt(z),1e-10,"Folded face nearest root despite absent triangle seed and singular/far-root center");Near(hits[0].position[2],z,1e-11,"Folded face stays on requested ray");
    }
    auto cube=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);cube.SetCurvature(2);
    mfem::VectorFunctionCoefficient warp(3,[](const mfem::Vector& p,mfem::Vector& x){x=p;x[0]+=.2*p[0]*p[0];});cube.Transform(warp);
    mfem::FiniteElementSpace scalar_space(&cube,cube.GetNodes()->FESpace()->FEColl());mfem::GridFunction scalar(&scalar_space);ScalarCoordinates(scalar,*cube.GetNodes());
    mfem::L2_FECollection integral(2,3,mfem::BasisType::GaussLegendre,mfem::FiniteElement::INTEGRAL);mfem::FiniteElementSpace vector_space(&cube,&integral,3);mfem::GridFunction displacement(&vector_space);
    mfem::VectorFunctionCoefficient shift(3,[](const mfem::Vector&,mfem::Vector& v){v.SetSize(3);v=0.;v[2]=.1;});displacement.ProjectCoefficient(shift);options.displacement=&displacement;
    const auto hits=registry.sample("curved",cube,scalar,{{{.37,.43,.5},{0,0,1},0}},options);Valid(hits[0]);Near(hits[0].position[2],1.1,2e-10,"Integral-mapped displacement uses the current parent integration point");
    Near(hits[0].value,Ramp({.37,.43,1}),2e-10,"Integral-map displacement retains reference field");
    std::cout<<"PASS folded-face fallback roots and integral-mapped displacement on nonaffine geometry\n";
}
class Custom final:public SurfaceTraceAlgorithm {
public:explicit Custom(double value):value_(value){}std::string id()const override{return "custom";}std::string name()const override{return "Custom";}
    std::vector<SurfaceTraceSample> sample(mfem::Mesh&,mfem::GridFunction&,const std::vector<SurfaceTraceRay>& rays,const RenderOptions&)const override{std::vector<SurfaceTraceSample> out(rays.size());for(auto& p:out){p.valid=true;p.value=value_;p.element=0;}return out;}
private:double value_;
};
void DispatchAndCSV(){
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);mfem::H1_FECollection collection(1,3);mfem::FiniteElementSpace space(&mesh,&collection);mfem::GridFunction field(&space);field=1.;
    SurfaceTraceRegistry registry;registry.registerAlgorithm(std::make_shared<Custom>(7));registry.registerAlgorithm(std::make_shared<Custom>(13));
    const std::vector<SurfaceTraceRay> rays{{{.5,.5,.5},{1,0,0},.1},{{2,2,2},{1,0,0},.2}};
    Near(registry.sample("custom",mesh,field,rays)[0].value,13,0,"Replaceable trace algorithm");Check(registry.algorithms().size()==3,"Replacing registry entry preserves unique IDs");
    RenderOptions excluded;excluded.excluded_materials={1};const auto filtered=registry.sample("custom",mesh,field,rays,excluded);Check(!filtered[0].valid&&filtered[0].element==-1&&std::isnan(filtered[0].value),"Custom algorithms cannot bypass material selection");
    bool threw=false;try{registry.sample("missing",mesh,field,rays);}catch(const std::invalid_argument&){threw=true;}Check(threw,"Unknown algorithm reported");
    ProfileMetadata metadata;metadata.field_name="field, \"quoted\"";metadata.frame=9;metadata.time=.5;metadata.algorithm="curved";metadata.excluded_materials={4,7};
    std::ostringstream csv;ExportSurfaceTraceCSV(csv,registry.sample("curved",mesh,field,rays),metadata);const auto text=csv.str();
    Check(std::count(text.begin(),text.end(),'\n')==3&&text.find("field, \"\"quoted\"\"")!=std::string::npos,"CSV rows and escaping");Check(text.find("boundary_attribute")!=std::string::npos&&text.find("ray_residual")!=std::string::npos&&text.find("nan")!=std::string::npos,"CSV metadata and explicit gaps");
    Check(text.find("excluded_materials")!=std::string::npos&&text.find("\"4;7\"")!=std::string::npos,"CSV material selection metadata");
    SurfaceTraceSample tiny;tiny.value=1e-100;tiny.valid=true;std::ostringstream fixed;fixed<<std::fixed<<std::setprecision(3);const auto flags=fixed.flags();ExportSurfaceTraceCSV(fixed,{tiny},metadata);Check(fixed.str().find("1e-100")!=std::string::npos&&fixed.flags()==flags&&fixed.precision()==3,"CSV preserves tiny values and caller formatting");
    std::ostringstream failed;failed.setstate(std::ios::badbit);threw=false;try{ExportSurfaceTraceCSV(failed,{tiny},metadata);}catch(const std::runtime_error&){threw=true;}Check(threw,"CSV output failures reported");
    RenderOptions too_large;too_large.refinement=64;threw=false;try{registry.sample("curved",mesh,field,MakeMeridianRays({},{0,0,1},{1,0,0},0,0,pi,2048),too_large);}catch(const std::invalid_argument& e){threw=std::string(e.what()).find("too large")!=std::string::npos;}Check(threw,"Oversized synchronous work rejected before sampling");
    std::cout<<"PASS extensible dispatch, replaced algorithms, CSV metadata/escaping/gaps\n";
}
}
int main(int argc,char** argv){try{Paths();SphereAndEllipsoid(argc>1?argv[1]:"subprojects/mfem/data");SelectionAndGaps();CurvedFacesAndVectorViews();FoldedFaceAndIntegralDisplacement();DispatchAndCSV();std::cout<<"All surface trace tests passed\n";return 0;}catch(const std::exception& e){std::cerr<<"Surface trace test failed: "<<e.what()<<'\n';return 1;}}
