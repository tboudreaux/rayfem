#include "libRayfem/vector/flow.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace RayFEM;using P=ProfilePosition;
void Check(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
void Near(double a,double b,double eps,const std::string& message){if(!std::isfinite(a)||std::abs(a-b)>eps)throw std::runtime_error(message+": "+std::to_string(a)+" vs "+std::to_string(b));}
double Distance(P a,P b){return std::hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);}
std::unique_ptr<mfem::GridFunction> Field(mfem::Mesh& mesh,mfem::FiniteElementCollection* fec,int components=1){auto* space=new mfem::FiniteElementSpace(&mesh,fec,components);auto result=std::make_unique<mfem::GridFunction>(space);result->MakeOwner(fec);return result;}
auto Constant(mfem::Mesh& mesh,int order=2){auto f=Field(mesh,new mfem::H1_FECollection(order,mesh.Dimension()),mesh.SpaceDimension());mfem::VectorFunctionCoefficient coefficient(mesh.SpaceDimension(),[](const mfem::Vector& x,mfem::Vector& v){v.SetSize(x.Size());v=0.;v[0]=1.;});f->ProjectCoefficient(coefficient);return f;}
double Length(const FlowLine& line){double total=0;for(std::size_t i=1;i<line.points.size();++i)total+=Distance(line.points[i-1].position,line.points[i].position);return total;}
void DenseAndConstant(){
    for(int dim:{2,3}){
        auto mesh=dim==2?mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true):mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);auto field=Constant(mesh);
        FlowOptions options;options.seed_resolution=8;options.max_seeds=32;auto cloud=SampleVectorField(mesh,*field,{},options);Check(cloud.points.size()==32,"Many distributed arrows within one element");Check(cloud.diagnostics.location_tests==0,"Automatic seeds need no inverse searches");
        for(int d=0;d<dim;++d){double lo=1,hi=0;for(const auto& p:cloud.points){lo=std::min(lo,p.position[d]);hi=std::max(hi,p.position[d]);Near(p.vector[0],1,1e-12,"Exact constant vector");}Check(hi-lo>.6,"Capped reference samples cover every spatial axis");}
        P center{.5,.5,dim==3?.5:0};options.max_length=.6;options.step_size=.05;options.max_steps=30;
        for(auto method:{FlowIntegrator::RK2,FlowIntegrator::RK4}){options.integrator=method;auto paths=BuildStreamlines(mesh,*field,{},options,{center});Check(paths.lines.size()==1,"Constant bidirectional streamline generated");const auto& line=paths.lines[0];Near(line.points.front().position[0],.2,1e-9,"Backward endpoint");Near(line.points.back().position[0],.8,1e-9,"Forward endpoint");Near(Length(line),.6,1e-9,"Combined arc-length budget");for(std::size_t i=1;i<line.points.size();++i)Check(line.points[i].position[0]>line.points[i-1].position[0],"Bidirectional order follows physical field");}
        *field*=1e-310;auto scaled=BuildStreamlines(mesh,*field,{},options,{center});Check(scaled.lines.size()==1,"Normalized paths are invariant to tiny nonzero vector amplitude");Near(Length(scaled.lines[0]),.6,1e-9,"Tiny vector amplitude keeps physical arc length");
    }
    for(auto geometry:{mfem::Element::TRIANGLE,mfem::Element::TETRAHEDRON,mfem::Element::WEDGE,mfem::Element::PYRAMID}){
        const bool triangle=geometry==mfem::Element::TRIANGLE;
        auto mesh=triangle?mfem::Mesh::MakeCartesian2D(2,2,geometry,true):mfem::Mesh::MakeCartesian3D(2,2,2,geometry);
        auto field=Constant(mesh,1);mfem::VectorFunctionCoefficient identity(mesh.SpaceDimension(),[](const mfem::Vector& x,mfem::Vector& value){value=x;});field->ProjectCoefficient(identity);FlowOptions options;options.seed_resolution=8;options.max_seeds=100;auto cloud=SampleVectorField(mesh,*field,{},options);
        Check(cloud.points.size()==std::size_t(triangle?64:100),"Reference seeds cover simplex, wedge and pyramid cells");
        for(const auto& point:cloud.points)for(int d=0;d<mesh.SpaceDimension();++d){Check(point.position[d]>=0&&point.position[d]<=1,"Mapped reference seed lies in physical domain");Near(point.vector[d],point.position[d],2e-12,"Reference seed field and geometry share the exact parent point");}
    }
    {
        auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON);for(int i=0;i<mesh.GetNV();++i)if(mesh.GetVertex(i)[0]==.5)mesh.GetVertex(i)[0]=.1;
        auto field=Constant(mesh);FlowOptions options;options.seed_resolution=8;options.max_seeds=512;auto cloud=SampleVectorField(mesh,*field,{},options);int small=0;for(auto point:cloud.points)small+=point.position[0]<.1;Check(small>40&&small<65,"Physical-volume weighting avoids bias toward small elements");
        options.max_steps=1;options.max_length=.5;auto paths=BuildStreamlines(mesh,*field,{},options,{{.5,.5,.5}});Check(paths.diagnostics.budget_exhausted,"Step-limited incomplete paths report their budget limit");
    }
    std::cout<<"PASS dense domain-wide arrows, capped reference-sample distribution, RK2/RK4 straight paths and ordered bidirection\n";
}
void DirectSeedCounts(){
    for(int dimension:{2,3}){
        auto mesh=dimension==2?mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true):
                              mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
        auto field=Constant(mesh);
        FlowOptions options;options.max_seeds=512;
        for(int count:{1,7,37,144}){
            options.seed_count=count;options.seed_resolution=1;
            const auto first=SampleVectorField(mesh,*field,{},options);
            Check(first.points.size()==std::size_t(count)&&first.diagnostics.seed_count==std::size_t(count),
                  "Direct seed count is exact and independent of mesh dimension");
            Check(first.diagnostics.location_tests==0,"Direct reference seeding avoids inverse searches");
            options.seed_resolution=64;
            const auto second=SampleVectorField(mesh,*field,{},options);
            Check(second.points.size()==first.points.size(),"Direct count overrides resolution");
            for(std::size_t i=0;i<first.points.size();++i)
                Check(first.points[i].position==second.points[i].position&&first.points[i].element==second.points[i].element,
                      "Direct seed allocation is deterministic");
        }
        options.seed_count=std::numeric_limits<int>::max();options.max_seeds=11;
        Check(SampleVectorField(mesh,*field,{},options).points.size()==11,"Oversized direct request is safely capped by max_seeds");
        options.max_points=5;
        Check(SampleVectorField(mesh,*field,{},options).points.size()==5,"Direct request respects point budget");
        options.max_points=1000;options.max_seeds=100;options.seed_count=37;options.max_evaluations=9;
        const auto limited=SampleVectorField(mesh,*field,{},options);
        Check(limited.points.size()==9&&limited.diagnostics.budget_exhausted&&limited.diagnostics.evaluations==9,
              "Direct request respects evaluation budget");
        options.max_evaluations=100000;options.seed_count=17;options.max_length=.001;options.step_size=.001;
        options.max_steps=4;options.bidirectional=false;
        const auto paths=BuildStreamlines(mesh,*field,{},options);
        Check(paths.diagnostics.seed_count==17&&paths.lines.size()==17,"Direct streamline count on a regular nonzero field");
        const P center{.5,.5,dimension==3?.5:0};
        const auto explicit_path=BuildStreamlines(mesh,*field,{},options,{center});
        Check(explicit_path.diagnostics.seed_count==1&&explicit_path.lines.size()==1,"Explicit seeds override automatic count");
        *field=0.;
        const auto zero=BuildStreamlines(mesh,*field,{},options);
        Check(zero.diagnostics.seed_count==17&&zero.lines.empty(),"Accepted seed count is distinct from visible paths in zero fields");
        options.seed_count=-1;bool invalid=false;
        try{SampleVectorField(mesh,*field,{},options);}catch(const std::invalid_argument&){invalid=true;}
        Check(invalid,"Negative direct counts are rejected");
        options.seed_count=0;options.seed_resolution=3;options.max_seeds=512;
        Check(SampleVectorField(mesh,*field,{},options).points.size()==std::size_t(dimension==2?9:27),
              "Zero direct count preserves legacy resolution behavior");
    }
    {
        auto mesh=mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON);
        for(int e=0;e<3;++e)mesh.GetElement(e)->SetAttribute(e+1);
        auto field=Constant(mesh);FlowOptions options;options.seed_count=53;
        RenderOptions render;render.excluded_materials={2};render.excluded_elements={0};
        auto cloud=SampleVectorField(mesh,*field,render,options);
        Check(cloud.points.size()==53,"Direct seeds redistribute over included elements");
        for(const auto& point:cloud.points)Check(point.element==2,"Direct count respects material and individual element exclusions");
        options.retained_elements={false,false,true};
        Check(SampleVectorField(mesh,*field,render,options).points.size()==53,"Direct count respects retained-cell selection");
        options.retained_elements={false,false,false};
        Check(SampleVectorField(mesh,*field,render,options).points.empty(),"An empty included domain cannot create direct seeds");
    }
    std::cout<<"PASS direct 2D/3D seed totals, deterministic placement, masks, caps, budgets, explicit overrides and zero-field path semantics\n";
}
void RotationsAndSpaces(){
    for(int dim:{2,3}){
        auto mesh=dim==2?mfem::Mesh::MakeCartesian2D(3,3,mfem::Element::QUADRILATERAL,true,2,2):mfem::Mesh::MakeCartesian3D(3,3,3,mfem::Element::HEXAHEDRON,2,2,2);mesh.SetCurvature(4);
        mfem::VectorFunctionCoefficient shift(dim,[](const mfem::Vector& x,mfem::Vector& y){y=x;for(int d=0;d<y.Size();++d)y[d]-=1;});mesh.Transform(shift);
        mfem::VectorFunctionCoefficient spin(dim,[](const mfem::Vector& x,mfem::Vector& v){v.SetSize(x.Size());v=0.;v[0]=-x[1];v[1]=x[0];});
        for(int type=0;type<3;++type){auto field=type==0?Field(mesh,new mfem::H1_FECollection(4,dim),dim):type==1?Field(mesh,new mfem::ND_FECollection(2,dim)):Field(mesh,new mfem::RT_FECollection(1,dim));field->ProjectCoefficient(spin);FlowOptions options;options.max_length=3.;options.step_size=.03;options.max_steps=220;options.bidirectional=false;options.max_seeds=1;auto paths=BuildStreamlines(mesh,*field,{},options,{{.5,0,0}});Check(paths.lines.size()==1,"Rotational streamline for H1/ND/RT");Check(paths.lines[0].points.size()>80,"Rotation traverses element interfaces");for(auto p:paths.lines[0].points){Near(std::hypot(p.position[0],p.position[1]),.5,2e-6,"RK4 circular radius");Near(p.vector[0],-p.position[1],2e-9,"Exact mapped physical vector x");Near(p.vector[1],p.position[0],2e-9,"Exact mapped physical vector y");}}
    }
    std::cout<<"PASS 2D/3D H1, Hcurl ND and Hdiv RT streamlines across elements\n";
}
void SelectionAndLimits(){
    auto mesh=mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON,3,1,1);for(int v=0;v<mesh.GetNV();++v){double& x=mesh.GetVertex(v)[0];x=x==0?0:x==1?.99:x==2?1.01:2;}
    const int attrs[]={2,7,42};for(int e=0;e<mesh.GetNE();++e)mesh.GetElement(e)->SetAttribute(attrs[e]);auto field=Constant(mesh);RenderOptions render;render.excluded_materials={7};FlowOptions options;options.bidirectional=false;options.step_size=.6;options.max_length=1.2;options.max_steps=50;auto paths=BuildStreamlines(mesh,*field,render,options,{{.8,.5,.5}});Check(paths.lines.size()==1,"Path approaches excluded barrier");for(auto p:paths.lines[0].points)Check(p.position[0]<.990001&&mesh.GetAttribute(p.element)==2,"Large steps cannot tunnel across thin excluded material");
    render.excluded_materials.clear();options.step_size=.05;options.clip_plane=ClipPlane{{.5f,0,0},{1,0,0}};auto cut=BuildStreamlines(mesh,*field,render,options,{{.2,.5,.5}});Check(!cut.lines.empty(),"Clipped path generated");for(auto line:cut.lines)for(auto p:line.points)Check(p.position[0]<=.500001,"Streamlines respect cut plane");
    options.clip_plane.reset();options.retained_elements={true,false,false};auto cloud=SampleVectorField(mesh,*field,render,options);for(auto p:cloud.points)Check(p.element==0,"Distributed arrows respect retained cells");
    options.retained_elements.clear();options.max_points=25;options.max_seeds=8;options.seed_resolution=4;options.step_size=.02;options.bidirectional=true;auto budget=BuildStreamlines(mesh,*field,render,options);std::size_t count=0;for(auto line:budget.lines)count+=line.points.size();Check(count<=25&&budget.diagnostics.budget_exhausted,"Total path-point budget enforced");Check(budget.lines.size()>=7,"Round-robin integration gives most seeds visible paths before budget exhaustion");
    options.max_points=10000;options.max_evaluations=12;auto eval=SampleVectorField(mesh,*field,render,options);Check(eval.diagnostics.evaluations<=12,"Evaluation budget enforced");options.max_evaluations=10000;options.max_location_tests=3;auto location=BuildStreamlines(mesh,*field,render,options,{{.5,.5,.5}});Check(location.diagnostics.location_tests<=3&&location.diagnostics.budget_exhausted,"Inverse-test budget enforced");
    options.max_location_tests=10000;int checks=0;options.cancelled=[&](){return ++checks>3;};auto cancelled=SampleVectorField(mesh,*field,render,options);Check(cancelled.diagnostics.cancelled&&checks<20,"Cancellation polled during preparation");
    auto scalar=Field(mesh,new mfem::H1_FECollection(1,3));bool rejected=false;try{SampleVectorField(mesh,*scalar);}catch(const std::invalid_argument&){rejected=true;}Check(rejected,"Scalar fields explicitly rejected");
    std::cout<<"PASS thin hidden barriers, clipping, retained cells, fair point/evaluation/location budgets and cancellation\n";
}
void CurvedDisplacedAndRational(const std::string& data){
    auto mesh=mfem::Mesh::MakeCartesian3D(3,3,3,mfem::Element::HEXAHEDRON,2,2,2);mesh.SetCurvature(5);
    mfem::VectorFunctionCoefficient spheroid(3,[](const mfem::Vector& x,mfem::Vector& y){double a=x[0]-1,b=x[1]-1,c=x[2]-1;y.SetSize(3);y[0]=1.5*a*std::sqrt(1-b*b/2-c*c/2+b*b*c*c/3);y[1]=.8*b*std::sqrt(1-a*a/2-c*c/2+a*a*c*c/3);y[2]=1.2*c*std::sqrt(1-a*a/2-b*b/2+a*a*b*b/3);});mesh.Transform(spheroid);
    auto field=Field(mesh,new mfem::H1_FECollection(5,3),3);mfem::VectorFunctionCoefficient spin(3,[](const mfem::Vector& x,mfem::Vector& v){v.SetSize(3);v[0]=-x[1];v[1]=x[0];v[2]=0;});field->ProjectCoefficient(spin);FlowOptions options;options.max_length=2.;options.step_size=.025;options.max_steps=120;options.bidirectional=false;auto curve=BuildStreamlines(mesh,*field,{},options,{{.4,0,0}});Check(curve.lines.size()==1&&curve.lines[0].points.size()>70,"P5 cube-projected spheroid flow remains smooth across curved cells");for(auto p:curve.lines[0].points)Near(std::hypot(p.position[0],p.position[1]),.4,4e-6,"High-order physical circular trajectory");
    mfem::GridFunction displacement(mesh.GetNodes()->FESpace());displacement=0.;for(int i=0;i<displacement.FESpace()->GetNDofs();++i)displacement[displacement.FESpace()->DofToVDof(i,0)]=2.;const mfem::Vector original(*mesh.GetNodes());RenderOptions render;render.displacement=&displacement;auto moved=BuildStreamlines(mesh,*field,render,options,{{2.4,0,0}});Check(moved.lines.size()==1,"Exact displaced geometry inversion");for(auto p:moved.lines[0].points){Near(std::hypot(p.position[0]-2,p.position[1]),.4,4e-6,"Displacement shifts physical path");Near(p.vector[1],p.position[0]-2,2e-8,"Vector keeps reference-mesh field semantics");}for(int i=0;i<original.Size();++i)Near((*mesh.GetNodes())[i],original[i],0,"Flow leaves geometry immutable");
    for(const auto* file:{"disc-nurbs.mesh","ball-nurbs.mesh"}){mfem::Mesh rational((data+"/"+file).c_str(),1,1);const int dim=rational.Dimension();auto* nodes=rational.GetNodes();mfem::FiniteElementSpace space(&rational,nodes->FESpace()->FEColl(),dim);mfem::GridFunction vector(&space);for(int i=0;i<space.GetNDofs();++i){vector[space.DofToVDof(i,0)]=-(*nodes)[nodes->FESpace()->DofToVDof(i,1)];vector[space.DofToVDof(i,1)]=(*nodes)[nodes->FESpace()->DofToVDof(i,0)];if(dim==3)vector[space.DofToVDof(i,2)]=0;}auto flow=BuildStreamlines(rational,vector,{},options,{{.4,0,0}});Check(flow.lines.size()==1&&flow.lines[0].points.size()>60,"Rational NURBS flow traverses exact patch geometry");for(auto p:flow.lines[0].points)Near(std::hypot(p.position[0],p.position[1]),.4,4e-6,"Rational circular trajectory");auto direct=options;direct.seed_count=73;auto cloud=SampleVectorField(rational,vector,{},direct);Check(cloud.points.size()==73,"Direct count on exact 2D/3D NURBS geometry");for(const auto& point:cloud.points){Near(point.vector[0],-point.position[1],2e-10,"Rational direct seed vector x");Near(point.vector[1],point.position[0],2e-10,"Rational direct seed vector y");}Check(rational.NURBSext!=nullptr,"NURBS representation retained");}
    auto embedded=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);embedded.SetCurvature(2,false,3);auto embedded_field=Constant(embedded);Check(!SampleVectorField(embedded,*embedded_field).points.empty(),"Embedded surface arrows use exact geometry seeds");bool rejected=false;try{BuildStreamlines(embedded,*embedded_field);}catch(const std::invalid_argument& error){rejected=std::string(error.what()).find("tangential")!=std::string::npos;}Check(rejected,"Unsupported embedded-surface integration is explicit");
    std::cout<<"PASS P5 cube-to-spheroid, exact displaced maps, rational 2D/3D NURBS, and explicit embedded-surface limitation\n";
}
void LocatorScaling(){
    auto mesh=mfem::Mesh::MakeCartesian3D(10,10,10,mfem::Element::HEXAHEDRON);auto field=Constant(mesh);FlowOptions options;options.seed_resolution=8;options.max_seeds=32;options.max_length=.2;options.step_size=.02;options.max_steps=20;
    auto begin=std::chrono::steady_clock::now();auto result=BuildStreamlines(mesh,*field,{},options);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();Check(result.lines.size()>20,"Many paths on 1000-cell mesh");Check(result.diagnostics.location_tests<result.diagnostics.evaluations*20,"Coherent/spatial search avoids full mesh scan per query");std::cout<<"PASS 1000-cell flow locator: "<<result.lines.size()<<" paths, "<<result.diagnostics.evaluations<<" evaluations, "<<result.diagnostics.location_tests<<" inversions, "<<ms<<" ms\n";
}
}
int main(int argc,char** argv){try{DenseAndConstant();DirectSeedCounts();RotationsAndSpaces();SelectionAndLimits();CurvedDisplacedAndRational(argc>1?argv[1]:"subprojects/mfem/data");LocatorScaling();std::cout<<"All flow tests passed\n";return 0;}catch(const std::exception& error){std::cerr<<"Flow test failed: "<<error.what()<<'\n';return 1;}}
