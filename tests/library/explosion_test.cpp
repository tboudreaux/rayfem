#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/explosion.hpp"
#include "libRayfem/profile/sampler.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(bool value,const std::string& message) { if(!value)throw std::runtime_error(message); }
void near(double actual,double expected,double tolerance,const std::string& message) {
    if(!std::isfinite(actual) || std::abs(actual-expected)>tolerance)
        throw std::runtime_error(message+": "+std::to_string(actual)+" vs "+std::to_string(expected));
}
double coordinate(Vector3 p,int d) { return d==0?p.x:d==1?p.y:p.z; }
double area(const RayFEM::SurfaceData& s,bool cap=false,double x=0) {
    double sum=0;
    for(std::size_t i=0;i<s.positions.size();i+=9) {
        if(cap && (std::abs(s.positions[i]-x)>2e-6 || std::abs(s.positions[i+3]-x)>2e-6 ||
                   std::abs(s.positions[i+6]-x)>2e-6))continue;
        double a[3],b[3];
        for(int d=0;d<3;++d) {a[d]=s.positions[i+3+d]-s.positions[i+d];b[d]=s.positions[i+6+d]-s.positions[i+d];}
        sum+=.5*std::hypot(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
    }
    return sum;
}
void unchanged(const mfem::Vector& original,const mfem::Vector& current) {
    check(original.Size()==current.Size(),"Node count changed");
    for(int i=0;i<original.Size();++i)near(current[i],original[i],0,"Explosion mutated physical geometry");
}
void cartesian(int dim) {
    auto mesh=dim==2?mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true,2,1):
                     mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.GetElement(0)->SetAttribute(1);mesh.GetElement(1)->SetAttribute(2);mesh.SetAttributes();
    mesh.SetCurvature(2);
    const mfem::Vector original(*mesh.GetNodes());
    mfem::H1_FECollection fec(1,dim);mfem::FiniteElementSpace space(&mesh,&fec);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x){return x[0]+2*x[1]+(x.Size()>2?3*x[2]:0);});
    field.ProjectCoefficient(ramp);
    RayFEM::RenderOptions options;options.refinement=2;
    const auto joined=RayFEM::BuildSurface(mesh,&field,options);
    options.element_explosion=.2;
    const auto separated=RayFEM::BuildSurface(mesh,&field,options);
    near(area(joined),dim==2?2:10,2e-6,"Joined area");
    near(area(separated),(dim==2?2:12)*.64,3e-6,"Separated cell areas");
    if(dim==3)check(separated.positions.size()*5==joined.positions.size()*6,"Both interior face traces must appear");
    near(separated.bounds.min.x,.1,2e-7,"First cell left edge");
    near(separated.bounds.max.x,1.9,2e-7,"Last cell right edge");
    bool firstInterface=false,secondInterface=false;
    for(std::size_t i=0;i<separated.values.size();++i) {
        const int material=separated.material_ids[i];const double cx=material==1?.5:1.5;
        const float* p=&separated.positions[3*i];
        check(material==1?p[0]<=.900001:p[0]>=1.099999,"Visible cells must have a gap");
        const double x=cx+(p[0]-cx)/.8,y=.5+(p[1]-.5)/.8,z=dim==3?.5+(p[2]-.5)/.8:0;
        near(separated.values[i],x+2*y+3*z,9e-7,"Explosion must preserve physical FE values");
        if(dim==3 && separated.boundary_ids[i]==0) {
            if(material==1 && std::abs(p[0]-.9)<1e-6) {firstInterface=true;near(separated.normals[3*i],1,1e-6,"First interior normal");}
            if(material==2 && std::abs(p[0]-1.1)<1e-6) {secondInterface=true;near(separated.normals[3*i],-1,1e-6,"Second interior normal");}
        }
    }
    if(dim==3)check(firstInterface&&secondInterface,"Synthetic interior face IDs and opposing normals");
    const auto outlines=RayFEM::BuildElementBoundaries(mesh,options);
    check(outlines.size()==std::size_t((dim==2?8:24)*options.refinement),"Outlines need a separate edge copy for each cell");
    for(const auto& line:outlines)for(auto p:{line.a,line.b})
        check(line.material_id==1?p.x<=.900001:p.x>=1.099999,"Edge outline crossed exploded gap");
    auto dofs=RayFEM::SampleDofs(mesh,field,options);
    check(dofs.size()==std::size_t(dim==2?8:16),"Shared DOFs need one rendered copy per element");
    for(const auto& p:dofs) {
        const double cx=p.element==0?.5:1.5;
        const double x=cx+(p.position.x-cx)/.8,y=.5+(p.position.y-.5)/.8,z=dim==3?.5+(p.position.z-.5)/.8:0;
        near(p.value,x+2*y+3*z,9e-7,"DOF physical value");
        near(p.coefficient,field[p.index],0,"DOF coefficient identity");
    }
    std::vector<bool> retained{true,false};
    check(RayFEM::SampleDofs(mesh,field,options,20000,&retained).size()==std::size_t(dim==2?4:8),"Whole-cell clipping and exploded DOFs");
    RayFEM::ProfileRegistry profiles;
    auto locations=RayFEM::MakeRaySamples({.05,.5,dim==3?.5:0},{1,0,0},1.9,5);
    auto samples=profiles.sample("coherent",mesh,field,locations,options);
    for(const auto& p:samples) {check(p.valid,"Display padding must not change physical profile coverage");near(p.value,p.position[0]+2*p.position[1]+3*p.position[2],1e-9,"Physical profile value");}
    options.excluded_materials={2};
    const auto selected=RayFEM::BuildSurface(mesh,&field,options);
    near(area(selected),(dim==2?1:6)*.64,3e-6,"Excluded cells stay excluded");
    for(int material:selected.material_ids)check(material==1,"Excluded material leaked");
    unchanged(original,*mesh.GetNodes());
}

void clipping() {
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.GetElement(1)->SetAttribute(2);mesh.SetAttributes();mesh.SetCurvature(2);
    mfem::H1_FECollection fec(2,3);mfem::FiniteElementSpace space(&mesh,&fec);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x){return x[0]+2*x[1]+3*x[2];});field.ProjectCoefficient(ramp);
    mfem::GridFunction displacement(mesh.GetNodes()->FESpace());displacement=0;
    for(int i=0;i<displacement.FESpace()->GetNDofs();++i)displacement[displacement.FESpace()->DofToVDof(i,0)]=.25;
    RayFEM::RenderOptions options;options.refinement=3;options.element_explosion=.2;options.displacement=&displacement;
    for(auto algorithm:{RayFEM::CutAlgorithm::Refined,RayFEM::CutAlgorithm::Planar}) {
        auto cut=RayFEM::BuildClippedSurface(mesh,&field,options,{{.75,0,0},{1,0,0}},RayFEM::ClipMode::CutThrough,algorithm);
        near(area(cut.surface),2.56,4e-6,"Exploded displaced cut is closed");
        near(area(cut.surface,true,.75),.64,2e-6,"Exploded cap is shrunk with its cell");
        for(std::size_t i=0;i<cut.surface.values.size();++i) {
            const float* p=&cut.surface.positions[3*i];check(p[0]<=.750001,"Cut side leaked");
            const double x=.5+(p[0]-.75)/.8,y=.5+(p[1]-.5)/.8,z=.5+(p[2]-.5)/.8;
            near(cut.surface.values[i],x+2*y+3*z,1e-6,"Exploded cap field interpolation");
        }
        for(const auto& line:cut.element_boundaries)check(line.a.x<=.750001&&line.b.x<=.750001,"Cut outline clipping");
    }
    auto whole=RayFEM::BuildClippedSurface(mesh,&field,options,{{1.25,0,0},{1,0,0}},RayFEM::ClipMode::WholeElements);
    check(whole.retained_elements==std::vector<bool>({true,false}),"Whole-elements selection uses displayed geometry");
    near(area(whole.surface),3.84,3e-6,"Whole exploded cell has all six faces");
    options.excluded_materials={2};
    auto selected=RayFEM::BuildClippedSurface(mesh,&field,options,{{2.5,0,0},{1,0,0}});
    near(area(selected.surface),3.84,3e-6,"Clipping honors material exclusions");
    for(int id:selected.surface.material_ids)check(id==1,"Excluded clipped material");
}

void compareElement(const RayFEM::SurfaceData& baseline,const RayFEM::SurfaceData& separated,
                    int material,Vector3 center,double amount) {
    std::size_t index=0;
    for(std::size_t i=0;i<separated.values.size();++i)if(separated.material_ids[i]==material) {
        check(index<baseline.values.size(),"Exploded element has extra vertices");
        for(int d=0;d<3;++d) {
            near(separated.positions[3*i+d],coordinate(center,d)+(1-amount)*(baseline.positions[3*index+d]-coordinate(center,d)),4e-7,"High-order affine display geometry");
            near(separated.normals[3*i+d],baseline.normals[3*index+d],2e-6,"Uniform display shrink preserves curved normals");
        }
        near(separated.values[i],baseline.values[index],2e-10,"Curved FE sample unchanged");
        check(separated.boundary_ids[i]==baseline.boundary_ids[index],"Curved boundary ID unchanged");
        ++index;
    }
    check(index==baseline.values.size(),"Exploded element dropped vertices");
}
void curved() {
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);mesh.SetCurvature(4);
    mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& x,mfem::Vector& y){y=x;y[1]+=.8*x[0]*(1-x[0])*x[2];});mesh.Transform(bend);
    const mfem::Vector original(*mesh.GetNodes());
    mfem::H1_FECollection fec(2,3);mfem::FiniteElementSpace space(&mesh,&fec);mfem::GridFunction field(&space);field=2;
    RayFEM::RenderOptions options;options.refinement=6;
    const auto baseline=RayFEM::BuildSurface(mesh,&field,options);
    options.element_explosion=.3;
    const auto separated=RayFEM::BuildSurface(mesh,&field,options);
    compareElement(baseline,separated,1,{.5,.6,.5},.3);
    near(area(separated),area(baseline)*.49,2e-6,"Curved area scales quadratically");
    unchanged(original,*mesh.GetNodes());
}
void nonconforming() {
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.EnsureNCMesh();mfem::Array<int> refine(1);refine[0]=0;mesh.GeneralRefinement(refine);
    check(mesh.GetNE()==9,"One coarse and eight refined cubes");
    mesh.SetCurvature(3);
    std::vector<Vector3> centers;
    std::vector<double> volumes;
    for(int e=0;e<mesh.GetNE();++e) {
        mesh.GetElement(e)->SetAttribute(e+1);
        mfem::Vector center;
        mesh.GetElementTransformation(e)->Transform(mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e)),center);
        centers.push_back({float(center[0]),float(center[1]),float(center[2])});
        volumes.push_back(mesh.GetElementVolume(e));
    }
    mesh.SetAttributes();
    mfem::H1_FECollection fec(2,3);mfem::FiniteElementSpace space(&mesh,&fec);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x){return x[0]+2*x[1]+3*x[2];});field.ProjectCoefficient(ramp);
    RayFEM::RenderOptions options;options.refinement=3;options.element_explosion=.2;
    const auto surface=RayFEM::BuildSurface(mesh,&field,options);
    std::vector<double> actualVolume(mesh.GetNE(),0),actualArea(mesh.GetNE(),0);
    for(std::size_t i=0;i<surface.values.size();i+=3) {
        const int e=surface.material_ids[i]-1;check(e>=0&&e<mesh.GetNE(),"NC cell attribute");
        double p[3][3];
        for(int k=0;k<3;++k) {
            check(surface.material_ids[i+k]==e+1,"NC triangle must stay on its owning cell");
            for(int d=0;d<3;++d)p[k][d]=surface.positions[3*(i+k)+d]-coordinate(centers[e],d);
            const double x=centers[e].x+p[k][0]/.8,y=centers[e].y+p[k][1]/.8,z=centers[e].z+p[k][2]/.8;
            near(surface.values[i+k],x+2*y+3*z,1.5e-6,"NC trace value belongs to its physical parent");
        }
        double a[3],b[3];for(int d=0;d<3;++d){a[d]=p[1][d]-p[0][d];b[d]=p[2][d]-p[0][d];}
        const double cx=a[1]*b[2]-a[2]*b[1],cy=a[2]*b[0]-a[0]*b[2],cz=a[0]*b[1]-a[1]*b[0];
        actualArea[e]+=.5*std::hypot(cx,cy,cz);
        actualVolume[e]+=(p[0][0]*cx+p[0][1]*cy+p[0][2]*cz)/6;
    }
    for(int e=0;e<mesh.GetNE();++e) {
        near(actualVolume[e],volumes[e]*.512,1e-6,"Each exploded NC cell must be closed with outward faces");
        near(actualArea[e],6*std::pow(volumes[e],2.0/3)*.64,3e-6,"NC coarse-face fragments must cover each side exactly once");
    }
    const auto outlines=RayFEM::BuildElementBoundaries(mesh,options);
    std::vector<int> edgeSegments(mesh.GetNE(),0);
    for(const auto& line:outlines) {
        const int e=line.material_id-1;check(e>=0&&e<mesh.GetNE(),"NC edge owning cell");
        ++edgeSegments[e];
        const double halfWidth=.4*std::cbrt(volumes[e]);
        for(auto p:{line.a,line.b}) {
            int boundaryCoordinates=0;
            for(int d=0;d<3;++d)
                if(std::abs(std::abs(coordinate(p,d)-coordinate(centers[e],d))-halfWidth)<2e-6)++boundaryCoordinates;
            check(boundaryCoordinates>=2,"NC face fragments must not create artificial element edges");
        }
    }
    for(int count:edgeSegments)check(count==12*options.refinement,"Each exploded NC cube has exactly 12 original edges");
}
void nurbs(const std::string& data) {
    mfem::Mesh mesh((data+"/ball-nurbs.mesh").c_str(),1,1);
    check(mesh.NURBSext,"NURBS fixture");
    for(int e=0;e<mesh.GetNE();++e)mesh.GetElement(e)->SetAttribute(e+1);mesh.SetAttributes();
    const mfem::Vector original(*mesh.GetNodes());
    auto* geometry=mesh.GetNodes()->FESpace();
    mfem::FiniteElementSpace space(&mesh,geometry->FEColl());mfem::GridFunction field(&space);
    mfem::GridFunction displacement(geometry);
    for(int i=0;i<geometry->GetNDofs();++i) {
        field[i]=0;
        for(int d=0;d<3;++d) {
            const int j=geometry->DofToVDof(i,d);
            field[i]+=(d+1)*original[j];displacement[j]=.2*original[j]+.05*(d+1);
        }
    }
    RayFEM::RenderOptions options;options.refinement=3;options.displacement=&displacement;options.element_explosion=.25;
    const auto separated=RayFEM::BuildSurface(mesh,&field,options);
    for(int e=0;e<mesh.GetNE();++e) {
        options.element_explosion=0;options.excluded_materials.clear();
        for(int other=0;other<mesh.GetNE();++other)if(other!=e)options.excluded_materials.push_back(other+1);
        const auto baseline=RayFEM::BuildSurface(mesh,&field,options);
        mfem::Vector center;mesh.GetElementTransformation(e)->Transform(mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e)),center);
        Vector3 moved{float(1.2*center[0]+.05),float(1.2*center[1]+.10),float(1.2*center[2]+.15)};
        compareElement(baseline,separated,e+1,moved,.25);
    }
    options.excluded_materials.clear();options.element_explosion=.25;
    for(auto algorithm:{RayFEM::CutAlgorithm::Refined,RayFEM::CutAlgorithm::Planar}) {
        auto cut=RayFEM::BuildClippedSurface(mesh,&field,options,{{.17,0,0},{1,0,0}},RayFEM::ClipMode::CutThrough,algorithm);
        check(!cut.surface.positions.empty(),"Exploded displaced NURBS clip");
        for(std::size_t i=0;i<cut.surface.positions.size();i+=3)check(cut.surface.positions[i]<=.170002,"NURBS cut side");
    }
    unchanged(original,*mesh.GetNodes());check(mesh.NURBSext,"Display explosion replaced rational geometry");
}
void validation() {
    auto mesh=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    for(double value:{-.01,1.0,std::numeric_limits<double>::quiet_NaN()}) {
        RayFEM::RenderOptions options;options.element_explosion=value;
        bool rejected=false;try {RayFEM::BuildSurface(mesh,nullptr,options);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"Invalid explosion rejected");
        rejected=false;try {RayFEM::BuildElementBoundaries(mesh,options);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"Invalid explosion in outlines rejected");
    }
}
}
int main(int argc,char** argv) {
    try {
        cartesian(2);cartesian(3);clipping();curved();nonconforming();nurbs(argc>1?argv[1]:"subprojects/mfem/data");validation();
        std::cout<<"PASS element explosion: topology, DOFs, physical analysis, clipping, attributes, curvature and NURBS\n";
    }catch(const std::exception& error){std::cerr<<"Explosion test failed: "<<error.what()<<'\n';return 1;}
}
