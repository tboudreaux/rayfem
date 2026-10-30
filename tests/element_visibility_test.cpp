#include "libRayfem/frame/tracking.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/profile/surface_trace.hpp"
#include "libRayfem/vector/flow.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using namespace RayFEM;
void check(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
void near(double actual,double expected,double tolerance,const std::string& message){
    if(!std::isfinite(actual)||std::abs(actual-expected)>tolerance)
        throw std::runtime_error(message+": "+std::to_string(actual)+" vs "+std::to_string(expected));
}
template<class F> void rejects(F function,const std::string& message){
    bool rejected=false;try{function();}catch(const std::invalid_argument&){rejected=true;}check(rejected,message);
}
std::unique_ptr<mfem::GridFunction> field(mfem::Mesh& mesh,mfem::FiniteElementCollection* fec,int components=1){
    auto* space=new mfem::FiniteElementSpace(&mesh,fec,components);
    auto result=std::make_unique<mfem::GridFunction>(space);result->MakeOwner(fec);return result;
}
RenderFrame fixture(int dimension=3){
    RenderFrame frame;frame.session_id=4;frame.mesh_id=8;frame.mesh_revision=2;
    frame.mesh=std::make_shared<mfem::Mesh>(dimension==3?
        mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON,3,1,1):
        mfem::Mesh::MakeCartesian2D(3,1,mfem::Element::QUADRILATERAL,true,3,1));
    frame.mesh->SetCurvature(3);
    auto scalar=field(*frame.mesh,new mfem::L2_FECollection(0,dimension));
    mfem::Array<int> dofs;
    for(int e=0;e<3;++e){scalar->FESpace()->GetElementDofs(e,dofs);(*scalar)[dofs[0]]=e==1?100:e+1;}
    frame.fields.push_back({"scalar",std::move(scalar)});
    auto vector=field(*frame.mesh,new mfem::H1_FECollection(1,dimension),dimension);
    mfem::VectorFunctionCoefficient flow(dimension,[dimension](const mfem::Vector&,mfem::Vector& value){value.SetSize(dimension);value=0.;value[0]=1.;});
    vector->ProjectCoefficient(flow);frame.fields.push_back({"vector",std::move(vector)});
    return frame;
}
double volume(const SurfaceData& s){
    double sum=0;
    for(std::size_t i=0;i<s.positions.size();i+=9){const float* p=&s.positions[i];
        sum+=double(p[0])*(p[4]*p[8]-p[5]*p[7])+double(p[1])*(p[5]*p[6]-p[3]*p[8])+double(p[2])*(p[3]*p[7]-p[4]*p[6]);}
    return sum/6;
}
void rendering(){
    auto frame=fixture();auto& mesh=*frame.mesh;auto& scalar=*frame.fields[0].gf;
    RenderOptions options;options.refinement=2;options.excluded_elements={1};
    check(IsElementIncluded(mesh,0,options)&&!IsElementIncluded(mesh,1,options)&&IsElementIncluded(mesh,2,options),"Hiding a cell must not hide its whole material");
    const auto surface=BuildSurface(mesh,&scalar,options);
    near(volume(surface),2,2e-6,"Hidden middle cell leaves two closed volumes");
    check(surface.element_ids.size()==surface.positions.size()/9,"Triangle element identities");
    check(std::none_of(surface.element_ids.begin(),surface.element_ids.end(),[](int e){return e==1;}),"Hidden cell cannot remain in picking geometry");
    check(std::count(surface.boundary_ids.begin(),surface.boundary_ids.end(),0)>0,"Hiding exposes internal faces");
    for(const auto& line:BuildElementBoundaries(mesh,options))
        check(!((line.a.x>1.000001&&line.a.x<1.999999)||(line.b.x>1.000001&&line.b.x<1.999999)),"Hidden cell outlines disappear");
    for(auto mode:{ClipMode::CutThrough,ClipMode::WholeElements})for(auto algorithm:{CutAlgorithm::Refined,CutAlgorithm::Planar}){
        const auto clipped=BuildClippedSurface(mesh,&scalar,options,{{1.5,0,0},{1,0,0}},mode,algorithm);
        near(volume(clipped.surface),1,2e-6,"Plane and element visibility combine");
        for(int e:clipped.surface.element_ids)check(e==0,"Clipping cap and exposed-face owner excludes hidden cell");
    }
    options.element_explosion=.2;
    near(volume(BuildSurface(mesh,&scalar,options)),2*.512,2e-6,"Explosion preserves individual exclusions");
    for(const auto& point:SampleDofs(mesh,*frame.fields[1].gf,options))check(point.element!=1,"Hidden DOF supporting parents disappear");
    for(const auto& point:SampleVectors(mesh,*frame.fields[1].gf,options))check(point.element!=1,"Hidden vector samples disappear");
    mesh.GetElement(2)->SetAttribute(7);mesh.SetAttributes();options.excluded_materials={7};
    near(volume(BuildSurface(mesh,&scalar,options)),.512,2e-6,"Material and element exclusions combine");
    options.excluded_elements.clear();options.excluded_materials.clear();options.element_explosion=0;
    near(volume(BuildSurface(mesh,&scalar,options)),3,2e-6,"Unhide restores original joined exterior");
}
void analysis(){
    for(int dimension:{2,3}){
        auto frame=fixture(dimension);auto& mesh=*frame.mesh;auto& scalar=*frame.fields[0].gf;
        RenderOptions options;options.excluded_elements={1};
        auto range=EvaluateFieldRange(mesh,scalar,options);
        near(range.min_value,1,1e-12,"Range excludes hidden extreme");near(range.max_value,3,1e-12,"Range excludes hidden extreme");
        const double z=dimension==3?.5:0;
        const auto locations=MakeRaySamples({.5,.5,z},{1,0,0},2,3);
        ProfileRegistry profiles;
        for(const auto& method:profiles.algorithms()){
            const auto result=profiles.sample(method.id,mesh,scalar,locations,options);
            check(result[0].valid&&!result[1].valid&&result[2].valid,"Profiles retain an explicit hidden-cell gap");
            near(result[0].value,1,1e-12,"Visible first cell value");near(result[2].value,3,1e-12,"Visible last cell value");
            ProfileSample shared;shared.position={1,.5,z};options.excluded_elements={0};
            const auto tie=profiles.sample(method.id,mesh,scalar,{shared},options);
            check(tie[0].valid&&tie[0].element==1,"Shared interface selects an included parent");
            near(tie[0].value,100,1e-12,"Discontinuous trace follows visible parent");options.excluded_elements={1};
        }
        TrackingRequest request;request.field_name="scalar";request.excluded_elements={1};
        for(const auto kind:{MeasurementKind::Integral,MeasurementKind::Mean,MeasurementKind::L2Norm,MeasurementKind::Minimum,MeasurementKind::Maximum}){
            request.kind=kind;FrameTracker tracker(request);tracker.append(frame);const auto& sample=tracker.samples().back();
            check(sample.valid,"Visible-element aggregate");near(sample.measure,2,2e-10,"Visible physical measure");
            const double expected=kind==MeasurementKind::Integral?4:kind==MeasurementKind::Mean?2:kind==MeasurementKind::L2Norm?std::sqrt(10.):kind==MeasurementKind::Minimum?1:3;
            near(sample.value,expected,2e-10,"Aggregate ignores hidden cell");
        }
        request.kind=MeasurementKind::PointProbe;request.probe={1.5,.5,z};FrameTracker probe(request);probe.append(frame);
        check(!probe.samples().back().valid,"Hidden probe becomes gap");
        FlowOptions flow;flow.seed_resolution=3;flow.max_seeds=40;flow.max_steps=80;flow.max_length=4;flow.step_size=.025;
        auto cloud=SampleVectorField(mesh,*frame.fields[1].gf,options,flow);check(!cloud.points.empty(),"Visible flow seeds");
        for(const auto& point:cloud.points)check(point.element!=1,"Vector cloud excludes hidden cell");
        auto paths=BuildStreamlines(mesh,*frame.fields[1].gf,options,flow,{{.5,.5,z},{1.5,.5,z}});
        check(!paths.lines.empty(),"Visible streamline remains");
        for(const auto& path:paths.lines)for(const auto& point:path.points){check(point.element!=1,"Streamline cannot enter hidden cell");check(point.position[0]<=1.000001,"Streamline stops at hidden neighbor");}
        options.excluded_elements={0,1,2};
        check(BuildSurface(mesh,&scalar,options).positions.empty(),"All cells hidden");
        check(SampleVectorField(mesh,*frame.fields[1].gf,options,flow).points.empty(),"All-hidden vector cloud");
        rejects([&]{EvaluateFieldRange(mesh,scalar,options);},"All-hidden range is explicitly unavailable");
    }
}
class IgnoreProfileSelection final:public ProfileAlgorithm {
public:
    std::string id()const override{return "ignore";}std::string name()const override{return "Ignore selection";}
    std::vector<ProfileSample> sample(mfem::Mesh&,mfem::GridFunction&,const std::vector<ProfileSample>& locations,const RenderOptions&)const override{
        auto result=locations;for(auto& p:result){p.valid=true;p.element=1;p.value=100;}return result;
    }
};
class IgnoreSurfaceSelection final:public SurfaceTraceAlgorithm {
public:
    std::string id()const override{return "ignore";}std::string name()const override{return "Ignore selection";}
    std::vector<SurfaceTraceSample> sample(mfem::Mesh&,mfem::GridFunction&,const std::vector<SurfaceTraceRay>& rays,const RenderOptions&)const override{
        std::vector<SurfaceTraceSample> result(rays.size());for(auto& p:result){p.valid=true;p.element=1;p.value=100;}return result;
    }
};
void tracesAndPlugins(){
    auto frame=fixture();auto& mesh=*frame.mesh;auto& scalar=*frame.fields[0].gf;
    RenderOptions options;options.excluded_elements={1};SurfaceTraceRegistry traces;
    const std::vector<SurfaceTraceRay> rays{{{.5,.5,-1},{0,0,1},0},{{1.5,.5,-1},{0,0,1},1},{{2.5,.5,-1},{0,0,1},2}};
    for(const auto& method:traces.algorithms()){
        const auto result=traces.sample(method.id,mesh,scalar,rays,options);
        check(result[0].valid&&!result[1].valid&&result[2].valid,"Surface traces skip hidden original boundary faces");
    }
    const auto locations=MakeRaySamples({.5,.5,.5},{1,0,0},2,3);
    ProfileRegistry profiles;profiles.registerAlgorithm(std::make_shared<IgnoreProfileSelection>());
    traces.registerAlgorithm(std::make_shared<IgnoreSurfaceSelection>());
    const auto profile=profiles.sample("ignore",mesh,scalar,locations,options);
    const auto trace=traces.sample("ignore",mesh,scalar,rays,options);
    check(!profile.front().valid&&profile.front().element==-1&&std::isnan(profile.front().value),"Profile plugin cannot bypass hidden cells");
    check(!trace.front().valid&&trace.front().element==-1&&std::isnan(trace.front().value),"Surface plugin cannot bypass hidden cells");
    ProfileMetadata metadata;metadata.excluded_elements={1};std::ostringstream csv;
    ExportProfileCSV(csv,profile,metadata);check(csv.str().find("excluded_elements")!=std::string::npos&&csv.str().find(",\"1\"\r\n")!=std::string::npos,"Profile CSV records cell selection");
    csv.str("");csv.clear();ExportSurfaceTraceCSV(csv,trace,metadata);
    check(csv.str().find("excluded_elements")!=std::string::npos&&csv.str().find(",\"1\"\n")!=std::string::npos,"Surface CSV records cell selection");
}
void trackingScope(){
    auto frame=fixture(2);
    auto nodal=field(*frame.mesh,new mfem::H1_FECollection(1,2));*nodal=9;
    int shared=-1,exclusive=-1;
    for(const auto& point:SampleDofs(*frame.mesh,*nodal)){
        if(std::abs(point.position.x-1)<1e-6)shared=point.index;
        if(std::abs(point.position.x)<1e-6)exclusive=point.index;
    }
    check(shared>=0&&exclusive>=0,"Shared and exclusive DOFs found");frame.fields.push_back({"nodal",std::move(nodal)});
    TrackingRequest request;request.field_name="nodal";request.excluded_elements={0};request.dof=shared;
    FrameTracker visible(request);visible.bindDof(frame);visible.append(frame);check(visible.samples().back().valid,"Shared DOF retains included support");
    request.dof=exclusive;FrameTracker hidden(request);hidden.bindDof(frame);hidden.append(frame);check(!hidden.samples().back().valid,"DOF with only hidden support becomes gap");
    hidden.setExcludedElements({});hidden.append(frame);check(hidden.samples().back().valid,"Unhide restores bound DOF without changing its association");
    request={};request.field_name="scalar";request.kind=MeasurementKind::Integral;FrameTracker tracker(request);
    tracker.setExcludedElements({1,1},ElementSelectionScope{frame.session_id,frame.mesh_id,frame.mesh_revision});
    check(tracker.request().excluded_elements==std::vector<int>{1},"Element selection canonicalized");
    tracker.append(frame);near(tracker.samples().back().value,4,1e-10,"Scoped hidden-element integral");
    ++frame.mesh_revision;tracker.append(frame);near(tracker.samples().back().value,104,1e-10,"Cell indices must not leak into another mesh revision");
    check(tracker.samples().back().excluded_elements.empty(),"Export metadata uses effective frame selection");
    std::ostringstream csv;ExportTrackingCSV(csv,tracker);check(csv.str().find("excluded_elements")!=std::string::npos&&csv.str().find(",\"1\",\"\"\r\n")!=std::string::npos,"Tracking CSV records element exclusions");
}
void invalidSelections(){
    auto frame=fixture();auto& mesh=*frame.mesh;auto& scalar=*frame.fields[0].gf;
    for(int index:{-1,mesh.GetNE()}){
        RenderOptions options;options.excluded_elements={index};
        rejects([&]{BuildSurface(mesh,&scalar,options);},"Out-of-range render exclusion");
        rejects([&]{BuildElementBoundaries(mesh,options);},"Out-of-range edge exclusion");
        rejects([&]{BuildClippedSurface(mesh,&scalar,options,{{.5,0,0},{1,0,0}});},"Out-of-range clip exclusion");
        rejects([&]{EvaluateFieldRange(mesh,scalar,options);},"Out-of-range field range exclusion");
        ProfileRegistry profiles;SurfaceTraceRegistry traces;
        rejects([&]{profiles.sample("coherent",mesh,scalar,{},options);},"Out-of-range profile exclusion");
        rejects([&]{traces.sample("curved",mesh,scalar,{},options);},"Out-of-range surface exclusion");
        rejects([&]{SampleVectorField(mesh,*frame.fields[1].gf,options);},"Out-of-range flow exclusion");
    }
    check(!IsElementIncluded(mesh,-1,{})&&!IsElementIncluded(mesh,mesh.GetNE(),{}),"Invalid candidate indices are safely rejected");
}
void compiledSelections(){
    auto frame=fixture();auto& mesh=*frame.mesh;
    RenderOptions options;options.excluded_elements={2,0,2};
    const ElementSelection first(mesh,options);
    check(!first.Includes(0)&&first.Includes(1)&&!first.Includes(2),"Unsorted duplicate exclusions compile correctly");
    check(!first.Includes(-1)&&!first.Includes(mesh.GetNE()),"Compiled selection rejects invalid candidates");
    options.excluded_elements[0]=1;
    const ElementSelection changed(mesh,options);
    check(!changed.Includes(1)&&first.Includes(1),"In-place vector mutation cannot leave a stale selection cache");
    auto copied=options;copied.excluded_elements={1};
    const ElementSelection independent(mesh,copied);
    check(independent.Includes(0)&&!independent.Includes(1)&&independent.Includes(2),"Copied render options compile independently");
    copied.excluded_elements.clear();copied.excluded_materials={7,7};
    mesh.GetElement(2)->SetAttribute(7);
    const ElementSelection material(mesh,copied);
    check(material.Includes(0)&&material.Includes(1)&&!material.Includes(2),"Material and index filters use the same snapshot");
    mesh.GetElement(2)->SetAttribute(1);
    const ElementSelection refreshed(mesh,copied);
    check(refreshed.Includes(2)&&!material.Includes(2),"New snapshot observes changed mesh attributes");
    const ElementSelection all(mesh,{});
    for(int e=0;e<mesh.GetNE();++e)check(all.Includes(e),"Empty exclusions retain every cell");
}
}
int main(){try{rendering();analysis();tracesAndPlugins();trackingScope();invalidSelections();compiledSelections();std::cout<<"PASS individual element visibility across rendering, analysis, plugins, tracking scopes and exports\n";}
catch(const std::exception& error){std::cerr<<"Element visibility test failed: "<<error.what()<<'\n';return 1;}}
