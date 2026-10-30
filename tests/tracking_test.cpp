#include "libRayfem/frame/tracking.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
using namespace RayFEM;
void Check(bool condition, const std::string& message) { if (!condition) throw std::runtime_error(message); }
void Near(double a,double b,double tolerance,const std::string& message) {
    if (!std::isfinite(a)||std::abs(a-b)>tolerance) { std::ostringstream out;out<<message<<": "<<a<<" vs "<<b;throw std::runtime_error(out.str()); }
}
std::unique_ptr<mfem::GridFunction> Function(mfem::Mesh& mesh,mfem::FiniteElementCollection* collection,int components=1,int ordering=mfem::Ordering::byNODES) {
    auto* space=new mfem::FiniteElementSpace(&mesh,collection,components,ordering);
    auto field=std::make_unique<mfem::GridFunction>(space);field->MakeOwner(collection);return field;
}
std::shared_ptr<RenderFrame> Frame(std::shared_ptr<mfem::Mesh> mesh,unsigned index=0,std::uint64_t session=1,std::uint64_t revision=1) {
    auto frame=std::make_shared<RenderFrame>();frame->mesh=std::move(mesh);frame->mesh_id=7;frame->session_id=session;frame->mesh_revision=revision;
    frame->metadata.frame_index=index;frame->metadata.sim_time=.125*index;return frame;
}
void Scalar(RenderFrame& frame,double value,int order=2,const std::string& name="temperature") {
    auto field=Function(*frame.mesh,new mfem::H1_FECollection(order,frame.mesh->Dimension()));*field=value;frame.fields.push_back({name,std::move(field)});
}
MeasurementSample Measure(RenderFrame& frame,MeasurementKind kind,FieldView view=FieldView::Value,int component=0) {
    TrackingRequest request;request.field_name="temperature";request.kind=kind;request.view=view;request.component=component;
    FrameTracker tracker(request);Check(tracker.append(frame),"First frame appended");return tracker.samples().back();
}
void RequireValid(const MeasurementSample& sample) { Check(sample.valid,"Measurement valid: "+sample.error); }

void DofsAndIdentity() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true));
    TrackingRequest request;request.field_name="vector";request.kind=MeasurementKind::DofCoefficient;request.dof=2;request.component=1;
    FrameTracker tracker(request,3);
    for(unsigned index=0;index<5;++index) {
        auto frame=Frame(mesh,index);
        auto field=Function(*mesh,new mfem::H1_FECollection(2,2),2,index%2?mfem::Ordering::byVDIM:mfem::Ordering::byNODES);
        *field=0.0;(*field)[field->FESpace()->DofToVDof(2,1)]=10+index;
        frame->fields.push_back({"vector",std::move(field)});
        if(index==0)tracker.bindDof(*frame);
        Check(tracker.append(*frame),"Distinct DOF frame appended");
        RequireValid(tracker.samples().back());Near(tracker.samples().back().value,10+index,0,"Ordering-aware vector coefficient");
        Check(!tracker.append(*frame),"Duplicate does not append");
        std::weak_ptr<RenderFrame> weak=frame;frame.reset();Check(weak.expired(),"Tracker must not retain full frames");
    }
    Check(tracker.samples().size()==3 && tracker.samples().front().frame==2,"Bounded series evicts oldest values");
    auto stale=Frame(mesh,0);Check(!tracker.append(*stale),"Evicted duplicate remains suppressed by identity high-water mark");
    auto missing=Frame(mesh,5);tracker.append(*missing);Check(!tracker.samples().back().valid && std::isnan(tracker.samples().back().value),"Missing field is a gap");
    auto changed=Frame(mesh,6);auto changed_field=Function(*mesh,new mfem::H1_FECollection(3,2),2);*changed_field=5.0;changed->fields.push_back({"vector",std::move(changed_field)});
    tracker.append(*changed);Check(!tracker.samples().back().valid && tracker.samples().back().error.find("association changed")!=std::string::npos,"Changed FE order is not the same DOF");
    auto other=Frame(mesh,0,99);auto f=Function(*mesh,new mfem::H1_FECollection(2,2),2);*f=42.0;other->fields.push_back({"vector",std::move(f)});
    tracker.append(*other);Check(!tracker.samples().back().valid,"DOF does not silently bind a new session");
    tracker.clear();tracker.bindDof(*other);tracker.append(*other);RequireValid(tracker.samples().back());Near(tracker.samples().back().value,42,0,"Explicit rebinding");
    auto revised=Frame(mesh,1,99,2);auto r=Function(*mesh,new mfem::H1_FECollection(2,2),2);*r=52.0;revised->fields.push_back({"vector",std::move(r)});
    tracker.append(*revised);Check(!tracker.samples().back().valid,"Mesh revision changes create DOF gap");
    FrameTracker backfill(request);backfill.bindDof(*other);auto old=Frame(mesh,0,1);auto o=Function(*mesh,new mfem::H1_FECollection(2,2),2);*o=2.0;old->fields.push_back({"vector",std::move(o)});
    backfill.append(*old);backfill.append(*other);Check(!backfill.samples()[0].valid&&backfill.samples()[1].valid,"Clicked-current DOF binds before backfill");
    TrackingRequest local_request;local_request.field_name="local";FrameTracker local(local_request);
    auto original=Frame(mesh);auto original_field=Function(*mesh,new mfem::L2_FECollection(1,2));*original_field=1.;original->fields.push_back({"local",std::move(original_field)});local.append(*original);
    auto reordered_mesh=std::make_shared<mfem::Mesh>(*mesh);auto* vertices=reordered_mesh->GetElement(0)->GetVertices();
    std::rotate(vertices,vertices+1,vertices+4); // Same global L2 numbering, changed reference orientation.
    auto reordered=Frame(reordered_mesh,1);auto reordered_field=Function(*reordered_mesh,new mfem::L2_FECollection(1,2));*reordered_field=2.;reordered->fields.push_back({"local",std::move(reordered_field)});local.append(*reordered);
    Check(!local.samples().back().valid,"Changed element reference orientation cannot silently reuse L2 DOF identity");
    std::cout<<"PASS ordering-aware DOFs, immutable association, gaps, dedup, bounded lifetime\n";
}
void PhysicalAggregates() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true));
    mesh->SetCurvature(2);
    mfem::VectorFunctionCoefficient bend(2,[](const mfem::Vector& x,mfem::Vector& y){y=x;y[0]+=.2*x[0]*(1-x[0]);});mesh->Transform(bend);
    auto frame=Frame(mesh);auto field=Function(*mesh,new mfem::H1_FECollection(2,2));
    mfem::FunctionCoefficient x([](const mfem::Vector& p){return p[0];});field->ProjectCoefficient(x);frame->fields.push_back({"temperature",std::move(field)});
    auto integral=Measure(*frame,MeasurementKind::Integral);RequireValid(integral);Near(integral.value,.5,1e-11,"Curved physical integral");Near(integral.measure,1,1e-11,"Curved area");
    auto mean=Measure(*frame,MeasurementKind::Mean);RequireValid(mean);Near(mean.value,.5,1e-11,"Mean must weight physical Jacobian, not reference samples");
    auto norm=Measure(*frame,MeasurementKind::L2Norm);RequireValid(norm);Near(norm.value,std::sqrt(1.0/3),1e-11,"Physical L2 norm");
    auto low=Measure(*frame,MeasurementKind::Minimum),high=Measure(*frame,MeasurementKind::Maximum);RequireValid(low);RequireValid(high);
    Check(low.value>0 && high.value<1 && low.value<.1 && high.value>.9,"Min/max are explicitly quadrature sampled");
    auto displacement=Function(*mesh,new mfem::H1_FECollection(2,2),2,mfem::Ordering::byVDIM);
    mfem::VectorFunctionCoefficient stretch(2,[](const mfem::Vector& p,mfem::Vector& d){d.SetSize(2);d[0]=p[0];d[1]=0;});displacement->ProjectCoefficient(stretch);frame->displacement=std::move(displacement);
    const mfem::Vector before(*mesh->GetNodes());
    integral=Measure(*frame,MeasurementKind::Integral);RequireValid(integral);Near(integral.value,1,1e-11,"Deformed measure doubles integral");Near(integral.measure,2,1e-11,"Deformed area");
    mean=Measure(*frame,MeasurementKind::Mean);RequireValid(mean);Near(mean.value,.5,1e-11,"Field values stay reference-frame material values");
    norm=Measure(*frame,MeasurementKind::L2Norm);RequireValid(norm);Near(norm.value,std::sqrt(2.0/3),1e-11,"Deformed L2 Jacobian weights");
    for(int i=0;i<before.Size();++i)Near((*mesh->GetNodes())[i],before[i],0,"Aggregation does not mutate nodes");
    TrackingRequest request;request.field_name="temperature";request.kind=MeasurementKind::Integral;request.use_displacement=false;
    FrameTracker original(request);original.append(*frame);Near(original.samples().back().value,.5,1e-11,"Displacement can be disabled");
    auto embedded=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true));embedded->SetCurvature(2,false,3);
    mfem::VectorFunctionCoefficient surface(3,[](const mfem::Vector& p,mfem::Vector& d){d=p;d[2]=std::sqrt(3.0)*p[0];});embedded->Transform(surface);
    auto surface_frame=Frame(embedded);Scalar(*surface_frame,3);auto area=Measure(*surface_frame,MeasurementKind::Integral);RequireValid(area);Near(area.measure,2,1e-11,"Embedded physical surface area");Near(area.value,6,1e-11,"Embedded surface integral");
    std::cout<<"PASS curved, deformed, and embedded physical mean/integral/L2 and sampled extrema\n";
}
void CachedLayoutUpdates() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true));
    auto frame=Frame(mesh);Scalar(*frame,4);
    TrackingRequest request;request.field_name="temperature";FrameTracker first(request);first.bindDof(*frame);
    Check(frame->fields[0].dof_layout_cache.has_value(),"DOF layout cache is attached to the owning frame field");
    const auto fingerprint=frame->fields[0].dof_layout_cache->fingerprint;
    FrameTracker second(request);second.bindDof(*frame);second.append(*frame);RequireValid(second.samples().back());
    Check(frame->fields[0].dof_layout_cache->fingerprint==fingerprint,"Several trackers share the same frame signature");
    auto& field=*frame->fields[0].gf;mesh->UniformRefinement();field.FESpace()->Update();field.Update();field=6.;
    FrameTracker updated(request);updated.bindDof(*frame);updated.append(*frame);RequireValid(updated.samples().back());
    Check(frame->fields[0].dof_layout_cache->fingerprint!=fingerprint &&
          frame->fields[0].dof_layout_cache->space_sequence==field.FESpace()->GetSequence() &&
          frame->fields[0].dof_layout_cache->mesh_sequence==mesh->GetSequence(),"MFEM sequence changes invalidate cached topology");
    std::cout<<"PASS shared bounded DOF signature cache and MFEM update invalidation\n";
}
void SessionsAndVectors() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON));
    TrackingRequest request;request.field_name="temperature";request.kind=MeasurementKind::Mean;FrameTracker series(request);
    for(unsigned session:{1,2})for(unsigned frame_id:{0,1}){auto frame=Frame(mesh,frame_id,session);Scalar(*frame,session+frame_id);series.append(*frame);RequireValid(series.samples().back());Near(series.samples().back().value,session+frame_id,1e-12,"Multiple-session series value");}
    Check(series.samples().size()==4 && series.samples()[2].session_id==2,"Same frame index in another session is distinct");
    auto spin_frame=Frame(mesh);auto spin=Function(*mesh,new mfem::ND_FECollection(2,3));
    mfem::VectorFunctionCoefficient rotation(3,[](const mfem::Vector& p,mfem::Vector& v){v.SetSize(3);v[0]=-p[1];v[1]=p[0];v[2]=0;});spin->ProjectCoefficient(rotation);spin_frame->fields.push_back({"temperature",std::move(spin)});
    auto curl=Measure(*spin_frame,MeasurementKind::Mean,FieldView::CurlMagnitude);RequireValid(curl);Near(curl.value,2,1e-10,"ND curl mean");
    auto unsupported=Measure(*spin_frame,MeasurementKind::Mean,FieldView::Divergence);Check(!unsupported.valid && !unsupported.error.empty(),"Unsupported vector derivative becomes gap");
    TrackingRequest dof;dof.field_name="temperature";dof.dof=0;FrameTracker coefficient(dof);coefficient.append(*spin_frame);RequireValid(coefficient.samples().back());Near(coefficient.samples().back().value,(*spin_frame->fields[0].gf)[0],0,"ND directional coefficient stays raw");
    dof.component=1;FrameTracker invalid(dof);invalid.append(*spin_frame);Check(!invalid.samples().back().valid,"ND physical components cannot index directional coefficients");
    auto flux_frame=Frame(mesh);auto flux=Function(*mesh,new mfem::RT_FECollection(1,3));mfem::VectorFunctionCoefficient radial(3,[](const mfem::Vector& p,mfem::Vector& v){v=p;});flux->ProjectCoefficient(radial);flux_frame->fields.push_back({"temperature",std::move(flux)});
    auto divergence=Measure(*flux_frame,MeasurementKind::Integral,FieldView::Divergence);RequireValid(divergence);Near(divergence.value,3,1e-10,"RT divergence integral");
    std::cout<<"PASS changing sessions and vector curl/divergence/directional coefficients\n";
}
class ProbeAlgorithm final:public ProfileAlgorithm {
public:std::string id()const override{return "custom";}std::string name()const override{return "custom probe";}
std::vector<ProfileSample> sample(mfem::Mesh&,mfem::GridFunction&,const std::vector<ProfileSample>& points,const RenderOptions&)const override{auto result=points;for(auto& point:result){point.valid=true;point.value=123;point.element=0;}return result;}
};
void ProbesAndCSV() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true));mesh->SetCurvature(2);
    auto frame=Frame(mesh,4);auto field=Function(*mesh,new mfem::H1_FECollection(2,2));mfem::FunctionCoefficient x([](const mfem::Vector& p){return p[0]+2*p[1];});field->ProjectCoefficient(x);frame->fields.push_back({"temperature, \"sensor\"",std::move(field)});
    TrackingRequest request;request.field_name="temperature, \"sensor\"";request.kind=MeasurementKind::PointProbe;request.probe={.25,.5,0};FrameTracker tracker(request);
    tracker.append(*frame);RequireValid(tracker.samples().back());Near(tracker.samples().back().value,1.25,1e-11,"Fixed-world point probe");
    auto missing=Frame(mesh,5);tracker.append(*missing);Check(!tracker.samples().back().valid,"Missing probe field is a gap");
    std::ostringstream csv;ExportTrackingCSV(csv,tracker);const auto csv_text=csv.str();Check(csv_text.find("temperature, \"\"sensor\"\"")!=std::string::npos,"Tracking CSV quotes metadata");Check(std::count(csv_text.begin(),csv_text.end(),'\n')==3,"Tracking CSV includes all samples");
    Check(csv_text.find("association")==std::string::npos && csv_text.find("is missing")!=std::string::npos,"CSV preserves invalid reasons");
    request.probe={3,3,0};FrameTracker outside(request);outside.append(*frame);Check(!outside.samples().back().valid,"Point outside mesh is an invalid gap");
    request.algorithm="custom";FrameTracker custom(request);custom.registry().registerAlgorithm(std::make_shared<ProbeAlgorithm>());custom.append(*frame);RequireValid(custom.samples().back());Near(custom.samples().back().value,123,0,"Tracking probe dispatches replaceable algorithm");
    std::cout<<"PASS fixed-world probes, algorithm dispatch, CSV escaping and gaps\n";
}
void Rational(const std::string& data) {
    auto mesh=std::make_shared<mfem::Mesh>((data+"/disc-nurbs.mesh").c_str(),1,1);auto frame=Frame(mesh);Scalar(*frame,1);
    TrackingRequest request;request.field_name="temperature";request.kind=MeasurementKind::Integral;request.quadrature_order=32;FrameTracker tracker(request);tracker.append(*frame);RequireValid(tracker.samples().back());Near(tracker.samples().back().value,8*std::acos(-1.0),1e-9,"Rational disk physical area");
    auto displacement=std::make_unique<mfem::GridFunction>(mesh->GetNodes()->FESpace());*displacement=*mesh->GetNodes();frame->displacement=std::move(displacement);frame->metadata.frame_index=1;tracker.append(*frame);RequireValid(tracker.samples().back());Near(tracker.samples().back().value,32*std::acos(-1.0),1e-8,"Rational deformed disk area");
    Check(mesh->NURBSext!=nullptr,"Tracking preserves original rational mesh");
    std::cout<<"PASS exact rational geometry weights and displaced rational area\n";
}
void RestrictedArchiveFields() {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true));
    mesh->GetElement(0)->SetAttribute(11);mesh->GetElement(1)->SetAttribute(17);mesh->SetAttributes();
    auto frame=Frame(mesh);
    auto field=Function(*mesh,new mfem::L2_FECollection(0,2));
    *field=0.;mfem::Array<int> dofs;
    field->FESpace()->GetElementDofs(0,dofs);(*field)[dofs[0]]=6.;
    frame->fields.push_back({"temperature",std::move(field)});
    frame->fields[0].supported_materials={11};
    auto mean=Measure(*frame,MeasurementKind::Mean);
    RequireValid(mean);Near(mean.value,6,1e-12,"Unsupported zero extension must not dilute archive field mean");
    auto integral=Measure(*frame,MeasurementKind::Integral);
    RequireValid(integral);Near(integral.value,3,1e-12,"Only represented materials enter archive field integral");
    Near(integral.measure,.5,1e-12,"Archive field measure excludes unsupported elements");
    TrackingRequest request;request.field_name="temperature";request.kind=MeasurementKind::PointProbe;
    request.probe={.75,.5,0};FrameTracker outside(request);outside.append(*frame);
    Check(!outside.samples().back().valid,"Probe in unsupported archive region is missing, not zero");
    request.kind=MeasurementKind::DofCoefficient;
    frame->fields[0].gf->FESpace()->GetElementDofs(1,dofs);request.dof=dofs[0];
    FrameTracker unsupported(request);unsupported.append(*frame);
    Check(!unsupported.samples().back().valid,"Unsupported archive DOF cannot be tracked as a physical zero");
    request.kind=MeasurementKind::Mean;request.excluded_materials={11};
    FrameTracker hidden(request);hidden.append(*frame);
    Check(!hidden.samples().back().valid,"User exclusions combine with intrinsic field support");
    hidden.setExcludedMaterials({});hidden.append(*frame);
    RequireValid(hidden.samples().back());Near(hidden.samples().back().value,6,1e-12,"Changing user selection preserves intrinsic field support");
    // Intrinsic support belongs to the saved frame, not the measurement
    // request: a later frame can legitimately define this field more widely.
    request.excluded_materials.clear();
    FrameTracker evolving(request);evolving.append(*frame);
    auto wider=Frame(mesh,1);
    auto wider_field=Function(*mesh,new mfem::L2_FECollection(0,2));
    wider_field->FESpace()->GetElementDofs(0,dofs);(*wider_field)[dofs[0]]=6.;
    wider_field->FESpace()->GetElementDofs(1,dofs);(*wider_field)[dofs[0]]=12.;
    wider->fields.push_back({"temperature",std::move(wider_field)});
    evolving.append(*wider);
    RequireValid(evolving.samples().back());
    Near(evolving.samples().back().value,9,1e-12,"Broader next-frame support must not inherit an earlier field exclusion");
    Near(evolving.samples().back().measure,1,1e-12,"Broader next-frame support restores the complete measure");
    Check(evolving.samples().front().excluded_materials==std::vector<int>{17} &&
          evolving.samples().back().excluded_materials.empty(),
          "Effective material selection is retained independently for each sample");
    std::ostringstream exported;ExportTrackingCSV(exported,evolving);
    std::istringstream rows(exported.str());std::string row;
    std::getline(rows,row);std::getline(rows,row);
    Check(row.ends_with(",\"17\",\"\",\"\"\r"),"CSV must record restricted saved-field support even without a user exclusion");
    std::getline(rows,row);
    Check(row.ends_with(",\"\",\"\"\r"),"CSV must record broader support independently in the next frame");
    evolving.setExcludedMaterials({17});evolving.append(*wider);
    RequireValid(evolving.samples().back());
    Near(evolving.samples().back().value,6,1e-12,"Explicit user exclusions remain active when field support widens");
    Check(evolving.samples().back().excluded_materials==std::vector<int>{17},
          "Effective sample metadata includes explicit user exclusions");
    std::cout<<"PASS restricted archive field means, integrals, probes and DOF support\n";
}
}
int main(int argc,char** argv){try{DofsAndIdentity();CachedLayoutUpdates();PhysicalAggregates();SessionsAndVectors();ProbesAndCSV();RestrictedArchiveFields();Rational(argc>1?argv[1]:"subprojects/mfem/data");std::cout<<"All tracking tests passed\n";return 0;}catch(const std::exception& error){std::cerr<<"Tracking test failed: "<<error.what()<<'\n';return 1;}}
