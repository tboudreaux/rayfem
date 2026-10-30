#include "libRayfem/frame/tracking.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using namespace RayFEM;
void Check(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
void Near(double a,double b,double eps,const std::string& message){if(!std::isfinite(a)||std::abs(a-b)>eps){std::ostringstream s;s<<message<<": "<<a<<" vs "<<b;throw std::runtime_error(s.str());}}
std::unique_ptr<mfem::GridFunction> Field(mfem::Mesh& mesh,mfem::FiniteElementCollection* fec,int components=1){auto* fes=new mfem::FiniteElementSpace(&mesh,fec,components);auto field=std::make_unique<mfem::GridFunction>(fes);field->MakeOwner(fec);return field;}
std::shared_ptr<mfem::Mesh> ThreeMaterials(int dim=3){
    auto mesh=std::make_shared<mfem::Mesh>(dim==3?mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON,3,1,1):mfem::Mesh::MakeCartesian2D(3,1,mfem::Element::QUADRILATERAL,true,3,1));
    const int ids[]={2,7,42};for(int e=0;e<mesh->GetNE();++e){mfem::Vector c;mesh->GetElementTransformation(e)->Transform(mfem::Geometries.GetCenter(mesh->GetElementBaseGeometry(e)),c);mesh->GetElement(e)->SetAttribute(ids[std::clamp(int(c[0]),0,2)]);}
    for(int b=0;b<mesh->GetNBE();++b)mesh->GetBdrElement(b)->SetAttribute(13+3*b);mesh->SetAttributes();return mesh;
}
double Volume(const SurfaceData& s){double sum=0;for(std::size_t k=0;k<s.positions.size();k+=9){const auto* p=&s.positions[k];sum+=double(p[0])*(p[4]*p[8]-p[5]*p[7])+double(p[1])*(p[5]*p[6]-p[3]*p[8])+double(p[2])*(p[3]*p[7]-p[4]*p[6]);}return sum/6;}
void Colors(const SurfaceData& surface,SurfaceColoring coloring){
    Check(surface.material_ids.size()==surface.values.size()&&surface.boundary_ids.size()==surface.values.size(),"Per-vertex attribute metadata aligned");
    for(std::size_t i=0;i<surface.values.size();++i){const auto color=AttributeColor(coloring==SurfaceColoring::Material?surface.material_ids[i]:surface.boundary_ids[i]);Check(surface.colors[4*i]==color.r&&surface.colors[4*i+1]==color.g&&surface.colors[4*i+2]==color.b,"Stable categorical color for exact attribute ID");}
}
void RenderSelection(){
    auto mesh=ThreeMaterials();RenderOptions options;options.refinement=1;options.coloring=SurfaceColoring::Material;
    auto all=BuildSurface(*mesh,nullptr,options);Check(all.positions.size()/9==28,"Three contiguous hexes expose fourteen quads");Colors(all,options.coloring);Near(Volume(all),3,1e-6,"Original outward orientation");
    options.excluded_materials={7};auto selected=BuildSurface(*mesh,nullptr,options);Colors(selected,options.coloring);
    Check(selected.positions.size()/9==24,"Removing middle material exposes both neighboring faces");Near(Volume(selected),2,1e-6,"Exposed interfaces wind out of each retained region");
    Check(std::count(selected.boundary_ids.begin(),selected.boundary_ids.end(),0)==12,"Two synthetic interfaces have boundary ID zero");
    Check(std::find(selected.material_ids.begin(),selected.material_ids.end(),7)==selected.material_ids.end(),"Hidden material emits no surface");
    for(std::size_t i=0;i<selected.values.size();++i){const double x=selected.positions[3*i];Check(x<=1.000001||x>=1.999999,"Surface has no hidden-region vertices");}
    options.coloring=SurfaceColoring::Boundary;auto boundaries=BuildSurface(*mesh,nullptr,options);Colors(boundaries,options.coloring);
    const auto attrs=FaceBoundaryAttributes(*mesh);for(int id:boundaries.boundary_ids)Check(id==0||std::find(attrs.begin(),attrs.end(),id)!=attrs.end(),"Boundary colors preserve actual nonsequential MFEM attributes");
    auto lines=BuildElementBoundaries(*mesh,options);Check(!lines.empty(),"Selected region has element edges");for(const auto& line:lines)Check(line.material_id!=7,"Edges exclude hidden material");
    options.excluded_materials={2};auto side2=BuildSurface(*mesh,nullptr,options);Near(Volume(side2),2,1e-6,"Side2 exposed faces use reverse winding");
    for(std::size_t i=0;i<side2.values.size();++i)if(side2.boundary_ids[i]==0)Near(side2.normals[3*i],-1,1e-6,"New left interface faces outward");
    auto invalid=Field(*mesh,new mfem::ND_FECollection(1,3));*invalid=1.;options.view=FieldView::Divergence;
    Check(!BuildSurface(*mesh,invalid.get(),options).values.empty(),"Attribute coloring is independent of unavailable field operations");
    options.excluded_materials={2,7,42};auto empty=BuildSurface(*mesh,invalid.get(),options);Check(empty.values.empty()&&empty.positions.empty()&&empty.bounds.min.x==0&&empty.bounds.max.x==0,"All-off surface is empty with safe bounds");Check(BuildElementBoundaries(*mesh,options).empty(),"All-off boundaries empty");options.coloring=SurfaceColoring::Field;Check(BuildSurface(*mesh,invalid.get(),options).values.empty(),"All-off selection requires no unavailable derivative evaluation");
    std::cout<<"PASS nonsequential material/boundary IDs, exposed interfaces, winding and all-off rendering\n";
}
void ClippingAndOverlays(){
    auto mesh=ThreeMaterials();RenderOptions options;options.refinement=2;options.excluded_materials={7};options.coloring=SurfaceColoring::Boundary;
    for(auto algorithm:{CutAlgorithm::Refined,CutAlgorithm::Planar}){
        auto cut=BuildClippedSurface(*mesh,nullptr,options,{{0,.5f,0},{0,1,0}},ClipMode::CutThrough,algorithm);Near(Volume(cut.surface),1,2e-6,"Cut caps close selected half-volumes");Colors(cut.surface,options.coloring);
        Check(std::find(cut.surface.material_ids.begin(),cut.surface.material_ids.end(),7)==cut.surface.material_ids.end(),"Cut cap cannot reintroduce hidden material");
        bool cap=false;for(std::size_t i=0;i<cut.surface.values.size();++i)if(std::abs(cut.surface.positions[3*i+1]-.5)<1e-6&&cut.surface.normals[3*i+1]>.9){cap=true;Check(cut.surface.boundary_ids[i]==0,"Plane cap has synthetic boundary ID zero");}Check(cap,"Both cut algorithms create selected caps");
        for(const auto& line:cut.element_boundaries)Check(line.material_id!=7,"Cut section outlines exclude hidden regions");
    }
    auto whole=BuildClippedSurface(*mesh,nullptr,options,{{2.1f,0,0},{1,0,0}},ClipMode::WholeElements);Near(Volume(whole.surface),1,1e-6,"Whole-element clip combines plane and material selection");Check(std::count(whole.retained_elements.begin(),whole.retained_elements.end(),true)==1,"Whole selection mask combines both filters");
    auto vector=Field(*mesh,new mfem::H1_FECollection(2,3),3);*vector=1.;auto samples=SampleVectors(*mesh,*vector,options);Check(samples.size()==2,"Vectors only sample included elements");for(auto p:samples)Check(mesh->GetAttribute(p.element)!=7,"Vector parent included");
    auto dofs=SampleDofs(*mesh,*vector,options);for(const auto& p:dofs)Check(mesh->GetAttribute(p.element)!=7,"DOF marker selects included supporting parent");
    auto clipped_dofs=SampleDofs(*mesh,*vector,options,20000,&whole.retained_elements);for(const auto& p:clipped_dofs)Check(whole.retained_elements[p.element],"DOFs honor both material and whole-element masks");
    options.excluded_materials={2,7,42};for(auto mode:{ClipMode::CutThrough,ClipMode::WholeElements}){auto empty=BuildClippedSurface(*mesh,nullptr,options,{{0,.5f,0},{0,1,0}},mode);Check(empty.surface.positions.empty()&&empty.element_boundaries.empty(),"All-off clipping stays empty");}
    Check(SampleDofs(*mesh,*vector,options).empty()&&SampleVectors(*mesh,*vector,options).empty(),"All-off overlays empty");
    auto flat=ThreeMaterials(2);options.excluded_materials={7};auto edges=BuildElementBoundaries(*flat,options);std::set<int> expected;for(int b=0;b<flat->GetNBE();++b)expected.insert(flat->GetBdrAttribute(b));bool actual=false,synthetic=false;for(auto edge:edges){Check(edge.material_id!=7,"2D edges exclude middle material");if(edge.boundary_id){actual=true;Check(expected.contains(edge.boundary_id),"2D edge uses actual boundary attribute");}else synthetic=true;}Check(actual&&synthetic,"2D boundary display distinguishes actual boundary IDs from exposed interfaces");
    std::cout<<"PASS refined/planar caps, whole-element clips, filtered overlays and 2D boundary IDs\n";
}
void AnalysisSelection(){
    auto mesh=ThreeMaterials(2);RenderFrame frame;frame.mesh=mesh;frame.session_id=1;frame.mesh_id=9;frame.mesh_revision=3;
    auto field=Field(*mesh,new mfem::L2_FECollection(0,2));mfem::Array<int> dofs;for(int e=0;e<mesh->GetNE();++e){field->FESpace()->GetElementDofs(e,dofs);(*field)[dofs[0]]=mesh->GetAttribute(e);}frame.fields.push_back({"material_value",std::move(field)});
    RenderOptions options;options.excluded_materials={7};ProfileRegistry registry;ProfileSample a,b,c,tie;a.position={.5,.5,0};b.position={1.5,.5,0};c.position={2.5,.5,0};tie.position={1,.5,0};
    for(const auto& algorithm:registry.algorithms()) {auto samples=registry.sample(algorithm.id,*mesh,*frame.fields[0].gf,{a,b,c},options);Check(samples[0].valid&&!samples[1].valid&&samples[2].valid&&std::isnan(samples[1].value),"Profiles return gaps in hidden regions");Near(samples[0].value,2,1e-12,"Selected left value");Near(samples[2].value,42,1e-12,"Selected right value");options.excluded_materials={2};auto interface=registry.sample(algorithm.id,*mesh,*frame.fields[0].gf,{tie},options);Check(interface[0].valid&&mesh->GetAttribute(interface[0].element)==7,"Shared-interface lookup chooses included parent");Near(interface[0].value,7,1e-12,"Discontinuous interface uses included trace");options.excluded_materials={7};}
    TrackingRequest request;request.field_name="material_value";request.kind=MeasurementKind::Integral;request.excluded_materials={7};FrameTracker integral(request);integral.append(frame);Check(integral.samples().back().valid,"Selected integral valid");Near(integral.samples().back().value,44,1e-11,"Integral excludes middle material");Near(integral.samples().back().measure,2,1e-11,"Measure excludes middle material");
    request.kind=MeasurementKind::Mean;FrameTracker mean(request);mean.append(frame);Near(mean.samples().back().value,22,1e-11,"Selected physical mean");request.kind=MeasurementKind::L2Norm;FrameTracker norm(request);norm.append(frame);Near(norm.samples().back().value,std::sqrt(4+42*42),1e-11,"Selected L2 norm");
    auto nodal=Field(*mesh,new mfem::H1_FECollection(1,2));*nodal=9.;auto markers=SampleDofs(*mesh,*nodal);int shared=-1,left=-1;for(auto p:markers){if(std::abs(p.position.x-1)<1e-6)shared=p.index;if(std::abs(p.position.x)<1e-6)left=p.index;}Check(shared>=0&&left>=0,"Found shared and exclusively hidden DOFs");frame.fields.push_back({"nodal",std::move(nodal)});
    request={};request.field_name="nodal";request.dof=shared;request.excluded_materials={2};FrameTracker support(request);support.bindDof(frame);support.append(frame);Check(support.samples().back().valid,"Shared DOF remains valid through any included parent");
    request.dof=left;FrameTracker hidden(request);hidden.bindDof(frame);hidden.append(frame);Check(!hidden.samples().back().valid,"DOF without included support becomes gap");
    support.setExcludedMaterials({42,7,2,7});Check(support.samples().empty()&&support.request().excluded_materials==std::vector<int>({2,7,42}),"Filter setter clears history and canonicalizes selection");support.append(frame);Check(!support.samples().back().valid,"All-off DOF is a gap");support.setExcludedMaterials({});frame.session_id=2;support.append(frame);Check(!support.samples().back().valid&&support.samples().back().error.find("association changed")!=std::string::npos,"Filter reset preserves original DOF association");
    integral.setExcludedMaterials({2,7,42});integral.append(frame);Check(!integral.samples().back().valid,"All-off aggregation is an invalid gap");options.excluded_materials={2,7,42};for(auto p:registry.sample("coherent",*mesh,*frame.fields[0].gf,{a,b,c},options))Check(!p.valid,"All-off profile is empty of valid values");
    std::ostringstream csv;ExportTrackingCSV(csv,integral);Check(csv.str().find("excluded_materials")!=std::string::npos&&csv.str().find("\"2;7;42\"")!=std::string::npos,"Tracking exports material selection");
    std::cout<<"PASS filtered inverse profiles, discontinuous ties, physical measures, DOF support, filter reset and CSV\n";
}
void NonconformingAndNurbs(const std::string& data){
    auto nc=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);nc.GetElement(0)->SetAttribute(2);nc.GetElement(1)->SetAttribute(7);nc.EnsureNCMesh();mfem::Array<int> refine(1);refine[0]=0;nc.GeneralRefinement(refine);RenderOptions options;options.refinement=2;options.excluded_materials={2};auto coarse=BuildSurface(nc,nullptr,options);Near(Volume(coarse),1,1e-6,"Nonconforming interface exposes only the retained coarse side");options.excluded_materials={7};auto fine=BuildSurface(nc,nullptr,options);Near(Volume(fine),1,1e-6,"Nonconforming interface exposes retained fine faces once");
    auto mesh=std::make_shared<mfem::Mesh>((data+"/ball-nurbs.mesh").c_str(),1,1);const int ids[]={2,7,42};for(int e=0;e<mesh->GetNE();++e)mesh->GetElement(e)->SetAttribute(ids[e%3]);mesh->SetAttributes();options.refinement=3;options.excluded_materials={7};options.coloring=SurfaceColoring::Material;auto rational=BuildSurface(*mesh,nullptr,options);Check(!rational.positions.empty(),"NURBS material selection produces geometry");Colors(rational,options.coloring);for(int id:rational.material_ids)Check(id!=7,"NURBS excludes hidden patches");
    RenderFrame frame;frame.mesh=mesh;auto field=Field(*mesh,new mfem::H1_FECollection(1,3));*field=1.;frame.fields.push_back({"one",std::move(field)});TrackingRequest request;request.field_name="one";request.kind=MeasurementKind::Integral;request.quadrature_order=16;FrameTracker all(request);all.append(frame);Check(all.samples().back().valid,"Rational full volume valid");double parts=0;for(int keep:ids){request.excluded_materials.clear();for(int id:ids)if(id!=keep)request.excluded_materials.push_back(id);FrameTracker part(request);part.append(frame);Check(part.samples().back().valid,"Rational selected volume valid");parts+=part.samples().back().value;}Near(parts,all.samples().back().value,1e-9,"Selected rational patch measures partition total measure");Check(mesh->NURBSext!=nullptr,"Filtering preserves rational mesh");
    std::cout<<"PASS nonconforming exposed interfaces and rational material geometry/measure\n";
}
}
int main(int argc,char** argv){try{RenderSelection();ClippingAndOverlays();AnalysisSelection();NonconformingAndNurbs(argc>1?argv[1]:"subprojects/mfem/data");std::cout<<"All selection tests passed\n";return 0;}catch(const std::exception& e){std::cerr<<"Selection test failed: "<<e.what()<<'\n';return 1;}}
