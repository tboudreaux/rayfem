#include "libRayfem/io/serif.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifndef RAYFEM_HAS_SERIF
#define RAYFEM_HAS_SERIF 0
#endif
#if RAYFEM_HAS_SERIF
#include "serif/utils/serialize/archive.hpp"
#include "serif/backend/mfem/serialize/resources.hpp"
#include "libRayfem/profile/sampler.hpp"
#include <chrono>
#include <map>
#include <set>

namespace {
namespace archive=serif::utils::serialize;
namespace backend=serif::backend::mfem::serialize;
using archive::Node;
void Check(bool condition,const std::string& message){if(!condition)throw std::runtime_error(message);}
void Near(double first,double second,double tolerance,const std::string& message){Check(std::isfinite(first)&&std::abs(first-second)<=tolerance,message);}
class Binding final:public serif::discretization::backend::mfem::DofMap {
public:Binding(mfem::FiniteElementSpace& space,const mfem::Array<int>& support):DofMap(space,support){}
};
struct Fixture {Node discretization,state;};
Node Term(const std::string& id,const std::string& family,int order,bool vector,bool restricted){
    using namespace archive;
    return object("coordinate",field("id",id),field("dimension",std::string{"test_units"}),
        field("space",object("functional_space",field("family",family),field("order",order),field("rank",std::uint64_t(vector)))),
        field("support",restricted?std::vector<std::string>{"left"}:std::vector<std::string>{"left","right"}));
}
Fixture MakeFixture(bool nc){
    using namespace archive;
    auto mesh=mfem::Mesh::MakeCartesian3D(2,1,1,mfem::Element::HEXAHEDRON,2,1,1);
    mesh.GetElement(0)->SetAttribute(2);mesh.GetElement(1)->SetAttribute(7);
    if(nc){mesh.EnsureNCMesh();mfem::Array<int> elements(1);elements[0]=0;mesh.GeneralRefinement(elements);}
    mesh.SetCurvature(3);
    mfem::VectorFunctionCoefficient curve(3,[](const mfem::Vector& x,mfem::Vector& y){y=x;y[0]+=.08*x[1]*x[2];});mesh.Transform(curve);
    mfem::H1_FECollection h1(3,3);mfem::L2_FECollection l2(2,3);mfem::RT_FECollection rt(2,3);
    mfem::FiniteElementSpace h1space(&mesh,&h1),l2space(&mesh,&l2),rtspace(&mesh,&rt),vector_space(&mesh,&h1,3,mfem::Ordering::byVDIM);
    mfem::GridFunction scalar(&h1space),density(&l2space),vector(&rtspace),displacement(&vector_space),compactification(&h1space);
    mfem::FunctionCoefficient linear([](const mfem::Vector& x){return 1+x[0]+2*x[1]+3*x[2];});scalar.ProjectCoefficient(linear);density.ProjectCoefficient(linear);
    mfem::VectorFunctionCoefficient rotation(3,[](const mfem::Vector& x,mfem::Vector& v){v.SetSize(3);v[0]=-x[1];v[1]=x[0];v[2]=.25;});vector.ProjectCoefficient(rotation);
    displacement=0.;compactification=0.;
    backend::ResourceWriter writer;
    Node domains=encode(std::vector<Node>{object("mfem.domain",field("name",std::string{"left"}),field("attribute",2)),object("mfem.domain",field("name",std::string{"right"}),field("attribute",7))});
    Node schema=object("mfem.schema",field("domains",domains),field("boundaries",std::vector<Node>{}));
    std::vector<Node> bindings,terms;std::vector<std::uint64_t> sizes;std::vector<double> values;
    auto add=[&](const std::string& id,const std::string& family,int order,mfem::GridFunction& field,bool restricted){
        mfem::Array<int> support(mesh.GetNE());for(int e=0;e<mesh.GetNE();++e)support[e]=!restricted||mesh.GetAttribute(e)==2;
        Binding binding(*field.FESpace(),support);const auto reduced=binding.gather(field);
        const auto term=Term(id,family,order,field.VectorDim()==3,restricted);
        bindings.push_back(object("mfem.term_binding",archive::field("term",term),archive::field("schema",schema),
            archive::field("coordinates",backend::binding_coordinates(binding)),archive::field("space",writer.space(*field.FESpace()))));
        terms.push_back(term);sizes.push_back(reduced.Size());values.insert(values.end(),reduced.GetData(),reduced.GetData()+reduced.Size());
    };
    add("temperature","h1",3,scalar,false);add("density","l2",2,density,true);add("rotation","hdiv",2,vector,false);
    for(int i=0;i<vector_space.GetNDofs();++i)displacement[vector_space.DofToVDof(i,0)]=.125;
    add("displacement.vector","h1",3,displacement,false);
    // The state order intentionally differs from binding order.
    std::swap(bindings[0],bindings[2]);
    serif::mapping::DomainMapper mapper({},std::make_unique<serif::mapping::compactification::KelvinCompactification>(serif::utils::user_options::KelvinCompactificationOptions{}));
    writer.mapping(compactification,mapper);
    Node coordinates{.type="coordinate_list"};for(std::size_t i=0;i<terms.size();++i)coordinates.children.emplace(std::to_string(i),terms[i]);
    Node state=object("stellar_state",field("layout",object("system_layout",field("values",coordinates),field("state_sizes",sizes))),field("coefficients",values));
    Node discretization=object("mfem.stellar_discretization",field("bindings",bindings),field("deformation",Node{.type="mfem.no_deformation"}),field("resources",std::move(writer).finish()));
    return {std::move(discretization),std::move(state)};
}
void Write(const std::filesystem::path& path,const Fixture& fixture,archive::Format format=archive::Format::binary,bool duplicate=true){
    auto contents=archive::pack(archive::named("configuration",archive::object("model.configured_discretization",archive::field("finite_elements",fixture.discretization))),archive::named("stellar_state",fixture.state));
    if(duplicate)archive::add_component(contents,archive::named("checkpoint",archive::object("newton_checkpoint",archive::field("state",fixture.state))));
    const auto written=archive::write(path,contents,{.format=format,.author="RayFEM loader regression"});
    Check(bool(written),written?"":written.error().message);
}
const RayFEM::FrameField& Find(const RayFEM::RenderFrame& frame,const std::string& name){
    for(const auto& field:frame.fields)if(field.name==name)return field;
    throw std::runtime_error("Missing field "+name);
}
void Verify(const std::filesystem::path& path,bool nc){
    std::shared_ptr<RayFEM::RenderFrame> frame;
    {
        const auto previous=mfem::get_error_action();mfem::set_error_action(mfem::MFEM_ERROR_ABORT);
        auto loaded=RayFEM::LoadSerifFile(path);const bool restored_policy=mfem::get_error_action()==mfem::MFEM_ERROR_ABORT;mfem::set_error_action(previous);
        Check(restored_policy,"Import changed the caller's MFEM error policy");Check(bool(loaded),loaded.error);Check(loaded.frames.size()==1,"Reference-only meshes should not become unrelated animation frames");frame=loaded.frames.front();
        Check(!loaded.warnings.empty(),"Restricted support and saved coordinates must be reported");
    }
    Check(frame->mesh->Nonconforming()==nc,"Nonconforming mesh topology changed");
    Check(frame->displacement&&frame->displacement->VectorDim()==3,"Saved displacement field not retained");
    const auto& temperature=Find(*frame,"temperature");const auto& density=Find(*frame,"density");const auto& rotation=Find(*frame,"rotation");
    Check(density.supported_materials==std::vector<int>{2},"Material support was lost");Check(temperature.supported_materials.empty(),"Unrestricted field was marked restricted");
    std::vector<int> exclusions{42};RayFEM::ExcludeUnsupportedMaterials(density,*frame->mesh,exclusions);Check(exclusions==std::vector<int>({7,42}),"Support must merge with caller exclusions");
    for(int e=0;e<frame->mesh->GetNE();++e){
        auto& transform=*frame->mesh->GetElementTransformation(e);mfem::IntegrationPoint ip;ip.Set3(.31,.42,.53);transform.SetIntPoint(&ip);mfem::Vector x;transform.Transform(ip,x);
        Near(temperature.gf->GetValue(e,ip),1+x[0]+2*x[1]+3*x[2],1e-10,"H1 field differs after owner destruction or coordinate mapping");
        if(frame->mesh->GetAttribute(e)==2)Near(density.gf->GetValue(e,ip),1+x[0]+2*x[1]+3*x[2],1e-10,"Restricted L2 field changed");
        mfem::Vector v;rotation.gf->GetVectorValue(e,ip,v);Near(v[0],-x[1],1e-9,"Oriented Hdiv field x changed");Near(v[1],x[0],1e-9,"Oriented Hdiv field y changed");
    }
    const auto surface=RayFEM::BuildSurface(*frame->mesh,temperature.gf.get());Check(!surface.positions.empty(),"Loaded frame must render after loader resources leave scope");
    Check(frame->estimated_bytes>0&&frame->estimated_mesh_bytes>0,"History estimates missing");
}
void Failures(const std::filesystem::path& directory){
    const auto path=directory/"invalid.serif";
    auto expect=[&](Fixture fixture,const std::string& fragment){Write(path,fixture);const auto result=RayFEM::LoadSerifFile(path);Check(!result&&result.error.find(fragment)!=std::string::npos,"Expected error '"+fragment+"', received: "+result.error);Check(result.frames.empty(),"Failed imports must not expose partial fields");};
    auto fixture=MakeFixture(false);fixture.discretization.version=2;expect(fixture,"schema version");
    fixture=MakeFixture(false);fixture.state.children.at("coefficients")=archive::encode(std::vector<double>{0});expect(fixture,"block size");
    fixture=MakeFixture(false);auto& bindings=fixture.discretization.children.at("bindings").children;
    auto& numbering=bindings.at("0").children.at("coordinates").children.at("full_to_reduced");std::get<std::vector<std::int64_t>>(numbering.value)[0]=999;expect(fixture,"numbering");
    fixture=MakeFixture(false);fixture.discretization.children.at("resources").children.at("meshes").children.at("0").children.erase("native");expect(fixture,"omits");
    fixture=MakeFixture(false);fixture.discretization.children.at("resources").children.at("mfem_version")=archive::encode(std::string{"different"});expect(fixture,"MFEM version");
    fixture=MakeFixture(false);fixture.discretization.children.at("bindings").children.at("0").children.at("term").children.at("space").children.at("rank")=archive::encode(std::uint64_t{0});
    fixture.state.children.at("layout").children.at("values").children.at("2").children.at("space").children.at("rank")=archive::encode(std::uint64_t{0});expect(fixture,"vector rank");
    fixture=MakeFixture(false);const std::string excessive="MFEM NC mesh v1.0\n\ndimension\n3\n\nelements\n2147483647\n";
    auto& saved_mesh=fixture.discretization.children.at("resources").children.at("meshes").children.at("0");saved_mesh.children.at("native")=archive::encode(excessive);saved_mesh.children.at("identity")=archive::encode(archive::fingerprint(excessive));expect(fixture,"table count");
    fixture=MakeFixture(false);Write(path,fixture);{std::ofstream file(path,std::ios::binary|std::ios::app);file<<'\n';}Check(!RayFEM::LoadSerifFile(path),"Trailing archive bytes were silently accepted");
    Write(path,fixture);{std::fstream file(path,std::ios::binary|std::ios::in|std::ios::out);file.seekg(-1,std::ios::end);char byte;file.get(byte);file.seekp(-1,std::ios::end);file.put(byte^1);}Check(!RayFEM::LoadSerifFile(path),"Damaged coefficient checksum was accepted");
    auto different=fixture.state;std::get<std::vector<double>>(different.children.at("coefficients").value)[0]+=1;
    auto contents=archive::pack(archive::named("discretization",fixture.discretization),archive::named("first",fixture.state),archive::named("second",different));Check(bool(archive::write(path,contents)),"Ambiguous fixture write failed");
    Check(!RayFEM::LoadSerifFile(path),"Different states silently selected");auto selected=RayFEM::LoadSerifFile(path,{.state_path="first"});Check(bool(selected),selected.error);
    fixture=MakeFixture(false);fixture.discretization.children.at("bindings").children.at("2").children.at("term").children.at("id")=archive::encode(std::string{"compactification"});
    fixture.state.children.at("layout").children.at("values").children.at("0").children.at("id")=archive::encode(std::string{"compactification"});Write(path,fixture);
    const auto collision=RayFEM::LoadSerifFile(path);Check(bool(collision),collision.error);
    Check(Find(*collision.frames.front(),"compactification").gf->Size()>0&&Find(*collision.frames.front(),"compactification (mapping)").gf->Size()>0,"Mapping diagnostic overwrote a state field name");
}
}
#endif

int main(int argc,char** argv){try{
#if RAYFEM_HAS_SERIF
    Check(RayFEM::SerifLoadingAvailable(),"Enabled serializer capability not advertised");
    if(argc==3&&std::string(argv[1])=="--write-fixture") {
        Write(argv[2],MakeFixture(true));Verify(argv[2],true);
        std::cout<<"Wrote validated NC curved SERiF viewer fixture: "<<argv[2]<<'\n';return 0;
    }
    const auto directory=std::filesystem::temp_directory_path()/("rayfem-serif-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::filesystem::create_directories(directory);
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove_all(path,error);}}cleanup{directory};
    for(bool nc:{false,true})for(auto format:{archive::Format::binary,archive::Format::text}){const auto path=directory/"roundtrip.serif";Write(path,MakeFixture(nc),format);Verify(path,nc);}
    Failures(directory);
    if(argc>1){
        auto reader=archive::Reader::open(argv[1]);Check(bool(reader),reader?"":reader.error().message);
        const auto checked=reader->validate();Check(bool(checked),checked?"":checked.error().message);
        auto loaded=RayFEM::LoadSerifFile(argv[1]);Check(bool(loaded),loaded.error);
        for(const auto& frame:loaded.frames)for(const auto& field:frame->fields){
            RayFEM::RenderOptions render;render.refinement=3;RayFEM::ExcludeUnsupportedMaterials(field,*frame->mesh,render.excluded_materials);
            const auto surface=RayFEM::BuildSurface(*frame->mesh,field.gf.get(),render);Check(!surface.positions.empty(),"External saved field has no renderable included surface");
            Check(std::isfinite(surface.min_value)&&std::isfinite(surface.max_value),"External field range is not finite");
            std::cout<<"  "<<field.name<<": "<<field.gf->Size()<<" coefficients, range ["<<surface.min_value<<", "<<surface.max_value<<"]\n";
        }
        std::cout<<"PASS external fixture: all archive sections validated, "<<loaded.frames.size()<<" mesh frame(s), "<<loaded.frames[0]->fields.size()<<" fields rendered\n";
    }
    std::cout<<"SERiF import passed: binary/text, curved/NC H1/L2/Hdiv fields, material support, independent ownership, schema/numbering/checksum rejection and explicit selection\n";
#else
    if(RayFEM::SerifLoadingAvailable())throw std::runtime_error("Disabled build advertises SERiF");
    const auto result=RayFEM::LoadSerifFile("missing.serif");
    if(result||result.error.find("unavailable")==std::string::npos)throw std::runtime_error("Missing reader is not reported clearly");
    std::cout<<"SERiF optional-dependency diagnostic passed\n";
#endif
    return 0;
}catch(const std::exception& error){std::cerr<<"SERiF test: "<<error.what()<<'\n';return 1;}}
