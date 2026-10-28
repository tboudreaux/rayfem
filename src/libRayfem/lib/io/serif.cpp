#include "libRayfem/io/serif.hpp"

#ifndef RAYFEM_HAS_SERIF
#define RAYFEM_HAS_SERIF 0
#endif

#if RAYFEM_HAS_SERIF
#include "serif/utils/serialize/archive.hpp"
#include "serif/backend/mfem/serialize/resources.hpp"
#include "../mesh/input_validation.hpp"
#include <atomic>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
namespace archive=serif::utils::serialize;
namespace backend=serif::backend::mfem::serialize;
using archive::Node;
constexpr std::uint64_t MaximumFileBytes=512ULL*1024*1024;
constexpr std::uint64_t MaximumSelectedBytes=256ULL*1024*1024;
constexpr std::size_t MaximumFields=256;

std::string ErrorText(const archive::Error& error) {
    return error.path.empty()?error.message:error.path+": "+error.message;
}
[[noreturn]] void Fail(const std::string& path,const std::string& message) {
    throw std::runtime_error(path+": "+message);
}
const Node& Child(const Node& node,const std::string& name) {
    const auto found=node.children.find(name);
    if(found==node.children.end())Fail(name,"required saved field is missing");
    return found->second;
}
void Schema(const Node& node,const std::string& expected) {
    if(node.type!=expected)Fail(expected,"expected schema, received '"+node.type+"'");
    if(node.version!=1)Fail(expected,"unsupported schema version "+std::to_string(node.version));
    if(!std::holds_alternative<std::monostate>(node.value))Fail(expected,"object contains a scalar payload");
}
template<class T> T Value(const Node& node) {
    auto result=archive::decode<T>(node);
    if(!result)throw std::runtime_error(ErrorText(result.error()));
    return std::move(*result);
}
template<class T> T Field(const Node& node,const std::string& name) {
    try{return Value<T>(Child(node,name));}
    catch(const std::exception& error){Fail(name,error.what());}
}
std::vector<const Node*> Array(const Node& node,const char* schema="array") {
    Schema(node,schema);
    if(node.children.size()>100000)Fail(schema,"too many saved entries");
    std::vector<const Node*> result;result.reserve(node.children.size());
    for(std::size_t i=0;i<node.children.size();++i)result.push_back(&Child(node,std::to_string(i)));
    return result;
}
Node Read(archive::Reader& reader,const std::string& path) {
    std::uint64_t bytes=0;
    for(const auto& entry:reader.ledger())if(entry.path==path || entry.path.starts_with(path+"/")) {
        if(entry.bytes>MaximumSelectedBytes-bytes)Fail(path,"selected data exceeds the 256 MiB import limit");
        bytes+=entry.bytes;
    }
    auto result=reader.node(path);
    if(!result)throw std::runtime_error(ErrorText(result.error()));
    return std::move(*result);
}
std::optional<Node> Select(archive::Reader& reader,const std::string& explicit_path,
                           const std::string& schema,bool required) {
    if(!explicit_path.empty()) {
        auto result=Read(reader,explicit_path);Schema(result,schema);return result;
    }
    std::optional<Node> selected;
    std::string selected_path;
    std::size_t candidates=0;
    for(const auto& entry:reader.ledger())if(entry.type==schema) {
        if(++candidates>16)Fail(schema,"too many candidates; supply an explicit archive path");
        auto result=Read(reader,entry.path);Schema(result,schema);
        if(selected && *selected!=result)Fail(schema,"multiple different records ("+selected_path+", "+entry.path+"); supply an explicit archive path");
        if(!selected){selected=std::move(result);selected_path=entry.path;}
    }
    if(!selected && required)Fail(schema,"archive does not contain a supported saved MFEM discretization");
    return selected;
}

// Reuse SERiF's true-DOF and orientation rules, including NC constraints. A
// matching vector length is insufficient evidence of coefficient identity.
class Binding final:public serif::discretization::backend::mfem::DofMap {
public:
    Binding(mfem::FiniteElementSpace& space,const mfem::Array<int>& support):DofMap(space,support){}
};

void ValidateResources(const Node& node) {
    Schema(node,"mfem.resources");
    const auto meshes=Array(Child(node,"meshes"));
    const auto collections=Array(Child(node,"collections"));
    const auto spaces=Array(Child(node,"spaces"));
    if(meshes.empty()||meshes.size()>32||collections.size()>MaximumFields||spaces.size()>MaximumFields)
        Fail("resources","resource table exceeds the viewer import limits");
    for(const auto* mesh:meshes) {
        Schema(*mesh,"mfem.mesh");
        if(mesh->children.contains("native"))RayFEM::detail::validateMeshText(Field<std::string>(*mesh,"native"));
    }
    for(const auto* collection:collections) {
        if(collection->type=="unsupported") {
            const auto* message=std::get_if<std::string>(&collection->value);
            Fail("collections",message?*message:"unsupported saved collection");
        }
        Schema(*collection,"mfem.collection");
        const int order=Field<int>(*collection,"order");
        if(order<0||order>32)Fail("collections/order","supported import range is 0..32");
    }
    for(const auto* space:spaces) {
        Schema(*space,"mfem.space");
        const int size=Field<int>(*space,"size"),dimension=Field<int>(*space,"vector_dimension");
        if(size<0||size>10000000||dimension<1||dimension>3)Fail("spaces","invalid or excessive coefficient count or vector dimension");
    }
}

std::string Name(const Node& coordinate) {
    Schema(coordinate,"coordinate");
    const auto id=Field<std::string>(coordinate,"id");
    if(id.empty()||id.size()>256||id.find('\0')!=std::string::npos)Fail("coordinate/id","invalid field name");
    return id;
}

mfem::Array<int> Support(const Node& binding,mfem::Mesh& mesh) {
    const auto& coordinates=Child(binding,"coordinates");Schema(coordinates,"mfem.binding_coordinates");
    const auto support=Field<std::vector<int>>(coordinates,"support");
    if(support.size()!=std::size_t(mesh.GetNE()))Fail("binding/support","saved support does not match mesh elements");
    const auto& schema=Child(binding,"schema");Schema(schema,"mfem.schema");
    std::map<std::string,int> domains;
    for(const auto* domain:Array(Child(schema,"domains"))) {
        Schema(*domain,"mfem.domain");
        const auto name=Field<std::string>(*domain,"name");
        if(!domains.emplace(name,Field<int>(*domain,"attribute")).second)Fail("schema/domains","duplicate domain name");
    }
    std::set<int> allowed;
    for(const auto& name:Field<std::vector<std::string>>(Child(binding,"term"),"support")) {
        const auto found=domains.find(name);
        if(found==domains.end())Fail("binding/support","unknown named domain '"+name+"'");
        allowed.insert(found->second);
    }
    mfem::Array<int> result(mesh.GetNE());
    for(int e=0;e<mesh.GetNE();++e) {
        const int expected=allowed.contains(mesh.GetAttribute(e))?1:0;
        if(support[e]!=expected)Fail("binding/support","saved element support disagrees with named material regions");
        result[e]=expected;
    }
    return result;
}

void ValidateTermSpace(const Node& term,const mfem::FiniteElementSpace& space) {
    const auto& descriptor=Child(term,"space");Schema(descriptor,"functional_space");
    const auto family=Field<std::string>(descriptor,"family");
    const auto* collection=space.FEColl();
    const bool h1=dynamic_cast<const mfem::H1_FECollection*>(collection);
    const bool l2=dynamic_cast<const mfem::L2_FECollection*>(collection);
    const bool rt=dynamic_cast<const mfem::RT_FECollection*>(collection);
    if(!((family=="h1"&&h1)||(family=="l2"&&l2)||(family=="hdiv"&&rt)))Fail("term/space","physical family disagrees with saved FE resources");
    const int order=collection->GetOrder()-(rt?1:0);
    const auto rank=Field<std::uint64_t>(descriptor,"rank");
    if(order!=Field<int>(descriptor,"order") || (rt&&(rank!=1||space.GetVDim()!=1)) || (rank==0&&space.GetVDim()!=1) ||
       (rank==1&&!rt&&space.GetVDim()!=3) || rank>1)
        Fail("term/space","order or vector rank disagrees with saved FE resources");
}

struct StateField {const Node* coordinate;std::size_t offset,size;};
std::vector<StateField> StateFields(const Node& state,std::vector<double>& values) {
    Schema(state,"stellar_state");const auto& layout=Child(state,"layout");Schema(layout,"system_layout");
    const auto terms=Array(Child(layout,"values"),"coordinate_list");
    const auto sizes=Field<std::vector<std::uint64_t>>(layout,"state_sizes");
    values=Field<std::vector<double>>(state,"coefficients");
    if(terms.size()!=sizes.size()||terms.size()>MaximumFields)Fail("state/layout","coordinate count does not match saved block sizes");
    if(!std::all_of(values.begin(),values.end(),[](double v){return std::isfinite(v);}))Fail("state/coefficients","non-finite coefficients cannot be plotted");
    std::set<std::string> names;std::vector<StateField> result;std::size_t offset=0;
    for(std::size_t i=0;i<terms.size();++i) {
        if(!names.insert(Name(*terms[i])).second)Fail("state/layout","duplicate coordinate identity");
        if(sizes[i]>values.size()-offset)Fail("state/layout","invalid coefficient block size");
        result.push_back({terms[i],offset,std::size_t(sizes[i])});offset+=sizes[i];
    }
    if(offset!=values.size())Fail("state/coefficients","coefficient count does not match layout");
    return result;
}
} // namespace
#endif

namespace RayFEM {
bool SerifLoadingAvailable(){return RAYFEM_HAS_SERIF!=0;}

SerifLoadResult LoadSerifFile(const std::filesystem::path& path,const SerifLoadOptions& options) {
    SerifLoadResult result;
#if !RAYFEM_HAS_SERIF
    (void)path;(void)options;
    result.error="SERiF loading is unavailable in this build. Configure RayFEM with an existing compatible SERiF installation and its MFEM dependency.";
#else
    try {
#ifndef MFEM_USE_EXCEPTIONS
        throw std::runtime_error("SERiF loading requires MFEM exception support.");
#else
        struct ErrorPolicy {
            mfem::ErrorAction previous=mfem::get_error_action();
            ErrorPolicy(){mfem::set_error_action(mfem::MFEM_ERROR_THROW);}
            ~ErrorPolicy(){mfem::set_error_action(previous);}
        } error_policy;
#endif
        if(std::filesystem::file_size(path)>MaximumFileBytes)Fail(path.string(),"archive exceeds the 512 MiB viewer import limit");
        archive::ReadOptions read_options;read_options.maximum_payload_bytes=MaximumSelectedBytes;read_options.maximum_entries=100000;
        auto opened=archive::Reader::open(path,read_options);
        if(!opened)throw std::runtime_error(ErrorText(opened.error()));
        auto& reader=*opened;
        auto discretization=Select(reader,options.discretization_path,"mfem.stellar_discretization",true);
        auto state=Select(reader,options.state_path,"stellar_state",false);
        const auto& resources_node=Child(*discretization,"resources");ValidateResources(resources_node);
        auto restored=backend::restore_resources(resources_node);
        if(!restored)throw std::runtime_error(ErrorText(restored.error()));
        auto owner=std::make_shared<backend::Resources>(std::move(*restored));
        static std::atomic<std::uint64_t> next_session{1ULL<<63};
        const auto session=next_session.fetch_add(1);
        std::map<mfem::Mesh*,std::shared_ptr<RenderFrame>> frames;
        auto frame_for=[&](mfem::Mesh* mesh) {
            auto& frame=frames[mesh];
            if(!frame) {
                frame=std::make_shared<RenderFrame>();frame->mesh=std::shared_ptr<mfem::Mesh>(owner,mesh);
                frame->session_id=session;frame->mesh_id=frames.size();frame->mesh_revision=1;
                frame->metadata.frame_index=static_cast<std::uint32_t>(frame->mesh_id-1);
                const auto label=path.filename().string()+" / mesh "+std::to_string(frame->mesh_id);
                std::fill(std::begin(frame->metadata.label),std::end(frame->metadata.label),'\0');
                std::copy_n(label.data(),std::min(label.size(),sizeof(frame->metadata.label)-1),frame->metadata.label);
                // Include shared resources conservatively in each mesh estimate.
                frame->estimated_mesh_bytes=std::filesystem::file_size(path)*4;
            }
            return frame;
        };
        std::map<std::string,const Node*> bindings;
        for(const auto* binding:Array(Child(*discretization,"bindings"))) {
            Schema(*binding,"mfem.term_binding");
            if(!bindings.emplace(Name(Child(*binding,"term")),binding).second)Fail("bindings","duplicate term identity");
        }
        bool displacement_loaded=false;
        if(state) {
            std::vector<double> values;
            for(const auto& item:StateFields(*state,values)) {
                const auto name=Name(*item.coordinate);const auto found=bindings.find(name);
                if(found==bindings.end()) {
                    result.warnings.push_back(name+": no saved MFEM binding; these coordinates are not plotted.");continue;
                }
                const auto& binding=*found->second;
                if(Child(binding,"term")!=*item.coordinate)Fail(name,"state and discretization physical coordinate schemas differ");
                if(!binding.children.contains("space")) {
                    std::ostringstream warning;warning<<name<<": global coordinate";
                    if(item.size==1)warning<<" = "<<std::setprecision(17)<<values[item.offset];
                    warning<<"; not a spatial field.";result.warnings.push_back(warning.str());continue;
                }
                const auto index=Field<std::uint64_t>(binding,"space");
                if(index>=owner->spaces.size())Fail(name,"space reference is outside restored resources");
                auto& space=*owner->spaces[index];ValidateTermSpace(*item.coordinate,space);
                const auto support=Support(binding,*space.GetMesh());
                Binding map(space,support);
                if(backend::binding_coordinates(map)!=Child(binding,"coordinates"))Fail(name,"saved reduced/true coefficient numbering differs from reconstructed support");
                if(item.size!=std::size_t(map.size()))Fail(name,"state coefficient count does not match the saved binding");
                if(item.size==0){result.warnings.push_back(name+": no supported degrees of freedom; field omitted.");continue;}
                mfem::Vector reduced(int(item.size));
                std::copy_n(values.data()+item.offset,item.size,reduced.GetData());
                auto field=std::make_unique<mfem::GridFunction>(&space);*field=map.scatter(reduced);
                for(int i=0;i<field->Size();++i)if(!std::isfinite((*field)[i]))Fail(name,"expanded coefficients are not finite");
                auto frame=frame_for(space.GetMesh());FrameField saved{name,std::move(field)};
                bool restricted=false;std::set<int> materials;
                for(int e=0;e<support.Size();++e) {if(support[e])materials.insert(space.GetMesh()->GetAttribute(e));else restricted=true;}
                if(restricted) {
                    saved.supported_materials.assign(materials.begin(),materials.end());
                    std::ostringstream warning;warning<<name<<": defined only on materials ";
                    for(int material:saved.supported_materials)warning<<material<<' ';
                    warning<<"(other regions are excluded from display and analysis).";result.warnings.push_back(warning.str());
                }
                if(name=="displacement.vector" && !restricted && saved.gf->VectorDim()==3) {
                    frame->displacement=std::make_unique<mfem::GridFunction>(&space);*frame->displacement=*saved.gf;
                    displacement_loaded=true;
                }
                frame->estimated_bytes+=std::size_t(space.GetVSize())*(sizeof(double)*3+sizeof(int)*4);
                frame->fields.push_back(std::move(saved));
            }
        } else result.warnings.push_back("No saved stellar state was found; displaying reference mesh and compactification only.");
        if(owner->compactification) {
            auto* space=owner->compactification->FESpace();auto frame=frame_for(space->GetMesh());
            auto field=std::make_unique<mfem::GridFunction>(space);*field=*owner->compactification;
            frame->estimated_bytes+=std::size_t(field->Size())*32;
            std::string name="compactification";
            auto exists=[&](const std::string& candidate){return std::any_of(frame->fields.begin(),frame->fields.end(),[&](const auto& item){return item.name==candidate;});};
            if(exists(name))name="compactification (mapping)";
            for(unsigned suffix=2;exists(name);++suffix)name="compactification (mapping "+std::to_string(suffix)+")";
            frame->fields.push_back({std::move(name),std::move(field)});
        }
        if(frames.empty())frame_for(owner->meshes.front().get());
        const auto deformation=discretization->children.find("deformation");
        if(!displacement_loaded && deformation!=discretization->children.end() && deformation->second.type!="mfem.no_deformation")
            result.warnings.push_back("Reference geometry is shown: the saved surface/deformation model has no pointwise displacement field. Surface-model coefficients have not been applied.");
        result.warnings.push_back("Values retain their saved physical units and FE interpretation. The compactified exterior is shown in saved mesh coordinates; the SERiF exterior mapping is not applied.");
        result.warnings.push_back("No physical simulation timestamp is imported; frame time is 0 and frame indices identify saved meshes.");
        for(auto& [mesh,frame]:frames)result.frames.push_back(std::move(frame));
        std::sort(result.frames.begin(),result.frames.end(),[](const auto& first,const auto& second){return first->mesh_id<second->mesh_id;});
    } catch(const std::exception& error) {result.frames.clear();result.error=error.what();}
#endif
    return result;
}
} // namespace RayFEM
