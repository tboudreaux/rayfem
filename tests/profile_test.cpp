#include "libRayfem/profile/sampler.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void Check(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
void Near(double value, double expected, double tolerance, const std::string& message) {
    if (!std::isfinite(value) || std::abs(value - expected) > tolerance) {
        std::ostringstream out;
        out << message << ": got " << value << ", expected " << expected;
        throw std::runtime_error(out.str());
    }
}
template<class Function> void Reject(Function function, const std::string& message) {
    bool rejected = false;
    try { function(); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, message);
}
RayFEM::ProfileSample Location(double x, double y, double z = 0) {
    RayFEM::ProfileSample sample;
    sample.position = {x, y, z};
    return sample;
}
double Polynomial(const RayFEM::ProfilePosition& x) { return 1 + x[0] + 2*x[1] + 3*x[2] + x[0]*x[0] - x[0]*x[1]; }
void GeometryRequests() {
    const auto ray = RayFEM::MakeRaySamples({1, 2, 3}, {3, 0, 0}, 2, 5);
    Check(ray.size() == 5, "Ray sample count");
    for (size_t i = 0; i < ray.size(); ++i) {
        Near(ray[i].s, .5*i, 1e-13, "Ray distance");
        Near(ray[i].position[0], 1+.5*i, 1e-13, "Normalized ray direction");
        Near(ray[i].position[1], 2, 1e-13, "Ray y");
        Near(ray[i].position[2], 3, 1e-13, "Ray z");
    }
    const auto plane = RayFEM::MakePlaneSamples({.5,.5,.5}, {1,2,3}, .8, .6, 5, 4);
    Check(plane.size() == 20, "Plane grid sample count");
    double min_u=1, max_u=-1, min_v=1, max_v=-1;
    for (const auto& sample : plane) {
        Near((sample.position[0]-.5) + 2*(sample.position[1]-.5) + 3*(sample.position[2]-.5), 0, 1e-12, "Plane perpendicular to normal");
        min_u=std::min(min_u,sample.u);max_u=std::max(max_u,sample.u);
        min_v=std::min(min_v,sample.v);max_v=std::max(max_v,sample.v);
    }
    Near(max_u-min_u,.8,1e-12,"Plane width"); Near(max_v-min_v,.6,1e-12,"Plane height");
    Reject([] { RayFEM::MakeRaySamples({0,0,0},{0,0,0},1,4); }, "Zero ray direction must reject");
    Reject([] { RayFEM::MakePlaneSamples({0,0,0},{0,0,0},1,1,4,4); }, "Zero plane normal must reject");
    Reject([] { RayFEM::MakeRaySamples({0,0,0},{1,0,0},-1,4); }, "Negative ray length must reject");
    Reject([] { RayFEM::MakePlaneSamples({0,0,0},{0,0,1},-1,1,4,4); }, "Negative plane width must reject");
    Reject([] { RayFEM::MakeRaySamples({0,0,0},{1,0,0},1,0); }, "Empty ray request must reject");
    std::cout << "PASS ray and plane geometry validation\n";
}
void AnalyticCases(RayFEM::ProfileRegistry& registry, int dim, bool curved) {
    auto mesh = dim == 2 ? mfem::Mesh::MakeCartesian2D(3,3,mfem::Element::QUADRILATERAL,true)
                         : mfem::Mesh::MakeCartesian3D(2,2,2,mfem::Element::HEXAHEDRON);
    if (curved) {
        mesh.SetCurvature(2);
        mfem::VectorFunctionCoefficient deformation(dim, [dim](const mfem::Vector& x,mfem::Vector& y) {
            y=x; y[0]+=.07*x[1]*(1-x[1]); y[1]+=.05*x[0]*(1-x[0]);
            if (dim==3) y[2]+=.04*x[0]*x[1];
        });
        mesh.Transform(deformation);
    }
    mfem::H1_FECollection collection(4,dim);
    mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([dim](const mfem::Vector& x) {
        return Polynomial({x[0],x[1],dim==3?x[2]:0});
    });
    field.ProjectCoefficient(coefficient);
    std::vector<RayFEM::ProfileSample> samples;
    for (int e=0;e<mesh.GetNE();++e) {
        mfem::IntegrationPoint ip;
        if(dim==2) ip.Set2(.27,.61); else ip.Set3(.27,.61,.38);
        mfem::Vector position;
        mesh.GetElementTransformation(e)->Transform(ip,position);
        samples.push_back(Location(position[0],position[1],dim==3?position[2]:0));
    }
    // Deliberately shuffled traversal exercises coherent fallback across elements.
    std::reverse(samples.begin(),samples.end());
    samples.push_back(Location(-3,-3,-3));
    if (dim==2) samples.push_back(Location(.5,.5,.2));
    std::vector<RayFEM::ProfileSample> previous;
    for (const std::string algorithm : {"exhaustive","coherent"}) {
        auto actual=registry.sample(algorithm,mesh,field,samples);
        Check(actual.size()==samples.size(),"All query locations retained");
        for(size_t i=0;i<actual.size();++i) {
            Check(actual[i].position==samples[i].position,"World sample positions preserved");
            if(i<static_cast<size_t>(mesh.GetNE())) {
                Check(actual[i].valid && actual[i].element>=0,"Interior sample found");
                Near(actual[i].value,Polynomial(actual[i].position),2e-8,"Analytic physical polynomial");
            } else {
                Check(!actual[i].valid && std::isnan(actual[i].value) && actual[i].element==-1,"Outside sample remains an explicit miss");
            }
            if(!previous.empty()) {
                Check(actual[i].valid==previous[i].valid && actual[i].element==previous[i].element,"Algorithms select same element");
                if(actual[i].valid) Near(actual[i].value,previous[i].value,2e-10,"Algorithms agree numerically");
            }
        }
        previous=std::move(actual);
    }
    if(!curved) {
        const auto plane=RayFEM::MakePlaneSamples({.5,.5,dim==3?.5:0},{0,0,1},.8,.8,9,7);
        const auto actual=registry.sample("coherent",mesh,field,plane);
        for(const auto& point:actual) { Check(point.valid,"Plane sample inside mesh");Near(point.value,Polynomial(point.position),1e-10,"Plane analytic value"); }
    }
    std::cout << "PASS " << dim << "D " << (curved?"curved":"straight") << " polynomial and misses\n";
}
void DiscontinuousCase(RayFEM::ProfileRegistry& registry) {
    auto mesh=mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true);
    mfem::L2_FECollection collection(0,2);
    mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::Array<int> dofs;
    for(int element=0;element<mesh.GetNE();++element) { space.GetElementDofs(element,dofs); field[dofs[0]]=10+element; }
    std::vector<RayFEM::ProfileSample> locations{Location(.8,.5),Location(.5,.5),Location(.2,.5),Location(.5,.5)};
    for(const std::string algorithm:{"exhaustive","coherent"}) {
        const auto actual=registry.sample(algorithm,mesh,field,locations);
        for(size_t i=0;i<actual.size();++i) { Check(actual[i].valid,"DG sample found");Near(actual[i].value,10+actual[i].element,1e-12,"DG element coefficient"); }
        Check(actual[1].element==0 && actual[3].element==0,"Shared-face tie chooses lowest element independent of prior query");
    }
    std::cout << "PASS discontinuous values and deterministic interface ties\n";
}
void VectorViews(RayFEM::ProfileRegistry& registry) {
    auto mesh=mfem::Mesh::MakeCartesian2D(2,2,mfem::Element::QUADRILATERAL,true);
    mfem::ND_FECollection nd(2,2); mfem::RT_FECollection rt(1,2);
    mfem::FiniteElementSpace nd_space(&mesh,&nd),rt_space(&mesh,&rt);
    mfem::GridFunction circulation(&nd_space),flux(&rt_space);
    mfem::VectorFunctionCoefficient rotation(2,[](const mfem::Vector& x,mfem::Vector& y){y.SetSize(2);y[0]=-x[1];y[1]=x[0];});
    mfem::VectorFunctionCoefficient radial(2,[](const mfem::Vector& x,mfem::Vector& y){y=x;});
    circulation.ProjectCoefficient(rotation);flux.ProjectCoefficient(radial);
    const auto locations=RayFEM::MakeRaySamples({.1,.21,0},{1,0,0},.8,17);
    for(const std::string algorithm:{"exhaustive","coherent"}) {
        RayFEM::RenderOptions options;options.view=RayFEM::FieldView::CurlComponent;
        for(const auto& sample:registry.sample(algorithm,mesh,circulation,locations,options)) {Check(sample.valid,"Curl location found");Near(sample.value,2,1e-10,"ND physical curl profile");}
        options.view=RayFEM::FieldView::Divergence;
        for(const auto& sample:registry.sample(algorithm,mesh,flux,locations,options)) {Check(sample.valid,"Divergence location found");Near(sample.value,2,1e-10,"RT physical divergence profile");}
        options.view=RayFEM::FieldView::Component;options.component=1;
        for(const auto& sample:registry.sample(algorithm,mesh,circulation,locations,options)) Near(sample.value,sample.position[0],1e-10,"ND physical component profile");
    }
    std::cout << "PASS ND curl/component and RT divergence profiles\n";
}
void DisplacementCase(RayFEM::ProfileRegistry& registry) {
    auto mesh=mfem::Mesh::MakeCartesian2D(2,2,mfem::Element::QUADRILATERAL,true);
    mesh.SetCurvature(2);
    const mfem::Vector original(*mesh.GetNodes());
    mfem::H1_FECollection collection(2,2);
    mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space),displacement(mesh.GetNodes()->FESpace());
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& x){return x[0]+2*x[1];});
    mfem::VectorFunctionCoefficient translation(2,[](const mfem::Vector&,mfem::Vector& y){y.SetSize(2);y[0]=.1;y[1]=.2;});
    field.ProjectCoefficient(coefficient);displacement.ProjectCoefficient(translation);
    RayFEM::RenderOptions options;options.displacement=&displacement;options.displacement_scale=2;
    const auto locations=RayFEM::MakeRaySamples({.35,.65,0},{1,0,0},.7,11);
    for(const std::string algorithm:{"exhaustive","coherent"}) {
        const auto actual=registry.sample(algorithm,mesh,field,locations,options);
        for(const auto& point:actual) {Check(point.valid,"Displaced geometry inverse finds interior");Near(point.value,(point.position[0]-.2)+2*(point.position[1]-.4),1e-10,"Profile evaluates original field on displaced geometry");}
    }
    for(int i=0;i<original.Size();++i) Near((*mesh.GetNodes())[i],original[i],0,"Profile must not mutate base mesh");
    mfem::H1_FECollection mismatch_collection(1,2);
    mfem::FiniteElementSpace mismatch_space(&mesh,&mismatch_collection,2);
    mfem::GridFunction mismatch(&mismatch_space);mismatch=0;
    options.displacement=&mismatch;
    Reject([&] {registry.sample("exhaustive",mesh,field,locations,options);},"Unsupported displacement FE must reject explicitly");
    std::cout << "PASS displacement inverse and unchanged base mesh\n";
}
class TestAlgorithm final : public RayFEM::ProfileAlgorithm {
public:
    mutable int calls=0;
    std::string id()const override{return "test-algorithm";}
    std::string name()const override{return "Test algorithm";}
    std::vector<RayFEM::ProfileSample> sample(mfem::Mesh&,mfem::GridFunction&,const std::vector<RayFEM::ProfileSample>& locations,const RayFEM::RenderOptions&)const override {
        ++calls;auto result=locations;
        for(size_t i=0;i<result.size();++i){result[i].value=100+i;result[i].valid=true;result[i].element=0;}
        return result;
    }
};
void RegistryAndCSV(RayFEM::ProfileRegistry& registry) {
    auto custom=std::make_shared<TestAlgorithm>();registry.registerAlgorithm(custom);
    bool listed=false;for(const auto& entry:registry.algorithms())if(entry.id==custom->id())listed=true;
    Check(listed,"Registered algorithm listed");
    auto mesh=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    mfem::H1_FECollection collection(1,2);mfem::FiniteElementSpace space(&mesh,&collection);mfem::GridFunction field(&space);field=1;
    auto locations=RayFEM::MakeRaySamples({.1,.2,0},{1,0,0},.8,3);
    auto actual=registry.sample(custom->id(),mesh,field,locations);
    Check(custom->calls==1,"Custom algorithm dispatch");
    for(size_t i=0;i<actual.size();++i)Near(actual[i].value,100+i,0,"Custom algorithm values");
    Reject([&]{registry.sample("unknown",mesh,field,locations);},"Unknown algorithm rejected");
    actual.back().valid=false;actual.back().value=std::numeric_limits<double>::quiet_NaN();actual.back().element=-1;
    RayFEM::ProfileMetadata metadata;
    metadata.field_name="temperature, \"upper\"";metadata.frame=42;metadata.time=.125;metadata.algorithm=custom->id();
    metadata.excluded_materials={2,42};
    std::ostringstream output;RayFEM::ExportProfileCSV(output,actual,metadata);
    const auto csv=output.str();
    Check(csv.find("temperature, \"\"upper\"\"")!=std::string::npos,"CSV escapes quotes and comma in field name");
    Check(csv.find("42")!=std::string::npos && csv.find("0.125")!=std::string::npos,"CSV frame and time metadata");
    Check(csv.find("test-algorithm")!=std::string::npos,"CSV algorithm metadata");
    Check(csv.find("valid")!=std::string::npos && csv.find("element")!=std::string::npos,"CSV preserves miss and parent information");
    Check(csv.starts_with("field,frame,time,view,component,algorithm,s,u,v,x,y,z,value,valid,element,excluded_materials,excluded_elements\r\n"),"CSV header includes material and element selection metadata");
    size_t selections=0;
    for(size_t offset=0;(offset=csv.find(",\"2;42\",\"\"\r\n",offset))!=std::string::npos;++offset)++selections;
    Check(selections==actual.size(),"Every CSV sample preserves the excluded material selection");
    // Every sample must remain present, including misses.
    Check(std::count(csv.begin(),csv.end(),'\n')==4,"CSV includes exactly one header and all sample rows");
    std::cout << "PASS registry dispatch and CSV metadata escaping\n";
}
void NURBSCase(RayFEM::ProfileRegistry& registry,const std::string& directory) {
    mfem::Mesh mesh((directory+"/disc-nurbs.mesh").c_str(),1,0,false);
    Check(mesh.NURBSext!=nullptr,"NURBS profile fixture");
    // A coordinate component is represented exactly by the rational basis.
    auto* node_space=mesh.GetNodes()->FESpace();
    mfem::GridFunction coordinates(node_space);coordinates=*mesh.GetNodes();
    RayFEM::RenderOptions options;options.view=RayFEM::FieldView::Component;options.component=0;
    std::vector<RayFEM::ProfileSample> locations;
    for(int e=0;e<mesh.GetNE();++e){mfem::IntegrationPoint ip;ip.Set2(.31,.63);mfem::Vector x;mesh.GetElementTransformation(e)->Transform(ip,x);locations.push_back(Location(x[0],x[1]));}
    for(const std::string algorithm:{"exhaustive","coherent"})for(const auto& point:registry.sample(algorithm,mesh,coordinates,locations,options)){Check(point.valid,"NURBS inverse finds point");Near(point.value,point.position[0],2e-9,"Rational coordinate field exact");}
    mfem::GridFunction displacement(node_space);
    for(int dof=0;dof<node_space->GetNDofs();++dof){
        displacement[node_space->DofToVDof(dof,0)]=.15;
        displacement[node_space->DofToVDof(dof,1)]=-.1;
    }
    options.displacement=&displacement;
    for(auto& point:locations){point.position[0]+=.15;point.position[1]-=.1;}
    const mfem::Vector original(*mesh.GetNodes());
    for(const std::string algorithm:{"exhaustive","coherent"})for(const auto& point:registry.sample(algorithm,mesh,coordinates,locations,options)){
        Check(point.valid,"Displaced rational NURBS inverse finds point");Near(point.value,point.position[0]-.15,2e-9,"Displaced rational coordinate field exact");
    }
    for(int i=0;i<original.Size();++i)Near((*mesh.GetNodes())[i],original[i],0,"NURBS profile leaves base nodes unchanged");
    Check(mesh.NURBSext!=nullptr,"Profiling preserves rational geometry");
    std::cout << "PASS rational NURBS profile inversion\n";
}
}
int main(int argc,char** argv){
    try{
        GeometryRequests();RayFEM::ProfileRegistry registry;
        AnalyticCases(registry,2,false);AnalyticCases(registry,2,true);AnalyticCases(registry,3,false);AnalyticCases(registry,3,true);
        DiscontinuousCase(registry);VectorViews(registry);DisplacementCase(registry);RegistryAndCSV(registry);
        if(argc>1)NURBSCase(registry,argv[1]);
        std::cout<<"All profile checks passed.\n";return 0;
    }catch(const std::exception& error){std::cerr<<"Profile test failed: "<<error.what()<<'\n';return 1;}
}
