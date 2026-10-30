#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/network/client.hpp"
#include "libRayfem/network/server.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {
constexpr double pi=3.14159265358979323846;
constexpr double amplitude=.8;
void check(bool okay,const std::string& message){if(!okay)throw std::runtime_error(message);}
void near(double a,double b,double tolerance,const std::string& message){
    if(!std::isfinite(a)||std::abs(a-b)>tolerance)throw std::runtime_error(message+": "+std::to_string(a)+" vs "+std::to_string(b));
}
std::array<double,3> axes(double time){double phase=.5*(1-std::cos(time));return {1+amplitude*phase,1-.5*amplitude*phase,1+.25*amplitude*phase};}
std::shared_ptr<RayFEM::RenderFrame> receive(RayFEM::Server& server){
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(8);
    while(std::chrono::steady_clock::now()<deadline){
        auto frames=server.fetchFrames(1);
        if(!frames.empty())return frames.front();
        check(server.getErrorCount()==0,server.getLastError());
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    throw std::runtime_error("Timed out receiving animated displacement: "+server.getLastError());
}
void unchanged(const mfem::Vector& a,const mfem::Vector& b,const std::string& what){
    check(a.Size()==b.Size(),what+" size");for(int i=0;i<a.Size();++i)near(a[i],b[i],0,what);
}
void checkSurface(const RayFEM::SurfaceData& base,const RayFEM::SurfaceData& deformed,
                  const std::array<double,3>& scale,double time,bool exact_sphere){
    check(base.positions.size()==deformed.positions.size(),"Animation must preserve surface topology");
    check(deformed.values.size()==base.values.size(),"Frame field size");
    for(std::size_t i=0;i<deformed.values.size();++i){
        double sphere=0;
        for(int d=0;d<3;++d){
            const double p=deformed.positions[3*i+d];
            near(p,scale[d]*base.positions[3*i+d],4e-7,"Animated vertex is not reference position times current axis");
            sphere+=(p/scale[d])*(p/scale[d]);
        }
        near(sphere,1,exact_sphere?2e-6:.012,"Deformed boundary must satisfy ellipsoid equation");
        near(deformed.values[i],base.values[i]+.3*std::sin(time),2e-11,"Frame field must stay attached to reference geometry");
    }
    const float base_lo[3]={base.bounds.min.x,base.bounds.min.y,base.bounds.min.z};
    const float base_hi[3]={base.bounds.max.x,base.bounds.max.y,base.bounds.max.z};
    const float low[3]={deformed.bounds.min.x,deformed.bounds.min.y,deformed.bounds.min.z};
    const float high[3]={deformed.bounds.max.x,deformed.bounds.max.y,deformed.bounds.max.z};
    for(int d=0;d<3;++d){
        near(low[d],scale[d]*base_lo[d],4e-7,"Animated minimum bound");near(high[d],scale[d]*base_hi[d],4e-7,"Animated maximum bound");
        if(exact_sphere){near(low[d],-scale[d],2e-6,"Exact NURBS ellipsoid negative axis");near(high[d],scale[d],2e-6,"Exact NURBS ellipsoid positive axis");}
    }
}
void runCase(RayFEM::Server& server,RayFEM::Client& client,const std::string& mesh_file,
             bool nurbs,mfem::Ordering::Type ordering,std::uint64_t mesh_id){
    mfem::Mesh mesh(mesh_file.c_str(),1,1);
    check(mesh.NURBSext&&mesh.GetNodes(),"Ball fixture must preserve original rational geometry");
    if(!nurbs)mesh.SetCurvature(5,false,3,ordering);
    check((mesh.NURBSext!=nullptr)==nurbs,"Requested geometry type");
    auto* nodes=mesh.GetNodes();const auto& node_space=*nodes->FESpace();
    const mfem::Vector original_nodes(*nodes);
    mfem::FiniteElementSpace scalar_space(&mesh,node_space.FEColl());
    mfem::FiniteElementSpace vector_space(&mesh,node_space.FEColl(),3,ordering);
    mfem::GridFunction scalar(&scalar_space),coordinates(&vector_space),displacement(&vector_space);
    std::vector<double> reference_scalar(scalar.Size());
    for(int dof=0;dof<node_space.GetNDofs();++dof){
        double value=0;
        for(int d=0;d<3;++d){
            const double coordinate=(*nodes)[node_space.DofToVDof(dof,d)];
            coordinates[vector_space.DofToVDof(dof,d)]=coordinate;value+=(d+1)*coordinate;
        }
        scalar[dof]=reference_scalar[dof]=value;
    }
    RayFEM::RenderOptions options;options.refinement=6;
    const auto base=RayFEM::BuildSurface(mesh,&scalar,options);
    check(!base.positions.empty(),"Sphere baseline surface");
    check(client.sendMesh(mesh_id,mesh),client.getLastError()); // Exactly one registration for all 33 states.
    std::vector<std::shared_ptr<RayFEM::RenderFrame>> history;
    const long sequence=mesh.GetSequence();
    for(int step=0;step<=32;++step){
        const double time=step*pi/16;const auto scale=axes(time);
        for(int dof=0;dof<node_space.GetNDofs();++dof){
            scalar[dof]=reference_scalar[dof]+.3*std::sin(time);
            for(int d=0;d<3;++d)displacement[vector_space.DofToVDof(dof,d)]=(scale[d]-1)*original_nodes[node_space.DofToVDof(dof,d)];
        }
        RayFEM::FrameMetadata metadata;metadata.frame_index=step;metadata.sim_time=time;
        check(client.sendFrame(mesh_id,{{"scalar",&scalar},{"reference coordinates",&coordinates}},metadata,&displacement),client.getLastError());
        auto frame=receive(server);
        check(frame->mesh_id==mesh_id&&frame->metadata.frame_index==std::uint32_t(step),"Animation frame identity");
        check(frame->fields.size()==2&&frame->displacement,"Atomic fields and displacement");
        check(frame->displacement->FESpace()->GetOrdering()==ordering,"Displacement ordering must survive transport");
        check((frame->mesh->NURBSext!=nullptr)==nurbs,"Transport preserves geometry type");
        if(!history.empty()){
            check(frame->mesh.get()==history.front()->mesh.get(),"Geometry must be reused across time states");
            check(frame->mesh_revision==history.front()->mesh_revision,"Mesh must not be re-registered between frames");
            check(frame->fields[0].gf.get()!=history.front()->fields[0].gf.get(),"Frame fields require independent storage");
            check(frame->displacement.get()!=history.front()->displacement.get(),"Frame displacement requires independent storage");
        }
        options.displacement=frame->displacement.get();
        const auto rendered=RayFEM::BuildSurface(*frame->mesh,frame->fields[0].gf.get(),options);
        checkSurface(base,rendered,scale,time,nurbs);
        mfem::IntegrationPoint ip;ip.Set3(.23,.37,.61);
        for(int e=0;e<mesh.GetNE();++e){
            mfem::Vector original,value,displaced;
            mesh.GetElementTransformation(e)->Transform(ip,original);
            frame->mesh->GetElementTransformation(e)->Transform(ip,displaced);
            frame->displacement->GetVectorValue(e,ip,value);displaced+=value;
            for(int d=0;d<3;++d)near(displaced[d],scale[d]*original[d],3e-12,"Interior MFEM mapping under time-varying affine displacement");
        }
        unchanged(*nodes,original_nodes,"Producer reference nodes changed");
        unchanged(*frame->mesh->GetNodes(),original_nodes,"Receiver reference nodes changed");
        check(mesh.GetSequence()==sequence,"Animation must not alter mesh sequence");
        history.push_back(std::move(frame));
    }
    // Revisit snapshots in a deliberately nonchronological order. This detects
    // accumulating offsets and aliasing of fields or displacement coefficients.
    for(int step:{16,0,32,8,24,16,0}){
        const auto& frame=history[step];options.displacement=frame->displacement.get();
        const auto replay=RayFEM::BuildSurface(*frame->mesh,frame->fields[0].gf.get(),options);
        checkSurface(base,replay,axes(frame->metadata.sim_time),frame->metadata.sim_time,nurbs);
    }
    near(history.front()->displacement->Norml2(),0,0,"Initial state is a sphere");
    check(history[16]->displacement->Norml2()>.5,"Midcycle displacement must be substantial");
    near(history.back()->displacement->Norml2(),0,1e-12,"Full cycle returns to the original sphere");
    options.displacement=history[16]->displacement.get();options.displacement_scale=.5;
    const auto half=RayFEM::BuildSurface(*history[16]->mesh,history[16]->fields[0].gf.get(),options);
    checkSurface(base,half,{1.4,.8,1.1},pi,nurbs);
    check(server.getDroppedFrameCount()==0&&server.getErrorCount()==0,"Animated roundtrip must not lose frames");
    std::cout<<"PASS 33-state "<<(nurbs?"NURBS":"order-5 H1")<<" sphere-to-ellipsoid, "
             <<(ordering==mfem::Ordering::byNODES?"byNODES":"byVDIM")<<", one mesh registration, replay and scale\n";
}
}
int main(int argc,char** argv){try{
    const std::string mesh=argc>1?argv[1]:"subprojects/mfem/data/ball-nurbs.mesh";
    const size_t port=35000+(getpid()%15000);RayFEM::Server server("127.0.0.1",port);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!server.ready()&&server.getErrorCount()==0&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(2));
    check(server.ready(),"Receiver startup: "+server.getLastError());RayFEM::Client client("127.0.0.1",port);
    std::uint64_t id=1;
    for(bool nurbs:{true,false})for(auto ordering:{mfem::Ordering::byNODES,mfem::Ordering::byVDIM})runCase(server,client,mesh,nurbs,ordering,id++);
    check(server.getReceivedFrameCount()==132,"All 132 time states must arrive");
    std::cout<<"Animated displacement tests passed: 132 roundtrip states across four geometry/ordering cases\n";
}catch(const std::exception& error){std::cerr<<"Displacement test failed: "<<error.what()<<'\n';return 1;}}
