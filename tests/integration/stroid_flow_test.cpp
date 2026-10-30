// Optional integration fixture: generate the meshes with the real Stroid CLI,
// then pass their paths. Stroid is not a required RayFEM build dependency.
#include "libRayfem/vector/flow.hpp"
#include "libRayfem/network/client.hpp"
#include "libRayfem/network/server.hpp"
#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
using namespace RayFEM;
void Check(bool okay,const std::string& message){if(!okay)throw std::runtime_error(message);}
void Near(double a,double b,double tolerance,const std::string& message){if(!std::isfinite(a)||std::abs(a-b)>tolerance)throw std::runtime_error(message+": "+std::to_string(a)+" vs "+std::to_string(b));}
double SpheroidRadius(const ProfilePosition& p){return std::sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]/(.75*.75));}
Vector3 Point(const ProfilePosition& p){return {float(p[0]),float(p[1]),float(p[2])};}
std::shared_ptr<RenderFrame> Receive(Server& server){
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
    while(std::chrono::steady_clock::now()<deadline){auto frames=server.fetchFrames(1);if(!frames.empty())return frames.front();Check(!server.getErrorCount(),server.getLastError());std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    throw std::runtime_error("Stroid stream receive timeout: "+server.getLastError());
}
void RotationValues(const StreamlineResult& flow,const std::array<double,3>& axes={1,1,1}){
    Check(!flow.lines.empty()&&!flow.diagnostics.budget_exhausted,"Rotation traces must finish within their budgets");
    for(const auto& line:flow.lines){
        Check(line.points.size()>15,"Rotation streamline is too short");
        const auto& start=line.points.front();
        const double invariant=start.position[0]*start.position[0]/axes[0]+start.position[1]*start.position[1]/axes[1];
        for(const auto& p:line.points){
            Check(p.element>=0,"Every streamline point needs its containing parent");
            Near(p.vector[0],-p.position[1]/axes[1],2e-9,"Actual FE rotation x component");
            Near(p.vector[1],p.position[0]/axes[0],2e-9,"Actual FE rotation y component");
            Near(p.vector[2],0,2e-11,"Actual FE rotation z component");
            Near(p.position[2],start.position[2],2e-9,"Rotation preserves height");
            Near(p.position[0]*p.position[0]/axes[0]+p.position[1]*p.position[1]/axes[1],invariant,2e-5,"Integrated trajectory invariant");
            Near(p.magnitude,std::hypot(p.vector[0],p.vector[1]),2e-10,"Physical vector magnitude");
        }
    }
}
void Screenshot(const std::filesystem::path& path,mfem::Mesh& mesh,mfem::GridFunction& scalar,
                const StreamlineResult& flow,const RenderOptions& render,const std::string& label){
    const auto surface=BuildSurface(mesh,&scalar,render);Model model=UploadSurface(surface);
    Check(model.meshCount>0,"Stroid surface GPU upload");
    const auto edges=BuildElementBoundaries(mesh,render);
    const float radius=.5f*Vector3Distance(surface.bounds.min,surface.bounds.max);
    Camera3D camera{{radius*1.6f,radius*1.0f,radius*2.3f},{0,0,0},{0,1,0},38,CAMERA_PERSPECTIVE};
    const auto target=LoadRenderTexture(960,760);Check(target.id,"Stroid offscreen render target");
    BeginTextureMode(target);ClearBackground({16,23,33,255});BeginMode3D(camera);
    DrawModelWires(model,{0,0,0},1,{39,63,82,255});
    for(const auto& edge:edges)DrawLine3D(edge.a,edge.b,{81,129,163,255});
    for(const auto& line:flow.lines)for(std::size_t i=1;i<line.points.size();++i)
        DrawLine3D(Point(line.points[i-1].position),Point(line.points[i].position),{64,245,169,255});
    EndMode3D();DrawText(label.c_str(),22,20,22,{224,236,246,255});
    DrawText("Actual Stroid P5 geometry | evaluated FE rotation streamlines",22,53,17,{144,177,196,255});EndTextureMode();
    Image image=LoadImageFromTexture(target.texture);Check(image.data,"Stroid framebuffer readback");ImageFlipVertical(&image);
    Color* pixels=LoadImageColors(image);Check(pixels,"Stroid framebuffer colors");
    std::size_t colored=0,green=0;
    for(int y=95;y<image.height-20;++y)for(int x=20;x<image.width-20;++x){const auto& p=pixels[y*image.width+x];if(int(p.r)+p.g+p.b>150)++colored;if(p.g>180&&p.g>p.r+50&&p.g>p.b+30)++green;}
    UnloadImageColors(pixels);const bool saved=ExportImage(image,path.string().c_str());UnloadImage(image);UnloadRenderTexture(target);UnloadModel(model);
    Check(saved&&colored>1000&&green>25,"Stroid screenshot must contain actual mesh and flow pixels");
}

void RunMesh(const std::string& file,bool nonconforming,std::uint64_t id,Server& server,Client& client,
             const std::filesystem::path& output,bool graphics){
    mfem::Mesh mesh(file.c_str(),1,1);Check(mesh.Dimension()==3&&mesh.SpaceDimension()==3&&mesh.GetNodes(),"Expected a 3D Stroid curvilinear mesh");
    Check(mesh.Nonconforming()==nonconforming,"Stroid conformity must match the generation configuration");
    const auto* geometry=mesh.GetNodes()->FESpace();Check(geometry->GetMaxElementOrder()==5,"Actual Stroid fixture must have order-5 geometry");
    std::map<int,int> materials;double minimum_jacobian=std::numeric_limits<double>::infinity();
    for(int e=0;e<mesh.GetNE();++e){++materials[mesh.GetAttribute(e)];auto* transform=mesh.GetElementTransformation(e);const auto& rule=mfem::IntRules.Get(mesh.GetElementBaseGeometry(e),10);
        for(int q=0;q<rule.GetNPoints();++q){transform->SetIntPoint(&rule.IntPoint(q));const double determinant=transform->Jacobian().Det();Check(std::isfinite(determinant)&&determinant>0,"Stroid high-order cell has invalid orientation");minimum_jacobian=std::min(minimum_jacobian,determinant);}}
    Check(materials.contains(11)&&materials.contains(17)&&materials.size()==(nonconforming?3u:2u),"Preserve real Stroid material IDs");if(nonconforming)Check(materials.contains(23),"Real vacuum material must exist");
    const mfem::Vector original_nodes(*mesh.GetNodes());
    mfem::FiniteElementSpace scalar_space(&mesh,geometry->FEColl()),vector_space(&mesh,geometry->FEColl(),3,mfem::Ordering::byVDIM);
    mfem::GridFunction scalar(&scalar_space),rotation(&vector_space),radial(&vector_space),displacement(&vector_space);
    for(int i=0;i<scalar_space.GetNDofs();++i){const double x=(*mesh.GetNodes())[geometry->DofToVDof(i,0)],y=(*mesh.GetNodes())[geometry->DofToVDof(i,1)],z=(*mesh.GetNodes())[geometry->DofToVDof(i,2)];scalar[i]=1+x+2*y+3*z;
        rotation[vector_space.DofToVDof(i,0)]=-y;rotation[vector_space.DofToVDof(i,1)]=x;rotation[vector_space.DofToVDof(i,2)]=0;
        radial[vector_space.DofToVDof(i,0)]=x;radial[vector_space.DofToVDof(i,1)]=y;radial[vector_space.DofToVDof(i,2)]=z;}
    RenderOptions render;render.refinement=5;if(nonconforming)render.excluded_materials={23};
    const auto stellar_surface=BuildSurface(mesh,&scalar,render);Check(!stellar_surface.values.empty(),"Actual Stroid stellar surface must render");
    double shape_error=0;for(std::size_t i=0;i<stellar_surface.values.size();++i){ProfilePosition p{stellar_surface.positions[3*i],stellar_surface.positions[3*i+1],stellar_surface.positions[3*i+2]};shape_error=std::max(shape_error,std::abs(SpheroidRadius(p)-1));Near(stellar_surface.values[i],1+p[0]+2*p[1]+3*p[2],5e-7,"Stroid surface parent-field evaluation");}
    Check(shape_error<2e-3,"P5 projected boundary must approximate the requested spheroid");
    FlowOptions options;options.seed_resolution=5;options.max_seeds=48;options.max_points=800;options.max_steps=100;options.step_size=.015;options.max_length=.9;options.bidirectional=false;options.max_location_tests=1000000;
    const auto cloud=SampleVectorField(mesh,rotation,render,options);Check(cloud.points.size()>8&&!cloud.diagnostics.budget_exhausted,"Distributed Stroid field samples");
    for(const auto& p:cloud.points){Near(p.vector[0],-p.position[1],2e-9,"Stroid cloud FE x component");Near(p.vector[1],p.position[0],2e-9,"Stroid cloud FE y component");Check(mesh.GetAttribute(p.element)!=23,"Cloud must honor real vacuum exclusion");}
    const std::vector<ProfilePosition> seeds{{.55,0,.1},{.12,0,.02},{.4,.2,-.2}};
    const auto flow=BuildStreamlines(mesh,rotation,render,options,seeds);RotationValues(flow);
    if(nonconforming){
        auto radial_options=options;radial_options.step_size=.02;radial_options.max_steps=140;radial_options.max_length=2;radial_options.max_points=400;
        auto all=render;all.excluded_materials.clear();const auto through=BuildStreamlines(mesh,radial,all,radial_options,{{.12,.025,.015}});
        Check(!through.lines.empty()&&!through.diagnostics.budget_exhausted,"Radial NC material traversal");std::map<int,int> visited;
        for(const auto& line:through.lines)for(const auto& p:line.points)++visited[mesh.GetAttribute(p.element)];
        Check(visited.contains(11)&&visited.contains(17)&&visited.contains(23),"Flow must cross actual core, envelope and vacuum parents");
        const auto stellar=BuildStreamlines(mesh,radial,render,radial_options,{{.12,.025,.015}});Check(!stellar.lines.empty(),"Selected stellar radial trace");double farthest=0;
        for(const auto& line:stellar.lines)for(const auto& p:line.points){Check(mesh.GetAttribute(p.element)!=23,"Trace entered excluded Stroid vacuum");farthest=std::max(farthest,SpheroidRadius(p.position));}
        Check(farthest>.9&&farthest<1.005,"Selected radial flow stops at stellar surface");
        auto no_core=render;no_core.excluded_materials.push_back(11);const auto rejected=BuildStreamlines(mesh,radial,no_core,radial_options,{{.12,.025,.015}});Check(rejected.lines.empty(),"Seed inside hidden core must be rejected");
    }
    Check(client.sendMesh(id,mesh),client.getLastError());std::vector<std::shared_ptr<RenderFrame>> history;
    for(int step=0;step<3;++step){const double amount=step==1?.1:0;const double scales[]={1+amount,1-amount,1+.5*amount};
        for(int i=0;i<geometry->GetNDofs();++i)for(int d=0;d<3;++d)displacement[vector_space.DofToVDof(i,d)]=(scales[d]-1)*original_nodes[geometry->DofToVDof(i,d)];
        FrameMetadata meta;meta.frame_index=step;meta.sim_time=.1*step;
        Check(client.sendFrame(id,{{"Rotation",&rotation},{"Scalar",&scalar}},meta,&displacement),client.getLastError());auto frame=Receive(server);
        Check(frame->fields.size()==2&&frame->displacement&&frame->mesh_id==id,"Atomic actual-Stroid frame");
        if(!history.empty())Check(frame->mesh==history.front()->mesh&&frame->mesh_revision==history.front()->mesh_revision,"Actual Stroid geometry sent only once");
        Check(frame->mesh->Nonconforming()==nonconforming&&frame->mesh->GetNodes()->FESpace()->GetMaxElementOrder()==5,"Transport preserves NC hierarchy and P5 geometry");history.push_back(std::move(frame));}
    auto displayed=render;displayed.displacement=history[1]->displacement.get();
    const auto displaced_flow=BuildStreamlines(*history[1]->mesh,*history[1]->fields[0].gf,displayed,options,{{.605,0,.105}});RotationValues(displaced_flow,{1.1,.9,1.05});
    if(graphics)Screenshot(output/(nonconforming?"stroid-order5-nc-flow.png":"stroid-order5-flow.png"),mesh,scalar,flow,render,nonconforming?"Stroid P5 / nonconforming / vacuum hidden":"Stroid P5 / projected spheroid");
    for(int i=0;i<original_nodes.Size();++i)Near((*mesh.GetNodes())[i],original_nodes[i],0,"Stroid tests mutated source nodes");
    Check(server.getErrorCount()==0&&server.getDroppedFrameCount()==0,"Actual Stroid streaming must not drop or reject frames");
    std::cout<<"PASS actual Stroid "<<(nonconforming?"NC":"conforming")<<": "<<mesh.GetNE()<<" hexes, geometry DOFs "<<geometry->GetVSize()<<" / true "<<geometry->GetTrueVSize()<<", materials";
    for(auto [attribute,count]:materials)std::cout<<' '<<attribute<<':'<<count;
    std::cout<<", min sampled Jacobian "<<minimum_jacobian<<", surface shape error "<<shape_error<<", cloud "<<cloud.points.size()<<", rotation paths "<<flow.lines.size()<<", 3 frames from one mesh\n";
}
}

int main(int argc,char** argv){try{
    Check(argc>=3,"Usage: stroid_flow_test <stroid-order5.mesh> <stroid-order5-nc.mesh> [output_dir] [--graphics]");
    const std::filesystem::path output=argc>3?argv[3]:"build-rayfem/test-output";const bool graphics=argc>4&&std::string(argv[4])=="--graphics";
    if(graphics){SetTraceLogLevel(LOG_WARNING);SetConfigFlags(FLAG_WINDOW_HIDDEN|FLAG_MSAA_4X_HINT);InitWindow(960,760,"RayFEM actual Stroid integration");Check(IsWindowReady(),"Stroid graphics window");}
    const std::size_t port=45000+getpid()%15000;Server server("127.0.0.1",port);const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while(!server.ready()&&!server.getErrorCount()&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(2));Check(server.ready(),server.getLastError());Client client("127.0.0.1",port);
    RunMesh(argv[1],false,71,server,client,output,graphics);RunMesh(argv[2],true,72,server,client,output,graphics);Check(server.getReceivedFrameCount()==6,"All six Stroid frames arrived");
    if(graphics)CloseWindow();std::cout<<"Actual Stroid flow/render/stream integration passed\n";return 0;
}catch(const std::exception& e){std::cerr<<"Stroid integration failed: "<<e.what()<<'\n';if(IsWindowReady())CloseWindow();return 1;}}
