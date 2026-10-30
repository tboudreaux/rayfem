#include "panels/flow_panel.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width=900,height=700;
Color background{20,26,35,255};
void require(bool ok,const std::string& message){if(!ok)throw std::runtime_error(message);}
struct Window {
    Window(){SetTraceLogLevel(LOG_WARNING);SetConfigFlags(FLAG_WINDOW_HIDDEN|FLAG_MSAA_4X_HINT);InitWindow(width,height,"RayFEM flow graphics checks");require(IsWindowReady(),"Graphics context unavailable");}
    ~Window(){CloseWindow();}
};
using Pixels=std::vector<Color>;
class Renderer {
    RenderTexture2D target=LoadRenderTexture(width,height);
    RayFEM::UI::StreamlineRenderer streamlines;
public:
    Renderer(){streamlines.Initialize();}
    ~Renderer(){streamlines.Shutdown();UnloadRenderTexture(target);}
    Pixels draw(const RayFEM::SurfaceData& surface,Camera3D camera,FlowPanel& flow,
                bool opaque,const std::filesystem::path& file={},const RayFEM::ClipPlane* plane=nullptr) {
        BeginTextureMode(target);ClearBackground(background);BeginMode3D(camera);
        rlDisableBackfaceCulling();
        if(opaque){auto model=RayFEM::UploadSurface(surface);DrawModel(model,{0,0,0},1,WHITE);rlDrawRenderBatchActive();UnloadModel(model);}
        flow.Draw(camera,0,streamlines,{float(width),float(height)},nullptr,plane);
        rlEnableBackfaceCulling();EndMode3D();EndTextureMode();
        BeginDrawing();ClearBackground(background);DrawTextureRec(target.texture,{0,0,float(width),-float(height)},{0,0},WHITE);EndDrawing();
        auto image=LoadImageFromTexture(target.texture);require(image.data,"Readback failed");ImageFlipVertical(&image);
        auto colors=LoadImageColors(image);require(colors,"Pixel conversion failed");Pixels result(colors,colors+width*height);UnloadImageColors(colors);
        if(!file.empty())require(ExportImage(image,file.string().c_str()),"Screenshot failed");UnloadImage(image);return result;
    }
};
std::size_t difference(const Pixels& a,const Pixels& b){std::size_t result=0;for(std::size_t i=0;i<a.size();++i)if(std::abs(int(a[i].r)-b[i].r)+std::abs(int(a[i].g)-b[i].g)+std::abs(int(a[i].b)-b[i].b)>30)++result;return result;}
std::size_t coverage(const Pixels& image,int x0=0,int x1=width,int y0=0,int y1=height){std::size_t n=0;for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){const auto c=image[y*width+x];if(std::abs(int(c.r)-background.r)+std::abs(int(c.g)-background.g)+std::abs(int(c.b)-background.b)>30)++n;}return n;}
void checkHiddenCellSnapshot(Renderer& renderer,const std::filesystem::path& output) {
    auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(3,1,mfem::Element::QUADRILATERAL,true,2,1.4));
    // The middle cell is narrower than one integration step. Some old
    // segments cross it with both endpoint parent IDs in the outer cells.
    constexpr double boundaries[]{-1,-.02,.02,1};
    for(int i=0;i<mesh->GetNV();++i) {
        auto* vertex=mesh->GetVertex(i);
        vertex[0]=boundaries[std::lround(vertex[0]*1.5)];vertex[1]-=.7;
    }
    mesh->SetCurvature(3);
    mfem::H1_FECollection h1(3,2);mfem::FiniteElementSpace space(mesh.get(),&h1,2);
    mfem::GridFunction field(&space);
    mfem::VectorFunctionCoefficient velocity(2,[](const mfem::Vector&,mfem::Vector& value){value.SetSize(2);value[0]=1;value[1]=0;});
    field.ProjectCoefficient(velocity);
    auto frame=std::make_shared<RayFEM::RenderFrame>();frame->mesh=mesh;
    RayFEM::RenderOptions options;options.refinement=3;
    const auto surface=RayFEM::BuildSurface(*mesh,nullptr,options);
    const float scale=Vector3Distance(surface.bounds.min,surface.bounds.max);
    Camera3D camera{{0,0,scale*1.8f},{0,0,0},{0,1,0},38,CAMERA_PERSPECTIVE};
    RayFEM::FlowOptions samples;samples.seed_count=samples.max_seeds=64;
    samples.step_size=scale*.02f;samples.max_length=scale*1.5f;
    const auto paths=RayFEM::BuildStreamlines(*mesh,field,options,samples);
    bool bridges=false;
    for(const auto& line:paths.lines)for(std::size_t i=1;i<line.points.size();++i)
        bridges|=line.points[i-1].element==0 && line.points[i].element==2;
    require(bridges,"Thin-cell fixture needs a segment whose endpoints skip the middle cell");

    FlowPanel flow;flow.UpdateContext(frame,&field,"uniform",options);flow.SetBounds(surface.bounds,mesh);
    flow.SetMode(2);flow.RequestUpdate();
    const auto full=renderer.draw(surface,camera,flow,false,output/"thin-cell-before.png");
    const auto centerCoverage=[](const Pixels& image){return coverage(image,width/2-1,width/2+2);};
    require(centerCoverage(full)>40,"Thin-cell fixture has no visible crossing paths");
    options.excluded_elements={1};flow.UpdateContext(frame,&field,"uniform",options);
    // Do not request a refresh: a throttled old snapshot must not bridge the
    // new exclusion. A fast automatic rebuild is also valid. The narrow
    // center strip stays away from round caps at the two retained boundaries.
    const auto pending=renderer.draw(surface,camera,flow,false,output/"thin-cell-hidden-preview.png");
    require(centerCoverage(pending)==0,"A stale flow snapshot bridges the newly hidden middle cell");
    flow.RequestUpdate();
    const auto hidden=renderer.draw(surface,camera,flow,false,output/"thin-cell-hidden-updated.png");
    require(centerCoverage(hidden)==0,"Updated streamlines cross an excluded cell");
    require(coverage(hidden,0,width/2-10)>100 && coverage(hidden,width/2+10,width)>100,
            "Hiding one cell removed flow from retained neighbors");
    options.excluded_elements.clear();flow.UpdateContext(frame,&field,"uniform",options);flow.RequestUpdate();
    const auto restored=renderer.draw(surface,camera,flow,false,output/"thin-cell-restored.png");
    require(centerCoverage(restored)>40,"Restoring a hidden cell did not restore crossing streamlines");
    std::cout<<"PASS flow graphics thin-cell exclusion: stale snapshot suppression, retained paths and restore\n";
}
void check(Renderer& renderer,std::shared_ptr<mfem::Mesh> mesh,const std::string& name,const std::filesystem::path& output){
    const mfem::Vector original(*mesh->GetNodes());
    mfem::H1_FECollection h1(5,mesh->Dimension());
    const auto* collection=mesh->NURBSext?mesh->GetNodes()->FESpace()->FEColl():&h1;
    mfem::FiniteElementSpace space(mesh.get(),collection,mesh->SpaceDimension());
    mfem::GridFunction field(&space);
    mfem::VectorFunctionCoefficient swirl(mesh->SpaceDimension(),[](const mfem::Vector& x,mfem::Vector& y){y.SetSize(x.Size());y=0.;y[0]=-x[1];y[1]=x[0];});
    if(mesh->NURBSext) {
        const auto* nodes=mesh->GetNodes();const auto* nodeSpace=nodes->FESpace();
        for(int i=0;i<space.GetNDofs();++i){
            field[space.DofToVDof(i,0)]=-(*nodes)[nodeSpace->DofToVDof(i,1)];
            field[space.DofToVDof(i,1)]=(*nodes)[nodeSpace->DofToVDof(i,0)];
            if(mesh->SpaceDimension()==3)field[space.DofToVDof(i,2)]=0;
        }
    } else field.ProjectCoefficient(swirl);
    auto frame=std::make_shared<RayFEM::RenderFrame>();frame->mesh=mesh;
    RayFEM::RenderOptions options;options.refinement=6;
    auto surface=RayFEM::BuildSurface(*mesh,nullptr,options);
    // This fixture measures depth visibility, independently of the colormap.
    // Coloring the occluder with this same field makes correctly overlaid
    // arrows nearly identical to the surface underneath (especially in 2D).
    for(std::size_t i=0;i<surface.colors.size();i+=4)
        surface.colors[i]=surface.colors[i+1]=surface.colors[i+2]=110;
    const auto center=Vector3Scale(Vector3Add(surface.bounds.min,surface.bounds.max),.5f);
    const float radius=.5f*Vector3Distance(surface.bounds.min,surface.bounds.max);
    Camera3D camera{Vector3Add(center,{0,0,radius*3.6f}),center,{0,1,0},38,CAMERA_PERSPECTIVE};
    FlowPanel flow;flow.UpdateContext(frame,&field,"rotation",options);flow.SetBounds(surface.bounds,mesh);
    const auto blank=renderer.draw(surface,camera,flow,false);
    const auto opaque=renderer.draw(surface,camera,flow,true);
    flow.SetMode(1);flow.RequestUpdate();
    const auto arrows=renderer.draw(surface,camera,flow,false,output/(name+"-arrows.png"));
    require(coverage(arrows)>350,name+": too few visible arrows");
    for(int y=0;y<2;++y)for(int x=0;x<2;++x)require(coverage(arrows,x*width/2,(x+1)*width/2,y*height/2,(y+1)*height/2)>35,name+": arrows fail to cover every domain quadrant");
    const auto overlay=renderer.draw(surface,camera,flow,true,output/(name+"-arrows-through-surface.png"));
    const auto overlayPixels=difference(opaque,overlay);
    require(overlayPixels>250,name+": opaque mesh hides interior arrows ("+std::to_string(overlayPixels)+" changed pixels)");
    const bool embedded=mesh->Dimension()!=mesh->SpaceDimension();
    flow.SetMode(embedded?3:2);flow.RequestUpdate();
    const auto paths=renderer.draw(surface,camera,flow,false,output/(name+(embedded?"-surface-arrows-both.png":"-streamlines.png")));
    require(coverage(paths)>350,name+": flow is empty or too sparse");
    if(!embedded)require(difference(paths,arrows)>350,name+": streamline and arrow modes are indistinguishable");
    else require(difference(paths,arrows)==0,name+": unsupported surface streamlines must preserve supported arrows");
    const RayFEM::ClipPlane cut{{center.x,0,0},{1,0,0}};flow.RequestUpdate();
    const auto clipped=renderer.draw(surface,camera,flow,false,output/(name+"-flow-clipped.png"),&cut);
    require(coverage(clipped)>100,name+": clipping removed all flow");
    require(coverage(clipped,width/2+4,width)==0,name+": flow leaked through clipping plane");
    for(int i=0;i<mesh->attributes.Size();++i)options.excluded_materials.push_back(mesh->attributes[i]);
    flow.UpdateContext(frame,&field,"rotation",options);flow.RequestUpdate();
    const auto hidden=renderer.draw(surface,camera,flow,false);
    require(difference(blank,hidden)==0,name+": excluded materials retain flow marks");
    options.excluded_materials.clear();flow.UpdateContext(frame,&field,"rotation",options);flow.SetMode(1);flow.RequestUpdate();
    const auto restored=renderer.draw(surface,camera,flow,false);
    require(coverage(restored)>350,name+": flow did not recover after restoring regions");
    for(int i=0;i<original.Size();++i)require(original[i]==(*mesh->GetNodes())[i],name+": flow mutated geometry");
    std::cout<<"PASS flow graphics "<<name<<": distributed arrows, visible interior, "<<(embedded?"surface arrow fallback":"streamlines")<<", clipping, exclusion and restore\n";
}
}
int main(int argc,char** argv){try{
    const std::filesystem::path output=argc>1?argv[1]:"build-rayfem/test-output";std::filesystem::create_directories(output);
    Window window;Renderer renderer;
    for(auto theme:{RayFEM::UI::ThemeMode::Dark,RayFEM::UI::ThemeMode::Light}) {
    RayFEM::UI::ApplyTheme(theme);background=RayFEM::UI::Palette().viewport_background;
    const auto themeOutput=output/(theme==RayFEM::UI::ThemeMode::Light?"light":"dark");std::filesystem::create_directories(themeOutput);
    checkHiddenCellSnapshot(renderer,themeOutput);
    auto quad=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true,2,2));quad->SetCurvature(5);
    mfem::VectorFunctionCoefficient center(2,[](const mfem::Vector& x,mfem::Vector& y){y=x;y-=1.;});quad->Transform(center);
    check(renderer,quad,"single-p5-element",themeOutput);
    auto embedded=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true,2,2));embedded->SetCurvature(5,false,3);
    mfem::VectorFunctionCoefficient paraboloid(3,[](const mfem::Vector& x,mfem::Vector& y){y.SetSize(3);y[0]=x[0]-1;y[1]=x[1]-1;y[2]=.15*(1-y[0]*y[0]-y[1]*y[1]);});embedded->Transform(paraboloid);
    check(renderer,embedded,"embedded-p5-surface",themeOutput);
    for(int i=2;i<argc;++i){const std::filesystem::path path=argv[i];check(renderer,std::make_shared<mfem::Mesh>(path.string().c_str(),1,1),path.stem().string(),themeOutput);}
    }
    std::cout<<"Flow graphics integration passed in dark and light themes\n";
}catch(const std::exception& error){std::cerr<<"Flow graphics failed: "<<error.what()<<'\n';return 1;}}
