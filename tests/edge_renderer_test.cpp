#include "../src/app/edge_renderer.hpp"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value,const char* message) {if(!value)throw std::runtime_error(message);}
bool near(Color a,Color b) {return std::abs(int(a.r)-b.r)+std::abs(int(a.g)-b.g)+std::abs(int(a.b)-b.b)<15;}
}

int main(int argc,char** argv) {
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_WINDOW_HIDDEN);
    InitWindow(800,600,"RayFEM outline test");
    if(!IsWindowReady())return 1;
    int status=0;
    {
        RayFEM::UI::EdgeRenderer renderer;renderer.Initialize();
        const auto target=LoadRenderTexture(800,600);
        try {
            auto mesh=mfem::Mesh::MakeCartesian3D(3,3,3,mfem::Element::HEXAHEDRON);
            mesh.SetCurvature(4);
            mfem::VectorFunctionCoefficient bend(3,[](const mfem::Vector& x,mfem::Vector& y) {
                y=x;y[0]+=.15*std::sin(3.141592653589793*x[1])*x[2];
            });mesh.Transform(bend);
            RayFEM::RenderOptions options;options.refinement=8;options.element_explosion=.28;
            auto lines=RayFEM::BuildElementBoundaries(mesh,options);
            for(auto& edge:lines)edge.boundary_id=7;
            auto model=renderer.Upload(lines);
            const Camera3D camera={{2.3f,2.0f,2.5f},{.5f,.5f,.5f},{0,1,0},40,CAMERA_PERSPECTIVE};
            const Color tint={40,180,230,255};
            for(bool categorical:{false,true}) {
                BeginTextureMode(target);ClearBackground(BLACK);BeginMode3D(camera);rlDisableBackfaceCulling();
                renderer.Draw(model,camera,{800,600},1,tint,categorical);
                rlEnableBackfaceCulling();EndMode3D();EndTextureMode();
                auto image=LoadImageFromTexture(target.texture);ImageFlipVertical(&image);
                auto* pixels=LoadImageColors(image);int colored=0;
                for(int i=0;i<800*600;++i)if(near(pixels[i],categorical?RayFEM::AttributeColor(7):tint))++colored;
                require(colored>1500,"Cached GPU outlines absent or incorrectly colored");
                if(argc>1) {
                    std::filesystem::create_directories(argv[1]);
                    ExportImage(image,(std::filesystem::path(argv[1])/(categorical?"exploded-boundary-outlines.png":"exploded-outlines.png")).c_str());
                }
                UnloadImageColors(pixels);UnloadImage(image);
            }
            UnloadModel(model);
            // Compare the former per-frame CPU path against cached GPU submission.
            // Timings are reported, never asserted, to avoid hardware-sensitive tests.
            const auto base=lines;
            while(lines.size()<100000)lines.insert(lines.end(),base.begin(),base.end());
            model=renderer.Upload(lines);
            auto benchmark=[&](bool cached) {
                std::vector<double> timings;
                for(int repeat=0;repeat<12;++repeat) {
                    BeginTextureMode(target);ClearBackground(BLACK);BeginMode3D(camera);rlDisableBackfaceCulling();
                    rlDrawRenderBatchActive();
                    auto start=std::chrono::steady_clock::now();
                    if(cached)renderer.Draw(model,camera,{800,600},1,tint,false);
                    else for(const auto& edge:lines) {
                        auto lift=[&](Vector3 p){return Vector3Add(p,Vector3Scale(Vector3Normalize(Vector3Subtract(camera.position,p)),.00015f));};
                        DrawLine3D(lift(edge.a),lift(edge.b),tint);
                    }
                    rlDrawRenderBatchActive();
                    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                    rlEnableBackfaceCulling();EndMode3D();EndTextureMode();
                    if(repeat>1)timings.push_back(ms);
                }
                std::sort(timings.begin(),timings.end());return timings[timings.size()/2];
            };
            const double old=benchmark(false),cached=benchmark(true);
            std::cout<<"PASS GPU exploded curved edges, theme tint and boundary colors\n"
                     <<lines.size()<<" edges: CPU submission "<<old<<" ms -> "<<cached<<" ms ("<<old/cached<<"x)\n";
            UnloadModel(model);
        } catch(const std::exception& e){std::cerr<<e.what()<<'\n';status=1;}
        UnloadRenderTexture(target);renderer.Shutdown();
    }
    CloseWindow();return status;
}
