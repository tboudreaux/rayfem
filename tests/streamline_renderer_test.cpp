#include "../src/app/streamline_renderer.hpp"
#include "raymath.h"
#include "rlgl.h"
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width=800,height=600;
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
struct OwnedModel {
    Model value{};
    ~OwnedModel(){if(value.meshCount)UnloadModel(value);}
};
struct Pixels {
    std::vector<Color> colors;
    Color Strongest(Vector2 p,int radius=2) const {
        Color best{};int strongest=-1;
        for(int y=std::max(0,int(p.y)-radius);y<=std::min(height-1,int(p.y)+radius);++y)
            for(int x=std::max(0,int(p.x)-radius);x<=std::min(width-1,int(p.x)+radius);++x) {
                const auto c=colors[y*width+x];
                if(int(c.r)>strongest){best=c;strongest=c.r;}
            }
        return best;
    }
    int Colored() const {
        return std::count_if(colors.begin(),colors.end(),[](Color c){return c.r>10||c.g>10||c.b>10;});
    }
};
}

int main(int argc,char** argv) {
    using namespace RayFEM::UI;
    SetTraceLogLevel(LOG_WARNING);SetConfigFlags(FLAG_WINDOW_HIDDEN);
    InitWindow(width,height,"RayFEM streamline renderer test");
    if(!IsWindowReady())return 1;
    int status=0;
    {
        StreamlineRenderer renderer;
        const auto target=LoadRenderTexture(width,height);
        try {
            renderer.Initialize();
            const Color base={25,95,150,255};
            const Camera3D camera={{0,0,5},{0,0,0},{0,1,0},4,CAMERA_ORTHOGRAPHIC};
            const std::vector<StreamlineSegment> segments={
                {{-1,-.75f,0},{1,-.75f,0},0,.5f,0,base,base},
                {{1,-.75f,0},{1,1.25f,0},.5f,1,0,base,base}};
            OwnedModel model{renderer.Upload(segments)};
            StreamlineStyle style;style.cycles_per_second=1;style.line_width=3;style.pulse_width=7;
            auto pixel=[&](Vector3 p){return GetWorldToScreenEx(p,camera,width,height);};
            auto render=[&](const Model& drawing,const Camera3D& view,double time,const StreamlineStyle& appearance,
                            const char* name=nullptr,bool occluder=false,bool through=false,Color background=BLACK) {
                BeginTextureMode(target);ClearBackground(background);BeginMode3D(view);
                if(occluder)DrawCube({0,0,.5f},3.5f,3.5f,.2f,BLACK);
                rlDrawRenderBatchActive();
                if(through){rlDisableDepthTest();rlDisableDepthMask();}
                renderer.Draw(drawing,view,{width,height},time,appearance);
                if(through){rlEnableDepthMask();rlEnableDepthTest();}
                EndMode3D();EndTextureMode();
                auto image=LoadImageFromTexture(target.texture);ImageFlipVertical(&image);
                auto* colors=LoadImageColors(image);
                Pixels result{{colors,colors+width*height}};
                if(argc>1 && name) {
                    std::filesystem::create_directories(argv[1]);
                    ExportImage(image,(std::filesystem::path(argv[1])/name).c_str());
                }
                UnloadImageColors(colors);UnloadImage(image);return result;
            };
            const auto first=render(model.value,camera,.25,style,"streamline-pulse-first-arm.png");
            const auto second=render(model.value,camera,.75,style,"streamline-pulse-second-arm.png");
            require(first.Strongest(pixel({0,-.75f,0})).r>180,"Pulse is absent from first arm");
            require(first.Strongest(pixel({1,.25f,0})).r<75,"Pulse incorrectly crosses to second arm");
            require(second.Strongest(pixel({1,.25f,0})).r>180,"Pulse does not travel around the bend");
            require(second.Strongest(pixel({0,-.75f,0})).r<75,"Old pulse remains behind");
            require(first.Strongest(pixel({0,.25f,0})).r<5,"Highlight cut across the bend");
            require(first.Colored()>1500,"Continuous base line is absent");
            for(float d=-.04f;d<=.04f;d+=.01f) {
                const Vector3 p=d<0?Vector3{1+d,-.75f,0}:Vector3{1,-.75f+d,0};
                require(first.Strongest(pixel(p),1).b>50,"Gap at the orthogonal line join");
            }
            style.animate=false;
            const auto fixed=render(model.value,camera,.25,style,"streamline-static.png");
            const auto fixedLater=render(model.value,camera,.75,style);
            require(fixed.colors.size()==fixedLater.colors.size()&&
                    std::equal(fixed.colors.begin(),fixed.colors.end(),fixedLater.colors.begin(),
                        [](Color a,Color b){return a.r==b.r&&a.g==b.g&&a.b==b.b&&a.a==b.a;}),
                    "Disabled animation changes with time");
            require(first.Colored()>fixed.Colored()+50,"Gaussian thickness did not expand the pulse");
            require(fixed.Strongest(pixel({0,-.75f,0})).b>100,"Static field color is missing");
            const auto occluded=render(model.value,camera,0,style,nullptr,true,false);
            const auto visible=render(model.value,camera,0,style,nullptr,true,true);
            require(occluded.Colored()==0,"Flow ignores normal surface occlusion");
            require(visible.Colored()>1500,"Through-surfaces flow is missing");

            // A clipped subsegment keeps its original path coordinates. Its
            // peak is at original arc .25, not at .25 of the remaining piece.
            const std::vector<StreamlineSegment> clipped={{{-.5f,-.75f,0},{1,-.75f,0},.125f,.5f,0,base,base}};
            OwnedModel clippedModel{renderer.Upload(clipped)};
            style.animate=true;
            const auto clip=render(clippedModel.value,camera,.25,style,"streamline-clipped-phase.png");
            require(clip.Strongest(pixel({0,-.75f,0})).r>180,"Clipping reset the pulse phase");
            require(clip.Strongest(pixel({-.75f,-.75f,0})).r<5,"Clipped-away line was drawn");

            style.line_width=1.25f;style.pulse_width=3;
            render(model.value,camera,.49,style,"streamline-default-dark.png");
            render(model.value,camera,.49,style,"streamline-default-light.png",false,false,{244,247,250,255});

            const Camera3D perspective={{0,0,0},{0,0,-1},{0,1,0},60,CAMERA_PERSPECTIVE};
            const std::vector<StreamlineSegment> behind={{{-.5f,0,1},{.5f,0,2},0,1,0,base,base}};
            OwnedModel behindModel{renderer.Upload(behind)};
            require(render(behindModel.value,perspective,0,style).Colored()==0,"Behind-camera flow was drawn");
            const std::vector<StreamlineSegment> crossing={{{-.1f,0,.2f},{.3f,0,-2},0,1,0,base,base}};
            OwnedModel crossingModel{renderer.Upload(crossing)};
            const auto nearPlane=render(crossingModel.value,perspective,.7,style,"streamline-near-plane.png");
            require(nearPlane.Colored()>50&&nearPlane.Colored()<6000,"Near-plane crossing produced a missing or enormous ribbon");
            std::cout<<"PASS GPU pulse travel, curved joins, field color, clipping phase, thickness, depth and camera clipping\n";
        }catch(const std::exception& error){std::cerr<<error.what()<<'\n';status=1;}
        UnloadRenderTexture(target);renderer.Shutdown();
    }
    CloseWindow();return status;
}
