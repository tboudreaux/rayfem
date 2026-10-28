#include "imgui.h"
#include "raylib.h"
#include "rlImGui.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
constexpr int width=360,height=240;
constexpr int rectanglesPerBand=18000;
constexpr Color colors[]={{240,50,60,255},{50,230,90,255},{60,120,245,255}};
void Check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Window {
    Window(){SetTraceLogLevel(LOG_WARNING);SetConfigFlags(FLAG_WINDOW_HIDDEN);InitWindow(width,height,"RayFEM large UI draw-list check");Check(IsWindowReady(),"Graphics context unavailable");rlImGuiSetup(true);ImGui::GetIO().IniFilename=nullptr;}
    ~Window(){rlImGuiShutdown();CloseWindow();}
};
struct Target {
    RenderTexture2D value=LoadRenderTexture(width,height);
    Target(){Check(IsRenderTextureValid(value),"Render target unavailable");}
    ~Target(){UnloadRenderTexture(value);}
};
void CheckFrame(Target& target,const std::filesystem::path& screenshot){
    rlImGuiBeginDelta(1.0f/60.0f);
    // The offscreen target is logical-resolution even on a Retina desktop.
    ImGui::GetIO().DisplayFramebufferScale={1,1};
    auto* list=ImGui::GetBackgroundDrawList();
    for(int band=0;band<3;++band){
        const float left=20+band*120;
        list->PushClipRect({left,20},{left+80,220},true);
        const auto c=colors[band];
        for(int i=0;i<rectanglesPerBand;++i){
            const float x=left+(i%40)*2,y=20+((i/40)%100)*2;
            list->AddRectFilled({x,y},{x+2,y+2},IM_COL32(c.r,c.g,c.b,c.a));
        }
        list->PopClipRect();
    }
    Check(list->VtxBuffer.Size>200000,"Stress fixture did not exceed the 16-bit vertex limit");
    int offsetCommands=0;
    for(const auto& command:list->CmdBuffer)if(command.ElemCount&&command.VtxOffset>0)++offsetCommands;
    Check(offsetCommands>=3,"Large list did not produce multiple vertex-base commands");
    BeginTextureMode(target.value);ClearBackground({15,20,25,255});rlImGuiEnd();EndTextureMode();
    auto image=LoadImageFromTexture(target.value.texture);Check(image.data,"Framebuffer readback failed");ImageFlipVertical(&image);
    auto pixels=LoadImageColors(image);Check(pixels,"Pixel conversion failed");
    bool correct=true;
    for(int band=0;band<3;++band){
        const auto expected=colors[band];
        for(int y=22;y<218;++y)for(int x=22+band*120;x<98+band*120;++x){
            const auto actual=pixels[y*width+x];
            correct&=actual.r==expected.r&&actual.g==expected.g&&actual.b==expected.b;
        }
    }
    UnloadImageColors(pixels);
    if(!screenshot.empty())Check(ExportImage(image,screenshot.string().c_str()),"Screenshot export failed");
    UnloadImage(image);
    Check(correct,"Vertices after an index reset were rendered in the wrong place or color");
    BeginDrawing();ClearBackground(BLACK);DrawTextureRec(target.value.texture,{0,0,float(width),-float(height)},{0,0},WHITE);EndDrawing();
    std::cout<<"PASS "<<list->VtxBuffer.Size<<" UI vertices, "<<offsetCommands<<" nonzero-base commands, and all three colored bands\n";
}
}
int main(int argc,char** argv){try{
    Window window;Target target;
    Check(ImGui::GetIO().BackendFlags&ImGuiBackendFlags_RendererHasVtxOffset,"Renderer must advertise vertex-base support");
    static_assert(sizeof(ImDrawIdx)==2,"This regression must exercise the default 16-bit ImGui index configuration");
    for(int frame=0;frame<3;++frame)CheckFrame(target,frame==2&&argc>1?std::filesystem::path(argv[1]):std::filesystem::path{});
    std::cout<<"Large UI draw-list graphics check passed\n";return 0;
}catch(const std::exception& error){std::cerr<<"Large UI draw-list check failed: "<<error.what()<<'\n';return 1;}}
