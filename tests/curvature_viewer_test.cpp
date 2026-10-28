// Optional actual-Stroid visual regression. No external mesh is required by
// the default CPU suite; pass its saved native mesh and an output directory.
#include "libRayfem/mesh/converter.hpp"
#include "raymath.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
constexpr int width=1000,height=760;
constexpr Color background{20,26,35,255};
const char* vertex_shader=R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 fragNormal;
out vec4 fragColor;
void main(){fragNormal=vertexNormal;fragColor=vertexColor;gl_Position=mvp*vec4(vertexPosition,1.0);}
)";
const char* fragment_shader=R"(#version 330
in vec3 fragNormal;
in vec4 fragColor;
out vec4 finalColor;
void main(){
    float light=0.45+0.55*abs(dot(normalize(fragNormal),normalize(vec3(0.4,0.6,0.7))));
    finalColor=vec4(fragColor.rgb*light,fragColor.a);
}
)";
void Check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void FlattenNormals(RayFEM::SurfaceData& surface) {
    for(std::size_t i=0;i<surface.positions.size();i+=9) {
        auto point=[&](int v){return Vector3{surface.positions[i+3*v],surface.positions[i+3*v+1],surface.positions[i+3*v+2]};};
        const auto n=Vector3Normalize(Vector3CrossProduct(Vector3Subtract(point(1),point(0)),Vector3Subtract(point(2),point(0))));
        for(int v=0;v<3;++v){surface.normals[i+3*v]=n.x;surface.normals[i+3*v+1]=n.y;surface.normals[i+3*v+2]=n.z;}
    }
}
std::vector<Color> Draw(const RayFEM::SurfaceData& surface,const Camera3D& camera,Shader shader,
                        const std::filesystem::path& file,const char* label) {
    Model model=RayFEM::UploadSurface(surface);Check(model.meshCount>0,"Nonempty GPU model");
    model.materials[0].shader=shader;
    const auto target=LoadRenderTexture(width,height);Check(target.id,"Render target");
    BeginTextureMode(target);ClearBackground(background);BeginMode3D(camera);
    DrawModel(model,{0,0,0},1,WHITE);EndMode3D();
    DrawText(label,26,22,23,{225,236,245,255});
    DrawText("Same Stroid P5 geometry, camera, color, and viewer lighting",26,55,17,{160,181,198,255});
    EndTextureMode();
    Image image=LoadImageFromTexture(target.texture);Check(image.data,"Framebuffer readback");ImageFlipVertical(&image);
    Color* pixels=LoadImageColors(image);Check(pixels,"Framebuffer colors");
    std::vector<Color> result(pixels,pixels+width*height);UnloadImageColors(pixels);
    Check(ExportImage(image,file.string().c_str()),"Screenshot export");
    UnloadImage(image);UnloadRenderTexture(target);UnloadModel(model);
    return result;
}
std::size_t Difference(const std::vector<Color>& a,const std::vector<Color>& b) {
    std::size_t count=0;
    for(int y=100;y<height-30;++y)for(int x=25;x<width-25;++x) {
        const auto& p=a[y*width+x];const auto& q=b[y*width+x];
        if(std::abs(int(p.r)-q.r)+std::abs(int(p.g)-q.g)+std::abs(int(p.b)-q.b)>18)++count;
    }
    return count;
}
}

int main(int argc,char** argv) {
    try {
        Check(argc>=3,"Usage: curvature_viewer_test <Stroid P5 mesh> <output directory>");
        const std::filesystem::path output=argv[2];std::filesystem::create_directories(output);
        mfem::Mesh mesh(argv[1],1,1);
        Check(mesh.GetNodes()&&mesh.GetNodes()->FESpace()->GetMaxElementOrder()==5,"Actual P5 geometry");
        RayFEM::RenderOptions options;options.refinement=3;options.excluded_materials={23};
        const auto coarse_smooth=RayFEM::BuildSurface(mesh,nullptr,options);
        auto old=coarse_smooth;FlattenNormals(old);
        options.refinement=RayFEM::RecommendedSurfaceRefinement(mesh);
        const auto improved=RayFEM::BuildSurface(mesh,nullptr,options);
        const auto center=Vector3Scale(Vector3Add(improved.bounds.min,improved.bounds.max),.5f);
        const float radius=.5f*Vector3Distance(improved.bounds.min,improved.bounds.max);
        const Camera3D camera{Vector3Add(center,Vector3Scale(Vector3Normalize({1.3f,.85f,1.8f}),radius*3.5f)),
                              center,{0,1,0},38,CAMERA_PERSPECTIVE};
        SetTraceLogLevel(LOG_WARNING);SetConfigFlags(FLAG_WINDOW_HIDDEN|FLAG_MSAA_4X_HINT);
        InitWindow(width,height,"RayFEM Stroid curvature regression");Check(IsWindowReady(),"Graphics context");
        Shader shader=LoadShaderFromMemory(vertex_shader,fragment_shader);Check(shader.id,"Viewer-equivalent diffuse shader");
        const auto before=Draw(old,camera,shader,output/"stroid-before.png","Before: 3 subdivisions, flat triangle lighting");
        const auto normals=Draw(coarse_smooth,camera,shader,output/"stroid-normals-only.png","Curved normals: original coarse tessellation");
        const auto after=Draw(improved,camera,shader,output/"stroid-after.png","After: automatic detail and curved geometry normals");
        Check(Difference(before,normals)>1000,"Analytical normals must visibly improve the lighting");
        Check(Difference(before,after)>1000,"Automatic detail and normals must visibly change the rendering");
        UnloadShader(shader);CloseWindow();
        std::cout<<"PASS actual Stroid viewer lighting: "<<old.positions.size()/9<<" -> "<<improved.positions.size()/9
                 <<" triangles, "<<Difference(before,normals)<<" normal-only changed pixels, "
                 <<Difference(before,after)<<" total changed pixels; 3 framebuffer readbacks/screenshots in "<<output<<'\n';
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL curvature graphics: "<<e.what()<<'\n';
        if(IsWindowReady())CloseWindow();
        return 1;
    }
}
