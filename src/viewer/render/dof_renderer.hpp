#pragma once

#include "libRayfem/mesh/converter.hpp"
#include "raylib.h"
#include "rlgl.h"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace RayFEM::UI {

// DOF markers as one static GPU mesh of screen-sized discs. Each marker is a
// quad whose corners the vertex shader offsets in pixels, so camera motion
// costs no CPU work. Tracked DOFs get their track color and an outer ring.
class DofRenderer {
public:
    static constexpr float Radius=3, TrackedRadius=5, RingRadius=7, RingWidth=1.5f;

    void Initialize() {
        const char* vertex=R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
uniform vec2 viewportSize;
out vec2 corner;
out vec4 fragColor;
out float outer;
out float tracked;
out vec3 worldPosition;
void main() {
    vec4 center=mvp*vec4(vertexPosition,1.0);
    outer=vertexNormal.x;
    tracked=vertexNormal.y;
    corner=vertexTexCoord;
    center.xy+=vertexTexCoord*outer*2.0/viewportSize*center.w;
    gl_Position=center;
    fragColor=vertexColor;
    worldPosition=vertexPosition;
})";
        const std::string fragment=R"(#version 330
in vec2 corner;
in vec4 fragColor;
in float outer;
in float tracked;
in vec3 worldPosition;
uniform vec4 dotColor;
uniform vec4 ringColor;
uniform int clipEnabled;
uniform vec3 planePoint;
uniform vec3 planeNormal;
out vec4 finalColor;
void main() {
    if(clipEnabled!=0 && dot(worldPosition-planePoint,planeNormal)>0.0) discard;
    float r=length(corner)*outer;
    if(tracked>0.5) {
        float fill=1.0-smoothstep()" + std::to_string(TrackedRadius-.5f) + R"(,)" + std::to_string(TrackedRadius+.5f) + R"(,r);
        float ring=1.0-smoothstep(0.0,1.0,abs(r-)" + std::to_string(RingRadius) + R"()-)" + std::to_string(RingWidth*.5f) + R"();
        vec4 color=mix(vec4(ringColor.rgb,ring*ringColor.a),fragColor,fill);
        if(color.a<=0.0) discard;
        finalColor=color;
    } else {
        float alpha=1.0-smoothstep()" + std::to_string(Radius-.5f) + R"(,)" + std::to_string(Radius+.5f) + R"(,r);
        if(alpha<=0.0) discard;
        finalColor=vec4(dotColor.rgb,dotColor.a*alpha);
    }
})";
        shader=LoadShaderFromMemory(vertex,fragment.c_str());
        viewportLocation=GetShaderLocation(shader,"viewportSize");
        dotLocation=GetShaderLocation(shader,"dotColor");
        ringLocation=GetShaderLocation(shader,"ringColor");
        clipLocation=GetShaderLocation(shader,"clipEnabled");
        pointLocation=GetShaderLocation(shader,"planePoint");
        normalLocation=GetShaderLocation(shader,"planeNormal");
    }
    void Shutdown() {if(shader.id)UnloadShader(shader);shader={};}

    // Every stride-th point; tracked indices carry their track color.
    Model Upload(const std::vector<DofPoint>& points,std::size_t stride,
                 const std::unordered_map<int,unsigned int>& tracked) const {
        std::size_t count=0;
        for(std::size_t i=0;i<points.size();i+=stride)++count;
        if(!count)return {};
        if(count>static_cast<std::size_t>(std::numeric_limits<int>::max()/6))
            throw std::length_error("Too many DOF markers");
        Mesh mesh{};
        mesh.vertexCount=static_cast<int>(count*6);
        mesh.triangleCount=mesh.vertexCount/3;
        mesh.vertices=static_cast<float*>(MemAlloc(mesh.vertexCount*3*sizeof(float)));
        mesh.normals=static_cast<float*>(MemAlloc(mesh.vertexCount*3*sizeof(float)));
        mesh.texcoords=static_cast<float*>(MemAlloc(mesh.vertexCount*2*sizeof(float)));
        mesh.colors=static_cast<unsigned char*>(MemAlloc(mesh.vertexCount*4));
        if(!mesh.vertices||!mesh.normals||!mesh.texcoords||!mesh.colors) {
            MemFree(mesh.vertices);MemFree(mesh.normals);MemFree(mesh.texcoords);MemFree(mesh.colors);
            throw std::bad_alloc();
        }
        int vertex=0;
        constexpr float corners[6][2]={{-1,-1},{1,-1},{1,1},{-1,-1},{1,1},{-1,1}};
        for(std::size_t i=0;i<points.size();i+=stride) {
            const auto& point=points[i];
            const auto found=tracked.find(point.index);
            const bool isTracked=found!=tracked.end();
            const unsigned int color=isTracked?found->second:0xffffffffu;
            const float outer=isTracked?RingRadius+RingWidth:Radius+.5f;
            for(const auto& c:corners) {
                mesh.vertices[3*vertex]=point.position.x;mesh.vertices[3*vertex+1]=point.position.y;mesh.vertices[3*vertex+2]=point.position.z;
                mesh.normals[3*vertex]=outer;mesh.normals[3*vertex+1]=isTracked?1.f:0.f;mesh.normals[3*vertex+2]=0;
                mesh.texcoords[2*vertex]=c[0];mesh.texcoords[2*vertex+1]=c[1];
                // ImGui packs colors as ABGR.
                mesh.colors[4*vertex]=color&0xff;mesh.colors[4*vertex+1]=(color>>8)&0xff;
                mesh.colors[4*vertex+2]=(color>>16)&0xff;mesh.colors[4*vertex+3]=(color>>24)&0xff;
                ++vertex;
            }
        }
        UploadMesh(&mesh,false);
        auto model=LoadModelFromMesh(mesh);
        model.materials[0].shader=shader;
        return model;
    }

    // Inside BeginMode3D. Markers draw over the surface, as an overlay.
    void Draw(const Model& model,Vector2 viewport,Color dot,Color ring,const ClipPlane* plane) const {
        if(!model.meshCount)return;
        const float dotColor[4]={dot.r/255.f,dot.g/255.f,dot.b/255.f,dot.a/255.f};
        const float ringColor[4]={ring.r/255.f,ring.g/255.f,ring.b/255.f,ring.a/255.f};
        const int clip=plane?1:0;
        SetShaderValue(shader,viewportLocation,&viewport,SHADER_UNIFORM_VEC2);
        SetShaderValue(shader,dotLocation,dotColor,SHADER_UNIFORM_VEC4);
        SetShaderValue(shader,ringLocation,ringColor,SHADER_UNIFORM_VEC4);
        SetShaderValue(shader,clipLocation,&clip,SHADER_UNIFORM_INT);
        if(plane) {
            SetShaderValue(shader,pointLocation,&plane->point,SHADER_UNIFORM_VEC3);
            SetShaderValue(shader,normalLocation,&plane->normal,SHADER_UNIFORM_VEC3);
        }
        rlDrawRenderBatchActive();
        rlDisableDepthTest();rlDisableBackfaceCulling();
        DrawModel(model,{0,0,0},1,WHITE);
        rlDrawRenderBatchActive();
        rlEnableDepthTest();
    }
private:
    Shader shader{};
    int viewportLocation=-1,dotLocation=-1,ringLocation=-1,clipLocation=-1,pointLocation=-1,normalLocation=-1;
};
} // namespace RayFEM::UI
