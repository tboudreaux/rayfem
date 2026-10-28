#pragma once

#include "libRayfem/mesh/clipping.hpp"
#include "raylib.h"
#include <limits>
#include <stdexcept>
#include <vector>

namespace RayFEM::UI {

// Static, screen-width ribbons keep element outlines on the GPU. Camera lift
// and pixel width are evaluated by the vertex shader, not once per CPU edge.
class EdgeRenderer {
public:
    void Initialize() {
        shader=LoadShaderFromMemory(R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
uniform vec3 cameraPosition;
uniform vec2 viewportSize;
uniform float lift;
out vec4 fragColor;
void main() {
    vec3 p=vertexPosition+normalize(cameraPosition-vertexPosition)*lift;
    vec3 q=vertexNormal+normalize(cameraPosition-vertexNormal)*lift;
    vec4 a=mvp*vec4(p,1.0), b=mvp*vec4(q,1.0);
    vec2 delta=(b.xy/max(abs(b.w),1e-8)-a.xy/max(abs(a.w),1e-8))*viewportSize;
    float lengthSquared=dot(delta,delta);
    vec2 side=lengthSquared>1e-12?vec2(-delta.y,delta.x)*inversesqrt(lengthSquared):vec2(0);
    a.xy+=side*vertexTexCoord.x*1.15/viewportSize*a.w;
    gl_Position=a;
    fragColor=vertexColor;
})",R"(#version 330
in vec4 fragColor;
uniform vec4 colDiffuse;
uniform int categorical;
out vec4 finalColor;
void main() {finalColor=categorical!=0?fragColor:colDiffuse;}
)");
        cameraLocation=GetShaderLocation(shader,"cameraPosition");
        viewportLocation=GetShaderLocation(shader,"viewportSize");
        liftLocation=GetShaderLocation(shader,"lift");
        categoricalLocation=GetShaderLocation(shader,"categorical");
    }
    void Shutdown() {if(shader.id)UnloadShader(shader);shader={};}

    Model Upload(const std::vector<LineSegment>& lines) const {
        if(lines.empty())return {};
        if(lines.size()>static_cast<std::size_t>(std::numeric_limits<int>::max()/72))
            throw std::length_error("Too many element outlines");
        Mesh mesh{};
        mesh.vertexCount=static_cast<int>(lines.size()*6);
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
        auto append=[&](Vector3 p,Vector3 other,float side,Color color) {
            mesh.vertices[3*vertex]=p.x;mesh.vertices[3*vertex+1]=p.y;mesh.vertices[3*vertex+2]=p.z;
            mesh.normals[3*vertex]=other.x;mesh.normals[3*vertex+1]=other.y;mesh.normals[3*vertex+2]=other.z;
            mesh.texcoords[2*vertex]=side;mesh.texcoords[2*vertex+1]=0;
            mesh.colors[4*vertex]=color.r;mesh.colors[4*vertex+1]=color.g;
            mesh.colors[4*vertex+2]=color.b;mesh.colors[4*vertex+3]=color.a;++vertex;
        };
        for(const auto& edge:lines) {
            const auto color=AttributeColor(edge.boundary_id);
            append(edge.a,edge.b,-1,color);append(edge.a,edge.b,1,color);append(edge.b,edge.a,-1,color);
            append(edge.a,edge.b,-1,color);append(edge.b,edge.a,-1,color);append(edge.b,edge.a,1,color);
        }
        UploadMesh(&mesh,false);
        auto model=LoadModelFromMesh(mesh);
        model.materials[0].shader=shader;
        return model;
    }

    void Draw(const Model& model,const Camera3D& camera,Vector2 viewport,float radius,
              Color color,bool byBoundary) const {
        if(!model.meshCount)return;
        const float offset=radius*.00015f;
        const int categorical=byBoundary?1:0;
        SetShaderValue(shader,cameraLocation,&camera.position,SHADER_UNIFORM_VEC3);
        SetShaderValue(shader,viewportLocation,&viewport,SHADER_UNIFORM_VEC2);
        SetShaderValue(shader,liftLocation,&offset,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,categoricalLocation,&categorical,SHADER_UNIFORM_INT);
        DrawModel(model,{0,0,0},1,color);
    }
private:
    Shader shader{};
    int cameraLocation=-1,viewportLocation=-1,liftLocation=-1,categoricalLocation=-1;
};
} // namespace RayFEM::UI
