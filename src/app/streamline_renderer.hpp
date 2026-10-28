#pragma once

#include "streamline_pulse.hpp"
#include "raylib.h"
#include <cstddef>
#include <span>
#include <stdexcept>

namespace RayFEM::UI {

struct StreamlineSegment {
    Vector3 a{},b{};
    float arc_a=0,arc_b=1; // Normalized arc length on the original complete path.
    float phase=0;         // Per-path phase offset, shared by all its segments.
    Color color_a=WHITE,color_b=WHITE;
};

struct StreamlineStyle {
    bool animate=true;
    float cycles_per_second=.22f;
    float pulse_length=.12f; // Gaussian full width at half maximum, in path units.
    float line_width=1.25f,pulse_width=3.f; // Screen pixels, including on HiDPI targets.
    Color pulse_color={255,248,220,255};
};

// One immutable GPU ribbon per segment. Only animation uniforms change between
// frames. Round, overlapping caps join bends without a geometry shader or a CPU
// rebuild. The caller owns returned Models and unloads them before Shutdown.
// Depth test/write state is inherited, including the viewer's through-surfaces
// mode. Draw in an origin-aligned viewport (as in the viewport render texture).
class StreamlineRenderer {
public:
    static constexpr std::size_t max_segments=200000; // <= 53 MiB CPU, plus GPU buffers.

    void Initialize() {
        if(shader.id)return;
        shader=LoadShaderFromMemory(R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
in vec2 vertexTexCoord2;
in vec4 vertexColor;
uniform mat4 mvp;
uniform vec2 viewportSize;
uniform float maximumHalfWidth;
flat out vec2 screenA;
flat out vec2 screenB;
flat out vec2 pathArc;
flat out vec2 clipDepth;
flat out float pathPhase;
out vec4 fragColor;
void main() {
    bool last=abs(vertexTexCoord.x)>1.5;
    vec4 a=mvp*vec4(last?vertexNormal:vertexPosition,1.0);
    vec4 b=mvp*vec4(last?vertexPosition:vertexNormal,1.0);
    vec2 arc=last?vec2(vertexTexCoord2.y,vertexTexCoord.y):vec2(vertexTexCoord.y,vertexTexCoord2.y);
    // Explicit near-plane clipping prevents huge/reversed ribbons when a path
    // crosses the camera. Its arc coordinate remains attached to the path.
    float da=a.z+a.w, db=b.z+b.w;
    if(da<0.0 && db<0.0) {
        gl_Position=vec4(2.0,2.0,2.0,1.0);
        screenA=vec2(0);screenB=vec2(0);pathArc=arc;
        clipDepth=vec2(1);pathPhase=vertexTexCoord2.x;fragColor=vertexColor;
        return;
    }
    if(da<0.0) {float t=da/(da-db);a=mix(a,b,t);arc.x=mix(arc.x,arc.y,t);}
    else if(db<0.0) {float t=db/(db-da);b=mix(b,a,t);arc.y=mix(arc.y,arc.x,t);}
    screenA=(a.xy/max(a.w,1e-8)*.5+.5)*viewportSize;
    screenB=(b.xy/max(b.w,1e-8)*.5+.5)*viewportSize;
    vec2 delta=screenB-screenA;
    float squared=dot(delta,delta);
    vec2 forward=squared>1e-12?delta*inversesqrt(squared):vec2(1,0);
    vec2 side=vec2(-forward.y,forward.x);
    vec4 p=last?b:a;
    vec2 expansion=(side*sign(vertexTexCoord.x)+forward*(last?1.0:-1.0))*maximumHalfWidth;
    p.xy+=2.0*expansion/viewportSize*p.w;
    gl_Position=p;
    pathArc=arc;clipDepth=vec2(a.w,b.w);pathPhase=vertexTexCoord2.x;
    fragColor=vertexColor;
})",R"(#version 330
flat in vec2 screenA;
flat in vec2 screenB;
flat in vec2 pathArc;
flat in vec2 clipDepth;
flat in float pathPhase;
in vec4 fragColor;
uniform float timePhase;
uniform float pulseLength;
uniform float lineWidth;
uniform float pulseWidth;
uniform vec4 pulseColor;
uniform int animated;
out vec4 finalColor;
void main() {
    vec2 delta=screenB-screenA;
    float squared=dot(delta,delta);
    float u=squared>1e-12?clamp(dot(gl_FragCoord.xy-screenA,delta)/squared,0.0,1.0):0.0;
    float t=u*clipDepth.x/max(mix(clipDepth.y,clipDepth.x,u),1e-8);
    float arc=mix(pathArc.x,pathArc.y,t);
    float distance=fract(arc-timePhase-pathPhase+.5)-.5;
    float normalized=distance/pulseLength;
    float pulse=animated!=0?exp(-2.77258872224*normalized*normalized):0.0;
    float width=mix(lineWidth,pulseWidth,pulse);
    float radial=length(gl_FragCoord.xy-mix(screenA,screenB,u));
    float coverage=1.0-smoothstep(max(0.0,width*.5-.5),width*.5+.5,radial);
    if(coverage<=.001)discard;
    // Retain some field color at the peak: the highlight shows travel without
    // losing the magnitude palette or creating a large additive glow.
    vec3 color=mix(fragColor.rgb,pulseColor.rgb,.82*pulse);
    float alpha=mix(fragColor.a,pulseColor.a,pulse)*coverage;
    finalColor=vec4(color,alpha);
})");
        viewportLocation=GetShaderLocation(shader,"viewportSize");
        maximumWidthLocation=GetShaderLocation(shader,"maximumHalfWidth");
        timeLocation=GetShaderLocation(shader,"timePhase");
        lengthLocation=GetShaderLocation(shader,"pulseLength");
        lineWidthLocation=GetShaderLocation(shader,"lineWidth");
        pulseWidthLocation=GetShaderLocation(shader,"pulseWidth");
        pulseColorLocation=GetShaderLocation(shader,"pulseColor");
        animatedLocation=GetShaderLocation(shader,"animated");
        if(!shader.id || viewportLocation<0 || maximumWidthLocation<0 || timeLocation<0 ||
           lengthLocation<0 || lineWidthLocation<0 || pulseWidthLocation<0 ||
           pulseColorLocation<0 || animatedLocation<0) {
            Shutdown();throw std::runtime_error("Could not initialize the streamline shader");
        }
    }

    void Shutdown() {if(shader.id)UnloadShader(shader);shader={};}

    Model Upload(std::span<const StreamlineSegment> segments) const {
        if(segments.empty())return {};
        if(!shader.id)throw std::logic_error("Initialize the streamline renderer before uploading");
        if(segments.size()>max_segments)throw std::length_error("Too many streamline segments");
        auto finite=[](Vector3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
        for(const auto& s:segments) {
            if(!finite(s.a)||!finite(s.b)||!std::isfinite(s.arc_a)||!std::isfinite(s.arc_b)||
               !std::isfinite(s.phase)||s.arc_a<0||s.arc_b>1||s.arc_b<s.arc_a)
                throw std::invalid_argument("Invalid streamline segment or arc coordinate");
        }
        Mesh mesh{};
        mesh.vertexCount=static_cast<int>(segments.size()*6);mesh.triangleCount=mesh.vertexCount/3;
        mesh.vertices=static_cast<float*>(MemAlloc(mesh.vertexCount*3*sizeof(float)));
        mesh.normals=static_cast<float*>(MemAlloc(mesh.vertexCount*3*sizeof(float)));
        mesh.texcoords=static_cast<float*>(MemAlloc(mesh.vertexCount*2*sizeof(float)));
        mesh.texcoords2=static_cast<float*>(MemAlloc(mesh.vertexCount*2*sizeof(float)));
        mesh.colors=static_cast<unsigned char*>(MemAlloc(mesh.vertexCount*4));
        if(!mesh.vertices||!mesh.normals||!mesh.texcoords||!mesh.texcoords2||!mesh.colors) {
            MemFree(mesh.vertices);MemFree(mesh.normals);MemFree(mesh.texcoords);MemFree(mesh.texcoords2);MemFree(mesh.colors);
            throw std::bad_alloc();
        }
        int vertex=0;
        auto append=[&](const StreamlineSegment& s,bool last,float side) {
            const auto p=last?s.b:s.a,q=last?s.a:s.b;const auto color=last?s.color_b:s.color_a;
            mesh.vertices[3*vertex]=p.x;mesh.vertices[3*vertex+1]=p.y;mesh.vertices[3*vertex+2]=p.z;
            mesh.normals[3*vertex]=q.x;mesh.normals[3*vertex+1]=q.y;mesh.normals[3*vertex+2]=q.z;
            mesh.texcoords[2*vertex]=side*(last?2.f:1.f);mesh.texcoords[2*vertex+1]=last?s.arc_b:s.arc_a;
            mesh.texcoords2[2*vertex]=static_cast<float>(StreamlinePhase(0,0,s.phase));
            mesh.texcoords2[2*vertex+1]=last?s.arc_a:s.arc_b;
            mesh.colors[4*vertex]=color.r;mesh.colors[4*vertex+1]=color.g;
            mesh.colors[4*vertex+2]=color.b;mesh.colors[4*vertex+3]=color.a;++vertex;
        };
        for(const auto& s:segments) {
            append(s,false,-1);append(s,true,-1);append(s,true,1);
            append(s,false,-1);append(s,true,1);append(s,false,1);
        }
        UploadMesh(&mesh,false);
        auto model=LoadModelFromMesh(mesh);model.materials[0].shader=shader;return model;
    }

    void Draw(const Model& model,const Camera3D&,Vector2 viewport,double seconds,
              const StreamlineStyle& style) const {
        if(!model.meshCount)return;
        if(!std::isfinite(viewport.x)||!std::isfinite(viewport.y)||!(viewport.x>0)||!(viewport.y>0)||
           !std::isfinite(style.line_width)||!std::isfinite(style.pulse_width)||!(style.line_width>0)||
           !std::isfinite(style.pulse_length)||!(style.pulse_length>0))
            throw std::invalid_argument("Invalid streamline appearance or viewport");
        const float phase=static_cast<float>(StreamlinePhase(seconds,style.cycles_per_second));
        const float lineWidth=std::clamp(style.line_width,.25f,32.f);
        const float pulseWidth=std::clamp(style.pulse_width,lineWidth,32.f);
        const float maximumHalfWidth=(style.animate?pulseWidth:lineWidth)*.5f+1;
        const float length=std::clamp(style.pulse_length,.001f,1.f);
        const float color[]={style.pulse_color.r/255.f,style.pulse_color.g/255.f,style.pulse_color.b/255.f,style.pulse_color.a/255.f};
        const int animated=style.animate?1:0;
        SetShaderValue(shader,viewportLocation,&viewport,SHADER_UNIFORM_VEC2);
        SetShaderValue(shader,maximumWidthLocation,&maximumHalfWidth,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,timeLocation,&phase,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,lengthLocation,&length,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,lineWidthLocation,&lineWidth,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,pulseWidthLocation,&pulseWidth,SHADER_UNIFORM_FLOAT);
        SetShaderValue(shader,pulseColorLocation,color,SHADER_UNIFORM_VEC4);
        SetShaderValue(shader,animatedLocation,&animated,SHADER_UNIFORM_INT);
        DrawModel(model,{0,0,0},1,WHITE);
    }
private:
    Shader shader{};
    int viewportLocation=-1,maximumWidthLocation=-1,timeLocation=-1,lengthLocation=-1;
    int lineWidthLocation=-1,pulseWidthLocation=-1,pulseColorLocation=-1,animatedLocation=-1;
};

} // namespace RayFEM::UI
