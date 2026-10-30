#pragma once

#include "interaction/camera_controller.hpp"
#include "ui/theme.hpp"
#include <array>
#include <limits>
#include <optional>
#include <string>

namespace RayFEM::UI {

// The cube uses the camera's actual screen basis, including roll. Its
// orthographic projection is independent of mesh position, extent and zoom.
inline Rectangle OrientationCubeBounds(Rectangle viewport) {
    if(!std::isfinite(viewport.x) || !std::isfinite(viewport.y) ||
       !std::isfinite(viewport.width) || !std::isfinite(viewport.height))return {};
    const float side=std::min({132.0f,viewport.width-24,viewport.height-24});
    if(!std::isfinite(side) || side<48)return {};
    return {viewport.x+viewport.width-side-12,viewport.y+12,side,side};
}
struct OrientationControlsLayout {
    Rectangle bounds{};
    Rectangle cube{};
    std::array<Rectangle,4> arrows{};
    Rectangle home{};
};
struct CameraRotationButton {
    CameraRotationAxis axis;
    float degrees;
    const char* tooltip;
};
inline constexpr std::array<CameraRotationButton,4> CameraRotationButtons{{
    {CameraRotationAxis::Back,45,"Rotate clockwise 45°"},
    {CameraRotationAxis::Back,5,"Rotate clockwise 5°"},
    {CameraRotationAxis::Back,-5,"Rotate counterclockwise 5°"},
    {CameraRotationAxis::Back,-45,"Rotate counterclockwise 45°"},
}};
inline OrientationControlsLayout OrientationControlsBounds(Rectangle viewport) {
    const auto original=OrientationCubeBounds(viewport);
    if(original.width==0)return {};
    const float width=std::min(166.0f,viewport.width-24);
    const float height=viewport.height-24;
    // Keep arrow targets usable in a short viewport (for example with plots
    // open), shrinking the cube independently of the four-button rows.
    if(width<141 || height<72)return {original,original,{}};
    const float buttonHeight=height>=120?28.0f:20.0f;
    const float cubeSide=std::min(132.0f,height-buttonHeight-4);
    const float x=viewport.x+viewport.width-width-12,y=viewport.y+12;
    OrientationControlsLayout result{{x,y,width,cubeSide+buttonHeight+4},
        {x+(width-cubeSide)*.5f,y,cubeSide,cubeSide},{}};
    const float buttonWidth=(width-16)/5;
    for(int i=0;i<4;++i)
        result.arrows[i]={x+(i<2?i:i+1)*(buttonWidth+4),y+cubeSide+4,buttonWidth,buttonHeight};
    result.home={x+2*(buttonWidth+4),y+cubeSide+4,buttonWidth,buttonHeight};
    return result;
}
inline int PickCameraRotationArrow(Vector2 mouse,const OrientationControlsLayout& layout) {
    for(int i=0;i<4;++i)
        if(layout.arrows[i].width>0 && CameraViewportContains(mouse,layout.arrows[i]))return i;
    return -1;
}
inline Vector2 ProjectOrientationCubePoint(Vector3 point,const Camera3D& camera,Rectangle box) {
    const auto basis=CameraBasis(camera);
    const float scale=box.width*.25f;
    return {box.x+box.width*.5f+Vector3DotProduct(point,basis.right)*scale,
            box.y+box.height*.5f-Vector3DotProduct(point,basis.up)*scale};
}
struct OrientationCubeHit {
    Vector3 direction{}; // One, two or three signed axes: face, edge or corner.
    Vector3 point{};
    int face_axis=0;
};
inline std::optional<OrientationCubeHit> PickOrientationCube(
        Vector2 mouse,const Camera3D& camera,Rectangle box) {
    if(box.width<=0 || box.height<=0 || !CameraViewportContains(mouse,box))return {};
    const auto basis=CameraBasis(camera);
    const float scale=box.width*.25f;
    const auto origin=Vector3Add(Vector3Scale(basis.back,4),
        Vector3Add(Vector3Scale(basis.right,(mouse.x-box.x-box.width*.5f)/scale),
                   Vector3Scale(basis.up,-(mouse.y-box.y-box.height*.5f)/scale)));
    const auto ray=Vector3Negate(basis.back);
    const float o[]={origin.x,origin.y,origin.z},d[]={ray.x,ray.y,ray.z};
    float enter=-std::numeric_limits<float>::infinity(),leave=std::numeric_limits<float>::infinity();
    int axis=-1;float sign=1;
    for(int i=0;i<3;++i) {
        if(!std::isfinite(o[i]) || !std::isfinite(d[i]))return {};
        if(std::abs(d[i])<1e-7f) {if(o[i]<-1 || o[i]>1)return {};continue;}
        float a=(-1-o[i])/d[i],b=(1-o[i])/d[i];
        if(a>b)std::swap(a,b);
        if(a>enter){enter=a;axis=i;sign=d[i]>0?-1.0f:1.0f;}
        leave=std::min(leave,b);
        if(enter>leave)return {};
    }
    if(axis<0 || enter<0 || !std::isfinite(enter))return {};
    const auto point=Vector3Add(origin,Vector3Scale(ray,enter));
    const float p[]={point.x,point.y,point.z};
    float target[3]{};
    for(int i=0;i<3;++i)target[i]=i==axis?sign:(std::abs(p[i])>=.6f?(p[i]<0?-1.0f:1.0f):0);
    return OrientationCubeHit{{target[0],target[1],target[2]},point,axis};
}
struct OrientationCubeInputState {
    bool captured=false;
    bool cube_press=false; // a left press on the cube itself: a click or a drag
    bool dragging=false;
    Vector2 press{},previous{};
};
struct OrientationCubeInputResult {
    bool owns_pointer=false;
    std::optional<OrientationCubeHit> selected;
    int rotation_button=-1;
    bool home=false;
    Vector2 orbit{}; // pixels to orbit the camera by while the cube is dragged
    bool dragging=false;
};
// The cube's orbit rate: a drag across the whole cube turns about 90°.
inline constexpr float OrientationCubeOrbitRate=.012f;
// A press on the card owns the pointer until release, including blank corners.
// A mesh/profile drag already in progress retains ownership when crossing it.
// On the cube, a press released in place aligns the camera to the face, edge
// or corner under it; moved past a few pixels, it orbits the camera instead.
inline OrientationCubeInputResult UpdateOrientationControlsInput(
        OrientationCubeInputState& state,const CameraInputSnapshot& input,
        const Camera3D& camera,const OrientationControlsLayout& layout,bool other_drag) {
    if(input.cancel_drag || other_drag) {state=OrientationCubeInputState{};return {};}
    const bool over=input.viewport_hovered && CameraViewportContains(input.mouse,layout.bounds);
    const bool wasCaptured=state.captured;
    if(!input.left_down && !input.middle_down)state.captured=false;
    if(over && (input.left_pressed || input.middle_pressed))state.captured=true;
    OrientationCubeInputResult result{over || wasCaptured || state.captured,{}};
    if(over && input.left_pressed) {
        result.rotation_button=PickCameraRotationArrow(input.mouse,layout);
        result.home=layout.home.width>0 && CameraViewportContains(input.mouse,layout.home);
        if(result.rotation_button<0 && !result.home && CameraViewportContains(input.mouse,layout.cube)) {
            state.cube_press=true;state.dragging=false;
            state.press=state.previous=input.mouse;
        }
    }
    if(state.cube_press && input.left_down && !input.left_pressed) {
        if(!state.dragging && Vector2DistanceSqr(input.mouse,state.press)>16)state.dragging=true;
        if(state.dragging) {
            result.orbit=Vector2Subtract(input.mouse,state.previous);
            state.previous=input.mouse;
        }
    }
    if(state.cube_press && !input.left_down) {
        if(!state.dragging)result.selected=PickOrientationCube(state.press,camera,layout.cube);
        state.cube_press=state.dragging=false;
    }
    result.dragging=state.dragging;
    return result;
}
inline OrientationCubeInputResult UpdateOrientationCubeInput(
        OrientationCubeInputState& state,const CameraInputSnapshot& input,
        const Camera3D& camera,Rectangle box,bool other_drag) {
    return UpdateOrientationControlsInput(state,input,camera,{box,box,{}},other_drag);
}
inline std::string OrientationCubeLabel(Vector3 direction) {
    std::string result;
    const float d[]={direction.x,direction.y,direction.z};
    const char* positive[]={"Right (+X)","Top (+Y)","Front (+Z)"};
    const char* negative[]={"Left (−X)","Bottom (−Y)","Back (−Z)"};
    for(int i=0;i<3;++i)if(d[i]!=0) {
        if(!result.empty())result+=" · ";
        result+=d[i]>0?positive[i]:negative[i];
    }
    return result;
}
inline void DrawOrientationCube(const Camera3D& camera,Rectangle box,Vector2 mouse,bool interactive,bool dragging=false) {
    if(box.width<=0 || box.height<=0)return;
    auto* draw=ImGui::GetWindowDrawList();
    const auto basis=CameraBasis(camera);
    const auto hover=interactive && !dragging?PickOrientationCube(mouse,camera,box):std::nullopt;
    const auto& palette=Palette();
    const bool light=CurrentTheme()==ThemeMode::Light;
    const ImVec2 low{box.x,box.y},high{box.x+box.width,box.y+box.height};
    draw->AddRectFilled(low,high,palette.plot_background,OverlayRounding);
    draw->AddRect(low,high,palette.grid,OverlayRounding);
    const float cuts[]={-1,-.6f,.6f,1};
    const char* labels[][2]={{"−X","+X"},{"−Y","+Y"},{"−Z","+Z"}};
    const ImU32 axes[]={light?IM_COL32(161,52,62,255):IM_COL32(247,143,153,255),
                        light?IM_COL32(37,110,64,255):IM_COL32(132,220,161,255),
                        light?IM_COL32(46,93,169,255):IM_COL32(136,181,248,255)};
    for(int axis=0;axis<3;++axis)for(int sign:{-1,1}) {
        float n[3]{};n[axis]=static_cast<float>(sign);
        const Vector3 normal{n[0],n[1],n[2]};
        const float facing=Vector3DotProduct(normal,basis.back);
        if(facing<=1e-5f)continue;
        const int u=(axis+1)%3,v=(axis+2)%3;
        for(int row=0;row<3;++row)for(int col=0;col<3;++col) {
            ImVec2 polygon[4];
            for(int corner=0;corner<4;++corner) {
                float p[3]{};p[axis]=static_cast<float>(sign);
                p[u]=cuts[col+(corner==1 || corner==2)];
                p[v]=cuts[row+(corner>=2)];
                const auto screen=ProjectOrientationCubePoint({p[0],p[1],p[2]},camera,box);
                polygon[corner]={screen.x,screen.y};
            }
            float area=0;
            for(int k=0;k<4;++k)area+=polygon[k].x*polygon[(k+1)%4].y-polygon[(k+1)%4].x*polygon[k].y;
            if(area<0)std::reverse(std::begin(polygon),std::end(polygon));
            float target[3]{};target[axis]=static_cast<float>(sign);
            target[u]=static_cast<float>(col-1);target[v]=static_cast<float>(row-1);
            const bool selected=hover && Vector3Equals(hover->direction,{target[0],target[1],target[2]});
            ImVec4 fill=ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
            const float shade=light?(.86f+.14f*facing):(1.1f+.5f*facing);
            fill.x=std::min(1.0f,fill.x*shade);fill.y=std::min(1.0f,fill.y*shade);fill.z=std::min(1.0f,fill.z*shade);
            if(selected) {
                const auto accent=ThemeText(palette.accent);
                fill.x=fill.x*.55f+accent.x*.45f;fill.y=fill.y*.55f+accent.y*.45f;fill.z=fill.z*.55f+accent.z*.45f;
            }
            draw->AddConvexPolyFilled(polygon,4,ImGui::ColorConvertFloat4ToU32(fill));
            draw->AddPolyline(polygon,4,palette.border,ImDrawFlags_Closed,.75f);
        }
        if(facing>.22f) {
            const auto center=ProjectOrientationCubePoint(normal,camera,box);
            const char* label=labels[axis][sign>0];
            const float fontSize=std::clamp(box.width*.105f,10.0f,14.0f);
            const auto textSize=ImGui::GetFont()->CalcTextSizeA(fontSize,100,0,label);
            draw->AddText(ImGui::GetFont(),fontSize,{center.x-textSize.x*.5f,center.y-textSize.y*.5f},axes[axis],label);
        }
    }
    if(dragging)ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
    else if(interactive && CameraViewportContains(mouse,box)) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        if(hover)ImGui::SetTooltip("%s\nClick to align · drag to rotate",OrientationCubeLabel(hover->direction).c_str());
        else if(ImGui::IsMouseHoveringRect(low,high))ImGui::SetTooltip("Click a face, edge or corner to align the camera, or drag to rotate");
    }
}
inline void DrawCameraRotationArrows(const OrientationControlsLayout& layout,Vector2 mouse,bool interactive,
                                     const std::array<Texture2D,4>& icons) {
    auto* draw=ImGui::GetWindowDrawList();
    const int hovered=interactive?PickCameraRotationArrow(mouse,layout):-1;
    for(int i=0;i<4;++i) {
        const auto box=layout.arrows[i];
        if(box.width<=0)continue;
        const auto& button=CameraRotationButtons[i];
        const bool over=i==hovered;
        const auto fill=ImGui::GetColorU32(over?(IsMouseButtonDown(MOUSE_BUTTON_LEFT)?ImGuiCol_ButtonActive:ImGuiCol_ButtonHovered):ImGuiCol_Button);
        draw->AddRectFilled({box.x,box.y},{box.x+box.width,box.y+box.height},fill,ControlRounding);
        const Vector2 center{box.x+box.width*.5f,box.y+box.height*.5f};
        const float half=std::max(0.0f,std::min(box.width,box.height)-4)*.5f;
        draw->AddImage(ImTextureID(icons[i].id),{center.x-half,center.y-half},
                       {center.x+half,center.y+half},{0,0},{1,1},Palette().text);
        if(over) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            ImGui::SetTooltip("%s",button.tooltip);
        }
    }
    if(layout.home.width>0) {
        const auto box=layout.home;
        const bool over=interactive && CameraViewportContains(mouse,box);
        draw->AddRectFilled({box.x,box.y},{box.x+box.width,box.y+box.height},
            ImGui::GetColorU32(over?(IsMouseButtonDown(MOUSE_BUTTON_LEFT)?ImGuiCol_ButtonActive:ImGuiCol_ButtonHovered):ImGuiCol_Button),ControlRounding);
        const float x=box.x+box.width*.5f,y=box.y+box.height*.5f,s=std::min(box.width,box.height)*.24f;
        const ImVec2 roof[]={{x-s,y},{x,y-s},{x+s,y}};
        draw->AddPolyline(roof,3,Palette().text,0,1.5f);
        const ImVec2 walls[]={{x-s*.7f,y-s*.15f},{x-s*.7f,y+s},{x+s*.7f,y+s},{x+s*.7f,y-s*.15f}};
        draw->AddPolyline(walls,4,Palette().text,0,1.5f);
        if(over) {ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);ImGui::SetTooltip("Home · 30° above the reference plane");}
    }
}
} // namespace RayFEM::UI
