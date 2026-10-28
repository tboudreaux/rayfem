#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/sampler.hpp"
#include "imgui.h"
#include "raymath.h"
#include "plot.hpp"
#include "frame_pipeline.hpp"
#include "ray_placement.hpp"
#include "widgets.hpp"
#include "theme.hpp"
#include "extras/IconsFontAwesome6.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>

// The panel owns only controls and sampled results. All point location and
// finite-element evaluation goes through the replaceable ProfileRegistry.
// With a worker (SetWorker), sampling runs there on each frame's compute twin
// and updates live while a ray is drawn or dragged; otherwise it runs inline.
class ProfilePanel {
    using Clock=std::chrono::steady_clock;
public:
    // Surface hit under a viewport ray, in displayed coordinates.
    using SurfacePick=std::function<std::optional<Vector3>(Ray)>;

    bool Enabled() const { return mode != 0; }
    bool Dragging() const { return handle>=0; }
    int Mode() const { return mode; }
    void SetMode(int value) {
        value=std::clamp(value,0,2);
        if(mode!=value){mode=value;dirty=true;message.clear();handle=-1;}
    }
    // The first ray of a session is placed on elements; later opens keep it.
    void OpenRay() { SetMode(1); if(!rayPlaced) placementRequest=-1; }
    // -1 asks the viewer to place a new ray; 0 or 1 to re-place that endpoint.
    std::optional<int> TakePlacementRequest() { auto request=placementRequest; placementRequest.reset(); return request; }
    const std::array<RayFEM::UI::RayAnchor,2>& Anchors() const { return anchors; }
    // Endpoints fixed to elements follow the mesh on every frame.
    void SetAnchors(const std::array<RayFEM::UI::RayAnchor,2>& value) {
        anchors=value;anchorMesh=context?context->mesh:nullptr;rayPlaced=true;
        ApplyAnchors(true);
    }
    void OpenPlane() { SetMode(2); }
    void SetWorker(RayFEM::UI::FramePipeline* pipeline) { worker=pipeline; }

    // Call even when this panel's controls are hidden in another sidebar tab.
    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                       mfem::GridFunction* field,const std::string& fieldName,
                       const RayFEM::RenderOptions& options) {
        const bool changed=context!=frame || fieldPointer!=field || currentFieldName!=fieldName ||
           settings.view!=options.view || settings.component!=options.component ||
           settings.displacement!=options.displacement || settings.displacement_scale!=options.displacement_scale ||
           settings.excluded_materials!=options.excluded_materials ||
           settings.excluded_elements!=options.excluded_elements;
        if(changed) dirty=true;
        context=frame;fieldPointer=field;settings=options;currentFieldName=fieldName;
        if(changed) ApplyAnchors(false);
    }

    void SetBounds(BoundingBox bounds, const std::shared_ptr<mfem::Mesh>& mesh) {
        meshBounds=bounds;
        if (lastMesh.lock()==mesh) return;
        lastMesh=mesh;
        center=Vector3Scale(Vector3Add(bounds.min,bounds.max),0.5f);
        origin={bounds.min.x,center.y,center.z};
        direction={1,0,0}; normal={0,0,1};
        length=std::max(0.01f,bounds.max.x-bounds.min.x);
        width=length; height=std::max(0.01f,bounds.max.y-bounds.min.y);
        scale=std::max(0.01f,Vector3Length(Vector3Subtract(bounds.max,bounds.min)));
        dirty=true;
    }

    void Controls(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                  mfem::GridFunction* field,const std::string& fieldName,
                  const RayFEM::RenderOptions& options) {
        UpdateContext(frame,field,fieldName,options);
        constexpr const char* badges[]={nullptr,"Ray","Plane"};
        if(!RayFEM::UI::BeginSection("Profile",badges[std::clamp(mode,0,2)]))return;
        ControlsBody();
        RayFEM::UI::EndSection();
    }
private:
    void ControlsBody() {
        ImGui::PushID("Profile controls");
        int selectedMode=mode;
        if(RayFEM::UI::Segmented("##mode",&selectedMode,{"Off","Ray","Plane"}))SetMode(selectedMode);
        Tip("Sample the field along a ray or across a plane. Move the handles in the viewport to position the profile.");
        if(!Enabled()) {
            ImGui::PopID();return;
        }
        if(mode==1) {
            if(ImGui::Button(ICON_FA_CROSSHAIRS "  Place on elements",{-1,0}))placementRequest=-1;
            Tip("Choose the start element, place the start on its surface, then do the same for the end. Placed endpoints move with the mesh.");
            for(int i=0;i<2;++i) {
                ImGui::PushID(i);
                RayFEM::UI::FieldLabel(i==0?"Start":"End");
                if(anchors[i].Valid())ImGui::Text("element %d",anchors[i].element);
                else ImGui::TextDisabled("free point");
                ImGui::SameLine();
                if(ImGui::SmallButton(anchors[i].Valid()?"Edit":"Place"))placementRequest=i;
                if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                    ImGui::SetTooltip("%s",anchors[i].Valid()?"Move this endpoint on its element, or choose another element.":"Fix this endpoint to an element so it follows the mesh.");
                ImGui::PopID();
            }
            const float width=(ImGui::GetContentRegionAvail().x-3*ImGui::GetStyle().ItemSpacing.x)/4;
            const char* axes[]={"X","Y","Z"};
            for(int axis=0;axis<3;++axis) {
                if(axis)ImGui::SameLine();
                if(ImGui::Button(axes[axis],{width,0}))AxisRay(axis);
                if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))ImGui::SetTooltip("Ray along %s through the mesh center",axes[axis]);
            }
            ImGui::SameLine();
            if(ImGui::Button(ICON_FA_VIDEO "###view_ray",{width,0}))ViewRay();
            Tip("Ray through the mesh center along your view direction");
        } else ImGui::TextDisabled("Drag the handles in the viewport.");
        RayFEM::UI::Switch("Live update",&liveUpdate);
        Tip("Resample automatically while you move the profile and when the frame changes. Turn off to sample on demand.");
        if(!liveUpdate) {
            ImGui::BeginDisabled(!context || !fieldPointer);
            if(ImGui::Button(samples.empty()?"Sample profile":"Update profile",{-1,0}))Sample(false);
            ImGui::EndDisabled();
        }
        if(!samples.empty()) {
            if(dirty)ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),liveUpdate?"Updating":"Update to refresh");
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().success),"Current sample");
            ImGui::TextDisabled("%zu / %zu inside  ·  %.1f ms",valid,samples.size(),sampleMs);
        }
        if(ImGui::TreeNode("Sampling settings")) {
            ImGui::PushItemWidth(-1);
            if(mode==1) {
                ImGui::TextDisabled("Origin");dirty|=ImGui::DragFloat3("##origin",&origin.x,scale*.005f,0,0,"%.4g");
                ImGui::TextDisabled("Direction");dirty|=ImGui::DragFloat3("##direction",&direction.x,.01f,0,0,"%.4g");
                Tip("The direction is normalized when sampled. Drag the endpoint to change direction and length together.");
                RayFEM::UI::FieldLabel("Length");dirty|=ImGui::DragFloat("##length",&length,scale*.01f,.0001f,scale*100,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                RayFEM::UI::FieldLabel("Samples");dirty|=ImGui::SliderInt("##count",&count,2,1024);
            } else {
                ImGui::TextDisabled("Center");dirty|=ImGui::DragFloat3("##center",&center.x,scale*.005f,0,0,"%.4g");
                ImGui::TextDisabled("Normal");dirty|=ImGui::DragFloat3("##normal",&normal.x,.01f,0,0,"%.4g");
                RayFEM::UI::FieldLabel("Width");dirty|=ImGui::DragFloat("##width",&width,scale*.01f,.0001f,scale*100,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                RayFEM::UI::FieldLabel("Height");dirty|=ImGui::DragFloat("##height",&height,scale*.01f,.0001f,scale*100,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                RayFEM::UI::FieldLabel("Columns");dirty|=ImGui::SliderInt("##columns",&columns,2,128);
                RayFEM::UI::FieldLabel("Rows");dirty|=ImGui::SliderInt("##rows",&rows,2,128);
            }
            const auto algorithms=registry.algorithms();
            auto current=std::find_if(algorithms.begin(),algorithms.end(),[&](const auto& a){return a.id==algorithm;});
            RayFEM::UI::FieldLabel("Sampling method");
            if(ImGui::BeginCombo("##method",current==algorithms.end()?algorithm.c_str():current->name.c_str())) {
                for(const auto& a:algorithms)if(ImGui::Selectable(a.name.c_str(),a.id==algorithm)){algorithm=a.id;dirty=true;}
                ImGui::EndCombo();
            }
            Tip("Both methods evaluate the finite-element field at the requested positions. Coherent mapping reuses nearby element searches.");
            ImGui::PopItemWidth();
            ImGui::TreePop();
        }
        if(!samples.empty() && ImGui::TreeNode("Export")) {
            ImGui::TextDisabled("CSV destination");ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##csv_path",path,sizeof(path));
            ImGui::BeginDisabled(dirty);
            if(ImGui::Button("Save CSV",{-1,0}))Export();
            ImGui::EndDisabled();
            if(dirty)ImGui::TextWrapped("Update the profile before exporting.");
            ImGui::TreePop();
        }
        if(!message.empty())ImGui::TextWrapped("%s",message.c_str());
        ImGui::PopID();
    }
public:

    // Viewport input. Returning true reserves the drag so it cannot also orbit
    // the camera. Rays are drawn on the mesh (pick finds the surface under the
    // cursor); endpoints snap to the surface unless Alt is held, Shift keeps
    // the ray on the nearest axis, and the middle handle moves the whole ray.
    bool HandleInput(ImVec2 viewport,ImVec2 size,const Camera3D& camera,bool hovered,const SurfacePick& pick={}) {
        lastCamera=camera;
        Collect();
        if (!Enabled() || !IsWindowFocused()) {handle=-1;hoverHandle=-1;return false;}
        const Vector2 mouse=GetMousePosition();
        const Vector3 forward=Vector3Normalize(Vector3Subtract(camera.target,camera.position));
        auto viewRay=[&]{return GetScreenToWorldRayEx({mouse.x-viewport.x,mouse.y-viewport.y},camera,static_cast<int>(size.x),static_cast<int>(size.y));};
        auto onPlane=[&](const Ray& ray,Vector3 through,Vector3 normalVector)->std::optional<Vector3> {
            const float denominator=Vector3DotProduct(ray.direction,normalVector);
            if(std::abs(denominator)<=1e-7f)return std::nullopt;
            const float t=Vector3DotProduct(Vector3Subtract(through,ray.position),normalVector)/denominator;
            return Vector3Add(ray.position,Vector3Scale(ray.direction,t));
        };
        bool reserved=false;
        // Handle hit testing: start, end and (for rays) the midpoint.
        Vector3 points[3];const int count=Handles(points);
        int near=-1;float best=144;
        for(int i=0;i<count;++i) {
            if(Vector3DotProduct(Vector3Subtract(points[i],camera.position),forward)<=0) continue;
            auto p=GetWorldToScreenEx(points[i],camera,static_cast<int>(size.x),static_cast<int>(size.y));
            float dx=p.x+viewport.x-mouse.x,dy=p.y+viewport.y-mouse.y;
            float d=dx*dx+dy*dy;
            if(d<best){best=d;near=i;}
        }
        hoverHandle=hovered && handle<0?near:-1;
        if(hoverHandle>=0 || handle>=0) ImGui::SetMouseCursor(handle==2||hoverHandle==2?ImGuiMouseCursor_ResizeAll:ImGuiMouseCursor_Hand);
        if (hovered && near>=0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            handle=near;anchor=points[handle];dragNormal=forward;pressOrigin=origin;pressCenter=center;
            // Dragging frees an endpoint from its element.
            if(mode==1){if(handle!=1)anchors[0]={};if(handle!=0)anchors[1]={};}
            pressEnd=Vector3Add(origin,Vector3Scale(Vector3Normalize(direction),length));
        }
        if (handle>=0) {
            if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {handle=-1;lastRequest={};return true;}
            const Ray ray=viewRay();
            const bool free=ImGui::GetIO().KeyAlt;
            std::optional<Vector3> point;
            if(mode==1 && handle<2 && !free && pick) point=pick(ray);
            if(!point) point=onPlane(ray,anchor,dragNormal);
            if(point) {
                if (mode==1) {
                    Vector3 start=pressOrigin,end=pressEnd;
                    if(handle==0) start=*point;
                    else if(handle==1) end=*point;
                    else {const Vector3 shift=Vector3Subtract(*point,anchor);start=Vector3Add(pressOrigin,shift);end=Vector3Add(pressEnd,shift);}
                    if(handle<2 && ImGui::GetIO().KeyShift) {
                        // Keep the fixed endpoint and project onto the nearest axis.
                        const Vector3 delta=Vector3Subtract(end,start);
                        const float c[3]={std::abs(delta.x),std::abs(delta.y),std::abs(delta.z)};
                        const int axis=c[0]>=c[1]&&c[0]>=c[2]?0:(c[1]>=c[2]?1:2);
                        Vector3 along{};(&along.x)[axis]=(&delta.x)[axis];
                        if(handle==0)start=Vector3Subtract(end,along);else end=Vector3Add(start,along);
                    }
                    const Vector3 delta=Vector3Subtract(end,start);
                    if(Vector3Length(delta)>scale*1e-4f) {origin=start;direction=Vector3Normalize(delta);length=Vector3Length(delta);}
                } else {
                    if(handle==0) center=*point;
                    else normal=Vector3Normalize(Vector3Subtract(*point,center));
                }
                dirty=true;
            }
            reserved=true;
        }
        Tick();
        return reserved;
    }

    // Plane outline in the 3D pass. Rays are drawn as an overlay by DrawHandles.
    void DrawGuide() const {
        if(mode!=2) return;
        Vector3 points[3];Handles(points);
        for (int i=0;i<2;++i) if (!std::isfinite(points[i].x)||!std::isfinite(points[i].y)||!std::isfinite(points[i].z)) return;
        DrawLine3D(points[0],points[1],RayFEM::UI::Palette().profile_guide);
        if(mode==2 && Vector3Length(normal)>0.001f && std::isfinite(width) && std::isfinite(height) && width>0 && height>0) {
            auto corners=RayFEM::MakePlaneSamples(Position(center),Position(normal),width,height,2,2);
            for(auto edge: {std::pair{0,1},std::pair{1,3},std::pair{3,2},std::pair{2,0}})
                DrawLine3D(Vector(corners[edge.first].position),Vector(corners[edge.second].position),RayFEM::UI::Palette().profile_guide);
        }
    }

    void DrawHandles(ImVec2 viewport,ImVec2 size,const Camera3D& camera) const {
        rayHoverS=std::numeric_limits<double>::quiet_NaN();
        if(!Enabled()) return;
        const auto& palette=RayFEM::UI::Palette();
        auto* draw=ImGui::GetWindowDrawList();
        draw->PushClipRect(viewport,{viewport.x+size.x,viewport.y+size.y},true);
        const float w=size.x,h=size.y;
        const Matrix viewProjection=MatrixMultiply(MatrixLookAt(camera.position,camera.target,camera.up),
            MatrixPerspective(camera.fovy*DEG2RAD,double(w)/double(h),rlGetCullDistanceNear(),rlGetCullDistanceFar()));
        auto project=[&](Vector3 p,ImVec2& out) {
            const Quaternion q=QuaternionTransform({p.x,p.y,p.z,1},viewProjection);
            if(!(q.w>1e-6f))return false;
            out={viewport.x+(q.x/q.w+1)*.5f*w,viewport.y+(1-q.y/q.w)*.5f*h};
            return true;
        };
        auto dashed=[&](ImVec2 a,ImVec2 b,ImU32 color,float thickness) {
            const float length=std::hypot(b.x-a.x,b.y-a.y);
            for(float t=0;t<length;t+=9) {
                const float u=t/length,v=std::min(t+5,length)/length;
                draw->AddLine({a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u},{a.x+(b.x-a.x)*v,a.y+(b.y-a.y)*v},color,thickness);
            }
        };
        const ImVec2 mouse=ImGui::GetMousePos();
        if(mode==1) {
            const Vector3 unit=Vector3Normalize(direction);
            auto along=[&](double fraction){return Vector3Add(origin,Vector3Scale(unit,static_cast<float>(fraction*length)));};
            {
                // Solid where samples fall inside the mesh, dashed outside it.
                const bool sampled=sampleMode==1 && samples.size()>1 && sampledLength>0;
                if(sampled) {
                    for(std::size_t i=1;i<samples.size();++i) {
                        ImVec2 a,b;
                        if(!project(along(samples[i-1].s/sampledLength),a) || !project(along(samples[i].s/sampledLength),b))continue;
                        if(samples[i-1].valid && samples[i].valid)draw->AddLine(a,b,palette.guide,3);
                        else dashed(a,b,RayFEM::UI::WithAlpha(palette.guide,200),1.5f);
                    }
                } else {
                    ImVec2 a,b;
                    if(project(origin,a) && project(along(1),b))draw->AddLine(a,b,palette.guide,2);
                }
                // Hovering the ray reports the distance and the nearest sample.
                ImVec2 a,b;
                const bool viewportHovered=ImGui::IsMouseHoveringRect(viewport,{viewport.x+size.x,viewport.y+size.y});
                if(viewportHovered && handle<0 && hoverHandle<0 && project(origin,a) && project(along(1),b)) {
                    const float dx=b.x-a.x,dy=b.y-a.y,lengthSquared=dx*dx+dy*dy;
                    const float t=lengthSquared>0?std::clamp(((mouse.x-a.x)*dx+(mouse.y-a.y)*dy)/lengthSquared,0.0f,1.0f):0;
                    const ImVec2 closest{a.x+dx*t,a.y+dy*t};
                    if(std::hypot(mouse.x-closest.x,mouse.y-closest.y)<7) {
                        rayHoverS=t*(sampled?sampledLength:length);
                        draw->AddCircle(closest,6,palette.selection,0,2);
                        ImGui::BeginTooltip();
                        ImGui::Text("Distance %.6g",rayHoverS);
                        if(sampled) {
                            const auto nearest=std::min_element(samples.begin(),samples.end(),[&](const auto& x,const auto& y){return std::abs(x.s-rayHoverS)<std::abs(y.s-rayHoverS);});
                            if(nearest->valid)ImGui::TextDisabled("Value %.9g",nearest->value);else ImGui::TextDisabled("Outside the mesh");
                        }
                        ImGui::EndTooltip();
                    }
                }
                // The plot's cursor, shown on the ray.
                ImVec2 marker;
                if(sampled && std::isfinite(plotHoverS) && project(along(plotHoverS/sampledLength),marker)) {
                    draw->AddCircleFilled(marker,5,palette.selection);
                    draw->AddCircle(marker,8,palette.guide,0,2);
                }
            }
        }
        Vector3 points[3];const int count=Handles(points);
        const char* labels[2][3]={{"Start","End","Move ray"},{"Plane center","Plane normal",""}};
        for(int i=0;i<count;++i) {
            ImVec2 screen;
            if(Vector3DotProduct(Vector3Subtract(points[i],camera.position),Vector3Subtract(camera.target,camera.position))<=0 || !project(points[i],screen)) continue;
            if(screen.x<viewport.x-8 || screen.x>viewport.x+size.x+8 || screen.y<viewport.y-8 || screen.y>viewport.y+size.y+8)continue;
            const bool active=handle==i||hoverHandle==i;
            if(i==2) {
                draw->AddCircleFilled(screen,active?6:4.5f,palette.handle_outline);
                draw->AddCircle(screen,active?6:4.5f,palette.guide,0,2);
            } else {
                draw->AddCircleFilled(screen,active?8:6,palette.guide);
                draw->AddCircle(screen,active?10:8,palette.handle_outline,0,2);
                if(mode==1 && anchors[i].Valid())draw->AddCircle(screen,active?13:11,palette.accent,0,1.5f);
            }
            // Labels appear only for the handle under the cursor or being dragged.
            if(!active)continue;
            std::string label=labels[mode==1?0:1][i];
            if(mode==1 && i<2 && anchors[i].Valid())label+="  ·  element "+std::to_string(anchors[i].element);
            const ImVec2 labelSize=ImGui::CalcTextSize(label.c_str());
            const ImVec2 boxSize={labelSize.x+12,labelSize.y+8};
            const ImVec2 labelMin={std::clamp(screen.x+13,viewport.x+4,std::max(viewport.x+4,viewport.x+size.x-boxSize.x-4)),
                                   std::clamp(screen.y-boxSize.y*.5f,viewport.y+4,std::max(viewport.y+4,viewport.y+size.y-boxSize.y-4))};
            draw->AddRectFilled(labelMin,{labelMin.x+boxSize.x,labelMin.y+boxSize.y},palette.plot_background,RayFEM::UI::OverlayRounding);
            draw->AddText({labelMin.x+6,labelMin.y+4},palette.guide_text,label.c_str());
        }
        draw->PopClipRect();
    }

    void DrawPlot(ImVec2 size) const {
        if(!Enabled())return;
        ImGui::BeginChild("Profile plot",size,ImGuiChildFlags_None,ImGuiWindowFlags_NoScrollbar);
        if(samples.empty()) {
            ImGui::Spacing();ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"No profile sampled");
            ImGui::TextWrapped(mode==1?"Draw a ray on the mesh, or choose an axis in Analyze.":"Position the handles in the viewport.");
            ImGui::EndChild();return;
        }
        ImGui::TextWrapped("%s",metadata.field_name.c_str());
        char details[128];
        std::snprintf(details,sizeof(details),"Frame %llu  ·  t %.5g  ·  %zu/%zu inside",
                      static_cast<unsigned long long>(metadata.frame),metadata.time,valid,samples.size());
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details).x);
        ImGui::TextDisabled("%s",details);
        if(dirty) {
            // Live profiles are always about to refresh; only a stale manual sample warns.
            const char* state=liveUpdate?"Updating":"Previous sample";
            RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(state).x);
            if(liveUpdate)ImGui::TextDisabled("%s",state);
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"%s",state);
        }
        if(sampleMode==1) {
            const RayFEM::UI::PlotSeriesView series{metadata.field_name.c_str(),linePoints,RayFEM::UI::PlotColor(0)};
            RayFEM::UI::PlotOptions options;options.x_label="Distance along ray";options.y_label="Value";options.legend=false;
            options.cursor_x=rayHoverS;
            plotHoverS=std::numeric_limits<double>::quiet_NaN();
            options.hovered_x=&plotHoverS;
            RayFEM::UI::PlotLines("Ray plot",std::span(&series,1),ImGui::GetContentRegionAvail(),options);
        } else DrawPlanePlot(ImGui::GetContentRegionAvail());
        ImGui::EndChild();
    }

private:
    static void Tip(const char* text) {
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
            ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*24);ImGui::TextUnformatted(text);ImGui::PopTextWrapPos();ImGui::EndTooltip();
        }
    }
    void DrawPlanePlot(ImVec2 size) const {
        const ImVec2 origin=ImGui::GetCursorScreenPos();
        size.x=std::max(1.0f,size.x);size.y=std::max(1.0f,size.y);
        ImGui::InvisibleButton("Plane plot",size);
        const bool hovered=ImGui::IsItemHovered();
        auto* draw=ImGui::GetWindowDrawList();
        draw->PushClipRect(origin,{origin.x+size.x,origin.y+size.y},true);
        draw->AddRectFilled(origin,{origin.x+size.x,origin.y+size.y},
                            RayFEM::UI::Palette().plot_background,RayFEM::UI::CardRounding);
        draw->AddRect({origin.x+.5f,origin.y+.5f},{origin.x+size.x-.5f,origin.y+size.y-.5f},
                      RayFEM::UI::Palette().border,RayFEM::UI::CardRounding);
        if(size.x<210 || size.y<100 || sampleColumns<2 || sampleRows<2 || samples.size()!=static_cast<size_t>(sampleColumns)*sampleRows) {
            draw->AddText({origin.x+12,origin.y+10},RayFEM::UI::Palette().muted,"Enlarge this panel to view the plane");draw->PopClipRect();return;
        }
        const bool finite=!planeValues.empty();
        const auto& scaleOptions=settings.color_scale;
        if(!planeMappingResolved || planeColorSettings.scale!=scaleOptions.scale ||
           planeColorSettings.log_threshold!=scaleOptions.log_threshold ||
           planeColorSettings.zscale_contrast!=scaleOptions.zscale_contrast) {
            planeColorSettings=scaleOptions;planeMappingResolved=true;planeColorError.clear();planeMapping.reset();
            try {
                // Heatmaps retain their own sampled-plane distribution. Scale
                // changes recolor this snapshot without resampling the field.
                planeMapping=RayFEM::ResolveColorScale(planeMinimum,planeMaximum,scaleOptions,planeValues);
            } catch(const std::exception& error) {planeColorError=error.what();}
        }
        if(!planeMapping) {
            draw->AddText({origin.x+12,origin.y+10},RayFEM::UI::Palette().muted,planeColorError.c_str());
            draw->PopClipRect();return;
        }
        const auto& mapping=*planeMapping;
        const double minimum=RayFEM::ColorScaleValue(0,mapping),maximum=RayFEM::ColorScaleValue(1,mapping);
        const double u0=samples.front().u,u1=samples.back().u,v0=samples.front().v,v1=samples.back().v;
        const double aspect=(u1-u0)/(v1-v0);
        const float textHeight=ImGui::GetTextLineHeight();
        std::array<std::string,3> uLabels,vLabels;
        float verticalLabelWidth=0;
        for(int i=0;i<3;++i) {
            uLabels[i]=RayFEM::UI::PlotTickLabel(RayFEM::UI::PlotCoordinate(i*.5,u0,u1),u0,u1);
            vLabels[i]=RayFEM::UI::PlotTickLabel(RayFEM::UI::PlotCoordinate(i*.5,v0,v1),v0,v1);
            verticalLabelWidth=std::max(verticalLabelWidth,ImGui::CalcTextSize(vLabels[i].c_str()).x);
        }
        char minimumLabel[64],maximumLabel[64];
        std::snprintf(minimumLabel,sizeof(minimumLabel),"%.5g",minimum);
        std::snprintf(maximumLabel,sizeof(maximumLabel),"%.5g",maximum);
        const float legendWidth=std::max(ImGui::CalcTextSize(minimumLabel).x,ImGui::CalcTextSize(maximumLabel).x);
        const float axisLabelWidth=ImGui::CalcTextSize("v").x;
        const float leftMargin=std::max(verticalLabelWidth+axisLabelWidth+19,
                                        ImGui::CalcTextSize(uLabels.front().c_str()).x*.5f);
        const float rightMargin=std::max(finite?37+legendWidth:0,
                                         ImGui::CalcTextSize(uLabels.back().c_str()).x*.5f);
        const float availableWidth=size.x-24-leftMargin-rightMargin;
        const float availableHeight=size.y-26-textHeight*2;
        const float plotWidth=std::min(availableWidth,static_cast<float>(availableHeight*aspect));
        const float plotHeight=static_cast<float>(plotWidth/aspect);
        if(!std::isfinite(plotWidth)||!std::isfinite(plotHeight)||plotWidth<1||plotHeight<1){draw->PopClipRect();return;}
        const float groupWidth=leftMargin+plotWidth+rightMargin;
        const ImVec2 low={origin.x+(size.x-groupWidth)*.5f+leftMargin,
                          origin.y+12+(availableHeight-plotHeight)*.5f};
        const ImVec2 high={low.x+plotWidth,low.y+plotHeight};
        auto cell=[&](int x,int y,bool upper){
            const float u=std::clamp((x+(upper?.5f:-.5f))/(sampleColumns-1),0.0f,1.0f);
            const float v=std::clamp((y+(upper?-.5f:.5f))/(sampleRows-1),0.0f,1.0f);
            return ImVec2{low.x+u*plotWidth,high.y-v*plotHeight};
        };
        for(int y=0;y<sampleRows;++y)for(int x=0;x<sampleColumns;++x) {
            const auto& sample=samples[y*sampleColumns+x];
            const auto mapped=RayFEM::FieldColor(sample.value,mapping,settings.color_map,settings.reverse_colors);
            const ImU32 color=sample.valid&&std::isfinite(sample.value)?IM_COL32(mapped.r,mapped.g,mapped.b,mapped.a):RayFEM::UI::Palette().missing;
            draw->AddRectFilled(cell(x,y,false),cell(x,y,true),color);
        }
        draw->AddRect(low,high,RayFEM::UI::Palette().border);
        for(int i=0;i<=2;++i) {
            const float t=i*.5f;
            const float x=low.x+t*plotWidth,y=high.y-t*plotHeight;
            draw->AddText({x-ImGui::CalcTextSize(uLabels[i].c_str()).x*.5f,high.y+6},
                          RayFEM::UI::Palette().muted,uLabels[i].c_str());
            draw->AddText({low.x-ImGui::CalcTextSize(vLabels[i].c_str()).x-9,y-textHeight*.5f},
                          RayFEM::UI::Palette().muted,vLabels[i].c_str());
        }
        draw->AddText({low.x+(plotWidth-ImGui::CalcTextSize("u").x)*.5f,high.y+textHeight+9},RayFEM::UI::Palette().text,"u");
        draw->AddText({low.x-verticalLabelWidth-axisLabelWidth-19,low.y},RayFEM::UI::Palette().text,"v");
        const float barX=high.x+20,barWidth=11;
        if(finite) {
            for(int i=0;i<64;++i)draw->AddRectFilled({barX,low.y+plotHeight*i/64},{barX+barWidth,low.y+plotHeight*(i+1)/64+.5f},Color(1-i/63.0f));
            draw->AddText({barX+17,low.y},RayFEM::UI::Palette().text,maximumLabel);
            draw->AddText({barX+17,high.y-textHeight},RayFEM::UI::Palette().text,minimumLabel);
        } else {
            const char* empty="No valid samples";
            draw->AddText({low.x+(plotWidth-ImGui::CalcTextSize(empty).x)*.5f,low.y+plotHeight*.5f},RayFEM::UI::Palette().text,empty);
        }
        const ImVec2 mouse=ImGui::GetMousePos();
        if(hovered && mouse.x>=low.x && mouse.x<=high.x && mouse.y>=low.y && mouse.y<=high.y) {
            const int x=std::clamp(static_cast<int>(std::lround((mouse.x-low.x)/plotWidth*(sampleColumns-1))),0,sampleColumns-1);
            const int y=std::clamp(static_cast<int>(std::lround((high.y-mouse.y)/plotHeight*(sampleRows-1))),0,sampleRows-1);
            const auto& sample=samples[y*sampleColumns+x];
            draw->AddLine({mouse.x,low.y},{mouse.x,high.y},RayFEM::UI::Palette().crosshair);
            draw->AddLine({low.x,mouse.y},{high.x,mouse.y},RayFEM::UI::Palette().crosshair);
            // A dual outline remains visible over both ends of the data map.
            draw->AddRect(cell(x,y,false),cell(x,y,true),IM_COL32(20,30,40,255),0,0,3.0f);
            draw->AddRect(cell(x,y,false),cell(x,y,true),IM_COL32(255,255,255,255),0,0,1.2f);
            ImGui::BeginTooltip();ImGui::Text("u %.7g  ·  v %.7g",sample.u,sample.v);
            ImGui::TextDisabled("Position %.6g, %.6g, %.6g",sample.position[0],sample.position[1],sample.position[2]);
            if(sample.valid && std::isfinite(sample.value))ImGui::Text("Value %.9g  ·  element %d",sample.value,sample.element);
            else ImGui::TextUnformatted("Outside mesh");ImGui::EndTooltip();
        }
        draw->PopClipRect();
    }
    static RayFEM::ProfilePosition Position(Vector3 p){return {p.x,p.y,p.z};}
    static Vector3 Vector(const RayFEM::ProfilePosition& p){return {static_cast<float>(p[0]),static_cast<float>(p[1]),static_cast<float>(p[2])};}
    ImU32 Color(float t) const {
        const auto c=RayFEM::SampleColorMap(settings.color_map,t,settings.reverse_colors);
        return IM_COL32(c.r,c.g,c.b,c.a);
    }
    // Start and end (plus the midpoint for rays); returns how many are used.
    int Handles(Vector3 (&points)[3]) const {
        points[0]=mode==1?origin:center;
        points[1]=Vector3Add(points[0],Vector3Scale(Vector3Normalize(mode==1?direction:normal),mode==1?length:scale*0.25f));
        points[2]=Vector3Scale(Vector3Add(points[0],points[1]),.5f);
        return mode==1?3:2;
    }
    // Distance from a point to the far side of the mesh bounds along a direction.
    float ExitDistance(Vector3 from,Vector3 along) const {
        float exit=std::numeric_limits<float>::infinity();
        const float low[3]={meshBounds.min.x,meshBounds.min.y,meshBounds.min.z},high[3]={meshBounds.max.x,meshBounds.max.y,meshBounds.max.z};
        const float p[3]={from.x,from.y,from.z},d[3]={along.x,along.y,along.z};
        for(int axis=0;axis<3;++axis)if(std::abs(d[axis])>1e-9f)
            exit=std::min(exit,std::max((low[axis]-p[axis])/d[axis],(high[axis]-p[axis])/d[axis]));
        return std::isfinite(exit)&&exit>0?exit:scale;
    }
public:
    // Presets: a ray between the mesh bounds along an axis or the view direction.
    void SetRay(Vector3 start,Vector3 unit,float distance) {
        origin=start;direction=unit;length=std::max(scale*.01f,distance);rayPlaced=true;anchors={};dirty=true;lastRequest={};
    }
    void AxisRay(int axis) {
        Vector3 unit{};(&unit.x)[axis]=1;
        Vector3 start=center;(&start.x)[axis]=(&meshBounds.min.x)[axis];
        SetRay(start,unit,(&meshBounds.max.x)[axis]-(&meshBounds.min.x)[axis]);
    }
    void ViewRay() {
        const Vector3 forward=Vector3Normalize(Vector3Subtract(lastCamera.target,lastCamera.position));
        if(!(Vector3Length(forward)>0))return;
        const float back=ExitDistance(center,Vector3Negate(forward));
        SetRay(Vector3Subtract(center,Vector3Scale(forward,back)),forward,back+ExitDistance(center,forward));
    }
private:
    // Re-evaluate anchored endpoints on the current frame.
    void ApplyAnchors(bool force) {
        if(!anchors[0].Valid() && !anchors[1].Valid())return;
        if(!context || !context->mesh)return;
        if(anchorMesh.lock()!=context->mesh) {
            anchors={};message="The mesh changed; place the ray endpoints again.";return;
        }
        Vector3 start=origin,end=Vector3Add(origin,Vector3Scale(Vector3Normalize(direction),length));
        try {
            if(anchors[0].Valid())if(auto p=RayFEM::UI::AnchorPosition(*context->mesh,anchors[0],settings))start=*p;
            if(anchors[1].Valid())if(auto p=RayFEM::UI::AnchorPosition(*context->mesh,anchors[1],settings))end=*p;
        } catch(const std::exception& e){message=e.what();return;}
        const Vector3 delta=Vector3Subtract(end,start);
        if(!(Vector3Length(delta)>0))return;
        const Vector3 unit=Vector3Normalize(delta);
        if(force || Vector3Distance(start,origin)>0 || Vector3Distance(unit,Vector3Normalize(direction))>0 || Vector3Length(delta)!=length) {
            origin=start;direction=unit;length=Vector3Length(delta);dirty=true;
            if(force)lastRequest={};
        }
    }
    // Live updates: a quick preview while dragging, then a full
    // sample once the profile is still. Frame changes resample too.
    void Tick() {
        if(!liveUpdate || !dirty || inFlight || !context || !fieldPointer || mode==0)return;
        const auto now=Clock::now();
        const bool moving=Dragging();
        if(now-lastRequest<std::chrono::milliseconds(moving?60:120))return;
        Sample(moving);
    }
    void Sample(bool preview) {
        if(!context || !fieldPointer) return;
        lastRequest=Clock::now();
        try {
            const int resolution=preview?std::min(count,96):count;
            const int previewColumns=preview?std::min(columns,16):columns,previewRows=preview?std::min(rows,16):rows;
            Job job;
            job.locations=mode==1?RayFEM::MakeRaySamples(Position(origin),Position(Vector3Normalize(direction)),length,resolution):
                RayFEM::MakePlaneSamples(Position(center),Position(normal),width,height,previewColumns,previewRows);
            job.eval=worker?context->compute:context;
            job.inlineField=worker?nullptr:fieldPointer;
            job.field=currentFieldName;job.settings=settings;job.algorithm=algorithm;job.registry=registry;
            job.mode=mode;job.columns=previewColumns;job.rows=previewRows;job.preview=preview;job.generation=++generation;
            job.metadata={currentFieldName,context->metadata.frame_index,context->metadata.sim_time,settings.view,settings.component,algorithm,settings.excluded_materials,settings.excluded_elements};
            job.frameIndex=context->metadata.frame_index;
            if(!job.eval)throw std::runtime_error("Frame has no compute copy");
            if(!worker) {Install(Run(std::move(job)));return;}
            inFlight=true;
            worker->Submit("profile",[job=std::move(job),state=shared]() mutable {
                auto result=Run(std::move(job));
                std::lock_guard lock(state->mutex);
                state->done.push_back(std::move(result));
            });
        } catch(const std::exception& e){message=e.what();}
    }
    struct Job {
        std::shared_ptr<RayFEM::RenderFrame> eval;
        mfem::GridFunction* inlineField=nullptr;
        std::string field,algorithm,error;
        RayFEM::RenderOptions settings;
        RayFEM::ProfileRegistry registry;
        std::vector<RayFEM::ProfileSample> locations,samples;
        RayFEM::ProfileMetadata metadata;
        int mode=1,columns=0,rows=0;
        bool preview=false;
        std::uint64_t generation=0;
        std::uint32_t frameIndex=0;
        double milliseconds=0;
    };
    struct Shared {std::mutex mutex;std::vector<Job> done;};
    // Evaluation only; touches job.eval (a compute twin on the worker).
    static Job Run(Job job) {
        const auto start=Clock::now();
        try {
            mfem::GridFunction* field=job.inlineField;
            auto settings=job.settings;
            if(!field) {
                for(const auto& item:job.eval->fields)if(item.name==job.field){field=item.gf.get();break;}
                if(!field)throw std::runtime_error("The profile field is not in this frame");
                settings.displacement=settings.displacement&&job.eval->displacement?job.eval->displacement.get():nullptr;
            }
            job.samples=job.registry.sample(job.algorithm,*job.eval->mesh,*field,job.locations,settings);
        } catch(const std::exception& e){job.error=e.what();}
        job.milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        job.locations.clear();job.eval.reset();
        return job;
    }
    void Collect() {
        std::vector<Job> done;
        {std::lock_guard lock(shared->mutex);done.swap(shared->done);}
        for(auto& job:done){inFlight=false;Install(std::move(job));}
    }
    void Install(Job job) {
        if(job.generation<installed)return;
        installed=job.generation;
        const bool latest=job.generation==generation;
        if(!job.error.empty()){message=job.error;if(latest)dirty=!liveUpdate;return;}
        samples=std::move(job.samples);sampleMode=job.mode;sampleColumns=job.columns;sampleRows=job.rows;
        metadata=std::move(job.metadata);
        sampledLength=sampleMode==1&&!samples.empty()?samples.back().s:0;
        valid=std::count_if(samples.begin(),samples.end(),[](const auto& s){return s.valid && std::isfinite(s.value);});
        linePoints.clear();linePoints.reserve(samples.size());
        planeValues.clear();planeValues.reserve(valid);
        for(const auto& s:samples) {
            linePoints.push_back({s.s,s.value,s.valid});
            if(s.valid && std::isfinite(s.value))planeValues.push_back(s.value);
        }
        planeMinimum=0;planeMaximum=1;
        if(!planeValues.empty()) {
            const auto [minimum,maximum]=std::minmax_element(planeValues.begin(),planeValues.end());
            planeMinimum=*minimum;planeMaximum=*maximum;
        }
        planeMappingResolved=false;
        sampleMs=job.milliseconds;
        // A preview leaves the profile dirty so the full sample follows.
        if(latest)dirty=job.preview;
        message.clear();
        std::snprintf(path,sizeof(path),"rayfem-exports/%s-frame-%u.csv",sampleMode==1?"ray":"plane",job.frameIndex);
    }
    void Export() {
        try {
            std::ostringstream csv;RayFEM::ExportProfileCSV(csv,samples,metadata);
            auto destination=std::filesystem::absolute(path);
            if(destination.has_parent_path()) std::filesystem::create_directories(destination.parent_path());
            std::ofstream output(destination);output.exceptions(std::ios::badbit|std::ios::failbit);output<<csv.str();output.close();
            message="Saved "+destination.string();
        } catch(const std::exception& e){message="Export failed: "+std::string(e.what());}
    }

    RayFEM::ProfileRegistry registry;
    int mode=0,count=128,columns=24,rows=24,sampleMode=0,sampleColumns=0,sampleRows=0,handle=-1,hoverHandle=-1;
    Vector3 origin{},direction{1,0,0},center{},normal{0,0,1},anchor{},dragNormal{};
    Vector3 pressOrigin{},pressEnd{},pressCenter{};
    std::array<RayFEM::UI::RayAnchor,2> anchors{};
    std::weak_ptr<mfem::Mesh> anchorMesh;
    std::optional<int> placementRequest;
    BoundingBox meshBounds{{-1,-1,-1},{1,1,1}};
    Camera3D lastCamera{};
    bool rayPlaced=false,liveUpdate=true,inFlight=false;
    double sampledLength=0;
    mutable double rayHoverS=std::numeric_limits<double>::quiet_NaN(),plotHoverS=std::numeric_limits<double>::quiet_NaN();
    RayFEM::UI::FramePipeline* worker=nullptr;
    std::shared_ptr<Shared> shared=std::make_shared<Shared>();
    std::uint64_t generation=0,installed=0;
    Clock::time_point lastRequest{};
    float length=1,width=1,height=1,scale=1;
    bool dirty=true;
    std::string algorithm="coherent",message,currentFieldName;
    std::weak_ptr<mfem::Mesh> lastMesh;
    std::shared_ptr<RayFEM::RenderFrame> context;
    mfem::GridFunction* fieldPointer=nullptr;
    RayFEM::RenderOptions settings;
    RayFEM::ProfileMetadata metadata;
    std::vector<RayFEM::ProfileSample> samples;
    std::vector<RayFEM::UI::PlotPoint> linePoints;
    std::vector<double> planeValues;
    double planeMinimum=0,planeMaximum=1;
    mutable bool planeMappingResolved=false;
    mutable std::optional<RayFEM::ColorScaleMapping> planeMapping;
    mutable RayFEM::ColorScaleOptions planeColorSettings;
    mutable std::string planeColorError;
    size_t valid=0;
    double sampleMs=0;
    char path[512]="rayfem-exports/profile.csv";
};
