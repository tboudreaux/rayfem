#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/sampler.hpp"
#include "imgui.h"
#include "raymath.h"
#include "plot.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>

// The panel owns only controls and sampled results. All point location and
// finite-element evaluation goes through the replaceable ProfileRegistry.
class ProfilePanel {
public:
    bool Enabled() const { return mode != 0; }
    bool Dragging() const { return handle>=0; }
    int Mode() const { return mode; }
    void SetMode(int value) {
        value=std::clamp(value,0,2);
        if(mode!=value){mode=value;dirty=true;message.clear();handle=-1;}
    }
    void OpenRay() { SetMode(1); }
    void OpenPlane() { SetMode(2); }

    // Call even when this panel's controls are hidden in another sidebar tab.
    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                       mfem::GridFunction* field,const std::string& fieldName,
                       const RayFEM::RenderOptions& options) {
        if(context!=frame || fieldPointer!=field || currentFieldName!=fieldName ||
           settings.view!=options.view || settings.component!=options.component ||
           settings.displacement!=options.displacement || settings.displacement_scale!=options.displacement_scale ||
           settings.excluded_materials!=options.excluded_materials ||
           settings.excluded_elements!=options.excluded_elements)
            dirty=true;
        context=frame;fieldPointer=field;settings=options;currentFieldName=fieldName;
    }

    void SetBounds(BoundingBox bounds, const std::shared_ptr<mfem::Mesh>& mesh) {
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
        ImGui::PushID("Profile controls");
        int selectedMode=mode;
        const char* modes[]={"Profiles off","Ray profile","Plane profile"};
        ImGui::SetNextItemWidth(-1);
        if(ImGui::Combo("##mode",&selectedMode,modes,3))SetMode(selectedMode);
        Tip("Sample the field along a ray or across a plane. Move the handles in the viewport to position the profile.");
        if(!Enabled()) {
            ImGui::PopID();return;
        }
        ImGui::TextDisabled("Drag handles, then sample.");
        ImGui::BeginDisabled(!context || !fieldPointer);
        if(ImGui::Button(samples.empty()?"Sample profile":"Update profile",{-1,0}))Sample();
        ImGui::EndDisabled();
        if(!samples.empty()) {
            if(dirty)ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Update to refresh");
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().success),"Current sample");
            ImGui::TextDisabled("%zu / %zu inside  ·  %.1f ms",valid,samples.size(),sampleMs);
        }
        ImGui::Spacing();
        if(ImGui::CollapsingHeader("Sampling settings")) {
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
        }
        if(!samples.empty() && ImGui::CollapsingHeader("Export")) {
            ImGui::TextDisabled("CSV destination");ImGui::SetNextItemWidth(-1);
            ImGui::InputText("##csv_path",path,sizeof(path));
            ImGui::BeginDisabled(dirty);
            if(ImGui::Button("Save CSV",{-1,0}))Export();
            ImGui::EndDisabled();
            if(dirty)ImGui::TextWrapped("Update the profile before exporting.");
        }
        if(!message.empty())ImGui::TextWrapped("%s",message.c_str());
        ImGui::PopID();
    }

    // Handles move in a screen-parallel plane through the selected handle.
    // Returning true reserves that drag so it cannot also orbit the camera.
    bool HandleInput(ImVec2 viewport,ImVec2 size,const Camera3D& camera,bool hovered) {
        if (!Enabled() || !IsWindowFocused()) {handle=-1;return false;}
        Vector3 points[2];Handles(points);
        Vector2 mouse=GetMousePosition();
        int near=-1;float best=100;
        for(int i=0;i<2;++i) {
            if(Vector3DotProduct(Vector3Subtract(points[i],camera.position),Vector3Subtract(camera.target,camera.position))<=0) continue;
            auto p=GetWorldToScreenEx(points[i],camera,static_cast<int>(size.x),static_cast<int>(size.y));
            float dx=p.x+viewport.x-mouse.x,dy=p.y+viewport.y-mouse.y;
            float d=dx*dx+dy*dy;
            if(d<best){best=d;near=i;}
        }
        if (hovered && near>=0 && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            handle=near;anchor=points[handle];dragNormal=Vector3Normalize(Vector3Subtract(camera.target,camera.position));
        }
        if (handle>=0) {
            if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {handle=-1;return true;}
            Ray ray=GetScreenToWorldRayEx({mouse.x-viewport.x,mouse.y-viewport.y},camera,static_cast<int>(size.x),static_cast<int>(size.y));
            float denominator=Vector3DotProduct(ray.direction,dragNormal);
            if(std::abs(denominator)>1e-7f) {
                float t=Vector3DotProduct(Vector3Subtract(anchor,ray.position),dragNormal)/denominator;
                Vector3 point=Vector3Add(ray.position,Vector3Scale(ray.direction,t));
                if (mode==1) {
                    if(handle==0) origin=point;
                    else {Vector3 d=Vector3Subtract(point,origin);length=std::max(0.0001f,Vector3Length(d));direction=Vector3Normalize(d);}
                } else {
                    if(handle==0) center=point;
                    else normal=Vector3Normalize(Vector3Subtract(point,center));
                }
                dirty=true;
            }
            return true;
        }
        return false;
    }

    void DrawGuide() const {
        if(!Enabled()) return;
        Vector3 points[2];Handles(points);
        for (const auto& p:points) if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) return;
        DrawLine3D(points[0],points[1],RayFEM::UI::Palette().profile_guide);
        if(mode==2 && Vector3Length(normal)>0.001f && std::isfinite(width) && std::isfinite(height) && width>0 && height>0) {
            auto corners=RayFEM::MakePlaneSamples(Position(center),Position(normal),width,height,2,2);
            for(auto edge: {std::pair{0,1},std::pair{1,3},std::pair{3,2},std::pair{2,0}})
                DrawLine3D(Vector(corners[edge.first].position),Vector(corners[edge.second].position),RayFEM::UI::Palette().profile_guide);
        }
    }

    void DrawHandles(ImVec2 viewport,ImVec2 size,const Camera3D& camera) const {
        if(!Enabled()) return;
        auto* draw=ImGui::GetWindowDrawList();
        draw->PushClipRect(viewport,{viewport.x+size.x,viewport.y+size.y},true);
        Vector3 points[2];Handles(points);
        ImVec2 previousLabelMin{},previousLabelMax{};
        bool hasPreviousLabel=false;
        for(int i=0;i<2;++i) {
            if(Vector3DotProduct(Vector3Subtract(points[i],camera.position),Vector3Subtract(camera.target,camera.position))<=0) continue;
            auto p=GetWorldToScreenEx(points[i],camera,static_cast<int>(size.x),static_cast<int>(size.y));
            ImVec2 screen={viewport.x+p.x,viewport.y+p.y};
            if(screen.x<viewport.x-8 || screen.x>viewport.x+size.x+8 ||
               screen.y<viewport.y-8 || screen.y>viewport.y+size.y+8)continue;
            draw->AddCircleFilled(screen,6,RayFEM::UI::Palette().guide);
            draw->AddCircle(screen,8,RayFEM::UI::Palette().handle_outline,0,2);
            const char* label=mode==1?(i==0?"ray origin":"ray end"):(i==0?"plane center":"plane normal");
            const ImVec2 labelSize=ImGui::CalcTextSize(label);
            const ImVec2 boxSize={labelSize.x+12,labelSize.y+8};
            const float right=std::max(viewport.x+4,viewport.x+size.x-boxSize.x-4);
            const float bottom=std::max(viewport.y+4,viewport.y+size.y-boxSize.y-4);
            ImVec2 labelMin={std::clamp(screen.x+11,viewport.x+4,right),
                             std::clamp(screen.y-boxSize.y*.5f,viewport.y+4,bottom)};
            if(hasPreviousLabel && labelMin.x<previousLabelMax.x+4 &&
               labelMin.x+boxSize.x>previousLabelMin.x-4 &&
               labelMin.y<previousLabelMax.y+4 && labelMin.y+boxSize.y>previousLabelMin.y-4) {
                const float below=previousLabelMax.y+5;
                labelMin.y=below<=bottom?below:std::max(viewport.y+4,previousLabelMin.y-boxSize.y-5);
            }
            const ImVec2 labelMax={labelMin.x+boxSize.x,labelMin.y+boxSize.y};
            draw->AddRectFilled(labelMin,labelMax,RayFEM::UI::Palette().plot_background,RayFEM::UI::OverlayRounding);
            draw->AddText({labelMin.x+6,labelMin.y+4},RayFEM::UI::Palette().guide_text,label);
            previousLabelMin=labelMin;previousLabelMax=labelMax;hasPreviousLabel=true;
        }
        draw->PopClipRect();
    }

    void DrawPlot(ImVec2 size) const {
        if(!Enabled())return;
        ImGui::BeginChild("Profile plot",size,ImGuiChildFlags_None,ImGuiWindowFlags_NoScrollbar);
        if(samples.empty()) {
            ImGui::Spacing();ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"No profile sampled");
            ImGui::TextWrapped("Position the handles, then choose Sample profile in Analyze.");
            ImGui::EndChild();return;
        }
        ImGui::TextWrapped("%s",metadata.field_name.c_str());
        char details[128];
        std::snprintf(details,sizeof(details),"Frame %llu  ·  t %.5g  ·  %zu/%zu inside",
                      static_cast<unsigned long long>(metadata.frame),metadata.time,valid,samples.size());
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details).x);
        ImGui::TextDisabled("%s",details);
        if(dirty) {
            RayFEM::UI::NextInlineItem(ImGui::CalcTextSize("Previous sample").x);
            ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Previous sample");
        }
        if(sampleMode==1) {
            const RayFEM::UI::PlotSeriesView series{metadata.field_name.c_str(),linePoints,RayFEM::UI::PlotColor(0)};
            RayFEM::UI::PlotOptions options;options.x_label="Distance along ray";options.y_label="Value";options.legend=false;
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
    void Handles(Vector3 (&points)[2]) const {
        points[0]=mode==1?origin:center;
        points[1]=Vector3Add(points[0],Vector3Scale(Vector3Normalize(mode==1?direction:normal),mode==1?length:scale*0.25f));
    }
    void Sample() {
        if(!context || !fieldPointer) return;
        try {
            auto start=std::chrono::steady_clock::now();
            auto locations=mode==1?RayFEM::MakeRaySamples(Position(origin),Position(direction),length,count):RayFEM::MakePlaneSamples(Position(center),Position(normal),width,height,columns,rows);
            auto result=registry.sample(algorithm,*context->mesh,*fieldPointer,locations,settings);
            samples=std::move(result);sampleMode=mode;sampleColumns=columns;sampleRows=rows;
            metadata={currentFieldName,context->metadata.frame_index,context->metadata.sim_time,settings.view,settings.component,algorithm,settings.excluded_materials,settings.excluded_elements};
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
            sampleMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            dirty=false;message.clear();
            std::snprintf(path,sizeof(path),"rayfem-exports/%s-frame-%u.csv",mode==1?"ray":"plane",context->metadata.frame_index);
        } catch(const std::exception& e){message=e.what();dirty=true;}
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
    int mode=0,count=128,columns=24,rows=24,sampleMode=0,sampleColumns=0,sampleRows=0,handle=-1;
    Vector3 origin{},direction{1,0,0},center{},normal{0,0,1},anchor{},dragNormal{};
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
