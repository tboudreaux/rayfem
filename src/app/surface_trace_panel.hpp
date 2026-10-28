#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/surface_trace.hpp"
#include "plot.hpp"
#include "raymath.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>

// Sampled results are an explicit snapshot. Keeping the current frame alive
// protects field/displacement pointers while the sidebar is on another tab.
class SurfaceTracePanel {
public:
    bool Enabled() const {return enabled;}
    void Open() {SetEnabled(true);}
    void SetEnabled(bool value) {enabled=value;}

    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                       mfem::GridFunction* field,const std::string& name,
                       const RayFEM::RenderOptions& options) {
        if(context!=frame || fieldPointer!=field || fieldName!=name ||
           settings.view!=options.view || settings.component!=options.component ||
           settings.displacement!=options.displacement || settings.displacement_scale!=options.displacement_scale ||
           settings.excluded_materials!=options.excluded_materials ||
           settings.excluded_elements!=options.excluded_elements)dirty=true;
        context=frame;fieldPointer=field;fieldName=name;settings=options;
    }

    void SetBounds(BoundingBox bounds,const std::shared_ptr<mfem::Mesh>& mesh) {
        if(lastMesh.lock()==mesh)return;
        lastMesh=mesh;
        center=Vector3Scale(Vector3Add(bounds.min,bounds.max),.5f);
        scale=std::max(.01f,Vector3Length(Vector3Subtract(bounds.max,bounds.min)));
        dirty=true;
    }

    void Controls(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                  mfem::GridFunction* field,const std::string& name,
                  const RayFEM::RenderOptions& options) {
        UpdateContext(frame,field,name,options);
        ImGui::PushID("Surface trace controls");
        if(!ImGui::CollapsingHeader("Surface trace",ImGuiTreeNodeFlags_DefaultOpen)) {ImGui::PopID();return;}
        ImGui::Checkbox("Enabled",&enabled);
        Tip("Measure a meridian or latitude on the outer surface.");
        if(!enabled) {ImGui::PopID();return;}
        ImGui::SetNextItemWidth(-1);
        if(ImGui::Combo("##trace_kind",&kind,"Meridian / colatitude\0Latitude / azimuth\0")) {
            beginAngle=0;endAngle=kind==0?180:360;fixedAngle=kind==0?0:90;dirty=true;
        }
        RayFEM::UI::FieldLabel(kind==0?"Azimuth (degrees)":"Colatitude (degrees)");
        dirty|=ImGui::SliderFloat("##fixed_angle",&fixedAngle,0,kind==0?360.0f:180.0f,"%.1f");
        Tip("Colatitude is measured from the axis: 0 at the north pole, 90 at the equator. Azimuth starts at the projected world X direction (world Y when X is nearly parallel to the axis).");
        const bool supported=context && context->mesh && context->mesh->SpaceDimension()==3;
        ImGui::BeginDisabled(!supported || !fieldPointer);
        if(ImGui::Button(samples.empty()?"Sample surface":"Update surface trace",{-1,0}))Sample();
        ImGui::EndDisabled();
        if(context && !supported)ImGui::TextWrapped("Choose a volume or a surface embedded in 3D.");
        if(!samples.empty()) {
            if(dirty)ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Update to refresh");
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().success),"Current sample");
            ImGui::TextDisabled("%zu / %zu hits  ·  %.1f ms",valid,samples.size(),sampleMs);
            if(ImGui::IsItemHovered())ImGui::SetTooltip("Largest ray residual: %.6g\nGaps mark rays with no exterior hit. Only original exterior faces are traced; material exclusions are respected.",maxResidual);
        }
        if(ImGui::CollapsingHeader("Trace settings")) {
            ImGui::PushItemWidth(-1);
            ImGui::TextDisabled("Center");dirty|=ImGui::DragFloat3("##center",&center.x,scale*.005f,0,0,"%.4g");
            Tip("Rays start here and use the nearest forward exterior hit. A center inside the object usually gives the clearest trace.");
            ImGui::TextDisabled("Axis");dirty|=ImGui::DragFloat3("##axis",&axis.x,.01f,0,0,"%.4g");
            RayFEM::UI::FieldLabel("Start angle (degrees)");dirty|=ImGui::DragFloat("##begin",&beginAngle,.5f,0,kind==0?180.0f:360.0f,"%.1f",ImGuiSliderFlags_AlwaysClamp);
            RayFEM::UI::FieldLabel("End angle (degrees)");dirty|=ImGui::DragFloat("##end",&endAngle,.5f,0,kind==0?180.0f:360.0f,"%.1f",ImGuiSliderFlags_AlwaysClamp);
            RayFEM::UI::FieldLabel("Samples");dirty|=ImGui::SliderInt("##samples",&count,2,2048);
            RayFEM::UI::FieldLabel("Surface refinement");dirty|=ImGui::SliderInt("##refinement",&refinement,1,16);
            Tip("More subdivisions improve surface search seeds and the approximate method. Field values are evaluated on the actual finite-element face.");
            const auto algorithms=registry.algorithms();
            const auto current=std::find_if(algorithms.begin(),algorithms.end(),[&](const auto& item){return item.id==algorithm;});
            RayFEM::UI::FieldLabel("Sampling method");
            if(ImGui::BeginCombo("##method",current==algorithms.end()?algorithm.c_str():current->name.c_str())) {
                for(const auto& item:algorithms)if(ImGui::Selectable(item.name.c_str(),item.id==algorithm)){algorithm=item.id;dirty=true;}
                ImGui::EndCombo();
            }
            Tip("Curved intersection solves for the ray on the true curved face. Refined approximation uses triangle intersections projected onto that face; the residual reports alignment error.");
            ImGui::PopItemWidth();
        }
        if(!samples.empty() && ImGui::CollapsingHeader("Export")) {
            ImGui::TextDisabled("CSV destination");ImGui::SetNextItemWidth(-1);ImGui::InputText("##path",path,sizeof(path));
            ImGui::BeginDisabled(dirty);
            if(ImGui::Button("Save CSV",{-1,0}))Export();
            Tip("Includes angle in radians, distance, physical position, and face details.");
            ImGui::EndDisabled();
            if(dirty)ImGui::TextWrapped("Update the trace before exporting.");
        }
        if(!message.empty())ImGui::TextWrapped("%s",message.c_str());
        ImGui::PopID();
    }

    void DrawGuide(const Camera3D& camera) const {
        if(!enabled || !context || !context->mesh || context->mesh->SpaceDimension()!=3)return;
        if(Finite(center) && Finite(axis) && Vector3Length(axis)>0) {
            const Vector3 tip=Vector3Add(center,Vector3Scale(Vector3Normalize(axis),scale*.3f));
            DrawLine3D(center,tip,RayFEM::UI::ThemeAlpha(RayFEM::UI::Palette().profile_guide,180));
            DrawSphere(center,scale*.004f,RayFEM::UI::Palette().profile_guide);
        }
        if(dirty)return;
        Vector3 previous{};bool previousValid=false;
        for(const auto& sample:samples) {
            Vector3 point=Vector(sample.position);
            if(!sample.valid || !Finite(point)){previousValid=false;continue;}
            // Lift only the guide toward the camera to avoid surface z-fighting.
            point=Vector3Add(point,Vector3Scale(Vector3Normalize(Vector3Subtract(camera.position,point)),scale*.0003f));
            if(previousValid)DrawLine3D(previous,point,RayFEM::UI::Palette().profile_guide);
            previous=point;previousValid=true;
        }
    }

    void DrawPlot(ImVec2 size) const {
        ImGui::BeginChild("Surface trace plot",size,ImGuiChildFlags_None,ImGuiWindowFlags_NoScrollbar);
        if(samples.empty()) {
            ImGui::Spacing();ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"No surface trace sampled");
            ImGui::TextWrapped("Choose a meridian or latitude in Analyze, position it, then sample.");
            ImGui::EndChild();return;
        }
        ImGui::SetNextItemWidth(120);ImGui::Combo("##surface_axis",&plotAxis,"Angle\0Arc length\0");
        Tip("Arc length is the sum of chords between neighboring valid hits; gaps are never bridged. Increase the sample count for a closer arc-length approximation.");
        char stamp[96];
        std::snprintf(stamp,sizeof(stamp),"  ·  frame %llu  ·  t %.5g",
                      static_cast<unsigned long long>(metadata.frame),metadata.time);
        const std::string details=metadata.field_name+stamp;
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details.c_str()).x);
        ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().muted));
        ImGui::TextWrapped("%s",details.c_str());ImGui::PopStyleColor();
        if(dirty) {
            RayFEM::UI::NextInlineItem(ImGui::CalcTextSize("Previous sample").x);
            ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Previous sample");
        }
        const auto& points=plotAxis==0?anglePoints:distancePoints;
        const RayFEM::UI::PlotSeriesView series{metadata.field_name.c_str(),points,RayFEM::UI::PlotColor(1)};
        RayFEM::UI::PlotOptions options;options.legend=false;options.y_label="Value";
        options.x_label=plotAxis==0?(sampleKind==0?"Colatitude (degrees)":"Azimuth (degrees)"):"Arc length";
        RayFEM::UI::PlotLines("##surface_values",std::span(&series,1),ImGui::GetContentRegionAvail(),options);
        ImGui::EndChild();
    }

private:
    static constexpr double radians=std::numbers::pi/180.0;
    static RayFEM::ProfilePosition Position(Vector3 p){return {p.x,p.y,p.z};}
    static Vector3 Vector(const RayFEM::ProfilePosition& p){return {static_cast<float>(p[0]),static_cast<float>(p[1]),static_cast<float>(p[2])};}
    static bool Finite(Vector3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
    static void Tip(const char* text) {
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
            ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*25);ImGui::TextUnformatted(text);ImGui::PopTextWrapPos();ImGui::EndTooltip();
        }
    }
    void Sample() {
        if(!context || !fieldPointer)return;
        try {
            const auto start=std::chrono::steady_clock::now();
            if(!(endAngle>beginAngle))throw std::invalid_argument("The end angle must be larger than the start angle.");
            const Vector3 unit=Vector3Normalize(axis);
            const RayFEM::ProfilePosition reference=std::abs(unit.x)<.95f?RayFEM::ProfilePosition{1,0,0}:RayFEM::ProfilePosition{0,1,0};
            auto rays=kind==0?
                RayFEM::MakeMeridianRays(Position(center),Position(axis),reference,fixedAngle*radians,beginAngle*radians,endAngle*radians,count):
                RayFEM::MakeAzimuthRays(Position(center),Position(axis),reference,fixedAngle*radians,beginAngle*radians,endAngle*radians,count);
            auto options=settings;options.refinement=refinement;
            auto result=registry.sample(algorithm,*context->mesh,*fieldPointer,rays,options);
            samples=std::move(result);sampleKind=kind;
            metadata={fieldName,context->metadata.frame_index,context->metadata.sim_time,settings.view,settings.component,algorithm,settings.excluded_materials,settings.excluded_elements};
            valid=0;maxResidual=0;anglePoints.clear();distancePoints.clear();
            anglePoints.reserve(samples.size());distancePoints.reserve(samples.size());
            for(const auto& sample:samples) {
                const bool finite=sample.valid && std::isfinite(sample.value);
                valid+=finite;
                if(finite && std::isfinite(sample.residual))maxResidual=std::max(maxResidual,sample.residual);
                anglePoints.push_back({sample.parameter/radians,sample.value,finite});
                distancePoints.push_back({sample.distance,sample.value,finite});
            }
            sampleMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            dirty=false;message.clear();
            std::snprintf(path,sizeof(path),"rayfem-exports/surface-%s-frame-%u.csv",kind==0?"meridian":"latitude",context->metadata.frame_index);
        } catch(const std::exception& error){message=error.what();dirty=true;}
    }
    void Export() {
        try {
            std::ostringstream csv;RayFEM::ExportSurfaceTraceCSV(csv,samples,metadata);
            const auto destination=std::filesystem::absolute(path);
            if(destination.has_parent_path())std::filesystem::create_directories(destination.parent_path());
            std::ofstream output(destination);output.exceptions(std::ios::badbit|std::ios::failbit);output<<csv.str();output.close();
            message="Saved "+destination.string();
        } catch(const std::exception& error){message="Export failed: "+std::string(error.what());}
    }

    RayFEM::SurfaceTraceRegistry registry;
    std::shared_ptr<RayFEM::RenderFrame> context;
    std::weak_ptr<mfem::Mesh> lastMesh;
    mfem::GridFunction* fieldPointer=nullptr;
    RayFEM::RenderOptions settings;
    RayFEM::ProfileMetadata metadata;
    std::vector<RayFEM::SurfaceTraceSample> samples;
    std::vector<RayFEM::UI::PlotPoint> anglePoints,distancePoints;
    std::string fieldName,algorithm="curved",message;
    bool enabled=false,dirty=true;
    int kind=0,sampleKind=0,count=256,refinement=4;
    mutable int plotAxis=0;
    Vector3 center{},axis{0,0,1};
    float scale=1,fixedAngle=0,beginAngle=0,endAngle=180;
    size_t valid=0;
    double sampleMs=0,maxResidual=0;
    char path[512]="rayfem-exports/surface.csv";
};
