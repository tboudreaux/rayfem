#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/surface_shape.hpp"
#include "pipeline/frame_pipeline.hpp"
#include "ui/plot.hpp"
#include "ui/widgets.hpp"
#include "raymath.h"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <numbers>
#include <optional>
#include <sstream>

// The outer surface's shape about an axis: radius against colatitude on
// several meridians and around the equator, the polar and equatorial radii,
// flattening, and the flattening of each frame measured while it is on. The
// rays meet the true curved faces (the curved surface trace), so the radii
// are not limited by the drawn tessellation; displacement is included.
class ShapePanel {
    using Clock=std::chrono::steady_clock;
public:
    bool Enabled() const {return enabled;}
    void SetEnabled(bool value) {enabled=value;if(enabled)dirty=true;}
    void SetWorker(RayFEM::UI::FramePipeline* pipeline) {worker=pipeline;}
    bool TakePlotRequest() {const bool request=plotRequest;plotRequest=false;return request;}

    // A rotation axis the client reported, if any: the default axis, and its
    // line places the default center.
    void SetModelAxis(std::optional<std::pair<Vector3,Vector3>> directionAndOrigin,std::string name) {
        modelAxis=directionAndOrigin;modelAxisName=std::move(name);
        if(!modelAxis && axisSource==0)axisSource=3;
        if(modelAxis && !modelAxisSeen){axisSource=0;modelAxisSeen=true;}
        UpdateFrame();
    }
    void SetBounds(BoundingBox bounds,const std::shared_ptr<mfem::Mesh>& mesh) {
        meshCenter=Vector3Scale(Vector3Add(bounds.min,bounds.max),.5f);
        scale=std::max(.01f,Vector3Length(Vector3Subtract(bounds.max,bounds.min)));
        if(lastMesh.lock()!=mesh){lastMesh=mesh;dirty=true;}
        UpdateFrame();
    }
    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,mfem::GridFunction* field,const std::string& name,
                       const RayFEM::RenderOptions& options) {
        const bool changed=context!=frame || settings.displacement!=options.displacement ||
            settings.displacement_scale!=options.displacement_scale || settings.excluded_materials!=options.excluded_materials ||
            settings.excluded_elements!=options.excluded_elements;
        if(changed)dirty=true;
        context=frame;fieldPointer=field;fieldName=name;settings=options;
        Collect();
        if(enabled && dirty && !inFlight && context && fieldPointer && Clock::now()-lastRequest>std::chrono::milliseconds(150))Measure();
    }

    void Controls() {
        if(!RayFEM::UI::BeginSection("Surface shape",enabled && shape?Flattening().c_str():nullptr,false))return;
        ImGui::PushID("Surface shape controls");
        if(RayFEM::UI::Switch("Enabled",&enabled) && enabled){dirty=true;plotRequest=true;}
        Tip("Measure the outer surface's radius from a center: polar and equatorial radii and flattening. Each frame shown while this is on adds to the flattening history.");
        if(!enabled){ImGui::PopID();RayFEM::UI::EndSection();return;}
        const bool supported=context && context->mesh && context->mesh->SpaceDimension()==3;
        if(context && !supported)ImGui::TextWrapped("Choose a volume or a surface embedded in 3D.");
        AxisControls();
        if(shape) Results();
        else if(!message.empty()) ImGui::TextWrapped("%s",message.c_str());
        else ImGui::TextDisabled("Measuring…");
        if(ImGui::TreeNode("Sampling")) {
            ImGui::PushItemWidth(-1);
            RayFEM::UI::FieldLabel("Colatitudes");dirty|=ImGui::SliderInt("##colatitudes",&options.colatitudes,19,721,"%d",ImGuiSliderFlags_AlwaysClamp);
            Tip("Rays per meridian from pole to pole, both poles included. 181 is one per degree.");
            RayFEM::UI::FieldLabel("Meridians");dirty|=ImGui::SliderInt("##meridians",&options.meridians,1,72,"%d",ImGuiSliderFlags_AlwaysClamp);
            Tip("Meridians evenly spaced in azimuth. The profile is their mean; more of them sample a non-axisymmetric surface and the volume radius better.");
            RayFEM::UI::FieldLabel("Equator rays");dirty|=ImGui::SliderInt("##equator",&options.equator,8,1440,"%d",ImGuiSliderFlags_AlwaysClamp);
            Tip("Rays around the equator, for the equatorial radius and its ellipticity.");
            ImGui::PopItemWidth();
            ImGui::TreePop();
        }
        if(shape && ImGui::TreeNode("Export")) {
            ImGui::TextDisabled("CSV destination");ImGui::SetNextItemWidth(-1);ImGui::InputText("##path",path,sizeof(path));
            if(ImGui::Button("Save profile CSV",{-1,0}))Export();
            Tip("The radius at each colatitude (mean, min and max over the meridians) and around the equator, with the measured radii in the header.");
            ImGui::BeginDisabled(history.empty());
            if(ImGui::Button("Save history CSV",{-1,0}))ExportHistory();
            ImGui::EndDisabled();
            Tip("Frame, time, radii and flattening of each frame measured.");
            ImGui::TreePop();
        }
        if(shape && !message.empty())ImGui::TextWrapped("%s",message.c_str());
        ImGui::PopID();
        RayFEM::UI::EndSection();
    }

    // The center, the axis between the poles, the measured meridian and the equator.
    void DrawGuide(const Camera3D& camera) const {
        if(!enabled || !shape)return;
        const auto& palette=RayFEM::UI::Palette();
        const Color guide=palette.profile_guide;
        auto lift=[&](Vector3 p){return Vector3Add(p,Vector3Scale(Vector3Normalize(Vector3Subtract(camera.position,p)),scale*.0004f));};
        DrawSphere(center,scale*.004f,guide);
        const Vector3 unit=Vector3Normalize(axis);
        if(std::isfinite(shape->north) && std::isfinite(shape->south))
            DrawLine3D(Vector3Add(center,Vector3Scale(unit,static_cast<float>(shape->north))),
                       Vector3Subtract(center,Vector3Scale(unit,static_cast<float>(shape->south))),guide);
        for(const auto* line:{&meridianPoints,&equatorPoints}) {
            Vector3 previous{};bool previousValid=false;
            for(const auto& point:*line) {
                if(!std::isfinite(point.x)){previousValid=false;continue;}
                const Vector3 p=lift(point);
                if(previousValid)DrawLine3D(previous,p,guide);
                previous=p;previousValid=true;
            }
            if(line==&equatorPoints && previousValid && !line->empty() && std::isfinite(line->front().x))DrawLine3D(previous,lift(line->front()),guide);
        }
    }

    void DrawPlot(ImVec2 size) {
        ImGui::BeginChild("Shape plot",size,ImGuiChildFlags_None,ImGuiWindowFlags_NoScrollbar);
        if(!shape) {
            ImGui::Spacing();ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"No surface shape measured");
            ImGui::TextWrapped("Turn on Surface shape in Analyze.");
            ImGui::EndChild();return;
        }
        ImGui::SetNextItemWidth(170);
        ImGui::Combo("##shape_plot",&plotKind,"Radius\0Radius / equatorial\0Flattening over time\0");
        char details[160];
        std::snprintf(details,sizeof(details),"  ·  frame %llu  ·  t %.5g  ·  flattening %.6g",
                      static_cast<unsigned long long>(metadata.frame),metadata.time,shape->flattening);
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details).x);
        ImGui::TextDisabled("%s",details);
        RayFEM::UI::PlotOptions plot;
        if(plotKind==2) {
            historyPoints.clear();
            for(const auto& [frame,entry]:history)historyPoints.push_back({entry.time,entry.flattening,std::isfinite(entry.flattening)});
            std::sort(historyPoints.begin(),historyPoints.end(),[](const auto& a,const auto& b){return a.x<b.x;});
            const RayFEM::UI::PlotSeriesView series{"Flattening",historyPoints,RayFEM::UI::PlotColor(0)};
            plot.legend=false;plot.x_label="Time";plot.y_label="Flattening";plot.cursor_x=metadata.time;
            RayFEM::UI::PlotLines("##shape_history",std::span(&series,1),ImGui::GetContentRegionAvail(),plot);
        } else {
            const double divide=plotKind==1 && shape->equatorial>0?shape->equatorial:1;
            meanPoints.clear();minPoints.clear();maxPoints.clear();
            for(const auto& p:shape->profile) {
                const double degrees=p.colatitude*180/std::numbers::pi;
                meanPoints.push_back({degrees,p.mean/divide,p.hits>0});
                minPoints.push_back({degrees,p.min/divide,p.hits>0});
                maxPoints.push_back({degrees,p.max/divide,p.hits>0});
            }
            std::vector<RayFEM::UI::PlotSeriesView> series{{"Mean radius",meanPoints,RayFEM::UI::PlotColor(0)}};
            if(options.meridians>1) {
                series.push_back({"Smallest",minPoints,RayFEM::UI::WithAlpha(RayFEM::UI::PlotColor(1),200)});
                series.push_back({"Largest",maxPoints,RayFEM::UI::WithAlpha(RayFEM::UI::PlotColor(2),200)});
            }
            plot.legend=series.size()>1;plot.x_label="Colatitude (degrees)";
            plot.y_label=plotKind==1?"Radius / equatorial radius":"Radius";
            RayFEM::UI::PlotLines("##shape_profile",series,ImGui::GetContentRegionAvail(),plot);
        }
        ImGui::EndChild();
    }

private:
    struct Job {
        std::shared_ptr<RayFEM::RenderFrame> eval;
        mfem::GridFunction* inlineField=nullptr;
        std::string field,error;
        RayFEM::RenderOptions settings;
        RayFEM::SurfaceShapeOptions options;
        std::vector<RayFEM::SurfaceTraceSample> samples;
        RayFEM::SurfaceShape shape;
        RayFEM::ProfileMetadata metadata;
        std::uint64_t generation=0;
        double milliseconds=0;
    };
    struct Shared {std::mutex mutex;std::vector<Job> done;};
    struct HistoryEntry {double time=0,equatorial=0,polar=0,flattening=0;};

    static void Tip(const char* text) {
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
            ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*25);ImGui::TextUnformatted(text);ImGui::PopTextWrapPos();ImGui::EndTooltip();
        }
    }
    static RayFEM::ProfilePosition Position(Vector3 p){return {p.x,p.y,p.z};}
    std::string Flattening() const {
        char text[32];std::snprintf(text,sizeof(text),"f %.4g",shape->flattening);return text;
    }

    // Axis: 0 the model's axis, 1-3 world X, Y, Z (3 is the default without
    // one), 4 custom. Center: 0 automatic, 1 custom.
    void UpdateFrame() {
        Vector3 next=axis;
        if(axisSource==0 && modelAxis)next=modelAxis->first;
        else if(axisSource>=1 && axisSource<=3)next={axisSource==1?1.f:0.f,axisSource==2?1.f:0.f,axisSource==3?1.f:0.f};
        else if(axisSource==0)next={0,0,1};
        Vector3 nextCenter=center;
        if(centerSource==0) {
            nextCenter=meshCenter;
            // On the model's axis line, nearest the mesh center.
            if(axisSource==0 && modelAxis) {
                const Vector3 d=Vector3Normalize(modelAxis->first),o=modelAxis->second;
                nextCenter=Vector3Add(o,Vector3Scale(d,Vector3DotProduct(Vector3Subtract(meshCenter,o),d)));
            }
        }
        if(!Vector3Equals(next,axis) || !Vector3Equals(nextCenter,center)){axis=next;center=nextCenter;dirty=true;}
    }

    void AxisControls() {
        RayFEM::UI::FieldLabel("Axis");
        std::vector<std::string> names{modelAxis?"Model: "+modelAxisName:"Model axis (none reported)","World X","World Y","World Z","Custom"};
        if(ImGui::BeginCombo("##shape_axis",names[axisSource].c_str())) {
            for(int i=0;i<static_cast<int>(names.size());++i) {
                ImGui::BeginDisabled(i==0 && !modelAxis);
                if(ImGui::Selectable(names[i].c_str(),i==axisSource)){axisSource=i;UpdateFrame();dirty=true;}
                ImGui::EndDisabled();
            }
            ImGui::EndCombo();
        }
        Tip("The polar axis. A rotating model's client can report its rotation axis; the poles are where the axis meets the surface.");
        ImGui::PushItemWidth(-1);
        if(axisSource==4){ImGui::TextDisabled("Direction");dirty|=ImGui::DragFloat3("##shape_direction",&axis.x,.01f,0,0,"%.4g");}
        int custom=centerSource;
        RayFEM::UI::FieldLabel("Center");
        if(RayFEM::UI::Segmented("##shape_center",&custom,{"Automatic","Custom"})){centerSource=custom;UpdateFrame();dirty=true;}
        Tip("Rays start here. Automatic is the mesh's center, moved onto the model's axis when measuring about it.");
        if(centerSource==1)dirty|=ImGui::DragFloat3("##shape_center_at",&center.x,scale*.002f,0,0,"%.5g");
        ImGui::PopItemWidth();
    }

    void Results() {
        const auto& s=*shape;
        if(ImGui::BeginTable("##shape_results",2,ImGuiTableFlags_SizingStretchProp)) {
            auto row=[](const char* name,double value,const char* tip) {
                ImGui::TableNextRow();ImGui::TableSetColumnIndex(0);ImGui::TextDisabled("%s",name);
                if(tip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))ImGui::SetTooltip("%s",tip);
                ImGui::TableSetColumnIndex(1);
                if(std::isfinite(value))ImGui::Text("%.7g",value);else ImGui::TextDisabled("—");
            };
            row("Equatorial radius",s.equatorial,"Mean radius around the equator.");
            row("Polar radius",s.polar,"Mean of the north and south pole radii.");
            row("Flattening",s.flattening,"1 − polar / equatorial radius.");
            row("Axis ratio",s.axis_ratio,"Polar / equatorial radius.");
            row("Volume radius",s.volume_radius,"The radius of a sphere with the same volume, from the meridians.");
            if(std::abs(s.north-s.south)>1e-9*std::max(1.0,s.polar)) {
                row("North pole",s.north,nullptr);row("South pole",s.south,nullptr);
            }
            if(s.equatorial_ellipticity>1e-9) row("Equator ellipticity",s.equatorial_ellipticity,"1 − shortest / longest equatorial radius: zero for an axisymmetric surface.");
            ImGui::EndTable();
        }
        ImGui::TextDisabled("%zu / %zu hits  ·  %.1f ms",s.hits,s.rays,milliseconds);
        if(ImGui::IsItemHovered())ImGui::SetTooltip("Largest ray residual: %.3g\nRays meet the true curved faces; misses leave gaps.",s.max_residual);
        if(ImGui::Button(ICON_FA_COPY "  Copy",{ImGui::GetContentRegionAvail().x*.5f-4,0})) {
            char text[512];
            std::snprintf(text,sizeof(text),"frame\t%llu\ntime\t%.17g\nequatorial_radius\t%.17g\npolar_radius\t%.17g\nflattening\t%.17g\naxis_ratio\t%.17g\nvolume_radius\t%.17g\n",
                          static_cast<unsigned long long>(metadata.frame),metadata.time,s.equatorial,s.polar,s.flattening,s.axis_ratio,s.volume_radius);
            ImGui::SetClipboardText(text);
        }
        Tip("Copy the measurements as tab-separated lines.");
        ImGui::SameLine();
        ImGui::BeginDisabled(history.empty());
        char clear[48];std::snprintf(clear,sizeof(clear),"Clear history (%zu)",history.size());
        if(ImGui::Button(clear,{-1,0}))history.clear();
        ImGui::EndDisabled();
        Tip("The flattening of each frame measured while this is on, plotted over time in the Shape plot.");
    }

    void Measure() {
        if(!context || !fieldPointer || !context->mesh || context->mesh->SpaceDimension()!=3)return;
        lastRequest=Clock::now();
        try {
            Job job;
            job.options=options;job.options.center=Position(center);job.options.axis=Position(axis);
            const Vector3 unit=Vector3Normalize(axis);
            job.options.reference=std::abs(unit.x)<.95f?RayFEM::ProfilePosition{1,0,0}:RayFEM::ProfilePosition{0,1,0};
            job.eval=worker?context->compute:context;
            job.inlineField=worker?nullptr:fieldPointer;
            job.field=fieldName;job.settings=settings;job.settings.refinement=3;  // seeds only; hits are on the curved faces
            job.generation=++generation;
            job.metadata={fieldName,context->metadata.frame_index,context->metadata.sim_time,settings.view,settings.component,"curved",
                          settings.excluded_materials,settings.excluded_elements};
            if(!job.eval)throw std::runtime_error("Frame has no compute copy");
            dirty=false;
            if(!worker){Install(Run(std::move(job)));return;}
            inFlight=true;
            worker->Submit("shape",[job=std::move(job),state=shared]() mutable {
                auto result=Run(std::move(job));
                std::lock_guard lock(state->mutex);
                state->done.push_back(std::move(result));
            });
        } catch(const std::exception& error){message=error.what();dirty=false;}
    }
    static Job Run(Job job) {
        const auto start=Clock::now();
        try {
            mfem::GridFunction* field=job.inlineField;
            auto settings=job.settings;
            if(!field) {
                for(const auto& item:job.eval->fields)if(item.name==job.field){field=item.gf.get();break;}
                if(!field)throw std::runtime_error("The field is not in this frame");
                settings.displacement=settings.displacement&&job.eval->displacement?job.eval->displacement.get():nullptr;
            }
            const RayFEM::SurfaceTraceRegistry registry;
            job.shape=RayFEM::SampleSurfaceShape(registry,*job.eval->mesh,*field,job.options,settings,&job.samples);
        } catch(const std::exception& e){job.error=e.what();}
        job.milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        job.eval.reset();
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
        if(!job.error.empty()){message=job.error;return;}
        message.clear();
        shape=std::move(job.shape);metadata=std::move(job.metadata);milliseconds=job.milliseconds;
        history[metadata.frame]={metadata.time,shape->equatorial,shape->polar,shape->flattening};
        // The first meridian and the equator, for the guide.
        meridianPoints.clear();equatorPoints.clear();
        const std::size_t meridianRays=static_cast<std::size_t>(job.options.meridians)*job.options.colatitudes;
        auto point=[](const RayFEM::SurfaceTraceSample& s){
            return s.valid?Vector3{float(s.position[0]),float(s.position[1]),float(s.position[2])}:Vector3{NAN,NAN,NAN};
        };
        for(int c=0;c<job.options.colatitudes;++c)meridianPoints.push_back(point(job.samples[c]));
        for(std::size_t e=meridianRays;e<job.samples.size();++e)equatorPoints.push_back(point(job.samples[e]));
        std::snprintf(path,sizeof(path),"rayfem-exports/shape-frame-%llu.csv",static_cast<unsigned long long>(metadata.frame));
    }
    void Write(const std::string& destinationPath,const std::string& text) {
        try {
            const auto destination=std::filesystem::absolute(destinationPath);
            if(destination.has_parent_path())std::filesystem::create_directories(destination.parent_path());
            std::ofstream output(destination);output.exceptions(std::ios::badbit|std::ios::failbit);output<<text;output.close();
            message="Saved "+destination.string();
        } catch(const std::exception& error){message="Export failed: "+std::string(error.what());}
    }
    void Export() {
        std::ostringstream csv;
        try{RayFEM::ExportSurfaceShapeCSV(csv,*shape,metadata);}catch(const std::exception& e){message=e.what();return;}
        Write(path,csv.str());
    }
    void ExportHistory() {
        std::ostringstream csv;csv.precision(17);
        csv<<"frame,time,equatorial_radius,polar_radius,flattening\n";
        for(const auto& [frame,e]:history)csv<<frame<<','<<e.time<<','<<e.equatorial<<','<<e.polar<<','<<e.flattening<<'\n';
        std::string destination=path;
        const auto slash=destination.find_last_of('/');
        destination=(slash==std::string::npos?std::string():destination.substr(0,slash+1))+"shape-history.csv";
        Write(destination,csv.str());
    }

    std::shared_ptr<RayFEM::RenderFrame> context;
    std::weak_ptr<mfem::Mesh> lastMesh;
    mfem::GridFunction* fieldPointer=nullptr;
    std::string fieldName,message,modelAxisName;
    RayFEM::RenderOptions settings;
    RayFEM::SurfaceShapeOptions options;
    std::optional<RayFEM::SurfaceShape> shape;
    RayFEM::ProfileMetadata metadata;
    std::map<std::uint64_t,HistoryEntry> history;
    std::optional<std::pair<Vector3,Vector3>> modelAxis;
    bool modelAxisSeen=false,enabled=false,dirty=true,inFlight=false,plotRequest=false;
    int axisSource=3,centerSource=0,plotKind=0;
    Vector3 center{},meshCenter{},axis{0,0,1};
    float scale=1;
    double milliseconds=0;
    std::vector<Vector3> meridianPoints,equatorPoints;
    std::vector<RayFEM::UI::PlotPoint> meanPoints,minPoints,maxPoints,historyPoints;
    RayFEM::UI::FramePipeline* worker=nullptr;
    std::shared_ptr<Shared> shared=std::make_shared<Shared>();
    std::uint64_t generation=0,installed=0;
    Clock::time_point lastRequest{};
    char path[512]="rayfem-exports/shape.csv";
};
