#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/profile/surface_trace.hpp"
#include "frame_pipeline.hpp"
#include "plot.hpp"
#include "ray_placement.hpp"
#include "surface_path.hpp"
#include "widgets.hpp"
#include "raymath.h"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <numbers>
#include <sstream>
#include <stdexcept>

// Values along the outer surface. Path mode connects points placed on outer
// faces; each segment follows the surface in the plane through its points and
// the center, the short or the long way round. Meridian and latitude trace a
// circle around an axis. With a worker (SetWorker), sampling runs there on the
// frame's compute twin and updates live; otherwise it runs inline.
class SurfaceTracePanel {
    using Clock=std::chrono::steady_clock;
public:
    using Placement=RayFEM::UI::RayPlacement;
    struct PlacementRequest { Placement::Kind kind; int index; };

    bool Enabled() const {return enabled;}
    // Path mode starts by placing points when there is no path yet.
    void Open() {SetEnabled(true);if(mode==0 && points.empty())placementRequest=PlacementRequest{Placement::Kind::Path,-1};}
    void SetEnabled(bool value) {enabled=value;}
    void SetWorker(RayFEM::UI::FramePipeline* pipeline) {worker=pipeline;}
    std::optional<PlacementRequest> TakePlacementRequest() {
        auto request=placementRequest;placementRequest.reset();
        // A new path replaces this one point by point; keep it for Cancel.
        if(request && request->kind==Placement::Kind::Path && request->index<0){backupPoints=points;backupLongWay=longWay;}
        return request;
    }
    const std::vector<RayFEM::UI::RayAnchor>& PathPoints() const {return points;}

    // A finished placement: the path points, one edited point, or the center.
    void ApplyPlacement(const Placement::Result& result) {
        revealSection=true;
        if(result.cancelled) {
            if(result.kind!=Placement::Kind::Path)return;
            points=backupPoints;longWay=backupLongWay;
        } else if(result.kind==Placement::Kind::Center) {
            if(!result.anchors.empty()){centerAnchor=result.anchors.front();centerSource=2;}
        } else {
            points=result.anchors;
            std::erase_if(points,[](const auto& p){return !p.Valid();});
            longWay.resize(points.size(),false);
        }
        anchorMesh=context?context->mesh:nullptr;
        mode=0;enabled=true;
        // Too few points for a path: drop the previous trace and any job
        // still sampling it.
        if(points.size()<2) {
            samples.clear();anglePoints.clear();distancePoints.clear();controlAngles.clear();controlDistances.clear();
            installed=++generation;
        }
        UpdatePositions();dirty=true;lastRequest={};
    }

    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                       mfem::GridFunction* field,const std::string& name,
                       const RayFEM::RenderOptions& options) {
        const bool changed=context!=frame || fieldPointer!=field || fieldName!=name ||
           settings.view!=options.view || settings.component!=options.component ||
           settings.displacement!=options.displacement || settings.displacement_scale!=options.displacement_scale ||
           settings.excluded_materials!=options.excluded_materials ||
           settings.excluded_elements!=options.excluded_elements;
        if(changed)dirty=true;
        context=frame;fieldPointer=field;fieldName=name;settings=options;
        if(changed)UpdatePositions();
        Collect();
        Tick();
    }

    void SetBounds(BoundingBox bounds,const std::shared_ptr<mfem::Mesh>& mesh) {
        if(lastMesh.lock()==mesh)return;
        lastMesh=mesh;
        meshCenter=Vector3Scale(Vector3Add(bounds.min,bounds.max),.5f);
        scale=std::max(.01f,Vector3Length(Vector3Subtract(bounds.max,bounds.min)));
        if(centerSource==0)center=meshCenter;
        UpdatePositions();
        dirty=true;
    }

    void Controls(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                  mfem::GridFunction* field,const std::string& name,
                  const RayFEM::RenderOptions& options) {
        UpdateContext(frame,field,name,options);
        if(revealSection) {
            // Show the placed points and segment directions after placement.
            ImGui::PushID("Surface trace");ImGui::GetStateStorage()->SetBool(ImGui::GetID("open"),true);ImGui::PopID();
            revealSection=false;
        }
        if(!RayFEM::UI::BeginSection("Surface trace",enabled?(mode==0?"Path":"On"):nullptr,false))return;
        ControlsBody();
        RayFEM::UI::EndSection();
    }
private:
    void ControlsBody() {
        ImGui::PushID("Surface trace controls");
        if(RayFEM::UI::Switch("Enabled",&enabled) && enabled)Open();
        Tip("Measure the field along the outer surface.");
        if(!enabled) {ImGui::PopID();return;}
        int selection=mode==0?0:kind+1;
        if(RayFEM::UI::Segmented("##trace_kind",&selection,{"Path","Meridian","Latitude"})) {
            mode=selection==0?0:1;
            if(mode==1){kind=selection-1;beginAngle=0;endAngle=kind==0?180:360;fixedAngle=kind==0?0:90;}
            dirty=true;lastRequest={};
            if(mode==0 && points.empty())placementRequest=PlacementRequest{Placement::Kind::Path,-1};
        }
        const bool supported=context && context->mesh && context->mesh->SpaceDimension()==3;
        if(mode==0)PathControls(); else AxisControls();
        if(context && !supported)ImGui::TextWrapped("Choose a volume or a surface embedded in 3D.");
        RayFEM::UI::Switch("Live update",&liveUpdate);
        Tip("Resample automatically when the trace or the frame changes. Turn off to sample on demand.");
        if(!liveUpdate) {
            ImGui::BeginDisabled(!supported || !fieldPointer);
            if(ImGui::Button(samples.empty()?"Sample surface":"Update surface trace",{-1,0}))Sample(false);
            ImGui::EndDisabled();
        }
        if(!samples.empty()) {
            if(dirty)ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),liveUpdate?"Updating":"Update to refresh");
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().success),"Current sample");
            ImGui::TextDisabled("%zu / %zu hits  ·  %.1f ms",valid,samples.size(),sampleMs);
            if(ImGui::IsItemHovered())ImGui::SetTooltip("Largest ray residual: %.6g\nGaps mark rays with no exterior hit. Only original exterior faces are traced; material exclusions are respected.",maxResidual);
        }
        if(ImGui::TreeNode("Trace settings")) {
            ImGui::PushItemWidth(-1);
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
            ImGui::TreePop();
        }
        if(!samples.empty() && ImGui::TreeNode("Export")) {
            ImGui::TextDisabled("CSV destination");ImGui::SetNextItemWidth(-1);ImGui::InputText("##path",path,sizeof(path));
            ImGui::BeginDisabled(dirty);
            if(ImGui::Button("Save CSV",{-1,0}))Export();
            Tip("Includes the path parameter in radians, distance, physical position, and face details.");
            ImGui::EndDisabled();
            if(dirty)ImGui::TextWrapped("Update the trace before exporting.");
            ImGui::TreePop();
        }
        if(!message.empty())ImGui::TextWrapped("%s",message.c_str());
        ImGui::PopID();
    }

    void PathControls() {
        if(ImGui::Button(ICON_FA_CROSSHAIRS "  Place path points",{-1,0}))placementRequest=PlacementRequest{Placement::Kind::Path,-1};
        Tip("Choose elements on the surface and place points on their outer faces, in order. Points move with the mesh.");
        previewSegment=-1;
        for(int i=0;i<int(points.size());++i) {
            ImGui::PushID(i);
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().guide_text),"%d",i+1);
            ImGui::SameLine(28);
            ImGui::TextDisabled("element %d",points[i].element);
            const float buttons=ImGui::CalcTextSize("Edit").x+ImGui::GetFrameHeight()+ImGui::GetStyle().FramePadding.x*4+ImGui::GetStyle().ItemSpacing.x;
            ImGui::SameLine(ImGui::GetCursorPosX()+std::max(0.0f,ImGui::GetContentRegionAvail().x-buttons));
            if(ImGui::SmallButton("Edit"))placementRequest=PlacementRequest{Placement::Kind::Path,i};
            ImGui::SameLine();
            if(ImGui::SmallButton(ICON_FA_XMARK)) {
                points.erase(points.begin()+i);
                if(i<int(longWay.size()))longWay.erase(longWay.begin()+i);
                UpdatePositions();dirty=true;lastRequest={};
                ImGui::PopID();break;
            }
            if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))ImGui::SetTooltip("Remove this point");
            ImGui::PopID();
        }
        if(!points.empty() && ImGui::Button(ICON_FA_PLUS "  Add point",{-1,0}))
            placementRequest=PlacementRequest{Placement::Kind::Path,int(points.size())};
        if(points.size()>=2) {
            if(RayFEM::UI::Switch("Closed loop",&closed)){dirty=true;lastRequest={};UpdatePositions();}
            Tip("Also follow the surface from the last point back to the first.");
            const int segments=Segments();
            if(int(longWay.size())<segments)longWay.resize(segments,false);
            ImGui::TextDisabled("Direction of each segment");
            for(int s=0;s<segments;++s) {
                ImGui::PushID(1000+s);
                ImGui::AlignTextToFramePadding();
                ImGui::Text("%d " ICON_FA_ARROW_RIGHT " %d",s+1,(s+1)%int(points.size())+1);
                ImGui::SameLine(84);
                int way=longWay[s]?1:0;
                if(RayFEM::UI::Segmented("##way",&way,{"Short way","Long way"})){longWay[s]=way==1;dirty=true;lastRequest={};UpdatePositions();}
                if(ImGui::IsMouseHoveringRect(ImGui::GetItemRectMin(),ImGui::GetItemRectMax()))previewSegment=s;
                if(std::find(ambiguous.begin(),ambiguous.end(),s)!=ambiguous.end())
                    ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Points are opposite through the center; the plane is ambiguous. Move the center.");
                ImGui::PopID();
            }
        }
        // The center sets each segment's plane.
        RayFEM::UI::FieldLabel("Center");
        if(centerSource==2)ImGui::TextDisabled("element %d",centerAnchor.element);
        else ImGui::TextDisabled("%s",centerSource==0?"mesh center":"custom");
        const float half=(ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)*.5f;
        if(ImGui::Button("Mesh center",{half,0})){centerSource=0;center=meshCenter;UpdatePositions();dirty=true;lastRequest={};}
        Tip("Use the center of the mesh bounds.");
        ImGui::SameLine();
        if(ImGui::Button("Element center",{half,0}))placementRequest=PlacementRequest{Placement::Kind::Center,0};
        Tip("Click an element: its center becomes the path center and moves with the mesh.");
        ImGui::SetNextItemWidth(-1);
        if(ImGui::DragFloat3("##path_center",&center.x,scale*.005f,0,0,"%.4g")){centerSource=1;dirty=true;lastRequest={};UpdatePositions();}
        Tip("Each segment lies in the plane through its two points and this center. A center inside the object gives the clearest paths.");
    }

    void AxisControls() {
        RayFEM::UI::FieldLabel(kind==0?"Azimuth (degrees)":"Colatitude (degrees)");
        dirty|=ImGui::SliderFloat("##fixed_angle",&fixedAngle,0,kind==0?360.0f:180.0f,"%.1f");
        Tip("Colatitude is measured from the axis: 0 at the north pole, 90 at the equator. Azimuth starts at the projected world X direction (world Y when X is nearly parallel to the axis).");
        ImGui::PushItemWidth(-1);
        ImGui::TextDisabled("Center");dirty|=ImGui::DragFloat3("##center",&center.x,scale*.005f,0,0,"%.4g");
        Tip("Rays start here and use the nearest forward exterior hit. A center inside the object usually gives the clearest trace.");
        ImGui::TextDisabled("Axis");dirty|=ImGui::DragFloat3("##axis",&axis.x,.01f,0,0,"%.4g");
        RayFEM::UI::FieldLabel("Start angle (degrees)");dirty|=ImGui::DragFloat("##begin",&beginAngle,.5f,0,kind==0?180.0f:360.0f,"%.1f",ImGuiSliderFlags_AlwaysClamp);
        RayFEM::UI::FieldLabel("End angle (degrees)");dirty|=ImGui::DragFloat("##end",&endAngle,.5f,0,kind==0?180.0f:360.0f,"%.1f",ImGuiSliderFlags_AlwaysClamp);
        ImGui::PopItemWidth();
    }
public:

    void DrawGuide(const Camera3D& camera) const {
        if(!enabled || !context || !context->mesh || context->mesh->SpaceDimension()!=3)return;
        const auto& palette=RayFEM::UI::Palette();
        if(mode==1 && Finite(center) && Finite(axis) && Vector3Length(axis)>0) {
            const Vector3 tip=Vector3Add(center,Vector3Scale(Vector3Normalize(axis),scale*.3f));
            DrawLine3D(center,tip,RayFEM::UI::ThemeAlpha(palette.profile_guide,180));
        }
        if(Finite(center))DrawSphere(center,scale*.004f,palette.profile_guide);
        // Path mode draws its curve in the overlay, where it stays visible.
        if(samples.empty() || sampleMode!=mode || mode==0)return;
        Vector3 previous{};bool previousValid=false;
        for(const auto& sample:samples) {
            Vector3 point=Vector(sample.position);
            if(!sample.valid || !Finite(point)){previousValid=false;continue;}
            // Lift only the guide toward the camera to avoid surface z-fighting.
            point=Vector3Add(point,Vector3Scale(Vector3Normalize(Vector3Subtract(camera.position,point)),scale*.0003f));
            if(previousValid)DrawLine3D(previous,point,palette.profile_guide);
            previous=point;previousValid=true;
        }
    }

    // Path points, the other way for a hovered segment, and the plot cursor.
    void DrawOverlay(ImVec2 origin,ImVec2 size,const Camera3D& camera) const {
        if(!enabled || mode!=0)return;
        const auto& palette=RayFEM::UI::Palette();
        auto* draw=ImGui::GetWindowDrawList();
        draw->PushClipRect(origin,{origin.x+size.x,origin.y+size.y},true);
        const Matrix viewProjection=MatrixMultiply(MatrixLookAt(camera.position,camera.target,camera.up),
            MatrixPerspective(camera.fovy*DEG2RAD,double(size.x)/double(size.y),rlGetCullDistanceNear(),rlGetCullDistanceFar()));
        auto project=[&](Vector3 p,ImVec2& out) {
            const Quaternion q=QuaternionTransform({p.x,p.y,p.z,1},viewProjection);
            if(!(q.w>1e-6f))return false;
            out={origin.x+(q.x/q.w+1)*.5f*size.x,origin.y+(1-q.y/q.w)*.5f*size.y};
            return true;
        };
        // The sampled path: solid where it faces the camera, faint where it
        // wraps behind the mesh (facing is judged from the path center).
        auto facing=[&](Vector3 point){return Vector3DotProduct(Vector3Subtract(point,center),Vector3Subtract(camera.position,point))>=0;};
        if(sampleMode==0) {
            ImVec2 previous{};bool previousValid=false,previousFront=false;
            for(const auto& sample:samples) {
                const Vector3 point=Vector(sample.position);ImVec2 screen;
                if(!sample.valid || !Finite(point) || !project(point,screen)){previousValid=false;continue;}
                const bool front=facing(point);
                if(previousValid) {
                    if(front && previousFront)draw->AddLine(previous,screen,palette.guide,2.5f);
                    else draw->AddLine(previous,screen,RayFEM::UI::WithAlpha(palette.guide,70),1.5f);
                }
                previous=screen;previousValid=true;previousFront=front;
            }
        }
        if(previewSegment>=0 && previewSegment<int(previewArcs.size())) {
            const auto& arc=previewArcs[previewSegment];
            for(std::size_t k=1;k<arc.size();k+=2) {
                ImVec2 a,b;
                if(project(arc[k-1],a) && project(arc[k],b))draw->AddLine(a,b,palette.guide,2);
            }
        }
        for(std::size_t i=0;i<positions.size();++i) {
            ImVec2 screen;
            if(!project(positions[i],screen))continue;
            // Points behind the mesh are dimmed so they do not read as
            // sitting on the facing surface.
            const unsigned char alpha=facing(positions[i])?255:110;
            draw->AddCircleFilled(screen,8,RayFEM::UI::WithAlpha(palette.guide,alpha));
            draw->AddCircle(screen,10,RayFEM::UI::WithAlpha(palette.handle_outline,alpha),0,2);
            const std::string label=std::to_string(i+1);
            const ImVec2 text=ImGui::CalcTextSize(label.c_str());
            draw->AddText({screen.x-text.x*.5f,screen.y-text.y*.5f},RayFEM::UI::WithAlpha(palette.handle_outline,alpha),label.c_str());
        }
        if(std::isfinite(plotHover) && sampleMode==0) {
            const RayFEM::SurfaceTraceSample* nearest=nullptr;
            double best=std::numeric_limits<double>::infinity();
            for(const auto& sample:samples)if(sample.valid && std::isfinite(sample.distance) && std::abs(sample.distance-plotHover)<best)
                {best=std::abs(sample.distance-plotHover);nearest=&sample;}
            ImVec2 marker;
            if(nearest && project(Vector(nearest->position),marker)) {
                draw->AddCircleFilled(marker,5,palette.selection);draw->AddCircle(marker,8,palette.guide,0,2);
            }
        }
        draw->PopClipRect();
    }

    void DrawPlot(ImVec2 size) const {
        ImGui::BeginChild("Surface trace plot",size,ImGuiChildFlags_None,ImGuiWindowFlags_NoScrollbar);
        if(samples.empty()) {
            ImGui::Spacing();ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"No surface trace sampled");
            ImGui::TextWrapped(mode==0?"Place path points on the surface in Analyze.":"Choose a meridian or latitude in Analyze.");
            ImGui::EndChild();return;
        }
        ImGui::SetNextItemWidth(120);ImGui::Combo("##surface_axis",&plotAxis,"Arc length\0Angle\0");
        Tip("Arc length is the sum of chords between neighboring valid hits; gaps are never bridged. Increase the sample count for a closer arc-length approximation.");
        char stamp[96];
        std::snprintf(stamp,sizeof(stamp),"  ·  frame %llu  ·  t %.5g",
                      static_cast<unsigned long long>(metadata.frame),metadata.time);
        const std::string details=metadata.field_name+stamp;
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details.c_str()).x);
        ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().muted));
        ImGui::TextWrapped("%s",details.c_str());ImGui::PopStyleColor();
        if(dirty) {
            const char* state=liveUpdate?"Updating":"Previous sample";
            RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(state).x);
            if(liveUpdate)ImGui::TextDisabled("%s",state);
            else ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"%s",state);
        }
        const bool angle=plotAxis==1;
        const auto& plotted=angle?anglePoints:distancePoints;
        const RayFEM::UI::PlotSeriesView series{metadata.field_name.c_str(),plotted,RayFEM::UI::PlotColor(1)};
        RayFEM::UI::PlotOptions options;options.legend=false;options.y_label="Value";
        options.x_label=angle?(sampleMode==0?"Path angle (degrees)":(sampleKind==0?"Colatitude (degrees)":"Azimuth (degrees)")):"Arc length";
        if(sampleMode==0)options.markers=angle?std::span<const double>(controlAngles):std::span<const double>(controlDistances);
        double hovered=std::numeric_limits<double>::quiet_NaN();
        if(!angle)options.hovered_x=&hovered;
        RayFEM::UI::PlotLines("##surface_values",std::span(&series,1),ImGui::GetContentRegionAvail(),options);
        plotHover=hovered;
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
    int Segments() const {return points.size()<2?0:int(closed?points.size():points.size()-1);}

    // Re-evaluate anchored points (and an element center) on the current frame.
    void UpdatePositions() {
        positions.clear();previewArcs.clear();
        if(!context || !context->mesh)return;
        if((!points.empty() || centerSource==2) && anchorMesh.lock()!=context->mesh) {
            if(!points.empty())message="The mesh changed; place the path again.";
            points.clear();longWay.clear();if(centerSource==2)centerSource=0;
            return;
        }
        try {
            if(centerSource==2)if(auto p=RayFEM::UI::AnchorPosition(*context->mesh,centerAnchor,settings))center=*p;
            for(const auto& point:points)if(auto p=RayFEM::UI::AnchorPosition(*context->mesh,point,settings))positions.push_back(*p);
        } catch(const std::exception& e){message=e.what();return;}
        // The other way round for each segment, approximated at the radii of
        // its ends, previewed while hovering that segment's direction.
        for(int s=0;s<Segments() && positions.size()==points.size();++s) {
            std::vector<Vector3> arc;
            try {
                const Vector3 a=positions[s],b=positions[(s+1)%positions.size()];
                const auto other=RayFEM::UI::MakeSegmentArc(center,a,b,!(s<int(longWay.size())&&longWay[s]),{0,0,1});
                const float ra=Vector3Distance(a,center),rb=Vector3Distance(b,center);
                for(int k=0;k<=48;++k) {
                    const double t=k/48.0;
                    arc.push_back(Vector3Add(center,Vector3Scale(RayFEM::UI::ArcDirection(other,other.sweep*t),ra+(rb-ra)*float(t))));
                }
            } catch(const std::exception&){}
            previewArcs.push_back(std::move(arc));
        }
    }

    struct Job {
        std::shared_ptr<RayFEM::RenderFrame> eval;
        mfem::GridFunction* inlineField=nullptr;
        std::string field,algorithm,error;
        RayFEM::RenderOptions settings;
        RayFEM::SurfaceTraceRegistry registry;
        std::vector<RayFEM::SurfaceTraceRay> rays;
        std::vector<RayFEM::SurfaceTraceSample> samples;
        std::vector<double> controls;
        std::vector<int> ambiguous;
        RayFEM::ProfileMetadata metadata;
        int mode=0,kind=0;
        bool preview=false;
        std::uint64_t generation=0;
        std::uint32_t frameIndex=0;
        double milliseconds=0;
    };
    struct Shared {std::mutex mutex;std::vector<Job> done;};

    // Live updates: resample after changes and on new frames.
    void Tick() {
        if(!enabled || !liveUpdate || !dirty || inFlight || !context || !fieldPointer)return;
        if(mode==0 && points.size()<2)return;
        if(Clock::now()-lastRequest<std::chrono::milliseconds(150))return;
        Sample(false);
    }

    void Sample(bool preview) {
        if(!context || !fieldPointer || !context->mesh || context->mesh->SpaceDimension()!=3)return;
        lastRequest=Clock::now();
        try {
            Job job;
            if(mode==0) {
                if(positions.size()<2)throw std::invalid_argument("Place at least two path points.");
                auto path=RayFEM::UI::MakeSurfacePathRays(center,positions,longWay,closed,count);
                job.rays=std::move(path.rays);job.controls=std::move(path.control_parameters);job.ambiguous=std::move(path.ambiguous_segments);
            } else {
                if(!(endAngle>beginAngle))throw std::invalid_argument("The end angle must be larger than the start angle.");
                const Vector3 unit=Vector3Normalize(axis);
                const RayFEM::ProfilePosition reference=std::abs(unit.x)<.95f?RayFEM::ProfilePosition{1,0,0}:RayFEM::ProfilePosition{0,1,0};
                job.rays=kind==0?
                    RayFEM::MakeMeridianRays(Position(center),Position(axis),reference,fixedAngle*radians,beginAngle*radians,endAngle*radians,count):
                    RayFEM::MakeAzimuthRays(Position(center),Position(axis),reference,fixedAngle*radians,beginAngle*radians,endAngle*radians,count);
            }
            job.eval=worker?context->compute:context;
            job.inlineField=worker?nullptr:fieldPointer;
            job.field=fieldName;job.algorithm=algorithm;job.registry=registry;
            job.settings=settings;job.settings.refinement=refinement;
            job.mode=mode;job.kind=kind;job.preview=preview;job.generation=++generation;
            job.metadata={fieldName,context->metadata.frame_index,context->metadata.sim_time,settings.view,settings.component,algorithm,settings.excluded_materials,settings.excluded_elements};
            job.frameIndex=context->metadata.frame_index;
            if(!job.eval)throw std::runtime_error("Frame has no compute copy");
            if(!worker){Install(Run(std::move(job)));return;}
            inFlight=true;
            worker->Submit("surface",[job=std::move(job),state=shared]() mutable {
                auto result=Run(std::move(job));
                std::lock_guard lock(state->mutex);
                state->done.push_back(std::move(result));
            });
        } catch(const std::exception& error){message=error.what();dirty=!liveUpdate;}
    }
    // Evaluation only; touches job.eval (a compute twin on the worker).
    static Job Run(Job job) {
        const auto start=Clock::now();
        try {
            mfem::GridFunction* field=job.inlineField;
            auto settings=job.settings;
            if(!field) {
                for(const auto& item:job.eval->fields)if(item.name==job.field){field=item.gf.get();break;}
                if(!field)throw std::runtime_error("The traced field is not in this frame");
                settings.displacement=settings.displacement&&job.eval->displacement?job.eval->displacement.get():nullptr;
            }
            job.samples=job.registry.sample(job.algorithm,*job.eval->mesh,*field,job.rays,settings);
        } catch(const std::exception& e){job.error=e.what();}
        job.milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        job.rays.clear();job.eval.reset();
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
        samples=std::move(job.samples);sampleMode=job.mode;sampleKind=job.kind;ambiguous=std::move(job.ambiguous);
        metadata=std::move(job.metadata);
        valid=0;maxResidual=0;anglePoints.clear();distancePoints.clear();
        anglePoints.reserve(samples.size());distancePoints.reserve(samples.size());
        for(const auto& sample:samples) {
            const bool finite=sample.valid && std::isfinite(sample.value);
            valid+=finite;
            if(finite && std::isfinite(sample.residual))maxResidual=std::max(maxResidual,sample.residual);
            anglePoints.push_back({sample.parameter/radians,sample.value,finite});
            distancePoints.push_back({sample.distance,sample.value,finite});
        }
        // Control points on the plot: the arc length of the first hit at or
        // after each point's path parameter.
        controlAngles.clear();controlDistances.clear();
        for(double parameter:job.controls) {
            controlAngles.push_back(parameter/radians);
            double distance=std::numeric_limits<double>::quiet_NaN();
            for(const auto& sample:samples)if(sample.parameter>=parameter-1e-9 && std::isfinite(sample.distance)){distance=sample.distance;break;}
            if(!std::isfinite(distance))for(auto it=samples.rbegin();it!=samples.rend();++it)if(std::isfinite(it->distance)){distance=it->distance;break;}
            controlDistances.push_back(distance);
        }
        sampleMs=job.milliseconds;
        if(latest)dirty=job.preview;
        message.clear();
        std::snprintf(path,sizeof(path),"rayfem-exports/surface-%s-frame-%u.csv",
                      sampleMode==0?"path":(sampleKind==0?"meridian":"latitude"),job.frameIndex);
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
    std::weak_ptr<mfem::Mesh> lastMesh,anchorMesh;
    mfem::GridFunction* fieldPointer=nullptr;
    RayFEM::RenderOptions settings;
    RayFEM::ProfileMetadata metadata;
    std::vector<RayFEM::SurfaceTraceSample> samples;
    std::vector<RayFEM::UI::PlotPoint> anglePoints,distancePoints;
    std::vector<double> controlAngles,controlDistances;
    std::string fieldName,algorithm="curved",message;
    bool enabled=false,dirty=true,liveUpdate=true,inFlight=false,closed=false;
    // mode: 0 path, 1 around an axis (kind: 0 meridian, 1 latitude).
    int mode=0,kind=0,sampleMode=0,sampleKind=0,count=256,refinement=4;
    // Center source: 0 mesh center, 1 custom, 2 an element's center.
    int centerSource=0,previewSegment=-1;
    mutable int plotAxis=0;
    mutable double plotHover=std::numeric_limits<double>::quiet_NaN();
    std::vector<RayFEM::UI::RayAnchor> points;
    RayFEM::UI::RayAnchor centerAnchor{};
    std::vector<bool> longWay;
    std::vector<int> ambiguous;
    std::vector<Vector3> positions;
    std::vector<std::vector<Vector3>> previewArcs;
    std::optional<PlacementRequest> placementRequest;
    bool revealSection=false;
    std::vector<RayFEM::UI::RayAnchor> backupPoints;
    std::vector<bool> backupLongWay;
    Vector3 center{},meshCenter{},axis{0,0,1};
    float scale=1,fixedAngle=0,beginAngle=0,endAngle=180;
    size_t valid=0;
    double sampleMs=0,maxResidual=0;
    RayFEM::UI::FramePipeline* worker=nullptr;
    std::shared_ptr<Shared> shared=std::make_shared<Shared>();
    std::uint64_t generation=0,installed=0;
    Clock::time_point lastRequest{};
    char path[512]="rayfem-exports/surface.csv";
};
