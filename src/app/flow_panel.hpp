#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/vector/flow.hpp"
#include "imgui.h"
#include "theme.hpp"
#include "raymath.h"
#include "rlgl.h"
#include "streamline_renderer.hpp"
#include "frame_pipeline.hpp"
#include "widgets.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

// CPU sampling snapshots. With a worker (SetWorker), sampling runs there on each
// frame's compute twin and the newest snapshot is shown when it arrives;
// without one it runs inline on the caller's thread. The cache holds weak
// frame/mesh references, so old flow previews never retain frame history.
// Draw must run inside BeginMode3D, with the usual depth test/write state.
class FlowPanel {
    using Clock=std::chrono::steady_clock;
    struct MagnitudeRange {
        std::weak_ptr<RayFEM::RenderFrame> frame;
        std::string field,error;
        std::vector<int> excluded;
        std::vector<int> excludedElements;
        RayFEM::FieldRange range;
    };
    struct Snapshot {
        std::weak_ptr<RayFEM::RenderFrame> frame;
        std::weak_ptr<mfem::Mesh> mesh;
        std::string field,key,message;
        std::uint32_t index=0;
        RayFEM::VectorCloud arrows;
        RayFEM::StreamlineResult paths;
        std::vector<std::vector<double>> distances;
        std::vector<int> excludedMaterials,excludedElements;
        std::vector<bool> retainedElements;
        std::shared_ptr<MagnitudeRange> magnitudeRange;
        double milliseconds=0;
        bool limited=false;
    };
    struct PathDrawing {
        Model model{};
        ~PathDrawing(){UnloadModel(model);}
    };
    // Everything one sampling run needs, captured by value for the worker.
    struct BuildInput {
        std::weak_ptr<RayFEM::RenderFrame> viewer;
        std::weak_ptr<mfem::Mesh> viewerMesh;
        std::shared_ptr<RayFEM::RenderFrame> eval;  // compute twin, or the viewer frame inline
        mfem::GridFunction* inlineField=nullptr;    // inline only: the caller's field
        std::string field,key;
        RayFEM::RenderOptions settings;
        RayFEM::FlowOptions flow;
        std::vector<bool> retained;
        int mode=0,arrowCount=0,pathSeeds=0;
        bool detailed=false;
        std::uint32_t index=0;
    };
    struct Outcome {std::shared_ptr<Snapshot> snapshot;std::string error,key;std::weak_ptr<RayFEM::RenderFrame> frame;};
    // Shared with running jobs, so a reset panel cannot leave them dangling.
    struct Shared {
        std::mutex mutex;
        std::vector<Outcome> outcomes;         // worker -> UI
        std::deque<std::shared_ptr<MagnitudeRange>> ranges;  // used by whichever thread samples
    };
public:
    // Run sampling on this worker from now on (UI thread, before drawing).
    void SetWorker(RayFEM::UI::FramePipeline* pipeline) {worker=pipeline;}
    bool Enabled() const {return mode!=0;}
    void Open() {if(!mode)mode=1;}
    void SetMode(int value) {mode=std::clamp(value,0,3);}
    void RequestUpdate() {forceUpdate=true;}
    // Release before closing the graphics context; archive resets also release
    // the shared owner through ordinary assignment while the context is alive.
    void ClearGraphics(){pathDrawing.reset();uploadedSnapshot.reset();pathGraphicsDirty=true;}

    void UpdateContext(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                       mfem::GridFunction* field,const std::string& name,
                       const RayFEM::RenderOptions& options) {
        const bool selectionChanged=!includedSelection || !context || !frame || context->mesh!=frame->mesh ||
            context->mesh_revision!=frame->mesh_revision || settings.excluded_materials!=options.excluded_materials ||
            settings.excluded_elements!=options.excluded_elements;
        if(selectionChanged) {
            pathGraphicsDirty=true;
            includedSelection.reset();
            if(frame && frame->mesh)try {includedSelection.emplace(*frame->mesh,options);}
            catch(const std::exception& error) {message=error.what();}
        }
        context=frame;fieldPointer=field;fieldName=name;settings=options;
    }
    void SetBounds(BoundingBox bounds,const std::shared_ptr<mfem::Mesh>& mesh) {
        if(lastMesh.lock()==mesh)return;
        lastMesh=mesh;
        scale=std::max(.001f,Vector3Length(Vector3Subtract(bounds.max,bounds.min)));
        pathLength=scale*1.5f;stepLength=scale*.02f;arrowLength=scale*.035f;
    }

    void Controls() {
        constexpr const char* modeNames[]={nullptr,"Arrows","Streamlines","Both"};
        if(!RayFEM::UI::BeginSection("Vector flow",modeNames[std::clamp(mode,0,3)]))return;
        ControlsBody();
        RayFEM::UI::EndSection();
    }
private:
    void ControlsBody() {
        ImGui::PushID("Vector flow controls");
        RayFEM::UI::Segmented("##mode",&mode,{"Off","Arrows","Lines","Both"});
        Tip("Show directions (arrows), streamlines, or both throughout the vector field.");
        if(!Enabled()){ImGui::PopID();return;}
        if(!Supported())ImGui::TextWrapped("Choose a field with two or three vector components.");
        ImGui::PushItemWidth(-1);
        if(mode==1 || mode==3) {
            RayFEM::UI::FieldLabel("Arrow count");ImGui::SliderInt("##arrow_count",&arrowCount,16,2048,"%d",ImGuiSliderFlags_AlwaysClamp);
        }
        if(mode>=2) {
            RayFEM::UI::FieldLabel("Streamline count");ImGui::SliderInt("##path_count",&pathSeeds,1,512,"%d",ImGuiSliderFlags_AlwaysClamp);
            Tip("Requested total across the selected domain. Zero vectors, clipping and the preview budget can produce fewer lines. Use Update flow for a denser result.");
        }
        ImGui::PopItemWidth();
        RayFEM::UI::Switch("Through surfaces",&throughSurface);
        Tip("Keeps interior arrows and streamlines visible through the mesh. Turn off to use normal surface occlusion.");
        RayFEM::UI::Switch("Automatic updates",&automatic);
        Tip("Updates at most four times per second, with a short compute budget. Pause on a frame and choose Update flow for more detail.");
        ImGui::BeginDisabled(!Supported());
        if(ImGui::Button("Update flow",{-1,0}))forceUpdate=true;
        ImGui::EndDisabled();
        if(current) {
            if(mode==1 || mode==3)ImGui::TextDisabled("%zu / %d arrows",current->arrows.points.size(),arrowCount);
            if(mode>=2)ImGui::TextDisabled("%zu / %d streamlines",current->paths.lines.size(),pathSeeds);
            if(colorMapping)ImGui::TextDisabled("Magnitude %.4g – %.4g",colorMapping->minimum,colorMapping->maximum);
            ImGui::TextDisabled("Frame %u  ·  %.1f ms",current->index,current->milliseconds);
            if(!currentRequest)ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),automatic?"Waiting to refresh":"Previous sample");
            else if(current->limited)ImGui::TextWrapped("Budget-limited preview. Update flow for more detail.");
        }
        if(ImGui::TreeNode("More settings")) {
            ImGui::PushItemWidth(-1);
            if(mode==1 || mode==3) {
                RayFEM::UI::FieldLabel("Arrow length");ImGui::DragFloat("##arrow_length",&arrowLength,scale*.001f,scale*.001f,scale*.3f,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                Tip("Arrows use a common length to reveal direction. Color indicates vector magnitude.");
            }
            if(mode>=2) {
                RayFEM::UI::FieldLabel("Maximum path length");ImGui::DragFloat("##path_length",&pathLength,scale*.01f,scale*.02f,scale*20,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                RayFEM::UI::FieldLabel("Step length");ImGui::DragFloat("##step_length",&stepLength,scale*.0005f,scale*.001f,scale*.2f,"%.4g",ImGuiSliderFlags_AlwaysClamp);
                ImGui::TextDisabled("Integration");RayFEM::UI::Segmented("##integrator",&integrator,{"RK2","RK4"});
                RayFEM::UI::Switch("Both directions",&bidirectional);
                RayFEM::UI::Switch("Moving highlight",&tracers);
                Tip("A bright pulse travels along each steady streamline to show direction. It does not represent a particle trajectory across simulation frames.");
                if(tracers) {
                    float percent=pulseLength*100;
                    RayFEM::UI::FieldLabel("Pulse length");
                    if(ImGui::SliderFloat("##pulse_length",&percent,3,40,"%.0f%%",ImGuiSliderFlags_AlwaysClamp))pulseLength=percent/100;
                    RayFEM::UI::Switch("Thicken highlight",&thickenPulse);
                }
            }
            ImGui::PopItemWidth();
            ImGui::TreePop();
        }
        if(!message.empty())ImGui::TextWrapped("%s",message.c_str());
        if(!colorError.empty())ImGui::TextWrapped("Flow colors unavailable: %s",colorError.c_str());
        ImGui::PopID();
    }
public:

    // Selection participates in the sampling cache key. It is also applied to
    // drawing, so a throttled preview cannot reintroduce newly hidden regions.
    void Draw(const Camera3D& camera,double time,RayFEM::UI::StreamlineRenderer& renderer,Vector2 viewport,
              const std::vector<bool>* retainedElements=nullptr,
              const RayFEM::ClipPlane* clipPlane=nullptr) {
        if(!Enabled())return;
        const std::vector<bool> empty;
        const auto& selection=retainedElements?*retainedElements:empty;
        if(retained!=selection){retained=selection;pathGraphicsDirty=true;}
        if(bool(plane)!=bool(clipPlane) || (plane && clipPlane &&
           (Vector3DistanceSqr(plane->point,clipPlane->point)>0 || Vector3DistanceSqr(plane->normal,clipPlane->normal)>0)))
            pathGraphicsDirty=true;
        plane=clipPlane?std::optional<RayFEM::ClipPlane>(*clipPlane):std::nullopt;
        if(!Supported()){currentRequest=false;return;}
        const auto key=Key();
        currentRequest=current && current->frame.lock()==context && current->key==key;
        if(!currentRequest && !forceUpdate) {
            for(const auto& item:cache)if(item->frame.lock()==context && item->key==key){current=item;currentRequest=true;message=current->message;break;}
        }
        Collect();
        currentRequest=current && current->frame.lock()==context && current->key==key;
        const auto now=Clock::now();
        const bool failedHere=failedFrame.lock()==context && failedKey==key;
        if(forceUpdate || (automatic && !currentRequest && !inFlight && !failedHere && now-lastAttempt>=std::chrono::milliseconds(250)))Build(key,forceUpdate);
        if(!current || current->mesh.lock()!=context->mesh || current->field!=fieldName)return;
        // Endpoint filtering alone is insufficient: an old segment can cross
        // a newly hidden thin cell while both endpoints lie in retained cells.
        // Resample before showing paths from a different visibility selection.
        if(current->excludedMaterials!=settings.excluded_materials || current->excludedElements!=settings.excluded_elements ||
           current->retainedElements!=retained) {
            if(!automatic)message="Visibility changed. Update flow to refresh.";
            return;
        }
        UpdateColors();
        if(mode>=2)UpdatePathDrawing(renderer);
        if(throughSurface) {
            rlDrawRenderBatchActive();rlDisableDepthTest();rlDisableDepthMask();
        }
        if(mode==1 || mode==3)for(const auto& point:current->arrows.points)DrawArrow(point,camera);
        if(mode>=2 && pathDrawing) {
            RayFEM::UI::StreamlineStyle style;
            style.animate=tracers;style.pulse_length=pulseLength;
            style.pulse_width=thickenPulse?3.0f:style.line_width;
            style.pulse_color=RayFEM::UI::Palette().flow_tracer;
            renderer.Draw(pathDrawing->model,camera,viewport,time,style);
        }
        if(throughSurface) {
            rlDrawRenderBatchActive();rlEnableDepthMask();rlEnableDepthTest();
        }
    }

private:
    static void Tip(const char* text) {
        if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) {
            ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*25);ImGui::TextUnformatted(text);ImGui::PopTextWrapPos();ImGui::EndTooltip();
        }
    }
    bool Supported() const {return context && context->mesh && fieldPointer && fieldPointer->VectorDim()>=2 && fieldPointer->VectorDim()<=3;}
    static Vector3 Vector(const RayFEM::ProfilePosition& point) {return {float(point[0]),float(point[1]),float(point[2])};}
    static bool Finite(Vector3 point) {return std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z);}
    static double Distance(const RayFEM::ProfilePosition& a,const RayFEM::ProfilePosition& b) {return std::hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2]);}
    bool Included(int element) const {
        return includedSelection && includedSelection->Includes(element) &&
               (retained.empty() || (element<int(retained.size()) && retained[element]));
    }
    double Side(Vector3 point) const {
        if(!plane)return -1;
        return double(point.x-plane->point.x)*plane->normal.x+double(point.y-plane->point.y)*plane->normal.y+double(point.z-plane->point.z)*plane->normal.z;
    }
    bool Clip(Vector3& a,Vector3& b,float* firstFraction=nullptr,float* lastFraction=nullptr) const {
        if(firstFraction)*firstFraction=0;
        if(lastFraction)*lastFraction=1;
        if(!Finite(a)||!Finite(b))return false;
        if(!plane)return true;
        const double first=Side(a),second=Side(b);
        if(first>0 && second>0)return false;
        if((first>0)!=(second>0)) {
            const float t=static_cast<float>(first/(first-second));
            const Vector3 hit=Vector3Lerp(a,b,t);
            if(first>0){a=hit;if(firstFraction)*firstFraction=t;}
            else {b=hit;if(lastFraction)*lastFraction=t;}
        }
        return true;
    }
    Color Tint(double magnitude,unsigned char alpha) const {
        // A failed full-volume range has no meaningful color legend. Keep the
        // geometry usable without inventing a range from sparse visible arrows.
        auto color=colorMapping ? RayFEM::FieldColor(magnitude,*colorMapping,settings.color_map,settings.reverse_colors)
                                : Color{145,151,164,255};
        color.a=alpha;
        return color;
    }
    void DrawArrow(const RayFEM::FlowPoint& point,const Camera3D& camera) const {
        if(!Included(point.element))return;
        const double magnitude=std::hypot(point.vector[0],point.vector[1],point.vector[2]);
        if(!(magnitude>0)||!std::isfinite(magnitude))return;
        const Vector3 direction={float(point.vector[0]/magnitude),float(point.vector[1]/magnitude),float(point.vector[2]/magnitude)};
        Vector3 a=Vector(point.position),b=Vector3Add(a,Vector3Scale(direction,arrowLength));
        if(!Clip(a,b))return;
        const Color color=Tint(point.magnitude,235);
        DrawLine3D(a,b,color);
        const float length=std::min(arrowLength,Vector3Distance(a,b));
        Vector3 side=Vector3CrossProduct(direction,Vector3Subtract(camera.position,b));
        if(Vector3LengthSqr(side)<1e-12f)side=Vector3CrossProduct(direction,std::abs(direction.y)<.8f?Vector3{0,1,0}:Vector3{1,0,0});
        side=Vector3Scale(Vector3Normalize(side),length*.18f);
        const Vector3 base=Vector3Subtract(b,Vector3Scale(direction,length*.28f));
        for(float sign:{-1.0f,1.0f}) {
            Vector3 tip=b,wing=Vector3Add(base,Vector3Scale(side,sign));
            if(Clip(tip,wing))DrawLine3D(tip,wing,color);
        }
    }
    void UpdatePathDrawing(RayFEM::UI::StreamlineRenderer& renderer) {
        std::ostringstream appearance;
        appearance<<static_cast<int>(settings.color_map)<<','<<settings.reverse_colors<<','<<std::setprecision(17);
        if(colorMapping)appearance<<static_cast<int>(colorMapping->scale)<<','<<colorMapping->minimum<<','
            <<colorMapping->maximum<<','<<colorMapping->log_threshold;
        const auto key=appearance.str();
        if(!pathGraphicsDirty && uploadedSnapshot.lock()==current && pathAppearance==key)return;
        std::vector<RayFEM::UI::StreamlineSegment> segments;
        std::size_t count=0;
        for(const auto& line:current->paths.lines)if(line.points.size()>1)count+=line.points.size()-1;
        segments.reserve(count);
        for(std::size_t i=0;i<current->paths.lines.size();++i) {
            const auto& points=current->paths.lines[i].points;
            const auto& distances=current->distances[i];
            if(points.size()<2 || !(distances.back()>0))continue;
            const double total=distances.back();
            const float phase=static_cast<float>(std::fmod(i*.38196601125,1.0));
            for(std::size_t j=1;j<points.size();++j) {
                if(!Included(points[j-1].element)||!Included(points[j].element))continue;
                Vector3 a=Vector(points[j-1].position),b=Vector(points[j].position);
                float first=0,last=1;
                if(!Clip(a,b,&first,&last) || Vector3DistanceSqr(a,b)<1e-20f)continue;
                const auto arc=[&](float t){return float(std::lerp(distances[j-1],distances[j],t)/total);};
                const auto color=[&](float t){return Tint(std::lerp(points[j-1].magnitude,points[j].magnitude,t),220);};
                segments.push_back({a,b,arc(first),arc(last),phase,color(first),color(last)});
            }
        }
        // Upload first, then replace the previous drawing. Animation updates
        // only shader uniforms; geometry and field colors stay on the GPU.
        auto drawing=std::make_shared<PathDrawing>();drawing->model=renderer.Upload(segments);
        pathDrawing=std::move(drawing);uploadedSnapshot=current;pathAppearance=key;pathGraphicsDirty=false;
    }
    std::string Key() const {
        std::ostringstream key;key<<std::hexfloat<<std::quoted(fieldName)<<'|'<<mode<<'|'<<arrowCount<<'|'<<pathSeeds<<'|'<<pathLength<<'|'<<stepLength<<'|'<<integrator<<'|'<<bidirectional
            <<'|'<<settings.displacement<<'|'<<settings.displacement_scale;
        for(int material:settings.excluded_materials)key<<'|'<<material;
        key<<";elements=";
        for(int element:settings.excluded_elements)key<<element<<',';
        key<<";plane=";
        if(plane)key<<plane->point.x<<','<<plane->point.y<<','<<plane->point.z<<','<<plane->normal.x<<','<<plane->normal.y<<','<<plane->normal.z;
        std::uint64_t hash=1469598103934665603ULL;
        for(bool value:retained){hash^=value?1:0;hash*=1099511628211ULL;}
        key<<";mask="<<retained.size()<<','<<hash;
        return key.str();
    }
    static std::shared_ptr<MagnitudeRange> RangeFor(std::deque<std::shared_ptr<MagnitudeRange>>& ranges,
                                                  const RayFEM::RenderOptions& settings,
                                                  const std::shared_ptr<RayFEM::RenderFrame>& frame,
                                                  mfem::GridFunction& field,const std::string& name) {
        // Colors always describe every included parent element. Sampling and
        // camera controls, clipping, deformation, and palettes do not enter
        // this cache key and cannot trigger another finite-element evaluation.
        std::erase_if(ranges,[](const auto& item){return item->frame.expired();});
        auto found=std::find_if(ranges.begin(),ranges.end(),[&](const auto& item) {
            return item->frame.lock()==frame && item->field==name && item->excluded==settings.excluded_materials &&
                   item->excludedElements==settings.excluded_elements;
        });
        if(found!=ranges.end()) {
            auto result=*found;
            std::rotate(found,found+1,ranges.end());
            return result;
        }
        auto result=std::make_shared<MagnitudeRange>();
        result->frame=frame;result->field=name;result->excluded=settings.excluded_materials;
        result->excludedElements=settings.excluded_elements;
        try {
            RayFEM::RenderOptions options;options.view=RayFEM::FieldView::Magnitude;
            options.excluded_materials=result->excluded;
            options.excluded_elements=result->excludedElements;
            result->range=RayFEM::EvaluateFieldRange(*frame->mesh,field,options,{.distribution_samples=1000,.reduce_to_fit=true});
        } catch(const std::exception& error) {result->error=error.what();}
        ranges.push_back(result);while(ranges.size()>64)ranges.pop_front();
        return result;
    }
    void UpdateColors() {
        if(!current->magnitudeRange || current->magnitudeRange->excluded!=settings.excluded_materials ||
           current->magnitudeRange->excludedElements!=settings.excluded_elements) {
            // While flow sampling is throttled, color the retained preview
            // against its own frame and the current material selection.
            const auto frame=current->frame.lock();
            mfem::GridFunction* field=nullptr;
            if(frame)for(const auto& item:frame->fields)if(item.name==current->field){field=item.gf.get();break;}
            if(!field) {
                colorMapping.reset();colorSource.reset();
                colorError="Update flow to refresh this frame.";
                return;
            }
            // The worker computes ranges with each snapshot; MFEM is not used here.
            if(worker)return;
            current->magnitudeRange=RangeFor(shared->ranges,settings,frame,*field,current->field);
        }
        const auto& source=current->magnitudeRange;
        const auto& options=settings.color_scale;
        if(colorSource==source && colorSettings.scale==options.scale &&
           colorSettings.log_threshold==options.log_threshold && colorSettings.zscale_contrast==options.zscale_contrast)return;
        colorSource=source;colorSettings=options;colorMapping.reset();colorError=source->error;
        if(!colorError.empty())return;
        try {
            colorMapping=RayFEM::ResolveColorScale(source->range.min_value,source->range.max_value,options,source->range.samples);
        } catch(const std::exception& error) {colorError=error.what();}
    }
    void Build(const std::string& key,bool detailed) {
        forceUpdate=false;lastAttempt=Clock::now();
        BuildInput input;
        input.viewer=context;input.viewerMesh=context->mesh;
        input.eval=worker?context->compute:context;
        if(!worker)input.inlineField=fieldPointer;
        input.field=fieldName;input.key=key;input.settings=settings;
        input.retained=retained;input.mode=mode;input.arrowCount=arrowCount;input.pathSeeds=pathSeeds;
        input.detailed=detailed;input.index=context->metadata.frame_index;
        auto& flow=input.flow;flow.clip_plane=plane;flow.retained_elements=retained;
        flow.max_points=detailed?128000:12000;flow.max_steps=detailed?320:160;flow.step_size=stepLength;flow.max_length=pathLength;
        flow.bidirectional=bidirectional;flow.integrator=integrator==0?RayFEM::FlowIntegrator::RK2:RayFEM::FlowIntegrator::RK4;
        flow.max_evaluations=detailed?600000:36000;flow.max_location_tests=detailed?1500000:120000;
        if(!input.eval) {message="Frame has no compute copy";failedKey=key;failedFrame=context;return;}
        if(!worker) {shared->outcomes.push_back(Run(std::move(input),shared->ranges));Collect();return;}
        inFlight=true;
        worker->Submit("flow",[input=std::move(input),state=shared]() mutable {
            auto outcome=Run(std::move(input),state->ranges);
            std::lock_guard lock(state->mutex);
            state->outcomes.push_back(std::move(outcome));
        });
    }
    // Install finished runs. The newest one becomes current, as with inline sampling.
    void Collect() {
        std::vector<Outcome> outcomes;
        {std::lock_guard lock(shared->mutex);outcomes.swap(shared->outcomes);}
        for(auto& outcome:outcomes) {
            inFlight=false;
            if(!outcome.snapshot) {
                message=outcome.error;failedKey=outcome.key;failedFrame=outcome.frame;currentRequest=false;
                continue;
            }
            auto result=std::move(outcome.snapshot);
            std::erase_if(cache,[&](const auto& item){return item->frame.lock()==result->frame.lock() && item->key==result->key;});
            cache.push_back(result);while(cache.size()>2)cache.pop_front();
            current=std::move(result);message=current->message;failedKey.clear();failedFrame.reset();
        }
    }
    // Sampling itself. Touches only input.eval and the given range cache.
    static Outcome Run(BuildInput input,std::deque<std::shared_ptr<MagnitudeRange>>& ranges) {
        Outcome outcome;outcome.key=input.key;outcome.frame=input.viewer;
        const auto start=Clock::now();
        const auto duration=std::chrono::milliseconds(input.detailed?800:24);
        const auto deadline=start+duration;
        try {
            auto& frame=*input.eval;
            // Inline runs use the caller's field and displacement unchanged; the
            // worker resolves both on the compute twin.
            mfem::GridFunction* field=input.inlineField;
            auto settings=input.settings;
            if(!field) {
                for(const auto& item:frame.fields)if(item.name==input.field){field=item.gf.get();break;}
                if(!field)throw std::runtime_error("The flow field is not in this frame");
                settings.displacement=settings.displacement&&frame.displacement?frame.displacement.get():nullptr;
            }
            auto result=std::make_shared<Snapshot>();result->frame=input.viewer;result->mesh=input.viewerMesh;
            result->excludedMaterials=settings.excluded_materials;result->excludedElements=settings.excluded_elements;
            result->retainedElements=input.retained;
            result->field=input.field;result->key=input.key;result->index=input.index;
            const bool pathsSupported=frame.mesh->Dimension()==frame.mesh->SpaceDimension();
            auto& flow=input.flow;
            if(input.mode==1 || input.mode==3) {
                const auto arrowDeadline=input.mode==3&&pathsSupported?start+duration/3:deadline;
                flow.cancelled=[arrowDeadline]{return Clock::now()>=arrowDeadline;};
                flow.max_seeds=flow.seed_count=input.arrowCount;
                result->arrows=RayFEM::SampleVectorField(*frame.mesh,*field,settings,flow);
            }
            if(input.mode>=2 && pathsSupported) {
                flow.cancelled=[deadline]{return Clock::now()>=deadline;};flow.max_seeds=flow.seed_count=input.pathSeeds;
                result->paths=RayFEM::BuildStreamlines(*frame.mesh,*field,settings,flow);
            }
            const bool sampledZero=!result->arrows.points.empty() && std::none_of(result->arrows.points.begin(),result->arrows.points.end(),[](const auto& point){return point.magnitude>0;});
            std::erase_if(result->arrows.points,[](const auto& point){return !(point.magnitude>0);});
            result->limited=result->arrows.diagnostics.cancelled||result->arrows.diagnostics.budget_exhausted||result->paths.diagnostics.cancelled||result->paths.diagnostics.budget_exhausted;
            for(const auto& line:result->paths.lines) {
                auto& distances=result->distances.emplace_back(line.points.size(),0);
                for(std::size_t j=0;j<line.points.size();++j) {
                    if(j)distances[j]=distances[j-1]+Distance(line.points[j-1].position,line.points[j].position);
                }
            }
            result->magnitudeRange=RangeFor(ranges,settings,input.eval,*field,input.field);
            result->milliseconds=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
            if(input.mode>=2 && !pathsSupported) {
                result->message=input.mode==3?"Showing arrows. Streamlines on surface meshes are not available.":"Streamlines on surface meshes are not available. Choose Arrows to inspect this field.";
            } else if(result->arrows.points.empty()&&result->paths.lines.empty()) {
                if(result->limited)result->message="Preview stopped before finding flow samples. Update flow for more detail.";
                else if(sampledZero)result->message="The vector field is zero at the sampled positions.";
                else result->message="No flow samples in the selected region.";
            }
            outcome.snapshot=std::move(result);
        } catch(const std::exception& error) {outcome.error=error.what();}
        return outcome;
    }

    std::shared_ptr<RayFEM::RenderFrame> context;
    std::weak_ptr<mfem::Mesh> lastMesh;
    mfem::GridFunction* fieldPointer=nullptr;
    RayFEM::RenderOptions settings;
    std::optional<RayFEM::ElementSelection> includedSelection;
    std::vector<bool> retained;
    std::optional<RayFEM::ClipPlane> plane;
    std::deque<std::shared_ptr<Snapshot>> cache;
    std::shared_ptr<Shared> shared=std::make_shared<Shared>();
    RayFEM::UI::FramePipeline* worker=nullptr;
    bool inFlight=false;
    std::shared_ptr<MagnitudeRange> colorSource;
    std::optional<RayFEM::ColorScaleMapping> colorMapping;
    RayFEM::ColorScaleOptions colorSettings;
    std::shared_ptr<Snapshot> current;
    std::weak_ptr<RayFEM::RenderFrame> failedFrame;
    std::string fieldName,message,failedKey,colorError;
    Clock::time_point lastAttempt{};
    std::shared_ptr<PathDrawing> pathDrawing;
    std::weak_ptr<Snapshot> uploadedSnapshot;
    std::string pathAppearance;
    bool pathGraphicsDirty=true;
    int mode=0,arrowCount=512,pathSeeds=64,integrator=1;
    float pulseLength=.12f;
    bool thickenPulse=true;
    float scale=1,pathLength=1.5f,stepLength=.02f,arrowLength=.035f;
    bool automatic=true,throughSurface=true,tracers=true,bidirectional=true,forceUpdate=false,currentRequest=false;
};
