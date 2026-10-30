#pragma once

#include "libRayfem/frame/tracking.hpp"
#include "ui/plot.hpp"
#include "ui/widgets.hpp"
#include "imgui.h"
#include <algorithm>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Each trace stores only scalar samples. A weak backfill queue does not extend
// the lifetime of frames evicted by the viewer's history budget.
class TrackingPanel {
    struct Track {
        size_t id;
        std::string label;
        RayFEM::FrameTracker tracker;
        bool visible=true,stopped=false;
        std::string issue;
        uint64_t session=0,mesh=0,revision=0;
        std::deque<std::weak_ptr<RayFEM::RenderFrame>> pending;
        Track(size_t id_,std::string label_,RayFEM::TrackingRequest request)
            :id(id_),label(std::move(label_)),tracker(std::move(request),4096) {}
    };
public:
    bool HasTracks() const {return !tracks.empty();}
    // Track a numeric value the client sends with each frame.
    void AddFrameValue(const std::shared_ptr<RayFEM::RenderFrame>& frame,const std::string& name,
                       const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        RayFEM::TrackingRequest request;
        request.field_name=name;request.kind=RayFEM::MeasurementKind::FrameValue;
        Add(frame,request,history);
    }
    bool TakePlotRequest() {const bool result=plotRequested;plotRequested=false;return result;}

    // Colors of every tracked DOF on this frame, matching DofColor per index.
    std::unordered_map<int,ImU32> DofColors(const RayFEM::RenderFrame& frame,const std::string& field) const {
        std::unordered_map<int,ImU32> colors;
        for(const auto& t:tracks) {
            const auto& r=t.tracker.request();
            if(colors.contains(r.dof) || !t.visible || r.kind!=RayFEM::MeasurementKind::DofCoefficient || r.field_name!=field ||
               t.session!=frame.session_id || t.mesh!=frame.mesh_id || t.revision!=frame.mesh_revision) continue;
            if(const ImU32 color=DofColor(frame,field,r.dof)) colors.emplace(r.dof,color);
        }
        return colors;
    }

    ImU32 DofColor(const RayFEM::RenderFrame& frame,const std::string& field,int dof) const {
        for(const auto& t:tracks) {
            const auto& r=t.tracker.request();
            if(t.visible && r.kind==RayFEM::MeasurementKind::DofCoefficient && r.field_name==field && r.dof==dof &&
               t.session==frame.session_id && t.mesh==frame.mesh_id && t.revision==frame.mesh_revision) {
                // The same stream IDs can carry an incompatible field basis.
                // Color only this frame's successfully validated association.
                for(auto it=t.tracker.samples().rbegin();it!=t.tracker.samples().rend();++it)
                    if(it->session_id==frame.session_id && it->mesh_id==frame.mesh_id &&
                       it->mesh_revision==frame.mesh_revision && it->frame==frame.metadata.frame_index) {
                        if(it->valid) return RayFEM::UI::PlotColor(t.id-1);
                        break;
                    }
            }
        }
        return 0;
    }

    void Observe(const std::shared_ptr<RayFEM::RenderFrame>& frame) {
        for(auto& t:tracks) if(!t.stopped) {
            if(t.pending.size()>=4096) {t.pending.pop_front();++expired;}
            t.pending.push_back(frame);
        }
    }

    void ProcessPending() {
        const auto start=std::chrono::steady_clock::now();
        size_t emptyVisits=0;
        // Advance the cursor before evaluation: even a single expensive
        // integral cannot starve another trace on the following UI frame.
        for(int count=0;!tracks.empty() && count<256;++count) {
            nextTrack%=tracks.size();
            auto& t=tracks[nextTrack];nextTrack=(nextTrack+1)%tracks.size();
            if(t.pending.empty() || t.stopped) {
                if(++emptyVisits>=tracks.size()) break;
                continue;
            }
            emptyVisits=0;
            auto frame=t.pending.front().lock();t.pending.pop_front();
            if(frame) {
                try {t.tracker.append(*frame);}
                catch(const std::exception& e) {t.stopped=true;t.issue=e.what();t.pending.clear();}
            } else ++expired;
            if(std::chrono::steady_clock::now()-start>std::chrono::milliseconds(6)) return;
        }
    }

    void SetExcludedMaterials(const std::vector<int>& exclusions,const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        if(excludedMaterials==exclusions) return;
        excludedMaterials=exclusions;
        for(auto& t:tracks) {
            t.tracker.setExcludedMaterials(exclusions);t.pending.clear();t.stopped=false;t.issue.clear();
            for(const auto& frame:history) t.pending.push_back(frame);
        }
        if(!tracks.empty()) message="Material selection changed; rebuilt from retained frames.";
    }

    void SetExcludedElements(const std::vector<int>& exclusions,
                             const std::shared_ptr<RayFEM::RenderFrame>& frame,
                             const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        const auto scope=frame?std::optional<RayFEM::ElementSelectionScope>{RayFEM::ElementSelectionScope{frame->session_id,frame->mesh_id,frame->mesh_revision}}:std::nullopt;
        if(excludedElements==exclusions && elementScope==scope)return;
        excludedElements=exclusions;elementScope=scope;
        for(auto& t:tracks) {
            t.tracker.setExcludedElements(exclusions,scope);t.pending.clear();t.stopped=false;t.issue.clear();
            for(const auto& previous:history)t.pending.push_back(previous);
        }
        if(!tracks.empty())message="Element visibility changed; rebuilt from retained frames.";
    }

    void AddDof(const std::shared_ptr<RayFEM::RenderFrame>& frame,const std::string& field,
                int index,int component,const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        RayFEM::TrackingRequest request;
        request.field_name=field;request.kind=RayFEM::MeasurementKind::DofCoefficient;
        request.dof=index;request.component=component;request.excluded_materials=excludedMaterials;
        request.excluded_elements=excludedElements;request.element_scope=elementScope;
        Add(frame,request,history);
    }

    void Controls(const std::shared_ptr<RayFEM::RenderFrame>& frame,const std::string& field,
                  const RayFEM::RenderOptions& options,
                  const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        const std::string badge=tracks.empty()?"":std::to_string(tracks.size())+(tracks.size()==1?" track":" tracks");
        if(!RayFEM::UI::BeginSection("Frame tracking",badge.c_str())) return;
        const char* names[]={"DOF coefficient","Minimum","Maximum","Mean","Integral","L2 norm","Point probe"};
        ImGui::SetNextItemWidth(-1);ImGui::Combo("##measurement",&kind,names,7);
        if(kind==0) {
            RayFEM::UI::FieldLabel("Degree of freedom");
            ImGui::InputInt("##dof",&dof);dof=std::max(0,dof);
            int components=1;
            if(frame) for(const auto& f:frame->fields) if(f.name==field) components=f.gf->FESpace()->GetVDim();
            coefficientComponent=std::clamp(coefficientComponent,0,std::max(0,components-1));
            if(components>1) {
                RayFEM::UI::FieldLabel("Coefficient axis");
                ImGui::SliderInt("##coefficient_axis",&coefficientComponent,0,components-1);
            }
            else coefficientComponent=0;
        } else if(kind==6) {
            ImGui::TextDisabled("Probe position");ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##probe",probe,0.01f);
            if(ImGui::IsItemHovered()) ImGui::SetTooltip("A fixed point in displayed space, sampled with the profile locator.");
        }
        ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().muted));
        ImGui::TextWrapped("%s",field.empty()?"No field":field.c_str());ImGui::PopStyleColor();
        if(ImGui::IsItemHovered()) ImGui::SetTooltip(kind==0?"Tracks the global coefficient. ND/RT values are oriented functionals; NURBS values are control coefficients.":"Uses the selected field operation. Mean, integral and L2 norm use quadrature and displayed volume; extrema are quadrature-sampled.");
        ImGui::BeginDisabled(!frame || field.empty() || tracks.size()>=8);
        if(ImGui::Button("Add track",{-1,0})) {
            RayFEM::TrackingRequest request;
            // Intrinsic field support is resolved separately for each frame.
            // Only the user's selection persists across an evolving series.
            request.excluded_materials=excludedMaterials;
            request.excluded_elements=excludedElements;request.element_scope=elementScope;
            request.field_name=field;request.kind=static_cast<RayFEM::MeasurementKind>(kind);
            request.dof=dof;request.component=kind==0?coefficientComponent:options.component;
            request.view=options.view;request.use_displacement=options.displacement!=nullptr;
            request.displacement_scale=options.displacement_scale;
            request.probe={probe[0],probe[1],probe[2]};
            Add(frame,request,history);
        }
        ImGui::EndDisabled();
        for(size_t i=0;i<tracks.size();) {
            auto& t=tracks[i];ImGui::PushID(static_cast<int>(t.id));
            ImGui::Spacing();ImGui::Separator();ImGui::Spacing();
            const auto color=ImGui::ColorConvertU32ToFloat4(RayFEM::UI::PlotColor(t.id-1));
            ImGui::PushStyleColor(ImGuiCol_CheckMark,color);
            ImGui::Checkbox("##visible",&t.visible);ImGui::PopStyleColor();ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextWrapped("%s",t.label.c_str());
            if(ImGui::IsItemHovered()) {
                ImGui::BeginTooltip();ImGui::TextUnformatted(t.label.c_str());ImGui::Text("%zu samples",t.tracker.samples().size());
                if(t.stopped) ImGui::TextWrapped("Stopped: %s",t.issue.c_str());
                if(!t.tracker.samples().empty() && !t.tracker.samples().back().valid)
                    ImGui::TextWrapped("%s",t.tracker.samples().back().error.c_str());
                ImGui::EndTooltip();
            }
            if(t.stopped) ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Stopped — add a new track");
            const float actionWidth=std::max(1.0f,(ImGui::GetContentRegionAvail().x-ImGui::GetStyle().ItemSpacing.x)*.5f);
            ImGui::BeginDisabled(!t.pending.empty());
            if(ImGui::Button("Save CSV",{actionWidth,0})) Export(t);
            ImGui::EndDisabled();
            ImGui::SameLine();const bool remove=ImGui::Button("Remove",{actionWidth,0});
            ImGui::PopID();
            if(remove) tracks.erase(tracks.begin()+i);else ++i;
        }
        if(!message.empty()) ImGui::TextWrapped("%s",message.c_str());
        if(expired) {ImGui::TextDisabled("%zu frames evicted before sampling",expired);}
        RayFEM::UI::EndSection();
    }

    void DrawPlot(ImVec2 size,const std::shared_ptr<RayFEM::RenderFrame>& current) const {
        if(tracks.empty()) {ImGui::TextDisabled("Use Pick DOF, or add a track in Analyze.");return;}
        // Axes: time, frame index, and the coordinates the current frame's
        // schema declares. Without meaningful time the primary coordinate
        // is the default.
        std::vector<std::string> axes{"Time","Frame"};
        if(current && current->schema) for(const auto& v:current->schema->values) if(v.coordinate) axes.push_back(v.name);
        if(axisName.empty() || std::find(axes.begin(),axes.end(),axisName)==axes.end())
            axisName=current && current->schema && !current->schema->has_time?
                (current->schema->primary.empty()?"Frame":current->schema->primary):"Time";
        ImGui::SetNextItemWidth(std::max(115.f,ImGui::CalcTextSize(axisName.c_str()).x+ImGui::GetFrameHeight()*2));
        if(ImGui::BeginCombo("##track_axis",axisName.c_str())) {
            for(const auto& a:axes) if(ImGui::Selectable(a.c_str(),a==axisName)) axisName=a;
            ImGui::EndCombo();
        }
        auto xOf=[&](const RayFEM::MeasurementSample& s)->std::optional<double> {
            if(axisName=="Time") return s.time;
            if(axisName=="Frame") return double(s.frame);
            return s.coordinate(axisName);
        };
        size_t pending=0;for(const auto& t:tracks) pending+=t.pending.size();
        char details[80];
        if(pending)std::snprintf(details,sizeof(details),"%zu samples pending",pending);
        else std::snprintf(details,sizeof(details),"Tracks incoming frames");
        RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(details).x);
        ImGui::TextDisabled("%s",details);
        std::vector<std::vector<RayFEM::UI::PlotPoint>> data;
        std::vector<RayFEM::UI::PlotSeriesView> series;
        data.reserve(tracks.size());series.reserve(tracks.size());
        for(const auto& t:tracks) if(t.visible) {
            auto& points=data.emplace_back();points.reserve(t.tracker.samples().size()+4);
            uint64_t previousSession=0,previousMesh=0,previousRevision=0,previousFrame=0;bool first=true;
            double previousX=0;
            for(const auto& s:t.tracker.samples()) {
                const auto at=xOf(s);
                if(!at) {if(!points.empty())points.push_back({points.back().x,0,false});first=true;continue;}  // a frame without this coordinate breaks the line
                const double x=*at;
                if(!first && (s.session_id!=previousSession || s.mesh_id!=previousMesh || s.mesh_revision!=previousRevision ||
                             static_cast<uint64_t>(s.frame)!=previousFrame+1 || x<previousX))
                    points.push_back({x,0,false});
                points.push_back({x,s.value,s.valid});
                previousSession=s.session_id;previousMesh=s.mesh_id;previousRevision=s.mesh_revision;previousFrame=s.frame;previousX=x;first=false;
            }
            series.push_back({t.label.c_str(),points,RayFEM::UI::PlotColor(t.id-1)});
        }
        RayFEM::UI::PlotOptions options;
        std::string xLabel=axisName=="Time"?"Simulation time":axisName;
        if(current && current->schema) if(const auto* field=current->schema->find(axisName); field && !field->unit.empty()) xLabel+=" ("+field->unit+")";
        options.x_label=xLabel.c_str();options.y_label="Value";
        if(current) {
            if(axisName=="Time") options.cursor_x=current->metadata.sim_time;
            else if(axisName=="Frame") options.cursor_x=current->metadata.frame_index;
            else if(const auto v=current->metadata.number(axisName)) options.cursor_x=*v;
        }
        RayFEM::UI::PlotLines("##time_traces",series,size,options);
    }
private:
    void Add(const std::shared_ptr<RayFEM::RenderFrame>& frame,const RayFEM::TrackingRequest& request,
             const std::deque<std::shared_ptr<RayFEM::RenderFrame>>& history) {
        if(!frame) return;
        if(tracks.size()>=8) {message="Up to 8 tracks. Remove one to add another.";return;}
        try {
            std::string label;
            if(request.kind==RayFEM::MeasurementKind::DofCoefficient)
                label="DOF "+std::to_string(request.dof)+" ["+std::to_string(request.component)+"]";
            else label=RayFEM::MeasurementName(request.kind);
            if(request.kind==RayFEM::MeasurementKind::FrameValue) {
                label=request.field_name;
                if(frame->schema) if(const auto* field=frame->schema->find(request.field_name); field && !field->unit.empty()) label+=" ("+field->unit+")";
            } else label+=" / "+request.field_name;
            Track t(nextId++,std::move(label),request);
            if(request.kind==RayFEM::MeasurementKind::DofCoefficient) t.tracker.bindDof(*frame);
            t.session=frame->session_id;t.mesh=frame->mesh_id;t.revision=frame->mesh_revision;
            for(const auto& f:history) t.pending.push_back(f);
            tracks.push_back(std::move(t));message.clear();plotRequested=true;
        } catch(const std::exception& e) {message=e.what();}
    }
    void Export(const Track& t) {
        try {
            std::filesystem::create_directories("rayfem-exports");
            const auto path="rayfem-exports/track-"+std::to_string(t.id)+".csv";
            std::ofstream file(path);if(!file) throw std::runtime_error("Cannot open "+path);
            RayFEM::ExportTrackingCSV(file,t.tracker);
            if(!file) throw std::runtime_error("Could not finish writing "+path);
            message="Saved "+path;
        } catch(const std::exception& e) {message=e.what();}
    }
    std::vector<Track> tracks;
    std::vector<int> excludedMaterials;
    std::vector<int> excludedElements;
    std::optional<RayFEM::ElementSelectionScope> elementScope;
    size_t nextId=1,expired=0,nextTrack=0;
    int kind=0,dof=0,coefficientComponent=0;
    mutable std::string axisName;
    bool plotRequested=false;
    float probe[3]={0,0,0};
    std::string message;
};
