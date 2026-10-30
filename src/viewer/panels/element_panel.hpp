#pragma once

#include "libRayfem/frame/render.hpp"
#include "libRayfem/mesh/element_statistics.hpp"
#include "ui/theme.hpp"
#include "ui/widgets.hpp"
#include "libRayfem/mesh/converter.hpp"
#include "ui/mouse_modifiers.hpp"
#include "imgui.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <optional>
#include <sstream>
#include <tuple>

class ElementPanel {
public:
    void Context(const std::shared_ptr<RayFEM::RenderFrame>& frame) {
        const auto mesh=frame?frame->mesh:nullptr;
        const Identity next=frame?Identity{frame->session_id,frame->mesh_id,frame->mesh_revision}:Identity{};
        const long sequence=mesh?mesh->GetSequence():0;
        if(bool(mesh)!=hasContext || next!=identity || mesh!=contextMesh.lock() || sequence!=meshSequence) {
            selected.clear();hidden.clear();statistics.reset();statisticsError.clear();
            lastSettings.reset();sampledFrame.reset();statisticsPending=true;
            identity=next;contextMesh=mesh;meshSequence=sequence;hasContext=bool(mesh);
            ++revision;
        }
        elementCount=mesh?mesh->GetNE():0;
    }

    const std::vector<int>& Selected() const {return selected;}
    const std::vector<int>& Hidden() const {return hidden;}
    std::uint64_t Revision() const {return revision;}

    void Select(std::optional<int> element,bool additive) {
        if(element && (*element<0 || *element>=elementCount))return;
        if(!element) {
            if(!additive && !selected.empty()) {selected.clear();Changed();}
            return;
        }
        if(!additive) {
            if(selected.size()!=1 || selected.front()!=*element) {selected.assign(1,*element);Changed();}
            return;
        }
        const auto found=std::lower_bound(selected.begin(),selected.end(),*element);
        if(found!=selected.end() && *found==*element)selected.erase(found);
        else selected.insert(found,*element);
        Changed();
    }

    // Visibility changes retain selection, including hidden selected elements.
    void SetVisible(int element,bool visible) {
        if(element<0 || element>=elementCount)return;
        const auto found=std::lower_bound(hidden.begin(),hidden.end(),element);
        const bool present=found!=hidden.end() && *found==element;
        if(visible && present) {hidden.erase(found);Changed();}
        else if(!visible && !present) {hidden.insert(found,element);Changed();}
    }

    void Controls(const std::shared_ptr<RayFEM::RenderFrame>& frame,mfem::GridFunction* field,
                  const std::string& fieldName,const RayFEM::RenderOptions& options) {
        Context(frame);
        ImGui::PushID("Element explorer");
        if(!frame || !frame->mesh) {
            ImGui::TextDisabled("No mesh loaded");ImGui::PopID();return;
        }
        // This panel owns the viewer's element mask; use its current state,
        // including button changes made after the parent built its settings.
        auto effective=options;
        effective.excluded_elements=hidden;
        Normalize(effective.excluded_materials);
        Settings current{field?fieldName:std::string{},effective.view,effective.component,
            effective.displacement!=nullptr,effective.displacement_scale,
            effective.excluded_materials,effective.excluded_elements};
        if(!lastSettings || *lastSettings!=current) {lastSettings=std::move(current);statisticsPending=true;}
        const std::string selectedBadge=selected.empty()?"":std::to_string(selected.size())+" selected";
        if(RayFEM::UI::BeginSection("Selection",selectedBadge.c_str())) {
            // One row of actions; the header already counts the selection.
            const float third=(ImGui::GetContentRegionAvail().x-2*ImGui::GetStyle().ItemSpacing.x)/3;
            ImGui::BeginDisabled(selected.empty());
            if(ImGui::Button(ICON_FA_EYE_SLASH " Hide",{third,0}))SetSelectedVisible(false);
            Tip("Hide the selected elements. Hidden elements stay selected, and visibility also controls physical analysis.");
            ImGui::SameLine();
            if(ImGui::Button(ICON_FA_EYE " Show",{third,0}))SetSelectedVisible(true);
            ImGui::SameLine();
            if(ImGui::Button(ICON_FA_XMARK " Clear",{third,0}))Select(std::nullopt,false);
            ImGui::EndDisabled();
            if(selected.empty())ImGui::TextDisabled("Click an element to inspect it.");
            else {
                if(statisticsPending)Compute(frame,field,fieldName,effective);
                const bool stale=sampledFrame.lock()!=frame;
                if(stale) {
                    ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning),"Earlier frame");
                    ImGui::SameLine();if(ImGui::SmallButton("Update"))Compute(frame,field,fieldName,effective);
                    Tip("Refresh statistics for the current frame. Streaming does not repeatedly recompute the selection.");
                }
                if(!statisticsError.empty()) {
                    ImGui::TextWrapped("%s",statisticsError.c_str());
                    if(ImGui::Button("Retry statistics",{-1,0}))Compute(frame,field,fieldName,effective);
                } else if(statistics)DrawStatistics();
            }
            RayFEM::UI::EndSection();
        }
        const std::string hiddenBadge=hidden.empty()?"":std::to_string(hidden.size())+" hidden";
        if(!RayFEM::UI::BeginSection("All elements",hiddenBadge.c_str())) {ImGui::PopID();return;}
        ImGui::SetNextItemWidth(std::max(100.0f,ImGui::GetContentRegionAvail().x-86));
        ImGui::Combo("##element_filter",&filter,"All elements\0Selected\0Hidden\0");
        ImGui::SameLine();ImGui::BeginDisabled(hidden.empty());
        if(ImGui::Button("Show all")) {hidden.clear();Changed();}
        ImGui::EndDisabled();
        ImGui::TextDisabled("%d elements  ·  %zu hidden",elementCount,hidden.size());
        Tip("Click a row to select it. Shift-click adds or removes it. Element IDs are zero-based.");
        DrawList(*frame->mesh,effective.excluded_materials);
        RayFEM::UI::EndSection();
        ImGui::PopID();
    }

private:
    using Identity=std::tuple<std::uint64_t,std::uint64_t,std::uint64_t>;
    struct Settings {
        std::string field;
        RayFEM::FieldView view;
        int component;
        bool displaced;
        double scale;
        std::vector<int> materials,elements;
        bool operator==(const Settings&) const = default;
    };
    static void Normalize(std::vector<int>& values) {
        std::sort(values.begin(),values.end());values.erase(std::unique(values.begin(),values.end()),values.end());
    }
    static void Tip(const char* message) {if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal))ImGui::SetTooltip("%s",message);}
    void Changed() {++revision;statisticsPending=true;}
    void SetSelectedVisible(bool visible) {
        std::vector<int> next;
        next.reserve(visible?hidden.size():hidden.size()+selected.size());
        if(visible)std::set_difference(hidden.begin(),hidden.end(),selected.begin(),selected.end(),std::back_inserter(next));
        else std::set_union(hidden.begin(),hidden.end(),selected.begin(),selected.end(),std::back_inserter(next));
        if(next!=hidden) {hidden=std::move(next);Changed();}
    }
    void Compute(const std::shared_ptr<RayFEM::RenderFrame>& frame,mfem::GridFunction* field,
                 const std::string& fieldName,const RayFEM::RenderOptions& options) {
        statistics.reset();statisticsError.clear();statisticsPending=false;
        sampledFrame=frame;sampleFrame=frame->metadata.frame_index;sampleTime=frame->metadata.sim_time;sampleField=fieldName;
        const auto start=std::chrono::steady_clock::now();
        try {statistics=RayFEM::ComputeElementStatistics(*frame->mesh,selected,field,options);}
        catch(const std::exception& error) {statisticsError=error.what();}
        sampleMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    }
    template<class Name>
    static std::string Counts(const std::map<int,std::size_t>& counts,Name&& name) {
        std::ostringstream result;
        for(const auto& [id,count]:counts) {
            if(result.tellp()>0)result<<", ";
            result<<name(id);
            if(counts.size()>1)result<<" ("<<count<<")";
        }
        return result.str();
    }
    static void TextRow(const char* label,const std::string& value) {
        ImGui::TableNextRow();ImGui::TableSetColumnIndex(0);ImGui::TextDisabled("%s",label);
        ImGui::TableSetColumnIndex(1);
        const bool shortened=value.size()>80;
        ImGui::TextWrapped("%s",shortened?(value.substr(0,77)+"...").c_str():value.c_str());
        if(shortened && ImGui::IsItemHovered())ImGui::SetTooltip("%s",value.c_str());
    }
    static void NumberRow(const char* label,double value) {
        char text[64];std::snprintf(text,sizeof(text),"%.7g",value);TextRow(label,text);
    }
    void DrawStatistics() const {
        const auto& result=*statistics;
        ImGui::TextDisabled("Frame %u  ·  t %.5g",sampleFrame,sampleTime);
        if(ImGui::IsItemHovered())ImGui::SetTooltip("%zu quadrature samples · %.1f ms",result.quadrature_samples,sampleMs);
        if(result.excluded_count)ImGui::TextDisabled("%zu selected excluded from analysis",result.excluded_count);
        if(!result.element_count)return;
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding,{6,5});
        const bool statisticsTable=ImGui::BeginTable("##statistics",2,ImGuiTableFlags_SizingStretchProp);
        ImGui::PopStyleVar();
        if(statisticsTable) {
            ImGui::TableSetupColumn("Property",ImGuiTableColumnFlags_WidthFixed,ImGui::CalcTextSize("Material IDs").x+16);
            ImGui::TableSetupColumn("Value",ImGuiTableColumnFlags_WidthStretch);
            NumberRow(result.dimension==3?"Volume":"Area",result.measure);
            const auto& c=result.centroid;char center[128];std::snprintf(center,sizeof(center),"%.4g,  %.4g,  %.4g",c[0],c[1],c[2]);
            TextRow("Centroid",center);
            TextRow("Material IDs",Counts(result.material_counts,[](int id){return std::to_string(id);}));
            TextRow("Geometry",Counts(result.geometry_counts,[](int id){return std::string(mfem::Geometry::Name[id]);}));
            TextRow(result.nurbs?"NURBS order":"Mesh order",Counts(result.geometry_orders,[](int id){return std::to_string(id);}));
            if(result.field) {
                TextRow("Field",sampleField);
                TextRow("Field order",Counts(result.field_orders,[](int id){return std::to_string(id);}));
                NumberRow("Sample min",result.field->minimum);
                Tip("Minimum over quadrature, dense reference-grid and finite-element node samples; an approximation to the true extremum.");
                NumberRow("Sample max",result.field->maximum);
                Tip("Maximum over quadrature, dense reference-grid and finite-element node samples; an approximation to the true extremum.");
                NumberRow("Mean",result.field->mean);NumberRow("Integral",result.field->integral);
            }
            ImGui::EndTable();
        }
        Tip("Physical geometry includes displacement. Explosion does not affect statistics. Extrema and bounds are sampled approximations; mean and integral use physical quadrature weights.");
        const bool showBounds=ImGui::TreeNode("Bounds");
        Tip("Sampled physical bounds, including displacement; curved extrema between samples may extend beyond them.");
        if(showBounds) {
            for(int d=0;d<result.space_dimension;++d)
                ImGui::Text("%c  %.6g to %.6g","XYZ"[d],result.bounds_min[d],result.bounds_max[d]);
            ImGui::TreePop();
        }
    }
    void DrawList(mfem::Mesh& mesh,const std::vector<int>& excludedMaterials) {
        const auto* filtered=filter==1?&selected:filter==2?&hidden:nullptr;
        const int rows=filtered?static_cast<int>(filtered->size()):elementCount;
        std::optional<std::pair<int,bool>> visibility;
        std::optional<std::pair<int,bool>> selection;
        // A list of about a dozen rows that scrolls on its own.
        const float rowHeight=ImGui::GetFrameHeight()+8;
        const float height=std::clamp(rowHeight*(rows+1)+4,rowHeight*3,rowHeight*12);
        ImGui::PushStyleVar(ImGuiStyleVar_CellPadding,{8,4});
        const bool open=ImGui::BeginTable("##element_list",3,ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerH|ImGuiTableFlags_PadOuterX|
                            ImGuiTableFlags_ScrollY|ImGuiTableFlags_SizingStretchProp,{0,height});
        ImGui::PopStyleVar();
        // Row labels center vertically with the eye toggle and material.
        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign,{0,.5f});
        if(open) {
            ImGui::TableSetupColumn("Show",ImGuiTableColumnFlags_WidthFixed,ImGui::CalcTextSize("Show").x+8);
            ImGui::TableSetupColumn("Element",ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Material",ImGuiTableColumnFlags_WidthFixed,ImGui::CalcTextSize("Material").x+8);
            ImGui::TableSetupScrollFreeze(0,1);ImGui::TableHeadersRow();
            ImGuiListClipper clipper;clipper.Begin(rows);
            while(clipper.Step())for(int row=clipper.DisplayStart;row<clipper.DisplayEnd;++row) {
                const int id=filtered?(*filtered)[row]:row;
                const int material=mesh.GetAttribute(id);
                bool visible=!std::binary_search(hidden.begin(),hidden.end(),id);
                const bool materialHidden=std::binary_search(excludedMaterials.begin(),excludedMaterials.end(),material);
                ImGui::PushID(id);ImGui::TableNextRow();ImGui::TableSetColumnIndex(1);
                const auto label=std::to_string(id);
                if(materialHidden)ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().muted));
                const bool clicked=ImGui::Selectable(label.c_str(),std::binary_search(selected.begin(),selected.end(),id),
                    ImGuiSelectableFlags_SpanAllColumns|ImGuiSelectableFlags_AllowOverlap,{0,ImGui::GetFrameHeight()});
                if(materialHidden)ImGui::PopStyleColor();
                ImGui::TableSetColumnIndex(0);
                // An eye toggle: bright when shown, muted when hidden.
                const float size=ImGui::GetFrameHeight();
                const ImVec2 at=ImGui::GetCursorScreenPos();
                if(ImGui::InvisibleButton("##visible",{size,size})){visible=!visible;visibility=std::pair{id,visible};}
                const bool checkboxHovered=ImGui::IsItemHovered();
                const char* eye=visible?ICON_FA_EYE:ICON_FA_EYE_SLASH;
                const ImVec2 eyeSize=ImGui::CalcTextSize(eye);
                ImGui::GetWindowDrawList()->AddText({at.x+(size-eyeSize.x)*.5f,at.y+(size-eyeSize.y)*.5f},
                    visible?(checkboxHovered?RayFEM::UI::Palette().accent:RayFEM::UI::Palette().text):RayFEM::UI::Palette().muted,eye);
                if(checkboxHovered && !materialHidden)ImGui::SetTooltip(visible?"Hide this element":"Show this element");
                if(materialHidden)Tip("This element's material is excluded. Enable its material in View to render or analyze it.");
                if(clicked && !checkboxHovered)selection=std::pair{id,
                    ImGui::IsMouseReleased(ImGuiMouseButton_Left)?RayFEM::UI::MouseSelectionShift():ImGui::GetIO().KeyShift};
                ImGui::TableSetColumnIndex(2);ImGui::AlignTextToFramePadding();
                const auto swatch=RayFEM::AttributeColor(material);
                const ImVec2 dot=ImGui::GetCursorScreenPos();
                ImGui::GetWindowDrawList()->AddCircleFilled({dot.x+5,dot.y+ImGui::GetFrameHeight()*.5f},4.5f,IM_COL32(swatch.r,swatch.g,swatch.b,255));
                ImGui::SetCursorScreenPos({dot.x+16,dot.y});ImGui::Text("%d",material);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
        ImGui::PopStyleVar();
        // Defer mutations until clipping has finished so Selected/Hidden row
        // indices cannot become invalid halfway through a filtered table.
        if(visibility)SetVisible(visibility->first,visibility->second);
        if(selection)Select(selection->first,selection->second);
    }

    Identity identity{};
    bool hasContext=false,statisticsPending=true;
    std::weak_ptr<mfem::Mesh> contextMesh;
    long meshSequence=0;
    int elementCount=0,filter=0;
    std::vector<int> selected,hidden;
    std::uint64_t revision=0;
    std::optional<Settings> lastSettings;
    std::optional<RayFEM::ElementStatistics> statistics;
    std::weak_ptr<RayFEM::RenderFrame> sampledFrame;
    std::string statisticsError,sampleField;
    std::uint32_t sampleFrame=0;
    double sampleTime=0,sampleMs=0;
};
