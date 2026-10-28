#pragma once

#include "imgui.h"
#include "raylib.h"
#include <array>
#include <cstddef>

namespace RayFEM::UI {
enum class ThemeMode { Dark, Light };

inline constexpr float CardRounding=10;
inline constexpr float ControlRounding=6;
inline constexpr float OverlayRounding=8;

// Establish the workspace's layout once. Theme changes only replace colors so
// user sizing and the current arrangement remain stable.
inline void ApplyLayout() {
    if(!ImGui::GetCurrentContext())return;
    auto& style=ImGui::GetStyle();
    style.WindowPadding={14,12};
    style.FramePadding={10,7};
    style.ItemSpacing={8,9};
    style.ItemInnerSpacing={7,5};
    style.CellPadding={8,5};
    style.WindowRounding=0;
    style.ChildRounding=CardRounding;
    style.FrameRounding=ControlRounding;
    style.GrabRounding=ControlRounding;
    style.PopupRounding=OverlayRounding;
    style.ScrollbarRounding=ControlRounding;
    style.TabRounding=ControlRounding;
    style.TabBorderSize=0;
    style.TabBarBorderSize=0;
    style.TabBarOverlineSize=2;
    style.WindowBorderSize=0;
    style.ChildBorderSize=1;
    style.FrameBorderSize=0;
    style.ScrollbarSize=10;
    style.SeparatorTextPadding={8,7};
}

// Keep related items together when they fit, otherwise let ImGui start a new
// row. Width includes the next item's complete visible and interactive area.
inline bool NextInlineItem(float width) {
    const float right=ImGui::GetCursorScreenPos().x+ImGui::GetContentRegionAvail().x;
    if(ImGui::GetItemRectMax().x+ImGui::GetStyle().ItemSpacing.x+width>right)return false;
    ImGui::SameLine();
    return true;
}

// Semantic colors shared by widgets, charts and raylib overlays. Scientific
// surface/heatmap colors keep their data meaning when the interface changes.
struct ThemePalette {
    ImU32 text=IM_COL32(225,232,245,255), muted=IM_COL32(153,174,193,255);
    ImU32 accent=IM_COL32(76,213,191,255), success=IM_COL32(91,211,166,255), warning=IM_COL32(245,185,102,255);
    ImU32 plot_background=IM_COL32(16,23,31,255), grid=IM_COL32(42,53,65,255), border=IM_COL32(70,86,101,255);
    ImU32 cursor=IM_COL32(212,224,235,175), crosshair=IM_COL32(173,196,215,140), selection=IM_COL32(225,237,245,255);
    ImU32 missing=IM_COL32(30,39,49,255), guide=IM_COL32(255,205,95,255), guide_text=IM_COL32(255,225,165,255);
    ImU32 handle_outline=IM_COL32(30,35,45,255), dof=IM_COL32(255,219,120,235), dof_ring=IM_COL32(235,245,250,255);
    Color canvas_background{14,18,24,255}, viewport_background{17,23,32,255};
    Color mesh_wire{55,67,80,255}, mesh_edges{25,35,48,255}, clip_guide{160,130,235,255};
    Color profile_guide{255,205,95,255}, flow_low{60,160,230,255}, flow_high{240,225,100,255}, flow_tracer{255,238,169,255};
    std::array<ImU32,6> series{IM_COL32(76,213,191,255),IM_COL32(243,187,102,255),IM_COL32(130,174,255,255),
        IM_COL32(212,152,245,255),IM_COL32(242,137,153,255),IM_COL32(180,215,125,255)};
};
inline const ThemePalette dark_palette{};
inline const ThemePalette light_palette=[] {
    ThemePalette p;
    p.text=IM_COL32(35,48,63,255);p.muted=IM_COL32(86,103,120,255);
    p.accent=IM_COL32(0,110,102,255);p.success=IM_COL32(24,112,79,255);p.warning=IM_COL32(150,85,12,255);
    p.plot_background=IM_COL32(252,253,255,255);p.grid=IM_COL32(224,230,237,255);p.border=IM_COL32(186,198,211,255);
    p.cursor=IM_COL32(56,76,94,220);p.crosshair=IM_COL32(56,76,94,160);p.selection=IM_COL32(28,50,70,255);
    p.missing=IM_COL32(229,235,241,255);p.guide=IM_COL32(157,77,9,255);p.guide_text=p.guide;
    p.handle_outline=IM_COL32(255,255,255,255);p.dof=IM_COL32(161,75,0,255);p.dof_ring=IM_COL32(26,42,56,255);
    p.canvas_background={238,242,247,255};p.viewport_background={236,242,246,255};
    p.mesh_wire={58,70,85,255};p.mesh_edges={25,35,48,255};p.clip_guide={118,65,177,255};
    p.profile_guide={157,77,9,255};p.flow_low={25,101,172,255};p.flow_high={146,77,0,255};p.flow_tracer={151,61,12,255};
    p.series={IM_COL32(0,114,105,255),IM_COL32(171,92,0,255),IM_COL32(40,103,180,255),
        IM_COL32(132,65,164,255),IM_COL32(184,66,91,255),IM_COL32(90,120,30,255)};
    return p;
}();
inline ThemeMode current_theme=ThemeMode::Dark;
inline ThemeMode CurrentTheme(){return current_theme;}
inline const ThemePalette& Palette(){return current_theme==ThemeMode::Light?light_palette:dark_palette;}
inline ImVec4 ThemeText(ImU32 color){return ImGui::ColorConvertU32ToFloat4(color);}
inline ImU32 ThemeSeriesColor(std::size_t index){const auto& colors=Palette().series;return colors[index%colors.size()];}
inline Color ThemeAlpha(Color color,unsigned char alpha){color.a=alpha;return color;}

// Reset only colors. User font size, spacing, rounding and docking layout stay
// untouched, and a new ImGui context can reapply the selected theme.
inline void ApplyTheme(ThemeMode mode) {
    current_theme=mode;
    if(!ImGui::GetCurrentContext())return;
    const bool light=mode==ThemeMode::Light;
    if(light)ImGui::StyleColorsLight();else ImGui::StyleColorsDark();
    auto* c=ImGui::GetStyle().Colors;
    const auto& p=Palette();
    c[ImGuiCol_Text]=ThemeText(p.text);c[ImGuiCol_TextDisabled]=ThemeText(p.muted);
    if(light) {
        c[ImGuiCol_WindowBg]={.94f,.955f,.975f,1};c[ImGuiCol_ChildBg]={.985f,.989f,.996f,1};c[ImGuiCol_PopupBg]={1,1,1,.99f};
        c[ImGuiCol_Border]={.80f,.84f,.88f,1};c[ImGuiCol_FrameBg]={.925f,.945f,.962f,1};
        c[ImGuiCol_FrameBgHovered]={.865f,.923f,.935f,1};c[ImGuiCol_FrameBgActive]={.80f,.895f,.91f,1};
        c[ImGuiCol_Button]={.91f,.938f,.953f,1};c[ImGuiCol_ButtonHovered]={.81f,.90f,.92f,1};c[ImGuiCol_ButtonActive]={.72f,.85f,.875f,1};
        c[ImGuiCol_Header]={.93f,.947f,.965f,1};c[ImGuiCol_HeaderHovered]={.845f,.912f,.93f,1};c[ImGuiCol_HeaderActive]={.77f,.868f,.895f,1};
        c[ImGuiCol_Tab]={.94f,.956f,.975f,1};c[ImGuiCol_TabHovered]={.81f,.90f,.92f,1};c[ImGuiCol_TabSelected]={.87f,.93f,.94f,1};
        c[ImGuiCol_ScrollbarBg]={.94f,.955f,.975f,1};c[ImGuiCol_ScrollbarGrab]={.65f,.72f,.79f,1};
        c[ImGuiCol_ScrollbarGrabHovered]={.51f,.62f,.70f,1};c[ImGuiCol_ScrollbarGrabActive]={.40f,.54f,.62f,1};
        c[ImGuiCol_TableRowBgAlt]={.1f,.3f,.5f,.035f};c[ImGuiCol_TextSelectedBg]={.16f,.56f,.61f,.24f};
        c[ImGuiCol_ModalWindowDimBg]={.20f,.26f,.34f,.30f};
    } else {
        c[ImGuiCol_WindowBg]={.045f,.055f,.075f,1};c[ImGuiCol_ChildBg]={.065f,.079f,.104f,1};c[ImGuiCol_PopupBg]={.085f,.10f,.13f,1};
        c[ImGuiCol_Border]={.15f,.18f,.23f,1};c[ImGuiCol_FrameBg]={.105f,.13f,.17f,1};
        c[ImGuiCol_FrameBgHovered]={.15f,.20f,.25f,1};c[ImGuiCol_FrameBgActive]={.17f,.25f,.30f,1};
        c[ImGuiCol_Button]={.12f,.18f,.23f,1};c[ImGuiCol_ButtonHovered]={.17f,.31f,.36f,1};c[ImGuiCol_ButtonActive]={.14f,.40f,.43f,1};
        c[ImGuiCol_Header]={.11f,.16f,.21f,1};c[ImGuiCol_HeaderHovered]={.15f,.25f,.30f,1};c[ImGuiCol_HeaderActive]={.13f,.33f,.36f,1};
        c[ImGuiCol_Tab]={.08f,.11f,.15f,1};c[ImGuiCol_TabHovered]={.14f,.28f,.32f,1};c[ImGuiCol_TabSelected]={.13f,.23f,.28f,1};
        c[ImGuiCol_ScrollbarBg]={.045f,.055f,.075f,1};c[ImGuiCol_ScrollbarGrab]={.25f,.32f,.39f,1};
        c[ImGuiCol_ScrollbarGrabHovered]={.32f,.43f,.50f,1};c[ImGuiCol_ScrollbarGrabActive]={.35f,.53f,.58f,1};
        c[ImGuiCol_TableRowBgAlt]={1,1,1,.025f};c[ImGuiCol_TextSelectedBg]={.20f,.70f,.65f,.30f};
        c[ImGuiCol_ModalWindowDimBg]={.02f,.04f,.06f,.65f};
    }
    c[ImGuiCol_BorderShadow]={0,0,0,0};
    c[ImGuiCol_CheckMark]=c[ImGuiCol_SliderGrab]=c[ImGuiCol_SliderGrabActive]=ThemeText(p.accent);
    c[ImGuiCol_TabSelectedOverline]=c[ImGuiCol_TabDimmedSelectedOverline]=ThemeText(p.accent);
    c[ImGuiCol_TabDimmed]=c[ImGuiCol_Tab];c[ImGuiCol_TabDimmedSelected]=c[ImGuiCol_TabSelected];
    c[ImGuiCol_Separator]=c[ImGuiCol_Border];c[ImGuiCol_SeparatorHovered]=c[ImGuiCol_SeparatorActive]=ThemeText(p.accent);
    c[ImGuiCol_TitleBg]=c[ImGuiCol_TitleBgCollapsed]=c[ImGuiCol_WindowBg];c[ImGuiCol_TitleBgActive]=c[ImGuiCol_ChildBg];
    c[ImGuiCol_MenuBarBg]=c[ImGuiCol_ChildBg];c[ImGuiCol_TableHeaderBg]=c[ImGuiCol_Header];
    c[ImGuiCol_TableBorderStrong]=c[ImGuiCol_Border];c[ImGuiCol_TableBorderLight]=ThemeText(p.grid);
    c[ImGuiCol_PlotLines]=ThemeText(p.series[0]);c[ImGuiCol_PlotLinesHovered]=ThemeText(p.series[1]);
    c[ImGuiCol_PlotHistogram]=ThemeText(p.series[1]);c[ImGuiCol_PlotHistogramHovered]=ThemeText(p.series[2]);
    c[ImGuiCol_TextLink]=c[ImGuiCol_InputTextCursor]=c[ImGuiCol_NavCursor]=ThemeText(p.accent);
    c[ImGuiCol_ResizeGrip]=ThemeText(p.border);c[ImGuiCol_ResizeGripHovered]=c[ImGuiCol_ResizeGripActive]=ThemeText(p.accent);
    c[ImGuiCol_DockingEmptyBg]=c[ImGuiCol_WindowBg];c[ImGuiCol_DockingPreview]=ThemeText(p.accent);c[ImGuiCol_DockingPreview].w=.35f;
}
} // namespace RayFEM::UI
