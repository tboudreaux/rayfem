#pragma once

#include "imgui.h"
#include "theme.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <span>
#include <string>
#include <vector>

namespace RayFEM::UI {

struct PlotPoint { double x = 0, y = 0; bool valid = true; };
struct PlotSeriesView {
    const char* label = "Series";
    std::span<const PlotPoint> points;
    ImU32 color = ThemeSeriesColor(0);
};
struct PlotOptions {
    const char* x_label = "";
    const char* y_label = "";
    bool legend = true;
    double cursor_x = std::numeric_limits<double>::quiet_NaN();
    // Optional output. A click inside the axes writes the domain coordinate.
    double* clicked_x = nullptr;
    // Optional output. Hovering inside the axes writes the domain coordinate.
    double* hovered_x = nullptr;
    // Optional vertical markers (for example path control points), numbered from 1.
    std::span<const double> markers{};
};
struct PlotRange {
    double x_min = 0, x_max = 1, y_min = 0, y_max = 1;
    size_t finite_points = 0;
};

inline ImU32 PlotColor(size_t index) { return ThemeSeriesColor(index); }
inline bool IsPlotPointValid(const PlotPoint& point) {
    return point.valid && std::isfinite(point.x) && std::isfinite(point.y);
}
// Scale before subtraction to avoid overflow for large, opposite-sign values.
inline double PlotFraction(double value, double low, double high) {
    if (!(high > low) || !std::isfinite(value)) return .5;
    const double scale = std::max(std::abs(low), std::abs(high));
    if (!(scale > 0)) return .5;
    return std::clamp((value/scale-low/scale)/(high/scale-low/scale), 0.0, 1.0);
}
inline double PlotCoordinate(double fraction, double low, double high) {
    fraction = std::clamp(fraction, 0.0, 1.0);
    // lerp preserves endpoints and stays between finite bounds, including
    // adjacent values near the largest representable number.
    return std::lerp(low,high,fraction);
}
inline std::string PlotTickLabel(double value,double low,double high) {
    // Suppress cancellation noise in tick labels only. Hover details and data
    // retain full precision, and genuinely small domains keep their scale.
    const double scale=std::max(std::abs(low),std::abs(high));
    const double zero_tolerance=scale>0?((high/scale-low/scale)*1e-9)*scale:0;
    if(std::abs(value)<=zero_tolerance)value=0;
    char label[64];std::snprintf(label,sizeof(label),"%.4g",value);
    return label;
}
inline void ExpandPlotRange(double& low, double& high, bool padding) {
    if (high == low) {
        const double margin = std::max(1.0, std::abs(low)*.05);
        const double a = low-margin, b = high+margin;
        if (std::isfinite(a)) low = a;
        if (std::isfinite(b)) high = b;
        return;
    }
    if (!padding) return;
    const double scale = std::max(std::abs(low), std::abs(high));
    const double margin = ((high/scale-low/scale)*.06)*scale;
    const double a = low-margin, b = high+margin;
    if (std::isfinite(a)) low = a;
    if (std::isfinite(b)) high = b;
}
inline PlotRange FindPlotRange(std::span<const PlotSeriesView> series) {
    PlotRange range;
    bool x_found = false, y_found = false;
    for (const auto& line : series) for (const auto& point : line.points) {
        // Missing values still retain their position on the horizontal axis.
        if (std::isfinite(point.x)) {
            if (!x_found) { range.x_min=range.x_max=point.x; x_found=true; }
            range.x_min=std::min(range.x_min,point.x); range.x_max=std::max(range.x_max,point.x);
        }
        if (!IsPlotPointValid(point)) continue;
        if (!y_found) { range.y_min=range.y_max=point.y; y_found=true; }
        range.y_min=std::min(range.y_min,point.y); range.y_max=std::max(range.y_max,point.y);
        ++range.finite_points;
    }
    ExpandPlotRange(range.x_min,range.x_max,false);
    ExpandPlotRange(range.y_min,range.y_max,true);
    return range;
}

// An in-window chart: callers keep ownership of point arrays. Invalid/non-finite
// samples create gaps; each series is independently scaled into shared axes.
inline void PlotLines(const char* id, std::span<const PlotSeriesView> series,
                      ImVec2 requested_size, const PlotOptions& options = {}) {
    const ImVec2 available=ImGui::GetContentRegionAvail();
    const ImVec2 size={std::max(1.0f,requested_size.x>0?requested_size.x:available.x),
                       std::max(1.0f,requested_size.y>0?requested_size.y:available.y)};
    const ImVec2 origin=ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id,size);
    const bool hovered=ImGui::IsItemHovered();
    auto* draw=ImGui::GetWindowDrawList();
    draw->PushClipRect(origin,{origin.x+size.x,origin.y+size.y},true);
    const ImVec2 card_max={origin.x+size.x,origin.y+size.y};
    draw->AddRectFilled(origin,card_max,Palette().plot_background,CardRounding);
    draw->AddRect({origin.x+.5f,origin.y+.5f},{card_max.x-.5f,card_max.y-.5f},
                  Palette().grid,CardRounding);
    if(size.x<170 || size.y<100) {
        draw->AddText({origin.x+12,origin.y+10},Palette().muted,"Enlarge this panel to view the plot");
        draw->PopClipRect();return;
    }
    const PlotRange range=FindPlotRange(series);
    const float text_height=ImGui::GetTextLineHeight();
    float legend_height=0;
    if(options.legend && !series.empty()) {
        float x=origin.x+12,y=origin.y+10;
        const int max_rows=size.y>=190?2:1;
        int row=0;
        size_t shown=0;
        for(const auto& line:series) {
            const char* label=line.label?line.label:"Series";
            std::string short_label=label;
            const float max_label=std::max(45.0f,std::min(160.0f,size.x-120));
            if(ImGui::CalcTextSize(short_label.c_str()).x>max_label) {
                while(!short_label.empty() && ImGui::CalcTextSize(short_label.c_str()).x>max_label-16) {
                    size_t cut=short_label.size()-1;
                    while(cut && (static_cast<unsigned char>(short_label[cut])&0xC0)==0x80)--cut;
                    short_label.resize(cut);
                }
                short_label+="...";
            }
            float item_width=ImGui::CalcTextSize(short_label.c_str()).x+34;
            if(x>origin.x+12 && x+item_width>origin.x+size.x-70) {
                if(++row>=max_rows)break;
                x=origin.x+12;y+=text_height+7;
            }
            draw->AddLine({x,y+text_height*.5f},{x+14,y+text_height*.5f},line.color,2);
            draw->AddText({x+21,y},Palette().text,short_label.c_str());
            x+=item_width;
            ++shown;
        }
        if(shown<series.size()) {
            char remaining[32];std::snprintf(remaining,sizeof(remaining),"+%zu more",series.size()-shown);
            draw->AddText({origin.x+size.x-65,y},Palette().muted,remaining);
        }
        legend_height=y-origin.y+text_height+9;
    }
    const bool y_label=options.y_label && *options.y_label;
    const float axis_header=y_label?text_height+4:0;
    ImVec2 low={origin.x+70,origin.y+std::max(legend_height,8.0f)+4+axis_header};
    const ImVec2 high={origin.x+size.x-18,origin.y+size.y-
        (options.x_label&&*options.x_label?text_height*2+10:text_height+11)};
    const float height=high.y-low.y;
    const int y_ticks=std::clamp(static_cast<int>(height/45),2,5);
    std::vector<std::string> y_labels;
    y_labels.reserve(y_ticks+1);
    for(int i=0;i<=y_ticks;++i) {
        y_labels.push_back(PlotTickLabel(PlotCoordinate(double(i)/y_ticks,range.y_min,range.y_max),range.y_min,range.y_max));
        low.x=std::max(low.x,origin.x+ImGui::CalcTextSize(y_labels.back().c_str()).x+20);
    }
    const float width=high.x-low.x;
    if(width<30 || height<25) {
        draw->AddText({origin.x+12,origin.y+std::max(legend_height,10.0f)},Palette().muted,"Enlarge this panel to view the plot");
        draw->PopClipRect();return;
    }
    auto screen=[&](const PlotPoint& point) {
        return ImVec2{low.x+static_cast<float>(PlotFraction(point.x,range.x_min,range.x_max))*width,
                      high.y-static_cast<float>(PlotFraction(point.y,range.y_min,range.y_max))*height};
    };
    const int x_ticks=std::clamp(static_cast<int>(width/100),2,6);
    for(int i=0;i<=x_ticks;++i) {
        const double fraction=double(i)/x_ticks;
        const float x=low.x+static_cast<float>(fraction)*width;
        draw->AddLine({x,low.y},{x,high.y},Palette().grid);
        const auto text=PlotTickLabel(PlotCoordinate(fraction,range.x_min,range.x_max),range.x_min,range.x_max);
        const float label_width=ImGui::CalcTextSize(text.c_str()).x;
        const float label_x=std::clamp(x-label_width*.5f,origin.x+3,origin.x+size.x-label_width-3);
        draw->AddText({label_x,high.y+7},Palette().muted,text.c_str());
    }
    for(int i=0;i<=y_ticks;++i) {
        const double fraction=double(i)/y_ticks;
        const float y=high.y-static_cast<float>(fraction)*height;
        draw->AddLine({low.x,y},{high.x,y},Palette().grid);
        const auto& text=y_labels[i];
        draw->AddText({low.x-ImGui::CalcTextSize(text.c_str()).x-10,y-text_height*.5f},Palette().muted,text.c_str());
    }
    draw->AddRect(low,high,Palette().border);
    if(options.x_label && *options.x_label) {
        const float label_width=ImGui::CalcTextSize(options.x_label).x;
        draw->AddText({low.x+(width-label_width)*.5f,origin.y+size.y-text_height-6},Palette().text,options.x_label);
    }
    if(y_label)
        draw->AddText({low.x,low.y-text_height-5},Palette().text,options.y_label);
    draw->PushClipRect({low.x-2,low.y-2},{high.x+2,high.y+2},true);
    for(const auto& line:series) {
        bool previous_valid=false;
        ImVec2 previous{};
        for(size_t i=0;i<line.points.size();++i) {
            const auto& point=line.points[i];
            if(!IsPlotPointValid(point)) {previous_valid=false;continue;}
            const ImVec2 current=screen(point);
            if(previous_valid) draw->AddLine(previous,current,line.color,2);
            const bool isolated=!previous_valid && (i+1==line.points.size() || !IsPlotPointValid(line.points[i+1]));
            if(line.points.size()<=32 || isolated) draw->AddCircleFilled(current,2.5f,line.color);
            previous=current;previous_valid=true;
        }
    }
    for(std::size_t m=0;m<options.markers.size();++m) {
        // The end of a path lands on x_max up to rounding; keep it.
        const double slack=1e-9*std::max(1.0,range.x_max-range.x_min);
        const double value=options.markers[m];
        if(!std::isfinite(value) || value<range.x_min-slack || value>range.x_max+slack)continue;
        const float x=low.x+static_cast<float>(PlotFraction(std::clamp(value,range.x_min,range.x_max),range.x_min,range.x_max))*width;
        draw->AddLine({x,low.y},{x,high.y},WithAlpha(Palette().guide,90),1);
        const std::string label=std::to_string(m+1);
        const float label_width=ImGui::CalcTextSize(label.c_str()).x;
        draw->AddText({x+3+label_width>high.x?x-3-label_width:x+3,low.y+2},Palette().guide_text,label.c_str());
    }
    if(std::isfinite(options.cursor_x) && options.cursor_x>=range.x_min && options.cursor_x<=range.x_max) {
        const float x=low.x+static_cast<float>(PlotFraction(options.cursor_x,range.x_min,range.x_max))*width;
        for(float y=low.y;y<high.y;y+=8)draw->AddLine({x,y},{x,std::min(y+4,high.y)},Palette().cursor);
    }
    const ImVec2 mouse=ImGui::GetMousePos();
    const bool inside=hovered && mouse.x>=low.x && mouse.x<=high.x && mouse.y>=low.y && mouse.y<=high.y;
    if(inside) {
        const double x=PlotCoordinate((mouse.x-low.x)/width,range.x_min,range.x_max);
        const double y=PlotCoordinate((high.y-mouse.y)/height,range.y_min,range.y_max);
        draw->AddLine({mouse.x,low.y},{mouse.x,high.y},Palette().crosshair);
        draw->AddLine({low.x,mouse.y},{high.x,mouse.y},Palette().crosshair);
        if(options.clicked_x && ImGui::IsMouseClicked(ImGuiMouseButton_Left))*options.clicked_x=x;
        if(options.hovered_x)*options.hovered_x=x;
        ImGui::BeginTooltip();
        ImGui::Text("%s %.7g",options.x_label&&*options.x_label?options.x_label:"x",x);
        ImGui::TextDisabled("Cursor value %.7g",y);
        bool any=false;
        for(const auto& line:series) {
            const PlotPoint* nearest=nullptr;double distance=std::numeric_limits<double>::infinity();
            for(const auto& point:line.points)if(IsPlotPointValid(point)) {
                const double dx=std::abs(PlotFraction(point.x,range.x_min,range.x_max)-PlotFraction(x,range.x_min,range.x_max));
                if(dx<distance){distance=dx;nearest=&point;}
            }
            if(nearest) {
                if(!any){ImGui::Separator();ImGui::TextDisabled("Nearest samples");any=true;}
                const ImVec4 color=ImGui::ColorConvertU32ToFloat4(line.color);
                ImGui::TextColored(color,"%s: %.9g",line.label?line.label:"Series",nearest->y);
                ImGui::SameLine();ImGui::TextDisabled("at %.7g",nearest->x);
                const ImVec2 p=screen(*nearest);
                draw->AddCircleFilled(p,4,line.color);
                draw->AddCircle(p,6,Palette().selection,0,1.2f);
            }
        }
        if(!any)ImGui::TextDisabled("No finite samples");
        ImGui::EndTooltip();
    }
    draw->PopClipRect();
    if(hovered && legend_height>0 && mouse.y<origin.y+legend_height) {
        ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*30);
        for(const auto& line:series) {
            ImGui::PushStyleColor(ImGuiCol_Text,ImGui::ColorConvertU32ToFloat4(line.color));
            ImGui::TextUnformatted(line.label?line.label:"Series");ImGui::PopStyleColor();
        }
        ImGui::PopTextWrapPos();ImGui::EndTooltip();
    }
    if(!range.finite_points) {
        const char* label="No valid samples";
        draw->AddText({low.x+(width-ImGui::CalcTextSize(label).x)*.5f,low.y+height*.5f},Palette().muted,label);
    }
    draw->PopClipRect();
}

} // namespace RayFEM::UI
