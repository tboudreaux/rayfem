#include "ui/plot.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
void Check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void Near(double actual,double expected,double tolerance,const char* message){
    if(!std::isfinite(actual)||std::abs(actual-expected)>tolerance)throw std::runtime_error(message);
}
void FiniteRanges(){
    using namespace RayFEM::UI;
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    const std::vector<PlotPoint> data{{0,2,true},{1,nan,false},{2,2,true},{nan,100,true},{3,inf,true},{4,-999,false}};
    const PlotSeriesView series{"Finite and missing",data};
    const auto range=FindPlotRange(std::span(&series,1));
    Check(range.finite_points==2,"Only finite valid samples contribute values");
    Near(range.x_min,0,0,"Gap positions preserve horizontal extent");Near(range.x_max,4,0,"Final missing sample preserves horizontal extent");
    Check(range.y_min<2 && range.y_max>2 && range.y_min>-999 && range.y_max<100,"Constant finite values get a useful domain without invalid outliers");
    Near(PlotFraction(2,range.y_min,range.y_max),.5,1e-12,"Constant trace centered vertically");
    Check(IsPlotPointValid(data[0]) && !IsPlotPointValid(data[1]) && !IsPlotPointValid(data[3]) && !IsPlotPointValid(data[4]) && !IsPlotPointValid(data[5]),"Invalid and nonfinite points are explicit gaps");
    const std::vector<PlotPoint> second{{-2,-4,true},{6,8,true}};
    const PlotSeriesView all[]={{"First",data},{"Second",second}};
    const auto combined=FindPlotRange(all);
    Check(combined.finite_points==4 && combined.x_min==-2 && combined.x_max==6 && combined.y_min<-4 && combined.y_max>8,"Multiple series share finite axes");
}
void ConstantAndEmpty(){
    using namespace RayFEM::UI;
    for(double value:{0.0,-42.0,1e-280,1e280,std::numeric_limits<double>::max(),-std::numeric_limits<double>::max()}){
        const PlotPoint point{value,value,true};const PlotSeriesView series{"Constant",std::span(&point,1)};
        const auto range=FindPlotRange(std::span(&series,1));
        Check(std::isfinite(range.x_min)&&std::isfinite(range.x_max)&&range.x_min<range.x_max,"Constant x has finite nonzero domain");
        Check(std::isfinite(range.y_min)&&std::isfinite(range.y_max)&&range.y_min<range.y_max,"Constant y has finite nonzero domain");
        const double f=PlotFraction(value,range.y_min,range.y_max);
        Check(std::isfinite(f)&&f>=0&&f<=1,"Constant sample maps into chart");
    }
    const auto empty=FindPlotRange({});
    Check(empty.finite_points==0 && empty.x_min<empty.x_max && empty.y_min<empty.y_max,"Empty chart has usable axes");
    const PlotPoint missing[]={{3,0,false},{9,std::numeric_limits<double>::quiet_NaN(),true}};
    const PlotSeriesView series{"Missing",missing};const auto range=FindPlotRange(std::span(&series,1));
    Check(range.finite_points==0 && range.x_min==3 && range.x_max==9,"All-missing trace retains requested horizontal domain");
}
void CoordinateRoundTrips(){
    using namespace RayFEM::UI;
    const double maximum=std::numeric_limits<double>::max();
    const std::pair<double,double> domains[]={{-maximum,maximum},{-3,17},{1e-280,9e-280},{-1e280,-2e279},{0,1}};
    for(auto [low,high]:domains)for(double fraction:{0.0,.1,.25,.5,.75,.9,1.0}){
        const double coordinate=PlotCoordinate(fraction,low,high);
        Check(std::isfinite(coordinate),"Coordinate interpolation must not overflow");
        Near(PlotFraction(coordinate,low,high),fraction,2e-14,"Pixel/domain roundtrip");
    }
    Near(PlotFraction(0,-maximum,maximum),.5,1e-15,"Opposite extreme values do not overflow subtraction");
    Near(PlotCoordinate(.5,-maximum,maximum),0,0,"Extreme midpoint is finite zero");
    Near(PlotFraction(-4,0,1),0,0,"Out-of-range coordinates clamp left");
    Near(PlotFraction(4,0,1),1,0,"Out-of-range coordinates clamp right");
    Near(PlotFraction(7,7,7),.5,0,"Degenerate direct mapping remains finite");
    const std::pair<double,double> narrow[]={{std::nextafter(maximum,0),maximum},{-maximum,-std::nextafter(maximum,0)},{1,std::nextafter(1,2)}};
    for(auto [low,high]:narrow)for(int i=0;i<=10000;++i){
        const double coordinate=PlotCoordinate(double(i)/10000,low,high);
        Check(std::isfinite(coordinate)&&coordinate>=low&&coordinate<=high,"Narrow extreme interpolation stays inside its finite bounds");
    }
}
void TickLabels(){
    using namespace RayFEM::UI;
    Check(PlotTickLabel(2.148e-10,-1,1)=="0","Near-zero unit-domain tick hides cancellation noise");
    Check(PlotTickLabel(-2.148e-10,-1,1)=="0","Near-zero tick never displays negative zero");
    Check(PlotTickLabel(2.148e-10,1e-10,3e-10)=="2.148e-10","Small physical domains retain meaningful scientific labels");
    Check(PlotTickLabel(1e280,-1e280,1e280)=="1e+280","Large finite labels preserve their exponent");
    Check(PlotTickLabel(-0.0,-1,1)=="0","Negative zero has a clean label");
}
double Luminance(ImU32 color){
    auto channel=[](double x){x/=255.;return x<=.04045?x/12.92:std::pow((x+.055)/1.055,2.4);};
    return .2126*channel((color>>IM_COL32_R_SHIFT)&255)+.7152*channel((color>>IM_COL32_G_SHIFT)&255)+.0722*channel((color>>IM_COL32_B_SHIFT)&255);
}
double Contrast(ImU32 a,ImU32 b){const double x=Luminance(a),y=Luminance(b);return (std::max(x,y)+.05)/(std::min(x,y)+.05);}
ImU32 Packed(Color color){return IM_COL32(color.r,color.g,color.b,color.a);}
void Themes(){
    using namespace RayFEM::UI;
    auto* context=ImGui::CreateContext();auto& style=ImGui::GetStyle();
    style.FramePadding={13,9};style.FrameRounding=7;style.WindowPadding={21,17};
    ImU32 darkBackground=0,lightBackground=0;
    for(auto mode:{ThemeMode::Dark,ThemeMode::Light,ThemeMode::Dark}){
        ApplyTheme(mode);const auto& p=Palette();
        Check(CurrentTheme()==mode,"Theme selection is immediately observable");
        Check(style.FramePadding.x==13&&style.FramePadding.y==9&&style.FrameRounding==7&&style.WindowPadding.x==21&&style.WindowPadding.y==17,"Theme changes preserve layout metrics");
        const auto child=ImGui::ColorConvertFloat4ToU32(style.Colors[ImGuiCol_ChildBg]);
        Check(Contrast(p.text,p.plot_background)>=7,"Plot labels have strong contrast");
        Check(Contrast(p.muted,p.plot_background)>=4.5,"Plot tick and secondary labels remain readable");
        Check(Contrast(p.text,child)>=7,"Panel text remains readable");
        for(auto color:{p.accent,p.success,p.warning})Check(Contrast(color,child)>=4.5,"Panel status text remains readable in either theme");
        for(std::size_t i=0;i<p.series.size();++i){Check(Contrast(PlotColor(i),p.plot_background)>=4.5,"Chart lines and tooltip series colors remain readable");Check(PlotColor(i)==ThemeSeriesColor(i),"Chart series use the current palette");}
        const auto viewport=Packed(p.viewport_background);
        for(auto color:{p.guide,p.dof,Packed(p.clip_guide),Packed(p.flow_low),Packed(p.flow_high),Packed(p.flow_tracer)})
            Check(Contrast(color,viewport)>=3,"Viewport overlays remain visible on their background");
        if(mode==ThemeMode::Dark)darkBackground=p.plot_background;else lightBackground=p.plot_background;
    }
    Check(Luminance(lightBackground)>.9&&Luminance(darkBackground)<.02,"Light and dark plots use their respective backgrounds");
    Check(Palette().plot_background==darkBackground,"Switching back restores the dark palette without stale chart colors");
    ImGui::DestroyContext(context);
}

}
int main(){try{FiniteRanges();ConstantAndEmpty();CoordinateRoundTrips();TickLabels();Themes();std::cout<<"Plot helpers passed: finite gaps, shared axes, constant/extreme domains, coordinate roundtrips, tick formatting, and light/dark theme contrast.\n";return 0;}catch(const std::exception& error){std::cerr<<"Plot test failed: "<<error.what()<<'\n';return 1;}}
