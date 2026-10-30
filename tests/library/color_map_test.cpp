#include "libRayfem/mesh/colormap.hpp"
#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace RayFEM;
void Check(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
bool Equal(Color a,Color b) {return a.r==b.r&&a.g==b.g&&a.b==b.b&&a.a==b.a;}
void ReferenceColors() {
    // Independent 8-bit references to Matplotlib 3.10.7 entries 0, 64, 128, 255.
    // Internal points catch channel swaps or accidentally substituted tables.
    constexpr std::array<ColorMap,4> maps={ColorMap::Viridis,ColorMap::Plasma,ColorMap::Inferno,ColorMap::Cividis};
    constexpr Color references[4][4]={
        {{68,1,84,255},{59,82,139,255},{33,145,140,255},{253,231,37,255}},
        {{13,8,135,255},{126,3,168,255},{204,71,120,255},{240,249,33,255}},
        {{0,0,4,255},{87,16,110,255},{188,55,84,255},{252,255,164,255}},
        {{0,34,78,255},{67,78,108,255},{125,124,120,255},{254,232,56,255}}
    };
    constexpr double coordinates[]={0,64./255,128./255,1};
    for (std::size_t i=0;i<maps.size();++i)
        for (int j=0;j<4;++j)
            Check(Equal(SampleColorMap(maps[i],coordinates[j]),references[i][j]),
                  "Scientific palette matches the authoritative reference");
    Check(Equal(SampleColorMap(ColorMap::CoolWarm,0),{59,76,192,255}),"CoolWarm blue endpoint");
    Check(Equal(SampleColorMap(ColorMap::CoolWarm,.5),{221,221,221,255}),"CoolWarm neutral center");
    Check(Equal(SampleColorMap(ColorMap::CoolWarm,1),{180,4,38,255}),"CoolWarm red endpoint");
    Check(Equal(SampleColorMap(ColorMap::Jet,0),{0,0,127,255}),"Legacy Jet lower endpoint");
    Check(Equal(SampleColorMap(ColorMap::Jet,.5),{127,255,127,255}),"Legacy Jet midpoint");
    Check(Equal(SampleColorMap(ColorMap::Jet,1),{127,0,0,255}),"Legacy Jet upper endpoint");
    Check(std::string(ColorMapName(ColorMap::Viridis))=="Viridis","Stable palette name");
    Check(Equal(SampleColorMap(static_cast<ColorMap>(-1),.3),SampleColorMap(ColorMap::Viridis,.3)),
          "Unknown palette safely defaults to Viridis");
}
void LimitsAndReversal() {
    const double huge=std::numeric_limits<double>::max();
    const double tiny=std::numeric_limits<double>::denorm_min();
    for (int i=0;i<ColorMapCount;++i) {
        const auto map=static_cast<ColorMap>(i);
        const auto midpoint=SampleColorMap(map,.5);
        for (double t:{0.,.1,.25,.5,.75,.9,1.})
            Check(Equal(SampleColorMap(map,t,true),SampleColorMap(map,1-t)),
                  "Reversal exchanges colors without changing their mapping");
        Check(Equal(SampleColorMap(map,-10),SampleColorMap(map,0))&&
              Equal(SampleColorMap(map,10),SampleColorMap(map,1)),"Palette coordinates clamp");
        Check(Equal(FieldColor(0,-huge,huge,map),midpoint),"Opposite extreme limits do not overflow");
        Check(Equal(FieldColor(huge/2,-huge,huge,map),SampleColorMap(map,.75)),"Extreme interior value");
        Check(Equal(FieldColor(2*tiny,tiny,3*tiny,map),midpoint),"Subnormal field retains its scale");
        Check(Equal(FieldColor(-huge,-tiny,tiny,map),SampleColorMap(map,0))&&
              Equal(FieldColor(huge,-tiny,tiny,map),SampleColorMap(map,1)),"Extreme outliers clamp safely");
        Check(Equal(FieldColor(0,-1e-100,1e-100,map),midpoint)&&
              !Equal(FieldColor(-1e-100,-1e-100,1e-100,map),FieldColor(1e-100,-1e-100,1e-100,map)),
              "Tiny values preserve useful color contrast");
        Check(Equal(FieldColor(3,3,3,map),midpoint)&&Equal(FieldColor(0,0,0,map),midpoint),
              "Constant fields use the palette midpoint");
        Check(Equal(FieldColor(3,3,3+1e-14,map),midpoint),"Near-constant relative limits are stable");
        Check(Equal(FieldColor(0,1,-1,map),midpoint),"Inverted limits fail safely");
        for (double value:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
            Check(Equal(SampleColorMap(map,value),midpoint)&&Equal(FieldColor(value,-1,1,map),midpoint),
                  "Invalid display inputs cannot produce invalid indexing or byte conversion");
        Check(Equal(FieldColor(-1,-1,1,map,true),SampleColorMap(map,1))&&
              Equal(FieldColor(1,-1,1,map,true),SampleColorMap(map,0)),"Field reversal preserves numerical limits");
    }
}
}
int main() {
    try {
        ReferenceColors();LimitsAndReversal();
        std::cout<<"PASS authoritative palettes, reversal, constants, extreme/subnormal limits and safe display inputs\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL "<<e.what()<<'\n';
        return 1;
    }
}
