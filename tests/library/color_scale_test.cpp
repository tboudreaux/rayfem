#include "libRayfem/mesh/colormap.hpp"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace RayFEM;
void Check(bool value,const std::string& message) {if(!value)throw std::runtime_error(message);}
void Near(double value,double expected,double tolerance,const std::string& message) {
    Check(std::isfinite(value) && std::abs(value-expected)<=tolerance,
          message+": "+std::to_string(value)+" vs "+std::to_string(expected));
}
bool Same(Color a,Color b) {return a.r==b.r&&a.g==b.g&&a.b==b.b&&a.a==b.a;}
template<class F> void Reject(F&& f) {
    try {f();} catch(const std::invalid_argument&) {return;}
    throw std::runtime_error("Invalid color scale option was accepted");
}

void LinearCompatibility() {
    Check(ColorScaleOptions{}.scale==ColorScale::Linear,"Linear remains the default");
    Check(std::string(ColorScaleName(ColorScale::SignedLog))=="Signed log" &&
          std::string(ColorScaleName(ColorScale::ZScale))=="ZScale","Readable scale names");
    const double huge=std::numeric_limits<double>::max(),tiny=std::numeric_limits<double>::denorm_min();
    for (auto limits:std::array<std::array<double,2>,7>{{{-10,20},{-huge,huge},{tiny,3*tiny},
                                                    {3,3},{3,3+1e-14},{0,0},{1,-1}}}) {
        const auto mapping=ResolveColorScale(limits[0],limits[1]);
        for (double v:{limits[0],limits[1],std::midpoint(limits[0],limits[1]),0.,huge,-huge})
            for (int m=0;m<ColorMapCount;++m) for (bool reverse:{false,true})
                Check(Same(FieldColor(v,mapping,static_cast<ColorMap>(m),reverse),
                           FieldColor(v,limits[0],limits[1],static_cast<ColorMap>(m),reverse)),
                      "Shared mapping preserves existing linear colors exactly");
    }
    const auto extreme=ResolveColorScale(-huge,huge);
    Near(ColorScaleFraction(huge/2,extreme),.75,1e-15,"Extreme linear fraction");
    Near(ColorScaleValue(.75,extreme)/huge,.5,1e-15,"Extreme inverse avoids overflow");
}

void SignedLogarithm() {
    const ColorScaleOptions logarithmic{.scale=ColorScale::SignedLog,.log_threshold=1};
    const auto mapping=ResolveColorScale(-99,99,logarithmic);
    // log1p(99)=log(100), log1p(9)=log(10): exact decade references.
    Near(ColorScaleFraction(-9,mapping),.25,1e-15,"Negative decade");
    Near(ColorScaleFraction(0,mapping),.5,0,"Zero is defined and centered for symmetric limits");
    Near(ColorScaleFraction(9,mapping),.75,1e-15,"Positive decade");
    Near(ColorScaleValue(.25,mapping),-9,2e-14,"Negative inverse decade");
    Near(ColorScaleValue(.75,mapping),9,2e-14,"Positive inverse decade");
    Check(ColorScaleFraction(-1e-12,mapping)<.5 && ColorScaleFraction(1e-12,mapping)>.5,
          "Signed log remains continuous and ordered through zero");
    for (auto limits:std::array<std::array<double,2>,3>{{{-99,99},{0,99},{-99,-1}}}) {
        const auto map=ResolveColorScale(limits[0],limits[1],logarithmic);
        double previous=-1;
        for (int i=0;i<=100;++i) {
            const double fraction=i/100.0,value=ColorScaleValue(fraction,map);
            Check(std::isfinite(value) && value>=limits[0] && value<=limits[1],"Inverse stays within limits");
            const double actual=ColorScaleFraction(value,map);
            Near(actual,fraction,3e-15,"Signed-log tick round trip");
            Check(actual>=previous,"Signed log is monotone");previous=actual;
        }
    }
    const auto automatic=ResolveColorScale(-100,20,{.scale=ColorScale::SignedLog});
    Near(automatic.log_threshold,1,0,"Automatic linear core follows field units");
    const auto rescaled=ResolveColorScale(-1e-248,2e-249,{.scale=ColorScale::SignedLog});
    Near(ColorScaleFraction(1e-249,rescaled),ColorScaleFraction(10,automatic),2e-15,
         "Automatic signed-log colors are invariant under unit scaling");
    for (int m=0;m<ColorMapCount;++m)
        Check(Same(FieldColor(9,mapping,static_cast<ColorMap>(m),true),
                   SampleColorMap(static_cast<ColorMap>(m),.25)),"Palette reversal follows log normalization");
}

void NumericEdges() {
    const double huge=std::numeric_limits<double>::max(),tiny=std::numeric_limits<double>::denorm_min();
    const double nan=std::numeric_limits<double>::quiet_NaN(),infinity=std::numeric_limits<double>::infinity();
    const auto mapping=ResolveColorScale(-huge,huge,{.scale=ColorScale::SignedLog,.log_threshold=tiny});
    Near(ColorScaleFraction(0,mapping),.5,0,"Extreme signed log zero");
    Near(ColorScaleValue(.75,mapping)/(std::sqrt(huge)*std::sqrt(tiny)),1,2e-13,
         "Log-domain inverse spans more than 600 decades without overflow");
    for (double t:{0.,.01,.25,.49,.5,.51,.75,.99,1.}) {
        const double value=ColorScaleValue(t,mapping);
        Check(std::isfinite(value),"Extreme inverse is finite");
        Near(ColorScaleFraction(value,mapping),t,2e-14,"Extreme signed-log round trip");
    }
    const auto microscopic=ResolveColorScale(tiny,3*tiny,{.scale=ColorScale::SignedLog,.log_threshold=huge});
    Near(ColorScaleFraction(2*tiny,microscopic),.5,1e-15,"Underflowing log ratio retains its linear limit");
    Near(ColorScaleValue(.5,microscopic),2*tiny,0,"Microscopic inverse retains representable values");
    Check(ResolveColorScale(-tiny,tiny,{.scale=ColorScale::SignedLog}).log_threshold==tiny,
          "Automatic threshold never underflows to zero");
    for (auto scale:{ColorScale::Linear,ColorScale::SignedLog,ColorScale::ZScale}) {
        const auto constant=ResolveColorScale(4,4,{.scale=scale});
        Near(ColorScaleFraction(4,constant),.5,0,"Constants use palette midpoint");
        Near(ColorScaleValue(.2,constant),4,0,"Constant inverse");
        const auto normal=ResolveColorScale(-1,1,{.scale=scale});
        for (double invalid:{nan,infinity,-infinity})
            Near(ColorScaleFraction(invalid,normal),.5,0,"Nonfinite display values fail safely");
        Near(ColorScaleFraction(0,ResolveColorScale(nan,1,{.scale=scale})),.5,0,"Nonfinite limits fail safely");
        Near(ColorScaleFraction(0,ResolveColorScale(1,-1,{.scale=scale})),.5,0,"Inverted limits fail safely");
    }
    Reject([]{ResolveColorScale(0,1,{.log_threshold=-1});});
    Reject([&]{ResolveColorScale(0,1,{.log_threshold=nan});});
    Reject([]{ResolveColorScale(0,1,{.zscale_contrast=1.1});});
    Reject([&]{ResolveColorScale(0,1,{.zscale_contrast=infinity});});
    Reject([]{ResolveColorScale(0,1,{.scale=static_cast<ColorScale>(-1)});});
}

void ZScaleReferences() {
    // A nearly affine 0..99 core with one outlier at each end. Golden limits
    // were evaluated independently with NumPy and the published Astropy
    // get_limits implementation; exactly 100/102 samples survive rejection.
    // https://docs.astropy.org/en/stable/_modules/astropy/visualization/interval.html#ZScaleInterval
    std::vector<double> samples{-1e6};
    for (int i=0;i<100;++i)samples.push_back(i+.1*std::sin(i));
    samples.push_back(1e6);
    const ColorScaleOptions options{.scale=ColorScale::ZScale};
    const auto fitted=ResolveColorScale(-1e6,1e6,options,samples);
    Near(fitted.minimum,-146.54365250782826,1e-9,"Reference ZScale lower limit");
    Near(fitted.maximum,257.4209895203965,1e-9,"Reference ZScale upper limit");
    Near(ColorScaleFraction(-1e6,fitted),0,0,"ZScale clips low outliers");
    Near(ColorScaleFraction(1e6,fitted),1,0,"ZScale clips high outliers");
    Near(ColorScaleValue(.5,fitted),55.43866850628412,1e-9,"ZScale uses a linear transfer after fitting");
    const auto contrast=ResolveColorScale(-1e6,1e6,{.scale=ColorScale::ZScale,.zscale_contrast=1},samples);
    Near(contrast.minimum,.44348209155055685,1e-9,"Higher contrast narrows lower limit");
    Near(contrast.maximum,101.43464259860676,1e-9,"Higher contrast narrows upper limit");
    const auto disabled=ResolveColorScale(-1e6,1e6,{.scale=ColorScale::ZScale,.zscale_contrast=0},samples);
    Near(disabled.minimum,-1e6,0,"Zero contrast retains complete minimum");
    Near(disabled.maximum,1e6,0,"Zero contrast retains complete maximum");
    const auto no_samples=ResolveColorScale(-3,5,options);
    Near(no_samples.minimum,-3,0,"Missing samples retain explicit limits");
    Near(no_samples.maximum,5,0,"Missing samples retain explicit limits");
    const std::array sparse{0.,1.,2.,3.};
    const auto sparse_map=ResolveColorScale(-3,5,options,sparse);
    Near(sparse_map.minimum,-3,0,"Insufficient fit retains full range");
    Near(sparse_map.maximum,5,0,"Insufficient fit retains full range");
    const double nan=std::numeric_limits<double>::quiet_NaN();
    samples.insert(samples.begin(),nan);samples.push_back(std::numeric_limits<double>::infinity());
    const auto filtered=ResolveColorScale(-1e6,1e6,options,samples);
    Near(filtered.minimum,fitted.minimum,0,"Nonfinite samples are excluded");
    Near(filtered.maximum,fitted.maximum,0,"Nonfinite samples are excluded");
    samples.erase(samples.begin());samples.pop_back();
    for (double& value:samples)value*=1e300;
    const auto large=ResolveColorScale(-1e306,1e306,options,samples);
    Near(large.minimum/1e300,fitted.minimum,1e-9,"ZScale fit is safe at extreme magnitudes");
    Near(large.maximum/1e300,fitted.maximum,1e-9,"ZScale fit is safe at extreme magnitudes");
    std::vector<double> affine{-1e6};
    for (int i=0;i<100;++i)affine.push_back(i);
    affine.push_back(1e6);
    const auto exact=ResolveColorScale(-1e6,1e6,options,affine);
    Near(exact.minimum,-146.5,1e-9,"Roundoff does not reject an exactly affine core");
    Near(exact.maximum,257.5,1e-9,"Roundoff does not reject an exactly affine core");
    std::vector<double> constant_background(100,3);
    constant_background.push_back(999);
    const auto sparse_peak=ResolveColorScale(3,999,options,constant_background);
    Near(sparse_peak.minimum,3,0,"Zero-slope background fit retains full lower limit");
    Near(sparse_peak.maximum,999,0,"Zero-slope background fit retains sparse outliers");
    Near(ColorScaleFraction(3,sparse_peak),0,0,"Constant background remains distinct from its outlier");
    Near(ColorScaleFraction(999,sparse_peak),1,0,"Sparse outlier remains visible");
    std::vector<double> long_samples(2000);
    std::iota(long_samples.begin(),long_samples.end(),0.0);
    const auto selected=ColorScaleDetail::ZSamples(long_samples);
    Check(selected.size()==1000 && selected.front()==0 && selected.back()==1999,
          "Bounded sampling spans the complete input rather than taking a prefix");
    const auto spanning=ResolveColorScale(0,1999,{.scale=ColorScale::ZScale,.zscale_contrast=1},long_samples);
    Check(std::isfinite(spanning.minimum) && std::isfinite(spanning.maximum) &&
          spanning.minimum>=0 && spanning.minimum<spanning.maximum && spanning.maximum<=1999,
          "Bounded-sample fit retains finite ordered limits within the full range");
}
}

int main() {
    try {
        LinearCompatibility();SignedLogarithm();NumericEdges();ZScaleReferences();
        std::cout<<"PASS linear compatibility, signed-log forward/inverse, zero/extreme values and reference ZScale fits\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL "<<e.what()<<'\n';return 1;
    }
}
