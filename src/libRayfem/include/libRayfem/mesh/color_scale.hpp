#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <span>
#include <stdexcept>
#include <vector>

namespace RayFEM {

enum class ColorScale { Linear, SignedLog, ZScale };
inline constexpr int ColorScaleCount = 3;

inline constexpr const char* ColorScaleName(ColorScale scale) {
    switch (scale) {
    case ColorScale::SignedLog: return "Signed log";
    case ColorScale::ZScale: return "ZScale";
    default: return "Linear";
    }
}

struct ColorScaleOptions {
    ColorScale scale = ColorScale::Linear;
    // Zero chooses 1% of the largest absolute limit. Positive values are in
    // field units and set the smooth transition from linear to logarithmic.
    double log_threshold = 0;
    // IRAF/DS9 contrast: larger values narrow the fitted display interval.
    // Zero disables fitting and retains the complete supplied interval.
    double zscale_contrast = .25;
};

struct ColorScaleMapping {
    ColorScale scale = ColorScale::Linear;
    double minimum = 0, maximum = 1, log_threshold = 1;
};

namespace ColorScaleDetail {
inline bool Ordered(double minimum, double maximum) {
    return std::isfinite(minimum) && std::isfinite(maximum) && minimum <= maximum;
}

inline bool HasSpan(double minimum, double maximum) {
    if (!Ordered(minimum,maximum) || minimum == maximum) return false;
    const double scale = std::max(std::abs(minimum),std::abs(maximum));
    return maximum/scale - minimum/scale > 1e-12;
}

inline double LinearFraction(double value, double minimum, double maximum) {
    if (!std::isfinite(value) || !HasSpan(minimum,maximum)) return .5;
    if (value <= minimum) return 0;
    if (value >= maximum) return 1;
    const double scale = std::max(std::abs(minimum),std::abs(maximum));
    return std::clamp((value/scale - minimum/scale)/(maximum/scale - minimum/scale),0.0,1.0);
}

inline double SignedLog(double value, double threshold) {
    const double magnitude = std::abs(value);
    // Dividing a huge value by a tiny threshold can overflow even though the
    // logarithm is finite. The second identity keeps all intermediates finite.
    const double ratio = magnitude/threshold;
    const double transformed = std::isfinite(ratio) ? std::log1p(ratio) :
        std::log(magnitude) - std::log(threshold) + std::log1p(threshold/magnitude);
    return std::copysign(transformed,value);
}

inline double InverseSignedLog(double value, double threshold) {
    const double magnitude = std::abs(value);
    if (magnitude < 1) return std::copysign(threshold*std::expm1(magnitude),value);
    // exp(value) itself may overflow while threshold*expm1(value) is finite.
    const double exponent = std::log(threshold) + magnitude + std::log1p(-std::exp(-magnitude));
    const double result = exponent >= std::log(std::numeric_limits<double>::max()) ?
        std::numeric_limits<double>::max() : std::exp(exponent);
    return std::copysign(result,value);
}

inline std::vector<double> ZSamples(std::span<const double> values) {
    const std::size_t finite = std::count_if(values.begin(),values.end(),[](double v){return std::isfinite(v);});
    const std::size_t count = std::min<std::size_t>(1000,finite);
    std::vector<double> samples;
    samples.reserve(count);
    if (!count) return samples;
    // Deterministic endpoint-inclusive subsampling, spanning the entire input.
    // Split the multiplication to avoid overflowing size_t for long spans.
    auto target = [&](std::size_t i) {
        if (count == 1) return std::size_t(0);
        return i*((finite-1)/(count-1)) + (i*((finite-1)%(count-1)))/(count-1);
    };
    std::size_t ordinal = 0;
    for (double value : values) if (std::isfinite(value)) {
        if (ordinal == target(samples.size())) {
            samples.push_back(value);
            if (samples.size() == count) break;
        }
        ++ordinal;
    }
    std::sort(samples.begin(),samples.end());
    return samples;
}

// Independent implementation of the ranked-sample fit described by IRAF:
// https://iraf.readthedocs.io/en/latest/tasks/images/tv/display.html#zscale-algorithm
// Rejection parameters and endpoint convention match Astropy/STScI zscale:
// https://docs.astropy.org/en/stable/_modules/astropy/visualization/interval.html#ZScaleInterval
// Finite samples are bounded to 1000; insufficient/unreliable fits retain the
// supplied full range. ZScale chooses limits, then uses a linear transfer.
inline void ZLimits(ColorScaleMapping& result, std::span<const double> values, double contrast) {
    if (contrast == 0 || values.empty()) return;
    const auto samples = ZSamples(values);
    const std::size_t n = samples.size(), minimum_good = std::max<std::size_t>(5,n/2);
    if (n < minimum_good || samples.front() == samples.back()) return;
    const double scale = std::max(std::abs(samples.front()),std::abs(samples.back()));
    std::vector<double> y(n),residual(n);
    for (std::size_t i=0;i<n;++i) y[i]=samples[i]/scale;
    std::vector<bool> rejected(n,false);
    std::size_t good=n,last_good=n+1;
    const std::size_t grow=std::max<std::size_t>(1,n/100);
    double slope=0;
    bool fitted=false;
    for (int iteration=0;iteration<5 && good>=minimum_good && good<last_good;++iteration) {
        double mean_x=0,mean_y=0,xx=0,xy=0;
        std::size_t count=0;
        // Welford covariance avoids subtracting large, nearly equal sums.
        for (std::size_t i=0;i<n;++i) if (!rejected[i]) {
            const double x=2.0*i/(n-1)-1,dx=x-mean_x,dy=y[i]-mean_y;
            ++count;mean_x+=dx/count;mean_y+=dy/count;
            xx+=dx*(x-mean_x);xy+=dx*(y[i]-mean_y);
        }
        if (!(xx>0)) return;
        slope=xy/xx;
        const double intercept=mean_y-slope*mean_x;
        double residual_mean=0,variance=0;
        count=0;
        for (std::size_t i=0;i<n;++i) {
            residual[i]=y[i]-(intercept+slope*(2.0*i/(n-1)-1));
            if (!rejected[i]) {
                const double delta=residual[i]-residual_mean;
                ++count;residual_mean+=delta/count;
                variance+=delta*(residual[i]-residual_mean);
            }
        }
        fitted=true;
        const double threshold=2.5*std::sqrt(std::max(0.0,variance/count));
        // An exactly affine sample can have rounding residuals. Do not reject
        // a correct fit because its residuals are at machine precision.
        if (threshold<=32*std::numeric_limits<double>::epsilon()) break;
        auto grown=rejected;
        for (std::size_t i=0;i<n;++i) if (rejected[i] || std::abs(residual[i])>threshold) {
            const std::size_t first=i>=(grow-1)/2 ? i-(grow-1)/2 : 0;
            const std::size_t last=std::min(n-1,i+grow/2);
            for (std::size_t j=first;j<=last;++j) grown[j]=true;
        }
        last_good=good;rejected=std::move(grown);
        good=std::count(rejected.begin(),rejected.end(),false);
    }
    if (!fitted || good<minimum_good || !std::isfinite(slope) || slope<0) return;
    const double median=std::midpoint(y[(n-1)/2],y[n/2]);
    const double pixel_slope=(slope*2/(n-1))/contrast;
    const double center=double((n-1)/2);
    const double low=std::clamp(median-(center-1)*pixel_slope,y.front(),y.back());
    const double high=std::clamp(median+(n-center)*pixel_slope,y.front(),y.back());
    // A constant background with a few isolated outliers can yield slope zero.
    // Collapsing its limits would map even those outliers to the midpoint.
    // Retain the complete interval whenever the fit has no usable color span.
    if (!HasSpan(low,high)) return;
    result.minimum=std::clamp(low*scale,result.minimum,result.maximum);
    result.maximum=std::clamp(high*scale,result.minimum,result.maximum);
}
} // namespace ColorScaleDetail

// Resolve one reusable mapping per dataset/settings change, not per vertex.
// Invalid option parameters throw. Invalid limits remain invalid so display
// helpers can safely show the midpoint; data producers should validate values.
inline ColorScaleMapping ResolveColorScale(double minimum, double maximum,
                                          const ColorScaleOptions& options = {},
                                          std::span<const double> samples = {}) {
    if (options.scale!=ColorScale::Linear && options.scale!=ColorScale::SignedLog && options.scale!=ColorScale::ZScale)
        throw std::invalid_argument("Unknown color scale");
    if (!std::isfinite(options.log_threshold) || options.log_threshold<0)
        throw std::invalid_argument("Signed log threshold must be finite and nonnegative");
    if (!std::isfinite(options.zscale_contrast) || options.zscale_contrast<0 || options.zscale_contrast>1)
        throw std::invalid_argument("ZScale contrast must be between zero and one");
    ColorScaleMapping result{options.scale,minimum,maximum,options.log_threshold};
    if (result.log_threshold==0) {
        const double magnitude=std::max(std::abs(minimum),std::abs(maximum));
        result.log_threshold=std::isfinite(magnitude) && magnitude>0 ?
            std::max(magnitude*.01,std::numeric_limits<double>::denorm_min()) : 1;
    }
    if (options.scale==ColorScale::ZScale && ColorScaleDetail::Ordered(minimum,maximum))
        ColorScaleDetail::ZLimits(result,samples,options.zscale_contrast);
    return result;
}

// Nonfinite inputs, invalid limits, and nearly constant relative intervals map
// to the midpoint, matching the original linear FieldColor behavior.
inline double ColorScaleFraction(double value, const ColorScaleMapping& mapping) {
    using namespace ColorScaleDetail;
    if (mapping.scale!=ColorScale::SignedLog)
        return LinearFraction(value,mapping.minimum,mapping.maximum);
    if (!std::isfinite(value) || !HasSpan(mapping.minimum,mapping.maximum) ||
        !std::isfinite(mapping.log_threshold) || mapping.log_threshold<=0) return .5;
    if (value<=mapping.minimum) return 0;
    if (value>=mapping.maximum) return 1;
    const double magnitude=std::max(std::abs(mapping.minimum),std::abs(mapping.maximum));
    // Deep inside the linear core, a ratio can underflow. The limiting linear
    // map is both more accurate and preserves distinguishable tiny values.
    if (magnitude/mapping.log_threshold<1e-8)
        return LinearFraction(value,mapping.minimum,mapping.maximum);
    const double low=SignedLog(mapping.minimum,mapping.log_threshold);
    const double high=SignedLog(mapping.maximum,mapping.log_threshold);
    if (!(high>low)) return .5;
    return std::clamp((SignedLog(value,mapping.log_threshold)-low)/(high-low),0.0,1.0);
}

// Inverse for colorbar labels and hover readouts. Endpoints are returned exactly;
// clamping and std::lerp avoid overflow even for opposite DBL_MAX limits.
inline double ColorScaleValue(double fraction, const ColorScaleMapping& mapping) {
    using namespace ColorScaleDetail;
    if (!Ordered(mapping.minimum,mapping.maximum)) return 0;
    if (!std::isfinite(fraction)) fraction=.5;
    fraction=std::clamp(fraction,0.0,1.0);
    if (!HasSpan(mapping.minimum,mapping.maximum)) return std::midpoint(mapping.minimum,mapping.maximum);
    if (fraction==0) return mapping.minimum;
    if (fraction==1) return mapping.maximum;
    if (mapping.scale==ColorScale::SignedLog) {
        if (!std::isfinite(mapping.log_threshold) || mapping.log_threshold<=0)
            return std::midpoint(mapping.minimum,mapping.maximum);
        const double magnitude=std::max(std::abs(mapping.minimum),std::abs(mapping.maximum));
        if (magnitude/mapping.log_threshold<1e-8)
            return std::lerp(mapping.minimum,mapping.maximum,fraction);
        const double transformed=std::lerp(SignedLog(mapping.minimum,mapping.log_threshold),
                                           SignedLog(mapping.maximum,mapping.log_threshold),fraction);
        return std::clamp(InverseSignedLog(transformed,mapping.log_threshold),mapping.minimum,mapping.maximum);
    }
    return std::lerp(mapping.minimum,mapping.maximum,fraction);
}

} // namespace RayFEM
