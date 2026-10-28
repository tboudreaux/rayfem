#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace RayFEM::UI {

// Keep animation time in double precision until it is wrapped for the GPU.
// In particular, a long-running viewer must not quantize a slowly moving pulse.
inline double StreamlinePhase(double seconds,double cyclesPerSecond,double offset=0) {
    if(!std::isfinite(seconds)||!std::isfinite(cyclesPerSecond)||!std::isfinite(offset))
        throw std::invalid_argument("Streamline animation parameters must be finite");
    const double cycles=cyclesPerSecond==0?0:
        std::fmod(seconds,1/std::abs(cyclesPerSecond))*cyclesPerSecond;
    const double phase=std::fmod(cycles+std::fmod(offset,1.0),1.0);
    return phase<0?phase+1:phase;
}

// The length is the full width at half maximum, measured as a fraction of the
// original complete path. Clipping and hiding never rescale this coordinate.
inline double StreamlinePulse(double arc,double phase,double length) {
    if(!std::isfinite(arc)||!std::isfinite(phase)||!std::isfinite(length)||!(length>0))
        throw std::invalid_argument("Streamline pulse coordinates must be finite and length positive");
    const double distance=std::remainder(std::remainder(arc,1.0)-std::remainder(phase,1.0),1.0);
    const double normalized=distance/length;
    return std::exp(-4*std::log(2.0)*normalized*normalized);
}

inline double StreamlinePulseWidth(double base,double peak,double envelope) {
    return base+(std::max(base,peak)-base)*std::clamp(envelope,0.0,1.0);
}

// Screen-space interpolation needs this correction for a perspective camera.
// It yields the original linear path fraction, not the projected pixel fraction.
inline double StreamlinePerspectiveFraction(double screenFraction,double firstW,double lastW) {
    const double u=std::clamp(screenFraction,0.0,1.0);
    if(!std::isfinite(screenFraction)||!std::isfinite(firstW)||!std::isfinite(lastW)||!(firstW>0)||!(lastW>0))
        throw std::invalid_argument("Streamline clip coordinates must have positive finite depth");
    return (u*firstW)/((1-u)*lastW+u*firstW);
}

} // namespace RayFEM::UI
