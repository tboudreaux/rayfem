#include "../src/app/streamline_pulse.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void near(double actual,double expected,const char* message,double tolerance=1e-12) {
    require(std::abs(actual-expected)<tolerance,message);
}
template<class Function> void invalid(Function&& function) {
    try{function();}catch(const std::invalid_argument&){return;}
    throw std::runtime_error("Invalid pulse input was accepted");
}
}

int main() {
    using namespace RayFEM::UI;
    try {
        near(StreamlinePhase(0,.22,.3),.3,"Initial path phase");
        near(StreamlinePhase(2,1,.25),.25,"Positive time wrapping");
        near(StreamlinePhase(-.25,1),.75,"Negative time wrapping");
        near(StreamlinePhase(.25,-1),.75,"Reverse animation wrapping");
        near(StreamlinePhase(1099511627776.25,1),.25,"Long viewer runtime lost subsecond phase");
        near(StreamlinePhase(10000,0,-.25),.75,"Static phase wrapping");
        near(StreamlinePulse(.25,.25,.12),1,"Pulse peak");
        near(StreamlinePulse(.31,.25,.12),.5,"Pulse forward half width");
        near(StreamlinePulse(.19,.25,.12),.5,"Pulse backward half width");
        near(StreamlinePulse(0,.98,.12),StreamlinePulse(1,.98,.12),"Pulse wrap discontinuity");
        near(StreamlinePulse(.02,.98,.12),StreamlinePulse(.94,.98,.12),"Wrapped Gaussian symmetry");
        require(StreamlinePulse(.75,.25,.12)<1e-15,"Pulse has an excessively long tail");
        // Separate clipped pieces use the same original coordinates, so hiding
        // the middle or splitting a bend cannot move or restart its highlight.
        for(double arc:{.13,.27,.61,.92}) {
            const double whole=StreamlinePulse(arc,.3,.12);
            near(StreamlinePulse(arc,StreamlinePhase(0,1,.3),.12),whole,"Split-path pulse changed");
        }
        near(StreamlinePulseWidth(1.25,3,1),3,"Peak width");
        near(StreamlinePulseWidth(1.25,3,0),1.25,"Base line disappeared");
        near(StreamlinePulseWidth(1.25,1.25,.8),1.25,"Disabled thickness still widens");
        near(StreamlinePulseWidth(2,1,.8),2,"Pulse shrank below base width");
        near(StreamlinePerspectiveFraction(.5,1,1),.5,"Orthographic path interpolation");
        near(StreamlinePerspectiveFraction(.5,1,3),.25,"Perspective path interpolation");
        near(StreamlinePerspectiveFraction(.75,1,3),.5,"Perspective midpoint phase");
        near(StreamlinePerspectiveFraction(0,1,3),0,"First path endpoint");
        near(StreamlinePerspectiveFraction(1,1,3),1,"Last path endpoint");
        const auto nan=std::numeric_limits<double>::quiet_NaN();
        invalid([&]{StreamlinePhase(nan,1);});invalid([&]{StreamlinePulse(0,0,0);});
        invalid([&]{StreamlinePulse(nan,0,.12);});invalid([&]{StreamlinePerspectiveFraction(.5,0,1);});
        invalid([&]{StreamlinePerspectiveFraction(nan,1,1);});
        std::cout<<"PASS Gaussian pulse, periodic arc phase, thickness and perspective interpolation\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
