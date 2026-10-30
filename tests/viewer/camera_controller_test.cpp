#include "interaction/camera_controller.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void check(bool okay,const std::string& what){if(!okay)throw std::runtime_error(what);}
void near(float a,float b,float tolerance,const std::string& what){check(std::isfinite(a)&&std::abs(a-b)<=tolerance,what);}
void near(Vector3 a,Vector3 b,float tolerance,const std::string& what){near(Vector3Distance(a,b),0,tolerance,what);}
Camera3D initial(){return {{0,0,5},{0,0,0},{0,1,0},40,CAMERA_PERSPECTIVE};}
RayFEM::CameraInputSnapshot left(Vector2 mouse,bool hovered=true,bool pressed=false){
    RayFEM::CameraInputSnapshot input;input.mouse=mouse;input.viewport_hovered=hovered;input.left_pressed=pressed;input.left_down=true;return input;
}
void orbitSequence(){
    auto camera=initial();RayFEM::CameraDragState drag;
    check(!RayFEM::UpdateCameraController(camera,drag,left({100,100},true,true),1),"Mouse press must not jump the camera");
    check(drag.orbiting,"Press in viewport must latch orbit");
    check(RayFEM::UpdateCameraController(camera,drag,left({180,100}),1),"Held motion must rotate");
    near(camera.position.x,5*std::sin(-.4f),1e-5,"Expected horizontal orbit angle");
    near(camera.position.z,5*std::cos(-.4f),1e-5,"Expected orbit position");
    near(camera.target,Vector3{0,0,0},1e-6,"Orbit must preserve target");
    const auto inside=camera.position;
    RayFEM::UpdateCameraController(camera,drag,left({220,120},false),1);
    check(Vector3Distance(inside,camera.position)>.1f,"Latched drag must continue outside viewport");
    near(Vector3Distance(camera.position,camera.target),5,1e-5,"Orbit must preserve distance");
    RayFEM::CameraInputSnapshot released;released.mouse={250,150};
    check(!RayFEM::UpdateCameraController(camera,drag,released,1),"Release must stop motion immediately");
    check(!drag.orbiting,"Release must clear latch");
    const auto stopped=camera.position;
    RayFEM::UpdateCameraController(camera,drag,left({300,200},false,true),1);
    RayFEM::UpdateCameraController(camera,drag,left({350,250},true),1);
    near(camera.position,stopped,0,"Press outside must not acquire ownership by moving inside");
    std::cout<<"PASS continuous orbit sequence, hit ownership and release\n";
}
void panSequence(){
    auto camera=initial();RayFEM::CameraDragState drag;
    RayFEM::CameraInputSnapshot input;input.mouse={100,100};input.viewport_hovered=true;input.middle_pressed=input.middle_down=true;
    check(!RayFEM::UpdateCameraController(camera,drag,input,1),"Pan press must not jump");
    input.middle_pressed=false;input.mouse={140,120};input.viewport_hovered=false;
    check(RayFEM::UpdateCameraController(camera,drag,input,1),"Held middle drag must pan outside viewport");
    near(camera.target,Vector3{-.3f,.15f,0},1e-6,"Pan follows camera right/up vectors");
    near(Vector3Subtract(camera.position,camera.target),Vector3{0,0,5},1e-6,"Pan preserves view direction and distance");
    input.cancel_drag=true;input.mouse={400,500};
    check(!RayFEM::UpdateCameraController(camera,drag,input,1),"Profile capture must cancel camera drag");
    check(!drag.panning&&!drag.orbiting,"Cancellation clears both drag latches");
    const auto stopped=camera.position;input.cancel_drag=false;input.mouse={500,600};
    RayFEM::UpdateCameraController(camera,drag,input,1);near(camera.position,stopped,0,"Cancelled held button must not resume without a new press");
    std::cout<<"PASS pan invariants and explicit input cancellation\n";
}
void selectionGesture(){
    auto camera=initial();RayFEM::CameraDragState drag;bool moved=false;
    const Vector2 press{100,100};
    auto update=[&](RayFEM::CameraInputSnapshot input) {
        RayFEM::PrepareCameraOrbitGesture(input,drag,press,moved);
        return RayFEM::UpdateCameraController(camera,drag,input,1);
    };
    update(left(press,true,true));
    for(Vector2 point: {Vector2{103,101},Vector2{104,100},Vector2{99,98},press}) {
        check(!update(left(point)),"Selection jitter must not orbit the camera");
        near(camera.position,initial().position,0,"Selection clicks preserve camera position exactly");
        near(camera.up,initial().up,0,"Selection clicks preserve camera up exactly");
        check(!moved && !drag.orbiting,"Sub-threshold motion remains a selectable click");
    }
    RayFEM::CameraInputSnapshot released;released.mouse={103,100};
    check(!update(released) && !moved,"Release after jitter remains a click without camera motion");

    update(left(press,true,true));
    update(left({103,100}));
    check(update(left({106,100},false)),"Threshold crossing starts orbit, including outside the viewport");
    check(moved && drag.orbiting,"Deliberate drag is latched and cannot select");
    near(camera.position.x,5*std::sin(-.03f),1e-6,"Orbit includes displacement from the original press");
    near(camera.position.z,5*std::cos(-.03f),1e-6,"Orbit preserves complete initial movement");
    update(left(press));
    check(moved,"Returning to the press point cannot rearm selection");
    released.mouse=press;update(released);
    check(moved && !drag.orbiting,"Release stops orbit without becoming a selection");

    camera=initial();drag={};moved=false;
    auto wheel=left(press,true,true);wheel.wheel=1;
    check(update(wheel),"Wheel zoom still works while a selection press is pending");
    near(Vector3Length(camera.position),5*std::exp(-.12f),1e-5,"Deferred orbit preserves zoom response");
    camera=initial();drag={};moved=false;
    auto middle=left(press,true,true);middle.middle_pressed=middle.middle_down=true;
    update(middle);middle.middle_pressed=false;middle.left_pressed=false;middle.mouse={102,101};
    check(update(middle),"Middle pan remains available independently of the click threshold");
    near(camera.target,Vector3{-.015f,.0075f,0},1e-6,"Deferred orbit preserves middle-pan response");
    auto canceled=left({200,100});canceled.cancel_drag=true;
    const auto before=camera;
    check(!update(canceled) && !drag.orbiting && !drag.panning,"Explicit capture cancellation takes precedence");
    near(camera.position,before.position,0,"Cancelled gesture never applies its accumulated motion");
    std::cout<<"PASS click jitter, original-press drag threshold, latched selection exclusion, zoom and pan\n";
}
void clamps(){
    auto camera=initial();RayFEM::CameraDragState drag;
    RayFEM::CameraInputSnapshot input;input.viewport_hovered=true;input.wheel=1;
    RayFEM::UpdateCameraController(camera,drag,input,1);near(Vector3Length(camera.position),5*std::exp(-.12f),1e-5,"Zoom response");
    input.wheel=10000;RayFEM::UpdateCameraController(camera,drag,input,1);near(Vector3Length(camera.position),.02f,1e-6,"Minimum zoom clamp");
    input.wheel=-10000;RayFEM::UpdateCameraController(camera,drag,input,1);near(Vector3Length(camera.position),10000,1e-2,"Maximum zoom clamp");
    input.viewport_hovered=false;input.wheel=4;const auto before=camera.position;
    check(!RayFEM::UpdateCameraController(camera,drag,input,1),"Wheel outside viewport ignored");near(camera.position,before,0,"Outside zoom unchanged");
    camera=initial();drag={};RayFEM::UpdateCameraController(camera,drag,left({0,0},true,true),1);
    RayFEM::UpdateCameraController(camera,drag,left({0,100000}),1);
    near(Vector3Distance(camera.position,camera.target),5,2e-5,"Large orbit preserves radius");
    RayFEM::UpdateCameraController(camera,drag,left({0,-100000}),1);
    near(Vector3DotProduct(Vector3Normalize(camera.position),camera.up),0,2e-5,"Large orbit keeps an orthogonal frame");
    camera.position=camera.target;drag={};RayFEM::UpdateCameraController(camera,drag,left({0,0},true,true),0);
    RayFEM::UpdateCameraController(camera,drag,left({30,20}),0);
    near(Vector3Distance(camera.position,camera.target),.02f,1e-6,"Coincident target recovers finite direction");
    input={};input.viewport_hovered=true;input.wheel=-10000;
    RayFEM::UpdateCameraController(camera,drag,input,0);near(Vector3Distance(camera.position,camera.target),.02f,1e-6,"Tiny radius yields an ordered zoom interval");
    std::cout<<"PASS zoom, large orbit and degenerate-camera bounds\n";
}
void longSequence(){
    auto camera=initial();RayFEM::CameraDragState drag;
    RayFEM::UpdateCameraController(camera,drag,left({0,0},true,true),1);
    for(int step=1;step<=2000;++step){
        RayFEM::UpdateCameraController(camera,drag,left({float(step)*2,float(std::sin(step*.05)*200)},step%3!=0),1);
        near(Vector3Distance(camera.position,camera.target),5,.001f,"Long orbit sequence must not drift in radius");
        near(camera.target,Vector3{0,0,0},0,"Long orbit target fixed");
    }
    check(RayFEM::CameraViewportContains({100,100},{100,100,400,300}),"Viewport includes top-left edge");
    check(!RayFEM::CameraViewportContains({500,150},{100,100,400,300}),"Viewport excludes right edge");
    check(!RayFEM::CameraViewportContains({150,400},{100,100,400,300}),"Viewport excludes bottom edge");
    std::cout<<"PASS 2000 input frames and immediate viewport hit testing\n";
}

void validFrame(const Camera3D& camera,const std::string& context){
    const auto basis=RayFEM::CameraBasis(camera);
    near(Vector3Length(basis.right),1,2e-5,context+" right unit length");
    near(Vector3Length(basis.up),1,2e-5,context+" up unit length");
    near(Vector3Length(basis.back),1,2e-5,context+" back unit length");
    near(Vector3DotProduct(basis.right,basis.up),0,2e-5,context+" right/up perpendicular");
    near(Vector3DotProduct(basis.up,basis.back),0,2e-5,context+" up/back perpendicular");
    near(Vector3CrossProduct(basis.right,basis.up),basis.back,2e-5,context+" right-handed frame");
}
void allCubeViews(){
    auto camera=initial();camera.target={2,-3,7};camera.position=Vector3Add(camera.target,{3,4,5});
    camera.fovy=47;camera.projection=CAMERA_ORTHOGRAPHIC;
    const float distance=Vector3Distance(camera.position,camera.target);
    const Vector3 target=camera.target;
    RayFEM::CameraViewTransition transition;
    int views=0;
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z){
        if(x==0&&y==0&&z==0)continue;
        const Vector3 direction=Vector3Normalize({float(x),float(y),float(z)});
        check(RayFEM::BeginCameraViewTransition(transition,camera,direction,10,1),"Cube direction begins transition");
        for(int step=1;step<=100;++step){
            RayFEM::UpdateCameraViewTransition(camera,transition,10+step/100.0);
            near(camera.target,target,0,"Every transition preserves target");
            near(Vector3Distance(camera.position,camera.target),distance,3e-5,"Every transition preserves orbit radius");
            near(camera.fovy,47,0,"Transition preserves field of view");
            check(camera.projection==CAMERA_ORTHOGRAPHIC,"Transition preserves projection");
            validFrame(camera,"Cube view transition");
        }
        check(!transition.active,"Transition finishes at its deadline");
        near(RayFEM::CameraBasis(camera).back,direction,2e-6,"Arrives at exact face, edge or corner direction");
        if(x==0&&z==0){
            near(camera.up,Vector3{0,0,float(-y)},2e-6,"Top/bottom have stable canonical up");
        }else{
            check(camera.up.y>0,"Other canonical views keep world Y pointing upward");
            near(Vector3DotProduct(camera.up,direction),0,2e-6,"Canonical up is perpendicular");
        }
        check(!RayFEM::UpdateCameraViewTransition(camera,transition,12),"Completed transition is inert");
        ++views;
    }
    check(views==26,"Exercise six faces, twelve edges and eight corners");
    std::cout<<"PASS all 26 cube views with fixed target, radius, projection and orthogonal frames\n";
}
void easingAndShortestPath(){
    auto camera=initial();RayFEM::CameraViewTransition transition;
    RayFEM::BeginCameraViewTransition(transition,camera,{1,0,0},0,1);
    const auto start=camera;
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,0),"Start timestamp does not alter camera");
    near(camera.position,start.position,0,"No start-position jump");
    near(camera.up,start.up,0,"No start-up jump");
    for(double t:{.001,.05,.25,.5,.75,.95,.999,1.0}){
        RayFEM::UpdateCameraViewTransition(camera,transition,t);
        const float angle=std::atan2(camera.position.x,camera.position.z);
        near(angle,float(PI*.5*RayFEM::CameraViewEase(t)),3e-6,"Orientation follows eased angular progress");
    }
    // The angle's endpoint velocity and acceleration inherit those of the
    // quintic, including the final deceleration.
    const double h=1e-4;
    auto ease=RayFEM::CameraViewEase;
    check(std::abs((ease(h)-ease(0))/h)<1e-6,"Initial angular velocity tends to zero");
    check(std::abs((ease(1)-ease(1-h))/h)<1e-6,"Final angular velocity tends to zero");
    check(std::abs((ease(2*h)-2*ease(h)+ease(0))/(h*h))<.01,"Initial angular acceleration tends to zero");
    check(std::abs((ease(1)-2*ease(1-h)+ease(1-2*h))/(h*h))<.01,"Final angular acceleration tends to zero");
    for(int i=1;i<=100;++i)check(ease(i/100.0)>ease((i-1)/100.0),"Easing is monotone");

    camera=initial();
    RayFEM::BeginCameraViewTransition(transition,camera,{0,0,-1},0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.5);
    near(std::abs(camera.position.x),5,2e-5,"Antipodal view traverses a finite half-circle");
    near(camera.position.z,0,2e-5,"Antipodal midpoint never collapses through target");
    validFrame(camera,"Antipodal midpoint");
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    near(camera.position,Vector3{0,0,-5},2e-5,"Antipodal arrival");

    const float angle=170*DEG2RAD;
    camera=initial();camera.position={5*std::sin(angle),0,5*std::cos(angle)};
    RayFEM::BeginCameraViewTransition(transition,camera,{-std::sin(angle),0,std::cos(angle)},0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.5);
    near(camera.position,Vector3{0,0,-5},2e-5,"Quaternion path takes the short arc across 180 degrees");
    std::cout<<"PASS S-curve angular progress, smooth endpoints and shortest-path antipodal handling\n";
}
void restartAndManualTakeover(){
    auto camera=initial();RayFEM::CameraViewTransition transition;
    RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},2,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,2.4);
    const auto midway=camera;
    RayFEM::BeginCameraViewTransition(transition,camera,{-1,1,-1},2.4);
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,2.4),"Restart begins at current orientation");
    near(camera.position,midway.position,0,"Retargeting has no position jump");
    near(camera.up,midway.up,0,"Retargeting has no roll jump");
    RayFEM::UpdateCameraViewTransition(camera,transition,2.41);
    check(Vector3Distance(camera.position,midway.position)<.001f,"Retargeting accelerates gently");

    transition.active=false;
    const auto interrupted=camera;
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,20),"Manual cancellation stops all animation");
    near(camera.position,interrupted.position,0,"Cancelled position is unchanged");
    near(camera.up,interrupted.up,0,"Cancelled up is unchanged");
    RayFEM::CameraDragState drag;
    RayFEM::UpdateCameraController(camera,drag,left({100,100},true,true),1);
    check(!RayFEM::UpdateCameraController(camera,drag,left({100,100}),1),"Manual takeover with no motion cannot jump");
    RayFEM::UpdateCameraController(camera,drag,left({101,101}),1);
    check(Vector3Distance(camera.position,interrupted.position)<.05f,"First manual pixel motion is continuous after interruption");
    validFrame(camera,"Manual takeover");

    for(float pole:{-1.f,1.f}){
        camera=initial();
        RayFEM::BeginCameraViewTransition(transition,camera,{0,pole,0},0,0);
        RayFEM::UpdateCameraViewTransition(camera,transition,0);
        const auto snapped=camera;
        drag={};
        RayFEM::UpdateCameraController(camera,drag,left({0,0},true,true),1);
        RayFEM::UpdateCameraController(camera,drag,left({1,1}),1);
        check(Vector3Distance(camera.position,snapped.position)<.05f,"Pole orbit starts without a pitch-clamp jump");
        near(Vector3Distance(camera.position,camera.target),5,2e-5,"Pole orbit retains radius");
        validFrame(camera,"Pole orbit");
        camera=snapped;drag={};
        RayFEM::CameraInputSnapshot pan;pan.mouse={0,0};pan.viewport_hovered=true;pan.middle_pressed=pan.middle_down=true;
        RayFEM::UpdateCameraController(camera,drag,pan,1);
        pan.middle_pressed=false;pan.mouse={40,20};
        RayFEM::UpdateCameraController(camera,drag,pan,1);
        near(Vector3Subtract(camera.position,camera.target),Vector3Subtract(snapped.position,snapped.target),2e-5,"Pole pan retains view direction");
        near(Vector3Distance(camera.target,snapped.target),std::sqrt(.3f*.3f+.15f*.15f),2e-5,"Pole pan has ordinary scale");
        validFrame(camera,"Pole pan");
    }
    std::cout<<"PASS retargeting, manual interruption and orbit/pan at both poles\n";
}
void transitionInvalidInputs(){
    auto camera=initial();RayFEM::CameraViewTransition transition;
    check(!RayFEM::BeginCameraViewTransition(transition,camera,{0,0,0},0),"Zero direction rejected");
    check(!RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},0,-1),"Negative duration rejected");
    check(!RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},std::numeric_limits<double>::infinity()),"Invalid timestamp rejected");
    RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},10);
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,9),"Early clock update is inert");
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,std::numeric_limits<double>::quiet_NaN()),"Invalid clock update is inert");
    near(camera.position,Vector3{0,0,5},0,"Invalid updates leave camera unchanged");
    camera.position=camera.target;
    check(!RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},0),"Coincident camera has no snap radius");
    check(!transition.active,"Invalid begin clears an older transition");
    validFrame(camera,"Degenerate basis fallback");
    std::cout<<"PASS invalid transition inputs and finite basis fallback\n";
}
RayFEM::CameraViewBasis steppedBasis(RayFEM::CameraViewBasis basis,RayFEM::CameraRotationAxis axis,double degrees){
    // Express each expected local rotation in the original orthogonal frame,
    // independently of the controller's quaternion construction.
    const float c=float(std::cos(degrees*PI/180)),s=float(std::sin(degrees*PI/180));
    const auto original=basis;
    using Axis=RayFEM::CameraRotationAxis;
    if(axis==Axis::Up){
        basis.back=Vector3Add(Vector3Scale(original.back,c),Vector3Scale(original.right,s));
        basis.right=Vector3Subtract(Vector3Scale(original.right,c),Vector3Scale(original.back,s));
    }else if(axis==Axis::Right){
        basis.back=Vector3Subtract(Vector3Scale(original.back,c),Vector3Scale(original.up,s));
        basis.up=Vector3Add(Vector3Scale(original.up,c),Vector3Scale(original.back,s));
    }else{
        basis.up=Vector3Subtract(Vector3Scale(original.up,c),Vector3Scale(original.right,s));
        basis.right=Vector3Add(Vector3Scale(original.right,c),Vector3Scale(original.up,s));
    }
    return basis;
}
void checkBasis(const Camera3D& camera,const RayFEM::CameraViewBasis& expected,const std::string& context){
    const auto actual=RayFEM::CameraBasis(camera);
    near(actual.back,expected.back,3e-6,context+" viewing direction");
    near(actual.up,expected.up,3e-6,context+" screen up");
    near(actual.right,expected.right,3e-6,context+" screen right");
}
void rotationSteps(){
    using Axis=RayFEM::CameraRotationAxis;
    auto rolled=initial();rolled.target={2,-3,7};rolled.position=Vector3Add(rolled.target,{3,4,5});
    rolled.up={1,2,-2};rolled.fovy=47;rolled.projection=CAMERA_ORTHOGRAPHIC;
    const auto basis=RayFEM::CameraBasis(rolled);
    const float radius=Vector3Distance(rolled.position,rolled.target);
    for(auto axis:{Axis::Up,Axis::Right,Axis::Back})for(double degrees:{-45.,-5.,5.,45.}){
        auto camera=rolled;RayFEM::CameraViewTransition transition;
        check(RayFEM::BeginCameraRotationStep(transition,camera,axis,degrees,10,1),"Rotation step accepted");
        check(!RayFEM::UpdateCameraViewTransition(camera,transition,10),"Step starts without a camera jump");
        near(camera.position,rolled.position,0,"Step leaves initial position unchanged");
        near(camera.up,rolled.up,0,"Step leaves initial roll unchanged");
        RayFEM::UpdateCameraViewTransition(camera,transition,10.5);
        checkBasis(camera,steppedBasis(basis,axis,degrees/2),"Step midpoint follows eased rotation");
        for(int tick=51;tick<=100;++tick){
            RayFEM::UpdateCameraViewTransition(camera,transition,10+tick/100.0);
            near(camera.target,rolled.target,0,"Step preserves orbit center");
            near(Vector3Distance(camera.position,camera.target),radius,4e-6,"Step preserves zoom");
            near(camera.fovy,47,0,"Step preserves field of view");
            check(camera.projection==CAMERA_ORTHOGRAPHIC,"Step preserves projection");
            validFrame(camera,"Rotation step");
        }
        check(!transition.active,"Step finishes on deadline");
        checkBasis(camera,steppedBasis(basis,axis,degrees),"Exact local step including original roll");
    }
    // Local axes stay usable at either pole, without falling back to world Y.
    for(float pole:{-1.f,1.f}){
        auto camera=initial();RayFEM::CameraViewTransition transition;
        RayFEM::BeginCameraViewTransition(transition,camera,{0,pole,0},0,0);
        RayFEM::UpdateCameraViewTransition(camera,transition,0);
        const auto poleBasis=RayFEM::CameraBasis(camera);
        RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,45,1,0);
        RayFEM::UpdateCameraViewTransition(camera,transition,1);
        checkBasis(camera,steppedBasis(poleBasis,Axis::Up,45),"Pole rotation uses local up");
    }
    std::cout<<"PASS exact signed 5/45 degree local orbit and roll steps with preserved framing\n";
}
void accumulatedRotationSteps(){
    using Axis=RayFEM::CameraRotationAxis;
    auto camera=initial();RayFEM::CameraViewTransition transition;
    for(int click=0;click<9;++click){
        check(RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,5,click*.01,1),"Rapid small step accepted");
    }
    RayFEM::UpdateCameraViewTransition(camera,transition,1.08);
    checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Up,45),"Nine quick 5 degree clicks accumulate to 45");

    camera=initial();transition={};
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,45,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.3);
    const auto midway=camera;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,45,.3,1);
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,.3),"Repeated click restarts at displayed position");
    near(camera.position,midway.position,0,"Repeated click never jumps forward to pending destination");
    near(camera.up,midway.up,0,"Repeated click never jumps in roll");
    RayFEM::UpdateCameraViewTransition(camera,transition,1.3);
    checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Up,90),"Two quick large steps accumulate to 90");

    // Changing direction cancels the pending increment, not just the fraction
    // already displayed, and changing axes uses the pending local frame.
    camera=initial();transition={};
    auto expected=RayFEM::CameraBasis(camera);
    for(auto axis:{Axis::Up,Axis::Right,Axis::Back}){
        RayFEM::BeginCameraRotationStep(transition,camera,axis,45,0,1);
        expected=steppedBasis(expected,axis,45);
    }
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,-45,0,1);
    expected=steppedBasis(expected,Axis::Back,-45);
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    checkBasis(camera,expected,"Mixed-axis pending steps retain their order");

    camera=initial();transition={};
    RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},0,1);
    const auto pending=transition.destination;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Right,5,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.2);
    transition.active=false;
    const auto interrupted=RayFEM::CameraBasis(camera);
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,-5,.2,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,1.2);
    checkBasis(camera,steppedBasis(interrupted,Axis::Up,-5),"Manual cancellation discards pending increments");
    camera=initial();transition={};
    RayFEM::BeginCameraViewTransition(transition,camera,{0,1,0},0,1);
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Right,5,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    checkBasis(camera,steppedBasis(pending,Axis::Right,5),"Arrow step can extend a pending cube snap");
    std::cout<<"PASS accumulated rapid clicks, mixed axes, reversal and interrupted steps\n";
}
void invalidRotationSteps(){
    using Axis=RayFEM::CameraRotationAxis;
    auto camera=initial();RayFEM::CameraViewTransition transition;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Up,45,0,1);
    const auto expected=transition.destination;
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    for(double angle:{nan,inf,-inf,0.,360.}){
        check(!RayFEM::BeginCameraRotationStep(transition,camera,Axis::Right,angle,0),"Invalid or empty step rejected");
    }
    check(!RayFEM::BeginCameraRotationStep(transition,camera,Axis::Right,5,nan),"Invalid step clock rejected");
    for(double duration:{nan,inf,-1.}){
        check(!RayFEM::BeginCameraRotationStep(transition,camera,Axis::Right,5,0,duration),"Invalid step duration rejected");
    }
    check(!RayFEM::BeginCameraRotationStep(transition,camera,static_cast<Axis>(99),5,0),"Invalid local axis rejected");
    auto bad=camera;bad.position=bad.target;
    check(!RayFEM::BeginCameraRotationStep(transition,bad,Axis::Right,5,0),"Coincident camera step rejected");
    bad=camera;bad.up.x=std::numeric_limits<float>::infinity();
    check(!RayFEM::BeginCameraRotationStep(transition,bad,Axis::Right,5,0),"Nonfinite camera step rejected");
    check(transition.active,"Rejected clicks preserve a valid pending step");
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    checkBasis(camera,expected,"Rejected click cannot alter pending destination");
    std::cout<<"PASS invalid step input leaves valid pending animation intact\n";
}
void unwrappedRollSteps(){
    using Axis=RayFEM::CameraRotationAxis;
    for(int clicks:{8,9})for(double sign:{-1.,1.}){
        auto camera=initial();RayFEM::CameraViewTransition transition;
        for(int click=0;click<clicks;++click){
            RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,sign*45,0,1);
        }
        double previousAngle=0,travel=0;
        for(int tick=1;tick<=100;++tick){
            RayFEM::UpdateCameraViewTransition(camera,transition,tick/100.0);
            const double angle=std::atan2(-camera.up.x,camera.up.y);
            const double increment=std::remainder(angle-previousAngle,2*double(PI));
            check(increment*sign>=-1e-6,"Queued roll never reverses to take a shorter path");
            travel+=increment;previousAngle=angle;
            near(float(travel),float(sign*45*clicks*PI/180*RayFEM::CameraViewEase(tick/100.0)),3e-6,
                 "Queued roll follows unwrapped eased angular travel");
            near(camera.position,initial().position,0,"Rolling preserves camera position");
            near(camera.target,initial().target,0,"Rolling preserves target");
            if(tick==50){
                checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Back,sign*45*clicks/2),
                           "Full-turn roll passes through opposite orientation at midpoint");
            }
        }
        near(float(travel*180/PI),float(sign*45*clicks),1e-4,"Every queued 45 degree increment is animated");
        checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Back,sign*45*clicks),
                   "Full-turn roll arrives at expected orientation");
    }

    auto camera=initial();RayFEM::CameraViewTransition transition;
    for(int click=0;click<8;++click)RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,45,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.5);
    const auto midway=camera;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,45,.5,1);
    near(float(transition.roll_degrees),225,1e-5,"A new click retains only the unfinished travel plus its increment");
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,.5),"Accumulating full turns does not jump");
    near(camera.up,midway.up,0,"Accumulating roll keeps the displayed orientation");
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Back,292.5),
               "Restarted roll continues in its requested direction beyond 180 degrees");
    RayFEM::UpdateCameraViewTransition(camera,transition,1.5);
    checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Back,405),"Nine large roll clicks total 405 degrees");

    camera=initial();transition={};
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,360,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.5);
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,-405,.5,1);
    near(float(transition.roll_degrees),-225,1e-5,"Reverse clicks subtract from remaining roll travel");
    RayFEM::UpdateCameraViewTransition(camera,transition,1);
    checkBasis(camera,steppedBasis(RayFEM::CameraBasis(initial()),Axis::Back,67.5),"Reverse roll follows its signed remaining angle");
    transition.active=false;
    const auto interrupted=RayFEM::CameraBasis(camera);
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,5,1,1);
    near(float(transition.roll_degrees),5,0,"Manual interruption discards unfinished full turns");
    RayFEM::UpdateCameraViewTransition(camera,transition,2);
    checkBasis(camera,steppedBasis(interrupted,Axis::Back,5),"Fresh roll begins at manually interrupted orientation");

    // A cube snap can finish underneath a roll, and further rolls preserve
    // both its destination and all pending winding.
    camera=initial();transition={};
    RayFEM::BeginCameraViewTransition(transition,camera,{1,1,1},0,1);
    const auto cubeDestination=transition.destination;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,360,0,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,.3);
    const auto displayed=camera;
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,45,.3,1);
    check(!RayFEM::UpdateCameraViewTransition(camera,transition,.3),"Roll during cube snap retargets continuously");
    near(camera.position,displayed.position,0,"Roll during cube snap preserves displayed position");
    near(camera.up,displayed.up,0,"Roll during cube snap preserves displayed up");
    RayFEM::UpdateCameraViewTransition(camera,transition,1.3);
    checkBasis(camera,steppedBasis(cubeDestination,Axis::Back,405),"Cube snap and full-turn roll both reach requested destinations");
    RayFEM::BeginCameraRotationStep(transition,camera,Axis::Back,360,2,1);
    RayFEM::BeginCameraViewTransition(transition,camera,{0,0,1},2,1);
    RayFEM::UpdateCameraViewTransition(camera,transition,3);
    checkBasis(camera,RayFEM::CameraBasis(initial()),"New cube snap clears previously pending roll winding");
    std::cout<<"PASS signed full-turn roll winding, continuous accumulation, reversal and cancellation\n";
}
}
int main(){try{
    orbitSequence();panSequence();selectionGesture();clamps();longSequence();
    allCubeViews();easingAndShortestPath();restartAndManualTakeover();transitionInvalidInputs();
    rotationSteps();accumulatedRotationSteps();invalidRotationSteps();
    unwrappedRollSteps();
}catch(const std::exception& error){std::cerr<<"Camera test failed: "<<error.what()<<'\n';return 1;}
std::cout<<"Camera controller tests passed\n";}
