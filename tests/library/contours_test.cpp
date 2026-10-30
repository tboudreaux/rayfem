#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/contours.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(bool okay,const std::string& what){if(!okay)throw std::runtime_error(what);}
void near(double a,double b,double eps,const std::string& what){if(!std::isfinite(a)||std::abs(a-b)>eps)throw std::runtime_error(what+": "+std::to_string(a)+" vs "+std::to_string(b));}
template<class F> void rejects(F f,const std::string& what){bool thrown=false;try{f();}catch(const std::invalid_argument&){thrown=true;}check(thrown,what);}

void SingleTriangle() {
    const std::vector<float> positions{0,0,0, 2,0,0, 0,2,0};
    const std::vector<float> values{0,1,2};
    const std::vector<double> half{.5};
    auto lines=RayFEM::ExtractContours(positions,values,half);
    check(lines.size()==1,"One crossing per level in a straddling triangle");
    // value = x/2 + y; the 0.5 isoline meets edge 0-1 at x=1 and edge 2-0 at y=.5.
    for(const auto& p:{lines[0].a,lines[0].b}) near(p.x/2+p.y,.5,1e-6,"Endpoint lies on the level");
    const std::vector<double> through_vertex{1};
    lines=RayFEM::ExtractContours(positions,values,through_vertex);
    check(lines.size()==1,"A level through a vertex still gives one segment");
    for(const auto& p:{lines[0].a,lines[0].b}) near(p.x/2+p.y,1,1e-6,"Vertex-level endpoint lies on the level");
    const std::vector<double> outside{-1,3};
    check(RayFEM::ExtractContours(positions,values,outside).empty(),"Levels outside the triangle give no lines");
    const std::vector<float> flat{1,1,1},invalid{0,NAN,2};
    check(RayFEM::ExtractContours(positions,flat,half).empty(),"Flat triangles give no lines");
    check(RayFEM::ExtractContours(positions,invalid,half).empty(),"Non-finite values are skipped");
    rejects([&]{RayFEM::ExtractContours(positions,std::vector<float>{0,1},half);},"Mismatched values reject");
    rejects([&]{RayFEM::ExtractContours(positions,values,std::vector<double>{1,.5});},"Unsorted levels reject");
}

void Levels() {
    const auto linear=RayFEM::ContourLevels(RayFEM::ResolveColorScale(0,10),4);
    check(linear.size()==4,"Requested level count");
    for(int i=0;i<4;++i) near(linear[i],2*(i+1),1e-12,"Linear levels split the range evenly");
    check(RayFEM::ContourLevels(RayFEM::ResolveColorScale(0,10),0).empty(),"Zero levels");
    RayFEM::ColorScaleOptions log;log.scale=RayFEM::ColorScale::SignedLog;log.log_threshold=.01;
    const auto mapping=RayFEM::ResolveColorScale(-100,1000,log);
    const auto levels=RayFEM::ContourLevels(mapping,9);
    check(levels.size()==9,"Signed-log level count");
    for(int i=0;i<9;++i) near(RayFEM::ColorScaleFraction(levels[i],mapping),(i+1)/10.0,1e-9,"Levels follow the colorbar");
}

// A linear field is exact on the tessellation, so every contour endpoint must
// sit on its level, on the outer surface and on the clipped section alike.
void Surfaces() {
    auto mesh=mfem::Mesh::MakeCartesian3D(3,3,3,mfem::Element::HEXAHEDRON);mesh.SetCurvature(2);
    mfem::H1_FECollection h1(2,3);mfem::FiniteElementSpace space(&mesh,&h1);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x){return x[0]+2*x[1]+3*x[2];});field.ProjectCoefficient(ramp);
    RayFEM::RenderOptions options;options.refinement=3;
    const auto levels=RayFEM::ContourLevels(RayFEM::ResolveColorScale(0,6),11);
    auto verify=[&](const RayFEM::SurfaceData& surface,const char* what,bool cap) {
        const std::vector<float> values(surface.values.begin(),surface.values.end());
        const auto lines=RayFEM::ExtractContours(surface.positions,values,levels);
        check(!lines.empty(),std::string(what)+" has contours");
        std::size_t onCap=0;
        for(const auto& line:lines) {
            const double a=line.a.x+2*line.a.y+3*line.a.z,b=line.b.x+2*line.b.y+3*line.b.z;
            near(a,b,2e-5,std::string(what)+" segment stays on one level");
            const auto level=std::min_element(levels.begin(),levels.end(),[&](double x,double y){return std::abs(x-a)<std::abs(y-a);});
            near(a,*level,2e-5,std::string(what)+" segment lies on a requested level");
            if(std::abs(line.a.x-.5)<1e-5 && std::abs(line.b.x-.5)<1e-5)++onCap;
        }
        if(cap)check(onCap>0,"The clipped section carries contours");
    };
    verify(RayFEM::BuildSurface(mesh,&field,options),"Outer surface",false);
    verify(RayFEM::BuildClippedSurface(mesh,&field,options,{{.5f,0,0},{1,0,0}}).surface,"Clipped surface",true);
}
} // namespace

int main() {
    try {
        SingleTriangle();Levels();Surfaces();
        std::cout<<"Contour checks passed.\n";return 0;
    } catch(const std::exception& error){std::cerr<<"Contour test failed: "<<error.what()<<'\n';return 1;}
}
