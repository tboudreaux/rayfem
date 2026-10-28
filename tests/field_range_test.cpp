#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/io/serif.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace RayFEM;
void Check(bool condition,const std::string& message) {if(!condition)throw std::runtime_error(message);}
void Near(double actual,double expected,double tolerance,const std::string& message) {
    Check(std::isfinite(actual)&&std::abs(actual-expected)<=tolerance,
          message+": "+std::to_string(actual)+" vs "+std::to_string(expected));
}
template<class F> void Reject(F&& operation,const std::string& fragment) {
    try {operation();} catch(const std::exception& e) {
        Check(std::string(e.what()).find(fragment)!=std::string::npos,
              "Expected '"+fragment+"', received '"+e.what()+"'");return;
    }
    throw std::runtime_error("Expected rejection: "+fragment);
}
std::array<unsigned char,4> ColorAt(const SurfaceData& data,std::size_t index) {
    return {data.colors[4*index],data.colors[4*index+1],data.colors[4*index+2],data.colors[4*index+3]};
}
void ComparePointEvaluator(mfem::Mesh& mesh,mfem::GridFunction& field,const RenderOptions& options) {
    const auto range=EvaluateFieldRange(mesh,field,options,{.subdivisions=4});
    double minimum=std::numeric_limits<double>::infinity(),maximum=-minimum;
    mfem::GeometryRefiner refiner;
    for(int e=0;e<mesh.GetNE();++e)if(IsElementIncluded(mesh,e,options)) {
        const auto geometry=mesh.GetElementBaseGeometry(e);
        auto sample=[&](const mfem::IntegrationPoint& point) {
            const double value=EvaluateField(field,e,point,options.view,options.component);
            minimum=std::min(minimum,value);maximum=std::max(maximum,value);
        };
        const auto& points=refiner.Refine(geometry,4)->RefPts;
        for(int p=0;p<points.GetNPoints();++p)sample(points.IntPoint(p));
        const mfem::IntegrationRule nodes(field.FESpace()->GetFE(e)->GetNodes());
        for(int p=0;p<nodes.GetNPoints();++p)sample(nodes.IntPoint(p));
        sample(mfem::Geometries.GetCenter(geometry));
    }
    Near(range.min_value,minimum,1e-11*std::max(1.,std::abs(minimum)),"Bulk lower agrees with independent point evaluator");
    Near(range.max_value,maximum,1e-11*std::max(1.,std::abs(maximum)),"Bulk upper agrees with independent point evaluator");
}
void InteriorPeak() {
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    mfem::H1_FECollection collection(2,3);mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient bump([](const mfem::Vector& p){
        return 64*p[0]*(1-p[0])*p[1]*(1-p[1])*p[2]*(1-p[2]);
    });field.ProjectCoefficient(bump);
    auto surface=BuildSurface(mesh,&field);
    Near(surface.max_value,0,1e-14,"The exterior misses an interior tensor-quadratic peak");
    const auto range=EvaluateFieldRange(mesh,field);
    ComparePointEvaluator(mesh,field,{});
    Near(range.min_value,0,1e-14,"Volume minimum");Near(range.max_value,1,1e-14,"Volume samples include the interior peak");
    Check(range.element_count==1 && range.sample_count==371 && range.max_subdivisions==6 &&
          !range.budget_limited,"Default order-aware sampling diagnostics");
    const auto original=surface;
    ApplyFieldColorRange(surface,range);
    Near(surface.max_value,1,0,"Legend uses volume range");
    Check(surface.values==original.values && surface.positions==original.positions &&
          surface.normals==original.normals && surface.material_ids==original.material_ids &&
          surface.boundary_ids==original.boundary_ids,"Recoloring preserves all geometric and scientific samples");
    Check(ColorAt(surface,0)==std::array<unsigned char,4>{68,1,84,255},
          "A zero exterior is the lower range endpoint, not the constant-field color");

    const auto reduced=EvaluateFieldRange(mesh,field,{}, {.max_samples=100});
    Check(reduced.budget_limited && reduced.min_subdivisions==2 && reduced.max_subdivisions==2 &&
          reduced.sample_count==55 && reduced.element_count==1,"Automatic budget reduction keeps every parent and FE node");
    Near(reduced.max_value,1,1e-14,"Reduced resolution still samples the element center");
    Reject([&]{EvaluateFieldRange(mesh,field,{}, {.subdivisions=6,.max_samples=100});},"budget");
    Reject([&]{EvaluateFieldRange(mesh,field,{}, {.max_samples=10});},"budget");

    // An odd lattice misses this peak location; explicit FE-node samples must
    // still include the Q2 midpoint, independently of lattice density.
    const auto odd=EvaluateFieldRange(mesh,field,{}, {.subdivisions=3});
    Near(odd.max_value,1,1e-14,"FE nodes supplement odd subdivision lattices");
    RenderOptions ignored;ignored.refinement=1;ignored.coloring=SurfaceColoring::Material;
    ignored.displacement=&field;ignored.displacement_scale=std::numeric_limits<double>::quiet_NaN();
    const auto same=EvaluateFieldRange(mesh,field,ignored);
    Check(same.min_value==range.min_value && same.max_value==range.max_value &&
          same.sample_count==range.sample_count,"Display displacement/refinement/coloring do not change the field range");
    for(float plane:{.25f,.75f}) {
        RenderOptions cut_options;cut_options.refinement=4;
        auto cut=BuildClippedSurface(mesh,&field,cut_options,{{plane,0,0},{1,0,0}});
        Check(!cut.surface.values.empty(),"A moved section has visible field samples");
        ApplyFieldColorRange(cut.surface,range);
        Check(cut.surface.min_value==range.min_value&&cut.surface.max_value==range.max_value,
              "Moving the clipping plane preserves the volume color range");
    }
    auto empty_cut=BuildClippedSurface(mesh,&field,{},{{-1,0,0},{1,0,0}});
    Check(empty_cut.surface.values.empty(),"Plane outside the mesh can hide the complete domain");
    ApplyFieldColorRange(empty_cut.surface,range);
    Check(empty_cut.surface.colors.empty()&&empty_cut.surface.max_value==range.max_value,
          "Empty clipping keeps valid volume legend metadata without manufacturing geometry");
    std::cout<<"PASS interior extrema, FE nodes, full-domain budget adaptation and color remapping\n";
}
void MaterialsAndValidation() {
    auto mesh=mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true);
    mesh.GetElement(0)->SetAttribute(2);mesh.GetElement(1)->SetAttribute(7);mesh.SetAttributes();
    mfem::L2_FECollection collection(0,2);mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);mfem::Array<int> dofs;
    space.GetElementDofs(0,dofs);field[dofs[0]]=-3;
    space.GetElementDofs(1,dofs);field[dofs[0]]=9;
    auto range=EvaluateFieldRange(mesh,field);Near(range.min_value,-3,0,"Discontinuous first parent");Near(range.max_value,9,0,"Discontinuous second parent");
    RenderOptions magnitude;magnitude.view=FieldView::Magnitude;
    const auto absolute=EvaluateFieldRange(mesh,field,magnitude);
    Near(absolute.min_value,3,0,"Scalar magnitude takes absolute values");Near(absolute.max_value,9,0,"Scalar magnitude upper");
    ComparePointEvaluator(mesh,field,magnitude);
    RenderOptions selected;selected.excluded_materials={7};
    range=EvaluateFieldRange(mesh,field,selected);
    Check(range.min_value==-3 && range.max_value==-3 && range.element_count==1,"Material support applies throughout the domain");
    selected.excluded_materials={2,7};Reject([&]{EvaluateFieldRange(mesh,field,selected);},"no included");
    selected.excluded_materials.clear();
    Reject([&]{EvaluateFieldRange(mesh,field,{}, {.subdivisions=65});},"subdivisions");
    Reject([&]{EvaluateFieldRange(mesh,field,{}, {.max_samples=0});},"budget");
    selected.view=static_cast<FieldView>(99);Reject([&]{EvaluateFieldRange(mesh,field,selected);},"operation");
    selected.view=FieldView::Component;selected.component=1;Reject([&]{EvaluateFieldRange(mesh,field,selected);},"component");
    auto other=mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true);
    Reject([&]{EvaluateFieldRange(other,field);},"supplied mesh");
    field[dofs[0]]=std::numeric_limits<double>::quiet_NaN();
    Reject([&]{EvaluateFieldRange(mesh,field);},"non-finite");
    selected={};selected.excluded_materials={7};
    range=EvaluateFieldRange(mesh,field,selected);
    Near(range.min_value,-3,0,"An excluded invalid field does not contaminate the selected domain");
    std::cout<<"PASS discontinuous materials, constants, invalid operations/data and strict budget errors\n";
}
void DistributionSampling() {
    auto mesh=mfem::Mesh::MakeCartesian2D(12,1,mfem::Element::QUADRILATERAL,true);
    mfem::L2_FECollection collection(0,2);mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);mfem::Array<int> dofs;
    for(int e=0;e<mesh.GetNE();++e) {
        mesh.GetElement(e)->SetAttribute(e+1);
        space.GetElementDofs(e,dofs);field[dofs[0]]=e==11 ? 10000. : 10.*e;
    }
    mesh.SetAttributes();
    const auto original=EvaluateFieldRange(mesh,field);
    Check(original.samples.empty(),"Existing range calls retain no distribution by default");
    const auto full=EvaluateFieldRange(mesh,field,{},
        {.distribution_samples=std::numeric_limits<std::size_t>::max()});
    Check(full.samples.size()==full.sample_count && full.samples.size()==324,
          "Distribution memory is bounded by the actual number of evaluated values");
    for(std::size_t count:{1u,2u,7u,24u,323u,324u,500u}) {
        const auto sampled=EvaluateFieldRange(mesh,field,{}, {.distribution_samples=count});
        const auto again=EvaluateFieldRange(mesh,field,{}, {.distribution_samples=count});
        const auto retained=std::min(count,full.sample_count);
        Check(sampled.samples.size()==retained && sampled.samples==again.samples,
              "Distribution sample count is bounded and deterministic");
        Check(sampled.min_value==original.min_value && sampled.max_value==original.max_value &&
              sampled.sample_count==original.sample_count && sampled.element_count==original.element_count,
              "Collecting a distribution preserves full-domain extrema and coverage");
        for(std::size_t i=0;i<retained;++i) {
            const auto index=retained==1 ? full.samples.size()/2 : i*(full.samples.size()-1)/(retained-1);
            Near(sampled.samples[i],full.samples[index],0,"Samples span the full ordered sequence evenly");
        }
    }
    const auto sampled=EvaluateFieldRange(mesh,field,{}, {.distribution_samples=24});
    for(int e=0;e<mesh.GetNE();++e) {
        const double value=e==11 ? 10000. : 10.*e;
        Check(std::find(sampled.samples.begin(),sampled.samples.end(),value)!=sampled.samples.end(),
              "Bounded sampling covers late materials as well as early ones");
    }
    RenderOptions selected;selected.excluded_materials={2,5,12};
    const auto filtered=EvaluateFieldRange(mesh,field,selected,{.distribution_samples=24});
    Check(filtered.min_value==0 && filtered.max_value==100 && filtered.element_count==9,
          "Excluded materials leave both extrema and the distribution");
    for(double value:filtered.samples)
        Check(value!=10 && value!=40 && value!=10000,"No excluded material contaminates zscale samples");
    for(int e=0;e<11;++e) if(e!=1 && e!=4)
        Check(std::find(filtered.samples.begin(),filtered.samples.end(),10.*e)!=filtered.samples.end(),
              "Distribution spans every retained material");
    std::cout<<"PASS bounded, deterministic whole-domain distribution sampling and exclusions\n";
}
void VectorOperations() {
    for (int dimension:{2,3}) {
        auto mesh=dimension==2 ? mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true):
                               mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
        mfem::ND_FECollection nd(2,dimension);mfem::RT_FECollection rt(1,dimension);
        mfem::H1_FECollection h1(2,dimension);
        mfem::FiniteElementSpace nd_space(&mesh,&nd),rt_space(&mesh,&rt),h1_space(&mesh,&h1,dimension);
        mfem::GridFunction circulation(&nd_space),flux(&rt_space),vector(&h1_space);
        mfem::VectorFunctionCoefficient rotation(dimension,[dimension](const mfem::Vector& p,mfem::Vector& v){
            v.SetSize(dimension);v=0.;v[0]=-p[1];v[1]=p[0];
        });
        mfem::VectorFunctionCoefficient radial(dimension,[](const mfem::Vector& p,mfem::Vector& v){v=p;});
        circulation.ProjectCoefficient(rotation);vector.ProjectCoefficient(rotation);flux.ProjectCoefficient(radial);
        RenderOptions options;options.view=FieldView::CurlMagnitude;
        for (auto* field:{&circulation,&vector}) {
            const auto range=EvaluateFieldRange(mesh,*field,options);
            Near(range.min_value,2,1e-11,"ND/H1 curl lower");Near(range.max_value,2,1e-11,"ND/H1 curl upper");
            ComparePointEvaluator(mesh,*field,options);
        }
        options.view=FieldView::CurlComponent;options.component=dimension==3?2:0;
        auto range=EvaluateFieldRange(mesh,circulation,options);Near(range.max_value,2,1e-11,"Signed curl component");
        options.view=FieldView::Divergence;options.component=0;
        range=EvaluateFieldRange(mesh,flux,options);
        Near(range.min_value,dimension,1e-11,"RT divergence lower");Near(range.max_value,dimension,1e-11,"RT divergence upper");
        ComparePointEvaluator(mesh,flux,options);
        Reject([&]{EvaluateFieldRange(mesh,circulation,options);},"Divergence");
        options.view=FieldView::CurlMagnitude;Reject([&]{EvaluateFieldRange(mesh,flux,options);},"Curl");
        options.view=FieldView::Magnitude;range=EvaluateFieldRange(mesh,flux,options);
        Near(range.min_value,0,1e-11,"Physical RT magnitude lower");Near(range.max_value,std::sqrt(dimension),1e-11,"Physical RT magnitude upper");
        ComparePointEvaluator(mesh,flux,options);
        options.view=FieldView::Value;range=EvaluateFieldRange(mesh,vector,options);
        Near(range.max_value,std::sqrt(2),1e-11,"Vector Value uses physical magnitude");
        ComparePointEvaluator(mesh,vector,options);
        options.view=FieldView::Component;range=EvaluateFieldRange(mesh,vector,options);
        Near(range.min_value,-1,1e-11,"Signed physical vector component");Near(range.max_value,0,1e-11,"Signed physical component upper");
        ComparePointEvaluator(mesh,vector,options);
        ComparePointEvaluator(mesh,circulation,options);ComparePointEvaluator(mesh,flux,options);
    }
    std::cout<<"PASS physical H1/ND/RT magnitudes, components, curl and divergence\n";
}
void OtherGeometries(const std::string& data) {
    for (auto type:{mfem::Element::TRIANGLE,mfem::Element::TETRAHEDRON,mfem::Element::WEDGE}) {
        const int dimension=type==mfem::Element::TRIANGLE?2:3;
        auto mesh=dimension==2?mfem::Mesh::MakeCartesian2D(1,1,type,true):
                              mfem::Mesh::MakeCartesian3D(1,1,1,type);
        mesh.SetCurvature(3,false,dimension);
        mfem::H1_FECollection collection(3,dimension);mfem::FiniteElementSpace space(&mesh,&collection);
        mfem::GridFunction field(&space);field=4.;
        const auto range=EvaluateFieldRange(mesh,field);
        Near(range.min_value,4,1e-11,"Simplex/prism constant lower");Near(range.max_value,4,1e-11,"Simplex/prism constant upper");
        Check(range.element_count==static_cast<std::size_t>(mesh.GetNE()),"Every simplex/prism included");
    }
    {
        mfem::Mesh pyramid((data+"/ref-pyramid.mesh").c_str(),1,1);
        mfem::H1_FECollection collection(1,3);mfem::FiniteElementSpace space(&pyramid,&collection);
        mfem::GridFunction field(&space);
        mfem::FunctionCoefficient z([](const mfem::Vector& p){return p[2];});field.ProjectCoefficient(z);
        const auto range=EvaluateFieldRange(pyramid,field);
        Near(range.min_value,0,1e-12,"Pyramid base is included");Near(range.max_value,1,1e-12,"Pyramid apex is included");
    }
    {
        auto curved=mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true);
        curved.SetCurvature(3);
        mfem::VectorFunctionCoefficient bend(2,[](const mfem::Vector& p,mfem::Vector& q){q=p;q[0]+=.2*p[0]*p[1];});
        curved.Transform(bend);
        mfem::L2_FECollection integral_collection(2,2,mfem::BasisType::GaussLegendre,mfem::FiniteElement::INTEGRAL);
        mfem::FiniteElementSpace integral_space(&curved,&integral_collection);mfem::GridFunction integral_field(&integral_space);
        mfem::FunctionCoefficient signed_field([](const mfem::Vector& p){return p[0]-2*p[1];});
        integral_field.ProjectCoefficient(signed_field);
        ComparePointEvaluator(curved,integral_field,{});
        RenderOptions magnitude;magnitude.view=FieldView::Magnitude;ComparePointEvaluator(curved,integral_field,magnitude);
    }
    mfem::Mesh rational((data+"/ball-nurbs.mesh").c_str(),1,1);
    auto* nodes=rational.GetNodes();mfem::FiniteElementSpace space(&rational,nodes->FESpace()->FEColl());
    mfem::GridFunction field(&space);
    for(int i=0;i<space.GetNDofs();++i)field[i]=(*nodes)[nodes->FESpace()->DofToVDof(i,0)];
    const auto range=EvaluateFieldRange(rational,field);
    Check(std::isfinite(range.min_value)&&std::isfinite(range.max_value)&&range.max_value>range.min_value,
          "Exact NURBS geometry and physical field evaluate throughout the domain");
    const auto again=EvaluateFieldRange(rational,field);
    Check(range.min_value==again.min_value&&range.max_value==again.max_value&&range.sample_count==again.sample_count,
          "Rational range sampling is deterministic");
    ComparePointEvaluator(rational,field,{});
    std::cout<<"PASS triangles, tetrahedra, prisms and rational NURBS volume sampling\n";
}
void ColorLimits() {
    SurfaceData data;data.positions.resize(15);data.values={-2,-1,0,1,2};
    ApplyFieldColorRange(data,{-1,1},ColorMap::Jet);
    Check(ColorAt(data,0)==ColorAt(data,1)&&ColorAt(data,3)==ColorAt(data,4),"Manual range clamps outliers");
    Check(ColorAt(data,1)==std::array<unsigned char,4>{0,0,127,255}&&
          ColorAt(data,3)==std::array<unsigned char,4>{127,0,0,255},"Jet endpoints match viewer palette");
    const auto huge=std::numeric_limits<double>::max();
    data.values={-huge,-huge/2,0,huge/2,huge};ApplyFieldColorRange(data,{-huge,huge},ColorMap::Jet);
    Check(ColorAt(data,2)==std::array<unsigned char,4>{127,255,127,255},"Extreme finite limits do not overflow");
    data.values={-1e-100,-.5e-100,0,.5e-100,1e-100};ApplyFieldColorRange(data,{-1e-100,1e-100},ColorMap::Jet);
    Check(ColorAt(data,0)!=ColorAt(data,4)&&ColorAt(data,2)==std::array<unsigned char,4>{127,255,127,255},
          "Tiny varying physical values keep their contrast");
    ApplyFieldColorRange(data,{3,3},ColorMap::Jet);
    Check(ColorAt(data,0)==std::array<unsigned char,4>{127,255,127,255},"Equal limits use the chosen palette midpoint");
    const auto before=data.colors;
    Reject([&]{ApplyFieldColorRange(data,{4,3});},"ordered");
    Check(data.colors==before&&data.min_value==3,"Invalid limits preserve the previous colors and metadata");
    data.values[2]=std::numeric_limits<double>::infinity();
    Reject([&]{ApplyFieldColorRange(data,{0,1});},"non-finite");
    Check(data.colors==before&&data.min_value==3,"Partially invalid surface never gets partial recoloring");
    SurfaceData empty;ApplyFieldColorRange(empty,{-5,5});
    Check(empty.colors.empty()&&empty.min_value==-5&&empty.max_value==5,"Empty clipped surfaces accept valid legend limits");
    std::cout<<"PASS manual/constant/extreme/tiny color limits and atomic validation\n";
}
void ColorRendering() {
    auto mesh=mfem::Mesh::MakeCartesian3D(1,1,1,mfem::Element::HEXAHEDRON);
    mfem::H1_FECollection collection(1,3);mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient ramp([](const mfem::Vector& p){return p[0]+2*p[1]+3*p[2];});
    field.ProjectCoefficient(ramp);
    RenderOptions options;options.refinement=2;
    const auto original=BuildSurface(mesh,&field,options);
    const auto clipped=BuildClippedSurface(mesh,&field,options,{{.5f,0,0},{1,0,0}}).surface;
    auto same_data=[](const SurfaceData& a,const SurfaceData& b) {
        return a.positions==b.positions&&a.normals==b.normals&&a.values==b.values&&
               a.material_ids==b.material_ids&&a.boundary_ids==b.boundary_ids;
    };
    auto mapped=[](const SurfaceData& data,ColorMap map,bool reverse) {
        for (std::size_t i=0;i<data.values.size();++i) {
            const auto color=FieldColor(data.values[i],data.min_value,data.max_value,map,reverse);
            Check(ColorAt(data,i)==std::array<unsigned char,4>{color.r,color.g,color.b,color.a},
                  "Every rendered vertex uses the chosen map and direction");
        }
    };
    for (int i=0;i<ColorMapCount;++i) for (bool reverse:{false,true}) {
        options.color_map=static_cast<ColorMap>(i);options.reverse_colors=reverse;
        const auto surface=BuildSurface(mesh,&field,options);
        const auto cut=BuildClippedSurface(mesh,&field,options,{{.5f,0,0},{1,0,0}}).surface;
        Check(same_data(surface,original)&&same_data(cut,clipped),
              "Palette choice preserves exterior and clipped geometry, field samples and IDs");
        mapped(surface,options.color_map,reverse);mapped(cut,options.color_map,reverse);
        auto recolored=original;
        ApplyFieldColorRange(recolored,{-3,9},options.color_map,reverse);
        Check(same_data(recolored,original),"Changing palette and limits preserves scientific samples");
        mapped(recolored,options.color_map,reverse);
        Check(recolored.min_value==-3&&recolored.max_value==9,"Reversal preserves numeric legend limits");
    }
    auto scaled=[](const SurfaceData& data,const ColorScaleMapping& mapping,ColorMap map,bool reverse) {
        Check(data.min_value==mapping.minimum && data.max_value==mapping.maximum,
              "Surface metadata exposes the resolved display limits");
        for(std::size_t i=0;i<data.values.size();++i) {
            const auto color=FieldColor(data.values[i],mapping,map,reverse);
            Check(ColorAt(data,i)==std::array<unsigned char,4>{color.r,color.g,color.b,color.a},
                  "Rendered colors use the resolved scale, palette and direction");
        }
    };
    for(auto scale:{ColorScale::Linear,ColorScale::SignedLog,ColorScale::ZScale}) {
        options.color_scale={.scale=scale,.log_threshold=.1};
        options.color_map=ColorMap::CoolWarm;options.reverse_colors=true;
        const auto surface=BuildSurface(mesh,&field,options);
        const auto cut=BuildClippedSurface(mesh,&field,options,{{.5f,0,0},{1,0,0}}).surface;
        Check(same_data(surface,original)&&same_data(cut,clipped),
              "Scale choice preserves exterior and clipped geometry, field values and IDs");
        auto expected=[&](const SurfaceData& data) {
            const auto [low,high]=std::minmax_element(data.values.begin(),data.values.end());
            return ResolveColorScale(*low,*high,options.color_scale,data.values);
        };
        scaled(surface,expected(surface),options.color_map,true);
        scaled(cut,expected(cut),options.color_map,true);
        auto recolored=original;
        const auto range=EvaluateFieldRange(mesh,field,options,{.distribution_samples=1000});
        const auto mapping=ResolveColorScale(range.min_value,range.max_value,options.color_scale,range.samples);
        ApplyFieldColorRange(recolored,mapping,options.color_map,true);
        Check(same_data(recolored,original),"Scaled recoloring preserves scientific and geometry data");
        scaled(recolored,mapping,options.color_map,true);
        SurfaceData empty;ApplyFieldColorRange(empty,mapping);
        Check(empty.colors.empty() && empty.min_value==mapping.minimum && empty.max_value==mapping.maximum,
              "Scaled empty clipping keeps the shared volume legend limits");
    }
    SurfaceData signed_data;signed_data.positions.resize(15);signed_data.values={-100,-1,0,1,100};
    const auto signed_mapping=ResolveColorScale(-100,100,{.scale=ColorScale::SignedLog,.log_threshold=1});
    ApplyFieldColorRange(signed_data,signed_mapping,ColorMap::CoolWarm);
    scaled(signed_data,signed_mapping,ColorMap::CoolWarm,false);
    const auto signed_original=signed_data;
    auto invalid=signed_mapping;invalid.maximum=-101;
    Reject([&]{ApplyFieldColorRange(signed_data,invalid);},"ordered");
    Check(signed_data.colors==signed_original.colors && signed_data.min_value==signed_original.min_value,
          "Invalid mapping leaves existing colors and legend intact");
    options.coloring=SurfaceColoring::Material;
    const auto material=BuildSurface(mesh,nullptr,options);
    const auto cut_material=BuildClippedSurface(mesh,nullptr,options,{{.5f,0,0},{1,0,0}}).surface;
    options.color_map=ColorMap::Viridis;options.reverse_colors=false;
    options.color_scale={};
    Check(BuildSurface(mesh,nullptr,options).colors==material.colors,
          "Field scales and schemes do not alter categorical material colors");
    Check(BuildClippedSurface(mesh,nullptr,options,{{.5f,0,0},{1,0,0}}).surface.colors==cut_material.colors,
          "Field scales do not alter categorical clipped material colors");
    std::cout<<"PASS palettes, scales and reversal across surfaces, clipped caps and recoloring\n";
}
void ZScaleRendering() {
    auto mesh=mfem::Mesh::MakeCartesian2D(20,1,mfem::Element::QUADRILATERAL,true);
    mfem::L2_FECollection collection(0,2);mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);mfem::Array<int> dofs;
    for(int e=0;e<mesh.GetNE();++e) {
        space.GetElementDofs(e,dofs);field[dofs[0]]=e==19 ? 10000. : double(e);
    }
    RenderOptions options;
    const auto linear=BuildSurface(mesh,&field,options);
    options.color_scale.scale=ColorScale::ZScale;
    const auto robust=BuildSurface(mesh,&field,options);
    const auto cut=BuildClippedSurface(mesh,&field,options,{{2,0,0},{1,0,0}}).surface;
    Check(linear.max_value==10000 && robust.max_value<1000 && cut.max_value<1000,
          "Zscale surface and clipping limits reject a sparse bright outlier population");
    Check(robust.values==linear.values && robust.positions==linear.positions && robust.colors!=linear.colors,
          "Zscale changes display contrast while preserving the outlier's scientific value and geometry");
    const auto full=EvaluateFieldRange(mesh,field,options,{.distribution_samples=1000});
    const auto mapping=ResolveColorScale(full.min_value,full.max_value,options.color_scale,full.samples);
    auto recolored=linear;ApplyFieldColorRange(recolored,mapping);
    Check(full.max_value==10000 && mapping.maximum<1000 && recolored.max_value==mapping.maximum,
          "Whole-domain extrema remain available alongside robust display limits");
    std::cout<<"PASS robust surface/clipping display ranges with retained scientific extrema\n";
}
void VariableOrderAndOrdering() {
    auto mesh=mfem::Mesh::MakeCartesian2D(2,1,mfem::Element::QUADRILATERAL,true);
    mesh.EnsureNCMesh();
    for(auto ordering:{mfem::Ordering::byNODES,mfem::Ordering::byVDIM}) {
        mfem::H1_FECollection collection(2,2);
        mfem::FiniteElementSpace space(&mesh,&collection,2,ordering);
        space.SetElementOrder(1,4);space.Update(false);
        mfem::GridFunction field(&space);
        mfem::VectorFunctionCoefficient coefficient(2,[](const mfem::Vector& p,mfem::Vector& v){
            v.SetSize(2);v[0]=p[0]-2*p[1];v[1]=p[0]+3*p[1];
        });field.ProjectCoefficient(coefficient);
        RenderOptions options;options.view=FieldView::Component;
        for(int component:{0,1}) {options.component=component;ComparePointEvaluator(mesh,field,options);}
        options.view=FieldView::Magnitude;ComparePointEvaluator(mesh,field,options);
        const auto range=EvaluateFieldRange(mesh,field);
        Check(range.min_subdivisions==6&&range.max_subdivisions==10,
              "Variable-order parents retain their own sampling resolutions");
    }
    std::cout<<"PASS variable polynomial order and both nodal vector orderings\n";
}
void Benchmark(const std::string& path) {
    mfem::Mesh mesh(path.c_str(),1,1);mfem::H1_FECollection collection(5,mesh.Dimension());
    mfem::FiniteElementSpace space(&mesh,&collection);mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& p){return p[0]+2*p[1]-p[2];});field.ProjectCoefficient(coefficient);
    const auto start=std::chrono::steady_clock::now();const auto range=EvaluateFieldRange(mesh,field);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"BENCH "<<mesh.GetNE()<<" cells, "<<range.sample_count<<" samples, subdivisions "<<range.min_subdivisions
             <<".."<<range.max_subdivisions<<", "<<ms<<" ms, ["<<range.min_value<<", "<<range.max_value<<"]\n";
}
void ArchiveBenchmark(const std::string& path) {
    const auto loaded=LoadSerifFile(path);Check(bool(loaded),loaded.error);
    for(const auto& frame:loaded.frames)for(const auto& field:frame->fields) {
        RenderOptions options;ExcludeUnsupportedMaterials(field,*frame->mesh,options.excluded_materials);
        const auto surface=BuildSurface(*frame->mesh,field.gf.get(),options);
        const auto start=std::chrono::steady_clock::now();
        const auto range=EvaluateFieldRange(*frame->mesh,*field.gf,options);
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::cout<<"ARCHIVE "<<field.name<<", "<<range.sample_count<<" samples, "<<ms<<" ms, surface ["
                 <<surface.min_value<<", "<<surface.max_value<<"], volume ["<<range.min_value<<", "<<range.max_value<<"]\n";
    }
}
}
int main(int argc,char** argv) {try {
    InteriorPeak();MaterialsAndValidation();DistributionSampling();VectorOperations();
    OtherGeometries(argc>1?argv[1]:"subprojects/mfem/data");ColorLimits();ColorRendering();ZScaleRendering();VariableOrderAndOrdering();
    if(argc>2)Benchmark(argv[2]);
    if(argc>3)ArchiveBenchmark(argv[3]);
    std::cout<<"All field range tests passed\n";return 0;
} catch(const std::exception& e) {std::cerr<<"Field range test: "<<e.what()<<'\n';return 1;}}
