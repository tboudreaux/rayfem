#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/explosion.hpp"
#include "surface_normal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace {
using RayFEM::LineSegment;
using RayFEM::RenderOptions;
using RayFEM::SurfaceData;
Vector3 add(Vector3 a, Vector3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vector3 sub(Vector3 a, Vector3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vector3 scale(Vector3 a, double s) { return {float(a.x*s),float(a.y*s),float(a.z*s)}; }
double dot(Vector3 a, Vector3 b) { return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z; }
Vector3 cross(Vector3 a, Vector3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
double norm(Vector3 v) { return std::sqrt(dot(v,v)); }
Vector3 unit(Vector3 v) { const double n=norm(v); return n>0?scale(v,1/n):Vector3{}; }
mfem::IntegrationPoint mixIP(const mfem::IntegrationPoint& a,const mfem::IntegrationPoint& b,double t) {
    mfem::IntegrationPoint p; p.Set3(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t); return p;
}
struct Sample { Vector3 p; mfem::IntegrationPoint ip; double value; int material=0,boundary=0; Vector3 normal{}; };
struct Plane {
    Vector3 p,n; double eps;
    double distance(Vector3 x) const { return dot(sub(x,p),n); }
};
void validate(mfem::Mesh& mesh,mfem::GridFunction* field,const RenderOptions& o) {
    RayFEM::ValidateElementExplosion(o);
    RayFEM::ValidateElementSelection(mesh,o);
    if(mesh.Dimension()<2 || mesh.Dimension()>3 || mesh.SpaceDimension()>3) throw std::invalid_argument("Clipping requires a 2D or 3D mesh");
    for(auto* gf:{field,o.displacement}) if(gf && (!gf->FESpace() || gf->FESpace()->GetMesh()!=&mesh)) throw std::invalid_argument("Clipping field must use the displayed mesh");
    if(o.displacement && (o.displacement->VectorDim()!=mesh.SpaceDimension() || o.displacement->FESpace()->FEColl()->GetRangeType(mesh.Dimension())!=mfem::FiniteElement::SCALAR)) throw std::invalid_argument("Displacement must be a pointwise spatial vector field");
    if(!std::isfinite(o.displacement_scale)) throw std::invalid_argument("Displacement scale must be finite");
}
using DisplayTransforms = std::vector<RayFEM::ElementDisplayTransform>;
DisplayTransforms displayTransforms(mfem::Mesh& mesh,const RenderOptions& o,const RayFEM::ElementSelection& selection) {
    DisplayTransforms result(mesh.GetNE());
    if(o.element_explosion>0)for(int e=0;e<mesh.GetNE();++e)
        if(selection.Includes(e))
            result[e]=RayFEM::MakeElementDisplayTransform(*mesh.GetElementTransformation(e),o);
    return result;
}
Sample sample(mfem::Mesh& mesh,mfem::GridFunction* field,int e,const mfem::IntegrationPoint& ip,const RenderOptions& o,
              const DisplayTransforms& display) {
    auto* transform=mesh.GetElementTransformation(e);
    mfem::Vector x; transform->Transform(ip,x);
    if(o.displacement) { mfem::Vector d; transform->SetIntPoint(&ip); o.displacement->GetVectorValue(*transform,ip,d); x.Add(o.displacement_scale,d); }
    for(int i=0;i<x.Size();++i) if(!std::isfinite(x[i])) throw std::runtime_error("Non-finite clipping geometry");
    return {display[e].Apply({float(x[0]),float(x[1]),x.Size()>2?float(x[2]):0}),ip,
            field?RayFEM::EvaluateField(*field,e,ip,o.view,o.component):0,mesh.GetAttribute(e),0};
}
Sample interpolate(const Sample& a,const Sample& b,double t,mfem::GridFunction* field,int e,const RenderOptions& o) {
    const auto ip=mixIP(a.ip,b.ip,t);
    return {add(a.p,scale(sub(b.p,a.p),t)),ip,field?RayFEM::EvaluateField(*field,e,ip,o.view,o.component):0,a.material,a.boundary,
            unit(add(scale(a.normal,1-t),scale(b.normal,t)))};
}
void triangle(SurfaceData& out,const Sample& a,const Sample& b,const Sample& c,int element,bool reverse=false) {
    Vector3 n=cross(sub(b.p,a.p),sub(c.p,a.p));
    if(norm(n)<std::numeric_limits<float>::min()) return;
    out.element_ids.push_back(element);
    n=unit(n); if(reverse)n=scale(n,-1);
    const Sample* points[3]={&a,reverse?&c:&b,reverse?&b:&c};
    for(auto* s:points) {
        Vector3 vertex_normal=norm(s->normal)>0?scale(s->normal,reverse?-1:1):n;
        out.positions.insert(out.positions.end(),{s->p.x,s->p.y,s->p.z}); out.normals.insert(out.normals.end(),{vertex_normal.x,vertex_normal.y,vertex_normal.z}); out.values.push_back(s->value);out.material_ids.push_back(s->material);out.boundary_ids.push_back(s->boundary);
    }
}
std::vector<Sample> clipped(std::vector<Sample> polygon,const Plane& plane,mfem::GridFunction* field,int e,const RenderOptions& o) {
    std::vector<Sample> out;
    for(std::size_t i=0;i<polygon.size();++i) {
        const auto& a=polygon[i]; const auto& b=polygon[(i+1)%polygon.size()];
        const double da=plane.distance(a.p),db=plane.distance(b.p);
        const bool ina=da<=plane.eps,inb=db<=plane.eps;
        if(ina)out.push_back(a);
        if(ina!=inb && std::abs(da-db)>plane.eps)out.push_back(interpolate(a,b,std::clamp(da/(da-db),0.0,1.0),field,e,o));
    }
    return out;
}
void finish(SurfaceData& out,bool field,const RenderOptions& o) {
    if(out.values.empty())return;
    auto [lo,hi]=std::minmax_element(out.values.begin(),out.values.end()); out.min_value=*lo;out.max_value=*hi;
    const bool field_colors=field && o.coloring==RayFEM::SurfaceColoring::Field;
    const auto mapping=field_colors ? RayFEM::ResolveColorScale(*lo,*hi,o.color_scale,out.values) : RayFEM::ColorScaleMapping{};
    if(field_colors){out.min_value=mapping.minimum;out.max_value=mapping.maximum;}
    out.bounds.min=out.bounds.max={out.positions[0],out.positions[1],out.positions[2]};
    for(std::size_t i=0;i<out.values.size();++i) {
        Color c=field_colors?RayFEM::FieldColor(out.values[i],mapping,o.color_map,o.reverse_colors):Color{155,195,220,255};
        if(o.coloring==RayFEM::SurfaceColoring::Material)c=RayFEM::AttributeColor(out.material_ids[i]);
        if(o.coloring==RayFEM::SurfaceColoring::Boundary)c=RayFEM::AttributeColor(out.boundary_ids[i]);
        out.colors.insert(out.colors.end(),{c.r,c.g,c.b,c.a});
        auto& l=out.bounds.min;auto& h=out.bounds.max;
        l.x=std::min(l.x,out.positions[3*i]);h.x=std::max(h.x,out.positions[3*i]);
        l.y=std::min(l.y,out.positions[3*i+1]);h.y=std::max(h.y,out.positions[3*i+1]);
        l.z=std::min(l.z,out.positions[3*i+2]);h.z=std::max(h.z,out.positions[3*i+2]);
    }
}
template<int N> std::vector<std::pair<int,int>> edgeList(const int (&edges)[N][2]) {
    std::vector<std::pair<int,int>> out; for(auto& edge:edges)out.emplace_back(edge[0],edge[1]);return out;
}
std::vector<std::pair<int,int>> edges(mfem::Geometry::Type g) {
    switch(g) {
    case mfem::Geometry::TRIANGLE:return edgeList(mfem::Geometry::Constants<mfem::Geometry::TRIANGLE>::Edges);
    case mfem::Geometry::SQUARE:return edgeList(mfem::Geometry::Constants<mfem::Geometry::SQUARE>::Edges);
    case mfem::Geometry::TETRAHEDRON:return edgeList(mfem::Geometry::Constants<mfem::Geometry::TETRAHEDRON>::Edges);
    case mfem::Geometry::CUBE:return edgeList(mfem::Geometry::Constants<mfem::Geometry::CUBE>::Edges);
    case mfem::Geometry::PRISM:return edgeList(mfem::Geometry::Constants<mfem::Geometry::PRISM>::Edges);
    case mfem::Geometry::PYRAMID:return edgeList(mfem::Geometry::Constants<mfem::Geometry::PYRAMID>::Edges);
    default:throw std::invalid_argument("Unsupported clipping cell geometry");
    }
}
// A face is evaluated through its owning volume element, preserving RT/ND
// orientation and the interior trace of discontinuous fields.
template<typename Callback> void patches(mfem::Mesh& mesh,const RenderOptions& o,const std::vector<bool>* retained,
                                        const RayFEM::ElementSelection& selection,Callback callback,bool all_faces=false) {
    auto selected=[&](int e){return selection.Includes(e)&&(!retained||(*retained)[e]);};
    if(mesh.Dimension()==2) {
        for(int e=0;e<mesh.GetNE();++e)if(selected(e))callback(e,mesh.GetElementBaseGeometry(e),nullptr,false,e);
        return;
    }
    for(int f=0;f<mesh.GetNumFaces();++f) {
        const auto info=mesh.GetFaceInformation(f);
        if(info.IsNonconformingCoarse())continue;
        // Only the reference maps are needed here. Physical NURBS transforms
        // must be loaded one owning element at a time by the sample callback.
        auto* tr=mesh.GetFaceElementTransformations(f,
            mfem::FaceElementTransformations::HAVE_LOC1 |
            mfem::FaceElementTransformations::HAVE_LOC2);
        if(!tr||tr->Elem1No<0)continue;
        const bool a=selected(tr->Elem1No),b=selected(tr->Elem2No);
        const bool exploded=o.element_explosion>0;
        if(!all_faces && !exploded && a==b)continue;
        if(a)callback(tr->Elem1No,mesh.GetFaceGeometry(f),&tr->Loc1,false,f);
        if(b && (!a || exploded))callback(tr->Elem2No,mesh.GetFaceGeometry(f),&tr->Loc2,true,f);
    }
}
std::vector<LineSegment> boundaries(mfem::Mesh& mesh,const RenderOptions& o,const std::vector<bool>* retained,
                                   const DisplayTransforms& display,const RayFEM::ElementSelection& selection) {
    std::vector<LineSegment> out;std::set<std::tuple<int,int,int>> seen;
    const int n=std::clamp(o.refinement,1,64);
    if(mesh.Dimension()==3 && o.element_explosion>0) {
        // Every exploded cell is visible. Enumerate its original reference
        // edges directly: NC fine-face patches subdivide a coarse face and
        // their internal edges must not become false coarse-cell boundaries.
        std::map<std::pair<int,int>,int> boundary_edges;
        mfem::Array<int> vertices;
        for(int b=0;b<mesh.GetNBE();++b) {
            const int attribute=mesh.GetBdrAttribute(b);
            if(attribute<=0)continue;
            mesh.GetBdrElementVertices(b,vertices);
            for(auto [a,c]:edges(mesh.GetBdrElementBaseGeometry(b))) {
                const auto key=std::minmax(vertices[a],vertices[c]);
                boundary_edges.try_emplace({key.first,key.second},attribute);
            }
        }
        for(int e=0;e<mesh.GetNE();++e) {
            if(!selection.Includes(e) || (retained && !(*retained)[e]))continue;
            const auto geometry=mesh.GetElementBaseGeometry(e);
            const auto* refs=mfem::Geometries.GetVertices(geometry);
            mesh.GetElementVertices(e,vertices);
            for(auto [a,b]:edges(geometry)) {
                const auto key=std::minmax(vertices[a],vertices[b]);
                const auto found=boundary_edges.find({key.first,key.second});
                const int attribute=found==boundary_edges.end()?0:found->second;
                Vector3 previous{};
                for(int k=0;k<=n;++k) {
                    const auto ip=mixIP(refs->IntPoint(a),refs->IntPoint(b),double(k)/n);
                    const auto p=sample(mesh,nullptr,e,ip,o,display).p;
                    if(k)out.push_back({previous,p,mesh.GetAttribute(e),attribute});
                    previous=p;
                }
            }
        }
        return out;
    }
    const auto boundary_attributes=RayFEM::FaceBoundaryAttributes(mesh);
    patches(mesh,o,retained,selection,[&](int e,mfem::Geometry::Type g,mfem::IntegrationPointTransformation* map,bool,int id) {
        mfem::Array<int> vertices;
        if(mesh.Dimension()==2)mesh.GetElementVertices(id,vertices);else mesh.GetFaceVertices(id,vertices);
        mfem::Array<int> element_edges,orientations;
        if(mesh.Dimension()==2)mesh.GetElementEdges(e,element_edges,orientations);
        const auto* refs=mfem::Geometries.GetVertices(g);
        int local_edge=0;
        for(auto [a,b]:edges(g)) {
            const int boundary=boundary_attributes[mesh.Dimension()==2?element_edges[local_edge++]:id];
            auto key=std::minmax(vertices[a],vertices[b]);
            // Shared topological edges split into one visible copy per element.
            if(!seen.emplace(o.element_explosion>0?e:-1,key.first,key.second).second)continue;
            Vector3 previous{};
            for(int k=0;k<=n;++k) {
                auto ip=mixIP(refs->IntPoint(a),refs->IntPoint(b),double(k)/n);
                if(map){auto local=ip;map->Transform(local,ip);}
                const auto p=sample(mesh,nullptr,e,ip,o,display).p;
                if(k)out.push_back({previous,p,mesh.GetAttribute(e),boundary});previous=p;
            }
        }
    });return out;
}
void intersectPolygon(const std::vector<Sample>& polygon,const Plane& plane,std::vector<LineSegment>& lines) {
    std::vector<Vector3> points;
    auto add_point=[&](Vector3 p){for(auto q:points)if(norm(sub(p,q))<=plane.eps*4)return;points.push_back(p);};
    for(std::size_t i=0;i<polygon.size();++i) {
        auto a=polygon[i].p,b=polygon[(i+1)%polygon.size()].p;
        double da=plane.distance(a),db=plane.distance(b);
        if(std::abs(da)<=plane.eps)add_point(a);
        if((da< -plane.eps&&db>plane.eps)||(da>plane.eps&&db< -plane.eps))add_point(add(a,scale(sub(b,a),da/(da-db))));
    }
    if(points.size()==2 && norm(sub(points[0],points[1]))>plane.eps)lines.push_back({points[0],points[1],polygon[0].material,0});
}
} // namespace

namespace RayFEM {
std::vector<LineSegment> BuildElementBoundaries(mfem::Mesh& mesh,const RenderOptions& options) {
    validate(mesh,nullptr,options);
    const ElementSelection selection(mesh,options);
    const auto display=displayTransforms(mesh,options,selection);
    return boundaries(mesh,options,nullptr,display,selection);
}
ClippedSurface BuildClippedSurface(mfem::Mesh& mesh,mfem::GridFunction* field,const RenderOptions& options,
                                   const ClipPlane& input,ClipMode mode,CutAlgorithm algorithm) {
    if(options.coloring!=SurfaceColoring::Field)field=nullptr;
    validate(mesh,field,options);
    const ElementSelection selection(mesh,options);
    const auto display=displayTransforms(mesh,options,selection);
    if(!std::isfinite(norm(input.normal))||norm(input.normal)<1e-20 || !std::isfinite(input.point.x)||!std::isfinite(input.point.y)||!std::isfinite(input.point.z))throw std::invalid_argument("Clipping plane needs finite coordinates and a nonzero normal");
    if(field)for(int e=0;e<mesh.GetNE();++e)if(selection.Includes(e)){EvaluateField(*field,e,mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e)),options.view,options.component);break;}
    mfem::Vector lower,upper;mesh.GetBoundingBox(lower,upper,2);
    double extent=0;for(int d=0;d<lower.Size();++d)extent=std::max(extent,std::abs(upper[d]-lower[d]));
    const Plane plane{input.point,unit(input.normal),std::max(1e-20,extent*1e-7)};
    ClippedSurface result;
    const int refinement=std::clamp(options.refinement,1,24);
    std::vector<bool> retained(mesh.GetNE(),false);
    for(int e=0;e<mesh.GetNE();++e)retained[e]=selection.Includes(e);
    if(mode==ClipMode::WholeElements)for(int e=0;e<mesh.GetNE();++e)if(retained[e]) {
        const auto* vertices=mfem::Geometries.GetVertices(mesh.GetElementBaseGeometry(e));
        for(int i=0;i<vertices->GetNPoints();++i)if(plane.distance(sample(mesh,nullptr,e,vertices->IntPoint(i),options,display).p)>plane.eps){retained[e]=false;break;}
    }
    mfem::GeometryRefiner refiner;
    const auto boundary_attributes=FaceBoundaryAttributes(mesh);
    patches(mesh,options,&retained,selection,[&](int e,mfem::Geometry::Type g,mfem::IntegrationPointTransformation* map,bool reverse,int face) {
        const auto* rg=refiner.Refine(g,refinement,1);
        std::vector<Sample> points;
        for(int i=0;i<rg->RefPts.GetNPoints();++i) {
            auto ip=rg->RefPts.IntPoint(i);if(map){auto local=ip;map->Transform(local,ip);}points.push_back(sample(mesh,field,e,ip,options,display));
            points.back().boundary=mesh.Dimension()==3?boundary_attributes[face]:0;
            points.back().normal=detail::SurfaceNormal(*mesh.GetElementTransformation(e),ip,options,map,&rg->RefPts.IntPoint(i));
        }
        const int stride=mfem::Geometry::NumVerts[g];
        for(int j=0;j<rg->RefGeoms.Size();j+=stride)for(int tri=0;tri<(stride==4?2:1);++tri) {
            std::vector<Sample> poly{points[rg->RefGeoms[j]],points[rg->RefGeoms[j+(tri?2:1)]],points[rg->RefGeoms[j+(tri?3:2)]]};
            if(mode==ClipMode::CutThrough)poly=clipped(std::move(poly),plane,field,e,options);
            for(std::size_t k=1;k+1<poly.size();++k)triangle(result.surface,poly[0],poly[k],poly[k+1],e,reverse);
        }
    });
    result.element_boundaries=boundaries(mesh,options,&retained,display,selection);
    if(mode==ClipMode::CutThrough) {
        std::vector<LineSegment> kept;
        for(auto line:result.element_boundaries) {
            const double a=plane.distance(line.a),b=plane.distance(line.b);
            if(a>plane.eps && b>plane.eps)continue;
            if((a>plane.eps)!=(b>plane.eps)){auto p=add(line.a,scale(sub(line.b,line.a),a/(a-b)));if(a>plane.eps)line.a=p;else line.b=p;}
            kept.push_back(line);
        }
        result.element_boundaries=std::move(kept);
    }
    if(mode==ClipMode::CutThrough && mesh.Dimension()==3) {
        std::size_t sampled_points=0;
        Vector3 u=unit(cross(plane.n,std::abs(plane.n.y)<0.9?Vector3{0,1,0}:Vector3{1,0,0}));
        Vector3 v=cross(plane.n,u);
        for(int e=0;e<mesh.GetNE();++e) {
            if(!retained[e])continue;
            const auto g=mesh.GetElementBaseGeometry(e);
            const auto* rg=refiner.Refine(g,algorithm==CutAlgorithm::Refined?refinement:1,1);
            sampled_points+=rg->RefPts.GetNPoints();
            if(sampled_points>2000000)throw std::runtime_error("Clipping sample budget exceeded; reduce refinement");
            std::vector<Sample> points;
            for(int i=0;i<rg->RefPts.GetNPoints();++i)points.push_back(sample(mesh,field,e,rg->RefPts.IntPoint(i),options,display));
            const int stride=mfem::Geometry::NumVerts[g];
            for(int offset=0;offset<rg->RefGeoms.Size();offset+=stride) {
                const bool tetra=g==mfem::Geometry::PYRAMID && rg->RefGeoms[offset+4]<0;
                const int count=tetra?4:stride;
                const auto cell_edges=edges(tetra?mfem::Geometry::TETRAHEDRON:g);
                double lo=std::numeric_limits<double>::infinity(),hi=-lo;
                for(int i=0;i<count;++i){double d=plane.distance(points[rg->RefGeoms[offset+i]].p);lo=std::min(lo,d);hi=std::max(hi,d);}
                // Half-open ownership prevents duplicate caps along subcell faces.
                if(lo>=-plane.eps || hi< -plane.eps)continue;
                std::vector<Sample> polygon;
                auto unique=[&](Sample s){for(const auto& p:polygon)if(norm(sub(p.p,s.p))<plane.eps*4)return;polygon.push_back(s);};
                for(auto [ia,ib]:cell_edges) {
                    const auto& a=points[rg->RefGeoms[offset+ia]];const auto& b=points[rg->RefGeoms[offset+ib]];
                    const double da=plane.distance(a.p),db=plane.distance(b.p);
                    if(std::abs(da)<=plane.eps)unique(a);
                    if(std::abs(db)<=plane.eps)unique(b);
                    if((da< -plane.eps&&db>plane.eps)||(da>plane.eps&&db< -plane.eps))unique(interpolate(a,b,da/(da-db),field,e,options));
                }
                if(polygon.size()<3)continue;
                Vector3 center{};for(const auto& p:polygon)center=add(center,p.p);center=scale(center,1.0/polygon.size());
                std::sort(polygon.begin(),polygon.end(),[&](const auto& a,const auto& b){return std::atan2(dot(sub(a.p,center),v),dot(sub(a.p,center),u))<std::atan2(dot(sub(b.p,center),v),dot(sub(b.p,center),u));});
                for(std::size_t k=1;k+1<polygon.size();++k) {
                    const auto& a=polygon[0];const auto& b=polygon[k];const auto& c=polygon[k+1];
                    if(algorithm==CutAlgorithm::Refined){triangle(result.surface,a,b,c,e);continue;}
                    const auto* tri=refiner.Refine(mfem::Geometry::TRIANGLE,refinement,1);
                    std::vector<Sample> samples;
                    for(int j=0;j<tri->RefPts.GetNPoints();++j) {
                        const auto& q=tri->RefPts.IntPoint(j);
                        Sample s=interpolate(a,b,q.x,field,e,options);
                        s.p=add(a.p,add(scale(sub(b.p,a.p),q.x),scale(sub(c.p,a.p),q.y)));
                        s.ip.Set3(a.ip.x+(b.ip.x-a.ip.x)*q.x+(c.ip.x-a.ip.x)*q.y,a.ip.y+(b.ip.y-a.ip.y)*q.x+(c.ip.y-a.ip.y)*q.y,a.ip.z+(b.ip.z-a.ip.z)*q.x+(c.ip.z-a.ip.z)*q.y);
                        s.value=field?EvaluateField(*field,e,s.ip,options.view,options.component):0;samples.push_back(s);
                    }
                    for(int j=0;j<tri->RefGeoms.Size();j+=3)triangle(result.surface,samples[tri->RefGeoms[j]],samples[tri->RefGeoms[j+1]],samples[tri->RefGeoms[j+2]],e);
                }
            }
        }
        // Intersections with original element faces outline cut elements without
        // drawing the internal diagonals used to triangulate the section itself.
        patches(mesh,options,&retained,selection,[&](int e,mfem::Geometry::Type g,mfem::IntegrationPointTransformation* map,bool,int) {
            const auto* rg=refiner.Refine(g,algorithm==CutAlgorithm::Refined?refinement:1,1);
            std::vector<Sample> points;
            for(int i=0;i<rg->RefPts.GetNPoints();++i){auto ip=rg->RefPts.IntPoint(i);auto local=ip;map->Transform(local,ip);points.push_back(sample(mesh,nullptr,e,ip,options,display));}
            const int stride=mfem::Geometry::NumVerts[g];
            for(int j=0;j<rg->RefGeoms.Size();j+=stride){std::vector<Sample> polygon;for(int k=0;k<stride;++k)polygon.push_back(points[rg->RefGeoms[j+k]]);intersectPolygon(polygon,plane,result.element_boundaries);}
        },true);
    }
    if(mode==ClipMode::WholeElements)result.retained_elements=std::move(retained);
    finish(result.surface,field!=nullptr,options);return result;
}
} // namespace RayFEM
