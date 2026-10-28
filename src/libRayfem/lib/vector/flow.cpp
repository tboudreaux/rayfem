#include "libRayfem/vector/flow.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>

namespace {
using namespace RayFEM;
using P=ProfilePosition;
P add(P a,P b){for(int d=0;d<3;++d)a[d]+=b[d];return a;}
P sub(P a,P b){for(int d=0;d<3;++d)a[d]-=b[d];return a;}
P scale(P a,double s){for(double& x:a)x*=s;return a;}
double norm(P a){return std::hypot(a[0],a[1],a[2]);}
double dot(P a,P b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
bool finite(P a){return std::all_of(a.begin(),a.end(),[](double v){return std::isfinite(v);});}
struct Bounds {
    P lo{INFINITY,INFINITY,INFINITY},hi{-INFINITY,-INFINITY,-INFINITY};
    void include(P p){for(int d=0;d<3;++d){lo[d]=std::min(lo[d],p[d]);hi[d]=std::max(hi[d],p[d]);}}
    void include(const Bounds& b){include(b.lo);include(b.hi);}
    bool contains(P p)const{for(int d=0;d<3;++d)if(p[d]<lo[d]||p[d]>hi[d])return false;return true;}
    double distance(P p)const{double s=0;for(int d=0;d<3;++d){double x=std::max({0.,lo[d]-p[d],p[d]-hi[d]});s+=x*x;}return s;}
};
struct Box {Bounds bounds;int element;double cumulative_weight=0;};
struct Node {Bounds bounds;int begin=0,end=0,left=-1,right=-1;};
struct Located {FlowPoint point;mfem::IntegrationPoint reference;};

class Sampler {
public:
    Sampler(mfem::Mesh& mesh,mfem::GridFunction& field,const RenderOptions& render,
            const FlowOptions& options,FlowDiagnostics& diagnostics)
        :original(mesh),field(field),options(options),diag(diagnostics) {
        ValidateElementSelection(mesh,render);
        const ElementSelection selection(mesh,render);
        if(mesh.Dimension()<2||mesh.Dimension()>3||mesh.SpaceDimension()>3||
           !field.FESpace()||field.FESpace()->GetMesh()!=&mesh||field.VectorDim()!=mesh.SpaceDimension())
            throw std::invalid_argument("Flow requires a spatial vector field on a 2D or 3D mesh");
        if(options.seed_count<0 || (options.seed_count==0&&(options.seed_resolution<1||options.seed_resolution>64))||options.max_seeds<1||options.max_seeds>20000||
           options.max_points<1||options.max_points>1000000||options.max_steps<1||options.max_steps>10000||
           options.max_evaluations<1||options.max_evaluations>5000000||options.max_location_tests<1||options.max_location_tests>10000000||
           !std::isfinite(options.step_size)||options.step_size<=0||!std::isfinite(options.max_length)||options.max_length<=0)
            throw std::invalid_argument("Flow needs positive finite lengths and bounded seed, point, step and evaluation limits");
        if(!options.retained_elements.empty()&&options.retained_elements.size()!=std::size_t(mesh.GetNE()))
            throw std::invalid_argument("Flow selection must have one flag per element");
        if(mesh.GetNE()>250000){diag.budget_exhausted=true;return;}
        if(options.clip_plane){const auto& plane=*options.clip_plane;clip_origin={plane.point.x,plane.point.y,plane.point.z};clip_normal={plane.normal.x,plane.normal.y,plane.normal.z};if(!finite(clip_origin)||!finite(clip_normal)||norm(clip_normal)==0)throw std::invalid_argument("Flow clipping plane must be finite and nonzero");clip_normal=scale(clip_normal,1/norm(clip_normal));}
        if(stopped())return;
        displaced=MakeDisplacedMesh(mesh,render);displayed=displaced?displaced.get():&mesh;
        included.resize(mesh.GetNE());
        boxes.reserve(mesh.GetNE());
        for(int e=0;e<mesh.GetNE();++e){
            if(stopped())return;
            included[e]=selection.Includes(e)&&(options.retained_elements.empty()||options.retained_elements[e]);
            if(!included[e])continue;
            auto* transform=dynamic_cast<mfem::IsoparametricTransformation*>(displayed->GetElementTransformation(e));
            if(!transform)throw std::invalid_argument("Flow requires an isoparametric MFEM geometry map");
            Bounds box;
            const auto& coordinates=transform->GetPointMat();
            for(int i=0;i<coordinates.Width();++i){P p{};for(int d=0;d<coordinates.Height();++d)p[d]=coordinates(d,i);if(!finite(p))throw std::invalid_argument("Flow geometry contains non-finite coordinates");box.include(p);}
            // Center-Jacobian weights cheaply spread automatic seeds over
            // physical volume, including deformed geometry and curved patches.
            const auto geometry=displayed->GetElementBaseGeometry(e);
            const auto& center=mfem::Geometries.GetCenter(geometry);
            transform->SetIntPoint(&center);
            double weight=transform->Weight()*mfem::Geometry::Volume[geometry];
            if(!std::isfinite(weight)||weight<=0)throw std::invalid_argument("Flow geometry has a nonpositive or non-finite element measure");
            seed_weight+=weight;
            // NURBS control points bound positive-weight rational geometry.
            // Polynomial nodal bounds are enlarged, with a bounded nearest-box
            // fallback below for curved lobes outside their sampled bounds.
            const double pad=std::max(1e-10,.15*norm(sub(box.hi,box.lo)));
            for(int d=0;d<3;++d){box.lo[d]-=pad;box.hi[d]+=pad;}
            boxes.push_back({box,e,seed_weight});
        }
        if(boxes.empty()||stopped())return;
        order.resize(boxes.size());std::iota(order.begin(),order.end(),0);
        nodes.reserve(2*boxes.size());build(0,int(boxes.size()));
        if(stopped())return;
        neighbors=&original.ElementToElementTable();
        ready=true;
    }
    bool stopped(){
        if(diag.cancelled||diag.budget_exhausted)return true;
        if(options.cancelled&&options.cancelled()){diag.cancelled=true;return true;}
        return false;
    }
    bool sample(P position,Located& output,const Located* hint=nullptr,bool global=true){
        if(!ready||stopped()||!finite(position)||!allowed(position))return false;
        if(diag.evaluations>=options.max_evaluations){diag.budget_exhausted=true;return false;}
        ++diag.evaluations;
        mfem::IntegrationPoint reference;
        int found=-1;
        if(hint&&hint->point.element>=0){
            const int e=hint->point.element;
            if(invert(e,position,reference,&hint->reference)){found=e;}
            if(found<0&&neighbors){
                const auto* row=neighbors->GetRow(e);
                for(int j=0;j<neighbors->RowSize(e);++j){const int candidate=row[j];if(candidate>=0&&candidate<int(included.size())&&included[candidate]&&invert(candidate,position,reference)){found=candidate;break;}if(stopped())return false;}
            }
        }
        if(found<0&&global){
            std::vector<int> candidates;query(0,position,candidates);std::sort(candidates.begin(),candidates.end());
            for(int e:candidates){if(stopped())return false;if(invert(e,position,reference)){found=e;break;}}
            if(found<0&&!stopped())for(int e:nearest(position)){
                if(std::binary_search(candidates.begin(),candidates.end(),e))continue;
                if(invert(e,position,reference)){found=e;break;}if(stopped())return false;
            }
        }
        if(found<0||stopped())return false;
        return evaluate(position,found,reference,output);
    }
    std::vector<Located> seeds(const std::vector<P>& requested={}){
        std::vector<Located> result;if(!ready)return result;
        const int limit=std::min(options.max_seeds,options.max_points);
        if(!requested.empty()){
            const std::size_t count=std::min<std::size_t>(requested.size(),limit);
            for(std::size_t i=0;i<count&&!stopped();++i){Located point;if(sample(requested[i*requested.size()/count],point)){result.push_back(point);++diag.seed_count;}else ++diag.rejected_seeds;}
            return result;
        }
        std::size_t total=options.seed_count;
        if(!total){total=1;for(int d=0;d<original.Dimension();++d)total*=options.seed_resolution;}
        const auto desired=std::min<std::size_t>(total,limit);
        result.reserve(desired);
        // Low-discrepancy points cover each axis even in short, capped previews.
        // The known parent avoids expensive inverse searches for exterior seeds.
        auto radical=[](std::size_t i,int base){double value=0,factor=1.;while(i){factor/=base;value+=factor*(i%base);i/=base;}return value;};
        for(std::size_t i=0;i<desired*16&&result.size()<desired&&!stopped();++i){
            const double selector=std::fmod((double(i)+.5)*.6180339887498948,1.)*seed_weight;
            const auto found=std::upper_bound(boxes.begin(),boxes.end(),selector,[](double value,const Box& b){return value<b.cumulative_weight;});
            const int element=(found==boxes.end()?boxes.back():*found).element;
            const double u=radical(i+1,2),v=radical(i+1,3),w=radical(i+1,5);
            mfem::IntegrationPoint ip;ip.Set3(u,v,w);
            switch(displayed->GetElementBaseGeometry(element)){
                case mfem::Geometry::TRIANGLE:ip.Set2(1-std::sqrt(u),v*std::sqrt(u));break;
                case mfem::Geometry::SQUARE:ip.Set2(u,v);break;
                case mfem::Geometry::TETRAHEDRON:{P sorted{u,v,w};std::sort(sorted.begin(),sorted.end());ip.Set3(sorted[0],sorted[1]-sorted[0],sorted[2]-sorted[1]);break;}
                case mfem::Geometry::PRISM:ip.Set3(1-std::sqrt(u),v*std::sqrt(u),w);break;
                case mfem::Geometry::PYRAMID:{const double height=std::cbrt(1-w);ip.Set3(height*u,height*v,1-height);break;}
                case mfem::Geometry::CUBE:break;
                default:throw std::invalid_argument("Flow does not support this reference-element geometry");
            }
            mfem::Vector x;displayed->GetElementTransformation(element)->Transform(ip,x);P position{};for(int d=0;d<x.Size();++d)position[d]=x[d];
            if(!finite(position)||!allowed(position)){++diag.rejected_seeds;continue;}
            if(diag.evaluations>=options.max_evaluations){diag.budget_exhausted=true;break;}
            ++diag.evaluations;Located point;
            if(evaluate(position,element,ip,point)){result.push_back(point);++diag.seed_count;}else ++diag.rejected_seeds;
        }
        return result;
    }
    bool step(const Located& current,double length,int direction,Located& next){
        if(current.point.magnitude<=0||stopped())return false;
        const auto unit=[&](const Located& p){P result{};for(int d=0;d<3;++d)result[d]=direction*(p.point.vector[d]/p.point.magnitude);return result;};
        const P k1=unit(current);Located a,b,c;
        if(!sample(add(current.point.position,scale(k1,.5*length)),a,&current,false)||a.point.magnitude<=0)return false;
        P delta;
        if(options.integrator==FlowIntegrator::RK2){delta=scale(unit(a),length);}
        else{
            if(!sample(add(current.point.position,scale(unit(a),.5*length)),b,&a,false)||b.point.magnitude<=0)return false;
            if(!sample(add(current.point.position,scale(unit(b),length)),c,&b,false)||c.point.magnitude<=0)return false;
            delta=scale(add(add(k1,scale(unit(a),2)),add(scale(unit(b),2),unit(c))),length/6);
        }
        // Validate intermediate chord points coherently as well as RK stages.
        // No global relocation is allowed while advancing a path: a long step
        // cannot jump across a hidden cell to another disconnected component.
        Located previous=current,probe;
        for(int j=1;j<=4;++j){if(!sample(add(current.point.position,scale(delta,double(j)/4)),probe,&previous,false))return false;previous=probe;}
        next=previous;return norm(sub(next.point.position,current.point.position))>0;
    }
private:
    bool evaluate(P position,int element,const mfem::IntegrationPoint& reference,Located& output){
        if(stopped())return false;
        auto& transform=*original.GetElementTransformation(element);transform.SetIntPoint(&reference);
        mfem::Vector value;field.GetVectorValue(transform,reference,value);
        if(value.Size()!=original.SpaceDimension())return false;
        FlowPoint point;point.position=position;point.element=element;
        for(int d=0;d<value.Size();++d)point.vector[d]=value[d];
        point.magnitude=norm(point.vector);if(!finite(point.vector)||!std::isfinite(point.magnitude))return false;
        output={point,reference};return true;
    }
    int build(int begin,int end){
        const int id=nodes.size();nodes.push_back({});nodes[id].begin=begin;nodes[id].end=end;
        for(int i=begin;i<end;++i)nodes[id].bounds.include(boxes[order[i]].bounds);
        if(end-begin<=8||stopped())return id;
        const auto extent=sub(nodes[id].bounds.hi,nodes[id].bounds.lo);const int axis=std::max_element(extent.begin(),extent.end())-extent.begin();const int mid=(begin+end)/2;
        std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](int a,int b){return boxes[a].bounds.lo[axis]+boxes[a].bounds.hi[axis]<boxes[b].bounds.lo[axis]+boxes[b].bounds.hi[axis];});
        const int left=build(begin,mid),right=build(mid,end);nodes[id].left=left;nodes[id].right=right;return id;
    }
    void query(int node,P p,std::vector<int>& found){
        if(stopped()||!nodes[node].bounds.contains(p))return;
        if(nodes[node].left>=0){query(nodes[node].left,p,found);query(nodes[node].right,p,found);return;}
        for(int i=nodes[node].begin;i<nodes[node].end;++i)if(boxes[order[i]].bounds.contains(p))found.push_back(boxes[order[i]].element);
    }
    std::vector<int> nearest(P p){
        using Candidate=std::pair<double,int>;
        std::priority_queue<Candidate,std::vector<Candidate>,std::greater<Candidate>> todo;
        std::vector<Candidate> best;todo.push({nodes[0].bounds.distance(p),0});int visits=0;
        while(!todo.empty()&&visits++<256&&!stopped()){
            auto [distance,index]=todo.top();todo.pop();if(best.size()==16&&distance>best.back().first)break;
            const auto& node=nodes[index];
            if(node.left>=0){todo.push({nodes[node.left].bounds.distance(p),node.left});todo.push({nodes[node.right].bounds.distance(p),node.right});continue;}
            for(int i=node.begin;i<node.end;++i){const auto& box=boxes[order[i]];best.push_back({box.bounds.distance(p),box.element});}
            std::sort(best.begin(),best.end());if(best.size()>16)best.resize(16);
        }
        std::vector<int> result;for(auto [distance,element]:best)result.push_back(element);return result;
    }
    bool allowed(P p)const{
        if(original.SpaceDimension()==2&&std::abs(p[2])>1e-10)return false;
        return !options.clip_plane||dot(sub(p,clip_origin),clip_normal)<=1e-10;
    }
    bool invert(int element,P position,mfem::IntegrationPoint& reference,const mfem::IntegrationPoint* guess=nullptr){
        if(stopped())return false;
        if(diag.location_tests>=options.max_location_tests){diag.budget_exhausted=true;return false;}++diag.location_tests;
        auto& transform=*displayed->GetElementTransformation(element);mfem::Vector point(original.SpaceDimension());for(int d=0;d<point.Size();++d)point[d]=position[d];
        mfem::InverseElementTransformation inverse(&transform);inverse.SetMaxIter(24);inverse.SetReferenceTol(1e-11);inverse.SetPhysicalRelTol(1e-10);inverse.SetElementTol(1e-9);if(guess)inverse.SetInitialGuess(*guess);
        int status=mfem::InverseElementTransformation::Unknown;
        auto converged=[&](){
            status=inverse.Transform(point,reference);
            if(status!=mfem::InverseElementTransformation::Inside||!mfem::Geometry::CheckPoint(displayed->GetElementBaseGeometry(element),reference,1e-8))return false;
            mfem::Vector x;transform.Transform(reference,x);double error=0,scale=1;for(int d=0;d<3;++d){const double value=d<x.Size()?x[d]:0;error=std::max(error,std::abs(value-position[d]));scale=std::max(scale,std::abs(position[d]));}return std::isfinite(error)&&error<=2e-9*scale;
        };
        if(converged())return true;if(status==mfem::InverseElementTransformation::Outside||stopped())return false;
        inverse.SetInitialGuessType(mfem::InverseElementTransformation::ClosestRefNode);inverse.SetInitGuessRelOrder(0);return converged();
    }
    mfem::Mesh& original;mfem::GridFunction& field;const FlowOptions& options;FlowDiagnostics& diag;
    std::unique_ptr<mfem::Mesh> displaced;mfem::Mesh* displayed=nullptr;const mfem::Table* neighbors=nullptr;
    std::vector<bool> included;std::vector<Box> boxes;std::vector<int> order;std::vector<Node> nodes;double seed_weight=0;P clip_origin{},clip_normal{};bool ready=false;
};
} // namespace

namespace RayFEM {
VectorCloud SampleVectorField(mfem::Mesh& mesh,mfem::GridFunction& field,const RenderOptions& render,const FlowOptions& options){
    VectorCloud result;Sampler sampler(mesh,field,render,options,result.diagnostics);
    for(const auto& sample:sampler.seeds())result.points.push_back(sample.point);
    return result;
}
StreamlineResult BuildStreamlines(mfem::Mesh& mesh,mfem::GridFunction& field,const RenderOptions& render,
                                 const FlowOptions& options,const std::vector<ProfilePosition>& explicit_seeds){
    if(mesh.Dimension()!=mesh.SpaceDimension())throw std::invalid_argument("Streamlines on embedded surfaces require tangential surface integration; use distributed arrows for this mesh");
    StreamlineResult result;Sampler sampler(mesh,field,render,options,result.diagnostics);const auto seeds=sampler.seeds(explicit_seeds);
    struct Branch{std::vector<FlowPoint> points;Located current;double length=0;bool active=true;};
    struct Path{Branch backward,forward;};std::vector<Path> paths;paths.reserve(seeds.size());
    for(const auto& seed:seeds){Path path;path.forward.current=path.backward.current=seed;path.forward.points.push_back(seed.point);path.backward.active=options.bidirectional;paths.push_back(std::move(path));}
    std::size_t total=seeds.size();
    for(int step=0;step<options.max_steps&&!sampler.stopped();++step){
        bool work=false;
        for(auto& path:paths){
            for(int direction:{1,-1}){
                auto& branch=direction==1?path.forward:path.backward;if(!branch.active)continue;work=true;
                if(total>=std::size_t(options.max_points)){result.diagnostics.budget_exhausted=true;break;}
                const double remaining=options.max_length/(options.bidirectional?2:1)-branch.length;
                if(remaining<=options.max_length*1e-12){branch.active=false;continue;}
                double length=std::min(options.step_size,remaining);Located next;bool valid=false;
                // A few smaller attempts approach boundaries without crossing
                // excluded cells or depending on an unbounded Newton search.
                for(int retry=0;retry<4&&!sampler.stopped();++retry){if(sampler.step(branch.current,length,direction,next)){valid=true;break;}length*=.5;}
                if(!valid){branch.active=false;++result.diagnostics.terminated_paths;continue;}
                branch.length+=norm(sub(next.point.position,branch.current.point.position));branch.current=next;branch.points.push_back(next.point);++total;
                if(sampler.stopped())break;
            }
            if(sampler.stopped())break;
        }
        if(!work)break;
    }
    if(!result.diagnostics.cancelled&&!result.diagnostics.budget_exhausted){
        const double target=options.max_length/(options.bidirectional?2:1);
        for(const auto& path:paths)for(const auto* branch:{&path.forward,&path.backward})
            if(branch->active&&target-branch->length>options.max_length*1e-12)result.diagnostics.budget_exhausted=true;
    }
    for(auto& path:paths){FlowLine line;line.points.reserve(path.backward.points.size()+path.forward.points.size());for(auto it=path.backward.points.rbegin();it!=path.backward.points.rend();++it)line.points.push_back(*it);line.points.insert(line.points.end(),path.forward.points.begin(),path.forward.points.end());if(line.points.size()>=2)result.lines.push_back(std::move(line));}
    return result;
}
} // namespace RayFEM
