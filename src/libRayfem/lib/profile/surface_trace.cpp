#include "libRayfem/profile/surface_trace.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {
using Position = RayFEM::ProfilePosition;
using Ray = RayFEM::SurfaceTraceRay;
using Sample = RayFEM::SurfaceTraceSample;
Position Add(Position a, const Position& b) { for(int d=0;d<3;++d)a[d]+=b[d];return a; }
Position Sub(Position a, const Position& b) { for(int d=0;d<3;++d)a[d]-=b[d];return a; }
Position Scale(Position a, double s) { for(double& x:a)x*=s;return a; }
double Dot(const Position& a,const Position& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Position Cross(const Position& a,const Position& b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
double Norm(const Position& a) { return std::hypot(a[0],a[1],a[2]); }
bool Finite(const Position& a) { return std::all_of(a.begin(),a.end(),[](double x){return std::isfinite(x);}); }
Position Normalize(Position a) {
    const double length=Norm(a);
    if(!Finite(a)||!std::isfinite(length)||length<=0)throw std::invalid_argument("Surface trace direction must be finite and nonzero");
    for(double& x:a)x/=length;
    return a;
}
struct Basis { Position axis,x,y; };
Basis SphericalBasis(Position axis,Position reference) {
    axis=Normalize(axis);reference=Sub(reference,Scale(axis,Dot(reference,axis)));
    reference=Normalize(reference);return {axis,reference,Cross(axis,reference)};
}
void ValidatePath(const Position& center,double fixed,double begin,double end,int count) {
    if(!Finite(center)||!std::isfinite(fixed)||!std::isfinite(begin)||!std::isfinite(end)||count<2||count>1000000)
        throw std::invalid_argument("Surface paths need finite angles/center and between 2 and 1000000 samples");
}
Position SphericalDirection(const Basis& basis,double theta,double phi) {
    return Add(Scale(basis.axis,std::cos(theta)),Add(Scale(basis.x,std::sin(theta)*std::cos(phi)),Scale(basis.y,std::sin(theta)*std::sin(phi))));
}
struct Patch { int element=-1,face=-1,boundary=-1,boundary_attribute=-1;mfem::Geometry::Type geometry; };
struct Triangle { int patch;Position p[3];mfem::IntegrationPoint uv[3]; };
struct Box { Position low,high; };
// Slab test against a box; only hits at or ahead of the origin count.
bool BoxHit(const Box& box,const Ray& ray) {
    double near=0,far=std::numeric_limits<double>::infinity();
    for(int d=0;d<3;++d) {
        if(ray.direction[d]==0){if(ray.origin[d]<box.low[d]||ray.origin[d]>box.high[d])return false;continue;}
        double a=(box.low[d]-ray.origin[d])/ray.direction[d],b=(box.high[d]-ray.origin[d])/ray.direction[d];
        if(a>b)std::swap(a,b);
        near=std::max(near,a);far=std::min(far,b);
        if(near>far)return false;
    }
    return true;
}

// Always return an actual parent-element reference coordinate and position.
// This also supports arbitrary scalar-basis spatial-vector displacement fields,
// without changing nodes or converting rational geometry into polynomial H1.
Position Map(mfem::Mesh& mesh,const Patch& patch,const mfem::IntegrationPoint& uv,
             const RayFEM::RenderOptions& options,mfem::IntegrationPoint* parent=nullptr) {
    mfem::IntegrationPoint ip=uv;
    if(patch.face>=0)mesh.GetFaceElementTransformations(patch.face,
        mfem::FaceElementTransformations::HAVE_LOC1)->Loc1.Transform(uv,ip);
    auto* transform=mesh.GetElementTransformation(patch.element);
    mfem::Vector p;transform->Transform(ip,p);
    if(options.displacement&&options.displacement_scale!=0) {
        transform->SetIntPoint(&ip);
        mfem::Vector offset;options.displacement->GetVectorValue(*transform,ip,offset);
        p.Add(options.displacement_scale,offset);
    }
    Position result{p[0],p[1],p[2]};
    if(!Finite(result))throw std::invalid_argument("Surface trace geometry has nonfinite coordinates");
    if(parent)*parent=ip;
    return result;
}

bool TriangleHit(const Triangle& triangle,const Ray& ray,double& distance,double& u,double& v) {
    const Position a=Sub(triangle.p[1],triangle.p[0]),b=Sub(triangle.p[2],triangle.p[0]);
    const Position p=Cross(ray.direction,b);
    const double determinant=Dot(a,p),scale=Norm(a)*Norm(b);
    if(scale==0||std::abs(determinant)<1e-13*scale)return false;
    const Position s=Sub(ray.origin,triangle.p[0]);u=Dot(s,p)/determinant;
    const Position q=Cross(s,a);v=Dot(ray.direction,q)/determinant;
    if(u<-1e-10||v<-1e-10||u+v>1+1e-10)return false;
    distance=Dot(b,q)/determinant;
    return std::isfinite(distance)&&distance>=-1e-11*std::max({Norm(a),Norm(b),std::abs(distance)});
}

// Solve X_face(u,v) = origin + t*direction. Central differences of the actual
// parent mapping also work for NURBS and displacement bases unlike the nodes.
bool CurvedHit(mfem::Mesh& mesh,const Patch& patch,const Ray& ray,const RayFEM::RenderOptions& options,
               mfem::IntegrationPoint& uv,double tolerance) {
    mfem::Geometry::ProjectPoint(patch.geometry,uv);
    for(int iteration=0;iteration<32;++iteration) {
        const Position p=Map(mesh,patch,uv,options);
        const double distance=Dot(Sub(p,ray.origin),ray.direction);
        const Position residual=Sub(Sub(p,ray.origin),Scale(ray.direction,distance));
        const double error=Norm(residual);
        constexpr double h=2e-6;
        auto plus=uv,minus=uv;plus.x+=h;minus.x-=h;
        const Position du=Scale(Sub(Map(mesh,patch,plus,options),Map(mesh,patch,minus,options)),.5/h);
        plus=minus=uv;plus.y+=h;minus.y-=h;
        const Position dv=Scale(Sub(Map(mesh,patch,plus,options),Map(mesh,patch,minus,options)),.5/h);
        const Position normal=Cross(du,dv);
        const double determinant=Dot(normal,ray.direction);
        if(!std::isfinite(determinant)||std::abs(determinant)<1e-13*Norm(du)*Norm(dv))return false;
        // A coplanar ray has no isolated surface crossing; do not return an
        // arbitrary seed merely because its perpendicular residual is zero.
        if(error<=tolerance)return distance>=-tolerance&&mfem::Geometry::CheckPoint(patch.geometry,uv,2e-10);
        const double step_u=-Dot(residual,Cross(dv,ray.direction))/determinant;
        const double step_v=-Dot(du,Cross(residual,ray.direction))/determinant;
        bool improved=false;
        for(double factor=1;factor>=1.0/1024;factor*=.5) {
            auto next=uv;next.x+=factor*step_u;next.y+=factor*step_v;
            mfem::Geometry::ProjectPoint(patch.geometry,next);
            const Position delta=Sub(Map(mesh,patch,next,options),ray.origin);
            if(Norm(Sub(delta,Scale(ray.direction,Dot(delta,ray.direction))))<error) {uv=next;improved=true;break;}
        }
        if(!improved)return false;
    }
    return false;
}

class FaceTrace final:public RayFEM::SurfaceTraceAlgorithm {
public:
    explicit FaceTrace(bool exact):exact_(exact){}
    std::string id()const override{return exact_?"curved":"refined";}
    std::string name()const override{return exact_?"Curved face intersection":"Refined face approximation";}
    std::vector<Sample> sample(mfem::Mesh& mesh,mfem::GridFunction& field,const std::vector<Ray>& rays,
                              const RayFEM::RenderOptions& options)const override {
        if(mesh.SpaceDimension()!=3||(mesh.Dimension()!=2&&mesh.Dimension()!=3))
            throw std::invalid_argument("Surface traces require 3D volume meshes or 2D surfaces embedded in 3D");
        if(!field.FESpace()||field.FESpace()->GetMesh()!=&mesh)
            throw std::invalid_argument("Surface trace field must use the sampled mesh");
        if(!std::isfinite(options.displacement_scale))throw std::invalid_argument("Displacement scale must be finite");
        if(options.displacement&&(!options.displacement->FESpace()||options.displacement->FESpace()->GetMesh()!=&mesh||options.displacement->VectorDim()!=3||
            (mesh.GetNE()&&options.displacement->FESpace()->GetFE(0)->GetRangeType()!=mfem::FiniteElement::SCALAR)))
            throw std::invalid_argument("Surface trace displacement must be a scalar-basis spatial vector field on the sampled mesh");
        const RayFEM::ElementSelection selection(mesh,options);
        for(int e=0;e<mesh.GetNE();++e)if(selection.Includes(e)) {
            RayFEM::EvaluateField(field,e,mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e)),options.view,options.component);break;
        }
        std::vector<int> boundaries(mesh.GetNumFaces(),-1);
        if(mesh.Dimension()==3)for(int b=0;b<mesh.GetNBE();++b)boundaries[mesh.GetBdrElementFaceIndex(b)]=b;
        std::vector<Patch> patches;
        if(mesh.Dimension()==2) {
            for(int e=0;e<mesh.GetNE();++e)if(selection.Includes(e))patches.push_back({e,-1,-1,-1,mesh.GetElementBaseGeometry(e)});
        } else for(int face=0;face<mesh.GetNumFaces();++face) {
            if(!mesh.GetFaceInformation(face).IsBoundary())continue;
            const int e=mesh.GetFaceElementTransformations(face,
                mfem::FaceElementTransformations::HAVE_LOC1)->Elem1No;
            if(!selection.Includes(e))continue;
            const int b=boundaries[face];
            patches.push_back({e,face,b,b>=0?mesh.GetBdrAttribute(b):-1,mesh.GetFaceGeometry(face)});
        }
        const std::size_t refinement=std::clamp(options.refinement,1,64);
        std::size_t triangle_count=0;
        for(const auto& patch:patches)triangle_count+=(patch.geometry==mfem::Geometry::SQUARE?2:1)*refinement*refinement;
        // Sampling is explicit but synchronous in the desktop application.
        // Reject explosive work before allocating geometry or starting Newton.
        if(triangle_count>250000||(!rays.empty()&&triangle_count>24000000/rays.size())||
           (exact_&&!rays.empty()&&patches.size()>100000/rays.size()))
            throw std::invalid_argument("Surface trace request is too large; reduce samples/refinement or hide materials (limits: 250000 triangles, 24 million ray/triangle checks, 100000 curved ray/face solves)");
        mfem::GeometryRefiner refiner;std::vector<Triangle> triangles;triangles.reserve(triangle_count);
        std::vector<Box> boxes;boxes.reserve(patches.size());
        Position low{},high{};bool have_bounds=false;
        for(int index=0;index<int(patches.size());++index) {
            const auto& patch=patches[index];
            if(patch.geometry!=mfem::Geometry::TRIANGLE&&patch.geometry!=mfem::Geometry::SQUARE)
                throw std::invalid_argument("Surface traces support triangle and quadrilateral faces");
            const auto* refined=refiner.Refine(patch.geometry,refinement,1);
            std::vector<Position> positions;
            for(int j=0;j<refined->RefPts.GetNPoints();++j) {
                positions.push_back(Map(mesh,patch,refined->RefPts.IntPoint(j),options));
                if(!have_bounds){low=high=positions.back();have_bounds=true;}
                for(int d=0;d<3;++d){low[d]=std::min(low[d],positions.back()[d]);high[d]=std::max(high[d],positions.back()[d]);}
            }
            // Pad the sampled hull so the seeding below still reaches a
            // curved patch that bulges past its tessellation.
            Box box{positions[0],positions[0]};
            for(const auto& p:positions)for(int d=0;d<3;++d){box.low[d]=std::min(box.low[d],p[d]);box.high[d]=std::max(box.high[d],p[d]);}
            const double pad=.25*Norm(Sub(box.high,box.low));
            for(int d=0;d<3;++d){box.low[d]-=pad;box.high[d]+=pad;}
            boxes.push_back(box);
            auto triangle=[&](int a,int b,int c) {triangles.push_back({index,{positions[a],positions[b],positions[c]},
                {refined->RefPts.IntPoint(a),refined->RefPts.IntPoint(b),refined->RefPts.IntPoint(c)}});};
            const int stride=patch.geometry==mfem::Geometry::SQUARE?4:3;
            for(int j=0;j<refined->RefGeoms.Size();j+=stride){const int* q=refined->RefGeoms.GetData()+j;triangle(q[0],q[1],q[2]);if(stride==4)triangle(q[0],q[2],q[3]);}
        }
        const double extent=have_bounds?Norm(Sub(high,low)):1;
        const double tolerance=std::max(1e-11*extent,32*std::numeric_limits<double>::epsilon()*std::max(Norm(low),Norm(high)));
        std::vector<Sample> result;result.reserve(rays.size());
        std::size_t curved_solves=0;
        for(const auto& requested:rays) {
            Sample best;best.parameter=requested.parameter;
            if(!Finite(requested.origin)||!Finite(requested.direction)||!std::isfinite(requested.parameter)||Norm(requested.direction)==0){result.push_back(best);continue;}
            Ray ray=requested;ray.direction=Normalize(ray.direction);
            std::vector<bool> seeded(patches.size(),false);
            auto candidate=[&](int patch_index,mfem::IntegrationPoint uv) {
                const auto& patch=patches[patch_index];
                if(exact_&&++curved_solves>100000)
                    throw std::invalid_argument("Surface trace request exceeds 100000 curved solves; reduce samples/refinement or hide materials");
                if(exact_&&!CurvedHit(mesh,patch,ray,options,uv,tolerance))return;
                seeded[patch_index]=true;
                mfem::IntegrationPoint parent;const Position p=Map(mesh,patch,uv,options,&parent);
                const double distance=Dot(Sub(p,ray.origin),ray.direction);
                if(distance<-tolerance)return;
                const double residual=Norm(Sub(Sub(p,ray.origin),Scale(ray.direction,distance)));
                if(best.valid&&(distance>best.ray_distance+tolerance||
                    (std::abs(distance-best.ray_distance)<=tolerance&&std::pair(patch.element,patch.face)>=std::pair(best.element,best.face))))return;
                best.position=p;best.reference={parent.x,parent.y,parent.z};best.ray_distance=std::max(0.0,distance);best.residual=residual;
                best.element=patch.element;best.face=patch.face;best.boundary=patch.boundary;best.boundary_attribute=patch.boundary_attribute;
                best.material=mesh.GetAttribute(patch.element);best.valid=true;
            };
            for(const auto& triangle:triangles) {
                double distance,u,v;if(!TriangleHit(triangle,ray,distance,u,v))continue;
                mfem::IntegrationPoint uv;uv.Set2((1-u-v)*triangle.uv[0].x+u*triangle.uv[1].x+v*triangle.uv[2].x,
                                                (1-u-v)*triangle.uv[0].y+u*triangle.uv[1].y+v*triangle.uv[2].y);
                mfem::Geometry::ProjectPoint(patches[triangle.patch].geometry,uv);
                candidate(triangle.patch,uv);
            }
            // A coarse tessellation can miss a bulging curved patch. Center
            // seeds recover these exterior/tangent-near queries; failed solves
            // remain explicit gaps, never nearest-vertex substitutions.
            // Only patches whose padded box the ray crosses can be hit, which
            // keeps the seeding proportional to the faces along each ray.
            if(exact_)for(int index=0;index<int(patches.size());++index)if(!seeded[index]&&BoxHit(boxes[index],ray)) {
                candidate(index,mfem::Geometries.GetCenter(patches[index].geometry));
                // Curved folds can make the center Jacobian singular even
                // when the ray intersects elsewhere. A bounded reference grid
                // supplies edge/corner starts and recovers the nearest root.
                // Keep all starts even if the center found a root: another
                // branch of a folded face can cross this ray closer to origin.
                const auto* fallback=refiner.Refine(patches[index].geometry,2,1);
                for(int j=0;j<fallback->RefPts.GetNPoints();++j)candidate(index,fallback->RefPts.IntPoint(j));
            }
            if(best.valid) {
                mfem::IntegrationPoint ip;ip.Set3(best.reference[0],best.reference[1],best.reference[2]);
                best.value=RayFEM::EvaluateField(field,best.element,ip,options.view,options.component);
            }
            result.push_back(best);
        }
        double distance=0;
        for(std::size_t i=0;i<result.size();++i)if(result[i].valid) {
            if(i&&result[i-1].valid)distance+=Norm(Sub(result[i].position,result[i-1].position));
            result[i].distance=distance;
        }
        return result;
    }
private:bool exact_;
};
std::string Quote(const std::string& text) { std::string result="\"";for(char c:text){if(c=='\"')result+='\"';result+=c;}return result+'\"'; }
} // namespace

namespace RayFEM {
std::vector<SurfaceTraceRay> MakeMeridianRays(ProfilePosition center,ProfilePosition axis,ProfilePosition reference,
        double azimuth,double begin,double end,int count) {
    ValidatePath(center,azimuth,begin,end,count);const auto basis=SphericalBasis(axis,reference);
    std::vector<SurfaceTraceRay> result;result.reserve(count);
    for(int i=0;i<count;++i){double theta=std::lerp(begin,end,double(i)/(count-1));result.push_back({center,SphericalDirection(basis,theta,azimuth),theta});}
    return result;
}
std::vector<SurfaceTraceRay> MakeAzimuthRays(ProfilePosition center,ProfilePosition axis,ProfilePosition reference,
        double colatitude,double begin,double end,int count) {
    ValidatePath(center,colatitude,begin,end,count);const auto basis=SphericalBasis(axis,reference);
    std::vector<SurfaceTraceRay> result;result.reserve(count);
    for(int i=0;i<count;++i){double phi=std::lerp(begin,end,double(i)/(count-1));result.push_back({center,SphericalDirection(basis,colatitude,phi),phi});}
    return result;
}
SurfaceTraceRegistry::SurfaceTraceRegistry(){registerAlgorithm(std::make_shared<FaceTrace>(true));registerAlgorithm(std::make_shared<FaceTrace>(false));}
void SurfaceTraceRegistry::registerAlgorithm(std::shared_ptr<const SurfaceTraceAlgorithm> algorithm) {
    if(!algorithm||algorithm->id().empty())throw std::invalid_argument("Surface trace algorithms need a nonempty ID");
    for(auto& current:algorithms_)if(current->id()==algorithm->id()){current=std::move(algorithm);return;}
    algorithms_.push_back(std::move(algorithm));
}
std::vector<ProfileAlgorithmInfo> SurfaceTraceRegistry::algorithms()const {
    std::vector<ProfileAlgorithmInfo> result;for(const auto& algorithm:algorithms_)result.push_back({algorithm->id(),algorithm->name()});return result;
}
std::vector<SurfaceTraceSample> SurfaceTraceRegistry::sample(const std::string& id,mfem::Mesh& mesh,mfem::GridFunction& field,
        const std::vector<SurfaceTraceRay>& rays,const RenderOptions& options)const {
    ValidateElementSelection(mesh,options);
    const ElementSelection selection(mesh,options);
    for(const auto& algorithm:algorithms_)if(algorithm->id()==id) {
        auto result=algorithm->sample(mesh,field,rays,options);
        if(!options.excluded_materials.empty()||!options.excluded_elements.empty())for(auto& p:result)if(p.valid&&!selection.Includes(p.element)) {
            p.valid=false;p.element=p.face=p.boundary=p.boundary_attribute=p.material=-1;
            p.value=p.distance=p.ray_distance=p.residual=std::numeric_limits<double>::quiet_NaN();
        }
        double distance=0;
        for(std::size_t i=0;i<result.size();++i)if(result[i].valid) {
            if(i&&result[i-1].valid)distance+=Norm(Sub(result[i].position,result[i-1].position));
            result[i].distance=distance;
        }
        return result;
    }
    throw std::invalid_argument("Unknown surface trace algorithm: "+id);
}
void ExportSurfaceTraceCSV(std::ostream& stream,const std::vector<SurfaceTraceSample>& samples,const ProfileMetadata& metadata) {
    std::ostringstream row;row.imbue(std::locale::classic());row<<std::setprecision(std::numeric_limits<double>::max_digits10);
    std::string excluded;for(int material:metadata.excluded_materials){if(!excluded.empty())excluded+=';';excluded+=std::to_string(material);}
    std::string elements;for(int element:metadata.excluded_elements){if(!elements.empty())elements+=';';elements+=std::to_string(element);}
    stream<<"field,frame,time,view,component,algorithm,parameter,distance,ray_distance,x,y,z,value,valid,element,face,boundary,boundary_attribute,material,reference_x,reference_y,reference_z,ray_residual,excluded_materials,excluded_elements\n";
    for(const auto& p:samples) {
        row.str("");row.clear();row<<Quote(metadata.field_name)<<','<<metadata.frame<<','<<metadata.time<<','<<int(metadata.view)<<','<<metadata.component<<','<<Quote(metadata.algorithm)<<','
        <<p.parameter<<','<<p.distance<<','<<p.ray_distance<<','<<p.position[0]<<','<<p.position[1]<<','<<p.position[2]<<','<<p.value<<','<<int(p.valid)<<','
        <<p.element<<','<<p.face<<','<<p.boundary<<','<<p.boundary_attribute<<','<<p.material<<','<<p.reference[0]<<','<<p.reference[1]<<','<<p.reference[2]<<','<<p.residual<<','<<Quote(excluded)<<','<<Quote(elements)<<'\n';
        stream<<row.str();
    }
    if(!stream)throw std::runtime_error("Could not write surface trace CSV");
}
} // namespace RayFEM
