#pragma once

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace RayFEM::UI {

struct ElementHit {
    int element = -1;
    Vector3 position{};
    double distance = 0; // Physical distance; input ray direction need not be unit length.
    std::uint32_t triangle = 0;
};

struct ElementPickStats { std::size_t boxes_tested = 0, triangles_tested = 0; };

// Index the actual displayed triangle soup, including curved tessellation,
// displacement, clipping caps and exploded interior faces. No graphics context
// is needed. The index owns its triangle positions, so the source buffers can
// be freed or welded for upload once it is built.
class ElementPicker {
public:
    void Build(std::vector<float> positions, std::span<const int> element_ids) {
        positions_ = {};
        groups_.clear(); group_lookup_.clear(); triangles_.clear(); order_.clear(); nodes_.clear();
        if (positions.size()%9 || element_ids.size() != positions.size()/9) {
            throw std::invalid_argument("Element picking requires one element ID per triangle");
        }
        if (element_ids.size() > std::numeric_limits<std::uint32_t>::max()) {
            throw std::length_error("Too many triangles for element picking");
        }
        for (std::size_t t = 0; t < element_ids.size(); ++t) {
            const int element = element_ids[t];
            if (element < 0) { throw std::invalid_argument("Element picking requires nonnegative owner IDs"); }
            const auto [found, inserted] = group_lookup_.try_emplace(element, groups_.size());
            if (inserted) { groups_.push_back({element, {}, 0, 0}); }
            auto& group = groups_[found->second];
            ++group.count;
            for (int vertex = 0; vertex < 3; ++vertex) {
                const float* p = positions.data()+9*t+3*vertex;
                for (int d = 0; d < 3; ++d) {
                    if (!std::isfinite(p[d])) { throw std::invalid_argument("Element picking requires finite positions"); }
                    group.bounds.low[d] = std::min(group.bounds.low[d], double(p[d]));
                    group.bounds.high[d] = std::max(group.bounds.high[d], double(p[d]));
                }
            }
        }
        if (groups_.size() > std::numeric_limits<int>::max()/2) {
            throw std::length_error("Too many elements for picking hierarchy");
        }
        std::size_t offset = 0;
        std::vector<std::size_t> next;
        next.reserve(groups_.size());
        for (auto& group : groups_) {
            group.first = offset; next.push_back(offset); offset += group.count;
        }
        triangles_.resize(element_ids.size());
        for (std::size_t t = 0; t < element_ids.size(); ++t) {
            triangles_[next[group_lookup_.at(element_ids[t])]++] = static_cast<std::uint32_t>(t);
        }
        order_.resize(groups_.size());
        std::iota(order_.begin(), order_.end(), std::uint32_t{0});
        nodes_.reserve(groups_.size()*2);
        if (!groups_.empty()) { BuildNode(0, groups_.size()); }
        positions_ = std::move(positions);
    }

    bool Empty() const { return nodes_.empty(); }
    std::span<const std::uint32_t> TrianglesForElement(int element) const {
        const auto found = group_lookup_.find(element);
        if (found == group_lookup_.end()) { return {}; }
        const auto& group = groups_[found->second];
        return std::span<const std::uint32_t>(triangles_).subspan(group.first, group.count);
    }
    std::size_t StorageBytes() const {
        return positions_.capacity()*sizeof(float) + triangles_.capacity()*sizeof(std::uint32_t) + order_.capacity()*sizeof(std::uint32_t) +
            groups_.capacity()*sizeof(Group) + nodes_.capacity()*sizeof(Node) +
            group_lookup_.size()*(sizeof(int)+sizeof(std::size_t)+3*sizeof(void*));
    }

    std::optional<ElementHit> Pick(Ray ray, ElementPickStats* stats = nullptr) const {
        if (stats) { *stats = {}; }
        if (Empty()) { return {}; }
        const DVec origin{ray.position.x,ray.position.y,ray.position.z};
        DVec direction{ray.direction.x,ray.direction.y,ray.direction.z};
        const double length = std::hypot(direction.x,direction.y,direction.z);
        if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) ||
            !std::isfinite(length) || length <= 0) { return {}; }
        direction = direction*(1/length);
        double closest = std::numeric_limits<double>::infinity();
        std::optional<ElementHit> hit;
        struct Visit { int node; double distance; };
        std::vector<Visit> stack;
        stack.reserve(64);
        auto box_hit = [&](const Box& box) {
            if (stats) { ++stats->boxes_tested; }
            return IntersectBox(box, origin, direction, closest);
        };
        const auto root_distance = box_hit(nodes_[0].bounds);
        if (!root_distance) { return {}; }
        stack.push_back({0,*root_distance});
        while (!stack.empty()) {
            const auto visit = stack.back(); stack.pop_back();
            if (visit.distance > closest) { continue; }
            const auto& node = nodes_[visit.node];
            if (node.left >= 0) {
                const auto a = box_hit(nodes_[node.left].bounds);
                const auto b = box_hit(nodes_[node.right].bounds);
                if (a && b) {
                    // Visit the nearer child first, so its hit can prune the
                    // remaining hierarchy before any triangle tests there.
                    if (*a <= *b) { stack.push_back({node.right,*b}); stack.push_back({node.left,*a}); }
                    else { stack.push_back({node.left,*a}); stack.push_back({node.right,*b}); }
                } else if (a) { stack.push_back({node.left,*a}); }
                else if (b) { stack.push_back({node.right,*b}); }
                continue;
            }
            for (std::size_t i = node.first; i < node.last; ++i) {
                const auto& group = groups_[order_[i]];
                if (!box_hit(group.bounds)) { continue; }
                for (std::size_t j = group.first; j < group.first+group.count; ++j) {
                    if (stats) { ++stats->triangles_tested; }
                    const auto triangle = triangles_[j];
                    const auto distance = IntersectTriangle(triangle, origin, direction);
                    if (!distance || *distance > closest ||
                        (*distance == closest && hit && triangle >= hit->triangle)) { continue; }
                    closest = *distance;
                    const auto point = origin+direction*closest;
                    hit = ElementHit{group.element,{float(point.x),float(point.y),float(point.z)},closest,triangle};
                }
            }
        }
        return hit;
    }

private:
    struct DVec {
        double x, y, z;
        DVec operator+(DVec b) const { return {x+b.x,y+b.y,z+b.z}; }
        DVec operator-(DVec b) const { return {x-b.x,y-b.y,z-b.z}; }
        DVec operator*(double s) const { return {x*s,y*s,z*s}; }
    };
    static DVec Cross(DVec a, DVec b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
    static double Dot(DVec a, DVec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
    struct Box {
        double low[3]{INFINITY,INFINITY,INFINITY};
        double high[3]{-INFINITY,-INFINITY,-INFINITY};
        void Extend(const Box& b) {
            for (int d = 0; d < 3; ++d) { low[d]=std::min(low[d],b.low[d]); high[d]=std::max(high[d],b.high[d]); }
        }
    };
    struct Group { int element; Box bounds; std::size_t first, count; };
    struct Node { Box bounds; std::size_t first, last; int left=-1, right=-1; };
    std::vector<float> positions_;
    std::vector<Group> groups_;
    std::unordered_map<int,std::size_t> group_lookup_;
    std::vector<std::uint32_t> triangles_, order_;
    std::vector<Node> nodes_;

    int BuildNode(std::size_t first, std::size_t last) {
        const int index = static_cast<int>(nodes_.size());
        Box bounds;
        for (std::size_t i = first; i < last; ++i) { bounds.Extend(groups_[order_[i]].bounds); }
        nodes_.push_back({bounds,first,last});
        if (last-first <= 4) { return index; }
        int axis = 0;
        for (int d = 1; d < 3; ++d) {
            if (bounds.high[d]-bounds.low[d] > bounds.high[axis]-bounds.low[axis]) { axis = d; }
        }
        const std::size_t middle = first+(last-first)/2;
        std::nth_element(order_.begin()+first,order_.begin()+middle,order_.begin()+last,[&](auto a, auto b) {
            const auto& x=groups_[a].bounds; const auto& y=groups_[b].bounds;
            const double cx=std::midpoint(x.low[axis],x.high[axis]),cy=std::midpoint(y.low[axis],y.high[axis]);
            return cx==cy ? groups_[a].element<groups_[b].element : cx<cy;
        });
        const int left=BuildNode(first,middle),right=BuildNode(middle,last);
        nodes_[index].left=left; nodes_[index].right=right;
        return index;
    }

    static std::optional<double> IntersectBox(const Box& box, DVec origin, DVec direction, double limit) {
        const double start[3]{origin.x,origin.y,origin.z},delta[3]{direction.x,direction.y,direction.z};
        double near = 0, far = limit;
        for (int d = 0; d < 3; ++d) {
            if (delta[d] == 0) {
                if (start[d] < box.low[d] || start[d] > box.high[d]) { return {}; }
                continue;
            }
            double a=(box.low[d]-start[d])/delta[d],b=(box.high[d]-start[d])/delta[d];
            if (a>b) { std::swap(a,b); }
            near=std::max(near,a); far=std::min(far,b);
            if (near>far) { return {}; }
        }
        return near;
    }

    std::optional<double> IntersectTriangle(std::uint32_t triangle, DVec origin, DVec direction) const {
        const float* p=positions_.data()+std::size_t(triangle)*9;
        const DVec a{p[0],p[1],p[2]},b{p[3],p[4],p[5]},c{p[6],p[7],p[8]};
        const DVec e1=b-a,e2=c-a,perpendicular=Cross(direction,e2);
        const double determinant=Dot(e1,perpendicular);
        const double scale=std::sqrt(Dot(e1,e1)*Dot(e2,e2));
        constexpr double tolerance=64*std::numeric_limits<double>::epsilon();
        if (std::abs(determinant)<=tolerance*scale) { return {}; }
        const DVec offset=origin-a;
        const double u=Dot(offset,perpendicular)/determinant;
        if (u < -tolerance || u > 1+tolerance) { return {}; }
        const DVec q=Cross(offset,e1);
        const double v=Dot(direction,q)/determinant;
        if (v < -tolerance || u+v > 1+tolerance) { return {}; }
        const double distance=Dot(e2,q)/determinant;
        return std::isfinite(distance) && distance>=0 ? std::optional<double>{distance} : std::nullopt;
    }
};

} // namespace RayFEM::UI
