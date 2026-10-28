#include "../src/app/ray_placement.hpp"
#include "../src/app/surface_path.hpp"
#include <numbers>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) { throw std::runtime_error(message); }
}
bool OnReferenceBoundary(const mfem::IntegrationPoint& p) {
    const double tolerance = 1e-12;
    for (double c : {p.x, p.y, p.z}) if (std::abs(c) < tolerance || std::abs(c - 1) < tolerance) return true;
    return false;
}

// An interior element's own surface: all six faces, mapped back to its cell.
void InteriorElement() {
    auto mesh = mfem::Mesh::MakeCartesian3D(3, 3, 3, mfem::Element::HEXAHEDRON);
    mesh.SetCurvature(2);
    mfem::VectorFunctionCoefficient warp(3, [](const mfem::Vector& x, mfem::Vector& y) {
        y = x; y[2] += .05*std::sin(3*x[0]);
    });
    mesh.Transform(warp);
    const int element = 13;
    RayFEM::RenderOptions options;
    const auto surface = RayFEM::UI::BuildElementSurface(mesh, element, nullptr, options, 4);
    Check(surface.triangles.size() == 6*2*4*4, "All six faces of an interior hexahedron are present");
    Check(surface.outline.size() == 6*4, "Every face edge is outlined");
    for (std::size_t i = 0; i < surface.points.size(); ++i) {
        Check(OnReferenceBoundary(surface.points[i]), "Surface points lie on the reference cell boundary");
        const auto p = RayFEM::UI::AnchorPosition(mesh, {element, surface.points[i]}, options);
        Check(p && Vector3Distance(*p, surface.positions[i]) < 1e-5f, "Surface positions match their reference points");
    }
    // A ray into the element center lands on its surface, on the ray.
    const Vector3 center = Vector3Scale(Vector3Add(surface.bounds.min, surface.bounds.max), .5f);
    const Ray ray{Vector3Add(center, {2, .1f, .05f}), Vector3Normalize(Vector3Subtract(center, Vector3Add(center, {2, .1f, .05f})))};
    const auto hit = RayFEM::UI::PickElementSurface(surface, element, ray);
    Check(hit && hit->element == element, "A ray toward the element hits its surface");
    Check(OnReferenceBoundary(hit->point), "The pick maps to a point on the cell boundary");
    const auto position = RayFEM::UI::AnchorPosition(mesh, *hit, options);
    const Vector3 offset = Vector3Subtract(*position, ray.position);
    const Vector3 off_ray = Vector3Subtract(offset, Vector3Scale(ray.direction, Vector3DotProduct(offset, ray.direction)));
    Check(Vector3Length(off_ray) < 2e-3f, "The picked material point lies on the view ray");
    const Ray miss{{10, 10, 10}, {1, 0, 0}};
    Check(!RayFEM::UI::PickElementSurface(surface, element, miss), "A ray away from the element misses");
}

// Anchors are material points: they move with the displacement.
void FollowsDisplacement() {
    auto mesh = mfem::Mesh::MakeCartesian3D(2, 2, 2, mfem::Element::HEXAHEDRON);
    mfem::H1_FECollection h1(1, 3);
    mfem::FiniteElementSpace space(&mesh, &h1, 3);
    mfem::GridFunction displacement(&space);
    mfem::VectorConstantCoefficient shift(mfem::Vector({.1, -.2, .3}));
    displacement.ProjectCoefficient(shift);
    RayFEM::UI::RayAnchor anchor{3, {}};
    anchor.point.Set3(.25, .5, 1);
    RayFEM::RenderOptions still, moved;
    moved.displacement = &displacement; moved.displacement_scale = 2;
    const auto a = RayFEM::UI::AnchorPosition(mesh, anchor, still);
    const auto b = RayFEM::UI::AnchorPosition(mesh, anchor, moved);
    Check(a && b && Vector3Distance(Vector3Subtract(*b, *a), {.2f, -.4f, .6f}) < 1e-6f, "Anchored points follow scaled displacement");
    Check(!RayFEM::UI::AnchorPosition(mesh, {99, {}}, still), "Anchors outside the mesh have no position");
}

// In 2D the element itself is the surface to place on.
void PlanarElement() {
    auto mesh = mfem::Mesh::MakeCartesian2D(2, 2, mfem::Element::QUADRILATERAL);
    const auto surface = RayFEM::UI::BuildElementSurface(mesh, 1, nullptr, {}, 3);
    Check(surface.triangles.size() == 2*3*3 && surface.outline.size() == 4, "A quadrilateral is its own surface");
    const Vector3 center = Vector3Scale(Vector3Add(surface.bounds.min, surface.bounds.max), .5f);
    const Ray ray{{center.x, center.y, 1}, {0, 0, -1}};
    const auto hit = RayFEM::UI::PickElementSurface(surface, 1, ray);
    Check(hit && std::abs(hit->point.x - .5) < 1e-6 && std::abs(hit->point.y - .5) < 1e-6, "A 2D pick maps to the reference point");
}
// Path segments sweep the plane through two points and the center, either
// the short way or the long way round.
void PathGeometry() {
    using RayFEM::UI::MakeSurfacePathRays;
    const double pi = std::numbers::pi;
    auto direction = [](const RayFEM::SurfaceTraceRay& ray) { return Vector3{float(ray.direction[0]), float(ray.direction[1]), float(ray.direction[2])}; };
    const std::vector<Vector3> quarter{{1, 0, 0}, {0, 1, 0}};
    const auto shortWay = MakeSurfacePathRays({0, 0, 0}, quarter, {false}, false, 64);
    Check(Vector3Distance(direction(shortWay.rays.front()), {1, 0, 0}) < 1e-6f &&
          Vector3Distance(direction(shortWay.rays.back()), {0, 1, 0}) < 1e-6f, "The short way runs from the first point to the second");
    Check(std::abs(shortWay.control_parameters.back() - pi/2) < 1e-9, "The short way sweeps a quarter turn");
    for (std::size_t i = 1; i < shortWay.rays.size(); ++i)
        Check(shortWay.rays[i].parameter > shortWay.rays[i-1].parameter, "Path parameters increase");
    const auto longWay = MakeSurfacePathRays({0, 0, 0}, quarter, {true}, false, 64);
    Check(std::abs(longWay.control_parameters.back() - 3*pi/2) < 1e-9 &&
          Vector3Distance(direction(longWay.rays.back()), {0, 1, 0}) < 1e-6f, "The long way sweeps three quarters and still ends at the second point");
    const auto middle = direction(longWay.rays[longWay.rays.size()/2]);
    Check(middle.x < -.6f && middle.y < -.6f, "The long way passes the opposite side");
    // A closed loop through three equator points: the last segment joins
    // opposite points, so its plane comes from the previous segment.
    const auto loop = MakeSurfacePathRays({0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {-1, 0, 0}}, {false, false, false}, true, 120);
    Check(loop.control_parameters.size() == 4 && std::abs(loop.control_parameters.back() - 2*pi) < 1e-6, "A closed equator loop is one full turn");
    Check(loop.ambiguous_segments == std::vector<int>{2}, "Opposite points are reported as ambiguous");
    for (const auto& ray : loop.rays) Check(std::abs(ray.direction[2]) < 1e-6, "The loop stays in the equator plane");
    bool rejected = false;
    try { MakeSurfacePathRays({0, 0, 0}, {{1, 0, 0}}, {}, false, 8); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "A single point is not a path");
}

// Path points sit on outer faces: interior elements offer none.
void ExteriorFaces() {
    auto mesh = mfem::Mesh::MakeCartesian3D(3, 3, 3, mfem::Element::HEXAHEDRON);
    // Element numbering follows a space-filling curve; find cells by their faces.
    int interior = -1, corner = -1;
    for (int e = 0; e < mesh.GetNE(); ++e) {
        mfem::Array<int> faces, orientations;
        mesh.GetElementFaces(e, faces, orientations);
        int outer = 0;
        for (int face : faces) outer += mesh.GetFaceInformation(face).IsBoundary();
        if (outer == 0) interior = e;
        if (outer == 3 && corner < 0) corner = e;
    }
    Check(interior >= 0 && corner >= 0, "The fixture has interior and corner cells");
    Check(RayFEM::UI::BuildElementSurface(mesh, interior, nullptr, {}, 3, true).triangles.empty(), "An interior element has no outer face");
    Check(RayFEM::UI::BuildElementSurface(mesh, corner, nullptr, {}, 3, true).triangles.size() == 3*2*3*3, "A corner element has three outer faces");
    Check(RayFEM::UI::BuildElementSurface(mesh, interior, nullptr, {}, 3).triangles.size() == 6*2*3*3, "Without the filter all six faces remain");
}

// Sampled through the surface-trace library, a path hits the outer faces
// between its points and evaluates the field there.
void PathTrace() {
    auto mesh = mfem::Mesh::MakeCartesian3D(2, 2, 2, mfem::Element::HEXAHEDRON);
    mfem::H1_FECollection h1(1, 3);
    mfem::FiniteElementSpace space(&mesh, &h1);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient x([](const mfem::Vector& p) { return p[0]; });
    field.ProjectCoefficient(x);
    const Vector3 center{.5f, .5f, .5f};
    const auto path = RayFEM::UI::MakeSurfacePathRays(center, {{1, .5f, .5f}, {.5f, 1, .5f}}, {false}, false, 40);
    const auto samples = RayFEM::SurfaceTraceRegistry().sample("curved", mesh, field, path.rays);
    Check(samples.front().valid && samples.back().valid, "The path ends hit the surface");
    Check(std::abs(samples.front().position[0] - 1) < 1e-9 && std::abs(samples.back().position[1] - 1) < 1e-9, "The path runs from the +x face to the +y face");
    for (const auto& sample : samples) {
        Check(sample.valid && std::abs(sample.position[2] - .5) < 1e-9, "Every hit lies in the path plane");
        Check(std::abs(sample.value - sample.position[0]) < 1e-9, "Values come from the field at each hit");
    }
}
} // namespace

int main() {
    try { InteriorElement(); FollowsDisplacement(); PlanarElement(); PathGeometry(); ExteriorFaces(); PathTrace(); }
    catch (const std::exception& e) { std::cerr << "Ray placement test failed: " << e.what() << '\n'; return 1; }
    std::cout << "Ray placement tests passed\n";
}
