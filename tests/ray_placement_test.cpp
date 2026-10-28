#include "../src/app/ray_placement.hpp"

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
} // namespace

int main() {
    try { InteriorElement(); FollowsDisplacement(); PlanarElement(); }
    catch (const std::exception& e) { std::cerr << "Ray placement test failed: " << e.what() << '\n'; return 1; }
    std::cout << "Ray placement tests passed\n";
}
