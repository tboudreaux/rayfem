#include "libRayfem/mesh/converter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

// Explicit CPU benchmark, not a timing-sensitive regression test. All runs
// rebuild their surfaces so results include the work needed for a new frame.
void Run(const char* name, int dimension, int side, int order, int refinement,
         bool displaced = false) {
    auto mesh = dimension == 2 ?
        mfem::Mesh::MakeCartesian2D(side, side, mfem::Element::QUADRILATERAL, true) :
        mfem::Mesh::MakeCartesian3D(side, side, side, mfem::Element::HEXAHEDRON);
    if (order > 1) {
        mesh.SetCurvature(order, false, 3);
        mfem::VectorFunctionCoefficient warp(3, [dimension](const mfem::Vector& x, mfem::Vector& y) {
            y = x;
            y[2] += .12*x[0]*(1-x[0])*x[1];
            if (dimension == 3) { y[0] += .08*x[1]*(1-x[1])*x[2]; }
        });
        mesh.Transform(warp);
    }
    mfem::H1_FECollection collection(order, dimension);
    mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& x) { return x[0]+2*x[1]; });
    field.ProjectCoefficient(coefficient);
    mfem::FiniteElementSpace motion_space(&mesh, &collection, mesh.SpaceDimension());
    mfem::GridFunction displacement(&motion_space);
    mfem::VectorFunctionCoefficient motion(mesh.SpaceDimension(), [](const mfem::Vector& x, mfem::Vector& y) {
        y = x; y *= .1;
    });
    if (displaced) { displacement.ProjectCoefficient(motion); }
    RayFEM::RenderOptions options;
    options.refinement = refinement;
    options.displacement = displaced ? &displacement : nullptr;
    std::vector<double> timings;
    std::size_t triangles = 0;
    double checksum = 0;
    for (int repeat = 0; repeat < 6; ++repeat) {
        const auto start = std::chrono::steady_clock::now();
        const auto surface = RayFEM::BuildSurface(mesh, &field, options);
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now()-start).count();
        if (repeat) { timings.push_back(elapsed); }
        triangles = surface.positions.size()/9;
        for (std::size_t i = 0; i < surface.values.size(); ++i) {
            const double expected = (surface.positions[3*i]+2*surface.positions[3*i+1])/(displaced ? 1.1 : 1.0);
            if (std::abs(surface.values[i]-expected) > 2e-6) {
                throw std::runtime_error("Benchmark field value/position mismatch");
            }
        }
        for (float value : surface.positions) { checksum += value; }
    }
    std::sort(timings.begin(), timings.end());
    std::cout << name << ": elements=" << mesh.GetNE() << " triangles=" << triangles
              << " median_ms=" << timings[timings.size()/2] << " checksum=" << checksum << '\n';
}

int main() {
    Run("curved_surface_p6", 2, 30, 6, 12);
    Run("curved_volume_p4", 3, 10, 4, 8);
    Run("displaced_volume_p4", 3, 10, 4, 8, true);
    Run("interior_heavy_linear", 3, 40, 1, 1);
}
