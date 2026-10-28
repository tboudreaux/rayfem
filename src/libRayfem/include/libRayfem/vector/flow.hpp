#pragma once
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/profile/sampler.hpp"
#include <functional>
#include <optional>

namespace RayFEM {
enum class FlowIntegrator { RK2, RK4 };
struct FlowPoint {
    ProfilePosition position{}, vector{};
    double magnitude=0;
    int element=-1;
};
struct FlowLine { std::vector<FlowPoint> points; };
struct FlowDiagnostics {
    // Accepted initial samples, including zero vectors. This is not the number
    // of visible streamlines: a seed that cannot advance yields no FlowLine.
    std::size_t seed_count=0, evaluations=0, location_tests=0;
    std::size_t rejected_seeds=0, terminated_paths=0;
    bool budget_exhausted=false, cancelled=false;
};
struct FlowOptions {
    // Low-discrepancy reference points distributed by approximate physical
    // element volume, mapped with exact geometry. Target count is resolution
    // raised to mesh dimension when seed_count is zero, capped by max_seeds
    // and max_points. Positive seed_count overrides seed_resolution.
    int seed_resolution=6, max_seeds=256;
    int max_points=20000, max_steps=256;
    double step_size=.02, max_length=2;
    bool bidirectional=true;
    FlowIntegrator integrator=FlowIntegrator::RK4;
    std::size_t max_evaluations=100000, max_location_tests=250000;
    std::optional<ClipPlane> clip_plane;
    // Empty means no additional selection; otherwise one flag per element.
    std::vector<bool> retained_elements;
    // Polled during preparation, point location, seeding and integration.
    std::function<bool()> cancelled;
    // Direct total automatic seed request, independent of mesh dimension.
    // Zero retains legacy resolution^dimension behavior. Positive requests
    // are capped by max_seeds and max_points; negative requests are invalid.
    // Clipping, non-finite fields, cancellation and evaluation budgets can
    // leave fewer accepted seeds. Explicit BuildStreamlines seeds take
    // precedence. Accepted seeds with zero vectors produce no visible paths.
    int seed_count=0;
};
struct VectorCloud { std::vector<FlowPoint> points; FlowDiagnostics diagnostics; };
struct StreamlineResult { std::vector<FlowLine> lines; FlowDiagnostics diagnostics; };

// Exact FE vector values on the displayed geometry. Scalar/axis selections in
// RenderOptions do not change the vector being integrated. Displacement uses
// the same exact geometry-space composition as profiles; no mesh is mutated.
VectorCloud SampleVectorField(mfem::Mesh& mesh,mfem::GridFunction& field,
                              const RenderOptions& render={},const FlowOptions& options={});
// Steady-frame, normalized-direction RK2/RK4 streamlines, bounded by physical
// arc length and point/evaluation budgets. Bidirectional points are ordered
// backward-start -> seed -> forward-end. Segments stop at excluded neighbors,
// clip planes, zero/nonfinite vectors, or failed exact inverse mapping.
// Embedded 2D surfaces in 3D currently support arrows only. Seed coverage
// and bounded inverse-search fallback are numerical samples, not a guarantee
// that every disconnected region or extreme curved lobe receives a seed.
StreamlineResult BuildStreamlines(mfem::Mesh& mesh,mfem::GridFunction& field,
                                 const RenderOptions& render={},const FlowOptions& options={},
                                 const std::vector<ProfilePosition>& explicit_seeds={});
} // namespace RayFEM
