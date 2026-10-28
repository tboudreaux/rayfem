#pragma once

#include "libRayfem/mesh/converter.hpp"
#include <array>
#include <cstddef>
#include <map>
#include <optional>

namespace RayFEM {

struct ElementStatisticsOptions {
    // Zero chooses from the field/geometry orders. Explicit orders are 1..64.
    int quadrature_order = 0;
    // Dense reference sampling for approximate bounds/extrema. Zero chooses
    // 2*max(field order, geometry order)+2, bounded to 4..64 subdivisions.
    int subdivisions = 0;
    // Combined quadrature, lattice, geometry-node and field-node budget.
    std::size_t max_samples = 2000000;
};

struct ElementFieldStatistics {
    double minimum = 0, maximum = 0;
    double mean = 0, integral = 0;
};

struct ElementStatistics {
    // Duplicates in the request are counted once. Exclusions apply before any
    // integration; elements contains the sorted, included selection only.
    std::size_t requested_count = 0, excluded_count = 0, element_count = 0;
    std::vector<int> elements;
    int dimension = 0, space_dimension = 0;
    bool nurbs = false;
    std::map<int, std::size_t> material_counts;
    // Geometry keys are mfem::Geometry::Type; order keys are polynomial orders.
    std::map<int, std::size_t> geometry_counts, geometry_orders, field_orders;
    // Length/area/volume according to dimension. Centroid uses the same physical
    // Jacobian weights. Bounds and field extrema are sampled approximations,
    // not certified enclosing bounds or exact global extrema.
    double measure = 0;
    std::array<double, 3> centroid{}, bounds_min{}, bounds_max{};
    std::optional<ElementFieldStatistics> field;
    std::size_t quadrature_samples = 0, bounds_samples = 0, field_samples = 0;
};

// CPU-only; call on the MFEM/rendering thread and cache per frame/selection.
// Supports 2D/3D and embedded surfaces, curved/NURBS geometry and physical
// nodal/ND/RT field views through the common EvaluateField dispatcher.
// Geometry includes displacement and its scale using MakeDisplacedMesh's exact
// geometry-basis rules. Field values remain attached to original parent
// coordinates, matching profiles/tracking; physical weights use displaced
// geometry. Explosion, clipping and coloring do not affect these statistics.
// Empty or fully excluded selections return zero measure and no field result.
// Invalid IDs, unsupported views, bad geometry and budget overflow throw.
ElementStatistics ComputeElementStatistics(mfem::Mesh& mesh,
    const std::vector<int>& selected_elements, mfem::GridFunction* field = nullptr,
    const RenderOptions& options = {}, const ElementStatisticsOptions& sampling = {});

} // namespace RayFEM
