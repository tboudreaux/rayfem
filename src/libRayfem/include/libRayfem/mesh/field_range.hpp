#pragma once

#include "libRayfem/mesh/converter.hpp"
#include <cstddef>

namespace RayFEM {

struct FieldRangeOptions {
    // Zero chooses an even 2*max(field order, geometry order)+2, bounded to
    // 4..64. To fit the whole-domain budget, automatic sampling can decrease
    // to an even ceil(max(2, field order, geometry order)), bounded at 64.
    // An explicit 1..64 chooses subdivisions per reference edge without this
    // automatic reduction.
    int subdivisions = 0;
    std::size_t max_samples = 2000000;
    // Retain at most this many evenly spaced values from the complete
    // deterministic sample sequence. Zero (the default) retains none.
    // This does not reduce extrema sampling or favor early elements.
    std::size_t distribution_samples = 0;
};

struct FieldRange {
    double min_value = 0, max_value = 0;
    std::size_t sample_count = 0, element_count = 0;
    int max_subdivisions = 0, min_subdivisions = 0;
    bool budget_limited = false;
    std::vector<double> samples;
};

// Deterministic sampled extrema over every included parent element, including
// vertices, a dense reference-volume lattice, FE nodes, and the element center.
// This is an approximation, not a bound or an exact global optimization.
// Discontinuous traces are sampled independently on both parents. Material
// exclusions and the selected field operation apply; displacement, surface
// refinement, coloring, clipping, and camera position do not affect the result.
// Empty selections, unsupported operations, invalid/nonfinite samples, and a
// preflight sample count exceeding max_samples throw; no partial range returns.
// MFEM evaluation must run on the same thread as other mesh/field operations.
FieldRange EvaluateFieldRange(mfem::Mesh& mesh, mfem::GridFunction& field,
                             const RenderOptions& options = {},
                             const FieldRangeOptions& sampling = {});

// Recolor existing field values using a shared scientific palette and update
// the legend limits. Equal/nearly constant limits use the palette midpoint.
// Values outside the supplied range are clamped. Geometry, values, and IDs are
// preserved. Empty surfaces are supported; invalid data leave the input intact.
void ApplyFieldColorRange(SurfaceData& surface, const FieldRange& range,
                         ColorMap map = ColorMap::Viridis, bool reverse = false);
// Apply an already resolved linear, signed-log, or zscale mapping. Zscale's
// fitted display limits are resolved from the desired sampling domain first.
void ApplyFieldColorRange(SurfaceData& surface, const ColorScaleMapping& mapping,
                         ColorMap map = ColorMap::Viridis, bool reverse = false);

} // namespace RayFEM
