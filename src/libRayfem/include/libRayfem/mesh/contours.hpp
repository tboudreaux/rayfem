#pragma once
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/color_scale.hpp"
#include <span>
#include <vector>

namespace RayFEM {
// Contour values evenly spaced in the color scale's mapped space, so lines
// follow the colorbar (including signed-log and ZScale). The range ends are
// excluded. Sorted, finite and distinct.
std::vector<double> ContourLevels(const ColorScaleMapping& mapping, int count);

// Isolines of the piecewise-linear field on a triangle soup: 9 position
// floats and 3 values per triangle, as in SurfaceData. Triangles with a
// non-finite value are skipped. Segments carry no material or boundary IDs.
std::vector<LineSegment> ExtractContours(std::span<const float> positions, std::span<const float> values,
                                         std::span<const double> levels);
} // namespace RayFEM
