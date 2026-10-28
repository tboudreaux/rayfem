#pragma once
#include "libRayfem/mesh/converter.hpp"
#include <vector>

namespace RayFEM {
enum class ClipMode { CutThrough, WholeElements };
enum class CutAlgorithm { Refined, Planar };
struct ClipPlane { Vector3 point{}; Vector3 normal{1, 0, 0}; };
struct LineSegment { Vector3 a{}, b{}; int material_id = 0, boundary_id = 0; };
struct ClippedSurface {
    SurfaceData surface;
    std::vector<LineSegment> element_boundaries;
    // WholeElements: one flag per original element, using the same displaced,
    // optionally exploded corner selection as surface generation. Empty for CutThrough.
    std::vector<bool> retained_elements;
    // CutThrough in 3D: subdivisions used for the section through cut cells.
    // Lower than the requested refinement when the cut is too large to sample.
    int section_refinement = 0;
};

// Original finite-element edges, subdivided through the parent geometry map.
// Includes every 2D element edge or exterior 3D face edge, without triangle diagonals.
// Explosion adds separate edge copies on each cell, including interior cells.
std::vector<LineSegment> BuildElementBoundaries(mfem::Mesh& mesh,
                                               const RenderOptions& options = {});
// Keep dot(x-plane.point,plane.normal)<=0. CutThrough creates a real section cap
// in 3D; WholeElements keeps cells whose displayed corners are all behind the plane.
// Refined intersects mapped volume subcells (curved/NURBS approximation improves
// with refinement); Planar intersects original straight corner edges, then refines
// the resulting section for field evaluation. Neither mutates the reference mesh.
ClippedSurface BuildClippedSurface(mfem::Mesh& mesh, mfem::GridFunction* field,
                                   const RenderOptions& options, const ClipPlane& plane,
                                   ClipMode mode = ClipMode::CutThrough,
                                   CutAlgorithm algorithm = CutAlgorithm::Refined);
} // namespace RayFEM
