#pragma once

#include "libRayfem/mesh/converter.hpp"

namespace RayFEM {

// A display-only affine map. Finite-element values and physical analysis
// coordinates remain those of the original (optionally displaced) mesh.
struct ElementDisplayTransform {
    Vector3 center{};
    double scale = 1.0;
    Vector3 Apply(Vector3 position) const;
};

void ValidateElementExplosion(const RenderOptions& options);
// Evaluate once per element or surface patch, then reuse for every point.
// The center includes displacement and comes from the true curved/NURBS map.
ElementDisplayTransform MakeElementDisplayTransform(mfem::ElementTransformation& transform,
                                                    const RenderOptions& options);

} // namespace RayFEM
