#pragma once

#include "libRayfem/mesh/converter.hpp"

namespace RayFEM::detail {
// Differential normal of the displayed parent mapping. A zero result denotes
// a singular point; the caller can use its nondegenerate triangle normal.
Vector3 SurfaceNormal(mfem::ElementTransformation& transform,
                      const mfem::IntegrationPoint& parent,
                      const RenderOptions& options,
                      mfem::IntegrationPointTransformation* face_map = nullptr,
                      const mfem::IntegrationPoint* face_point = nullptr);
}
