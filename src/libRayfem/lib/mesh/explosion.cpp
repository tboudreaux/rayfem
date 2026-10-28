#include "libRayfem/mesh/explosion.hpp"

#include <cmath>
#include <stdexcept>

namespace RayFEM {

Vector3 ElementDisplayTransform::Apply(Vector3 position) const {
    if(scale == 1.0) return position;
    return {float(center.x + scale*(double(position.x)-center.x)),
            float(center.y + scale*(double(position.y)-center.y)),
            float(center.z + scale*(double(position.z)-center.z))};
}

void ValidateElementExplosion(const RenderOptions& options) {
    if(!std::isfinite(options.element_explosion) || options.element_explosion < 0 ||
       options.element_explosion > .95)
        throw std::invalid_argument("Element explosion must be finite and between 0 and 0.95");
}

ElementDisplayTransform MakeElementDisplayTransform(mfem::ElementTransformation& transform,
                                                    const RenderOptions& options) {
    ValidateElementExplosion(options);
    if(options.element_explosion == 0) return {};
    const auto& center = mfem::Geometries.GetCenter(transform.GetGeometryType());
    mfem::Vector position;
    transform.Transform(center, position);
    if(options.displacement) {
        mfem::Vector displacement;
        transform.SetIntPoint(&center);
        options.displacement->GetVectorValue(transform, center, displacement);
        position.Add(options.displacement_scale, displacement);
    }
    for(int i=0; i<position.Size(); ++i)
        if(!std::isfinite(position[i])) throw std::runtime_error("Element center is not finite");
    return {{float(position[0]),float(position[1]),position.Size()>2?float(position[2]):0},
            1.0-options.element_explosion};
}

} // namespace RayFEM
