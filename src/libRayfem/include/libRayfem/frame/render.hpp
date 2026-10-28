#pragma once

#include "mfem.hpp"
#include "libRayfem/network/packet.hpp"
#include <memory>
#include <algorithm>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace RayFEM {
struct FrameField {
    std::string name;
    std::unique_ptr<mfem::GridFunction> gf;
    // Received frame fields and their topology are immutable. Multiple DOF
    // traces can share this bounded derived signature without retaining frames.
    // MFEM updates invalidate it through the mesh/space sequence counters.
    struct DofLayoutCache {
        const mfem::FiniteElementSpace* space;
        const mfem::Mesh* mesh;
        long space_sequence, mesh_sequence;
        std::uint64_t fingerprint;
    };
    mutable std::optional<DofLayoutCache> dof_layout_cache;
    // Empty means unrestricted. Imported scientific fields may exist only on
    // named material regions; their coefficient extension elsewhere is not data.
    std::vector<int> supported_materials;
};

inline void ExcludeUnsupportedMaterials(const FrameField& field, const mfem::Mesh& mesh,
                                       std::vector<int>& exclusions) {
    if (field.supported_materials.empty()) return;
    for (int i=0;i<mesh.attributes.Size();++i) {
        const int attribute=mesh.attributes[i];
        if (std::find(field.supported_materials.begin(),field.supported_materials.end(),attribute)==field.supported_materials.end())
            exclusions.push_back(attribute);
    }
    std::sort(exclusions.begin(),exclusions.end());
    exclusions.erase(std::unique(exclusions.begin(),exclusions.end()),exclusions.end());
}

// Geometry is shared by every frame using the same mesh registration. A frame
// owns its fields; GPU resources belong to the viewer and never to this object.
struct RenderFrame {
    FrameMetadata metadata;
    uint64_t mesh_id = 0;
    uint64_t session_id = 0;
    uint64_t mesh_revision = 0;
    std::shared_ptr<mfem::Mesh> mesh;
    std::vector<FrameField> fields;
    std::unique_ptr<mfem::GridFunction> displacement;
    // Conservative retention estimates, including serialized-size allowances
    // for MFEM bookkeeping. These are not exact allocator or GPU measurements.
    size_t estimated_bytes = 0;       // This frame's fields and finite-element spaces.
    size_t estimated_mesh_bytes = 0;  // Shared geometry, counted once per mesh.
    // Optional twin of this frame on separately owned MFEM objects, so a
    // background thread can evaluate it while this frame is used elsewhere.
    // MFEM meshes and spaces are not safe to share between threads.
    std::shared_ptr<RenderFrame> compute;
};

template<class Frames>
size_t EstimateHistoryBytes(const Frames& frames) {
    size_t bytes = 0;
    std::unordered_set<const mfem::Mesh*> meshes;
    for (const auto& frame : frames) {
        if (!frame) continue;
        bytes += frame->estimated_bytes;
        if (frame->mesh && meshes.insert(frame->mesh.get()).second)
            bytes += frame->estimated_mesh_bytes;
    }
    return bytes;
}
} // namespace RayFEM
