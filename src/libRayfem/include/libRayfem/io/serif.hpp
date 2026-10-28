#pragma once

#include "libRayfem/frame/render.hpp"
#include <filesystem>
#include <string>
#include <vector>

namespace RayFEM {
struct SerifLoadOptions {
    // Exact archive subtree paths; empty selects the unique matching schema.
    // Duplicate, identical checkpoint/state records count as one state.
    std::string discretization_path;
    std::string state_path;
};

struct SerifLoadResult {
    std::vector<std::shared_ptr<RenderFrame>> frames;
    std::vector<std::string> warnings;
    std::string error;
    explicit operator bool() const { return error.empty() && !frames.empty(); }
};

// Optional capability, backed by a separately built/installed SERiF library.
bool SerifLoadingAvailable();

// Main/render thread only: reconstruction uses MFEM's mutable geometry caches.
// Returned frames retain every borrowed mesh, collection and FE space. They
// remain valid after this result and the archive reader are destroyed.
// Scientific fields retain saved material support; rendering/analysis must
// apply ExcludeUnsupportedMaterials before evaluating an imported field.
// Surface-model coordinates and global scalars are reported as warnings,
// never silently interpreted as nodal coefficients or physical time.
SerifLoadResult LoadSerifFile(const std::filesystem::path& path,
                            const SerifLoadOptions& options = {});
} // namespace RayFEM
