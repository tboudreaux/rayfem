#pragma once

// Optional MFEM headers use identifiers such as RED, also defined by raylib.
#include "mfem.hpp"
#include "raylib.h"
#include "libRayfem/mesh/colormap.hpp"
#include "libRayfem/mesh/color_scale.hpp"
#include <vector>

namespace RayFEM {

enum class FieldView { Value, Magnitude, Component, Divergence, CurlMagnitude, CurlComponent };
enum class SurfaceColoring { Field, Material, Boundary };

struct RenderOptions {
    // Subdivisions per reference edge; zero is treated as one.
    int refinement = 3;
    FieldView view = FieldView::Value;
    int component = 0;
    mfem::GridFunction* displacement = nullptr;
    double displacement_scale = 1.0;
    // Display-only element padding: 0 keeps shared faces joined; positive
    // values shrink each displaced element about its mapped center. [0, .95].
    double element_explosion = 0.0;
    SurfaceColoring coloring = SurfaceColoring::Field;
    // Element attribute IDs, independent of field values and coloring mode.
    std::vector<int> excluded_materials;
    // Zero-based parent element indices. Hidden cells are excluded from both
    // rendering and physical analysis; all indices must belong to this mesh.
    std::vector<int> excluded_elements;
    ColorMap color_map = ColorMap::Viridis;
    bool reverse_colors = false;
    ColorScaleOptions color_scale;
};

struct SurfaceData {
    // Triangle soup: three consecutive vertices form one triangle.
    std::vector<float> positions, normals;
    std::vector<unsigned char> colors;
    std::vector<double> values; // Selected field values in Field mode; zero for attribute coloring.
    // Per vertex IDs. Boundary 0 marks a synthetic exposed face/cap or a 2D
    // element interior. Actual boundary edge IDs are on LineSegment instead.
    std::vector<int> material_ids, boundary_ids;
    // One owning mesh element index per triangle (positions.size()/9).
    // Includes exposed interior faces and clipping caps; display deformation
    // and explosion never change these physical element identities.
    std::vector<int> element_ids;
    double min_value = 0.0, max_value = 0.0;
    BoundingBox bounds{};
};

struct DofPoint {
    Vector3 position{};
    // Global scalar-space DOF, shared across vector components. Signed local
    // orientation indices are decoded; vector coefficients use DofToVDof.
    int index = -1, element = -1;
    double value = 0.0;       // Selected field view evaluated at position.
    Vector3 vector{};        // Physical field evaluated at position.
    double coefficient = 0.0; // Global coefficient for selected component.
    bool directional = false; // ND/RT coefficient is a directional functional.
    bool interpolatory = true; // False for NURBS/positive bases and ND/RT.
};

struct VectorSample { Vector3 position{}; Vector3 vector{}; int element = -1; };

bool IsElementIncluded(const mfem::Mesh& mesh, int element, const RenderOptions& options);
// Reject invalid element indices before rendering or physical analysis.
void ValidateElementSelection(const mfem::Mesh& mesh, const RenderOptions& options);
// Immutable inclusion snapshot for repeated queries within one operation.
// Compiles unsorted/duplicate exclusions in O(elements + exclusions), then
// Includes() is O(1). Construct again after options or mesh attributes change.
// The common case without exclusions needs no mask allocation.
class ElementSelection {
public:
    ElementSelection(const mfem::Mesh& mesh, const RenderOptions& options);
    bool Includes(int element) const {
        return element >= 0 && element < element_count_ &&
            (included_.empty() || included_[element]);
    }
private:
    int element_count_;
    std::vector<bool> included_;
};
// Geometry-aware viewer default; explicit RenderOptions::refinement is unchanged.
// Includes displacement order, with a bounded 3..32 subdivisions per edge.
int RecommendedSurfaceRefinement(const mfem::Mesh& mesh,
                                 const mfem::GridFunction* displacement = nullptr);
// Stable categorical palette; ID 0 is a neutral synthetic/unassigned boundary.
Color AttributeColor(int attribute);
// Face/edge index -> actual MFEM boundary attribute, or zero if unassigned.
std::vector<int> FaceBoundaryAttributes(const mfem::Mesh& mesh);

// CPU-only. Parent element evaluation supports curved/NURBS geometry, Piola
// maps and displacement. Volume meshes emit the exterior of included elements,
// exposing their interfaces with excluded materials. Unsupported
// field/view combinations throw std::invalid_argument before MFEM can abort.
SurfaceData BuildSurface(mfem::Mesh& mesh, mfem::GridFunction* field = nullptr,
                         const RenderOptions& options = {});
// Optional selection has one flag per element. Selection occurs before DOF
// deduplication, so shared DOFs use a retained parent and its physical trace.
std::vector<DofPoint> SampleDofs(mfem::Mesh& mesh, mfem::GridFunction& field,
                                const RenderOptions& options = {}, int max_points = 20000,
                                const std::vector<bool>* retained_elements = nullptr);
std::vector<VectorSample> SampleVectors(mfem::Mesh& mesh, mfem::GridFunction& field,
                                       const RenderOptions& options = {}, int max_samples = 2000,
                                       const std::vector<bool>* retained_elements = nullptr);
double EvaluateField(mfem::GridFunction& field, int element,
                     const mfem::IntegrationPoint& point, FieldView view = FieldView::Value,
                     int component = 0);

// Must be called on the rendering thread with an active graphics context.
Model UploadSurface(const SurfaceData& surface);
Model LoadModelFromMFEM(mfem::Mesh& mesh, mfem::GridFunction& field, int refinement_level = 5);

} // namespace RayFEM
