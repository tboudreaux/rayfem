#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int width = 1000, height = 760;
constexpr Color background{20, 26, 35, 255};

void require(bool condition, const std::string& message) {
    if (!condition) { throw std::runtime_error(message); }
}

struct Window {
    explicit Window(bool visible) {
        SetTraceLogLevel(LOG_WARNING);
        SetConfigFlags(FLAG_MSAA_4X_HINT | (visible ? 0 : FLAG_WINDOW_HIDDEN));
        InitWindow(width, height, "RayFEM graphics integration checks");
        require(IsWindowReady(), "Could not create a graphics context");
    }
    ~Window() { CloseWindow(); }
};

struct GpuModel {
    Model model;
    explicit GpuModel(const RayFEM::SurfaceData& surface) : model(RayFEM::UploadSurface(surface)) {
        require(model.meshCount > 0 && model.materialCount > 0, "Surface upload produced an empty model");
    }
    ~GpuModel() { UnloadModel(model); }
    GpuModel(const GpuModel&) = delete;
    GpuModel& operator=(const GpuModel&) = delete;
};

struct Snapshot {
    std::vector<Color> pixels;
};

std::size_t differentPixels(const Snapshot& a, const Snapshot& b) {
    require(a.pixels.size() == b.pixels.size(), "Readback dimensions changed");
    std::size_t count = 0;
    // Exclude labels and legends so only scene changes count toward assertions.
    for (int y = 100; y < height - 110; ++y) {
        for (int x = 25; x < width - 25; ++x) {
            const auto& p = a.pixels[y * width + x];
            const auto& q = b.pixels[y * width + x];
            if (std::abs(int(p.r) - q.r) + std::abs(int(p.g) - q.g) +
                std::abs(int(p.b) - q.b) > 24) { ++count; }
        }
    }
    return count;
}

Camera3D fittedCamera(BoundingBox bounds, int dimension) {
    const Vector3 center = Vector3Scale(Vector3Add(bounds.min, bounds.max), 0.5f);
    const float radius = std::max(0.1f, 0.5f * Vector3Distance(bounds.min, bounds.max));
    const Vector3 direction = dimension == 2 ? Vector3{0, 0, 1} :
        Vector3Normalize(Vector3{1.3f, 0.85f, 1.8f});
    return {Vector3Add(center, Vector3Scale(direction, radius * 3.5f)),
            center, {0, 1, 0}, 38, CAMERA_PERSPECTIVE};
}

class Renderer {
public:
    Renderer() : target(LoadRenderTexture(width, height)) {
        require(target.id != 0, "Could not allocate render texture");
    }
    ~Renderer() { UnloadRenderTexture(target); }

    Snapshot draw(const RayFEM::SurfaceData& surface, Camera3D camera,
                  const std::string& title, const std::string& subtitle,
                  const std::filesystem::path& output = {},
                  const std::vector<RayFEM::DofPoint>& dofs = {},
                  const std::vector<RayFEM::VectorSample>& vectors = {},
                  bool show_surface = true,
                  const std::vector<RayFEM::LineSegment>& boundaries = {}, bool expect_empty = false) {
        // Match the viewer's all-materials-hidden path: no zero-sized mesh is
        // uploaded and the frame still clears/presents its render target.
        std::unique_ptr<GpuModel> model;
        if (!surface.positions.empty()) { model=std::make_unique<GpuModel>(surface); }
        require(model || expect_empty, title+": unexpected empty surface");
        const float radius = std::max(0.1f, 0.5f * Vector3Distance(surface.bounds.min, surface.bounds.max));
        BeginTextureMode(target);
        ClearBackground(background);
        BeginMode3D(camera);
        rlDisableBackfaceCulling();
        if (show_surface && model) { DrawModel(model->model, {0, 0, 0}, 1, WHITE); }
        // A tiny camera-facing offset avoids depth fighting with the surface.
        const Vector3 edge_bias = Vector3Scale(Vector3Normalize(Vector3Subtract(camera.position, camera.target)), radius * 0.001f);
        for (const auto& edge : boundaries) {
            DrawLine3D(Vector3Add(edge.a, edge_bias), Vector3Add(edge.b, edge_bias), {225, 235, 245, 255});
        }
        if (!vectors.empty()) {
            if (model) { DrawModelWires(model->model, {0, 0, 0}, 1, {69, 92, 112, 255}); }
            float max_length = 0;
            for (const auto& sample : vectors) {
                max_length = std::max(max_length, Vector3Length(sample.vector));
            }
            const float scale = max_length > 0 ? radius * 0.28f / max_length : 0;
            for (const auto& sample : vectors) {
                Vector3 delta = Vector3Scale(sample.vector, scale);
                Vector3 end = Vector3Add(sample.position, delta);
                Vector3 direction = Vector3Normalize(delta);
                Vector3 side = Vector3Normalize(Vector3CrossProduct(direction,
                    std::abs(direction.y) < 0.9f ? Vector3{0, 1, 0} : Vector3{1, 0, 0}));
                Vector3 back = Vector3Subtract(end, Vector3Scale(delta, 0.25f));
                Vector3 wing = Vector3Scale(side, Vector3Length(delta) * 0.13f);
                DrawLine3D(sample.position, end, {255, 216, 128, 255});
                DrawLine3D(end, Vector3Add(back, wing), {255, 216, 128, 255});
                DrawLine3D(end, Vector3Subtract(back, wing), {255, 216, 128, 255});
            }
        }
        rlEnableBackfaceCulling();
        EndMode3D();
        for (const auto& dof : dofs) {
            Vector2 p = GetWorldToScreenEx(dof.position, camera, width, height);
            if (p.x > 0 && p.x < width && p.y > 95 && p.y < height - 105) {
                DrawCircleV(p, 3.4f, {255, 226, 148, 245});
                DrawCircleLines(int(p.x), int(p.y), 3.4f, {31, 39, 51, 240});
            }
        }
        DrawRectangle(0, 0, width, 90, {14, 19, 27, 255});
        DrawText("RayFEM", 30, 20, 28, {83, 222, 202, 255});
        DrawText(title.c_str(), 174, 24, 22, {225, 235, 245, 255});
        DrawText(subtitle.c_str(), 31, 60, 17, {145, 166, 184, 255});
        DrawRectangle(0, height - 90, width, 90, {14, 19, 27, 255});
        DrawText(TextFormat("%zu surface triangles   |   %zu DOF markers   |   %zu vector samples",
                            surface.positions.size() / 9, dofs.size(), vectors.size()),
                 31, height - 72, 17, {174, 195, 211, 255});
        for (int x = 0; x < 260; ++x) {
            double t = double(x) / 259;
            auto byte = [](double value) { return static_cast<unsigned char>(255 * std::clamp(value, 0.0, 1.0)); };
            DrawRectangle(31 + x, height - 36, 1, 10,
                {byte(1.5 - std::abs(4*t - 3)), byte(1.5 - std::abs(4*t - 2)),
                 byte(1.5 - std::abs(4*t - 1)), 255});
        }
        DrawText(TextFormat("%.5g  to  %.5g", surface.min_value, surface.max_value),
                 310, height - 39, 17, {174, 195, 211, 255});
        EndTextureMode();

        // Present as well as read back the framebuffer so visible runs exercise
        // the same context's ordinary swap path. No user input is required.
        BeginDrawing();
        ClearBackground(background);
        DrawTextureRec(target.texture, {0, 0, float(width), -float(height)}, {0, 0}, WHITE);
        EndDrawing();
        Image image = LoadImageFromTexture(target.texture);
        require(image.data != nullptr, "Framebuffer readback failed");
        ImageFlipVertical(&image);
        Color* colors = LoadImageColors(image);
        if (!colors) { UnloadImage(image); throw std::runtime_error("Readback color conversion failed"); }
        Snapshot snapshot;
        snapshot.pixels.assign(colors, colors + width * height);
        UnloadImageColors(colors);
        const bool exported = output.empty() || ExportImage(image, output.string().c_str());
        UnloadImage(image);
        require(exported, "Could not export screenshot " + output.string());
        std::size_t covered = 0;
        for (int y = 100; y < height - 110; ++y) for (int x = 25; x < width - 25; ++x) {
            const auto& p = snapshot.pixels[y * width + x];
            if (std::abs(int(p.r) - background.r) + std::abs(int(p.g) - background.g) +
                std::abs(int(p.b) - background.b) > 24) { ++covered; }
        }
        require(expect_empty ? covered==0 : covered > 1500,
                title + (expect_empty ? ": hidden geometry remains in framebuffer" : ": rendered scene is blank or unexpectedly small"));
        ++rendered;
        return snapshot;
    }
    int rendered = 0;
private:
    RenderTexture2D target;
};

std::unique_ptr<mfem::Mesh> generatedMesh(const std::string& name) {
    const bool is_2d = name == "quad" || name == "tri";
    auto mesh = is_2d ? std::make_unique<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(3, 3,
        name == "quad" ? mfem::Element::QUADRILATERAL : mfem::Element::TRIANGLE, true)) :
        std::make_unique<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(2, 2, 2,
        name == "hex" ? mfem::Element::HEXAHEDRON : mfem::Element::TETRAHEDRON));
    mesh->SetCurvature(4);
    mfem::VectorFunctionCoefficient bend(mesh->SpaceDimension(),
        [](const mfem::Vector& x, mfem::Vector& y) {
            y = x;
            constexpr double pi = 3.14159265358979323846;
            y[0] += 0.15 * std::sin(pi * x[1]) * (0.25 + x[0]);
            y[1] += 0.15 * std::sin(pi * x[0]) * (0.25 + x[1]);
            if (x.Size() == 3) { y[2] += 0.12 * std::sin(pi * x[0]) * (0.25 + x[2]); }
        });
    mesh->Transform(bend);
    return mesh;
}

void runCase(Renderer& renderer, mfem::Mesh& mesh, const std::string& name,
             const std::filesystem::path& directory) {
    const int dim = mesh.Dimension();
    require(mesh.GetNodes() != nullptr, "Fixture needs original geometry nodes");
    const mfem::Vector original_nodes(*mesh.GetNodes());
    const bool was_nurbs = mesh.NURBSext != nullptr;
    mfem::H1_FECollection h1(4, dim);
    mfem::ND_FECollection nd(3, dim);
    mfem::RT_FECollection rt(2, dim);
    mfem::FiniteElementSpace h1_space(&mesh, &h1), nd_space(&mesh, &nd), rt_space(&mesh, &rt);
    mfem::GridFunction scalar(&h1_space), alternate(&h1_space), circulation(&nd_space), flux(&rt_space);
    mfem::GridFunction displacement(mesh.GetNodes()->FESpace());
    mfem::FunctionCoefficient ramp([](const mfem::Vector& x) { return x[0] + 2*x[1] + (x.Size() == 3 ? 0.4*x[2] : 0); });
    mfem::FunctionCoefficient other([](const mfem::Vector& x) { return 2*x[0] - x[1] + (x.Size() == 3 ? x[2] : 0); });
    mfem::VectorFunctionCoefficient spin(dim, [dim](const mfem::Vector& x, mfem::Vector& value) {
        value.SetSize(dim); value = 0.0; value[0] = -x[1]; value[1] = x[0];
    });
    mfem::VectorFunctionCoefficient radial(dim, [](const mfem::Vector& x, mfem::Vector& value) { value = x; });
    scalar.ProjectCoefficient(ramp); alternate.ProjectCoefficient(other);
    circulation.ProjectCoefficient(spin); flux.ProjectCoefficient(radial);
    RayFEM::RenderOptions options; options.refinement = 7;
    auto base_surface = RayFEM::BuildSurface(mesh, &scalar, options);
    const Camera3D camera = fittedCamera(base_surface.bounds, dim);
    const std::string geometry = was_nurbs ? "Original rational NURBS geometry" : "Curved order-4 geometry";
    const Snapshot base = renderer.draw(base_surface, camera, name + " / scalar field", geometry,
                                        directory / (name + "-scalar.png"));
    if(name=="hex" || name=="ball-nurbs") {
        for(double amount:{.08,.35,.8}) {
            options.element_explosion=amount;
            const auto exploded=RayFEM::BuildSurface(mesh,&scalar,options);
            const auto edges=RayFEM::BuildElementBoundaries(mesh,options);
            require(exploded.positions.size()>base_surface.positions.size(),"Explosion omitted internal faces");
            const auto separated=renderer.draw(exploded,camera,name+" / exploded elements",geometry,
                directory/(name+"-exploded-"+std::to_string(int(amount*100))+".png"),{},{},true,edges);
            require(differentPixels(base,separated)>1500,"Element padding did not change framebuffer");
        }
        options.element_explosion=0;
    }
    if (name == "hex" || name == "ball-nurbs") {
        const auto boundaries = RayFEM::BuildElementBoundaries(mesh, options);
        const Snapshot outlined = renderer.draw(base_surface, camera, name + " / element boundaries",
            "Original finite-element edges, without surface-triangle diagonals",
            directory / (name + "-boundaries.png"), {}, {}, true, boundaries);
        require(differentPixels(base, outlined) > 50, name + ": original element edges did not render");
        const float cut_x = 0.5f * (base_surface.bounds.min.x + base_surface.bounds.max.x)
                          + 0.15f * (base_surface.bounds.max.x - base_surface.bounds.min.x);
        const RayFEM::ClipPlane plane{{cut_x, 0, 0}, {1, 0, 0}};
        for (int mode = 0; mode < 3; ++mode) {
            const char* label = mode == 0 ? "cut-refined" : mode == 1 ? "cut-planar" : "cut-whole-elements";
            auto cut = RayFEM::BuildClippedSurface(mesh, &scalar, options, plane,
                mode == 2 ? RayFEM::ClipMode::WholeElements : RayFEM::ClipMode::CutThrough,
                mode == 1 ? RayFEM::CutAlgorithm::Planar : RayFEM::CutAlgorithm::Refined);
            require(!cut.surface.positions.empty(), name + ": clipping removed all test geometry");
            const Snapshot cut_image = renderer.draw(cut.surface, camera, name + " / " + label,
                "Interior section values and original element outlines",
                directory / (name + "-" + label + ".png"), {}, {}, true, cut.element_boundaries);
            require(differentPixels(base, cut_image) > 1500, name + ": clipping did not change the framebuffer");
        }
    }
    auto swapped_surface = RayFEM::BuildSurface(mesh, &alternate, options);
    const Snapshot swapped = renderer.draw(swapped_surface, camera, name + " / field swap", geometry);
    require(differentPixels(base, swapped) > 1500, name + ": field swap did not change the image");
    auto dofs = RayFEM::SampleDofs(mesh, scalar, options);
    require(!dofs.empty(), name + ": no DOF overlay samples");
    const Snapshot with_dofs = renderer.draw(base_surface, camera, name + " / degrees of freedom", geometry,
                                             directory / (name + "-dofs.png"), dofs);
    require(differentPixels(base, with_dofs) > 100, name + ": DOF markers did not appear in the image");
    options.view = RayFEM::FieldView::CurlMagnitude;
    auto curl_surface = RayFEM::BuildSurface(mesh, &circulation, options);
    auto vectors = RayFEM::SampleVectors(mesh, circulation, options);
    require(!vectors.empty(), name + ": no ND vector samples");
    const Snapshot with_vectors = renderer.draw(curl_surface, camera, name + " / H(curl) vector field", "Directional field samples and curved mesh edges",
                                                 directory / (name + "-vectors.png"), {}, vectors, false);
    std::size_t arrow_pixels = 0;
    for (int y = 100; y < height - 110; ++y) for (int x = 25; x < width - 25; ++x) {
        const auto& pixel = with_vectors.pixels[y * width + x];
        if (pixel.r > 210 && pixel.g > 165 && pixel.b < 170) { ++arrow_pixels; }
    }
    require(arrow_pixels > 20, name + ": vector arrows did not appear in the image");
    options.view = RayFEM::FieldView::Divergence;
    auto div_surface = RayFEM::BuildSurface(mesh, &flux, options);
    renderer.draw(div_surface, camera, name + " / H(div) divergence", geometry);

    // Keep the same mesh and field objects through twelve independent frames.
    // Geometry DOFs receive reference offsets; no deformation is accumulated.
    options.view = RayFEM::FieldView::Value;
    options.displacement = &displacement;
    const auto& space = *displacement.FESpace();
    for (int step = 1; step <= 12; ++step) {
        const double theta = 0.35 * std::sin(step * 0.12);
        for (int dof = 0; dof < space.GetNDofs(); ++dof) {
            const int ix = space.DofToVDof(dof, 0), iy = space.DofToVDof(dof, 1);
            const double x = original_nodes[ix], y = original_nodes[iy];
            displacement[ix] = (std::cos(theta)-1)*x - std::sin(theta)*y;
            displacement[iy] = std::sin(theta)*x + (std::cos(theta)-1)*y;
            if (dim == 3) { const int iz = space.DofToVDof(dof, 2); displacement[iz] = 0.15*std::sin(step*0.12)*original_nodes[iz]; }
        }
        auto displaced_surface = RayFEM::BuildSurface(mesh, &scalar, options);
        const auto filename = step == 12 ? directory / (name + "-displaced.png") : std::filesystem::path{};
        const Snapshot displaced = renderer.draw(displaced_surface, camera, name + " / displaced frame",
                                                   "Reference geometry reused; displacement applied for this frame", filename);
        if (step == 12) { require(differentPixels(base, displaced) > 1500, name + ": displacement did not change the image"); }
        require(mesh.GetNodes()->Size() == original_nodes.Size(), "Geometry coefficient count changed");
        for (int i = 0; i < original_nodes.Size(); ++i) {
            require((*mesh.GetNodes())[i] == original_nodes[i], "Rendering mutated the reference mesh");
        }
        require((mesh.NURBSext != nullptr) == was_nurbs, "NURBS geometry was converted");
    }
    std::cout << "PASS graphics " << name << ": surfaces, field swap, curl/div, DOFs, vectors, 12 displaced frames\n";
}

void runEllipsoidCycle(Renderer& renderer, const std::filesystem::path& mesh_file,
                       const std::filesystem::path& directory, bool nurbs) {
    mfem::Mesh mesh(mesh_file.string().c_str(), 1, 1);
    require(mesh.NURBSext && mesh.GetNodes(), "Ellipsoid fixture needs original NURBS geometry");
    if (!nurbs) { mesh.SetCurvature(5); }
    auto& node_space = *mesh.GetNodes()->FESpace();
    const mfem::Vector original_nodes(*mesh.GetNodes());
    mfem::FiniteElementSpace scalar_space(&mesh, node_space.FEColl());
    mfem::GridFunction scalar(&scalar_space), displacement(&node_space);
    for (int dof = 0; dof < scalar_space.GetNDofs(); ++dof) {
        scalar[dof] = original_nodes[node_space.DofToVDof(dof, 0)]
                    + 2 * original_nodes[node_space.DofToVDof(dof, 1)]
                    + 0.4 * original_nodes[node_space.DofToVDof(dof, 2)];
    }
    const std::string name = nurbs ? "sphere-nurbs" : "sphere-curved-h1";
    RayFEM::RenderOptions options; options.refinement = 10;
    options.displacement = &displacement;
    // Frame-independent framing and fixed field coefficients/range make image
    // changes attributable solely to geometry. Keep the full ellipsoid in view.
    const Camera3D camera = fittedCamera({{-1.8f, -1, -1.2f}, {1.8f, 1, 1.2f}}, 3);
    Snapshot first, peak, previous;
    double low = 0, high = 0;
    for (int step = 0; step <= 16; ++step) {
        const double time = step * 3.14159265358979323846 / 8;
        const double stretch = 0.4 * (1 - std::cos(time));
        const double offsets[] = {stretch, -0.5 * stretch, 0.25 * stretch};
        for (int dof = 0; dof < node_space.GetNDofs(); ++dof) {
            for (int d = 0; d < 3; ++d) {
                const int vdof = node_space.DofToVDof(dof, d);
                displacement[vdof] = offsets[d] * original_nodes[vdof];
            }
        }
        const auto surface = RayFEM::BuildSurface(mesh, &scalar, options);
        if (step == 0) { low = surface.min_value; high = surface.max_value; }
        require(surface.min_value == low && surface.max_value == high,
                name + ": fixed field/color range changed during geometry animation");
        const std::string stage = step == 0 ? "start" : step == 8 ? "ellipsoid" : "return";
        const auto filename = step == 0 || step == 8 || step == 16 ?
            directory / (name + "-" + stage + ".png") : std::filesystem::path{};
        const Snapshot current = renderer.draw(surface, camera, name + " / animated displacement",
            "Fixed camera and field; one reference mesh; sphere to ellipsoid to sphere", filename);
        if (step == 0) { first = current; }
        else { require(differentPixels(previous, current) > 100, name + ": consecutive geometry states look unchanged"); }
        if (step == 8) {
            peak = current;
            require(differentPixels(first, peak) > 10000, name + ": sphere-to-ellipsoid deformation did not render");
        }
        if (step == 16) {
            require(differentPixels(first, current) == 0, name + ": full cycle did not restore the original sphere image");
            require(differentPixels(peak, current) > 10000, name + ": return state still looks deformed");
        }
        for (int i = 0; i < original_nodes.Size(); ++i) {
            require((*mesh.GetNodes())[i] == original_nodes[i], name + ": animation mutated the original mesh");
        }
        previous = current;
    }
    std::cout << "PASS graphics " << name << ": 17 independent time states, fixed camera/field, sphere-ellipsoid-sphere image equivalence\n";
}

bool nearColor(Color a, Color b) {
    return std::abs(int(a.r)-b.r)+std::abs(int(a.g)-b.g)+std::abs(int(a.b)-b.b)<12;
}
std::size_t colorPixels(const Snapshot& snapshot, Color color) {
    std::size_t count=0;
    for(int y=100;y<height-110;++y)for(int x=25;x<width-25;++x)
        if(nearColor(snapshot.pixels[y*width+x],color))++count;
    return count;
}
void runMaterialSelection(Renderer& renderer,const std::filesystem::path& directory) {
    auto mesh=mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON,3,1,1);
    const int ids[]={2,7,42};
    for(int e=0;e<mesh.GetNE();++e) {
        mfem::Vector center;mesh.GetElementTransformation(e)->Transform(mfem::Geometries.GetCenter(mesh.GetElementBaseGeometry(e)),center);
        mesh.GetElement(e)->SetAttribute(ids[std::clamp(int(center[0]),0,2)]);
    }
    for(int b=0;b<mesh.GetNBE();++b)mesh.GetBdrElement(b)->SetAttribute(13+3*b);
    mesh.SetAttributes();mesh.SetCurvature(3);
    // Also leave a reusable native-viewer fixture for interactive checks.
    {std::ofstream file(directory/"material-three-regions.mesh");mesh.Print(file);}
    RayFEM::RenderOptions options;options.refinement=3;options.coloring=RayFEM::SurfaceColoring::Material;
    const auto original=RayFEM::BuildSurface(mesh,nullptr,options);
    auto camera=fittedCamera(original.bounds,3);
    camera.position=Vector3Add(camera.target,Vector3Scale(Vector3Normalize({-1.3f,.85f,1.8f}),
        Vector3Distance(camera.position,camera.target)));
    const auto materials=renderer.draw(original,camera,"Material IDs 2, 7, 42","Categorical colors come from stable element attribute IDs",directory/"material-full.png");
    for(int id:ids)require(colorPixels(materials,RayFEM::AttributeColor(id))>500,"Categorical material color missing from GPU readback");
    options.excluded_materials={2};
    const auto filtered_surface=RayFEM::BuildSurface(mesh,nullptr,options);
    const auto filtered=renderer.draw(filtered_surface,camera,"Material 2 hidden","Retained materials keep their colors; the new interface is exposed",directory/"material-filtered.png");
    require(colorPixels(filtered,RayFEM::AttributeColor(2))==0,"Hidden material color remains on screen");
    require(colorPixels(filtered,RayFEM::AttributeColor(7))>500&&colorPixels(filtered,RayFEM::AttributeColor(42))>500,"Filtering changed retained material colors");
    require(differentPixels(materials,filtered)>5000,"Material filtering did not change visible geometry");
    options.coloring=RayFEM::SurfaceColoring::Boundary;options.excluded_materials.clear();
    const auto boundary_surface=RayFEM::BuildSurface(mesh,nullptr,options);
    const auto boundary=renderer.draw(boundary_surface,camera,"Boundary attribute IDs","Original nonsequential boundary IDs color the exterior faces",directory/"boundary-full.png");
    int visible_boundaries=0;
    for(int b=0;b<mesh.GetNBE();++b)if(colorPixels(boundary,RayFEM::AttributeColor(mesh.GetBdrAttribute(b)))>300)++visible_boundaries;
    require(visible_boundaries>=3,"Actual boundary colors did not render on distinct exterior faces");
    require(differentPixels(materials,boundary)>5000,"Boundary coloring did not replace material coloring");
    options.excluded_materials={2};
    const auto exposed_surface=RayFEM::BuildSurface(mesh,nullptr,options);
    const auto exposed=renderer.draw(exposed_surface,camera,"Exposed interface / boundary 0","Synthetic interfaces use neutral ID 0, preserving actual exterior IDs",directory/"boundary-exposed.png");
    require(colorPixels(exposed,RayFEM::AttributeColor(0))>500,"Synthetic exposed interface did not render");
    const auto pixel=GetWorldToScreenEx({1,.5f,.5f},camera,width,height);
    require(pixel.x>=0&&pixel.x<width&&pixel.y>=100&&pixel.y<height-110,"Exposed face center outside test viewport");
    require(nearColor(exposed.pixels[int(pixel.y)*width+int(pixel.x)],RayFEM::AttributeColor(0)),"New interface face is absent, incorrectly colored, or occluded by hidden material");
    const auto cut=RayFEM::BuildClippedSurface(mesh,nullptr,options,{{0,.6f,0},{0,1,0}},RayFEM::ClipMode::CutThrough);
    const auto cut_image=renderer.draw(cut.surface,camera,"Filtered mesh with section cap","Only retained materials contribute to the synthetic clipping surface",directory/"boundary-filtered-cut.png");
    require(differentPixels(exposed,cut_image)>1000,"Filtered clipping did not change framebuffer");
    options.excluded_materials={2,7,42};
    const auto empty_surface=RayFEM::BuildSurface(mesh,nullptr,options);
    require(empty_surface.positions.empty(),"All-off material selection produced geometry");
    const auto empty=renderer.draw(empty_surface,camera,"All materials hidden","An empty selection clears the scene without uploading empty geometry",directory/"material-empty.png",{},{},true,{},true);
    require(differentPixels(materials,empty)>5000,"All-off framebuffer still contains geometry");
    options.excluded_materials.clear();options.coloring=RayFEM::SurfaceColoring::Material;
    const auto restored=renderer.draw(RayFEM::BuildSurface(mesh,nullptr,options),camera,"Materials restored","Restoring selection reproduces original colors and geometry");
    require(differentPixels(materials,restored)==0,"Restoring all materials changed their geometry or colors");
    std::cout<<"PASS graphics material/boundary colors, exposed interfaces, filtered caps, empty selection and restoration\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        const std::filesystem::path output = argc > 1 ? argv[1] : "build-rayfem/test-output";
        const std::filesystem::path data = argc > 2 ? argv[2] : "subprojects/mfem/data";
        const bool visible = argc > 3 && std::string(argv[3]) == "--visible";
        std::filesystem::create_directories(output);
        Window window(visible);
        Renderer renderer;
        for (const auto* name : {"quad", "tri", "hex", "tet"}) {
            auto mesh = generatedMesh(name);
            runCase(renderer, *mesh, name, output);
        }
        for (const auto* name : {"disc-nurbs", "beam-hex-nurbs", "ball-nurbs"}) {
            mfem::Mesh mesh((data / (std::string(name) + ".mesh")).string().c_str(), 1, 1);
            require(mesh.NURBSext != nullptr, std::string(name) + " is not a NURBS fixture");
            runCase(renderer, mesh, name, output);
        }
        runEllipsoidCycle(renderer, data / "ball-nurbs.mesh", output, true);
        runEllipsoidCycle(renderer, data / "ball-nurbs.mesh", output, false);
        runMaterialSelection(renderer, output);
        std::cout << "Graphics integration passed: " << renderer.rendered
                  << " framebuffer readbacks; 48 screenshots in " << output << '\n';
    } catch (const std::exception& error) {
        std::cerr << "Graphics integration failed: " << error.what() << '\n';
        return 1;
    }
}
