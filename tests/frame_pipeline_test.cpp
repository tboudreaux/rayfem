#include "../src/app/frame_pipeline.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) { throw std::runtime_error(message); }
}

std::shared_ptr<RayFEM::RenderFrame> MakeFrame(const std::shared_ptr<mfem::Mesh>& mesh, double phase,
                                               std::vector<std::unique_ptr<mfem::FiniteElementCollection>>& keep) {
    auto frame = std::make_shared<RayFEM::RenderFrame>();
    frame->mesh = mesh;
    keep.push_back(std::make_unique<mfem::H1_FECollection>(3, 3));
    auto* space = new mfem::FiniteElementSpace(mesh.get(), keep.back().get());
    auto field = std::make_unique<mfem::GridFunction>(space);
    mfem::FunctionCoefficient wave([phase](const mfem::Vector& x) { return std::sin(3*x[0]+phase)+x[1]*x[2]; });
    field->ProjectCoefficient(wave);
    frame->fields.push_back({"wave", std::move(field), {}, {}});
    auto* motion_space = new mfem::FiniteElementSpace(mesh.get(), keep.back().get(), 3);
    frame->displacement = std::make_unique<mfem::GridFunction>(motion_space);
    mfem::VectorFunctionCoefficient motion(3, [](const mfem::Vector& x, mfem::Vector& y) { y = x; y *= .05; });
    frame->displacement->ProjectCoefficient(motion);
    return frame;
}

// Every original triangle must come back from the welded chunks unchanged.
void CheckWelded(const RayFEM::SurfaceData& soup, const RayFEM::UI::IndexedSurface& welded, const char* what,
                 bool colors = true) {
    Check(welded.triangles == soup.positions.size()/9, "Welded triangle count");
    for (const auto& chunk : welded.chunks) Check(chunk.positions.size()/3 <= 65535, "Chunk fits 16-bit indices");
    for (std::size_t t = 0; t < welded.triangles; ++t) {
        const auto [c, local] = welded.Locate(t);
        const auto& chunk = welded.chunks[c];
        for (int corner = 0; corner < 3; ++corner) {
            const std::size_t v = chunk.indices[3*local+corner], o = 3*t+corner;
            for (int d = 0; d < 3; ++d) {
                Check(chunk.positions[3*v+d] == soup.positions[3*o+d], what);
                Check(chunk.normals[3*v+d] == soup.normals[3*o+d], what);
            }
            if (colors) for (int k = 0; k < 4; ++k) Check(chunk.colors[4*v+k] == soup.colors[4*o+k], what);
            if (!chunk.values.empty()) Check(chunk.values[v] == soup.values[o], what);
        }
    }
}

void Welding() {
    auto mesh = mfem::Mesh::MakeCartesian3D(4, 4, 4, mfem::Element::HEXAHEDRON);
    mesh.SetCurvature(3);
    mfem::H1_FECollection h1(3, 3);
    mfem::FiniteElementSpace space(&mesh, &h1);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient f([](const mfem::Vector& x) { return x[0]*x[1]+x[2]; });
    field.ProjectCoefficient(f);
    RayFEM::RenderOptions options; options.refinement = 6;
    const auto soup = RayFEM::BuildSurface(mesh, &field, options);
    const auto welded = RayFEM::UI::WeldSurface(soup);
    CheckWelded(soup, welded, "Welded surface reproduces every triangle");
    Check(welded.chunks.size() == 1 && welded.vertices*4 < soup.positions.size()/3, "Quad patches weld about sixfold");
    // Small chunks exercise chunk boundaries and triangle lookup.
    const auto chunked = RayFEM::UI::WeldSurface(soup, 500);
    Check(chunked.chunks.size() > 3, "Chunk limit splits the surface");
    for (const auto& chunk : chunked.chunks) Check(chunk.positions.size()/3 <= 500, "Chunks respect their vertex limit");
    CheckWelded(soup, chunked, "Chunked surface reproduces every triangle");
}

// Copies must never refer back to the source mesh: the worker and the UI
// thread would otherwise share mutable MFEM state.
void Copies() {
    auto mesh = std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(2, 2, 2, mfem::Element::HEXAHEDRON));
    mesh->SetCurvature(3);
    std::vector<std::unique_ptr<mfem::FiniteElementCollection>> keep;
    auto first = MakeFrame(mesh, 0, keep), second = MakeFrame(mesh, 1, keep);
    auto copied_mesh = RayFEM::UI::CopyMesh(*mesh);
    Check(copied_mesh.get() != mesh.get() && copied_mesh->GetNodes() &&
          copied_mesh->GetNodes()->FESpace()->GetMesh() == copied_mesh.get(), "Mesh copy owns its nodes");
    RayFEM::UI::SpaceCache spaces;
    auto a = RayFEM::UI::CopyFrame(*first, copied_mesh, spaces);
    auto b = RayFEM::UI::CopyFrame(*second, copied_mesh, spaces);
    for (const auto* frame : {a.get(), b.get()}) {
        Check(frame->mesh == copied_mesh, "Copied frame uses the destination mesh");
        Check(frame->fields[0].gf->FESpace()->GetMesh() == copied_mesh.get(), "Copied field lives on the destination mesh");
        Check(frame->displacement->FESpace()->GetMesh() == copied_mesh.get(), "Copied displacement lives on the destination mesh");
    }
    Check(a->fields[0].gf->FESpace() == b->fields[0].gf->FESpace(), "Frames with one layout share a space");
    for (int i = 0; i < first->fields[0].gf->Size(); ++i)
        Check((*a->fields[0].gf)(i) == (*first->fields[0].gf)(i), "Copied coefficients match");
    Check(a->fields[0].gf->FESpace()->FEColl() != first->fields[0].gf->FESpace()->FEColl(),
          "Copies use their own collection");
    bool rejected = false;
    try { RayFEM::UI::CopyFrame(*first, nullptr, spaces); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "A missing destination mesh is rejected");
}

// A worker build matches the same build done directly on the viewer copy.
void WorkerBuild() {
    RayFEM::Server server("127.0.0.1", 55871);
    RayFEM::UI::FramePipeline pipeline(server);
    auto mesh = std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(3, 3, 3, mfem::Element::HEXAHEDRON));
    mesh->SetCurvature(2);
    std::vector<std::unique_ptr<mfem::FiniteElementCollection>> keep;
    auto frame = MakeFrame(mesh, .5, keep);
    pipeline.RunOnWorker([&] { pipeline.AttachComputeTwins({frame}); return 0; });
    Check(frame->compute && frame->compute->mesh != frame->mesh, "Twin has its own mesh");
    RayFEM::UI::SurfaceRequest request;
    request.frame = frame; request.key = "k"; request.field = "wave";
    request.options.refinement = 4; request.options.displacement_scale = 1;
    request.deform = true; request.boundaries = true;
    pipeline.Request(request);
    std::optional<RayFEM::UI::SurfaceResult> result;
    for (int i = 0; i < 2000 && !result; ++i) {
        result = pipeline.TakeResult();
        if (!result) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Check(result && result->error.empty(), "Worker build finished");
    auto options = request.options;
    options.displacement = frame->displacement.get();
    const auto direct = RayFEM::BuildSurface(*frame->mesh, frame->fields[0].gf.get(), options);
    // The worker recolors with the volume range; geometry and values match exactly.
    CheckWelded(direct, result->indexed, "Worker geometry and values match a direct build", false);
    Check(result->vertex_count == direct.positions.size()/3, "Worker reports the original vertex count");
    Check(result->indexed.vertices*3 < result->vertex_count, "Welding removes most duplicate vertices");
    Check(result->has_range && result->boundaries_ready && !result->boundaries.empty() && !result->picker.Empty(),
          "Worker provides range, outlines and picking");
}
} // namespace

int main() {
    try { Welding(); Copies(); WorkerBuild(); }
    catch (const std::exception& e) { std::cerr << "Frame pipeline test failed: " << e.what() << '\n'; return 1; }
    std::cout << "Frame pipeline tests passed\n";
}
