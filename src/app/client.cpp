#include "libRayfem/network/client.hpp"
#include "mfem.hpp"
#include "CLI/CLI.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string host = "localhost";
    int port = 5555;
    std::string geometry = "quad";
    std::string mesh_file;
    int order = 3;
    int resolution = 4;
    int frames = 120;
    int materials = 0;
    double interval = 1.0 / 30.0;
    double fps = 30.0;
    double dt = 0.05;
    double curvature = 0.18;
    double deformation = 0.12;
    std::string motion = "rotation";
    bool no_displacement = false;
    bool quiet = false;
    std::uint64_t mesh_id = 1;
};

std::unique_ptr<mfem::Mesh> makeMesh(const Options& options) {
    if (!options.mesh_file.empty()) {
        auto mesh = std::make_unique<mfem::Mesh>(options.mesh_file.c_str(), 1, 1);
        // Preserve the original geometry space, including NURBS control points.
        // EnsureNodes also converts existing NURBS nodes to polynomial H1 in
        // MFEM, so call it only when geometry nodes are actually absent.
        if (!mesh->GetNodes()) { mesh->EnsureNodes(); }
        return mesh;
    }

    const bool is_2d = options.geometry == "quad" || options.geometry == "tri";
    const int n = options.resolution;
    auto mesh = is_2d
        ? std::make_unique<mfem::Mesh>(mfem::Mesh::MakeCartesian2D(
              n, n, options.geometry == "quad" ? mfem::Element::QUADRILATERAL
                                               : mfem::Element::TRIANGLE,
              true, 1.0, 1.0))
        : std::make_unique<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(
              n, n, n, options.geometry == "hex" ? mfem::Element::HEXAHEDRON
                                                 : mfem::Element::TETRAHEDRON,
              1.0, 1.0, 1.0));
    mesh->SetCurvature(options.order);
    mfem::VectorFunctionCoefficient bend(mesh->SpaceDimension(),
        [&options](const mfem::Vector& x, mfem::Vector& y) {
            y = x;
            constexpr double pi = 3.14159265358979323846;
            y[0] += options.curvature * std::sin(pi * x[1]) * (0.25 + x[0]);
            y[1] += options.curvature * std::sin(pi * x[0]) * (0.25 + x[1]);
            if (x.Size() == 3) {
                y[2] += 0.5 * options.curvature * std::sin(pi * x[0])
                      * std::sin(pi * x[1]) * (0.25 + x[2]);
            }
        });
    mesh->Transform(bend);
    return mesh;
}

// An affine displacement is represented exactly by both polynomial nodal
// geometry and rational NURBS control points. Use DofToVDof rather than assuming
// either of MFEM's vector orderings.
void updateDisplacement(const mfem::GridFunction& nodes,
                        mfem::GridFunction& displacement,
                        double amplitude, double time, const std::string& motion) {
    const auto& space = *nodes.FESpace();
    const int dim = space.GetVDim();
    const double theta = amplitude * std::sin(time);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double stretch = amplitude * 0.5 * (1.0 - std::cos(time));
    for (int dof = 0; dof < space.GetNDofs(); ++dof) {
        const int ix = space.DofToVDof(dof, 0);
        const int iy = space.DofToVDof(dof, 1);
        const double x = nodes[ix];
        const double y = nodes[iy];
        displacement[ix] = motion == "ellipsoid" ? stretch * x : (c - 1.0) * x - s * y;
        displacement[iy] = motion == "ellipsoid" ? -0.5 * stretch * y : s * x + (c - 1.0) * y;
        if (dim == 3) {
            const int iz = space.DofToVDof(dof, 2);
            displacement[iz] = (motion == "ellipsoid" ? 0.25 * stretch :
                               0.3 * amplitude * std::sin(time)) * nodes[iz];
        }
    }
}

int stream(const Options& options) {
    auto mesh = makeMesh(options);
    if(options.materials>0) {
        std::vector<double> centers(mesh->GetNE());
        for(int e=0;e<mesh->GetNE();++e) {
            mfem::Vector x;
            mesh->GetElementTransformation(e)->Transform(mfem::Geometries.GetCenter(mesh->GetElementBaseGeometry(e)),x);
            centers[e]=x[0];
        }
        if(!centers.empty()) {
            const auto [lo,hi]=std::minmax_element(centers.begin(),centers.end());
            const double low=*lo,extent=*hi-low;
            for(int e=0;e<mesh->GetNE();++e) {
                const int group=extent>1e-14?std::min(options.materials-1,int((centers[e]-low)/extent*options.materials)):0;
                mesh->GetElement(e)->SetAttribute(group+1);
            }
            mesh->SetAttributes();
        }
    }
    const int dim = mesh->Dimension();
    if ((dim != 2 && dim != 3) || dim != mesh->SpaceDimension()) {
        throw std::runtime_error("The example requires a 2D or 3D volume mesh; "
                                 "embedded surface meshes need different vector spaces.");
    }

    // Declaration order keeps collections and spaces alive until their grid
    // functions are destroyed. The mesh owns the geometry's finite element space.
    mfem::H1_FECollection h1_collection(options.order, dim);
    mfem::L2_FECollection l2_collection(std::max(0, options.order - 1), dim);
    mfem::ND_FECollection nd_collection(options.order, dim);
    mfem::RT_FECollection rt_collection(options.order - 1, dim);
    mfem::FiniteElementSpace h1(mesh.get(), &h1_collection);
    mfem::FiniteElementSpace l2(mesh.get(), &l2_collection);
    mfem::FiniteElementSpace vector_h1(mesh.get(), &h1_collection, dim,
                                     mfem::Ordering::byVDIM);
    mfem::FiniteElementSpace nd(mesh.get(), &nd_collection);
    mfem::FiniteElementSpace rt(mesh.get(), &rt_collection);
    mfem::GridFunction temperature(&h1);
    mfem::GridFunction discontinuous(&l2);
    mfem::GridFunction velocity(&vector_h1);
    mfem::GridFunction circulation(&nd);
    mfem::GridFunction flux(&rt);
    mfem::GridFunction displacement(mesh->GetNodes()->FESpace());

    mfem::VectorFunctionCoefficient velocity_coefficient(dim,
        [dim](const mfem::Vector& x, mfem::Vector& value) {
            value.SetSize(dim);
            value[0] = 0.2 * x[0] - x[1];
            value[1] = x[0] + 0.3 * x[1];
            if (dim == 3) { value[2] = 0.4 * x[2]; }
        });
    mfem::VectorFunctionCoefficient circulation_coefficient(dim,
        [dim](const mfem::Vector& x, mfem::Vector& value) {
            value.SetSize(dim);
            value = 0.0;
            value[0] = -x[1];
            value[1] = x[0];
        });
    mfem::VectorFunctionCoefficient flux_coefficient(dim,
        [](const mfem::Vector& x, mfem::Vector& value) { value = x; });
    velocity.ProjectCoefficient(velocity_coefficient);
    circulation.ProjectCoefficient(circulation_coefficient);
    flux.ProjectCoefficient(flux_coefficient);

    std::vector<RayFEM::NamedField> fields = {
        {"temperature (H1)", &temperature},
        {"discontinuous (L2)", &discontinuous},
        {"velocity (H1 vector)", &velocity},
        {"circulation (ND / Hcurl)", &circulation},
        {"flux (RT / Hdiv)", &flux}
    };
    RayFEM::Client client(options.host, static_cast<std::size_t>(options.port));
    if (!client.sendMesh(options.mesh_id, *mesh)) {
        throw std::runtime_error("Failed to send mesh: " + client.getLastError());
    }
    std::cout << "Streaming " << mesh->GetNE() << " elements, order "
              << options.order << ", " << fields.size() << " fields/frame to "
              << options.host << ':' << options.port
              << "; mesh " << options.mesh_id << " sent once.\n";

    const auto start = std::chrono::steady_clock::now();
    int sent = 0;
    for (int frame = 0; frame < options.frames; ++frame) {
        const double time = frame * options.dt;
        mfem::FunctionCoefficient temperature_coefficient(
            [time](const mfem::Vector& x) {
                return x[0] + 2.0 * x[1] + (x.Size() == 3 ? 3.0 * x[2] : 0.0)
                     + 0.5 * std::sin(time);
            });
        mfem::FunctionCoefficient discontinuous_coefficient(
            [time](const mfem::Vector& x) {
                return (x[0] < 0.5 ? -1.0 : 1.0) + 0.25 * std::cos(time);
            });
        temperature.ProjectCoefficient(temperature_coefficient);
        discontinuous.ProjectCoefficient(discontinuous_coefficient);
        updateDisplacement(*mesh->GetNodes(), displacement, options.deformation, time, options.motion);

        RayFEM::FrameMetadata metadata;
        metadata.frame_index = static_cast<std::uint32_t>(frame);
        metadata.sim_time = time;
        std::snprintf(metadata.label, sizeof(metadata.label), "%s / %s / order %d",
                      options.mesh_file.empty() ? options.geometry.c_str() : "mesh file",
                      options.motion.c_str(), options.order);
        if (!client.sendFrame(options.mesh_id, fields, metadata,
                              options.no_displacement ? nullptr : &displacement)) {
            throw std::runtime_error("Failed to send frame " + std::to_string(frame)
                                     + ": " + client.getLastError());
        }
        ++sent;
        if (!options.quiet && (frame == 0 || frame + 1 == options.frames || frame % 30 == 0)) {
            std::cout << "Sent frame " << frame << " at t=" << time << '\n';
        }
        // Fixed deadlines avoid accumulating projection and serialization time
        // in the requested cadence. Zero interval is a rapid-stream stress test.
        if (options.interval > 0.0 && frame + 1 < options.frames) {
            std::this_thread::sleep_until(start + std::chrono::duration<double>(
                                                   (frame + 1) * options.interval));
        }
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "Sent " << sent << " complete frames in " << elapsed << " s ("
              << sent / std::max(elapsed, 1e-9) << " frames/s).\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    CLI::App app{"RayFEM multi-field, curved-mesh streaming example"};
    app.add_option("--host", options.host, "Viewer hostname")->capture_default_str();
    app.add_option("--port", options.port, "Viewer port")->check(CLI::Range(1, 65535));
    auto* geometry = app.add_option("--geometry", options.geometry, "quad, tri, hex, or tet")
        ->check(CLI::IsMember({"quad", "tri", "hex", "tet"}));
    app.add_option("--mesh-file", options.mesh_file, "MFEM mesh, including NURBS geometry")
        ->check(CLI::ExistingFile)->excludes(geometry);
    app.add_option("--order", options.order, "Field order; also geometry order for generated meshes")
        ->check(CLI::Range(1, 12));
    app.add_option("--resolution", options.resolution, "Generated cells along each axis")
        ->check(CLI::Range(1, 64));
    app.add_option("--materials", options.materials, "Assign this many material bands along x (0 preserves mesh IDs)")->check(CLI::Range(0,16));
    app.add_option("--frames", options.frames, "Number of complete frames to send")
        ->check(CLI::Range(1, std::numeric_limits<int>::max()));
    auto* interval = app.add_option("--interval", options.interval, "Seconds between frames; 0 sends rapidly")
        ->check(CLI::NonNegativeNumber);
    auto* fps = app.add_option("--fps", options.fps, "Requested stream rate, default 30")
        ->check(CLI::PositiveNumber)->excludes(interval);
    app.add_option("--dt", options.dt, "Simulation time step, independent of send rate")
        ->check(CLI::PositiveNumber);
    app.add_option("--curvature", options.curvature, "Generated geometry bend; 0 gives straight elements")
        ->check(CLI::Range(0.0, 0.3));
    app.add_option("--deformation", options.deformation, "Oscillating displacement amplitude")
        ->check(CLI::Range(0.0, 0.9));
    app.add_option("--motion", options.motion, "rotation or ellipsoid (period 2*pi in simulation time)")
        ->check(CLI::IsMember({"rotation", "ellipsoid"}));
    app.add_flag("--no-displacement", options.no_displacement, "Send undeformed frames");
    app.add_flag("--quiet", options.quiet, "Only print the stream summary");
    app.add_option("--mesh-id", options.mesh_id, "Identifier used for mesh reuse");
    CLI11_PARSE(app, argc, argv);
    if (fps->count() != 0) { options.interval = 1.0 / options.fps; }
    try {
        return stream(options);
    } catch (const std::exception& error) {
        std::cerr << "RayFEM example: " << error.what() << '\n';
        return 1;
    }
}
