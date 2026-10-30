// A small moving-mesh diffusion solver and RayFEM streaming example.
//
// Start/reuse RayFEM:     ./build-rayfem/src/sandbox
// Run without a viewer:   ./build-rayfem/src/sandbox --no-stream
//
// On the pulsating square x(X,t) = c + s(t)*(X-c), solve
//
//   partial_t T + div(w*T) = diffusivity * Laplacian(T),
//   n . grad(T) = 0,     w = mesh velocity.
//
// This is conservative diffusion: T is heat per unit area (temperature with
// unit volumetric heat capacity). Expansion dilutes T, contraction increases
// it, and insulated boundaries conserve total heat. In moving FE coordinates,
// backward Euler is simply (M_new + dt*K_new)*T_new = M_old*T_old.
//
// No remeshing, essential boundary conditions, or nonlinear solver is needed.

#include "libRayfem/network/client.hpp"
#include "libRayfem/init.hpp"
#include "mfem.hpp"
#include "CLI/CLI.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
constexpr int dimension = 2;
constexpr double pi = std::numbers::pi;

struct Options {
    std::string host = "localhost";
    int port = 5555;
    int control_port = 0;
    std::string viewer_executable;
    int resolution = 12, order = 2, steps = 300;
    double dt = 0.02, diffusivity = 0.02;
    double amplitude = 0.2, period = 2.0, fps = 30.0;
    bool no_stream = false;
};

int Run(const Options& o) {
    // 1. Keep this reference mesh unchanged for the entire stream. Geometry
    //    nodes make the displacement representation explicit, even at order 1.
    mfem::Mesh reference = mfem::Mesh::MakeCartesian2D(
        o.resolution, o.resolution, mfem::Element::QUADRILATERAL, true, 1.0, 1.0);
    reference.SetCurvature(o.order, false, dimension, mfem::Ordering::byVDIM);
    mfem::Mesh moving(reference); // Deep copy; only the solver's mesh moves.

    mfem::H1_FECollection collection(o.order, dimension);
    mfem::FiniteElementSpace solver_space(&moving, &collection);
    mfem::FiniteElementSpace display_space(&reference, &collection);
    mfem::GridFunction temperature(&solver_space), displayed_temperature(&display_space);

    // Smooth positive initial data with zero normal derivative on all sides.
    mfem::FunctionCoefficient initial([](const mfem::Vector& x) {
        return 1.0 + 0.6 * std::cos(2*pi*x[0]) * std::cos(2*pi*x[1]);
    });
    temperature.ProjectCoefficient(initial);

    auto* geometry_space = reference.GetNodes()->FESpace();
    mfem::GridFunction displacement(geometry_space), mesh_velocity(geometry_space);
    const mfem::Vector reference_nodes(*reference.GetNodes());
    auto move_mesh = [&](double time) {
        const double phase = 2*pi*time/o.period;
        const double scale = 1.0 + o.amplitude*std::sin(phase);
        const double speed = o.amplitude*(2*pi/o.period)*std::cos(phase);
        for (int dof = 0; dof < geometry_space->GetNDofs(); ++dof) {
            for (int axis = 0; axis < dimension; ++axis) {
                const int index = geometry_space->DofToVDof(dof, axis);
                const double radius = reference_nodes[index] - 0.5;
                displacement[index] = (scale - 1.0)*radius;
                mesh_velocity[index] = speed*radius;
                (*moving.GetNodes())[index] = reference_nodes[index] + displacement[index];
            }
        }
        moving.NodesUpdated(); // Invalidate MFEM's cached geometric factors.
        return scale;
    };
    move_mesh(0.0);

    // 2. Standard serial MFEM assembly. Natural boundary conditions are
    //    insulated. Fixed topology means the spaces and DOF numbering persist.
    mfem::ConstantCoefficient conductivity(o.diffusivity);
    mfem::BilinearForm mass(&solver_space), diffusion(&solver_space);
    mass.AddDomainIntegrator(new mfem::MassIntegrator());
    diffusion.AddDomainIntegrator(new mfem::DiffusionIntegrator(conductivity));
    auto assemble = [&] {
        mass.Update(); diffusion.Update(); // Clear old matrix entries first.
        // Keep structural zeros: moving geometry can make a previously zero
        // entry nonzero when the same sparse pattern is assembled again.
        mass.Assemble(0); mass.Finalize(0);
        diffusion.Assemble(0); diffusion.Finalize(0);
    };
    assemble();
    mfem::Vector ones(temperature.Size()), rhs(temperature.Size()), work(temperature.Size());
    ones = 1.0;
    auto heat = [&] { mass.Mult(temperature, work); return ones*work; };
    auto energy = [&] { mass.Mult(temperature, work); return temperature*work; };
    const double initial_heat = heat(), initial_energy = energy();
    double previous_energy = initial_energy, maximum_heat_error = 0.0;
    int maximum_iterations = 0;

    // 3. Register geometry ONCE. Every frame below carries all named fields
    //    from one state, plus offsets from the original geometry (never from
    //    the previous frame). --no-stream skips sockets and wall-clock pacing.
    std::unique_ptr<RayFEM::Client> viewer;
    constexpr std::uint64_t mesh_id = 1;
    if (!o.no_stream) {
        RayFEM::WindowParams window;
        window.host=o.host;window.port=o.port;window.control_port=o.control_port;
        window.executable=o.viewer_executable;
        const auto initialized=RayFEM::init(window);
        if(!initialized) throw std::runtime_error("Viewer startup failed: "+initialized.error);
        std::cout<<(initialized.status==RayFEM::InitStatus::Started?"Started RayFEM.\n":"Using the running RayFEM viewer.\n");
        viewer = std::make_unique<RayFEM::Client>(o.host, o.port);
        if (!viewer->sendMesh(mesh_id, reference))
            throw std::runtime_error("Mesh send failed: " + viewer->getLastError());
    }
    auto send_frame = [&](int step) {
        // These spaces have identical topology, collection, and numbering.
        // Copy coefficients onto the REFERENCE mesh: sending a field attached
        // to 'moving' would violate RayFEM's mesh registration contract.
        displayed_temperature = temperature;
        if (!viewer) return;
        RayFEM::FrameMetadata metadata;
        metadata.frame_index = static_cast<std::uint32_t>(step);
        metadata.sim_time = step*o.dt;
        if (!viewer->sendFrame(mesh_id,
                {{"Temperature", &displayed_temperature},
                 {"Mesh velocity", &mesh_velocity}},
                metadata, &displacement))
            throw std::runtime_error("Frame send failed: " + viewer->getLastError());
    };
    send_frame(0);
    std::cout << "Diffusion on " << reference.GetNE() << " pulsating quadrilaterals, "
              << solver_space.GetNDofs() << " temperature DOFs.\n"
              << (viewer ? "Mesh sent once; streaming Temperature and Mesh velocity.\n"
                         : "Offline solve (no viewer or pacing).\n");

    // 4. One linear, backward-Euler solve per step. The old mass matrix must
    //    form the RHS BEFORE changing geometry and assembling the new forms.
    const auto started = std::chrono::steady_clock::now();
    for (int step = 1; step <= o.steps; ++step) {
        mass.Mult(temperature, rhs);
        const double scale = move_mesh(step*o.dt);
        assemble();
        std::unique_ptr<mfem::SparseMatrix> system(
            mfem::Add(1.0, mass.SpMat(), o.dt, diffusion.SpMat()));
        mfem::GSSmoother preconditioner(*system); // Symmetric Gauss-Seidel.
        mfem::CGSolver cg;
        cg.SetOperator(*system);
        cg.SetPreconditioner(preconditioner);
        cg.SetRelTol(1e-12); cg.SetAbsTol(1e-14);
        cg.SetMaxIter(500); cg.SetPrintLevel(0);
        cg.iterative_mode = true; // Previous coefficients are the initial guess.
        cg.Mult(rhs, temperature);
        if (!cg.GetConverged()) throw std::runtime_error("Diffusion solve did not converge");
        system->Mult(temperature, work); work -= rhs;
        if (!std::isfinite(work.Norml2()) || work.Norml2() > 1e-9*rhs.Norml2()+1e-13)
            throw std::runtime_error("Diffusion solve failed its residual check");
        maximum_iterations = std::max(maximum_iterations, cg.GetNumIterations());

        // Small numerical checks also make --no-stream useful as a smoke test.
        const double total_heat = heat(), current_energy = energy();
        const double relative_error = std::abs(total_heat-initial_heat)/std::abs(initial_heat);
        maximum_heat_error = std::max(maximum_heat_error, relative_error);
        if (!std::isfinite(current_energy) || !std::isfinite(relative_error) || relative_error > 1e-8)
            throw std::runtime_error("Insulated diffusion failed its total-heat check");
        mass.Mult(ones, work);
        if (std::abs(ones*work-scale*scale) > 1e-9*scale*scale)
            throw std::runtime_error("Mass matrix does not match the moving domain area");
        if (o.amplitude == 0 && current_energy > previous_energy + 1e-10*initial_energy)
            throw std::runtime_error("Fixed-mesh diffusion increased its L2 energy");
        previous_energy = current_energy;

        if (viewer && o.fps > 0)
            std::this_thread::sleep_until(started + std::chrono::duration<double>(step/o.fps));
        send_frame(step);
    }
    // Tiny diffusion steps can be below the linear solver/roundoff tolerance;
    // require measurable smoothing only when the accumulated damping is useful.
    if (o.amplitude == 0 && o.diffusivity*o.dt*o.steps > 1e-6 &&
        previous_energy >= initial_energy*(1.0 - 1e-10))
        throw std::runtime_error("Fixed-mesh diffusion did not smooth the initial field");
    std::cout << std::setprecision(10) << "Completed " << o.steps << " steps; t = "
              << o.steps*o.dt << ", heat = " << heat()
              << ", maximum relative heat drift = " << maximum_heat_error
              << ", maximum CG iterations = " << maximum_iterations << ".\n";
    if (viewer) std::cout << "Sent " << o.steps+1 << " complete frames from one mesh registration.\n";
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    Options o;
    CLI::App app{"MFEM diffusion on a pulsating mesh, streamed to RayFEM"};
    app.add_option("--host", o.host, "RayFEM host")->capture_default_str();
    app.add_option("--port", o.port, "RayFEM port")->check(CLI::Range(1,65535));
    app.add_option("--control-port", o.control_port, "Viewer control port; zero uses stream port plus one")->check(CLI::Range(0,65535));
    app.add_option("--viewer", o.viewer_executable, "Optional RayFEM executable path for automatic startup");
    app.add_option("--resolution", o.resolution, "Cells per side")->check(CLI::Range(2,64));
    app.add_option("--order", o.order, "H1 field and geometry order")->check(CLI::Range(1,8));
    app.add_option("--steps", o.steps, "Time steps (plus the initial frame)")->check(CLI::Range(1,1000000));
    app.add_option("--dt", o.dt, "Simulation time step")->check(CLI::Range(1e-6,1.0));
    app.add_option("--diffusivity", o.diffusivity, "Positive diffusion coefficient")->check(CLI::Range(1e-8,10.0));
    app.add_option("--amplitude", o.amplitude, "Pulsation amplitude; zero gives a fixed mesh")->check(CLI::Range(0.0,0.8));
    app.add_option("--period", o.period, "Pulsation period in simulation time")->check(CLI::Range(1e-3,1000.0));
    app.add_option("--fps", o.fps, "Stream cadence; zero sends without pacing")->check(CLI::Range(0.0,1000.0));
    app.add_flag("--no-stream", o.no_stream, "Solve and check numerics without a viewer");
    CLI11_PARSE(app, argc, argv);
    try { return Run(o); }
    catch (const std::exception& error) {
        std::cerr << "Sandbox: " << error.what() << '\n';
        return 1;
    }
}
