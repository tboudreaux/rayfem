// A 2D heat equation solved with MFEM and streamed to a running viewer.
//
//   du/dt = kappa * Laplacian(u)  on the unit square,  u = 0 on its boundary,
//   u(x, y, 0) = sin(pi x) sin(pi y),
//
// whose exact solution is exp(-2 pi^2 kappa t) sin(pi x) sin(pi y). Backward
// Euler: (M + dt*kappa*K) u_new = M u_old. Each step streams the temperature,
// the exact solution and the error, with the step and the L2 error as frame
// values. The program fails if the final error or the maximum principle is
// out of bounds, or if a send is refused.
//
//   diffusion_stream --port 5555 [--host 127.0.0.1] [--steps 40] [--dt 0.01]
//                    [--kappa 0.1] [--order 2] [--cells 12] [--pace 0]
//
// It does not start a viewer; python/tests/test_diffusion_stream.py starts one
// with the rayfem Python package and checks that every frame arrived.
#include "libRayfem/network/client.hpp"
#include "mfem.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    std::string host = "127.0.0.1";
    int port = 5555, steps = 40, order = 2, cells = 12;
    double dt = 0.01, kappa = 0.1, pace = 0;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string option = argv[i], value = argv[i + 1];
        if (option == "--host") host = value;
        else if (option == "--port") port = std::stoi(value);
        else if (option == "--steps") steps = std::stoi(value);
        else if (option == "--order") order = std::stoi(value);
        else if (option == "--cells") cells = std::stoi(value);
        else if (option == "--dt") dt = std::stod(value);
        else if (option == "--kappa") kappa = std::stod(value);
        else if (option == "--pace") pace = std::stod(value);
        else { std::fprintf(stderr, "Unknown option %s\n", option.c_str()); return 2; }
    }
    constexpr double pi = std::numbers::pi;
    const double rate = 2*pi*pi*kappa;

    mfem::Mesh mesh = mfem::Mesh::MakeCartesian2D(cells, cells, mfem::Element::QUADRILATERAL, true, 1.0, 1.0);
    mfem::H1_FECollection collection(order, 2);
    mfem::FiniteElementSpace space(&mesh, &collection);
    mfem::Array<int> boundary(mesh.bdr_attributes.Max()), essential;
    boundary = 1;
    space.GetEssentialTrueDofs(boundary, essential);

    double time = 0;
    mfem::FunctionCoefficient exact([&](const mfem::Vector& x) {
        return std::exp(-rate*time)*std::sin(pi*x[0])*std::sin(pi*x[1]);
    });
    mfem::GridFunction u(&space), reference(&space), error(&space);
    u.ProjectCoefficient(exact);

    mfem::BilinearForm mass(&space), system(&space);
    mass.AddDomainIntegrator(new mfem::MassIntegrator);
    mass.Assemble();
    mass.Finalize();
    mfem::ConstantCoefficient diffusivity(dt*kappa);
    system.AddDomainIntegrator(new mfem::MassIntegrator);
    system.AddDomainIntegrator(new mfem::DiffusionIntegrator(diffusivity));
    system.Assemble();

    RayFEM::Client client(host, static_cast<size_t>(port));
    RayFEM::FrameSchema schema;
    schema.values = {{"step", RayFEM::ValueType::Integer, true, "", "Backward Euler step"},
                     {"L2 error", RayFEM::ValueType::Real, false, "", "L2 norm of the error against the exact solution"},
                     {"max", RayFEM::ValueType::Real, false, "", "Largest temperature"}};
    schema.tags = {{"final", "The last step"}};
    if (!client.registerSchema(schema) || !client.sendMesh(1, mesh)) {
        std::fprintf(stderr, "Send refused: %s\n", client.getLastError().c_str());
        return 1;
    }
    const std::vector<RayFEM::NamedField> fields = {{"temperature", &u}, {"exact", &reference}, {"error", &error}};

    mfem::Vector rhs(space.GetVSize());
    mfem::GSSmoother smoother;
    mfem::CGSolver solver;
    solver.SetRelTol(1e-12);
    solver.SetMaxIter(2000);
    double previous_max = u.Max();
    bool monotone = true;
    for (int step = 0; step <= steps; ++step) {
        if (step > 0) {
            // (M + dt*kappa*K) u_new = M u_old, with u = 0 on the boundary.
            mass.Mult(u, rhs);
            mfem::GridFunction next(&space);
            next = 0.0;
            mfem::OperatorPtr matrix;
            mfem::Vector solution, right;
            system.FormLinearSystem(essential, next, rhs, matrix, solution, right);
            smoother.SetOperator(*matrix);
            solver.SetPreconditioner(smoother);
            solver.SetOperator(*matrix);
            solver.Mult(right, solution);
            system.RecoverFEMSolution(solution, rhs, next);
            u = next;
            time += dt;
            monotone = monotone && u.Max() <= previous_max + 1e-12 && u.Min() >= -1e-10;
            previous_max = u.Max();
        }
        reference.ProjectCoefficient(exact);
        error = u;
        error -= reference;
        const double l2 = u.ComputeL2Error(exact);
        RayFEM::FrameMetadata metadata;
        metadata.frame_index = static_cast<uint32_t>(step);
        metadata.sim_time = time;
        std::snprintf(metadata.label, sizeof(metadata.label), "heat kappa=%g dt=%g", kappa, dt);
        metadata.values = {{"step", int64_t{step}}, {"L2 error", l2}, {"max", u.Max()}};
        if (step == steps) metadata.tags = {"final"};
        if (!client.sendFrame(1, fields, metadata)) {
            std::fprintf(stderr, "Frame %d refused: %s\n", step, client.getLastError().c_str());
            return 1;
        }
        if (pace > 0) std::this_thread::sleep_for(std::chrono::duration<double>(pace));
    }
    // Relative to the exact solution's L2 norm, exp(-rate*t)/2.
    const double relative = u.ComputeL2Error(exact)/(std::exp(-rate*time)/2);
    std::printf("STREAMED %d frames; t=%.3f, relative L2 error %.3e, maximum principle %s\n", steps + 1, time, relative,
                monotone ? "held" : "violated");
    // Backward Euler's first-order time error dominates: about rate^2*dt*t/2.
    return relative < 0.05 && monotone ? 0 : 1;
}
