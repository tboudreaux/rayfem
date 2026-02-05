#include "libRayfem/network/client.hpp"
#include "mfem.hpp"
#include "fourdst/config/config.h"
#include "CLI/CLI.hpp"
#include <print>

struct config {
    std::string host = "localhost";
    int port = 5555;
    std::string mesh_file = "stroid.mesh";
};

int main(int argc, char** argv) {
    fourdst::config::Config<config> cfg;
    CLI::App app{"RayFEM Client"};
    fourdst::config::register_as_cli(cfg, app, "config");
    CLI11_PARSE(app, argc, argv)

    auto mesh = std::make_unique<mfem::Mesh>(cfg->mesh_file.c_str());
    mesh->EnsureNodes();

    mfem::FiniteElementSpace fespace(mesh.get(), new mfem::H1_FECollection(1, mesh->Dimension()), 1);
    mfem::GridFunction gf(&fespace);
    gf = 0.5;

    RayFEM::Client client(cfg->host, cfg->port);

    RayFEM::FrameMetadata meta;
    meta.frame_index = 0;
    meta.sim_time = 0.0;


    if (client.send(*mesh, gf, meta)) {
        std::println("Frame sent successfully.");
    } else {
        std::println("Failed to send frame.");
    }


    return 0;
}