#include <print>
#include "raylib.h"

#include "libRayfem/network/server.hpp"

#include "fourdst/config/config.h"

#include "CLI/CLI.hpp"
#include "czmq.h"

struct Options {
    std::string listen = "0.0.0.0";
    int port = 5555;
    int target_fps = 60;
    int screenwidth = 800;
    int screenheight = 600;
};

int main(int argc, char** argv) {
    zsys_init();

    fourdst::config::Config<Options> config;
    CLI::App app{"RayFEM Server"};
    fourdst::config::register_as_cli(config, app, "config");
    CLI11_PARSE(app, argc, argv)

    const int screenwidth = config->screenwidth;
    const int screenheight = config->screenheight;

    InitWindow(screenwidth, screenheight, "raylib");

    SetTargetFPS(config->target_fps);

    RayFEM::Server server(config->listen, config->port);
    bool hasRecivedData = false;

    while (!WindowShouldClose()) {
        auto frame = server.fetchLatestFrame();
        if (frame) {
            hasRecivedData = true;
        }

        BeginDrawing();
            ClearBackground(RAYWHITE);
            std::string msg = std::format("Has Received {} frames", server.getReceivedFrameCount());
            DrawText(msg.c_str(), 250, 280, 20, DARKGREEN);
        EndDrawing();
    }
    CloseWindow();
}