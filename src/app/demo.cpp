#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>

// MFEM Headers
#include <mfem.hpp>

#include "lib/visual.hpp"
#include "lib/window.hpp"
#include "lib/script_controller.hpp"
#include "lib/stream_reader.hpp"
#include "lib/file_reader.hpp"
#include "lib/coll_reader.hpp"
#include "lib/sdl/sdl.hpp"
#include "lib/egl/egl.hpp"

// ImGui and SDL2
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <SDL.h>
#include <GL/glew.h>

/**
 * GLVisTextureView: A helper to wrap the GLVis renderer
 * into an OpenGL Framebuffer for ImGui consumption.
 */
class GLVisTextureView {
public:
    GLuint fbo = 0;
    GLuint texture = 0;
    GLuint depth_rbo = 0;
    int width = 800, height = 600;

    void Init(const int w, const int h) {
        width = w; height = h;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

        // GLVis needs a depth buffer for 3D rendering
        glGenRenderbuffers(1, &depth_rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_rbo);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            std::cerr << "FBO Error: Incomplete" << std::endl;

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Resize(int w, int h) {
        if (w <= 0 || h <= 0 || (w == width && h == height)) return;
        width = w; height = h;

        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);

        glBindRenderbuffer(GL_RENDERBUFFER, depth_rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    }
};

struct AppState {
    std::mutex mtx;
    std::unique_ptr<Window> glvis_win;
    std::unique_ptr<gl3::MeshRenderer> renderer;
    int port = 19916;
} g_state;

[[noreturn]] void ServerThread() {
    mfem::socketserver server(g_state.port);
    if (!server.good()) {
        const std::string err_msg = "Failed to start server on port " + std::to_string(g_state.port);
        throw std::runtime_error(err_msg);
    }

    while (true) {
        std::unique_ptr<mfem::socketstream> isock = std::make_unique<mfem::socketstream>();
        if (server.accept(*isock) < 0) continue;

        std::string data_type;
        *isock >> data_type >> std::ws;

        std::lock_guard<std::mutex> lock(g_state.mtx);
        if (g_state.glvis_win) {
            DataState new_state;
            StreamReader reader(new_state);
            if (reader.ReadStream(*isock, data_type) == 0) {
                g_state.glvis_win->SetNewMeshAndSolution(std::move(new_state));
            }
        }
    }
}

int main(int argc, char** argv) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) return -1;

    const char* glsl_version = "#version 330";
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

    SDL_Window* window = SDL_CreateWindow("GLVis ImGui", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1);

    if (glewInit() != GLEW_OK) return -1;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();

    // Use modern SDL2 backend casing
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

    g_state.glvis_win = std::make_unique<Window>();
    g_state.renderer = std::make_unique<gl3::MeshRenderer>();

    GLVisTextureView view;
    view.Init(800, 600);

    std::thread srv(ServerThread);
    srv.detach();

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window)) done = true;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()->ID);

        ImGui::Begin("Controls");
        ImGui::Text("Server on port: %d", g_state.port);
        if (ImGui::Button("Cycle Palette")) {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            if (g_state.glvis_win->vs) g_state.glvis_win->vs->palette.NextIndex();
        }
        ImGui::End();

        ImGui::Begin("Renderer");
        ImVec2 size = ImGui::GetContentRegionAvail();
        view.Resize(static_cast<int>(size.x), static_cast<int>(size.y));

        glBindFramebuffer(GL_FRAMEBUFFER, view.fbo);
        glViewport(0, 0, view.width, view.height);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        {
            std::lock_guard<std::mutex> lock(g_state.mtx);
            if (g_state.glvis_win->vs) {
                glm::mat4 proj = glm::perspective(glm::radians(45.0f), size.x/size.y, 0.1f, 100.0f);
                g_state.glvis_win->vs->SetProjectionMtx(proj);
                const auto scene_info = g_state.glvis_win->vs->GetSceneObjs();
                g_state.renderer->render(scene_info.queue);
            }
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        ImGui::Image((void*)static_cast<intptr_t>(view.texture), size, ImVec2(0, 1), ImVec2(1, 0));
        ImGui::End();

        ImGui::Render();
        glViewport(0, 0, static_cast<int>(io.DisplaySize.x), static_cast<int>(io.DisplaySize.y));
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
