// Widgets in an auto-sizing popup: on its first frame the popup has no
// width yet, and ImGui aborts on zero-sized buttons.
#include "ui/widgets.hpp"

#include <iostream>
#include <stdexcept>

namespace {
void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}

int main() {
    try {
        ImGui::CreateContext();
        auto& io = ImGui::GetIO();
        io.DisplaySize = {800, 600};
        io.DeltaTime = 1.0f / 60;
        io.IniFilename = nullptr;
        unsigned char* pixels = nullptr;
        int w = 0, h = 0;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
        int choice = 0;
        float width = 0;
        for (int frame = 0; frame < 3; ++frame) {
            ImGui::NewFrame();
            ImGui::Begin("host", nullptr, ImGuiWindowFlags_NoDecoration);
            if (frame == 0) ImGui::OpenPopup("settings");
            if (ImGui::BeginPopup("settings")) {
                RayFEM::UI::Segmented("##mode", &choice, {"Oldest first", "Thin older frames"});
                width = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x;
                ImGui::EndPopup();
            }
            ImGui::End();
            ImGui::Render();
        }
        Check(width > 100, "The segments take their labels' width in a new popup");
        ImGui::DestroyContext();
        std::cout << "widgets passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "widgets test failed: " << e.what() << '\n';
        return 1;
    }
}
