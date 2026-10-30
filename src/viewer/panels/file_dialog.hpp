#pragma once

#include "imgui.h"
#include "ui/theme.hpp"
#include "extras/IconsFontAwesome6.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

// A small in-app browser, shared by every platform supported by the viewer.
// Loading is left to the caller; the dialog only chooses an existing archive.
class ArchiveFileDialog {
public:
    void Open() {
        if (directory.empty()) {
            std::error_code ec;
            Navigate(std::filesystem::current_path(ec));
        }
        ImGui::OpenPopup("Open archive");
    }

    std::optional<std::filesystem::path> Draw(bool supported) {
        std::optional<std::filesystem::path> result;
        ImGui::SetNextWindowSize({680, 490}, ImGuiCond_FirstUseEver);
        bool open = true;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,RayFEM::UI::OverlayRounding);
        const bool visible=ImGui::BeginPopupModal("Open archive", &open, ImGuiWindowFlags_NoSavedSettings);
        ImGui::PopStyleVar();
        if (!visible)
            return result;
        if (!supported) {
            ImGui::TextWrapped("This build does not include SERiF archive support.");
            ImGui::Spacing();
            ImGui::TextWrapped("Configure RayFEM with an existing SERiF library to enable .serif files. "
                               "See the optional SERiF section in docs/building.md.");
            if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
            return result;
        }
        ImGui::TextDisabled("Folder or .serif file");
        ImGui::SetNextItemWidth(-1);
        bool choose = ImGui::InputText("##archive_path", path.data(), path.size(),
                                      ImGuiInputTextFlags_EnterReturnsTrue |
                                      ImGuiInputTextFlags_AutoSelectAll);
        if (ImGui::Button(ICON_FA_ARROW_TURN_UP "  Up")) Navigate(directory.parent_path());
        ImGui::SameLine();
        if (ImGui::Button(ICON_FA_ARROWS_ROTATE "  Refresh")) Navigate(directory);
        ImGui::SameLine();
        ImGui::TextDisabled(".serif");
        std::optional<std::filesystem::path> nextDirectory;
        ImGui::BeginChild("Archive files", {0, -88}, ImGuiChildFlags_Borders);
        if (resetScroll) {
            ImGui::SetScrollY(0);
            resetScroll = false;
        }
        const float rowHeight=ImGui::GetFrameHeight();
        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign,{0,.5f});
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(entries.size()),rowHeight+ImGui::GetStyle().ItemSpacing.y);
        while (clipper.Step()) for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
            const auto& entry = entries[i];
            const auto filename = entry.path.filename().string();
            const std::string label = (entry.directory ? ICON_FA_FOLDER "  " : ICON_FA_FILE "  ") + filename;
            ImGui::PushID(filename.c_str());
            if (ImGui::Selectable(label.c_str(), entry.path.string() == path.data(),
                                  ImGuiSelectableFlags_AllowDoubleClick,{0,rowHeight})) {
                SetPath(entry.path);
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    if (entry.directory) nextDirectory = entry.path;
                    else choose = true;
                }
            }
            if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                ImGui::SetTooltip("%s",entry.path.string().c_str());
            ImGui::PopID();
        }
        ImGui::PopStyleVar();
        if (entries.empty() && error.empty()) ImGui::TextDisabled("No archives in this folder");
        ImGui::EndChild();
        if (nextDirectory) Navigate(*nextDirectory);
        if (!error.empty()) ImGui::TextWrapped("%s", error.c_str());
        else ImGui::TextDisabled("Double-click to open, or paste a full path.");
        choose |= ImGui::Button("Open", {90, 0});
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {90, 0})) ImGui::CloseCurrentPopup();
        if (choose) {
            std::filesystem::path candidate(path.data());
            if (candidate.is_relative()) candidate = directory / candidate;
            std::error_code ec;
            if (std::filesystem::is_directory(candidate, ec)) Navigate(candidate);
            else if (!std::filesystem::is_regular_file(candidate, ec))
                error = ec ? ec.message() : "Choose an existing .serif file.";
            else if (candidate.extension() != ".serif")
                error = "Choose a .serif archive.";
            else {
                result = candidate;
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::EndPopup();
        return result;
    }

private:
    struct Entry { std::filesystem::path path; bool directory; };
    void SetPath(const std::filesystem::path& value) {
        const auto text = value.string();
        if (text.size() >= path.size()) {
            error = "This path is too long for the file browser.";
            return;
        }
        std::snprintf(path.data(), path.size(), "%s", text.c_str());
    }
    void Navigate(const std::filesystem::path& value) {
        std::error_code ec;
        auto absolute = std::filesystem::absolute(value, ec).lexically_normal();
        if (ec) { error = ec.message(); return; }
        std::filesystem::directory_iterator it(absolute, ec);
        if (ec) { error = ec.message(); return; }
        directory = absolute;
        resetScroll = true;
        entries.clear();
        error.clear();
        SetPath(directory);
        size_t visited = 0;
        for (const auto end = std::filesystem::directory_iterator{}; it != end; it.increment(ec)) {
            if (ec) break;
            if (++visited > 20000) {
                error = "Large folder: showing the first 20,000 entries. Paste a full path to open another file.";
                break;
            }
            const auto& item = *it;
            std::error_code entryError;
            const bool folder = item.is_directory(entryError);
            if (entryError) continue;
            if (!folder && (item.path().extension() != ".serif" || !item.is_regular_file(entryError))) continue;
            if (item.path().filename().string().starts_with('.')) continue;
            entries.push_back({item.path(), folder});
        }
        if (ec) error = ec.message();
        std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
            return a.directory != b.directory ? a.directory > b.directory : a.path.filename() < b.path.filename();
        });
    }
    std::filesystem::path directory;
    std::array<char, 4096> path{};
    std::vector<Entry> entries;
    std::string error;
    bool resetScroll = false;
};
