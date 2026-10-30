#pragma once

// The viewer's stream listener, which can move to another host and port while
// the viewer runs. It owns the RayFEM::Server, so the frame pipeline's worker
// fetches through it while the UI thread replaces the server underneath.
//
// A move opens the new endpoint before closing the old one, so a failed bind
// (a port in use, an unknown address) leaves the viewer listening where it
// was. When the new endpoint shares a port with the current one, the current
// one has to close first; if the new one then fails, the old one is reopened.
// Frames already decoded but not yet fetched are kept across a move; counters
// add up across listeners. Mesh registrations belong to a listener, so clients
// send their mesh again after reconnecting, as they do on startup.

#include "libRayfem/network/server.hpp"
#include "ui/theme.hpp"
#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace RayFEM::UI {

struct Endpoint {
    std::string host = "127.0.0.1";
    int port = 5555;
    int control = 5556;  // init, stats and shutdown; 0 for none
    bool operator==(const Endpoint&) const = default;
};

// Why an endpoint cannot be used, or empty when it can.
inline std::string CheckEndpoint(const Endpoint& endpoint) {
    if (endpoint.host.empty()) return "Enter a host to listen on";
    if (endpoint.host.find_first_of(" \t/") != std::string::npos) return "The host must be an address or name, without a scheme or path";
    if (endpoint.port < 1 || endpoint.port > 65535) return "The port must be between 1 and 65535";
    if (endpoint.control < 0 || endpoint.control > 65535) return "The control port must be between 1 and 65535";
    if (endpoint.control == endpoint.port) return "The control port must differ from the stream port";
    return {};
}

// Addresses that accept connections from other machines.
inline bool ListensOnNetwork(const std::string& host) {
    return host != "127.0.0.1" && host != "localhost" && host != "::1" && host != "[::1]";
}

// Where clients should send: a wildcard host is not an address to connect to.
inline std::string ClientTarget(const Endpoint& endpoint) {
    if (endpoint.host == "0.0.0.0" || endpoint.host == "::" || endpoint.host == "*")
        return "port " + std::to_string(endpoint.port) + " on this machine";
    return endpoint.host + ":" + std::to_string(endpoint.port);
}

class Listener {
public:
    explicit Listener(Endpoint endpoint) : endpoint_(std::move(endpoint)) {
        std::string error;
        server_ = Start(endpoint_, error);
        if (!server_) throw std::runtime_error(error);
    }
    Listener(const Listener&) = delete;
    Listener& operator=(const Listener&) = delete;

    // Any thread (the frame pipeline's worker); frames from before a move first.
    std::vector<std::shared_ptr<RenderFrame>> fetchFrames(size_t max_frames) {
        std::lock_guard lock(mutex_);
        std::vector<std::shared_ptr<RenderFrame>> result;
        while (!carried_.empty() && result.size() < max_frames) {
            result.push_back(std::move(carried_.front()));
            carried_.erase(carried_.begin());
        }
        if (server_ && result.size() < max_frames)
            for (auto& frame : server_->fetchFrames(max_frames - result.size())) result.push_back(std::move(frame));
        return result;
    }

    // UI thread. On failure the listener keeps (or reopens) its endpoint and
    // error says why.
    bool Move(const Endpoint& next, std::string& error) {
        error = CheckEndpoint(next);
        if (!error.empty()) return false;
        if (next == endpoint_ && server_ && server_->ready()) return true;
        if (auto opened = Start(next, error)) {
            Replace(std::move(opened));
            endpoint_ = next;
            return true;
        }
        if (!SharesPort(next, endpoint_)) return false;
        // The old listener holds a port the new one needs: close it and retry.
        Replace(nullptr);
        if (auto opened = Start(next, error)) {
            Replace(std::move(opened));
            endpoint_ = next;
            return true;
        }
        std::string reopen;
        if (auto restored = Start(endpoint_, reopen)) Replace(std::move(restored));
        else error += "; could not reopen " + endpoint_.host + ":" + std::to_string(endpoint_.port) + ": " + reopen;
        return false;
    }

    const Endpoint& endpoint() const { return endpoint_; }
    bool listening() const { return server_ && server_->ready(); }

    // The Server interface the viewer uses, on the UI thread. Counts add up
    // across moves.
    bool ready() const { return listening(); }
    bool receiving() const { return receiving_; }
    void setReceiving(bool receiving) { receiving_ = receiving; if (server_) server_->setReceiving(receiving); }
    void markViewerReady(bool ready = true) { viewer_ready_ = ready; if (server_) server_->markViewerReady(ready); }
    bool shutdownRequested() const { return server_ && server_->shutdownRequested(); }
    size_t getReceivedFrameCount() const { return base_.received + (server_ ? server_->getReceivedFrameCount() : 0); }
    size_t getDroppedFrameCount() const { return base_.dropped + (server_ ? server_->getDroppedFrameCount() : 0); }
    size_t getErrorCount() const { return base_.errors + (server_ ? server_->getErrorCount() : 0); }
    size_t getSkippedFrameCount() const { return base_.skipped + (server_ ? server_->getSkippedFrameCount() : 0); }
    size_t getPendingFrameCount() const { return server_ ? server_->getPendingFrameCount() : 0; }
    std::string getLastError() const {
        if (!server_) return "Not listening";
        return server_->getLastError();
    }

private:
    struct Counts { size_t received = 0, dropped = 0, errors = 0, skipped = 0; };

    static bool SharesPort(const Endpoint& a, const Endpoint& b) {
        auto has = [](const Endpoint& e, int port) { return port && (e.port == port || e.control == port); };
        return has(b, a.port) || has(b, a.control);
    }

    std::unique_ptr<Server> Start(const Endpoint& endpoint, std::string& error) const {
        std::unique_ptr<Server> server;
        try {
            server = std::make_unique<Server>(endpoint.host, static_cast<size_t>(endpoint.port),
                                              static_cast<size_t>(endpoint.control));
        } catch (const std::exception& e) { error = e.what(); return nullptr; }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!server->ready() && !server->getErrorCount() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (!server->ready()) {
            error = server->getLastError().empty() ? "Timed out starting the listener" : server->getLastError();
            return nullptr;
        }
        server->setReceiving(receiving_);
        server->markViewerReady(viewer_ready_);
        return server;
    }

    // Swap in next (or nothing), keeping frames the old listener decoded.
    void Replace(std::unique_ptr<Server> next) {
        std::unique_ptr<Server> previous;
        {
            std::lock_guard lock(mutex_);
            if (server_) {
                for (auto frames = server_->fetchFrames(64); !frames.empty(); frames = server_->fetchFrames(64))
                    for (auto& frame : frames) carried_.push_back(std::move(frame));
                base_.received += server_->getReceivedFrameCount();
                base_.dropped += server_->getDroppedFrameCount();
                base_.errors += server_->getErrorCount();
                base_.skipped += server_->getSkippedFrameCount();
            }
            previous = std::move(server_);
            server_ = std::move(next);
        }
        // Closing joins the old network thread; do it outside the lock.
        if (previous) previous->markViewerReady(false);
    }

    Endpoint endpoint_;
    std::unique_ptr<Server> server_;
    mutable std::mutex mutex_;
    std::vector<std::shared_ptr<RenderFrame>> carried_;
    Counts base_;
    bool receiving_ = true, viewer_ready_ = false;
};

// The Listener section of the session menu: host, ports and Apply.
struct ListenerSettings {
    char host[256] = {};
    int port = 0, control = 0;
    bool loaded = false;
    std::string message;  // the outcome of the last Apply
    bool failed = false;

    void Load(const Endpoint& endpoint) {
        std::snprintf(host, sizeof(host), "%s", endpoint.host.c_str());
        port = endpoint.port;
        control = endpoint.control;
        loaded = true;
    }
    Endpoint Edited() const { return {host, port, control}; }
};

inline void DrawListenerSettings(Listener& listener, ListenerSettings& settings) {
    const Endpoint& current = listener.endpoint();
    if (!settings.loaded) settings.Load(current);
    auto tip = [](const char* text) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s", text);
    };
    const float field = ImGui::GetFontSize()*9;
    ImGui::SetNextItemWidth(field);
    ImGui::InputText("##listen_host", settings.host, sizeof(settings.host), ImGuiInputTextFlags_CharsNoBlank);
    tip("The address to listen on: 127.0.0.1 for this machine only, 0.0.0.0 (or ::) for every network interface.");
    ImGui::SameLine(0, 0);
    if (ImGui::BeginCombo("##listen_presets", nullptr, ImGuiComboFlags_NoPreview)) {
        const char* presets[][2] = {{"127.0.0.1", "This machine only"}, {"0.0.0.0", "All IPv4 interfaces"}, {"::", "All interfaces"}};
        for (const auto& [address, what] : presets)
            if (ImGui::Selectable((std::string(address) + "  " + what).c_str(), std::string_view(settings.host) == address))
                std::snprintf(settings.host, sizeof(settings.host), "%s", address);
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Host");
    const int previousPort = settings.port;
    ImGui::SetNextItemWidth(field);
    if (ImGui::InputInt("Port", &settings.port, 1, 10) && settings.control == previousPort + 1)
        settings.control = settings.port + 1;  // the control port follows when it was the default
    settings.port = std::clamp(settings.port, 1, 65535);
    tip("The port clients stream frames to.");
    ImGui::SetNextItemWidth(field);
    ImGui::InputInt("Control port", &settings.control, 1, 10);
    settings.control = std::clamp(settings.control, 1, 65535);
    tip("The port rayfem.init, stats and shutdown use; by default the stream port + 1.");

    const Endpoint edited = settings.Edited();
    const std::string problem = CheckEndpoint(edited);
    const bool changed = !(edited == current) || !listener.listening();
    if (ListensOnNetwork(edited.host))
        ImGui::TextColored(ThemeText(Palette().warning), "Other machines can reach this address. The stream\n"
                           "and control ports have no authentication, and the\ncontrol port can close the viewer.");
    if (!problem.empty()) ImGui::TextColored(ThemeText(Palette().warning), "%s", problem.c_str());
    ImGui::BeginDisabled(!changed || !problem.empty());
    if (ImGui::Button("Apply")) {
        std::string error;
        settings.failed = !listener.Move(edited, error);
        settings.message = settings.failed ? error
            : "Listening on " + edited.host + ":" + std::to_string(edited.port) + ". Clients streaming to the old "
              "address now have their sends refused; reconnect them here. Python's rayfem.stats and "
              "rayfem.shutdown need port=" + std::to_string(edited.port) + ".";
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(edited == current);
    if (ImGui::Button("Revert")) { settings.Load(current); settings.message.clear(); }
    ImGui::EndDisabled();
    if (!settings.message.empty()) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + ImGui::GetFontSize()*22);
        ImGui::TextColored(ThemeText(settings.failed ? Palette().warning : Palette().success), "%s", settings.message.c_str());
        ImGui::PopTextWrapPos();
    }
}

} // namespace RayFEM::UI
