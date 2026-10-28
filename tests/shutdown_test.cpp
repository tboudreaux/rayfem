#include "libRayfem/init.hpp"
#include "libRayfem/network/server.hpp"
#include "libRayfem/network/zmq.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>

#if defined(__APPLE__) || defined(__linux__)
#include <unistd.h>
#endif

namespace {
using namespace RayFEM;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void Ready(Server& server) {
    const auto deadline = Clock::now() + 3s;
    while (!server.ready() && !server.getErrorCount() && Clock::now() < deadline) std::this_thread::sleep_for(5ms);
    Check(server.ready(), "Server startup: " + server.getLastError());
}

// The real receiver and lifecycle protocol, with a main loop that models UI
// cleanup. A shutdown acknowledgement alone must not count as a closed viewer.
struct Viewer {
    std::atomic<bool> stop{false}, requested{false}, cleaned{false};
    std::thread thread;
    Viewer(int port, int control, bool ready = true, bool honor = true, int cleanup_ms = 0) {
        std::promise<void> startup;
        auto started = startup.get_future();
        thread = std::thread([&, port, control, ready, honor, cleanup_ms, startup = std::move(startup)]() mutable {
            try {
                Server server("127.0.0.1", port, control);
                Ready(server);
                server.markViewerReady(ready);
                startup.set_value();
                while (!stop) {
                    requested = server.shutdownRequested();
                    if (requested && honor) break;
                    std::this_thread::sleep_for(2ms);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(cleanup_ms));
                cleaned = true;
            } catch (...) { startup.set_exception(std::current_exception()); }
        });
        try { started.get(); } catch (...) { stop = true; thread.join(); throw; }
    }
    ~Viewer() { stop = true; thread.join(); }
};

struct ForeignControl {
    std::atomic<bool> stop{false};
    std::atomic<int> shutdowns{0};
    std::thread thread;
    ForeignControl(int port, bool valid_probe) {
        std::promise<void> startup;
        auto started = startup.get_future();
        thread = std::thread([&, port, valid_probe, startup = std::move(startup)]() mutable {
            try {
                SocketPtr socket(zsock_new(ZMQ_REP));
                zsock_set_linger(socket, 0); zsock_set_rcvtimeo(socket, 20);
                Check(zsock_bind(socket, "tcp://127.0.0.1:%d", port) >= 0, "Foreign control bind");
                startup.set_value();
                while (!stop) {
                    FramePtr frame(zframe_recv(socket));
                    if (!frame) continue;
                    const std::string request(reinterpret_cast<const char*>(zframe_data(frame)), zframe_size(frame));
                    if (request.starts_with("RAYFEM_SHUTDOWN")) ++shutdowns;
                    const std::string response = valid_probe && request.starts_with("RAYFEM_PROBE ")
                        ? "RAYFEM_READY" + request.substr(12) : "RAYFEM_INVALID 1";
                    zstr_send(socket, response.c_str());
                }
            } catch (...) { startup.set_exception(std::current_exception()); }
        });
        try { started.get(); } catch (...) { stop = true; thread.join(); throw; }
    }
    ~ForeignControl() { stop = true; thread.join(); }
};

std::string Request(int control, const std::string& message, bool multipart = false) {
    SocketPtr socket(zsock_new(ZMQ_REQ));
    zsock_set_linger(socket, 0); zsock_set_sndtimeo(socket, 1000); zsock_set_rcvtimeo(socket, 1000);
    Check(zsock_connect(socket, "tcp://127.0.0.1:%d", control) == 0, "Test control connect");
    if (multipart) { zstr_sendm(socket, message.c_str()); zstr_send(socket, "extra"); }
    else zstr_send(socket, message.c_str());
    FramePtr frame(zframe_recv(socket));
    Check(bool(frame.get()), "Test control reply");
    return {reinterpret_cast<const char*>(zframe_data(frame)), zframe_size(frame)};
}

#if defined(__APPLE__) || defined(__linux__)
int Helper(int argc, char** argv) {
    std::map<std::string, std::string> options;
    for (int i = 1; i + 1 < argc; i += 2) options[argv[i]] = argv[i + 1];
    Server server(options.at("--listen"), std::stoi(options.at("--port")), std::stoi(options.at("--control-port")));
    Ready(server); server.markViewerReady();
    const auto deadline = Clock::now() + 8s;
    while (!server.shutdownRequested() && Clock::now() < deadline) std::this_thread::sleep_for(5ms);
    if (!server.shutdownRequested()) return 1;
    // Stand in for renderer/graphics destruction before receiver destruction.
    server.markViewerReady(false); std::this_thread::sleep_for(150ms);
    return 0;
}

void Run(const char* self) {
    const int base = 51000 + (getpid() % 1000) * 10;
    WindowParams parameters; parameters.port = base; parameters.timeout_ms = 1500;
    // Irrelevant launch options must not prevent shutdown.
    parameters.width = -1; parameters.height = -1; parameters.executable = "/missing-viewer";
    Check(shutdown(parameters).status == ShutdownStatus::NotRunning, "Absent local viewer");
    for (int invalid : {0, -1, std::numeric_limits<int>::max()}) {
        auto bad = parameters; bad.port = invalid;
        Check(shutdown(bad).status == ShutdownStatus::Failed, "Invalid stream port");
    }
    { auto bad = parameters; bad.control_port = bad.port;
      Check(shutdown(bad).status == ShutdownStatus::Failed, "Equal ports");
      bad = parameters; bad.host = "tcp://127.0.0.1";
      Check(shutdown(bad).status == ShutdownStatus::Failed, "URL host");
      bad = parameters; bad.timeout_ms = 0;
      Check(shutdown(bad).status == ShutdownStatus::Failed, "Invalid timeout"); }
    {
        Viewer viewer(base, base + 1, true, true, 180);
        const auto start = Clock::now();
        const auto result = shutdown(parameters);
        Check(result.status == ShutdownStatus::Stopped, "Graceful shutdown: " + result.error);
        Check(viewer.requested && viewer.cleaned && Clock::now() - start >= 180ms, "Wait for cleanup before success");
        Check(shutdown(parameters).status == ShutdownStatus::NotRunning, "Repeated shutdown is idempotent");
    }
    parameters.port = base + 2; parameters.control_port = base + 40;
    {
        Viewer viewer(parameters.port, parameters.control_port, false);
        auto wrong = parameters; wrong.port += 1;
        Check(shutdown(wrong).status == ShutdownStatus::Failed, "Wrong stream endpoint rejected");
        Check(!viewer.requested, "Mismatched endpoint cannot shut down viewer");
        Check(shutdown(parameters).status == ShutdownStatus::Stopped, "Shutdown during startup on custom control port");
    }
    parameters.port = base + 4; parameters.control_port = 0;
    { Viewer stream_only(parameters.port, 0);
      Check(shutdown(parameters).status == ShutdownStatus::Failed, "A stream listener is not an absent viewer"); }
    { ForeignControl foreign(parameters.port + 1, false);
      Check(shutdown(parameters).status == ShutdownStatus::Failed, "Foreign endpoint must be rejected");
      Check(foreign.shutdowns == 0, "Never send shutdown to an unverified service"); }
    { ForeignControl old_viewer(parameters.port + 1, true);
      Check(shutdown(parameters).status == ShutdownStatus::Failed, "Old viewer without shutdown support must fail");
      Check(old_viewer.shutdowns == 1, "Old viewer handshake verified before shutdown attempt"); }
    parameters.port = base + 6;
    {
        Viewer stalled(parameters.port, parameters.port + 1, true, false);
        auto brief = parameters; brief.timeout_ms = 250;
        const auto result = shutdown(brief);
        Check(result.status == ShutdownStatus::Failed && result.error.find("finish closing") != std::string::npos,
              "Acknowledgement without teardown must time out: " + result.error);
        Check(stalled.requested && !stalled.cleaned, "A timeout must never kill the viewer");
    }
    parameters.port = base + 8;
    {
        Viewer viewer(parameters.port, parameters.port + 1);
        const std::string suffix = " 1 " + std::to_string(parameters.port) + " abc123";
        Check(Request(parameters.port + 1, "RAYFEM_SHUTDOWN 2 " + std::to_string(parameters.port) + " abc123") == "RAYFEM_INVALID 1", "Wrong protocol version");
        Check(Request(parameters.port + 1, "RAYFEM_SHUTDOWN" + suffix + " extra") == "RAYFEM_INVALID 1", "Extra tokens");
        Check(Request(parameters.port + 1, "RAYFEM_SHUTDOWN" + suffix, true) == "RAYFEM_INVALID 1", "Multipart shutdown rejected");
        Check(Request(parameters.port + 1, "RAYFEM_PROBE" + suffix).starts_with("RAYFEM_READY"), "Malformed requests leave viewer usable");
        auto a = std::async(std::launch::async, [&] { return shutdown(parameters); });
        auto b = std::async(std::launch::async, [&] { return shutdown(parameters); });
        const auto first = a.get(), second = b.get();
        Check((first.status == ShutdownStatus::Stopped && second.status == ShutdownStatus::NotRunning) ||
              (second.status == ShutdownStatus::Stopped && first.status == ShutdownStatus::NotRunning), "Concurrent shutdown is idempotent");
    }
    parameters = WindowParams{}; parameters.port = base + 10;
    parameters.executable = std::filesystem::canonical(self).string(); parameters.timeout_ms = 3000;
    for (int cycle = 0; cycle < 2; ++cycle) {
        const auto started = init(parameters);
        Check(started.status == InitStatus::Started, "Real child process launch/relaunch: " + started.error);
        Check(init(parameters).status == InitStatus::AlreadyRunning, "Reuse before shutdown");
        const auto stopped = shutdown(parameters);
        Check(stopped.status == ShutdownStatus::Stopped, "Real child graceful exit: " + stopped.error);
    }
    Check(bool(ShutdownResult{ShutdownStatus::NotRunning, {}}) && !bool(ShutdownResult{}), "Result boolean semantics");
}
#endif
} // namespace

int main(int argc, char** argv) {
    try {
#if defined(__APPLE__) || defined(__linux__)
        if (argc > 1 && std::string(argv[1]) == "--listen") return Helper(argc, argv);
        Run(argv[0]);
        std::cout << "Shutdown checks passed: graceful cleanup, endpoint identity, custom ports, startup, absence, old viewers, malformed requests, timeout, concurrency, and process restart.\n";
        return 0;
#else
        std::cout << "Shutdown integration tests require macOS/Linux\n";
        return 77;
#endif
    } catch (const std::exception& error) { std::cerr << "Shutdown test: " << error.what() << '\n'; return 1; }
}
