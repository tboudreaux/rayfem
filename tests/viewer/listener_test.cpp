// Moving the viewer's listener while it runs, and what clients report when
// the viewer they stream to has moved.
#include "libRayfem/frame/tracking.hpp"  // before zmq.hpp: its syslog macros clash with raylib
#include "libRayfem/init.hpp"
#include "libRayfem/network/client.hpp"
#include "pipeline/listener.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void WaitFor(const std::function<bool()>& condition, const std::string& message) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (!condition()) {
        if (std::chrono::steady_clock::now() > deadline) throw std::runtime_error(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

struct Scene {
    mfem::Mesh mesh = mfem::Mesh::MakeCartesian2D(3, 3, mfem::Element::QUADRILATERAL);
    mfem::H1_FECollection collection{1, 2};
    mfem::FiniteElementSpace space{&mesh, &collection};
    mfem::GridFunction u{&space};
    Scene() { u = 1.0; }
    std::vector<RayFEM::NamedField> Fields() { return {{"u", &u}}; }
};

RayFEM::FrameMetadata Frame(uint32_t index) {
    RayFEM::FrameMetadata metadata;
    metadata.frame_index = index;
    metadata.sim_time = index;
    return metadata;
}

// Frames the listener has decoded, counted as they are fetched.
size_t Receive(RayFEM::UI::Listener& listener, size_t count, size_t& total) {
    WaitFor([&] { total += listener.fetchFrames(16).size(); return total >= count; },
            "Timed out receiving frames: " + listener.getLastError());
    return total;
}

bool Mentions(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }
} // namespace

int main() {
    try {
        // Ports from the process id, so parallel test runs do not collide.
        const int base = 41000 + static_cast<int>(getpid() % 4000) * 4;
        const std::string host = "127.0.0.1";
        const RayFEM::UI::Endpoint first{host, base, base + 1};
        const RayFEM::UI::Endpoint second{host, base + 2, base + 3};
        Scene scene;
        size_t fetched = 0;

        RayFEM::UI::Listener listener(first);
        listener.markViewerReady();
        RayFEM::Client client(host, static_cast<size_t>(first.port));
        Check(client.sendMesh(1, scene.mesh) && client.sendFrame(1, scene.Fields(), Frame(0)),
              "Send to the first listener: " + client.getLastError());
        Receive(listener, 1, fetched);
        Check(client.connected(), "The client sees its viewer");

        // Frames that arrived before a move but were not yet fetched survive it.
        Check(client.sendFrame(1, scene.Fields(), Frame(1)), "Second frame: " + client.getLastError());
        WaitFor([&] { return listener.getPendingFrameCount() >= 1; }, "Second frame did not arrive");
        std::string error;
        Check(listener.Move(second, error), "Move to a free port: " + error);
        Check(listener.endpoint() == second && listener.listening(), "Listening on the new endpoint");
        Receive(listener, 2, fetched);
        Check(listener.getReceivedFrameCount() == 2, "Counts carry across a move");

        // The moved viewer answers init and stats on its new control port.
        RayFEM::WindowParams params;
        params.host = host;
        params.port = second.port;
        params.control_port = second.control;
        params.timeout_ms = 2000;
        const auto stats = RayFEM::stats(params);
        Check(stats.error.empty(), "Stats on the new control port: " + stats.error);
        params.port = first.port;
        params.control_port = first.control;
        params.timeout_ms = 300;
        Check(!RayFEM::stats(params).error.empty(), "Nothing answers on the old control port");

        // A client of the old address has its sends refused, not queued.
        WaitFor([&] { return !client.connected(); }, "The client notices the viewer left");
        Check(!client.sendFrame(1, scene.Fields(), Frame(2)), "A frame to a moved viewer is refused");
        Check(Mentions(client.getLastError(), "No viewer is listening on " + host + ":" + std::to_string(first.port)),
              "Refusal names the address: " + client.getLastError());
        Check(!client.sendMesh(2, scene.mesh), "A mesh to a moved viewer is refused");

        // A port in use: the move fails and the listener stays where it was.
        RayFEM::Server occupant(host, static_cast<size_t>(first.port), 0);
        WaitFor([&] { return occupant.ready() || occupant.getErrorCount(); }, "Occupant did not start");
        Check(occupant.ready(), "Occupant listening");
        Check(!listener.Move({host, first.port, base + 3}, error) && !error.empty(), "A port in use is refused");
        Check(listener.endpoint() == second && listener.listening(), "A failed move keeps the listener");
        RayFEM::Client moved(host, static_cast<size_t>(second.port));
        Check(moved.sendMesh(1, scene.mesh) && moved.sendFrame(1, scene.Fields(), Frame(0)),
              "Send after a failed move: " + moved.getLastError());
        Receive(listener, 3, fetched);

        // The same stream port with another control port has to close first;
        // the connected client comes back to a listener without its mesh.
        const RayFEM::UI::Endpoint rebound{host, second.port, base + 1};
        Check(listener.Move(rebound, error), "Move sharing the stream port: " + error);
        Check(listener.endpoint() == rebound, "Rebound endpoint");
        WaitFor([&] { return moved.connected() && !moved.sendFrame(1, scene.Fields(), Frame(1)); },
                "The client reconnects and its frame is refused");
        Check(Mentions(moved.getLastError(), "no longer has the mesh"), "Refusal says why: " + moved.getLastError());
        Check(moved.sendMesh(1, scene.mesh) && moved.sendFrame(1, scene.Fields(), Frame(1)),
              "Send after re-registering: " + moved.getLastError());
        Receive(listener, 4, fetched);

        // A failed move that shared the port reopens the old endpoint.
        Check(!listener.Move({"203.0.113.7", rebound.port, rebound.control}, error), "An address not on this machine fails");
        Check(listener.endpoint() == rebound && listener.listening(), "The old endpoint is reopened: " + listener.getLastError());
        Check(!listener.Move({host, rebound.port, rebound.port}, error) && Mentions(error, "control port"), "Invalid endpoints are checked");

        std::cout << "listener moves and client refusals passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "listener test failed: " << e.what() << '\n';
        return 1;
    }
}
