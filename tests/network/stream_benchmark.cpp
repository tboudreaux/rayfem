// How long a solver waits on streaming. Streams an H1 mesh with scalar
// fields and a displacement to a receiver on this machine and reports the
// time spent in each send call, which is time the solver is not solving.
//
//   stream_benchmark [--elements N] [--order P] [--fields F] [--frames K]
//                    [--text] [--queue] [--no-viewer]
//
// The receiver is the viewer's Server, with a thread decoding frames as the
// viewer's does; when the solver outpaces it, sends wait. --queue sends
// through the client's background queue; --no-viewer streams to a port no
// viewer listens on, the case where sending without a queue waits longest.

#include "libRayfem/network/client.hpp"
#include "libRayfem/network/server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
double Milliseconds(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }

struct Summary { double median = 0, mean = 0, worst = 0; };
Summary Summarize(std::vector<double> times) {
    Summary s;
    if (times.empty()) return s;
    std::sort(times.begin(), times.end());
    s.median = times[times.size() / 2];
    for (double t : times) s.mean += t;
    s.mean /= static_cast<double>(times.size());
    s.worst = times.back();
    return s;
}
} // namespace

int main(int argc, char** argv) {
    int elements = 40, order = 2, fieldCount = 3, frames = 20;
    bool text = false, queue = false, viewer = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&] { return i + 1 < argc ? std::atoi(argv[++i]) : 0; };
        if (arg == "--elements") elements = next();
        else if (arg == "--order") order = next();
        else if (arg == "--fields") fieldCount = next();
        else if (arg == "--frames") frames = next();
        else if (arg == "--text") text = true;
        else if (arg == "--queue") queue = true;
        else if (arg == "--no-viewer") viewer = false;
        else { std::cerr << "unknown option " << arg << '\n'; return 2; }
    }
    mfem::Mesh mesh = mfem::Mesh::MakeCartesian3D(elements, elements, elements, mfem::Element::HEXAHEDRON);
    mfem::H1_FECollection collection(order, 3);
    mfem::FiniteElementSpace scalar(&mesh, &collection), vector(&mesh, &collection, 3);
    std::vector<std::unique_ptr<mfem::GridFunction>> fields;
    std::vector<RayFEM::NamedField> named;
    for (int f = 0; f < fieldCount; ++f) {
        fields.push_back(std::make_unique<mfem::GridFunction>(&scalar));
        for (int i = 0; i < fields.back()->Size(); ++i) (*fields.back())(i) = std::sin(0.001 * i + f);
        named.push_back({"field" + std::to_string(f), fields.back().get()});
    }
    mfem::GridFunction displacement(&vector);
    displacement = 0.0;
    const size_t values = static_cast<size_t>(fieldCount) * scalar.GetVSize() + vector.GetVSize();
    std::cout << mesh.GetNE() << " elements, order " << order << ", " << fieldCount << " fields of "
              << scalar.GetVSize() << " values and a displacement: " << std::fixed << std::setprecision(1)
              << values * 8 / 1048576.0 << " MiB of coefficients per frame" << (text ? " (as text)" : "") << '\n';

    const int port = 47000 + static_cast<int>(::getpid() % 1000) * 2;
    RayFEM::Server server("127.0.0.1", port);
    std::atomic<bool> done{false};
    std::atomic<size_t> decoded{0};
    std::vector<double> decodeTimes;  // per frame, after the first (which also decodes the mesh)
    std::thread decoder([&] {
        std::shared_ptr<RayFEM::RenderFrame> latest;  // keeps the decoded mesh, as the viewer's history does
        while (!done) {
            const auto begin = Clock::now();
            auto frames = server.fetchFrames(1);
            const size_t count = frames.size();
            if (count) latest = frames.back();
            if (count && decoded++) decodeTimes.push_back(Milliseconds(Clock::now() - begin));
            if (!count) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    RayFEM::Client client("127.0.0.1", viewer ? port : port + 1);
    client.setBinaryFields(!text);
    if (queue) client.startQueue();
    if (!viewer) frames = std::min(frames, 3);

    auto start = Clock::now();
    if (!client.sendMesh(1, mesh) && viewer) { std::cerr << client.getLastError() << '\n'; return 1; }
    const double meshTime = Milliseconds(Clock::now() - start);

    std::vector<double> times;
    const auto runStart = Clock::now();
    for (int k = 0; k < frames; ++k) {
        for (auto& field : fields) (*field)(0) = k;  // the solver changed something
        RayFEM::FrameMetadata metadata;
        metadata.frame_index = static_cast<uint32_t>(k);
        metadata.sim_time = k;
        start = Clock::now();
        if (!client.sendFrame(1, named, metadata, &displacement) && viewer) { std::cerr << client.getLastError() << '\n'; return 1; }
        times.push_back(Milliseconds(Clock::now() - start));
    }
    const double sendingTime = Milliseconds(Clock::now() - runStart);
    if (queue) client.flush(60000);
    // Wait for the receiver, to report the whole pipeline's throughput.
    const auto deadline = Clock::now() + std::chrono::seconds(viewer ? 60 : 0);
    while (Clock::now() < deadline &&
           decoded.load() + server.getDroppedFrameCount() < static_cast<size_t>(frames))
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const double totalTime = Milliseconds(Clock::now() - runStart);
    done = true;
    decoder.join();

    const auto s = Summarize(times);
    std::cout << std::setprecision(2)
              << "mesh:   " << meshTime << " ms\n"
              << "frame:  median " << s.median << " ms, mean " << s.mean << " ms, worst " << s.worst << " ms ("
              << values * 8 / 1048576.0 / (s.mean / 1000) << " MiB/s in the solver's thread)\n"
              << "sent " << frames << " frames in " << sendingTime << " ms; received"
              << " and decoded in " << totalTime << " ms";
    if (server.getDroppedFrameCount()) std::cout << " (" << server.getDroppedFrameCount() << " dropped by the receiver)";
    std::cout << '\n';
    if (queue) {
        const auto q = client.queueStats();
        std::cout << "queue:  " << q.sent << " sent, " << q.failed << " failed, " << q.dropped << " dropped\n";
    }
    if (!decodeTimes.empty()) {
        const auto d = Summarize(decodeTimes);
        std::cout << "decode: median " << d.median << " ms, mean " << d.mean << " ms per frame in the viewer\n";
    }
    return 0;
}
