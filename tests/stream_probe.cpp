// Receives frames with RayFEM's own Server and prints what it decoded, one
// line per frame, so tests in other languages can check their sends end to
// end. Usage: stream_probe PORT FRAMES [TIMEOUT_SECONDS]
#include "libRayfem/network/server.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <variant>

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: stream_probe PORT FRAMES [TIMEOUT]\n"); return 2; }
    const size_t port = std::strtoul(argv[1], nullptr, 10), wanted = std::strtoul(argv[2], nullptr, 10);
    const double timeout = argc > 3 ? std::atof(argv[3]) : 20;
    RayFEM::Server server("127.0.0.1", port);
    const auto start = std::chrono::steady_clock::now();
    while (!server.ready()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::printf("READY\n");
    std::fflush(stdout);
    size_t received = 0;
    while (received < wanted) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > timeout) {
            std::printf("TIMEOUT after %zu frames; errors %zu: %s\n", received, server.getErrorCount(), server.getLastError().c_str());
            return 1;
        }
        for (const auto& frame : server.fetchFrames(8)) {
            ++received;
            std::printf("FRAME index=%u time=%.17g label=%s elements=%d fields=%zu", frame->metadata.frame_index,
                        frame->metadata.sim_time, frame->metadata.label, frame->mesh->GetNE(), frame->fields.size());
            for (const auto& field : frame->fields) {
                double sum = 0;
                for (int i = 0; i < field.gf->Size(); ++i) sum += (*field.gf)(i);
                std::printf(" field=%s:%d:%.17g", field.name.c_str(), field.gf->Size(), sum);
            }
            if (frame->displacement) std::printf(" displacement=%d:%.17g", frame->displacement->Size(), frame->displacement->Normlinf());
            for (const auto& value : frame->metadata.values) {
                std::printf(" value=%s:", value.name.c_str());
                std::visit([](const auto& v) {
                    using T = std::decay_t<decltype(v)>;
                    if constexpr (std::is_same_v<T, std::string>) std::printf("%s", v.c_str());
                    else if constexpr (std::is_same_v<T, double>) std::printf("%.17g", v);
                    else std::printf("%lld", static_cast<long long>(v));
                }, value.value);
            }
            for (const auto& tag : frame->metadata.tags) std::printf(" tag=%s", tag.c_str());
            if (frame->schema) std::printf(" has_time=%d primary=%s", frame->schema->has_time ? 1 : 0, frame->schema->primary.c_str());
            std::printf("\n");
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::printf("DONE errors=%zu\n", server.getErrorCount());
    return 0;
}
